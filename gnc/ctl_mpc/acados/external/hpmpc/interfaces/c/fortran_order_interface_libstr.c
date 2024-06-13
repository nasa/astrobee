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

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#ifdef BLASFEO
#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_d_blas.h>
#include <blasfeo_d_aux.h>
#endif

#include "../../include/target.h"
#include "../../include/block_size.h"
#include "../../include/aux_d.h"
#include "../../include/aux_s.h"
#include "../../include/blas_d.h"
#include "../../include/lqcp_solvers.h"
#include "../../include/mpc_aux.h"
#include "../../include/mpc_solvers.h"

// Debug flag
#ifndef PC_DEBUG
#define PC_DEBUG 0
#endif /* PC_DEBUG */



int hpmpc_d_ip_ocp_hard_tv_work_space_size_bytes(int N, int *nx, int *nu, int *nb, int **hidxb, int *ng, int N2)
	{
	int ii;
	int size = 0;
	int i_size = 0;
	for(ii=0; ii<N; ii++)
		{
		size += blasfeo_memsize_dmat(nu[ii]+nx[ii]+1, nx[ii+1]); // BAbt
		size += blasfeo_memsize_dmat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii]); // RSQrq
		size += blasfeo_memsize_dmat(nu[ii]+nx[ii]+1, ng[ii]); // DCt
		size += 3*blasfeo_memsize_dvec(nx[ii]); // b, rb, pi
		size += 3*blasfeo_memsize_dvec(nu[ii]+nx[ii]); // rq, rrq, ux
		size += 5*blasfeo_memsize_dvec(2*nb[ii]+2*ng[ii]); // d, lam, t, rd, rm
		}
	ii = N;
	size += blasfeo_memsize_dmat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii]); // RSQrq
	size += blasfeo_memsize_dmat(nu[ii]+nx[ii]+1, ng[ii]); // DCt
	size += 3*blasfeo_memsize_dvec(nx[ii]); // b, rb, pi
	size += 3*blasfeo_memsize_dvec(nu[ii]+nx[ii]); // rq, rrq, ux
	size += 5*blasfeo_memsize_dvec(2*nb[ii]+2*ng[ii]); // d, lam, t, rd, rm
	if(N2<N) // partial condensing
		{
		int nx2[N2+1];
		int nu2[N2+1];
		int nb2[N2+1];
		int ng2[N2+1];
		int work_space_sizes[5];
		d_part_cond_compute_problem_size_libstr(N, nx, nu, nb, hidxb, ng, N2, nx2, nu2, nb2, ng2);
		size += d_part_cond_work_space_size_bytes_libstr(N, nx, nu, nb, hidxb, ng, N2, nx2, nu2, nb2, ng2, &work_space_sizes[0]);
		size += d_part_cond_memory_space_size_bytes_libstr(N, nx, nu, nb, hidxb, ng, N2, nx2, nu2, nb2, ng2);
		size += d_ip2_res_mpc_hard_work_space_size_bytes_libstr(N2, nx2, nu2, nb2, ng2);
		for(ii=0; ii<N2; ii++) // not last stage !!!!!
			{
			size += blasfeo_memsize_dmat(nu2[ii]+nx2[ii]+1, nx2[ii+1]); // BAbt2
			size += blasfeo_memsize_dmat(nu2[ii]+nx2[ii]+1, nu2[ii]+nx2[ii]); // RSQrq2
			size += blasfeo_memsize_dmat(nu2[ii]+nx2[ii]+1, ng2[ii]); // DCt2
			size += 2*blasfeo_memsize_dvec(nx2[ii]); // b2, pi2
			size += 2*blasfeo_memsize_dvec(nu2[ii]+nx2[ii]); // rq2, ux2
			size += 3*blasfeo_memsize_dvec(2*nb2[ii]+2*ng2[ii]); // d2, lam2, t2
			i_size += nb2[ii]; // idxb2
			}
		size += 2*64; // typical cache line size, used for alignement
		}
	else // fully sparse solver
		{
		size += d_ip2_res_mpc_hard_work_space_size_bytes_libstr(N, nx, nu, nb, ng);
		size += 2*64; // typical cache line size, used for alignement
		}
	size += d_res_res_mpc_hard_work_space_size_bytes_libstr(N, nx, nu, nb, ng);
	size += i_size * sizeof(double);
	return size;
	}





int hpmpc_d_ip_ocp_hard_tv_work_space_size_bytes_noidxb(int N, int *nx, int *nu, int *nb, int *nbx, int *nbu, int *ng, int N2)
	{
	int ii;
	int size = 0;
	int i_size = 0;
	for(ii=0; ii<N; ii++)
		{
		size += blasfeo_memsize_dmat(nu[ii]+nx[ii]+1, nx[ii+1]); // BAbt
		size += blasfeo_memsize_dmat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii]); // RSQrq
		size += blasfeo_memsize_dmat(nu[ii]+nx[ii]+1, ng[ii]); // DCt
		size += 3*blasfeo_memsize_dvec(nx[ii]); // b, rb, pi
		size += 3*blasfeo_memsize_dvec(nu[ii]+nx[ii]); // rq, rrq, ux
		size += 5*blasfeo_memsize_dvec(2*nb[ii]+2*ng[ii]); // d, lam, t, rd, rm
		}
	ii = N;
	size += blasfeo_memsize_dmat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii]); // RSQrq
	size += blasfeo_memsize_dmat(nu[ii]+nx[ii]+1, ng[ii]); // DCt
	size += 3*blasfeo_memsize_dvec(nx[ii]); // b, rb, pi
	size += 3*blasfeo_memsize_dvec(nu[ii]+nx[ii]); // rq, rrq, ux
	size += 5*blasfeo_memsize_dvec(2*nb[ii]+2*ng[ii]); // d, lam, t, rd, rm
	if(N2<N) // partial condensing
		{
		int nx2[N2+1];
		int nu2[N2+1];
		int nb2[N2+1];
		int ng2[N2+1];
		int work_space_sizes[5];
		d_part_cond_compute_problem_size_libstr_noidxb(N, nx, nu, nb, nbx, nbu, ng, N2, nx2, nu2, nb2, ng2);
		size += d_part_cond_work_space_size_bytes_libstr(N, nx, nu, nb, NULL, ng, N2, nx2, nu2, nb2, ng2, &work_space_sizes[0]);
		size += d_part_cond_memory_space_size_bytes_libstr(N, nx, nu, nb, NULL, ng, N2, nx2, nu2, nb2, ng2);
		size += d_ip2_res_mpc_hard_work_space_size_bytes_libstr(N2, nx2, nu2, nb2, ng2);
		for(ii=0; ii<N2; ii++) // not last stage !!!!!
			{
			size += blasfeo_memsize_dmat(nu2[ii]+nx2[ii]+1, nx2[ii+1]); // BAbt2
			size += blasfeo_memsize_dmat(nu2[ii]+nx2[ii]+1, nu2[ii]+nx2[ii]); // RSQrq2
			size += blasfeo_memsize_dmat(nu2[ii]+nx2[ii]+1, ng2[ii]); // DCt2
			size += 2*blasfeo_memsize_dvec(nx2[ii]); // b2, pi2
			size += 2*blasfeo_memsize_dvec(nu2[ii]+nx2[ii]); // rq2, ux2
			size += 3*blasfeo_memsize_dvec(2*nb2[ii]+2*ng2[ii]); // d2, lam2, t2
			i_size += nb2[ii]; // idxb2
			}
		size += 2*64; // typical cache line size, used for alignement
		}
	else // fully sparse solver
		{
		size += d_ip2_res_mpc_hard_work_space_size_bytes_libstr(N, nx, nu, nb, ng);
		size += 2*64; // typical cache line size, used for alignement
		}
	size += d_res_res_mpc_hard_work_space_size_bytes_libstr(N, nx, nu, nb, ng);
	size += i_size * sizeof(double);
	return size;
	}





int fortran_order_d_ip_ocp_hard_tv( 
							int *kk, int k_max, double mu0, double mu_tol,
							int N, int *nx, int *nu, int *nb, int **hidxb, int *ng,
							int N2,
							int warm_start,
							double **A, double **B, double **b, 
							double **Q, double **S, double **R, double **q, double **r, 
							double **lb, double **ub,
							double **C, double **D, double **lg, double **ug,
							double **x, double **u, double **pi, double **lam, //double **t,
							double *inf_norm_res,
							void *work0, 
							double *stat)

	{

//printf("\nstart of wrapper\n");

	int hpmpc_status = -1;


	int ii, jj, ll, idx;



	// XXX sequential update not implemented
	if(N2>N)
		N2 = N;



	// check for consistency of problem size
	// nb <= nu+nx
	for(ii=0; ii<=N; ii++)
		{
		if(nb[ii]>nu[ii]+nx[ii])
			{
			printf("\nERROR: At stage %d, the number of bounds nb=%d can not be larger than the number of variables nu+nx=%d.\n\n", ii, nb[ii], nu[ii]+nx[ii]);
			exit(1);
			}
		}


	double alpha_min = 1e-8; // minimum accepted step length
	double temp;
	
	int info = 0;




//printf("\n%d\n", ((size_t) work0) & 63);

	// align to (typical) cache line size
	size_t addr = (( (size_t) work0 ) + 63 ) / 64 * 64;
	char *c_ptr = (char *) addr;


//printf("\n%d\n", ((size_t) ptr) & 63);

	// data structure
	struct blasfeo_dmat hsBAbt[N];
	struct blasfeo_dmat hsRSQrq[N+1];
	struct blasfeo_dmat hsDCt[N+1];
	struct blasfeo_dvec hsb[N];
	struct blasfeo_dvec hsrq[N+1];
	struct blasfeo_dvec hsd[N+1];
	struct blasfeo_dvec hsux[N+1];
	struct blasfeo_dvec hspi[N+1];
	struct blasfeo_dvec hslam[N+1];
	struct blasfeo_dvec hst[N+1];
	struct blasfeo_dvec hsrrq[N+1];
	struct blasfeo_dvec hsrb[N];
	struct blasfeo_dvec hsrd[N+1];
	struct blasfeo_dvec hsrm[N+1];
	void *work_ipm;
	void *work_res;

	for(ii=0; ii<N; ii++)
		{
		blasfeo_create_dmat(nu[ii]+nx[ii]+1, nx[ii+1], &hsBAbt[ii], (void *) c_ptr);
		c_ptr += hsBAbt[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dmat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], &hsRSQrq[ii], (void *) c_ptr);
		c_ptr += hsRSQrq[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dmat(nu[ii]+nx[ii], ng[ii], &hsDCt[ii], (void *) c_ptr);
		c_ptr += hsDCt[ii].memsize;
		}
	
	for(ii=0; ii<N; ii++)
		{
		blasfeo_create_dvec(nx[ii+1], &hsb[ii], (void *) c_ptr);
		c_ptr += hsb[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(nu[ii]+nx[ii], &hsrq[ii], (void *) c_ptr);
		c_ptr += hsrq[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(2*nb[ii]+2*ng[ii], &hsd[ii], (void *) c_ptr);
		c_ptr += hsd[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(nu[ii]+nx[ii], &hsux[ii], (void *) c_ptr);
		c_ptr += hsux[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(nx[ii], &hspi[ii], (void *) c_ptr);
		c_ptr += hspi[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(2*nb[ii]+2*ng[ii], &hslam[ii], (void *) c_ptr);
		c_ptr += hslam[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(2*nb[ii]+2*ng[ii], &hst[ii], (void *) c_ptr);
		c_ptr += hst[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(nu[ii]+nx[ii], &hsrrq[ii], (void *) c_ptr);
		c_ptr += hsrrq[ii].memsize;
		}

	for(ii=0; ii<N; ii++)
		{
		blasfeo_create_dvec(nx[ii+1], &hsrb[ii], (void *) c_ptr);
		c_ptr += hsrb[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(2*nb[ii]+2*ng[ii], &hsrd[ii], (void *) c_ptr);
		c_ptr += hsrd[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(2*nb[ii]+2*ng[ii], &hsrm[ii], (void *) c_ptr);
		c_ptr += hsrm[ii].memsize;
		}
	
	work_res = (void *) c_ptr;
	c_ptr += d_res_res_mpc_hard_work_space_size_bytes_libstr(N, nx, nu, nb, ng);



	// convert matrices

	// TODO use pointers to exploit time invariant !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	// dynamic system
	for(ii=0; ii<N; ii++)
		{
		blasfeo_pack_tran_dmat(nx[ii+1], nu[ii], B[ii], nx[ii+1], &hsBAbt[ii], 0, 0);
		blasfeo_pack_tran_dmat(nx[ii+1], nx[ii], A[ii], nx[ii+1], &hsBAbt[ii], nu[ii], 0);
		blasfeo_pack_tran_dmat(nx[ii+1], 1, b[ii], nx[ii+1], &hsBAbt[ii], nu[ii]+nx[ii], 0);
		blasfeo_pack_dvec(nx[ii+1], b[ii], &hsb[ii], 0);
		}
//	for(ii=0; ii<N; ii++)
//		d_print_strmat(nu[ii]+nx[ii]+1, nx[ii+1], &hsBAbt[ii], 0, 0);
//	for(ii=0; ii<N; ii++)
//		blasfeo_print_tran_dvec(nx[ii+1], &hsb[ii], 0);
//	exit(1);

	// general constraints
	for(ii=0; ii<N; ii++)
		{
		blasfeo_pack_tran_dmat(ng[ii], nu[ii], D[ii], ng[ii], &hsDCt[ii], 0, 0);
		blasfeo_pack_tran_dmat(ng[ii], nx[ii], C[ii], ng[ii], &hsDCt[ii], nu[ii], 0);
		}
	ii = N;
	blasfeo_pack_tran_dmat(ng[ii], nx[ii], C[ii], ng[ii], &hsDCt[ii], 0, 0);
//	for(ii=0; ii<=N; ii++)
//		d_print_strmat(nu[ii]+nx[ii], ng[ii], &hsDCt[ii], 0, 0);
//	exit(1);

	// cost function
	for(ii=0; ii<N; ii++)
		{
		blasfeo_pack_dmat(nu[ii], nu[ii], R[ii], nu[ii], &hsRSQrq[ii], 0, 0);
		blasfeo_pack_tran_dmat(nu[ii], nx[ii], S[ii], nu[ii], &hsRSQrq[ii], nu[ii], 0);
		blasfeo_pack_dmat(nx[ii], nx[ii], Q[ii], nx[ii], &hsRSQrq[ii], nu[ii], nu[ii]);
		blasfeo_pack_tran_dmat(nu[ii], 1, r[ii], nu[ii], &hsRSQrq[ii], nu[ii]+nx[ii], 0);
		blasfeo_pack_tran_dmat(nx[ii], 1, q[ii], nx[ii], &hsRSQrq[ii], nu[ii]+nx[ii], nu[ii]);
		blasfeo_pack_dvec(nu[ii], r[ii], &hsrq[ii], 0);
		blasfeo_pack_dvec(nx[ii], q[ii], &hsrq[ii], nu[ii]);
		}
	ii = N;
	blasfeo_pack_dmat(nx[ii], nx[ii], Q[ii], nx[ii], &hsRSQrq[ii], 0, 0);
	blasfeo_pack_tran_dmat(nx[ii], 1, q[ii], nx[ii], &hsRSQrq[ii], nx[ii], 0);
	blasfeo_pack_dvec(nx[ii], q[ii], &hsrq[ii], 0);
//	for(ii=0; ii<=N; ii++)
//		d_print_strmat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], &hsRSQrq[ii], 0, 0);
//	for(ii=0; ii<=N; ii++)
//		blasfeo_print_tran_dvec(nu[ii]+nx[ii], &hsrq[ii], 0);
//	exit(1);

	// estimate mu0 if not user-provided
//	printf("%f\n", mu0);
	if(mu0<=0)
		{
		for(ii=1; ii<N; ii++)
			{
			for(jj=0; jj<nu[ii]; jj++) for(ll=0; ll<nu[ii]; ll++) mu0 = fmax(mu0, R[ii][jj*nu[ii]+ll]);
			for(jj=0; jj<nx[ii]*nu[ii]; jj++) mu0 = fmax(mu0, S[ii][jj]);
			for(jj=0; jj<nx[ii]; jj++) for(ll=0; ll<nx[ii]; ll++) mu0 = fmax(mu0, Q[ii][jj*nx[ii]+ll]);
			for(jj=0; jj<nu[ii]; jj++) mu0 = fmax(mu0, r[ii][jj]);
			for(jj=0; jj<nx[ii]; jj++) mu0 = fmax(mu0, q[ii][jj]);
			}
		ii=N;
		for(jj=0; jj<nx[ii]; jj++) for(ll=0; ll<nx[ii]; ll++) mu0 = fmax(mu0, Q[ii][jj*nx[ii]+ll]);
		for(jj=0; jj<nx[ii]; jj++) mu0 = fmax(mu0, q[ii][jj]);
		}
//	printf("%f\n", mu0);
//	exit(1);

	// TODO how to handle equality constraints?
	// box constraints 
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_pack_dvec(nb[ii], lb[ii], &hsd[ii], 0);
		blasfeo_pack_dvec(nb[ii], ub[ii], &hsd[ii], nb[ii]+ng[ii]);
		}
	// general constraints
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_pack_dvec(ng[ii], lg[ii], &hsd[ii], nb[ii]+0);
		blasfeo_pack_dvec(ng[ii], ug[ii], &hsd[ii], nb[ii]+nb[ii]+ng[ii]);
		}
//	for(ii=0; ii<=N; ii++)
//		blasfeo_print_tran_dvec(2*nb[ii]+2*ng[ii], &hsd[ii], 0);
//	exit(1);

#if 0
		for(ii=0; ii<N; ii++)
			d_print_strmat(nu[ii]+nx[ii]+1, nx[ii+1], &hsBAbt[ii], 0, 0);
		for(ii=0; ii<=N; ii++)
			d_print_strmat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], &hsRSQrq[ii], 0, 0);
		for(ii=0; ii<=N; ii++)
			d_print_strmat(nu[ii]+nx[ii], ng[ii], &hsDCt[ii], 0, 0);
		for(ii=0; ii<=N; ii++)
			blasfeo_print_tran_dvec(2*nb[ii]+2*ng[ii], &hsd[ii], 0);
		for(ii=0; ii<=N; ii++)
			int_print_mat(1, nb[ii], hidxb[ii], 1);
//		exit(1);
#endif

	



	if(N2<N) // partial condensing
		{

		// compute partially condensed problem size
		int nx2[N2+1];
		int nu2[N2+1];
		int nb2[N2+1];
		int ng2[N2+1];

		d_part_cond_compute_problem_size_libstr(N, nx, nu, nb, hidxb, ng, N2, nx2, nu2, nb2, ng2);

//		for(ii=0; ii<=N2; ii++)
//			printf("\n%d %d %d %d\n", nu2[ii], nx2[ii], nb2[ii], ng2[ii]);
//		exit(1);

		// data structure of partially condensed system
		struct blasfeo_dmat hsBAbt2[N2];
		struct blasfeo_dvec hsb2[N2];
		struct blasfeo_dmat hsRSQrq2[N2+1];
		struct blasfeo_dvec hsrq2[N2+1];
		struct blasfeo_dmat hsDCt2[N2+1];
		struct blasfeo_dvec hsd2[N2+1];
		struct blasfeo_dvec hsux2[N2+1];
		struct blasfeo_dvec hspi2[N2+1];
		struct blasfeo_dvec hslam2[N2+1];
		struct blasfeo_dvec hst2[N2+1];
		int *hidxb2[N2+1];
		void *memory_part_cond;
		void *work_part_cond;
		void *work_part_expand;
		int work_part_cond_sizes[5];
		int work_part_expand_sizes[2];

		// align (again) to (typical) cache line size
		addr = (( (size_t) c_ptr ) + 63 ) / 64 * 64;
		c_ptr = (char *) addr;

		work_part_cond = (void *) c_ptr;
		c_ptr += d_part_cond_work_space_size_bytes_libstr(N, nx, nu, nb, hidxb, ng, N2, nx2, nu2, nb2, ng2, work_part_cond_sizes);

		memory_part_cond = (void *) c_ptr;
		c_ptr += d_part_cond_memory_space_size_bytes_libstr(N, nx, nu, nb, hidxb, ng, N2, nx2, nu2, nb2, ng2);

		work_ipm = (void *) c_ptr;
		c_ptr += d_ip2_res_mpc_hard_work_space_size_bytes_libstr(N2, nx2, nu2, nb2, ng2);

		for(ii=0; ii<N2; ii++)
			{
			blasfeo_create_dmat(nu2[ii]+nx2[ii]+1, nx2[ii+1], &hsBAbt2[ii], (void *) c_ptr);
			c_ptr += hsBAbt2[ii].memsize;
			}

		for(ii=0; ii<N2; ii++) // not last stage !!!!!
			{
			blasfeo_create_dmat(nu2[ii]+nx2[ii]+1, nu2[ii]+nx2[ii], &hsRSQrq2[ii], (void *) c_ptr);
			c_ptr += hsRSQrq2[ii].memsize;
			}
		hsRSQrq2[N2] = hsRSQrq[N];

		for(ii=0; ii<N2; ii++) // not last stage !!!!!
			{
			blasfeo_create_dmat(nu2[ii]+nx2[ii], ng2[ii], &hsDCt2[ii], (void *) c_ptr);
			c_ptr += hsDCt2[ii].memsize;
			}
		hsDCt2[N2] = hsDCt[N];
		
		for(ii=0; ii<N2; ii++)
			{
			blasfeo_create_dvec(nx2[ii+1], &hsb2[ii], (void *) c_ptr);
			c_ptr += hsb2[ii].memsize;
			}

		for(ii=0; ii<N2; ii++) // not last stage !!!!!
			{
			blasfeo_create_dvec(nu2[ii]+nx2[ii], &hsrq2[ii], (void *) c_ptr);
			c_ptr += hsrq2[ii].memsize;
			}
		hsrq2[N2] = hsrq[N];

		for(ii=0; ii<N2; ii++) // not last stage !!!!!
			{
			blasfeo_create_dvec(2*nb2[ii]+2*ng2[ii], &hsd2[ii], (void *) c_ptr);
			c_ptr += hsd2[ii].memsize;
			}
		hsd2[N2] = hsd[N];

		for(ii=0; ii<N2; ii++) // not last stage !!!!!
			{
			blasfeo_create_dvec(nu2[ii]+nx2[ii], &hsux2[ii], (void *) c_ptr);
			c_ptr += hsux2[ii].memsize;
			}
		hsux2[N2] = hsux[N];

		for(ii=0; ii<N2; ii++) // not last stage !!!!!
			{
			blasfeo_create_dvec(nx2[ii], &hspi2[ii], (void *) c_ptr);
			c_ptr += hspi2[ii].memsize;
			}
		hspi2[N2] = hspi[N];

		for(ii=0; ii<N2; ii++) // not last stage !!!!!
			{
			blasfeo_create_dvec(2*nb2[ii]+2*ng2[ii], &hslam2[ii], (void *) c_ptr);
			c_ptr += hslam2[ii].memsize;
			}
		hslam2[N2] = hslam[N];

		for(ii=0; ii<N2; ii++) // not last stage !!!!!
			{
			blasfeo_create_dvec(2*nb2[ii]+2*ng2[ii], &hst2[ii], (void *) c_ptr);
			c_ptr += hst2[ii].memsize;
			}
		hst2[N2] = hst[N];

		for(ii=0; ii<N2; ii++) // not last stage !!!!!
			{
			hidxb2[ii] = (int *) c_ptr;
			c_ptr += nb2[ii]*sizeof(int);
			}
		hidxb2[N2] = hidxb[N];

		// partial condensing routine (computing also hidxb2) !!!
		d_part_cond_libstr(N, nx, nu, nb, hidxb, ng, hsBAbt, hsRSQrq, hsDCt, hsd, N2, nx2, nu2, nb2, hidxb2, ng2, hsBAbt2, hsRSQrq2, hsDCt2, hsd2, memory_part_cond, work_part_cond, &work_part_cond_sizes[0]);

#if 0
		for(ii=0; ii<N2; ii++)
			d_print_strmat(nu2[ii]+nx2[ii]+1, nx2[ii+1], &hsBAbt2[ii], 0, 0);
		for(ii=0; ii<=N2; ii++)
			d_print_strmat(nu2[ii]+nx2[ii]+1, nu2[ii]+nx2[ii], &hsRSQrq2[ii], 0, 0);
		for(ii=0; ii<=N2; ii++)
			d_print_strmat(nu2[ii]+nx2[ii], ng2[ii], &hsDCt2[ii], 0, 0);
		for(ii=0; ii<=N2; ii++)
			blasfeo_print_tran_dvec(2*nb2[ii]+2*ng2[ii], &hsd2[ii], 0);
		for(ii=0; ii<=N2; ii++)
			int_print_mat(1, nb2[ii], hidxb2[ii], 1);
		exit(1);
#endif


		// initial guess TODO part cond
		if(warm_start)
			{

//			for(ii=0; ii<N; ii++)
//				for(jj=0; jj<nu[ii]; jj++)
//					hux[ii][jj] = u[ii][jj];

//			for(ii=0; ii<=N; ii++)
//				for(jj=0; jj<nx[ii]; jj++)
//					hux[ii][nu[ii]+jj] = x[ii][jj];

			}
//		for(ii=0; ii<=N; ii++)
//			d_print_mat(1, nu[ii]+nx[ii], hux[ii], 1);
//		exit(1);


		// IPM solver on partially condensed system
		hpmpc_status = d_ip2_res_mpc_hard_libstr(kk, k_max, mu0, mu_tol, alpha_min, warm_start, stat, N2, nx2, nu2, nb2, hidxb2, ng2, hsBAbt2, hsRSQrq2, hsDCt2, hsd2, hsux2, 1, hspi2, hslam2, hst2, work_ipm);

#if 0
		printf("\nux\n");
		for(ii=0; ii<=N2; ii++)
			blasfeo_print_exp_tran_dvec(nu2[ii]+nx2[ii], &hsux2[ii], 0);
		printf("\npi\n");
		for(ii=0; ii<N2; ii++)
			blasfeo_print_exp_tran_dvec(nx2[ii+1], &hspi2[ii], 0);
		printf("\nlam\n");
		for(ii=0; ii<=N2; ii++)
			blasfeo_print_exp_tran_dvec(2*nb2[ii]+2*ng2[ii], &hslam2[ii], 0);
		printf("\nt\n");
		for(ii=0; ii<=N2; ii++)
			blasfeo_print_exp_tran_dvec(2*nb2[ii]+2*ng2[ii], &hst2[ii], 0);
		exit(2);
#endif

		// expand work space
		work_part_expand = (void *) c_ptr;
		c_ptr += d_part_expand_work_space_size_bytes_libstr(N, nx, nu, nb, ng, work_part_expand_sizes);


		// expand solution of full space system
		d_part_expand_solution_libstr(N, nx, nu, nb, hidxb, ng, hsBAbt, hsb, hsRSQrq, hsrq, hsDCt, hsux, hspi, hslam, hst, N2, nx2, nu2, nb2, hidxb2, ng2, hsux2, hspi2, hslam2, hst2, work_part_expand, work_part_expand_sizes);

//		for(ii=0; ii<=N; ii++)
//			blasfeo_print_tran_dvec(nu[ii]+nx[ii], &hsux[ii], 0);
//		exit(2);

		}
	else // full space system
		{

		// align (again) to (typical) cache line size
		addr = (( (size_t) c_ptr ) + 63 ) / 64 * 64;
		c_ptr = (char *) addr;

		// ipm work space
		work_ipm = (void *) c_ptr;
		c_ptr += d_ip2_res_mpc_hard_work_space_size_bytes_libstr(N, nx, nu, nb, ng);

		// initial guess
		if(warm_start)
			{

			for(ii=0; ii<N; ii++)
				blasfeo_pack_dvec(nu[ii], u[ii], &hsux[ii], 0);

			for(ii=0; ii<=N; ii++)
				blasfeo_pack_dvec(nx[ii], x[ii], &hsux[ii], nu[ii]);

			}

		// IPM solver on full space system
		hpmpc_status = d_ip2_res_mpc_hard_libstr(kk, k_max, mu0, mu_tol, alpha_min, warm_start, stat, N, nx, nu, nb, hidxb, ng, hsBAbt, hsRSQrq, hsDCt, hsd, hsux, 1, hspi, hslam, hst, work_ipm);

		}

#if 0
	printf("\nux\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_exp_tran_dvec(nu[ii]+nx[ii], &hsux[ii], 0);
	printf("\npi\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_exp_tran_dvec(nx[ii], &hspi[ii], 0);
	printf("\nlam\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_exp_tran_dvec(2*nb[ii]+2*ng[ii], &hslam[ii], 0);
	printf("\nt\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_exp_tran_dvec(2*nb[ii]+2*ng[ii], &hst[ii], 0);
	exit(1);
#endif

	// copy back inputs and states
	for(ii=0; ii<N; ii++)
		blasfeo_unpack_dvec(nu[ii], &hsux[ii], 0, u[ii]);

	for(ii=0; ii<=N; ii++)
		blasfeo_unpack_dvec(nx[ii], &hsux[ii], nu[ii], x[ii]);



	// compute infinity norm of residuals on exit

	double mu;

	d_res_res_mpc_hard_libstr(N, nx, nu, nb, hidxb, ng, hsBAbt, hsb, hsRSQrq, hsrq, hsux, hsDCt, hsd, hspi, hslam, hst, hsrrq, hsrb, hsrd, hsrm, &mu, work_res);

#if 0
	for(ii=0; ii<=N; ii++)
		blasfeo_print_exp_tran_dvec(nu[ii]+nx[ii], &hsrrq[ii], 0);
	for(ii=0; ii<N; ii++)
		blasfeo_print_exp_tran_dvec(nx[ii+1], &hsrb[ii], 0);
	for(ii=0; ii<=N; ii++)
		blasfeo_print_exp_tran_dvec(2*nb[ii]+2*ng[ii], &hsrd[ii], 0);
	for(ii=0; ii<=N; ii++)
		blasfeo_print_exp_tran_dvec(2*nb[ii]+2*ng[ii], &hsrm[ii], 0);
	exit(1);
#endif
	
	double *ptr;

	ptr = hsrrq[0].pa;
	temp = fabs(ptr[0]);
	for(ii=0; ii<=N; ii++)
		{
		ptr = hsrrq[ii].pa;
		for(jj=0; jj<nu[ii]+nx[ii]; jj++) 
			temp = fmax( temp, fabs(ptr[jj]) );
		}
	inf_norm_res[0] = temp;

	ptr = hsrb[0].pa;
	temp = fabs(ptr[0]);
	for(ii=0; ii<N; ii++)
		{
		ptr = hsrb[ii].pa;
		for(jj=0; jj<nx[ii+1]; jj++) 
			temp = fmax( temp, fabs(ptr[jj]) );
		}
	inf_norm_res[1] = temp;

	ptr = hsrd[0].pa;
	temp = fabs(ptr[0]);
	for(ii=0; ii<=N; ii++)
		{
		ptr = hsrd[ii].pa;
		for(jj=0; jj<2*nb[ii]+2*ng[ii]; jj++) 
			{
			temp = fmax( temp, fabs(ptr[jj]) );
			}
		}
	inf_norm_res[2] = temp;

	ptr = hsrm[0].pa;
	temp = fabs(ptr[0]);
	for(ii=0; ii<=N; ii++)
		{
		ptr = hsrm[ii].pa;
		for(jj=0; jj<2*nb[ii]+2*ng[ii]; jj++) 
			{
			temp = fmax( temp, fabs(ptr[jj]) );
			} }
	inf_norm_res[3] = temp;

	inf_norm_res[4] = mu;

	// copy back multipliers

	for(ii=0; ii<N; ii++)
		blasfeo_unpack_dvec(nx[ii+1], &hspi[ii+1], 0, pi[ii]);

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_unpack_dvec(2*nb[ii]+2*ng[ii], &hslam[ii], 0, lam[ii]);
//		blasfeo_unpack_dvec(2*nb[ii]+2*ng[ii], &hst[ii], 0, t[ii]);
		}

//	printf("\nend of wrapper\n");

    return hpmpc_status;

	}





int c_order_d_ip_ocp_hard_tv( 
							int *kk, int k_max, double mu0, double mu_tol,
							int N, int *nx, int *nu, int *nb, int **hidxb, int *ng,
							int N2,
							int warm_start,
							double **A, double **B, double **b, 
							double **Q, double **S, double **R, double **q, double **r, 
							double **lb, double **ub,
							double **C, double **D, double **lg, double **ug,
							double **x, double **u, double **pi, double **lam, //double **t,
							double *inf_norm_res,
							void *work0, 
							double *stat)

	{

//printf("\nstart of wrapper\n");

	int hpmpc_status = -1;


	int ii, jj, ll, idx;



	// XXX sequential update not implemented
	if(N2>N)
		N2 = N;



	// check for consistency of problem size
	// nb <= nu+nx
	for(ii=0; ii<=N; ii++)
		{
		if(nb[ii]>nu[ii]+nx[ii])
			{
			printf("\nERROR: At stage %d, the number of bounds nb=%d can not be larger than the number of variables nu+nx=%d.\n\n", ii, nb[ii], nu[ii]+nx[ii]);
			exit(1);
			}
		}


	double alpha_min = 1e-8; // minimum accepted step length
	double temp;
	
	int info = 0;




//printf("\n%d\n", ((size_t) work0) & 63);

	// align to (typical) cache line size
	size_t addr = (( (size_t) work0 ) + 63 ) / 64 * 64;
	char *c_ptr = (char *) addr;


//printf("\n%d\n", ((size_t) ptr) & 63);

	// data structure
	struct blasfeo_dmat hsBAbt[N];
	struct blasfeo_dmat hsRSQrq[N+1];
	struct blasfeo_dmat hsDCt[N+1];
	struct blasfeo_dvec hsb[N];
	struct blasfeo_dvec hsrq[N+1];
	struct blasfeo_dvec hsd[N+1];
	struct blasfeo_dvec hsux[N+1];
	struct blasfeo_dvec hspi[N+1];
	struct blasfeo_dvec hslam[N+1];
	struct blasfeo_dvec hst[N+1];
	struct blasfeo_dvec hsrrq[N+1];
	struct blasfeo_dvec hsrb[N];
	struct blasfeo_dvec hsrd[N+1];
	struct blasfeo_dvec hsrm[N+1];
	void *work_ipm;
	void *work_res;

	for(ii=0; ii<N; ii++)
		{
		blasfeo_create_dmat(nu[ii]+nx[ii]+1, nx[ii+1], &hsBAbt[ii], (void *) c_ptr);
		c_ptr += hsBAbt[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dmat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], &hsRSQrq[ii], (void *) c_ptr);
		c_ptr += hsRSQrq[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dmat(nu[ii]+nx[ii], ng[ii], &hsDCt[ii], (void *) c_ptr);
		c_ptr += hsDCt[ii].memsize;
		}
	
	for(ii=0; ii<N; ii++)
		{
		blasfeo_create_dvec(nx[ii+1], &hsb[ii], (void *) c_ptr);
		c_ptr += hsb[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(nu[ii]+nx[ii], &hsrq[ii], (void *) c_ptr);
		c_ptr += hsrq[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(2*nb[ii]+2*ng[ii], &hsd[ii], (void *) c_ptr);
		c_ptr += hsd[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(nu[ii]+nx[ii], &hsux[ii], (void *) c_ptr);
		c_ptr += hsux[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(nx[ii], &hspi[ii], (void *) c_ptr);
		c_ptr += hspi[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(2*nb[ii]+2*ng[ii], &hslam[ii], (void *) c_ptr);
		c_ptr += hslam[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(2*nb[ii]+2*ng[ii], &hst[ii], (void *) c_ptr);
		c_ptr += hst[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(nu[ii]+nx[ii], &hsrrq[ii], (void *) c_ptr);
		c_ptr += hsrrq[ii].memsize;
		}

	for(ii=0; ii<N; ii++)
		{
		blasfeo_create_dvec(nx[ii+1], &hsrb[ii], (void *) c_ptr);
		c_ptr += hsrb[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(2*nb[ii]+2*ng[ii], &hsrd[ii], (void *) c_ptr);
		c_ptr += hsrd[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(2*nb[ii]+2*ng[ii], &hsrm[ii], (void *) c_ptr);
		c_ptr += hsrm[ii].memsize;
		}
	
	work_res = (void *) c_ptr;
	c_ptr += d_res_res_mpc_hard_work_space_size_bytes_libstr(N, nx, nu, nb, ng);



	// convert matrices

	// TODO use pointers to exploit time invariant !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	// dynamic system
	for(ii=0; ii<N; ii++)
		{
		blasfeo_pack_dmat(nu[ii], nx[ii+1], B[ii], nu[ii], &hsBAbt[ii], 0, 0);
		blasfeo_pack_dmat(nx[ii], nx[ii+1], A[ii], nx[ii], &hsBAbt[ii], nu[ii], 0);
		blasfeo_pack_tran_dmat(nx[ii+1], 1, b[ii], nx[ii+1], &hsBAbt[ii], nu[ii]+nx[ii], 0);
		blasfeo_pack_dvec(nx[ii+1], b[ii], &hsb[ii], 0);
		}
#if 0
	for(ii=0; ii<N; ii++)
		d_print_strmat(nu[ii]+nx[ii]+1, nx[ii+1], &hsBAbt[ii], 0, 0);
	for(ii=0; ii<N; ii++)
		blasfeo_print_tran_dvec(nx[ii+1], &hsb[ii], 0);
//	exit(1);
#endif

	// general constraints
	for(ii=0; ii<N; ii++)
		{
		blasfeo_pack_dmat(nu[ii], ng[ii], D[ii], nu[ii], &hsDCt[ii], 0, 0);
		blasfeo_pack_dmat(nx[ii], ng[ii], C[ii], nx[ii], &hsDCt[ii], nu[ii], 0);
		}
	ii = N;
	blasfeo_pack_dmat(nx[ii], ng[ii], C[ii], nx[ii], &hsDCt[ii], 0, 0);
#if 0
	for(ii=0; ii<=N; ii++)
		d_print_strmat(nu[ii]+nx[ii], ng[ii], &hsDCt[ii], 0, 0);
//	exit(1);
#endif

	// cost function
	for(ii=0; ii<N; ii++)
		{
		blasfeo_pack_tran_dmat(nu[ii], nu[ii], R[ii], nu[ii], &hsRSQrq[ii], 0, 0);
		blasfeo_pack_dmat(nx[ii], nu[ii], S[ii], nx[ii], &hsRSQrq[ii], nu[ii], 0);
		blasfeo_pack_tran_dmat(nx[ii], nx[ii], Q[ii], nx[ii], &hsRSQrq[ii], nu[ii], nu[ii]);
		blasfeo_pack_tran_dmat(nu[ii], 1, r[ii], nu[ii], &hsRSQrq[ii], nu[ii]+nx[ii], 0);
		blasfeo_pack_tran_dmat(nx[ii], 1, q[ii], nx[ii], &hsRSQrq[ii], nu[ii]+nx[ii], nu[ii]);
		blasfeo_pack_dvec(nu[ii], r[ii], &hsrq[ii], 0);
		blasfeo_pack_dvec(nx[ii], q[ii], &hsrq[ii], nu[ii]);
		}
	ii = N;
	blasfeo_pack_tran_dmat(nx[ii], nx[ii], Q[ii], nx[ii], &hsRSQrq[ii], 0, 0);
	blasfeo_pack_tran_dmat(nx[ii], 1, q[ii], nx[ii], &hsRSQrq[ii], nx[ii], 0);
	blasfeo_pack_dvec(nx[ii], q[ii], &hsrq[ii], 0);
#if 0
	for(ii=0; ii<=N; ii++)
		d_print_strmat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], &hsRSQrq[ii], 0, 0);
	for(ii=0; ii<=N; ii++)
		blasfeo_print_tran_dvec(nu[ii]+nx[ii], &hsrq[ii], 0);
//	exit(1);
#endif

	// estimate mu0 if not user-provided
//	printf("%f\n", mu0);
	if(mu0<=0)
		{
		for(ii=1; ii<N; ii++)
			{
			for(jj=0; jj<nu[ii]; jj++) for(ll=0; ll<nu[ii]; ll++) mu0 = fmax(mu0, R[ii][jj*nu[ii]+ll]);
			for(jj=0; jj<nx[ii]*nu[ii]; jj++) mu0 = fmax(mu0, S[ii][jj]);
			for(jj=0; jj<nx[ii]; jj++) for(ll=0; ll<nx[ii]; ll++) mu0 = fmax(mu0, Q[ii][jj*nx[ii]+ll]);
			for(jj=0; jj<nu[ii]; jj++) mu0 = fmax(mu0, r[ii][jj]);
			for(jj=0; jj<nx[ii]; jj++) mu0 = fmax(mu0, q[ii][jj]);
			}
		ii=N;
		for(jj=0; jj<nx[ii]; jj++) for(ll=0; ll<nx[ii]; ll++) mu0 = fmax(mu0, Q[ii][jj*nx[ii]+ll]);
		for(jj=0; jj<nx[ii]; jj++) mu0 = fmax(mu0, q[ii][jj]);
		}
//	printf("%f\n", mu0);
//	exit(1);

	// TODO how to handle equality constraints?
	// box constraints 
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_pack_dvec(nb[ii], lb[ii], &hsd[ii], 0);
		blasfeo_pack_dvec(nb[ii], ub[ii], &hsd[ii], nb[ii]+ng[ii]);
		}
	// general constraints
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_pack_dvec(ng[ii], lg[ii], &hsd[ii], nb[ii]+0);
		blasfeo_pack_dvec(ng[ii], ug[ii], &hsd[ii], nb[ii]+nb[ii]+ng[ii]);
		}
//	for(ii=0; ii<=N; ii++)
//		blasfeo_print_tran_dvec(2*nb[ii]+2*ng[ii], &hsd[ii], 0);
//	exit(1);

#if 0
		for(ii=0; ii<N; ii++)
			d_print_strmat(nu[ii]+nx[ii]+1, nx[ii+1], &hsBAbt[ii], 0, 0);
		for(ii=0; ii<=N; ii++)
			d_print_strmat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], &hsRSQrq[ii], 0, 0);
		for(ii=0; ii<=N; ii++)
			d_print_strmat(nu[ii]+nx[ii], ng[ii], &hsDCt[ii], 0, 0);
		for(ii=0; ii<=N; ii++)
			blasfeo_print_tran_dvec(2*nb[ii]+2*ng[ii], &hsd[ii], 0);
		for(ii=0; ii<=N; ii++)
			int_print_mat(1, nb[ii], hidxb[ii], 1);
//		exit(1);
#endif

	



	if(N2<N) // partial condensing
		{

		// compute partially condensed problem size
		int nx2[N2+1];
		int nu2[N2+1];
		int nb2[N2+1];
		int ng2[N2+1];

		d_part_cond_compute_problem_size_libstr(N, nx, nu, nb, hidxb, ng, N2, nx2, nu2, nb2, ng2);

//		for(ii=0; ii<=N2; ii++)
//			printf("\n%d %d %d %d\n", nu2[ii], nx2[ii], nb2[ii], ng2[ii]);
//		exit(1);

		// data structure of partially condensed system
		struct blasfeo_dmat hsBAbt2[N2];
		struct blasfeo_dvec hsb2[N2];
		struct blasfeo_dmat hsRSQrq2[N2+1];
		struct blasfeo_dvec hsrq2[N2+1];
		struct blasfeo_dmat hsDCt2[N2+1];
		struct blasfeo_dvec hsd2[N2+1];
		struct blasfeo_dvec hsux2[N2+1];
		struct blasfeo_dvec hspi2[N2+1];
		struct blasfeo_dvec hslam2[N2+1];
		struct blasfeo_dvec hst2[N2+1];
		int *hidxb2[N2+1];
		void *memory_part_cond;
		void *work_part_cond;
		void *work_part_expand;
		int work_part_cond_sizes[5];
		int work_part_expand_sizes[2];

		// align (again) to (typical) cache line size
		addr = (( (size_t) c_ptr ) + 63 ) / 64 * 64;
		c_ptr = (char *) addr;

		work_part_cond = (void *) c_ptr;
		c_ptr += d_part_cond_work_space_size_bytes_libstr(N, nx, nu, nb, hidxb, ng, N2, nx2, nu2, nb2, ng2, work_part_cond_sizes);

		memory_part_cond = (void *) c_ptr;
		c_ptr += d_part_cond_memory_space_size_bytes_libstr(N, nx, nu, nb, hidxb, ng, N2, nx2, nu2, nb2, ng2);

		work_ipm = (void *) c_ptr;
		c_ptr += d_ip2_res_mpc_hard_work_space_size_bytes_libstr(N2, nx2, nu2, nb2, ng2);

		for(ii=0; ii<N2; ii++)
			{
			blasfeo_create_dmat(nu2[ii]+nx2[ii]+1, nx2[ii+1], &hsBAbt2[ii], (void *) c_ptr);
			c_ptr += hsBAbt2[ii].memsize;
			}

		for(ii=0; ii<N2; ii++) // not last stage !!!!!
			{
			blasfeo_create_dmat(nu2[ii]+nx2[ii]+1, nu2[ii]+nx2[ii], &hsRSQrq2[ii], (void *) c_ptr);
			c_ptr += hsRSQrq2[ii].memsize;
			}
		hsRSQrq2[N2] = hsRSQrq[N];

		for(ii=0; ii<N2; ii++) // not last stage !!!!!
			{
			blasfeo_create_dmat(nu2[ii]+nx2[ii], ng2[ii], &hsDCt2[ii], (void *) c_ptr);
			c_ptr += hsDCt2[ii].memsize;
			}
		hsDCt2[N2] = hsDCt[N];
		
		for(ii=0; ii<N2; ii++)
			{
			blasfeo_create_dvec(nx2[ii+1], &hsb2[ii], (void *) c_ptr);
			c_ptr += hsb2[ii].memsize;
			}

		for(ii=0; ii<N2; ii++) // not last stage !!!!!
			{
			blasfeo_create_dvec(nu2[ii]+nx2[ii], &hsrq2[ii], (void *) c_ptr);
			c_ptr += hsrq2[ii].memsize;
			}
		hsrq2[N2] = hsrq[N];

		for(ii=0; ii<N2; ii++) // not last stage !!!!!
			{
			blasfeo_create_dvec(2*nb2[ii]+2*ng2[ii], &hsd2[ii], (void *) c_ptr);
			c_ptr += hsd2[ii].memsize;
			}
		hsd2[N2] = hsd[N];

		for(ii=0; ii<N2; ii++) // not last stage !!!!!
			{
			blasfeo_create_dvec(nu2[ii]+nx2[ii], &hsux2[ii], (void *) c_ptr);
			c_ptr += hsux2[ii].memsize;
			}
		hsux2[N2] = hsux[N];

		for(ii=0; ii<N2; ii++) // not last stage !!!!!
			{
			blasfeo_create_dvec(nx2[ii], &hspi2[ii], (void *) c_ptr);
			c_ptr += hspi2[ii].memsize;
			}
		hspi2[N2] = hspi[N];

		for(ii=0; ii<N2; ii++) // not last stage !!!!!
			{
			blasfeo_create_dvec(2*nb2[ii]+2*ng2[ii], &hslam2[ii], (void *) c_ptr);
			c_ptr += hslam2[ii].memsize;
			}
		hslam2[N2] = hslam[N];

		for(ii=0; ii<N2; ii++) // not last stage !!!!!
			{
			blasfeo_create_dvec(2*nb2[ii]+2*ng2[ii], &hst2[ii], (void *) c_ptr);
			c_ptr += hst2[ii].memsize;
			}
		hst2[N2] = hst[N];

		for(ii=0; ii<N2; ii++) // not last stage !!!!!
			{
			hidxb2[ii] = (int *) c_ptr;
			c_ptr += nb2[ii]*sizeof(int);
			}
		hidxb2[N2] = hidxb[N];

		// partial condensing routine (computing also hidxb2) !!!
		d_part_cond_libstr(N, nx, nu, nb, hidxb, ng, hsBAbt, hsRSQrq, hsDCt, hsd, N2, nx2, nu2, nb2, hidxb2, ng2, hsBAbt2, hsRSQrq2, hsDCt2, hsd2, memory_part_cond, work_part_cond, &work_part_cond_sizes[0]);

#if 0
		for(ii=0; ii<N2; ii++)
			d_print_strmat(nu2[ii]+nx2[ii]+1, nx2[ii+1], &hsBAbt2[ii], 0, 0);
		for(ii=0; ii<=N2; ii++)
			d_print_strmat(nu2[ii]+nx2[ii]+1, nu2[ii]+nx2[ii], &hsRSQrq2[ii], 0, 0);
		for(ii=0; ii<=N2; ii++)
			d_print_strmat(nu2[ii]+nx2[ii], ng2[ii], &hsDCt2[ii], 0, 0);
		for(ii=0; ii<=N2; ii++)
			blasfeo_print_tran_dvec(2*nb2[ii]+2*ng2[ii], &hsd2[ii], 0);
		for(ii=0; ii<=N2; ii++)
			int_print_mat(1, nb2[ii], hidxb2[ii], 1);
		exit(1);
#endif


		// initial guess TODO part cond
		if(warm_start)
			{

//			for(ii=0; ii<N; ii++)
//				for(jj=0; jj<nu[ii]; jj++)
//					hux[ii][jj] = u[ii][jj];

//			for(ii=0; ii<=N; ii++)
//				for(jj=0; jj<nx[ii]; jj++)
//					hux[ii][nu[ii]+jj] = x[ii][jj];

			}
//		for(ii=0; ii<=N; ii++)
//			d_print_mat(1, nu[ii]+nx[ii], hux[ii], 1);
//		exit(1);


		// IPM solver on partially condensed system
		hpmpc_status = d_ip2_res_mpc_hard_libstr(kk, k_max, mu0, mu_tol, alpha_min, warm_start, stat, N2, nx2, nu2, nb2, hidxb2, ng2, hsBAbt2, hsRSQrq2, hsDCt2, hsd2, hsux2, 1, hspi2, hslam2, hst2, work_ipm);

#if 0
		printf("\nux\n");
		for(ii=0; ii<=N2; ii++)
			blasfeo_print_exp_tran_dvec(nu2[ii]+nx2[ii], &hsux2[ii], 0);
		printf("\npi\n");
		for(ii=0; ii<N2; ii++)
			blasfeo_print_exp_tran_dvec(nx2[ii+1], &hspi2[ii], 0);
		printf("\nlam\n");
		for(ii=0; ii<=N2; ii++)
			blasfeo_print_exp_tran_dvec(2*nb2[ii]+2*ng2[ii], &hslam2[ii], 0);
		printf("\nt\n");
		for(ii=0; ii<=N2; ii++)
			blasfeo_print_exp_tran_dvec(2*nb2[ii]+2*ng2[ii], &hst2[ii], 0);
		exit(2);
#endif

		// expand work space
		work_part_expand = (void *) c_ptr;
		c_ptr += d_part_expand_work_space_size_bytes_libstr(N, nx, nu, nb, ng, work_part_expand_sizes);


		// expand solution of full space system
		d_part_expand_solution_libstr(N, nx, nu, nb, hidxb, ng, hsBAbt, hsb, hsRSQrq, hsrq, hsDCt, hsux, hspi, hslam, hst, N2, nx2, nu2, nb2, hidxb2, ng2, hsux2, hspi2, hslam2, hst2, work_part_expand, work_part_expand_sizes);

//		for(ii=0; ii<=N; ii++)
//			blasfeo_print_tran_dvec(nu[ii]+nx[ii], &hsux[ii], 0);
//		exit(2);

		}
	else // full space system
		{

		// align (again) to (typical) cache line size
		addr = (( (size_t) c_ptr ) + 63 ) / 64 * 64;
		c_ptr = (char *) addr;

		// ipm work space
		work_ipm = (void *) c_ptr;
		c_ptr += d_ip2_res_mpc_hard_work_space_size_bytes_libstr(N, nx, nu, nb, ng);

		// initial guess
		if(warm_start)
			{

			for(ii=0; ii<N; ii++)
				blasfeo_pack_dvec(nu[ii], u[ii], &hsux[ii], 0);

			for(ii=0; ii<=N; ii++)
				blasfeo_pack_dvec(nx[ii], x[ii], &hsux[ii], nu[ii]);

			}

		// IPM solver on full space system
		hpmpc_status = d_ip2_res_mpc_hard_libstr(kk, k_max, mu0, mu_tol, alpha_min, warm_start, stat, N, nx, nu, nb, hidxb, ng, hsBAbt, hsRSQrq, hsDCt, hsd, hsux, 1, hspi, hslam, hst, work_ipm);

		}

#if 0
	printf("\nux\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_exp_tran_dvec(nu[ii]+nx[ii], &hsux[ii], 0);
	printf("\npi\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_exp_tran_dvec(nx[ii], &hspi[ii], 0);
	printf("\nlam\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_exp_tran_dvec(2*nb[ii]+2*ng[ii], &hslam[ii], 0);
	printf("\nt\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_exp_tran_dvec(2*nb[ii]+2*ng[ii], &hst[ii], 0);
//	exit(1);
#endif

	// copy back inputs and states
	for(ii=0; ii<N; ii++)
		blasfeo_unpack_dvec(nu[ii], &hsux[ii], 0, u[ii]);

	for(ii=0; ii<=N; ii++)
		blasfeo_unpack_dvec(nx[ii], &hsux[ii], nu[ii], x[ii]);



	// compute infinity norm of residuals on exit

	double mu;

	d_res_res_mpc_hard_libstr(N, nx, nu, nb, hidxb, ng, hsBAbt, hsb, hsRSQrq, hsrq, hsux, hsDCt, hsd, hspi, hslam, hst, hsrrq, hsrb, hsrd, hsrm, &mu, work_res);

#if 0
	printf("\nres_rq\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_exp_tran_dvec(nu[ii]+nx[ii], &hsrrq[ii], 0);
	printf("\nres_b\n");
	for(ii=0; ii<N; ii++)
		blasfeo_print_exp_tran_dvec(nx[ii+1], &hsrb[ii], 0);
	printf("\nres_d\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_exp_tran_dvec(2*nb[ii]+2*ng[ii], &hsrd[ii], 0);
	printf("\nres_m\n");
	for(ii=0; ii<=N; ii++)
		blasfeo_print_exp_tran_dvec(2*nb[ii]+2*ng[ii], &hsrm[ii], 0);
//	exit(1);
#endif
	
	double *ptr;

	ptr = hsrrq[0].pa;
	temp = fabs(ptr[0]);
	for(ii=0; ii<=N; ii++)
		{
		ptr = hsrrq[ii].pa;
		for(jj=0; jj<nu[ii]+nx[ii]; jj++) 
			temp = fmax( temp, fabs(ptr[jj]) );
		}
	inf_norm_res[0] = temp;

	ptr = hsrb[0].pa;
	temp = fabs(ptr[0]);
	for(ii=0; ii<N; ii++)
		{
		ptr = hsrb[ii].pa;
		for(jj=0; jj<nx[ii+1]; jj++) 
			temp = fmax( temp, fabs(ptr[jj]) );
		}
	inf_norm_res[1] = temp;

	ptr = hsrd[0].pa;
	temp = fabs(ptr[0]);
	for(ii=0; ii<=N; ii++)
		{
		ptr = hsrd[ii].pa;
		for(jj=0; jj<2*nb[ii]+2*ng[ii]; jj++) 
			{
			temp = fmax( temp, fabs(ptr[jj]) );
			}
		}
	inf_norm_res[2] = temp;

	ptr = hsrm[0].pa;
	temp = fabs(ptr[0]);
	for(ii=0; ii<=N; ii++)
		{
		ptr = hsrm[ii].pa;
		for(jj=0; jj<2*nb[ii]+2*ng[ii]; jj++) 
			{
			temp = fmax( temp, fabs(ptr[jj]) );
			} }
	inf_norm_res[3] = temp;

	inf_norm_res[4] = mu;

	// copy back multipliers

	for(ii=0; ii<N; ii++)
		blasfeo_unpack_dvec(nx[ii+1], &hspi[ii+1], 0, pi[ii]);

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_unpack_dvec(2*nb[ii]+2*ng[ii], &hslam[ii], 0, lam[ii]);
//		blasfeo_unpack_dvec(2*nb[ii]+2*ng[ii], &hst[ii], 0, t[ii]);
		}

//	printf("\nend of wrapper\n");

    return hpmpc_status;

	}





void fortran_order_d_ip_last_kkt_new_rhs_ocp_hard_libstr( 
							int N, int *nx, int *nu, int *nb, int **hidxb, int *ng, // TODO use memory to save them !!!
							int N2, // TODO use memory to save them !!!
							double **b, 
							double **q, double **r, 
							double **lb, double **ub,
							double **lg, double **ug,
							double **x, double **u, double **pi, double **lam, //double **t,
							double *inf_norm_res, // XXX remove ???
							void *work0) 

	{

//printf("\nstart of wrapper\n");

	int ii, jj, ll, idx;



	// XXX sequential update not implemented
	if(N2>N)
		N2 = N;



	// check for consistency of problem size
	// nb <= nu+nx
	for(ii=0; ii<=N; ii++)
		{
		if(nb[ii]>nu[ii]+nx[ii])
			{
			printf("\nERROR: At stage %d, the number of bounds nb=%d can not be larger than the number of variables nu+nx=%d.\n\n", ii, nb[ii], nu[ii]+nx[ii]);
			exit(1);
			}
		}


	double temp;
	
	int info = 0;




//printf("\n%d\n", ((size_t) work0) & 63);

	// align to (typical) cache line size
	size_t addr = (( (size_t) work0 ) + 63 ) / 64 * 64;
	char *c_ptr = (char *) addr;


//printf("\n%d\n", ((size_t) ptr) & 63);

	// data structure
	struct blasfeo_dmat hsBAbt[N];
	struct blasfeo_dmat hsRSQrq[N+1];
	struct blasfeo_dmat hsDCt[N+1];
	struct blasfeo_dvec hsb[N];
	struct blasfeo_dvec hsrq[N+1];
	struct blasfeo_dvec hsd[N+1];
	struct blasfeo_dvec hsux[N+1];
	struct blasfeo_dvec hspi[N+1];
	struct blasfeo_dvec hslam[N+1];
	struct blasfeo_dvec hst[N+1];
	struct blasfeo_dvec hsrrq[N+1];
	struct blasfeo_dvec hsrb[N];
	struct blasfeo_dvec hsrd[N+1];
	struct blasfeo_dvec hsrm[N+1];
	void *work_ipm;
	void *work_res;

	for(ii=0; ii<N; ii++)
		{
		blasfeo_create_dmat(nu[ii]+nx[ii]+1, nx[ii+1], &hsBAbt[ii], (void *) c_ptr);
		c_ptr += hsBAbt[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dmat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], &hsRSQrq[ii], (void *) c_ptr);
		c_ptr += hsRSQrq[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dmat(nu[ii]+nx[ii], ng[ii], &hsDCt[ii], (void *) c_ptr);
		c_ptr += hsDCt[ii].memsize;
		}
	
	for(ii=0; ii<N; ii++)
		{
		blasfeo_create_dvec(nx[ii+1], &hsb[ii], (void *) c_ptr);
		c_ptr += hsb[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(nu[ii]+nx[ii], &hsrq[ii], (void *) c_ptr);
		c_ptr += hsrq[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(2*nb[ii]+2*ng[ii], &hsd[ii], (void *) c_ptr);
		c_ptr += hsd[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(nu[ii]+nx[ii], &hsux[ii], (void *) c_ptr);
		c_ptr += hsux[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(nx[ii], &hspi[ii], (void *) c_ptr);
		c_ptr += hspi[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(2*nb[ii]+2*ng[ii], &hslam[ii], (void *) c_ptr);
		c_ptr += hslam[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(2*nb[ii]+2*ng[ii], &hst[ii], (void *) c_ptr);
		c_ptr += hst[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(nu[ii]+nx[ii], &hsrrq[ii], (void *) c_ptr);
		c_ptr += hsrrq[ii].memsize;
		}

	for(ii=0; ii<N; ii++)
		{
		blasfeo_create_dvec(nx[ii+1], &hsrb[ii], (void *) c_ptr);
		c_ptr += hsrb[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(2*nb[ii]+2*ng[ii], &hsrd[ii], (void *) c_ptr);
		c_ptr += hsrd[ii].memsize;
		}

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_create_dvec(2*nb[ii]+2*ng[ii], &hsrm[ii], (void *) c_ptr);
		c_ptr += hsrm[ii].memsize;
		}
	
	work_res = (void *) c_ptr;
	c_ptr += d_res_res_mpc_hard_work_space_size_bytes_libstr(N, nx, nu, nb, ng);



	// convert matrices

	// TODO use pointers to exploit time invariant !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	// dynamic system
	for(ii=0; ii<N; ii++)
		{
//		blasfeo_pack_tran_dmat(nx[ii+1], 1, b[ii], nx[ii+1], &hsBAbt[ii], nu[ii]+nx[ii], 0); // XXX ???
		blasfeo_pack_dvec(nx[ii+1], b[ii], &hsb[ii], 0);
		}
//	for(ii=0; ii<N; ii++)
//		d_print_strmat(nu[ii]+nx[ii]+1, nx[ii+1], &hsBAbt[ii], 0, 0);
//	for(ii=0; ii<N; ii++)
//		blasfeo_print_tran_dvec(nx[ii+1], &hsb[ii], 0);
//	exit(1);

	// general constraints
//	for(ii=0; ii<=N; ii++)
//		d_print_strmat(nu[ii]+nx[ii], ng[ii], &hsDCt[ii], 0, 0);
//	exit(1);

	// cost function
	for(ii=0; ii<N; ii++)
		{
//		blasfeo_pack_tran_dmat(nu[ii], 1, r[ii], nu[ii], &hsRSQrq[ii], nu[ii]+nx[ii], 0); // XXX ???
//		blasfeo_pack_tran_dmat(nx[ii], 1, q[ii], nx[ii], &hsRSQrq[ii], nu[ii]+nx[ii], nu[ii]); // XXX ???
		blasfeo_pack_dvec(nu[ii], r[ii], &hsrq[ii], 0);
		blasfeo_pack_dvec(nx[ii], q[ii], &hsrq[ii], nu[ii]);
		}
	ii = N;
//	blasfeo_pack_tran_dmat(nx[ii], 1, q[ii], nx[ii], &hsRSQrq[ii], nx[ii], 0); // XXX ???
	blasfeo_pack_dvec(nx[ii], q[ii], &hsrq[ii], 0);
//	for(ii=0; ii<=N; ii++)
//		d_print_strmat(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], &hsRSQrq[ii], 0, 0);
//	for(ii=0; ii<=N; ii++)
//		blasfeo_print_tran_dvec(nu[ii]+nx[ii], &hsrq[ii], 0);
//	exit(1);

	// TODO how to handle equality constraints?
	// box constraints 
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_pack_dvec(nb[ii], lb[ii], &hsd[ii], 0);
		blasfeo_pack_dvec(nb[ii], ub[ii], &hsd[ii], nb[ii]+ng[ii]);
		}
	// general constraints
	for(ii=0; ii<=N; ii++)
		{
		blasfeo_pack_dvec(ng[ii], lg[ii], &hsd[ii], nb[ii]+0);
		blasfeo_pack_dvec(ng[ii], ug[ii], &hsd[ii], nb[ii]+nb[ii]+ng[ii]);
		}
//	for(ii=0; ii<=N; ii++)
//		blasfeo_print_tran_dvec(2*nb[ii]+2*ng[ii], &hsd[ii], 0);
//	exit(1);

	

// TODO


	if(N2<N) // partial condensing
		{

		// compute partially condensed problem size
		int nx2[N2+1];
		int nu2[N2+1];
		int nb2[N2+1];
		int ng2[N2+1];

		d_part_cond_compute_problem_size_libstr(N, nx, nu, nb, hidxb, ng, N2, nx2, nu2, nb2, ng2);

//		for(ii=0; ii<=N2; ii++)
//			printf("\n%d %d %d %d\n", nu2[ii], nx2[ii], nb2[ii], ng2[ii]);
//		exit(1);

		// data structure of partially condensed system
		struct blasfeo_dmat hsBAbt2[N2];
		struct blasfeo_dvec hsb2[N2];
		struct blasfeo_dmat hsRSQrq2[N2+1];
		struct blasfeo_dvec hsrq2[N2+1];
		struct blasfeo_dmat hsDCt2[N2+1];
		struct blasfeo_dvec hsd2[N2+1];
		struct blasfeo_dvec hsux2[N2+1];
		struct blasfeo_dvec hspi2[N2+1];
		struct blasfeo_dvec hslam2[N2+1];
		struct blasfeo_dvec hst2[N2+1];
		int *hidxb2[N2+1];
		void *memory_part_cond;
		void *work_part_cond;
		void *work_part_expand;
		int work_part_cond_sizes[5];
		int work_part_expand_sizes[2];

		// align (again) to (typical) cache line size
		addr = (( (size_t) c_ptr ) + 63 ) / 64 * 64;
		c_ptr = (char *) addr;

		work_part_cond = (void *) c_ptr;
		c_ptr += d_part_cond_work_space_size_bytes_libstr(N, nx, nu, nb, hidxb, ng, N2, nx2, nu2, nb2, ng2, work_part_cond_sizes);

		memory_part_cond = (void *) c_ptr;
		c_ptr += d_part_cond_memory_space_size_bytes_libstr(N, nx, nu, nb, hidxb, ng, N2, nx2, nu2, nb2, ng2);

		work_ipm = (void *) c_ptr;
		c_ptr += d_ip2_res_mpc_hard_work_space_size_bytes_libstr(N2, nx2, nu2, nb2, ng2);

		for(ii=0; ii<N2; ii++)
			{
			blasfeo_create_dmat(nu2[ii]+nx2[ii]+1, nx2[ii+1], &hsBAbt2[ii], (void *) c_ptr);
			c_ptr += hsBAbt2[ii].memsize;
			}

		for(ii=0; ii<N2; ii++) // not last stage !!!!!
			{
			blasfeo_create_dmat(nu2[ii]+nx2[ii]+1, nu2[ii]+nx2[ii], &hsRSQrq2[ii], (void *) c_ptr);
			c_ptr += hsRSQrq2[ii].memsize;
			}
		hsRSQrq2[N2] = hsRSQrq[N];

		for(ii=0; ii<N2; ii++) // not last stage !!!!!
			{
			blasfeo_create_dmat(nu2[ii]+nx2[ii], ng2[ii], &hsDCt2[ii], (void *) c_ptr);
			c_ptr += hsDCt2[ii].memsize;
			}
		hsDCt2[N2] = hsDCt[N];
		
		for(ii=0; ii<N2; ii++)
			{
			blasfeo_create_dvec(nx2[ii+1], &hsb2[ii], (void *) c_ptr);
			c_ptr += hsb2[ii].memsize;
			}

		for(ii=0; ii<N2; ii++) // not last stage !!!!!
			{
			blasfeo_create_dvec(nu2[ii]+nx2[ii], &hsrq2[ii], (void *) c_ptr);
			c_ptr += hsrq2[ii].memsize;
			}
		hsrq2[N2] = hsrq[N];

		for(ii=0; ii<N2; ii++) // not last stage !!!!!
			{
			blasfeo_create_dvec(2*nb2[ii]+2*ng2[ii], &hsd2[ii], (void *) c_ptr);
			c_ptr += hsd2[ii].memsize;
			}
		hsd2[N2] = hsd[N];

		for(ii=0; ii<N2; ii++) // not last stage !!!!!
			{
			blasfeo_create_dvec(nu2[ii]+nx2[ii], &hsux2[ii], (void *) c_ptr);
			c_ptr += hsux2[ii].memsize;
			}
		hsux2[N2] = hsux[N];

		for(ii=0; ii<N2; ii++) // not last stage !!!!!
			{
			blasfeo_create_dvec(nx2[ii], &hspi2[ii], (void *) c_ptr);
			c_ptr += hspi2[ii].memsize;
			}
		hspi2[N2] = hspi[N];

		for(ii=0; ii<N2; ii++) // not last stage !!!!!
			{
			blasfeo_create_dvec(2*nb2[ii]+2*ng2[ii], &hslam2[ii], (void *) c_ptr);
			c_ptr += hslam2[ii].memsize;
			}
		hslam2[N2] = hslam[N];

		for(ii=0; ii<N2; ii++) // not last stage !!!!!
			{
			blasfeo_create_dvec(2*nb2[ii]+2*ng2[ii], &hst2[ii], (void *) c_ptr);
			c_ptr += hst2[ii].memsize;
			}
		hst2[N2] = hst[N];

		for(ii=0; ii<N2; ii++) // not last stage !!!!!
			{
			hidxb2[ii] = (int *) c_ptr;
			c_ptr += nb2[ii]*sizeof(int);
			}
		hidxb2[N2] = hidxb[N];

		// partial condensing routine (computing also hidxb2) !!!
		d_part_cond_rhs_libstr(N, nx, nu, nb, hidxb, ng, hsBAbt, hsb, hsRSQrq, hsrq, hsDCt, hsd, N2, nx2, nu2, nb2, hidxb2, ng2, hsBAbt2, hsb2, hsRSQrq2, hsrq2, hsDCt2, hsd2, memory_part_cond, work_part_cond, &work_part_cond_sizes[2]);

#if 0
		for(ii=0; ii<N2; ii++)
			blasfeo_print_tran_dvec(nx2[ii+1], &hsb2[ii], 0);
		for(ii=0; ii<=N2; ii++)
			blasfeo_print_tran_dvec(nu2[ii]+nx2[ii], &hsrq2[ii], 0);
		for(ii=0; ii<=N2; ii++)
			blasfeo_print_tran_dvec(2*nb2[ii]+2*ng2[ii], &hsd2[ii], 0);
		for(ii=0; ii<=N2; ii++)
			int_print_mat(1, nb2[ii], hidxb2[ii], 1);
		exit(1);
#endif

		// IPM solver on partially condensed system
		d_kkt_solve_new_rhs_res_mpc_hard_libstr(N2, nx2, nu2, nb2, hidxb2, ng2, hsBAbt2, hsb2, hsRSQrq2, hsrq2, hsDCt2, hsd2, hsux2, 1, hspi2, hslam2, hst2, work_ipm);

#if 0
		for(ii=0; ii<=N2; ii++)
			blasfeo_print_tran_dvec(nu2[ii]+nx2[ii], &hsux2[ii], 0);
		for(ii=0; ii<N2; ii++)
			blasfeo_print_tran_dvec(nx2[ii+1], &hspi2[ii], 0);
		for(ii=0; ii<=N2; ii++)
			blasfeo_print_tran_dvec(2*nb2[ii]+2*ng2[ii], &hslam2[ii], 0);
		for(ii=0; ii<=N2; ii++)
			blasfeo_print_tran_dvec(2*nb2[ii]+2*ng2[ii], &hst2[ii], 0);
		exit(2);
#endif

		// expand work space
		work_part_expand = (void *) c_ptr;
		c_ptr += d_part_expand_work_space_size_bytes_libstr(N, nx, nu, nb, ng, work_part_expand_sizes);


		// expand solution of full space system
		d_part_expand_solution_libstr(N, nx, nu, nb, hidxb, ng, hsBAbt, hsb, hsRSQrq, hsrq, hsDCt, hsux, hspi, hslam, hst, N2, nx2, nu2, nb2, hidxb2, ng2, hsux2, hspi2, hslam2, hst2, work_part_expand, work_part_expand_sizes);

//		for(ii=0; ii<=N; ii++)
//			blasfeo_print_tran_dvec(nu[ii]+nx[ii], &hsux[ii], 0);
//		exit(2);

		}
	else // full space system
		{

		// align (again) to (typical) cache line size
		addr = (( (size_t) c_ptr ) + 63 ) / 64 * 64;
		c_ptr = (char *) addr;

		// ipm work space
		work_ipm = (void *) c_ptr;
		c_ptr += d_ip2_res_mpc_hard_work_space_size_bytes_libstr(N, nx, nu, nb, ng);

		// IPM solver on full space system
		d_kkt_solve_new_rhs_res_mpc_hard_libstr(N, nx, nu, nb, hidxb, ng, hsBAbt, hsb, hsRSQrq, hsrq, hsDCt, hsd, hsux, 1, hspi, hslam, hst, work_ipm);

		}

//	for(ii=0; ii<=N; ii++)
//		blasfeo_print_tran_dvec(nu[ii]+nx[ii], &hsux[ii], 0);
//	for(ii=0; ii<N; ii++)
//		blasfeo_print_tran_dvec(nx[ii+1], &hspi[ii], 0);
//	for(ii=0; ii<=N; ii++)
//		blasfeo_print_tran_dvec(2*nb[ii]+2*ng[ii], &hslam[ii], 0);
//	for(ii=0; ii<=N; ii++)
//		blasfeo_print_tran_dvec(2*nb[ii]+2*ng[ii], &hst[ii], 0);
//	exit(1);

	// copy back inputs and states
	for(ii=0; ii<N; ii++)
		blasfeo_unpack_dvec(nu[ii], &hsux[ii], 0, u[ii]);

	for(ii=0; ii<=N; ii++)
		blasfeo_unpack_dvec(nx[ii], &hsux[ii], nu[ii], x[ii]);



	// compute infinity norm of residuals on exit

	double mu;

	d_res_res_mpc_hard_libstr(N, nx, nu, nb, hidxb, ng, hsBAbt, hsb, hsRSQrq, hsrq, hsux, hsDCt, hsd, hspi, hslam, hst, hsrrq, hsrb, hsrd, hsrm, &mu, work_res);

//	for(ii=0; ii<=N; ii++)
//		blasfeo_print_exp_tran_dvec(nu[ii]+nx[ii], &hsrrq[ii], 0);
//	for(ii=0; ii<N; ii++)
//		blasfeo_print_exp_tran_dvec(nx[ii+1], &hsrb[ii], 0);
//	for(ii=0; ii<=N; ii++)
//		blasfeo_print_exp_tran_dvec(2*nb[ii]+2*ng[ii], &hsrd[ii], 0);
//	for(ii=0; ii<=N; ii++)
//		blasfeo_print_exp_tran_dvec(2*nb[ii]+2*ng[ii], &hsrm[ii], 0);
//	exit(1);
	
	double *ptr;

	ptr = hsrrq[0].pa;
	temp = fabs(ptr[0]);
	for(ii=0; ii<=N; ii++)
		{
		ptr = hsrrq[ii].pa;
		for(jj=0; jj<nu[ii]+nx[ii]; jj++) 
			temp = fmax( temp, fabs(ptr[jj]) );
		}
	inf_norm_res[0] = temp;

	ptr = hsrb[0].pa;
	temp = fabs(ptr[0]);
	for(ii=0; ii<N; ii++)
		{
		ptr = hsrb[ii].pa;
		for(jj=0; jj<nx[ii+1]; jj++) 
			temp = fmax( temp, fabs(ptr[jj]) );
		}
	inf_norm_res[1] = temp;

	ptr = hsrd[0].pa;
	temp = fabs(ptr[0]);
	for(ii=0; ii<=N; ii++)
		{
		ptr = hsrd[ii].pa;
		for(jj=0; jj<2*nb[ii]+2*ng[ii]; jj++) 
			{
			temp = fmax( temp, fabs(ptr[jj]) );
			}
		}
	inf_norm_res[2] = temp;

	ptr = hsrm[0].pa;
	temp = fabs(ptr[0]);
	for(ii=0; ii<=N; ii++)
		{
		ptr = hsrm[ii].pa;
		for(jj=0; jj<2*nb[ii]+2*ng[ii]; jj++) 
			{
			temp = fmax( temp, fabs(ptr[jj]) );
			}
		}
	inf_norm_res[3] = temp;

	inf_norm_res[4] = mu;

	// copy back multipliers

	for(ii=0; ii<N; ii++)
		blasfeo_unpack_dvec(nx[ii+1], &hspi[ii+1], 0, pi[ii]);

	for(ii=0; ii<=N; ii++)
		{
		blasfeo_unpack_dvec(2*nb[ii]+2*ng[ii], &hslam[ii], 0, lam[ii]);
//		blasfeo_unpack_dvec(2*nb[ii]+2*ng[ii], &hst[ii], 0, t[ii]);
		}

//	printf("\nend of wrapper\n");

    return;

	}



