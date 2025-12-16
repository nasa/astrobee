/**************************************************************************************************
*                                                                                                 *
* This file is part of HPIPM.                                                                     *
*                                                                                                 *
* HPIPM -- High-Performance Interior Point Method.                                                *
* Copyright (C) 2017-2018 by Gianluca Frison.                                                     *
* Developed at IMTEK (University of Freiburg) under the supervision of Moritz Diehl.              *
* All rights reserved.                                                                            *
*                                                                                                 *
* This program is free software: you can redistribute it and/or modify                            *
* it under the terms of the GNU General Public License as published by                            *
* the Free Software Foundation, either version 3 of the License, or                               *
* (at your option) any later version                                                              *.
*                                                                                                 *
* This program is distributed in the hope that it will be useful,                                 *
* but WITHOUT ANY WARRANTY; without even the implied warranty of                                  *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                                   *
* GNU General Public License for more details.                                                    *
*                                                                                                 *
* You should have received a copy of the GNU General Public License                               *
* along with this program.  If not, see <https://www.gnu.org/licenses/>.                          *
*                                                                                                 *
* The authors designate this particular file as subject to the "Classpath" exception              *
* as provided by the authors in the LICENSE file that accompained this code.                      *
*                                                                                                 *
* Author: Gianluca Frison, gianluca.frison (at) imtek.uni-freiburg.de                             *
*                                                                                                 *
**************************************************************************************************/



#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <sys/time.h>

#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_v_aux_ext_dep.h>
#include <blasfeo_d_aux_ext_dep.h>
#include <blasfeo_i_aux_ext_dep.h>
#include <blasfeo_d_aux.h>
#include <blasfeo_d_blas.h>

#include "../include/hpipm_d_rk_int.h"
#include "../include/hpipm_d_erk_int.h"
#include "../include/hpipm_d_ocp_qp.h"
#include "../include/hpipm_d_ocp_qp_sol.h"
#include "../include/hpipm_d_ocp_qp_ipm.h"
#include "../include/hpipm_d_ocp_nlp.h"
#include "../include/hpipm_d_ocp_nlp_sol.h"
#include "../include/hpipm_d_ocp_nlp_hyb.h"



int odeFun(double **arg, double **res, int *iw, double *w, int mem);
int vdeFun(double **arg, double **res, int *iw, double *w, int mem);
int adjFun(double **arg, double **res, int *iw, double *w, int mem);



struct vde_fun_arg
	{
	int nx;
	int nu;
	};



void ode_fun_model(int t, double *x, double *u, void *ode_arg, double *out)
	{

	struct vde_fun_arg *arg = ode_arg;

	int nx = arg->nx;
	int nu = arg->nu;

	double *x_out = out;

	double *casadi_arg[2];
	double *casadi_res[1];

	casadi_arg[0] = x;
	casadi_arg[1] = u;

	casadi_res[0] = x_out;

	int* iw = 0;
	double* w = 0;
	int mem = 0;

	odeFun(casadi_arg, casadi_res, iw, w, mem);

	return;

	}


void vde_fun_model(int t, double *x, double *u, void *ode_arg, double *out)
	{

	struct vde_fun_arg *arg = ode_arg;

	int nx = arg->nx;
	int nu = arg->nu;

	double *Su = x + nx;
	double *Sx = x + nx + nu * nx;

	double *x_out = out;
	double *Su_out = out + nx;
	double *Sx_out = out + nx + nu * nx;

	double *casadi_arg[4];
	double *casadi_res[3];

	casadi_arg[0] = x;
	casadi_arg[1] = Sx;
	casadi_arg[2] = Su;
	casadi_arg[3] = u;

	casadi_res[0] = x_out;
	casadi_res[1] = Sx_out;
	casadi_res[2] = Su_out;

	int* iw = 0;
	double* w = 0;
	int mem = 0;

	vdeFun(casadi_arg, casadi_res, iw, w, mem);

	return;

	}


void vde_adj_model(int t, double *adj_in, void *ode_arg, double *adj_out)
	{

	struct vde_fun_arg *arg = ode_arg;

	int nx = arg->nx;
	int nu = arg->nu;

	// extract inputs
	double *x = adj_in + 0;
	double *l = adj_in + nx;
	double *u = adj_in + nx + nx;

	// extract output
	double *l_u_out = adj_out + 0;
	double *l_x_out = adj_out + nu;

	double *casadi_arg[3];
	double *casadi_res[1];

	casadi_arg[0] = x;
	casadi_arg[1] = l;
	casadi_arg[2] = u;

	casadi_res[0] = l_u_out;

	int* iw = 0;
	double* w = 0;
	int mem = 0;

	adjFun(casadi_arg, casadi_res, iw, w, mem);

	return;

	}



int main()
	{

	int ii;

/************************************************
* problem size
************************************************/	
	
	int nx_ = 4;
	int nu_ = 1;
	int N   = 25;

/************************************************
* initial state and control
************************************************/	
	
	double *x0 = malloc(nx_*sizeof(double));
	x0[0] = 0.0;
	x0[1] = 0.5;
	x0[2] = 0.0;
	x0[3] = 0.0;

	double *u0 = malloc(nu_*sizeof(double));
	u0[0] = 0.0;

/************************************************
* quadratic cost function
************************************************/	
	
	double *Q; d_zeros(&Q, nx_, nx_);
//	for(ii=0; ii<nx_; ii++) Q[ii*(nx_+1)] = 10.0;
	Q[0+nx_*0] = 1;
	Q[1+nx_*1] = 1000;
	Q[2+nx_*2] = 1;
	Q[3+nx_*3] = 0.1;

	double *R; d_zeros(&R, nu_, nu_);
	for(ii=0; ii<nu_; ii++) R[ii*(nu_+1)] = 0.1;

	double *S; d_zeros(&S, nu_, nx_);

	double *q; d_zeros(&q, nx_, 1);
	for(ii=0; ii<nx_; ii++) q[ii] = 0.0;

	double *r; d_zeros(&r, nu_, 1);
	for(ii=0; ii<nu_; ii++) r[ii] = 0.0;

	d_print_mat(nx_, nx_, Q, nx_);
	d_print_mat(nu_, nu_, R, nu_);
	d_print_mat(nu_, nx_, S, nu_);
	d_print_mat(1, nx_, q, 1);
	d_print_mat(1, nu_, r, 1);

/************************************************
* integrator type and arg
************************************************/	
	
#if 1
	// rk4
	int nsta = 4; // number of stages
	int expl = 1;
	double A_rk[] = {0.0, 0.0, 0.0, 0.0,
	                 0.5, 0.0, 0.0, 0.0,
	                 0.0, 0.5, 0.0, 0.0,
	                 0.0, 0.0, 1.0, 0.0};
	double B_rk[] = {1.0/6.0, 1.0/3.0, 1.0/3.0, 1.0/6.0};
	double C_rk[] = {0.0, 0.5, 0.5, 0.0};
#elif 1
	// midpoint rule
	int nsta = 2; // number of stages
	int expl = 1;
	double A_rk[] = {0.0, 0.0,
	                 0.5, 0.0};
	double B_rk[] = {0.0, 1.0};
	double C_rk[] = {0.0, 0.5};
#else
	// explicit euler
	int nsta = 1; // number of stages
	int expl = 1;
	double A_rk[] = {0.0};
	double B_rk[] = {1.0};
	double C_rk[] = {0.0};
#endif

	// erk data structure
	hpipm_size_t memsize_rk_data = d_memsize_rk_data(nsta);
	printf("\nmemsize rk data %d\n", memsize_rk_data);
	void *memory_rk_data = malloc(memsize_rk_data);

	struct d_rk_data rk_data;
	d_create_rk_data(nsta, &rk_data, memory_rk_data);

	d_cvt_rowmaj_to_rk_data(expl, A_rk, B_rk, C_rk, &rk_data);

	double Ts = 0.1;

	// erk arg structure
	struct d_erk_arg erk_arg;
	erk_arg.rk_data = &rk_data;
	erk_arg.steps = 10;
	erk_arg.h = Ts/erk_arg.steps;
	erk_arg.for_sens = 1; // XXX needed ???
	erk_arg.adj_sens = 1;

/************************************************
* ocp qp
************************************************/	

	int nx[N+1];
	int nu[N+1];
	int nb[N+1];
	int ng[N+1];
	int ns[N+1];

	nx[0] = nx_;//0;
	nu[0] = nu_;
	nb[0] = nu_+nx_;
	ng[0] = 0;
	ns[0] = 0;
	for(ii=1; ii<N; ii++)
		{
		nx[ii] = nx_;
		nu[ii] = nu_;
		nb[ii] = nu_;
		ng[ii] = 0;
		ns[ii] = 0;
		}
	nx[N] = nx_;
	nu[N] = 0;
	nb[N] = 0;//nx_;
	ng[N] = 0;
	ns[N] = 0;

/************************************************
* box & general constraints
************************************************/	

	int *idxb0; int_zeros(&idxb0, nb[0], 1);
	double *d_lb0; d_zeros(&d_lb0, nb[0], 1);
	double *d_ub0; d_zeros(&d_ub0, nb[0], 1);
	double *d_lg0; d_zeros(&d_lg0, ng[0], 1);
	double *d_ug0; d_zeros(&d_ug0, ng[0], 1);
	for(ii=0; ii<nb[0]; ii++)
		{
		if(ii<nu[0]) // input
			{
			d_lb0[ii] = - 10.0; // umin
			d_ub0[ii] =   10.0; // umax
			}
		else // (initial) state
			{
			d_lb0[ii] = x0[ii-nu[0]]; // xmin
			d_ub0[ii] = x0[ii-nu[0]]; // xmax
			}
		idxb0[ii] = ii;
		}

	int *idxb1; int_zeros(&idxb1, nb[1], 1);
	double *d_lb1; d_zeros(&d_lb1, nb[1], 1);
	double *d_ub1; d_zeros(&d_ub1, nb[1], 1);
	double *d_lg1; d_zeros(&d_lg1, ng[1], 1);
	double *d_ug1; d_zeros(&d_ug1, ng[1], 1);
	for(ii=0; ii<nb[1]; ii++)
		{
		if(ii<nu[1]) // input
			{
			d_lb1[ii] = - 10.0; // umin
			d_ub1[ii] =   10.0; // umax
			idxb1[ii] = ii;
			}
		}

	int *idxbN; int_zeros(&idxbN, nb[N], 1);
	double *d_lbN; d_zeros(&d_lbN, nb[N], 1);
	double *d_ubN; d_zeros(&d_ubN, nb[N], 1);
	double *d_lgN; d_zeros(&d_lgN, ng[N], 1);
	double *d_ugN; d_zeros(&d_ugN, ng[N], 1);
	if(nb[N]==nx[N])
		{
		d_lbN[0] = - 1000.0; //
		d_ubN[0] =   1000.0; //
		idxbN[0] = 0;
		d_lbN[1] = - 0.0; //
		d_ubN[1] =   0.0; //
		idxbN[1] = 1;
		d_lbN[2] = - 10.0; //
		d_ubN[2] =   10.0; //
		idxbN[2] = 2;
		d_lbN[3] = - 10.0; //
		d_ubN[3] =   10.0; //
		idxbN[3] = 3;
		}

	double *C0; d_zeros(&C0, ng[0], nx[0]);
	double *D0; d_zeros(&D0, ng[0], nu[0]);

	double *C1; d_zeros(&C1, ng[1], nx[1]);
	double *D1; d_zeros(&D1, ng[1], nu[1]);

	double *CN; d_zeros(&CN, ng[N], nx[N]);
	double *DN; d_zeros(&DN, ng[N], nu[N]);

#if 0
	// box constraints
	int_print_mat(1, nb[0], idxb0, 1);
	d_print_mat(1, nb[0], d_lb0, 1);
	d_print_mat(1, nb[0], d_ub0, 1);
	int_print_mat(1, nb[1], idxb1, 1);
	d_print_mat(1, nb[1], d_lb1, 1);
	d_print_mat(1, nb[1], d_ub1, 1);
	int_print_mat(1, nb[N], idxbN, 1);
	d_print_mat(1, nb[N], d_lbN, 1);
	d_print_mat(1, nb[N], d_ubN, 1);
	// general constraints
	d_print_mat(1, ng[0], d_lg0, 1);
	d_print_mat(1, ng[0], d_ug0, 1);
	d_print_mat(ng[0], nu[0], D0, ng[0]);
	d_print_mat(ng[0], nx[0], C0, ng[0]);
	d_print_mat(1, ng[1], d_lg1, 1);
	d_print_mat(1, ng[1], d_ug1, 1);
	d_print_mat(ng[1], nu[1], D1, ng[1]);
	d_print_mat(ng[1], nx[1], C1, ng[1]);
	d_print_mat(1, ng[N], d_lgN, 1);
	d_print_mat(1, ng[N], d_ugN, 1);
	d_print_mat(ng[N], nu[N], DN, ng[N]);
	d_print_mat(ng[N], nx[N], CN, ng[N]);
	exit(1);
#endif

/************************************************
* soft constraints
************************************************/	

	double *Zl0; d_zeros(&Zl0, ns[0], 1);
	for(ii=0; ii<ns[0]; ii++)
		Zl0[ii] = 1e3;
	double *Zu0; d_zeros(&Zu0, ns[0], 1);
	for(ii=0; ii<ns[0]; ii++)
		Zu0[ii] = 1e3;
	double *zl0; d_zeros(&zl0, ns[0], 1);
	for(ii=0; ii<ns[0]; ii++)
		zl0[ii] = 1e2;
	double *zu0; d_zeros(&zu0, ns[0], 1);
	for(ii=0; ii<ns[0]; ii++)
		zu0[ii] = 1e2;
	int *idxs0; int_zeros(&idxs0, ns[0], 1);
	for(ii=0; ii<ns[0]; ii++)
		idxs0[ii] = nu[0]+ii;

	double *Zl1; d_zeros(&Zl1, ns[1], 1);
	for(ii=0; ii<ns[1]; ii++)
		Zl1[ii] = 1e3;
	double *Zu1; d_zeros(&Zu1, ns[1], 1);
	for(ii=0; ii<ns[1]; ii++)
		Zu1[ii] = 1e3;
	double *zl1; d_zeros(&zl1, ns[1], 1);
	for(ii=0; ii<ns[1]; ii++)
		zl1[ii] = 1e2;
	double *zu1; d_zeros(&zu1, ns[1], 1);
	for(ii=0; ii<ns[1]; ii++)
		zu1[ii] = 1e2;
	int *idxs1; int_zeros(&idxs1, ns[1], 1);
	for(ii=0; ii<ns[1]; ii++)
		idxs1[ii] = nu[1]+ii;

	double *ZlN; d_zeros(&ZlN, ns[N], 1);
	for(ii=0; ii<ns[N]; ii++)
		ZlN[ii] = 1e3;
	double *ZuN; d_zeros(&ZuN, ns[N], 1);
	for(ii=0; ii<ns[N]; ii++)
		ZuN[ii] = 1e3;
	double *zlN; d_zeros(&zlN, ns[N], 1);
	for(ii=0; ii<ns[N]; ii++)
		zlN[ii] = 1e2;
	double *zuN; d_zeros(&zuN, ns[N], 1);
	for(ii=0; ii<ns[N]; ii++)
		zuN[ii] = 1e2;
	int *idxsN; int_zeros(&idxsN, ns[N], 1);
	for(ii=0; ii<ns[N]; ii++)
		idxsN[ii] = nu[N]+ii;

#if 0
	// soft constraints
	int_print_mat(1, ns[0], idxs0, 1);
	d_print_mat(1, ns[0], Zl0, 1);
	d_print_mat(1, ns[0], Zu0, 1);
	d_print_mat(1, ns[0], zl0, 1);
	d_print_mat(1, ns[0], zu0, 1);
	int_print_mat(1, ns[1], idxs1, 1);
	d_print_mat(1, ns[1], Zl1, 1);
	d_print_mat(1, ns[1], Zu1, 1);
	d_print_mat(1, ns[1], zl1, 1);
	d_print_mat(1, ns[1], zu1, 1);
	int_print_mat(1, ns[N], idxsN, 1);
	d_print_mat(1, ns[N], ZlN, 1);
	d_print_mat(1, ns[N], ZuN, 1);
	d_print_mat(1, ns[N], zlN, 1);
	d_print_mat(1, ns[N], zuN, 1);
#endif

/************************************************
* input and state reference
************************************************/	

	double *x_ref = malloc(nx_*sizeof(double));
	for(ii=0; ii<nx_; ii++) x_ref[ii] = 0.0;
	
	double *u_ref = malloc(nu_*sizeof(double));
	for(ii=0; ii<nu_; ii++) u_ref[ii] = 0.0;
	
/************************************************
* ocp nlp model
************************************************/	

	// stage 0
//	double *fs0 = malloc(nx_*nu_*sizeof(double));
//	for(ii=0; ii<nx_*nu_; ii++)
//		fs0[ii] = 0.0;
//
//	struct d_ocp_nlp_model model0;
//	model0.expl_vde = &d_van_der_pol_vde0;
//	model0.forward_seed = fs0;
//	model0.arg = NULL;

	// stage 1
	double *fs1 = malloc(nx_*(nu_+nx_)*sizeof(double));
	for(ii=0; ii<nx_*(nu_+nx_); ii++)
		fs1[ii] = 0.0;
	for(ii=0; ii<nx_; ii++)
		fs1[nu_*nx_+ii*(nx_+1)] = 1.0;

	struct vde_fun_arg vde_arg;
	vde_arg.nx = nx_;
	vde_arg.nu = nu_;

	struct d_ocp_nlp_model model1;
	model1.expl_ode = &ode_fun_model;
	model1.expl_vde_for = &vde_fun_model;
	model1.expl_vde_adj = &vde_adj_model;
	model1.forward_seed = fs1;
	model1.arg = &vde_arg;

/************************************************
* ocp nlp data
************************************************/	

	struct d_ocp_nlp_model models[N];
	double *hQ[N+1];
	double *hS[N+1];
	double *hR[N+1];
	double *hx_ref[N+1];
	double *hu_ref[N+1];
	double *hd_lb[N+1];
	double *hd_ub[N+1];
	double *hd_lg[N+1];
	double *hd_ug[N+1];
	double *hC[N+1];
	double *hD[N+1];
	int *hidxb[N+1];
	double *hZl[N+1];
	double *hZu[N+1];
	double *hzl[N+1];
	double *hzu[N+1];
	int *hidxs[N+1]; // XXX

	models[0] = model1;
	hQ[0] = Q;
	hS[0] = S;
	hR[0] = R;
	hx_ref[0] = x_ref;
	hu_ref[0] = u_ref;
	hidxb[0] = idxb0;
	hd_lb[0] = d_lb0;
	hd_ub[0] = d_ub0;
	hd_lg[0] = d_lg0;
	hd_ug[0] = d_ug0;
	hC[0] = C0;
	hD[0] = D0;
	hZl[0] = Zl0;
	hZu[0] = Zu0;
	hzl[0] = zl0;
	hzu[0] = zu0;
	hidxs[0] = idxs0;
	for(ii=1; ii<N; ii++)
		{
		models[ii] = model1;
		hQ[ii] = Q;
		hS[ii] = S;
		hR[ii] = R;
		hx_ref[ii] = x_ref;
		hu_ref[ii] = u_ref;
		hidxb[ii] = idxb1;
		hd_lb[ii] = d_lb1;
		hd_ub[ii] = d_ub1;
		hd_lg[ii] = d_lg1;
		hd_ug[ii] = d_ug1;
		hC[ii] = C1;
		hD[ii] = D1;
		hZl[ii] = Zl1;
		hZu[ii] = Zu1;
		hzl[ii] = zl1;
		hzu[ii] = zu1;
		hidxs[ii] = idxs1;
		}
	hQ[N] = Q;
	hS[N] = S;
	hR[N] = R;
	hx_ref[N] = x_ref;
	hu_ref[N] = u_ref;
	hidxb[N] = idxbN;
	hd_lb[N] = d_lbN;
	hd_ub[N] = d_ubN;
	hd_lg[N] = d_lgN;
	hd_ug[N] = d_ugN;
	hC[N] = CN;
	hD[N] = DN;
	hZl[N] = ZlN;
	hZu[N] = ZuN;
	hzl[N] = zlN;
	hzu[N] = zuN;
	hidxs[N] = idxsN;
	
/************************************************
* ocp nlp
************************************************/	
	
	hpipm_size_t nlp_size = d_memsize_ocp_nlp(N, nx, nu, nb, ng, ns);
	printf("\nnlpsize = %d\n", nlp_size);
	void *nlp_mem = malloc(nlp_size);

	struct d_ocp_nlp nlp;
	d_create_ocp_nlp(N, nx, nu, nb, ng, ns, &nlp, nlp_mem);

	d_cvt_colmaj_to_ocp_nlp(models, hQ, hS, hR, hx_ref, hu_ref, hidxb, hd_lb, hd_ub, hC, hD, hd_lg, hd_ug, hZl, hZu, hzl, hzu, hidxs, &nlp);

/************************************************
* ocp nlp sol
************************************************/	
	
	hpipm_size_t nlp_sol_size = d_memsize_ocp_nlp_sol(N, nx, nu, nb, ng, ns);
	printf("\nnlp sol size = %d\n", nlp_sol_size);
	void *nlp_sol_mem = malloc(nlp_sol_size);

	struct d_ocp_nlp_sol nlp_sol;
	d_create_ocp_nlp_sol(N, nx, nu, nb, ng, ns, &nlp_sol, nlp_sol_mem);

/************************************************
* ocp nlp hyb arg
************************************************/	

	hpipm_size_t hyb_arg_size = d_memsize_ocp_nlp_hyb_arg(&nlp);
	printf("\nipm arg size = %d\n", hyb_arg_size);
	void *hyb_arg_mem = malloc(hyb_arg_size);

	struct d_ocp_nlp_hyb_arg hyb_arg;
	d_create_ocp_nlp_hyb_arg(&nlp, &hyb_arg, hyb_arg_mem);
	d_set_default_ocp_nlp_hyb_arg(&hyb_arg);

	struct d_erk_arg erk_args[N];
	for(ii=0; ii<N; ii++)
		erk_args[ii] = erk_arg;

//	struct d_ocp_nlp_hyb_arg hyb_arg;
//	hyb_arg.ipm_arg = &arg;
	hyb_arg.erk_arg = erk_args;
//	hyb_arg.alpha_min = 1e-8;
	hyb_arg.nlp_res_g_max = 1e-6;
	hyb_arg.nlp_res_b_max = 1e-8;
	hyb_arg.nlp_res_d_max = 1e-8;
	hyb_arg.nlp_res_m_max = 1e-8;
//	hyb_arg.nlp_iter_max = 20;
//	hyb_arg.stat_max = 20;
	hyb_arg.N2 = 2;
//	hyb_arg.pred_corr = 1;

/************************************************
* ipm arg
************************************************/	

//	struct d_ocp_qp_ipm_arg arg;
//	arg.alpha_min = 1e-8;
//	arg.res_g_max = 1e-1;
//	arg.res_b_max = 1e-1;
//	arg.res_d_max = 1e-1;
//	arg.res_m_max = 1e-0;
//	arg.mu0 = 1000.0;
//	arg.iter_max = 20;
//	arg.stat_max = 20;
//	arg.pred_corr = 1;

/************************************************
* ocp nlp hyb ws
************************************************/	

	hpipm_size_t nlp_ws_size = d_memsize_ocp_nlp_hyb(&nlp, &hyb_arg);
	printf("\nnlp ws size = %d\n", nlp_ws_size);
	void *nlp_ws_mem = malloc(nlp_ws_size);
	
	struct d_ocp_nlp_hyb_workspace hyb_ws;
	d_create_ocp_nlp_hyb(&nlp, &hyb_arg, &hyb_ws, nlp_ws_mem);

/************************************************
* ocp nlp hyb
************************************************/	

	int nlp_return;

	struct timeval tv0, tv1;
	int rep, nrep = 100;

	gettimeofday(&tv0, NULL); // start

	for(rep=0; rep<nrep; rep++)
		{
		nlp_return = d_solve_ocp_nlp_hyb(&nlp, &nlp_sol, &hyb_arg, &hyb_ws);
		}

	gettimeofday(&tv1, NULL); // stop

	double time_ocp_nlp_hyb = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

/************************************************
* extract and print solution
************************************************/	

	double *u[N+1]; for(ii=0; ii<=N; ii++) d_zeros(u+ii, nu[ii], 1);
	double *x[N+1]; for(ii=0; ii<=N; ii++) d_zeros(x+ii, nx[ii], 1);
	double *ls[N+1]; for(ii=0; ii<=N; ii++) d_zeros(ls+ii, ns[ii], 1);
	double *us[N+1]; for(ii=0; ii<=N; ii++) d_zeros(us+ii, ns[ii], 1);
	double *pi[N]; for(ii=0; ii<N; ii++) d_zeros(pi+ii, nx[ii+1], 1);
	double *lam_lb[N+1]; for(ii=0; ii<=N; ii++) d_zeros(lam_lb+ii, nb[ii], 1);
	double *lam_ub[N+1]; for(ii=0; ii<=N; ii++) d_zeros(lam_ub+ii, nb[ii], 1);
	double *lam_lg[N+1]; for(ii=0; ii<=N; ii++) d_zeros(lam_lg+ii, ng[ii], 1);
	double *lam_ug[N+1]; for(ii=0; ii<=N; ii++) d_zeros(lam_ug+ii, ng[ii], 1);
	double *lam_ls[N+1]; for(ii=0; ii<=N; ii++) d_zeros(lam_ls+ii, ns[ii], 1);
	double *lam_us[N+1]; for(ii=0; ii<=N; ii++) d_zeros(lam_us+ii, ns[ii], 1);

	d_cvt_ocp_nlp_sol_to_colmaj(&nlp, &nlp_sol, u, x, ls, us, pi, lam_lb, lam_ub, lam_lg, lam_ug, lam_ls, lam_us);

#if 1
	printf("\nalpha_aff\tmu_aff\t\tsigma\t\talpha\t\tmu\n");
	d_print_exp_tran_mat(5, hyb_ws.iter_qp+hyb_ws.iter_nlp, hyb_ws.ipm_workspace->stat, 5);

	printf("\nsolution\n\n");
	printf("\nu\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, nu[ii], u[ii], 1);
	printf("\nx\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, nx[ii], x[ii], 1);
//	printf("\nls\n");
//	for(ii=0; ii<=N; ii++)
//		d_print_mat(1, ns[ii], ls[ii], 1);
//	printf("\nus\n");
//	for(ii=0; ii<=N; ii++)
//		d_print_mat(1, ns[ii], us[ii], 1);
	printf("\npi\n");
	for(ii=0; ii<N; ii++)
		d_print_mat(1, nx[ii+1], pi[ii], 1);
	printf("\nlam_lb\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, nb[ii], lam_lb[ii], 1);
	printf("\nlam_ub\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, nb[ii], lam_ub[ii], 1);
//	printf("\nlam_lg\n");
//	for(ii=0; ii<=N; ii++)
//		d_print_mat(1, ng[ii], lam_lg[ii], 1);
//	printf("\nlam_ug\n");
//	for(ii=0; ii<=N; ii++)
//		d_print_mat(1, ng[ii], lam_ug[ii], 1);
//	printf("\nlam_ls\n");
//	for(ii=0; ii<=N; ii++)
//		d_print_mat(1, ns[ii], lam_ls[ii], 1);
//	printf("\nlam_us\n");
//	for(ii=0; ii<=N; ii++)
//		d_print_mat(1, ns[ii], lam_us[ii], 1);

//	printf("\nt_lb\n");
//	for(ii=0; ii<=N; ii++)
//		d_print_mat(1, nb[ii], (nlp_sol.t+ii)->pa, 1);
//	printf("\nt_ub\n");
//	for(ii=0; ii<=N; ii++)
//		d_print_mat(1, nb[ii], (nlp_sol.t+ii)->pa+nb[ii]+ng[ii], 1);
//	printf("\nt_lg\n");
//	for(ii=0; ii<=N; ii++)
//		d_print_mat(1, ng[ii], (nlp_sol.t+ii)->pa+nb[ii], 1);
//	printf("\nt_ug\n");
//	for(ii=0; ii<=N; ii++)
//		d_print_mat(1, ng[ii], (nlp_sol.t+ii)->pa+2*nb[ii]+ng[ii], 1);
//	printf("\nt_ls\n");
//	for(ii=0; ii<=N; ii++)
//		d_print_mat(1, ns[ii], (nlp_sol.t+ii)->pa+2*nb[ii]+2*ng[ii], 1);
//	printf("\nt_us\n");
//	for(ii=0; ii<=N; ii++)
//		d_print_mat(1, ns[ii], (nlp_sol.t+ii)->pa+2*nb[ii]+2*ng[ii]+ns[ii], 1);
#endif

/************************************************
* print hyb statistics
************************************************/	

	printf("\nnlp_res_g = %e, nlp_res_b = %e, nlp_res_d = %e, nlp_res_m = %e\n", hyb_ws.nlp_res_g, hyb_ws.nlp_res_b, hyb_ws.nlp_res_d, hyb_ws.nlp_res_m);
	printf("\nocp nlp hyb: iter_qp = %d, iter_nlp = %d, time = %e [s]\n\n", hyb_ws.iter_qp, hyb_ws.iter_nlp, time_ocp_nlp_hyb);

/************************************************
* free memory
************************************************/	
	
	free(x0);
	free(u0);
	
/************************************************
* return
************************************************/	
	
	return 0;

	}
