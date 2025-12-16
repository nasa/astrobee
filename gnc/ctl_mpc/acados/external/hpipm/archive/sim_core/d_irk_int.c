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



#if defined(RUNTIME_CHECKS)
#include <stdlib.h>
#include <stdio.h>
#endif

#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_d_aux.h>
#include <blasfeo_d_blas.h>

#include "../include/hpipm_d_rk_int.h"
#include "../include/hpipm_d_irk_int.h"




hpipm_size_t d_memsize_irk_int(struct d_rk_data *rk_data, int nx, int nf, int np)
	{

	int ns = rk_data->ns;

	int nX = nx*(1+nf);

	hpipm_size_t size = 0;

	size += 3*sizeof(struct blasfeo_dmat); // JG rG K

	size += 1*blasfeo_memsize_dmat(nx*ns, nx*ns); // JG
	size += 2*blasfeo_memsize_dmat(nx*ns, nf+1); // rG K

	size += 1*nx*sizeof(double); // xt1
	size += 4*nX*sizeof(double); // x xt0 Kt rGt
	size += 1*np*sizeof(double); // p
	size += 2*nx*nx*sizeof(double); // Jt0 Jt1

	size += 1*nx*ns*sizeof(int); // ipiv

	size = (size+63)/64*64; // make multiple of typical cache line size
	size += 64; // align to typical cache line size

	return size;

	}



void d_create_irk_int(struct d_rk_data *rk_data, int nx, int nf, int np, struct d_irk_workspace *ws, void *mem)
	{

	ws->rk_data = rk_data;
	ws->nx = nx;
	ws->nf = nf;
	ws->np = np;

	int ns = rk_data->ns;

	int nX = nx*(1+nf);


	// matrix struct stuff
	struct blasfeo_dmat *sm_ptr = (struct blasfeo_dmat *) mem;

	// JG
	ws->JG = sm_ptr;
	sm_ptr += 1;
	// rG
	ws->rG = sm_ptr;
	sm_ptr += 1;
	// K
	ws->K = sm_ptr;
	sm_ptr += 1;


	// double stuff
	double *d_ptr = (double *) sm_ptr;

	// x
	ws->x = d_ptr;
	d_ptr += nX;
	// xt0
	ws->xt0 = d_ptr;
	d_ptr += nX;
	// xt1
	ws->xt1 = d_ptr;
	d_ptr += nX;
	// Kt
	ws->Kt = d_ptr;
	d_ptr += nX;
	// Kt
	ws->rGt = d_ptr;
	d_ptr += nX;
	// p
	ws->p = d_ptr;
	d_ptr += np;
	// Jt0
	ws->Jt0 = d_ptr;
	d_ptr += nx*nx;
	// Jt1
	ws->Jt1 = d_ptr;
	d_ptr += nx*nx;


	// int stuff
	int *i_ptr = (int *) d_ptr;

	// ipiv
	ws->ipiv = i_ptr;
	i_ptr += ns*nx;


	// align to typical cache line size
	long long l_ptr = (long long) i_ptr;
	l_ptr = (l_ptr+63)/64*64;


	// void stuff
	char *c_ptr;
	c_ptr = (char *) l_ptr;

	// JG
	blasfeo_create_dmat(nx*ns, nx*ns, ws->JG, c_ptr);
	c_ptr += ws->JG->memsize;
	// rG
	blasfeo_create_dmat(nx*ns, nf+1, ws->rG, c_ptr);
	c_ptr += ws->rG->memsize;
	// rG
	blasfeo_create_dmat(nx*ns, nf+1, ws->K, c_ptr);
	c_ptr += ws->K->memsize;



	ws->memsize = d_memsize_irk_int(rk_data, nx, nf, np);


#if defined(RUNTIME_CHECKS)
	if(c_ptr > ((char *) mem) + ws->memsize)
		{
		printf("\nCreate_irk_int: outsize memory bounds!\n\n");
		exit(1);
		}
#endif


	return;

	}



void d_init_irk_int(double *x0, double *fs0, double *p0, void (*d_res_impl_vde)(int t, double *xdot, double *x, double *p, void *ode_args, double *res), void (*d_jac_impl_ode)(int t, double *xdot, double *x, double *p, void *ode_args, double *jac), void *ode_args, struct d_irk_workspace *ws)
	{

	int ii;

	struct d_rk_data *rk_data = ws->rk_data;
	int ns = rk_data->ns;

	int nx = ws->nx;
	int nf = ws->nf;
	int np = ws->np;
	struct blasfeo_dmat *K = ws->K;

	int nX = nx*(1+nf);

	double *x = ws->x;
	double *p = ws->p;

	for(ii=0; ii<nx*nf; ii++)
		x[ii] = fs0[ii];

	for(ii=0; ii<nx; ii++)
		x[nx*nf+ii] = x0[ii];

	for(ii=0; ii<np; ii++)
		p[ii] = p0[ii];
	
	// TODO initialize K !!!!!
//	for(ii=0; ii<ns; ii++)
//		blasfeo_pack_dmat(nx, nf+1, xdot0, nx, K, ii*nx, 0);
	blasfeo_dgese(nx*ns, nf+1, 0.0, K, 0, 0);

	ws->d_res_impl_vde = d_res_impl_vde;
	ws->d_jac_impl_ode = d_jac_impl_ode;
	ws->ode_args = ode_args;

	return;

	}



void d_irk_int(struct d_irk_args *irk_args, struct d_irk_workspace *ws)
	{

	int steps = irk_args->steps;
	int newton_iter = irk_args->newton_iter;
	double h = irk_args->h;

	struct d_rk_data *rk_data = ws->rk_data;
	int nx = ws->nx;
	int nf = ws->nf;
	double *x = ws->x;
	double *p = ws->p;
	double *xt0 = ws->xt0;
	double *xt1 = ws->xt1;
	double *Kt = ws->Kt;
	double *rGt = ws->rGt;
	double *Jt0 = ws->Jt0;
	double *Jt1 = ws->Jt1;
	int *ipiv = ws->ipiv;
	struct blasfeo_dmat *JG = ws->JG;
	struct blasfeo_dmat *rG = ws->rG;
	struct blasfeo_dmat *K = ws->K;

	int ns = rk_data->ns;
	double *A_rk = rk_data->A_rk;
	double *B_rk = rk_data->B_rk;
	double *C_rk = rk_data->C_rk;

	int nX = nx*(1+nf);

	struct blasfeo_dvec sxt0;
	blasfeo_create_dvec(nX, &sxt0, xt0);
	struct blasfeo_dvec sxt1;
	blasfeo_create_dvec(nx, &sxt1, xt1);

	int ii, jj, step, iter, ss;
	double t, a, b;

	t = 0.0;
	for(step=0; step<steps; step++)
		{
		for(iter=0; iter<newton_iter; iter++)
			{
//			printf("\newton_iter = %d\n", iter);
			for(ss=0; ss<ns; ss++)
				{
//				printf("\nss = %d\n", ss);
				for(ii=0; ii<nX; ii++)
					xt0[ii] = x[ii];
				for(ii=0; ii<ns; ii++)
					{
					a = A_rk[ss+ns*ii];
					if(a!=0)
						{
						a = a * h;
						for(jj=0; jj<1+nf; jj++)
							{
							blasfeo_dcolex(nx, K, ii*nx, jj, &sxt1, 0);
							blasfeo_daxpy(nx, a, &sxt1, 0, &sxt0, jj*nx, &sxt0, jj*nx);
							}
						}
					}
				blasfeo_unpack_dmat(nx, nf+1, K, ss*nx, 0, Kt, nx);
				ws->d_res_impl_vde(t+h*C_rk[ss], Kt, xt0, p, ws->ode_args, rGt);
				ws->d_jac_impl_ode(t+h*C_rk[ss], Kt, xt0, p, ws->ode_args, Jt0);
				blasfeo_pack_dmat(nx, nf+1, rGt, nx, rG, ss*nx, 0);
				for(ii=0; ii<ns; ii++)
					{
					a = - h * A_rk[ss+ns*ii];
					for(jj=0; jj<nx*nx; jj++)
						Jt1[jj] = a * Jt0[jj];
					blasfeo_pack_dmat(nx, nx, Jt1, nx, JG, ss*nx, ii*nx);
					}
				}
			blasfeo_ddiare(ns*nx, 1.0, JG, 0, 0);
			blasfeo_dgetrf_rowpivot(ns*nx, ns*nx, JG, 0, 0, JG, 0, 0, ipiv); // LU factorization with pivoting
			blasfeo_drowpe(ns*nx, ipiv, rG);  // row permutations
			blasfeo_dtrsm_llnu(ns*nx, 1+nf, 1.0, JG, 0, 0, rG, 0, 0, rG, 0, 0);  // L backsolve
			blasfeo_dtrsm_lunn(ns*nx, 1+nf, 1.0, JG, 0, 0, rG, 0, 0, rG, 0, 0);  // U backsolve
			blasfeo_dgead(ns*nx, nf+1, 1.0, rG, 0, 0, K, 0, 0);
			}
		for(ss=0; ss<ns; ss++)
			{
			b = h*B_rk[ss];
			for(ii=0; ii<1+nf; ii++)
				{
				blasfeo_dcolex(nx, K, ss*nx, ii, &sxt1, 0);
				for(jj=0; jj<nx; jj++)
					x[jj+ii*nx] += b*xt1[jj];
				}

			}
		t += h;
		}

	return;

	}




