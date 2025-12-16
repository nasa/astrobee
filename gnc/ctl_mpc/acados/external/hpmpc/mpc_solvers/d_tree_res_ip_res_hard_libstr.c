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


#ifdef BLASFEO

#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_d_blas.h>
#include <blasfeo_d_aux.h>

#include "../include/tree.h"


int d_tree_res_res_mpc_hard_work_space_size_bytes_libstr(int Nn, struct node *tree, int *nx, int *nu, int *nb, int *ng)
	{

	int ii, tmp0, tmp1;

	int size = 0;

	int ngM = 0;
	int nbM = 0;
	for(ii=0; ii<Nn; ii++)
		{
		ngM = ng[ii]>ngM ? ng[ii] : ngM;
		nbM = nb[ii]>nbM ? nb[ii] : nbM;
		}

	tmp0 = 2*blasfeo_memsize_dvec(ngM); // res_work[0], res_work[1]
	tmp1 = blasfeo_memsize_dvec(nbM);
	
	size += tmp0>tmp1 ? tmp0 : tmp1;

	// make multiple of (typical) cache line size
	size = (size+63)/64*64;

	return size;

	}



void d_tree_res_res_mpc_hard_libstr(int Nn, struct node *tree, int *nx, int *nu, int *nb, int **idxb, int *ng, struct blasfeo_dmat *hsBAbt, struct blasfeo_dvec *hsb, struct blasfeo_dmat *hsRSQrq, struct blasfeo_dvec *hsrq, struct blasfeo_dvec *hsux, struct blasfeo_dmat *hsDCt, struct blasfeo_dvec *hsd, struct blasfeo_dvec *hspi, struct blasfeo_dvec *hslam, struct blasfeo_dvec *hst, struct blasfeo_dvec *hsres_rq, struct blasfeo_dvec *hsres_b, struct blasfeo_dvec *hsres_d, struct blasfeo_dvec *hsres_m, double *mu, void *work)
	{

	int ii, jj, ll;

	char *c_ptr;

	struct blasfeo_dvec hswork_0, hswork_1;
	double *work0, *work1;

	int nu0, nu1, nx0, nx1, nxm, nb0, ng0, nb_tot;

	int nkids, idxkid;

	double
		mu2;

	// initialize mu
	nb_tot = 0;
	mu2 = 0;

	// loop over nodes
	for(ii=0; ii<Nn; ii++)
		{

		// work on node

		nu0 = nu[ii];
		nx0 = nx[ii];
		nb0 = nb[ii];
		ng0 = ng[ii];

		blasfeo_dveccp(nu0+nx0, &hsrq[ii], 0, &hsres_rq[ii], 0);

		// no previous multiplier at the first stage
		if(ii>0)
			blasfeo_daxpy(nx0, -1.0, &hspi[ii], 0, &hsres_rq[ii], nu0, &hsres_rq[ii], nu0);

		blasfeo_dsymv_l(nu0+nx0, nu0+nx0, 1.0, &hsRSQrq[ii], 0, 0, &hsux[ii], 0, 1.0, &hsres_rq[ii], 0, &hsres_rq[ii], 0);

		if(nb0>0)
			{

			nb_tot += nb0;

			blasfeo_create_dvec(nb0, &hswork_0, work);
			blasfeo_daxpy(nb0, -1.0, &hslam[ii], 0, &hslam[ii], nb0, &hswork_0, 0);
			blasfeo_dvecad_sp(nb0, 1.0, &hswork_0, 0, idxb[ii], &hsres_rq[ii], 0);

			blasfeo_dvecex_sp(nb0, -1.0, idxb[ii], &hsux[ii], 0, &hsres_d[ii], 0);
			blasfeo_dveccp(nb0, &hsres_d[ii], 0, &hsres_d[ii], nb0);
			blasfeo_daxpy(2*nb0, 1.0, &hsd[ii], 0, &hsres_d[ii], 0, &hsres_d[ii], 0);
			blasfeo_daxpy(nb0, 1.0, &hst[ii], 0, &hsres_d[ii], 0, &hsres_d[ii], 0);
			blasfeo_daxpy(nb0,- 1.0, &hst[ii], nb0, &hsres_d[ii], nb0, &hsres_d[ii], nb0);

			}

		if(ng0>0)
			{

			c_ptr = (char *) work;
			blasfeo_create_dvec(ng0, &hswork_0, (void *) c_ptr);
			c_ptr += hswork_0.memsize;
			blasfeo_create_dvec(ng0, &hswork_1, (void *) c_ptr);
			c_ptr += hswork_1.memsize;
			work0 = hswork_0.pa;
			work1 = hswork_1.pa;

			nb_tot += ng0;

			blasfeo_daxpy(ng0, -1.0, &hslam[ii], 2*nb0, &hslam[ii], 2*nb0+ng0, &hswork_0, 0);

			blasfeo_daxpy(ng0, 1.0, &hst[ii], 2*nb0, &hsd[ii], 2*nb0, &hsres_d[ii], 2*nb0);
			blasfeo_daxpy(ng0, -1.0, &hst[ii], 2*nb0+ng0, &hsd[ii], 2*nb0+ng0, &hsres_d[ii], 2*nb0+ng0);

			blasfeo_dgemv_nt(nu0+nx0, ng0, 1.0, 1.0, &hsDCt[ii], 0, 0, &hswork_0, 0, &hsux[ii], 0, 1.0, 0.0, &hsres_rq[ii], 0, &hswork_1, 0, &hsres_rq[ii], 0, &hswork_1, 0);

			blasfeo_daxpy(ng0, -1.0, &hswork_1, 0, &hsres_d[ii], 2*nb0, &hsres_d[ii], 2*nb0);
			blasfeo_daxpy(ng0, -1.0, &hswork_1, 0, &hsres_d[ii], 2*nb0+ng0, &hsres_d[ii], 2*nb0+ng0);

			}

		mu2 += blasfeo_dvecmuldot(2*nb0+2*ng0, &hslam[ii], 0, &hst[ii], 0, &hsres_m[ii], 0);

		// work on kids

		nkids = tree[ii].nkids;

		for(jj=0; jj<nkids; jj++)
			{

			idxkid = tree[ii].kids[jj];

			nu1 = nu[idxkid];
			nx1 = nx[idxkid];

			blasfeo_daxpy(nx1, -1.0, &hsux[idxkid], nu1, &hsb[idxkid-1], 0, &hsres_b[idxkid-1], 0);

			blasfeo_dgemv_nt(nu0+nx0, nx1, 1.0, 1.0, &hsBAbt[idxkid-1], 0, 0, &hspi[idxkid], 0, &hsux[ii], 0, 1.0, 1.0, &hsres_rq[ii], 0, &hsres_b[idxkid-1], 0, &hsres_rq[ii], 0, &hsres_b[idxkid-1], 0);

			}

		}

	// normalize mu
	if(nb_tot!=0)
		{
		mu2 /= 2.0*nb_tot;
		mu[0] = mu2;
		}

	return;

	}



#endif

