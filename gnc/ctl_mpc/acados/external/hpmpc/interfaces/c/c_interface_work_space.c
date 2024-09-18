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

#if defined(BLASFEO)
#include <blasfeo_target.h>
#include <blasfeo_common.h>
#endif

#include "../../include/target.h"
#include "../../include/block_size.h"
#include "../../include/lqcp_solvers.h"
#include "../../include/mpc_solvers.h"



// work space: dynamic definition as function return value

// Riccati-based IP method for box-constrained MPC, double precision
// XXX assume size(double) >= size(int) && size(double) >= size(int *) !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
int hpmpc_d_ip_mpc_hard_tv_work_space_size_bytes(int N, int nx, int nu, int nb, int ng, int ngN)
	{

	const int bs  = D_MR; //d_get_mr();
	const int ncl = D_NCL;

	const int nz   = nx+nu+1;
	const int pnz  = bs*((nz+bs-1)/bs);
	const int pnx  = (nx+bs-1)/bs*bs;
	const int pnb  = bs*((nb+bs-1)/bs);
	const int png  = bs*((ng+bs-1)/bs);
	const int pngN = bs*((ngN+bs-1)/bs);
	const int cnz  = ncl*((nx+nu+1+ncl-1)/ncl);
	const int cnx  = ncl*((nx+ncl-1)/ncl);
	const int cnux = (nu+nx+ncl-1)/ncl*ncl;
	const int cng  = ncl*((ng+ncl-1)/ncl);
	const int cngN = ncl*((ngN+ncl-1)/ncl);
	const int cnxg = ncl*((nx+ng+ncl-1)/ncl);
	const int cnl  = cnux<cnx+ncl ? cnx+ncl : cnux;

	int work_space_size = (8 + bs + (N+1)*(nb + pnz*cnx + pnz*cnux + pnz*cnl + pnz*cng + 7*pnz + 6*pnx + 23*pnb + 19*png) + pnz*(cngN-cng) + 19*(pngN-png) + pnz + (cngN<cnxg ? pnz*cnxg : pnz*cngN) );

	return work_space_size*sizeof(double);

	}



int hpmpc_d_ip_ocp_hard_tv_work_space_size_bytes(int N, int *nx, int *nu, int *nb, int **hidxb, int *ng, int N2)
	{

	// XXX sequential update not implemented
	if(N2>N)
		N2 = N;

	const int bs  = D_MR; //d_get_mr();
	const int ncl = D_NCL;

	int ii;

	int pnx[N+1];
	int pnz[N+1];
	int pnb[N+1];
	int png[N+1];
	int cnx[N+1];
	int cnux[N+1];
	int cng[N+1];

	for(ii=0; ii<N; ii++)
		{
		pnx[ii] = (nx[ii]+bs-1)/bs*bs;
		pnz[ii] = (nu[ii]+nx[ii]+1+bs-1)/bs*bs;
		pnb[ii] = (nb[ii]+bs-1)/bs*bs;
		png[ii] = (ng[ii]+bs-1)/bs*bs;
		cnx[ii] = (nx[ii]+ncl-1)/ncl*ncl;
		cnux[ii] = (nu[ii]+nx[ii]+ncl-1)/ncl*ncl;
		cng[ii] = (ng[ii]+ncl-1)/ncl*ncl;
		}
	ii = N;
	pnx[ii] = (nx[ii]+bs-1)/bs*bs;
	pnz[ii] = (nx[ii]+1+bs-1)/bs*bs;
	pnb[ii] = (nb[ii]+bs-1)/bs*bs;
	png[ii] = (ng[ii]+bs-1)/bs*bs;
	cnx[ii] = (nx[ii]+ncl-1)/ncl*ncl;
	cnux[ii] = (nx[ii]+ncl-1)/ncl*ncl;
	cng[ii] = (ng[ii]+ncl-1)/ncl*ncl;

	int d_size = bs; // ???????

	for(ii=0; ii<N; ii++)
		{
		d_size += pnz[ii]*cnx[ii+1] + pnz[ii]*cng[ii] + pnz[ii]*cnux[ii] + 3*pnx[ii] + 3*pnz[ii] + 8*pnb[ii] + 8*png[ii];
		}
	ii = N;
	d_size += pnz[ii]*cng[ii] + pnz[ii]*cnux[ii] + 3*pnx[ii] + 3*pnz[ii] + 8*pnb[ii] + 8*png[ii];

	int size = 2*64; // align twice

	if(N2<N) // partial condensing
		{

		// compute problem size
		int nx2[N2+1];
		int nu2[N2+1];
		int nb2[N2+1];
		int ng2[N2+1];

		d_part_cond_compute_problem_size(N, nx, nu, nb, hidxb, ng, N2, nx2, nu2, nb2, ng2);

		int pnx2[N2+1];
		int pnz2[N2+1];
		int pnb2[N2+1];
		int png2[N2+1];

		for(ii=0; ii<=N2; ii++)
			{
			pnx2[ii] = (nx2[ii]+bs-1)/bs*bs;
			pnz2[ii] = (nu2[ii]+nx2[ii]+1+bs-1)/bs*bs;
			pnb2[ii] = (nb2[ii]+bs-1)/bs*bs;
			png2[ii] = (ng2[ii]+bs-1)/bs*bs;
			}

		// partial condensing
		size += d_part_cond_memory_space_size_bytes(N, nx, nu, nb, hidxb, ng, N2, nx2, nu2, nb2, ng2);
		size += d_part_cond_work_space_size_bytes(N, nx, nu, nb, hidxb, ng, N2, nx2, nu2, nb2, ng2);

		// partial expanding
		size += d_part_expand_work_space_size_bytes(N, nx, nu, nb, ng);

		// IPM
		size += d_ip2_res_mpc_hard_tv_work_space_size_bytes(N2, nx2, nu2, nb2, ng2);

		for(ii=0; ii<=N2; ii++)
			d_size += pnz2[ii] + pnx2[ii] + 4*pnb2[ii] + 4*png2[ii];

		}
	else //
		{

		// IPM
		size += d_ip2_res_mpc_hard_tv_work_space_size_bytes(N, nx, nu, nb, ng);

		}

	size += d_size*sizeof(double);

	size = (size + 63) / 64 * 64; // make multiple of (typical) cache line size

	return size;

	}



// TODO
int hpmpc_d_ip_ocp_soft_tv_work_space_size_bytes(int N, int *nx, int *nu, int *nb, int **hidxb, int *ng, int *ns)
	{

	const int bs  = D_MR; //d_get_mr();
	const int ncl = D_NCL;

	int ii;

	int pnx[N+1];
	int pnz[N+1];
	int pnb[N+1];
	int png[N+1];
	int pns[N+1];
	int cnx[N+1];
	int cnux[N+1];
	int cng[N+1];

	for(ii=0; ii<N; ii++)
		{
		pnx[ii] = (nx[ii]+bs-1)/bs*bs;
		pnz[ii] = (nu[ii]+nx[ii]+1+bs-1)/bs*bs;
		pnb[ii] = (nb[ii]+bs-1)/bs*bs;
		png[ii] = (ng[ii]+bs-1)/bs*bs;
		pns[ii] = (ns[ii]+bs-1)/bs*bs;
		cnx[ii] = (nx[ii]+ncl-1)/ncl*ncl;
		cnux[ii] = (nu[ii]+nx[ii]+ncl-1)/ncl*ncl;
		cng[ii] = (ng[ii]+ncl-1)/ncl*ncl;
		}
	ii = N;
	pnx[ii] = (nx[ii]+bs-1)/bs*bs;
	pnz[ii] = (nx[ii]+1+bs-1)/bs*bs;
	pnb[ii] = (nb[ii]+bs-1)/bs*bs;
	png[ii] = (ng[ii]+bs-1)/bs*bs;
	pns[ii] = (ns[ii]+bs-1)/bs*bs;
	cnx[ii] = (nx[ii]+ncl-1)/ncl*ncl;
	cnux[ii] = (nx[ii]+ncl-1)/ncl*ncl;
	cng[ii] = (ng[ii]+ncl-1)/ncl*ncl;

	int d_size = bs; // ???????

	for(ii=0; ii<N; ii++)
		{
		d_size += pnz[ii]*cnx[ii+1] + pnz[ii]*cng[ii] + pnz[ii]*cnux[ii] + 3*pnx[ii] + 3*pnz[ii] + 8*pnb[ii] + 8*png[ii] + 16*pns[ii]; // TODO
		}
	ii = N;
	d_size += pnz[ii]*cng[ii] + pnz[ii]*cnux[ii] + 3*pnx[ii] + 3*pnz[ii] + 8*pnb[ii] + 8*png[ii] + 16*pns[ii]; // TODO

	int size = 2*64; // align twice

	// IPM
	size += d_ip2_mpc_soft_tv_work_space_size_bytes(N, nx, nu, nb, ng, ns);

	size += d_size*sizeof(double);

	size = (size + 63) / 64 * 64; // make multiple of (typical) cache line size

	return size;

	}


