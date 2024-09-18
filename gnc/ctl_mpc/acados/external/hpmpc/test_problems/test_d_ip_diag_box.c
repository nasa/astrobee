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
#include <stdio.h>
#include <sys/time.h>
#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX) || defined(TARGET_X64_SSE3) || defined(TARGET_X86_ATOM) || defined(TARGET_AMD_SSE3)
#include <xmmintrin.h> // needed to flush to zero sub-normals with _MM_SET_FLUSH_ZERO_MODE (_MM_FLUSH_ZERO_ON); in the main()
#endif

// to throw floating-point exception
/*#define _GNU_SOURCE*/
/*#include <fenv.h>*/

#include "test_param.h"
#include "../problem_size.h"
#include "../include/aux_d.h"
#include "../include/lqcp_solvers.h"
#include "../include/mpc_solvers.h"
#include "../include/block_size.h"
#include "../include/c_interface.h"
#include "../include/blas_d.h"

// data matrices
#include "test_matrices_variable_nx.h"



int main()
	{
	
	printf("\n");
	printf("\n");
	printf("\n");
	printf(" HPMPC -- Library for High-Performance implementation of solvers for MPC.\n");
	printf(" Copyright (C) 2014-2015 by Technical University of Denmark. All rights reserved.\n");
	printf("\n");
	printf(" HPMPC is distributed in the hope that it will be useful,\n");
	printf(" but WITHOUT ANY WARRANTY; without even the implied warranty of\n");
	printf(" MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.\n");
	printf(" See the GNU Lesser General Public License for more details.\n");
	printf("\n");
	printf("\n");
	printf("\n");
	
	int ii, jj, ll;

	double **dummy;
	int ** int_dummy;

	
	const int bs = D_MR; //d_get_mr();
	const int ncl = D_NCL;
	const int nal = bs*ncl; // number of doubles per cache line
	
	int nx, nu, N, nrep;

	// timing variables
	float time_ric_diag, time_ric_full, time_ric_full_tv, time_ip_diag, time_ip_full, time_ip_full_tv;

/************************************************
* test of riccati eye/diag & size-variant
************************************************/
	
#if 1

	// horizon length
	N = 11;

	// base nx and nu
	int nx0 = 2;
	int nu0 = 1;

	// size-varing
	int nxx[N+1];
	for(ii=0; ii<=N; ii++) nxx[ii] = (N+1-ii)*nx0 + nu0;

	int pnxx[N+1];
	for(ii=0; ii<=N; ii++) pnxx[ii] = (nxx[ii]+bs-1)/bs*bs;

	int cnxx[N+1];
	for(ii=0; ii<=N; ii++) cnxx[ii] = (nxx[ii]+ncl-1)/ncl*ncl;

	int nuu[N+1];
	for(ii=0; ii<N; ii++) nuu[ii] = nu0;
	nuu[N] = 0; // !!!!!

	int pnuu[N+1];
	for(ii=0; ii<N; ii++) pnuu[ii] = (nuu[ii]+bs-1)/bs*bs;
	pnuu[N] = 0; // !!!!!

	int cnuu[N+1];
	for(ii=0; ii<N; ii++) cnuu[ii] = (nuu[ii]+ncl-1)/ncl*ncl;
	cnuu[N] = 0; // !!!!!

	//for(ii=0; ii<=N; ii++) printf("\n%d %d %d\n", nxx[ii], pnxx[ii], cnxx[ii]);
	//for(ii=0; ii<N; ii++)  printf("\n%d %d %d\n", nuu[ii], pnuu[ii], cnuu[ii]);



	// factorization
	printf("\nRiccati diag\n\n");

	// data memory space
	double *hdA[N];
	double *hpBt[N];
	double *hpR[N];
	double *hpS[N];
	double *hpQ[N+1];
	double *hpLK[N];
	double *hpP[N+1];
	double *pK;

	for(ii=0; ii<N; ii++)
		{
		d_zeros_align(&hdA[ii], pnxx[ii], 1);
		d_zeros_align(&hpBt[ii], pnuu[ii], cnxx[ii+1]);
		d_zeros_align(&hpR[ii], pnuu[ii], cnuu[ii]);
		d_zeros_align(&hpS[ii], pnxx[ii], cnuu[ii]);
		d_zeros_align(&hpQ[ii], pnxx[ii], cnxx[ii]);
		d_zeros_align(&hpLK[ii], pnuu[ii]+pnxx[ii], cnuu[ii]);
		d_zeros_align(&hpP[ii], pnxx[ii], cnxx[ii]);
		}
	d_zeros_align(&hpQ[N], pnxx[N], cnxx[N]);
	d_zeros_align(&hpP[N], pnxx[N], cnxx[N]);
	d_zeros_align(&pK, pnxx[0], cnuu[0]); // max(nx) x nax(nu)

	// dA
	for(ii=0; ii<N; ii++)
		for(jj=0; jj<nxx[ii+1]; jj++)
			hdA[ii][jj] = 1.0;

	//d_print_mat(1, cnxx[1], hdA[0], 1);

	// B
	double *eye_nu0; d_zeros(&eye_nu0, nu0, nu0);
	for(jj=0; jj<nu0; jj++) eye_nu0[jj*(nu0+1)] = 1.0;
	double *ptrB = BBB;
	for(ii=0; ii<N; ii++)
		{
		d_cvt_mat2pmat(nuu[ii], nuu[ii], eye_nu0, nuu[ii], 0, hpBt[ii], cnxx[ii+1]);
		d_cvt_tran_mat2pmat(nxx[ii+1]-nuu[ii], nuu[ii], ptrB, nxx[ii+1]-nuu[ii], 0, hpBt[ii]+nuu[ii]*bs, cnxx[ii+1]);
		ptrB += nxx[ii+1] - nuu[ii];
		}
	free(eye_nu0);

	//d_print_pmat(pnuu[0], cnxx[1], bs, hpBt[0], cnxx[0]);
	//d_print_pmat(pnuu[1], cnxx[2], bs, hpBt[1], cnxx[1]);
	//d_print_pmat(pnuu[2], cnxx[3], bs, hpBt[2], cnxx[2]);
	//d_print_pmat(pnuu[N-1], cnxx[N-1], bs, hpBt[N-2], cnxx[N-2]);
	//d_print_pmat(pnuu[N-1], cnxx[N], bs, hpBt[N-1], cnxx[N-1]);

	// R
	// penalty on du
	for(ii=0; ii<N; ii++)
		for(jj=0; jj<nuu[ii]; jj++)
			hpR[ii][jj/bs*bs*cnuu[ii]+jj%bs+jj*bs] = 0.0;

	//for(ii=0; ii<N; ii++)
	//	d_print_pmat(pnuu[ii], cnuu[ii], bs, hpR[ii], pnuu[ii]);
	//d_print_pmat(pnuu[0], cnuu[0], bs, hpR[0], pnuu[0]);

	// S (zero)

	// Q
	for(ii=0; ii<=N; ii++)
		{
		// penalty on u
		for(jj=0; jj<nu0; jj++) 
			hpQ[ii][jj/bs*bs*cnxx[ii]+jj%bs+jj*bs] = 1.0;
		// penalty on x
//		for(jj==1; jj<nxx[ii]-nx0; jj++) 
//			hpQ[ii][jj/bs*bs*cnxx[ii]+jj%bs+jj*bs] = 0.0002;
		for(jj=nxx[ii]-nx0; jj<nxx[ii]; jj++) 
			hpQ[ii][jj/bs*bs*cnxx[ii]+jj%bs+jj*bs] = 1.0;
		}

	//for(ii=0; ii<=N; ii++)
	//	d_print_pmat(pnxx[ii], cnxx[ii], bs, hpQ2[ii], cnxx[ii]);
	//d_print_pmat(pnxx[0], cnxx[0], bs, hpQ2[0], cnxx[0]);
	//d_print_pmat(pnxx[1], cnxx[1], bs, hpQ2[1], cnxx[1]);
	//d_print_pmat(pnxx[N-1], cnxx[N-1], bs, hpQ2[N-1], cnxx[N-1]);
	//d_print_pmat(pnxx[N], cnxx[N], bs, hpQ2[N], cnxx[N]);
	//exit(1);

	// work space
	double *diag; d_zeros_align(&diag, pnxx[0]+pnuu[0], 1);


	// factorization
	printf("\nfactorization ...\n");
	d_ric_diag_trf_mpc(N, nxx, nuu, hdA, hpBt, hpR, hpS, hpQ, hpLK, pK, hpP, diag);
	printf("\nfactorization done\n\n");

#if 1
	//d_print_pmat(nxx[0], nxx[0], bs, hpP[0], cnxx[0]);
	//d_print_pmat(nxx[1], nxx[1], bs, hpP[1], cnxx[1]);
	//d_print_pmat(nxx[N-2], nxx[N-2], bs, hpP[N-2], cnxx[N-2]);
	//d_print_pmat(nxx[N-1], nxx[N-1], bs, hpP[N-1], cnxx[N-1]);
	//d_print_pmat(nxx[N], nxx[N], bs, hpP[N], cnxx[N]);

	//for(ii=0; ii<=N; ii++)
	//	d_print_pmat(pnuu[ii]+nxx[ii], nuu[ii], bs, hpLK[ii], cnuu[ii]);
	//d_print_pmat(pnuu[0]+nxx[0], nuu[0], bs, hpLK[0], cnuu[0]);
	//d_print_pmat(pnuu[1]+nxx[1], nuu[1], bs, hpLK[1], cnuu[1]);
	//d_print_pmat(pnuu[2]+nxx[2], nuu[2], bs, hpLK[2], cnuu[2]);
	//d_print_pmat(pnuu[N-3]+nxx[N-3], nuu[N-3], bs, hpLK[N-3], cnuu[N-3]);
	//d_print_pmat(pnuu[N-2]+nxx[N-2], nuu[N-2], bs, hpLK[N-2], cnuu[N-2]);
	//d_print_pmat(pnuu[N-1]+nxx[N-1], nuu[N-1], bs, hpLK[N-1], cnuu[N-1]);
#endif



	// backward-forward solution

	// data memory space
	double *hrq[N+1];
	double *hux[N+1];
	double *hpi[N+1];
	double *hPb[N];
	double *hb[N];

	for(ii=0; ii<N; ii++)
		{
		d_zeros_align(&hrq[ii], pnuu[ii]+pnxx[ii], 1);
		d_zeros_align(&hux[ii], pnuu[ii]+pnxx[ii], 1);
		d_zeros_align(&hpi[ii], pnxx[ii], 1);
		d_zeros_align(&hPb[ii], pnxx[ii+1], 1);
		d_zeros_align(&hb[ii], pnxx[ii+1], 1);
		}
	d_zeros_align(&hrq[N], pnuu[N]+pnxx[N], 1);
	d_zeros_align(&hux[N], pnuu[N]+pnxx[N], 1);
	d_zeros_align(&hpi[N], pnxx[N], 1);

	double *work_diag; d_zeros_align(&work_diag, pnxx[0], 1);

	for(ii=0; ii<=N; ii++)
		for(jj=0; jj<nuu[ii]; jj++)
			hrq[ii][jj] = 0.0;

	for(ii=0; ii<=N; ii++)
		for(jj=0; jj<nxx[ii]; jj++)
			hrq[ii][nuu[ii]+jj] = 0.0;

	for(ii=0; ii<N; ii++)
		for(jj=0; jj<nxx[ii+1]; jj++)
			hb[ii][jj] = 0.0;

	// x0
	for(jj=0; jj<nuu[0]; jj++)
		{
		hux[0][jj] = 0.0;
		}
	for(; jj<nuu[0]+nu0; jj++)
		{
		hux[0][jj] = 7.5097;
		}
	for(; jj<nxx[0]; jj+=2)
		{
		hux[0][jj+0] = 15.01940;
		hux[0][jj+1] =  0.0;
		}
	//d_print_mat(1, nuu[0]+nxx[0], hux2[0], 1);


	printf("\nbackward-forward solution ...\n");
	d_ric_diag_trs_mpc(N, nxx, nuu, hdA, hpBt, hpLK, hpP, hb, hrq, hux, 1, hPb, 1, hpi, work_diag);
	printf("\nbackward-forward solution done\n\n");

#if 1
	printf("\nux\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, nuu[ii]+nxx[ii], hux[ii], 1);
#endif



	// residuals

	// data memory space
	double *hres_rq[N+1];
	double *hres_b[N];

	for(ii=0; ii<N; ii++)
		{
		d_zeros_align(&hres_rq[ii], pnuu[ii]+pnxx[ii], 1);
		d_zeros_align(&hres_b[ii], pnxx[ii+1], 1);
		}
	d_zeros_align(&hres_rq[N], pnuu[N]+pnxx[N], 1);


	printf("\nresuduals ...\n");
	d_res_diag_mpc(N, nxx, nuu, hdA, hpBt, hpR, hpS, hpQ, hb, hrq, hux, hpi, hres_rq, hres_b, work_diag);
	printf("\nresiduals done\n\n");

#if 1
	printf("\nres_q\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, nuu[ii]+nxx[ii], hres_rq[ii], 1);

	printf("\nres_b\n");
	for(ii=0; ii<N; ii++)
		d_print_mat(1, nxx[ii+1], hres_b[ii], 1);
#endif





	// timing
	struct timeval tv20, tv21;

#if 1
	printf("\ntiming ...\n\n");
	gettimeofday(&tv20, NULL); // start

	nrep = 10000;
	for(ii=0; ii<nrep; ii++)
		{
		d_ric_diag_trf_mpc(N, nxx, nuu, hdA, hpBt, hpR, hpS, hpQ, hpLK, pK, hpP, diag);
		d_ric_diag_trs_mpc(N, nxx, nuu, hdA, hpBt, hpLK, hpP, hb, hrq, hux, 1, hPb, 1, hpi, work_diag);
		}

	gettimeofday(&tv21, NULL); // start

	time_ric_diag = (float) (tv21.tv_sec-tv20.tv_sec)/(nrep+0.0)+(tv21.tv_usec-tv20.tv_usec)/(nrep*1e6);
	printf("\ntiming done\n\n");
#endif




#if 1
	printf("\nRiccati full\n\n");
	// size-variant full
	int nzz[N+1];
	for(ii=0; ii<=N; ii++) nzz[ii] = nuu[ii] + nxx[ii] + 1;

	int pnzz[N+1];
	for(ii=0; ii<=N; ii++) pnzz[ii] = (nzz[ii]+bs-1)/bs*bs;

	int cnzz[N+1];
	for(ii=0; ii<=N; ii++) cnzz[ii] = (nzz[ii]+ncl-1)/ncl*ncl;

	int anzz[N+1];
	for(ii=0; ii<=N; ii++) anzz[ii] = (nzz[ii]+nal-1)/nal*nal;

	int cnll[N+1];
	for(ii=0; ii<=N; ii++) cnll[ii] = cnzz[ll]<cnxx[ll]+ncl ? cnxx[ll]+ncl : cnzz[ll];

	int nzero[N+1];
	for(ii=0; ii<=N; ii++) nzero[ii] = 0;

	double *hpBAbt_tv[N];
	double *hpRSQ_tv[N+1];
	double *hpL_tv[N+1];
	double *hl[N+1];

	for(ii=0; ii<N; ii++)
		{
		d_zeros_align(&hpBAbt_tv[ii], pnzz[ii], cnxx[ii+1]);
		d_zeros_align(&hpRSQ_tv[ii], pnzz[ii], cnzz[ii]);
		d_zeros_align(&hpL_tv[ii], pnzz[ii], cnll[ii]);
		d_zeros_align(&hl[ii], anzz[ii], 1);
		}
	d_zeros_align(&hpRSQ_tv[N], pnzz[N], cnzz[N]);
	d_zeros_align(&hpL_tv[N], pnzz[N], cnll[N]);
	d_zeros_align(&hl[N], anzz[ii], 1);
	
	double *work_ric_tv; d_zeros_align(&work_ric_tv, pnzz[0], cnxx[0]);

	for(ii=0; ii<N; ii++)
		{
		d_copy_pmat(nuu[ii], nxx[ii+1], bs, hpBt[ii], cnxx[ii], hpBAbt_tv[ii], cnxx[ii+1]);
		for(jj=0; jj<nxx[ii+1]; jj++) hpBAbt_tv[ii][(nuu[ii]+jj)/bs*bs*cnxx[ii+1]+(nuu[ii]+jj)%bs+jj*bs] = 1.0;
		for(jj=0; jj<nxx[ii+1]; jj++) hpBAbt_tv[ii][(nuu[ii]+nxx[ii])/bs*bs*cnxx[ii+1]+(nuu[ii]+nxx[ii])%bs+jj*bs] = hb[ii][jj];
		//d_print_pmat(nzz[ii], nxx[ii+1], bs, hpBAbt_tv[ii], cnxx[ii+1]);
		}
	
	for(ii=0; ii<=N; ii++)
		{
		// R
		// penalty on du
		for(jj=0; jj<nuu[ii]; jj++)
			hpRSQ_tv[ii][jj/bs*bs*cnzz[ii]+jj%bs+jj*bs] = 0.0;
		// Q
		// penalty on u
		for(; jj<nuu[ii]+nu0; jj++) 
			hpRSQ_tv[ii][jj/bs*bs*cnzz[ii]+jj%bs+jj*bs] = 1.0;
		// penalty on x
		for(jj=nuu[ii]+nxx[ii]-nx0; jj<nuu[ii]+nxx[ii]; jj++) 
			hpRSQ_tv[ii][jj/bs*bs*cnzz[ii]+jj%bs+jj*bs] = 1.0;
		// r q
		for(jj=0; jj<nuu[ii]+nxx[ii]; jj++) hpRSQ_tv[ii][(nuu[ii]+nxx[ii])/bs*bs*cnzz[ii]+(nuu[ii]+nxx[ii])%bs+jj*bs] = hrq[ii][jj];
		//d_print_pmat(nzz[ii], nzz[ii], bs, hpRSQ_tv[ii], cnzz[ii]);
		}


	printf("\nfactorization and backward-forward solution ...\n");
	d_ric_sv_mpc_tv(N, nxx, nuu, hpBAbt_tv, hpRSQ_tv, hux, hpL_tv, work_ric_tv, diag, COMPUTE_MULT, hpi, nzero, int_dummy, dummy, dummy, nzero, dummy, dummy, dummy, 0);
	printf("\nfactorization and backward-forward solution done\n\n");

#if 0
	for(ii=0; ii<=N; ii++)
		d_print_pmat(nzz[ii], nzz[ii], bs, hpL_tv[ii], cnzz[ii]);
#endif

	printf("\nux\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, nuu[ii]+nxx[ii], hux[ii], 1);


	for(ii=0; ii<N; ii++)
		for(jj=0; jj<nxx[ii+1]; jj++)
			hux[ii+1][nuu[ii+1]+jj] = hb[ii][jj];

	printf("\nbackward-forward solution ...\n");
	d_ric_trs_mpc_tv(N, nxx, nuu, hpBAbt_tv, hpL_tv, hrq, hl, hux, work_ric_tv, 1, hPb, COMPUTE_MULT, hpi, nzero, int_dummy, dummy, nzero, dummy, dummy);
	printf("\nbackward-forward solution done\n\n");

	printf("\nux\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, nuu[ii]+nxx[ii], hux[ii], 1);
	
	//exit(1);


	printf("\nresuduals ...\n");
	d_res_diag_mpc(N, nxx, nuu, hdA, hpBt, hpR, hpS, hpQ, hb, hrq, hux, hpi, hres_rq, hres_b, work_diag);
	printf("\nresiduals done\n\n");

#if 1
	printf("\nres_q\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, nuu[ii]+nxx[ii], hres_rq[ii], 1);

	printf("\nres_b\n");
	for(ii=0; ii<N; ii++)
		d_print_mat(1, nxx[ii+1], hres_b[ii], 1);
#endif
	
#if 1
	printf("\ntiming ...\n\n");
	gettimeofday(&tv20, NULL); // start

	nrep = 10000;
	for(ii=0; ii<nrep; ii++)
		{
		d_ric_sv_mpc_tv(N, nxx, nuu, hpBAbt_tv, hpRSQ_tv, hux, hpL_tv, work_ric_tv, diag, COMPUTE_MULT, hpi, nzero, int_dummy, dummy, dummy, nzero, dummy, dummy, dummy, 0);
		}

	gettimeofday(&tv21, NULL); // start

	time_ric_full_tv = (float) (tv21.tv_sec-tv20.tv_sec)/(nrep+0.0)+(tv21.tv_usec-tv20.tv_usec)/(nrep*1e6);
	printf("\ntiming done\n\n");
#endif
	

#endif





#if 1
	// IPM
	printf("\nIPM diag\n\n");

	int kk = -1;
	int kmax = 50;
	double mu0 = 1;
	double mu_tol = 1e-8;
	double alpha_min = 1e-12;
	double sigma_par[] = {0.4, 0.3, 0.01};
	double stat[5*50] = {};

	int nbb[N+1];
	nbb[0] = nu0;//nuu[0]; // XXX !!!!!!!!!!!!!!
	for(ii=1; ii<N; ii++) nbb[ii] = 2*nu0 + nx0; //nuu[ii] + nxx[ii];
	nbb[N] = nu0 + nx0;

	int *(idxb[N+1]);
	for(ii=0; ii<=N; ii++)
		{
		idxb[ii] = (int *) malloc(nbb[ii]*sizeof(int));
		}

	int pnbb[N+1];
	for(ii=0; ii<=N; ii++) pnbb[ii] = (nbb[ii]+bs-1)/bs*bs;

	// data memory space
	double *hd[N+1];
	double *hlam[N+1];
	double *ht[N+1];
	double *hres_d[N+1];
	for(ii=0; ii<=N; ii++)
		{
		d_zeros_align(&hd[ii], 2*pnbb[ii], 1);
		d_zeros_align(&hlam[ii], 2*pnbb[ii], 1);
		d_zeros_align(&ht[ii], 2*pnbb[ii], 1);
		d_zeros_align(&hres_d[ii], 2*pnbb[ii], 1);
		}

	double mu = -1;

	//printf("\nbounds\n");
	ii = 0; // initial stage
	ll = 0;
	for(jj=0; jj<nuu[ii]; jj++)
		{
		hd[ii][ll]                  = -20.5;
		hd[ii][pnbb[ii]+ll]         = -20.5;
		idxb[ii][ll] = jj;
		ll++;
		}
	//d_print_mat(1, 2*pnbb[ii], hd[ii], 1);
	for(ii=1; ii<=N; ii++)
		{
		ll = 0;
		for(jj=0; jj<nuu[ii]; jj++)
			{
			hd[ii][ll]          = -20.5;
			hd[ii][pnbb[ii]+ll] = -20.5;
			idxb[ii][ll] = jj;
			ll++;
			}
		for(; jj<nuu[ii]+nu0; jj++)
			{
			hd[ii][ll]          = - 2.5; // -2.5
			hd[ii][pnbb[ii]+ll] = -10.0; // -10
			idxb[ii][ll] = jj;
			ll++;
			}
		//for(; jj<nbb[ii]-nx0; jj++)
		//for(; jj<nbb[ii]; jj++)
			//{
			//hd[ii][jj]          = -100.0;
			//hd[ii][pnbb[ii]+jj] = -100.0;
			//idxb[ii][ll] = jj;
			//ll++;
			//}
		jj += nx0*(N-ii);
		hd[ii][ll+0]          = - 0.0; //   0
		hd[ii][pnbb[ii]+ll+0] = -20.0; // -20
		idxb[ii][ll] = jj;
		ll++;
		jj++;
		hd[ii][ll+0]          = -10.0; // -10
		hd[ii][pnbb[ii]+ll+0] = -10.0; // -10
		idxb[ii][ll] = jj;
		ll++;
		jj++;
		//d_print_mat(1, 2*pnbb[ii], hd[ii], 1);
		}
#if 0
	for(ii=0; ii<=N; ii++)
		{
		for(jj=0; jj<nbb[ii]; jj++)
			printf("%d\t", idxb[ii][jj]);
		printf("\n");
		}
	exit(1);
#endif

	for(jj=0; jj<nuu[0]; jj++)
		{
		hux[0][jj] = 0.0;
		}
	for(; jj<nuu[0]+nu0; jj++)
		{
		hux[0][jj] = 7.5097;
		}
	for(; jj<nxx[0]; jj+=2)
		{
		hux[0][jj+0] = 15.01940;
		hux[0][jj+1] =  0.0;
		}
	//d_print_mat(1, nuu[0]+nxx[0], hux2[0], 1);


	int pnxM = pnxx[0];
	int pnuM = pnuu[0];
	int cnuM = cnuu[0];

	int anxx[N+1];
	for(ii=0; ii<=N; ii++) anxx[ii] = (nxx[ii]+nal-1)/nal*nal;

	int anuu[N+1];
	for(ii=0; ii<=N; ii++) anuu[ii] = (nuu[ii]+nal-1)/nal*nal;

	int work_space_ip_double = 0;
	for(ii=0; ii<=N; ii++)
		work_space_ip_double += anuu[ii] + 3*anxx[ii] + (pnuu[ii]+pnxx[ii])*cnuu[ii] + pnxx[ii]*cnxx[ii] + 12*pnbb[ii];
	work_space_ip_double += pnxM*cnuM + pnxM + pnuM;
	int work_space_ip_int = (N+1)*7*sizeof(int);
	work_space_ip_int = (work_space_ip_int+63)/64*64;
	work_space_ip_int /= sizeof(int);
	printf("\nIPM diag work space size: %d double + %d int\n\n", work_space_ip_double, work_space_ip_int);
	double *work_space_ip; d_zeros_align(&work_space_ip, work_space_ip_double+(work_space_ip_int+1)/2, 1); // XXX assume sizeof(double) = 2 * sizeof(int) !!!!!


	printf("\nIPM solution ...\n");
	d_ip2_diag_mpc(&kk, kmax, mu0, mu_tol, alpha_min, 0, sigma_par, stat, N, nxx, nuu, nbb, idxb, hdA, hpBt, hpR, hpS, hpQ, hb, hd, hrq, hux, 1, hpi, hlam, ht, work_space_ip);
	printf("\nIPM solution done\n");


	printf("\nux\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, nuu[ii]+nxx[ii], hux[ii], 1);

	printf("\nlam\n");
	for(ii=0; ii<=N; ii++)
		{
		d_print_mat(1, nbb[ii], hlam[ii], 1);
		d_print_mat(1, nbb[ii], hlam[ii]+pnbb[ii], 1);
		}

	printf("\nt\n");
	for(ii=0; ii<=N; ii++)
		{
		d_print_mat(1, nbb[ii], ht[ii], 1);
		d_print_mat(1, nbb[ii], ht[ii]+pnbb[ii], 1);
		}

	printf("\nstatistics\n\n");
	for(ii=0; ii<kk; ii++)
		printf("%d\t%f\t%f\t%f\t%e\t%f\t%f\t%e\n", ii+1, stat[5*ii+0], stat[5*ii+1], stat[5*ii+2], stat[5*ii+2], stat[5*ii+3], stat[5*ii+4], stat[5*ii+4]);
	printf("\n\n");


	// residuals
	printf("\nresuduals IPM ...\n");
	d_res_ip_diag_mpc(N, nxx, nuu, nbb, idxb, hdA, hpBt, hpR, hpS, hpQ, hb, hrq, hd, hux, hpi, hlam, ht, hres_rq, hres_b, hres_d, &mu, work_diag);
	printf("\nresiduals IPM done\n");

	printf("\nres_rq\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, nuu[ii]+nxx[ii], hres_rq[ii], 1);

	printf("\nres_b\n");
	for(ii=0; ii<N; ii++)
		d_print_mat(1, nxx[ii+1], hres_b[ii], 1);

	printf("\nres_d\n");
	for(ii=0; ii<=N; ii++)
		{
		d_print_mat(1, nbb[ii], hres_d[ii], 1);
		d_print_mat(1, nbb[ii], hres_d[ii]+pnbb[ii], 1);
		}

	printf("\nres_mu\n");
	d_print_mat(1, 1, &mu, 1);


	// timing
	printf("\ntiming ...\n\n");
	gettimeofday(&tv20, NULL); // start

	nrep = 1000;
	for(ii=0; ii<nrep; ii++)
		{
		d_ip2_diag_mpc(&kk, kmax, mu0, mu_tol, alpha_min, 0, sigma_par, stat, N, nxx, nuu, nbb, idxb, hdA, hpBt, hpR, hpS, hpQ, hb, hd, hrq, hux, 1, hpi, hlam, ht, work_space_ip);
		}

	gettimeofday(&tv21, NULL); // start
	printf("\ntiming done\n\n");

	time_ip_diag = (float) (tv21.tv_sec-tv20.tv_sec)/(nrep+0.0)+(tv21.tv_usec-tv20.tv_usec)/(nrep*1e6);


	// simulation
	printf("\nsimulation ...\n\n");
	nrep = 15;
	for(ii=0; ii<nrep; ii++)
		{

		d_ip2_diag_mpc(&kk, kmax, mu0, mu_tol, alpha_min, 0, sigma_par, stat, N, nxx, nuu, nbb, idxb, hdA, hpBt, hpR, hpS, hpQ, hb, hd, hrq, hux, 1, hpi, hlam, ht, work_space_ip);

		dgemv_t_lib(nuu[0], nxx[0], hpBt[0], cnxx[0], hux[0], hux[0]+nuu[0], 1);
		for(jj=0; jj<nxx[0]-nx0-nu0; jj++) hux[0][nuu[0]+nxx[0]-jj-1] = hux[0][nuu[0]+nxx[0]-jj-1-nx0];

		printf("\nsimulation step = %d, IPM iterations = %d, mu = %e\n\n", ii, kk, stat[5*(kk-1)+4]);
		d_print_mat(1, nuu[0]+nxx[0], hux[0], 1);

		}
	printf("\nsimulation done\n\n");
	//exit(1);





#if 1
	// IPM
	printf("\nIPM full\n\n");

	int ngg[N+1];
	for(ii=0; ii<=N; ii++) ngg[ii] = 0;

	int pngg[N+1];
	for(ii=0; ii<=N; ii++) pngg[ii] = (ngg[ii]+bs-1)/bs*bs;

	//int pnzM = pnzz[0]; // max
	//int cnxgM = cnxx[0]; // max

	//int work_space_int_size = 7*(N+1);
	//int work_space_double_size = pnzM*cnxgM + pnzM;
	//for(ii=0; ii<=N; ii++)
	//	work_space_double_size += pnzz[ii]*cnll[ii] + 3*anzz[ii] + 2*anxx[ii] + 14*pnbb[ii] + 10*pngg[ii];
	
	//printf("\nIPM diag work space size: %d double + %d int\n\n", work_space_double_size, work_space_int_size);
	//double *work_ipm_tv_double; d_zeros_align(&work_ipm_tv_double, work_space_double_size, 1);
	double *work_ipm_tv_double; d_zeros_align(&work_ipm_tv_double, d_ip2_hard_mpc_tv_work_space_size_double(N, nxx, nuu, nbb, ngg), 1);
	//int *work_ipm_tv_int = (int *) malloc(work_space_int_size*sizeof(int));
	int *work_ipm_tv_int = (int *) malloc(d_ip2_hard_mpc_tv_work_space_size_int(N, nxx, nuu, nbb, ngg)*sizeof(int));


	for(jj=0; jj<nuu[0]; jj++)
		{
		hux[0][jj] = 0.0;
		}
	for(; jj<nuu[0]+nu0; jj++)
		{
		hux[0][jj] = 7.5097;
		}
	for(; jj<nxx[0]; jj+=2)
		{
		hux[0][jj+0] = 15.01940;
		hux[0][jj+1] =  0.0;
		}
	//d_print_mat(1, nuu[0]+nxx[0], hux2[0], 1);



	printf("\nIPM solution ...\n");
	d_ip2_hard_mpc_tv(&kk, kmax, mu0, mu_tol, alpha_min, 0, sigma_par, stat, N, nxx, nuu, nbb, idxb, ngg, hpBAbt_tv, hpRSQ_tv, dummy, hd, hux, 1, hpi, hlam, ht, work_ipm_tv_double, work_ipm_tv_int);
	printf("\nIPM solution done\n");



	printf("\nux\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, nuu[ii]+nxx[ii], hux[ii], 1);

	printf("\nlam\n");
	for(ii=0; ii<=N; ii++)
		{
		d_print_mat(1, nbb[ii], hlam[ii], 1);
		d_print_mat(1, nbb[ii], hlam[ii]+pnbb[ii], 1);
		}

	printf("\nt\n");
	for(ii=0; ii<=N; ii++)
		{
		d_print_mat(1, nbb[ii], ht[ii], 1);
		d_print_mat(1, nbb[ii], ht[ii]+pnbb[ii], 1);
		}

	printf("\nstatistics\n\n");
	for(ii=0; ii<kk; ii++)
		printf("%d\t%f\t%f\t%f\t%e\t%f\t%f\t%e\n", ii+1, stat[5*ii+0], stat[5*ii+1], stat[5*ii+2], stat[5*ii+2], stat[5*ii+3], stat[5*ii+4], stat[5*ii+4]);
	printf("\n\n");


	printf("\nresiduals ...\n\n");
	d_res_ip_hard_mpc_tv(N, nxx, nuu, nbb, idxb, ngg, hpBAbt_tv, hpRSQ_tv, hrq, hux, dummy, hd, hpi, hlam, ht, hres_rq, hres_b, hres_d, &mu);
	printf("\nresiduals dones\n\n");

	printf("\nres_rq\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, nuu[ii]+nxx[ii], hres_rq[ii], 1);

	printf("\nres_b\n");
	for(ii=0; ii<N; ii++)
		d_print_mat(1, nxx[ii+1], hres_b[ii], 1);

	printf("\nres_d\n");
	for(ii=0; ii<=N; ii++)
		{
		d_print_mat(1, nbb[ii], hres_d[ii], 1);
		d_print_mat(1, nbb[ii], hres_d[ii]+pnbb[ii], 1);
		}

	printf("\nres_mu\n");
	d_print_mat(1, 1, &mu, 1);



	// timing
	printf("\ntiming ...\n\n");
	gettimeofday(&tv20, NULL); // start

	nrep = 1000;
	for(ii=0; ii<nrep; ii++)
		{
		d_ip2_hard_mpc_tv(&kk, kmax, mu0, mu_tol, alpha_min, 0, sigma_par, stat, N, nxx, nuu, nbb, idxb, ngg, hpBAbt_tv, hpRSQ_tv, dummy, hd, hux, 1, hpi, hlam, ht, work_ipm_tv_double, work_ipm_tv_int);
		}

	gettimeofday(&tv21, NULL); // start
	printf("\ntiming done\n\n");

	time_ip_full_tv = (float) (tv21.tv_sec-tv20.tv_sec)/(nrep+0.0)+(tv21.tv_usec-tv20.tv_usec)/(nrep*1e6);



	free(work_ric_tv);
	free(work_ipm_tv_double);
	free(work_ipm_tv_int);
	for(ii=0; ii<N; ii++)
		{
		free(hpBAbt_tv[ii]);
		free(hpRSQ_tv[ii]);
		free(hpL_tv[ii]);
		free(hl[ii]);
		}
	free(hpRSQ_tv[N]);
	free(hpL_tv[N]);
	free(hl[N]);
	
	//exit(1);

#endif



	// free memory
	for(ii=0; ii<=N; ii++)
		{
		free(idxb[ii]);
		free(hd[ii]);
		free(hlam[ii]);
		free(ht[ii]);
		}
	free(work_space_ip);
#endif




	for(ii=0; ii<N; ii++)
		{
		free(hdA[ii]);
		free(hpBt[ii]);
		free(hpR[ii]);
		free(hpS[ii]);
		free(hpQ[ii]);
		free(hpLK[ii]);
		free(hpP[ii]);
		free(hrq[ii]);
		free(hux[ii]);
		free(hpi[ii]);
		free(hPb[ii]);
		free(hb[ii]);
		free(hres_rq[ii]);
		free(hres_b[ii]);
		}
	free(hpQ[N]);
	free(hpP[N]);
	free(pK);
	free(hrq[N]);
	free(hux[N]);
	free(hpi[N]);
	free(work_diag);
	free(hres_rq[N]);



/************************************************
* test of normal riccati & IPM
************************************************/
	
	printf("\nRiccati full\n\n");

	nx = 25;
	nu = 1;
	N = 11;

	int rep;

	int nz = nx+nu+1;
	int anz = nal*((nz+nal-1)/nal);
	int anx = nal*((nx+nal-1)/nal);
	int pnz = bs*((nz+bs-1)/bs);
	int pnx = bs*((nx+bs-1)/bs);
	int pnu = bs*((nu+bs-1)/bs);
	int cnz = ncl*((nx+nu+1+ncl-1)/ncl);
	int cnx = ncl*((nx+ncl-1)/ncl);
	int cnu = ncl*((nu+ncl-1)/ncl);

	int cnl = cnz<cnx+ncl ? cnx+ncl : cnz;

	const int ncx = nx;


#if 1

	double *BAb_temp; d_zeros(&BAb_temp, nx, nu+nx+1);
	double *hpBAbt2[N];

	ptrB = BBB;
	for(ii=0; ii<N; ii++)
		{
		//printf("\n%d\n", ii);
		d_zeros_align(&hpBAbt2[ii], pnz, cnx);
		for(jj=0; jj<nx*(nx+nu+1); jj++) BAb_temp[jj] = 0.0;
		for(jj=0; jj<nu; jj++) BAb_temp[jj*(nx+1)] = 1.0;
		d_copy_mat(nxx[ii+1]-1, nuu[ii], ptrB, nxx[ii+1]-1, BAb_temp+1, nx);
		ptrB += nxx[ii+1]-1;
		for(jj=0; jj<nxx[ii+1]; jj++) BAb_temp[nuu[ii]*nx+jj*(nx+1)] = 1.0;
		//for(jj=0; jj<nxx[ii+1]; jj++) BAb_temp[(nuu[ii]+nxx[ii+1])*nx+jj] = 1.0;
		//d_print_mat(nx, nu+nx+1, BAb_temp, nx);
		d_cvt_tran_mat2pmat(nx, nx+nu+1, BAb_temp, nx, 0, hpBAbt2[ii], cnx);
		//d_print_pmat(nx+nu+1, nx, bs, hpBAbt2[ii], cnx);
		}

	double *RSQ; d_zeros(&RSQ, nz, nz);
	double *hpRSQ[N+1];

	for(ii=0; ii<=N; ii++)
		{
		//printf("\n%d\n", ii);
		d_zeros_align(&hpRSQ[ii], pnz, cnz);
		for(jj=0; jj<nz*nz; jj++) RSQ[jj] = 0.0;
		for(jj=nu; jj<2*nu; jj++) RSQ[jj*(nz+1)] = 1.0;
		for(jj=nu+nxx[ii]-nx0; jj<nu+nxx[ii]; jj++) RSQ[jj*(nz+1)] = 1.0;
		d_cvt_mat2pmat(nz, nz, RSQ, nz, 0, hpRSQ[ii], cnz);
		//d_print_pmat(nz, nz, bs, hpRSQ[ii], cnz);
		}

	double *hpL[N+1];
	double *hq2[N+1];
	double *hux2[N+1];
	double *hpi2[N+1];
	double *hPb2[N];
	for(jj=0; jj<N; jj++)
		{
		d_zeros_align(&hq2[jj], pnz, 1); // it has to be pnz !!!
		d_zeros_align(&hpL[jj], pnz, cnl);
		d_zeros_align(&hux2[jj], pnz, 1); // it has to be pnz !!!
		d_zeros_align(&hpi2[jj], pnx, 1);
		d_zeros_align(&hPb2[jj], pnx, 1);
		}
	d_zeros_align(&hpL[N], pnz, cnl);
	d_zeros_align(&hq2[N], pnz, 1); // it has to be pnz !!!
	d_zeros_align(&hux2[N], pnz, 1); // it has to be pnz !!!
	d_zeros_align(&hpi2[N], pnx, 1);

	
	//double *work; d_zeros_align(&work, 2*anz, 1);
	double *work; d_zeros_align(&work, pnz, cnx);


	for(jj=0; jj<nx+nu; jj++) hux2[0][jj] = 0.0;
	for(jj=0; jj<nu; jj++)
		{
		hux2[0][nu+jj] = 7.5097;
		}
	for(; jj<nx; jj+=2)
		{
		hux2[0][nu+jj+0] = 15.01940;
		hux2[0][nu+jj+1] =  0.0;
		}

	printf("\nfactorization and backward-forward solution ...\n");
	d_ric_sv_mpc(nx, nu, N, hpBAbt2, hpRSQ, 0, dummy, dummy, hux2, hpL, work, diag, COMPUTE_MULT, hpi2, 0, 0, 0, dummy, dummy, dummy, 0);
	printf("\nfactorization and backward-forward solution done\n\n");

	//for(ii=0; ii<=N; ii++)
	//	d_print_pmat(pnz, cnl-3, bs, hpL[ii], cnl);
	//d_print_pmat(pnz, nu, bs, hpL[0], cnl);
	//d_print_pmat(pnz, cnl-3, bs, hpL[1], cnl);
	//d_print_pmat(pnz, cnl-3, bs, hpL[2], cnl);
	//d_print_pmat(pnz, cnl-3, bs, hpL[N-3], cnl);
	//d_print_pmat(pnz, cnl-3, bs, hpL[N-2], cnl);
	//d_print_pmat(pnz, cnl-3, bs, hpL[N-1], cnl);
	//d_print_pmat(pnz, cnl, bs, hpL[N], cnl);

#if 1
	printf("\nux Riccati full\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, nx+nu, hux2[ii], 1);
#endif

	
	// residuals

	double *hres_rq2[N+1];
	double *hres_b2[N];

	for(ii=0; ii<N; ii++)
		{
		d_zeros_align(&hres_rq2[ii], pnz, 1);
		d_zeros_align(&hres_b2[ii], pnx, 1);
		}
	d_zeros_align(&hres_rq2[N], pnz, 1);
	

	printf("\nresuduals ...\n");
	d_res_mpc(nx, nu, N, hpBAbt2, hpRSQ, hq2, hux2, hpi2, hres_rq2, hres_b2);
	printf("\nresiduals done\n\n");

	printf("\nres_q full\n");
	d_print_mat(1, nu, hres_rq2[ii], 1);
	for(ii=0; ii<N; ii++)
		d_print_mat(1, nx+nu, hres_rq2[ii], 1);

	printf("\nres_b full\n");
	for(ii=0; ii<N; ii++)
		d_print_mat(1, nx, hres_b2[ii], 1);



	// timing
	//struct timeval tv20, tv21;

#if 1
	printf("\ntiming ...\n\n");
	gettimeofday(&tv20, NULL); // start

	nrep = 10000;
	for(ii=0; ii<nrep; ii++)
		{
		d_ric_sv_mpc(nx, nu, N, hpBAbt2, hpRSQ, 0, dummy, dummy, hux2, hpL, work, diag, COMPUTE_MULT, hpi2, 0, 0, 0, dummy, dummy, dummy, 0);
		}

	gettimeofday(&tv21, NULL); // start

	time_ric_full = (float) (tv21.tv_sec-tv20.tv_sec)/(nrep+0.0)+(tv21.tv_usec-tv20.tv_usec)/(nrep*1e6);
	printf("\ntiming done\n\n");
#endif



	printf("\nIPM full\n\n");

	int nb  = nu+nx;
	int ng  = 0;
	int ngN = 0;

	int pnb  = (nb+bs-1)/bs*bs;
	int png  = (ng+bs-1)/bs*bs;
	int pngN = (ngN+bs-1)/bs*bs;



	double *hd2[N+1];
	double *hlam2[N+1];
	double *ht2[N+1];

	for(ii=0; ii<N; ii++)
		{
		d_zeros_align(&hd2[ii], 2*pnb+2*png, 1);
		d_zeros_align(&hlam2[ii],2*pnb+2*png, 1);
		d_zeros_align(&ht2[ii], 2*pnb+2*png, 1);
		}
	d_zeros_align(&hd2[N], 2*pnb+2*pngN, 1);
	d_zeros_align(&hlam2[N],2*pnb+2*pngN, 1);
	d_zeros_align(&ht2[N], 2*pnb+2*pngN, 1);

	// work space // more than enought !!!!!
	double *work_ipm_full; d_zeros_align(&work_ipm_full, hpmpc_ip_hard_mpc_dp_work_space(N, nx, nu, nb, ng, ngN), 1);

	// bounds
	for(ii=0; ii<=N; ii++)
		{
		for(jj=0; jj<nu; jj++)
			{
			hd2[ii][jj]     = -20.5;
			hd2[ii][pnb+jj] = -20.5;
			}
		for(; jj<2*nu; jj++)
			{
			hd2[ii][jj]     = - 2.5;
			hd2[ii][pnb+jj] = -10.0;
			}
		for(; jj<2*nu+(N-ii)*nx0; jj++)
			{
			hd2[ii][jj]     = -100.0;
			hd2[ii][pnb+jj] = -100.0;
			}
		hd2[ii][jj+0]     =   0.0;
		hd2[ii][pnb+jj+0] = -20.0;
		hd2[ii][jj+1]     = -10.0;
		hd2[ii][pnb+jj+1] = -10.0;
		jj += 2;
		for(; jj<nu+nx; jj++)
			{
			hd2[ii][jj]     = -100.0;
			hd2[ii][pnb+jj] = -100.0;
			}
		//d_print_mat(1, nb, hd2[ii], 1);
		//d_print_mat(1, nb, hd2[ii]+pnb, 1);
		}
	//exit(1);



	printf("\nIPM full solve ...\n\n");
	d_ip2_hard_mpc(&kk, kmax, mu0, mu_tol, alpha_min, 0, sigma_par, stat, nx, nu, N, nb, ng, ngN, hpBAbt2, hpRSQ, dummy, hd2, hux2, 1, hpi2, hlam2, ht2, work_ipm_full);
	printf("\nIPM full solve done\n\n");



#if 1
	printf("\nux IPM full\n");
	for(ii=0; ii<=N; ii++)
		d_print_mat(1, nx+nu, hux2[ii], 1);
#endif
	
	printf("\nstatistics\n\n");
	for(ii=0; ii<kk; ii++)
		printf("%d\t%f\t%f\t%f\t%e\t%f\t%f\t%e\n", ii+1, stat[5*ii+0], stat[5*ii+1], stat[5*ii+2], stat[5*ii+2], stat[5*ii+3], stat[5*ii+4], stat[5*ii+4]);
	printf("\n\n");



	// timing
	printf("\ntiming ...\n\n");
	gettimeofday(&tv20, NULL); // start

	nrep = 1000;
	for(ii=0; ii<nrep; ii++)
		{
		d_ip2_hard_mpc(&kk, kmax, mu0, mu_tol, alpha_min, 0, sigma_par, stat, nx, nu, N, nb, ng, ngN, hpBAbt2, hpRSQ, dummy, hd2, hux2, 1, hpi2, hlam2, ht2, work_ipm_full);
		}

	gettimeofday(&tv21, NULL); // start
	printf("\ntiming done\n\n");

	time_ip_full = (float) (tv21.tv_sec-tv20.tv_sec)/(nrep+0.0)+(tv21.tv_usec-tv20.tv_usec)/(nrep*1e6);



	// free memory
	free(work_ipm_full);
	for(ii=0; ii<N; ii++)
		{
		free(hd2[ii]);
		free(hlam2[ii]);
		free(ht2[ii]);
		}
	free(hd2[N]);
	free(hlam2[N]);
	free(ht2[N]);


	// free memory 
	free(work);
	free(RSQ);
	free(BAb_temp);
	for(ii=0; ii<N; ii++)
		{
		free(hpBAbt2[ii]);
		free(hpRSQ[ii]);
		free(hpL[ii]);
		free(hux2[ii]);
		free(hpi2[ii]);
		free(hq2[ii]);
		free(hPb2[ii]);
		free(hres_rq2[ii]);
		free(hres_b2[ii]);
		}
	free(hpRSQ[N]);
	free(hpL[N]);
	free(hux2[N]);
	free(hpi2[N]);
	free(hq2[N]);
	free(hres_rq2[N]);

#endif

	printf("\nric diag time = %e\t\tric full time = %e\t\tric full tv time = %e\t\tip diag time = %e\t\tip full time = %e\t\tip full tv time = %e\n\n", time_ric_diag, time_ric_full, time_ric_full_tv, time_ip_diag, time_ip_full, time_ip_full_tv);


#endif

	}
