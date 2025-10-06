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
#include <math.h>

#include "../include/aux_d.h"
#include "../include/blas_d.h"
#include "../include/block_size.h"



int main()
	{
	
	int i, j, rep;
	
	const int bs  = D_MR;
	const int ncl = D_NCL;
	
	printf("\nbs = %d\n\n", bs);
	
	int n = 12;
	int nrep = 1;
	
	double *A; d_zeros(&A, n, n);
	double *B; d_zeros(&B, n, n);
	double *C; d_zeros(&C, n, n);
	double *L; d_zeros(&L, n, n);
	
	for(i=0; i<n*n; i++)
		{
		A[i] = i;
		}
	
	B[0] = 2;
	for(i=1; i<n-1; i++)
		{
		B[i*(n+1)+0] = 2;
		}
	B[n*n-1] = 2;
	
	for(i=0; i<n; i++)
		C[i*(n+1)] = 2;
	for(i=0; i<n-1; i++)
		C[1+i*(n+1)] = 1;

//	d_print_mat(n, n, C, n);

	int pn = ((n+bs-1)/bs)*bs;//+4;	
	int cn = ((n+ncl-1)/ncl)*ncl;//+4;	
	int cn2 = ((2*n+ncl-1)/ncl)*ncl;	

	double *pA; d_zeros_align(&pA, pn, cn);
	double *pB; d_zeros_align(&pB, pn, cn);
	double *pC; d_zeros_align(&pC, pn, cn);
	double *pL; d_zeros_align(&pL, pn, cn);
	double *pD; d_zeros_align(&pD, pn, cn);
	double *pE; d_zeros_align(&pE, pn, cn2);
	double *pF; d_zeros_align(&pF, pn, cn);
	double *diag; d_zeros_align(&diag, pn, 1);
	
	d_cvt_mat2pmat(n, n, A, n, 0, pA, cn);
	d_cvt_mat2pmat(n, n, B, n, 0, pB, cn);
/*	s_cvt_mat2pmat(n, n, 0, bss, sA, n, spA, pns);*/
/*	s_cvt_mat2pmat(n, n, 0, bss, sB, n, spB, pns);*/
	d_cvt_mat2pmat(n, n, B, n, 0, pD, cn);
	d_cvt_mat2pmat(n, n, A, n, 0, pE, cn2);

#if 0
	d_copy_pmat_general(8, 9, 3, pA+3, cn, 0, pC+0, cn);
	d_print_pmat(n, n, bs, pA, cn);
	d_print_pmat(n, n, bs, pC, cn);
	exit(1);
#endif
	
	double *x; d_zeros_align(&x, n, 1);
	double *y; d_zeros_align(&y, n, 1);
	
	pA[0] = -1;
	x[2] = 1;
	for(i=0; i<n; i++) x[i] = i;
	for(i=0; i<n; i++) y[i] = 1;

/*	for(i=0; i<pn*pn; i++) pC[i] = -1;*/
/*	for(i=0; i<pn*pn; i++) spC[i] = -1;*/
	
//	d_print_pmat(pn, pn, bs, pA, pn);
//	d_print_pmat(pn, pn, bs, pB, pn);
//	d_print_pmat(pn, pn, bs, pC, pn);
//	d_print_mat(n, n, B, n);

//	double *x; d_zeros_align(&x, pn, 1);
//	double *y; d_zeros_align(&y, pn, 1);
//	x[3] = 1.0;

/*	d_cvt_mat2pmat(n, n, C, n, 0, pC, pn);*/
/*	d_cvt_mat2pmat(n, n, C, n, bs-n%bs, pC+((bs-n%bs))%bs*(bs+1), pn);*/
	d_print_pmat(pn, cn, bs, pA, cn);
	d_print_pmat(pn, cn, bs, pB, cn);
	d_print_pmat(pn, cn, bs, pC, cn);
	d_print_pmat(pn, cn, bs, pD, cn);
	d_print_mat(1, n, x, 1);
	d_print_mat(1, n, y, 1);

	/* timing */
	struct timeval tv0, tv1;
	gettimeofday(&tv0, NULL); // start
	
/*	d_print_pmat(n, n, bs, pC, pn);*/
	
	nrep = 1;

	for(rep=0; rep<nrep; rep++)
		{

//		dgemm_nt_lib(n, n, n, pB, cn, pA, cn, 0, pC, cn, pD, cn, 0, 0);
//		dgemm_nn_lib(n, n, n, pB, cn, pA, cn, pC, cn, pD, cn, 0, 0, 0);
//		dsyrk_dpotrf_lib(8, 2, n, pA, cn, pB, cn, pD, cn, diag, 1, 0);
//		dsyrk_dpotrf_lib_new(11, n, n, pA, cn, pA, cn, 1, pB, cn, pD, cn, diag);
//		dsyrk_nt_lib(n, n, n, pA, cn, pA, cn, 1, pB, cn, pD, cn);
//		dpotrf_lib(n, n, pD, cn, pD, cn, x);
//		dtrtri_lib(n, pD, cn, 1, x, pL, cn);
//		dlauum_lib(7, pL, cn, pL, cn, 0, pF, cn, pF, cn);
//		dsyrk_nn_lib(n, n, n, pA, cn, pB, cn, pC, cn, pD, cn, 0);
//		dgemm_diag_left_lib(n, n, x, pA, cn, pC, cn, pD, cn, 0);
//		dsyrk_diag_left_right_lib(n, x, x, pA, cn, pC, cn, pD, cn, 0);
//		dtrmm_nt_u_lib(6, 4, pA, cn, pB, cn, pD, cn);
//		dgemm_nt_lib(13, n, n, pA, pn, pB, pn, pC, pn, 0);
/*		dgemm_nt_lib(n, n, n, pB, pn, pA, pn, pC, pn, 0);*/
/*		dtrmm_lib(n, n, pA, pn, pB, pn, pC, pn);*/
//		dtrtr_l_lib(7, 1, pA+1, pn, 0, pC+0, pn);
//		dtrtr_u_lib(9, 3, pA+3, pn, 0, pC+0, pn);
		dtrcp_l_lib(9, 3, pA+3, pn, 0, pC+0, pn);
//		dttmm_ll_lib(n, pA, pn, pA, pn, pC, pn);
//		dttmm_uu_lib(n, pA, pn, pA, pn, pC, pn);
//		dtrmm_u_lib(n, n, pA, pn, pB, pn, pC, pn);
//		dtrma_lib(7, 3, pA, pn, pC+1, pn);

/*		dsyrk_lib(n, n, pA, pn, pC, pn);*/
/*		dgemm_nt_lib(n, n, n, pA, pn, pA, pn, pB, pn, 0);*/
/*		dpotrf_dcopy_lib(n, 0, pC, pn, pL, pn);*/



/*		dgemm_pup_nn_lib(n, n, n, pA, pn, B, n, pC, pn, 0);*/
/*		dgemm_ppp_nt_lib(n, n, n, pA, pn, pA, pn, pC+(bs-n)*(bs+1), pn, 1);*/
/*		d_print_pmat(pn, pn, bs, pC, pn);*/
/*		dpotrf_p_dcopy_u_lib(n, (bs-n%bs)%bs, pC+((bs-n%bs))%bs*(bs+1), pn, L, n);*/
/*		d_print_pmat(pn, pn, bs, pC, pn);*/
/*		d_print_mat(n, n, L, n);*/
/*		exit(2);*/

//		dgemm_nt_lib(n, n, n, A, n, B, n, C, n, 0);
//		dgemm_nt_lib_asm(n, n, n, pA, pn, pB, pn, pC, pn, 0);
//		sgemm_nt_lib_neon(n, n, n, spA, pns, spB, pns, spC, pns, 0);
//		dsymm_nt_lib(n, n, A, n, B, n, C, n);
//		dpotrf_lib(n, B, n);
//		dgemm_nt_lib2(n, pB, pA, pC, pn);
//		dgemv_n_lib(n-1, n, 1, pn, pA+1, x, y);
/*		dtrmv_p_n_lib(n-1, 1, pA+1+bs, pn, x, y, 0);*/
/*		dtrmv_p_t_lib(n-1, 1, pA+1+bs, pn, x, y, 0);*/
/*		dsymv_p_lib(n, 0, pA, pn, x, y, 0);*/

		}
	
	gettimeofday(&tv1, NULL); // stop

	float time = (float) (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);
	float flop = 2.0*n*n*n;
//	float flop = 1.0/3.0*n*n*n;
	float Gflops = 1e-9*flop/time;
	float Gflops_max = 1*1;
	
	printf("\nn\tGflops\t\t%%\n%d\t%f\t%f\n\n", n, Gflops, 100.0*Gflops/Gflops_max);

	if(n<=24)
		{
		d_print_pmat(pn, pn, bs, pA, pn);
		d_print_pmat(pn, pn, bs, pC, pn);
//		d_print_pmat(pn, cn, bs, pD, cn);
//		d_print_pmat(pn, cn, bs, pL, cn);
//		d_print_pmat(pn, cn, bs, pF, cn);
//		d_print_pmat(n, n, bs, pB, pn);
/*		d_print_pmat(n, n, bs, pA, pn);*/
/*		d_print_mat(n, n, B, n);*/
/*		d_print_pmat(n, n, bs, pB, pn);*/
/*		d_print_pmat(n, n, bs, pC, pn);*/
/*		d_print_pmat(n, n, bs, pL, pn);*/
//		s_print_pmat(n, n, bss, spC, pns);
/*		d_print_mat(n, 1, y, pn);*/
//		d_print_pmat(n, n, bs, pE+n*bs, cn2);
		}





	return 0;
	
	}
