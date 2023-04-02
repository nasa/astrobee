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

//#include "data/pendulum_A.c"



int main()
	{
	
	int i, j, rep;
	
	const int bs  = D_MR;
	const int ncl = D_NCL;
	
	printf("\nbs = %d\n\n", bs);

#if 1

	int n = 16;

	int pn = (n+bs-1)/bs*bs;
	int cn = (n+ncl-1)/ncl*ncl;

	double *A; d_zeros(&A, n, n);
	for(i=0; i<n*n; i++)
//		A[i] = n*n-i;
		A[i] = i;
	
	double *Q; d_zeros(&Q, n, n);
	for(i=0; i<n; i++)
//		Q[i+n*i] = 100.0;
		Q[i+n*i] = 1.0;
	
	double *pA; d_zeros_align(&pA, pn, cn);
	d_cvt_mat2pmat(n, n, A, n, 0, pA, cn);

	double *pQ; d_zeros_align(&pQ, pn, cn);
	d_cvt_mat2pmat(n, n, Q, n, 0, pQ, cn);

	double *pC; d_zeros_align(&pC, pn, cn);
	dgemm_nt_lib(n, n, n, pA, cn, pA, cn, 1, pQ, cn, pC, cn, 0, 0);

	d_print_pmat(n, n, bs, pA, cn);
	d_print_pmat(n, n, bs, pQ, cn);
	d_print_pmat(n, n, bs, pC, cn);

	double *pD; d_zeros_align(&pD, pn, cn);
	double *inv_diag_D; d_zeros_align(&inv_diag_D, pn, 1);

	int *ipiv; i_zeros(&ipiv, n, 1);

//	dgetrf_lib(4, 16, pC, cn, pD, cn, inv_diag_D);
	dgetrf_pivot_lib(16, 16, pC, cn, pD, cn, inv_diag_D, ipiv);

//	d_print_pmat(n, n, bs, pC, cn);
	d_print_pmat_e(n, n, bs, pD, cn);
//	d_print_mat(1, n, inv_diag_D, 1);
	i_print_mat(1, n, ipiv, 1);

	free(A);
	free(Q);
	free(pA);
	free(pQ);
	free(pC);
	free(pD);
	free(inv_diag_D);
	free(ipiv);


#else

	int n = 15;

	int pn = (n+bs-1)/bs*bs;
	int cn = (n+ncl-1)/ncl*ncl;

	d_print_mat(n, n, A, n);

	double *pA; d_zeros_align(&pA, pn, cn);
	double *pD; d_zeros_align(&pD, pn, cn);
	double *inv_diag_D; d_zeros_align(&inv_diag_D, pn, 1);

	d_cvt_tran_mat2pmat(n, n, A, n, 0, pA, cn);
//	d_cvt_mat2pmat(n, n, A, n, 0, pA, cn);

	d_print_pmat(n, n, bs, pA, cn);


	// lu fact
	dgetrf_lib(n, pA, cn, pD, cn, inv_diag_D);

	d_print_pmat(n, n, bs, pD, cn);
	d_print_mat(1, n, inv_diag_D, 1);


	// free memory
	free(pA);
	free(pD);
	free(inv_diag_D);

#endif

	return 0;

	}
	
