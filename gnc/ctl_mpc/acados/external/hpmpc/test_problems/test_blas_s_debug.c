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

#include "../include/aux_s.h"
#include "../include/blas_s.h"
#include "../include/block_size.h"



int main()
	{
	
	int i, j, rep;
	
	const int bs  = S_MR;
	const int ncl = S_NCL;
	
	printf("\nbs = %d\n\n", bs);
	
	int n = 12;
	int nrep = 1;
	
	float *A; s_zeros(&A, n, n);
	float *B; s_zeros(&B, n, n);
	float *C; s_zeros(&C, n, n);
	float *L; s_zeros(&L, n, n);
	
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

//	s_print_mat(n, n, A, n);


	int pn = ((n+bs-1)/bs)*bs;//+4;	
	int cn = ((n+ncl-1)/ncl)*ncl;//+4;	
	int cn2 = ((2*n+ncl-1)/ncl)*ncl;	

	float *pA; s_zeros_align(&pA, pn, cn);
	float *pB; s_zeros_align(&pB, pn, cn);
	float *pC; s_zeros_align(&pC, pn, cn);
	float *pL; s_zeros_align(&pL, pn, cn);
	float *pD; s_zeros_align(&pD, pn, cn);
	float *pE; s_zeros_align(&pE, pn, cn2);
	float *pF; s_zeros_align(&pF, pn, cn);
	float *diag; s_zeros_align(&diag, pn, 1);
	
	s_cvt_mat2pmat(n, n, A, n, 0, pA, cn);
	s_cvt_mat2pmat(n, n, B, n, 0, pB, cn);
	s_cvt_mat2pmat(n, n, B, n, 0, pD, cn);
	s_cvt_mat2pmat(n, n, A, n, 0, pE, cn2);

//	s_print_pmat(n, n, bs, pA, cn);

	float *x; s_zeros_align(&x, n, 1);
	float *y; s_zeros_align(&y, n, 1);
	
	pA[0] = -1;
	x[2] = 1;
	for(i=0; i<n; i++) x[i] = i;
	for(i=0; i<n; i++) y[i] = 1;

	s_print_pmat(pn, cn, bs, pA, cn);
	s_print_pmat(pn, cn, bs, pB, cn);
	s_print_pmat(pn, cn, bs, pC, cn);
	s_print_pmat(pn, cn, bs, pD, cn);
	s_print_mat(1, n, x, 1);
	s_print_mat(1, n, y, 1);

	/* timing */
	struct timeval tv0, tv1;
	gettimeofday(&tv0, NULL); // start
	
	nrep = 1;

	for(rep=0; rep<nrep; rep++)
		{
		sgemm_nt_lib(n, n, n, pA, cn, pB, cn, 0, pC, cn); //, pD, cn, 0, 0);
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
		s_print_pmat(n, n, bs, pA, cn);
		s_print_pmat(n, n, bs, pC, cn);
		}

	return 0;

	}

