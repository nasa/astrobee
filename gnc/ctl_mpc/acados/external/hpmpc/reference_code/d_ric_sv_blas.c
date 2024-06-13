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

// BLAS
//void dgemm_( char *transa, char *transb, int *m, int *n, int *k, double *alpha, double *A, int *lda, double *B, int *ldb, double *beta, double *C, int *ldc );
//void dtrmm_( char *side, char *uplo, char *transa, char *diag, int *m, int *n, double *alpha, double *A, int *lda, double *B, int *ldb ); // side='L' if B := alpha*op(A)*B
//void dtrsm_( char *side, char *uplo, char *transa, char *diag, int *m, int *n, double *alpha, double *A, int *lda, double *B, int *ldb ); // side='L' if op(A)*X = alpha*B
//void dsyrk_( char *uplo, char *trans, int *n, int *k, double *alpha, double *A, int *lda, double *beta, double *C, int *ldc ); // trans='N' if C := alpha*A*A' + beta*C
//void dgemv_( char *trans, int *m, int *n, double *alpha, double *A, int *lda, double *x, int *incx, double *beta, double *y, int *incy );
//void dtrmv_( char *uplo, char *trans, char *diag, int *n, double *A, int *lda, double *x, int *incx );
//void dtrsv_( char *uplo, char *trans, char *diag, int *n, double *A, int *lda, double *x, int *incx );

// LAPACK
//void dlauum_( char *uplo, int *n, double *A, int *lda, int *info ); // info==0 success
//void dpotrf_( char *uplo, int *n, double *A, int *lda, int *info );
//void dtrtri_( char *uplo, char *diag, int *n, double *A, int *lda, int *info ); // diag='N' if not_unit_triangular

#include "blas.h"
#include "lapack.h"



#if 0
// TODO return error code
void d_ric_trf_mhe_if_blas(int nx, int nw, int ndN, int N, double **A, double **G, double **Q, double **R, double **AGU, double **Up, double **Ue, double **Ur, double *Ud)
	{

	int ii, jj, nn;

	int nxw = nx+nw;

	int info;
	char c_l = 'L';
	char c_n = 'N';
	char c_r = 'R';
	char c_u = 'U';
	double d_0 = 0.0;
	double d_1 = 1.0;

	for(nn=0; nn<N; nn++)
		{

		//printf("\nn = %d\n\n", nn);

		// copy U to E
		for(jj=0; jj<nx; jj++)
			for(ii=0; ii<=jj; ii++)
				Ue[nn][ii+nx*jj] = Up[nn][ii+nx*jj];
		//d_print_mat(nx, nx, Ue[nn], nx);
		
		// E = U*U'
		dlauum_( &c_u, &nx, Ue[nn], &nx, &info );
		//d_print_mat(nx, nx, Ue[nn], nx);
		
		// E = Q + U*U'
		for(jj=0; jj<nx; jj++)
			for(ii=0; ii<=jj; ii++)
				Ue[nn][ii+nx*jj] += Q[nn][ii+nx*jj];
		//d_print_mat(nx, nx, Ue[nn], nx);
		
		// Ue = chol(E)
		dpotrf_( &c_u, &nx, Ue[nn], &nx, &info );
		//d_print_mat(nx, nx, Ue[nn], nx);
		
		// copy A to AGU
		for(jj=0; jj<nx*nx; jj++)
			AGU[nn][jj] = A[nn][jj];
		//d_print_mat(nx, nx+nw, AGU[nn], nx);
		
		// AUe = A * inv(Ue)
		dtrsm_( &c_r, &c_u, &c_n, &c_n, &nx, &nx, &d_1, Ue[nn], &nx, AGU[nn], &nx );
		//d_print_mat(nx, nx+nw, AGU[nn], nx);

		// copy R to Ur
		for(jj=0; jj<nw; jj++)
			for(ii=0; ii<=jj; ii++)
				Ur[nn][ii+nw*jj] = R[nn][ii+nw*jj];
		//d_print_mat(nw, nw, Ur[nn], nw);

		// Ur = chol(R)
		dpotrf_( &c_u, &nw, Ur[nn], &nw, &info );
		//d_print_mat(nw, nw, Ur[nn], nw);

		// copy G to AGU
		for(jj=0; jj<nx*nw; jj++)
			AGU[nn][nx*nx+jj] = G[nn][jj];
		//d_print_mat(nx, nx+nw, AGU[nn], nx);

		// GUr = G * inv(Ur)
		dtrsm_( &c_r, &c_u, &c_n, &c_n, &nx, &nw, &d_1, Ur[nn], &nw, AGU[nn]+nx*nx, &nx );
		//d_print_mat(nx, nx+nw, AGU[nn], nx);

		// U = AGu * AGu'
		dsyrk_( &c_u, &c_n, &nx, &nxw, &d_1, AGU[nn], &nx, &d_0, Up[nn+1], &nx );
		//d_print_mat(nx, nx, Up[nn+1], nx);
		
		// U = chol(U)
		dpotrf_( &c_u, &nx, Up[nn+1], &nx, &info );
		//d_print_mat(nx, nx, Up[nn+1], nx);
		
		// U = inv(U)
		dtrtri_( &c_u, &c_n, &nx, Up[nn+1], &nx, &info );
		//d_print_mat(nx, nx, Up[nn+1], nx);

		//exit(1);
		
		}

	// copy U to E
	for(jj=0; jj<nx; jj++)
		for(ii=0; ii<=jj; ii++)
			Ue[N][ii+nx*jj] = Up[N][ii+nx*jj];
	
	// E = U*U'
	dlauum_( &c_u, &nx, Ue[N], &nx, &info );

	// E = Q + U*U'
	for(jj=0; jj<nx; jj++)
		for(ii=0; ii<=jj; ii++)
			Ue[N][ii+nx*jj] += Q[N][ii+nx*jj];

	// Ue = chol(E)
	dpotrf_( &c_u, &nx, Ue[N], &nx, &info );

	if(ndN>0)
		{

		// copy D to AGU
		for(jj=0; jj<ndN*nx; jj++)
			AGU[N][jj] = A[N][jj];
		//d_print_mat(nx, nx+nw, AGU[nn], nx);
		
		// AUe = D * inv(Ue)
		dtrsm_( &c_r, &c_u, &c_n, &c_n, &ndN, &nx, &d_1, Ue[N], &nx, AGU[N], &ndN );
		//d_print_mat(nx, nx+nw, AGU[nn], nx);

		// U = AGU * AGU'
		dsyrk_( &c_u, &c_n, &ndN, &nx, &d_1, AGU[N], &ndN, &d_0, Ud, &ndN );
		//d_print_mat(ndN, ndN, Ud, ndN);

		// Ud = chol(Ud)
		dpotrf_( &c_u, &ndN, Ud, &ndN, &info );
		//d_print_mat(ndN, ndN, Ud, ndN);

		}

	//exit(1);

	}



void d_ric_trs_mhe_if_blas(int nx, int nw, int ndN, int N, double **AGU, double **Up, double **Ue, double **Ur, double *Ud, double **q, double **r, double **f, double **xp, double **x, double **w, double **lam, double *work)
	{

	int ii, jj, nn;

	int nxw = nx+nw;

	int info;
	int i_1 = 1;
	char c_l = 'L';
	char c_n = 'N';
	char c_r = 'R';
	char c_t = 'T';
	char c_u = 'U';
	double d_0 = 0.0;
	double d_1 = 1.0;
	double d_m1 = -1.0;

	double *x_temp, *w_temp;
	x_temp = work;
	work += nx;
	w_temp = work;
	work += nw;

	// forward substitution
	for(nn=0; nn<N; nn++)
		{

		for(ii=0; ii<nx; ii++) x_temp[ii] = xp[nn][ii];
		//d_print_mat(1, nx, x_temp, 1);
		//d_print_mat(nx, nx, Up[nn], nx);
		dtrmv_( &c_u, &c_t, &c_n, &nx, Up[nn], &nx, x_temp, &i_1 );
		//d_print_mat(1, nx, x_temp, 1);
		dtrmv_( &c_u, &c_n, &c_n, &nx, Up[nn], &nx, x_temp, &i_1 );
		//d_print_mat(1, nx, x_temp, 1);
		for(ii=0; ii<nx; ii++) x[nn][ii] = - q[nn][ii] + x_temp[ii];
		//d_print_mat(1, nx, x[nn], 1);

		dtrsv_( &c_u, &c_t, &c_n, &nx, Ue[nn], &nx, x[nn], &i_1 );
		//d_print_mat(1, nx, x[nn], 1);
		//for(ii=0; ii<nx; ii++) x[nn][ii] = - x[nn][ii];
		//d_print_mat(1, nx, x[nn], 1);
		for(ii=0; ii<nx; ii++) xp[nn+1][ii] = f[nn][ii];
		//d_print_mat(1, nx, xp[nn+1], 1);
		dgemv_( &c_n, &nx, &nx, &d_1, AGU[nn], &nx, x[nn], &i_1, &d_1, xp[nn+1], &i_1 );
		//d_print_mat(1, nx, xp[nn+1], 1);

		for(ii=0; ii<nw; ii++) w[nn][ii] = r[nn][ii];
		dtrsv_( &c_u, &c_t, &c_n, &nw, Ur[nn], &nw, w[nn], &i_1 );
		//d_print_mat(1, nw, w[nn], 1);
		dgemv_( &c_n, &nx, &nw, &d_m1, AGU[nn]+nx*nx, &nx, w[nn], &i_1, &d_1, xp[nn+1], &i_1 );
		//d_print_mat(1, nx, xp[nn+1], 1);

		//if(nn==1)
		//return;
		//exit(1);

		}

	//d_print_mat(1, nx, xp[N], 1);
	for(ii=0; ii<nx; ii++) x_temp[ii] = xp[N][ii];
	//d_print_mat(1, nx, x_temp, 1);
	//d_print_mat(nx, nx, Up[nn], nx);
	dtrmv_( &c_u, &c_t, &c_n, &nx, Up[N], &nx, x_temp, &i_1 );
	//d_print_mat(1, nx, x_temp, 1);
	dtrmv_( &c_u, &c_n, &c_n, &nx, Up[N], &nx, x_temp, &i_1 );
	//d_print_mat(1, nx, x_temp, 1);
	for(ii=0; ii<nx; ii++) x[N][ii] = - q[N][ii] + x_temp[ii];
	//d_print_mat(1, nx, x[N], 1);


	// backwars substitution
	if(ndN<=0)
		{
		dtrsv_( &c_u, &c_t, &c_n, &nx, Ue[N], &nx, x[N], &i_1 );
		//d_print_mat(1, nx, x[N], 1);
		dtrsv_( &c_u, &c_n, &c_n, &nx, Ue[N], &nx, x[N], &i_1 );
		//d_print_mat(1, nx, x[N], 1);
		}
	else
		{
		//d_print_mat(nx, nx, Ue[N], nx);
		//d_print_mat(ndN, nx, AGU[N], ndN);
		dtrsv_( &c_u, &c_t, &c_n, &nx, Ue[N], &nx, x[N], &i_1 );
		//d_print_mat(1, nx, x[N], 1);
		for(ii=0; ii<ndN; ii++) lam[N][ii] = f[N][ii];
		//d_print_mat(1, ndN, lam[N], 1);
		dgemv_( &c_n, &ndN, &nx, &d_1, AGU[N], &ndN, x[N], &i_1, &d_m1, lam[N], &i_1 );
		//d_print_mat(1, ndN, lam[N], 1);
		dtrsv_( &c_u, &c_t, &c_n, &ndN, Ud, &ndN, lam[N], &i_1 );
		dtrsv_( &c_u, &c_n, &c_n, &ndN, Ud, &ndN, lam[N], &i_1 );
		//d_print_mat(1, ndN, lam[N], 1);

		//d_print_mat(1, nx, x[N], 1);
		dgemv_( &c_t, &ndN, &nx, &d_m1, AGU[N], &ndN, lam[N], &i_1, &d_1, x[N], &i_1 );
		dtrsv_( &c_u, &c_n, &c_n, &nx, Ue[N], &nx, x[N], &i_1 );
		//d_print_mat(1, nx, x[N], 1);

		}
	
	for(nn=0; nn<N; nn++)
		{

		for(ii=0; ii<nx; ii++) lam[N-nn-1][ii] = xp[N-nn][ii] - x[N-nn][ii];
		//d_print_mat(1, nx, lam[N-nn-1], 1);
		dtrmv_( &c_u, &c_t, &c_n, &nx, Up[N-nn], &nx, lam[N-nn-1], &i_1 );
		//d_print_mat(1, nx, lam[N-nn-1], 1);
		dtrmv_( &c_u, &c_n, &c_n, &nx, Up[N-nn], &nx, lam[N-nn-1], &i_1 );
		//d_print_mat(1, nx, lam[N-nn-1], 1);

		dgemv_( &c_t, &nx, &nx, &d_m1, AGU[N-nn-1], &nx, lam[N-nn-1], &i_1, &d_1, x[N-nn-1], &i_1 );
		dtrsv_( &c_u, &c_n, &c_n, &nx, Ue[N-nn-1], &nx, x[N-nn-1], &i_1 );
		//d_print_mat(1, nx, x[N-nn-1], 1);

		dgemv_( &c_t, &nx, &nw, &d_m1, AGU[N-nn-1]+nx*nx, &nx, lam[N-nn-1], &i_1, &d_m1, w[N-nn-1], &i_1 );
		dtrsv_( &c_u, &c_n, &c_n, &nw, Ur[N-nn-1], &nw, w[N-nn-1], &i_1 );
		//d_print_mat(1, nw, w[N-nn-1], 1);

		}

	//d_print_mat(1, nw, w[0], 1);
	//exit(1);

	return;

	}
#endif



void d_ric_sv_mpc_blas(int nx, int nu, int N, double **BAbt, double **Q, double **Lp, double *BAbtL)
	{

	int ii, jj, nn;
	
	int nz  = nx+nu+1;
	int nx1 = nx+1;
	
	int info;
	char c_l = 'L';
	char c_n = 'N';
	char c_r = 'R';
	char c_t = 'T';
	char c_u = 'U';
	double d_0 = 0.0;
	double d_1 = 1.0;

	// copy P_N
	for(jj=0; jj<nx; jj++)
		for(ii=jj; ii<nx; ii++)
			Lp[N][nu+ii+nz*(nu+jj)] = Q[N][nu+ii+nz*(nu+jj)];

	// L_N = chol(P_N)
	dpotrf_( &c_l, &nx, Lp[N], &nx, &info );

	for(nn=0; nn<N-1; nn++)
		{

		// copy BAbt
		for(ii=0; ii<nx*nz; ii++)
			BAbtL[ii] = BAbt[N-nn-1][ii];

		// BAbtL = BAbt * L
		dtrmm_( &c_r, &c_l, &c_n, &c_n, &nz, &nx, &d_1, Lp[N-nn]+nu*(nz+1), &nz, BAbtL, &nz );
		for(ii=0; ii<nx; ii++) BAbtL[nx+nu+ii*nz] += Lp[N-nn][nx+nu+(nu+ii)*nz];

		// copy P_N
		for(jj=0; jj<nz; jj++)
			for(ii=jj; ii<nz; ii++)
				Lp[N-nn-1][ii+nz*jj] = Q[N-nn-1][ii+nz*jj];

		// Q_n += BAbtL * BAbtL'
		dsyrk_( &c_l, &c_n, &nz, &nx, &d_1, BAbtL, &nz, &d_1, Lp[N-nn-1], &nz );

		// L_n = chol(Q_n)
		dpotrf_( &c_l, &nz, Lp[N-nn-1], &nz, &info );

		}
	
	// copy BAbt
	for(ii=0; ii<nx*nz; ii++)
		BAbtL[ii] = BAbt[0][ii];

	// BAbtL = BAbt * L
	dtrmm_( &c_r, &c_l, &c_n, &c_n, &nz, &nx, &d_1, Lp[1]+nu*(nz+1), &nz, BAbtL, &nz );
	for(ii=0; ii<nx; ii++) BAbtL[nx+nu+ii*nz] += Lp[1][nx+nu+(nu+ii)*nz];

	// copy P_N
	for(jj=0; jj<nu; jj++)
		for(ii=jj; ii<nz; ii++)
			Lp[0][ii+nz*jj] = Q[0][ii+nz*jj];

	// Q_n += BAbtL * BAbtL'
	dsyrk_( &c_l, &c_n, &nu, &nx, &d_1, BAbtL, &nz, &d_1, Lp[0], &nz );
	dgemm_( &c_n, &c_t, &nx1, &nu, &nx, &d_1, BAbtL+nu, &nz, BAbtL, &nz, &d_1, Lp[0]+nu, &nz );

	// L_n = chol(Q_n)
	dpotrf_( &c_l, &nu, Lp[0], &nz, &info );
	dtrsm_( &c_r, &c_l, &c_t, &c_n, &nx1, &nu, &d_1, Lp[0], &nz, Lp[0]+nu, &nz );

	// TODO forward loop

	}



void d_back_ric_trf_tv_blas(int N, int *nx, int *nu, double **BAbt, double **Q, double **Lp, double *BAbtL)
	{

	int nu0, nu1, nx0, nx1, nz0, nz1, nux0, nux1;

	int ii, jj, ll, nn, tmp;

	int info;
	int i_1 = 1;
	char c_l = 'L';
	char c_n = 'N';
	char c_r = 'R';
	char c_t = 'T';
	char c_u = 'U';
	double d_0 = 0.0;
	double d_1 = 1.0;

	// factorization and backward substitution 

	// final stage 
	nu0 = nu[N];
	nx0 = nx[N];
	nux0 = nu0+nx0;
	nz0 = nux0+1;

	// copy Q_N
	for(jj=0; jj<nux0; jj++)
		{
		tmp = nux0-jj;
		dcopy_(&tmp, Q[N]+jj*(nz0+1), &i_1, Lp[N]+jj*(nz0+1), &i_1);
		// TODO dcopy_
//		for(ii=jj; ii<nux0; ii++)
//			Lp[N][ii+nz0*jj] = Q[N][ii+nz0*jj];
		}

	// L_N = chol(P_N)
	dpotrf_( &c_l, &nux0, Lp[N], &nz0, &info );



	// middle stages 
	for(nn=0; nn<N; nn++)
		{	
		nu1 = nu0;
		nx1 = nx0;
		nux1 = nux0;
		nz1 = nz0;
		nu0 = nu[N-nn-1];
		nx0 = nx[N-nn-1];
		nux0 = nu0+nx0;
		nz0 = nux0+1;

		// copy BAbt
		for(jj=0; jj<nx1; jj++)
			{
			dcopy_(&nux0, BAbt[N-nn-1]+jj*nz0, &i_1, BAbtL+jj*nz0, &i_1);
			// TODO dcopy_
//			for(ii=0; ii<nux0; ii++)
//				BAbtL[ii+jj*nz0] = BAbt[N-nn-1][ii+jj*nz0];
			}

		// BAbtL = BAbt * L
		dtrmm_( &c_r, &c_l, &c_n, &c_n, &nux0, &nx1, &d_1, Lp[N-nn]+nu1*(nz1+1), &nz1, BAbtL, &nz0 );

		// copy Q_n
		for(jj=0; jj<nux0; jj++)
			{
			tmp = nux0-jj;
			dcopy_(&tmp, Q[N-nn-1]+jj*(nz0+1), &i_1, Lp[N-nn-1]+jj*(nz0+1), &i_1);
			// TODO dcopy_
//			for(ii=jj; ii<nux0; ii++)
//				Lp[N-nn-1][ii+nz0*jj] = Q[N-nn-1][ii+nz0*jj];
			}

		// Q_n += BAbtL * BAbtL'
		dsyrk_( &c_l, &c_n, &nux0, &nx1, &d_1, BAbtL, &nz0, &d_1, Lp[N-nn-1], &nz0 );

		// L_n = chol(Q_n)
		dpotrf_( &c_l, &nux0, Lp[N-nn-1], &nz0, &info );


		}

	}



void d_back_ric_trs_tv_blas(int N, int *nx, int *nu, double **BAbt, double **b, double **Lp, double **q, double **l, double **ux, double *work, int compute_Pb, double ** Pb, int compute_pi, double **pi)
	{
	
	int nu0, nu1, nx0, nx1, nux0, nux1, nz0, nz1;

	int ii, jj, nn;
	
	int info;
	int i_1 = 1;
	char c_l = 'L';
	char c_n = 'N';
	char c_r = 'R';
	char c_t = 'T';
	char c_u = 'U';
	double d_0 = 0.0;
	double d_1 = 1.0;
	double d_m1 = -1.0;

	// backward substitution 

	// final stage
	nu0 = nu[N];
	nx0 = nx[N];
	nux0 = nu0+nx0;
	nz0 = nux0+1;

	// copy q in l
//	for(ii=0; ii<nu0+nx0; ii++) hl[N][ii] = hq[N][ii];
	dcopy_(&nux0, q[N], &i_1, l[N], &i_1);

	// middle stages
	for(nn=0; nn<N; nn++)
		{
		nu1 = nu0;
		nx1 = nx0;
		nz1 = nz0;
		nu0 = nu[N-nn-1];
		nx0 = nx[N-nn-1];
		nux0 = nu0+nx0;
		nz0 = nux0+1;
		if(compute_Pb)
			{
			dcopy_(&nx1, b[N-nn-1], &i_1, Pb[N-nn-1], &i_1);
			dtrmv_(&c_l, &c_t, &c_n, &nx1, Lp[N-nn]+nu1*(nz1+1), &nz1, Pb[N-nn-1], &i_1);
			dtrmv_(&c_l, &c_n, &c_n, &nx1, Lp[N-nn]+nu1*(nz1+1), &nz1, Pb[N-nn-1], &i_1);
			}
		// copy q in l
		dcopy_(&nux0, q[N-nn-1], &i_1, l[N-nn-1], &i_1);

		dcopy_(&nx1, l[N-nn], &i_1, work, &i_1);
		daxpy_(&nx1, &d_1, Pb[N-nn-1], &i_1, work, &i_1);

		dgemv_(&c_n, &nux0, &nx1, &d_1, BAbt[N-nn-1], &nz0, work, &i_1, &d_1, l[N-nn-1], &i_1);
		dtrsv_(&c_l, &c_n, &c_n, &nu0, Lp[N-nn-1], &nz0, l[N-nn-1], &i_1);
		dgemv_(&c_n, &nx0, &nu0, &d_m1, Lp[N-nn-1]+nu0, &nz0, l[N-nn-1], &i_1, &d_1, l[N-nn-1]+nu0, &i_1);
		}


	// forward substitution 

	nu1 = nu[0];
	nx1 = nx[0];
	nux1 = nu1+nx1;
	nz1 = nux1+1;
	// first stage
	nn = 0;
	nu0 = nu1;
	nx0 = nx1;
	nz0 = nz1;
	nu1 = nu[nn+1];
	nx1 = nx[nn+1];
	nux1 = nu1+nx1;
	nz1 = nux1+1;

	for(jj=0; jj<nu0; jj++) ux[nn][jj] = - l[nn][jj];
	dtrsv_(&c_l, &c_t, &c_n, &nx0, Lp[nn]+nu0*(nz0+1), &nz0, ux[nn]+nu0, &i_1);
	dgemv_(&c_t, &nx0, &nu0, &d_m1, Lp[nn]+nu0, &nz0, ux[nn]+nu0, &i_1, &d_1, ux[nn], &i_1);
	dtrsv_(&c_l, &c_t, &c_n, &nu0, Lp[nn], &nz0, ux[nn], &i_1);
	dcopy_(&nx1, b[nn], &i_1, ux[nn+1]+nu1, &i_1);
	dgemv_(&c_t, &nux0, &nx1, &d_1, BAbt[nn], &nz0, ux[nn], &i_1, &d_1, ux[nn+1]+nu1, &i_1);
	if(compute_pi)
		{
		dcopy_(&nx1, ux[nn+1]+nu1, &i_1, pi[nn+1], &i_1);
		dtrmv_(&c_l, &c_t, &c_n, &nx1, Lp[nn+1]+nu1*(nz1+1), &nz1, pi[nn+1], &i_1);
		dtrmv_(&c_l, &c_n, &c_n, &nx1, Lp[nn+1]+nu1*(nz1+1), &nz1, pi[nn+1], &i_1);
		daxpy_(&nx1, &d_1, l[nn+1]+nu1, &i_1, pi[nn+1], &i_1);
		}
	// middle stages
	for(nn=1; nn<N; nn++)
		{
		nu0 = nu1;
		nx0 = nx1;
		nux0 = nux1;
		nz0 = nz1;
		nu1 = nu[nn+1];
		nx1 = nx[nn+1];
		nux1 = nu1+nx1;
		nz1 = nux1+1;
		for(jj=0; jj<nu0; jj++) ux[nn][jj] = - l[nn][jj];
		dgemv_(&c_t, &nx0, &nu0, &d_m1, Lp[nn]+nu0, &nz0, ux[nn]+nu0, &i_1, &d_1, ux[nn], &i_1);
		dtrsv_(&c_l, &c_t, &c_n, &nu0, Lp[nn], &nz0, ux[nn], &i_1);
		dcopy_(&nx1, b[nn], &i_1, ux[nn+1]+nu1, &i_1);
		dgemv_(&c_t, &nux0, &nx1, &d_1, BAbt[nn], &nz0, ux[nn], &i_1, &d_1, ux[nn+1]+nu1, &i_1);

		if(compute_pi)
			{
			dcopy_(&nx1, ux[nn+1]+nu1, &i_1, pi[nn+1], &i_1);
			dtrmv_(&c_l, &c_t, &c_n, &nx1, Lp[nn+1]+nu1*(nz1+1), &nz1, pi[nn+1], &i_1);
			dtrmv_(&c_l, &c_n, &c_n, &nx1, Lp[nn+1]+nu1*(nz1+1), &nz1, pi[nn+1], &i_1);
			daxpy_(&nx1, &d_1, l[nn+1]+nu1, &i_1, pi[nn+1], &i_1);
			}
		}

	}



int d_forward_schur_trf_tv_blas(int N, int *nv, int *ne, double reg, int *diag_hessian, double **QA, double **LA, double **Le, double *Le_tmp)
	{

	// XXX ONLY FOR FULL HESSIAN !!!!!

	int ii, jj, ll;	



	float diag_min;
	diag_min = 1.0;


	int nv0, ne0, ne1, nve0;

	int tmp;

	int info;
	char c_l = 'L';
	char c_n = 'N';
	char c_r = 'R';
	char c_t = 'T';
	char c_u = 'U';
	double d_0 = 0.0;
	double d_1 = 1.0;
	int i_1 = 1;


	// first stage
	ii = 0;
	nv0 = nv[0];
	ne0 = ne[0];
	nve0 = nv0 + ne0;

	if(diag_hessian[0])
		{

		return -1;

		}
	else
		{

		for(jj=0; jj<nv0; jj++)
			{
			tmp = nv0+ne0-jj;
			dcopy_(&tmp, QA[ii]+jj*(nve0+1), &i_1, LA[ii]+jj*(nve0+1), &i_1);
			}

		for(jj=0; jj<nv0; jj++)
			LA[ii][jj*(nve0+1)] += reg;

		dpotrf_(&c_l, &nv0, LA[ii], &nve0, &info);
		dtrsm_(&c_r, &c_l, &c_t, &c_n, &ne0, &nv0, &d_1, LA[ii], &nve0, LA[ii]+nv0, &nve0);

		}
		
	dsyrk_(&c_l, &c_n, &ne0, &nv0, &d_1, LA[ii]+nv0, &nve0, &d_0, Le[ii], &ne0);
	for(jj=0; jj<ne0; jj++) Le[ii][jj*(ne0+1)] += reg;
	dpotrf_(&c_l, &ne0, Le[ii], &ne0, &info);

	dtrtri_(&c_l, &c_n, &ne0, Le[ii], &ne0, &info);



	// middle stages
	for(ii=1; ii<N; ii++)
		{

		ne1  = ne0;
		nv0  = nv[ii];
		ne0  = ne[ii];
		nve0 = nv0 + ne0;

		if(diag_hessian[ii])
			{

			return -1;

			}
		else
			{

			for(jj=0; jj<ne1; jj++)
				{
				tmp = ne1-jj;
				dcopy_(&tmp, Le[ii-1]+jj*(ne1+1), &i_1, Le_tmp+jj*(ne1+1), &i_1);
				}

			dlauum_(&c_l, &ne1, Le_tmp, &ne1, &info);

			for(jj=0; jj<nv0; jj++)
				{
				tmp = nv0+ne0-jj;
				dcopy_(&tmp, QA[ii]+jj*(nve0+1), &i_1, LA[ii]+jj*(nve0+1), &i_1);
				}

			for(jj=0; jj<nv0; jj++)
				LA[ii][jj*(nve0+1)] += reg;

			for(jj=0; jj<ne1; jj++)
				{
				tmp = ne1-jj;
				daxpy_(&tmp, &d_1, Le_tmp+jj*(ne1+1), &i_1, LA[ii]+jj*(nve0+1), &i_1);
				}

			dpotrf_(&c_l, &nv0, LA[ii], &nve0, &info);
			dtrsm_(&c_r, &c_l, &c_t, &c_n, &ne0, &nv0, &d_1, LA[ii], &nve0, LA[ii]+nv0, &nve0);

			}


		dsyrk_(&c_l, &c_n, &ne0, &nv0, &d_1, LA[ii]+nv0, &nve0, &d_0, Le[ii], &ne0);
		for(jj=0; jj<ne0; jj++) Le[ii][jj*(ne0+1)] += reg;
		dpotrf_(&c_l, &ne0, Le[ii], &ne0, &info);

		dtrtri_(&c_l, &c_n, &ne0, Le[ii], &ne0, &info);

		}


	// last stage
	ii = N;
	ne1  = ne0;
	nv0  = nv[ii];
	ne0  = ne[ii];
	nve0 = nv0 + ne0;


	if(diag_hessian[N])
		{

		return -1;

		}
	else
		{

		for(jj=0; jj<ne1; jj++)
			{
			tmp = ne1-jj;
			dcopy_(&tmp, Le[ii-1]+jj*(ne1+1), &i_1, Le_tmp+jj*(ne1+1), &i_1);
			}

		dlauum_(&c_l, &ne1, Le_tmp, &ne1, &info);

		for(jj=0; jj<nv0; jj++)
			{
			tmp = nv0+ne0-jj;
			dcopy_(&tmp, QA[ii]+jj*(nve0+1), &i_1, LA[ii]+jj*(nve0+1), &i_1);
			}

		for(jj=0; jj<nv0; jj++)
			LA[ii][jj*(nve0+1)] += reg;

		for(jj=0; jj<ne1; jj++)
			{
			tmp = ne1-jj;
			daxpy_(&tmp, &d_1, Le_tmp+jj*(ne1+1), &i_1, LA[ii]+jj*(nve0+1), &i_1);
			}

		}

	dpotrf_(&c_l, &nv0, LA[ii], &nve0, &info);
	dtrsm_(&c_r, &c_l, &c_t, &c_n, &ne0, &nv0, &d_1, LA[ii], &nve0, LA[ii]+nv0, &nve0);

	if(ne0>0)
		{

		dsyrk_(&c_l, &c_n, &ne0, &nv0, &d_1, LA[ii]+nv0, &nve0, &d_0, Le[ii], &ne0);
		for(jj=0; jj<ne0; jj++) Le[ii][jj*(ne0+1)] += reg;
		dpotrf_(&c_l, &ne0, Le[ii], &ne0, &info);

		dtrtri_(&c_l, &c_n, &ne0, Le[ii], &ne0, &info);
		
		}

//	d_print_mat(nv0+ne0, nv0, LA[ii], nv0);
//	exit(1);

	return 0;

	}



int d_forward_schur_trs_tv_blas(int N, int *nv, int *ne, int *diag_hessian, double **qb, double **LA, double **Le, double **xupi, double *work)
	{

	int ii, jj, ll;	

	int nv0, nv1, ne0, ne1, nve0;

	int info;
	int i_1 = 1;
	char c_l = 'L';
	char c_n = 'N';
	char c_r = 'R';
	char c_t = 'T';
	char c_u = 'U';
	double d_0 = 0.0;
	double d_1 = 1.0;
	double d_m1 = -1.0;


	// forward recursion

	// first stage
	ii = 0;
	nv0  = nv[0];
	ne0  = ne[0];
	nve0 = nv0 + ne0;

	dcopy_(&nve0, qb[ii], &i_1, xupi[ii], &i_1);

	if(diag_hessian[ii])
		{

		return -1;

		}
	else
		{

		dtrsv_(&c_l, &c_n, &c_n, &nv0, LA[ii], &nve0, xupi[ii], &i_1);
		dgemv_(&c_n, &ne0, &nv0, &d_m1, LA[ii]+nv0, &nve0, xupi[ii], &i_1, &d_1, xupi[ii]+nv0, &i_1);

		}
	


	// middle stages
	for(ii=1; ii<=N; ii++)
		{

		ne1  = ne0;
		nv1  = nv0;
		nv0  = nv[ii];
		ne0  = ne[ii];
		nve0 = nv0 + ne0;

		dcopy_(&ne1, xupi[ii-1]+nv1, &i_1, work, &i_1);
		dtrmv_(&c_l, &c_n, &c_n, &ne1, Le[ii-1], &ne1, work, &i_1);
		dtrmv_(&c_l, &c_t, &c_n, &ne1, Le[ii-1], &ne1, work, &i_1);

		dcopy_(&nve0, qb[ii], &i_1, xupi[ii], &i_1);
		daxpy_(&ne1, &d_m1, work, &i_1, xupi[ii], &i_1);

		if(diag_hessian[ii])
			{

			return -1;

			}
		else
			{

			dtrsv_(&c_l, &c_n, &c_n, &nv0, LA[ii], &nve0, xupi[ii], &i_1);
			dgemv_(&c_n, &ne0, &nv0, &d_m1, LA[ii]+nv0, &nve0, xupi[ii], &i_1, &d_1, xupi[ii]+nv0, &i_1);

			}

//		d_print_mat(1, nv0+ne0, xupi[ii], 1);
//		exit(1);

		}
	

	// last stage
	ii = N;
	if(ne0>0)
		{
		dtrmv_(&c_l, &c_n, &c_n, &ne0, Le[N], &ne0, xupi[N]+nv0, &i_1);
		dtrmv_(&c_l, &c_t, &c_n, &ne0, Le[N], &ne0, xupi[N]+nv0, &i_1);
		}
	// backward recursion

	// last stage
	for(jj=0; jj<nv0; jj++) xupi[N][jj] = - xupi[N][jj];

//	d_print_mat(1, pnv0+pne0, hxupi[ii], 1);

	if(diag_hessian[ii])
		{

		return -1;

		}
	else
		{

		dgemv_(&c_t, &ne0, &nv0, &d_m1, LA[N]+nv0, &nve0, xupi[N]+nv0, &i_1, &d_1, xupi[N], &i_1);
		dtrsv_(&c_l, &c_t, &c_n, &nv0, LA[N], &nve0, xupi[N], &i_1);

		}
	
//	d_print_mat(1, nv0+ne0, xupi[ii], 1);
//	return 0;
//	exit(1);

//	ne1 = ne[N-1];

	// middle stages
	for(ii=1; ii<N; ii++)
		{
		
		ne0  = ne1;
		ne1  = ne[N-ii-1];//ne0;
		nv0  = nv[N-ii];
		nve0 = nv0 + ne0;

		for(jj=0; jj<ne0; jj++) xupi[N-ii][nv0+jj] = xupi[N-ii][nv0+jj] - xupi[N-ii+1][jj];
		dtrmv_(&c_l, &c_n, &c_n, &ne0, Le[N-ii], &ne0, xupi[N-ii]+nv0, &i_1);
		dtrmv_(&c_l, &c_t, &c_n, &ne0, Le[N-ii], &ne0, xupi[N-ii]+nv0, &i_1);

		for(jj=0; jj<nv0; jj++) xupi[N-ii][jj] = - xupi[N-ii][jj];

		if(diag_hessian[N-ii])
			{

			return -1;

			}
		else
			{

			dgemv_(&c_t, &ne0, &nv0, &d_m1, LA[N-ii]+nv0, &nve0, xupi[N-ii]+nv0, &i_1, &d_1, xupi[N-ii], &i_1);
			dtrsv_(&c_l, &c_t, &c_n, &nv0, LA[N-ii], &nve0, xupi[N-ii], &i_1);

			}

		}
	
	// first stage
	ii = N;
	ne0  = ne1;
	ne1  = ne[N-ii-1];//ne0;
	nv0  = nv[N-ii];
	nve0 = nv0 + ne0;

	for(jj=0; jj<ne0; jj++) xupi[N-ii][nv0+jj] = xupi[N-ii][nv0+jj] - xupi[N-ii+1][jj];
	dtrmv_(&c_l, &c_n, &c_n, &ne0, Le[N-ii], &ne0, xupi[N-ii]+nv0, &i_1);
	dtrmv_(&c_l, &c_t, &c_n, &ne0, Le[N-ii], &ne0, xupi[N-ii]+nv0, &i_1);

	for(jj=0; jj<nv0; jj++) xupi[N-ii][jj] = - xupi[N-ii][jj];

	if(diag_hessian[N-ii])
		{

		return -1;

		}
	else
		{

		dgemv_(&c_t, &ne0, &nv0, &d_m1, LA[N-ii]+nv0, &nve0, xupi[N-ii]+nv0, &i_1, &d_1, xupi[N-ii], &i_1);
		dtrsv_(&c_l, &c_t, &c_n, &nv0, LA[N-ii], &nve0, xupi[N-ii], &i_1);

		}
	
//	d_print_mat(1, pnv0+pne0, hxupi[N-ii], 1);
//	exit(1);

	return 0;

	}




