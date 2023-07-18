/**************************************************************************************************
*                                                                                                 *
* This file is part of HPIPM.                                                                     *
*                                                                                                 *
* HPIPM -- High-Performance Interior Point Method.                                                *
* Copyright (C) 2019 by Gianluca Frison.                                                          *
* Developed at IMTEK (University of Freiburg) under the supervision of Moritz Diehl.              *
* All rights reserved.                                                                            *
*                                                                                                 *
* The 2-Clause BSD License                                                                        *
*                                                                                                 *
* Redistribution and use in source and binary forms, with or without                              *
* modification, are permitted provided that the following conditions are met:                     *
*                                                                                                 *
* 1. Redistributions of source code must retain the above copyright notice, this                  *
*    list of conditions and the following disclaimer.                                             *
* 2. Redistributions in binary form must reproduce the above copyright notice,                    *
*    this list of conditions and the following disclaimer in the documentation                    *
*    and/or other materials provided with the distribution.                                       *
*                                                                                                 *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND                 *
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED                   *
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE                          *
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR                 *
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES                  *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;                    *
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND                     *
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT                      *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS                   *
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                    *
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
#include <blasfeo_s_aux_ext_dep.h>
#include <blasfeo_i_aux_ext_dep.h>
#include <blasfeo_s_aux.h>

#include <hpipm_tree.h>
#include <hpipm_scenario_tree.h>
#include <hpipm_s_tree_ocp_qp.h>
#include <hpipm_s_tree_ocp_qp_sol.h>
#include <hpipm_s_tree_ocp_qp_ipm_hard.h>

#include "s_tools.h"



#define KEEP_X0 0

// printing
#define PRINT 0



#if ! defined(EXT_DEP)
/* creates a zero matrix */
void s_zeros(float **pA, int row, int col)
	{
	*pA = malloc((row*col)*sizeof(float));
	float *A = *pA;
	int i;
	for(i=0; i<row*col; i++) A[i] = 0.0;
	}
/* frees matrix */
void s_free(float *pA)
	{
	free( pA );
	}
/* prints a matrix in column-major format */
void s_print_mat(int m, int n, float *A, int lda)
	{
	int i, j;
	for(i=0; i<m; i++)
		{
		for(j=0; j<n; j++)
			{
			printf("%9.5f ", A[i+lda*j]);
			}
		printf("\n");
		}
	printf("\n");
	}	
/* prints the transposed of a matrix in column-major format */
void s_print_tran_mat(int row, int col, float *A, int lda)
	{
	int i, j;
	for(j=0; j<col; j++)
		{
		for(i=0; i<row; i++)
			{
			printf("%9.5f ", A[i+lda*j]);
			}
		printf("\n");
		}
	printf("\n");
	}	
/* prints a matrix in column-major format (exponential notation) */
void s_print_exp_mat(int m, int n, float *A, int lda)
	{
	int i, j;
	for(i=0; i<m; i++)
		{
		for(j=0; j<n; j++)
			{
			printf("%e\t", A[i+lda*j]);
			}
		printf("\n");
		}
	printf("\n");
	}	
/* prints the transposed of a matrix in column-major format (exponential notation) */
void s_print_exp_tran_mat(int row, int col, float *A, int lda)
	{
	int i, j;
	for(j=0; j<col; j++)
		{
		for(i=0; i<row; i++)
			{
			printf("%e\t", A[i+lda*j]);
			}
		printf("\n");
		}
	printf("\n");
	}	
/* creates a zero matrix aligned */
void int_zeros(int **pA, int row, int col)
	{
	void *temp = malloc((row*col)*sizeof(int));
	*pA = temp;
	int *A = *pA;
	int i;
	for(i=0; i<row*col; i++) A[i] = 0;
	}
/* frees matrix */
void int_free(int *pA)
	{
	free( pA );
	}
/* prints a matrix in column-major format */
void int_print_mat(int row, int col, int *A, int lda)
	{
	int i, j;
	for(i=0; i<row; i++)
		{
		for(j=0; j<col; j++)
			{
			printf("%d ", A[i+lda*j]);
			}
		printf("\n");
		}
	printf("\n");
	}	
#endif



/************************************************ 
Mass-spring system: nx/2 masses connected each other with springs (in a row), and the first and the last one to walls. nu (<=nx) controls act on the first nu masses. The system is sampled with sampling time Ts. 
************************************************/
void mass_spring_system(float Ts, int nx, int nu, float *A, float *B, float *b, float *x0)
	{

	int nx2 = nx*nx;

	int info = 0;

	int pp = nx/2; // number of masses
	
/************************************************
* build the continuous time system 
************************************************/
	
	float *T; s_zeros(&T, pp, pp);
	int ii;
	for(ii=0; ii<pp; ii++) T[ii*(pp+1)] = -2;
	for(ii=0; ii<pp-1; ii++) T[ii*(pp+1)+1] = 1;
	for(ii=1; ii<pp; ii++) T[ii*(pp+1)-1] = 1;

	float *Z; s_zeros(&Z, pp, pp);
	float *I; s_zeros(&I, pp, pp); for(ii=0; ii<pp; ii++) I[ii*(pp+1)]=1.0; // = eye(pp);
	float *Ac; s_zeros(&Ac, nx, nx);
	smcopy(pp, pp, Z, pp, Ac, nx);
	smcopy(pp, pp, T, pp, Ac+pp, nx);
	smcopy(pp, pp, I, pp, Ac+pp*nx, nx);
	smcopy(pp, pp, Z, pp, Ac+pp*(nx+1), nx); 
	free(T);
	free(Z);
	free(I);
	
	s_zeros(&I, nu, nu); for(ii=0; ii<nu; ii++) I[ii*(nu+1)]=1.0; //I = eye(nu);
	float *Bc; s_zeros(&Bc, nx, nu);
	smcopy(nu, nu, I, nu, Bc+pp, nx);
	free(I);
	
/************************************************
* compute the discrete time system 
************************************************/

	float *bb; s_zeros(&bb, nx, 1);
	smcopy(nx, 1, bb, nx, b, nx);
		
	smcopy(nx, nx, Ac, nx, A, nx);
	sscal_3l(nx2, Ts, A);
	expm(nx, A);
	
	s_zeros(&T, nx, nx);
	s_zeros(&I, nx, nx); for(ii=0; ii<nx; ii++) I[ii*(nx+1)]=1.0; //I = eye(nx);
	smcopy(nx, nx, A, nx, T, nx);
	saxpy_3l(nx2, -1.0, I, T);
	sgemm_nn_3l(nx, nu, nx, T, nx, Bc, nx, B, nx);
	free(T);
	free(I);
	
	int *ipiv = (int *) malloc(nx*sizeof(int));
	sgesv_3l(nx, nu, Ac, nx, ipiv, B, nx, &info);
	free(ipiv);

	free(Ac);
	free(Bc);
	free(bb);
	
			
/************************************************
* initial state 
************************************************/
	
	if(nx==4)
		{
		x0[0] = 5;
		x0[1] = 10;
		x0[2] = 15;
		x0[3] = 20;
		}
	else
		{
		int jj;
		for(jj=0; jj<nx; jj++)
			x0[jj] = 1;
		}

	}



int main()
	{

	int ii, jj;
	int stage;

	int nx_ = 8;
	int nu_ = 3;

	int md = 3;
	int Nr = 2;
	int Nh = 3;

	// stage-wise size
	int nx[Nh+1];
	int nu[Nh+1];
	int nb[Nh+1];
	int ng[Nh+1];

	nx[0] = 0;
	nu[0] = nu_;
	nb[0] = nu[0]+nx[0];
	ng[0] = 0;
	for(ii=1; ii<Nh; ii++)
		{
		nx[ii] = nx_;
		nu[ii] = nu_;
		nb[ii] = nu[ii]+nx[ii];
		ng[ii] = 0;
		}
	nx[Nh] = nx_;
	nu[Nh] = 0;
	nb[Nh] = nu[Nh]+nx[Nh];
	ng[Nh] = 0;

/************************************************
* dynamical system
************************************************/	

	float *A; s_zeros(&A, nx_, nx_); // states update matrix

	float *B; s_zeros(&B, nx_, nu_); // inputs matrix

	float *b; s_zeros(&b, nx_, 1); // states offset
	float *x0; s_zeros(&x0, nx_, 1); // initial state

	float Ts = 0.5; // sampling time
	mass_spring_system(Ts, nx_, nu_, A, B, b, x0);
	
	for(jj=0; jj<nx_; jj++)
		b[jj] = 0.1;
	
	for(jj=0; jj<nx_; jj++)
		x0[jj] = 0;
	x0[0] = 2.5;
	x0[1] = 2.5;

	float *b0; s_zeros(&b0, nx_, 1);
	sgemv_n_3l(nx_, nx_, A, nx_, x0, b0);
	saxpy_3l(nx_, 1.0, b, b0);

#if 0
	s_print_mat(nx_, nx_, A, nx_);
	s_print_mat(nx_, nu_, B, nu_);
	s_print_mat(1, nx_, b, 1);
	s_print_mat(1, nx_, x0, 1);
	s_print_mat(1, nx_, b0, 1);
#endif

/************************************************
* cost function
************************************************/	
	
	float *Q; s_zeros(&Q, nx_, nx_);
	for(ii=0; ii<nx_; ii++) Q[ii*(nx_+1)] = 1.0;

	float *R; s_zeros(&R, nu_, nu_);
	for(ii=0; ii<nu_; ii++) R[ii*(nu_+1)] = 2.0;

	float *S; s_zeros(&S, nu_, nx_);

	float *q; s_zeros(&q, nx_, 1);
	for(ii=0; ii<nx_; ii++) q[ii] = 0.1;

	float *r; s_zeros(&r, nu_, 1);
	for(ii=0; ii<nu_; ii++) r[ii] = 0.2;

	float *r0; s_zeros(&r0, nu_, 1);
	sgemv_n_3l(nu_, nx_, S, nu_, x0, r0);
	saxpy_3l(nu_, 1.0, r, r0);

#if 0
	s_print_mat(nx_, nx_, Q, nx_);
	s_print_mat(nu_, nu_, R, nu_);
	s_print_mat(nu_, nx_, S, nu_);
	s_print_mat(1, nx_, q, 1);
	s_print_mat(1, nu_, r, 1);
	s_print_mat(1, nu_, r0, 1);
#endif

	// maximum element in cost functions
	float mu0 = 2.0;

/************************************************
* box & general constraints
************************************************/	

	int *idxb0; int_zeros(&idxb0, nb[0], 1);
	float *d_lb0; s_zeros(&d_lb0, nb[0], 1);
	float *d_ub0; s_zeros(&d_ub0, nb[0], 1);
	float *d_lg0; s_zeros(&d_lg0, ng[0], 1);
	float *d_ug0; s_zeros(&d_ug0, ng[0], 1);
	for(ii=0; ii<nb[0]; ii++)
		{
		if(ii<nu[0]) // input
			{
			d_lb0[ii] = - 0.5; // umin
			d_ub0[ii] =   0.5; // umax
			}
		else // state
			{
			d_lb0[ii] = - 4.0; // xmin
			d_ub0[ii] =   4.0; // xmax
			}
		idxb0[ii] = ii;
		}
	for(ii=0; ii<ng[0]; ii++)
		{
		if(ii<nu[0]-nb[0]) // input
			{
			d_lg0[ii] = - 0.5; // umin
			d_ug0[ii] =   0.5; // umax
			}
		else // state
			{
			d_lg0[ii] = - 4.0; // xmin
			d_ug0[ii] =   4.0; // xmax
			}
		}

	int *idxb1; int_zeros(&idxb1, nb[1], 1);
	float *d_lb1; s_zeros(&d_lb1, nb[1], 1);
	float *d_ub1; s_zeros(&d_ub1, nb[1], 1);
	float *d_lg1; s_zeros(&d_lg1, ng[1], 1);
	float *d_ug1; s_zeros(&d_ug1, ng[1], 1);
	for(ii=0; ii<nb[1]; ii++)
		{
		if(ii<nu[1]) // input
			{
			d_lb1[ii] = - 0.5; // umin
			d_ub1[ii] =   0.5; // umax
			}
		else // state
			{
			d_lb1[ii] = - 4.0; // xmin
			d_ub1[ii] =   4.0; // xmax
			}
		idxb1[ii] = ii;
		}
	for(ii=0; ii<ng[1]; ii++)
		{
		if(ii<nu[1]-nb[1]) // input
			{
			d_lg1[ii] = - 0.5; // umin
			d_ug1[ii] =   0.5; // umax
			}
		else // state
			{
			d_lg1[ii] = - 4.0; // xmin
			d_ug1[ii] =   4.0; // xmax
			}
		}


	int *idxbN; int_zeros(&idxbN, nb[Nh], 1);
	float *d_lbN; s_zeros(&d_lbN, nb[Nh], 1);
	float *d_ubN; s_zeros(&d_ubN, nb[Nh], 1);
	float *d_lgN; s_zeros(&d_lgN, ng[Nh], 1);
	float *d_ugN; s_zeros(&d_ugN, ng[Nh], 1);
	for(ii=0; ii<nb[Nh]; ii++)
		{
		d_lbN[ii] = - 4.0; // xmin
		d_ubN[ii] =   4.0; // xmax
		idxbN[ii] = ii;
		}
	for(ii=0; ii<ng[Nh]; ii++)
		{
		d_lgN[ii] =   0.1; // dmin
		d_ugN[ii] =   0.1; // dmax
		}

	float *C0; s_zeros(&C0, ng[0], nx[0]);
	float *D0; s_zeros(&D0, ng[0], nu[0]);
	for(ii=0; ii<nu[0]-nb[0] & ii<ng[0]; ii++)
		D0[ii+(nb[0]+ii)*ng[0]] = 1.0;
	for(; ii<ng[0]; ii++)
		C0[ii+(nb[0]+ii-nu[0])*ng[0]] = 1.0;

	float *C1; s_zeros(&C1, ng[1], nx[1]);
	float *D1; s_zeros(&D1, ng[1], nu[1]);
	for(ii=0; ii<nu[1]-nb[1] & ii<ng[1]; ii++)
		D1[ii+(nb[1]+ii)*ng[1]] = 1.0;
	for(; ii<ng[1]; ii++)
		C1[ii+(nb[1]+ii-nu[1])*ng[1]] = 1.0;

	float *CN; s_zeros(&CN, ng[Nh], nx[Nh]);
	float *DN; s_zeros(&DN, ng[Nh], nu[Nh]);
	for(ii=0; ii<nu[Nh]-nb[Nh] & ii<ng[Nh]; ii++)
		DN[ii+(nb[Nh]+ii)*ng[Nh]] = 1.0;
	for(; ii<ng[Nh]; ii++)
		CN[ii+(nb[Nh]+ii-nu[Nh])*ng[Nh]] = 1.0;

#if 0
	// box constraints
	int_print_mat(1, nb[0], idxb0, 1);
	s_print_mat(1, nb[0], d_lb0, 1);
	s_print_mat(1, nb[0], d_ub0, 1);
	int_print_mat(1, nb[1], idxb1, 1);
	s_print_mat(1, nb[1], d_lb1, 1);
	s_print_mat(1, nb[1], d_ub1, 1);
	int_print_mat(1, nb[Nh], idxbN, 1);
	s_print_mat(1, nb[Nh], d_lbN, 1);
	s_print_mat(1, nb[Nh], d_ubN, 1);
	// general constraints
	s_print_mat(1, ng[0], d_lg0, 1);
	s_print_mat(1, ng[0], d_ug0, 1);
	s_print_mat(ng[0], nu[0], D0, ng[0]);
	s_print_mat(ng[0], nx[0], C0, ng[0]);
	s_print_mat(1, ng[1], d_lg1, 1);
	s_print_mat(1, ng[1], d_ug1, 1);
	s_print_mat(ng[1], nu[1], D1, ng[1]);
	s_print_mat(ng[1], nx[1], C1, ng[1]);
	s_print_mat(1, ng[Nh], d_lgN, 1);
	s_print_mat(1, ng[Nh], d_ugN, 1);
	s_print_mat(ng[Nh], nu[Nh], DN, ng[Nh]);
	s_print_mat(ng[Nh], nx[Nh], CN, ng[Nh]);
#endif

/************************************************
* create scenario tree
************************************************/	

	hpipm_size_t tree_memsize = memsize_sctree(md, Nr, Nh);
	printf("\ntree memsize = %d\n", tree_memsize);
	void *tree_memory = malloc(tree_memsize);

	struct sctree st;
	create_sctree(md, Nr, Nh, &st, tree_memory);

	int Nn = st.Nn;

#if 0
	int Nn = st.Nn;
	printf("\nscenario tree\n");
	for(ii=0; ii<Nn; ii++)
		{
		printf("\n");
		printf("idx = %d\n", (st.root+ii)->idx);
		printf("stage = %d\n", (st.root+ii)->stage);
		printf("real = %d\n", (st.root+ii)->real);
		printf("idxkid = %d\n", (st.root+ii)->idxkid);
		printf("dad = %d\n", (st.root+ii)->dad);
		printf("nkids = %d\n", (st.root+ii)->nkids);
		printf("kids =");
		for(jj=0; jj<(st.root+ii)->nkids; jj++)
			printf(" %d", (st.root+ii)->kids[jj]);
		printf("\n\n");
		}
#endif

/************************************************
* cast scenario tree into tree
************************************************/	

	struct tree ttree;
	cast_sctree2tree(&st, &ttree);

#if 0
	Nn = ttree.Nn;
	printf("\ntree\n");
	for(ii=0; ii<Nn; ii++)
		{
		printf("\n");
		printf("idx = %d\n", (ttree.root+ii)->idx);
		printf("stage = %d\n", (ttree.root+ii)->stage);
		printf("real = %d\n", (ttree.root+ii)->real);
		printf("idxkid = %d\n", (ttree.root+ii)->idxkid);
		printf("dad = %d\n", (ttree.root+ii)->dad);
		printf("nkids = %d\n", (ttree.root+ii)->nkids);
		printf("kids =");
		for(jj=0; jj<(ttree.root+ii)->nkids; jj++)
			printf(" %d", (ttree.root+ii)->kids[jj]);
		printf("\n\n");
		}
#endif

/************************************************
* tree ocp problem size
************************************************/	

	// node-wise size
	int nxt[Nn];
	int nut[Nn];
	int nbt[Nn];
	int ngt[Nn];

	for(ii=0; ii<Nn; ii++)
		{
		stage = (ttree.root+ii)->stage;
		nxt[ii] = nx[stage];
		nut[ii] = nu[stage];
		nbt[ii] = nb[stage];
		ngt[ii] = ng[stage];
		}
	
#if 0
	for(ii=0; ii<Nn; ii++)
		{
		printf("\n%d %d %d %d\n", nxt[ii], nut[ii], nbt[ii], ngt[ii]);
		}
#endif

/************************************************
* tree ocp data
************************************************/	

	// stage-wise data

	float *hA[Nh];
	float *hB[Nh];
	float *hb[Nh];
	float *hQ[Nh+1];
	float *hS[Nh+1];
	float *hR[Nh+1];
	float *hq[Nh+1];
	float *hr[Nh+1];
	float *hd_lb[Nh+1];
	float *hd_ub[Nh+1];
	float *hd_lg[Nh+1];
	float *hd_ug[Nh+1];
	float *hC[Nh+1];
	float *hD[Nh+1];
	int *hidxb[Nh+1];

	hA[0] = A;
	hB[0] = B;
	hb[0] = b0;
	hQ[0] = Q;
	hS[0] = S;
	hR[0] = R;
	hq[0] = q;
	hr[0] = r0;
	hidxb[0] = idxb0;
	hd_lb[0] = d_lb0;
	hd_ub[0] = d_ub0;
	hd_lg[0] = d_lg0;
	hd_ug[0] = d_ug0;
	hC[0] = C0;
	hD[0] = D0;
	for(ii=1; ii<Nh; ii++)
		{
		hA[ii] = A;
		hB[ii] = B;
		hb[ii] = b;
		hQ[ii] = Q;
		hS[ii] = S;
		hR[ii] = R;
		hq[ii] = q;
		hr[ii] = r;
		hidxb[ii] = idxb1;
		hd_lb[ii] = d_lb1;
		hd_ub[ii] = d_ub1;
		hd_lg[ii] = d_lg1;
		hd_ug[ii] = d_ug1;
		hC[ii] = C1;
		hD[ii] = D1;
		}
	hQ[Nh] = Q;
	hS[Nh] = S;
	hR[Nh] = R;
	hq[Nh] = q;
	hr[Nh] = r;
	hidxb[Nh] = idxbN;
	hd_lb[Nh] = d_lbN;
	hd_ub[Nh] = d_ubN;
	hd_lg[Nh] = d_lgN;
	hd_ug[Nh] = d_ugN;
	hC[Nh] = CN;
	hD[Nh] = DN;
	
	// node-wise data

	float *hAt[Nn-1];
	float *hBt[Nn-1];
	float *hbt[Nn-1];
	float *hQt[Nn];
	float *hSt[Nn];
	float *hRt[Nn];
	float *hqt[Nn];
	float *hrt[Nn];
	float *hd_lbt[Nn];
	float *hd_ubt[Nn];
	float *hd_lgt[Nn];
	float *hd_ugt[Nn];
	float *hCt[Nn];
	float *hDt[Nn];
	int *hidxbt[Nn];

	for(ii=0; ii<Nn-1; ii++)
		{
		stage = (ttree.root+ii+1)->stage-1;
		hAt[ii] = hA[stage];
		hBt[ii] = hB[stage];
		hbt[ii] = hb[stage];
		}

	for(ii=0; ii<Nn; ii++)
		{
		stage = (ttree.root+ii)->stage;
		hQt[ii] = hQ[stage];
		hRt[ii] = hR[stage];
		hSt[ii] = hS[stage];
		hqt[ii] = hq[stage];
		hrt[ii] = hr[stage];
		hd_lbt[ii] = hd_lb[stage];
		hd_ubt[ii] = hd_ub[stage];
		hd_lgt[ii] = hd_lg[stage];
		hd_ugt[ii] = hd_ug[stage];
		hidxbt[ii] = hidxb[stage];
		}

/************************************************
* create tree ocp qp
************************************************/	

	hpipm_size_t tree_ocp_qp_memsize = s_memsize_tree_ocp_qp(&ttree, nxt, nut, nbt, ngt);
	printf("\ntree ocp qp memsize = %d\n", tree_ocp_qp_memsize);
	void *tree_ocp_qp_memory = malloc(tree_ocp_qp_memsize);

	struct s_tree_ocp_qp qp;
	s_create_tree_ocp_qp(&ttree, nxt, nut, nbt, ngt, &qp, tree_ocp_qp_memory);
	s_cvt_colmaj_to_tree_ocp_qp(hAt, hBt, hbt, hQt, hSt, hRt, hqt, hrt, hidxbt, hd_lbt, hd_ubt, hCt, hDt, hd_lgt, hd_ugt, &qp);

#if 0
	struct blasfeo_smat *tmat;
	struct blasfeo_svec *tvec;
	for(ii=0; ii<Nn-1; ii++)
		{
		tmat = qp.BAbt+ii;
		blasfeo_print_smat(tmat->m, tmat->n, tmat, 0, 0);
		}
	for(ii=0; ii<Nn-1; ii++)
		{
		tvec = qp.b+ii;
		blasfeo_print_tran_svec(tvec->m, tvec, 0);
		}
	for(ii=0; ii<Nn; ii++)
		{
		tmat = qp.RSQrq+ii;
		blasfeo_print_smat(tmat->m, tmat->n, tmat, 0, 0);
		}
	for(ii=0; ii<Nn; ii++)
		{
		tvec = qp.rq+ii;
		blasfeo_print_tran_svec(tvec->m, tvec, 0);
		}
	for(ii=0; ii<Nn; ii++)
		{
		tvec = qp.d_lb+ii;
		blasfeo_print_tran_svec(tvec->m, tvec, 0);
		}
	for(ii=0; ii<Nn; ii++)
		{
		tvec = qp.d_ub+ii;
		blasfeo_print_tran_svec(tvec->m, tvec, 0);
		}
	for(ii=0; ii<Nn; ii++)
		{
		tvec = qp.d_lg+ii;
		blasfeo_print_tran_svec(tvec->m, tvec, 0);
		}
	for(ii=0; ii<Nn; ii++)
		{
		tvec = qp.d_ug+ii;
		blasfeo_print_tran_svec(tvec->m, tvec, 0);
		}
	for(ii=0; ii<Nn; ii++)
		{
		int_print_mat(1, qp.nb[ii], qp.idxb[ii], 1);
		}
#endif

/************************************************
* ocp qp sol
************************************************/	
	
	hpipm_size_t tree_ocp_qp_sol_size = s_memsize_tree_ocp_qp_sol(&ttree, nxt, nut, nbt, ngt);
	printf("\ntree ocp qp sol memsize = %d\n", tree_ocp_qp_sol_size);
	void *tree_ocp_qp_sol_memory = malloc(tree_ocp_qp_sol_size);

	struct s_tree_ocp_qp_sol qp_sol;
	s_create_tree_ocp_qp_sol(&ttree, nxt, nut, nbt, ngt, &qp_sol, tree_ocp_qp_sol_memory);

/************************************************
* ipm
************************************************/	

	struct s_ipm_hard_tree_ocp_qp_arg arg;
	arg.alpha_min = 1e-8;
	arg.mu_max = 1e-12;
	arg.iter_max = 20;
	arg.mu0 = 2.0;

	hpipm_size_t ipm_size = s_memsize_ipm_hard_tree_ocp_qp(&qp, &arg);
	printf("\nipm size = %d\n", ipm_size);
	void *ipm_memory = malloc(ipm_size);

	struct s_ipm_hard_tree_ocp_qp_workspace workspace;
	s_create_ipm_hard_tree_ocp_qp(&qp, &arg, &workspace, ipm_memory);

	int rep, nrep=100;

	struct timeval tv0, tv1;

	gettimeofday(&tv0, NULL); // start

	for(rep=0; rep<nrep; rep++)
		{
//		s_solve_ipm_hard_tree_ocp_qp(&qp, &qp_sol, &workspace);
		s_solve_ipm2_hard_tree_ocp_qp(&qp, &qp_sol, &workspace);
		}

	gettimeofday(&tv1, NULL); // stop

	float time_ocp_ipm = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);

/************************************************
* extract and print solution
************************************************/	

	float *u[Nn]; for(ii=0; ii<Nn; ii++) s_zeros(u+ii, nut[ii], 1);
	float *x[Nn]; for(ii=0; ii<Nn; ii++) s_zeros(x+ii, nxt[ii], 1);
	float *pi[Nn-1]; for(ii=0; ii<Nn-1; ii++) s_zeros(pi+ii, nxt[ii+1], 1);
	float *lam_lb[Nn]; for(ii=0; ii<Nn; ii++) s_zeros(lam_lb+ii, nbt[ii], 1);
	float *lam_ub[Nn]; for(ii=0; ii<Nn; ii++) s_zeros(lam_ub+ii, nbt[ii], 1);
	float *lam_lg[Nn]; for(ii=0; ii<Nn; ii++) s_zeros(lam_lg+ii, ngt[ii], 1);
	float *lam_ug[Nn]; for(ii=0; ii<Nn; ii++) s_zeros(lam_ug+ii, ngt[ii], 1);

	s_cvt_tree_ocp_qp_sol_to_colmaj(&qp, &qp_sol, u, x, pi, lam_lb, lam_ub, lam_lg, lam_ug);

#if 1
	printf("\nsolution\n\n");
	printf("\nu\n");
	for(ii=0; ii<Nn; ii++)
		s_print_mat(1, nut[ii], u[ii], 1);
	printf("\nx\n");
	for(ii=0; ii<Nn; ii++)
		s_print_mat(1, nxt[ii], x[ii], 1);
	printf("\npi\n");
	for(ii=0; ii<Nn-1; ii++)
		s_print_mat(1, nxt[ii+1], pi[ii], 1);
	printf("\nlam_lb\n");
	for(ii=0; ii<Nn; ii++)
		s_print_mat(1, nbt[ii], lam_lb[ii], 1);
	printf("\nlam_ub\n");
	for(ii=0; ii<Nn; ii++)
		s_print_mat(1, nbt[ii], lam_ub[ii], 1);
	printf("\nlam_lg\n");
	for(ii=0; ii<Nn; ii++)
		s_print_mat(1, ngt[ii], lam_lg[ii], 1);
	printf("\nlam_ug\n");
	for(ii=0; ii<Nn; ii++)
		s_print_mat(1, ngt[ii], lam_ug[ii], 1);

	printf("\nt_lb\n");
	for(ii=0; ii<Nn; ii++)
		s_print_mat(1, nbt[ii], (qp_sol.t_lb+ii)->pa, 1);
	printf("\nt_ub\n");
	for(ii=0; ii<Nn; ii++)
		s_print_mat(1, nbt[ii], (qp_sol.t_ub+ii)->pa, 1);
	printf("\nt_lg\n");
	for(ii=0; ii<Nn; ii++)
		s_print_mat(1, ngt[ii], (qp_sol.t_lg+ii)->pa, 1);
	printf("\nt_ug\n");
	for(ii=0; ii<Nn; ii++)
		s_print_mat(1, ngt[ii], (qp_sol.t_ug+ii)->pa, 1);

	printf("\nresiduals\n\n");
	printf("\nres_g\n");
	for(ii=0; ii<Nn; ii++)
		s_print_exp_mat(1, nut[ii]+nxt[ii], (workspace.res_g+ii)->pa, 1);
	printf("\nres_b\n");
	for(ii=0; ii<Nn-1; ii++)
		s_print_exp_mat(1, nxt[ii+1], (workspace.res_b+ii)->pa, 1);
	printf("\nres_m_lb\n");
	for(ii=0; ii<Nn; ii++)
		s_print_exp_mat(1, nbt[ii], (workspace.res_m_lb+ii)->pa, 1);
	printf("\nres_m_ub\n");
	for(ii=0; ii<Nn; ii++)
		s_print_exp_mat(1, nbt[ii], (workspace.res_m_ub+ii)->pa, 1);
	printf("\nres_m_lg\n");
	for(ii=0; ii<Nn; ii++)
		s_print_exp_mat(1, ngt[ii], (workspace.res_m_lg+ii)->pa, 1);
	printf("\nres_m_ug\n");
	for(ii=0; ii<Nn; ii++)
		s_print_exp_mat(1, ngt[ii], (workspace.res_m_ug+ii)->pa, 1);
	printf("\nres_d_lb\n");
	for(ii=0; ii<Nn; ii++)
		s_print_exp_mat(1, nbt[ii], (workspace.res_d_lb+ii)->pa, 1);
	printf("\nres_d_ub\n");
	for(ii=0; ii<Nn; ii++)
		s_print_exp_mat(1, nbt[ii], (workspace.res_d_ub+ii)->pa, 1);
	printf("\nres_d_lg\n");
	for(ii=0; ii<Nn; ii++)
		s_print_exp_mat(1, ngt[ii], (workspace.res_d_lg+ii)->pa, 1);
	printf("\nres_d_ug\n");
	for(ii=0; ii<Nn; ii++)
		s_print_exp_mat(1, ngt[ii], (workspace.res_d_ug+ii)->pa, 1);
	printf("\nres_mu\n");
	printf("\n%e\n\n", workspace.res_mu);
#endif

	printf("\nipm iter = %d\n", workspace.iter);
	printf("\nalpha_aff\tmu_aff\t\tsigma\t\talpha\t\tmu\n");
	s_print_exp_tran_mat(5, workspace.iter, workspace.stat, 5);

	printf("\nocp ipm time = %e [s]\n\n", time_ocp_ipm);

/************************************************
* free memory
************************************************/	

	free(A);
	free(B);
	free(b);
	free(x0);
	free(b0);
	free(Q);
	free(R);
	free(S);
	free(q);
	free(r);
	free(r0);
	int_free(idxb0);
	s_free(d_lb0);
	s_free(d_ub0);
	int_free(idxb1);
	s_free(d_lb1);
	s_free(d_ub1);
	int_free(idxbN);
	s_free(d_lbN);
	s_free(d_ubN);
	s_free(C0);
	s_free(D0);
	s_free(d_lg0);
	s_free(d_ug0);
	s_free(C1);
	s_free(D1);
	s_free(d_lg1);
	s_free(d_ug1);
	s_free(CN);
	s_free(DN);
	s_free(d_lgN);
	s_free(d_ugN);

	for(ii=0; ii<Nn-1; ii++)
		{
		s_free(u[ii]);
		s_free(x[ii]);
		s_free(pi[ii]);
		s_free(lam_lb[ii]);
		s_free(lam_ub[ii]);
		s_free(lam_lg[ii]);
		s_free(lam_ug[ii]);
		}
	s_free(u[ii]);
	s_free(x[ii]);
	s_free(lam_lb[ii]);
	s_free(lam_ub[ii]);
	s_free(lam_lg[ii]);
	s_free(lam_ug[ii]);

	free(tree_memory);
	free(tree_ocp_qp_memory);
	free(tree_ocp_qp_sol_memory);
	free(ipm_memory);

	return 0;

	}

