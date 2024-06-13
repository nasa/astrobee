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



void OCP_QCQP_DIM_PRINT(struct OCP_QCQP_DIM *qp_dim)
	{
	int ii;

	int N   = qp_dim->N;
	int *nx = qp_dim->nx;
	int *nu = qp_dim->nu;
	int *nbx = qp_dim->nbx;
	int *nbu = qp_dim->nbu;
	int *ng = qp_dim->ng;
	int *nq = qp_dim->nq;
	int *ns = qp_dim->ns;
	int *nsbx = qp_dim->nsbx;
	int *nsbu = qp_dim->nsbu;
	int *nsg = qp_dim->nsg;
	int *nsq = qp_dim->nsq;

	printf("N = %d\n\n", N);

	printf("nx =\n");
	for (ii = 0; ii <= N; ii++)
		printf("\t%d", nx[ii]);
	printf("\n\n");

	printf("nu =\n");
	for (ii = 0; ii <= N; ii++)
		printf("\t%d", nu[ii]);
	printf("\n\n");

	printf("nbx =\n");
	for (ii = 0; ii <= N; ii++)
		printf("\t%d", nbx[ii]);
	printf("\n\n");

	printf("nbu =\n");
	for (ii = 0; ii <= N; ii++)
		printf("\t%d", nbu[ii]);
	printf("\n\n");

	printf("ng =\n");
	for (ii = 0; ii <= N; ii++)
		printf("\t%d", ng[ii]);
	printf("\n\n");

	printf("nq =\n");
	for (ii = 0; ii <= N; ii++)
		printf("\t%d", nq[ii]);
	printf("\n\n");

	printf("ns =\n");
	for (ii = 0; ii <= N; ii++)
		printf("\t%d", ns[ii]);
	printf("\n\n");

	printf("nsbx =\n");
	for (ii = 0; ii <= N; ii++)
		printf("\t%d", nsbx[ii]);
	printf("\n\n");

	printf("nsbu =\n");
	for (ii = 0; ii <= N; ii++)
		printf("\t%d", nsbu[ii]);
	printf("\n\n");

	printf("nsg =\n");
	for (ii = 0; ii <= N; ii++)
		printf("\t%d", nsg[ii]);
	printf("\n\n");

	printf("nsq =\n");
	for (ii = 0; ii <= N; ii++)
		printf("\t%d", nsq[ii]);
	printf("\n\n");

	return;
	}



void OCP_QCQP_DIM_CODEGEN(char *file_name, char *mode, struct OCP_QCQP_DIM *qp_dim)
	{
	int ii;

	FILE *file = fopen(file_name, mode);

	int N   = qp_dim->N;
	int *nx = qp_dim->nx;
	int *nu = qp_dim->nu;
	int *nbx = qp_dim->nbx;
	int *nbu = qp_dim->nbu;
	int *ng = qp_dim->ng;
	int *nq = qp_dim->nq;
	int *ns = qp_dim->ns;
	int *nsbx = qp_dim->nsbx;
	int *nsbu = qp_dim->nsbu;
	int *nsg = qp_dim->nsg;
	int *nsq = qp_dim->nsq;

	fprintf(file, "/***************\n* dim\n***************/\n");

	// N
	fprintf(file, "/* N */\n");
	fprintf(file, "int N = %d;\n", N);
	// nx
	fprintf(file, "/* nx */\n");
	fprintf(file, "static int nnx[] = {");
	for(ii=0; ii<=N; ii++)
		fprintf(file, "%d, ", nx[ii]);
	fprintf(file, "};\n");
	fprintf(file, "int *nx = nnx;\n");
	// nu
	fprintf(file, "/* nu */\n");
	fprintf(file, "static int nnu[] = {");
	for(ii=0; ii<=N; ii++)
		fprintf(file, "%d, ", nu[ii]);
	fprintf(file, "};\n");
	fprintf(file, "int *nu = nnu;\n");
	// nbx
	fprintf(file, "/* nbx */\n");
	fprintf(file, "static int nnbx[] = {");
	for(ii=0; ii<=N; ii++)
		fprintf(file, "%d, ", nbx[ii]);
	fprintf(file, "};\n");
	fprintf(file, "int *nbx = nnbx;\n");
	// nbu
	fprintf(file, "/* nbu */\n");
	fprintf(file, "static int nnbu[] = {");
	for(ii=0; ii<=N; ii++)
		fprintf(file, "%d, ", nbu[ii]);
	fprintf(file, "};\n");
	fprintf(file, "int *nbu = nnbu;\n");
	// ng
	fprintf(file, "/* ng */\n");
	fprintf(file, "static int nng[] = {");
	for(ii=0; ii<=N; ii++)
		fprintf(file, "%d, ", ng[ii]);
	fprintf(file, "};\n");
	fprintf(file, "int *ng = nng;\n");
	// nq
	fprintf(file, "/* nq */\n");
	fprintf(file, "static int nnq[] = {");
	for(ii=0; ii<=N; ii++)
		fprintf(file, "%d, ", nq[ii]);
	fprintf(file, "};\n");
	fprintf(file, "int *nq = nnq;\n");
	// ns
	fprintf(file, "/* ns */\n");
	fprintf(file, "static int nns[] = {");
	for(ii=0; ii<=N; ii++)
		fprintf(file, "%d, ", ns[ii]);
	fprintf(file, "};\n");
	fprintf(file, "int *ns = nns;\n");
	// nsbx
	fprintf(file, "/* nsbx */\n");
	fprintf(file, "static int nnsbx[] = {");
	for(ii=0; ii<=N; ii++)
		fprintf(file, "%d, ", nsbx[ii]);
	fprintf(file, "};\n");
	fprintf(file, "int *nsbx = nnsbx;\n");
	// nsbu
	fprintf(file, "/* nsbu */\n");
	fprintf(file, "static int nnsbu[] = {");
	for(ii=0; ii<=N; ii++)
		fprintf(file, "%d, ", nsbu[ii]);
	fprintf(file, "};\n");
	fprintf(file, "int *nsbu = nnsbu;\n");
	// nsg
	fprintf(file, "/* nsg */\n");
	fprintf(file, "static int nnsg[] = {");
	for(ii=0; ii<=N; ii++)
		fprintf(file, "%d, ", nsg[ii]);
	fprintf(file, "};\n");
	fprintf(file, "int *nsg = nnsg;\n");
	// nsq
	fprintf(file, "/* nsq */\n");
	fprintf(file, "static int nnsq[] = {");
	for(ii=0; ii<=N; ii++)
		fprintf(file, "%d, ", nsq[ii]);
	fprintf(file, "};\n");
	fprintf(file, "int *nsq = nnsq;\n");

	fclose(file);

	return;
	}



void OCP_QCQP_PRINT(struct OCP_QCQP_DIM *dim, struct OCP_QCQP *qp)
	{
	int ii, jj;

	int N   = dim->N;
	int *nx = dim->nx;
	int *nu = dim->nu;
	int *nb = dim->nb;
	int *ng = dim->ng;
	int *nq = dim->nq;
	int *ns = dim->ns;

	printf("BAt =\n");
	for (ii = 0; ii < N; ii++)
		BLASFEO_PRINT_MAT(nu[ii]+nx[ii], nx[ii+1], qp->BAbt+ii, 0, 0);

	printf("b =\n");
	for (ii = 0; ii < N; ii++)
		BLASFEO_PRINT_TRAN_VEC(nx[ii+1], qp->b+ii, 0);

	printf("RSQ =\n");
	for (ii = 0; ii <= N; ii++)
		BLASFEO_PRINT_MAT(nu[ii]+nx[ii], nu[ii]+nx[ii], qp->RSQrq+ii, 0, 0);

	printf("Z =\n");
	for (ii = 0; ii <= N; ii++)
		BLASFEO_PRINT_TRAN_VEC(2*ns[ii], qp->Z+ii, 0);

	printf("rqz =\n");
	for (ii = 0; ii <= N; ii++)
		BLASFEO_PRINT_TRAN_VEC(nu[ii]+nx[ii]+2*ns[ii], qp->rqz+ii, 0);

	printf("idxb = \n");
	for (ii = 0; ii <= N; ii++)
		int_print_mat(1, nb[ii], qp->idxb[ii], 1);

	printf("d =\n");
	for (ii = 0; ii <= N; ii++)
		BLASFEO_PRINT_TRAN_VEC(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], qp->d+ii, 0);

	printf("d_mask =\n");
	for (ii = 0; ii <= N; ii++)
		BLASFEO_PRINT_TRAN_VEC(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], qp->d_mask+ii, 0);

	printf("DCt =\n");
	for (ii = 0; ii <= N; ii++)
		BLASFEO_PRINT_MAT(nu[ii]+nx[ii], ng[ii], qp->DCt+ii, 0, 0);

	printf("Hq =\n");
	for (ii = 0; ii <= N; ii++)
		if(nq[ii]==0)
			printf("\n\n");
		else
			for(jj=0; jj<nq[ii]; jj++)
				BLASFEO_PRINT_MAT(nu[ii]+nx[ii], nu[ii]+nx[ii], &qp->Hq[ii][jj], 0, 0);

	printf("Hq_nzero = \n");
	for (ii = 0; ii <= N; ii++)
		int_print_mat(1, nq[ii], qp->Hq_nzero[ii], 1);

	printf("gq =\n");
	for (ii = 0; ii <= N; ii++)
		if(nq[ii]==0)
			printf("\n\n");
		else
			for(jj=0; jj<nq[ii]; jj++)
				BLASFEO_PRINT_TRAN_MAT(nu[ii]+nx[ii], 1, qp->DCt+ii, 0, ng[ii]+jj);

	printf("idxs_rev = \n");
	for (ii = 0; ii <= N; ii++)
		int_print_mat(1, nb[ii]+ng[ii], qp->idxs_rev[ii], 1);

	printf("m =\n");
	for (ii = 0; ii <= N; ii++)
		BLASFEO_PRINT_TRAN_VEC(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], qp->m+ii, 0);

	return;
	}



void OCP_QCQP_CODEGEN(char *file_name, char *mode, struct OCP_QCQP_DIM *dim, struct OCP_QCQP *qp)
	{
	int nn, ii, jj, kk;

	FILE *file = fopen(file_name, mode);

	int N   = dim->N;
	int *nx = dim->nx;
	int *nu = dim->nu;
	int *nb = dim->nb;
	int *ng = dim->ng;
	int *nq = dim->nq;
	int *ns = dim->ns;

	fprintf(file, "/***************\n* qp\n***************/\n");

	// A
	fprintf(file, "/* A */\n");
	for(nn=0; nn<N; nn++)
		{
#ifdef DOUBLE_PRECISION
		fprintf(file, "static double A%d[] = {", nn);
#else
		fprintf(file, "static float A%d[] = {", nn);
#endif
		for(ii=0; ii<nx[nn]; ii++)
			{
			for(jj=0; jj<nx[nn+1]; jj++)
				{
#ifdef DOUBLE_PRECISION
				fprintf(file, "%18.15e, ", BLASFEO_DMATEL(qp->BAbt+nn, nu[nn]+ii, jj));
#else
				fprintf(file, "%18.15e, ", BLASFEO_SMATEL(qp->BAbt+nn, nu[nn]+ii, jj));
#endif
				}
			}
		fprintf(file, "};\n");
		}
#ifdef DOUBLE_PRECISION
	fprintf(file, "static double *AA[] = {");
#else
	fprintf(file, "static float *AA[] = {");
#endif
	for(nn=0; nn<N; nn++)
		fprintf(file, "A%d, ", nn);
	fprintf(file, "};\n");
#ifdef DOUBLE_PRECISION
	fprintf(file, "double **hA = AA;\n");
#else
	fprintf(file, "float **hA = AA;\n");
#endif

	// B
	fprintf(file, "/* B */\n");
	for(nn=0; nn<N; nn++)
		{
#ifdef DOUBLE_PRECISION
		fprintf(file, "static double B%d[] = {", nn);
#else
		fprintf(file, "static float B%d[] = {", nn);
#endif
		for(ii=0; ii<nu[nn]; ii++)
			{
			for(jj=0; jj<nx[nn+1]; jj++)
				{
#ifdef DOUBLE_PRECISION
				fprintf(file, "%18.15e, ", BLASFEO_DMATEL(qp->BAbt+nn, ii, jj));
#else
				fprintf(file, "%18.15e, ", BLASFEO_SMATEL(qp->BAbt+nn, ii, jj));
#endif
				}
			}
		fprintf(file, "};\n");
		}
#ifdef DOUBLE_PRECISION
	fprintf(file, "static double *BB[] = {");
#else
	fprintf(file, "static float *BB[] = {");
#endif
	for(nn=0; nn<N; nn++)
		fprintf(file, "B%d, ", nn);
	fprintf(file, "};\n");
#ifdef DOUBLE_PRECISION
	fprintf(file, "double **hB = BB;\n");
#else
	fprintf(file, "float **hB = BB;\n");
#endif

	// b
	fprintf(file, "/* b */\n");
	for(nn=0; nn<N; nn++)
		{
#ifdef DOUBLE_PRECISION
		fprintf(file, "static double b%d[] = {", nn);
#else
		fprintf(file, "static float b%d[] = {", nn);
#endif
		for(jj=0; jj<nx[nn+1]; jj++)
			{
#ifdef DOUBLE_PRECISION
			fprintf(file, "%18.15e, ", BLASFEO_DVECEL(qp->b+nn, jj));
#else
			fprintf(file, "%18.15e, ", BLASFEO_SVECEL(qp->b+nn, jj));
#endif
			}
		fprintf(file, "};\n");
		}
#ifdef DOUBLE_PRECISION
	fprintf(file, "static double *bb[] = {");
#else
	fprintf(file, "static float *bb[] = {");
#endif
	for(nn=0; nn<N; nn++)
		fprintf(file, "b%d, ", nn);
	fprintf(file, "};\n");
#ifdef DOUBLE_PRECISION
	fprintf(file, "double **hb = bb;\n");
#else
	fprintf(file, "float **hb = bb;\n");
#endif

	// Q
	fprintf(file, "/* Q */\n");
	for(nn=0; nn<=N; nn++)
		{
#ifdef DOUBLE_PRECISION
		fprintf(file, "static double Q%d[] = {", nn);
#else
		fprintf(file, "static float Q%d[] = {", nn);
#endif
		for(jj=0; jj<nx[nn]; jj++)
			{
			for(ii=0; ii<nx[nn]; ii++)
				{
#ifdef DOUBLE_PRECISION
				fprintf(file, "%18.15e, ", BLASFEO_DMATEL(qp->RSQrq+nn, nu[nn]+ii, nu[nn]+jj));
#else
				fprintf(file, "%18.15e, ", BLASFEO_SMATEL(qp->RSQrq+nn, nu[nn]+ii, nu[nn]+jj));
#endif
				}
			}
		fprintf(file, "};\n");
		}
#ifdef DOUBLE_PRECISION
	fprintf(file, "static double *QQ[] = {");
#else
	fprintf(file, "static float *QQ[] = {");
#endif
	for(nn=0; nn<=N; nn++)
		fprintf(file, "Q%d, ", nn);
	fprintf(file, "};\n");
#ifdef DOUBLE_PRECISION
	fprintf(file, "double **hQ = QQ;\n");
#else
	fprintf(file, "float **hQ = QQ;\n");
#endif

	// S
	fprintf(file, "/* S */\n");
	for(nn=0; nn<=N; nn++)
		{
#ifdef DOUBLE_PRECISION
		fprintf(file, "static double S%d[] = {", nn);
#else
		fprintf(file, "static float S%d[] = {", nn);
#endif
		for(ii=0; ii<nx[nn]; ii++)
			{
			for(jj=0; jj<nu[nn]; jj++)
				{
#ifdef DOUBLE_PRECISION
				fprintf(file, "%18.15e, ", BLASFEO_DMATEL(qp->RSQrq+nn, nu[nn]+ii, jj));
#else
				fprintf(file, "%18.15e, ", BLASFEO_SMATEL(qp->RSQrq+nn, nu[nn]+ii, jj));
#endif
				}
			}
		fprintf(file, "};\n");
		}
#ifdef DOUBLE_PRECISION
	fprintf(file, "static double *SS[] = {");
#else
	fprintf(file, "static float *SS[] = {");
#endif
	for(nn=0; nn<=N; nn++)
		fprintf(file, "S%d, ", nn);
	fprintf(file, "};\n");
#ifdef DOUBLE_PRECISION
	fprintf(file, "double **hS = SS;\n");
#else
	fprintf(file, "float **hS = SS;\n");
#endif

	// R
	fprintf(file, "/* R */\n");
	for(nn=0; nn<=N; nn++)
		{
#ifdef DOUBLE_PRECISION
		fprintf(file, "static double R%d[] = {", nn);
#else
		fprintf(file, "static float R%d[] = {", nn);
#endif
		for(jj=0; jj<nu[nn]; jj++)
			{
			for(ii=0; ii<nu[nn]; ii++)
				{
#ifdef DOUBLE_PRECISION
				fprintf(file, "%18.15e, ", BLASFEO_DMATEL(qp->RSQrq+nn, ii, jj));
#else
				fprintf(file, "%18.15e, ", BLASFEO_SMATEL(qp->RSQrq+nn, ii, jj));
#endif
				}
			}
		fprintf(file, "};\n");
		}
#ifdef DOUBLE_PRECISION
	fprintf(file, "static double *RR[] = {");
#else
	fprintf(file, "static float *RR[] = {");
#endif
	for(nn=0; nn<=N; nn++)
		fprintf(file, "R%d, ", nn);
	fprintf(file, "};\n");
#ifdef DOUBLE_PRECISION
	fprintf(file, "double **hR = RR;\n");
#else
	fprintf(file, "float **hR = RR;\n");
#endif

	// r
	fprintf(file, "/* r */\n");
	for(nn=0; nn<=N; nn++)
		{
#ifdef DOUBLE_PRECISION
		fprintf(file, "static double r%d[] = {", nn);
#else
		fprintf(file, "static float r%d[] = {", nn);
#endif
		for(jj=0; jj<nu[nn]; jj++)
			{
#ifdef DOUBLE_PRECISION
			fprintf(file, "%18.15e, ", BLASFEO_DVECEL(qp->rqz+nn, jj));
#else
			fprintf(file, "%18.15e, ", BLASFEO_SVECEL(qp->rqz+nn, jj));
#endif
			}
		fprintf(file, "};\n");
		}
#ifdef DOUBLE_PRECISION
	fprintf(file, "static double *rr[] = {");
#else
	fprintf(file, "static float *rr[] = {");
#endif
	for(nn=0; nn<=N; nn++)
		fprintf(file, "r%d, ", nn);
	fprintf(file, "};\n");
#ifdef DOUBLE_PRECISION
	fprintf(file, "double **hr = rr;\n");
#else
	fprintf(file, "float **hr = rr;\n");
#endif

	// q
	fprintf(file, "/* q */\n");
	for(nn=0; nn<=N; nn++)
		{
#ifdef DOUBLE_PRECISION
		fprintf(file, "static double q%d[] = {", nn);
#else
		fprintf(file, "static float q%d[] = {", nn);
#endif
		for(jj=0; jj<nx[nn]; jj++)
			{
#ifdef DOUBLE_PRECISION
			fprintf(file, "%18.15e, ", BLASFEO_DVECEL(qp->rqz+nn, nu[nn]+jj));
#else
			fprintf(file, "%18.15e, ", BLASFEO_SVECEL(qp->rqz+nn, nu[nn]+jj));
#endif
			}
		fprintf(file, "};\n");
		}
#ifdef DOUBLE_PRECISION
	fprintf(file, "static double *qq[] = {");
#else
	fprintf(file, "static float *qq[] = {");
#endif
	for(nn=0; nn<=N; nn++)
		fprintf(file, "q%d, ", nn);
	fprintf(file, "};\n");
#ifdef DOUBLE_PRECISION
	fprintf(file, "double **hq = qq;\n");
#else
	fprintf(file, "float **hq = qq;\n");
#endif

	// idxbu
	fprintf(file, "/* idxbu */\n");
	for(nn=0; nn<=N; nn++)
		{
		fprintf(file, "static int idxbu%d[] = {", nn);
		for(jj=0; jj<nb[nn]; jj++)
			{
			if(qp->idxb[nn][jj]<nu[nn])
				{
				fprintf(file, "%d, ", qp->idxb[nn][jj]);
				}
			}
		fprintf(file, "};\n");
		}
	fprintf(file, "static int *iidxbu[] = {");
	for(nn=0; nn<=N; nn++)
		fprintf(file, "idxbu%d, ", nn);
	fprintf(file, "};\n");
	fprintf(file, "int **hidxbu = iidxbu;\n");

	// lbu
	fprintf(file, "/* lbu */\n");
	for(nn=0; nn<=N; nn++)
		{
#ifdef DOUBLE_PRECISION
		fprintf(file, "static double lbu%d[] = {", nn);
#else
		fprintf(file, "static float lbu%d[] = {", nn);
#endif
		for(jj=0; jj<nb[nn]; jj++)
			{
			if(qp->idxb[nn][jj]<nu[nn])
				{
				fprintf(file, "%18.15e, ", BLASFEO_DVECEL(qp->d+nn, jj));
				}
			}
		fprintf(file, "};\n");
		}
#ifdef DOUBLE_PRECISION
	fprintf(file, "static double *llbu[] = {");
#else
	fprintf(file, "static float *llbu[] = {");
#endif
	for(nn=0; nn<=N; nn++)
		fprintf(file, "lbu%d, ", nn);
	fprintf(file, "};\n");
#ifdef DOUBLE_PRECISION
	fprintf(file, "double **hlbu = llbu;\n");
#else
	fprintf(file, "float **hlbu = llbu;\n");
#endif

	// lbu_mask
	fprintf(file, "/* lbu_mask */\n");
	for(nn=0; nn<=N; nn++)
		{
#ifdef DOUBLE_PRECISION
		fprintf(file, "static double lbu_mask%d[] = {", nn);
#else
		fprintf(file, "static float lbu_mask%d[] = {", nn);
#endif
		for(jj=0; jj<nb[nn]; jj++)
			{
			if(qp->idxb[nn][jj]<nu[nn])
				{
				fprintf(file, "%18.15e, ", BLASFEO_DVECEL(qp->d_mask+nn, jj));
				}
			}
		fprintf(file, "};\n");
		}
#ifdef DOUBLE_PRECISION
	fprintf(file, "static double *llbu_mask[] = {");
#else
	fprintf(file, "static float *llbu_mask[] = {");
#endif
	for(nn=0; nn<=N; nn++)
		fprintf(file, "lbu_mask%d, ", nn);
	fprintf(file, "};\n");
#ifdef DOUBLE_PRECISION
	fprintf(file, "double **hlbu_mask = llbu_mask;\n");
#else
	fprintf(file, "float **hlbu_mask = llbu_mask;\n");
#endif

	// ubu
	fprintf(file, "/* ubu */\n");
	for(nn=0; nn<=N; nn++)
		{
#ifdef DOUBLE_PRECISION
		fprintf(file, "static double ubu%d[] = {", nn);
#else
		fprintf(file, "static float ubu%d[] = {", nn);
#endif
		for(jj=0; jj<nb[nn]; jj++)
			{
			if(qp->idxb[nn][jj]<nu[nn])
				{
				fprintf(file, "%18.15e, ", -BLASFEO_DVECEL(qp->d+nn, nb[nn]+ng[nn]+nq[nn]+jj));
				}
			}
		fprintf(file, "};\n");
		}
#ifdef DOUBLE_PRECISION
	fprintf(file, "static double *uubu[] = {");
#else
	fprintf(file, "static float *uubu[] = {");
#endif
	for(nn=0; nn<=N; nn++)
		fprintf(file, "ubu%d, ", nn);
	fprintf(file, "};\n");
#ifdef DOUBLE_PRECISION
	fprintf(file, "double **hubu = uubu;\n");
#else
	fprintf(file, "float **hubu = uubu;\n");
#endif

	// ubu_mask
	fprintf(file, "/* ubu_mask */\n");
	for(nn=0; nn<=N; nn++)
		{
#ifdef DOUBLE_PRECISION
		fprintf(file, "static double ubu_mask%d[] = {", nn);
#else
		fprintf(file, "static float ubu_mask%d[] = {", nn);
#endif
		for(jj=0; jj<nb[nn]; jj++)
			{
			if(qp->idxb[nn][jj]<nu[nn])
				{
				fprintf(file, "%18.15e, ", BLASFEO_DVECEL(qp->d_mask+nn, nb[nn]+ng[nn]+nq[nn]+jj));
				}
			}
		fprintf(file, "};\n");
		}
#ifdef DOUBLE_PRECISION
	fprintf(file, "static double *uubu_mask[] = {");
#else
	fprintf(file, "static float *uubu_mask[] = {");
#endif
	for(nn=0; nn<=N; nn++)
		fprintf(file, "ubu_mask%d, ", nn);
	fprintf(file, "};\n");
#ifdef DOUBLE_PRECISION
	fprintf(file, "double **hubu_mask = uubu_mask;\n");
#else
	fprintf(file, "float **hubu_mask = uubu_mask;\n");
#endif

	// idxbx
	fprintf(file, "/* idxbx */\n");
	for(nn=0; nn<=N; nn++)
		{
		fprintf(file, "static int idxbx%d[] = {", nn);
		for(jj=0; jj<nb[nn]; jj++)
			{
			if(qp->idxb[nn][jj]>=nu[nn])
				{
				fprintf(file, "%d, ", qp->idxb[nn][jj]-nu[nn]);
				}
			}
		fprintf(file, "};\n");
		}
	fprintf(file, "static int *iidxbx[] = {");
	for(nn=0; nn<=N; nn++)
		fprintf(file, "idxbx%d, ", nn);
	fprintf(file, "};\n");
	fprintf(file, "int **hidxbx = iidxbx;\n");

	// lbx
	fprintf(file, "/* lbx */\n");
	for(nn=0; nn<=N; nn++)
		{
#ifdef DOUBLE_PRECISION
		fprintf(file, "static double lbx%d[] = {", nn);
#else
		fprintf(file, "static float lbx%d[] = {", nn);
#endif
		for(jj=0; jj<nb[nn]; jj++)
			{
			if(qp->idxb[nn][jj]>=nu[nn])
				{
				fprintf(file, "%18.15e, ", BLASFEO_DVECEL(qp->d+nn, jj));
				}
			}
		fprintf(file, "};\n");
		}
#ifdef DOUBLE_PRECISION
	fprintf(file, "static double *llbx[] = {");
#else
	fprintf(file, "static float *llbx[] = {");
#endif
	for(nn=0; nn<=N; nn++)
		fprintf(file, "lbx%d, ", nn);
	fprintf(file, "};\n");
#ifdef DOUBLE_PRECISION
	fprintf(file, "double **hlbx = llbx;\n");
#else
	fprintf(file, "float **hlbx = llbx;\n");
#endif

	// lbx_mask
	fprintf(file, "/* lbx_mask */\n");
	for(nn=0; nn<=N; nn++)
		{
#ifdef DOUBLE_PRECISION
		fprintf(file, "static double lbx_mask%d[] = {", nn);
#else
		fprintf(file, "static float lbx_mask%d[] = {", nn);
#endif
		for(jj=0; jj<nb[nn]; jj++)
			{
			if(qp->idxb[nn][jj]>=nu[nn])
				{
				fprintf(file, "%18.15e, ", BLASFEO_DVECEL(qp->d_mask+nn, jj));
				}
			}
		fprintf(file, "};\n");
		}
#ifdef DOUBLE_PRECISION
	fprintf(file, "static double *llbx_mask[] = {");
#else
	fprintf(file, "static float *llbx_mask[] = {");
#endif
	for(nn=0; nn<=N; nn++)
		fprintf(file, "lbx_mask%d, ", nn);
	fprintf(file, "};\n");
#ifdef DOUBLE_PRECISION
	fprintf(file, "double **hlbx_mask = llbx_mask;\n");
#else
	fprintf(file, "float **hlbx_mask = llbx_mask;\n");
#endif

	// ubx
	fprintf(file, "/* ubx */\n");
	for(nn=0; nn<=N; nn++)
		{
#ifdef DOUBLE_PRECISION
		fprintf(file, "static double ubx%d[] = {", nn);
#else
		fprintf(file, "static float ubx%d[] = {", nn);
#endif
		for(jj=0; jj<nb[nn]; jj++)
			{
			if(qp->idxb[nn][jj]>=nu[nn])
				{
				fprintf(file, "%18.15e, ", -BLASFEO_DVECEL(qp->d+nn, nb[nn]+ng[nn]+nq[nn]+jj));
				}
			}
		fprintf(file, "};\n");
		}
#ifdef DOUBLE_PRECISION
	fprintf(file, "static double *uubx[] = {");
#else
	fprintf(file, "static float *uubx[] = {");
#endif
	for(nn=0; nn<=N; nn++)
		fprintf(file, "ubx%d, ", nn);
	fprintf(file, "};\n");
#ifdef DOUBLE_PRECISION
	fprintf(file, "double **hubx = uubx;\n");
#else
	fprintf(file, "float **hubx = uubx;\n");
#endif

	// ubx_mask
	fprintf(file, "/* ubx_mask */\n");
	for(nn=0; nn<=N; nn++)
		{
#ifdef DOUBLE_PRECISION
		fprintf(file, "static double ubx_mask%d[] = {", nn);
#else
		fprintf(file, "static float ubx_mask%d[] = {", nn);
#endif
		for(jj=0; jj<nb[nn]; jj++)
			{
			if(qp->idxb[nn][jj]>=nu[nn])
				{
				fprintf(file, "%18.15e, ", BLASFEO_DVECEL(qp->d_mask+nn, nb[nn]+ng[nn]+nq[nn]+jj));
				}
			}
		fprintf(file, "};\n");
		}
#ifdef DOUBLE_PRECISION
	fprintf(file, "static double *uubx_mask[] = {");
#else
	fprintf(file, "static float *uubx_mask[] = {");
#endif
	for(nn=0; nn<=N; nn++)
		fprintf(file, "ubx_mask%d, ", nn);
	fprintf(file, "};\n");
#ifdef DOUBLE_PRECISION
	fprintf(file, "double **hubx_mask = uubx_mask;\n");
#else
	fprintf(file, "float **hubx_mask = uubx_mask;\n");
#endif

	// C
	fprintf(file, "/* C */\n");
	for(nn=0; nn<=N; nn++)
		{
#ifdef DOUBLE_PRECISION
		fprintf(file, "static double C%d[] = {", nn);
#else
		fprintf(file, "static float C%d[] = {", nn);
#endif
		for(ii=0; ii<nx[nn]; ii++)
			{
			for(jj=0; jj<ng[nn]; jj++)
				{
#ifdef DOUBLE_PRECISION
				fprintf(file, "%18.15e, ", BLASFEO_DMATEL(qp->DCt+nn, nu[nn]+ii, jj));
#else
				fprintf(file, "%18.15e, ", BLASFEO_SMATEL(qp->DCt+nn, nu[nn]+ii, jj));
#endif
				}
			}
		fprintf(file, "};\n");
		}
#ifdef DOUBLE_PRECISION
	fprintf(file, "static double *CC[] = {");
#else
	fprintf(file, "static float *CC[] = {");
#endif
	for(nn=0; nn<=N; nn++)
		fprintf(file, "C%d, ", nn);
	fprintf(file, "};\n");
#ifdef DOUBLE_PRECISION
	fprintf(file, "double **hC = CC;\n");
#else
	fprintf(file, "float **hC = CC;\n");
#endif

	// D
	fprintf(file, "/* D */\n");
	for(nn=0; nn<=N; nn++)
		{
#ifdef DOUBLE_PRECISION
		fprintf(file, "static double D%d[] = {", nn);
#else
		fprintf(file, "static float D%d[] = {", nn);
#endif
		for(ii=0; ii<nu[nn]; ii++)
			{
			for(jj=0; jj<ng[nn]; jj++)
				{
#ifdef DOUBLE_PRECISION
				fprintf(file, "%18.15e, ", BLASFEO_DMATEL(qp->DCt+nn, ii, jj));
#else
				fprintf(file, "%18.15e, ", BLASFEO_SMATEL(qp->DCt+nn, ii, jj));
#endif
				}
			}
		fprintf(file, "};\n");
		}
#ifdef DOUBLE_PRECISION
	fprintf(file, "static double *DD[] = {");
#else
	fprintf(file, "static float *DD[] = {");
#endif
	for(nn=0; nn<=N; nn++)
		fprintf(file, "D%d, ", nn);
	fprintf(file, "};\n");
#ifdef DOUBLE_PRECISION
	fprintf(file, "double **hD = DD;\n");
#else
	fprintf(file, "float **hD = DD;\n");
#endif

	// lg
	fprintf(file, "/* lg */\n");
	for(nn=0; nn<=N; nn++)
		{
#ifdef DOUBLE_PRECISION
		fprintf(file, "static double lg%d[] = {", nn);
#else
		fprintf(file, "static float lg%d[] = {", nn);
#endif
		for(jj=0; jj<ng[nn]; jj++)
			{
			fprintf(file, "%18.15e, ", BLASFEO_DVECEL(qp->d+nn, nb[nn]+jj));
			}
		fprintf(file, "};\n");
		}
#ifdef DOUBLE_PRECISION
	fprintf(file, "static double *llg[] = {");
#else
	fprintf(file, "static float *llg[] = {");
#endif
	for(nn=0; nn<=N; nn++)
		fprintf(file, "lg%d, ", nn);
	fprintf(file, "};\n");
#ifdef DOUBLE_PRECISION
	fprintf(file, "double **hlg = llg;\n");
#else
	fprintf(file, "float **hlg = llg;\n");
#endif

	// lg_mask
	fprintf(file, "/* lg_mask */\n");
	for(nn=0; nn<=N; nn++)
		{
#ifdef DOUBLE_PRECISION
		fprintf(file, "static double lg_mask%d[] = {", nn);
#else
		fprintf(file, "static float lg_mask%d[] = {", nn);
#endif
		for(jj=0; jj<ng[nn]; jj++)
			{
			fprintf(file, "%18.15e, ", BLASFEO_DVECEL(qp->d_mask+nn, nb[nn]+jj));
			}
		fprintf(file, "};\n");
		}
#ifdef DOUBLE_PRECISION
	fprintf(file, "static double *llg_mask[] = {");
#else
	fprintf(file, "static float *llg_mask[] = {");
#endif
	for(nn=0; nn<=N; nn++)
		fprintf(file, "lg_mask%d, ", nn);
	fprintf(file, "};\n");
#ifdef DOUBLE_PRECISION
	fprintf(file, "double **hlg_mask = llg_mask;\n");
#else
	fprintf(file, "float **hlg_mask = llg_mask;\n");
#endif

	// ug
	fprintf(file, "/* ug */\n");
	for(nn=0; nn<=N; nn++)
		{
#ifdef DOUBLE_PRECISION
		fprintf(file, "static double ug%d[] = {", nn);
#else
		fprintf(file, "static float ug%d[] = {", nn);
#endif
		for(jj=0; jj<ng[nn]; jj++)
			{
			fprintf(file, "%18.15e, ", -BLASFEO_DVECEL(qp->d+nn, 2*nb[nn]+ng[nn]+nq[nn]+jj));
			}
		fprintf(file, "};\n");
		}
#ifdef DOUBLE_PRECISION
	fprintf(file, "static double *uug[] = {");
#else
	fprintf(file, "static float *uug[] = {");
#endif
	for(nn=0; nn<=N; nn++)
		fprintf(file, "ug%d, ", nn);
	fprintf(file, "};\n");
#ifdef DOUBLE_PRECISION
	fprintf(file, "double **hug = uug;\n");
#else
	fprintf(file, "float **hug = uug;\n");
#endif

	// ug_mask
	fprintf(file, "/* ug_mask */\n");
	for(nn=0; nn<=N; nn++)
		{
#ifdef DOUBLE_PRECISION
		fprintf(file, "static double ug_mask%d[] = {", nn);
#else
		fprintf(file, "static float ug_mask%d[] = {", nn);
#endif
		for(jj=0; jj<ng[nn]; jj++)
			{
			fprintf(file, "%18.15e, ", BLASFEO_DVECEL(qp->d_mask+nn, 2*nb[nn]+ng[nn]+nq[nn]+jj));
			}
		fprintf(file, "};\n");
		}
#ifdef DOUBLE_PRECISION
	fprintf(file, "static double *uug_mask[] = {");
#else
	fprintf(file, "static float *uug_mask[] = {");
#endif
	for(nn=0; nn<=N; nn++)
		fprintf(file, "ug_mask%d, ", nn);
	fprintf(file, "};\n");
#ifdef DOUBLE_PRECISION
	fprintf(file, "double **hug_mask = uug_mask;\n");
#else
	fprintf(file, "float **hug_mask = uug_mask;\n");
#endif

	// Qq
	fprintf(file, "/* Qq */\n");
	for(nn=0; nn<=N; nn++)
		{
#ifdef DOUBLE_PRECISION
		fprintf(file, "static double Qq%d[] = {", nn);
#else
		fprintf(file, "static float Qq%d[] = {", nn);
#endif
		for(kk=0; kk<nq[nn]; kk++)
			{
			for(jj=0; jj<nx[nn]; jj++)
				{
				for(ii=0; ii<nx[nn]; ii++)
					{
#ifdef DOUBLE_PRECISION
					fprintf(file, "%18.15e, ", BLASFEO_DMATEL(&qp->Hq[nn][kk], nu[nn]+ii, nu[nn]+jj));
#else
					fprintf(file, "%18.15e, ", BLASFEO_SMATEL(&qp->Hq[nn][kk], nu[nn]+ii, nu[nn]+jj));
#endif
					}
				}
			}
		fprintf(file, "};\n");
		}
#ifdef DOUBLE_PRECISION
	fprintf(file, "static double *QQq[] = {");
#else
	fprintf(file, "static float *QQq[] = {");
#endif
	for(nn=0; nn<=N; nn++)
		fprintf(file, "Qq%d, ", nn);
	fprintf(file, "};\n");
#ifdef DOUBLE_PRECISION
	fprintf(file, "double **hQq = QQq;\n");
#else
	fprintf(file, "float **hQq = QQq;\n");
#endif

	// Sq
	fprintf(file, "/* Sq */\n");
	for(nn=0; nn<=N; nn++)
		{
#ifdef DOUBLE_PRECISION
		fprintf(file, "static double Sq%d[] = {", nn);
#else
		fprintf(file, "static float Sq%d[] = {", nn);
#endif
		for(kk=0; kk<nq[nn]; kk++)
			{
			for(ii=0; ii<nx[nn]; ii++)
				{
				for(jj=0; jj<nu[nn]; jj++)
					{
#ifdef DOUBLE_PRECISION
					fprintf(file, "%18.15e, ", BLASFEO_DMATEL(&qp->Hq[nn][kk], nu[nn]+ii, jj));
#else
					fprintf(file, "%18.15e, ", BLASFEO_SMATEL(&qp->Hq[nn][kk], nu[nn]+ii, jj));
#endif
					}
				}
			}
		fprintf(file, "};\n");
		}
#ifdef DOUBLE_PRECISION
	fprintf(file, "static double *SSq[] = {");
#else
	fprintf(file, "static float *SSq[] = {");
#endif
	for(nn=0; nn<=N; nn++)
		fprintf(file, "Sq%d, ", nn);
	fprintf(file, "};\n");
#ifdef DOUBLE_PRECISION
	fprintf(file, "double **hSq = SSq;\n");
#else
	fprintf(file, "float **hSq = SSq;\n");
#endif

	// Rq
	fprintf(file, "/* Rq */\n");
	for(nn=0; nn<=N; nn++)
		{
#ifdef DOUBLE_PRECISION
		fprintf(file, "static double Rq%d[] = {", nn);
#else
		fprintf(file, "static float Rq%d[] = {", nn);
#endif
		for(kk=0; kk<nq[nn]; kk++)
			{
			for(jj=0; jj<nu[nn]; jj++)
				{
				for(ii=0; ii<nu[nn]; ii++)
					{
#ifdef DOUBLE_PRECISION
					fprintf(file, "%18.15e, ", BLASFEO_DMATEL(&qp->Hq[nn][kk], ii, jj));
#else
					fprintf(file, "%18.15e, ", BLASFEO_SMATEL(&qp->Hq[nn][kk], ii, jj));
#endif
					}
				}
			}
		fprintf(file, "};\n");
		}
#ifdef DOUBLE_PRECISION
	fprintf(file, "static double *RRq[] = {");
#else
	fprintf(file, "static float *RRq[] = {");
#endif
	for(nn=0; nn<=N; nn++)
		fprintf(file, "Rq%d, ", nn);
	fprintf(file, "};\n");
#ifdef DOUBLE_PRECISION
	fprintf(file, "double **hRq = RRq;\n");
#else
	fprintf(file, "float **hRq = RRq;\n");
#endif

	// qq
	fprintf(file, "/* qq */\n");
	for(nn=0; nn<=N; nn++)
		{
#ifdef DOUBLE_PRECISION
		fprintf(file, "static double qq%d[] = {", nn);
#else
		fprintf(file, "static float qq%d[] = {", nn);
#endif
		for(kk=0; jj<nq[nn]; kk++)
			{
			for(jj=0; jj<nx[nn]; jj++)
				{
#ifdef DOUBLE_PRECISION
//				fprintf(file, "%18.15e, ", BLASFEO_DVECEL(&qp->gq[nn][kk], nu[nn]+jj));
				fprintf(file, "%18.15e, ", BLASFEO_DMATEL(qp->DCt+nn, nu[nn]+jj, ng[nn]+kk));
#else
//				fprintf(file, "%18.15e, ", BLASFEO_SVECEL(&qp->gq[nn][kk], nu[nn]+jj));
				fprintf(file, "%18.15e, ", BLASFEO_SMATEL(qp->DCt+nn, nu[nn]+jj, ng[nn]+kk));
#endif
				}
			}
		fprintf(file, "};\n");
		}
#ifdef DOUBLE_PRECISION
	fprintf(file, "static double *qqq[] = {");
#else
	fprintf(file, "static float *qqq[] = {");
#endif
	for(nn=0; nn<=N; nn++)
		fprintf(file, "qq%d, ", nn);
	fprintf(file, "};\n");
#ifdef DOUBLE_PRECISION
	fprintf(file, "double **hqq = qqq;\n");
#else
	fprintf(file, "float **hqq = qqq;\n");
#endif

	// rq
	fprintf(file, "/* rq */\n");
	for(nn=0; nn<=N; nn++)
		{
#ifdef DOUBLE_PRECISION
		fprintf(file, "static double rq%d[] = {", nn);
#else
		fprintf(file, "static float rq%d[] = {", nn);
#endif
		for(kk=0; jj<nq[nn]; kk++)
			{
			for(jj=0; jj<nu[nn]; jj++)
				{
#ifdef DOUBLE_PRECISION
//				fprintf(file, "%18.15e, ", BLASFEO_DVECEL(&qp->gq[nn][kk], jj));
				fprintf(file, "%18.15e, ", BLASFEO_DMATEL(qp->DCt+nn, jj, ng[nn]+kk));
#else
//				fprintf(file, "%18.15e, ", BLASFEO_SVECEL(&qp->gq[nn][kk], jj));
				fprintf(file, "%18.15e, ", BLASFEO_SMATEL(qp->DCt+nn, jj, ng[nn]+kk));
#endif
				}
			}
		fprintf(file, "};\n");
		}
#ifdef DOUBLE_PRECISION
	fprintf(file, "static double *rrq[] = {");
#else
	fprintf(file, "static float *rrq[] = {");
#endif
	for(nn=0; nn<=N; nn++)
		fprintf(file, "rq%d, ", nn);
	fprintf(file, "};\n");
#ifdef DOUBLE_PRECISION
	fprintf(file, "double **hrq = rrq;\n");
#else
	fprintf(file, "float **hrq = rrq;\n");
#endif

	// uq
	fprintf(file, "/* uq */\n");
	for(nn=0; nn<=N; nn++)
		{
#ifdef DOUBLE_PRECISION
		fprintf(file, "static double uq%d[] = {", nn);
#else
		fprintf(file, "static float uq%d[] = {", nn);
#endif
		for(jj=0; jj<nq[nn]; jj++)
			{
			fprintf(file, "%18.15e, ", -BLASFEO_DVECEL(qp->d+nn, 2*nb[nn]+2*ng[nn]+nq[nn]+jj));
			}
		fprintf(file, "};\n");
		}
#ifdef DOUBLE_PRECISION
	fprintf(file, "static double *uuq[] = {");
#else
	fprintf(file, "static float *uuq[] = {");
#endif
	for(nn=0; nn<=N; nn++)
		fprintf(file, "uq%d, ", nn);
	fprintf(file, "};\n");
#ifdef DOUBLE_PRECISION
	fprintf(file, "double **huq = uuq;\n");
#else
	fprintf(file, "float **huq = uuq;\n");
#endif

	// uq_mask
	fprintf(file, "/* uq_mask */\n");
	for(nn=0; nn<=N; nn++)
		{
#ifdef DOUBLE_PRECISION
		fprintf(file, "static double uq_mask%d[] = {", nn);
#else
		fprintf(file, "static float uq_mask%d[] = {", nn);
#endif
		for(jj=0; jj<ng[nn]; jj++)
			{
			fprintf(file, "%18.15e, ", BLASFEO_DVECEL(qp->d_mask+nn, 2*nb[nn]+2*ng[nn]+nq[nn]+jj));
			}
		fprintf(file, "};\n");
		}
#ifdef DOUBLE_PRECISION
	fprintf(file, "static double *uuq_mask[] = {");
#else
	fprintf(file, "static float *uuq_mask[] = {");
#endif
	for(nn=0; nn<=N; nn++)
		fprintf(file, "uq_mask%d, ", nn);
	fprintf(file, "};\n");
#ifdef DOUBLE_PRECISION
	fprintf(file, "double **huq_mask = uuq_mask;\n");
#else
	fprintf(file, "float **huq_mask = uuq_mask;\n");
#endif

	// Zl
	fprintf(file, "/* Zl */\n");
#ifdef DOUBLE_PRECISION
	fprintf(file, "double **hZl;\n");
#else
	fprintf(file, "float **hZl;\n");
#endif

	// Zu
	fprintf(file, "/* Zu */\n");
#ifdef DOUBLE_PRECISION
	fprintf(file, "double **hZu;\n");
#else
	fprintf(file, "float **hZu;\n");
#endif

	// zl
	fprintf(file, "/* zl */\n");
#ifdef DOUBLE_PRECISION
	fprintf(file, "double **hzl;\n");
#else
	fprintf(file, "float **hzl;\n");
#endif

	// zu
	fprintf(file, "/* zu */\n");
#ifdef DOUBLE_PRECISION
	fprintf(file, "double **hzu;\n");
#else
	fprintf(file, "float **hzu;\n");
#endif

	// idxs_rev
	fprintf(file, "/* idxs_rev */\n");
	fprintf(file, "int **hidxs_rev;\n");

	// idxs // TODO remove !!!
	fprintf(file, "/* idxs */\n");
	fprintf(file, "int **hidxs;\n");

	// lls
	fprintf(file, "/* lls */\n");
#ifdef DOUBLE_PRECISION
	fprintf(file, "double **hlls;\n");
#else
	fprintf(file, "float **hlls;\n");
#endif

	// lus
	fprintf(file, "/* lus */\n");
#ifdef DOUBLE_PRECISION
	fprintf(file, "double **hlus;\n");
#else
	fprintf(file, "float **hlus;\n");
#endif

	// XXX what follows is not part of the QP !!!

	// u_guess
//	fprintf(file, "/* u_guess */\n");
//	fprintf(file, "double **hu_guess;\n");

	// x_guess
//	fprintf(file, "/* x_guess */\n");
//	fprintf(file, "double **hx_guess;\n");

	// sl_guess
//	fprintf(file, "/* sl_guess */\n");
//	fprintf(file, "double **hsl_guess;\n");

	// su_guess
//	fprintf(file, "/* su_guess */\n");
//	fprintf(file, "double **hsu_guess;\n");

	fclose(file);

	return;
	}



void OCP_QCQP_SOL_PRINT(struct OCP_QCQP_DIM *qp_dim, struct OCP_QCQP_SOL *qp_sol)
	{
	int ii;

	int N   = qp_dim->N;
	int *nx = qp_dim->nx;
	int *nu = qp_dim->nu;
	int *nb = qp_dim->nb;
	int *ng = qp_dim->ng;
	int *nq = qp_dim->nq;
	int *ns = qp_dim->ns;

	printf("uxs =\n");
	for (ii = 0; ii <= N; ii++)
		BLASFEO_PRINT_TRAN_VEC(nu[ii]+nx[ii]+2*ns[ii], &qp_sol->ux[ii], 0);

	printf("pi =\n");
	for (ii = 0; ii < N; ii++)
		BLASFEO_PRINT_TRAN_VEC(nx[ii+1], &qp_sol->pi[ii], 0);

	printf("lam =\n");
	for (ii = 0; ii <= N; ii++)
		BLASFEO_PRINT_TRAN_VEC(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], &qp_sol->lam[ii], 0);

	printf("t =\n");
	for (ii = 0; ii <= N; ii++)
		BLASFEO_PRINT_TRAN_VEC(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], &qp_sol->t[ii], 0);

	return;
	}



void OCP_QCQP_IPM_ARG_CODEGEN(char *file_name, char *mode, struct OCP_QCQP_DIM *qp_dim, struct OCP_QCQP_IPM_ARG *arg)
	{
	int ii;

	FILE *file = fopen(file_name, mode);

	fprintf(file, "/***************\n* arg\n***************/\n");

	// iter_max
	fprintf(file, "/* mode */\n");
	fprintf(file, "int mode = %d;\n", arg->mode);
	// iter_max
	fprintf(file, "/* iter_max */\n");
	fprintf(file, "int iter_max = %d;\n", arg->iter_max);
	// alpha_min
	fprintf(file, "/* alpha_min */\n");
	fprintf(file, "double alpha_min = %18.15e;\n", arg->alpha_min);
	// mu0
	fprintf(file, "/* mu0 */\n");
	fprintf(file, "double mu0 = %18.15e;\n", arg->mu0);
	// tol_stat
	fprintf(file, "/* tol_stat */\n");
	fprintf(file, "double tol_stat = %18.15e;\n", arg->res_g_max);
	// tol_eq
	fprintf(file, "/* tol_eq */\n");
	fprintf(file, "double tol_eq = %18.15e;\n", arg->res_b_max);
	// tol_ineq
	fprintf(file, "/* tol_ineq */\n");
	fprintf(file, "double tol_ineq = %18.15e;\n", arg->res_d_max);
	// tol_comp
	fprintf(file, "/* tol_comp */\n");
	fprintf(file, "double tol_comp = %18.15e;\n", arg->res_m_max);
	// reg_prim
	fprintf(file, "/* reg_prim */\n");
	fprintf(file, "double reg_prim = %18.15e;\n", arg->reg_prim);
	// warm_start
	fprintf(file, "/* warm_start */\n");
	fprintf(file, "int warm_start = %d;\n", arg->warm_start);
	// pred_corr
	fprintf(file, "/* pred_corr */\n");
	fprintf(file, "int pred_corr = %d;\n", arg->pred_corr);
	// ric_alg
	fprintf(file, "/* ric_alg */\n");
	fprintf(file, "int ric_alg = %d;\n", arg->square_root_alg);
	// split_step
	fprintf(file, "/* split_step */\n");
	fprintf(file, "int split_step = %d;\n", arg->split_step);

	fclose(file);

	return;
	}



void OCP_QCQP_RES_PRINT(struct OCP_QCQP_DIM *qp_dim, struct OCP_QCQP_RES *qp_res)
	{
	int ii;

	int N   = qp_dim->N;
	int *nx = qp_dim->nx;
	int *nu = qp_dim->nu;
	int *nb = qp_dim->nb;
	int *ng = qp_dim->ng;
	int *nq = qp_dim->nq;
	int *ns = qp_dim->ns;

	printf("res_g =\n");
	for (ii = 0; ii <= N; ii++)
		BLASFEO_PRINT_EXP_TRAN_VEC(nu[ii]+nx[ii]+2*ns[ii], &qp_res->res_g[ii], 0);

	printf("res_b =\n");
	for (ii = 0; ii < N; ii++)
		BLASFEO_PRINT_EXP_TRAN_VEC(nx[ii+1], &qp_res->res_b[ii], 0);

	printf("res_d =\n");
	for (ii = 0; ii <= N; ii++)
		BLASFEO_PRINT_EXP_TRAN_VEC(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], &qp_res->res_d[ii], 0);

	printf("res_m =\n");
	for (ii = 0; ii <= N; ii++)
		BLASFEO_PRINT_EXP_TRAN_VEC(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], &qp_res->res_m[ii], 0);

	return;
	}





