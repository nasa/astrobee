/**************************************************************************************************
*                                                                                                 *
* This file is part of BLASFEO.                                                                   *
*                                                                                                 *
* BLASFEO -- BLAS For Embedded Optimization.                                                      *
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



#ifdef DOUBLE_PRECISION
#define REL_TOL 1e-11
#else
#define REL_TOL 9e-4
#endif



// Test {double,single} precision common

void print_compilation_flags()
	{
	SHOW_DEFINE(BLASFEO_LA)
	SHOW_DEFINE(BLASFEO_TARGET)
	SHOW_DEFINE(BLASFEO_MF)
	SHOW_DEFINE(PRECISION)
#ifdef TEST_BLAS_API
	printf("API\t\t= blas\n");
#else
	printf("API\t\t= BLASFEO\n");
#endif
	SHOW_DEFINE(K_MAX_STACK)
	SHOW_DEFINE(PACKING_ALG)
	SHOW_DEFINE(ROUTINE_FULLNAME)
	}



void initialize_test_args(struct TestArgs * targs)
	{
	// sub-mastrix offset, sweep start
	targs->ai0 = 0;
	targs->bi0 = 0;
	targs->di0 = 0;

	targs->xj0 = 0;

	targs->ais = 1;
	targs->bis = 1;
	targs->dis = 1;

	targs->xjs = 1;

	// sub-matrix dimensions, sweep start
	targs->ni0 = 1;
	targs->nj0 = 1;
	targs->nk0 = 1;

	// sub-matrix dimensions, sweep lenght
	targs->nis = 1;
	targs->njs = 1;
	targs->nks = 1;

	targs->alphas = 1;
	targs->alpha_l[0] = 1.0;
	targs->alpha_l[1] = 0.0;
	targs->alpha_l[2] = 0.0001;
	targs->alpha_l[3] = 0.02;
	targs->alpha_l[4] = 400.0;
	targs->alpha_l[5] = 50000.0;

	targs->betas = 1;
	targs->beta_l[0] = 1.0;
	targs->beta_l[1] = 0.0;
	targs->beta_l[2] = 0.0001;
	targs->beta_l[3] = 0.02;
	targs->beta_l[4] = 400.0;
	targs->beta_l[5] = 50000.0;

	targs->total_calls = 1;
	}



int compute_total_calls(struct TestArgs * targs)
	{
	int total_calls =
		targs->alphas *
		targs->betas *
		targs->nis *
		targs->njs *
		targs->nks *
		targs->ais *
		targs->bis *
		targs->dis *
		targs->xjs;

	return total_calls;
	}



void initialize_args(struct RoutineArgs * args)
	{
	args->alpha = 1.5;
	args->beta = 1.5;

	args->err_i = 0;
	args->err_j = 0;

	// sizes
	args->n = 0;
	args->m = 0;
	args->k = 0;

	// offset
	args->ai = 0;
	args->aj = 0;

	args->bi = 0;
	args->bj = 0;

	args->ci = 0;
	args->cj = 0;

	args->di = 0;
	args->dj = 0;
	}



/* prints a matrix in column-major format */
// TODO remove !!!!!!!!!!!!!!!!!!!!
void print_xmat_debug(int m, int n, struct STRMAT_REF *sA, int ai, int aj, int err_i, int err_j, int ERR)
	{

	/* REAL *pA = sA->pA + ai + aj*lda; */
	int lda = sA->m;
	REAL *pA = sA->pA;
	int j0,i0, ie, je;
	int i, j;
	const int max_rows = 16;
	const int max_cols = 9;
	const int offset = 2;

	i0 = (ai - offset >=0 )? ai - offset : 0;
	ie = ai + m + offset;
	j0 = (aj - offset >=0 )? aj - offset : 0;
	je = aj + n + offset;

	if (ie-i0 > max_rows)
	{
		i0 = (err_i - ((int)(max_rows/2)) >=0 )? err_i - ((int)(max_rows/2)) : 0;
		ie = err_i + ((int)(max_rows/2)) ;
	}
	if (je-j0 > max_cols)
	{
		j0 = (err_j - ((int)(max_rows/2)) >=0 )? err_j - ((int)(max_rows/2)) : 0;
		je = err_j + ((int)(max_rows/2)) ;
	}

	/* i0 = err_i-subsize; */
	/* j0 = err_j-subsize; */
	/* if (i0 < ai) i0 = ai; */
	/* if (j0 < aj) j0 = aj; */
	/* ie = err_i+subsize; */
	/* je = err_j+subsize; */
	/* if (ie > ai+m) ie = ai+m; */
	/* if (je > aj+n) je = aj+n; */

	if (!ERR)
	{
		i0 = ai;
		j0 = aj;
		ie = ai+m;
		je = aj+n;
	}

	printf("%s\t", "REF");
	for(j=j0; j<je; j++) printf("%7d\t", j);
	printf("\n");
	for(j=j0; j<je+1; j++) printf("-------\t");
	printf("\n");

	for(i=i0; i<ie; i++)
		{
		for(j=j0; j<je; j++)
			{
			if (j == j0)  printf("%3d |\t", i);

			if ((i==err_i) && (j==err_j) && ERR)
				printf(ANSI_COLOR_RED"%6.2f\t"ANSI_COLOR_RESET, pA[i+lda*j]);
			else if ((i >= ai) && (i <= ai+m) && (j >= aj) && (j < aj+n))
				printf(ANSI_COLOR_GREEN"%6.2f\t"ANSI_COLOR_RESET, pA[i+lda*j]);
			else printf(ANSI_COLOR_BLUE"%6.2f\t"ANSI_COLOR_RESET, pA[i+lda*j]);

			}
		printf("\n");
		}
	printf("\n");
	return;
	}



/* prints a blasfeo matrix */
void blasfeo_print_xmat_debug(int m, int n, struct STRMAT *sA, int ai, int aj, int err_i, int err_j, int ERR, char *label)
	{
	int i0, j0, ie, je;
	int ii, jj, j, ip0, ipe, ip;

	const int max_rows = 16;
	const int max_cols = 9;
	const int offset = 2;

	i0 = (ai - offset >=0 )? ai - offset : 0;
	ie = ai + m + offset;
	j0 = (aj - offset >=0 )? aj - offset : 0;
	je = aj + n + offset;

	if (ie-i0 > max_rows)
	{
		i0 = (err_i - ((int)(max_rows/2)) >=0 )? err_i - ((int)(max_rows/2)) : 0;
		ie = err_i + ((int)(max_rows/2)) ;
	}
	if (je-j0 > max_cols)
	{
		j0 = (err_j - ((int)(max_rows/2)) >=0 )? err_j - ((int)(max_rows/2)) : 0;
		je = err_j + ((int)(max_rows/2)) ;
	}

	if (!ERR)
	{
		i0 = ai;
		j0 = aj;
		ie = ai+m;
		je = aj+n;
	}

	printf("%s\t", label);
	for(j=j0; j<je; j++) printf("%7d\t", j);
	printf("\n");
	for(j=j0; j<je+1; j++) printf("-------\t");
	printf("\n");

	for(ii=i0; ii<ie; ii++)
		{
		if(j0<je)
			printf("%3d |\t", ii);
		for(jj=j0; jj<je; jj++)
			{
			if((ii==err_i) & (jj==err_j) & ERR)
				printf(ANSI_COLOR_RED"%6.2f\t"ANSI_COLOR_RESET, MATEL_LIBSTR(sA, ii, jj));
			else if((ii >= ai) & (ii < ai+m) & (jj >= aj) && (jj < aj+n))
				printf(ANSI_COLOR_GREEN"%6.2f\t"ANSI_COLOR_RESET, MATEL_LIBSTR(sA, ii, jj));
			else printf("%6.2f\t", MATEL_LIBSTR(sA, ii, jj));
			}
		printf("\n");
		}
	printf("\n");

	return;
	}



static void printbits(void *c, size_t n)
{
	unsigned char *t = c;
	if (c == NULL)
	return;
	while (n > 0)
	{
		int q;
		--n;
		for(q = 0x80; q; q >>= 1) printf("%x", !!(t[n] & q));
	}
	printf("\n");
}



// 1 to 1 comparison of every element
int GECMP_LIBSTR(
	int m, int n, int bi, int bj,
	struct STRMAT *sD, struct STRMAT_REF *rD,
	int* err_i, int* err_j, int debug)
	{
	int ii, jj;

	for(ii = 0; ii < m; ii++)
		{
		for(jj = 0; jj < n; jj++)
			{

			// strtucture mat
			REAL sbi = MATEL_LIBSTR(sD, ii, jj);
			// reference mat
			REAL rbi = MATEL_REF(rD, ii, jj);

			if ( (sbi != rbi) & ( fabs(sbi-rbi) > REL_TOL*(fabs(sbi)+fabs(rbi)) ) & ( fabs(sbi-rbi) > REL_TOL))
				{
					*err_i = ii;
					*err_j = jj;
					if (!debug) return 1;

					printf("\n\nFailed at index %d,%d, (HP) %2.18f != %2.18f (RF)\n", ii, jj, sbi, rbi);
					printf("Absolute error: %3.5e\n", fabs(sbi-rbi));
					printf("Relative error: %3.5e\n", fabs(sbi-rbi)/(fabs(sbi)+fabs(rbi)));
					printf("\nBitwise comparison:\n");
					printf("HP:  ");
					printbits(&sbi, sizeof(REAL));
					printf("REF: ");
					printbits(&rbi, sizeof(REAL));
					printf("\n");

					printf("\nResult matrix:\n");
					blasfeo_print_xmat_debug(m, n, sD, bi, bj, ii, jj, 1, "HP");
					blasfeo_print_xmat_debug(m, n, rD, bi, bj, ii, jj, 1, "REF");

					return 1;
				}
			}
		}

	return 0;
	}


int GECMP_BLASAPI(int m, int n, int bi, int bj, struct STRMAT_REF *sD, struct STRMAT_REF *rD, int* err_i, int* err_j, int debug)
	{
	int ii, jj;

	for(ii = 0; ii < m; ii++)
		{
		for(jj = 0; jj < n; jj++)
			{

			// strtucture mat
			REAL sbi = MATEL_REF(sD, ii, jj);
			// reference mat
			REAL rbi = MATEL_REF(rD, ii, jj);

			if ( (sbi != rbi) & ( fabs(sbi-rbi) > REL_TOL*(fabs(sbi)+fabs(rbi)) ) & ( fabs(sbi-rbi) > REL_TOL))
				{
					*err_i = ii;
					*err_j = jj;
					if (!debug) return 1;

					printf("\n\nFailed at index %d,%d, (HP) %2.18f != %2.18f (RF)\n", ii, jj, sbi, rbi);
					printf("Absolute error: %3.5e\n", fabs(sbi-rbi));
					printf("Relative error: %3.5e\n", fabs(sbi-rbi)/(fabs(sbi)+fabs(rbi)));
					printf("\nBitwise comparison:\n");
					printf("HP:  ");
					printbits(&sbi, sizeof(REAL));
					printf("REF: ");
					printbits(&rbi, sizeof(REAL));
					printf("\n");

					printf("\nResult matrix:\n");
					print_xmat_debug(m, n, sD, bi, bj, ii, jj, 1);
					print_xmat_debug(m, n, rD, bi, bj, ii, jj, 1);

					return 1;
				}
			}
		}

	return 0;
	}
