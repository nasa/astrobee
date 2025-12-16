/*  -- LAPACK driver routine (version 3.2) -- */
/*     Univ. of Tennessee, Univ. of California Berkeley and NAG Ltd.. */
/*     November 2006 */

/*  Purpose */
/*  ======= */

/*  DSTEDC computes all eigenvalues and, optionally, eigenvectors of a */
/*  symmetric tridiagonal matrix using the divide and conquer method. */
/*  The eigenvectors of a full or band real symmetric matrix can also be */
/*  found if DSYTRD or DSPTRD or DSBTRD has been used to reduce this */
/*  matrix to tridiagonal form. */

/*  This code makes very mild assumptions about floating point */
/*  arithmetic. It will work on machines with a guard digit in */
/*  add/subtract, or on those binary machines without guard digits */
/*  which subtract like the Cray X-MP, Cray Y-MP, Cray C-90, or Cray-2. */
/*  It could conceivably fail on hexadecimal or decimal machines */
/*  without guard digits, but we know of none.  See DLAED3 for details. */

/*  Arguments */
/*  ========= */

/*  COMPZ   (input) CHARACTER*1 */
/*          = 'N':  Compute eigenvalues only. */
/*          = 'I':  Compute eigenvectors of tridiagonal matrix also. */
/*          = 'V':  Compute eigenvectors of original dense symmetric */
/*                  matrix also.  On entry, Z contains the orthogonal */
/*                  matrix used to reduce the original matrix to */
/*                  tridiagonal form. */

/*  N       (input) INTEGER */
/*          The dimension of the symmetric tridiagonal matrix.  N >= 0. */

/*  D       (input/output) DOUBLE PRECISION array, dimension (N) */
/*          On entry, the diagonal elements of the tridiagonal matrix. */
/*          On exit, if INFO = 0, the eigenvalues in ascending order. */

/*  E       (input/output) DOUBLE PRECISION array, dimension (N-1) */
/*          On entry, the subdiagonal elements of the tridiagonal matrix. */
/*          On exit, E has been destroyed. */

/*  Z       (input/output) DOUBLE PRECISION array, dimension (LDZ,N) */
/*          On entry, if COMPZ = 'V', then Z contains the orthogonal */
/*          matrix used in the reduction to tridiagonal form. */
/*          On exit, if INFO = 0, then if COMPZ = 'V', Z contains the */
/*          orthonormal eigenvectors of the original symmetric matrix, */
/*          and if COMPZ = 'I', Z contains the orthonormal eigenvectors */
/*          of the symmetric tridiagonal matrix. */
/*          If  COMPZ = 'N', then Z is not referenced. */

/*  LDZ     (input) INTEGER */
/*          The leading dimension of the array Z.  LDZ >= 1. */
/*          If eigenvectors are desired, then LDZ >= max(1,N). */

/*  WORK    (workspace/output) DOUBLE PRECISION array, */
/*                                         dimension (LWORK) */
/*          On exit, if INFO = 0, WORK(1) returns the optimal LWORK. */

/*  LWORK   (input) INTEGER */
/*          The dimension of the array WORK. */
/*          If COMPZ = 'N' or N <= 1 then LWORK must be at least 1. */
/*          If COMPZ = 'V' and N > 1 then LWORK must be at least */
/*                         ( 1 + 3*N + 2*N*lg N + 3*N**2 ), */
/*                         where lg( N ) = smallest integer k such */
/*                         that 2**k >= N. */
/*          If COMPZ = 'I' and N > 1 then LWORK must be at least */
/*                         ( 1 + 4*N + N**2 ). */
/*          Note that for COMPZ = 'I' or 'V', then if N is less than or */
/*          equal to the minimum divide size, usually 25, then LWORK need */
/*          only be max(1,2*(N-1)). */

/*          If LWORK = -1, then a workspace query is assumed; the routine */
/*          only calculates the optimal size of the WORK array, returns */
/*          this value as the first entry of the WORK array, and no error */
/*          message related to LWORK is issued by XERBLA. */

/*  IWORK   (workspace/output) INTEGER array, dimension (MAX(1,LIWORK)) */
/*          On exit, if INFO = 0, IWORK(1) returns the optimal LIWORK. */

/*  LIWORK  (input) INTEGER */
/*          The dimension of the array IWORK. */
/*          If COMPZ = 'N' or N <= 1 then LIWORK must be at least 1. */
/*          If COMPZ = 'V' and N > 1 then LIWORK must be at least */
/*                         ( 6 + 6*N + 5*N*lg N ). */
/*          If COMPZ = 'I' and N > 1 then LIWORK must be at least */
/*                         ( 3 + 5*N ). */
/*          Note that for COMPZ = 'I' or 'V', then if N is less than or */
/*          equal to the minimum divide size, usually 25, then LIWORK */
/*          need only be 1. */

/*          If LIWORK = -1, then a workspace query is assumed; the */
/*          routine only calculates the optimal size of the IWORK array, */
/*          returns this value as the first entry of the IWORK array, and */
/*          no error message related to LIWORK is issued by XERBLA. */

/*  INFO    (output) INTEGER */
/*          = 0:  successful exit. */
/*          < 0:  if INFO = -i, the i-th argument had an illegal value. */
/*          > 0:  The algorithm failed to compute an eigenvalue while */
/*                working on the submatrix lying in rows and columns */
/*                INFO/(N+1) through mod(INFO,N+1). */

/*  Further Details */
/*  =============== */

/*  Based on contributions by */
/*     Jeff Rutter, Computer Science Division, University of California */
/*     at Berkeley, USA */
/*  Modified by Francoise Tisseur, University of Tennessee. */

/*  ===================================================================== */

#include <math.h>
#include <stdbool.h>

#include <blasfeo_d_blas_api.h>



#if defined(FORTRAN_BLAS_API)
#define blasfeo_blas_dgemm dgemm_
#define blasfeo_lapack_dlaed0 dlaed0_
#define blasfeo_lapack_dstedc dstedc_
#endif



#define min(x, y) (((x) < (y)) ? (x) : (y))
#define max(x, y) (((x) > (y)) ? (x) : (y))



void dlacpy_(char *, int *, int *, double *, int *, double *, int *);
void blasfeo_lapack_dlaed0(int *, int *, int *, double *, double *, double *, int *, double *, int *, double *, int *, int *);
double dlamch_(char *);
double dlanst_(char *, int *, double *, double *);
void dlascl_(char *, int *, int *, double *, double *, int *, int *, double *, int *, int *);
void dlaset_(char *, int *, int *, double *, double *, double *, int *);
void dlasrt_(char *, int *, double *, int *);
void dsteqr_(char *, int *, double *, double *, double *, int *, double *, int *);
void dsterf_(int *, double *, double *, int *);
void dswap_(int *, double *, int *, double *, int *);
int ilaenv_(int *, char *, char *, int *, int *, int *, int *);
bool lsame_(char *, char *);
void xerbla_(char *, int *);



void blasfeo_lapack_dstedc(char *compz, int *pn, double *d, double *e, double *Z, int *pldz, double *work, int *lwork, int *iwork, int *liwork, int *info)
	{

	int n = *pn;
	int ldz = *pldz;

	int i_0 = 0;
	int i_1 = 1;
	int i_2 = 2;
	int i_9 = 9;
	double d_0 = 0.0;
	double d_1 = 1.0;

	int smlsiz, liwmin, lwmin, lgn, icompz, storez, start, finish, m, strtrw, k;
	double orgnrm, eps, tiny, p;

	int ii, jj, idx;
	int i_t0;

    /* Function Body */
    *info = 0;
    bool lquery = *lwork == -1 || *liwork == -1;

    if (lsame_(compz, "N"))
		{
		icompz = 0;
		}
	else if (lsame_(compz, "V"))
		{
		icompz = 1;
		}
	else if (lsame_(compz, "I"))
		{
		icompz = 2;
		}
	else
		{
		icompz = -1;
		}
		if
	(icompz < 0)
		{
		*info = -1;
		}
	else if (n < 0)
		{
		*info = -2;
		}
	else if (ldz < 1 || icompz > 0 && ldz < max(1,n))
		{
		*info = -6;
		}

    if (*info == 0)
		{

/*        Compute the workspace requirements */

		smlsiz = ilaenv_(&i_9, "DSTEDC", " ", &i_0, &i_0, &i_0, &i_0);
		if (n <= 1 || icompz == 0)
			{
			liwmin = 1;
			lwmin = 1;
			}
		else if (n <= smlsiz)
			{
			liwmin = 1;
			lwmin = 2*(n - 1);
			}
		else
			{
			lgn = (int) (log((double) (n)) / log(2.0));
			if ((1<<lgn) < n)
				{
				lgn++;
				}
			if ((1<<lgn) < n)
				{
				lgn++;
				}
			if (icompz == 1)
				{
				lwmin = 1 + 3*n + 2*n*lgn + 4*n*n;
				liwmin = 6 + 6*n + 5*n*lgn;
				}
			else if (icompz == 2)
				{
				lwmin = 1 + 4*n + n*n;
				liwmin = 3 + 5*n;
				}
			}
		work[0] = (double) lwmin;
		iwork[0] = liwmin;

		if (*lwork < lwmin && ! lquery)
			{
			*info = -8;
			}
		else if (*liwork < liwmin && ! lquery)
			{
			*info = -10;
			}
		}

    if (*info != 0)
		{
		i_t0 = -(*info);
		xerbla_("DSTEDC", &i_t0);
		return;
		}
	else if (lquery)
		{
		return;
		}

/*     Quick return if possible */

    if (n == 0)
		{
		return;
		}
	if (n == 1)
		{
		if (icompz != 0)
			{
			Z[0] = 1.0;
			}
		return;
		}

/*     If the following conditional clause is removed, then the routine */
/*     will use the Divide and Conquer routine to compute only the */
/*     eigenvalues, which requires (3N + 3N**2) real workspace and */
/*     (2 + 5N + 2N lg(N)) integer workspace. */
/*     Since on many architectures DSTERF is much faster than any other */
/*     algorithm for finding eigenvalues only, it is used here */
/*     as the default. If the conditional clause is removed, then */
/*     information on the size of workspace needs to be changed. */

/*     If COMPZ = 'N', use DSTERF to compute the eigenvalues. */

    if (icompz == 0)
		{
		dsterf_(&n, &d[0], &e[0], info);
		goto end;
		}

/*     If N is smaller than the minimum divide size (SMLSIZ+1), then */
/*     solve the problem with another solver. */

    if (n <= smlsiz)
		{

		dsteqr_(compz, &n, &d[0], &e[0], &Z[0], &ldz, &work[0], info);

		}
	else
		{

	/*        If COMPZ = 'V', the Z matrix must be stored elsewhere for later */
	/*        use. */

		if (icompz == 1)
			{
			storez = 1 + n*n;
			}
		else
			{
			storez = 1;
			}

		if (icompz == 2)
			{
			dlaset_("Full", &n, &n, &d_0, &d_1, &Z[0], &ldz);
			}

	/*        Scale. */

		orgnrm = dlanst_("M", &n, &d[0], &e[0]);
		if (orgnrm == 0.0)
			{
			goto end;
			}

		eps = dlamch_("Epsilon");

		start = 1;

	/*        while ( START <= N ) */

	L10:
		if (start <= n)
			{

	/*           Let FINISH be the position of the next subdiagonal entry */
	/*           such that E( FINISH ) <= TINY or FINISH = N if no such */
	/*           subdiagonal exists.  The matrix identified by the elements */
	/*           between START and FINISH constitutes an independent */
	/*           sub-problem. */

			finish = start;
	L20:
			if (finish < n)
				{
				tiny = eps * sqrt(fabs(d[finish-1])) * sqrt(fabs(d[finish + 0]));
				if (fabs(e[finish-1]) > tiny)
					{
					finish++;
					goto L20;
					}
				}

	/*           (Sub) Problem determined.  Compute its size and solve it. */

			m = finish - start + 1;
			if (m == 1)
				{
				start = finish + 1;
				goto L10;
				}
			if (m > smlsiz)
				{

		/*              Scale. */

				orgnrm = dlanst_("M", &m, &d[start-1], &e[start-1]);
				dlascl_("G", &i_0, &i_0, &orgnrm, &d_1, &m, &i_1, &d[start-1], &m, info);
				i_t0 = m - 1;
				dlascl_("G", &i_0, &i_0, &orgnrm, &d_1, &i_t0, &i_1, &e[start-1], &i_t0, info);

				if (icompz == 1)
					{
					strtrw = 1;
					}
				else
					{
					strtrw = start;
					}
				blasfeo_lapack_dlaed0(&icompz, &n, &m, &d[start-1], &e[start-1], &Z[strtrw-1 + (start-1) * ldz], &ldz, &work[0], &n, &work[storez-1], &iwork[0], info);
				if (*info != 0)
					{
					*info = (*info / (m + 1) + start - 1) * (n + 1) + *info % (m + 1) + start - 1;
					goto end;
					}

		/*              Scale back. */

				dlascl_("G", &i_0, &i_0, &d_1, &orgnrm, &m, &i_1, &d[start-1], &m, info);

				}
			else
				{
				if (icompz == 1)
					{

		/*                 Since QR won't update a Z matrix which is larger than */
		/*                 the length of D, we must solve the sub-problem in a */
		/*                 workspace and then multiply back into Z. */

					dsteqr_("I", &m, &d[start-1], &e[start-1], &work[0], &m, &work[m*m ], info);
					dlacpy_("A", &n, &m, &Z[(start-1) * ldz], &ldz, &work[storez-1], &n);
					blasfeo_blas_dgemm("N", "N", &n, &m, &m, &d_1, &work[storez-1], &n, &work[0], &m, &d_0, &Z[(start-1) * ldz], &ldz);
					}
				else if (icompz == 2)
					{
					dsteqr_("I", &m, &d[start-1], &e[start-1], &Z[start-1 + (start-1) * ldz], &ldz, &work[0], info);
					}
				else
					{
					dsterf_(&m, &d[start-1], &e[start-1], info);
					}
				if (*info != 0)
					{
					*info = start * (n + 1) + finish;
					goto end;
					}
				}

			start = finish + 1;
			goto L10;
			}

	/*        endwhile */

	/*        If the problem split any number of times, then the eigenvalues */
	/*        will not be properly ordered.  Here we permute the eigenvalues */
	/*        (and the associated eigenvectors) into ascending order. */

//		if (m != n) // XXX as in lapack 3.10
//			{
			if (icompz == 0)
				{

	/*              Use Quick Sort */

				dlasrt_("I", &n, &d[0], info);

				}
			else
				{

	/*              Use Selection Sort to minimize swaps of eigenvectors */

				for (ii = 1; ii < n; ii++)
					{
					idx = ii - 1;
					k = idx;
					p = d[idx];
					for (jj = ii; jj < n; jj++)
						{
						if (d[jj] < p)
							{
							k = jj;
							p = d[jj];
							}
						}
					if (k != idx)
						{
						d[k] = d[idx];
						d[idx] = p;
						dswap_(&n, &Z[(idx) * ldz], &i_1, &Z[k * ldz], &i_1);
						}
					}
				}
//			}
		}

end:
    work[0] = (double) lwmin;
    iwork[0] = liwmin;

    return;

/*     End of DSTEDC */

	} /* dstedc_ */

