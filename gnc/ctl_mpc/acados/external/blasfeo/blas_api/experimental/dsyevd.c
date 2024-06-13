/*  -- LAPACK driver routine (version 3.2) -- */
/*     Univ. of Tennessee, Univ. of California Berkeley and NAG Ltd.. */
/*     November 2006 */

/*  Purpose */
/*  ======= */

/*  DSYEVD computes all eigenvalues and, optionally, eigenvectors of a */
/*  real symmetric matrix A. If eigenvectors are desired, it uses a */
/*  divide and conquer algorithm. */

/*  The divide and conquer algorithm makes very mild assumptions about */
/*  floating point arithmetic. It will work on machines with a guard */
/*  digit in add/subtract, or on those binary machines without guard */
/*  digits which subtract like the Cray X-MP, Cray Y-MP, Cray C-90, or */
/*  Cray-2. It could conceivably fail on hexadecimal or decimal machines */
/*  without guard digits, but we know of none. */

/*  Because of large use of BLAS of level 3, DSYEVD needs N**2 more */
/*  workspace than DSYEVX. */

/*  Arguments */
/*  ========= */

/*  JOBZ    (input) CHARACTER*1 */
/*          = 'N':  Compute eigenvalues only; */
/*          = 'V':  Compute eigenvalues and eigenvectors. */

/*  UPLO    (input) CHARACTER*1 */
/*          = 'U':  Upper triangle of A is stored; */
/*          = 'L':  Lower triangle of A is stored. */

/*  N       (input) INTEGER */
/*          The order of the matrix A.  N >= 0. */

/*  A       (input/output) DOUBLE PRECISION array, dimension (LDA, N) */
/*          On entry, the symmetric matrix A.  If UPLO = 'U', the */
/*          leading N-by-N upper triangular part of A contains the */
/*          upper triangular part of the matrix A.  If UPLO = 'L', */
/*          the leading N-by-N lower triangular part of A contains */
/*          the lower triangular part of the matrix A. */
/*          On exit, if JOBZ = 'V', then if INFO = 0, A contains the */
/*          orthonormal eigenvectors of the matrix A. */
/*          If JOBZ = 'N', then on exit the lower triangle (if UPLO='L') */
/*          or the upper triangle (if UPLO='U') of A, including the */
/*          diagonal, is destroyed. */

/*  LDA     (input) INTEGER */
/*          The leading dimension of the array A.  LDA >= max(1,N). */

/*  W       (output) DOUBLE PRECISION array, dimension (N) */
/*          If INFO = 0, the eigenvalues in ascending order. */

/*  WORK    (workspace/output) DOUBLE PRECISION array, */
/*                                         dimension (LWORK) */
/*          On exit, if INFO = 0, WORK(1) returns the optimal LWORK. */

/*  LWORK   (input) INTEGER */
/*          The dimension of the array WORK. */
/*          If N <= 1,               LWORK must be at least 1. */
/*          If JOBZ = 'N' and N > 1, LWORK must be at least 2*N+1. */
/*          If JOBZ = 'V' and N > 1, LWORK must be at least */
/*                                                1 + 6*N + 2*N**2. */

/*          If LWORK = -1, then a workspace query is assumed; the routine */
/*          only calculates the optimal sizes of the WORK and IWORK */
/*          arrays, returns these values as the first entries of the WORK */
/*          and IWORK arrays, and no error message related to LWORK or */
/*          LIWORK is issued by XERBLA. */

/*  IWORK   (workspace/output) INTEGER array, dimension (MAX(1,LIWORK)) */
/*          On exit, if INFO = 0, IWORK(1) returns the optimal LIWORK. */

/*  LIWORK  (input) INTEGER */
/*          The dimension of the array IWORK. */
/*          If N <= 1,                LIWORK must be at least 1. */
/*          If JOBZ  = 'N' and N > 1, LIWORK must be at least 1. */
/*          If JOBZ  = 'V' and N > 1, LIWORK must be at least 3 + 5*N. */

/*          If LIWORK = -1, then a workspace query is assumed; the */
/*          routine only calculates the optimal sizes of the WORK and */
/*          IWORK arrays, returns these values as the first entries of */
/*          the WORK and IWORK arrays, and no error message related to */
/*          LWORK or LIWORK is issued by XERBLA. */

/*  INFO    (output) INTEGER */
/*          = 0:  successful exit */
/*          < 0:  if INFO = -i, the i-th argument had an illegal value */
/*          > 0:  if INFO = i and JOBZ = 'N', then the algorithm failed */
/*                to converge; i off-diagonal elements of an intermediate */
/*                tridiagonal form did not converge to zero; */
/*                if INFO = i and JOBZ = 'V', then the algorithm failed */
/*                to compute an eigenvalue while working on the submatrix */
/*                lying in rows and columns INFO/(N+1) through */
/*                mod(INFO,N+1). */

/*  Further Details */
/*  =============== */

/*  Based on contributions by */
/*     Jeff Rutter, Computer Science Division, University of California */
/*     at Berkeley, USA */
/*  Modified by Francoise Tisseur, University of Tennessee. */

/*  Modified description of INFO. Sven, 16 Feb 05. */
/*  ===================================================================== */

#include <math.h>
#include <stdbool.h>

#include <blasfeo_d_blas_api.h>



#if defined(FORTRAN_BLAS_API)
#define blasfeo_lapack_dormtr dormtr_
#define blasfeo_lapack_dstedc dstedc_
#define blasfeo_lapack_dsytrd dsytrd_
#define blasfeo_lapack_dsyevd dsyevd_
#endif



#define min(x, y) (((x) < (y)) ? (x) : (y))
#define max(x, y) (((x) > (y)) ? (x) : (y))



void dlacpy_(char *, int *, int *, double *, int *, double *, int *);
double dlamch_(char *);
double dlansy_(char *, char *, int *, double *, int *, double *);
void dlascl_(char *, int *, int *, double *, double *, int *, int *, double *, int *, int *);
void blasfeo_lapack_dormtr(char *, char *, char *, int *, int *, double *, int *, double *, double *, int *, double *, int *, int *);
void dscal_(int *, double *, double *, int *);
void blasfeo_lapack_dstedc(char *, int *, double *, double *, double *, int *, double *, int*, int*, int*, int*);
void dsterf_(int *, double *, double *, int *);
void blasfeo_lapack_dsytrd(char *, int *, double *, int *, double *, double *, double *, double *, int *, int *);
int ilaenv_(int *, char *, char *, int *, int *, int *, int *);
bool lsame_(char *, char *);
void xerbla_(char *, int *);



void blasfeo_lapack_dsyevd(char *jobz, char *uplo, int *pn, double *A, int *plda, double *w, double *work, int *lwork, int *iwork, int *liwork, int *info)
	{

	int n = *pn;
	int lda = *plda;

	int i_0 = 0;
	int i_1 = 1;
	int i_m1 = -1;
	double d_1 = 1.0;

	int liwmin, lwmin, lopt, liopt, iscale, inde, indtau, indwrk, llwork, indwk2, llwrk2, iinfo;
	double safmin, eps, smlnum, bignum, rmin, rmax, anrm, sigma;

	int i_t0;
	double d_t0;

	/* Parameter adjustments */

	/* Function Body */
	bool wantz = lsame_(jobz, "V");
	bool lower = lsame_(uplo, "L");
	bool lquery = *lwork == -1 || *liwork == -1;

	*info = 0;
	if (! (wantz || lsame_(jobz, "N")))
		{
		*info = -1;
		}
	else if (! (lower || lsame_(uplo, "U")))
		{
		*info = -2;
		}
	else if (n < 0)
		{
		*info = -3;
		}
	else if (lda < max(1,n))
		{
		*info = -5;
		}

	if (*info == 0)
		{
		if (n <= 1)
			{
			liwmin = 1;
			lwmin = 1;
			lopt = lwmin;
			liopt = liwmin;
			}
		else
			{
			if (wantz)
				{
				liwmin = 5*n + 3;
				lwmin = 2*n*n + 6*n + 1;
				}
			else
				{
				liwmin = 1;
				lwmin = 2*n + 1;
				}
			i_t0 = 2*n + ilaenv_(&i_1, "DSYTRD", uplo, &n, &i_m1, &i_m1, &i_m1);
			lopt = max(lwmin, i_t0);
			liopt = liwmin;
			}
		work[0] = (double) lopt;
		iwork[0] = liopt;

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
		xerbla_("DSYEVD", &i_t0);
		return;
		}
	else if (lquery)
		{
		return;
		}

/*	 Quick return if possible */

	if (n == 0)
		{
		return;
		}

	if (n == 1)
		{
		w[0] = A[0];
		if (wantz)
			{
			A[0] = 1.0;
			}
		return;
		}

/*	 Get machine constants. */

	safmin = dlamch_("Safe minimum");
	eps = dlamch_("Precision");
	smlnum = safmin / eps;
	bignum = 1.0 / smlnum;
	rmin = sqrt(smlnum);
	rmax = sqrt(bignum);

/*	 Scale matrix to allowable range, if necessary. */

	anrm = dlansy_("M", uplo, &n, &A[0], &lda, &work[0]);
	iscale = 0;
	if (anrm > 0.0 && anrm < rmin)
		{
		iscale = 1;
		sigma = rmin / anrm;
		}
	else if (anrm > rmax)
		{
		iscale = 1;
		sigma = rmax / anrm;
		}
	if (iscale == 1)
		{
		dlascl_(uplo, &i_0, &i_0, &d_1, &sigma, &n, &n, &A[0], &lda, info);
	}

/*	 Call DSYTRD to reduce symmetric matrix to tridiagonal form. */

	inde = 0; //1;
	indtau = inde + n;
	indwrk = indtau + n;
	llwork = *lwork - indwrk; // + 1;
	indwk2 = indwrk + n*n;
	llwrk2 = *lwork - indwk2; // + 1;

	blasfeo_lapack_dsytrd(uplo, &n, &A[0], &lda, &w[0], &work[inde], &work[indtau], &work[indwrk], &llwork, &iinfo);
//	lopt = (int) (2*n + work[indwrk]); // removed in 3.10 vs 3.2.1

/*	 For eigenvalues only, call DSTERF.  For eigenvectors, first call */
/*	 DSTEDC to generate the eigenvector matrix, WORK(INDWRK), of the */
/*	 tridiagonal matrix, then call DORMTR to multiply it by the */
/*	 Householder transformations stored in A. */

	if (! wantz)
		{
		dsterf_(&n, &w[0], &work[inde], info);
		}
	else
		{
		blasfeo_lapack_dstedc("I", &n, &w[0], &work[inde], &work[indwrk], &n, &work[indwk2], &llwrk2, &iwork[0], liwork, info);
		blasfeo_lapack_dormtr("L", uplo, "N", &n, &n, &A[0], &lda, &work[indtau], &work[indwrk], &n, &work[indwk2], &llwrk2, &iinfo);
		dlacpy_("A", &n, &n, &work[indwrk], &n, &A[0], &lda);
//		lopt = max(lopt, 2*n*n + 6*n + 1); // removed in 3.10 vs 3.2.1
		}

/*	 If matrix was scaled, then rescale eigenvalues appropriately. */

	if (iscale == 1)
		{
		d_t0 = 1.0 / sigma;
		dscal_(&n, &d_t0, &w[0], &i_1);
		}

	work[0] = (double) lopt;
	iwork[0] = liopt;

	return;

/*	 End of DSYEVD */

	} /* dsyevd_ */
