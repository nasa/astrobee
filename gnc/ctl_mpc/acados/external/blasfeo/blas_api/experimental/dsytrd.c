/*  -- LAPACK routine (version 3.2) -- */
/*	 Univ. of Tennessee, Univ. of California Berkeley and NAG Ltd.. */
/*	 November 2006 */

/*  Purpose */
/*  ======= */

/*  DSYTRD reduces a real symmetric matrix A to real symmetric */
/*  tridiagonal form T by an orthogonal similarity transformation: */
/*  Q**T * A * Q = T. */

/*  Arguments */
/*  ========= */

/*  UPLO	(input) CHARACTER*1 */
/*		  = 'U':  Upper triangle of A is stored; */
/*		  = 'L':  Lower triangle of A is stored. */

/*  N	   (input) INTEGER */
/*		  The order of the matrix A.  N >= 0. */

/*  A	   (input/output) DOUBLE PRECISION array, dimension (LDA,N) */
/*		  On entry, the symmetric matrix A.  If UPLO = 'U', the leading */
/*		  N-by-N upper triangular part of A contains the upper */
/*		  triangular part of the matrix A, and the strictly lower */
/*		  triangular part of A is not referenced.  If UPLO = 'L', the */
/*		  leading N-by-N lower triangular part of A contains the lower */
/*		  triangular part of the matrix A, and the strictly upper */
/*		  triangular part of A is not referenced. */
/*		  On exit, if UPLO = 'U', the diagonal and first superdiagonal */
/*		  of A are overwritten by the corresponding elements of the */
/*		  tridiagonal matrix T, and the elements above the first */
/*		  superdiagonal, with the array TAU, represent the orthogonal */
/*		  matrix Q as a product of elementary reflectors; if UPLO */
/*		  = 'L', the diagonal and first subdiagonal of A are over- */
/*		  written by the corresponding elements of the tridiagonal */
/*		  matrix T, and the elements below the first subdiagonal, with */
/*		  the array TAU, represent the orthogonal matrix Q as a product */
/*		  of elementary reflectors. See Further Details. */

/*  LDA	 (input) INTEGER */
/*		  The leading dimension of the array A.  LDA >= max(1,N). */

/*  D	   (output) DOUBLE PRECISION array, dimension (N) */
/*		  The diagonal elements of the tridiagonal matrix T: */
/*		  D(i) = A(i,i). */

/*  E	   (output) DOUBLE PRECISION array, dimension (N-1) */
/*		  The off-diagonal elements of the tridiagonal matrix T: */
/*		  E(i) = A(i,i+1) if UPLO = 'U', E(i) = A(i+1,i) if UPLO = 'L'. */

/*  TAU	 (output) DOUBLE PRECISION array, dimension (N-1) */
/*		  The scalar factors of the elementary reflectors (see Further */
/*		  Details). */

/*  WORK	(workspace/output) DOUBLE PRECISION array, dimension (MAX(1,LWORK)) */
/*		  On exit, if INFO = 0, WORK(1) returns the optimal LWORK. */

/*  LWORK   (input) INTEGER */
/*		  The dimension of the array WORK.  LWORK >= 1. */
/*		  For optimum performance LWORK >= N*NB, where NB is the */
/*		  optimal blocksize. */

/*		  If LWORK = -1, then a workspace query is assumed; the routine */
/*		  only calculates the optimal size of the WORK array, returns */
/*		  this value as the first entry of the WORK array, and no error */
/*		  message related to LWORK is issued by XERBLA. */

/*  INFO	(output) INTEGER */
/*		  = 0:  successful exit */
/*		  < 0:  if INFO = -i, the i-th argument had an illegal value */

/*  Further Details */
/*  =============== */

/*  If UPLO = 'U', the matrix Q is represented as a product of elementary */
/*  reflectors */

/*	 Q = H(n-1) . . . H(2) H(1). */

/*  Each H(i) has the form */

/*	 H(i) = I - tau * v * v' */

/*  where tau is a real scalar, and v is a real vector with */
/*  v(i+1:n) = 0 and v(i) = 1; v(1:i-1) is stored on exit in */
/*  A(1:i-1,i+1), and tau in TAU(i). */

/*  If UPLO = 'L', the matrix Q is represented as a product of elementary */
/*  reflectors */

/*	 Q = H(1) H(2) . . . H(n-1). */

/*  Each H(i) has the form */

/*	 H(i) = I - tau * v * v' */

/*  where tau is a real scalar, and v is a real vector with */
/*  v(1:i) = 0 and v(i+1) = 1; v(i+2:n) is stored on exit in A(i+2:n,i), */
/*  and tau in TAU(i). */

/*  The contents of A on exit are illustrated by the following examples */
/*  with n = 5: */

/*  if UPLO = 'U':					   if UPLO = 'L': */

/*	(  d   e   v2  v3  v4 )			  (  d				  ) */
/*	(	  d   e   v3  v4 )			  (  e   d			  ) */
/*	(		  d   e   v4 )			  (  v1  e   d		  ) */
/*	(			  d   e  )			  (  v1  v2  e   d	  ) */
/*	(				  d  )			  (  v1  v2  v3  e   d  ) */

/*  where d and e denote diagonal and off-diagonal elements of T, and vi */
/*  denotes an element of the vector defining H(i). */

/*  ===================================================================== */

#include <math.h>
#include <stdbool.h>

#include <blasfeo_d_blas_api.h>



#if defined(FORTRAN_BLAS_API)
#define blasfeo_lapack_dlatrd dlatrd_
#define blasfeo_blas_dsyr2k dsyr2k_
#define blasfeo_lapack_dsytd2 dsytd2_
#define blasfeo_lapack_dsytrd dsytrd_
#endif



#define min(x, y) (((x) < (y)) ? (x) : (y))
#define max(x, y) (((x) > (y)) ? (x) : (y))



void blasfeo_lapack_dlatrd(char *, int *, int *, double *, int *, double *, double *, double *, int *);
void blasfeo_lapack_dsytd2(char *, int *, double *, int *, double *, double *, double *, int *);
int ilaenv_(int *, char *, char *, int *, int *, int *, int *);
bool lsame_(char *, char *);
void xerbla_(char *, int *);



void blasfeo_lapack_dsytrd(char *uplo, int *pn, double *A, int *plda, double *d, double *e, double *tau, double * work, int *lwork, int *info)
	{

	int n = *pn;
	int lda = *plda;

//	int i_0 = 0;
	int i_1 = 1;
	int i_2 = 2;
	int i_3 = 3;
	int i_m1 = -1;
	double d_1 = 1.0;
	double d_m1 = -1.0;

	int nb, lwkopt, nx, iws, ldwork, nbmin, iinfo;

	int i_t0;

	int ii, jj, kk;

	/* Function Body */
	*info = 0;
	bool upper = lsame_(uplo, "U");
	bool lquery = *lwork == -1;
	if (! upper && ! lsame_(uplo, "L"))
		{
		*info = -1;
		}
	else if (n < 0)
		{
		*info = -2;
		}
	else if (lda < max(1,n))
		{
		*info = -4;
		}
	else if (*lwork < 1 && ! lquery)
		{
		*info = -9;
		}

	if (*info == 0)
		{

/*		Determine the block size. */

		nb = ilaenv_(&i_1, "DSYTRD", uplo, &n, &i_m1, &i_m1, &i_m1);
		lwkopt = n * nb;
		work[0] = (double) lwkopt;
		}

	if (*info != 0)
		{
		i_t0 = -(*info);
		xerbla_("DSYTRD", &i_t0);
		return;
		}
	else if (lquery)
		{
		return;
		}

/*	 Quick return if possible */

	if (n == 0)
		{
		work[0] = 1.;
		return;
		}

	nx = n;
	iws = 1;
	if (nb > 1 && nb < n)
		{

/*		Determine when to cross over from blocked to unblocked code */
/*		(last block is always handled by unblocked code). */

		i_t0 = ilaenv_(&i_3, "DSYTRD", uplo, &n, &i_m1, &i_m1, &i_m1);
		nx = max(nb, i_t0);
		if (nx < n)
			{

/*		   Determine if workspace is large enough for blocked code. */

			ldwork = n;
			iws = ldwork * nb;
			if (*lwork < iws)
				{

/*			  Not enough workspace to use optimal NB:  determine the */
/*			  minimum value of NB, and reduce NB or force use of */
/*			  unblocked code by setting NX = N. */

				i_t0 = *lwork / ldwork;
				nb = max(i_t0, 1);
				nbmin = ilaenv_(&i_2, "DSYTRD", uplo, &n, &i_m1, &i_m1, &i_m1);
				if (nb < nbmin)
					{
					nx = n;
					}
				}
			}
		else
			{
			nx = n;
			}
		}
	else
		{
		nb = 1;
		}

	if (upper)
		{

/*		Reduce the upper triangle of A. */
/*		Columns 1:kk are handled by the unblocked method. */

		kk = n - (n-nx+nb-1) / nb * nb;
		for (ii=n-nb; ii>=kk; ii-=nb)
			{

/*		   Reduce columns i:i+nb-1 to tridiagonal form and form the */
/*		   matrix W which is needed to update the unreduced part of */
/*		   the matrix */

			i_t0 = ii+nb;
			blasfeo_lapack_dlatrd(uplo, &i_t0, &nb, &A[0], &lda, &e[0], &tau[0], &work[0], &ldwork);

/*		   Update the unreduced submatrix A(1:i-1,1:i-1), using an */
/*		   update of the form:  A := A - V*W' - W*V' */

			i_t0 = ii;
			blasfeo_blas_dsyr2k(uplo, "No transpose", &i_t0, &nb, &d_m1, &A[ii*lda], &lda, &work[0], &ldwork, &d_1, &A[0], &lda);

/*		   Copy superdiagonal elements back into A, and diagonal */
/*		   elements into D */

			for (jj=ii; jj<ii+nb; jj++)
				{
				A[jj-1+jj*lda] = e[jj-1];
				d[jj] = A[jj+jj*lda];
				}
			}

/*		Use unblocked code to reduce the last or only block */

		blasfeo_lapack_dsytd2(uplo, &kk, &A[0], &lda, &d[0], &e[0], &tau[0], &iinfo);
		}
	else
		{

/*		Reduce the lower triangle of A */

	for (ii=0; ii<n-nx; ii+=nb)
		{

/*		   Reduce columns i:i+nb-1 to tridiagonal form and form the */
/*		   matrix W which is needed to update the unreduced part of */
/*		   the matrix */

		i_t0 = n-ii;
		blasfeo_lapack_dlatrd(uplo, &i_t0, &nb, &A[ii+ii*lda], &lda, &e[ii], &tau[ii], &work[0], &ldwork);

/*		   Update the unreduced submatrix A(i+ib:n,i+ib:n), using */
/*		   an update of the form:  A := A - V*W' - W*V' */

		i_t0 = n-ii-nb;
		blasfeo_blas_dsyr2k(uplo, "No transpose", &i_t0, &nb, &d_m1, &A[ii+nb+ii*lda], &lda, &work[nb], &ldwork, &d_1, &A[ii+nb+(ii+nb)*lda], &lda);

/*		   Copy subdiagonal elements back into A, and diagonal */
/*		   elements into D */

		for (jj=ii; jj<ii+nb; jj++)
			{
			A[jj+1+jj*lda] = e[jj];
			d[jj] = A[jj+jj*lda];
			}
		}

/*		Use unblocked code to reduce the last or only block */

	i_t0 = n-ii;
	blasfeo_lapack_dsytd2(uplo, &i_t0, &A[ii+ii*lda], &lda, &d[ii], &e[ii], &tau[ii], &iinfo);
	}

	work[0] = (double) lwkopt;
	return;

/*	 End of DSYTRD */

	} /* dsytrd_ */
