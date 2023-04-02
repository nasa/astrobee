/*  -- LAPACK routine (version 3.2) -- */
/*	 Univ. of Tennessee, Univ. of California Berkeley and NAG Ltd.. */
/*	 November 2006 */

/*  Purpose */
/*  ======= */

/*  DSYTD2 reduces a real symmetric matrix A to symmetric tridiagonal */
/*  form T by an orthogonal similarity transformation: Q' * A * Q = T. */

/*  Arguments */
/*  ========= */

/*  UPLO	(input) CHARACTER*1 */
/*		  Specifies whether the upper or lower triangular part of the */
/*		  symmetric matrix A is stored: */
/*		  = 'U':  Upper triangular */
/*		  = 'L':  Lower triangular */

/*  N	   (input) INTEGER */
/*		  The order of the matrix A.  N >= 0. */

/*  A	   (input/output) DOUBLE PRECISION array, dimension (LDA,N) */
/*		  On entry, the symmetric matrix A.  If UPLO = 'U', the leading */
/*		  n-by-n upper triangular part of A contains the upper */
/*		  triangular part of the matrix A, and the strictly lower */
/*		  triangular part of A is not referenced.  If UPLO = 'L', the */
/*		  leading n-by-n lower triangular part of A contains the lower */
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

/*  INFO	(output) INTEGER */
/*		  = 0:  successful exit */
/*		  < 0:  if INFO = -i, the i-th argument had an illegal value. */

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
#define blasfeo_blas_dsymv dsymv_
#define blasfeo_blas_ddot ddot_
#define blasfeo_blas_daxpy daxpy_
#define blasfeo_lapack_dsytd2 dsytd2_
#endif



#define min(x, y) (((x) < (y)) ? (x) : (y))
#define max(x, y) (((x) > (y)) ? (x) : (y))



void dlarfg_(int *, double *, double *, int *, double *);
void dsyr2_(char *, int *, double *, double *, int *, double *, int *, double *, int *);
bool lsame_(char *, char *);
void xerbla_(char *, int *);



void blasfeo_lapack_dsytd2(char *uplo, int *pn, double *A, int *plda, double *d, double *e, double *tau, int *info)
	{

	int n = *pn;
	int lda = *plda;

	int i_1 = 1;
	double d_0 = 0.0;
	double d_m1 = -1.0;

	double taui, alpha;

	int i_t0;

	int ii;

	/* Function Body */
	*info = 0;
	bool upper = lsame_(uplo, "U");
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
	if (*info != 0)
		{
		i_t0 = -(*info);
		xerbla_("DSYTD2", &i_t0);
		return;
		}

/*	 Quick return if possible */

	if (n <= 0)
		{
		return;
		}

	if (upper)
		{

/*		Reduce the upper triangle of A */

		for (ii=n-1; ii>=1; ii--)
			{

/*		   Generate elementary reflector H(i) = I - tau * v * v' */
/*		   to annihilate A(1:i-1,i+1) */

			dlarfg_(&ii, &A[ii-1+ii*lda], &A[ii*lda], &i_1, &taui);
			e[ii-1] = A[ii-1+ii*lda];

			if (taui != 0.0)
				{

/*			  Apply H(i) from both sides to A(1:i,1:i) */

				A[ii-1+ii*lda] = 1.0;

/*			  Compute  x := tau * A * v  storing x in TAU(1:i) */

				blasfeo_blas_dsymv(uplo, &ii, &taui, &A[0], &lda, &A[0+ii*lda], &i_1, &d_0, &tau[0], &i_1);

/*			  Compute  w := x - 1/2 * tau * (x'*v) * v */

				alpha = -0.5 * taui * blasfeo_blas_ddot(&ii, &tau[0], &i_1, &A[0+ii*lda], &i_1);
				blasfeo_blas_daxpy(&ii, &alpha, &A[0+ii*lda], &i_1, &tau[0], &i_1);

/*			  Apply the transformation as A rank-2 update: */
/*				 A := A - v * w' - w * v' */

				dsyr2_(uplo, &ii, &d_m1, &A[0+ii*lda], &i_1, &tau[0], &i_1, &A[0], &lda);

				A[ii-1+ii*lda] = e[ii-1];
				}
			d[ii] = A[ii+ii*lda];
			tau[ii-1] = taui;
			}
		d[0] = A[0];
		}
	else
		{

/*		Reduce the lower triangle of A */

		for (ii=0; ii<n-1; ii++)
			{

/*		   Generate elementary reflector H(i) = I - tau * v * v' */
/*		   to annihilate A(i+2:n,i) */

			i_t0 = n - ii - 1;
			dlarfg_(&i_t0, &A[ii+1+ii*lda], &A[min(ii+2, n-1)+ii*lda], &i_1, &taui);
			e[ii] = A[ii+1+ii*lda];

			if (taui != 0.0)
				{

/*			  Apply H(i) from both sides to A(i+1:n,i+1:n) */

				A[ii+1+ii*lda] = 1.0;

/*			  Compute  x := tau * A * v  storing y in TAU(i:n-1) */

				i_t0 = n - ii - 1;
				blasfeo_blas_dsymv(uplo, &i_t0, &taui, &A[ii+1+(ii+1)*lda], &lda, &A[ii+1+ii*lda], &i_1, &d_0, &tau[ii], &i_1);

/*			  Compute  w := x - 1/2 * tau * (x'*v) * v */

				i_t0 = n - ii - 1;
				alpha = -0.5 * taui * blasfeo_blas_ddot(&i_t0, &tau[ii], &i_1, &A[ii+1+ii*lda], &i_1);
				i_t0 = n - ii - 1;
				blasfeo_blas_daxpy(&i_t0, &alpha, &A[ii+1+ii*lda], &i_1, &tau[ii], &i_1);

/*			  Apply the transformation as A rank-2 update: */
/*				 A := A - v * w' - w * v' */

				i_t0 = n - ii - 1;
				dsyr2_(uplo, &i_t0, &d_m1, &A[ii+1+ii*lda], &i_1, &tau[ii], &i_1, &A[ii+1+(ii+1)*lda], &lda);

				A[ii+1+ii*lda] = e[ii];
				}
			d[ii] = A[ii+ii*lda];
			tau[ii] = taui;
		}
		d[n-1] = A[n-1+(n-1)*lda];
		}

	return;

/*	 End of DSYTD2 */

} /* dsytd2_ */
