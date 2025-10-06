/*  -- LAPACK auxiliary routine (version 3.2) -- */
/*     Univ. of Tennessee, Univ. of California Berkeley and NAG Ltd.. */
/*     November 2006 */

/*     .. Scalar Arguments .. */
/*     .. */
/*     .. Array Arguments .. */
/*     .. */

/*  Purpose */
/*  ======= */

/*  DLATRD reduces NB rows and columns of a real symmetric matrix A to */
/*  symmetric tridiagonal form by an orthogonal similarity */
/*  transformation Q' * A * Q, and returns the matrices V and W which are */
/*  needed to apply the transformation to the unreduced part of A. */

/*  If UPLO = 'U', DLATRD reduces the last NB rows and columns of a */
/*  matrix, of which the upper triangle is supplied; */
/*  if UPLO = 'L', DLATRD reduces the first NB rows and columns of a */
/*  matrix, of which the lower triangle is supplied. */

/*  This is an auxiliary routine called by DSYTRD. */

/*  Arguments */
/*  ========= */

/*  UPLO    (input) CHARACTER*1 */
/*          Specifies whether the upper or lower triangular part of the */
/*          symmetric matrix A is stored: */
/*          = 'U': Upper triangular */
/*          = 'L': Lower triangular */

/*  N       (input) INTEGER */
/*          The order of the matrix A. */

/*  NB      (input) INTEGER */
/*          The number of rows and columns to be reduced. */

/*  A       (input/output) DOUBLE PRECISION array, dimension (LDA,N) */
/*          On entry, the symmetric matrix A.  If UPLO = 'U', the leading */
/*          n-by-n upper triangular part of A contains the upper */
/*          triangular part of the matrix A, and the strictly lower */
/*          triangular part of A is not referenced.  If UPLO = 'L', the */
/*          leading n-by-n lower triangular part of A contains the lower */
/*          triangular part of the matrix A, and the strictly upper */
/*          triangular part of A is not referenced. */
/*          On exit: */
/*          if UPLO = 'U', the last NB columns have been reduced to */
/*            tridiagonal form, with the diagonal elements overwriting */
/*            the diagonal elements of A; the elements above the diagonal */
/*            with the array TAU, represent the orthogonal matrix Q as a */
/*            product of elementary reflectors; */
/*          if UPLO = 'L', the first NB columns have been reduced to */
/*            tridiagonal form, with the diagonal elements overwriting */
/*            the diagonal elements of A; the elements below the diagonal */
/*            with the array TAU, represent the  orthogonal matrix Q as a */
/*            product of elementary reflectors. */
/*          See Further Details. */

/*  LDA     (input) INTEGER */
/*          The leading dimension of the array A.  LDA >= (1,N). */

/*  E       (output) DOUBLE PRECISION array, dimension (N-1) */
/*          If UPLO = 'U', E(n-nb:n-1) contains the superdiagonal */
/*          elements of the last NB columns of the reduced matrix; */
/*          if UPLO = 'L', E(1:nb) contains the subdiagonal elements of */
/*          the first NB columns of the reduced matrix. */

/*  TAU     (output) DOUBLE PRECISION array, dimension (N-1) */
/*          The scalar factors of the elementary reflectors, stored in */
/*          TAU(n-nb:n-1) if UPLO = 'U', and in TAU(1:nb) if UPLO = 'L'. */
/*          See Further Details. */

/*  W       (output) DOUBLE PRECISION array, dimension (LDW,NB) */
/*          The n-by-nb matrix W required to update the unreduced part */
/*          of A. */

/*  LDW     (input) INTEGER */
/*          The leading dimension of the array W. LDW >= max(1,N). */

/*  Further Details */
/*  =============== */

/*  If UPLO = 'U', the matrix Q is represented as a product of elementary */
/*  reflectors */

/*     Q = H(n) H(n-1) . . . H(n-nb+1). */

/*  Each H(i) has the form */

/*     H(i) = I - tau * v * v' */

/*  where tau is a real scalar, and v is a real vector with */
/*  v(i:n) = 0 and v(i-1) = 1; v(1:i-1) is stored on exit in A(1:i-1,i), */
/*  and tau in TAU(i-1). */

/*  If UPLO = 'L', the matrix Q is represented as a product of elementary */
/*  reflectors */

/*     Q = H(1) H(2) . . . H(nb). */

/*  Each H(i) has the form */

/*     H(i) = I - tau * v * v' */

/*  where tau is a real scalar, and v is a real vector with */
/*  v(1:i) = 0 and v(i+1) = 1; v(i+1:n) is stored on exit in A(i+1:n,i), */
/*  and tau in TAU(i). */

/*  The elements of the vectors v together form the n-by-nb matrix V */
/*  which is needed, with W, to apply the transformation to the unreduced */
/*  part of the matrix, using a symmetric rank-2k update of the form: */
/*  A := A - V*W' - W*V'. */

/*  The contents of A on exit are illustrated by the following examples */
/*  with n = 5 and nb = 2: */

/*  if UPLO = 'U':                       if UPLO = 'L': */

/*    (  a   a   a   v4  v5 )              (  d                  ) */
/*    (      a   a   v4  v5 )              (  1   d              ) */
/*    (          a   1   v5 )              (  v1  1   a          ) */
/*    (              d   1  )              (  v1  v2  a   a      ) */
/*    (                  d  )              (  v1  v2  a   a   a  ) */

/*  where d denotes a diagonal element of the reduced matrix, a denotes */
/*  an element of the original matrix that is unchanged, and vi denotes */
/*  an element of the vector defining H(i). */

/*  ===================================================================== */

#include <math.h>
#include <stdbool.h>

#include <blasfeo_d_blas_api.h>



#if defined(FORTRAN_BLAS_API)
#define blasfeo_blas_dgemv dgemv_
#define blasfeo_blas_dsymv dsymv_
#define blasfeo_blas_ddot ddot_
#define blasfeo_blas_daxpy daxpy_
#define blasfeo_lapack_dlatrd dlatrd_
#endif



#define min(x, y) (((x) < (y)) ? (x) : (y))
#define max(x, y) (((x) > (y)) ? (x) : (y))



void dlarfg_(int *, double *, double *, int *, double *);
void dscal_(int *, double *, double *, int *);
bool lsame_(char *, char *);



void blasfeo_lapack_dlatrd(char *uplo, int *pn, int *pnb, double *A, int *plda, double *e, double *tau, double *w, int *pldw)
	{

	int n = *pn;
	int nb = *pnb;
	int lda = *plda;
	int ldw = *pldw;

	int i_1 = 1;
	double d_0 = 0.0;
	double d_1 = 1.0;
	double d_m1 = -1.0;

	int iw;
	double alpha;

	int i_t0, i_t1;

	int ii;

    /* Function Body */
    if (n <= 0)
		{
		return;
		}

    if (lsame_(uplo, "U"))
		{

/*        Reduce last NB columns of upper triangle */

		for (ii=n; ii>=n-nb+1; ii--)
			{
			iw = ii-n+nb;
			if (ii<n)
				{

/*              Update A(1:i,i) */

				i_t0 = n-ii;
				blasfeo_blas_dgemv("No transpose", &ii, &i_t0, &d_m1, &A[0+ii*lda], &lda, &w[ii-1+iw*ldw], &ldw, &d_1, &A[0+(ii-1)*lda], &i_1);
				i_t0 = n-ii;
				blasfeo_blas_dgemv("No transpose", &ii, &i_t0, &d_m1, &w[0+iw*ldw], &ldw, &A[ii-1+ii*lda], &lda, &d_1, &A[0+(ii-1)*lda], &i_1);
				}
			if (ii > 1)
				{

/*              Generate elementary reflector H(i) to annihilate */
/*              A(1:i-2,i) */

				i_t0 = ii-1;
				dlarfg_(&i_t0, &A[ii-2+(ii-1)*lda], &A[0+(ii-1)*lda], &i_1, &tau[ii-2]);
				e[ii-2] = A[ii-2+(ii-1)*lda];
				A[ii-2+(ii-1)*lda] = 1.0;

/*              Compute W(1:i-1,i) */

				i_t0 = ii-1;
				blasfeo_blas_dsymv("Upper", &i_t0, &d_1, &A[0], &lda, &A[0+(ii-1)*lda], &i_1, &d_0, &w[0+(iw-1)*ldw], &i_1);
				if (ii < n)
					{
					i_t0 = ii - 1;
					i_t1 = n - ii;
					blasfeo_blas_dgemv("Transpose", &i_t0, &i_t1, &d_1, &w[0+iw*ldw], &ldw, &A[0+(ii-1)*lda], &i_1, &d_0, &w[ii+(iw-1)*ldw], &i_1);
					i_t0 = ii - 1;
					i_t1 = n - ii;
					blasfeo_blas_dgemv("No transpose", &i_t0, &i_t1, &d_m1, &A[0+ii*lda], &lda, &w[ii+(iw-1)*ldw], &i_1, &d_1, &w[0+(iw-1)*ldw], &i_1);
					i_t0 = ii - 1;
					i_t1 = n - ii;
					blasfeo_blas_dgemv("Transpose", &i_t0, &i_t1, &d_1, &A[0+ii*lda], &lda, &A[0+(ii-1)*lda], &i_1, &d_0, &w[ii+(iw-1)*ldw], &i_1);
					i_t0 = ii - 1;
					i_t1 = n - ii;
					blasfeo_blas_dgemv("No transpose", &i_t0, &i_t1, &d_m1, &w[0+iw*ldw+1], &ldw, &w[ii+(iw-1)*ldw], &i_1, &d_1, &w[0+(iw-1)*ldw], &i_1);
					}
				i_t0 = ii - 1;
				dscal_(&i_t0, &tau[ii-2], &w[0+(iw-1)*ldw], &i_1);
				i_t0 = ii - 1;
				alpha = -0.5 * tau[ii-2] * blasfeo_blas_ddot(&i_t0, &w[0+(iw-1)*ldw], &i_1, &A[0+(ii-1)*lda], &i_1);
				i_t0 = ii - 1;
				blasfeo_blas_daxpy(&i_t0, &alpha, &A[0+(ii-1)*lda], &i_1, &w[0+(iw-1)*ldw], &i_1);
				}

			}
		}
	else
		{

/*        Reduce first NB columns of lower triangle */

		for (ii=0; ii<nb; ii++)
			{

/*           Update A(i:n,i) */

			i_t0 = n - ii;
			i_t1 = ii;
			blasfeo_blas_dgemv("No transpose", &i_t0, &i_t1, &d_m1, &A[ii], &lda, &w[ii], &ldw, &d_1, &A[ii+ii*lda], &i_1);
			i_t0 = n - ii;
			i_t1 = ii;
			blasfeo_blas_dgemv("No transpose", &i_t0, &i_t1, &d_m1, &w[ii], &ldw, &A[ii], &lda, &d_1, &A[ii+ii*lda], &i_1);
			if (ii < n-1)
				{

/*              Generate elementary reflector H(i) to annihilate */
/*              A(i+2:n,i) */

				i_t0 = n - ii - 1;
				i_t1 = ii + 2;
				dlarfg_(&i_t0, &A[ii+1+ii*lda], &A[min(i_t1, n-1)+ii*lda], &i_1, &tau[ii]);
				e[ii] = A[ii+1+ii*lda];
				A[ii+1+ii*lda] = 1.0;

/*              Compute W(i+1:n,i) */

				i_t0 = n - ii - 1;
				blasfeo_blas_dsymv("Lower", &i_t0, &d_1, &A[ii+1+(ii+1)*lda], &lda, &A[ii+1+ii*lda], &i_1, &d_0, &w[ii+1+ii*ldw], &i_1);
				i_t0 = n - ii - 1;
				i_t1 = ii;
				blasfeo_blas_dgemv("Transpose", &i_t0, &i_t1, &d_1, &w[ii+1], &ldw, &A[ii+1+ii*lda], &i_1, &d_0, &w[0+ii*ldw], &i_1);
				i_t0 = n - ii - 1;
				i_t1 = ii;
				blasfeo_blas_dgemv("No transpose", &i_t0, &i_t1, &d_m1, &A[ii+1], &lda, &w[0+ii*ldw], &i_1, &d_1, &w[ii+1+ii*ldw], &i_1);
				i_t0 = n - ii - 1;
				i_t1 = ii;
				blasfeo_blas_dgemv("Transpose", &i_t0, &i_t1, &d_1, &A[ii+1], &lda, &A[ii+1+ii*lda], &i_1, &d_0, &w[0+ii*ldw], &i_1);
				i_t0 = n - ii - 1;
				i_t1 = ii;
				blasfeo_blas_dgemv("No transpose", &i_t0, &i_t1, &d_m1, &w[ii+1], &ldw, &w[0+ii*ldw], &i_1, &d_1, &w[ii+1+ii*ldw], &i_1);
				i_t0 = n - ii - 1;
				dscal_(&i_t0, &tau[ii], &w[ii+1+ii*ldw], &i_1);
				i_t0 = n - ii - 1;
				alpha = -0.5 * tau[ii] * blasfeo_blas_ddot(&i_t0, &w[ii+1+ii*ldw], &i_1, &A[ii+1+ii*lda], &i_1);
				i_t0 = n - ii - 1;
				blasfeo_blas_daxpy(&i_t0, &alpha, &A[ii+1+ii*lda], &i_1, &w[ii+1+ii*ldw], &i_1);
				}

			}
		}

    return;

/*     End of DLATRD */

	} /* dlatrd_ */
