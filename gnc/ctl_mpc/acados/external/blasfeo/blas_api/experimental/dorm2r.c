/*  -- LAPACK routine (version 3.2) -- */
/*     Univ. of Tennessee, Univ. of California Berkeley and NAG Ltd.. */
/*     November 2006 */

/*  Purpose */
/*  ======= */

/*  DORM2R overwrites the general real m by n matrix C with */

/*        Q * C  if SIDE = 'L' and TRANS = 'N', or */

/*        Q'* C  if SIDE = 'L' and TRANS = 'T', or */

/*        C * Q  if SIDE = 'R' and TRANS = 'N', or */

/*        C * Q' if SIDE = 'R' and TRANS = 'T', */

/*  where Q is a real orthogonal matrix defined as the product of k */
/*  elementary reflectors */

/*        Q = H(1) H(2) . . . H(k) */

/*  as returned by DGEQRF. Q is of order m if SIDE = 'L' and of order n */
/*  if SIDE = 'R'. */

/*  Arguments */
/*  ========= */

/*  SIDE    (input) CHARACTER*1 */
/*          = 'L': apply Q or Q' from the Left */
/*          = 'R': apply Q or Q' from the Right */

/*  TRANS   (input) CHARACTER*1 */
/*          = 'N': apply Q  (No transpose) */
/*          = 'T': apply Q' (Transpose) */

/*  M       (input) INTEGER */
/*          The number of rows of the matrix C. M >= 0. */

/*  N       (input) INTEGER */
/*          The number of columns of the matrix C. N >= 0. */

/*  K       (input) INTEGER */
/*          The number of elementary reflectors whose product defines */
/*          the matrix Q. */
/*          If SIDE = 'L', M >= K >= 0; */
/*          if SIDE = 'R', N >= K >= 0. */

/*  A       (input) DOUBLE PRECISION array, dimension (LDA,K) */
/*          The i-th column must contain the vector which defines the */
/*          elementary reflector H(i), for i = 1,2,...,k, as returned by */
/*          DGEQRF in the first k columns of its array argument A. */
/*          A is modified by the routine but restored on exit. */

/*  LDA     (input) INTEGER */
/*          The leading dimension of the array A. */
/*          If SIDE = 'L', LDA >= max(1,M); */
/*          if SIDE = 'R', LDA >= max(1,N). */

/*  TAU     (input) DOUBLE PRECISION array, dimension (K) */
/*          TAU(i) must contain the scalar factor of the elementary */
/*          reflector H(i), as returned by DGEQRF. */

/*  C       (input/output) DOUBLE PRECISION array, dimension (LDC,N) */
/*          On entry, the m by n matrix C. */
/*          On exit, C is overwritten by Q*C or Q'*C or C*Q' or C*Q. */

/*  LDC     (input) INTEGER */
/*          The leading dimension of the array C. LDC >= max(1,M). */

/*  WORK    (workspace) DOUBLE PRECISION array, dimension */
/*                                   (N) if SIDE = 'L', */
/*                                   (M) if SIDE = 'R' */

/*  INFO    (output) INTEGER */
/*          = 0: successful exit */
/*          < 0: if INFO = -i, the i-th argument had an illegal value */

/*  ===================================================================== */

#include <math.h>
#include <stdbool.h>

#include <blasfeo_d_blas_api.h>



#if defined(FORTRAN_BLAS_API)
#define blasfeo_lapack_dlarf dlarf_
#define blasfeo_lapack_dlorm2r dorm2r0_
#endif



#define min(x, y) (((x) < (y)) ? (x) : (y))
#define max(x, y) (((x) > (y)) ? (x) : (y))



void blasfeo_lapack_dlarf(char *, int *, int *, double *, int *, double *, double *, int *, double *);
bool lsame_(char *, char *);
void xerbla_(char *, int *);



void blasfeo_lapack_dorm2r(char *side, char *trans, int *pm, int *pn, int *pk, double *A, int *plda, double *tau, double *C, int *pldc, double *work, int *info)
	{

	int m = *pm;
	int n = *pn;
	int k = *pk;
	int lda = *plda;
	int ldc = *pldc;

	int i_1 = 1;

	int mi, ni, ic, jc, nq;
	int i_t0, i_t1;
	double aii;

	int ii, i1, i2, i3;

    /* Function Body */
    *info = 0;
    bool left = lsame_(side, "L");
    bool notran = lsame_(trans, "N");

/*     NQ is the order of Q */

    if (left)
		{
		nq = m;
		}
	else
		{
		nq = n;
		}
	if (! left && ! lsame_(side, "R")) {
		*info = -1;
		}
	else if (! notran && ! lsame_(trans, "T"))
		{
		*info = -2;
		}
	else if (m < 0)
		{
		*info = -3;
		}
	else if (n < 0)
		{
		*info = -4;
		}
	else if (k < 0 || k > nq)
		{
		*info = -5;
		}
	else if (lda < max(1,nq))
		{
		*info = -7;
		}
	else if (ldc < max(1,m))
		{
		*info = -10;
		}
	if (*info != 0)
		{
		i_t0 = -(*info);
		xerbla_("DORM2R", &i_t0);
		return;
		}

/*     Quick return if possible */

    if (m == 0 || n == 0 || k == 0)
		{
		return;
		}

    if (left && ! notran || ! left && notran)
		{
		i1 = 1;
		i2 = k;
		i3 = 1;
		}
	else
		{
		i1 = k;
		i2 = 1;
		i3 = -1;
		}

    if (left)
		{
		ni = n;
		jc = 1;
		}
	else
		{
		mi = m;
		ic = 1;
		}

    i_t0 = i2;
    i_t1 = i3;
    for (ii = i1; i_t1 < 0 ? ii >= i_t0 : ii <= i_t0; ii += i_t1)
		{
		if (left)
			{

	/*           H(i) is applied to C(i:m,1:n) */

			mi = m - ii + 1;
			ic = ii;
			}
		else
			{

	/*           H(i) is applied to C(1:m,i:n) */

			ni = n - ii + 1;
			jc = ii;
			}

	/*        Apply H(i) */

		aii = A[ii-1+(ii-1)*lda];
		A[ii-1+(ii-1)*lda] = 1.0;
		blasfeo_lapack_dlarf(side, &mi, &ni, &A[ii-1+(ii-1)*lda], &i_1, &tau[ii-1], &C[ic-1+(jc-1)*ldc], &ldc, &work[0]);
		A[ii-1+(ii-1)*lda] = aii;
		}

    return;

/*     End of DORM2R */

	} /* dorm2r_ */

