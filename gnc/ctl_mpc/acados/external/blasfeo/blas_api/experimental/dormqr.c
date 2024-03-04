/*  -- LAPACK routine (version 3.2) -- */
/*	 Univ. of Tennessee, Univ. of California Berkeley and NAG Ltd.. */
/*	 November 2006 */

/*  Purpose */
/*  ======= */

/*  DORMQR overwrites the general real M-by-N matrix C with */

/*				  SIDE = 'L'	 SIDE = 'R' */
/*  TRANS = 'N':	  Q * C		  C * Q */
/*  TRANS = 'T':	  Q**T * C	   C * Q**T */

/*  where Q is a real orthogonal matrix defined as the product of k */
/*  elementary reflectors */

/*		Q = H(1) H(2) . . . H(k) */

/*  as returned by DGEQRF. Q is of order M if SIDE = 'L' and of order N */
/*  if SIDE = 'R'. */

/*  Arguments */
/*  ========= */

/*  SIDE	(input) CHARACTER*1 */
/*		  = 'L': apply Q or Q**T from the Left; */
/*		  = 'R': apply Q or Q**T from the Right. */

/*  TRANS   (input) CHARACTER*1 */
/*		  = 'N':  No transpose, apply Q; */
/*		  = 'T':  Transpose, apply Q**T. */

/*  M	   (input) INTEGER */
/*		  The number of rows of the matrix C. M >= 0. */

/*  N	   (input) INTEGER */
/*		  The number of columns of the matrix C. N >= 0. */

/*  K	   (input) INTEGER */
/*		  The number of elementary reflectors whose product defines */
/*		  the matrix Q. */
/*		  If SIDE = 'L', M >= K >= 0; */
/*		  if SIDE = 'R', N >= K >= 0. */

/*  A	   (input) DOUBLE PRECISION array, dimension (LDA,K) */
/*		  The i-th column must contain the vector which defines the */
/*		  elementary reflector H(i), for i = 1,2,...,k, as returned by */
/*		  DGEQRF in the first k columns of its array argument A. */
/*		  A is modified by the routine but restored on exit. */

/*  LDA	 (input) INTEGER */
/*		  The leading dimension of the array A. */
/*		  If SIDE = 'L', LDA >= max(1,M); */
/*		  if SIDE = 'R', LDA >= max(1,N). */

/*  TAU	 (input) DOUBLE PRECISION array, dimension (K) */
/*		  TAU(i) must contain the scalar factor of the elementary */
/*		  reflector H(i), as returned by DGEQRF. */

/*  C	   (input/output) DOUBLE PRECISION array, dimension (LDC,N) */
/*		  On entry, the M-by-N matrix C. */
/*		  On exit, C is overwritten by Q*C or Q**T*C or C*Q**T or C*Q. */

/*  LDC	 (input) INTEGER */
/*		  The leading dimension of the array C. LDC >= max(1,M). */

/*  WORK	(workspace/output) DOUBLE PRECISION array, dimension (MAX(1,LWORK)) */
/*		  On exit, if INFO = 0, WORK(1) returns the optimal LWORK. */

/*  LWORK   (input) INTEGER */
/*		  The dimension of the array WORK. */
/*		  If SIDE = 'L', LWORK >= max(1,N); */
/*		  if SIDE = 'R', LWORK >= max(1,M). */
/*		  For optimum performance LWORK >= N*NB if SIDE = 'L', and */
/*		  LWORK >= M*NB if SIDE = 'R', where NB is the optimal */
/*		  blocksize. */

/*		  If LWORK = -1, then a workspace query is assumed; the routine */
/*		  only calculates the optimal size of the WORK array, returns */
/*		  this value as the first entry of the WORK array, and no error */
/*		  message related to LWORK is issued by XERBLA. */

/*  INFO	(output) INTEGER */
/*		  = 0:  successful exit */
/*		  < 0:  if INFO = -i, the i-th argument had an illegal value */

/*  ===================================================================== */

#include <math.h>
#include <stdbool.h>

#include <blasfeo_d_blas_api.h>



#if defined(FORTRAN_BLAS_API)
#define blasfeo_lapack_dlarfb dlarfb_
#define blasfeo_lapack_dlarft dlarft_
#define blasfeo_lapack_dorm2r dorm2r_
#define blasfeo_lapack_dormqr dormqr_
#endif



#define min(x, y) (((x) < (y)) ? (x) : (y))
#define max(x, y) (((x) > (y)) ? (x) : (y))



void blasfeo_lapack_dlarfb(char *, char *, char *, char *, int *, int *, int *, double *, int *, double *, int *, double *, int *, double *, int *);
void blasfeo_lapack_dlarft(char *, char *, int *, int *, double *, int *, double *, double *, int *);
void blasfeo_lapack_dorm2r(char *, char *, int *, int *, int *, double *, int *, double *, double *, int *, double *, int *);
int ilaenv_(int *, char *, char *, int *, int *, int *, int *);
bool lsame_(char *, char *);
void xerbla_(char *, int *);



void blasfeo_lapack_dormqr(char *side, char *trans, int *pm, int *pn, int *pk, double *A, int *plda, double *tau, double *C, int *pldc, double *work, int *lwork, int *info)
	{

	int m = *pm;
	int n = *pn;
	int k = *pk;
	int lda = *plda;
	int ldc = *pldc;

	int i_1 = 1;
	int i_2 = 2;
//	int i_65 = 65;
	int i_m1 = -1;

	int nbmax = 64; // XXX as in lapack 3.10
	int ldt = nbmax+1; // XXX as in lapack 3.10
	int tsize = ldt*nbmax; // XXX as in lapack 3.10

	char s_t0[2];
	int i_t0, i_t1, i_t2;

//	double t[4160]	/* was [65][64] */;

	int nb, nq, nw, lwkopt, ldwork, iws, iwt, nbmin, mi, ni, ic, jc, ib, iinfo;
	int ii, i1, i2, i3;

	/* Function Body */
	*info = 0;
	bool left = lsame_(side, "L");
	bool notran = lsame_(trans, "N");
	bool lquery = *lwork == -1;

/*	 NQ is the order of Q and NW is the minimum dimension of WORK */

	if (left)
		{
		nq = m;
		nw = max(1,n); // XXX as in lapack 3.10
		}
	else
		{
		nq = n;
		nw = max(1,m); // XXX as in lapack 3.10
		}
	if (! left && ! lsame_(side, "R"))
		{
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
	else if (*lwork < nw && ! lquery) // XXX as in lapack 3.10
		{
		*info = -12;
		}

	if (*info == 0)
		{

/*		Determine the block size.  NB may be at most NBMAX, where NBMAX */
/*		is used to define the local array T. */

		// concatenate 'side' and 'trans' strings
		s_t0[0] = side[0];
		s_t0[1] = trans[0];

		i_t1 = ilaenv_(&i_1, "DORMQR", s_t0, &m, &n, &k, &i_m1);
		nb = min(nbmax,i_t1); // XXX as in lapack 3.10
		lwkopt = nw*nb + tsize; // XXX as in lapack 3.10
		work[0] = (double) lwkopt;
		}

	if (*info != 0)
		{
		i_t0 = -(*info);
		xerbla_("DORMQR", &i_t0);
		return;
		}
	else if (lquery)
		{
		return;
		}

/*	 Quick return if possible */

	if (m == 0 || n == 0 || k == 0)
		{
		work[0] = 1.;
		return;
		}

	nbmin = 2;
	ldwork = nw;
	if (nb > 1 && nb < k)
		{
//		iws = nw * nb;
//		if (*lwork < iws)
		if (*lwork < lwkopt) // XXX as in lapack 3.10
			{
			nb = (*lwork - tsize) / ldwork; // XXX as in lapack 3.10

			// concatenate 'side' and 'trans' strings
			s_t0[0] = side[0];
			s_t0[1] = trans[0];

			i_t1 = ilaenv_(&i_2, "DORMQR", s_t0, &m, &n, &k, &i_m1);
			nbmin = max(2,i_t1);
			}
		}
//	else
//		{
//		iws = nw;
//		}

	if (nb < nbmin || nb >= k)
		{

/*		Use unblocked code */

		blasfeo_lapack_dorm2r(side, trans, &m, &n, &k, &A[0], &lda, &tau[0], &C[0], &ldc, &work[0], &iinfo);
		}
	else
		{

/*		Use blocked code */

		iwt = 0 + nw*nb; // XXX as in lapack 3.10

		if (left && ! notran || ! left && notran)
			{
			i1 = 1;
			i2 = k;
			i3 = nb;
			}
		else
			{
			i1 = (k - 1) / nb * nb + 1;
			i2 = 1;
			i3 = -nb;
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
			ib = min(nb,k - ii + 1);

/*		   Form the triangular factor of the block reflector */
/*		   H = H(i) H(i+1) . . . H(i+ib-1) */

			i_t2 = nq - ii + 1; // XXX as in lapack 3.10
			blasfeo_lapack_dlarft("Forward", "Columnwise", &i_t2, &ib, &A[ii-1+(ii-1)*lda], &lda, &tau[ii-1], &work[iwt], &ldt); // XXX as in lapack 3.10
			if (left)
				{

/*			  H or H' is applied to C(i:m,1:n) */

				mi = m - ii + 1;
				ic = ii;
				}
			else
				{

/*			  H or H' is applied to C(1:m,i:n) */

				ni = n - ii + 1;
				jc = ii;
				}

/*		   Apply H or H' */

			blasfeo_lapack_dlarfb(side, trans, "Forward", "Columnwise", &mi, &ni, &ib, &A[ii-1+(ii-1)*lda], &lda, &work[iwt], &ldt, &C[ic-1+(jc-1)*ldc], &ldc, &work[0], &ldwork); // XXX as in lapack 3.10
			}
		}

	work[0] = (double) lwkopt;
	return;

/*	 End of DORMQR */

	} /* dormqr_ */
