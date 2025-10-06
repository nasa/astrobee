/*  -- LAPACK routine (version 3.2) -- */
/*	 Univ. of Tennessee, Univ. of California Berkeley and NAG Ltd.. */
/*	 November 2006 */

/*  Purpose */
/*  ======= */

/*  DORMTR overwrites the general real M-by-N matrix C with */

/*				  SIDE = 'L'	 SIDE = 'R' */
/*  TRANS = 'N':	  Q * C		  C * Q */
/*  TRANS = 'T':	  Q**T * C	   C * Q**T */

/*  where Q is a real orthogonal matrix of order nq, with nq = m if */
/*  SIDE = 'L' and nq = n if SIDE = 'R'. Q is defined as the product of */
/*  nq-1 elementary reflectors, as returned by DSYTRD: */

/*  if UPLO = 'U', Q = H(nq-1) . . . H(2) H(1); */

/*  if UPLO = 'L', Q = H(1) H(2) . . . H(nq-1). */

/*  Arguments */
/*  ========= */

/*  SIDE	(input) CHARACTER*1 */
/*		  = 'L': apply Q or Q**T from the Left; */
/*		  = 'R': apply Q or Q**T from the Right. */

/*  UPLO	(input) CHARACTER*1 */
/*		  = 'U': Upper triangle of A contains elementary reflectors */
/*				 from DSYTRD; */
/*		  = 'L': Lower triangle of A contains elementary reflectors */
/*				 from DSYTRD. */

/*  TRANS   (input) CHARACTER*1 */
/*		  = 'N':  No transpose, apply Q; */
/*		  = 'T':  Transpose, apply Q**T. */

/*  M	   (input) INTEGER */
/*		  The number of rows of the matrix C. M >= 0. */

/*  N	   (input) INTEGER */
/*		  The number of columns of the matrix C. N >= 0. */

/*  A	   (input) DOUBLE PRECISION array, dimension */
/*							   (LDA,M) if SIDE = 'L' */
/*							   (LDA,N) if SIDE = 'R' */
/*		  The vectors which define the elementary reflectors, as */
/*		  returned by DSYTRD. */

/*  LDA	 (input) INTEGER */
/*		  The leading dimension of the array A. */
/*		  LDA >= max(1,M) if SIDE = 'L'; LDA >= max(1,N) if SIDE = 'R'. */

/*  TAU	 (input) DOUBLE PRECISION array, dimension */
/*							   (M-1) if SIDE = 'L' */
/*							   (N-1) if SIDE = 'R' */
/*		  TAU(i) must contain the scalar factor of the elementary */
/*		  reflector H(i), as returned by DSYTRD. */

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
#define blasfeo_lapack_dormqr dormqr_
#define blasfeo_lapack_dormtr dormtr_
#endif



#define min(x, y) (((x) < (y)) ? (x) : (y))
#define max(x, y) (((x) > (y)) ? (x) : (y))



void dormql_(char *, char *, int *, int *, int *, double *, int *, double *, double *, int *, double *, int *, int *);
void blasfeo_lapack_dormqr(char *, char *, int *, int *, int *, double *, int *, double *, double *, int *, double *, int *, int *);
int ilaenv_(int *, char *, char *, int *, int *, int *, int *);
bool lsame_(char *, char *);
void xerbla_(char *, int *);



void blasfeo_lapack_dormtr(char *side, char *uplo, char *trans, int *pm, int *pn, double *A, int *plda, double *tau, double *C, int *pldc, double *work, int *lwork, int *info)
	{

	int m = *pm;
	int n = *pn;
	int lda = *plda;
	int ldc = *pldc;

	int i_1 = 1;
	int i_m1 = -1;

	int nb, nq, nw, mi, ni, lwkopt, i1, i2, iinfo;

	char s_t0[2];
	int i_t0, i_t1;

	/* Function Body */
	*info = 0;
	bool left = lsame_(side, "L");
	bool upper = lsame_(uplo, "U");
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
	else if (! upper && ! lsame_(uplo, "L"))
		{
		*info = -2;
		}
	else if (! lsame_(trans, "N") && ! lsame_(trans, "T"))
		{
		*info = -3;
		}
	else if (m < 0)
		{
		*info = -4;
		}
	else if (n < 0)
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
		// concatenate 'side' and 'trans' strings
		s_t0[0] = side[0];
		s_t0[1] = trans[0];
		if (upper)
			{
			if (left)
				{
				i_t0 = m - 1;
				i_t1 = m - 1;
				nb = ilaenv_(&i_1, "DORMQL", s_t0, &i_t0, &n, &i_t1, &i_m1);
				}
			else
				{
				i_t0 = n - 1;
				i_t1 = n - 1;
				nb = ilaenv_(&i_1, "DORMQL", s_t0, &m, &i_t0, &i_t1, &i_m1);
				}
			}
		else
			{
			if (left)
				{
				i_t0 = m - 1;
				i_t1 = m - 1;
				nb = ilaenv_(&i_1, "DORMQR", s_t0, &i_t0, &n, &i_t1, &i_m1);
				}
			else
				{
				i_t0 = n - 1;
				i_t1 = n - 1;
				nb = ilaenv_(&i_1, "DORMQR", s_t0, &m, &i_t0, &i_t1, &i_m1);
				}
			}
		lwkopt = nw * nb; // XXX as in lapack 3.10
		work[0] = (double) lwkopt;
		}

	if (*info != 0)
		{
		i_t0 = -(*info);
		xerbla_("DORMTR", &i_t0);
		return;
		}
	else if (lquery)
		{
		return;
		}

/*	 Quick return if possible */

	if (m == 0 || n == 0 || nq == 1)
		{
		work[0] = 1.;
		return;
		}

	if (left)
		{
		mi = m - 1;
		ni = n;
		}
	else
		{
		mi = m;
		ni = n - 1;
		}

	if (upper)
		{

/*		Q was determined by a call to DSYTRD with UPLO = 'U' */

		i_t0 = nq - 1;
		dormql_(side, trans, &mi, &ni, &i_t0, &A[0+1*lda], &lda, &tau[0], &C[0], &ldc, &work[0], lwork, &iinfo);
		}
	else
		{

/*		Q was determined by a call to DSYTRD with UPLO = 'L' */

		if (left)
			{
			i1 = 1;
			i2 = 0;
			}
		else
			{
			i1 = 0;
			i2 = 1;
			}
		i_t0 = nq - 1;
		blasfeo_lapack_dormqr(side, trans, &mi, &ni, &i_t0, &A[1+0*lda], &lda, &tau[0], &C[i1+i2*ldc], &ldc, &work[0], lwork, &iinfo);
		}

	work[0] = (double) lwkopt;

	return;

/*	 End of DORMTR */

	} /* dormtr_ */	
