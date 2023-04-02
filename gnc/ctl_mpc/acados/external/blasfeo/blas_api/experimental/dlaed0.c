/*  -- LAPACK routine (version 3.2) -- */
/*     Univ. of Tennessee, Univ. of California Berkeley and NAG Ltd.. */
/*     November 2006 */

/*  Purpose */
/*  ======= */

/*  DLAED0 computes all eigenvalues and corresponding eigenvectors of a */
/*  symmetric tridiagonal matrix using the divide and conquer method. */

/*  Arguments */
/*  ========= */

/*  ICOMPQ  (input) INTEGER */
/*          = 0:  Compute eigenvalues only. */
/*          = 1:  Compute eigenvectors of original dense symmetric matrix */
/*                also.  On entry, Q contains the orthogonal matrix used */
/*                to reduce the original matrix to tridiagonal form. */
/*          = 2:  Compute eigenvalues and eigenvectors of tridiagonal */
/*                matrix. */

/*  QSIZ   (input) INTEGER */
/*         The dimension of the orthogonal matrix used to reduce */
/*         the full matrix to tridiagonal form.  QSIZ >= N if ICOMPQ = 1. */

/*  N      (input) INTEGER */
/*         The dimension of the symmetric tridiagonal matrix.  N >= 0. */

/*  D      (input/output) DOUBLE PRECISION array, dimension (N) */
/*         On entry, the main diagonal of the tridiagonal matrix. */
/*         On exit, its eigenvalues. */

/*  E      (input) DOUBLE PRECISION array, dimension (N-1) */
/*         The off-diagonal elements of the tridiagonal matrix. */
/*         On exit, E has been destroyed. */

/*  Q      (input/output) DOUBLE PRECISION array, dimension (LDQ, N) */
/*         On entry, Q must contain an N-by-N orthogonal matrix. */
/*         If ICOMPQ = 0    Q is not referenced. */
/*         If ICOMPQ = 1    On entry, Q is a subset of the columns of the */
/*                          orthogonal matrix used to reduce the full */
/*                          matrix to tridiagonal form corresponding to */
/*                          the subset of the full matrix which is being */
/*                          decomposed at this time. */
/*         If ICOMPQ = 2    On entry, Q will be the identity matrix. */
/*                          On exit, Q contains the eigenvectors of the */
/*                          tridiagonal matrix. */

/*  LDQ    (input) INTEGER */
/*         The leading dimension of the array Q.  If eigenvectors are */
/*         desired, then  LDQ >= max(1,N).  In any case,  LDQ >= 1. */

/*  QSTORE (workspace) DOUBLE PRECISION array, dimension (LDQS, N) */
/*         Referenced only when ICOMPQ = 1.  Used to store parts of */
/*         the eigenvector matrix when the updating matrix multiplies */
/*         take place. */

/*  LDQS   (input) INTEGER */
/*         The leading dimension of the array QSTORE.  If ICOMPQ = 1, */
/*         then  LDQS >= max(1,N).  In any case,  LDQS >= 1. */

/*  WORK   (workspace) DOUBLE PRECISION array, */
/*         If ICOMPQ = 0 or 1, the dimension of WORK must be at least */
/*                     1 + 3*N + 2*N*lg N + 2*N**2 */
/*                     ( lg( N ) = smallest integer k */
/*                                 such that 2^k >= N ) */
/*         If ICOMPQ = 2, the dimension of WORK must be at least */
/*                     4*N + N**2. */

/*  IWORK  (workspace) INTEGER array, */
/*         If ICOMPQ = 0 or 1, the dimension of IWORK must be at least */
/*                        6 + 6*N + 5*N*lg N. */
/*                        ( lg( N ) = smallest integer k */
/*                                    such that 2^k >= N ) */
/*         If ICOMPQ = 2, the dimension of IWORK must be at least */
/*                        3 + 5*N. */

/*  INFO   (output) INTEGER */
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

/*  ===================================================================== */

#include <math.h>
#include <stdbool.h>

#include <blasfeo_d_blas_api.h>



#if defined(FORTRAN_BLAS_API)
#define blasfeo_lapack_dlaed1 dlaed1_
#define blasfeo_blas_dgemm dgemm_
#define blasfeo_lapack_dlaed0 dlaed0_
#endif



#define min(x, y) (((x) < (y)) ? (x) : (y))
#define max(x, y) (((x) > (y)) ? (x) : (y))



void dcopy_(int *, double *, int *, double *, int *);
void dlacpy_(char *, int *, int *, double *, int *, double *, int *);
void blasfeo_lapack_dlaed1(int *, double *, double *, int *, int *, double *, int *, double *, int *, int *);
void dlaed7_(int *, int *, int *, int *, int *, int *, double *, double *, int *, int *, double *, int *, double *, int *, int *, int *, int *, int *, double *, double *, int *, int *);
void dsteqr_(char *, int *, double *, double *, double *, int *, double *, int *);
int ilaenv_(int *, char *, char *, int *, int *, int *, int *);
void xerbla_(char *, int *);



void blasfeo_lapack_dlaed0(int *icompq, int *qsiz, int *pn, double *d, double *e, double *Q, int *pldq, double *Qstore, int *pldqs, double *work, int *iwork, int *info)
	{

	int i_0 = 0;
	int i_1 = 1;
	int i_9 = 9;
	double d_1 = 1.0;
	double d_0 = 0.0;

	int n = *pn;
	int ldq = *pldq;
	int ldqs = *pldqs;

	int smlsiz, subpbs, tlvls, spm1, submat, smm1, indxq, lgn, iprmpt, iperm, iqptr, igivpt, igivcl, igivnm, iq, iwrem, curr, curlvl, spm2, matsiz, msd2, curprb;

	int i_t0;

	int ii, jj, kk;

    /* Function Body */
    *info = 0;

    if (*icompq < 0 || *icompq > 2)
		{
		*info = -1;
		}
	else if (*icompq == 1 && *qsiz < max(0,n))
		{
		*info = -2;
		}
	else if (n < 0)
		{
		*info = -3;
		}
	else if (ldq < max(1,n))
		{
		*info = -7;
		}
	else if (ldqs < max(1,n))
		{
		*info = -9;
		}
	if (*info != 0)
		{
		i_t0 = -(*info);
		xerbla_("DLAED0", &i_t0);
		return;
		}

/*     Quick return if possible */

    if (n == 0)
		{
		return;
		}

    smlsiz = ilaenv_(&i_9, "DLAED0", " ", &i_0, &i_0, &i_0, &i_0);

/*     Determine the size and placement of the submatrices, and save in */
/*     the leading elements of IWORK. */

    iwork[0] = n;
    subpbs = 1;
    tlvls = 0;
L10:
    if (iwork[subpbs-1] > smlsiz)
		{
		for (jj = subpbs; jj >= 1; jj--)
			{
			iwork[2*jj-1] = (iwork[jj-1] + 1) / 2;
			iwork[2*jj-2] = iwork[jj-1] / 2;
			}
		tlvls++;
		subpbs *= 2;
		goto L10;
		}
    for (jj = 1; jj < subpbs; jj++)
		{
		iwork[jj] += iwork[jj-1];
		}

/*     Divide the matrix into SUBPBS submatrices of size at most SMLSIZ+1 */
/*     using rank-1 modifications (cuts). */

    spm1 = subpbs - 1;
    for (ii = 0; ii < spm1; ii++)
		{
		submat = iwork[ii];
		smm1 = submat - 1;
		d[smm1] -= fabs(e[smm1]);
		d[submat] -= fabs(e[smm1]);
		}

    indxq = 4*n + 3;
    if (*icompq != 2)
		{

/*        Set up workspaces for eigenvalues only/accumulate new vectors */
/*        routine */

		lgn = (int) ( log((double) (n)) / log(2.0) );
		if ((1<<lgn) < n)
			{
			lgn++;
			}
		if ((1<<lgn) < n)
			{
			lgn++;
			}
		iprmpt = indxq + n;
		iperm = iprmpt + n*lgn;
		iqptr = iperm + n*lgn;
		igivpt = iqptr + n + 2;
		igivcl = igivpt + n*lgn;

		igivnm = 0;
		iq = igivnm + 2*n*lgn;
		iwrem = iq + n*n + 1;

/*        Initialize pointers */

		for (ii = 0; ii <= subpbs; ii++)
			{
			iwork[iprmpt + ii] = 1;
			iwork[igivpt + ii] = 1;
			}
		iwork[iqptr] = 1;
		}

/*     Solve each submatrix eigenproblem at the bottom of the divide and */
/*     conquer tree. */

    curr = 0;
    for (ii = 0; ii <= spm1; ++ii)
		{
		if (ii == 0)
			{
			submat = 1;
			matsiz = iwork[0];
			}
		else
			{
			submat = iwork[ii-1] + 1;
			matsiz = iwork[ii] - iwork[ii-1];
			}
		if (*icompq == 2)
			{
			dsteqr_("I", &matsiz, &d[submat-1], &e[submat-1], &Q[submat-1 + (submat-1) * ldq], &ldq, &work[0], info);
			if (*info != 0)
				{
				goto L130;
				}
			}
		else
			{
			dsteqr_("I", &matsiz, &d[submat-1], &e[submat-1], &work[iq - 1 + iwork[iqptr + curr]], &matsiz, &work[0], info);
			if (*info != 0)
				{
				goto L130;
				}
			if (*icompq == 1)
				{
				blasfeo_blas_dgemm("N", "N", qsiz, &matsiz, &matsiz, &d_1, &Q[(submat-1) * ldq], &ldq, &work[iq - 1 + iwork[iqptr + curr]], &matsiz, &d_0, &Qstore[(submat-1) * ldqs], &ldqs);
				}
			iwork[iqptr + curr + 1] = iwork[iqptr + curr] + matsiz * matsiz;
			curr++;
			}
		kk = 1;
		for (jj = submat; jj <= iwork[ii]; jj++)
			{
			iwork[indxq + jj - 1] = kk;
			kk++;
			}
		}

/*     Successively merge eigensystems of adjacent submatrices */
/*     into eigensystem for the corresponding larger matrix. */

/*     while ( SUBPBS > 1 ) */

    curlvl = 1;
L80:
    if (subpbs > 1)
		{
		spm2 = subpbs - 2;
		for (ii = 0; ii <= spm2; ii += 2)
			{
			if (ii == 0)
				{
				submat = 1;
				matsiz = iwork[1];
				msd2 = iwork[0];
				curprb = 0;
				}
			else
				{
				submat = iwork[ii-1] + 1;
				matsiz = iwork[ii+1] - iwork[ii-1];
				msd2 = matsiz / 2;
				curprb++;
				}

	/*     Merge lower order eigensystems (of size MSD2 and MATSIZ - MSD2) */
	/*     into an eigensystem of size MATSIZ. */
	/*     DLAED1 is used only for the full eigensystem of a tridiagonal */
	/*     matrix. */
	/*     DLAED7 handles the cases in which eigenvalues only or eigenvalues */
	/*     and eigenvectors of a full symmetric matrix (which was reduced to */
	/*     tridiagonal form) are desired. */

			if (*icompq == 2)
				{
				blasfeo_lapack_dlaed1(&matsiz, &d[submat-1], &Q[submat-1 + (submat-1) * ldq], &ldq, &iwork[indxq + submat - 1], &e[submat + msd2 - 2], &msd2, &work[0], &iwork[subpbs], info);
				}
			else
				{
				dlaed7_(icompq, &matsiz, qsiz, &tlvls, &curlvl, &curprb, &d[submat-1], &Qstore[(submat-1) * ldqs], &ldqs, &iwork[indxq + submat- 1], &e[submat + msd2 - 2], &msd2, &work[iq], &iwork[iqptr], &iwork[iprmpt], &iwork[iperm], &iwork[igivpt], &iwork[igivcl], &work[igivnm], &work[iwrem], &iwork[subpbs], info);
				}
			if (*info != 0)
				{
				goto L130;
				}
			iwork[ii / 2] = iwork[ii + 1];
			}
		subpbs /= 2;
		curlvl++;
		goto L80;
		}

/*     end while */

/*     Re-merge the eigenvalues/vectors which were deflated at the final */
/*     merge step. */

    if (*icompq == 1)
		{
		for (ii = 1; ii <= n; ii++)
			{
			jj = iwork[indxq + ii - 1];
			work[ii-1] = d[jj-1];
			dcopy_(qsiz, &Qstore[(jj-1) * ldqs], &i_1, &Q[(ii-1) * ldq], &i_1);
			}
		dcopy_(&n, &work[0], &i_1, &d[0], &i_1);
		}
	else if (*icompq == 2)
		{
		for (ii = 1; ii <= n; ii++)
			{
			jj = iwork[indxq + ii - 1];
			work[ii-1] = d[jj-1];
			dcopy_(&n, &Q[(jj-1) * ldq], &i_1, &work[n * ii], &i_1);
			}
		dcopy_(&n, &work[0], &i_1, &d[0], &i_1);
		dlacpy_("A", &n, &n, &work[n], &n, &Q[0], &ldq);
		}
	else
		{
		for (ii = 1; ii <= n; ii++)
			{
			jj = iwork[indxq + ii - 1];
			work[ii-1] = d[jj-1];
			}
		dcopy_(&n, &work[0], &i_1, &d[0], &i_1);
		}
    goto L140;

L130:
    *info = submat * (n + 1) + submat + matsiz - 1;

L140:
    return;

/*     End of DLAED0 */

} /* dlaed0_ */
