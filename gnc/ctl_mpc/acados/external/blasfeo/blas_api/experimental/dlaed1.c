/*  -- LAPACK routine (version 3.2) -- */
/*     Univ. of Tennessee, Univ. of California Berkeley and NAG Ltd.. */
/*     November 2006 */

/*  Purpose */
/*  ======= */

/*  DLAED1 computes the updated eigensystem of a diagonal */
/*  matrix after modification by a rank-one symmetric matrix.  This */
/*  routine is used only for the eigenproblem which requires all */
/*  eigenvalues and eigenvectors of a tridiagonal matrix.  DLAED7 handles */
/*  the case in which eigenvalues only or eigenvalues and eigenvectors */
/*  of a full symmetric matrix (which was reduced to tridiagonal form) */
/*  are desired. */

/*    T = Q(in) ( D(in) + RHO * Z*Z' ) Q'(in) = Q(out) * D(out) * Q'(out) */

/*     where Z = Q'u, u is a vector of length N with ones in the */
/*     CUTPNT and CUTPNT + 1 th elements and zeros elsewhere. */

/*     The eigenvectors of the original matrix are stored in Q, and the */
/*     eigenvalues are in D.  The algorithm consists of three stages: */

/*        The first stage consists of deflating the size of the problem */
/*        when there are multiple eigenvalues or if there is a zero in */
/*        the Z vector.  For each such occurence the dimension of the */
/*        secular equation problem is reduced by one.  This stage is */
/*        performed by the routine DLAED2. */

/*        The second stage consists of calculating the updated */
/*        eigenvalues. This is done by finding the roots of the secular */
/*        equation via the routine DLAED4 (as called by DLAED3). */
/*        This routine also calculates the eigenvectors of the current */
/*        problem. */

/*        The final stage consists of computing the updated eigenvectors */
/*        directly using the updated eigenvalues.  The eigenvectors for */
/*        the current problem are multiplied with the eigenvectors from */
/*        the overall problem. */

/*  Arguments */
/*  ========= */

/*  N      (input) INTEGER */
/*         The dimension of the symmetric tridiagonal matrix.  N >= 0. */

/*  D      (input/output) DOUBLE PRECISION array, dimension (N) */
/*         On entry, the eigenvalues of the rank-1-perturbed matrix. */
/*         On exit, the eigenvalues of the repaired matrix. */

/*  Q      (input/output) DOUBLE PRECISION array, dimension (LDQ,N) */
/*         On entry, the eigenvectors of the rank-1-perturbed matrix. */
/*         On exit, the eigenvectors of the repaired tridiagonal matrix. */

/*  LDQ    (input) INTEGER */
/*         The leading dimension of the array Q.  LDQ >= max(1,N). */

/*  INDXQ  (input/output) INTEGER array, dimension (N) */
/*         On entry, the permutation which separately sorts the two */
/*         subproblems in D into ascending order. */
/*         On exit, the permutation which will reintegrate the */
/*         subproblems back into sorted order, */
/*         i.e. D( INDXQ( I = 1, N ) ) will be in ascending order. */

/*  RHO    (input) DOUBLE PRECISION */
/*         The subdiagonal entry used to create the rank-1 modification. */

/*  CUTPNT (input) INTEGER */
/*         The location of the last eigenvalue in the leading sub-matrix. */
/*         min(1,N) <= CUTPNT <= N/2. */

/*  WORK   (workspace) DOUBLE PRECISION array, dimension (4*N + N**2) */

/*  IWORK  (workspace) INTEGER array, dimension (4*N) */

/*  INFO   (output) INTEGER */
/*          = 0:  successful exit. */
/*          < 0:  if INFO = -i, the i-th argument had an illegal value. */
/*          > 0:  if INFO = 1, an eigenvalue did not converge */

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
#define blasfeo_lapack_dlaed3 dlaed3_
#define blasfeo_lapack_dlaed1 dlaed1_
#endif



#define min(x, y) (((x) < (y)) ? (x) : (y))
#define max(x, y) (((x) > (y)) ? (x) : (y))



void dcopy_(int *, double *, int *, double *, int *);
void dlacpy_(char *, int *, int *, double *, int *, double *, int *);
void dlaed2_(int *, int *, int *, double *, double *, int *, int *, double *, double *, double *, double *, double *, int *, int *, int *, int *, int *);
void blasfeo_lapack_dlaed3(int *, int *, int *, double *, double *, int *, double *, double *, double *, int *, int *, double *, double *, int *);
void dlamrg_(int *, int *, double *, int *, int *, int *);
void xerbla_(char *, int *);



void blasfeo_lapack_dlaed1(int *pn, double *d, double *Q, int *pldq, int *indxq, double *rho, int *cutpnt, double *work, int *iwork, int *info)
	{

	int n = *pn;
	int ldq = *pldq;

	int i_1 = 1;
	int i_m1 = -1;

	int iz, idlmda, iw, iq2, indx, indxc, coltyp, indxp, zpp1, is, n1, n2, k;

	int ii;

	int i_t0;

    /* Function Body */
    *info = 0;

    if (n < 0)
		{
		*info = -1;
		}
	else if (ldq < max(1,n))
		{
		*info = -4;
		}
	else if (min(1,n/2) > *cutpnt || n / 2 < *cutpnt)
		{
		*info = -7;
		}
	if (*info != 0)
		{
		i_t0 = -(*info);
		xerbla_("DLAED1", &i_t0);
		return;
		}

/*     Quick return if possible */

    if (n == 0)
		{
		return;
		}

/*     The following values are integer pointers which indicate */
/*     the portion of the workspace */
/*     used by a particular array in DLAED2 and DLAED3. */

    iz = 0;
    idlmda = iz + n;
    iw = idlmda + n;
    iq2 = iw + n;

    indx = 0;
    indxc = indx + n;
    coltyp = indxc + n;
    indxp = coltyp + n;


/*     Form the z-vector which consists of the last row of Q_1 and the */
/*     first row of Q_2. */

    dcopy_(cutpnt, &Q[*cutpnt -1], &ldq, &work[iz], &i_1);
    zpp1 = *cutpnt + 1;
    i_t0 = n - *cutpnt;
    dcopy_(&i_t0, &Q[zpp1-1 + (zpp1-1) * ldq], &ldq, &work[iz + *cutpnt], &i_1);

/*     Deflate eigenvalues. */

    dlaed2_(&k, &n, cutpnt, &d[0], &Q[0], &ldq, &indxq[0], rho, &work[iz], &work[idlmda], &work[iw], &work[iq2], &iwork[indx], &iwork[indxc], &iwork[indxp], &iwork[coltyp], info);

    if (*info != 0)
		{
		goto L20;
		}

/*     Solve Secular Equation. */

    if (k != 0)
		{
		is = (iwork[coltyp] + iwork[coltyp+1]) * *cutpnt + (iwork[coltyp+1] + iwork[coltyp+2]) * (n - *cutpnt) + iq2+1;
		blasfeo_lapack_dlaed3(&k, &n, cutpnt, &d[0], &Q[0], &ldq, rho, &work[idlmda], &work[iq2], &iwork[indxc], &iwork[coltyp], &work[iw], &work[is-1], info);
		if (*info != 0)
			{
			goto L20;
			}

/*     Prepare the INDXQ sorting permutation. */

		n1 = k;
		n2 = n - k;
		dlamrg_(&n1, &n2, &d[0], &i_1, &i_m1, &indxq[0]);
		}
	else
		{
		for (ii = 0; ii < n; ii++)
			{
			indxq[ii] = ii+1;
			}
		}

L20:
    return;

/*     End of DLAED1 */

	} /* dlaed1_ */
