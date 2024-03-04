/*  -- LAPACK routine (version 3.2) -- */
/*	 Univ. of Tennessee, Univ. of California Berkeley and NAG Ltd.. */
/*	 November 2006 */

/*  Purpose */
/*  ======= */

/*  DLAED3 finds the roots of the secular equation, as defined by the */
/*  values in D, W, and RHO, between 1 and K.  It makes the */
/*  appropriate calls to DLAED4 and then updates the eigenvectors by */
/*  multiplying the matrix of eigenvectors of the pair of eigensystems */
/*  being combined by the matrix of eigenvectors of the K-by-K system */
/*  which is solved here. */

/*  This code makes very mild assumptions about floating point */
/*  arithmetic. It will work on machines with a guard digit in */
/*  add/subtract, or on those binary machines without guard digits */
/*  which subtract like the Cray X-MP, Cray Y-MP, Cray C-90, or Cray-2. */
/*  It could conceivably fail on hexadecimal or decimal machines */
/*  without guard digits, but we know of none. */

/*  Arguments */
/*  ========= */

/*  K	   (input) INTEGER */
/*		  The number of terms in the rational function to be solved by */
/*		  DLAED4.  K >= 0. */

/*  N	   (input) INTEGER */
/*		  The number of rows and columns in the Q matrix. */
/*		  N >= K (deflation may result in N>K). */

/*  N1	  (input) INTEGER */
/*		  The location of the last eigenvalue in the leading submatrix. */
/*		  min(1,N) <= N1 <= N/2. */

/*  D	   (output) DOUBLE PRECISION array, dimension (N) */
/*		  D(I) contains the updated eigenvalues for */
/*		  1 <= I <= K. */

/*  Q	   (output) DOUBLE PRECISION array, dimension (LDQ,N) */
/*		  Initially the first K columns are used as workspace. */
/*		  On output the columns 1 to K contain */
/*		  the updated eigenvectors. */

/*  LDQ	 (input) INTEGER */
/*		  The leading dimension of the array Q.  LDQ >= max(1,N). */

/*  RHO	 (input) DOUBLE PRECISION */
/*		  The value of the parameter in the rank one update equation. */
/*		  RHO >= 0 required. */

/*  DLAMDA  (input/output) DOUBLE PRECISION array, dimension (K) */
/*		  The first K elements of this array contain the old roots */
/*		  of the deflated updating problem.  These are the poles */
/*		  of the secular equation. May be changed on output by */
/*		  having lowest order bit set to zero on Cray X-MP, Cray Y-MP, */
/*		  Cray-2, or Cray C-90, as described above. */

/*  Q2	  (input) DOUBLE PRECISION array, dimension (LDQ2, N) */
/*		  The first K columns of this matrix contain the non-deflated */
/*		  eigenvectors for the split problem. */

/*  INDX	(input) INTEGER array, dimension (N) */
/*		  The permutation used to arrange the columns of the deflated */
/*		  Q matrix into three groups (see DLAED2). */
/*		  The rows of the eigenvectors found by DLAED4 must be likewise */
/*		  permuted before the matrix multiply can take place. */

/*  CTOT	(input) INTEGER array, dimension (4) */
/*		  A count of the total number of the various types of columns */
/*		  in Q, as described in INDX.  The fourth column type is any */
/*		  column which has been deflated. */

/*  W	   (input/output) DOUBLE PRECISION array, dimension (K) */
/*		  The first K elements of this array contain the components */
/*		  of the deflation-adjusted updating vector. Destroyed on */
/*		  output. */

/*  S	   (workspace) DOUBLE PRECISION array, dimension (N1 + 1)*K */
/*		  Will contain the eigenvectors of the repaired matrix which */
/*		  will be multiplied by the previously accumulated eigenvectors */
/*		  to update the system. */

/*  LDS	 (input) INTEGER */
/*		  The leading dimension of S.  LDS >= max(1,K). */

/*  INFO	(output) INTEGER */
/*		  = 0:  successful exit. */
/*		  < 0:  if INFO = -i, the i-th argument had an illegal value. */
/*		  > 0:  if INFO = 1, an eigenvalue did not converge */

/*  Further Details */
/*  =============== */

/*  Based on contributions by */
/*	 Jeff Rutter, Computer Science Division, University of California */
/*	 at Berkeley, USA */
/*  Modified by Francoise Tisseur, University of Tennessee. */

/*  ===================================================================== */

#include <math.h>
#include <stdbool.h>

#include <blasfeo_d_blas_api.h>



#if defined(FORTRAN_BLAS_API)
#define blasfeo_blas_dgemm dgemm_
#define blasfeo_lapack_dlaed3 dlaed3_
#endif



#define min(x, y) (((x) < (y)) ? (x) : (y))
#define max(x, y) (((x) > (y)) ? (x) : (y))



void dcopy_(int *, double *, int *, double *, int *);
void dlacpy_(char *, int *, int *, double *, int *, double *, int *);
void dlaed4_(int *, int *, double *, double *, double *, double *, double *, int *);
double dlamc3_(double *, double *);
void dlaset_(char *, int *, int *, double *, double *, double *, int *);
double dnrm2_(int *, double *, int *);
void xerbla_(char *, int *);



//double d_sign(double *a, double *b)
//	{
//	double x = (*a >= 0 ? *a : - *a);
//	return( *b >= 0 ? x : -x);
//	}



void blasfeo_lapack_dlaed3(int *pk, int *pn, int *pn1, double *d, double *Q, int *pldq, double *rho, double *dlamda, double *q2, int *indx, int *ctot, double *w, double *s, int *info)
	{

	int k = *pk;
	int n = *pn;
	int n1 = *pn1;
	int ldq = *pldq;

	int i_1 = 1;
	double d_0 = 0.0;
	double d_1 = 1.0;

	int ii, jj, i_t0, n2, iq2, n12, n23, idx;
	double d_t0, temp;

	/* Function Body */
	*info = 0;

	if (k < 0)
		{
		*info = -1;
		}
	else if (n < k)
		{
		*info = -2;
		}
	else if (ldq < max(1,n))
		{
		*info = -6;
		}
	if (*info != 0)
		{
		i_t0 = -(*info);
		xerbla_("DLAED3", &i_t0);
		return;
		}

/*	 Quick return if possible */

	if (k == 0)
		{
		return;
		}

/*	 Modify values DLAMDA(i) to make sure all DLAMDA(i)-DLAMDA(jj) can */
/*	 be computed with high relative accuracy (barring over/underflow). */
/*	 This is a problem on machines without a guard digit in */
/*	 add/subtract (Cray XMP, Cray YMP, Cray C 90 and Cray 2). */
/*	 The following code replaces DLAMDA(I) by 2*DLAMDA(I)-DLAMDA(I), */
/*	 which on any of these machines zeros out the bottommost */
/*	 bit of DLAMDA(I) if it is 1; this makes the subsequent */
/*	 subtractions DLAMDA(I)-DLAMDA(J) unproblematic when cancellation */
/*	 occurs. On binary machines with a guard digit (almost all */
/*	 machines) it does not change DLAMDA(I) at all. On hexadecimal */
/*	 and decimal machines with a guard digit, it slightly */
/*	 changes the bottommost bits of DLAMDA(I). It does not account */
/*	 for hexadecimal or decimal machines without guard digits */
/*	 (we know of none). We use a subroutine call to compute */
/*	 2*DLAMBDA(I) to prevent optimizing compilers from eliminating */
/*	 this code. */

	for (ii = 0; ii < k; ii++)
		{
		dlamda[ii] = dlamc3_(&dlamda[ii], &dlamda[ii]) - dlamda[ii];
		}

	for (jj = 0; jj < k; jj++)
		{
		i_t0 = jj+1;
		dlaed4_(&k, &i_t0, &dlamda[0], &w[0], &Q[jj * ldq], rho, &d[jj], info);

/*		If the zero finder fails, the computation is terminated. */

		if (*info != 0)
			{
			goto L120;
			}
		}

	if (k == 1)
		{
		goto L110;
		}
	if (k == 2)
		{
		for (jj = 0; jj < k; jj++)
			{
			w[0] = Q[jj * ldq + 0];
			w[1] = Q[jj * ldq + 1];
			idx = indx[0];
			Q[jj * ldq + 0] = w[idx-1];
			idx = indx[1];
			Q[jj * ldq + 1] = w[idx-1];
			}
		goto L110;
		}

/*	 Compute updated W. */

	dcopy_(&k, &w[0], &i_1, &s[0], &i_1);

/*	 Initialize W(I) = Q(I,I) */

	i_t0 = ldq + 1;
	dcopy_(&k, &Q[0], &i_t0, &w[0], &i_1);
	for (jj = 0; jj < k; jj++)
		{
		for (ii = 0; ii < jj; ii++)
			{
			w[ii] *= Q[ii + jj * ldq] / (dlamda[ii] - dlamda[jj]);
			}
		for (ii = jj + 1; ii < k; ii++)
			{
			w[ii] *= Q[ii + jj * ldq] / (dlamda[ii] - dlamda[jj]);
			}
		}
	for (ii = 0; ii < k; ii++)
		{
		d_t0 = sqrt(-w[ii]);
//		w[ii] = d_sign(&d_t0, &s[ii]);
		w[ii] = s[ii]>=0.0 ? d_t0 : -d_t0;
		}

/*	 Compute eigenvectors of the modified rank-1 modification. */

	for (jj = 0; jj < k; jj++)
		{
		for (ii = 0; ii < k; ii++)
			{
			s[ii] = w[ii] / Q[ii + jj * ldq];
			}
		temp = dnrm2_(&k, &s[0], &i_1);
		for (ii = 0; ii < k; ii++)
			{
			idx = indx[ii];
			Q[ii + jj * ldq] = s[idx-1] / temp;
			}
		}

/*	 Compute the updated eigenvectors. */

L110:

	n2 = n - n1;
	n12 = ctot[0] + ctot[1];
	n23 = ctot[1] + ctot[2];

	dlacpy_("A", &n23, &k, &Q[ctot[0]], &ldq, &s[0], &n23);
	iq2 = n1 * n12;
	if (n23 != 0)	
		{
		blasfeo_blas_dgemm("N", "N", &n2, &k, &n23, &d_1, &q2[iq2], &n2, &s[0], &n23, &d_0, &Q[n1], &ldq);
		}
	else
		{
		dlaset_("A", &n2, &k, &d_0, &d_0, &Q[n1], &ldq);
		}

	dlacpy_("A", &n12, &k, &Q[0], &ldq, &s[0], &n12);
	if (n12 != 0)
		{
		blasfeo_blas_dgemm("N", "N", &n1, &k, &n12, &d_1, &q2[0], &n1, &s[0], &n12, &d_0, &Q[0], &ldq);
		}
	else
		{
		dlaset_("A", &n1, &k, &d_0, &d_0, &Q[0], &ldq);
		}


L120:
	return;

/*	 End of DLAED3 */

	} /* dlaed3_ */
