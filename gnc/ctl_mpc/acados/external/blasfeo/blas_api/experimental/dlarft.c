/*  -- LAPACK auxiliary routine (version 3.2) -- */
/*     Univ. of Tennessee, Univ. of California Berkeley and NAG Ltd.. */
/*     November 2006 */

/*  Purpose */
/*  ======= */

/*  DLARFT forms the triangular factor T of a real block reflector H */
/*  of order n, which is defined as a product of k elementary reflectors. */

/*  If DIRECT = 'F', H = H(1) H(2) . . . H(k) and T is upper triangular; */

/*  If DIRECT = 'B', H = H(k) . . . H(2) H(1) and T is lower triangular. */

/*  If STOREV = 'C', the vector which defines the elementary reflector */
/*  H(i) is stored in the i-th column of the array V, and */

/*     H  =  I - V * T * V' */

/*  If STOREV = 'R', the vector which defines the elementary reflector */
/*  H(i) is stored in the i-th row of the array V, and */

/*     H  =  I - V' * T * V */

/*  Arguments */
/*  ========= */

/*  DIRECT  (input) CHARACTER*1 */
/*          Specifies the order in which the elementary reflectors are */
/*          multiplied to form the block reflector: */
/*          = 'F': H = H(1) H(2) . . . H(k) (Forward) */
/*          = 'B': H = H(k) . . . H(2) H(1) (Backward) */

/*  STOREV  (input) CHARACTER*1 */
/*          Specifies how the vectors which define the elementary */
/*          reflectors are stored (see also Further Details): */
/*          = 'C': columnwise */
/*          = 'R': rowwise */

/*  N       (input) INTEGER */
/*          The order of the block reflector H. N >= 0. */

/*  K       (input) INTEGER */
/*          The order of the triangular factor T (= the number of */
/*          elementary reflectors). K >= 1. */

/*  V       (input/output) DOUBLE PRECISION array, dimension */
/*                               (LDV,K) if STOREV = 'C' */
/*                               (LDV,N) if STOREV = 'R' */
/*          The matrix V. See further details. */

/*  LDV     (input) INTEGER */
/*          The leading dimension of the array V. */
/*          If STOREV = 'C', LDV >= max(1,N); if STOREV = 'R', LDV >= K. */

/*  TAU     (input) DOUBLE PRECISION array, dimension (K) */
/*          TAU(i) must contain the scalar factor of the elementary */
/*          reflector H(i). */

/*  T       (output) DOUBLE PRECISION array, dimension (LDT,K) */
/*          The k by k triangular factor T of the block reflector. */
/*          If DIRECT = 'F', T is upper triangular; if DIRECT = 'B', T is */
/*          lower triangular. The rest of the array is not used. */

/*  LDT     (input) INTEGER */
/*          The leading dimension of the array T. LDT >= K. */

/*  Further Details */
/*  =============== */

/*  The shape of the matrix V and the storage of the vectors which define */
/*  the H(i) is best illustrated by the following example with n = 5 and */
/*  k = 3. The elements equal to 1 are not stored; the corresponding */
/*  array elements are modified but restored on exit. The rest of the */
/*  array is not used. */

/*  DIRECT = 'F' and STOREV = 'C':         DIRECT = 'F' and STOREV = 'R': */

/*               V = (  1       )                 V = (  1 v1 v1 v1 v1 ) */
/*                   ( v1  1    )                     (     1 v2 v2 v2 ) */
/*                   ( v1 v2  1 )                     (        1 v3 v3 ) */
/*                   ( v1 v2 v3 ) */
/*                   ( v1 v2 v3 ) */

/*  DIRECT = 'B' and STOREV = 'C':         DIRECT = 'B' and STOREV = 'R': */

/*               V = ( v1 v2 v3 )                 V = ( v1 v1  1       ) */
/*                   ( v1 v2 v3 )                     ( v2 v2 v2  1    ) */
/*                   (  1 v2 v3 )                     ( v3 v3 v3 v3  1 ) */
/*                   (     1 v3 ) */
/*                   (        1 ) */

/*  ===================================================================== */

#include <math.h>
#include <stdbool.h>

#include <blasfeo_d_blas_api.h>



#if defined(FORTRAN_BLAS_API)
#define blasfeo_blas_dgemv dgemv_
#define blasfeo_lapack_dlarft dlarft_
#endif



#define min(x, y) (((x) < (y)) ? (x) : (y))
#define max(x, y) (((x) > (y)) ? (x) : (y))



void dtrmv_(char *, char *, char *, int *, double *, int *, double *, int *);
bool lsame_(char *, char *);



void blasfeo_lapack_dlarft(char *direct, char *storev, int *pn, int *pk, double *V, int *pldv, double *tau, double *T, int *pldt)
	{

	int n = *pn;
	int k = *pk;
	int ldv = *pldv;
	int ldt = *pldt;

	int i_1 = 1;
	double d_0 = 0.0;
	double d_1 = 1.0;

	int lastv, prevlastv;
	double vii;

	int i_t0, i_t1;
	double d_t0;

	int ii, jj;

    /* Function Body */
    if (n == 0)
		{
		return;
		}

    if (lsame_(direct, "F"))
		{
		prevlastv = n-1;
		for (ii = 0; ii < k; ii++)
			{
			prevlastv = max(ii,prevlastv);
			if (tau[ii] == 0.0)
				{

	/*              H(i)  =  I */

				for (jj = 0; jj <= ii; jj++)
					{
					T[jj + ii * ldt] = 0.0;
					}
				}
			else
				{

		/*              general case */

				// XXX modify as in lapack 3.10
//				vii = V[ii + ii * ldv];
//				V[ii + ii * ldv] = 1.0;
				if (lsame_(storev, "C"))
					{
		/*                 Skip any trailing zeros. */
					for (lastv = n-1; lastv >= ii+1; lastv--)
						{
						if (V[lastv + ii * ldv] != 0.0)
							{
							break;
							}
						}
					for (jj=0; jj<=ii-1; jj++)
						{
						T[jj + ii*ldt] = - tau[ii] * V[ii + jj*ldv];
						}
					jj = min(lastv,prevlastv);

		/*                 T(1:i-1,i) := - tau(i) * V(i:j,1:i-1)' * V(i:j,i) */

//					i_t0 = jj - ii;
					i_t0 = jj - ii; // - 1;
					i_t1 = ii;
					d_t0 = -tau[ii];
//					blasfeo_blas_dgemv("Transpose", &i_t0, &i_t1, &d_t0, &V[ii], &ldv, &V[ii + ii * ldv], &i_1, &d_0, &T[ii * ldt], &i_1);
					blasfeo_blas_dgemv("Transpose", &i_t0, &i_t1, &d_t0, &V[ii+1], &ldv, &V[ii+1 + ii * ldv], &i_1, &d_1, &T[ii * ldt], &i_1);
					}
				else
					{
		/*                 Skip any trailing zeros. */
					for (lastv = n-1; lastv >= ii+1; lastv--)
						{
						if (V[ii + lastv * ldv] != 0.0)
							{
							break;
							}
						}
					for (jj=0; jj<=ii-1; jj++)
						{
						T[jj + ii*ldt] = - tau[ii] * V[jj + ii*ldv];
						}
					jj = min(lastv,prevlastv);

		/*                 T(1:i-1,i) := - tau(i) * V(1:i-1,i:j) * V(i,i:j)' */

					i_t0 = ii;
//					i_t1 = jj - ii;
					i_t1 = jj - ii; // - 1;
					d_t0 = -tau[ii];
//					blasfeo_blas_dgemv("No transpose", &i_t0, &i_t1, &d_t0, &V[ii * ldv], &ldv, &V[ii + ii * ldv], &ldv, &d_0, &T[ii * ldt], &i_1);
					blasfeo_blas_dgemv("No transpose", &i_t0, &i_t1, &d_t0, &V[(ii+1) * ldv], &ldv, &V[ii + (ii+1) * ldv], &ldv, &d_1, &T[ii * ldt], &i_1);
					}
//				V[ii + ii * ldv] = vii;

		/*              T(1:i-1,i) := T(1:i-1,1:i-1) * T(1:i-1,i) */

				i_t0 = ii;
				dtrmv_("Upper", "No transpose", "Non-unit", &i_t0, &T[0], &ldt, &T[ii * ldt], &i_1);
				T[ii + ii * ldt] = tau[ii];
				if (ii > 0)
					{
					prevlastv = max(prevlastv,lastv);
					}
				else
					{
					prevlastv = lastv;
					}
				}
			}
		}
	else
		{
		prevlastv = 0;
		for (ii = k-1; ii >= 0; ii--)
			{
			if (tau[ii] == 0.0)
				{

	/*              H(i)  =  I */

				for (jj = ii; jj < k; jj++)
					{
					T[jj + ii * ldt] = 0.0;
					}
				}
			else
				{

		/*              general case */

				if (ii < k-1)
					{
					if (lsame_(storev, "C"))
						{
						// XXX modify as in lapack 3.10
//						vii = V[n - k + ii + ii * ldv];
//						V[n - k + ii + ii * ldv] = 1.0;
			/*                    Skip any leading zeros. */
						for (lastv = 0; lastv < ii; lastv++)
							{
							if (V[lastv + ii * ldv] != 0.)
								{
								break;
								}
							}
						for (jj = ii+1; jj < k; jj++)
							{
							T[jj + ii*ldt] = - tau[ii] * V[n-k+ii + jj*ldv];
							}
						jj = max(lastv,prevlastv);

			/*                    T(i+1:k,i) := */
			/*                            - tau(i) * V(j:n-k+i,i+1:k)' * V(j:n-k+i,i) */

//						i_t0 = n - k + ii - jj + 1; // + 2;
						i_t0 = n - k + ii - jj; // + 1;
						i_t1 = k - ii - 1;
						d_t0 = -tau[ii];
//						blasfeo_blas_dgemv("Transpose", &i_t0, &i_t1, &d_t0, &V[jj + (ii+1) * ldv], &ldv, &V[jj + ii * ldv], &i_1, &d_0, &T[ii+1 + ii * ldt], &i_1);
						blasfeo_blas_dgemv("Transpose", &i_t0, &i_t1, &d_t0, &V[jj + (ii+1) * ldv], &ldv, &V[jj + ii * ldv], &i_1, &d_1, &T[ii+1 + ii * ldt], &i_1);
//						V[n - k + ii + ii * ldv] = vii;
						}
					else
						{
						// XXX modify as in lapack 3.10
//						vii = V[ii + (n - k + ii) * ldv];
//						V[ii + (n - k + ii) * ldv] = 1.0;
			/*                    Skip any leading zeros. */
						for (lastv = 0; lastv < ii; lastv++)
							{
							if (V[ii + lastv * ldv] != 0.0)
								{
								break;
								}
							}
						for (jj = ii+1; jj < k; jj++)
							{
							T[jj + ii*ldt] = - tau[ii] * V[jj + (n-k+ii)*ldv];
							}
						jj = max(lastv,prevlastv);

			/*                    T(i+1:k,i) := */
			/*                            - tau(i) * V(i+1:k,j:n-k+i) * V(i,j:n-k+i)' */

						i_t0 = k - ii;
//						i_t1 = n - k + ii - jj + 1; // + 2;
						i_t1 = n - k + ii - jj; // + 1;
						d_t0 = -tau[ii];
//						blasfeo_blas_dgemv("No transpose", &i_t0, &i_t1, &d_t0, &V[ii+1 + jj * ldv], &ldv, &V[ii + jj * ldv], &ldv, &d_0, &T[ii+1 + ii * ldt], &i_1);
						blasfeo_blas_dgemv("No transpose", &i_t0, &i_t1, &d_t0, &V[ii+1 + jj * ldv], &ldv, &V[ii + jj * ldv], &ldv, &d_1, &T[ii+1 + ii * ldt], &i_1);
//						V[ii + (n - k + ii) * ldv] = vii;
						}

		/*                 T(i+1:k,i) := T(i+1:k,i+1:k) * T(i+1:k,i) */

					i_t0 = k - ii;
					dtrmv_("Lower", "No transpose", "Non-unit", &i_t0, &T[ii+1 + (ii+1) * ldt], &ldt, &T[ii+1 + ii * ldt], &i_1);
					if (ii > 0)
						{
						prevlastv = min(prevlastv,lastv);
						}
					else
						{
						prevlastv = lastv;
						}
					}
				T[ii + ii * ldt] = tau[ii];
				}
			}
		}
    return;

/*     End of DLARFT */

	} /* dlarft_ */

