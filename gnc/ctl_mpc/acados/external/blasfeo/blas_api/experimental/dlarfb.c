/*  -- LAPACK auxiliary routine (version 3.2) -- */
/*     Univ. of Tennessee, Univ. of California Berkeley and NAG Ltd.. */
/*     November 2006 */

/*  Purpose */
/*  ======= */

/*  DLARFB applies a real block reflector H or its transpose H' to a */
/*  real m by n matrix C, from either the left or the right. */

/*  Arguments */
/*  ========= */

/*  SIDE    (input) CHARACTER*1 */
/*          = 'L': apply H or H' from the Left */
/*          = 'R': apply H or H' from the Right */

/*  TRANS   (input) CHARACTER*1 */
/*          = 'N': apply H (No transpose) */
/*          = 'T': apply H' (Transpose) */

/*  DIRECT  (input) CHARACTER*1 */
/*          Indicates how H is formed from a product of elementary */
/*          reflectors */
/*          = 'F': H = H(1) H(2) . . . H(k) (Forward) */
/*          = 'B': H = H(k) . . . H(2) H(1) (Backward) */

/*  STOREV  (input) CHARACTER*1 */
/*          Indicates how the vectors which define the elementary */
/*          reflectors are stored: */
/*          = 'C': Columnwise */
/*          = 'R': Rowwise */

/*  M       (input) INTEGER */
/*          The number of rows of the matrix C. */

/*  N       (input) INTEGER */
/*          The number of columns of the matrix C. */

/*  K       (input) INTEGER */
/*          The order of the matrix T (= the number of elementary */
/*          reflectors whose product defines the block reflector). */

/*  V       (input) DOUBLE PRECISION array, dimension */
/*                                (LDV,K) if STOREV = 'C' */
/*                                (LDV,M) if STOREV = 'R' and SIDE = 'L' */
/*                                (LDV,N) if STOREV = 'R' and SIDE = 'R' */
/*          The matrix V. See further details. */

/*  LDV     (input) INTEGER */
/*          The leading dimension of the array V. */
/*          If STOREV = 'C' and SIDE = 'L', LDV >= max(1,M); */
/*          if STOREV = 'C' and SIDE = 'R', LDV >= max(1,N); */
/*          if STOREV = 'R', LDV >= K. */

/*  T       (input) DOUBLE PRECISION array, dimension (LDT,K) */
/*          The triangular k by k matrix T in the representation of the */
/*          block reflector. */

/*  LDT     (input) INTEGER */
/*          The leading dimension of the array T. LDT >= K. */

/*  C       (input/output) DOUBLE PRECISION array, dimension (LDC,N) */
/*          On entry, the m by n matrix C. */
/*          On exit, C is overwritten by H*C or H'*C or C*H or C*H'. */

/*  LDC     (input) INTEGER */
/*          The leading dimension of the array C. LDA >= max(1,M). */

/*  WORK    (workspace) DOUBLE PRECISION array, dimension (LDWORK,K) */

/*  LDWORK  (input) INTEGER */
/*          The leading dimension of the array WORK. */
/*          If SIDE = 'L', LDWORK >= max(1,N); */
/*          if SIDE = 'R', LDWORK >= max(1,M). */

/*  ===================================================================== */

#include <math.h>
#include <stdbool.h>

#include <blasfeo_d_blas_api.h>



#if defined(FORTRAN_BLAS_API)
#define blasfeo_blas_dgemm dgemm_
#define blasfeo_blas_dtrmm dtrmm_
#define blasfeo_lapack_dlarfb dlarfb_
#endif



#define min(x, y) (((x) < (y)) ? (x) : (y))
#define max(x, y) (((x) > (y)) ? (x) : (y))



void dcopy_(int *, double *, int *, double *, int *);
int iladlc_(int *, int *, double *, int *);
int iladlr_(int *, int *, double *, int *);
bool lsame_(char *, char *);



void blasfeo_lapack_dlarfb(char *side, char *trans, char *direct, char *storev, int *pm, int *pn, int *pk, double *V, int *pldv, double *T, int *pldt, double *C, int *pldc, double *work, int *pldwork)
	{

	int m = *pm;
	int n = *pn;
	int k = *pk;

	int ldv = *pldv;
	int ldt = *pldt;
	int ldc = *pldc;
	int ldwork = *pldwork;

    char transt[1];

	int lastv, lastc;
	int i_t0, i_t1;

	int i_1 = 1;
	double d_1 = 1.0;
	double d_m1 = -1.0;

	int ii, jj;

    /* Parameter adjustments */
	// TODO
    int v_offset = 1 + ldv;
    V -= v_offset;
    int t_offset = 1 + ldt;
    T -= t_offset;
    int c_offset = 1 + ldc;
    C -= c_offset;
    int work_offset = 1 + ldwork;
    work -= work_offset;

    /* Function Body */
    if (m <= 0 || n <= 0)
		{
		return;
		}

    if (lsame_(trans, "N"))
		{
		*(unsigned char *)transt = 'T';
		}
	else
		{
		*(unsigned char *)transt = 'N';
		}

    if (lsame_(storev, "C"))
		{

		if (lsame_(direct, "F"))
			{

	/*           Let  V =  ( V1 )    (first K rows) */
	/*                     ( V2 ) */
	/*           where  V1  is unit lower triangular. */

			if (lsame_(side, "L"))
				{

		/*              Form  H * C  or  H' * C  where  C = ( C1 ) */
		/*                                                  ( C2 ) */

				i_t0 = k, i_t1 = iladlr_(&m, &k, &V[v_offset], &ldv);
				lastv = max(i_t0,i_t1);
				lastc = iladlc_(&lastv, &n, &C[c_offset], &ldc);

		/*              W := C' * V  =  (C1'*V1 + C2'*V2)  (stored in WORK) */

		/*              W := C1' */

				i_t0 = k;
				for (jj = 1; jj <= i_t0; ++jj)
					{
					dcopy_(&lastc, &C[jj + ldc], &ldc, &work[jj * ldwork + 1], &i_1);
					}

		/*              W := W * V1 */

				blasfeo_blas_dtrmm("Right", "Lower", "No transpose", "Unit", &lastc, &k, &d_1, &V[v_offset], &ldv, &work[work_offset], &ldwork);
				if (lastv > k)
					{

		/*                 W := W + C2'*V2 */

					i_t0 = lastv - k;
					blasfeo_blas_dgemm("Transpose", "No transpose", &lastc, &k, &i_t0, &d_1, &C[k + 1 + ldc], &ldc, &V[k + 1 + ldv], &ldv, &d_1, &work[work_offset], &ldwork);
					}

		/*              W := W * T'  or  W * T */

				blasfeo_blas_dtrmm("Right", "Upper", transt, "Non-unit", &lastc, &k, &d_1, &T[t_offset], &ldt, &work[work_offset], &ldwork);

		/*              C := C - V * W' */

				if (lastv > k)
					{

		/*                 C2 := C2 - V2 * W' */

					i_t0 = lastv - k;
					blasfeo_blas_dgemm("No transpose", "Transpose", &i_t0, &lastc, &k, &d_m1, &V[k + 1 + ldv], &ldv, &work[work_offset], &ldwork, &d_1, &C[k + 1 + ldc], &ldc);
					}

		/*              W := W * V1' */

				blasfeo_blas_dtrmm("Right", "Lower", "Transpose", "Unit", &lastc, &k, &d_1, &V[v_offset], &ldv, &work[work_offset], &ldwork);

		/*              C1 := C1 - W' */

				i_t0 = k;
				for (jj = 1; jj <= i_t0; jj++)
					{
					i_t1 = lastc;
					for (ii = 1; ii <= i_t1; ii++)
						{
						C[jj + ii * ldc] -= work[ii + jj * ldwork];
						}
					}

				}
			else if (lsame_(side, "R"))
				{

		/*              Form  C * H  or  C * H'  where  C = ( C1  C2 ) */

				i_t0 = k, i_t1 = iladlr_(&n, &k, &V[v_offset], &ldv);
				lastv = max(i_t0,i_t1);
				lastc = iladlr_(&m, &lastv, &C[c_offset], &ldc);

		/*              W := C * V  =  (C1*V1 + C2*V2)  (stored in WORK) */

		/*              W := C1 */

				i_t0 = k;
				for (jj = 1; jj <= i_t0; ++jj)
					{
					dcopy_(&lastc, &C[jj * ldc + 1], &i_1, &work[jj * ldwork + 1], &i_1);
					}

		/*              W := W * V1 */

				blasfeo_blas_dtrmm("Right", "Lower", "No transpose", "Unit", &lastc, &k, &d_1, &V[v_offset], &ldv, &work[work_offset], &ldwork);
				if (lastv > k)
					{

		/*                 W := W + C2 * V2 */

					i_t0 = lastv - k;
					blasfeo_blas_dgemm("No transpose", "No transpose", &lastc, &k, &i_t0, &d_1, &C[(k + 1) * ldc + 1], &ldc, &V[k + 1 + ldv], &ldv, &d_1, &work[work_offset], &ldwork);
					}

		/*              W := W * T  or  W * T' */

				blasfeo_blas_dtrmm("Right", "Upper", trans, "Non-unit", &lastc, &k, &d_1, &T[t_offset], &ldt, &work[work_offset], &ldwork);

		/*              C := C - W * V' */

				if (lastv > k)
					{

		/*                 C2 := C2 - W * V2' */

					i_t0 = lastv - k;
					blasfeo_blas_dgemm("No transpose", "Transpose", &lastc, &i_t0, &k, &d_m1, &work[work_offset], &ldwork, &V[k + 1 + ldv], &ldv, &d_1, &C[(k + 1) * ldc + 1], &ldc);
					}

		/*              W := W * V1' */

				blasfeo_blas_dtrmm("Right", "Lower", "Transpose", "Unit", &lastc, &k, &d_1, &V[v_offset], &ldv, &work[work_offset], &ldwork);

		/*              C1 := C1 - W */

				i_t0 = k;
				for (jj = 1; jj <= i_t0; jj++)
					{
					i_t1 = lastc;
					for (ii = 1; ii <= i_t1; ii++)
						{
					C[ii + jj * ldc] -= work[ii + jj * ldwork];
						}
					}
				}

			}
		else
			{

	/*           Let  V =  ( V1 ) */
	/*                     ( V2 )    (last K rows) */
	/*           where  V2  is unit upper triangular. */

			if (lsame_(side, "L"))
				{

		/*              Form  H * C  or  H' * C  where  C = ( C1 ) */
		/*                                                  ( C2 ) */

				i_t0 = k, i_t1 = iladlr_(&m, &k, &V[v_offset], &ldv);
				lastv = max(i_t0,i_t1);
				lastc = iladlc_(&lastv, &n, &C[c_offset], &ldc);

		/*              W := C' * V  =  (C1'*V1 + C2'*V2)  (stored in WORK) */

		/*              W := C2' */

				i_t0 = k;
				for (jj = 1; jj <= i_t0; jj++)
					{
					dcopy_(&lastc, &C[lastv - k + jj + ldc], &ldc, &work[jj * ldwork + 1], &i_1);
					}

		/*              W := W * V2 */

				blasfeo_blas_dtrmm("Right", "Upper", "No transpose", "Unit", &lastc, &k, &d_1, &V[lastv - k + 1 + ldv], &ldv, &work[work_offset], &ldwork);
				if (lastv > k)
					{

		/*                 W := W + C1'*V1 */

					i_t0 = lastv - k;
					blasfeo_blas_dgemm("Transpose", "No transpose", &lastc, &k, &i_t0, &d_1, &C[c_offset], &ldc, &V[v_offset], &ldv, &d_1, &work[work_offset], &ldwork);
					}

		/*              W := W * T'  or  W * T */

				blasfeo_blas_dtrmm("Right", "Lower", transt, "Non-unit", &lastc, &k, &d_1, &T[t_offset], &ldt, &work[work_offset], &ldwork);

		/*              C := C - V * W' */

				if (lastv > k)
					{

		/*                 C1 := C1 - V1 * W' */

					i_t0 = lastv - k;
					blasfeo_blas_dgemm("No transpose", "Transpose", &i_t0, &lastc, &k, &d_m1, &V[v_offset], &ldv, &work[work_offset], &ldwork, &d_1, &C[c_offset], &ldc);
				}

		/*              W := W * V2' */

				blasfeo_blas_dtrmm("Right", "Upper", "Transpose", "Unit", &lastc, &k, &d_1, &V[lastv - k + 1 + ldv], &ldv, &work[work_offset], &ldwork);

		/*              C2 := C2 - W' */

				i_t0 = k;
				for (jj = 1; jj <= i_t0; jj++)
					{
					i_t1 = lastc;
					for (ii = 1; ii <= i_t1; ii++)
						{
						C[lastv - k + jj + ii * ldc] -= work[ii + jj * ldwork];
						}
					}

				}
			else if (lsame_(side, "R"))
				{

		/*              Form  C * H  or  C * H'  where  C = ( C1  C2 ) */

				i_t0 = k, i_t1 = iladlr_(&n, &k, &V[v_offset], &ldv);
				lastv = max(i_t0,i_t1);
				lastc = iladlr_(&m, &lastv, &C[c_offset], &ldc);

		/*              W := C * V  =  (C1*V1 + C2*V2)  (stored in WORK) */

		/*              W := C2 */

				i_t0 = k;
				for (jj = 1; jj <= i_t0; jj++)
					{
					dcopy_(&lastc, &C[(n - k + jj) * ldc + 1], &i_1, &work[jj * ldwork + 1], &i_1);
					}

		/*              W := W * V2 */

				blasfeo_blas_dtrmm("Right", "Upper", "No transpose", "Unit", &lastc, &k, &d_1, &V[lastv - k + 1 + ldv], &ldv, &work[work_offset], &ldwork);
				if (lastv > k)
					{

		/*                 W := W + C1 * V1 */

					i_t0 = lastv - k;
					blasfeo_blas_dgemm("No transpose", "No transpose", &lastc, &k, &i_t0, &d_1, &C[c_offset], &ldc, &V[v_offset], &ldv, &d_1, &work[work_offset], &ldwork);
					}

		/*              W := W * T  or  W * T' */

				blasfeo_blas_dtrmm("Right", "Lower", trans, "Non-unit", &lastc, &k, &d_1, &T[t_offset], &ldt, &work[work_offset], &ldwork);

		/*              C := C - W * V' */

				if (lastv > k)
					{

		/*                 C1 := C1 - W * V1' */

					i_t0 = lastv - k;
					blasfeo_blas_dgemm("No transpose", "Transpose", &lastc, &i_t0, &k, &d_m1, &work[work_offset], &ldwork, &V[v_offset], &ldv, &d_1, &C[c_offset], &ldc);
					}

		/*              W := W * V2' */

				blasfeo_blas_dtrmm("Right", "Upper", "Transpose", "Unit", &lastc, &k, &d_1, &V[lastv - k + 1 + ldv], &ldv, &work[work_offset], &ldwork);

		/*              C2 := C2 - W */

				i_t0 = k;
				for (jj = 1; jj <= i_t0; jj++)
					{
					i_t1 = lastc;
					for (ii = 1; ii <= i_t1; ++ii)
						{
						C[ii + (lastv - k + jj) * ldc] -= work[ii + jj * ldwork];
						}
					}
				}
			}

		}
	else if (lsame_(storev, "R"))
		{

		if (lsame_(direct, "F"))
			{

	/*           Let  V =  ( V1  V2 )    (V1: first K columns) */
	/*           where  V1  is unit upper triangular. */

			if (lsame_(side, "L"))
				{

		/*              Form  H * C  or  H' * C  where  C = ( C1 ) */
		/*                                                  ( C2 ) */

				i_t0 = k, i_t1 = iladlc_(&k, &m, &V[v_offset], &ldv);
				lastv = max(i_t0,i_t1);
				lastc = iladlc_(&lastv, &n, &C[c_offset], &ldc);

		/*              W := C' * V'  =  (C1'*V1' + C2'*V2') (stored in WORK) */

		/*              W := C1' */

				i_t0 = k;
				for (jj = 1; jj <= i_t0; jj++)
					{
					dcopy_(&lastc, &C[jj + ldc], &ldc, &work[jj * ldwork + 1], &i_1);
					}

		/*              W := W * V1' */

				blasfeo_blas_dtrmm("Right", "Upper", "Transpose", "Unit", &lastc, &k, &d_1, &V[v_offset], &ldv, &work[work_offset], &ldwork);
				if (lastv > k)
					{

		/*                 W := W + C2'*V2' */

					i_t0 = lastv - k;
					blasfeo_blas_dgemm("Transpose", "Transpose", &lastc, &k, &i_t0, &d_1, &C[k + 1 + ldc], &ldc, &V[(k + 1) * ldv + 1], &ldv, &d_1, &work[work_offset], &ldwork);
					}

		/*              W := W * T'  or  W * T */

				blasfeo_blas_dtrmm("Right", "Upper", transt, "Non-unit", &lastc, &k, &
					d_1, &T[t_offset], &ldt, &work[work_offset], &ldwork);

		/*              C := C - V' * W' */

				if (lastv > k)
					{

		/*                 C2 := C2 - V2' * W' */

					i_t0 = lastv - k;
					blasfeo_blas_dgemm("Transpose", "Transpose", &i_t0, &lastc, &k, &d_m1, &V[(k + 1) * ldv + 1], &ldv, &work[work_offset], &ldwork, &d_1, &C[k + 1 + ldc], &ldc);
					}

		/*              W := W * V1 */

				blasfeo_blas_dtrmm("Right", "Upper", "No transpose", "Unit", &lastc, &k, &d_1, &V[v_offset], &ldv, &work[work_offset], &ldwork);

		/*              C1 := C1 - W' */

				i_t0 = k;
				for (jj = 1; jj <= i_t0; jj++)
					{
					i_t1 = lastc;
					for (ii = 1; ii <= i_t1; ii++)
						{
						C[jj + ii * ldc] -= work[ii + jj * ldwork];
						}
					}

				}
			else if (lsame_(side, "R"))
				{

		/*              Form  C * H  or  C * H'  where  C = ( C1  C2 ) */

				i_t0 = k, i_t1 = iladlc_(&k, &n, &V[v_offset], &ldv);
				lastv = max(i_t0,i_t1);
				lastc = iladlr_(&m, &lastv, &C[c_offset], &ldc);

		/*              W := C * V'  =  (C1*V1' + C2*V2')  (stored in WORK) */

		/*              W := C1 */

				i_t0 = k;
				for (jj = 1; jj <= i_t0; jj++)
					{
					dcopy_(&lastc, &C[jj * ldc + 1], &i_1, &work[jj * ldwork + 1], &i_1);
					}

		/*              W := W * V1' */

				blasfeo_blas_dtrmm("Right", "Upper", "Transpose", "Unit", &lastc, &k, &d_1, &V[v_offset], &ldv, &work[work_offset], &ldwork);
				if (lastv > k)
					{

		/*                 W := W + C2 * V2' */

					i_t0 = lastv - k;
					blasfeo_blas_dgemm("No transpose", "Transpose", &lastc, &k, &i_t0, &d_1, &C[(k + 1) * ldc + 1], &ldc, &V[(k + 1) * ldv + 1], &ldv, &d_1, &work[work_offset], &ldwork);
					}

		/*              W := W * T  or  W * T' */

				blasfeo_blas_dtrmm("Right", "Upper", trans, "Non-unit", &lastc, &k, &d_1, &T[t_offset], &ldt, &work[work_offset], &ldwork);

		/*              C := C - W * V */

				if (lastv > k)
					{

		/*                 C2 := C2 - W * V2 */

					i_t0 = lastv - k;
					blasfeo_blas_dgemm("No transpose", "No transpose", &lastc, &i_t0, &k, &d_m1, &work[work_offset], &ldwork, &V[(k + 1) * ldv + 1], &ldv, &d_1, &C[(k + 1) * ldc + 1], &ldc);
					}

		/*              W := W * V1 */

				blasfeo_blas_dtrmm("Right", "Upper", "No transpose", "Unit", &lastc, &k, &d_1, &V[v_offset], &ldv, &work[work_offset], &ldwork);

		/*              C1 := C1 - W */

				i_t0 = k;
				for (jj = 1; jj <= i_t0; jj++)
					{
					i_t1 = lastc;
					for (ii = 1; ii <= i_t1; ii++)
						{
						C[ii + jj * ldc] -= work[ii + jj * ldwork];
						}
					}

				}

			}
		else
			{

	/*           Let  V =  ( V1  V2 )    (V2: last K columns) */
	/*           where  V2  is unit lower triangular. */

			if (lsame_(side, "L"))
				{

		/*              Form  H * C  or  H' * C  where  C = ( C1 ) */
		/*                                                  ( C2 ) */

				i_t0 = k, i_t1 = iladlc_(&k, &m, &V[v_offset], &ldv);
				lastv = max(i_t0,i_t1);
				lastc = iladlc_(&lastv, &n, &C[c_offset], &ldc);

		/*              W := C' * V'  =  (C1'*V1' + C2'*V2') (stored in WORK) */

		/*              W := C2' */

				i_t0 = k;
				for (jj = 1; jj <= i_t0; jj++)
					{
					dcopy_(&lastc, &C[lastv - k + jj + ldc], &ldc, &work[jj * ldwork + 1], &i_1);
					}

		/*              W := W * V2' */

				blasfeo_blas_dtrmm("Right", "Lower", "Transpose", "Unit", &lastc, &k, &d_1, &V[(lastv - k + 1) * ldv + 1], &ldv, &work[work_offset], &ldwork);
				if (lastv > k)	
					{

		/*                 W := W + C1'*V1' */

					i_t0 = lastv - k;
					blasfeo_blas_dgemm("Transpose", "Transpose", &lastc, &k, &i_t0, &d_1, &C[c_offset], &ldc, &V[v_offset], &ldv, &d_1, &work[work_offset], &ldwork);
					}

		/*              W := W * T'  or  W * T */

				blasfeo_blas_dtrmm("Right", "Lower", transt, "Non-unit", &lastc, &k, &d_1, &T[t_offset], &ldt, &work[work_offset], &ldwork);

		/*              C := C - V' * W' */

				if (lastv > k)
					{

		/*                 C1 := C1 - V1' * W' */

					i_t0 = lastv - k;
					blasfeo_blas_dgemm("Transpose", "Transpose", &i_t0, &lastc, &k, &d_m1, &V[v_offset], &ldv, &work[work_offset], &ldwork, &d_1, &C[c_offset], &ldc);
					}

		/*              W := W * V2 */

				blasfeo_blas_dtrmm("Right", "Lower", "No transpose", "Unit", &lastc, &k, &d_1, &V[(lastv - k + 1) * ldv + 1], &ldv, &work[work_offset], &ldwork);

		/*              C2 := C2 - W' */

				i_t0 = k;
				for (jj = 1; jj <= i_t0; jj++)
					{
					i_t1 = lastc;
					for (ii = 1; ii <= i_t1; ii++)
						{
						C[lastv - k + jj + ii * ldc] -= work[ii + jj * ldwork];
						}
					}

				}
			else if (lsame_(side, "R"))
				{

		/*              Form  C * H  or  C * H'  where  C = ( C1  C2 ) */

				i_t0 = k, i_t1 = iladlc_(&k, &n, &V[v_offset], &ldv);
				lastv = max(i_t0,i_t1);
				lastc = iladlr_(&m, &lastv, &C[c_offset], &ldc);

		/*              W := C * V'  =  (C1*V1' + C2*V2')  (stored in WORK) */

		/*              W := C2 */

				i_t0 = k;
				for (jj = 1; jj <= i_t0; jj++)
					{
					dcopy_(&lastc, &C[(lastv - k + jj) * ldc + 1], &i_1, &work[jj * ldwork + 1], &i_1);
					}

		/*              W := W * V2' */

				blasfeo_blas_dtrmm("Right", "Lower", "Transpose", "Unit", &lastc, &k, &d_1, &V[(lastv - k + 1) * ldv + 1], &ldv, &work[work_offset], &ldwork);
				if (lastv > k)
					{

		/*                 W := W + C1 * V1' */

					i_t0 = lastv - k;
					blasfeo_blas_dgemm("No transpose", "Transpose", &lastc, &k, &i_t0, &d_1, &C[c_offset], &ldc, &V[v_offset], &ldv, &d_1, &work[work_offset], &ldwork);
					}

		/*              W := W * T  or  W * T' */

				blasfeo_blas_dtrmm("Right", "Lower", trans, "Non-unit", &lastc, &k, &d_1, &T[t_offset], &ldt, &work[work_offset], &ldwork);

		/*              C := C - W * V */

				if (lastv > k)
					{

		/*                 C1 := C1 - W * V1 */

					i_t0 = lastv - k;
					blasfeo_blas_dgemm("No transpose", "No transpose", &lastc, &i_t0, &k, &d_m1, &work[work_offset], &ldwork, &V[v_offset],	&ldv, &d_1, &C[c_offset], &ldc);
					}

		/*              W := W * V2 */

				blasfeo_blas_dtrmm("Right", "Lower", "No transpose", "Unit", &lastc, &k, &d_1, &V[(lastv - k + 1) * ldv + 1], &ldv, &work[work_offset], &ldwork);

		/*              C1 := C1 - W */

				i_t0 = k;
				for (jj = 1; jj <= i_t0; jj++)
					{
					i_t1 = lastc;
					for (ii = 1; ii <= i_t1; ii++)
						{
						C[ii + (lastv - k + jj) * ldc] -= work[ii + jj * ldwork];
						}
					}

				}

			}
		}

    return;

/*     End of DLARFB */

	} /* dlarfb_ */

