/*  -- LAPACK auxiliary routine (version 3.2) -- */
/*     Univ. of Tennessee, Univ. of California Berkeley and NAG Ltd.. */
/*     November 2006 */

/*  Purpose */
/*  ======= */

/*  DLARF applies a real elementary reflector H to a real m by n matrix */
/*  C, from either the left or the right. H is represented in the form */

/*        H = I - tau * v * v' */

/*  where tau is a real scalar and v is a real vector. */

/*  If tau = 0, then H is taken to be the unit matrix. */

/*  Arguments */
/*  ========= */

/*  SIDE    (input) CHARACTER*1 */
/*          = 'L': form  H * C */
/*          = 'R': form  C * H */

/*  M       (input) INTEGER */
/*          The number of rows of the matrix C. */

/*  N       (input) INTEGER */
/*          The number of columns of the matrix C. */

/*  V       (input) DOUBLE PRECISION array, dimension */
/*                     (1 + (M-1)*abs(INCV)) if SIDE = 'L' */
/*                  or (1 + (N-1)*abs(INCV)) if SIDE = 'R' */
/*          The vector v in the representation of H. V is not used if */
/*          TAU = 0. */

/*  INCV    (input) INTEGER */
/*          The increment between elements of v. INCV <> 0. */

/*  TAU     (input) DOUBLE PRECISION */
/*          The value tau in the representation of H. */

/*  C       (input/output) DOUBLE PRECISION array, dimension (LDC,N) */
/*          On entry, the m by n matrix C. */
/*          On exit, C is overwritten by the matrix H * C if SIDE = 'L', */
/*          or C * H if SIDE = 'R'. */

/*  LDC     (input) INTEGER */
/*          The leading dimension of the array C. LDC >= max(1,M). */

/*  WORK    (workspace) DOUBLE PRECISION array, dimension */
/*                         (N) if SIDE = 'L' */
/*                      or (M) if SIDE = 'R' */

/*  ===================================================================== */

#include <math.h>
#include <stdbool.h>

#include <blasfeo_d_blas_api.h>



#if defined(FORTRAN_BLAS_API)
#define blasfeo_blas_dgemv dgemv_
#define blasfeo_blas_dger dger_
#define blasfeo_lapack_dlarf dlarf_
#endif



#define min(x, y) (((x) < (y)) ? (x) : (y))
#define max(x, y) (((x) > (y)) ? (x) : (y))



int iladlc_(int *, int *, double *, int *);
int iladlr_(int *, int *, double *, int *);
bool lsame_(char *, char *);



void blasfeo_lapack_dlarf(char *side, int *pm, int *pn, double *v, int *pincv, double *tau, double *C, int *pldc, double *work)
	{

	int m = *pm;
	int n = *pn;
	int ldc = *pldc;
	int incv = *pincv;

	int i_1 = 1;
	double d_1 = 1.0;
	double d_0 = 0.0;

	double d_t0;

	int ii;

    /* Function Body */
    bool applyleft = lsame_(side, "L");
    int lastv = 0;
    int lastc = 0;
    if (*tau != 0.0)
		{
/*     Set up variables for scanning V.  LASTV begins pointing to the end */
/*     of V. */
		if (applyleft)
			{
			lastv = m;
			}
		else
			{
			lastv = n;
			}
		if (incv > 0)
			{
			ii = (lastv - 1) * incv + 1;
			}
		else
			{
			ii = 1;
			}
/*     Look for the last non-zero row in V. */
		while(lastv > 0 && v[ii-1] == 0.0)
			{
			lastv--;
			ii -= incv;
			}
		if (applyleft)
			{
/*     Scan for the last non-zero column in C(1:lastv,:). */
			lastc = iladlc_(&lastv, &n, &C[0], &ldc);
			}
		else
			{
/*     Scan for the last non-zero row in C(:,1:lastv). */
			lastc = iladlr_(&m, &lastv, &C[0], &ldc);
			}
		}
/*     Note that lastc.eq.0 renders the BLAS operations null; no special */
/*     case is needed at this level. */
    if (applyleft)
		{

/*        Form  H * C */

		if (lastv > 0)
			{

/*           w(1:lastc,1) := C(1:lastv,1:lastc)' * v(1:lastv,1) */

			blasfeo_blas_dgemv("Transpose", &lastv, &lastc, &d_1, &C[0], &ldc, &v[0], &incv, &d_0, &work[0], &i_1);

/*           C(1:lastv,1:lastc) := C(...) - v(1:lastv,1) * w(1:lastc,1)' */

			d_t0 = -(*tau);
			blasfeo_blas_dger(&lastv, &lastc, &d_t0, &v[0], &incv, &work[0], &i_1, &C[0], &ldc);
			}
		}
	else
		{

/*        Form  C * H */

		if (lastv > 0)
			{

/*           w(1:lastc,1) := C(1:lastc,1:lastv) * v(1:lastv,1) */

			blasfeo_blas_dgemv("No transpose", &lastc, &lastv, &d_1, &C[0], &ldc, &v[0], &incv, &d_0, &work[0], &i_1);

/*           C(1:lastc,1:lastv) := C(...) - w(1:lastc,1) * v(1:lastv,1)' */

			d_t0 = -(*tau);
			blasfeo_blas_dger(&lastc, &lastv, &d_t0, &work[0], &i_1, &v[0], &incv, &C[0], &ldc);
			}
		}

    return;

/*     End of DLARF */

	} /* dlarf_ */
