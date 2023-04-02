#ifndef BLAS_H
#define BLAS_H
/** \file   blas.h
 *  \brief  Header file for Fortran BLAS.
 */


#define F77_CALL(x)  x ## _
#define F77_NAME(x)  F77_CALL(x)

#ifdef  __cplusplus
extern "C" {
#endif

/* Level 1 BLAS */

extern double F77_NAME(dasum)(const int *n, const double *dx, const int *incx);
extern void   F77_NAME(daxpy)(const int *n, const double *alpha,
                              const double *dx, const int *incx,
                              double *dy, const int *incy);
extern void   F77_NAME(dcopy)(const int *n, const double *dx, const int *incx,
                              double *dy, const int *incy);
extern double F77_NAME(ddot) (const int *n, const double *dx, const int *incx,
                              const double *dy, const int *incy);
extern double F77_NAME(dnrm2)(const int *n, const double *dx, const int *incx);
extern void   F77_NAME(drot) (const int *n, double *dx, const int *incx,
                              double *dy, const int *incy, const double *c,
                              const double *s);
extern void   F77_NAME(drotg)(const double *a, const double *b, double *c, 
                              double *s);
extern void   F77_NAME(drotm)(const int *n, double *dx, const int *incx,
                              double *dy, const int *incy,const double *dparam);
extern void   F77_NAME(drotmg)(const double *dd1, const double *dd2, 
                               const double *dx1, const double *dy1, 
                               double *param);
extern void   F77_NAME(dscal)(const int *n, const double *alpha, double *dx,
                              const int *incx);
extern void   F77_NAME(dswap)(const int *n, double *dx, const int *incx,
                              double *dy, const int *incy);
extern int    F77_NAME(idamax)(const int *n, const double *dx, const int *incx);

/* Level 2 BLAS */

extern void   F77_NAME(dgbmv)(const char *trans, const int *m, const int *n,
                              const int *kl,const int *ku, const double *alpha,
                              const double *a, const int *lda, const double *x,
                              const int *incx, const double *beta, double *y,
                              const int *incy);
extern void   F77_NAME(dgemv)(const char *trans, const int *m, const int *n,
                              const double *alpha, const double *a,
                              const int *lda, const double *x, const int *incx,
                              const double *beta, double *y, const int *incy);
extern void   F77_NAME(dsbmv)(const char *uplo, const int *n, const int *k,
                              const double *alpha, const double *a,
                              const int *lda, const double *x, const int *incx,
                              const double *beta, double *y, const int *incy);
extern void   F77_NAME(dspmv)(const char *uplo, const int *n,
                              const double *alpha, const double *ap,
                              const double *x, const int *incx,
                              const double *beta, double *y, const int *incy);
extern void   F77_NAME(dsymv)(const char *uplo, const int *n,
                              const double *alpha, const double *a,
                              const int *lda, const double *x, const int *incx,
                              const double *beta, double *y, const int *incy);
extern void   F77_NAME(dtbmv)(const char *uplo, const char *trans,
                              const char *diag, const int *n, const int *k,
                              const double *a, const int *lda,
                              double *x, const int *incx);
extern void   F77_NAME(dtpmv)(const char *uplo, const char *trans,
                              const char *diag, const int *n, const double *ap,
                              double *x, const int *incx);
extern void   F77_NAME(dtrmv)(const char *uplo, const char *trans,
                              const char *diag, const int *n, const double *a,
                              const int *lda, double *x, const int *incx);
extern void   F77_NAME(dtbsv)(const char *uplo, const char *trans,
                              const char *diag, const int *n, const int *k,
                              const double *a, const int *lda,
                              double *x, const int *incx);
extern void   F77_NAME(dtpsv)(const char *uplo, const char *trans,
                              const char *diag, const int *n,
                              const double *ap, double *x, const int *incx);
extern void   F77_NAME(dtrsv)(const char *uplo, const char *trans,
                              const char *diag, const int *n,
                              const double *a, const int *lda,
                              double *x, const int *incx);
extern void   F77_NAME(dger) (const int *m, const int *n, const double *alpha,
                              double *x, const int *incx,
                              double *y, const int *incy,
                              double *a, const int *lda);
extern void   F77_NAME(dsyr) (const char *uplo, const int *n,
                              const double *alpha, const double *x,
                              const int *incx, double *a, const int *lda);
extern void   F77_NAME(dspr) (const char *uplo, const int *n,
                              const double *alpha, const double *x,
                              const int *incx, double *ap);
extern void   F77_NAME(dsyr2)(const char *uplo, const int *n, 
                              const double *alpha, const double *x,
                              const int *incx, const double *y, const int *incy,
                              double *a, const int *lda);
extern void   F77_NAME(dspr2)(const char *uplo, const int *n,
                              const double *alpha, const double *x,
                              const int *incx, const double *y,
                              const int *incy, double *ap);

/* Level 3 BLAS */

extern void   F77_NAME(dgemm)(const char *transa, const char *transb,
                              const int *m, const int *n, const int *k,
                              const double *alpha, const double *a,
                              const int *lda, const double *b, const int *ldb,
                              const double *beta, double *c, const int *ldc);
extern void   F77_NAME(dtrsm)(const char *side, const char *uplo,
                              const char *transa, const char *diag,
                              const int *m, const int *n, const double *alpha,
                              const double *a, const int *lda,
                              double *b, const int *ldb);
extern void   F77_NAME(dtrmm)(const char *side, const char *uplo,
                              const char *transa, const char *diag,
                              const int *m, const int *n, const double *alpha,
                              const double *a, const int *lda,
                              double *b, const int *ldb);
extern void   F77_NAME(dsymm)(const char *side, const char *uplo, const int *m,
                              const int *n, const double *alpha,
                              const double *a, const int *lda,
                              const double *b, const int *ldb,
                              const double *beta, double *c, const int *ldc);
extern void   F77_NAME(dsyrk)(const char *uplo, const char *trans,
                              const int *n, const int *k,
                              const double *alpha, const double *a,
                              const int *lda, const double *beta,
                              double *c, const int *ldc);
extern void   F77_NAME(dsyr2k)(const char *uplo, const char *trans,
                               const int *n, const int *k,
                               const double *alpha, const double *a,
                               const int *lda, const double *b, const int *ldb,
                               const double *beta, double *c, const int *ldc);

#ifdef  __cplusplus
}
#endif

#endif /* BLAS_H */
