#ifndef LAPACK_H
#define LAPACK_H

#include "blas.h"

#ifdef  __cplusplus
extern "C" {
#endif

extern void F77_NAME(dbdsqr)(const char* uplo, const int* n, const int* ncvt,
		 const int* nru, const int* ncc, double* d, double* e,
		 double* vt, const int* ldvt, double* u, const int* ldu,
		 double* c, const int* ldc, double* work, int* info);
extern void F77_NAME(ddisna)(const char* job, const int* m, const int* n,
		 double* d, double* sep, int* info);

extern void F77_NAME(dgbbrd)(const char* vect, const int* m, const int* n,
		 const int* ncc, const int* kl, const int* ku,
		 double* ab, const int* ldab,
		 double* d, double* e, double* q,
		 const int* ldq, double* pt, const int* ldpt,
		 double* c, const int* ldc,
		 double* work, int* info);
extern void F77_NAME(dgbcon)(const char* norm, const int* n, const int* kl,
		 const int* ku, double* ab, const int* ldab,
		 int* ipiv, const double* anorm, double* rcond,
		 double* work, int* iwork, int* info);
extern void F77_NAME(dgbequ)(const int* m, const int* n, const int* kl, const int* ku,
		 double* ab, const int* ldab, double* r, double* c,
		 double* rowcnd, double* colcnd, double* amax, int* info);
extern void F77_NAME(dgbrfs)(const char* trans, const int* n, const int* kl,
		 const int* ku, const int* nrhs, double* ab,
		 const int* ldab, double* afb, const int* ldafb,
		 int* ipiv, double* b, const int* ldb,
		 double* x, const int* ldx, double* ferr, double* berr,
		 double* work, int* iwork, int* info);
extern void F77_NAME(dgbsv)(const int* n, const int* kl,const int* ku,
		const int* nrhs, double* ab, const int* ldab,
		int* ipiv, double* b, const int* ldb, int* info);
extern void F77_NAME(dgbsvx)(const int* fact, const char* trans,
		 const int* n, const int* kl,const int* ku,
		 const int* nrhs, double* ab, const int* ldab,
		 double* afb, const int* ldafb, int* ipiv,
		 const char* equed, double* r, double* c, 
		 double* b, const int* ldb,
		 double* x, const int* ldx,
		 double* rcond, double* ferr, double* berr,
		 double* work, int* iwork, int* info);
extern void F77_NAME(dgbtf2)(const int* m, const int* n, const int* kl,const int* ku,
		 double* ab, const int* ldab, int* ipiv, int* info);
extern void F77_NAME(dgbtrf)(const int* m, const int* n, const int* kl,const int* ku,
    		  double* ab, const int* ldab, int* ipiv, int* info);
extern void F77_NAME(dgbtrs)(const char* trans, const int* n,
		 const int* kl, const int* ku, const int* nrhs,
		 const double* ab, const int* ldab, const int* ipiv,
		 double* b, const int* ldb, int* info);

extern void F77_NAME(dgebak)(const char* job, const char* side, const int* n,
		 const int* ilo, const int* ihi, double* scale,
		 const int* m, double* v, const int* ldv, int* info);
extern void F77_NAME(dgebal)(const char* job, const int* n, double* a, const int* lda,
    		  int* ilo, int* ihi, double* scale, int* info);
extern void F77_NAME(dgebd2)(const int* m, const int* n, double* a, const int* lda,
		 double* d, double* e, double* tauq, double* taup,
		 double* work, int* info);
extern void F77_NAME(dgebrd)(const int* m, const int* n, double* a, const int* lda,
		 double* d, double* e, double* tauq, double* taup,
		 double* work, const int* lwork, int* info);
extern void F77_NAME(dgecon)(const char* norm, const int* n,
		 const double* a, const int* lda,
		 const double* anorm, double* rcond,
		 double* work, int* iwork, int* info);
extern void F77_NAME(dgeequ)(const int* m, const int* n, double* a, const int* lda,
		 double* r, double* c, double* rowcnd, double* colcnd,
		 double* amax, int* info);
extern void F77_NAME(dgees)(const char* jobvs, const char* sort,
		int (*select)(const double*, const double*),
		const int* n, double* a, const int* lda,
		int* sdim, double* wr, double* wi,
		double* vs, const int* ldvs,
		double* work, const int* lwork, int* bwork, int* info);
extern void F77_NAME(dgeesx)(const char* jobvs, const char* sort,
		 int (*select)(const double*, const double*),
		 const char* sense, const int* n, double* a,
		 const int* lda, int* sdim, double* wr, double* wi,
		 double* vs, const int* ldvs, double* rconde,
		 double* rcondv, double* work, const int* lwork,
		 int* iwork, const int* liwork, int* bwork, int* info);
extern void F77_NAME(dgeev)(const char* jobvl, const char* jobvr,
		const int* n, double* a, const int* lda,
		double* wr, double* wi, double* vl, const int* ldvl,
		double* vr, const int* ldvr,
		double* work, const int* lwork, int* info);
extern void F77_NAME(dgeevx)(const char* balanc, const char* jobvl, const char* jobvr,
		 const char* sense, const int* n, double* a, const int* lda,
		 double* wr, double* wi, double* vl, const int* ldvl,
		 double* vr, const int* ldvr, int* ilo, int* ihi,
		 double* scale, double* abnrm, double* rconde, double* rcondv,
		 double* work, const int* lwork, int* iwork, int* info);
extern void F77_NAME(dgegv)(const char* jobvl, const char* jobvr,
		const int* n, double* a, const int* lda,
		double* b, const int* ldb,
		double* alphar, double* alphai,
		const double* beta, double* vl, const int* ldvl,
		double* vr, const int* ldvr,
		double* work, const int* lwork, int* info);
extern void F77_NAME(dgehd2)(const int* n, const int* ilo, const int* ihi,
		 double* a, const int* lda, double* tau,
		 double* work, int* info);
extern void F77_NAME(dgehrd)(const int* n, const int* ilo, const int* ihi,
		 double* a, const int* lda, double* tau,
		 double* work, const int* lwork, int* info);
extern void F77_NAME(dgelq2)(const int* m, const int* n,
		 double* a, const int* lda, double* tau,
		 double* work, int* info);
extern void F77_NAME(dgelqf)(const int* m, const int* n,
		 double* a, const int* lda, double* tau,
		 double* work, const int* lwork, int* info);
extern void F77_NAME(dgels)(const char* trans, const int* m, const int* n,
		const int* nrhs, double* a, const int* lda,
		double* b, const int* ldb,
		double* work, const int* lwork, int* info);
extern void F77_NAME(dgelss)(const int* m, const int* n, const int* nrhs,
		 double* a, const int* lda, double* b, const int* ldb,
		 double* s, double* rcond, int* rank,
		 double* work, const int* lwork, int* info);
extern void F77_NAME(dgelsy)(const int* m, const int* n, const int* nrhs,
		 double* a, const int* lda, double* b, const int* ldb,
		 int* jpvt, const double* rcond, int* rank,
		 double* work, const int* lwork, int* info);
extern void F77_NAME(dgeql2)(const int* m, const int* n, double* a, const int* lda,
		 double* tau, double* work, int* info);
extern void F77_NAME(dgeqlf)(const int* m, const int* n,
		 double* a, const int* lda, double* tau,
		 double* work, const int* lwork, int* info);
extern void F77_NAME(dgeqp3)(const int* m, const int* n, double* a, const int* lda,
		 int* jpvt, double* tau, double* work, const int* lwork,
		 int* info);
extern void F77_NAME(dgeqpf)(const int* m, const int* n, double* a, const int* lda,
		 int* jpvt, double* tau, double* work, int* info);
extern void F77_NAME(dgeqr2)(const int* m, const int* n, double* a, const int* lda,
		 double* tau, double* work, int* info);
extern void F77_NAME(dgeqrf)(const int* m, const int* n, double* a, const int* lda,
		 double* tau, double* work, const int* lwork, int* info);
extern void F77_NAME(dgerfs)(const char* trans, const int* n, const int* nrhs,
		 double* a, const int* lda, double* af, const int* ldaf,
		 int* ipiv, double* b, const int* ldb,
		 double* x, const int* ldx, double* ferr, double* berr,
		 double* work, int* iwork, int* info);
extern void F77_NAME(dgerq2)(const int* m, const int* n, double* a, const int* lda,
		 double* tau, double* work, int* info);
extern void F77_NAME(dgerqf)(const int* m, const int* n, double* a, const int* lda,
		 double* tau, double* work, const int* lwork, int* info);
extern void F77_NAME(dgesv)(const int* n, const int* nrhs, double* a, const int* lda,
		int* ipiv, double* b, const int* ldb, int* info);
extern void F77_NAME(dgesvd)(const char* jobu, const char* jobvt, const int* m,
		 const int* n, double* a, const int* lda, double* s,
		 double* u, const int* ldu, double* vt, const int* ldvt,
		 double* work, const int* lwork, int* info);
extern void F77_NAME(dgesvx)(const int* fact, const char* trans, const int* n,
		 const int* nrhs, double* a, const int* lda,
		 double* af, const int* ldaf, int* ipiv,
		 char *equed, double* r, double* c,
		 double* b, const int* ldb,
		 double* x, const int* ldx,
		 double* rcond, double* ferr, double* berr,
		 double* work, int* iwork, int* info);
extern void F77_NAME(dgetf2)(const int* m, const int* n, double* a, const int* lda,
		 int* ipiv, int* info);
extern void F77_NAME(dgetrf)(const int* m, const int* n, double* a, const int* lda,
		 int* ipiv, int* info);
extern void F77_NAME(dgetri)(const int* n, double* a, const int* lda,
		 int* ipiv, double* work, const int* lwork, int* info);
extern void F77_NAME(dgetrs)(const char* trans, const int* n, const int* nrhs,
		 const double* a, const int* lda, const int* ipiv,
		 double* b, const int* ldb, int* info);

extern void F77_NAME(dggbak)(const char* job, const char* side,
		 const int* n, const int* ilo, const int* ihi,
		 double* lscale, double* rscale, const int* m,
		 double* v, const int* ldv, int* info);
extern void F77_NAME(dggbal)(const char* job, const int* n, double* a, const int* lda,
		 double* b, const int* ldb, int* ilo, int* ihi,
		 double* lscale, double* rscale, double* work, int* info);
extern void F77_NAME(dgges)(const char* jobvsl, const char* jobvsr, const char* sort,
		int (*delztg)(double*, double*, double*),
		const int* n, double* a, const int* lda,
		double* b, const int* ldb, double* alphar,
		double* alphai, const double* beta,
		double* vsl, const int* ldvsl,
		double* vsr, const int* ldvsr,
		double* work, const int* lwork, int* bwork, int* info);

extern void F77_NAME(dggglm)(const int* n, const int* m, const int* p,
		 double* a, const int* lda, double* b, const int* ldb,
		 double* d, double* x, double* y,
		 double* work, const int* lwork, int* info);
extern void F77_NAME(dgghrd)(const char* compq, const char* compz, const int* n,
		 const int* ilo, const int* ihi, double* a, const int* lda,
		 double* b, const int* ldb, double* q, const int* ldq,
		 double* z, const int* ldz, int* info);
extern void F77_NAME(dgglse)(const int* m, const int* n, const int* p,
		 double* a, const int* lda,
		 double* b, const int* ldb,
		 double* c, double* d, double* x,
		 double* work, const int* lwork, int* info);
extern void F77_NAME(dggqrf)(const int* n, const int* m, const int* p,
		 double* a, const int* lda, double* taua,
		 double* b, const int* ldb, double* taub,
		 double* work, const int* lwork, int* info);
extern void F77_NAME(dggrqf)(const int* m, const int* p, const int* n,
		 double* a, const int* lda, double* taua,
		 double* b, const int* ldb, double* taub,
		 double* work, const int* lwork, int* info);
extern void F77_NAME(dggsvd)(const char* jobu, const char* jobv, const char* jobq,
		 const int* m, const int* n, const int* p,
		 const int* k, const int* l,
		 double* a, const int* lda,
		 double* b, const int* ldb,
		 const double* alpha, const double* beta,
		 double* u, const int* ldu,
		 double* v, const int* ldv,
		 double* q, const int* ldq,
		 double* work, int* iwork, int* info);

extern void F77_NAME(dgtcon)(const char* norm, const int* n, double* dl, double* d,
		 double* du, double* du2, int* ipiv, const double* anorm,
		 double* rcond, double* work, int* iwork, int* info);
extern void F77_NAME(dgtrfs)(const char* trans, const int* n, const int* nrhs,
		 double* dl, double* d, double* du, double* dlf,
		 double* df, double* duf, double* du2,
		 int* ipiv, double* b, const int* ldb,
		 double* x, const int* ldx,
		 double* ferr, double* berr,
		 double* work, int* iwork, int* info);
extern void F77_NAME(dgtsv)(const int* n, const int* nrhs,
		double* dl, double* d, double* du,
		double* b, const int* ldb, int* info);
extern void F77_NAME(dgtsvx)(const int* fact, const char* trans,
		 const int* n, const int* nrhs,
		 double* dl, double* d, double* du,
		 double* dlf, double* df, double* duf,
		 double* du2, int* ipiv,
		 double* b, const int* ldb,
		 double* x, const int* ldx,
		 double* rcond, double* ferr, double* berr,
		 double* work, int* iwork, int* info);
extern void F77_NAME(dgttrf)(const int* n, double* dl, double* d,
		 double* du, double* du2, int* ipiv, int* info);
extern void F77_NAME(dgttrs)(const char* trans, const int* n, const int* nrhs,
		 double* dl, double* d, double* du, double* du2,
		 int* ipiv, double* b, const int* ldb, int* info);

extern void F77_NAME(dopgtr)(const char* uplo, const int* n,
		 const double* ap, const double* tau,
		 double* q, const int* ldq,
		 double* work, int* info);
extern void F77_NAME(dopmtr)(const char* side, const char* uplo,
		 const char* trans, const int* m, const int* n,
		 const double* ap, const double* tau,
		 double* c, const int* ldc,
		 double* work, int* info);
extern void F77_NAME(dorg2l)(const int* m, const int* n, const int* k,
		 double* a, const int* lda,
		 const double* tau, double* work, int* info);
extern void F77_NAME(dorg2r)(const int* m, const int* n, const int* k,
		 double* a, const int* lda,
		 const double* tau, double* work, int* info);
extern void F77_NAME(dorgbr)(const char* vect, const int* m,
		 const int* n, const int* k,
		 double* a, const int* lda,
		 const double* tau, double* work,
		 const int* lwork, int* info);
extern void F77_NAME(dorghr)(const int* n, const int* ilo, const int* ihi,
		 double* a, const int* lda, const double* tau,
		 double* work, const int* lwork, int* info);
extern void F77_NAME(dorgl2)(const int* m, const int* n, const int* k,
		 double* a, const int* lda, const double* tau,
		 double* work, int* info);
extern void F77_NAME(dorglq)(const int* m, const int* n, const int* k,
		 double* a, const int* lda,
		 const double* tau, double* work,
		 const int* lwork, int* info);
extern void F77_NAME(dorgql)(const int* m, const int* n, const int* k,
		 double* a, const int* lda,
		 const double* tau, double* work,
		 const int* lwork, int* info);
extern void F77_NAME(dorgqr)(const int* m, const int* n, const int* k,
		 double* a, const int* lda, const double* tau,
		 double* work, const int* lwork, int* info);
extern void F77_NAME(dorgr2)(const int* m, const int* n, const int* k,
		 double* a, const int* lda, const double* tau,
		 double* work, int* info);
extern void F77_NAME(dorgrq)(const int* m, const int* n, const int* k,
		 double* a, const int* lda, const double* tau,
		 double* work, const int* lwork, int* info);
extern void F77_NAME(dorgtr)(const char* uplo, const int* n,
		 double* a, const int* lda, const double* tau,
		 double* work, const int* lwork, int* info);
extern void F77_NAME(dorm2l)(const char* side, const char* trans,
		 const int* m, const int* n, const int* k,
		 const double* a, const int* lda,
		 const double* tau, double* c, const int* ldc,
		 double* work, int* info);
extern void F77_NAME(dorm2r)(const char* side, const char* trans,
		 const int* m, const int* n, const int* k,
		 const double* a, const int* lda, const double* tau,
		 double* c, const int* ldc, double* work, int* info);
extern void F77_NAME(dormbr)(const char* vect, const char* side, const char* trans,
		 const int* m, const int* n, const int* k,
		 const double* a, const int* lda, const double* tau,
		 double* c, const int* ldc,
		 double* work, const int* lwork, int* info);
extern void F77_NAME(dormhr)(const char* side, const char* trans, const int* m,
		 const int* n, const int* ilo, const int* ihi,
		 const double* a, const int* lda, const double* tau,
		 double* c, const int* ldc,
		 double* work, const int* lwork, int* info);
extern void F77_NAME(dorml2)(const char* side, const char* trans,
		 const int* m, const int* n, const int* k,
		 const double* a, const int* lda, const double* tau,
		 double* c, const int* ldc, double* work, int* info);
extern void F77_NAME(dormlq)(const char* side, const char* trans,
		 const int* m, const int* n, const int* k,
		 const double* a, const int* lda,
		 const double* tau, double* c, const int* ldc,
		 double* work, const int* lwork, int* info);
extern void F77_NAME(dormql)(const char* side, const char* trans,
		 const int* m, const int* n, const int* k,
		 const double* a, const int* lda,
		 const double* tau, double* c, const int* ldc,
		 double* work, const int* lwork, int* info);
extern void F77_NAME(dormqr)(const char* side, const char* trans,
		 const int* m, const int* n, const int* k,
		 const double* a, const int* lda,
		 const double* tau, double* c, const int* ldc,
		 double* work, const int* lwork, int* info);
extern void F77_NAME(dormr2)(const char* side, const char* trans,
		 const int* m, const int* n, const int* k,
		 const double* a, const int* lda,
		 const double* tau, double* c, const int* ldc,
		 double* work, int* info);
extern void F77_NAME(dormrq)(const char* side, const char* trans,
		 const int* m, const int* n, const int* k,
		 const double* a, const int* lda,
		 const double* tau, double* c, const int* ldc,
		 double* work, const int* lwork, int* info);
extern void F77_NAME(dormtr)(const char* side, const char* uplo,
		 const char* trans, const int* m, const int* n,
		 const double* a, const int* lda,
		 const double* tau, double* c, const int* ldc,
		 double* work, const int* lwork, int* info);

extern void F77_NAME(dpbcon)(const char* uplo, const int* n, const int* kd,
		 const double* ab, const int* ldab,
		 const double* anorm, double* rcond,
		 double* work, int* iwork, int* info);
extern void F77_NAME(dpbequ)(const char* uplo, const int* n, const int* kd,
		 const double* ab, const int* ldab,
		 double* s, double* scond, double* amax, int* info);
extern void F77_NAME(dpbrfs)(const char* uplo, const int* n,
		 const int* kd, const int* nrhs,
		 const double* ab, const int* ldab,
		 const double* afb, const int* ldafb,
		 const double* b, const int* ldb,
		 double* x, const int* ldx,
		 double* ferr, double* berr,
		 double* work, int* iwork, int* info);
extern void F77_NAME(dpbstf)(const char* uplo, const int* n, const int* kd,
		 double* ab, const int* ldab, int* info);
extern void F77_NAME(dpbsv)(const char* uplo, const int* n,
		const int* kd, const int* nrhs,
		double* ab, const int* ldab,
		double* b, const int* ldb, int* info);
extern void F77_NAME(dpbsvx)(const int* fact, const char* uplo, const int* n,
		 const int* kd, const int* nrhs,
		 double* ab, const int* ldab,
		 double* afb, const int* ldafb,
		 char* equed, double* s,
		 double* b, const int* ldb,
		 double* x, const int* ldx, double* rcond,
		 double* ferr, double* berr,
		 double* work, int* iwork, int* info);
extern void F77_NAME(dpbtf2)(const char* uplo, const int* n, const int* kd,
		 double* ab, const int* ldab, int* info);
extern void F77_NAME(dpbtrf)(const char* uplo, const int* n, const int* kd,
		 double* ab, const int* ldab, int* info);
extern void F77_NAME(dpbtrs)(const char* uplo, const int* n,
		 const int* kd, const int* nrhs,
		 const double* ab, const int* ldab,
		 double* b, const int* ldb, int* info);

extern void F77_NAME(dpocon)(const char* uplo, const int* n,
		 const double* a, const int* lda,
		 const double* anorm, double* rcond,
		 double* work, int* iwork, int* info);
extern void F77_NAME(dpoequ)(const int* n, const double* a, const int* lda,
		 double* s, double* scond, double* amax, int* info);
extern void F77_NAME(dporfs)(const char* uplo, const int* n, const int* nrhs,
		 const double* a, const int* lda,
		 const double* af, const int* ldaf,
		 const double* b, const int* ldb,
		 double* x, const int* ldx,
		 double* ferr, double* berr,
		 double* work, int* iwork, int* info);
extern void F77_NAME(dposv)(const char* uplo, const int* n, const int* nrhs,
		double* a, const int* lda,
		double* b, const int* ldb, int* info);
extern void F77_NAME(dposvx)(const int* fact, const char* uplo,
		 const int* n, const int* nrhs,
		 double* a, const int* lda,
		 double* af, const int* ldaf, char* equed,
		 double* s, double* b, const int* ldb,
		 double* x, const int* ldx, double* rcond,
		 double* ferr, double* berr, double* work,
		 int* iwork, int* info);
extern void F77_NAME(dpotf2)(const char* uplo, const int* n,
		 double* a, const int* lda, int* info);
extern void F77_NAME(dpotrf)(const char* uplo, const int* n,
		 double* a, const int* lda, int* info);
extern void F77_NAME(dpotri)(const char* uplo, const int* n,
		 double* a, const int* lda, int* info);
extern void F77_NAME(dpotrs)(const char* uplo, const int* n,
		 const int* nrhs, const double* a, const int* lda,
		 double* b, const int* ldb, int* info);
extern void F77_NAME(dppcon)(const char* uplo, const int* n,
		 const double* ap, const double* anorm, double* rcond,
		 double* work, int* iwork, int* info);
extern void F77_NAME(dppequ)(const char* uplo, const int* n,
		 const double* ap, double* s, double* scond,
		 double* amax, int* info);

extern void F77_NAME(dpprfs)(const char* uplo, const int* n, const int* nrhs,
		 const double* ap, const double* afp,
		 const double* b, const int* ldb,
		 double* x, const int* ldx,
		 double* ferr, double* berr,
		 double* work, int* iwork, int* info);
extern void F77_NAME(dppsv)(const char* uplo, const int* n,
		const int* nrhs, const double* ap,
		double* b, const int* ldb, int* info);
extern void F77_NAME(dppsvx)(const int* fact, const char* uplo,
		 const int* n, const int* nrhs, double* ap,
		 double* afp, char* equed, double* s,
		 double* b, const int* ldb,
		 double* x, const int* ldx,
		 double* rcond, double* ferr, double* berr,
		 double* work, int* iwork, int* info);
extern void F77_NAME(dpptrf)(const char* uplo, const int* n, double* ap, int* info);
extern void F77_NAME(dpptri)(const char* uplo, const int* n, double* ap, int* info);
extern void F77_NAME(dpptrs)(const char* uplo, const int* n,
		 const int* nrhs, const double* ap,
    		 double* b, const int* ldb, int* info);

extern void F77_NAME(dptcon)(const int* n,
		 const double* d, const double* e,
    		 const double* anorm, double* rcond,
    		 double* work, int* info);
extern void F77_NAME(dpteqr)(const char* compz, const int* n, double* d,
		 double* e, double* z, const int* ldz,
    		 double* work, int* info);
extern void F77_NAME(dptrfs)(const int* n, const int* nrhs,
    		 const double* d, const double* e,
    		 const double* df, const double* ef,
    		 const double* b, const int* ldb,
    		 double* x, const int* ldx,
    		 double* ferr, double* berr,
    		 double* work, int* info);
extern void F77_NAME(dptsv)(const int* n, const int* nrhs, double* d,
    		double* e, double* b, const int* ldb, int* info);
extern void F77_NAME(dptsvx)(const int* fact, const int* n,
    		 const int* nrhs,
    		 const double* d, const double* e,
    		 double* df, double* ef,
    		 const double* b, const int* ldb,
    		 double* x, const int* ldx, double* rcond,
    		 double* ferr, double* berr,
    		 double* work, int* info);
extern void F77_NAME(dpttrf)(const int* n, double* d, double* e, int* info);
extern void F77_NAME(dpttrs)(const int* n, const int* nrhs,
    		 const double* d, const double* e,
    		 double* b, const int* ldb, int* info); 
extern void F77_NAME(drscl)(const int* n, const double* da,
    		double* x, const int* incx);


extern void F77_NAME(dsbev)(const char* jobz, const char* uplo,
    		const int* n, const int* kd,
    		double* ab, const int* ldab,
    		double* w, double* z, const int* ldz,
    		double* work, int* info);
extern void F77_NAME(dsbevd)(const char* jobz, const char* uplo,
    		 const int* n, const int* kd,
    		 double* ab, const int* ldab,
    		 double* w, double* z, const int* ldz,
    		 double* work, const int* lwork,
    		 int* iwork, const int* liwork, int* info);
extern void F77_NAME(dsbevx)(const char* jobz, const char* range,
    		 const char* uplo, const int* n, const int* kd,
    		 double* ab, const int* ldab,
    		 double* q, const int* ldq,
    		 const double* vl, const double* vu,
    		 const int* il, const int* iu,
    		 const double* abstol,
    		 int* m, double* w,
    		 double* z, const int* ldz,
    		 double* work, int* iwork,
    		 int* ifail, int* info);
extern void F77_NAME(dsbgst)(const char* vect, const char* uplo,
    		 const int* n, const int* ka, const int* kb,
    		 double* ab, const int* ldab,
    		 double* bb, const int* ldbb,
    		 double* x, const int* ldx,
    		 double* work, int* info);
extern void F77_NAME(dsbgv)(const char* jobz, const char* uplo,
    		const int* n, const int* ka, const int* kb,
    		double* ab, const int* ldab,
    		double* bb, const int* ldbb,
    		double* w, double* z, const int* ldz,
    		double* work, int* info);
extern void F77_NAME(dsbtrd)(const char* vect, const char* uplo,
    		 const int* n, const int* kd,
    		 double* ab, const int* ldab,
    		 double* d, double* e,
    		 double* q, const int* ldq,
    		 double* work, int* info);

extern void F77_NAME(dspcon)(const char* uplo, const int* n,
    		 const double* ap, const int* ipiv,
    		 const double* anorm, double* rcond,
    		 double* work, int* iwork, int* info);
extern void F77_NAME(dspev)(const char* jobz, const char* uplo, const int* n,
    		double* ap, double* w, double* z, const int* ldz,
    		double* work, int* info);
extern void F77_NAME(dspevd)(const char* jobz, const char* uplo,
    		 const int* n, double* ap, double* w,
    		 double* z, const int* ldz,
    		 double* work, const int* lwork,
    		 int* iwork, const int* liwork, int* info);
extern void F77_NAME(dspevx)(const char* jobz, const char* range,
    		 const char* uplo, const int* n, double* ap,
    		 const double* vl, const double* vu,
    		 const int* il, const int* iu,
    		 const double* abstol,
    		 int* m, double* w,
    		 double* z, const int* ldz,
    		 double* work, int* iwork,
    		 int* ifail, int* info);
extern void F77_NAME(dspgst)(const int* itype, const char* uplo,
    		 const int* n, double* ap, double* bp, int* info);
extern void F77_NAME(dspgv)(const int* itype, const char* jobz,
    		const char* uplo, const int* n,
    		double* ap, double* bp, double* w,
    		double* z, const int* ldz,
    		double* work, int* info);

extern void F77_NAME(dsprfs)(const char* uplo, const int* n,
    		 const int* nrhs, const double* ap,
    		 const double* afp, const int* ipiv,
    		 const double* b, const int* ldb,
    		 double* x, const int* ldx,
    		 double* ferr, double* berr,
    		 double* work, int* iwork, int* info);

extern void F77_NAME(dspsv)(const char* uplo, const int* n,
    		const int* nrhs, double* ap, int* ipiv,
    		double* b, const int* ldb, int* info);

extern void F77_NAME(dspsvx)(const int* fact, const char* uplo,
    		 const int* n, const int* nrhs,
    		 const double* ap, double* afp, int* ipiv,
    		 const double* b, const int* ldb,
    		 double* x, const int* ldx,
    		 double* rcond, double* ferr, double* berr,
    		 double* work, int* iwork, int* info);

extern void F77_NAME(dsptrd)(const char* uplo, const int* n,
    		 double* ap, double* d, double* e,
    		 double* tau, int* info);

extern void F77_NAME(dsptrf)(const char* uplo, const int* n,
    		 double* ap, int* ipiv, int* info);

extern void F77_NAME(dsptri)(const char* uplo, const int* n,
    		 double* ap, const int* ipiv,
    		 double* work, int* info);

extern void F77_NAME(dsptrs)(const char* uplo, const int* n,
    		 const int* nrhs, const double* ap,
    		 const int* ipiv, double* b, const int* ldb, int* info);


extern void F77_NAME(dstebz)(const char* range, const char* order, const int* n,
    		 const double* vl, const double* vu,
    		 const int* il, const int* iu,
    		 const double *abstol,
    		 const double* d, const double* e,
    		 int* m, int* nsplit, double* w,
    		 int* iblock, int* isplit,
    		 double* work, int* iwork,
    		 int* info);
extern void F77_NAME(dstedc)(const char* compz, const int* n,
    		 double* d, double* e,
    		 double* z, const int* ldz,
    		 double* work, const int* lwork,
    		 int* iwork, const int* liwork, int* info);
extern void F77_NAME(dstein)(const int* n, const double* d, const double* e,
    		 const int* m, const double* w,
    		 const int* iblock, const int* isplit,
    		 double* z, const int* ldz,
    		 double* work, int* iwork,
    		 int* ifail, int* info);
extern void F77_NAME(dsteqr)(const char* compz, const int* n, double* d, double* e,
		 double* z, const int* ldz, double* work, int* info);
extern void F77_NAME(dsterf)(const int* n, double* d, double* e, int* info);
extern void F77_NAME(dstev)(const char* jobz, const int* n,
    		double* d, double* e,
    		double* z, const int* ldz,
    		double* work, int* info);
extern void F77_NAME(dstevd)(const char* jobz, const int* n,
    		 double* d, double* e,
    		 double* z, const int* ldz,
    		 double* work, const int* lwork,
    		 int* iwork, const int* liwork, int* info);
extern void F77_NAME(dstevx)(const char* jobz, const char* range,
    		 const int* n, double* d, double* e,
    		 const double* vl, const double* vu,
    		 const int* il, const int* iu,
    		 const double* abstol,
    		 int* m, double* w,
    		 double* z, const int* ldz,
    		 double* work, int* iwork,
    		 int* ifail, int* info);

extern void F77_NAME(dsycon)(const char* uplo, const int* n,
    		 const double* a, const int* lda,
    		 const int* ipiv,
    		 const double* anorm, double* rcond,
    		 double* work, int* iwork, int* info);
extern void F77_NAME(dsyev)(const char* jobz, const char* uplo,
    		const int* n, double* a, const int* lda,
    		double* w, double* work, const int* lwork, int* info);
extern void F77_NAME(dsyevd)(const char* jobz, const char* uplo,
    		 const int* n, double* a, const int* lda,
    		 double* w, double* work, const int* lwork,
    		 int* iwork, const int* liwork, int* info);
extern void F77_NAME(dsyevx)(const char* jobz, const char* range,
    		 const char* uplo, const int* n,
    		 double* a, const int* lda,
    		 const double* vl, const double* vu,
    		 const int* il, const int* iu,
    		 const double* abstol,
    		 int* m, double* w,
    		 double* z, const int* ldz,
    		 double* work, const int* lwork, int* iwork,
		 int* ifail, int* info);
extern void F77_NAME(dsyevr)(const char *jobz, const char *range, const char *uplo,
		 const int *n, double *a, const int *lda,
		 const double *vl, const double *vu,
		 const int *il, const int *iu,
		 const double *abstol, int *m, double *w, 
		 double *z, const int *ldz, int *isuppz, 
		 double *work, const int *lwork,
		 int *iwork, const int *liwork,
		 int *info);
extern void F77_NAME(dsygs2)(const int* itype, const char* uplo,
    		 const int* n, double* a, const int* lda,
    		 const double* b, const int* ldb, int* info);
extern void F77_NAME(dsygst)(const int* itype, const char* uplo,
    		 const int* n, double* a, const int* lda,
    		 const double* b, const int* ldb, int* info);
extern void F77_NAME(dsygv)(const int* itype, const char* jobz,
    		const char* uplo, const int* n,
    		double* a, const int* lda,
    		double* b, const int* ldb,
    		double* w, double* work, const int* lwork,
    		int* info);
extern void F77_NAME(dsyrfs)(const char* uplo, const int* n,
    		 const int* nrhs,
    		 const double* a, const int* lda,
    		 const double* af, const int* ldaf,
    		 const int* ipiv,
    		 const double* b, const int* ldb,
    		 double* x, const int* ldx,
    		 double* ferr, double* berr,
    		 double* work, int* iwork, int* info);

extern void F77_NAME(dsysv)(const char* uplo, const int* n,
    		const int* nrhs,
    		double* a, const int* lda, int* ipiv,
    		double* b, const int* ldb,
    		double* work, const int* lwork, int* info);

extern void F77_NAME(dsysvx)(const int* fact, const char* uplo,
    		 const int* n, const int* nrhs,
    		 const double* a, const int* lda,
    		 double* af, const int* ldaf, int* ipiv,
    		 const double* b, const int* ldb,
    		 double* x, const int* ldx, double* rcond,
    		 double* ferr, double* berr,
    		 double* work, const int* lwork,
    		 int* iwork, int* info);

extern void F77_NAME(dsytd2)(const char* uplo, const int* n,
    		 double* a, const int* lda,
    		 double* d, double* e, double* tau,
    		 int* info);

extern void F77_NAME(dsytf2)(const char* uplo, const int* n,
    		 double* a, const int* lda,
    		 int* ipiv, int* info);

extern void F77_NAME(dsytrd)(const char* uplo, const int* n,
    		 double* a, const int* lda,
    		 double* d, double* e, double* tau,
    		 double* work, const int* lwork, int* info);

extern void F77_NAME(dsytrf)(const char* uplo, const int* n,
    		 double* a, const int* lda, int* ipiv,
    		 double* work, const int* lwork, int* info);

extern void F77_NAME(dsytri)(const char* uplo, const int* n,
    		 double* a, const int* lda, const int* ipiv,
    		 double* work, int* info); 

extern void F77_NAME(dsytrs)(const char* uplo, const int* n,
    		 const int* nrhs,
    		 const double* a, const int* lda,
    		 const int* ipiv,
    		 double* b, const int* ldb, int* info);

extern void F77_NAME(dtbcon)(const char* norm, const char* uplo,
    		 const char* diag, const int* n, const int* kd,
    		 const double* ab, const int* ldab,
    		 double* rcond, double* work,
    		 int* iwork, int* info); 
extern void F77_NAME(dtbrfs)(const char* uplo, const char* trans,
    		 const char* diag, const int* n, const int* kd,
    		 const int* nrhs,
    		 const double* ab, const int* ldab,
    		 const double* b, const int* ldb,
    		 double* x, const int* ldx,
    		 double* ferr, double* berr,
    		 double* work, int* iwork, int* info);  
extern void F77_NAME(dtbtrs)(const char* uplo, const char* trans,
    		 const char* diag, const int* n,
    		 const int* kd, const int* nrhs,
    		 const double* ab, const int* ldab,
    		 double* b, const int* ldb, int* info); 

extern void F77_NAME(dtgevc)(const char* side, const char* howmny,
    		 const int* select, const int* n,
    		 const double* a, const int* lda,
    		 const double* b, const int* ldb,
    		 double* vl, const int* ldvl,
    		 double* vr, const int* ldvr,
    		 const int* mm, int* m, double* work, int* info);

extern void F77_NAME(dtgsja)(const char* jobu, const char* jobv, const char* jobq,
    		 const int* m, const int* p, const int* n,
    		 const int* k, const int* l,
    		 double* a, const int* lda,
    		 double* b, const int* ldb,
    		 const double* tola, const double* tolb,
    		 double* alpha, double* beta,
    		 double* u, const int* ldu,
    		 double* v, const int* ldv,
    		 double* q, const int* ldq,
    		 double* work, int* ncycle, int* info);
extern void F77_NAME(dtpcon)(const char* norm, const char* uplo,
    		 const char* diag, const int* n,
    		 const double* ap, double* rcond,
    		 double* work, int* iwork, int* info);

extern void F77_NAME(dtprfs)(const char* uplo, const char* trans,
    		 const char* diag, const int* n,
    		 const int* nrhs, const double* ap,
    		 const double* b, const int* ldb,
    		 double* x, const int* ldx,
    		 double* ferr, double* berr,
    		 double* work, int* iwork, int* info);

extern void F77_NAME(dtptri)(const char* uplo, const char* diag,
    		 const int* n, double* ap, int* info);

extern void F77_NAME(dtptrs)(const char* uplo, const char* trans,
    		 const char* diag, const int* n,
    		 const int* nrhs, const double* ap,
    		 double* b, const int* ldb, int* info); 

extern void F77_NAME(dtrcon)(const char* norm, const char* uplo,
    		 const char* diag, const int* n,
    		 const double* a, const int* lda,
    		 double* rcond, double* work,
    		 int* iwork, int* info);

extern void F77_NAME(dtrevc)(const char* side, const char* howmny,
    		 const int* select, const int* n,
    		 const double* t, const int* ldt,
    		 double* vl, const int* ldvl,
    		 double* vr, const int* ldvr,
    		 const int* mm, int* m, double* work, int* info);

extern void F77_NAME(dtrexc)(const char* compq, const int* n,
    		 double* t, const int* ldt,
    		 double* q, const int* ldq,
    		 int* ifst, int* ILST,
    		 double* work, int* info);

extern void F77_NAME(dtrrfs)(const char* uplo, const char* trans,
    		 const char* diag, const int* n, const int* nrhs,
    		 const double* a, const int* lda,
    		 const double* b, const int* ldb,
    		 double* x, const int* ldx,
    		 double* ferr, double* berr,
    		 double* work, int* iwork, int* info); 

extern void F77_NAME(dtrsen)(const char* job, const char* compq,
    		 const int* select, const int* n,
    		 double* t, const int* ldt,
    		 double* q, const int* ldq,
    		 double* wr, double* wi,
    		 int* m, double* s, double* sep,
    		 double* work, const int* lwork,
    		 int* iwork, const int* liwork, int* info);

extern void F77_NAME(dtrsna)(const char* job, const char* howmny,
    		 const int* select, const int* n,
    		 const double* t, const int* ldt,
    		 const double* vl, const int* ldvl,
    		 const double* vr, const int* ldvr,
    		 double* s, double* sep, const int* mm,
    		 int* m, double* work, const int* lwork,
    		 int* iwork, int* info);

extern void F77_NAME(dtrsyl)(const char* trana, const char* tranb,
    		 const int* isgn, const int* m, const int* n,
    		 const double* a, const int* lda,
    		 const double* b, const int* ldb,
    		 double* c, const int* ldc,
    		 double* scale, int* info);  

extern void F77_NAME(dtrti2)(const char* uplo, const char* diag,
    		 const int* n, double* a, const int* lda,
    		 int* info); 

extern void F77_NAME(dtrtri)(const char* uplo, const char* diag,
    		 const int* n, double* a, const int* lda,
    		 int* info); 

extern void F77_NAME(dtrtrs)(const char* uplo, const char* trans,
    		 const char* diag, const int* n, const int* nrhs,
    		 const double* a, const int* lda,
    		 double* b, const int* ldb, int* info); 

extern void F77_NAME(dtzrqf)(const int* m, const int* n,
    		 double* a, const int* lda,
    		 double* tau, int* info);



extern void F77_NAME(dhgeqz)(const char* job, const char* compq, const char* compz,
		 const int* n, const int *ILO, const int* IHI,
		 double* a, const int* lda,
		 double* b, const int* ldb,
		 double* alphar, double* alphai, const double* beta,
		 double* q, const int* ldq,
		 double* z, const int* ldz,
		 double* work, const int* lwork, int* info);
extern void F77_NAME(dhsein)(const char* side, const char* eigsrc,
		 const char* initv, int* select,
		 const int* n, double* h, const int* ldh,
		 double* wr, double* wi,
		 double* vl, const int* ldvl,
		 double* vr, const int* ldvr,
		 const int* mm, int* m, double* work,
		 int* ifaill, int* ifailr, int* info);
extern void F77_NAME(dhseqr)(const char* job, const char* compz, const int* n,
		 const int* ilo, const int* ihi,
		 double* h, const int* ldh,
		 double* wr, double* wi,
		 double* z, const int* ldz,
		 double* work, const int* lwork, int* info);
extern void F77_NAME(dlabad)(double* small, double* large);
extern void F77_NAME(dlabrd)(const int* m, const int* n, const int* nb,
		 double* a, const int* lda, double* d, double* e,
		 double* tauq, double* taup,
		 double* x, const int* ldx, double* y, const int* ldy);
extern void F77_NAME(dlacon)(const int* n, double* v, double* x,
		 int* isgn, double* est, int* kase);
extern void F77_NAME(dlacpy)(const char* uplo, const int* m, const int* n,
		 const double* a, const int* lda,
		 double* b, const int* ldb);
extern void F77_NAME(dladiv)(const double* a, const double* b,
		 const double* c, const double* d,
		 double* p, double* q);
extern void F77_NAME(dlae2)(const double* a, const double* b, const double* c,
		double* rt1, double* rt2);
extern void F77_NAME(dlaebz)(const int* ijob, const int* nitmax, const int* n,
		 const int* mmax, const int* minp, const int* nbmin,
		 const double* abstol, const double* reltol,
		 const double* pivmin, double* d, double* e,
		 double* e2, int* nval, double* ab, double* c,
		 int* mout, int* nab, double* work, int* iwork,
		 int* info);
extern void F77_NAME(dlaed0)(const int* icompq, const int* qsiz, const int* n,
		 double* d, double* e, double* q, const int* ldq,
		 double* qstore, const int* ldqs,
		 double* work, int* iwork, int* info);
extern void F77_NAME(dlaed1)(const int* n, double* d, double* q, const int* ldq,
		 int* indxq, const double* rho, const int* cutpnt,
		 double* work, int* iwork, int* info);
extern void F77_NAME(dlaed2)(const int* k, const int* n, double* d,
		 double* q, const int* ldq, int* indxq,
		 double* rho, const int* cutpnt, double* z,
		 double* dlamda, double* q2, const int *ldq2,
		 int* indxc, int* w, int* indxp, int* indx,
		 int* coltyp, int* info);
extern void F77_NAME(dlaed3)(const int* k, const int* kstart,
		 const int *kstop, const int* n,
		 double* d, double* q, const int* ldq,
		 const double* rho, const int* cutpnt,
		 double* dlamda, int* q2, const int* ldq2,
		 int* indxc, int* ctot, double* w,
		 double* s, const int* lds, int* info);
extern void F77_NAME(dlaed4)(const int* n, const int* i, const double* d,
		 const double* z, const double* delta,
		 const double* rho, double* dlam, int* info);
extern void F77_NAME(dlaed5)(const int* i, const double* d, const double* z,
		 double* delta, const double* rho, double* dlam);
extern void F77_NAME(dlaed6)(const int* kniter, const int* orgati,
		 const double* rho, const double* d,
		 const double* z, const double* finit,
		 double* tau, int* info);
extern void F77_NAME(dlaed7)(const int* icompq, const int* n,
		 const int* qsiz, const int* tlvls,
		 const int* curlvl, const int* curpbm,
		 double* d, double* q, const int* ldq,
		 int* indxq, const double* rho, const int* cutpnt,
		 double* qstore, double* qptr, const int* prmptr,
		 const int* perm, const int* givptr,
		 const int* givcol, const double* givnum,
		 double* work, int* iwork, int* info);
extern void F77_NAME(dlaed8)(const int* icompq, const int* k,
		 const int* n, const int* qsiz,
		 double* d, double* q, const int* ldq,
		 const int* indxq, double* rho,
		 const int* cutpnt, const double* z,
		 double* dlamda, double* q2, const int* ldq2,
		 double* w, int* perm, int* givptr,
		 int* givcol, double* givnum, int* indxp,
		 int* indx, int* info);
extern void F77_NAME(dlaed9)(const int* k, const int* kstart, const int* kstop,
		 const int* n, double* d, double* q, const int* ldq,
		 const double* rho, const double* dlamda,
		 const double* w, double* s, const int* lds, int* info);
extern void F77_NAME(dlaeda)(const int* n, const int* tlvls, const int* curlvl,
		 const int* curpbm, const int* prmptr, const int* perm,
		 const int* givptr, const int* givcol,
		 const double* givnum, const double* q,
		 const int* qptr, double* z, double* ztemp, int* info);
extern void F77_NAME(dlaein)(const int* rightv, const int* noinit, const int* n,
		 const double* h, const int* ldh,
		 const double* wr, const double* wi,
		 double* vr, double* vi,
		 double* b, const int* ldb, double* work,
		 const double* eps3, const double* smlnum,
		 const double* bignum, int* info);
extern void F77_NAME(dlaev2)(const double* a, const double* b, const double* c, 
		 double* rt1, double* rt2, double* cs1, double *sn1);
extern void F77_NAME(dlaexc)(const int* wantq, const int* n, double* t, const int* ldt,
    		  double* q, const int* ldq, const int* j1,
		 const int* n1, const int* n2, double* work, int* info);
extern void F77_NAME(dlag2)(const double* a, const int* lda, const double* b,
		const int* ldb, const double* safmin,
		double* scale1, double* scale2,
		double* wr1, double* wr2, double* wi);
extern void F77_NAME(dlags2)(const int* upper,
		 const double* a1, const double* a2, const double* a3,
		 const double* b1, const double* b2, const double* b3,
		 double* csu, double* snu,
		 double* csv, double* snv, double *csq, double *snq);
extern void F77_NAME(dlagtf)(const int* n, double* a, const double* lambda,
		 double* b, double* c, const double *tol,
		 double* d, int* in, int* info);
extern void F77_NAME(dlagtm)(const char* trans, const int* n, const int* nrhs,
		 const double* alpha, const double* dl,
		 const double* d, const double* du,
		 const double* x, const int* ldx, const double* beta,
		 double* b, const int* ldb);
extern void F77_NAME(dlagts)(const int* job, const int* n,
		 const double* a, const double* b,
		 const double* c, const double* d,
		 const int* in, double* y, double* tol, int* info);
extern void F77_NAME(dlahqr)(const int* wantt, const int* wantz, const int* n,
		 const int* ilo, const int* ihi,
		 double* H, const int* ldh, double* wr, double* wi,
		 const int* iloz, const int* ihiz,
		 double* z, const int* ldz, int* info);
extern void F77_NAME(dlahrd)(const int* n, const int* k, const int* nb,
		 double* a, const int* lda,
		 double* tau, double* t, const int* ldt,
		 double* y, const int* ldy);
extern void F77_NAME(dlaic1)(const int* job, const int* j, const double* x,
		 const double* sest, const double* w,
		 const double* gamma, double* sestpr,
		 double* s, double* c);
extern void F77_NAME(dlaln2)(const int* ltrans, const int* na, const int* nw,
		 const double* smin, const double* ca,
		 const double* a, const int* lda,
		 const double* d1, const double* d2,
		 const double* b, const int* ldb,
		 const double* wr, const double* wi,
		 double* x, const int* ldx, double* scale,
		 double* xnorm, int* info);
extern double F77_NAME(dlamch)(const char* cmach);
extern void F77_NAME(dlamrg)(const int* n1, const int* n2, const double* a,
		 const int* dtrd1, const int* dtrd2, int* index);
extern double F77_NAME(dlangb)(const char* norm, const int* n,
		 const int* kl, const int* ku, const double* ab,
		 const int* ldab, double* work);
extern double F77_NAME(dlange)(const char* norm, const int* m, const int* n,
		 const double* a, const int* lda, double* work);
extern double F77_NAME(dlangt)(const char* norm, const int* n,
		 const double* dl, const double* d,
		 const double* du);
extern double F77_NAME(dlanhs)(const char* norm, const int* n,
		 const double* a, const int* lda, double* work);
extern double F77_NAME(dlansb)(const char* norm, const char* uplo,
		 const int* n, const int* k,
		 const double* ab, const int* ldab, double* work);
extern double F77_NAME(dlansp)(const char* norm, const char* uplo,
		 const int* n, const double* ap, double* work);
extern double F77_NAME(dlanst)(const char* norm, const int* n,
		 const double* d, const double* e);
extern double F77_NAME(dlansy)(const char* norm, const char* uplo, const int* n,
		 const double* a, const int* lda, double* work);
extern double F77_NAME(dlantb)(const char* norm, const char* uplo,
		 const char* diag, const int* n, const int* k,
		 const double* ab, const int* ldab, double* work);
extern double F77_NAME(dlantp)(const char* norm, const char* uplo, const char* diag,
		 const int* n, const double* ap, double* work);
extern double F77_NAME(dlantr)(const char* norm, const char* uplo,
		 const char* diag, const int* m, const int* n,
		 const double* a, const int* lda, double* work);
extern void F77_NAME(dlanv2)(double* a, double* b, double* c, double* d,
		 double* rt1r, double* rt1i, double* rt2r, double* rt2i,
		 double* cs, double *sn);
extern void F77_NAME(dlapll)(const int* n, double* x, const int* incx,
		 double* y, const int* incy, double* ssmin);
extern void F77_NAME(dlapmt)(const int* forwrd, const int* m, const int* n,
		 double* x, const int* ldx, const int* k);
extern double F77_NAME(dlapy2)(const double* x, const double* y);
extern double F77_NAME(dlapy3)(const double* x, const double* y, const double* z);
extern void F77_NAME(dlaqgb)(const int* m, const int* n,
		 const int* kl, const int* ku,
		 double* ab, const int* ldab,
		 double* r, double* c,
		 double* rowcnd, double* colcnd,
		 const double* amax, char* equed);
extern void F77_NAME(dlaqge)(const int* m, const int* n,
		 double* a, const int* lda,
		 double* r, double* c,
		 double* rowcnd, double* colcnd,
		 const double* amax, char* equed);
extern void F77_NAME(dlaqsb)(const char* uplo, const int* n, const int* kd,
		 double* ab, const int* ldab, const double* s,
		 const double* scond, const double* amax, char* equed);
extern void F77_NAME(dlaqsp)(const char* uplo, const int* n,
		 double* ap, const double* s, const double* scond,
		 const double* amax, int* equed);
extern void F77_NAME(dlaqsy)(const char* uplo, const int* n,
		 double* a, const int* lda,
		 const double* s, const double* scond, 
		 const double* amax, int* equed);
extern void F77_NAME(dlaqtr)(const int* ltran, const int* lreal, const int* n,
		 const double* t, const int* ldt,
		 const double* b, const double* w,
		 double* scale, double* x, double* work, int* info);
extern void F77_NAME(dlar2v)(const int* n, double* x, double* y,
		 double* z, const int* incx,
		 const double* c, const double* s,
		 const int* incc);
extern void F77_NAME(dlarf)(const char* side, const int* m, const int* n,
		const double* v, const int* incv, const double* tau,
		double* c, const int* ldc, double* work);
extern void F77_NAME(dlarfb)(const char* side, const char* trans,
		 const char* direct, const char* storev,
		 const int* m, const int* n, const int* k,
		 const double* v, const int* ldv,
		 const double* t, const int* ldt,
		 double* c, const int* ldc,
		 double* work, const int* lwork);
extern void F77_NAME(dlarfg)(const int* n, const double* alpha,
		 double* x, const int* incx, double* tau);
extern void F77_NAME(dlarft)(const char* direct, const char* storev,
		 const int* n, const int* k, double* v, const int* ldv,
		 const double* tau, double* t, const int* ldt);
extern void F77_NAME(dlarfx)(const char* side, const int* m, const int* n,
		 const double* v, const double* tau,
		 double* c, const int* ldc, double* work);
extern void F77_NAME(dlargv)(const int* n, double* x, const int* incx,
		 double* y, const int* incy, double* c, const int* incc);
extern void F77_NAME(dlarnv)(const int* idist, int* iseed, const int* n, double* x);
extern void F77_NAME(dlartg)(const double* f, const double* g, double* cs,
		 double* sn, double *r);
extern void F77_NAME(dlartv)(const int* n, double* x, const int* incx,
		 double* y, const int* incy,
		 const double* c, const double* s,
		 const int* incc);
extern void F77_NAME(dlaruv)(int* iseed, const int* n, double* x);

extern void F77_NAME(dlas2)(const double* f, const double* g, const double* h,
    		 double* ssmin, double* ssmax);

extern void F77_NAME(dlascl)(const char* type,
		 const int* kl,const int* ku,
		 double* cfrom, double* cto,
		 const int* m, const int* n,
		 double* a, const int* lda, int* info);
    
extern void F77_NAME(dlaset)(const char* uplo, const int* m, const int* n,
		 const double* alpha, const double* beta,
		 double* a, const int* lda);
extern void F77_NAME(dlasq1)(const int* n, double* d, double* e,
		 double* work, int* info);
extern void F77_NAME(dlasq2)(const int* m, double* q, double* e,
		 double* qq, double* ee, const double* eps,
		 const double* tol2, const double* small2,
		 double* sup, int* kend, int* info);
extern void F77_NAME(dlasq3)(int* n, double* q, double* e, double* qq,
		 double* ee, double* sup, double *sigma,
		 int* kend, int* off, int* iphase,
		 const int* iconv, const double* eps,
		 const double* tol2, const double* small2);
extern void F77_NAME(dlasq4)(const int* n, const double* q, const double* e,
		 double* tau, double* sup);
extern void F77_NAME(dlasr)(const char* side, const char* pivot,
		const char* direct, const int* m, const int* n,
		const double* c, const double* s,
		double* a, const int* lda);
extern void F77_NAME(dlasrt)(const char* id, const int* n, double* d, int* info);
extern void F77_NAME(dlassq)(const int* n, const double* x, const int* incx,
		 double* scale, double* sumsq);
extern void F77_NAME(dlasv2)(const double* f, const double* g, const double* h,
		 double* ssmin, double* ssmax, double* snr, double* csr,
		 double* snl, double* csl);
extern void F77_NAME(dlaswp)(const int* n, double* a, const int* lda,
		 const int* k1, const int* k2,
		 const int* ipiv, const int* incx);
extern void F77_NAME(dlasy2)(const int* ltranl, const int* ltranr,
		 const int* isgn, const int* n1, const int* n2,
		 const double* tl, const int* ldtl,
		 const double* tr, const int* ldtr,
		 const double* b, const int* ldb,
		 double* scale, double* x, const int* ldx,
		 double* xnorm, int* info);
extern void F77_NAME(dlasyf)(const char* uplo, const int* n,
		 const int* nb, const int* kb,
		 double* a, const int* lda, int* ipiv,
		 double* w, const int* ldw, int* info);
extern void F77_NAME(dlatbs)(const char* uplo, const char* trans,
		 const char* diag, const char* normin,
		 const int* n, const int* kd,
		 const double* ab, const int* ldab,
		 double* x, double* scale, double* cnorm, int* info);
extern void F77_NAME(dlatps)(const char* uplo, const char* trans,
		 const char* diag, const char* normin,
		 const int* n, const double* ap,
		 double* x, double* scale, double* cnorm, int* info);
extern void F77_NAME(dlatrd)(const char* uplo, const int* n, const int* nb,
		 double* a, const int* lda, double* e, double* tau,
		 double* w, const int* ldw);
extern void F77_NAME(dlatrs)(const char* uplo, const char* trans,
		 const char* diag, const char* normin,
		 const int* n, const double* a, const int* lda,
		 double* x, double* scale, double* cnorm, int* info);
extern void F77_NAME(dlatzm)(const char* side, const int* m, const int* n,
		 const double* v, const int* incv,
		 const double* tau, double* c1, double* c2,
		 const int* ldc, double* work);
extern void F77_NAME(dlauu2)(const char* uplo, const int* n,
		 double* a, const int* lda, int* info);
extern void F77_NAME(dlauum)(const char* uplo, const int* n,
		 double* a, const int* lda, int* info);


#ifdef  __cplusplus
}
#endif

#endif /* LAPACK_H */
