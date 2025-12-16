/**************************************************************************************************
*                                                                                                 *
* This file is part of BLASFEO.                                                                   *
*                                                                                                 *
* BLASFEO -- BLAS for embedded optimization.                                                      *
* Copyright (C) 2019 by Gianluca Frison.                                                          *
* Developed at IMTEK (University of Freiburg) under the supervision of Moritz Diehl.              *
* All rights reserved.                                                                            *
*                                                                                                 *
* The 2-Clause BSD License                                                                        *
*                                                                                                 *
* Redistribution and use in source and binary forms, with or without                              *
* modification, are permitted provided that the following conditions are met:                     *
*                                                                                                 *
* 1. Redistributions of source code must retain the above copyright notice, this                  *
*    list of conditions and the following disclaimer.                                             *
* 2. Redistributions in binary form must reproduce the above copyright notice,                    *
*    this list of conditions and the following disclaimer in the documentation                    *
*    and/or other materials provided with the distribution.                                       *
*                                                                                                 *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND                 *
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED                   *
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE                          *
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR                 *
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES                  *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;                    *
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND                     *
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT                      *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS                   *
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                    *
*                                                                                                 *
* Author: Gianluca Frison, gianluca.frison (at) imtek.uni-freiburg.de                             *
*                                                                                                 *
**************************************************************************************************/



#if defined(LA_EXTERNAL_BLAS_WRAPPER)



// dpotrf
void POTRF_L(int m, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	REAL d1 = 1.0;
	REAL *pC = sC->pA+ci+cj*sC->m;
	REAL *pD = sD->pA+di+dj*sD->m;
	int i1 = 1;
	int info;
	int tmp;
	int ldc = sC->m;
	int ldd = sD->m;
	if(!(pC==pD))
		{
		for(jj=0; jj<m; jj++)
			{
			tmp = m-jj;
			COPY(&tmp, pC+jj*ldc+jj, &i1, pD+jj*ldd+jj, &i1);
			}
		}
	POTRF(&cl, &m, pD, &ldd, &info);
	if(info!=0)
		{
		if(info>0)
			{
			printf("\nxpotrf: leading minor of order %d is not positive definite, the factorization could not be completed.\n", info);
			exit(1);
			}
		else
			{
			printf("\nxpotrf: the %d-th argument had an illegal value\n", -info);
			exit(1);
			}
		}
	return;
	}



// dpotrf
void POTRF_L_MN(int m, int n, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	REAL d1 = 1.0;
	REAL *pC = sC->pA+ci+cj*sC->m;
	REAL *pD = sD->pA+di+dj*sD->m;
	int i1 = 1;
	int mmn = m-n;
	int info;
	int tmp;
	int ldc = sC->m;
	int ldd = sD->m;
	if(!(pC==pD))
		{
		for(jj=0; jj<n; jj++)
			{
			tmp = m-jj;
			COPY(&tmp, pC+jj*ldc+jj, &i1, pD+jj*ldd+jj, &i1);
			}
		}
	POTRF(&cl, &n, pD, &ldd, &info);
	if(info!=0)
		{
		if(info>0)
			{
			printf("\nxpotrf: leading minor of order %d is not positive definite, the factorization could not be completed.\n", info);
			exit(1);
			}
		else
			{
			printf("\nxpotrf: the %d-th argument had an illegal value\n", -info);
			exit(1);
			}
		}
	TRSM(&cr, &cl, &ct, &cn, &mmn, &n, &d1, pD, &ldd, pD+n, &ldd);
	return;
	}



// dpotrf
void POTRF_U(int m, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	REAL d1 = 1.0;
	REAL *pC = sC->pA+ci+cj*sC->m;
	REAL *pD = sD->pA+di+dj*sD->m;
	int i1 = 1;
	int info;
	int tmp;
	int ldc = sC->m;
	int ldd = sD->m;
	if(!(pC==pD))
		{
		for(jj=0; jj<m; jj++)
			{
			tmp = jj+1;
			COPY(&tmp, pC+jj*ldc, &i1, pD+jj*ldd, &i1);
			}
		}
	POTRF(&cu, &m, pD, &ldd, &info);
	if(info!=0)
		{
		if(info>0)
			{
			printf("\nxpotrf: leading minor of order %d is not positive definite, the factorization could not be completed.\n", info);
			exit(1);
			}
		else
			{
			printf("\nxpotrf: the %d-th argument had an illegal value\n", -info);
			exit(1);
			}
		}
	return;
	}



// dsyrk dpotrf
void SYRK_POTRF_LN(int m, int k, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	REAL d1 = 1.0;
	REAL *pA = sA->pA + ai + aj*sA->m;
	REAL *pB = sB->pA + bi + bj*sB->m;
	REAL *pC = sC->pA + ci + cj*sC->m;
	REAL *pD = sD->pA + di + dj*sD->m;
	int i1 = 1;
	int info;
	int lda = sA->m;
	int ldb = sB->m;
	int ldc = sC->m;
	int ldd = sD->m;
	if(!(pC==pD))
		{
		for(jj=0; jj<m; jj++)
			COPY(&m, pC+jj*ldc, &i1, pD+jj*ldd, &i1);
		}
	if(pA==pB)
		{
		SYRK(&cl, &cn, &m, &k, &d1, pA, &lda, &d1, pD, &ldd);
		POTRF(&cl, &m, pD, &ldd, &info);
		}
	else
		{
		GEMM(&cn, &ct, &m, &m, &k, &d1, pA, &lda, pB, &ldb, &d1, pD, &ldd);
		POTRF(&cl, &m, pD, &ldd, &info);
		}
	if(info!=0)
		{
		if(info>0)
			{
			printf("\nxpotrf: leading minor of order %d is not positive definite, the factorization could not be completed.\n", info);
			exit(1);
			}
		else
			{
			printf("\nxpotrf: the %d-th argument had an illegal value\n", -info);
			exit(1);
			}
		}
	return;
	}



// dsyrk dpotrf
void SYRK_POTRF_LN_MN(int m, int n, int k, struct XMAT *sA, int ai, int aj, struct XMAT *sB, int bi, int bj, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	char cl = 'l';
	char cn = 'n';
	char cr = 'r';
	char ct = 't';
	char cu = 'u';
	REAL d1 = 1.0;
	REAL *pA = sA->pA + ai + aj*sA->m;
	REAL *pB = sB->pA + bi + bj*sB->m;
	REAL *pC = sC->pA + ci + cj*sC->m;
	REAL *pD = sD->pA + di + dj*sD->m;
	int i1 = 1;
	int mmn = m-n;
	int info;
	int lda = sA->m;
	int ldb = sB->m;
	int ldc = sC->m;
	int ldd = sD->m;
	if(!(pC==pD))
		{
		for(jj=0; jj<n; jj++)
			COPY(&m, pC+jj*ldc, &i1, pD+jj*ldd, &i1);
		}
	if(pA==pB)
		{
		SYRK(&cl, &cn, &n, &k, &d1, pA, &lda, &d1, pD, &ldd);
		GEMM(&cn, &ct, &mmn, &n, &k, &d1, pA+n, &lda, pB, &ldb, &d1, pD+n, &ldd);
		POTRF(&cl, &n, pD, &ldd, &info);
		if(info!=0)
			{
			if(info>0)
				{
				printf("\nxpotrf: leading minor of order %d is not positive definite, the factorization could not be completed.\n", info);
				exit(1);
				}
			else
				{
				printf("\nxpotrf: the %d-th argument had an illegal value\n", -info);
				exit(1);
				}
			}
		TRSM(&cr, &cl, &ct, &cn, &mmn, &n, &d1, pD, &ldd, pD+n, &ldd);
		}
	else
		{
		GEMM(&cn, &ct, &m, &n, &k, &d1, pA, &lda, pB, &ldb, &d1, pD, &ldd);
		POTRF(&cl, &n, pD, &ldd, &info);
		if(info!=0)
			{
			if(info>0)
				{
				printf("\nxpotrf: leading minor of order %d is not positive definite, the factorization could not be completed.\n", info);
				exit(1);
				}
			else
				{
				printf("\nxpotrf: the %d-th argument had an illegal value\n", -info);
				exit(1);
				}
			}
		TRSM(&cr, &cl, &ct, &cn, &mmn, &n, &d1, pD, &ldd, pD+n, &ldd);
		}
	return;
	}



// dgetrf without pivoting
static void GETF2_NOPIVOT(int m, int n, REAL *A, int lda)
	{
	if(m<=0 | n<=0)
		return;
	int i, j;
	int jmax = m<n ? m : n;
	REAL dtmp;
	REAL dm1 = -1.0;
	int itmp0, itmp1;
	int i1 = 1;
	for(j=0; j<jmax; j++)
		{
		itmp0 = m-j-1;
		dtmp = 1.0/A[j+lda*j];
		SCAL(&itmp0, &dtmp, &A[(j+1)+lda*j], &i1);
		itmp1 = n-j-1;
		GER(&itmp0, &itmp1, &dm1, &A[(j+1)+lda*j], &i1, &A[j+lda*(j+1)], &lda, &A[(j+1)+lda*(j+1)], &lda);
		}
	return;
	}



// dgetrf without pivoting
void GETRF_NOPIVOT(int m, int n, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj)
	{
	// TODO with custom level 2 LAPACK + level 3 BLAS
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	REAL d1 = 1.0;
	REAL *pC = sC->pA+ci+cj*sC->m;
	REAL *pD = sD->pA+di+dj*sD->m;
	int i1 = 1;
	int ldc = sC->m;
	int ldd = sD->m;
	if(!(pC==pD))
		{
		for(jj=0; jj<n; jj++)
			COPY(&m, pC+jj*ldc, &i1, pD+jj*ldd, &i1);
		}
	GETF2_NOPIVOT(m, n, pD, ldd);
	return;
	}



// dgetrf pivoting
void GETRF_ROWPIVOT(int m, int n, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj, int *ipiv)
	{
	// TODO with custom level 2 LAPACK + level 3 BLAS
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	int tmp = m<n ? m : n;
	REAL d1 = 1.0;
	REAL *pC = sC->pA+ci+cj*sC->m;
	REAL *pD = sD->pA+di+dj*sD->m;
	int i1 = 1;
	int info;
	int ldc = sC->m;
	int ldd = sD->m;
	if(!(pC==pD))
		{
		for(jj=0; jj<n; jj++)
			COPY(&m, pC+jj*ldc, &i1, pD+jj*ldd, &i1);
		}
	GETRF(&m, &n, pD, &ldd, ipiv, &info);
	for(jj=0; jj<tmp; jj++)
		ipiv[jj] -= 1;
	return;
	}



int GEQRF_WORK_SIZE(int m, int n)
	{
	REAL dwork;
	REAL *pD, *dD;
	int lwork = -1;
	int info;
	int ldd = m;
	GEQRF_(&m, &n, pD, &ldd, dD, &dwork, &lwork, &info);
	int size = dwork;
	return size*sizeof(REAL);
	}



void GEQRF(int m, int n, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj, void *work)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	REAL *pC = sC->pA+ci+cj*sC->m;
	REAL *pD = sD->pA+di+dj*sD->m;
	REAL *dD = sD->dA+di;
	REAL *dwork = (REAL *) work;
	int i1 = 1;
	int info = -1;
	int ldc = sC->m;
	int ldd = sD->m;
	if(!(pC==pD))
		{
		for(jj=0; jj<n; jj++)
			COPY(&m, pC+jj*ldc, &i1, pD+jj*ldd, &i1);
		}
//	GEQR2(&m, &n, pD, &ldd, dD, dwork, &info);
	int lwork = -1;
	GEQRF_(&m, &n, pD, &ldd, dD, dwork, &lwork, &info);
	lwork = dwork[0];
	GEQRF_(&m, &n, pD, &ldd, dD, dwork, &lwork, &info);
	return;
	}



int GELQF_WORK_SIZE(int m, int n)
	{
	REAL dwork;
	REAL *pD, *dD;
	int lwork = -1;
	int info;
	int ldd = m;
	GELQF_(&m, &n, pD, &ldd, dD, &dwork, &lwork, &info);
	int size = dwork;
	return size*sizeof(REAL);
	}



void GELQF(int m, int n, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj, void *work)
	{
	if(m<=0 | n<=0)
		return;

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	REAL *pC = sC->pA+ci+cj*sC->m;
	REAL *pD = sD->pA+di+dj*sD->m;
	REAL *dD = sD->dA+di;
	REAL *dwork = (REAL *) work;
	int i1 = 1;
	int info = -1;
	int ldc = sC->m;
	int ldd = sD->m;
	if(!(pC==pD))
		{
		for(jj=0; jj<n; jj++)
			COPY(&m, pC+jj*ldc, &i1, pD+jj*ldd, &i1);
		}
//	GEQR2(&m, &n, pD, &ldd, dD, dwork, &info);
	int lwork = -1;
	GELQF_(&m, &n, pD, &ldd, dD, dwork, &lwork, &info);
	lwork = dwork[0];
	GELQF_(&m, &n, pD, &ldd, dD, dwork, &lwork, &info);
	return;
	}



// generate Q matrix
int ORGLQ_WORK_SIZE(int m, int n, int k)
	{
//	printf("\nblasfeo_orglq_worksize: feature not implemented yet\n");
//	exit(1);
	REAL dwork;
	REAL *pD, *dD;
	int lwork = -1;
	int info;
	int ldd = m;
	ORGLQ_(&m, &n, &k, pD, &ldd, dD, &dwork, &lwork, &info);
	int size = dwork;
	return size*sizeof(REAL);
	}



// generate Q matrix
void ORGLQ(int m, int n, int k, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj, void *work)
	{
	if(m<=0 | n<=0)
		return;

//	printf("\nblasfeo_orglq: feature not implemented yet\n");
//	exit(1);

	// invalidate stored inverse diagonal of result matrix
	sD->use_dA = 0;

	int jj;
	REAL *pC = sC->pA+ci+cj*sC->m;
	REAL *dC = sC->dA+ci;
	REAL *pD = sD->pA+di+dj*sD->m;
	REAL *dD = sD->dA+di;
	REAL *dwork = (REAL *) work;
	int i1 = 1;
	int info = -1;
	int ldc = sC->m;
	int ldd = sD->m;
	if(!(pC==pD))
		{
		for(jj=0; jj<n; jj++)
			COPY(&m, pC+jj*ldc, &i1, pD+jj*ldd, &i1);
		COPY(&k, dC, &i1, dD, &i1);
		}
//	GEQR2(&m, &n, pD, &ldd, dD, dwork, &info);
	int lwork = -1;
	ORGLQ_(&m, &n, &k, pD, &ldd, dD, dwork, &lwork, &info);
	lwork = dwork[0];
	ORGLQ_(&m, &n, &k, pD, &ldd, dD, dwork, &lwork, &info);
	return;
	}



// LQ factorization with positive diagonal elements
// XXX this is a hack that only returns the correct L matrix
// TODO fix also Q !!!!
void GELQF_PD(int m, int n, struct XMAT *sC, int ci, int cj, struct XMAT *sD, int di, int dj, void *work)
	{
//	if(m<=0 | n<=0)
//		return;
//	printf("\nblasfeo_gelqf_pd: feature not implemented yet\n");
//	exit(1);
	GELQF(m, n, sC, ci, cj, sD, di, dj, work);
	int ldd = sD->m;
	REAL *pD = sD->pA+di+dj*ldd;
	int ii, jj;
	int imax = m<=n ? m : n;
	for(ii=0; ii<imax; ii++)
		{
		if(pD[ii+ldd*ii]<0.0)
			{
			for(jj=ii; jj<m; jj++)
				{
				pD[jj+ldd*ii] = - pD[jj+ldd*ii];
				}
			}
		}
	return;
	}



// LQ factorization with positive diagonal elements, array of matrices
// [L, A] <= lq( [L. A] )
// L lower triangular, of size (m)x(m)
// A full of size (m)x(n1)
void GELQF_PD_LA(int m, int n1, struct XMAT *sD, int di, int dj, struct XMAT *sA, int ai, int aj, void *work)

	{
	if(m<=0)
		return;
	printf("\nblasfeo_gelqf_pd_la: feature not implemented yet\n");
	exit(1);
	}



// LQ factorization with positive diagonal elements, array of matrices
// [L, L, A] <= lq( [L. L, A] )
// L lower triangular, of size (m)x(m)
// A full of size (m)x(n1)
void GELQF_PD_LLA(int m, int n1, struct XMAT *sD, int di, int dj, struct XMAT *sL, int li, int lj, struct XMAT *sA, int ai, int aj, void *work)

	{
	if(m<=0)
		return;
	printf("\nblasfeo_gelqf_pd_lla: feature not implemented yet\n");
	exit(1);
	}


#else

#error : wrong LA choice

#endif
