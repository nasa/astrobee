/**************************************************************************************************
*                                                                                                 *
* This file is part of BLASFEO.                                                                   *
*                                                                                                 *
* BLASFEO -- BLAS For Embedded Optimization.                                                      *
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

#include <stdlib.h>
#include <stdio.h>

#include <blasfeo.h>


int main()
	{

	printf("\nExample of LU factorization and backsolve\n\n");

#if defined(LA_HIGH_PERFORMANCE)

	printf("\nLA provided by BLASFEO\n\n");

#elif defined(LA_REFERENCE)

	printf("\nLA provided by REFERENCE\n\n");

#elif defined(LA_EXTERNAL_BLAS_WRAPPER)

	printf("\nLA provided by EXTERNAL_BLAS_WRAPPER\n\n");

#else

	printf("\nLA provided by ???\n\n");
	exit(2);

#endif

	printf( "Testing processor\n" );

	char supportString[50];
	blasfeo_processor_library_string( supportString );
	printf( "Library requires processor features:%s\n", supportString );

	int features = 0;
	int procCheckSucceed = blasfeo_processor_cpu_features( &features );
	blasfeo_processor_feature_string( features, supportString );
	printf( "Processor supports features:%s\n", supportString );

	if( !procCheckSucceed )
	{
		printf("Current processor does not support the current compiled BLASFEO library.\n");
		printf("Please get a BLASFEO library compatible with this processor.\n");
		exit(3);
	}

	int ii;

	int n = 16;

	//
	// matrices in column-major format
	//

	double *A; d_zeros(&A, n, n);
	for(ii=0; ii<n*n; ii++) A[ii] = ii;
//	d_print_mat(n, n, A, n);

	// spd matrix
	double *B; d_zeros(&B, n, n);
	for(ii=0; ii<n; ii++) B[ii*(n+1)] = 1.0;
//	d_print_mat(n, n, B, n);

	// identity
	double *I; d_zeros(&I, n, n);
	for(ii=0; ii<n; ii++) I[ii*(n+1)] = 1.0;
//	d_print_mat(n, n, B, n);

	// result matrix
	double *D; d_zeros(&D, n, n);
//	d_print_mat(n, n, D, n);

	// permutation indeces
	int *ipiv; int_zeros(&ipiv, n, 1);

	//
	// matrices in matrix struct format
	//

	// work space enough for 6 matrix structs for size n times n
	int size_strmat = 6*blasfeo_memsize_dmat(n, n);
	void *memory_strmat; v_zeros_align(&memory_strmat, size_strmat);
	char *ptr_memory_strmat = (char *) memory_strmat;

	struct blasfeo_dmat sA;
//	blasfeo_allocate_dmat(n, n, &sA);
	blasfeo_create_dmat(n, n, &sA, ptr_memory_strmat);
	ptr_memory_strmat += sA.memsize;
	// convert from column major matrix to strmat
	blasfeo_pack_dmat(n, n, A, n, &sA, 0, 0);
	printf("\nA = \n");
	blasfeo_print_dmat(n, n, &sA, 0, 0);

	struct blasfeo_dmat sB;
//	blasfeo_allocate_dmat(n, n, &sB);
	blasfeo_create_dmat(n, n, &sB, ptr_memory_strmat);
	ptr_memory_strmat += sB.memsize;
	// convert from column major matrix to strmat
	blasfeo_pack_dmat(n, n, B, n, &sB, 0, 0);
	printf("\nB = \n");
	blasfeo_print_dmat(n, n, &sB, 0, 0);

	struct blasfeo_dmat sI;
//	blasfeo_allocate_dmat(n, n, &sI);
	blasfeo_create_dmat(n, n, &sI, ptr_memory_strmat);
	ptr_memory_strmat += sI.memsize;
	// convert from column major matrix to strmat

	struct blasfeo_dmat sD;
//	blasfeo_allocate_dmat(n, n, &sD);
	blasfeo_create_dmat(n, n, &sD, ptr_memory_strmat);
	ptr_memory_strmat += sD.memsize;

	struct blasfeo_dmat sLU;
//	blasfeo_allocate_dmat(n, n, &sD);
	blasfeo_create_dmat(n, n, &sLU, ptr_memory_strmat);
	ptr_memory_strmat += sLU.memsize;

	struct blasfeo_dmat sLUt;
//	blasfeo_allocate_dmat(n, n, &sD);
	blasfeo_create_dmat(n, n, &sLUt, ptr_memory_strmat);
	ptr_memory_strmat += sLUt.memsize;

	blasfeo_dgemm_nt(n, n, n, 1.0, &sA, 0, 0, &sA, 0, 0, 1.0, &sB, 0, 0, &sD, 0, 0);
	printf("\nB+A*A' = \n");
	blasfeo_print_dmat(n, n, &sD, 0, 0);

//	blasfeo_dgetrf_nopivot(n, n, &sD, 0, 0, &sD, 0, 0);
	blasfeo_dgetrf_rp(n, n, &sD, 0, 0, &sLU, 0, 0, ipiv);
	printf("\nLU = \n");
	blasfeo_print_dmat(n, n, &sLU, 0, 0);
	printf("\nipiv = \n");
	int_print_mat(1, n, ipiv, 1);

#if 0 // solve A X = P L U X = B  =>  L U X = P^T B
	blasfeo_pack_dmat(n, n, I, n, &sI, 0, 0);
	printf("\nI = \n");
	blasfeo_print_dmat(n, n, &sI, 0, 0);

	blasfeo_drowpe(n, ipiv, &sI);
	printf("\nperm(I) = \n");
	blasfeo_print_dmat(n, n, &sI, 0, 0);

	blasfeo_dtrsm_llnu(n, n, 1.0, &sLU, 0, 0, &sI, 0, 0, &sD, 0, 0);
	printf("\nperm(inv(L)) = \n");
	blasfeo_print_dmat(n, n, &sD, 0, 0);
	blasfeo_dtrsm_lunn(n, n, 1.0, &sLU, 0, 0, &sD, 0, 0, &sD, 0, 0);
	printf("\ninv(A) = \n");
	blasfeo_print_dmat(n, n, &sD, 0, 0);

	// convert from strmat to column major matrix
	blasfeo_unpack_dmat(n, n, &sD, 0, 0, D, n);
#elif 0 // solve X^T A^T = X^T (P L U)^T = B^T  =>  X^T U^T L^T = B^T P
	blasfeo_pack_tran_dmat(n, n, I, n, &sI, 0, 0);
	printf("\nI' = \n");
	blasfeo_print_dmat(n, n, &sI, 0, 0);

	blasfeo_dcolpe(n, ipiv, &sB);
	printf("\nperm(I') = \n");
	blasfeo_print_dmat(n, n, &sB, 0, 0);

	blasfeo_dtrsm_rltu(n, n, 1.0, &sLU, 0, 0, &sB, 0, 0, &sD, 0, 0);
	printf("\nperm(inv(L')) = \n");
	blasfeo_print_dmat(n, n, &sD, 0, 0);
	blasfeo_dtrsm_rutn(n, n, 1.0, &sLU, 0, 0, &sD, 0, 0, &sD, 0, 0);
	printf("\ninv(A') = \n");
	blasfeo_print_dmat(n, n, &sD, 0, 0);

	// convert from strmat to column major matrix
	blasfeo_unpack_tran_dmat(n, n, &sD, 0, 0, D, n);
#else // solve A^T X = (P L U)^T X = U^T L^T P^T X = B
	blasfeo_dgetr(n, n, &sLU, 0, 0, &sLUt, 0, 0);

	blasfeo_pack_dmat(n, n, I, n, &sI, 0, 0);
	printf("\nI = \n");
	blasfeo_print_dmat(n, n, &sI, 0, 0);

	blasfeo_dtrsm_llnn(n, n, 1.0, &sLUt, 0, 0, &sI, 0, 0, &sD, 0, 0);
	printf("\ninv(U^T) = \n");
	blasfeo_print_dmat(n, n, &sD, 0, 0);

	blasfeo_dtrsm_lunu(n, n, 1.0, &sLUt, 0, 0, &sD, 0, 0, &sD, 0, 0);
	printf("\n(inv(L^T)*inv(U^T)) = \n");
	blasfeo_print_dmat(n, n, &sD, 0, 0);

	blasfeo_drowpei(n, ipiv, &sD);
	printf("\nperm(inv(L^T)*inv(U^T)) = \n");
	blasfeo_print_dmat(n, n, &sD, 0, 0);

	// convert from strmat to column major matrix
	blasfeo_unpack_dmat(n, n, &sD, 0, 0, D, n);
#endif

	// print matrix in column-major format
	printf("\ninv(A) = \n");
	d_print_mat(n, n, D, n);



	//
	// free memory
	//

	d_free(A);
	d_free(B);
	d_free(D);
	d_free(I);
	int_free(ipiv);
//	blasfeo_free_dmat(&sA);
//	blasfeo_free_dmat(&sB);
//	blasfeo_free_dmat(&sD);
//	blasfeo_free_dmat(&sI);
	v_free_align(memory_strmat);

	return 0;

	}
