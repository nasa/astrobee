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

	float *A; s_zeros(&A, n, n);
	for(ii=0; ii<n*n; ii++) A[ii] = ii;
//	s_print_mat(n, n, A, n);

	// spd matrix
	float *B; s_zeros(&B, n, n);
	for(ii=0; ii<n; ii++) B[ii*(n+1)] = 1.0;
//	s_print_mat(n, n, B, n);

	// identity
	float *I; s_zeros(&I, n, n);
	for(ii=0; ii<n; ii++) I[ii*(n+1)] = 1.0;
//	s_print_mat(n, n, B, n);

	// result matrix
	float *D; s_zeros(&D, n, n);
//	s_print_mat(n, n, D, n);

	// permutation indeces
	int *ipiv; int_zeros(&ipiv, n, 1);

	//
	// matrices in matrix struct format
	//

	// work space enough for 5 matrix structs for size n times n
	int size_strmat = 5*blasfeo_memsize_smat(n, n);
	void *memory_strmat; v_zeros_align(&memory_strmat, size_strmat);
	char *ptr_memory_strmat = (char *) memory_strmat;

	struct blasfeo_smat sA;
//	blasfeo_allocate_smat(n, n, &sA);
	blasfeo_create_smat(n, n, &sA, ptr_memory_strmat);
	ptr_memory_strmat += sA.memsize;
	// convert from column major matrix to strmat
	blasfeo_pack_smat(n, n, A, n, &sA, 0, 0);
	printf("\nA = \n");
	blasfeo_print_smat(n, n, &sA, 0, 0);

	struct blasfeo_smat sB;
//	blasfeo_allocate_smat(n, n, &sB);
	blasfeo_create_smat(n, n, &sB, ptr_memory_strmat);
	ptr_memory_strmat += sB.memsize;
	// convert from column major matrix to strmat
	blasfeo_pack_smat(n, n, B, n, &sB, 0, 0);
	printf("\nB = \n");
	blasfeo_print_smat(n, n, &sB, 0, 0);

	struct blasfeo_smat sI;
//	blasfeo_allocate_smat(n, n, &sI);
	blasfeo_create_smat(n, n, &sI, ptr_memory_strmat);
	ptr_memory_strmat += sI.memsize;
	// convert from column major matrix to strmat

	struct blasfeo_smat sD;
//	blasfeo_allocate_smat(n, n, &sD);
	blasfeo_create_smat(n, n, &sD, ptr_memory_strmat);
	ptr_memory_strmat += sD.memsize;

	struct blasfeo_smat sLU;
//	blasfeo_allocate_smat(n, n, &sD);
	blasfeo_create_smat(n, n, &sLU, ptr_memory_strmat);
	ptr_memory_strmat += sLU.memsize;

	blasfeo_sgemm_nt(n, n, n, 1.0, &sA, 0, 0, &sA, 0, 0, 1.0, &sB, 0, 0, &sD, 0, 0);
	printf("\nB+A*A' = \n");
	blasfeo_print_smat(n, n, &sD, 0, 0);

//	blasfeo_sgetrf_nopivot(n, n, &sD, 0, 0, &sD, 0, 0);
	blasfeo_sgetrf_rp(n, n, &sD, 0, 0, &sLU, 0, 0, ipiv);
	printf("\nLU = \n");
	blasfeo_print_smat(n, n, &sLU, 0, 0);
	printf("\nipiv = \n");
	int_print_mat(1, n, ipiv, 1);

#if 0 // solve P L U X = P B
	blasfeo_pack_smat(n, n, I, n, &sI, 0, 0);
	printf("\nI = \n");
	blasfeo_print_smat(n, n, &sI, 0, 0);

	blasfeo_srowpe(n, ipiv, &sI);
	printf("\nperm(I) = \n");
	blasfeo_print_smat(n, n, &sI, 0, 0);

	blasfeo_strsm_llnu(n, n, 1.0, &sLU, 0, 0, &sI, 0, 0, &sD, 0, 0);
	printf("\nperm(inv(L)) = \n");
	blasfeo_print_smat(n, n, &sD, 0, 0);
	blasfeo_strsm_lunn(n, n, 1.0, &sLU, 0, 0, &sD, 0, 0, &sD, 0, 0);
	printf("\ninv(A) = \n");
	blasfeo_print_smat(n, n, &sD, 0, 0);

	// convert from strmat to column major matrix
	blasfeo_unpack_smat(n, n, &sD, 0, 0, D, n);
#else // solve X^T (P L U)^T = B^T P^T
	blasfeo_pack_tran_smat(n, n, I, n, &sI, 0, 0);
	printf("\nI' = \n");
	blasfeo_print_smat(n, n, &sI, 0, 0);

	blasfeo_scolpe(n, ipiv, &sB);
	printf("\nperm(I') = \n");
	blasfeo_print_smat(n, n, &sB, 0, 0);

	blasfeo_strsm_rltu(n, n, 1.0, &sLU, 0, 0, &sB, 0, 0, &sD, 0, 0);
	printf("\nperm(inv(L')) = \n");
	blasfeo_print_smat(n, n, &sD, 0, 0);
	blasfeo_strsm_rutn(n, n, 1.0, &sLU, 0, 0, &sD, 0, 0, &sD, 0, 0);
	printf("\ninv(A') = \n");
	blasfeo_print_smat(n, n, &sD, 0, 0);

	// convert from strmat to column major matrix
	blasfeo_unpack_tran_smat(n, n, &sD, 0, 0, D, n);
#endif

	// print matrix in column-major format
	printf("\ninv(A) = \n");
	s_print_mat(n, n, D, n);



	//
	// free memory
	//

	s_free(A);
	s_free(B);
	s_free(D);
	s_free(I);
	int_free(ipiv);
//	blasfeo_free_smat(&sA);
//	blasfeo_free_smat(&sB);
//	blasfeo_free_smat(&sD);
//	blasfeo_free_smat(&sI);
	v_free_align(memory_strmat);

	return 0;

	}

