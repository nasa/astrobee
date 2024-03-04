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

    int ii;  // loop index

    int n = 12;  // matrix size

    // A
    struct blasfeo_dmat sA;            // matrix structure
    blasfeo_allocate_dmat(n, n, &sA);  // allocate and assign memory needed by A

    // B
    struct blasfeo_dmat sB;                       // matrix structure
    int B_size = blasfeo_memsize_dmat(n, n);      // size of memory needed by B
    void *B_mem_align;
    v_zeros_align(&B_mem_align, B_size);          // allocate memory needed by B
    blasfeo_create_dmat(n, n, &sB, B_mem_align);  // assign aligned memory to struct

    // C
    struct blasfeo_dmat sC;                                                  // matrix structure
    int C_size = blasfeo_memsize_dmat(n, n);                                 // size of memory needed by C
    C_size += 64;                                                            // 64-bytes alignment
    void *C_mem = malloc(C_size);
    void *C_mem_align = (void *) ((((unsigned long long) C_mem)+63)/64*64);  // align memory pointer
    blasfeo_create_dmat(n, n, &sC, C_mem_align);                             // assign aligned memory to struct

    // A
    double *A = malloc(n*n*sizeof(double));
    for(ii=0; ii<n*n; ii++)
        A[ii] = ii;
    int lda = n;
    blasfeo_pack_dmat(n, n, A, lda, &sA, 0, 0);  // convert from column-major to BLASFEO dmat
    free(A);

    // B
    blasfeo_dgese(n, n, 0.0, &sB, 0, 0);    // set B to zero
    for(ii=0; ii<n; ii++)
        BLASFEO_DMATEL(&sB, ii, ii) = 1.0;  // set B diagonal to 1.0 accessing dmat elements

    // C
    blasfeo_dgese(n, n, -1.0, &sC, 0, 0);  // set C to -1.0

    blasfeo_dgemm_nt(n, n, n, 1.0, &sA, 0, 0, &sB, 0, 0, 0.0, &sC, 0, 0, &sC, 0, 0);

    printf("\nC = \n");
    blasfeo_print_dmat(n, n, &sC, 0, 0);

    blasfeo_free_dmat(&sA);
    v_free_align(B_mem_align);
    free(C_mem);

    return 0;

    }

