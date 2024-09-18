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

#include <stdlib.h>
#include <stdio.h>


#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_d_aux.h>
#include <blasfeo_d_kernel.h>
#include <blasfeo_d_blas.h>



#if defined(FORTRAN_BLAS_API)
#define blasfeo_lapack_dgetrs dgetrs_
#define blasfeo_lapack_dlaswp dlaswp_
#define blasfeo_blas_dtrsm dtrsm_
#endif



void blasfeo_lapack_dgetrs(char *trans, int *pm, int *pn, double *A, int *plda, int *ipiv, double *B, int *pldb, int *info)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_lapack_dgetrs %c %d %d %p %d %p %p %d %d\n", *trans, *pm, *pn, A, *plda, ipiv, B, *pldb, *info);
#endif

	int m = *pm;
	int n = *pn;

//	printf("\nm %d n %d lda %d ldb %d\n", m, n, *plda, *pldb);

	char c_l = 'l';
	char c_n = 'n';
	char c_t = 't';
	char c_u = 'u';

	int i_1 = 1;
	int i_m1 = -1;

	double d_1 = 1.0;

	*info = 0;

	if(m==0 | n==0)
		return;
	
//	printf("\n%c\n", *trans);
	if(*trans=='n' | *trans=='N')
		{
		blasfeo_lapack_dlaswp(pm, B, pldb, &i_1, pm, ipiv, &i_1);
		blasfeo_blas_dtrsm(&c_l, &c_l, &c_n, &c_u, pm, pn, &d_1, A, plda, B, pldb);
		blasfeo_blas_dtrsm(&c_l, &c_u, &c_n, &c_n, pm, pn, &d_1, A, plda, B, pldb);
		}
	else
		{
		blasfeo_blas_dtrsm(&c_l, &c_u, &c_t, &c_n, pm, pn, &d_1, A, plda, B, pldb);
		blasfeo_blas_dtrsm(&c_l, &c_l, &c_t, &c_u, pm, pn, &d_1, A, plda, B, pldb);
		blasfeo_lapack_dlaswp(pm, B, pldb, &i_1, pm, ipiv, &i_m1);
		}

	return;

	}


