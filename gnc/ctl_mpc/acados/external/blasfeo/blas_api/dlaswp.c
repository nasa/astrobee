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



#if defined(FORTRAN_BLAS_API)
#define blasfeo_lapack_dlaswp dlaswp_
#endif



void blasfeo_lapack_dlaswp(int *pn, double *A, int *plda, int *pk1, int *pk2, int *ipiv, int *pincx)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_lapack_dlaswp %d %p %d %d %d %p %d\n", *pn, A, *plda, *pk1, *pk2, ipiv, *pincx);
#endif

	int n = *pn;
	int lda = *plda;
	int k1 = *pk1;
	int k2 = *pk2;
	int incx = *pincx;

	int ix0, i1, i2;

	int ii, jj, ix, ip;

	double tmp;

	if(incx>=0)
		{
		ix0 = k1;
		i1 = k1;
		i2 = k2;
		ix = ix0;
		for(ii=i1; ii<=i2; ii++)
			{
			ip = ipiv[-1+ix];
			if(ip!=ii)
				{
				for(jj=0; jj<n; jj++)
					{
					tmp = A[-1+ii+jj*lda];
					A[-1+ii+jj*lda] = A[-1+ip+jj*lda];
					A[-1+ip+jj*lda] = tmp;
					}
				}
			ix = ix + incx;
			}
		}
	else
		{
//		ix0 = k1 + (k1-k2)*incx;
		ix0 = 1 + (1-k2)*incx;
		i1 = k2;
		i2 = k1;
		ix = ix0;
		for(ii=i1; ii>=i2; ii--)
			{
			ip = ipiv[-1+ix];
			if(ip!=ii)
				{
				for(jj=0; jj<n; jj++)
					{
					tmp = A[-1+ii+jj*lda];
					A[-1+ii+jj*lda] = A[-1+ip+jj*lda];
					A[-1+ip+jj*lda] = tmp;
					}
				}
			ix = ix + incx;
			}
		}


	return;

	}

