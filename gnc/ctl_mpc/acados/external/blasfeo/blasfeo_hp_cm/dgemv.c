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
#include <blasfeo_block_size.h>
#include <blasfeo_common.h>
#include <blasfeo_stdlib.h>
#include <blasfeo_d_aux.h>
#include <blasfeo_d_kernel.h>
#include <blasfeo_memory.h>

#include <blasfeo_timing.h>



#if ( defined(BLAS_API) & defined(MF_PANELMAJ) )
#define blasfeo_dmat blasfeo_cm_dmat
#define blasfeo_dvec blasfeo_cm_dvec
#define blasfeo_hp_dgemv_n blasfeo_hp_cm_dgemv_n
#define blasfeo_hp_dgemv_t blasfeo_hp_cm_dgemv_t
#define blasfeo_dgemv_n blasfeo_cm_dgemv_n
#define blasfeo_dgemv_t blasfeo_cm_dgemv_t
#endif



void blasfeo_hp_dgemv_n(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, double beta, struct blasfeo_dvec *sy, int yi, struct blasfeo_dvec *sz, int zi)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_dgemv_n (cm) %d %d %f %p %d %d %p %d %f %p %d %p %d\n", m, n, alpha, sA, ai, aj, sx, xi, beta, sy, yi, sz, zi);
#endif

	if(m<=0 | n<=0 | (alpha==0 & beta==0))
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	double *A = sA->pA + ai + aj*lda;
	double *x = sx->pa + xi;
	double *y = sy->pa + yi;
	double *z = sz->pa + zi;

	int ii;

	// copy and scale y into z
	if(beta==0.0)
		{
		ii = 0;
		for(; ii<m-3; ii+=4)
			{
			z[ii+0] = 0.0;
			z[ii+1] = 0.0;
			z[ii+2] = 0.0;
			z[ii+3] = 0.0;
			}
		for(; ii<m; ii++)
			{
			z[ii+0] = 0.0;
			}
		}
	else
		{
		ii = 0;
		for(; ii<m-3; ii+=4)
			{
			z[ii+0] = beta*y[ii+0];
			z[ii+1] = beta*y[ii+1];
			z[ii+2] = beta*y[ii+2];
			z[ii+3] = beta*y[ii+3];
			}
		for(; ii<m; ii++)
			{
			z[ii+0] = beta*y[ii+0];
			}
		}

	// main loop
	ii = 0;
	for(; ii<n-3; ii+=4)
		{
		kernel_dgemv_n_4_libc(m, &alpha, A+ii*lda, lda, x+ii, z);
		}
	// clean up at the end
	if(ii<n)
		{
		kernel_dgemv_n_4_vs_libc(m, &alpha, A+ii*lda, lda, x+ii, z, n-ii);
		}
	
	return;
	}



void blasfeo_hp_dgemv_t(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, double beta, struct blasfeo_dvec *sy, int yi, struct blasfeo_dvec *sz, int zi)
	{

#if defined(PRINT_NAME)
	printf("\nblasfeo_hp_dgemv_t (cm) %d %d %f %p %d %d %p %d %f %p %d %p %d\n", m, n, alpha, sA, ai, aj, sx, xi, beta, sy, yi, sz, zi);
#endif

	if(m<=0 | n<=0 | (alpha==0 & beta==0))
		return;

	// extract pointer to column-major matrices from structures
	int lda = sA->m;
	double *A = sA->pA + ai + aj*lda;
	double *x = sx->pa + xi;
	double *y = sy->pa + yi;
	double *z = sz->pa + zi;

	int ii;

	// main loop
	ii = 0;
	for(; ii<n-3; ii+=4)
		{
		kernel_dgemv_t_4_libc(m, &alpha, A+ii*lda, lda, x, &beta, y+ii, z+ii);
		}
	// clean up at the end
	if(ii<n)
		{
		kernel_dgemv_t_4_vs_libc(m, &alpha, A+ii*lda, lda, x, &beta, y+ii, z+ii, n-ii);
		}
	
	return;
	}



#if defined(LA_HIGH_PERFORMANCE)



void blasfeo_dgemv_n(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, double beta, struct blasfeo_dvec *sy, int yi, struct blasfeo_dvec *sz, int zi)

	{
	blasfeo_hp_dgemv_n(m, n, alpha, sA, ai, aj, sx, xi, beta, sy, yi, sz, zi);
	}



void blasfeo_dgemv_t(int m, int n, double alpha, struct blasfeo_dmat *sA, int ai, int aj, struct blasfeo_dvec *sx, int xi, double beta, struct blasfeo_dvec *sy, int yi, struct blasfeo_dvec *sz, int zi)

	{
	blasfeo_hp_dgemv_t(m, n, alpha, sA, ai, aj, sx, xi, beta, sy, yi, sz, zi);
	}



#endif
