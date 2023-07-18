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

//#define TIME_INT


#ifdef TIME_INT
#include <blasfeo_timing.h>
#endif



void POTRF(char *uplo, int *pm, REAL *C, int *pldc, int *info)
	{

#ifdef TIME_INT
    blasfeo_timer timer;
	blasfeo_tic(&timer);
#endif

#if defined(DIM_CHECK)
	if( !(*uplo=='l' | *uplo=='l' | *uplo=='U' | *uplo=='U') )
		{
		printf("\nBLASFEO: potrf: wrong value for uplo\n");
		return;
		}
#endif

	REAL dC0[K_MAX_STACK];
	REAL *dC;
	if(*pm>K_MAX_STACK)
		{
		dC = (REAL *) malloc(*pm*sizeof(REAL));
		}
	else
		{
		dC = dC0;
		}

	struct MAT sC;
	sC.pA = C;
	sC.m = *pldc;
	sC.dA = dC;

	int ldc = *pldc;

	int ii;

	if(*uplo=='l' | *uplo=='L')
		{
		POTRF_L(*pm, &sC, 0, 0, &sC, 0, 0);
		}
	else
		{
		POTRF_U(*pm, &sC, 0, 0, &sC, 0, 0);
		}

	if(*pm>K_MAX_STACK)
		{
		free(dC);
		}

	*info = 0;
	for(ii=0; ii<*pm; ii++)
		{
		if(C[ii*(ldc+1)]==0.0)
			{
			*info = ii+1;
			goto end_label;
			}
		}
#ifdef TIME_INT
	double flops;
	flops = 1.0/3.0 * *pm * *pm * *pm;
	double time = blasfeo_toc(&timer);
	double Gflops = 1e-9 * flops / time;
	double Gflops_max = 3.4 * 16;
    printf("\nblasfeo potrf\t%c\t%d\t%f\t%f\n", *uplo, *pm, Gflops, 100.0*Gflops/Gflops_max);
#endif

end_label:

	return;

	}

