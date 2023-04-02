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



void TRMM(char *side, char *uplo, char *transa, char *diag, int *pm, int *pn, REAL *alpha, REAL *A, int *plda, REAL *B, int *pldb)
	{

#ifdef TIME_INT
    blasfeo_timer timer;
	blasfeo_tic(&timer);
#endif

#if defined(DIM_CHECK)
	if( !(*side=='l' | *side=='L' | *side=='r' | *side=='R') )
		{
		printf("\nBLASFEO: dtrmm: wrong value for side\n");
		return;
		}
	if( !(*uplo=='l' | *uplo=='L' | *uplo=='u' | *uplo=='U') )
		{
		printf("\nBLASFEO: dtrmm: wrong value for uplo\n");
		return;
		}
	if( !(*transa=='c' | *transa=='C' | *transa=='n' | *transa=='N' | *transa=='t' | *transa=='T') )
		{
		printf("\nBLASFEO: dtrmm: wrong value for transa\n");
		return;
		}
	if( !(*diag=='n' | *diag=='N' | *diag=='u' | *diag=='U') )
		{
		printf("\nBLASFEO: dtrmm: wrong value for diag\n");
		return;
		}
#endif

	struct MAT sA;
	sA.pA = A;
	sA.m = *plda;

	struct MAT sB;
	sB.pA = B;
	sB.m = *pldb;

	if(*side=='l' | *side=='L') // _l
		{
		if(*uplo=='l' | *uplo=='L') // _ll
			{
			if(*transa=='n' | *transa=='N') // _lln
				{
				if(*diag=='n' | *diag=='N') // _llnn
					{
					TRMM_LLNN(*pm, *pn, *alpha, &sA, 0, 0, &sB, 0, 0, &sB, 0, 0);
					}
				else //if(*diag=='u' | *diag=='U') // _llnu
					{
					TRMM_LLNU(*pm, *pn, *alpha, &sA, 0, 0, &sB, 0, 0, &sB, 0, 0);
					}
				}
			else //if(*transa=='t' | *transa=='T' | *transa=='c' | *transa=='C') // _llt
				{
				if(*diag=='n' | *diag=='N') // _lltn
					{
					TRMM_LLTN(*pm, *pn, *alpha, &sA, 0, 0, &sB, 0, 0, &sB, 0, 0);
					}
				else //if(*diag=='u' | *diag=='U') // _lltu
					{
					TRMM_LLTU(*pm, *pn, *alpha, &sA, 0, 0, &sB, 0, 0, &sB, 0, 0);
					}
				}
			}
		else //if(*uplo=='u' | *uplo=='U') // _lu
			{
			if(*transa=='n' | *transa=='N') // _lun
				{
				if(*diag=='n' | *diag=='N') // _lunn
					{
					TRMM_LUNN(*pm, *pn, *alpha, &sA, 0, 0, &sB, 0, 0, &sB, 0, 0);
					}
				else //if(*diag=='u' | *diag=='U') // _lunu
					{
					TRMM_LUNU(*pm, *pn, *alpha, &sA, 0, 0, &sB, 0, 0, &sB, 0, 0);
					}
				}
			else //if(*transa=='t' | *transa=='T' | *transa=='c' | *transa=='C') // _lut
				{
				if(*diag=='n' | *diag=='N') // _lutn
					{
					TRMM_LUTN(*pm, *pn, *alpha, &sA, 0, 0, &sB, 0, 0, &sB, 0, 0);
					}
				else //if(*diag=='u' | *diag=='U') // _lutu
					{
					TRMM_LUTU(*pm, *pn, *alpha, &sA, 0, 0, &sB, 0, 0, &sB, 0, 0);
					}
				}
			}
		}
	else //if(*side=='r' | *side=='R') // _r
		{
		if(*uplo=='l' | *uplo=='L') // _rl
			{
			if(*transa=='n' | *transa=='N') // _rln
				{
				if(*diag=='n' | *diag=='N') // _rlnn
					{
					TRMM_RLNN(*pm, *pn, *alpha, &sA, 0, 0, &sB, 0, 0, &sB, 0, 0);
					}
				else //if(*diag=='u' | *diag=='U') // _rlnu
					{
					TRMM_RLNU(*pm, *pn, *alpha, &sA, 0, 0, &sB, 0, 0, &sB, 0, 0);
					}
				}
			else //if(*transa=='t' | *transa=='T' | *transa=='c' | *transa=='C') // _rlt
				{
				if(*diag=='n' | *diag=='N') // _rltn
					{
					TRMM_RLTN(*pm, *pn, *alpha, &sA, 0, 0, &sB, 0, 0, &sB, 0, 0);
					}
				else //if(*diag=='u' | *diag=='U') // _rltu
					{
					TRMM_RLTU(*pm, *pn, *alpha, &sA, 0, 0, &sB, 0, 0, &sB, 0, 0);
					}
				}
			}
		else //if(*uplo=='u' | *uplo=='U') // _ru
			{
			if(*transa=='n' | *transa=='N') // _run
				{
				if(*diag=='n' | *diag=='N') // _runn
					{
					TRMM_RUNN(*pm, *pn, *alpha, &sA, 0, 0, &sB, 0, 0, &sB, 0, 0);
					}
				else //if(*diag=='u' | *diag=='U') // _runu
					{
					TRMM_RUNU(*pm, *pn, *alpha, &sA, 0, 0, &sB, 0, 0, &sB, 0, 0);
					}
				}
			else //if(*transa=='t' | *transa=='T' | *transa=='c' | *transa=='C') // _rut
				{
				if(*diag=='n' | *diag=='N') // _rutn
					{
					TRMM_RUTN(*pm, *pn, *alpha, &sA, 0, 0, &sB, 0, 0, &sB, 0, 0);
					}
				else //if(*diag=='u' | *diag=='U') // _rutu
					{
					TRMM_RUTU(*pm, *pn, *alpha, &sA, 0, 0, &sB, 0, 0, &sB, 0, 0);
					}
				}
			}
		}

#ifdef TIME_INT
	double flops;
	if( *side=='l' | *side=='L' )
		{
		flops = *pm * *pm * *pn;
		}
	else
		{
		flops = *pm * *pn * *pn;
		}
	double time = blasfeo_toc(&timer);
	double Gflops = 1e-9 * flops / time;
	double Gflops_max = 3.4 * 16;
    printf("\nblasfeo trmm\t%c\t%c\t%c\t%c\t%d\t%d\t%f\t%f\n", *side, *uplo, *transa, *diag, *pm, *pn, Gflops, 100.0*Gflops/Gflops_max);
#endif

	return;

	}



