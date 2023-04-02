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



void TRSM(char *side, char *uplo, char *transa, char *diag, int *pm, int *pn, REAL *alpha, REAL *A, int *plda, REAL *B, int *pldb)
	{

#ifdef TIME_INT
    blasfeo_timer timer;
	blasfeo_tic(&timer);
#endif

#if defined(DIM_CHECK)
	if( !(*side=='l' | *side=='L' | *side=='r' | *side=='R') )
		{
		printf("\nBLASFEO: dtrsm: wrong value for side\n");
		return;
		}
	if( !(*uplo=='l' | *uplo=='L' | *uplo=='u' | *uplo=='U') )
		{
		printf("\nBLASFEO: dtrsm: wrong value for uplo\n");
		return;
		}
	if( !(*transa=='c' | *transa=='C' | *transa=='n' | *transa=='N' | *transa=='t' | *transa=='T') )
		{
		printf("\nBLASFEO: dtrsm: wrong value for transa\n");
		return;
		}
	if( !(*diag=='n' | *diag=='N' | *diag=='u' | *diag=='U') )
		{
		printf("\nBLASFEO: dtrsm: wrong value for diag\n");
		return;
		}
#endif

#if defined(FALLBACK_TO_EXTERNAL_BLAS)
	// TODO
#endif

	int p = (*side=='l' | *side=='L') ? *pm : *pn;

	REAL dA0[K_MAX_STACK];
	REAL *dA;
	if(p>K_MAX_STACK)
		{
		dA = (REAL *) malloc(p*sizeof(REAL));
		}
	else
		{
		dA = dA0;
		}

	struct MAT sA;
	sA.pA = A;
	sA.m = *plda;
	sA.dA = dA;
	sA.use_dA = 0; // always recompute diagonal

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
					TRSM_LLNN(*pm, *pn, *alpha, &sA, 0, 0, &sB, 0, 0, &sB, 0, 0);
					}
				else //if(*diag=='u' | *diag=='U') // _llnu
					{
					TRSM_LLNU(*pm, *pn, *alpha, &sA, 0, 0, &sB, 0, 0, &sB, 0, 0);
					}
				}
			else //if(*transa=='t' | *transa=='T' | *transa=='c' | *transa=='C') // _llt
				{
				if(*diag=='n' | *diag=='N') // _lltn
					{
					TRSM_LLTN(*pm, *pn, *alpha, &sA, 0, 0, &sB, 0, 0, &sB, 0, 0);
					}
				else //if(*diag=='u' | *diag=='U') // _lltu
					{
					TRSM_LLTU(*pm, *pn, *alpha, &sA, 0, 0, &sB, 0, 0, &sB, 0, 0);
					}
				}
			}
		else //if(*uplo=='u' | *uplo=='U') // _lu
			{
			if(*transa=='n' | *transa=='N') // _lun
				{
				if(*diag=='n' | *diag=='N') // _lunn
					{
					TRSM_LUNN(*pm, *pn, *alpha, &sA, 0, 0, &sB, 0, 0, &sB, 0, 0);
					}
				else //if(*diag=='u' | *diag=='U') // _lunu
					{
					TRSM_LUNU(*pm, *pn, *alpha, &sA, 0, 0, &sB, 0, 0, &sB, 0, 0);
					}
				}
			else //if(*transa=='t' | *transa=='T' | *transa=='c' | *transa=='C') // _lut
				{
				if(*diag=='n' | *diag=='N') // _lutn
					{
					TRSM_LUTN(*pm, *pn, *alpha, &sA, 0, 0, &sB, 0, 0, &sB, 0, 0);
					}
				else //if(*diag=='u' | *diag=='U') // _lutu
					{
					TRSM_LUTU(*pm, *pn, *alpha, &sA, 0, 0, &sB, 0, 0, &sB, 0, 0);
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
					TRSM_RLNN(*pm, *pn, *alpha, &sA, 0, 0, &sB, 0, 0, &sB, 0, 0);
					}
				else //if(*diag=='u' | *diag=='U') // _rlnu
					{
					TRSM_RLNU(*pm, *pn, *alpha, &sA, 0, 0, &sB, 0, 0, &sB, 0, 0);
					}
				}
			else //if(*transa=='t' | *transa=='T' | *transa=='c' | *transa=='C') // _rlt
				{
				if(*diag=='n' | *diag=='N') // _rltn
					{
					TRSM_RLTN(*pm, *pn, *alpha, &sA, 0, 0, &sB, 0, 0, &sB, 0, 0);
					}
				else //if(*diag=='u' | *diag=='U') // _rltu
					{
					TRSM_RLTU(*pm, *pn, *alpha, &sA, 0, 0, &sB, 0, 0, &sB, 0, 0);
					}
				}
			}
		else //if(*uplo=='u' | *uplo=='U') // _ru
			{
			if(*transa=='n' | *transa=='N') // _run
				{
				if(*diag=='n' | *diag=='N') // _runn
					{
					TRSM_RUNN(*pm, *pn, *alpha, &sA, 0, 0, &sB, 0, 0, &sB, 0, 0);
					}
				else //if(*diag=='u' | *diag=='U') // _runu
					{
					TRSM_RUNU(*pm, *pn, *alpha, &sA, 0, 0, &sB, 0, 0, &sB, 0, 0);
					}
				}
			else //if(*transa=='t' | *transa=='T' | *transa=='c' | *transa=='C') // _rut
				{
				if(*diag=='n' | *diag=='N') // _rutn
					{
					TRSM_RUTN(*pm, *pn, *alpha, &sA, 0, 0, &sB, 0, 0, &sB, 0, 0);
					}
				else //if(*diag=='u' | *diag=='U') // _rutu
					{
					TRSM_RUTU(*pm, *pn, *alpha, &sA, 0, 0, &sB, 0, 0, &sB, 0, 0);
					}
				}
			}
		}

	if(p>K_MAX_STACK)
		{
		free(dA);
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
    printf("\nblasfeo trsm\t%c\t%c\t%c\t%c\t%d\t%d\t%f\t%f\n", *side, *uplo, *transa, *diag, *pm, *pn, Gflops, 100.0*Gflops/Gflops_max);
#endif

	return;

	}


