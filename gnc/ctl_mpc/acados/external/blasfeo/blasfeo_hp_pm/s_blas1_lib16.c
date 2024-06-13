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
#include <math.h>

//#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE)
//#include <mmintrin.h>
//#include <xmmintrin.h>  // SSE
//#include <emmintrin.h>  // SSE2
//#include <pmmintrin.h>  // SSE3
//#include <smmintrin.h>  // SSE4
//#include <immintrin.h>  // AVX
//#endif

#include <blasfeo_common.h>
//#include <blasfeo_s_kernel.h>
#include <blasfeo_s_blasfeo_ref_api.h>



void blasfeo_hp_saxpy(int m, float alpha, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sy, int yi, struct blasfeo_svec *sz, int zi)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_saxpy(m, alpha, sx, xi, sy, yi, sz, zi);
#else
	printf("\nblasfeo_saxpy: feature not implemented yet\n");
	exit(1);
#endif
	}



void blasfeo_hp_saxpby(int m, float alpha, struct blasfeo_svec *sx, int xi, float beta, struct blasfeo_svec *sy, int yi, struct blasfeo_svec *sz, int zi)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_saxpby(m, alpha, sx, xi, beta, sy, yi, sz, zi);
#else
	printf("\nblasfeo_saxpby: feature not implemented yet\n");
	exit(1);
#endif
	}



// multiply two vectors
void blasfeo_hp_svecmul(int m, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sy, int yi, struct blasfeo_svec *sz, int zi)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_svecmul(m, sx, xi, sy, yi, sz, zi);
#else
	printf("\nblasfeo_svecmul: feature not implemented yet\n");
	exit(1);
#endif
	}



// multiply two vectors and add result to another vector
void blasfeo_hp_svecmulacc(int m, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sy, int yi, struct blasfeo_svec *sz, int zi)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_svecmulacc(m, sx, xi, sy, yi, sz, zi);
#else
	printf("\nblasfeo_svecmulacc: feature not implemented yet\n");
	exit(1);
#endif
	}



// multiply two vectors and compute dot product
float blasfeo_hp_svecmuldot(int m, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sy, int yi, struct blasfeo_svec *sz, int zi)
	{
#if defined(BLASFEO_REF_API)
	return blasfeo_ref_svecmuldot(m, sx, xi, sy, yi, sz, zi);
#else
	printf("\nblasfeo_svecmuldot: feature not implemented yet\n");
	exit(1);
#endif
	}



// compute dot product of two vectors
float blasfeo_hp_sdot(int m, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sy, int yi)
	{
#if defined(BLASFEO_REF_API)
	return blasfeo_ref_sdot(m, sx, xi, sy, yi);
#else
	printf("\nblasfeo_sdot: feature not implemented yet\n");
	exit(1);
#endif
	}



void blasfeo_hp_srotg(float a, float b, float *c, float *s)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_srotg(a, b, c, s);
#else
	printf("\nblasfeo_srotg: feature not implemented yet\n");
	exit(1);
#endif
	}



void blasfeo_hp_scolrot(int m, struct blasfeo_smat *sA, int ai, int aj0, int aj1, float c, float s)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_scolrot(m, sA, ai, aj0, aj1, c, s);
#else
	printf("\nblasfeo_scolrot: feature not implemented yet\n");
	exit(1);
#endif
	}
	


void blasfeo_hp_srowrot(int m, struct blasfeo_smat *sA, int ai0, int ai1, int aj, float c, float s)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_srowrot(m, sA, ai0, ai1, aj, c, s);
#else
	printf("\nblasfeo_srowrot: feature not implemented yet\n");
	exit(1);
#endif
	}
	


#if defined(LA_HIGH_PERFORMANCE)



void blasfeo_saxpy(int m, float alpha, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sy, int yi, struct blasfeo_svec *sz, int zi)
	{
	blasfeo_hp_saxpy(m, alpha, sx, xi, sy, yi, sz, zi);
	}



void blasfeo_saxpby(int m, float alpha, struct blasfeo_svec *sx, int xi, float beta, struct blasfeo_svec *sy, int yi, struct blasfeo_svec *sz, int zi)
	{
	blasfeo_hp_saxpby(m, alpha, sx, xi, beta, sy, yi, sz, zi);
	}



void blasfeo_svecmul(int m, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sy, int yi, struct blasfeo_svec *sz, int zi)
	{
	blasfeo_hp_svecmul(m, sx, xi, sy, yi, sz, zi);
	}



void blasfeo_svecmulacc(int m, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sy, int yi, struct blasfeo_svec *sz, int zi)
	{
	blasfeo_hp_svecmulacc(m, sx, xi, sy, yi, sz, zi);
	}



float blasfeo_svecmuldot(int m, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sy, int yi, struct blasfeo_svec *sz, int zi)
	{
	return blasfeo_hp_svecmuldot(m, sx, xi, sy, yi, sz, zi);
	}



float blasfeo_sdot(int m, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sy, int yi)
	{
	return blasfeo_hp_sdot(m, sx, xi, sy, yi);
	}



void blasfeo_srotg(float a, float b, float *c, float *s)
	{
	blasfeo_hp_srotg(a, b, c, s);
	}



void blasfeo_scolrot(int m, struct blasfeo_smat *sA, int ai, int aj0, int aj1, float c, float s)
	{
	blasfeo_hp_scolrot(m, sA, ai, aj0, aj1, c, s);
	}



void blasfeo_srowrot(int m, struct blasfeo_smat *sA, int ai0, int ai1, int aj, float c, float s)
	{
	blasfeo_hp_srowrot(m, sA, ai0, ai1, aj, c, s);
	}



#endif


