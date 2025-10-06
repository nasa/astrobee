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
//#include <blasfeo_d_kernel.h>
#if defined(BLASFEO_REF_API)
#include <blasfeo_d_blasfeo_ref_api.h>
#endif



void blasfeo_hp_daxpy(int m, double alpha, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sy, int yi, struct blasfeo_dvec *sz, int zi)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_daxpy(m, alpha, sx, xi, sy, yi, sz, zi);
#else
	printf("\nblasfeo_daxpy: feature not implemented yet\n");
	exit(1);
#endif
	}



void blasfeo_hp_daxpby(int m, double alpha, struct blasfeo_dvec *sx, int xi, double beta, struct blasfeo_dvec *sy, int yi, struct blasfeo_dvec *sz, int zi)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_daxpby(m, alpha, sx, xi, beta, sy, yi, sz, zi);
#else
	printf("\nblasfeo_daxpby: feature not implemented yet\n");
	exit(1);
#endif
	}



// multiply two vectors
void blasfeo_hp_dvecmul(int m, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sy, int yi, struct blasfeo_dvec *sz, int zi)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_dvecmul(m, sx, xi, sy, yi, sz, zi);
#else
	printf("\nblasfeo_dvecmul: feature not implemented yet\n");
	exit(1);
#endif
	}



// multiply two vectors and add result to another vector
void blasfeo_hp_dvecmulacc(int m, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sy, int yi, struct blasfeo_dvec *sz, int zi)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_dvecmulacc(m, sx, xi, sy, yi, sz, zi);
#else
	printf("\nblasfeo_dvecmulacc: feature not implemented yet\n");
	exit(1);
#endif
	}



// multiply two vectors and compute dot product
double blasfeo_hp_dvecmuldot(int m, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sy, int yi, struct blasfeo_dvec *sz, int zi)
	{
#if defined(BLASFEO_REF_API)
	return blasfeo_ref_dvecmuldot(m, sx, xi, sy, yi, sz, zi);
#else
	printf("\nblasfeo_dvecmuldot: feature not implemented yet\n");
	exit(1);
#endif
	}



// compute dot product of two vectors
double blasfeo_hp_ddot(int m, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sy, int yi)
	{
#if defined(BLASFEO_REF_API)
	return blasfeo_ref_ddot(m, sx, xi, sy, yi);
#else
	printf("\nblasfeo_ddot: feature not implemented yet\n");
	exit(1);
#endif
	}



void blasfeo_hp_drotg(double a, double b, double *c, double *s)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_drotg(a, b, c, s);
#else
	printf("\nblasfeo_drotg: feature not implemented yet\n");
	exit(1);
#endif
	}



void blasfeo_hp_dcolrot(int m, struct blasfeo_dmat *sA, int ai, int aj0, int aj1, double c, double s)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_dcolrot(m, sA, ai, aj0, aj1, c, s);
#else
	printf("\nblasfeo_dcolrot: feature not implemented yet\n");
	exit(1);
#endif
	}
	


void blasfeo_hp_drowrot(int m, struct blasfeo_dmat *sA, int ai0, int ai1, int aj, double c, double s)
	{
#if defined(BLASFEO_REF_API)
	blasfeo_ref_drowrot(m, sA, ai0, ai1, aj, c, s);
#else
	printf("\nblasfeo_drowrot: feature not implemented yet\n");
	exit(1);
#endif
	}
	


#if defined(LA_HIGH_PERFORMANCE)



void blasfeo_daxpy(int m, double alpha, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sy, int yi, struct blasfeo_dvec *sz, int zi)
	{
	blasfeo_hp_daxpy(m, alpha, sx, xi, sy, yi, sz, zi);
	}



void blasfeo_daxpby(int m, double alpha, struct blasfeo_dvec *sx, int xi, double beta, struct blasfeo_dvec *sy, int yi, struct blasfeo_dvec *sz, int zi)
	{
	blasfeo_hp_daxpby(m, alpha, sx, xi, beta, sy, yi, sz, zi);
	}



void blasfeo_dvecmul(int m, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sy, int yi, struct blasfeo_dvec *sz, int zi)
	{
	blasfeo_hp_dvecmul(m, sx, xi, sy, yi, sz, zi);
	}



void blasfeo_dvecmulacc(int m, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sy, int yi, struct blasfeo_dvec *sz, int zi)
	{
	blasfeo_hp_dvecmulacc(m, sx, xi, sy, yi, sz, zi);
	}



double blasfeo_dvecmuldot(int m, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sy, int yi, struct blasfeo_dvec *sz, int zi)
	{
	return blasfeo_hp_dvecmuldot(m, sx, xi, sy, yi, sz, zi);
	}



double blasfeo_ddot(int m, struct blasfeo_dvec *sx, int xi, struct blasfeo_dvec *sy, int yi)
	{
	return blasfeo_hp_ddot(m, sx, xi, sy, yi);
	}



void blasfeo_drotg(double a, double b, double *c, double *s)
	{
	blasfeo_hp_drotg(a, b, c, s);
	}



void blasfeo_dcolrot(int m, struct blasfeo_dmat *sA, int ai, int aj0, int aj1, double c, double s)
	{
	blasfeo_hp_dcolrot(m, sA, ai, aj0, aj1, c, s);
	}



void blasfeo_drowrot(int m, struct blasfeo_dmat *sA, int ai0, int ai1, int aj, double c, double s)
	{
	blasfeo_hp_drowrot(m, sA, ai0, ai1, aj, c, s);
	}



#endif

