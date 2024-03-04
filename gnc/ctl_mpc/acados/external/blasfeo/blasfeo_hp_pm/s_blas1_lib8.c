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

#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE)
#include <mmintrin.h>
#include <xmmintrin.h>  // SSE
#include <emmintrin.h>  // SSE2
#include <pmmintrin.h>  // SSE3
#include <smmintrin.h>  // SSE4
#include <immintrin.h>  // AVX
#endif

#include <blasfeo_common.h>
#include <blasfeo_s_kernel.h>



// z = y + alpha*x, with increments equal to 1
void blasfeo_hp_saxpy(int m, float alpha, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sy, int yi, struct blasfeo_svec *sz, int zi)
	{
	float *x = sx->pa + xi;
	float *y = sy->pa + yi;
	float *z = sz->pa + zi;
	int ii;
	ii = 0;
	for( ; ii<m-3; ii+=4)
		{
		z[ii+0] = y[ii+0] + alpha*x[ii+0];
		z[ii+1] = y[ii+1] + alpha*x[ii+1];
		z[ii+2] = y[ii+2] + alpha*x[ii+2];
		z[ii+3] = y[ii+3] + alpha*x[ii+3];
		}
	for( ; ii<m; ii++)
		{
		z[ii+0] = y[ii+0] + alpha*x[ii+0];
		}
	return;
	}



void blasfeo_hp_saxpby(int m, float alpha, struct blasfeo_svec *sx, int xi, float beta, struct blasfeo_svec *sy, int yi, struct blasfeo_svec *sz, int zi)
	{
	if(m<=0)
		return;
	int ii;
	float *x = sx->pa + xi;
	float *y = sy->pa + yi;
	float *z = sz->pa + zi;
	ii = 0;
	for(; ii<m-3; ii+=4)
		{
		z[ii+0] = beta*y[ii+0] + alpha*x[ii+0];
		z[ii+1] = beta*y[ii+1] + alpha*x[ii+1];
		z[ii+2] = beta*y[ii+2] + alpha*x[ii+2];
		z[ii+3] = beta*y[ii+3] + alpha*x[ii+3];
		}
	for(; ii<m; ii++)
		z[ii+0] = beta*y[ii+0] + alpha*x[ii+0];
	return;
	}



// multiply two vectors
void blasfeo_hp_svecmul(int m, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sy, int yi, struct blasfeo_svec *sz, int zi)
	{

	if(m<=0)
		return;

	float *x = sx->pa + xi;
	float *y = sy->pa + yi;
	float *z = sz->pa + zi;
	int ii;
#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	__m256
		v_tmp,
		v_x0, v_y0;
#endif

	ii = 0;

#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		v_x0 = _mm256_loadu_ps( &x[ii+0] );
		v_y0 = _mm256_loadu_ps( &y[ii+0] );
		v_tmp = _mm256_mul_ps( v_x0, v_y0 );
		_mm256_storeu_ps( &z[ii+0], v_tmp );
		}
#endif
	for(; ii<m; ii++)
		{
		z[ii+0] = x[ii+0] * y[ii+0];
		}
	return;
	}



// multiply two vectors and add result to another vector
void blasfeo_hp_svecmulacc(int m, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sy, int yi, struct blasfeo_svec *sz, int zi)
	{

	if(m<=0)
		return;

	float *x = sx->pa + xi;
	float *y = sy->pa + yi;
	float *z = sz->pa + zi;
	int ii;
#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	__m256
		v_tmp,
		v_x0, v_y0, v_z0;
#endif

	ii = 0;

#if defined(TARGET_X64_INTEL_HASWELL) || defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	for(; ii<m-7; ii+=8)
		{
		v_x0 = _mm256_loadu_ps( &x[ii+0] );
		v_y0 = _mm256_loadu_ps( &y[ii+0] );
		v_z0 = _mm256_loadu_ps( &z[ii+0] );
		v_tmp = _mm256_mul_ps( v_x0, v_y0 );
		v_z0 = _mm256_add_ps( v_z0, v_tmp );
		_mm256_storeu_ps( &z[ii+0], v_z0 );
		}
#endif
	for(; ii<m; ii++)
		{
		z[ii+0] += x[ii+0] * y[ii+0];
		}
	return;
	}



// multiply two vectors and compute dot product
float blasfeo_hp_svecmuldot(int m, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sy, int yi, struct blasfeo_svec *sz, int zi)
	{

	if(m<=0)
		return 0.0;

	float *x = sx->pa + xi;
	float *y = sy->pa + yi;
	float *z = sz->pa + zi;
	int ii;
	float dot = 0.0;

	ii = 0;
	for(; ii<m; ii++)
		{
		z[ii+0] = x[ii+0] * y[ii+0];
		dot += z[ii+0];
		}
	return dot;
	}



// compute dot product
float blasfeo_hp_sdot(int m, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sy, int yi)
	{

	if(m<=0)
		return 0.0;

	float *x = sx->pa + xi;
	float *y = sy->pa + yi;
	int ii;
	float dot = 0.0;

	ii = 0;
	for(; ii<m; ii++)
		{
		dot += x[ii+0] * y[ii+0];
		}
	return dot;
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



//void blasfeo_srotg(float a, float b, float *c, float *s)
//	{
//	blasfeo_hp_srotg(a, b, c, s);
//	}



//void blasfeo_scolrot(int m, struct blasfeo_smat *sA, int ai, int aj0, int aj1, float c, float s)
//	{
//	blasfeo_hp_scolrot(m, sA, ai, aj0, aj1, c, s);
//	}



//void blasfeo_srowrot(int m, struct blasfeo_smat *sA, int ai0, int ai1, int aj, float c, float s)
//	{
//	blasfeo_hp_srowrot(m, sA, ai0, ai1, aj, c, s);
//	}



#endif
