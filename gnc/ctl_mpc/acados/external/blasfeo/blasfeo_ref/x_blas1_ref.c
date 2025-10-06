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



void REF_AXPY(int m, REAL alpha, struct XVEC *sx, int xi, struct XVEC *sy, int yi, struct XVEC *sz, int zi)
	{
	if(m<=0)
		return;
	int ii;
	REAL *x = sx->pa + xi;
	REAL *y = sy->pa + yi;
	REAL *z = sz->pa + zi;
	ii = 0;
	for(; ii<m-3; ii+=4)
		{
		z[ii+0] = y[ii+0] + alpha*x[ii+0];
		z[ii+1] = y[ii+1] + alpha*x[ii+1];
		z[ii+2] = y[ii+2] + alpha*x[ii+2];
		z[ii+3] = y[ii+3] + alpha*x[ii+3];
		}
	for(; ii<m; ii++)
		z[ii+0] = y[ii+0] + alpha*x[ii+0];
	return;
	}



void REF_AXPBY(int m, REAL alpha, struct XVEC *sx, int xi, REAL beta, struct XVEC *sy, int yi, struct XVEC *sz, int zi)
	{
	if(m<=0)
		return;
	int ii;
	REAL *x = sx->pa + xi;
	REAL *y = sy->pa + yi;
	REAL *z = sz->pa + zi;
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
void REF_VECMUL(int m, struct XVEC *sx, int xi, struct XVEC *sy, int yi, struct XVEC *sz, int zi)
	{
	if(m<=0)
		return;
	REAL *x = sx->pa + xi;
	REAL *y = sy->pa + yi;
	REAL *z = sz->pa + zi;
	int ii;
	ii = 0;
	for(; ii<m-3; ii+=4)
		{
		z[ii+0] = x[ii+0] * y[ii+0];
		z[ii+1] = x[ii+1] * y[ii+1];
		z[ii+2] = x[ii+2] * y[ii+2];
		z[ii+3] = x[ii+3] * y[ii+3];
		}
	for(; ii<m; ii++)
		{
		z[ii+0] = x[ii+0] * y[ii+0];
		}
	return;
	}



// multiply two vectors and add result to another vector
void REF_VECMULACC(int m, struct XVEC *sx, int xi, struct XVEC *sy, int yi, struct XVEC *sz, int zi)
	{
	if(m<=0)
		return;
	REAL *x = sx->pa + xi;
	REAL *y = sy->pa + yi;
	REAL *z = sz->pa + zi;
	int ii;
	ii = 0;
	for(; ii<m-3; ii+=4)
		{
		z[ii+0] += x[ii+0] * y[ii+0];
		z[ii+1] += x[ii+1] * y[ii+1];
		z[ii+2] += x[ii+2] * y[ii+2];
		z[ii+3] += x[ii+3] * y[ii+3];
		}
	for(; ii<m; ii++)
		{
		z[ii+0] += x[ii+0] * y[ii+0];
		}
	return;
	}



// multiply two vectors and compute dot product
REAL REF_VECMULDOT(int m, struct XVEC *sx, int xi, struct XVEC *sy, int yi, struct XVEC *sz, int zi)
	{
	if(m<=0)
		return 0.0;
	REAL *x = sx->pa + xi;
	REAL *y = sy->pa + yi;
	REAL *z = sz->pa + zi;
	int ii;
	REAL dot = 0.0;
	ii = 0;
	for(; ii<m-3; ii+=4)
		{
		z[ii+0] = x[ii+0] * y[ii+0];
		z[ii+1] = x[ii+1] * y[ii+1];
		z[ii+2] = x[ii+2] * y[ii+2];
		z[ii+3] = x[ii+3] * y[ii+3];
		dot += z[ii+0] + z[ii+1] + z[ii+2] + z[ii+3];
		}
	for(; ii<m; ii++)
		{
		z[ii+0] = x[ii+0] * y[ii+0];
		dot += z[ii+0];
		}
	return dot;
	}



// compute dot product of two vectors
REAL REF_DOT(int m, struct XVEC *sx, int xi, struct XVEC *sy, int yi)
	{
	if(m<=0)
		return 0.0;
	REAL *x = sx->pa + xi;
	REAL *y = sy->pa + yi;
	int ii;
	REAL dot = 0.0;
	ii = 0;
	for(; ii<m-3; ii+=4)
		{
		dot += x[ii+0] * y[ii+0];
		dot += x[ii+1] * y[ii+1];
		dot += x[ii+2] * y[ii+2];
		dot += x[ii+3] * y[ii+3];
		}
	for(; ii<m; ii++)
		{
		dot += x[ii+0] * y[ii+0];
		}
	return dot;
	}



// construct givens plane rotation
void REF_ROTG(REAL a, REAL b, REAL *c, REAL *s)
	{
	REAL aa = FABS(a);
	REAL bb = FABS(b);
	REAL roe = (aa >= bb) ? a : b;
	REAL scale = aa + bb;
	REAL r;
	if (scale == 0)
		{
		*c = 1.0;
		*s = 0.0;
		}
	else
		{
		aa = a/scale;
		bb = b/scale;
		r = scale * SQRT(aa*aa + bb*bb);
		r = r * (roe >= 0 ? 1 : -1);
		*c = a / r;
		*s = b / r;	
		}
	return;
	}



// apply plane rotation to the aj0 and aj1 columns of A at row index ai
void REF_COLROT(int m, struct XMAT *sA, int ai, int aj0, int aj1, REAL c, REAL s)
	{
	int lda = sA->m;
	REAL *px = sA->pA + ai + aj0*lda;
	REAL *py = sA->pA + ai + aj1*lda;
	int ii;
	REAL d_tmp;
	for(ii=0; ii<m; ii++)
		{
		d_tmp  = c*px[ii] + s*py[ii];
		py[ii] = c*py[ii] - s*px[ii];
		px[ii] = d_tmp;
		}
	return;
	}



// apply plane rotation to the ai0 and ai1 rows of A at column index aj
void REF_ROWROT(int m, struct XMAT *sA, int ai0, int ai1, int aj, REAL c, REAL s)
	{
	int lda = sA->m;
	REAL *px = sA->pA + ai0 + aj*lda;
	REAL *py = sA->pA + ai1 + aj*lda;
	int ii;
	REAL d_tmp;
	for(ii=0; ii<m; ii++)
		{
		d_tmp  = c*px[ii*lda] + s*py[ii*lda];
		py[ii*lda] = c*py[ii*lda] - s*px[ii*lda];
		px[ii*lda] = d_tmp;
		}
	return;
	}



#if (defined(LA_REFERENCE) & defined(REF)) | (defined(LA_HIGH_PERFORMANCE) & defined(HP_CM))



void AXPY(int m, REAL alpha, struct XVEC *sx, int xi, struct XVEC *sy, int yi, struct XVEC *sz, int zi)
	{
	REF_AXPY(m, alpha, sx, xi, sy, yi, sz, zi);
	}



void AXPBY(int m, REAL alpha, struct XVEC *sx, int xi, REAL beta, struct XVEC *sy, int yi, struct XVEC *sz, int zi)
	{
	REF_AXPBY(m, alpha, sx, xi, beta, sy, yi, sz, zi);
	}



void VECMUL(int m, struct XVEC *sx, int xi, struct XVEC *sy, int yi, struct XVEC *sz, int zi)
	{
	REF_VECMUL(m, sx, xi, sy, yi, sz, zi);
	}



void VECMULACC(int m, struct XVEC *sx, int xi, struct XVEC *sy, int yi, struct XVEC *sz, int zi)
	{
	REF_VECMULACC(m, sx, xi, sy, yi, sz, zi);
	}



REAL VECMULDOT(int m, struct XVEC *sx, int xi, struct XVEC *sy, int yi, struct XVEC *sz, int zi)
	{
	return REF_VECMULDOT(m, sx, xi, sy, yi, sz, zi);
	}



REAL DOT(int m, struct XVEC *sx, int xi, struct XVEC *sy, int yi)
	{
	return REF_DOT(m, sx, xi, sy, yi);
	}



void ROTG(REAL a, REAL b, REAL *c, REAL *s)
	{
	REF_ROTG(a, b, c, s);
	}



void COLROT(int m, struct XMAT *sA, int ai, int aj0, int aj1, REAL c, REAL s)
	{
	REF_COLROT(m, sA, ai, aj0, aj1, c, s);
	}



void ROWROT(int m, struct XMAT *sA, int ai0, int ai1, int aj, REAL c, REAL s)
	{
	REF_ROWROT(m, sA, ai0, ai1, aj, c, s);
	}



#endif
