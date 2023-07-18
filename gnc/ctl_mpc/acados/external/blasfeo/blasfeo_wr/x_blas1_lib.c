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



#if defined(LA_EXTERNAL_BLAS_WRAPPER)



void AXPY(int m, REAL alpha, struct XVEC *sx, int xi, struct XVEC *sy, int yi, struct XVEC *sz, int zi)
	{
	if(m<=0)
		return;
	int i1 = 1;
	REAL *x = sx->pa + xi;
	REAL *y = sy->pa + yi;
	REAL *z = sz->pa + zi;
	if(y!=z)
		COPY_(&m, y, &i1, z, &i1);
	AXPY_(&m, &alpha, x, &i1, z, &i1);
	return;
	}


void AXPBY(int m, REAL alpha, struct XVEC *sx, REAL beta, int xi, struct XVEC *sy, int yi, struct XVEC *sz, int zi)
	{
	if(m<=0)
		return;
	int i1 = 1;
	REAL *x = sx->pa + xi;
	REAL *y = sy->pa + yi;
	REAL *z = sz->pa + zi;
	if(y!=z)
		COPY_(&m, y, &i1, z, &i1);
	SCAL_(&m, &beta, z, &i1);
	AXPY_(&m, &alpha, x, &i1, z, &i1);
	return;
	}


// multiply two vectors
void VECMUL(int m, struct XVEC *sx, int xi, struct XVEC *sy, int yi, struct XVEC *sz, int zi)
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
void VECMULACC(int m, struct XVEC *sx, int xi, struct XVEC *sy, int yi, struct XVEC *sz, int zi)
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
REAL VECMULDOT(int m, struct XVEC *sx, int xi, struct XVEC *sy, int yi, struct XVEC *sz, int zi)
	{
	if(m<=0)
		return 0.0;
	REAL *x = sx->pa + xi;
	REAL *y = sy->pa + yi;
	REAL *z = sz->pa + zi;
	int ii;
	REAL dot = 0.0;
	ii = 0;
	for(; ii<m; ii++)
		{
		z[ii+0] = x[ii+0] * y[ii+0];
		dot += z[ii+0];
		}
	return dot;
	}



// compute dot product of two vectors
REAL DOT(int m, struct XVEC *sx, int xi, struct XVEC *sy, int yi)
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
void ROTG(REAL a, REAL b, REAL *c, REAL *s)
	{
	ROTG_(&a, &b, c, s);
	return;
	}



// apply plane rotation to the aj0 and aj1 columns of A at row index ai
void COLROT(int m, struct XMAT *sA, int ai, int aj0, int aj1, REAL c, REAL s)
	{
	int lda = sA->m;
	REAL *px = sA->pA + ai + aj0*lda;
	REAL *py = sA->pA + ai + aj1*lda;
	int i1 = 1;
	ROT_(&m, px, &i1, py, &i1, &c, &s);
	return;
	}



// apply plane rotation to the ai0 and ai1 rows of A at column index aj
void ROWROT(int m, struct XMAT *sA, int ai0, int ai1, int aj, REAL c, REAL s)
	{
	int lda = sA->m;
	REAL *px = sA->pA + ai0 + aj*lda;
	REAL *py = sA->pA + ai1 + aj*lda;
	ROT_(&m, px, &lda, py, &lda, &c, &s);
	return;
	}



#else

#error : wrong LA choice

#endif


