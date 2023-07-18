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

void GEMV(char *trans, int *pm, int *pn, REAL *alpha, REAL *A, int *plda, REAL *x0, int *pincx, REAL *beta, REAL *y0, int *pincy)
	{

#if defined(DIM_CHECK)
	if( !(*trans=='n' | *trans=='t' | *trans=='c' | *trans=='N' | *trans=='T' | *trans=='C') )
		{
		printf("\nBLASFEO: gemv: wrong value for trans: %c\n", trans);
		return;
		}
	if(*pincx==0 | *pincy==0)
		{
		printf("\nBLASFEO: gemv: wrong value for incx or incy: %d %d\n", *pincx, *pincy);
		return;
		}
#endif

	int ii;

	REAL *x, *y;
	int kx, ky;

	REAL x_stack[K_MAX_STACK];
	REAL y_stack[K_MAX_STACK];

	int m = *pm;
	int n = *pn;
	int incx = *pincx;
	int incy = *pincy;

	int lx, ly;
	if(*trans=='n' | *trans=='N')
		{
		lx = n;
		ly = m;
		}
	else
		{
		lx = m;
		ly = n;
		}

	if(incx==1)
		{
		x = x0;
		}
	else
		{
		if(lx>K_MAX_STACK)
			x = malloc(lx*sizeof(REAL));
		else
			x = x_stack;

		if(incx>0)
			kx = 0;
		else
			kx = - (lx-1) * incx;

		for(ii=0; ii<lx; ii++)
			x[ii] = x0[kx + ii*incx];
		}

	if(incy==1)
		{
		y = y0;
		}
	else
		{
		if(ly>K_MAX_STACK)
			y = malloc(ly*sizeof(REAL));
		else
			y = y_stack;

		if(incy>0)
			ky = 0;
		else
			ky = - (ly-1) * incy;

		for(ii=0; ii<ly; ii++)
			y[ii] = y0[ky + ii*incy];
		}

	struct MAT sA;
	sA.pA = A;
	sA.m = *plda;

	struct VEC sx;
	sx.pa = x;

	struct VEC sy;
	sy.pa = y;

	if(*trans=='n' | *trans=='N')
		{
		GEMV_N(m, n, *alpha, &sA, 0, 0, &sx, 0, *beta, &sy, 0, &sy, 0);
		}
	else
		{
		GEMV_T(m, n, *alpha, &sA, 0, 0, &sx, 0, *beta, &sy, 0, &sy, 0);
		}

	if(incx!=1)
		{
		if(lx>K_MAX_STACK)
			free(x);
		}

	if(incy!=1)
		{
		for(ii=0; ii<ly; ii++)
			y0[ky + ii*incy] = y[ii];

		if(ly>K_MAX_STACK)
			free(y);
		}

	return;

	}

