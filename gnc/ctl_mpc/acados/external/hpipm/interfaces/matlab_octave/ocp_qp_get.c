/**************************************************************************************************
*                                                                                                 *
* This file is part of HPIPM.                                                                     *
*                                                                                                 *
* HPIPM -- High-Performance Interior Point Method.                                                *
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
// system
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
// hpipm
#include "hpipm_d_ocp_qp_dim.h"
#include "hpipm_d_ocp_qp.h"
// mex
#include "mex.h"



void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
	{

//	mexPrintf("\nin ocp_qp_get\n");

	long long *l_ptr;

	int ii;

	/* RHS */

	// qp
	l_ptr = mxGetData( prhs[0] );
	struct d_ocp_qp *qp = (struct d_ocp_qp *) *l_ptr;

	// dim
	struct d_ocp_qp_dim *dim = qp->dim;
	int N = dim->N;
	int *nx = dim->nx;
	int *nu = dim->nu;
	int *nbx = dim->nbx;
	int *ns = dim->ns;

	// field
	char *field = mxArrayToString( prhs[1] );

	// stage0
	int stage0 = mxGetScalar( prhs[2] );

	// stage1
	int stage1;
	if(nrhs==4)
		{
		stage1 = mxGetScalar( prhs[3] );
		}
	
	/* body */

	// A
	if( !strcmp(field, "A") )
		{
		if(nrhs==4)
			{
			int size_sum = 0;
			for(ii=stage0; ii<=stage1; ii++)
				{
				size_sum += nx[ii]*nx[ii+1];
				}
			plhs[0] = mxCreateNumericMatrix(size_sum, 1, mxDOUBLE_CLASS, mxREAL);
			double *mat = mxGetPr( plhs[0] );
			size_sum = 0;
			for(ii=stage0; ii<=stage1; ii++)
				{
				d_ocp_qp_get(field, ii, qp, mat+size_sum);
				size_sum += nx[ii]*nx[ii+1];
				}
			}
		else
			{
			plhs[0] = mxCreateNumericMatrix(nx[stage0], nx[stage0+1], mxDOUBLE_CLASS, mxREAL);
			double *mat = mxGetPr( plhs[0] );
			d_ocp_qp_get(field, stage0, qp, mat);
			}
		}
	// lbx ubx
	else if( !strcmp(field, "lbx") | !strcmp(field, "lx") | !strcmp(field, "ubx") | !strcmp(field, "ux") )
		{
		if(nrhs==4)
			{
			int nbx_sum = 0;
			for(ii=stage0; ii<=stage1; ii++)
				{
				nbx_sum += nbx[ii];
				}
			plhs[0] = mxCreateNumericMatrix(nbx_sum, 1, mxDOUBLE_CLASS, mxREAL);
			double *mat = mxGetPr( plhs[0] );
			nbx_sum = 0;
			for(ii=stage0; ii<=stage1; ii++)
				{
				d_ocp_qp_get(field, ii, qp, mat+nbx_sum);
				nbx_sum += nbx[ii];
				}
			}
		else
			{
			plhs[0] = mxCreateNumericMatrix(nbx[stage0], 1, mxDOUBLE_CLASS, mxREAL);
			double *mat = mxGetPr( plhs[0] );
			d_ocp_qp_get(field, stage0, qp, mat);
			}
		}
	else
		{
		mexPrintf("\nocp_qp_get: field not supported: %s\n", field);
		return;
		}

	return;

	}





