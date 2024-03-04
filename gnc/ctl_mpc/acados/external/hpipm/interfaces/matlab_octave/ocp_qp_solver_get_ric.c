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
#include "hpipm_d_ocp_qp_ipm.h"
// mex
#include "mex.h"



void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
	{

//	mexPrintf("\nin ocp_qp_sol_get\n");

	long long *l_ptr;

	int ii, jj;

	/* RHS */

	// qp
	l_ptr = mxGetData( prhs[0] );
	struct d_ocp_qp *qp = (struct d_ocp_qp *) *l_ptr;

	// dim
	struct d_ocp_qp_dim *dim = qp->dim;
	int N = dim->N;
	int *nx = dim->nx;
	int *nu = dim->nu;

	// arg
	l_ptr = mxGetData( prhs[1] );
	struct d_ocp_qp_ipm_arg *arg = (struct d_ocp_qp_ipm_arg *) *l_ptr;

	// ws
	l_ptr = mxGetData( prhs[2] );
	struct d_ocp_qp_ipm_ws *ws = (struct d_ocp_qp_ipm_ws *) *l_ptr;

	// field
	char *field = mxArrayToString( prhs[3] );

	// stage0
	int stage0 = mxGetScalar( prhs[4] );

	// TODO stage 1 ???

	if(!strcmp(field, "ric_P"))
		{
		plhs[0] = mxCreateNumericMatrix(nx[stage0], nx[stage0], mxDOUBLE_CLASS, mxREAL);
		double *mat = mxGetPr( plhs[0] );
		d_ocp_qp_ipm_get_ric_P(qp, arg, ws, stage0, mat);
		}
	else if(!strcmp(field, "ric_p"))
		{
		plhs[0] = mxCreateNumericMatrix(nx[stage0], 1, mxDOUBLE_CLASS, mxREAL);
		double *mat = mxGetPr( plhs[0] );
		d_ocp_qp_ipm_get_ric_p(qp, arg, ws, stage0, mat);
		}
	else if(!strcmp(field, "ric_K"))
		{
		plhs[0] = mxCreateNumericMatrix(nu[stage0], nx[stage0], mxDOUBLE_CLASS, mxREAL);
		double *mat = mxGetPr( plhs[0] );
		d_ocp_qp_ipm_get_ric_K(qp, arg, ws, stage0, mat);
		}
	else if(!strcmp(field, "ric_k"))
		{
		plhs[0] = mxCreateNumericMatrix(nu[stage0], 1, mxDOUBLE_CLASS, mxREAL);
		double *mat = mxGetPr( plhs[0] );
		d_ocp_qp_ipm_get_ric_k(qp, arg, ws, stage0, mat);
		}
	else
		{
		mexPrintf("\nocp_qp_solver_get_ric: field not supported: %s\n", field);
		return;
		}

	return;

	}



