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
#include "hpipm_d_ocp_qcqp_ipm.h"
// mex
#include "mex.h"



void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
	{

//	mexPrintf("\nin ocp_qcqp_sol_get\n");

	long long *l_ptr;

	int ii, jj;

	/* RHS */

	// ws
	l_ptr = mxGetData( prhs[0] );
	struct d_ocp_qcqp_ipm_ws *ws = (struct d_ocp_qcqp_ipm_ws *) *l_ptr;

	// field
	char *field = mxArrayToString( prhs[1] );

	if(!strcmp(field, "status") | !strcmp(field, "iter"))
		{
		plhs[0] = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
		double *mat_ptr = mxGetPr( plhs[0] );
		int tmp_int;
		d_ocp_qcqp_ipm_get(field, ws, &tmp_int);
		*mat_ptr = (double) tmp_int;
		}
	else if(!strcmp(field, "max_res_stat") | !strcmp(field, "max_res_eq") | !strcmp(field, "max_res_ineq") | !strcmp(field, "max_res_comp"))
		{
		plhs[0] = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
		double *mat_ptr = mxGetPr( plhs[0] );
		d_ocp_qcqp_ipm_get(field, ws, mat_ptr);
		}
	else if(!strcmp(field, "stat"))
		{
		int iter;
		int stat_m;
		double *stat;
		d_ocp_qcqp_ipm_get("iter", ws, &iter);
		d_ocp_qcqp_ipm_get("stat_m", ws, &stat_m);
		d_ocp_qcqp_ipm_get("stat", ws, &stat);
		plhs[0] = mxCreateNumericMatrix(iter+1, stat_m+1, mxDOUBLE_CLASS, mxREAL);
		double *mat_ptr = mxGetPr( plhs[0] );
		for(ii=0; ii<iter+1; ii++)
			{
			mat_ptr[ii+0] = ii;
			for(jj=0; jj<stat_m; jj++)
				{
				mat_ptr[ii+(jj+1)*(iter+1)] = stat[jj+ii*stat_m];
				}
			}
		}
	else
		{
		mexPrintf("\nocp_qcqp_solver_get: field not supported: %s\n", field);
		return;
		}

	return;

	}


