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
#include "hpipm_d_ocp_qp_sol.h"
// mex
#include "mex.h"



void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
	{

//	mexPrintf("\nin ocp_qp_sol_get\n");

	long long *l_ptr;

	int ii;

	/* RHS */

	// sol
	l_ptr = mxGetData( prhs[0] );
	struct d_ocp_qp_sol *sol = (struct d_ocp_qp_sol *) *l_ptr;

	// dim
	struct d_ocp_qp_dim *dim = sol->dim;
	int N = dim->N;
	int *nx = dim->nx;
	int *nu = dim->nu;
	int *nbu = dim->nbu;
	int *nbx = dim->nbx;
	int *ng = dim->ng;
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

	if(!strcmp(field, "x"))
		{
		if(nrhs==4)
			{
			int nx_sum = 0;
			for(ii=stage0; ii<=stage1; ii++)
				{
				nx_sum += nx[ii];
				}
			plhs[0] = mxCreateNumericMatrix(nx_sum, 1, mxDOUBLE_CLASS, mxREAL);
			double *x = mxGetPr( plhs[0] );
			nx_sum = 0;
			for(ii=stage0; ii<=stage1; ii++)
				{
				d_ocp_qp_sol_get(field, ii, sol, x+nx_sum);
				nx_sum += nx[ii];
				}
			}
		else
			{
			plhs[0] = mxCreateNumericMatrix(nx[stage0], 1, mxDOUBLE_CLASS, mxREAL);
			double *x = mxGetPr( plhs[0] );
			d_ocp_qp_sol_get(field, stage0, sol, x);
			}
		}
	else if(!strcmp(field, "u"))
		{
		if(nrhs==4)
			{
			int nu_sum = 0;
			for(ii=stage0; ii<=stage1; ii++)
				{
				nu_sum += nu[ii];
				}
			plhs[0] = mxCreateNumericMatrix(nu_sum, 1, mxDOUBLE_CLASS, mxREAL);
			double *u = mxGetPr( plhs[0] );
			nu_sum = 0;
			for(ii=stage0; ii<=stage1; ii++)
				{
				d_ocp_qp_sol_get(field, ii, sol, u+nu_sum);
				nu_sum += nu[ii];
				}
			}
		else
			{
			plhs[0] = mxCreateNumericMatrix(nu[stage0], 1, mxDOUBLE_CLASS, mxREAL);
			double *u = mxGetPr( plhs[0] );
			d_ocp_qp_sol_get(field, stage0, sol, u);
			}
		}
	else if(!strcmp(field, "pi"))
		{
		if(nrhs==4)
			{
			stage1 = stage1<=N-1 ? stage1 : N-1; // no last stage
			int nx_sum = 0;
			for(ii=stage0; ii<=stage1; ii++)
				{
				nx_sum += nx[ii+1];
				}
			plhs[0] = mxCreateNumericMatrix(nx_sum, 1, mxDOUBLE_CLASS, mxREAL);
			double *pi = mxGetPr( plhs[0] );
			nx_sum = 0;
			for(ii=stage0; ii<=stage1; ii++)
				{
				d_ocp_qp_sol_get(field, ii, sol, pi+nx_sum);
				nx_sum += nx[ii+1];
				}
			}
		else
			{
			plhs[0] = mxCreateNumericMatrix(nx[stage0+1], 1, mxDOUBLE_CLASS, mxREAL);
			double *pi = mxGetPr( plhs[0] );
			d_ocp_qp_sol_get(field, stage0, sol, pi);
			}
		}
	else if( (!strcmp(field, "sl")) | (!strcmp(field, "su")) )
		{
		if(nrhs==4)
			{
			int ns_sum = 0;
			for(ii=stage0; ii<=stage1; ii++)
				{
				ns_sum += ns[ii];
				}
			plhs[0] = mxCreateNumericMatrix(ns_sum, 1, mxDOUBLE_CLASS, mxREAL);
			double *s = mxGetPr( plhs[0] );
			ns_sum = 0;
			for(ii=stage0; ii<=stage1; ii++)
				{
				d_ocp_qp_sol_get(field, ii, sol, s+ns_sum);
				ns_sum += ns[ii];
				}
			}
		else
			{
			plhs[0] = mxCreateNumericMatrix(ns[stage0], 1, mxDOUBLE_CLASS, mxREAL);
			double *s = mxGetPr( plhs[0] );
			d_ocp_qp_sol_get(field, stage0, sol, s);
			}
		}
	else if( (!strcmp(field, "lam_lbu")) | (!strcmp(field, "lam_ubu")) )
		{
		if(nrhs==4)
			{
			int nbu_sum = 0;
			for(ii=stage0; ii<=stage1; ii++)
				{
				nbu_sum += nbu[ii];
				}
			plhs[0] = mxCreateNumericMatrix(nbu_sum, 1, mxDOUBLE_CLASS, mxREAL);
			double *lam = mxGetPr( plhs[0] );
			nbu_sum = 0;
			for(ii=stage0; ii<=stage1; ii++)
				{
				d_ocp_qp_sol_get(field, ii, sol, lam+nbu_sum);
				nbu_sum += nbu[ii];
				}
			}
		else
			{
			plhs[0] = mxCreateNumericMatrix(nbu[stage0], 1, mxDOUBLE_CLASS, mxREAL);
			double *lam = mxGetPr( plhs[0] );
			d_ocp_qp_sol_get(field, stage0, sol, lam);
			}
		}
	else if( (!strcmp(field, "lam_lbx")) | (!strcmp(field, "lam_ubx")) )
		{
		if(nrhs==4)
			{
			int nbx_sum = 0;
			for(ii=stage0; ii<=stage1; ii++)
				{
				nbx_sum += nbx[ii];
				}
			plhs[0] = mxCreateNumericMatrix(nbx_sum, 1, mxDOUBLE_CLASS, mxREAL);
			double *lam = mxGetPr( plhs[0] );
			nbx_sum = 0;
			for(ii=stage0; ii<=stage1; ii++)
				{
				d_ocp_qp_sol_get(field, ii, sol, lam+nbx_sum);
				nbx_sum += nbx[ii];
				}
			}
		else
			{
			plhs[0] = mxCreateNumericMatrix(nbx[stage0], 1, mxDOUBLE_CLASS, mxREAL);
			double *lam = mxGetPr( plhs[0] );
			d_ocp_qp_sol_get(field, stage0, sol, lam);
			}
		}
	else if( (!strcmp(field, "lam_lg")) | (!strcmp(field, "lam_ug")) )
		{
		if(nrhs==4)
			{
			int ng_sum = 0;
			for(ii=stage0; ii<=stage1; ii++)
				{
				ng_sum += ng[ii];
				}
			plhs[0] = mxCreateNumericMatrix(ng_sum, 1, mxDOUBLE_CLASS, mxREAL);
			double *lam = mxGetPr( plhs[0] );
			ng_sum = 0;
			for(ii=stage0; ii<=stage1; ii++)
				{
				d_ocp_qp_sol_get(field, ii, sol, lam+ng_sum);
				ng_sum += ng[ii];
				}
			}
		else
			{
			plhs[0] = mxCreateNumericMatrix(ng[stage0], 1, mxDOUBLE_CLASS, mxREAL);
			double *lam = mxGetPr( plhs[0] );
			d_ocp_qp_sol_get(field, stage0, sol, lam);
			}
		}
	else
		{
		mexPrintf("\nocp_qp_sol_get: field not supported: %s\n", field);
		return;
		}

	return;

	}




