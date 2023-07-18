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
#include "hpipm_d_dense_qp_dim.h"
#include "hpipm_d_dense_qp_ipm.h"
// mex
#include "mex.h"



void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
	{

//	printf("\nin dense_solver_arg_create\n");

	mxArray *tmp_mat;
	long long *l_ptr;
	char *c_ptr;

	/* RHS */

	// dim
	l_ptr = mxGetData( prhs[0] );
	struct d_dense_qp_dim *dim = (struct d_dense_qp_dim *) *l_ptr;

	// mode
	char *str_mode = mxArrayToString( prhs[1] );

	int mode;
	if(!strcmp(str_mode, "speed_abs"))
		{
		mode = SPEED_ABS;
		}
	else if(!strcmp(str_mode, "speed"))
		{
		mode = SPEED;
		}
	else if(!strcmp(str_mode, "balance"))
		{
		mode = BALANCE;
		}
	else if(!strcmp(str_mode, "robust"))
		{
		mode = ROBUST;
		}
	else
		{
		mode = SPEED;
		mexPrintf("\ndense_qp_solver_arg_create: mode not supported: %s; speed mode used instead\n", str_mode);
		}

	/* body */

	hpipm_size_t arg_size = sizeof(struct d_dense_qp_ipm_arg) + d_dense_qp_ipm_arg_memsize(dim);
	void *arg_mem = malloc(arg_size);

	c_ptr = arg_mem;

	struct d_dense_qp_ipm_arg *arg = (struct d_dense_qp_ipm_arg *) c_ptr;
	c_ptr += sizeof(struct d_dense_qp_ipm_arg);

	d_dense_qp_ipm_arg_create(dim, arg, c_ptr);
	c_ptr += d_dense_qp_ipm_arg_memsize(dim);

	d_dense_qp_ipm_arg_set_default(mode, arg);

	/* LHS */

	tmp_mat = mxCreateNumericMatrix(1, 1, mxINT64_CLASS, mxREAL);
	l_ptr = mxGetData(tmp_mat);
	l_ptr[0] = (long long) arg_mem;
	plhs[0] = tmp_mat;

	return;

	}




