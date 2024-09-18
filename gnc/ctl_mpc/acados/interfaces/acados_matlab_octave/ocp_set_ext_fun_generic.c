/*
 * Copyright 2019 Gianluca Frison, Dimitris Kouzoupis, Robin Verschueren,
 * Andrea Zanelli, Niels van Duijkeren, Jonathan Frey, Tommaso Sartor,
 * Branimir Novoselnik, Rien Quirynen, Rezart Qelibari, Dang Doan,
 * Jonas Koenemann, Yutao Chen, Tobias Schöls, Jonas Schlagenhauf, Moritz Diehl
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */

// system
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
// acados
#include "acados_c/ocp_nlp_interface.h"
#include "acados/utils/external_function_generic.h"
#include "acados_c/external_function_interface.h"
// mex
#include "mex.h"
#include "mex_macros.h"



// external functions for the model
int FUN_NAME(void **, void **, void *);



void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{

    int ii, status;
    long long *ptr;
//    mexPrintf("\nin ocp_expl_ext_fun_create\n");

    /* RHS */
    const mxArray *C_ocp = prhs[0];
    const mxArray *matlab_model = prhs[2];

    // config
    ptr = (long long *) mxGetData( mxGetField( C_ocp, 0, "config" ) );
    ocp_nlp_config *config = (ocp_nlp_config *) ptr[0];
    // dims
    ptr = (long long *) mxGetData( mxGetField( C_ocp, 0, "dims" ) );
    ocp_nlp_dims *dims = (ocp_nlp_dims *) ptr[0];
    // in
    ptr = (long long *) mxGetData( mxGetField( C_ocp, 0, "in" ) );
    ocp_nlp_in *in = (ocp_nlp_in *) ptr[0];

    // model
    int np = 0;
    if (mxGetField( matlab_model, 0, "dim_np" )!=NULL)
    {
        np = mxGetScalar( mxGetField( matlab_model, 0, "dim_np" ) );
    }

    /* LHS */
    /* copy existing fields */
    plhs[0] = mxDuplicateArray(prhs[1]);

    /* populate new fields */
    external_function_param_generic *ext_fun_ptr;

    ext_fun_ptr = (external_function_param_generic *) malloc((N1-N0+1)*sizeof(external_function_param_generic));
    // NOTE: N0, N1, PHASE, SET_FIELD are given as compiler pre-processing macros
    for (ii=0; ii<N1-N0+1; ii++)
    {
        external_function_param_generic_set_fun(ext_fun_ptr+ii, &FUN_NAME);
        external_function_param_generic_create(ext_fun_ptr+ii, np);
        status = SETTER(config, dims, in, N0+ii, STR(SET_FIELD), ext_fun_ptr+ii);
    }
    // populate output struct
    ptr = mxGetData(mxGetField(plhs[0], 0, STR(MEX_FIELD)));
    ptr[PHASE] = (long long) ext_fun_ptr;
    
    return;

}



