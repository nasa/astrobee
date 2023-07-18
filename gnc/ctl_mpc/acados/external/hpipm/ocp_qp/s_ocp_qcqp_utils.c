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

#include <stdlib.h>
#include <stdio.h>

#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_s_aux.h>
#include <blasfeo_s_aux_ext_dep.h>
#include <blasfeo_i_aux_ext_dep.h>

#include <hpipm_s_ocp_qcqp_dim.h>
#include <hpipm_s_ocp_qp.h>
#include <hpipm_s_ocp_qcqp_sol.h>
#include "hpipm_s_ocp_qcqp_ipm.h"



#define SINGLE_PRECISION



#define BLASFEO_PRINT_MAT blasfeo_print_smat
#define BLASFEO_PRINT_TRAN_MAT blasfeo_print_tran_smat
#define BLASFEO_PRINT_TRAN_VEC blasfeo_print_tran_svec
#define BLASFEO_PRINT_EXP_TRAN_VEC blasfeo_print_exp_tran_svec
#define OCP_QCQP s_ocp_qcqp
#define OCP_QCQP_DIM s_ocp_qcqp_dim
#define OCP_QCQP_IPM_ARG s_ocp_qcqp_ipm_arg
#define OCP_QCQP_RES s_ocp_qcqp_res
#define OCP_QCQP_SOL s_ocp_qcqp_sol



#define OCP_QCQP_DIM_PRINT s_ocp_qcqp_dim_print
#define OCP_QCQP_DIM_CODEGEN s_ocp_qcqp_dim_codegen
#define OCP_QCQP_PRINT s_ocp_qcqp_print
#define OCP_QCQP_CODEGEN s_ocp_qcqp_codegen
#define OCP_QCQP_SOL_PRINT s_ocp_qcqp_sol_print
#define OCP_QCQP_IPM_ARG_CODEGEN s_ocp_qcqp_ipm_arg_codegen
#define OCP_QCQP_RES_PRINT s_ocp_qcqp_res_print



#include "x_ocp_qcqp_utils.c"


