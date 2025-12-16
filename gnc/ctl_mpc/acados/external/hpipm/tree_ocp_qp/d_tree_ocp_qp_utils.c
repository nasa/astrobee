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
#include <blasfeo_d_aux.h>
#include <blasfeo_d_aux_ext_dep.h>
#include <blasfeo_i_aux_ext_dep.h>

#include <hpipm_tree.h>
#include <hpipm_d_tree_ocp_qp_dim.h>
#include <hpipm_d_tree_ocp_qp.h>
#include <hpipm_d_tree_ocp_qp_sol.h>
#include "hpipm_d_tree_ocp_qp_ipm.h"



#define DOUBLE_PRECISION



#define BLASFEO_PRINT_MAT blasfeo_print_dmat
#define BLASFEO_PRINT_TRAN_VEC blasfeo_print_tran_dvec
#define TREE_OCP_QP d_tree_ocp_qp
#define TREE_OCP_QP_DIM d_tree_ocp_qp_dim
#define TREE_OCP_QP_IPM_ARG d_tree_ocp_qp_ipm_arg
#define TREE_OCP_QP_RES d_tree_ocp_qp_res
#define TREE_OCP_QP_SOL d_tree_ocp_qp_sol



#define TREE_OCP_QP_DIM_PRINT d_tree_ocp_qp_dim_print
#define TREE_OCP_QP_DIM_CODEGEN d_tree_ocp_qp_dim_codegen
#define TREE_OCP_QP_PRINT d_tree_ocp_qp_print
#define TREE_OCP_QP_CODEGEN d_tree_ocp_qp_codegen
#define TREE_OCP_QP_SOL_PRINT d_tree_ocp_qp_sol_print
#define TREE_OCP_QP_IPM_ARG_PRINT d_tree_ocp_qp_ipm_arg_print
#define TREE_OCP_QP_IPM_ARG_CODEGEN d_tree_ocp_qp_ipm_arg_codegen
#define TREE_OCP_QP_RES_PRINT d_tree_ocp_qp_res_print



#include "x_tree_ocp_qp_utils.c"

