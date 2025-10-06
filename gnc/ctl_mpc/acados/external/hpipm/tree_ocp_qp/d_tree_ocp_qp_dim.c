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

#include <hpipm_tree.h>
#include <hpipm_scenario_tree.h>
#include <hpipm_d_tree_ocp_qp_dim.h>
#include <hpipm_aux_string.h>
#include <hpipm_aux_mem.h>



#define TREE_OCP_QP_DIM d_tree_ocp_qp_dim



#define TREE_OCP_QP_DIM_MEMSIZE d_tree_ocp_qp_dim_memsize
#define TREE_OCP_QP_DIM_CREATE d_tree_ocp_qp_dim_create
#define TREE_OCP_QP_DIM_SET_ALL d_tree_ocp_qp_dim_set_all
#define TREE_OCP_QP_DIM_SET_TREE d_tree_ocp_qp_dim_set_tree
#define TREE_OCP_QP_DIM_SET d_tree_ocp_qp_dim_set
#define TREE_OCP_QP_DIM_SET_NX d_tree_ocp_qp_dim_set_nx
#define TREE_OCP_QP_DIM_SET_NU d_tree_ocp_qp_dim_set_nu
#define TREE_OCP_QP_DIM_SET_NBX d_tree_ocp_qp_dim_set_nbx
#define TREE_OCP_QP_DIM_SET_NBU d_tree_ocp_qp_dim_set_nbu
#define TREE_OCP_QP_DIM_SET_NG d_tree_ocp_qp_dim_set_ng
#define TREE_OCP_QP_DIM_SET_NS d_tree_ocp_qp_dim_set_ns
#define TREE_OCP_QP_DIM_SET_NSBX d_tree_ocp_qp_dim_set_nsbx
#define TREE_OCP_QP_DIM_SET_NSBU d_tree_ocp_qp_dim_set_nsbu
#define TREE_OCP_QP_DIM_SET_NSG d_tree_ocp_qp_dim_set_nsg
#define TREE_OCP_QP_DIM_SET_NBXE d_tree_ocp_qp_dim_set_nbxe
#define TREE_OCP_QP_DIM_SET_NBUE d_tree_ocp_qp_dim_set_nbue
#define TREE_OCP_TREE_QP_DIM_SET_NGE d_tree_ocp_qp_dim_set_nge


#include "x_tree_ocp_qp_dim.c"
