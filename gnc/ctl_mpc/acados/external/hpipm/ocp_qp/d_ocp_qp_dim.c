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

#include <hpipm_d_ocp_qp_dim.h>
#include <hpipm_aux_string.h>
#include <hpipm_aux_mem.h>



#define OCP_QP_DIM d_ocp_qp_dim



#define OCP_QP_DIM_STRSIZE d_ocp_qp_dim_strsize
#define OCP_QP_DIM_MEMSIZE d_ocp_qp_dim_memsize
#define OCP_QP_DIM_CREATE d_ocp_qp_dim_create
#define OCP_QP_DIM_COPY_ALL d_ocp_qp_dim_copy_all
#define OCP_QP_DIM_SET_ALL d_ocp_qp_dim_set_all
#define OCP_QP_DIM_SET d_ocp_qp_dim_set
#define OCP_QP_DIM_SET_NX d_ocp_qp_dim_set_nx
#define OCP_QP_DIM_SET_NU d_ocp_qp_dim_set_nu
#define OCP_QP_DIM_SET_NBX d_ocp_qp_dim_set_nbx
#define OCP_QP_DIM_SET_NBU d_ocp_qp_dim_set_nbu
#define OCP_QP_DIM_SET_NG d_ocp_qp_dim_set_ng
#define OCP_QP_DIM_SET_NS d_ocp_qp_dim_set_ns
#define OCP_QP_DIM_SET_NSBX d_ocp_qp_dim_set_nsbx
#define OCP_QP_DIM_SET_NSBU d_ocp_qp_dim_set_nsbu
#define OCP_QP_DIM_SET_NSG d_ocp_qp_dim_set_nsg
#define OCP_QP_DIM_SET_NBXE d_ocp_qp_dim_set_nbxe
#define OCP_QP_DIM_SET_NBUE d_ocp_qp_dim_set_nbue
#define OCP_QP_DIM_SET_NGE d_ocp_qp_dim_set_nge
#define OCP_QP_DIM_GET d_ocp_qp_dim_get
#define OCP_QP_DIM_GET_N d_ocp_qp_dim_get_N
#define OCP_QP_DIM_GET_NX d_ocp_qp_dim_get_nx
#define OCP_QP_DIM_GET_NU d_ocp_qp_dim_get_nu
#define OCP_QP_DIM_GET_NBX d_ocp_qp_dim_get_nbx
#define OCP_QP_DIM_GET_NBU d_ocp_qp_dim_get_nbu
#define OCP_QP_DIM_GET_NG d_ocp_qp_dim_get_ng
#define OCP_QP_DIM_GET_NS d_ocp_qp_dim_get_ns
#define OCP_QP_DIM_GET_NSBX d_ocp_qp_dim_get_nsbx
#define OCP_QP_DIM_GET_NSBU d_ocp_qp_dim_get_nsbu
#define OCP_QP_DIM_GET_NSG d_ocp_qp_dim_get_nsg
#define OCP_QP_DIM_GET_NBXE d_ocp_qp_dim_get_nbxe
#define OCP_QP_DIM_GET_NBUE d_ocp_qp_dim_get_nbue
#define OCP_QP_DIM_GET_NGE d_ocp_qp_dim_get_nge


#include "x_ocp_qp_dim.c"
