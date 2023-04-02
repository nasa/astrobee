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

#include <hpipm_s_dense_qp_dim.h>
#include <hpipm_aux_string.h>



#define DENSE_QP_DIM s_dense_qp_dim

#define DENSE_QP_DIM_MEMSIZE s_dense_qp_dim_memsize
#define DENSE_QP_DIM_CREATE s_dense_qp_dim_create
#define DENSE_QP_DIM_SET_ALL s_dense_qp_dim_set_all
#define DENSE_QP_DIM_SET s_dense_qp_dim_set
#define DENSE_QP_DIM_SET_NV s_dense_qp_dim_set_nv
#define DENSE_QP_DIM_SET_NE s_dense_qp_dim_set_ne
#define DENSE_QP_DIM_SET_NB s_dense_qp_dim_set_nb
#define DENSE_QP_DIM_SET_NG s_dense_qp_dim_set_ng
#define DENSE_QP_DIM_SET_NSB s_dense_qp_dim_set_nsb
#define DENSE_QP_DIM_SET_NSG s_dense_qp_dim_set_nsg
#define DENSE_QP_DIM_SET_NS s_dense_qp_dim_set_ns


#include "x_dense_qp_dim.c"


