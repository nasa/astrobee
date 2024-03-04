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
#include <blasfeo_d_blas.h>
#include <blasfeo_d_aux.h>

#include "../include/hpipm_d_ocp_qcqp_dim.h"
#include "../include/hpipm_d_ocp_qcqp.h"
#include "../include/hpipm_d_ocp_qcqp_sol.h"
#include "../include/hpipm_d_dense_qcqp_dim.h"
#include "../include/hpipm_d_dense_qcqp.h"
#include "../include/hpipm_d_dense_qcqp_sol.h"



#define DENSE_QCQP d_dense_qcqp
#define DENSE_QCQP_DIM d_dense_qcqp_dim
#define DENSE_QCQP_DIM_SET_NV d_dense_qcqp_dim_set_nv
#define DENSE_QCQP_DIM_SET_NE d_dense_qcqp_dim_set_ne
#define DENSE_QCQP_DIM_SET_NB d_dense_qcqp_dim_set_nb
#define DENSE_QCQP_DIM_SET_NG d_dense_qcqp_dim_set_ng
#define DENSE_QCQP_DIM_SET_NQ d_dense_qcqp_dim_set_nq
#define DENSE_QCQP_DIM_SET_NS d_dense_qcqp_dim_set_ns
#define DENSE_QCQP_DIM_SET_NSB d_dense_qcqp_dim_set_nsb
#define DENSE_QCQP_DIM_SET_NSG d_dense_qcqp_dim_set_nsg
#define DENSE_QCQP_DIM_SET_NSQ d_dense_qcqp_dim_set_nsq
#define DENSE_QCQP_SOL d_dense_qcqp_sol
#define DENSE_QP d_dense_qp
#define DENSE_QP_DIM d_dense_qp_dim
#define DENSE_QP_SOL d_dense_qp_sol
#define DIARE blasfeo_ddiare
#define GECP blasfeo_dgecp
#define GETR blasfeo_dgetr
#define OCP_QCQP d_ocp_qcqp
#define OCP_QCQP_DIM d_ocp_qcqp_dim
#define OCP_QCQP_SOL d_ocp_qcqp_sol
#define OCP_QP d_ocp_qp
#define OCP_QP_DIM d_ocp_qp_dim
#define OCP_QP_SOL d_ocp_qp_sol
#define SIZE_STRMAT blasfeo_memsize_dmat
#define SIZE_STRVEC blasfeo_memsize_dvec
#define STRMAT blasfeo_dmat
#define STRVEC blasfeo_dvec
#define VECCP blasfeo_dveccp

#define CAST_QCQP_COMPUTE_DIM d_cast_qcqp_compute_dim
#define CAST_QCQP_COND d_cast_qcqp_cond



#include "x_cast_qcqp.c"


