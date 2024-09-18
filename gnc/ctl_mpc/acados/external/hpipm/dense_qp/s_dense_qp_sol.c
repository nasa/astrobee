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

#include <hpipm_s_dense_qp_dim.h>
#include <hpipm_s_dense_qp.h>
#include <hpipm_s_dense_qp_sol.h>
#include <hpipm_aux_string.h>
#include <hpipm_aux_mem.h>



#define CREATE_STRVEC blasfeo_create_svec
#define UNPACK_VEC blasfeo_unpack_svec
#define DENSE_QP s_dense_qp
#define DENSE_QP_DIM s_dense_qp_dim
#define DENSE_QP_SOL s_dense_qp_sol
#define REAL float
#define STRVEC blasfeo_svec
#define SIZE_STRVEC blasfeo_memsize_svec
#define VECCP_LIBSTR blasfeo_sveccp

#define DENSE_QP_SOL_MEMSIZE s_dense_qp_sol_memsize
#define DENSE_QP_SOL_CREATE s_dense_qp_sol_create
#define DENSE_QP_SOL_GET_ALL s_dense_qp_sol_get_all
#define DENSE_QP_SOL_GET s_dense_qp_sol_get
#define DENSE_QP_SOL_GET_V s_dense_qp_sol_get_v
#define DENSE_QP_SOL_GET_VALID_OBJ s_dense_qp_sol_get_valid_obj
#define DENSE_QP_SOL_GET_OBJ s_dense_qp_sol_get_obj



#include "x_dense_qp_sol.c"


