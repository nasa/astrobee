/**************************************************************************************************
*                                                                                                 *
* This file is part of BLASFEO.                                                                   *
*                                                                                                 *
* BLASFEO -- BLAS For Embedded Optimization.                                                      *
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

#include <blasfeo_block_size.h>
#include <blasfeo_common.h>
#include <blasfeo_stdlib.h>
#include <blasfeo_s_aux_ext_dep.h>
#include <blasfeo_s_aux.h>



#define CREATE_MAT blasfeo_create_smat
#define CREATE_VEC blasfeo_create_svec
#define MEMSIZE_MAT blasfeo_memsize_smat
#define MEMSIZE_VEC blasfeo_memsize_svec
#define REAL float
#define MAT blasfeo_smat
#define MATEL BLASFEO_SMATEL
#define VEC blasfeo_svec
#define VECEL BLASFEO_SVECEL



#define ALLOCATE_MAT blasfeo_allocate_smat
#define ALLOCATE_VEC blasfeo_allocate_svec
#define FREE_MAT blasfeo_free_smat
#define FREE_VEC blasfeo_free_svec
#define PRINT_MAT blasfeo_print_smat
#define PRINT_TRAN_MAT blasfeo_print_tran_smat
#define PRINT_VEC blasfeo_print_svec
#define PRINT_TRAN_VEC blasfeo_print_tran_svec
#define PRINT_TO_FILE_MAT blasfeo_print_to_file_smat
#define PRINT_TO_FILE_EXP_MAT blasfeo_print_to_file_exp_smat
#define PRINT_TO_FILE_VEC blasfeo_print_to_file_svec
#define PRINT_TO_FILE_TRAN_VEC blasfeo_print_to_file_tran_svec
#define PRINT_TO_STRING_MAT blasfeo_print_to_string_smat
#define PRINT_TO_STRING_VEC blasfeo_print_to_string_svec
#define PRINT_TO_STRING_TRAN_VEC blasfeo_print_to_string_tran_svec
#define PRINT_EXP_MAT blasfeo_print_exp_smat
#define PRINT_EXP_TRAN_MAT blasfeo_print_exp_tran_smat
#define PRINT_EXP_VEC blasfeo_print_exp_svec
#define PRINT_EXP_TRAN_VEC blasfeo_print_exp_tran_svec



#include "x_aux_ext_dep.c"

