/**************************************************************************************************
*                                                                                                 *
* This file is part of HPIPM.                                                                     *
*                                                                                                 *
* HPIPM -- High-Performance Interior Point Method.                                                *
* Copyright (C) 2017-2018 by Gianluca Frison.                                                     *
* Developed at IMTEK (University of Freiburg) under the supervision of Moritz Diehl.              *
* All rights reserved.                                                                            *
*                                                                                                 *
* This program is free software: you can redistribute it and/or modify                            *
* it under the terms of the GNU General Public License as published by                            *
* the Free Software Foundation, either version 3 of the License, or                               *
* (at your option) any later version                                                              *.
*                                                                                                 *
* This program is distributed in the hope that it will be useful,                                 *
* but WITHOUT ANY WARRANTY; without even the implied warranty of                                  *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                                   *
* GNU General Public License for more details.                                                    *
*                                                                                                 *
* You should have received a copy of the GNU General Public License                               *
* along with this program.  If not, see <https://www.gnu.org/licenses/>.                          *
*                                                                                                 *
* The authors designate this particular file as subject to the "Classpath" exception              *
* as provided by the authors in the LICENSE file that accompained this code.                      *
*                                                                                                 *
* Author: Gianluca Frison, gianluca.frison (at) imtek.uni-freiburg.de                             *
*                                                                                                 *
**************************************************************************************************/



#if defined(RUNTIME_CHECKS)
#include <stdlib.h>
#include <stdio.h>
#endif

#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_s_aux.h>

#include "../include/hpipm_s_ocp_nlph"
#include "../include/hpipm_s_ocp_nlpsol.h"



#define CREATE_STRVEC blasfeo_create_svec
#define CVT_STRVEC2VEC blasfeo_unpack_svec
#define OCP_NLP s_ocp_nlp
#define OCP_NLP_SOL s_ocp_nlpsol
#define REAL float
#define STRVEC blasfeo_svec
#define SIZE_STRVEC blasfeo_memsize_svec
#define VECCP_LIBSTR blasfeo_sveccp

#define CREATE_OCP_NLP_SOL s_create_ocp_nlpsol
#define MEMSIZE_OCP_NLP_SOL s_memsize_ocp_nlpsol
#define CVT_OCP_NLP_SOL_TO_COLMAJ s_cvt_ocp_nlpsol_to_colmaj
#define CVT_OCP_NLP_SOL_TO_ROWMAJ s_cvt_ocp_nlpsol_to_rowmaj
#define CVT_OCP_NLP_SOL_TO_LIBSTR s_cvt_ocp_nlpsol_to_libstr



#include "x_ocp_nlpsol.c"

