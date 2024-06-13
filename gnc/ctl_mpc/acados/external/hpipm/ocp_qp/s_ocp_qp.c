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



#include <stdio.h>
#include <stdlib.h>

#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_s_aux.h>

#include <hpipm_s_ocp_qp_dim.h>
#include <hpipm_s_ocp_qp.h>
#include <hpipm_aux_string.h>
#include <hpipm_aux_mem.h>



#define BLASFEO_VECEL BLASFEO_SVECEL



#define CREATE_STRMAT blasfeo_create_smat
#define CREATE_STRVEC blasfeo_create_svec
#define PACK_MAT blasfeo_pack_smat
#define CVT_STRMAT2MAT blasfeo_unpack_smat
#define PACK_TRAN_MAT blasfeo_pack_tran_smat
#define CVT_TRAN_STRMAT2MAT blasfeo_unpack_tran_smat
#define PACK_VEC blasfeo_pack_svec
#define UNPACK_VEC blasfeo_unpack_svec
#define GECP blasfeo_sgecp
#define GESE blasfeo_sgese
#define OCP_QP s_ocp_qp
#define OCP_QP_DIM s_ocp_qp_dim
#define REAL float
#define STRMAT blasfeo_smat
#define STRVEC blasfeo_svec
#define SIZE_STRMAT blasfeo_memsize_smat
#define SIZE_STRVEC blasfeo_memsize_svec
#define VECCP blasfeo_sveccp
#define VECSC blasfeo_svecsc
#define VECSE blasfeo_svecse

#define OCP_QP_STRSIZE s_ocp_qp_strsize
#define OCP_QP_MEMSIZE s_ocp_qp_memsize
#define OCP_QP_CREATE s_ocp_qp_create
#define OCP_QP_COPY_ALL s_ocp_qp_copy_all
#define OCP_QP_SET_ALL_ZERO s_ocp_qp_set_all_zero
#define OCP_QP_SET_RHS_ZERO s_ocp_qp_set_rhs_zero
#define OCP_QP_SET_ALL s_ocp_qp_set_all
#define OCP_QP_SET_ALL_ROWMAJ s_ocp_qp_set_all_rowmaj
#define OCP_QP_SET s_ocp_qp_set
#define OCP_QP_SET_EL s_ocp_qp_set_el
#define OCP_QP_SET_A s_ocp_qp_set_A
#define OCP_QP_SET_B s_ocp_qp_set_B
#define OCP_QP_SET_BVEC s_ocp_qp_set_b
#define OCP_QP_SET_Q s_ocp_qp_set_Q
#define OCP_QP_SET_S s_ocp_qp_set_S
#define OCP_QP_SET_R s_ocp_qp_set_R
#define OCP_QP_SET_QVEC s_ocp_qp_set_q
#define OCP_QP_SET_RVEC s_ocp_qp_set_r
#define OCP_QP_SET_LB s_ocp_qp_set_lb
#define OCP_QP_SET_LB_MASK s_ocp_qp_set_lb_mask
#define OCP_QP_SET_UB s_ocp_qp_set_ub
#define OCP_QP_SET_UB_MASK s_ocp_qp_set_ub_mask
#define OCP_QP_SET_LBX s_ocp_qp_set_lbx
#define OCP_QP_SET_LBX_MASK s_ocp_qp_set_lbx_mask
#define OCP_QP_SET_EL_LBX s_ocp_qp_set_el_lbx
#define OCP_QP_SET_UBX s_ocp_qp_set_ubx
#define OCP_QP_SET_UBX_MASK s_ocp_qp_set_ubx_mask
#define OCP_QP_SET_EL_UBX s_ocp_qp_set_el_ubx
#define OCP_QP_SET_LBU s_ocp_qp_set_lbu
#define OCP_QP_SET_LBU_MASK s_ocp_qp_set_lbu_mask
#define OCP_QP_SET_UBU s_ocp_qp_set_ubu
#define OCP_QP_SET_UBU_MASK s_ocp_qp_set_ubu_mask
#define OCP_QP_SET_IDXB s_ocp_qp_set_idxb
#define OCP_QP_SET_IDXBX s_ocp_qp_set_idxbx
#define OCP_QP_SET_JBX s_ocp_qp_set_Jbx
#define OCP_QP_SET_IDXBU s_ocp_qp_set_idxbu
#define OCP_QP_SET_JBU s_ocp_qp_set_Jbu
#define OCP_QP_SET_C s_ocp_qp_set_C
#define OCP_QP_SET_D s_ocp_qp_set_D
#define OCP_QP_SET_LG s_ocp_qp_set_lg
#define OCP_QP_SET_LG_MASK s_ocp_qp_set_lg_mask
#define OCP_QP_SET_UG s_ocp_qp_set_ug
#define OCP_QP_SET_UG_MASK s_ocp_qp_set_ug_mask
#define OCP_QP_SET_ZL s_ocp_qp_set_Zl
#define OCP_QP_SET_ZU s_ocp_qp_set_Zu
#define OCP_QP_SET_ZLVEC s_ocp_qp_set_zl
#define OCP_QP_SET_ZUVEC s_ocp_qp_set_zu
#define OCP_QP_SET_IDXS s_ocp_qp_set_idxs
#define OCP_QP_SET_IDXS_REV s_ocp_qp_set_idxs_rev
#define OCP_QP_SET_JSBU s_ocp_qp_set_Jsbu
#define OCP_QP_SET_JSBX s_ocp_qp_set_Jsbx
#define OCP_QP_SET_JSG s_ocp_qp_set_Jsg
#define OCP_QP_SET_LLS s_ocp_qp_set_lls
#define OCP_QP_SET_LLS_MASK s_ocp_qp_set_lls_mask
#define OCP_QP_SET_LUS s_ocp_qp_set_lus
#define OCP_QP_SET_LUS_MASK s_ocp_qp_set_lus_mask
#define OCP_QP_SET_IDXE s_ocp_qp_set_idxe
#define OCP_QP_SET_IDXBXE s_ocp_qp_set_idxbxe
#define OCP_QP_SET_IDXBUE s_ocp_qp_set_idxbue
#define OCP_QP_SET_IDXGE s_ocp_qp_set_idxge
#define OCP_QP_SET_JBXE s_ocp_qp_set_Jbxe
#define OCP_QP_SET_JBUE s_ocp_qp_set_Jbue
#define OCP_QP_SET_JGE s_ocp_qp_set_Jge
#define OCP_QP_SET_DIAG_H_FLAG s_ocp_qp_set_diag_H_flag
// getters
#define OCP_QP_GET s_ocp_qp_get
#define OCP_QP_GET_A s_ocp_qp_get_A
#define OCP_QP_GET_B s_ocp_qp_get_B
#define OCP_QP_GET_BVEC s_ocp_qp_get_b
#define OCP_QP_GET_Q s_ocp_qp_get_Q
#define OCP_QP_GET_S s_ocp_qp_get_S
#define OCP_QP_GET_R s_ocp_qp_get_R
#define OCP_QP_GET_QVEC s_ocp_qp_get_q
#define OCP_QP_GET_RVEC s_ocp_qp_get_r
#define OCP_QP_GET_LBX s_ocp_qp_get_lbx
#define OCP_QP_GET_LBX_MASK s_ocp_qp_get_lbx_mask
#define OCP_QP_GET_UBX s_ocp_qp_get_ubx
#define OCP_QP_GET_UBX_MASK s_ocp_qp_get_ubx_mask
#define OCP_QP_GET_LBU s_ocp_qp_get_lbu
#define OCP_QP_GET_LBU_MASK s_ocp_qp_get_lbu_mask
#define OCP_QP_GET_UBU s_ocp_qp_get_ubu
#define OCP_QP_GET_UBU_MASK s_ocp_qp_get_ubu_mask
#define OCP_QP_GET_IDXB s_ocp_qp_get_idxb
#define OCP_QP_GET_IDXBX s_ocp_qp_get_idxbx
#define OCP_QP_GET_JBX s_ocp_qp_get_Jbx
#define OCP_QP_GET_IDXBU s_ocp_qp_get_idxbu
#define OCP_QP_GET_JBU s_ocp_qp_get_Jbu
#define OCP_QP_GET_LB s_ocp_qp_get_lb
#define OCP_QP_GET_LB_MASK s_ocp_qp_get_lb_mask
#define OCP_QP_GET_UB s_ocp_qp_get_ub
#define OCP_QP_GET_UB_MASK s_ocp_qp_get_ub_mask
#define OCP_QP_GET_C s_ocp_qp_get_C
#define OCP_QP_GET_D s_ocp_qp_get_D
#define OCP_QP_GET_LG s_ocp_qp_get_lg
#define OCP_QP_GET_LG_MASK s_ocp_qp_get_lg_mask
#define OCP_QP_GET_UG s_ocp_qp_get_ug
#define OCP_QP_GET_UG_MASK s_ocp_qp_get_ug_mask
#define OCP_QP_GET_ZL s_ocp_qp_get_Zl
#define OCP_QP_GET_ZU s_ocp_qp_get_Zu
#define OCP_QP_GET_ZLVEC s_ocp_qp_get_zl
#define OCP_QP_GET_ZUVEC s_ocp_qp_get_zu
#define OCP_QP_GET_IDXS s_ocp_qp_get_idxs
#define OCP_QP_GET_IDXS_REV s_ocp_qp_get_idxs_rev
#define OCP_QP_GET_JSBU s_ocp_qp_get_Jsbu
#define OCP_QP_GET_JSBX s_ocp_qp_get_Jsbx
#define OCP_QP_GET_JSG s_ocp_qp_get_Jsg
#define OCP_QP_GET_LLS s_ocp_qp_get_lls
#define OCP_QP_GET_LLS_MASK s_ocp_qp_get_lls_mask
#define OCP_QP_GET_LUS s_ocp_qp_get_lus
#define OCP_QP_GET_LUS_MASK s_ocp_qp_get_lus_mask

#include "x_ocp_qp.c"

