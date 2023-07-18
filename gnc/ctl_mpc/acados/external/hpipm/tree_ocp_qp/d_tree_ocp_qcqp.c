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

#include <hpipm_tree.h>
#include <hpipm_scenario_tree.h>
#include <hpipm_d_tree_ocp_qcqp_dim.h>
#include <hpipm_d_tree_ocp_qcqp.h>
#include <hpipm_aux_string.h>
#include <hpipm_aux_mem.h>



#define BLASFEO_VECEL BLASFEO_DVECEL



#define CREATE_STRMAT blasfeo_create_dmat
#define CREATE_STRVEC blasfeo_create_dvec
#define PACK_MAT blasfeo_pack_dmat
#define PACK_TRAN_MAT blasfeo_pack_tran_dmat
#define PACK_VEC blasfeo_pack_dvec
#define REAL double
#define SIZE_STRMAT blasfeo_memsize_dmat
#define SIZE_STRVEC blasfeo_memsize_dvec
#define STRMAT blasfeo_dmat
#define STRVEC blasfeo_dvec
#define TREE_OCP_QCQP d_tree_ocp_qcqp
#define TREE_OCP_QCQP_DIM d_tree_ocp_qcqp_dim
#define VECSC blasfeo_dvecsc
#define VECSE blasfeo_dvecse

#define TREE_OCP_QCQP_STRSIZE d_tree_ocp_qcqp_strsize
#define TREE_OCP_QCQP_MEMSIZE d_tree_ocp_qcqp_memsize
#define TREE_OCP_QCQP_CREATE d_tree_ocp_qcqp_create
#define TREE_OCP_QCQP_SET_ALL d_tree_ocp_qcqp_set_all
#define TREE_OCP_QCQP_SET d_tree_ocp_qcqp_set
#define TREE_OCP_QCQP_SET_A d_tree_ocp_qcqp_set_A
#define TREE_OCP_QCQP_SET_B d_tree_ocp_qcqp_set_B
#define TREE_OCP_QCQP_SET_BVEC d_tree_ocp_qcqp_set_b
#define TREE_OCP_QCQP_SET_Q d_tree_ocp_qcqp_set_Q
#define TREE_OCP_QCQP_SET_S d_tree_ocp_qcqp_set_S
#define TREE_OCP_QCQP_SET_R d_tree_ocp_qcqp_set_R
#define TREE_OCP_QCQP_SET_QVEC d_tree_ocp_qcqp_set_q
#define TREE_OCP_QCQP_SET_RVEC d_tree_ocp_qcqp_set_r
#define TREE_OCP_QCQP_SET_LB d_tree_ocp_qcqp_set_lb
#define TREE_OCP_QCQP_SET_LB_MASK d_tree_ocp_qcqp_set_lb_mask
#define TREE_OCP_QCQP_SET_UB d_tree_ocp_qcqp_set_ub
#define TREE_OCP_QCQP_SET_UB_MASK d_tree_ocp_qcqp_set_ub_mask
#define TREE_OCP_QCQP_SET_LBX d_tree_ocp_qcqp_set_lbx
#define TREE_OCP_QCQP_SET_LBX_MASK d_tree_ocp_qcqp_set_lbx_mask
#define TREE_OCP_QCQP_SET_UBX d_tree_ocp_qcqp_set_ubx
#define TREE_OCP_QCQP_SET_UBX_MASK d_tree_ocp_qcqp_set_ubx_mask
#define TREE_OCP_QCQP_SET_LBU d_tree_ocp_qcqp_set_lbu
#define TREE_OCP_QCQP_SET_LBU_MASK d_tree_ocp_qcqp_set_lbu_mask
#define TREE_OCP_QCQP_SET_UBU d_tree_ocp_qcqp_set_ubu
#define TREE_OCP_QCQP_SET_UBU_MASK d_tree_ocp_qcqp_set_ubu_mask
#define TREE_OCP_QCQP_SET_IDXB d_tree_ocp_qcqp_set_idxb
#define TREE_OCP_QCQP_SET_IDXBX d_tree_ocp_qcqp_set_idxbx
#define TREE_OCP_QCQP_SET_JBX d_tree_ocp_qcqp_set_Jbx
#define TREE_OCP_QCQP_SET_IDXBU d_tree_ocp_qcqp_set_idxbu
#define TREE_OCP_QCQP_SET_JBU d_tree_ocp_qcqp_set_Jbu
#define TREE_OCP_QCQP_SET_C d_tree_ocp_qcqp_set_C
#define TREE_OCP_QCQP_SET_D d_tree_ocp_qcqp_set_D
#define TREE_OCP_QCQP_SET_LG d_tree_ocp_qcqp_set_lg
#define TREE_OCP_QCQP_SET_LG_MASK d_tree_ocp_qcqp_set_lg_mask
#define TREE_OCP_QCQP_SET_UG d_tree_ocp_qcqp_set_ug
#define TREE_OCP_QCQP_SET_UG_MASK d_tree_ocp_qcqp_set_ug_mask
#define TREE_OCP_QCQP_SET_QQ d_tree_ocp_qcqp_set_Qq
#define TREE_OCP_QCQP_SET_SQ d_tree_ocp_qcqp_set_Sq
#define TREE_OCP_QCQP_SET_RQ d_tree_ocp_qcqp_set_Rq
#define TREE_OCP_QCQP_SET_QQVEC d_tree_ocp_qcqp_set_qq
#define TREE_OCP_QCQP_SET_RQVEC d_tree_ocp_qcqp_set_rq
#define TREE_OCP_QCQP_SET_UQ d_tree_ocp_qcqp_set_uq
#define TREE_OCP_QCQP_SET_UQ_MASK d_tree_ocp_qcqp_set_uq_mask
#define TREE_OCP_QCQP_SET_ZL d_tree_ocp_qcqp_set_Zl
#define TREE_OCP_QCQP_SET_ZU d_tree_ocp_qcqp_set_Zu
#define TREE_OCP_QCQP_SET_ZLVEC d_tree_ocp_qcqp_set_zl
#define TREE_OCP_QCQP_SET_ZUVEC d_tree_ocp_qcqp_set_zu
#define TREE_OCP_QCQP_SET_IDXS d_tree_ocp_qcqp_set_idxs
#define TREE_OCP_QCQP_SET_IDXS_REV d_tree_ocp_qcqp_set_idxs_rev
#define TREE_OCP_QCQP_SET_JSBU d_tree_ocp_qcqp_set_Jsbu
#define TREE_OCP_QCQP_SET_JSBX d_tree_ocp_qcqp_set_Jsbx
#define TREE_OCP_QCQP_SET_JSG d_tree_ocp_qcqp_set_Jsg
#define TREE_OCP_QCQP_SET_JSQ d_tree_ocp_qcqp_set_Jsq
#define TREE_OCP_QCQP_SET_LLS d_tree_ocp_qcqp_set_lls
#define TREE_OCP_QCQP_SET_LLS_MASK d_tree_ocp_qcqp_set_lls_mask
#define TREE_OCP_QCQP_SET_LUS d_tree_ocp_qcqp_set_lus
#define TREE_OCP_QCQP_SET_LUS_MASK d_tree_ocp_qcqp_set_lus_mask
//#define TREE_OCP_QCQP_SET_IDXE d_tree_ocp_qcqp_set_idxe
//#define TREE_OCP_QCQP_SET_IDXBXE d_tree_ocp_qcqp_set_idxbxe
//#define TREE_OCP_QCQP_SET_IDXBUE d_tree_ocp_qcqp_set_idxbue
//#define TREE_OCP_QCQP_SET_IDXGE d_tree_ocp_qcqp_set_idxge
//#define TREE_OCP_QCQP_SET_JBXE d_tree_ocp_qcqp_set_Jbxe
//#define TREE_OCP_QCQP_SET_JBUE d_tree_ocp_qcqp_set_Jbue
//#define TREE_OCP_QCQP_SET_JGE d_tree_ocp_qcqp_set_Jge
//#define TREE_OCP_QCQP_SET_DIAG_H_FLAG d_tree_ocp_qcqp_set_diag_H_flag



#include "x_tree_ocp_qcqp.c"

