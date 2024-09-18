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
#include <math.h>

#include <blasfeo_common.h>
#include <blasfeo_block_size.h>



#define REF



#if defined(MF_COLMAJ)
	#define XMATEL_A(X, Y) pA[(X)+lda*(Y)]
	#define XMATEL_B(X, Y) pB[(X)+ldb*(Y)]
#else // MF_PANELMAJ
	#define XMATEL_A(X, Y) MATEL(sA, X, Y)
	#define XMATEL_B(X, Y) MATEL(sB, X, Y)
	#define PS S_PS
	#define PLD S_PLD
#endif



#define FABS fabsf
#define FMAX fmaxf
#define FMIN fminf
#define SQRT sqrtf
#define REAL float
#define MAT blasfeo_smat
#define MATEL BLASFEO_SMATEL
#define VEC blasfeo_svec
#define VECEL BLASFEO_SVECEL



#define REF_MEMSIZE_MAT blasfeo_ref_memsize_smat
#define REF_MEMSIZE_DIAG_MAT blasfeo_ref_memsize_diag_smat
#define REF_MEMSIZE_VEC blasfeo_ref_memsize_svec
#define REF_CREATE_MAT blasfeo_ref_create_smat
#define REF_CREATE_VEC blasfeo_ref_create_svec
#define REF_PACK_MAT blasfeo_ref_pack_smat
#define REF_PACK_L_MAT blasfeo_ref_pack_l_smat
#define REF_PACK_U_MAT blasfeo_ref_pack_u_smat
#define REF_PACK_TRAN_MAT blasfeo_ref_pack_tran_smat
#define REF_PACK_VEC blasfeo_ref_pack_svec
#define REF_UNPACK_MAT blasfeo_ref_unpack_smat
#define REF_UNPACK_TRAN_MAT blasfeo_ref_unpack_tran_smat
#define REF_UNPACK_VEC blasfeo_ref_unpack_svec
#define REF_GECPSC blasfeo_ref_sgecpsc
#define REF_GECP blasfeo_ref_sgecp
#define REF_GESC blasfeo_ref_sgesc
#define REF_GEAD blasfeo_ref_sgead
#define REF_GESE blasfeo_ref_sgese
#define REF_GETR blasfeo_ref_sgetr
#define REF_GEIN1 blasfeo_ref_sgein1
#define REF_GEEX1 blasfeo_ref_sgeex1
#define REF_TRCP_L blasfeo_ref_strcp_l
#define REF_TRTR_L blasfeo_ref_strtr_l
#define REF_TRTR_U blasfeo_ref_strtr_u
#define REF_VECSE blasfeo_ref_svecse
#define REF_VECCP blasfeo_ref_sveccp
#define REF_VECSC blasfeo_ref_svecsc
#define REF_VECCPSC blasfeo_ref_sveccpsc
#define REF_VECAD blasfeo_ref_svecad
#define REF_VECAD_SP blasfeo_ref_svecad_sp
#define REF_VECIN_SP blasfeo_ref_svecin_sp
#define REF_VECEX_SP blasfeo_ref_svecex_sp
#define REF_VECEXAD_SP blasfeo_ref_svecexad_sp
#define REF_VECIN1 blasfeo_ref_svecin1
#define REF_VECEX1 blasfeo_ref_svecex1
#define REF_VECPE blasfeo_ref_svecpe
#define REF_VECPEI blasfeo_ref_svecpei
#define REF_VECCL blasfeo_ref_sveccl
#define REF_VECCL_MASK blasfeo_ref_sveccl_mask
#define REF_VECZE blasfeo_ref_svecze
#define REF_VECNRM_INF blasfeo_ref_svecnrm_inf
#define REF_VECNRM_2 blasfeo_ref_svecnrm_2
#define REF_DIAIN blasfeo_ref_sdiain
#define REF_DIAIN_SP blasfeo_ref_sdiain_sp
#define REF_DIAEX blasfeo_ref_sdiaex
#define REF_DIAEX_SP blasfeo_ref_sdiaex_sp
#define REF_DIAAD blasfeo_ref_sdiaad
#define REF_DIAAD_SP blasfeo_ref_sdiaad_sp
#define REF_DIAADIN_SP blasfeo_ref_sdiaadin_sp
#define REF_DIARE blasfeo_ref_sdiare
#define REF_ROWEX blasfeo_ref_srowex
#define REF_ROWIN blasfeo_ref_srowin
#define REF_ROWAD blasfeo_ref_srowad
#define REF_ROWAD_SP blasfeo_ref_srowad_sp
#define REF_ROWSW blasfeo_ref_srowsw
#define REF_ROWPE blasfeo_ref_srowpe
#define REF_ROWPEI blasfeo_ref_srowpei
#define REF_COLEX blasfeo_ref_scolex
#define REF_COLIN blasfeo_ref_scolin
#define REF_COLAD blasfeo_ref_scolad
#define REF_COLSC blasfeo_ref_scolsc
#define REF_COLSW blasfeo_ref_scolsw
#define REF_COLPE blasfeo_ref_scolpe
#define REF_COLPEI blasfeo_ref_scolpei

#define MEMSIZE_MAT blasfeo_memsize_smat
#define MEMSIZE_DIAG_MAT blasfeo_memsize_diag_smat
#define MEMSIZE_VEC blasfeo_memsize_svec
#define CREATE_MAT blasfeo_create_smat
#define CREATE_VEC blasfeo_create_svec
#define PACK_MAT blasfeo_pack_smat
#define PACK_L_MAT blasfeo_pack_l_smat
#define PACK_U_MAT blasfeo_pack_u_smat
#define PACK_TRAN_MAT blasfeo_pack_tran_smat
#define PACK_VEC blasfeo_pack_svec
#define UNPACK_MAT blasfeo_unpack_smat
#define UNPACK_TRAN_MAT blasfeo_unpack_tran_smat
#define UNPACK_VEC blasfeo_unpack_svec
#define GECPSC blasfeo_sgecpsc
#define GECP blasfeo_sgecp
#define GESC blasfeo_sgesc
#define GEAD blasfeo_sgead
#define GESE blasfeo_sgese
#define GETR blasfeo_sgetr
#define GEIN1 blasfeo_sgein1
#define GEEX1 blasfeo_sgeex1
#define TRCP_L blasfeo_strcp_l
#define TRTR_L blasfeo_strtr_l
#define TRTR_U blasfeo_strtr_u
#define VECSE blasfeo_svecse
#define VECCP blasfeo_sveccp
#define VECSC blasfeo_svecsc
#define VECCPSC blasfeo_sveccpsc
#define VECAD blasfeo_svecad
#define VECAD_SP blasfeo_svecad_sp
#define VECIN_SP blasfeo_svecin_sp
#define VECEX_SP blasfeo_svecex_sp
#define VECEXAD_SP blasfeo_svecexad_sp
#define VECIN1 blasfeo_svecin1
#define VECEX1 blasfeo_svecex1
#define VECPE blasfeo_svecpe
#define VECPEI blasfeo_svecpei
#define VECCL blasfeo_sveccl
#define VECCL_MASK blasfeo_sveccl_mask
#define VECZE blasfeo_svecze
#define VECNRM_INF blasfeo_svecnrm_inf
#define VECNRM_2 blasfeo_svecnrm_2
#define DIAIN blasfeo_sdiain
#define DIAIN_SP blasfeo_sdiain_sp
#define DIAEX blasfeo_sdiaex
#define DIAEX_SP blasfeo_sdiaex_sp
#define DIAAD blasfeo_sdiaad
#define DIAAD_SP blasfeo_sdiaad_sp
#define DIAADIN_SP blasfeo_sdiaadin_sp
#define DIARE blasfeo_sdiare
#define ROWEX blasfeo_srowex
#define ROWIN blasfeo_srowin
#define ROWAD blasfeo_srowad
#define ROWAD_SP blasfeo_srowad_sp
#define ROWSW blasfeo_srowsw
#define ROWPE blasfeo_srowpe
#define ROWPEI blasfeo_srowpei
#define COLEX blasfeo_scolex
#define COLIN blasfeo_scolin
#define COLAD blasfeo_scolad
#define COLSC blasfeo_scolsc
#define COLSW blasfeo_scolsw
#define COLPE blasfeo_scolpe
#define COLPEI blasfeo_scolpei



// LA_REFERENCE | LA_EXTERNAL_BLAS_WRAPPER
#include "x_aux_ref.c"
