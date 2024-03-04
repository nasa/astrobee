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

/*
 * auxiliary functions for LA:REFERENCE (column major)
 *
 * auxiliary/d_aux_lib*.c
 *
 */

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
	#define PS D_PS
	#define PLD D_PLD
#endif



#define FABS fabs
#define FMAX fmax
#define FMIN fmin
#define SQRT sqrt
#define REAL double
#define MAT blasfeo_dmat
#define MATEL BLASFEO_DMATEL
#define VEC blasfeo_dvec
#define VECEL BLASFEO_DVECEL



#define REF_MEMSIZE_MAT blasfeo_ref_memsize_dmat
#define REF_MEMSIZE_DIAG_MAT blasfeo_ref_memsize_diag_dmat
#define REF_MEMSIZE_VEC blasfeo_ref_memsize_dvec
#define REF_CREATE_MAT blasfeo_ref_create_dmat
#define REF_CREATE_VEC blasfeo_ref_create_dvec
#define REF_PACK_MAT blasfeo_ref_pack_dmat
#define REF_PACK_L_MAT blasfeo_ref_pack_l_dmat
#define REF_PACK_U_MAT blasfeo_ref_pack_u_dmat
#define REF_PACK_TRAN_MAT blasfeo_ref_pack_tran_dmat
#define REF_PACK_VEC blasfeo_ref_pack_dvec
#define REF_UNPACK_MAT blasfeo_ref_unpack_dmat
#define REF_UNPACK_TRAN_MAT blasfeo_ref_unpack_tran_dmat
#define REF_UNPACK_VEC blasfeo_ref_unpack_dvec
#define REF_GECP blasfeo_ref_dgecp
#define REF_GESC blasfeo_ref_dgesc
#define REF_GECPSC blasfeo_ref_dgecpsc
#define REF_GEAD blasfeo_ref_dgead
#define REF_GESE blasfeo_ref_dgese
#define REF_GETR blasfeo_ref_dgetr
#define REF_GEIN1 blasfeo_ref_dgein1
#define REF_GEEX1 blasfeo_ref_dgeex1
#define REF_TRCP_L blasfeo_ref_dtrcp_l
#define REF_TRTR_L blasfeo_ref_dtrtr_l
#define REF_TRTR_U blasfeo_ref_dtrtr_u
#define REF_VECSE blasfeo_ref_dvecse
#define REF_VECCP blasfeo_ref_dveccp
#define REF_VECSC blasfeo_ref_dvecsc
#define REF_VECCPSC blasfeo_ref_dveccpsc
#define REF_VECAD blasfeo_ref_dvecad
#define REF_VECAD_SP blasfeo_ref_dvecad_sp
#define REF_VECIN_SP blasfeo_ref_dvecin_sp
#define REF_VECEX_SP blasfeo_ref_dvecex_sp
#define REF_VECEXAD_SP blasfeo_ref_dvecexad_sp
#define REF_VECIN1 blasfeo_ref_dvecin1
#define REF_VECEX1 blasfeo_ref_dvecex1
#define REF_VECPE blasfeo_ref_dvecpe
#define REF_VECPEI blasfeo_ref_dvecpei
#define REF_VECCL blasfeo_ref_dveccl
#define REF_VECCL_MASK blasfeo_ref_dveccl_mask
#define REF_VECZE blasfeo_ref_dvecze
#define REF_VECNRM_INF blasfeo_ref_dvecnrm_inf
#define REF_VECNRM_2 blasfeo_ref_dvecnrm_2
#define REF_DIAIN blasfeo_ref_ddiain
#define REF_DIAIN_SP blasfeo_ref_ddiain_sp
#define REF_DIAEX blasfeo_ref_ddiaex
#define REF_DIAEX_SP blasfeo_ref_ddiaex_sp
#define REF_DIAAD blasfeo_ref_ddiaad
#define REF_DIAAD_SP blasfeo_ref_ddiaad_sp
#define REF_DIAADIN_SP blasfeo_ref_ddiaadin_sp
#define REF_DIARE blasfeo_ref_ddiare
#define REF_ROWEX blasfeo_ref_drowex
#define REF_ROWIN blasfeo_ref_drowin
#define REF_ROWAD blasfeo_ref_drowad
#define REF_ROWAD_SP blasfeo_ref_drowad_sp
#define REF_ROWSW blasfeo_ref_drowsw
#define REF_ROWPE blasfeo_ref_drowpe
#define REF_ROWPEI blasfeo_ref_drowpei
#define REF_COLEX blasfeo_ref_dcolex
#define REF_COLIN blasfeo_ref_dcolin
#define REF_COLAD blasfeo_ref_dcolad
#define REF_COLSC blasfeo_ref_dcolsc
#define REF_COLSW blasfeo_ref_dcolsw
#define REF_COLPE blasfeo_ref_dcolpe
#define REF_COLPEI blasfeo_ref_dcolpei

#define MEMSIZE_MAT blasfeo_memsize_dmat
#define MEMSIZE_DIAG_MAT blasfeo_memsize_diag_dmat
#define MEMSIZE_VEC blasfeo_memsize_dvec
#define CREATE_MAT blasfeo_create_dmat
#define CREATE_VEC blasfeo_create_dvec
#define PACK_MAT blasfeo_pack_dmat
#define PACK_L_MAT blasfeo_pack_l_dmat
#define PACK_U_MAT blasfeo_pack_u_dmat
#define PACK_TRAN_MAT blasfeo_pack_tran_dmat
#define PACK_VEC blasfeo_pack_dvec
#define UNPACK_MAT blasfeo_unpack_dmat
#define UNPACK_TRAN_MAT blasfeo_unpack_tran_dmat
#define UNPACK_VEC blasfeo_unpack_dvec
#define GECP blasfeo_dgecp
#define GESC blasfeo_dgesc
#define GECPSC blasfeo_dgecpsc
#define GEAD blasfeo_dgead
#define GESE blasfeo_dgese
#define GETR blasfeo_dgetr
#define GEIN1 blasfeo_dgein1
#define GEEX1 blasfeo_dgeex1
#define TRCP_L blasfeo_dtrcp_l
#define TRTR_L blasfeo_dtrtr_l
#define TRTR_U blasfeo_dtrtr_u
#define VECSE blasfeo_dvecse
#define VECCP blasfeo_dveccp
#define VECSC blasfeo_dvecsc
#define VECCPSC blasfeo_dveccpsc
#define VECAD blasfeo_dvecad
#define VECAD_SP blasfeo_dvecad_sp
#define VECIN_SP blasfeo_dvecin_sp
#define VECEX_SP blasfeo_dvecex_sp
#define VECEXAD_SP blasfeo_dvecexad_sp
#define VECIN1 blasfeo_dvecin1
#define VECEX1 blasfeo_dvecex1
#define VECPE blasfeo_dvecpe
#define VECPEI blasfeo_dvecpei
#define VECCL blasfeo_dveccl
#define VECCL_MASK blasfeo_dveccl_mask
#define VECZE blasfeo_dvecze
#define VECNRM_INF blasfeo_dvecnrm_inf
#define VECNRM_2 blasfeo_dvecnrm_2
#define DIAIN blasfeo_ddiain
#define DIAIN_SP blasfeo_ddiain_sp
#define DIAEX blasfeo_ddiaex
#define DIAEX_SP blasfeo_ddiaex_sp
#define DIAAD blasfeo_ddiaad
#define DIAAD_SP blasfeo_ddiaad_sp
#define DIAADIN_SP blasfeo_ddiaadin_sp
#define DIARE blasfeo_ddiare
#define ROWEX blasfeo_drowex
#define ROWIN blasfeo_drowin
#define ROWAD blasfeo_drowad
#define ROWAD_SP blasfeo_drowad_sp
#define ROWSW blasfeo_drowsw
#define ROWPE blasfeo_drowpe
#define ROWPEI blasfeo_drowpei
#define COLEX blasfeo_dcolex
#define COLIN blasfeo_dcolin
#define COLAD blasfeo_dcolad
#define COLSC blasfeo_dcolsc
#define COLSW blasfeo_dcolsw
#define COLPE blasfeo_dcolpe
#define COLPEI blasfeo_dcolpei



// LA_REFERENCE | LA_EXTERNAL_BLAS_WRAPPER
#include "x_aux_ref.c"
