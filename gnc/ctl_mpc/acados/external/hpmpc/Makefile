###################################################################################################
#                                                                                                 #
# This file is part of HPMPC.                                                                     #
#                                                                                                 #
# HPMPC -- Library for High-Performance implementation of solvers for MPC.                        #
# Copyright (C) 2014-2015 by Technical University of Denmark. All rights reserved.                #
#                                                                                                 #
# HPMPC is free software; you can redistribute it and/or                                          #
# modify it under the terms of the GNU Lesser General Public                                      #
# License as published by the Free Software Foundation; either                                    #
# version 2.1 of the License, or (at your option) any later version.                              #
#                                                                                                 #
# HPMPC is distributed in the hope that it will be useful,                                        #
# but WITHOUT ANY WARRANTY; without even the implied warranty of                                  #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                                            #
# See the GNU Lesser General Public License for more details.                                     #
#                                                                                                 #
# You should have received a copy of the GNU Lesser General Public                                #
# License along with HPMPC; if not, write to the Free Software                                    #
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA                  #
#                                                                                                 #
# Author: Gianluca Frison, giaf (at) dtu.dk                                                       #
#                                                                                                 #
###################################################################################################


include ./Makefile.rule

OBJS = 

ifeq ($(TARGET), X64_AVX2)
# auxiliary
OBJS += ./auxiliary/d_aux_lib4.o ./auxiliary/d_aux_extern_depend_lib4.o
OBJS +=
OBJS += ./auxiliary/i_aux.o
# kernel
ifeq ($(USE_BLASFEO), 1)
OBJS += 
OBJS += 
else
OBJS += ./kernel/avx2/kernel_dgemm_avx2_lib4.o ./kernel/avx2/kernel_dtrmm_avx2_lib4.o  ./kernel/avx2/kernel_dtrsm_avx2_lib4.o ./kernel/avx2/kernel_dsyrk_avx2_lib4.o  ./kernel/avx2/kernel_dpotrf_avx2_lib4.o ./kernel/avx2/kernel_dgemv_avx2_lib4.o ./kernel/avx2/kernel_dtrmv_avx2_lib4.o ./kernel/avx2/kernel_dtrsv_avx2_lib4.o ./kernel/avx2/kernel_dsymv_avx2_lib4.o ./kernel/avx2/kernel_dtran_avx2_lib4.o ./kernel/avx2/kernel_dttmm_avx2_lib4.o ./kernel/avx2/kernel_dtrinv_avx2_lib4.o ./kernel/avx/kernel_dcopy_avx_lib4.o ./kernel/avx2/kernel_dgetrf_avx2_lib4.o
OBJS += 
endif
# blas
ifeq ($(USE_BLASFEO), 1)
else
OBJS += ./blas/blas_d_lib4.o
OBJS +=
endif
# lqcp solvers
ifeq ($(USE_BLASFEO), 1)
OBJS += ./lqcp_solvers/d_back_ric_rec_libstr.o ./lqcp_solvers/d_tree_back_ric_rec_libstr.o ./lqcp_solvers/d_part_cond_libstr.o
OBJS +=
else
OBJS += ./lqcp_solvers/d_back_ric_rec.o ./lqcp_solvers/d_for_schur_rec.o ./lqcp_solvers/d_res.o ./lqcp_solvers/d_part_cond.o
OBJS +=
endif
# mpc auxiliary
ifeq ($(USE_BLASFEO), 1)
OBJS += ./mpc_solvers/avx/d_aux_ip_hard_libstr.o
OBJS +=
else
OBJS += ./mpc_solvers/avx/d_aux_ip_hard_lib4.o ./mpc_solvers/avx/d_res_ip_res_hard.o ./mpc_solvers/c99/d_aux_ip_soft_lib4.o
OBJS +=
endif
# mpc solvers
ifeq ($(USE_BLASFEO), 1)
OBJS += ./mpc_solvers/d_ip2_res_hard_libstr.o ./mpc_solvers/d_tree_ip2_res_hard_libstr.o ./mpc_solvers/d_res_ip_res_hard_libstr.o ./mpc_solvers/d_tree_res_ip_res_hard_libstr.o
OBJS +=
else
OBJS += ./mpc_solvers/d_ip2_hard.o ./mpc_solvers/d_res_ip_hard.o ./mpc_solvers/d_ip2_res_hard.o ./mpc_solvers/d_ip2_soft.o ./mpc_solvers/d_res_ip_soft.o
OBJS +=
endif
# C/fortran interface
ifeq ($(USE_BLASFEO), 1)
OBJS += ./interfaces/c/fortran_order_interface_libstr.o
else
OBJS += ./interfaces/c/c_interface_work_space.o ./interfaces/c/c_order_interface.o ./interfaces/c/fortran_order_interface.o
endif
endif

ifeq ($(TARGET), X64_AVX)
# auxiliary
OBJS += ./auxiliary/d_aux_lib4.o ./auxiliary/d_aux_extern_depend_lib4.o
OBJS +=
OBJS += ./auxiliary/i_aux.o
# kernel
ifeq ($(USE_BLASFEO), 1)
OBJS += 
OBJS += 
else
OBJS += ./kernel/avx/kernel_dgemm_avx_lib4.o ./kernel/avx/kernel_dtrmm_avx_lib4.o  ./kernel/avx/kernel_dtrsm_avx_lib4.o ./kernel/avx/kernel_dsyrk_avx_lib4.o  ./kernel/avx/kernel_dpotrf_avx_lib4.o ./kernel/avx/kernel_dgemv_avx_lib4.o ./kernel/avx/kernel_dtrmv_avx_lib4.o ./kernel/avx/kernel_dtrsv_avx_lib4.o ./kernel/avx/kernel_dsymv_avx_lib4.o ./kernel/avx/kernel_dtran_avx_lib4.o ./kernel/avx/kernel_dttmm_avx_lib4.o ./kernel/avx/kernel_dtrinv_avx_lib4.o ./kernel/avx/kernel_dcopy_avx_lib4.o ./kernel/avx/kernel_dgetrf_avx_lib4.o
OBJS += 
endif
# blas
ifeq ($(USE_BLASFEO), 1)
else
OBJS += ./blas/blas_d_lib4.o
OBJS +=
endif
# lqcp solvers
ifeq ($(USE_BLASFEO), 1)
OBJS += ./lqcp_solvers/d_back_ric_rec_libstr.o ./lqcp_solvers/d_tree_back_ric_rec_libstr.o ./lqcp_solvers/d_part_cond_libstr.o
OBJS +=
else
OBJS += ./lqcp_solvers/d_back_ric_rec.o ./lqcp_solvers/d_for_schur_rec.o ./lqcp_solvers/d_res.o ./lqcp_solvers/d_part_cond.o
OBJS +=
endif
# mpc auxiliary
ifeq ($(USE_BLASFEO), 1)
OBJS += ./mpc_solvers/avx/d_aux_ip_hard_libstr.o
OBJS +=
else
OBJS += ./mpc_solvers/avx/d_aux_ip_hard_lib4.o ./mpc_solvers/avx/d_res_ip_res_hard.o ./mpc_solvers/c99/d_aux_ip_soft_lib4.o
OBJS +=
endif
# mpc solvers
ifeq ($(USE_BLASFEO), 1)
OBJS += ./mpc_solvers/d_ip2_res_hard_libstr.o ./mpc_solvers/d_tree_ip2_res_hard_libstr.o ./mpc_solvers/d_res_ip_res_hard_libstr.o ./mpc_solvers/d_tree_res_ip_res_hard_libstr.o
OBJS +=
else
OBJS += ./mpc_solvers/d_ip2_hard.o ./mpc_solvers/d_res_ip_hard.o ./mpc_solvers/d_ip2_res_hard.o ./mpc_solvers/d_ip2_soft.o ./mpc_solvers/d_res_ip_soft.o
OBJS +=
endif
# C/fortran interface
ifeq ($(USE_BLASFEO), 1)
OBJS += ./interfaces/c/fortran_order_interface_libstr.o
else
OBJS += ./interfaces/c/c_interface_work_space.o ./interfaces/c/c_order_interface.o ./interfaces/c/fortran_order_interface.o
endif
endif

ifeq ($(TARGET), X64_SSE3)
# auxiliary
OBJS += ./auxiliary/d_aux_lib4.o ./auxiliary/d_aux_extern_depend_lib4.o
OBJS +=
OBJS += ./auxiliary/i_aux.o
# kernel
ifeq ($(USE_BLASFEO), 1)
OBJS += 
OBJS += 
else
OBJS += ./kernel/sse3/kernel_dgemm_sse3_lib4.o ./kernel/sse3/kernel_dtrmm_sse3_lib4.o  ./kernel/sse3/kernel_dtrsm_sse3_lib4.o ./kernel/sse3/kernel_dsyrk_sse3_lib4.o  ./kernel/sse3/kernel_dpotrf_sse3_lib4.o ./kernel/c99/kernel_dgemv_c99_lib4.o ./kernel/c99/kernel_dtrmv_c99_lib4.o ./kernel/c99/kernel_dtrsv_c99_lib4.o ./kernel/c99/kernel_dsymv_c99_lib4.o ./kernel/c99/kernel_dtran_c99_lib4.o ./kernel/c99/kernel_dttmm_c99_lib4.o ./kernel/c99/kernel_dtrinv_c99_lib4.o ./kernel/c99/kernel_dcopy_c99_lib4.o ./kernel/c99/kernel_dgetrf_c99_lib4.o
OBJS += 
endif
# blas
ifeq ($(USE_BLASFEO), 1)
else
OBJS += ./blas/blas_d_lib4.o
OBJS +=
endif
# lqcp solvers
ifeq ($(USE_BLASFEO), 1)
OBJS += ./lqcp_solvers/d_back_ric_rec_libstr.o ./lqcp_solvers/d_tree_back_ric_rec_libstr.o ./lqcp_solvers/d_part_cond_libstr.o
OBJS +=
else
OBJS += ./lqcp_solvers/d_back_ric_rec.o ./lqcp_solvers/d_for_schur_rec.o ./lqcp_solvers/d_res.o ./lqcp_solvers/d_part_cond.o
OBJS +=
endif
# mpc auxiliary
ifeq ($(USE_BLASFEO), 1)
OBJS += ./mpc_solvers/c99/d_aux_ip_hard_libstr.o
OBJS +=
else
OBJS += ./mpc_solvers/c99/d_aux_ip_hard_lib4.o ./mpc_solvers/c99/d_res_ip_res_hard.o ./mpc_solvers/c99/d_aux_ip_soft_lib4.o
OBJS +=
endif
# mpc solvers
ifeq ($(USE_BLASFEO), 1)
OBJS += ./mpc_solvers/d_ip2_res_hard_libstr.o ./mpc_solvers/d_tree_ip2_res_hard_libstr.o ./mpc_solvers/d_res_ip_res_hard_libstr.o ./mpc_solvers/d_tree_res_ip_res_hard_libstr.o
OBJS +=
else
OBJS += ./mpc_solvers/d_ip2_hard.o ./mpc_solvers/d_res_ip_hard.o ./mpc_solvers/d_ip2_res_hard.o ./mpc_solvers/d_ip2_soft.o ./mpc_solvers/d_res_ip_soft.o
OBJS +=
endif
# C/fortran interface
ifeq ($(USE_BLASFEO), 1)
OBJS += ./interfaces/c/fortran_order_interface_libstr.o
else
OBJS += ./interfaces/c/c_interface_work_space.o ./interfaces/c/c_order_interface.o ./interfaces/c/fortran_order_interface.o
endif
endif

ifeq ($(TARGET), C99_4X4)
# auxiliary
OBJS += ./auxiliary/d_aux_lib4.o ./auxiliary/d_aux_extern_depend_lib4.o
OBJS +=
OBJS += ./auxiliary/i_aux.o
# kernel
ifeq ($(USE_BLASFEO), 1)
OBJS += 
OBJS += 
else
OBJS += ./kernel/c99/kernel_dgemm_c99_lib4.o ./kernel/c99/kernel_dtrmm_c99_lib4.o  ./kernel/c99/kernel_dtrsm_c99_lib4.o ./kernel/c99/kernel_dsyrk_c99_lib4.o  ./kernel/c99/kernel_dpotrf_c99_lib4.o ./kernel/c99/kernel_dgemv_c99_lib4.o ./kernel/c99/kernel_dtrmv_c99_lib4.o ./kernel/c99/kernel_dtrsv_c99_lib4.o ./kernel/c99/kernel_dsymv_c99_lib4.o ./kernel/c99/kernel_dtran_c99_lib4.o ./kernel/c99/kernel_dttmm_c99_lib4.o ./kernel/c99/kernel_dtrinv_c99_lib4.o ./kernel/c99/kernel_dcopy_c99_lib4.o ./kernel/c99/kernel_dgetrf_c99_lib4.o
OBJS += 
endif
# blas
ifeq ($(USE_BLASFEO), 1)
else
OBJS += ./blas/blas_d_lib4.o
OBJS +=
endif
# lqcp solvers
ifeq ($(USE_BLASFEO), 1)
OBJS += ./lqcp_solvers/d_back_ric_rec_libstr.o ./lqcp_solvers/d_tree_back_ric_rec_libstr.o ./lqcp_solvers/d_part_cond_libstr.o
OBJS +=
else
OBJS += ./lqcp_solvers/d_back_ric_rec.o ./lqcp_solvers/d_for_schur_rec.o ./lqcp_solvers/d_res.o ./lqcp_solvers/d_part_cond.o
OBJS +=
endif
# mpc auxiliary
ifeq ($(USE_BLASFEO), 1)
OBJS += ./mpc_solvers/c99/d_aux_ip_hard_libstr.o
OBJS +=
else
OBJS += ./mpc_solvers/c99/d_aux_ip_hard_lib4.o ./mpc_solvers/c99/d_res_ip_res_hard.o ./mpc_solvers/c99/d_aux_ip_soft_lib4.o
OBJS +=
endif
# mpc solvers
ifeq ($(USE_BLASFEO), 1)
OBJS += ./mpc_solvers/d_ip2_res_hard_libstr.o ./mpc_solvers/d_tree_ip2_res_hard_libstr.o ./mpc_solvers/d_res_ip_res_hard_libstr.o ./mpc_solvers/d_tree_res_ip_res_hard_libstr.o
OBJS +=
else
OBJS += ./mpc_solvers/d_ip2_hard.o ./mpc_solvers/d_res_ip_hard.o ./mpc_solvers/d_ip2_res_hard.o ./mpc_solvers/d_ip2_soft.o ./mpc_solvers/d_res_ip_soft.o
OBJS +=
endif
# C/fortran interface
ifeq ($(USE_BLASFEO), 1)
OBJS += ./interfaces/c/fortran_order_interface_libstr.o
else
OBJS += ./interfaces/c/c_interface_work_space.o ./interfaces/c/c_order_interface.o ./interfaces/c/fortran_order_interface.o
endif
endif

ifeq ($(TARGET), CORTEX_A57)
# auxiliary
OBJS += ./auxiliary/d_aux_lib4.o ./auxiliary/d_aux_extern_depend_lib4.o
OBJS +=
OBJS += ./auxiliary/i_aux.o
# kernel
ifeq ($(USE_BLASFEO), 1)
OBJS += 
OBJS += 
else
OBJS += ./kernel/armv8a/kernel_dgemm_neon_lib4.o ./kernel/armv8a/kernel_dgemm_neon_assembly_lib4.o ./kernel/c99/kernel_dtrmm_c99_lib4.o  ./kernel/c99/kernel_dtrsm_c99_lib4.o ./kernel/c99/kernel_dsyrk_c99_lib4.o  ./kernel/c99/kernel_dpotrf_c99_lib4.o ./kernel/c99/kernel_dgemv_c99_lib4.o ./kernel/c99/kernel_dtrmv_c99_lib4.o ./kernel/c99/kernel_dtrsv_c99_lib4.o ./kernel/c99/kernel_dsymv_c99_lib4.o ./kernel/c99/kernel_dtran_c99_lib4.o ./kernel/c99/kernel_dttmm_c99_lib4.o ./kernel/c99/kernel_dtrinv_c99_lib4.o ./kernel/c99/kernel_dcopy_c99_lib4.o ./kernel/c99/kernel_dgetrf_c99_lib4.o
OBJS += 
endif
# blas
ifeq ($(USE_BLASFEO), 1)
else
OBJS += ./blas/blas_d_lib4.o
OBJS +=
endif
# lqcp solvers
ifeq ($(USE_BLASFEO), 1)
OBJS += ./lqcp_solvers/d_back_ric_rec_libstr.o ./lqcp_solvers/d_tree_back_ric_rec_libstr.o ./lqcp_solvers/d_part_cond_libstr.o
OBJS +=
else
OBJS += ./lqcp_solvers/d_back_ric_rec.o ./lqcp_solvers/d_for_schur_rec.o ./lqcp_solvers/d_res.o ./lqcp_solvers/d_part_cond.o
OBJS +=
endif
# mpc auxiliary
ifeq ($(USE_BLASFEO), 1)
OBJS += ./mpc_solvers/c99/d_aux_ip_hard_libstr.o
OBJS +=
else
OBJS += ./mpc_solvers/c99/d_aux_ip_hard_lib4.o ./mpc_solvers/c99/d_res_ip_res_hard.o ./mpc_solvers/c99/d_aux_ip_soft_lib4.o
OBJS +=
endif
# mpc solvers
ifeq ($(USE_BLASFEO), 1)
OBJS += ./mpc_solvers/d_ip2_res_hard_libstr.o ./mpc_solvers/d_tree_ip2_res_hard_libstr.o ./mpc_solvers/d_res_ip_res_hard_libstr.o ./mpc_solvers/d_tree_res_ip_res_hard_libstr.o
OBJS +=
else
OBJS += ./mpc_solvers/d_ip2_hard.o ./mpc_solvers/d_res_ip_hard.o ./mpc_solvers/d_ip2_res_hard.o ./mpc_solvers/d_ip2_soft.o ./mpc_solvers/d_res_ip_soft.o
OBJS +=
endif
# C/fortran interface
ifeq ($(USE_BLASFEO), 1)
OBJS += ./interfaces/c/fortran_order_interface_libstr.o
else
OBJS += ./interfaces/c/c_interface_work_space.o ./interfaces/c/c_order_interface.o ./interfaces/c/fortran_order_interface.o
endif
endif

ifeq ($(TARGET), CORTEX_A15)
# auxiliary
OBJS += ./auxiliary/d_aux_lib4.o ./auxiliary/d_aux_extern_depend_lib4.o
OBJS +=
OBJS += ./auxiliary/i_aux.o
# kernel
ifeq ($(USE_BLASFEO), 1)
OBJS += 
OBJS += 
else
OBJS += ./kernel/armv7a/kernel_dgemm_vfpv3_lib4.o ./kernel/armv7a/kernel_dtrmm_vfpv3_lib4.o  ./kernel/armv7a/kernel_dtrsm_vfpv3_lib4.o ./kernel/armv7a/kernel_dsyrk_vfpv3_lib4.o  ./kernel/armv7a/kernel_dpotrf_vfpv3_lib4.o ./kernel/armv7a/kernel_dgemv_c99_lib4.o ./kernel/armv7a/kernel_dtrmv_c99_lib4.o ./kernel/armv7a/kernel_dtrsv_c99_lib4.o ./kernel/c99/kernel_dsymv_c99_lib4.o ./kernel/c99/kernel_dtran_c99_lib4.o ./kernel/c99/kernel_dttmm_c99_lib4.o ./kernel/c99/kernel_dtrinv_c99_lib4.o  ./kernel/c99/kernel_dcopy_c99_lib4.o ./kernel/c99/kernel_dgetrf_c99_lib4.o
OBJS += 
endif
# blas
ifeq ($(USE_BLASFEO), 1)
else
OBJS += ./blas/blas_d_lib4.o
OBJS +=
endif
# lqcp solvers
ifeq ($(USE_BLASFEO), 1)
OBJS += ./lqcp_solvers/d_back_ric_rec_libstr.o ./lqcp_solvers/d_tree_back_ric_rec_libstr.o ./lqcp_solvers/d_part_cond_libstr.o
OBJS +=
else
OBJS += ./lqcp_solvers/d_back_ric_rec.o ./lqcp_solvers/d_for_schur_rec.o ./lqcp_solvers/d_res.o ./lqcp_solvers/d_part_cond.o
OBJS +=
endif
# mpc auxiliary
ifeq ($(USE_BLASFEO), 1)
OBJS += ./mpc_solvers/c99/d_aux_ip_hard_libstr.o
OBJS +=
else
OBJS += ./mpc_solvers/c99/d_aux_ip_hard_lib4.o ./mpc_solvers/c99/d_res_ip_res_hard.o ./mpc_solvers/c99/d_aux_ip_soft_lib4.o
OBJS +=
endif
# mpc solvers
ifeq ($(USE_BLASFEO), 1)
OBJS += ./mpc_solvers/d_ip2_res_hard_libstr.o ./mpc_solvers/d_tree_ip2_res_hard_libstr.o ./mpc_solvers/d_res_ip_res_hard_libstr.o ./mpc_solvers/d_tree_res_ip_res_hard_libstr.o
OBJS +=
else
OBJS += ./mpc_solvers/d_ip2_hard.o ./mpc_solvers/d_res_ip_hard.o ./mpc_solvers/d_ip2_res_hard.o ./mpc_solvers/d_ip2_soft.o ./mpc_solvers/d_res_ip_soft.o
OBJS +=
endif
# C/fortran interface
ifeq ($(USE_BLASFEO), 1)
OBJS += ./interfaces/c/fortran_order_interface_libstr.o
else
OBJS += ./interfaces/c/c_interface_work_space.o ./interfaces/c/c_order_interface.o ./interfaces/c/fortran_order_interface.o
endif
endif

ifeq ($(TARGET), CORTEX_A9)
# auxiliary
OBJS += ./auxiliary/d_aux_lib4.o ./auxiliary/d_aux_extern_depend_lib4.o
OBJS +=
OBJS += ./auxiliary/i_aux.o
# kernel
ifeq ($(USE_BLASFEO), 1)
OBJS += 
OBJS += 
else
OBJS += ./kernel/armv7a/kernel_dgemm_vfpv3_lib4.o ./kernel/armv7a/kernel_dtrmm_vfpv3_lib4.o  ./kernel/armv7a/kernel_dtrsm_vfpv3_lib4.o ./kernel/armv7a/kernel_dsyrk_vfpv3_lib4.o  ./kernel/armv7a/kernel_dpotrf_vfpv3_lib4.o ./kernel/armv7a/kernel_dgemv_c99_lib4.o ./kernel/armv7a/kernel_dtrmv_c99_lib4.o ./kernel/armv7a/kernel_dtrsv_c99_lib4.o ./kernel/c99/kernel_dsymv_c99_lib4.o ./kernel/c99/kernel_dtran_c99_lib4.o ./kernel/c99/kernel_dttmm_c99_lib4.o ./kernel/c99/kernel_dtrinv_c99_lib4.o ./kernel/c99/kernel_dcopy_c99_lib4.o ./kernel/c99/kernel_dgetrf_c99_lib4.o
OBJS += 
endif
# blas
ifeq ($(USE_BLASFEO), 1)
else
OBJS += ./blas/blas_d_lib4.o
OBJS +=
endif
# lqcp solvers
ifeq ($(USE_BLASFEO), 1)
OBJS += ./lqcp_solvers/d_back_ric_rec_libstr.o ./lqcp_solvers/d_tree_back_ric_rec_libstr.o ./lqcp_solvers/d_part_cond_libstr.o
OBJS +=
else
OBJS += ./lqcp_solvers/d_back_ric_rec.o ./lqcp_solvers/d_for_schur_rec.o ./lqcp_solvers/d_res.o ./lqcp_solvers/d_part_cond.o
OBJS +=
endif
# mpc auxiliary
ifeq ($(USE_BLASFEO), 1)
OBJS += ./mpc_solvers/c99/d_aux_ip_hard_libstr.o
OBJS +=
else
OBJS += ./mpc_solvers/c99/d_aux_ip_hard_lib4.o ./mpc_solvers/c99/d_res_ip_res_hard.o ./mpc_solvers/c99/d_aux_ip_soft_lib4.o
OBJS +=
endif
# mpc solvers
ifeq ($(USE_BLASFEO), 1)
OBJS += ./mpc_solvers/d_ip2_res_hard_libstr.o ./mpc_solvers/d_tree_ip2_res_hard_libstr.o ./mpc_solvers/d_res_ip_res_hard_libstr.o ./mpc_solvers/d_tree_res_ip_res_hard_libstr.o
OBJS +=
else
OBJS += ./mpc_solvers/d_ip2_hard.o ./mpc_solvers/d_res_ip_hard.o ./mpc_solvers/d_ip2_res_hard.o ./mpc_solvers/d_ip2_soft.o ./mpc_solvers/d_res_ip_soft.o
OBJS +=
endif
# C/fortran interface
ifeq ($(USE_BLASFEO), 1)
OBJS += ./interfaces/c/fortran_order_interface_libstr.o
else
OBJS += ./interfaces/c/c_interface_work_space.o ./interfaces/c/c_order_interface.o ./interfaces/c/fortran_order_interface.o
endif
endif

ifeq ($(TARGET), CORTEX_A7)
# auxiliary
OBJS += ./auxiliary/d_aux_lib4.o ./auxiliary/d_aux_extern_depend_lib4.o
OBJS +=
OBJS += ./auxiliary/i_aux.o
# kernel
ifeq ($(USE_BLASFEO), 1)
OBJS += 
OBJS += 
else
OBJS += ./kernel/armv7a/kernel_dgemm_vfpv3_lib4.o ./kernel/armv7a/kernel_dtrmm_vfpv3_lib4.o  ./kernel/armv7a/kernel_dtrsm_vfpv3_lib4.o ./kernel/armv7a/kernel_dsyrk_vfpv3_lib4.o  ./kernel/armv7a/kernel_dpotrf_vfpv3_lib4.o ./kernel/armv7a/kernel_dgemv_c99_lib4.o ./kernel/armv7a/kernel_dtrmv_c99_lib4.o ./kernel/armv7a/kernel_dtrsv_c99_lib4.o ./kernel/c99/kernel_dsymv_c99_lib4.o ./kernel/c99/kernel_dtran_c99_lib4.o ./kernel/c99/kernel_dttmm_c99_lib4.o ./kernel/c99/kernel_dtrinv_c99_lib4.o ./kernel/c99/kernel_dcopy_c99_lib4.o ./kernel/c99/kernel_dgetrf_c99_lib4.o
OBJS += 
endif
# blas
ifeq ($(USE_BLASFEO), 1)
else
OBJS += ./blas/blas_d_lib4.o
OBJS +=
endif
# lqcp solvers
ifeq ($(USE_BLASFEO), 1)
OBJS += ./lqcp_solvers/d_back_ric_rec_libstr.o ./lqcp_solvers/d_tree_back_ric_rec_libstr.o ./lqcp_solvers/d_part_cond_libstr.o
OBJS +=
else
OBJS += ./lqcp_solvers/d_back_ric_rec.o ./lqcp_solvers/d_for_schur_rec.o ./lqcp_solvers/d_res.o ./lqcp_solvers/d_part_cond.o
OBJS +=
endif
# mpc auxiliary
ifeq ($(USE_BLASFEO), 1)
OBJS += ./mpc_solvers/c99/d_aux_ip_hard_libstr.o
OBJS +=
else
OBJS += ./mpc_solvers/c99/d_aux_ip_hard_lib4.o ./mpc_solvers/c99/d_res_ip_res_hard.o ./mpc_solvers/c99/d_aux_ip_soft_lib4.o
OBJS +=
endif
# mpc solvers
ifeq ($(USE_BLASFEO), 1)
OBJS += ./mpc_solvers/d_ip2_res_hard_libstr.o ./mpc_solvers/d_tree_ip2_res_hard_libstr.o ./mpc_solvers/d_res_ip_res_hard_libstr.o ./mpc_solvers/d_tree_res_ip_res_hard_libstr.o
OBJS +=
else
OBJS += ./mpc_solvers/d_ip2_hard.o ./mpc_solvers/d_res_ip_hard.o ./mpc_solvers/d_ip2_res_hard.o ./mpc_solvers/d_ip2_soft.o ./mpc_solvers/d_res_ip_soft.o
OBJS +=
endif
# C/fortran interface
ifeq ($(USE_BLASFEO), 1)
OBJS += ./interfaces/c/fortran_order_interface_libstr.o
else
OBJS += ./interfaces/c/c_interface_work_space.o ./interfaces/c/c_order_interface.o ./interfaces/c/fortran_order_interface.o
endif
endif


all: static_library test_problem

static_library: target
	( cd auxiliary; $(MAKE) obj)
	( cd kernel; $(MAKE) obj)
	( cd blas; $(MAKE) obj)
	( cd lqcp_solvers; $(MAKE) obj)
	( cd mpc_solvers; $(MAKE) obj)
	( cd interfaces; $(MAKE) obj)
#ifneq ($(REF_BLAS), 0)
	make -C reference_code obj
#endif
	ar rcs libhpmpc.a $(OBJS)
	@echo
	@echo " libhpmpc.a static library build complete."
	@echo

shared_library: target
	( cd auxiliary; $(MAKE) obj)
	( cd kernel; $(MAKE) obj)
	( cd blas; $(MAKE) obj)
	( cd lqcp_solvers; $(MAKE) obj)
	( cd mpc_solvers; $(MAKE) obj)
	( cd interfaces; $(MAKE) obj)
#ifneq ($(REF_BLAS), 0)
	make -C reference_code obj
#endif
	gcc -shared -o libhpmpc.so $(OBJS)
	@echo
	@echo " libhpmpc.so shared library build complete."
	@echo

target:
	touch ./include/target.h
ifeq ($(TARGET), X64_AVX2)
	echo "#ifndef TARGET_X64_AVX2" > ./include/target.h
	echo "#define TARGET_X64_AVX2" >> ./include/target.h
	echo "#endif" >> ./include/target.h
endif
ifeq ($(TARGET), X64_AVX)
	echo "#ifndef TARGET_X64_AVX" > ./include/target.h
	echo "#define TARGET_X64_AVX" >> ./include/target.h
	echo "#endif" >> ./include/target.h
endif
ifeq ($(TARGET), X64_SSE3)
	echo "#ifndef TARGET_X64_SSE3" > ./include/target.h
	echo "#define TARGET_X64_SSE3" >> ./include/target.h
	echo "#endif" >> ./include/target.h
endif
ifeq ($(TARGET), C99_4X4)
	echo "#ifndef TARGET_C99_4X4" > ./include/target.h
	echo "#define TARGET_C99_4X4" >> ./include/target.h
	echo "#endif" >> ./include/target.h
endif
ifeq ($(TARGET), CORTEX_A57)
	echo "#ifndef TARGET_CORTEX_A57" > ./include/target.h
	echo "#define TARGET_CORTEX_A57" >> ./include/target.h
	echo "#endif" >> ./include/target.h
endif
ifeq ($(TARGET), CORTEX_A15)
	echo "#ifndef TARGET_CORTEX_A15" > ./include/target.h
	echo "#define TARGET_CORTEX_A15" >> ./include/target.h
	echo "#endif" >> ./include/target.h
endif
ifeq ($(TARGET), CORTEX_A9)
	echo "#ifndef TARGET_CORTEX_A9" > ./include/target.h
	echo "#define TARGET_CORTEX_A9" >> ./include/target.h
	echo "#endif" >> ./include/target.h
endif
ifeq ($(TARGET), CORTEX_A7)
	echo "#ifndef TARGET_CORTEX_A7" > ./include/target.h
	echo "#define TARGET_CORTEX_A7" >> ./include/target.h
	echo "#endif" >> ./include/target.h
endif
ifeq ($(USE_BLASFEO), 1)
	echo "#ifndef BLASFEO" >> ./include/target.h
	echo "#define BLASFEO" >> ./include/target.h
	echo "#endif" >> ./include/target.h
endif

test_problem:
	cp libhpmpc.a ./test_problems/libhpmpc.a
#	cp libhpmpc.a ./interfaces/octave/libhpmpc.a
	make -C test_problems obj
	@echo
	@echo " Test problem build complete."
	@echo

run:
	./test_problems/test.out

install_static:
	mkdir -p $(PREFIX)/hpmpc
	mkdir -p $(PREFIX)/hpmpc/lib
	cp -f libhpmpc.a $(PREFIX)/hpmpc/lib/libhpmpc.a
	mkdir -p $(PREFIX)/hpmpc/include
	cp -rf ./include/*.h $(PREFIX)/hpmpc/include/

install_shared:
	mkdir -p $(PREFIX)/hpmpc
	mkdir -p $(PREFIX)/hpmpc/lib
	cp -f libhpmpc.a $(PREFIX)/hpmpc/lib/libhpmpc.so
	mkdir -p $(PREFIX)/hpmpc/include
	cp -rf ./include/*.h $(PREFIX)/hpmpc/include/

uninstall:
	rm -rf $(PREFIX)/hpmpc

clean:
	make -C auxiliary clean
	make -C kernel clean
	make -C blas clean
	make -C lqcp_solvers clean
	make -C mpc_solvers clean
	make -C test_problems clean
	make -C interfaces/octave clean
	make -C interfaces/c clean
	make -C reference_code clean
#	rm -f $(OBJS)
#	rm -f test.out
#	rm -f *.s
	rm -f target_generator.out
	rm -f *.o
	rm -f libhpmpc.a
	rm -f libhpmpc.so
#	rm -f ./matlab/HPMPC.a
