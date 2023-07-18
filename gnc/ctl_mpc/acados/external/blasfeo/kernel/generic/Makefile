###################################################################################################
#                                                                                                 #
# This file is part of BLASFEO.                                                                   #
#                                                                                                 #
# BLASFEO -- BLAS for embedded optimization.                                                      #
# Copyright (C) 2019 by Gianluca Frison.                                                          #
# Developed at IMTEK (University of Freiburg) under the supervision of Moritz Diehl.              #
# All rights reserved.                                                                            #
#                                                                                                 #
# The 2-Clause BSD License                                                                        #
#                                                                                                 #
# Redistribution and use in source and binary forms, with or without                              #
# modification, are permitted provided that the following conditions are met:                     #
#                                                                                                 #
# 1. Redistributions of source code must retain the above copyright notice, this                  #
#    list of conditions and the following disclaimer.                                             #
# 2. Redistributions in binary form must reproduce the above copyright notice,                    #
#    this list of conditions and the following disclaimer in the documentation                    #
#    and/or other materials provided with the distribution.                                       #
#                                                                                                 #
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND                 #
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED                   #
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE                          #
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR                 #
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES                  #
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;                    #
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND                     #
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT                      #
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS                   #
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                    #
#                                                                                                 #
# Author: Gianluca Frison, gianluca.frison (at) imtek.uni-freiburg.de                             #
#                                                                                                 #
###################################################################################################

include ../../Makefile.rule


ifeq ($(TARGET), X64_INTEL_HASWELL)
KERNEL_OBJS = kernel_dgemv_4_lib4.o \
		kernel_dsymv_4_lib4.o \
		kernel_dpack_buffer_lib4.o \
		kernel_dger_lib4.o \
		kernel_ddot_lib.o \
		kernel_daxpy_lib.o \
		\
		kernel_sgemm_4x4_lib4.o \
		kernel_sgemm_diag_lib4.o \
		kernel_sgemv_4_lib4.o \
		kernel_ssymv_4_lib4.o \
		kernel_sgetrf_pivot_lib4.o \
		kernel_spack_lib4.o \
		kernel_sdot_lib.o \
		kernel_saxpy_lib.o \
		\
		kernel_sgemm_8x4_lib8.o \

endif

ifeq ($(TARGET), X64_INTEL_SANDY_BRIDGE)
KERNEL_OBJS = kernel_dgemv_4_lib4.o \
		kernel_dsymv_4_lib4.o \
		kernel_dpack_buffer_lib4.o \
		kernel_dger_lib4.o \
		kernel_ddot_lib.o \
		kernel_daxpy_lib.o \
		\
		kernel_sgemm_4x4_lib4.o \
		kernel_sgemm_diag_lib4.o \
		kernel_sgemv_4_lib4.o \
		kernel_ssymv_4_lib4.o \
		kernel_sgetrf_pivot_lib4.o \
		kernel_spack_lib4.o \
		kernel_sdot_lib.o \
		kernel_saxpy_lib.o \
		\
		kernel_sgemm_8x4_lib8.o \

endif

ifeq ($(TARGET), X64_INTEL_CORE)
KERNEL_OBJS = \
		kernel_dgemm_4x4_lib4.o \
		kernel_dgemm_diag_lib4.o \
		kernel_dgemv_4_lib4.o \
		kernel_dsymv_4_lib4.o \
		kernel_dgetrf_pivot_lib4.o \
		kernel_dgeqrf_4_lib4.o \
		kernel_dgecp_lib4.o \
		kernel_dgetr_lib4.o \
		kernel_dpack_lib4.o \
		kernel_dpack_buffer_lib4.o \
		kernel_dger_lib4.o \
		kernel_ddot_lib.o \
		kernel_daxpy_lib.o \
		kernel_dgetr_lib.o \
		\
		kernel_sgemm_4x4_lib4.o \
		kernel_sgemm_diag_lib4.o \
		kernel_sgemv_4_lib4.o \
		kernel_ssymv_4_lib4.o \
		kernel_sgetrf_pivot_lib4.o \
		kernel_sgecp_lib4.o \
		kernel_sgetr_lib4.o \
		kernel_spack_lib4.o \
		kernel_sdot_lib.o \
		kernel_saxpy_lib.o \
		\
		kernel_d_aux_lib.o \

endif

ifeq ($(TARGET), X64_AMD_BULLDOZER)
KERNEL_OBJS = \
		kernel_dgemm_4x4_lib4.o \
		kernel_dgemm_diag_lib4.o \
		kernel_dgemv_4_lib4.o \
		kernel_dsymv_4_lib4.o \
		kernel_dgetrf_pivot_lib4.o \
		kernel_dgeqrf_4_lib4.o \
		kernel_dgecp_lib4.o \
		kernel_dgetr_lib4.o \
		kernel_dpack_lib4.o \
		kernel_dpack_buffer_lib4.o \
		kernel_dger_lib4.o \
		kernel_ddot_lib.o \
		kernel_daxpy_lib.o \
		kernel_dgetr_lib.o \
		\
		kernel_sgemm_4x4_lib4.o \
		kernel_sgemm_diag_lib4.o \
		kernel_sgemv_4_lib4.o \
		kernel_ssymv_4_lib4.o \
		kernel_sgetrf_pivot_lib4.o \
		kernel_sgecp_lib4.o \
		kernel_sgetr_lib4.o \
		kernel_spack_lib4.o \
		kernel_sdot_lib.o \
		kernel_saxpy_lib.o \
		\
		kernel_d_aux_lib.o \

endif

ifeq ($(TARGET), X86_AMD_JAGUAR)
KERNEL_OBJS = \
		kernel_dgemm_4x4_lib4.o \
		kernel_dgemm_diag_lib4.o \
		kernel_dgemv_4_lib4.o \
		kernel_dsymv_4_lib4.o \
		kernel_dgetrf_pivot_lib4.o \
		kernel_dgeqrf_4_lib4.o \
		kernel_dgecp_lib4.o \
		kernel_dgetr_lib4.o \
		kernel_dpack_lib4.o \
		kernel_dpack_buffer_lib4.o \
		kernel_dger_lib4.o \
		kernel_ddot_lib.o \
		kernel_daxpy_lib.o \
		kernel_dgetr_lib.o \
		\
		kernel_sgemm_4x4_lib4.o \
		kernel_sgemm_diag_lib4.o \
		kernel_sgemv_4_lib4.o \
		kernel_ssymv_4_lib4.o \
		kernel_sgetrf_pivot_lib4.o \
		kernel_sgecp_lib4.o \
		kernel_sgetr_lib4.o \
		kernel_spack_lib4.o \
		kernel_sdot_lib.o \
		kernel_saxpy_lib.o \
		\
		kernel_d_aux_lib.o \

endif

ifeq ($(TARGET), X86_AMD_BARCELONA)
KERNEL_OBJS = \
		kernel_dgemm_4x4_lib4.o \
		kernel_dgemm_diag_lib4.o \
		kernel_dgemv_4_lib4.o \
		kernel_dsymv_4_lib4.o \
		kernel_dgetrf_pivot_lib4.o \
		kernel_dgeqrf_4_lib4.o \
		kernel_dgecp_lib4.o \
		kernel_dgetr_lib4.o \
		kernel_dpack_lib4.o \
		kernel_dpack_buffer_lib4.o \
		kernel_dger_lib4.o \
		kernel_ddot_lib.o \
		kernel_daxpy_lib.o \
		kernel_dgetr_lib.o \
		\
		kernel_sgemm_4x4_lib4.o \
		kernel_sgemm_diag_lib4.o \
		kernel_sgemv_4_lib4.o \
		kernel_ssymv_4_lib4.o \
		kernel_sgetrf_pivot_lib4.o \
		kernel_sgecp_lib4.o \
		kernel_sgetr_lib4.o \
		kernel_spack_lib4.o \
		kernel_sdot_lib.o \
		kernel_saxpy_lib.o \
		\
		kernel_d_aux_lib.o \

endif

ifeq ($(TARGET), $(filter $(TARGET), ARMV8A_APPLE_M1 ARMV8A_ARM_CORTEX_A76 ARMV8A_ARM_CORTEX_A73 ARMV8A_ARM_CORTEX_A57))
KERNEL_OBJS = \
		kernel_dgemm_4x4_lib4.o \
		kernel_dgemm_diag_lib4.o \
		kernel_dgemv_4_lib4.o \
		kernel_dsymv_4_lib4.o \
		kernel_dgetrf_pivot_lib4.o \
		kernel_dgeqrf_4_lib4.o \
		kernel_dgecp_lib4.o \
		kernel_dgetr_lib4.o \
		kernel_dpack_lib4.o \
		kernel_dpack_buffer_lib4.o \
		kernel_dger_lib4.o \
		kernel_ddot_lib.o \
		kernel_daxpy_lib.o \
		kernel_dgetr_lib.o \
		\
		kernel_sgemm_4x4_lib4.o \
		kernel_sgemm_diag_lib4.o \
		kernel_sgemv_4_lib4.o \
		kernel_ssymv_4_lib4.o \
		kernel_sgetrf_pivot_lib4.o \
		kernel_sgecp_lib4.o \
		kernel_sgetr_lib4.o \
		kernel_spack_lib4.o \
		kernel_sdot_lib.o \
		kernel_saxpy_lib.o \
		\
		kernel_align_generic.o \

endif

ifeq ($(TARGET), $(filter $(TARGET), ARMV8A_ARM_CORTEX_A55 ARMV8A_ARM_CORTEX_A53))
KERNEL_OBJS = \
		kernel_dgemm_4x4_lib4.o \
		kernel_dgemm_diag_lib4.o \
		kernel_dgemv_4_lib4.o \
		kernel_dsymv_4_lib4.o \
		kernel_dgetrf_pivot_lib4.o \
		kernel_dgeqrf_4_lib4.o \
		kernel_dgecp_lib4.o \
		kernel_dgetr_lib4.o \
		kernel_dpack_lib4.o \
		kernel_dpack_buffer_lib4.o \
		kernel_dger_lib4.o \
		kernel_ddot_lib.o \
		kernel_daxpy_lib.o \
		kernel_dgetr_lib.o \
		\
		kernel_sgemm_4x4_lib4.o \
		kernel_sgemm_diag_lib4.o \
		kernel_sgemv_4_lib4.o \
		kernel_ssymv_4_lib4.o \
		kernel_sgetrf_pivot_lib4.o \
		kernel_sgecp_lib4.o \
		kernel_sgetr_lib4.o \
		kernel_spack_lib4.o \
		kernel_sdot_lib.o \
		kernel_saxpy_lib.o \
		\
		kernel_align_generic.o \

endif

ifeq ($(TARGET), ARMV7A_ARM_CORTEX_A15)
KERNEL_OBJS = \
		kernel_dgemm_4x4_lib4.o \
		kernel_dgemm_diag_lib4.o \
		kernel_dgemv_4_lib4.o \
		kernel_dsymv_4_lib4.o \
		kernel_dgetrf_pivot_lib4.o \
		kernel_dgeqrf_4_lib4.o \
		kernel_dgecp_lib4.o \
		kernel_dgetr_lib4.o \
		kernel_dpack_lib4.o \
		kernel_dpack_buffer_lib4.o \
		kernel_dger_lib4.o \
		kernel_ddot_lib.o \
		kernel_daxpy_lib.o \
		kernel_dgetr_lib.o \
		\
		kernel_sgemm_4x4_lib4.o \
		kernel_sgemm_diag_lib4.o \
		kernel_sgemv_4_lib4.o \
		kernel_ssymv_4_lib4.o \
		kernel_sgetrf_pivot_lib4.o \
		kernel_sgecp_lib4.o \
		kernel_sgetr_lib4.o \
		kernel_spack_lib4.o \
		kernel_sdot_lib.o \
		kernel_saxpy_lib.o \
		\
		kernel_align_generic.o \
		\
		kernel_d_aux_lib.o \

endif

ifeq ($(TARGET), $(filter $(TARGET), ARMV7A_ARM_CORTEX_A9 ARMV7A_ARM_CORTEX_A7))
KERNEL_OBJS = \
		kernel_dgemm_4x4_lib4.o \
		kernel_dgemm_diag_lib4.o \
		kernel_dgemv_4_lib4.o \
		kernel_dsymv_4_lib4.o \
		kernel_dgetrf_pivot_lib4.o \
		kernel_dgeqrf_4_lib4.o \
		kernel_dgecp_lib4.o \
		kernel_dgetr_lib4.o \
		kernel_dpack_lib4.o \
		kernel_dpack_buffer_lib4.o \
		kernel_dger_lib4.o \
		kernel_ddot_lib.o \
		kernel_daxpy_lib.o \
		kernel_dgetr_lib.o \
		\
		kernel_sgemm_4x4_lib4.o \
		kernel_sgemm_diag_lib4.o \
		kernel_sgemv_4_lib4.o \
		kernel_ssymv_4_lib4.o \
		kernel_sgetrf_pivot_lib4.o \
		kernel_sgecp_lib4.o \
		kernel_sgetr_lib4.o \
		kernel_spack_lib4.o \
		kernel_sdot_lib.o \
		kernel_saxpy_lib.o \
		\
		kernel_align_generic.o \
		\
		kernel_d_aux_lib.o \

endif

ifeq ($(TARGET), GENERIC)
KERNEL_OBJS = \
		kernel_dgemm_4x4_lib4.o \
		kernel_dgemm_diag_lib4.o \
		kernel_dgemv_4_lib4.o \
		kernel_dsymv_4_lib4.o \
		kernel_dgetrf_pivot_lib4.o \
		kernel_dgeqrf_4_lib4.o \
		kernel_dgecp_lib4.o \
		kernel_dgetr_lib4.o \
		kernel_dpack_lib4.o \
		kernel_dpack_buffer_lib4.o \
		kernel_dger_lib4.o \
		kernel_ddot_lib.o \
		kernel_daxpy_lib.o \
		kernel_dgetr_lib.o \
		\
		kernel_sgemm_4x4_lib4.o \
		kernel_sgemm_diag_lib4.o \
		kernel_sgemv_4_lib4.o \
		kernel_ssymv_4_lib4.o \
		kernel_sgetrf_pivot_lib4.o \
		kernel_sgecp_lib4.o \
		kernel_sgetr_lib4.o \
		kernel_spack_lib4.o \
		kernel_sdot_lib.o \
		kernel_saxpy_lib.o \
		\
		kernel_align_generic.o \
		\
		kernel_d_aux_lib.o \

endif



OBJS =

ifeq ($(LA), HIGH_PERFORMANCE)
OBJS += $(KERNEL_OBJS)
endif # LA choice

ifeq ($(BLASFEO_HP_API), 1)
OBJS += $(KERNEL_OBJS)
endif # LA choice


obj: $(OBJS)

clean:
	rm -f *.o
	rm -f *.s
	rm -f *.gcda
	rm -f *.gcno
	rm -f *.gcov

kernel_dgemm_4x4_lib4.o: kernel_dgemm_4x4_lib4.c kernel_dgemm_4x4_lib.c
kernel_dgemv_4_lib4.o: kernel_dgemv_4_lib4.c kernel_dgemv_4_lib.c
kernel_dsymv_4_lib4.o: kernel_dsymv_4_lib4.c kernel_dsymv_4_lib.c
kernel_dger_lib4.o: kernel_dger_lib4.c kernel_dger_lib.c
kernel_sgemm_4x4_lib4.o: kernel_sgemm_4x4_lib4.c kernel_sgemm_4x4_lib.c
