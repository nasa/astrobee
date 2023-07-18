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

include ../../Makefile.rule

OBJS = 

ifeq ($(TARGET), X64_AVX)
OBJS += 
OBJS += 
endif

ifeq ($(TARGET), X64_AVX)
OBJS += 
OBJS += 
endif

ifeq ($(TARGET), C99_4X4)
ifeq ($(USE_BLASFEO), 1)
else
OBJS += kernel_dgemm_c99_lib4.o kernel_dtrmm_c99_lib4.o kernel_dtrsm_c99_lib4.o kernel_dsyrk_c99_lib4.o kernel_dpotrf_c99_lib4.o kernel_dgemv_c99_lib4.o kernel_dtrmv_c99_lib4.o kernel_dtrsv_c99_lib4.o kernel_dsymv_c99_lib4.o kernel_dtran_c99_lib4.o kernel_dttmm_c99_lib4.o kernel_dtrinv_c99_lib4.o kernel_dcopy_c99_lib4.o kernel_dgetrf_c99_lib4.o
OBJS += 
endif
endif

ifeq ($(TARGET), X64_SSE3)
ifeq ($(USE_BLASFEO), 1)
else
OBJS += kernel_dgemv_c99_lib4.o kernel_dtrmv_c99_lib4.o kernel_dtrsv_c99_lib4.o kernel_dsymv_c99_lib4.o kernel_dtran_c99_lib4.o kernel_dttmm_c99_lib4.o kernel_dtrinv_c99_lib4.o kernel_dcopy_c99_lib4.o kernel_dgetrf_c99_lib4.o
OBJS += 
endif
endif

ifeq ($(TARGET), CORTEX_A57)
ifeq ($(USE_BLASFEO), 1)
else
OBJS += kernel_dtrmm_c99_lib4.o kernel_dtrsm_c99_lib4.o kernel_dsyrk_c99_lib4.o kernel_dpotrf_c99_lib4.o kernel_dgemv_c99_lib4.o kernel_dtrmv_c99_lib4.o kernel_dtrsv_c99_lib4.o kernel_dsymv_c99_lib4.o kernel_dtran_c99_lib4.o kernel_dttmm_c99_lib4.o kernel_dtrinv_c99_lib4.o kernel_dcopy_c99_lib4.o kernel_dgetrf_c99_lib4.o
OBJS += 
endif
endif

ifeq ($(TARGET), CORTEX_A15)
ifeq ($(USE_BLASFEO), 1)
else
OBJS += kernel_dsymv_c99_lib4.o kernel_dtran_c99_lib4.o kernel_dttmm_c99_lib4.o kernel_dtrinv_c99_lib4.o kernel_dcopy_c99_lib4.o kernel_dgetrf_c99_lib4.o
OBJS += 
endif
endif

ifeq ($(TARGET), CORTEX_A9)
ifeq ($(USE_BLASFEO), 1)
else
OBJS += kernel_dsymv_c99_lib4.o kernel_dtran_c99_lib4.o kernel_dttmm_c99_lib4.o kernel_dtrinv_c99_lib4.o kernel_dcopy_c99_lib4.o kernel_dgetrf_c99_lib4.o
OBJS += 
endif
endif

ifeq ($(TARGET), CORTEX_A7)
ifeq ($(USE_BLASFEO), 1)
else
OBJS += kernel_dsymv_c99_lib4.o kernel_dtran_c99_lib4.o kernel_dttmm_c99_lib4.o kernel_dtrinv_c99_lib4.o kernel_dcopy_c99_lib4.o kernel_dgetrf_c99_lib4.o
OBJS += 
endif
endif

obj: $(OBJS)

clean:
	rm -f *.o

