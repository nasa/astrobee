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

include ../Makefile.rule


HP_OBJS =

ifeq ($(TARGET), X64_INTEL_SKYLAKE_X)
#
HP_OBJS += d_blas1_lib8.o
HP_OBJS += d_blas2_lib8.o
HP_OBJS += d_blas2_diag_lib.o
HP_OBJS += d_blas3_lib8.o
HP_OBJS += d_blas3_diag_lib8.o
HP_OBJS += d_lapack_lib8.o
#
HP_OBJS += s_blas1_lib16.o
HP_OBJS += s_blas2_lib16.o
HP_OBJS += s_blas2_diag_lib.o
HP_OBJS += s_blas3_lib16.o
HP_OBJS += s_blas3_diag_lib16.o
HP_OBJS += s_lapack_lib16.o
endif

ifeq ($(TARGET), $(filter $(TARGET), X64_INTEL_HASWELL X64_INTEL_SANDY_BRIDGE))
#
HP_OBJS += d_blas1_lib4.o
HP_OBJS += d_blas2_lib4.o
HP_OBJS += d_blas2_diag_lib.o
HP_OBJS += d_blas3_lib4.o
HP_OBJS += d_blas3_diag_lib4.o
HP_OBJS += d_lapack_lib4.o
#
HP_OBJS += s_blas1_lib8.o
HP_OBJS += s_blas2_lib8.o
HP_OBJS += s_blas2_diag_lib.o
HP_OBJS += s_blas3_lib8.o
HP_OBJS += s_blas3_diag_lib8.o
HP_OBJS += s_lapack_lib8.o
endif

ifeq ($(TARGET), $(filter $(TARGET), X64_INTEL_CORE X64_AMD_BULLDOZER X86_AMD_JAGUAR X86_AMD_BARCELONA ARMV8A_APPLE_M1 ARMV8A_ARM_CORTEX_A76 ARMV8A_ARM_CORTEX_A73 ARMV8A_ARM_CORTEX_A57 ARMV8A_ARM_CORTEX_A55 ARMV8A_ARM_CORTEX_A53 ARMV7A_ARM_CORTEX_A15 ARMV7A_ARM_CORTEX_A9 ARMV7A_ARM_CORTEX_A7 GENERIC))
#
HP_OBJS += d_blas1_lib4.o
HP_OBJS += d_blas2_lib4.o
HP_OBJS += d_blas2_diag_lib.o
HP_OBJS += d_blas3_lib4.o
HP_OBJS += d_blas3_diag_lib4.o
HP_OBJS += d_lapack_lib4.o
#
HP_OBJS += s_blas1_lib4.o
HP_OBJS += s_blas2_lib4.o
HP_OBJS += s_blas2_diag_lib.o
HP_OBJS += s_blas3_lib4.o
HP_OBJS += s_blas3_diag_lib4.o
HP_OBJS += s_lapack_lib4.o
endif


OBJS =

ifeq ($(MF), PANELMAJ)

ifeq ($(LA), HIGH_PERFORMANCE)
OBJS += $(HP_OBJS)
else

ifeq ($(BLASFEO_HP_API), 1)
OBJS += $(HP_OBJS)
endif

endif # LA HP

endif # MF PANELMAJ


obj: $(OBJS)

clean:
	rm -f *.o
	rm -f *.s

d_blas2_diag_lib.o: d_blas2_diag_lib.c x_blas2_diag_lib.c
s_blas2_diag_lib.o: s_blas2_diag_lib.c x_blas2_diag_lib.c
