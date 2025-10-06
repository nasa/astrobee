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

OBJS =

OBJS += blasfeo_stdlib.o \
        blasfeo_processor_features.o \
        memory.o \
		d_aux_common.o \
		s_aux_common.o

ifeq ($(LA), HIGH_PERFORMANCE)

ifeq ($(MF), PANELMAJ)

ifeq ($(TARGET), X64_INTEL_SKYLAKE_X)
OBJS += d_aux_lib8.o
OBJS += s_aux_lib16.o
#OBJS += m_aux_lib48.o
endif

ifeq ($(TARGET), $(filter $(TARGET), X64_INTEL_HASWELL X64_INTEL_SANDY_BRIDGE))
OBJS += d_aux_lib4.o
OBJS += s_aux_lib8.o
OBJS += m_aux_lib48.o
endif

ifeq ($(TARGET), $(filter $(TARGET), X64_INTEL_CORE X64_AMD_BULLDOZER X86_AMD_JAGUAR X86_AMD_BARCELONA ARMV8A_APPLE_M1 ARMV8A_ARM_CORTEX_A76 ARMV8A_ARM_CORTEX_A73 ARMV8A_ARM_CORTEX_A57 ARMV8A_ARM_CORTEX_A55 ARMV8A_ARM_CORTEX_A53 ARMV7A_ARM_CORTEX_A15 ARMV7A_ARM_CORTEX_A9 ARMV7A_ARM_CORTEX_A7 GENERIC))
OBJS += d_aux_lib4.o
OBJS += s_aux_lib4.o
OBJS += m_aux_lib44.o
endif

else # MF COLMAJ

# TODO optimized hp cm version
OBJS += d_aux_hp_cm.o
OBJS += s_aux_hp_cm.o

endif # MF choice

ifeq ($(BLASFEO_REF_API), 1)
OBJS += d_aux_ref.o
OBJS += s_aux_ref.o
endif

else # LA_REFERENCE | LA_EXTERNAL_BLAS_WRAPPER

OBJS += d_aux_ref.o
OBJS += s_aux_ref.o
OBJS += m_aux_lib.o

endif # LA choice


#ext dep
ifeq ($(EXT_DEP), 1)

OBJS += d_aux_ext_dep_common.o
OBJS += s_aux_ext_dep_common.o
OBJS += d_aux_ext_dep.o
OBJS += s_aux_ext_dep.o
OBJS += v_aux_ext_dep_lib.o
OBJS += i_aux_ext_dep_lib.o
OBJS += timing.o

endif


obj: $(OBJS)

clean:
	rm -f *.o
	rm -f *.s

d_aux_ref.o: d_aux_ref.c x_aux_ref.c
s_aux_ref.o: s_aux_ref.c x_aux_ref.c
d_aux_ext_dep_common.o: d_aux_ext_dep_common.c x_aux_ext_dep_common.c
s_aux_ext_dep_common.o: s_aux_ext_dep_common.c x_aux_ext_dep_common.c
d_aux_ext_dep.o: d_aux_ext_dep.c x_aux_ext_dep.c
s_aux_ext_dep.o: s_aux_ext_dep.c x_aux_ext_dep.c

d_aux_hp_cm.o: d_aux_hp_cm.c x_aux_ref.c
s_aux_hp_cm.o: s_aux_hp_cm.c x_aux_ref.c
