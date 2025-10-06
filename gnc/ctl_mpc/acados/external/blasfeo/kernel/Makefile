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

obj: $(OBJS)
ifeq ($(TARGET), X64_INTEL_SKYLAKE_X)
	( cd avx512; $(MAKE) obj)
	( cd avx2; $(MAKE) obj)
	( cd avx; $(MAKE) obj)
	( cd sse3; $(MAKE) obj)
endif

ifeq ($(TARGET), X64_INTEL_HASWELL)
	( cd avx2; $(MAKE) obj)
	( cd avx; $(MAKE) obj)
	( cd sse3; $(MAKE) obj)
	( cd generic; $(MAKE) obj)
endif

ifeq ($(TARGET), X64_INTEL_SANDY_BRIDGE)
	( cd avx; $(MAKE) obj)
	( cd sse3; $(MAKE) obj)
	( cd generic; $(MAKE) obj)
endif

ifeq ($(TARGET), X64_INTEL_CORE)
	( cd sse3; $(MAKE) obj)
	( cd generic; $(MAKE) obj)
endif

ifeq ($(TARGET), X64_AMD_BULLDOZER)
	( cd fma; $(MAKE) obj)
	( cd sse3; $(MAKE) obj)
	( cd generic; $(MAKE) obj)
endif

ifeq ($(TARGET), X86_AMD_JAGUAR)
	( cd avx_x86; $(MAKE) obj)
	( cd sse3_x86; $(MAKE) obj)
	( cd generic; $(MAKE) obj)
endif

ifeq ($(TARGET), X86_AMD_BARCELONA)
	( cd sse3_x86; $(MAKE) obj)
	( cd generic; $(MAKE) obj)
endif

ifeq ($(TARGET), $(filter $(TARGET), ARMV8A_APPLE_M1 ARMV8A_ARM_CORTEX_A76 ARMV8A_ARM_CORTEX_A73 ARMV8A_ARM_CORTEX_A57 ARMV8A_ARM_CORTEX_A55 ARMV8A_ARM_CORTEX_A53))
	( cd armv8a; $(MAKE) obj)
	( cd generic; $(MAKE) obj)
endif

ifeq ($(TARGET), $(filter $(TARGET), ARMV7A_ARM_CORTEX_A15 ARMV7A_ARM_CORTEX_A9 ARMV7A_ARM_CORTEX_A7))
	( cd armv7a; $(MAKE) obj)
	( cd generic; $(MAKE) obj)
endif

ifeq ($(TARGET), GENERIC)
	( cd generic; $(MAKE) obj)
endif

clean:
	rm -f *.o
	rm -f *.s
	rm -f *.gcda
	rm -f *.gcno
	rm -f *.gcov
	make -C avx512 clean
	make -C avx2 clean
	make -C avx clean
	make -C avx_x86 clean
	make -C sse3 clean
	make -C sse3_x86 clean
	make -C fma clean
	make -C armv8a clean
	make -C armv7a clean
	make -C generic clean

