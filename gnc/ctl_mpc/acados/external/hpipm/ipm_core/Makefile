###################################################################################################
#                                                                                                 #
# This file is part of HPIPM.                                                                     #
#                                                                                                 #
# HPIPM -- High-Performance Interior Point Method.                                                #
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

ifeq ($(TARGET), AVX512)
OBJS += d_core_qp_ipm_aux_avx512.o
OBJS += s_core_qp_ipm_aux_avx512.o
endif
ifeq ($(TARGET), AVX)
OBJS += d_core_qp_ipm_aux_avx.o
OBJS += s_core_qp_ipm_aux_avx.o
endif
ifeq ($(TARGET), GENERIC)
OBJS += d_core_qp_ipm_aux.o
OBJS += s_core_qp_ipm_aux.o
endif

OBJS += d_core_qp_ipm.o
OBJS += s_core_qp_ipm.o

obj: $(OBJS)

clean:
	rm -f *.o
	rm -f *.s

d_core_qp_ipm.o: d_core_qp_ipm.c x_core_qp_ipm.c
s_core_qp_ipm.o: s_core_qp_ipm.c x_core_qp_ipm.c
d_core_qp_ipm_aux.o: d_core_qp_ipm_aux.c x_core_qp_ipm_aux.c
s_core_qp_ipm_aux.o: s_core_qp_ipm_aux.c x_core_qp_ipm_aux.c
