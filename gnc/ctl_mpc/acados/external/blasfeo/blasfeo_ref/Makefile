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


REF_OBJS =
#
REF_OBJS += d_blas1_ref.o
REF_OBJS += d_blas2_ref.o
REF_OBJS += d_blas2_diag_ref.o
REF_OBJS += d_blas3_ref.o
REF_OBJS += d_blas3_diag_ref.o
REF_OBJS += d_lapack_ref.o
#
REF_OBJS += s_blas1_ref.o
REF_OBJS += s_blas2_ref.o
REF_OBJS += s_blas2_diag_ref.o
REF_OBJS += s_blas3_ref.o
REF_OBJS += s_blas3_diag_ref.o
REF_OBJS += s_lapack_ref.o


HP_CM_OBJS =
#
HP_CM_OBJS += d_blas1_hp_cm.o
HP_CM_OBJS += d_blas2_hp_cm.o
HP_CM_OBJS += d_blas2_diag_hp_cm.o
HP_CM_OBJS += d_blas3_hp_cm.o
HP_CM_OBJS += d_blas3_diag_hp_cm.o
HP_CM_OBJS += d_lapack_hp_cm.o
#
HP_CM_OBJS += s_blas1_hp_cm.o
HP_CM_OBJS += s_blas2_hp_cm.o
HP_CM_OBJS += s_blas2_diag_hp_cm.o
HP_CM_OBJS += s_blas3_hp_cm.o
HP_CM_OBJS += s_blas3_diag_hp_cm.o
HP_CM_OBJS += s_lapack_hp_cm.o


REF_BLAS_OBJS =
#
REF_BLAS_OBJS += d_blas3_ref_blas.o
REF_BLAS_OBJS += d_lapack_ref_blas.o
#
REF_BLAS_OBJS += s_blas3_ref_blas.o


OBJS =

# REFERENCE
ifeq ($(LA), REFERENCE)
OBJS += $(REF_OBJS)
else

ifeq ($(BLASFEO_REF_API), 1)
OBJS += $(REF_OBJS)
endif

endif

# HIGH-PERFORMANCE COLMAJ
ifeq ($(MF), COLMAJ)

ifeq ($(LA), HIGH_PERFORMANCE)
OBJS += $(HP_CM_OBJS)
else

ifeq ($(BLASFEO_HP_API), 1)
OBJS += $(HP_CM_OBJS)
endif

endif # LA HP

endif # MF COLMAJ

# BLAS API REFERENCE PANELMAJ
ifeq ($(BLAS_API), 1)
ifeq ($(LA), REFERENCE)
ifeq ($(MF), PANELMAJ)
OBJS += $(REF_BLAS_OBJS)
endif
endif
endif


obj: $(OBJS)

clean:
	rm -f *.o
	rm -f *.s

d_blas1_ref.o: d_blas1_ref.c x_blas1_ref.c
d_blas2_ref.o: d_blas2_ref.c x_blas2_ref.c
d_blas2_diag_ref.o: d_blas2_diag_ref.c x_blas2_diag_ref.c
d_blas3_ref.o: d_blas3_ref.c x_blas3_ref.c
d_blas3_diag_ref.o: d_blas3_diag_ref.c x_blas3_diag_ref.c
d_lapack_ref.o: d_lapack_ref.c x_lapack_ref.c

s_blas1_ref.o: s_blas1_ref.c x_blas1_ref.c
s_blas2_ref.o: s_blas2_ref.c x_blas2_ref.c
s_blas2_diag_ref.o: s_blas2_diag_ref.c x_blas2_diag_ref.c
s_blas3_ref.o: s_blas3_ref.c x_blas3_ref.c
s_blas3_diag_ref.o: s_blas3_diag_ref.c x_blas3_diag_ref.c
s_lapack_ref.o: s_lapack_ref.c x_lapack_ref.c


d_blas1_hp_cm.o: d_blas1_hp_cm.c x_blas1_ref.c
d_blas2_hp_cm.o: d_blas2_hp_cm.c x_blas2_ref.c
d_blas2_diag_hp_cm.o: d_blas2_diag_hp_cm.c x_blas2_diag_ref.c
d_blas3_hp_cm.o: d_blas3_hp_cm.c x_blas3_ref.c
d_blas3_diag_hp_cm.o: d_blas3_diag_hp_cm.c x_blas3_diag_ref.c
d_lapack_hp_cm.o: d_lapack_hp_cm.c x_lapack_ref.c

s_blas1_hp_cm.o: s_blas1_hp_cm.c x_blas1_ref.c
s_blas2_hp_cm.o: s_blas2_hp_cm.c x_blas2_ref.c
s_blas2_diag_hp_cm.o: s_blas2_diag_hp_cm.c x_blas2_diag_ref.c
s_blas3_hp_cm.o: s_blas3_hp_cm.c x_blas3_ref.c
s_blas3_diag_hp_cm.o: s_blas3_diag_hp_cm.c x_blas3_diag_ref.c
s_lapack_hp_cm.o: s_lapack_hp_cm.c x_lapack_ref.c


d_blas3_ref_blas.o: d_blas3_ref_blas.c x_blas3_ref.c
d_lapack_ref_blas.o: d_lapack_ref_blas.c x_lapack_ref.c

s_blas3_ref_blas.o: s_blas3_ref_blas.c x_blas3_ref.c
