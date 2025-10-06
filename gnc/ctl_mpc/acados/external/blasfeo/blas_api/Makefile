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

ifeq ($(BLAS_API), 1)

OBJS += dcopy.o
OBJS += daxpy.o
OBJS += ddot.o
OBJS += dgemm_ref.o # XXX
OBJS += dsyrk_ref.o # XXX
OBJS += dtrmm_ref.o # XXX
OBJS += dtrsm_ref.o # XXX
OBJS += dsyr2k_ref.o # XXX
OBJS += dgesv.o
OBJS += dgetrf_ref.o # XXX
OBJS += dgetrs.o
OBJS += dlaswp.o
OBJS += dposv.o
OBJS += dpotrf_ref.o # XXX
OBJS += dpotrs.o
OBJS += dtrtrs.o
OBJS += dgetr_ref.o # XXX
OBJS += dgemv_ref.o # XXX
OBJS += dsymv_ref.o # XXX
OBJS += dger_ref.o # XXX

OBJS += saxpy.o
OBJS += sdot.o
OBJS += sgemm_ref.o # XXX
OBJS += strsm_ref.o # XXX
OBJS += spotrf_ref.o # XXX


endif # BLAS_API

obj: $(OBJS)
ifeq ($(EXPERIMENTAL), 1)
	( cd experimental; $(MAKE) obj)
endif

clean:
	make -C experimental clean
	rm -f *.o
	rm -f *.s
	rm -f *.gcda
	rm -f *.gcno
	rm -f *.gcov

dgemm_ref.o: dgemm_ref.c xgemm_ref.c ../blasfeo_hp_cm/dgemm.c
dsyrk_ref.o: dsyrk_ref.c xsyrk_ref.c
dtrmm_ref.o: dtrmm_ref.c xtrmm_ref.c
dtrsm_ref.o: dtrsm_ref.c xtrsm_ref.c
dsyr2k_ref.o: dsyr2k_ref.c xsyr2k_ref.c
dpotrf_ref.o: dpotrf_ref.c xpotrf_ref.c
dgetrf_ref.o: dgetrf_ref.c xgetrf_ref.c
dgetr_ref.o: dgetr_ref.c xgetr_ref.c
dgemv_ref.o: dgemv_ref.c xgemv_ref.c
dsymv_ref.o: dsymv_ref.c xsymv_ref.c
dger_ref.o: dger_ref.c xger_ref.c

sgemm_ref.o: sgemm_ref.c xgemm_ref.c ../blasfeo_hp_cm/sgemm.c
strsm_ref.o: strsm_ref.c xtrsm_ref.c
spotrf_ref.o: spotrf_ref.c xpotrf_ref.c
