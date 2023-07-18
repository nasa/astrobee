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

LIBS =

#LIBS += libblasfeo.a
LIBS += $(CURRENT_DIR)/lib/libblasfeo.a

# add different link library for different EXTERNAL_BLAS implementation
#include ../Makefile.external_blas
LIBS += $(LIBS_EXTERNAL_BLAS)

ifeq ($(COMPLEMENT_WITH_NETLIB_BLAS), 1)
LIBS += -lgfortran
endif

LIBS += -lm


### select one single test ###

## dgemm kernel
# OBJS = example_kernel_dgemm.o
# OBJS = kernel_dgemm.o

## factorization
# OBJS = example_d_lu_factorization.o
# OBJS = example_d_lq_factorization.o
# OBJS = example_s_lu_factorization.o

## riccati
OBJS = example_d_riccati_recursion.o tools.o
# OBJS = example_s_riccati_recursion.o tools.o

## getting started
# OBJS = getting_started.o

### all individual tests ###

getting_started \
example_d_lu_factorization \
example_d_lq_factorization \
example_s_lu_factorization \
: %: %.o
	$(CC) -o $@.out $^ $(LIBS)
	./$@.out

example_d_riccati_recursion \
example_s_riccati_recursion \
: %: tools.o %.o
	$(CC) -o $@.out $^ $(LIBS)
	./$@.out



CFLAGS += -I../include/


build: common $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) $(LIBS) -o example.out

common:
	cp ../lib/libblasfeo.a .

run:
	./example.out

adb_push:
	adb push example.out /data/local/tmp/example.out

adb_run:
	adb shell /data/local/tmp/example.out

clean:
	rm -rf ./*.o
	rm -rf ./*.out

deep_clean: clean
