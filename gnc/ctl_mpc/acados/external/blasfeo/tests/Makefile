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

# ----------- Include
include ../Makefile.rule

# ----------- Envs

LIBS =
SHARED_LIBS =

LIBS += $(BINARY_DIR)/libblasfeo.a
SHARED_LIBS += -Wl,-rpath=$(BINARY_DIR) -L $(BINARY_DIR) -lblasfeo

#include ../Makefile.external_blas
LIBS += $(LIBS_EXTERNAL_BLAS)
SHARED_LIBS += $(SHARED_LIBS_EXTERNAL_BLAS)

LIBS += -lm
SHARED_LIBS += -lm

# set test verbosity
ifdef VERBOSE
ifeq ($(VERBOSE), 0)
else
CFLAGS += -DVERBOSE=$(VERBOSE)
endif
endif

ifndef VERBOSE
CFLAGS += -DVERBOSE=0
endif
ifdef ROUTINE
CFLAGS += -DROUTINE=$(ROUTINE)
endif

ifdef ROUTINE_CLASS_C
CFLAGS += -DROUTINE_CLASS_C=$(ROUTINE_CLASS_C)
endif

ifeq ($(PRECISION), d)
CFLAGS += -DPRECISION_DOUBLE
endif
ifeq ($(PRECISION), s)
CFLAGS += -DPRECISION_SINGLE
endif

ifeq ($(ROUTINE_API), BLAS)
CFLAGS += -DTEST_BLAS_API
TEST_BLAS_API=1
endif

ifeq ($(TEST_BLAS_API), 1)
ifeq ($(EXTERNAL_BLAS), 0)
$(error No EXTERNAL_BLAS specified, install specify one reference blas implementation i.e. OPENBLAS)
endif
endif

# select one single test
ONE_OBJS = test_d_blasfeo_api.o
# ONE_OBJS = test_s_blasfeo_api.o
# ONE_OBJS = test_d_blas_api.o
# ONE_OBJS = test_s_blas_api.o
# ONE_OBJS = test_valgrind.o

OBJS = test.o

%.o: %.c
	#
	# build executable obj $(BINARY_DIR)/$@
	$(CC) $(CFLAGS) -c $< -o $(BINARY_DIR)/$@
	$(CC) $(CFLAGS) $(BINARY_DIR)/$@ -o $(BINARY_DIR)/$@.out $(LIBS)
	#

# ----------- Targets

.DEFAULT_GOAL := obj
.PHONY: obj

obj: one

bin_dir:
	mkdir -p $(BINARY_DIR)/

common: bin_dir
	cp ../lib/libblasfeo.a ./$(BINARY_DIR)

one: common $(ONE_OBJS)

run_one:
	./$(BINARY_DIR)/$(ONE_OBJS).out

build: $(OBJS)

run:
	./$(BINARY_DIR)/test.o.out
	#

update: build run

full: common build run

adb_push:
	adb push ./$(BINARY_DIR)/$(ONE_OBJS).out /data/local/tmp/$(ONE_OBJS).out

adb_run:
	adb shell /data/local/tmp/$(ONE_OBJS).out

sde_run:
	~/sde/sde64 -- ./$(BINARY_DIR)/$(ONE_OBJS).out

clean:
	rm -rf ./*.o
	rm -rf ./*.out
	rm -rf ./$(BINARY_DIR)/*

deep_clean: clean

purge: deep_clean
	rm -rf ./build/
	rm -rf ./reports/
