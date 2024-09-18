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

LIBS =

LIBS += $(BINARY_DIR)/libblasfeo.a

#include ../Makefile.external_blas
LIBS += $(LIBS_EXTERNAL_BLAS)

LIBS += -lm
#LIBS += -fopenmp

#ifeq ($(EXTERNAL_BLAS), 0)
#get_refblas:
#	$(info )
#	$(info ERROR EXTERNAL_BLAS is not set or invalid)
#	$(info )
#	$(error .)
#else
get_refblas:
	@echo Using EXTERNAL_BLAS=$(EXTERNAL_BLAS)
#endif

ifdef GHZ_MAX
CFLAGS += -DGHZ_MAX=$(GHZ_MAX)
get_cpu_freq:
	@echo Using GHZ_MAX=$(GHZ_MAX) from command line option
else
ifeq (,$(wildcard ./cpu_freq.h))
get_cpu_freq:
	$(info )
	$(info ERROR GHZ_MAX is not set, create cpu_freq.h or specify GHZ_MAX Makefile option)
	$(info )
	$(error .)
else
get_cpu_freq:
	@echo Using GHZ_MAX from cpu_freq.h
endif
endif

# all benchmarks
ONE_OBJS = benchmark_d_blasfeo_api.o
#ONE_OBJS = benchmark_s_blasfeo_api.o
#ONE_OBJS = benchmark_d_blas_api.o
#ONE_OBJS = benchmark_s_blas_api.o

%.o: %.c
	# build obj
	$(CC) $(CFLAGS) -c $< -o $(BINARY_DIR)/$@
	$(CC) $(CFLAGS) $(BINARY_DIR)/$@ -o $(BINARY_DIR)/$@.out $(LIBS)

# ----------- Targets

bin_dir:
	# create bin folder if not existent
	mkdir -p $(BINARY_DIR)/
	mkdir -p $(BINARY_DIR)/BLASFEO_API/
ifeq ($(BLAS_API), 1)
	mkdir -p $(BINARY_DIR)/BLAS_API/
endif

common: get_refblas get_cpu_freq bin_dir
	cp ../lib/libblasfeo.a ./$(BINARY_DIR)
#	echo $(OBJS)

.DEFAULT_GOAL := build
.PHONY: build

build: common $(ONE_OBJS)

run:
	./$(BINARY_DIR)/$(ONE_OBJS).out

perf:
	perf stat -e cpu-clock,instructions,cpu-cycles,bus-cycles,cache-misses,cache-references,L1-dcache-load-misses,L1-dcache-loads,L1-dcache-stores,LLC-load-misses,LLC-loads,LLC-stores,LLC-store-misses,dTLB-load-misses,dTLB-loads,dTLB-stores,dTLB-store-misses ./$(BINARY_DIR)/$(ONE_OBJS).out

adb_push:
	adb push ./$(BINARY_DIR)/$(ONE_OBJS).out /data/local/tmp/$(ONE_OBJS).out

adb_run:
	adb shell /data/local/tmp/$(ONE_OBJS).out

adb_run_taskset:
	adb shell taskset 2 /data/local/tmp/$(ONE_OBJS).out

clean:
	rm -rf ./*.o
	rm -rf ./*.out
	rm -rf ./$(BINARY_DIR)/BLASFEO_API/*.o
	rm -rf ./$(BINARY_DIR)/BLASFEO_API/*.out
	rm -rf ./$(BINARY_DIR)/BLAS_API/*.o
	rm -rf ./$(BINARY_DIR)/BLAS_API/*.out
	rm -rf ./$(BINARY_DIR)/libblasfeo.a

deep_clean: clean
	rm -rf ./figures/
	rm -rf ./build/



# figures

figures_benchmark_one:
	cp figures_benchmark_one.m ./build/
	cd ./build/ ; octave-cli figures_benchmark_one.m



# benchmark BLASFEO API
include Makefile.blasfeo_api



# benchmark BLAS API
include Makefile.blas_api
