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

CC = gcc

CFLAGS = -O2 -fPIC -mavx2 -DOS_LINUX
ASFLAGS = -mavx2 -DOS_LINUX

#OBJS = kernel/avx/kernel_dgemm_4x4_lib.o tests/test.o
OBJS = kernel/avx2/kernel_dgemm_12x4_lib.o kernel/avx2/kernel_dgemm_8x4_lib.o kernel/avx2/kernel_dgemm_4x4_lib.o kernel/avx2/kernel_dpatr_lib4.o kernel/avx2/kernel_dpack_lib4.o blas/dgemm.o blas/dpotrf.o

#OBJS += tests/test.o
OBJS += tests/benchmark.o

build: $(OBJS)
	$(CC) $(OBJS) /home/gianluca/blasfeo/lib/libblasfeo.a /opt/openblas/lib/libopenblas.a -lm -o tests/test.out
	@echo ""
	@echo "Done!"
	@echo ""

run:
	mkdir -p tests/data
#	./tests/test.out
	./tests/test.out > tests/data/blas.m

print:
	cp tests/print_blas.m tests/data
	cd tests/data; octave-cli print_blas.m
	cd tests/data/ ; epstopdf blas.eps -out blas.pdf ; rm blas.eps

clean:
	rm -f *.o
