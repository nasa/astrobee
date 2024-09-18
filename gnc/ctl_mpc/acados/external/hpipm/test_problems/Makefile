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
include ../Makefile.external_blas

PRINT = 1
CFLAGS += -DPRINT=$(PRINT)

LIBS = $(TOP)/lib/libhpipm.a $(BLASFEO_PATH)/lib/libblasfeo.a $(LIBS_EXTERNAL_BLAS)


### select one single test ###

#OBJS_TEST = d_tools.o test_d_cond.o
#OBJS_TEST = d_tools.o test_d_cond_qcqp.o
#OBJS_TEST = d_tools.o test_d_cast_qcqp.o
#OBJS_TEST = d_tools.o test_d_part_cond.o
#OBJS_TEST = d_tools.o test_d_part_cond_qcqp.o
#OBJS_TEST = test_d_dense.o
#OBJS_TEST = test_d_dense_qcqp.o
OBJS_TEST = d_tools.o test_d_ocp.o
#OBJS_TEST = d_tools.o test_d_ocp_qcqp.o
#OBJS_TEST = d_tools.o test_d_ocp_phase_I.o
#OBJS_TEST = d_tools.o test_d_tree_ocp.o
#OBJS_TEST = d_tools.o test_d_tree_ocp_qcqp.o
#OBJS_TEST = d_tools.o test_d_sim.o

obj: $(OBJS_TEST)
	$(CC) -o test.out $(OBJS_TEST) $(LIBS) #-pg


### all individual tests ###

# mass spring system
test_d_ocp \
test_d_ocp_qcqp \
test_d_cond \
test_d_cond_qcqp \
test_d_part_cond \
test_d_part_cond_qcqp \
test_d_tree_ocp \
test_d_tree_ocp_qcqp \
test_d_sim \
: %: d_tools.o %.o
	$(CC) -o $@.out $^ $(LIBS)
	./$@.out

# stand-alone
test_d_dense \
test_d_dense_qcqp \
: %: %.o
	$(CC) -o $@.out $< $(LIBS)
	./$@.out


### clean ###
clean:
	rm -f *.o
	rm -f *.out
	rm -f libhpipm.a

