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

include ../../Makefile.rule
include ../../Makefile.external_blas

LIBS = $(TOP)/lib/libhpipm.a $(BLASFEO_PATH)/lib/libblasfeo.a $(LIBS_EXTERNAL_BLAS)


### select data file ###

OCP_QP_DATA = data/getting_started_data.o
#OCP_QP_DATA = data/ocp_qp_data.o 
#OCP_QP_DATA = ../matlab_octave/ocp_qp_data.o
#OCP_QP_DATA = ../python/ocp_qp_data.o
OCP_QCQP_DATA = data/pmsm_ocp_qcqp_data.o
#OCP_QCQP_DATA = data/ocp_qcqp_data.o 


### select one single example ###

#OBJS_EXMP = $(OCP_QP_DATA) example_d_ocp_qp_unconstr.o
OBJS_EXMP = $(OCP_QP_DATA) example_d_ocp_qp.o
#OBJS_EXMP = $(OCP_QP_DATA) example_d_ocp_qp_x0emb.o
#OBJS_EXMP = $(OCP_QP_DATA) example_d_ocp_qp_part_cond.o
#OBJS_EXMP = $(OCP_QP_DATA) example_d_ocp_qp_x0emb_part_cond.o
#OBJS_EXMP = $(OCP_QCQP_DATA) example_d_ocp_qcqp_part_cond.o
#OBJS_EXMP = $(OCP_QP_DATA) example_d_ocp_qp_sens.o
#OBJS_EXMP = $(OCP_QP_DATA) example_d_ocp_qp_part_cond_sens.o

obj: $(OBJS_EXMP)
	$(CC) -o  example.out $(OBJS_EXMP) $(LIBS)


### all individual tests ###

# ocp qp data
example_d_ocp_qp_unconstr \
example_d_ocp_qp \
example_d_ocp_qp_part_cond \
example_d_ocp_qp_sens \
example_d_ocp_qp_part_cond_sens \
: %: $(OCP_QP_DATA) %.o
	$(CC) -o $@.out $^ $(LIBS)
	./$@.out

# ocp qcqp data
example_d_ocp_qcqp_part_cond \
: %: $(OCP_QCQP_DATA) %.o
	$(CC) -o $@.out $^ $(LIBS)
	./$@.out


### clean ###
clean:
	rm -f *.o
	rm -f data/*.o
	rm -f *.out
	rm -f libhpipm.a


