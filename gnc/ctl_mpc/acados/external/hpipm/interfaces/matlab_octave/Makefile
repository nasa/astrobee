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

OBJS = \
	ocp_qp_dim_create.mex \
	ocp_qp_dim_destroy.mex \
	ocp_qp_dim_set.mex \
	ocp_qp_dim_get.mex \
	ocp_qp_dim_print.mex \
	ocp_qp_dim_codegen.mex \
	ocp_qp_create.mex \
	ocp_qp_destroy.mex \
	ocp_qp_set.mex \
	ocp_qp_get.mex \
	ocp_qp_print.mex \
	ocp_qp_codegen.mex \
	ocp_qp_sol_create.mex \
	ocp_qp_sol_destroy.mex \
	ocp_qp_sol_get.mex \
	ocp_qp_sol_set.mex \
	ocp_qp_sol_print.mex \
	ocp_qp_solver_arg_create.mex \
	ocp_qp_solver_arg_destroy.mex \
	ocp_qp_solver_arg_set.mex \
	ocp_qp_solver_arg_codegen.mex \
	ocp_qp_solver_create.mex \
	ocp_qp_solver_destroy.mex \
	ocp_qp_solver_solve.mex \
	ocp_qp_solver_get.mex \
	ocp_qp_solver_get_ric.mex \
	\
	ocp_qcqp_dim_create.mex \
	ocp_qcqp_dim_destroy.mex \
	ocp_qcqp_dim_set.mex \
	ocp_qcqp_dim_get.mex \
	ocp_qcqp_dim_print.mex \
	ocp_qcqp_dim_codegen.mex \
	ocp_qcqp_create.mex \
	ocp_qcqp_destroy.mex \
	ocp_qcqp_set.mex \
	ocp_qcqp_get.mex \
	ocp_qcqp_print.mex \
	ocp_qcqp_codegen.mex \
	ocp_qcqp_sol_create.mex \
	ocp_qcqp_sol_destroy.mex \
	ocp_qcqp_sol_get.mex \
	ocp_qcqp_sol_set.mex \
	ocp_qcqp_sol_print.mex \
	ocp_qcqp_solver_arg_create.mex \
	ocp_qcqp_solver_arg_destroy.mex \
	ocp_qcqp_solver_arg_set.mex \
	ocp_qcqp_solver_arg_codegen.mex \
	ocp_qcqp_solver_create.mex \
	ocp_qcqp_solver_destroy.mex \
	ocp_qcqp_solver_solve.mex \
	ocp_qcqp_solver_get.mex \
	\
	dense_qp_dim_create.mex \
	dense_qp_dim_destroy.mex \
	dense_qp_dim_set.mex \
	dense_qp_dim_print.mex \
	dense_qp_create.mex \
	dense_qp_destroy.mex \
	dense_qp_set.mex \
	dense_qp_print.mex \
	dense_qp_sol_create.mex \
	dense_qp_sol_destroy.mex \
	dense_qp_sol_get.mex \
	dense_qp_sol_print.mex \
	dense_qp_solver_arg_create.mex \
	dense_qp_solver_arg_destroy.mex \
	dense_qp_solver_arg_set.mex \
	dense_qp_solver_create.mex \
	dense_qp_solver_destroy.mex \
	dense_qp_solver_solve.mex \
	dense_qp_solver_get.mex \
	\
#	part_cond_arg_create.mex \
	part_cond_arg_destroy.mex \
	part_cond_arg_set.mex \

all: set_cflags_octave
	$(MAKE) compile_mex_with_octave

compile_mex_with_octave: $(OBJS)
#	octave-cli compile_mex_ocp_qp.m

set_cflags_octave:
	octave-cli set_cflags_octave.m

compile_mex_with_matlab:
	matlab -nodisplay -nosplash -nodesktop -r "run('compile_mex_all.m');exit;"
#	~/R2017a/bin/matlab -nodisplay -nosplash -nodesktop -r "run('compile_mex_all.m');exit;"
#	~/R2019a/bin/matlab -nodisplay -nosplash -nodesktop -r "run('compile_mex_all.m');exit;"

clean:
	rm -f *.so
	rm -f *.mex
	rm -f *.mexa64
	rm -f *.o
	rm -f cflags_octave.txt

%.mex : %.c
	octave-cli compile_mex_one.m $<
