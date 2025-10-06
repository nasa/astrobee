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


include ./Makefile.rule

OBJS = 

ifeq ($(TARGET), AVX512)
# ipm core
# double
OBJS += ipm_core/d_core_qp_ipm_aux_avx512.o
# single
OBJS += ipm_core/s_core_qp_ipm_aux_avx512.o
endif

ifeq ($(TARGET), AVX)
# ipm core
# double
OBJS += ipm_core/d_core_qp_ipm_aux_avx.o
# single
OBJS += ipm_core/s_core_qp_ipm_aux_avx.o
endif

ifeq ($(TARGET), GENERIC)
# ipm core
# double
OBJS += ipm_core/d_core_qp_ipm_aux.o
# single
OBJS += ipm_core/s_core_qp_ipm_aux.o
endif

# cond
# double
OBJS += cond/d_cond_aux.o
OBJS += cond/d_cond.o
OBJS += cond/d_part_cond.o
OBJS += cond/d_cond_qcqp.o
OBJS += cond/d_cast_qcqp.o
OBJS += cond/d_part_cond_qcqp.o
# single
OBJS += cond/s_cond_aux.o
OBJS += cond/s_cond.o
OBJS += cond/s_part_cond.o
OBJS += cond/s_cond_qcqp.o
OBJS += cond/s_cast_qcqp.o
OBJS += cond/s_part_cond_qcqp.o

# dense qp
# double
OBJS += dense_qp/d_dense_qp_dim.o 
OBJS += dense_qp/d_dense_qp.o 
OBJS += dense_qp/d_dense_qp_sol.o 
OBJS += dense_qp/d_dense_qp_res.o 
OBJS += dense_qp/d_dense_qp_kkt.o 
OBJS += dense_qp/d_dense_qp_ipm.o
OBJS += dense_qp/d_dense_qp_utils.o
OBJS += dense_qp/d_dense_qcqp_dim.o 
OBJS += dense_qp/d_dense_qcqp.o 
OBJS += dense_qp/d_dense_qcqp_sol.o 
OBJS += dense_qp/d_dense_qcqp_res.o 
OBJS += dense_qp/d_dense_qcqp_ipm.o 
OBJS += dense_qp/d_dense_qcqp_utils.o 
# single
OBJS += dense_qp/s_dense_qp_dim.o 
OBJS += dense_qp/s_dense_qp.o 
OBJS += dense_qp/s_dense_qp_sol.o 
OBJS += dense_qp/s_dense_qp_res.o 
OBJS += dense_qp/s_dense_qp_kkt.o 
OBJS += dense_qp/s_dense_qp_ipm.o
OBJS += dense_qp/s_dense_qp_utils.o
OBJS += dense_qp/s_dense_qcqp_dim.o 
OBJS += dense_qp/s_dense_qcqp.o 
OBJS += dense_qp/s_dense_qcqp_sol.o 
OBJS += dense_qp/s_dense_qcqp_res.o 
OBJS += dense_qp/s_dense_qcqp_ipm.o 
OBJS += dense_qp/s_dense_qcqp_utils.o 
#mixed
OBJS += dense_qp/m_dense_qp_dim.o 
OBJS += dense_qp/m_dense_qp.o 

# ipm core
# double
#OBJS += ipm_core/d_core_qp_ipm_aux.o 
OBJS += ipm_core/d_core_qp_ipm.o
# single
#OBJS += ipm_core/s_core_qp_ipm_aux.o 
OBJS += ipm_core/s_core_qp_ipm.o

# ocp nlp
#OBJS += ocp_nlp/d_ocp_nlp.o ocp_nlp/d_ocp_nlp_sol.o ocp_nlp/d_ocp_nlp_aux.o ocp_nlp/d_ocp_nlp_hyb.o ocp_nlp/d_ocp_nlp_ipm.o ocp_nlp/d_ocp_nlp_sqp.o
#OBJS +=

# ocp qp
# double
OBJS += ocp_qp/d_ocp_qp_dim.o
OBJS += ocp_qp/d_ocp_qp.o
OBJS += ocp_qp/d_ocp_qp_sol.o
OBJS += ocp_qp/d_ocp_qp_res.o
OBJS += ocp_qp/d_ocp_qp_kkt.o
OBJS += ocp_qp/d_ocp_qp_ipm.o
OBJS += ocp_qp/d_ocp_qp_utils.o
OBJS += ocp_qp/d_ocp_qp_red.o
OBJS += ocp_qp/d_ocp_qcqp_dim.o
OBJS += ocp_qp/d_ocp_qcqp.o
OBJS += ocp_qp/d_ocp_qcqp_sol.o
OBJS += ocp_qp/d_ocp_qcqp_res.o
OBJS += ocp_qp/d_ocp_qcqp_ipm.o
OBJS += ocp_qp/d_ocp_qcqp_utils.o
# single
OBJS += ocp_qp/s_ocp_qp_dim.o
OBJS += ocp_qp/s_ocp_qp.o
OBJS += ocp_qp/s_ocp_qp_sol.o
OBJS += ocp_qp/s_ocp_qp_res.o
OBJS += ocp_qp/s_ocp_qp_kkt.o
OBJS += ocp_qp/s_ocp_qp_ipm.o
OBJS += ocp_qp/s_ocp_qp_utils.o
OBJS += ocp_qp/s_ocp_qp_red.o
OBJS += ocp_qp/s_ocp_qcqp_dim.o
OBJS += ocp_qp/s_ocp_qcqp.o
OBJS += ocp_qp/s_ocp_qcqp_sol.o
OBJS += ocp_qp/s_ocp_qcqp_res.o
OBJS += ocp_qp/s_ocp_qcqp_ipm.o
OBJS += ocp_qp/s_ocp_qcqp_utils.o
# mixed
#OBJS += ocp_qp/m_ocp_qp.o                       ocp_qp/m_ocp_qp_kkt.o ocp_qp/m_ocp_qp_ipm.o

# sim core
# double
OBJS += sim_core/d_sim_rk.o
OBJS += sim_core/d_sim_erk.o
# single
OBJS += sim_core/s_sim_rk.o
OBJS += sim_core/s_sim_erk.o
#OBJS +=

# tree ocp qp
# common
OBJS += tree_ocp_qp/scenario_tree.o
# double
OBJS += tree_ocp_qp/d_tree_ocp_qp_dim.o
OBJS += tree_ocp_qp/d_tree_ocp_qp.o
OBJS += tree_ocp_qp/d_tree_ocp_qp_sol.o
OBJS += tree_ocp_qp/d_tree_ocp_qp_res.o
OBJS += tree_ocp_qp/d_tree_ocp_qp_kkt.o
OBJS += tree_ocp_qp/d_tree_ocp_qp_ipm.o
OBJS += tree_ocp_qp/d_tree_ocp_qp_utils.o
OBJS += tree_ocp_qp/d_tree_ocp_qcqp_dim.o
OBJS += tree_ocp_qp/d_tree_ocp_qcqp.o
OBJS += tree_ocp_qp/d_tree_ocp_qcqp_sol.o
OBJS += tree_ocp_qp/d_tree_ocp_qcqp_res.o
OBJS += tree_ocp_qp/d_tree_ocp_qcqp_ipm.o
OBJS += tree_ocp_qp/d_tree_ocp_qcqp_utils.o
# single
OBJS += tree_ocp_qp/s_tree_ocp_qp_dim.o
OBJS += tree_ocp_qp/s_tree_ocp_qp.o
OBJS += tree_ocp_qp/s_tree_ocp_qp_sol.o
OBJS += tree_ocp_qp/s_tree_ocp_qp_res.o
OBJS += tree_ocp_qp/s_tree_ocp_qp_kkt.o
OBJS += tree_ocp_qp/s_tree_ocp_qp_ipm.o
OBJS += tree_ocp_qp/s_tree_ocp_qp_utils.o
OBJS += tree_ocp_qp/s_tree_ocp_qcqp_dim.o
OBJS += tree_ocp_qp/s_tree_ocp_qcqp.o
OBJS += tree_ocp_qp/s_tree_ocp_qcqp_sol.o
OBJS += tree_ocp_qp/s_tree_ocp_qcqp_res.o
OBJS += tree_ocp_qp/s_tree_ocp_qcqp_ipm.o
OBJS += tree_ocp_qp/s_tree_ocp_qcqp_utils.o

# aux
OBJS += auxiliary/aux_string.o
OBJS += auxiliary/aux_mem.o
OBJS += auxiliary/timing.o

all: clean static_library

static_library: target
	( cd cond; $(MAKE) obj TOP=$(TOP) )
	( cd dense_qp; $(MAKE) obj TOP=$(TOP) )
	( cd ipm_core; $(MAKE) obj TOP=$(TOP) )
	( cd ocp_qp; $(MAKE) obj TOP=$(TOP) )
	( cd tree_ocp_qp; $(MAKE) obj TOP=$(TOP) )
	( cd auxiliary; $(MAKE) obj TOP=$(TOP) )
	( cd sim_core; $(MAKE) obj TOP=$(TOP) )
	ar rcs libhpipm.a $(OBJS) 
	cp libhpipm.a ./lib/
	@echo
	@echo " libhpipm.a static library build complete."
	@echo

shared_library: target
	( cd cond; $(MAKE) obj TOP=$(TOP) )
	( cd dense_qp; $(MAKE) obj TOP=$(TOP) )
	( cd ipm_core; $(MAKE) obj TOP=$(TOP) )
	( cd ocp_qp; $(MAKE) obj TOP=$(TOP) )
	( cd tree_ocp_qp; $(MAKE) obj TOP=$(TOP) )
	( cd auxiliary; $(MAKE) obj TOP=$(TOP) )
	( cd sim_core; $(MAKE) obj TOP=$(TOP) )
	$(CC) -L$(BLASFEO_PATH)/lib -shared -o libhpipm.so $(OBJS) -lblasfeo -lm
	cp libhpipm.so ./lib/
	@echo
	@echo " libhpipm.so shared library build complete."
	@echo

target:
	touch ./include/hpipm_target.h
ifeq ($(TARGET), AVX512)
	echo "#ifndef TARGET_AVX512" > ./include/hpipm_target.h
	echo "#define TARGET_AVX512" >> ./include/hpipm_target.h
	echo "#endif" >> ./include/hpipm_target.h
endif
ifeq ($(TARGET), AVX)
	echo "#ifndef TARGET_AVX" > ./include/hpipm_target.h
	echo "#define TARGET_AVX" >> ./include/hpipm_target.h
	echo "#endif" >> ./include/hpipm_target.h
endif
ifeq ($(TARGET), GENERIC)
	echo "#ifndef TARGET_GENERIC" > ./include/hpipm_target.h
	echo "#define TARGET_GENERIC" >> ./include/hpipm_target.h
	echo "#endif" >> ./include/hpipm_target.h
endif

install_static:
	mkdir -p $(PREFIX)/hpipm
	mkdir -p $(PREFIX)/hpipm/lib
	cp -f libhpipm.a $(PREFIX)/hpipm/lib/
	mkdir -p $(PREFIX)/hpipm/include
	cp -f ./include/*.h $(PREFIX)/hpipm/include/

install_shared:
	mkdir -p $(PREFIX)/hpipm
	mkdir -p $(PREFIX)/hpipm/lib
	cp -f libhpipm.so $(PREFIX)/hpipm/lib/
	mkdir -p $(PREFIX)/hpipm/include
	cp -f ./include/*.h $(PREFIX)/hpipm/include/

test_problems:
	make -C test_problems obj TOP=$(TOP)
	@echo
	@echo " Test problem build complete."
	@echo

run_test_problems:
	./test_problems/test.out

sde_run_test_problems:
	~/sde/sde64 -- ./test_problems/test.out

examples:
	cp libhpipm.a ./examples/c/libhpipm.a
	( cd examples/c; $(MAKE) obj )
	@echo
	@echo " Examples build complete."
	@echo

run_examples:
	./examples/c/example.out

sde_run_examples:
	~/sde/sde64 -- ./examples/c/example.out

benchmarks:
	( cd benchmark; $(MAKE) )
	@echo
	@echo " Benchmarks build complete."
	@echo

run_benchmarks:
	( cd benchmark; $(MAKE) run )

.PHONY: test_problems examples benchmarks

clean:
	rm -f libhpipm.a
	rm -f libhpipm.so
	rm -f ./lib/libhpipm.a
	rm -f ./lib/libhpipm.so
	make -C auxiliary clean
	make -C cond clean
	make -C dense_qp clean
	make -C ipm_core clean
	make -C ocp_qp clean
	make -C sim_core clean
	make -C tree_ocp_qp clean
	make -C test_problems clean
	make -C examples/c clean
	make -C benchmark clean

