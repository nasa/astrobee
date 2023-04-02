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
include Makefile.netlib_cblas

BLAS_VERSION = 3.10.0
BLAS_DIR = blas

CBLAS_DIR = cblas

LAPACK_VERSION = 3.10.0
LAPACK_DIR = lapack


obj_blas: | $(BLAS_DIR)
	@echo "compiling BLAS ..."
	cp $(BLAS_DIR)/make.inc.bkp $(BLAS_DIR)/make.inc
	@echo "" >> $(BLAS_DIR)/make.inc
	@echo "#" >> $(BLAS_DIR)/make.inc
	@echo "# BLASFEO's custom flags" >> $(BLAS_DIR)/make.inc
	@echo "#" >> $(BLAS_DIR)/make.inc
	@echo "OPTS = $(CFLAGS) -O3" >> $(BLAS_DIR)/make.inc
	(cd $(BLAS_DIR); $(MAKE) all)


$(BLAS_DIR):
	@echo "downloading BLAS ..."
	wget www.netlib.org/blas/blas-$(BLAS_VERSION).tgz -O blas-$(BLAS_VERSION).tgz
	mkdir -p $(BLAS_DIR)
	tar -xzf blas-$(BLAS_VERSION).tgz -C $(BLAS_DIR) --strip-components 1
	cp $(BLAS_DIR)/make.inc $(BLAS_DIR)/make.inc.bkp


obj_cblas: | $(CBLAS_DIR)
	@echo "compiling CBLAS ..."
	cp $(CBLAS_DIR)/Makefile.in.bkp $(CBLAS_DIR)/Makefile.in
	@echo "" >> $(CBLAS_DIR)/Makefile.in
	@echo "#" >> $(CBLAS_DIR)/Makefile.in
	@echo "# BLASFEO's custom flags" >> $(CBLAS_DIR)/Makefile.in
	@echo "#" >> $(CBLAS_DIR)/Makefile.in
	@echo "CC = $(CC)" >> $(CBLAS_DIR)/Makefile.in
	@echo "FC = $(CC:gcc=gfortran)" >> $(CBLAS_DIR)/Makefile.in
	@echo "CFLAGS = $(CFLAGS) -DADD_" >> $(CBLAS_DIR)/Makefile.in
	@echo "FFLAGS = $(CFLAGS)" >> $(CBLAS_DIR)/Makefile.in
	@echo "BLLIB = $(BLAS_DIR)/blas_LINUX.a" >> $(CBLAS_DIR)/Makefile.in
	@echo "CFLAGS += $(BLASFEO_CBLAS_DEF)" >> $(CBLAS_DIR)/Makefile.in
ifeq ($(EXT_DEP), 0)
	cp $(CBLAS_DIR)/src/cblas_xerbla.c $(CBLAS_DIR)/src/cblas_xerbla_bkp.c
	@echo "void cblas_xerbla(int info, const char *rout, const char *form, ...)" > $(CBLAS_DIR)/src/cblas_xerbla.c
	@echo "{ return; }" >> $(CBLAS_DIR)/src/cblas_xerbla.c
endif
	(cd $(CBLAS_DIR); $(MAKE) alllib)
ifeq ($(EXT_DEP), 0)
	cp $(CBLAS_DIR)/src/cblas_xerbla_bkp.c $(CBLAS_DIR)/src/cblas_xerbla.c
endif


$(CBLAS_DIR):
	@echo "downloading CBLAS ..."
	wget www.netlib.org/blas/blast-forum/cblas.tgz -O cblas.tgz
	mkdir -p $(CBLAS_DIR)
	tar -xzf cblas.tgz -C $(CBLAS_DIR) --strip-components 1
	cp $(CBLAS_DIR)/Makefile.in $(CBLAS_DIR)/Makefile.in.bkp


obj_lapack: | $(LAPACK_DIR)
	@echo "compiling LAPACK ..."
	cp $(LAPACK_DIR)/make.inc.example $(LAPACK_DIR)/make.inc
	@echo "" >> $(LAPACK_DIR)/make.inc
	@echo "#" >> $(LAPACK_DIR)/make.inc
	@echo "# BLASFEO's custom flags" >> $(LAPACK_DIR)/make.inc
	@echo "#" >> $(LAPACK_DIR)/make.inc
	@echo "OPTS = $(CFLAGS) -O2 -frecursive" >> $(LAPACK_DIR)/make.inc
	@echo "DRVOPTS = $(OPTS)" >> $(LAPACK_DIR)/make.inc
	@echo "NOOPT = $(CFLAGS) -O0 -frecursive" >> $(LAPACK_DIR)/make.inc
	(cd $(LAPACK_DIR); $(MAKE) lib)


$(LAPACK_DIR):
	@echo "downloading LAPACK ..."
	# up to 3.8.0
	# wget www.netlib.org/lapack/lapack-$(LAPACK_VERSION).tar.gz -O lapack-$(LAPACK_VERSION).tar.gz # up to 3.8.0
	# from 3.9.0
	wget https://github.com/Reference-LAPACK/lapack/archive/refs/tags/v$(LAPACK_VERSION).tar.gz -O lapack-$(LAPACK_VERSION).tar.gz
	mkdir -p $(LAPACK_DIR)
	tar -xzf lapack-$(LAPACK_VERSION).tar.gz -C $(LAPACK_DIR) --strip-components 1


obj_lapacke: obj_lapack
	@echo "compiling LAPACKE ..."
	@echo "CFLAGS = $(CFLAGS)" >> $(LAPACK_DIR)/make.inc
	# XXX hack to fix a bug in lapacke in lapack-3.8.0
	#sed -i 's/OBJ_B = \\/OBJ_B = lapacke_dsytrf_aa_work.o lapacke_ssytrf_aa_work.o \\/g' $(LAPACK_DIR)/LAPACKE/src/Makefile
	# XXX hack till here
	(cd $(LAPACK_DIR); $(MAKE) lapackelib)


clean:
ifneq ($(wildcard $(BLAS_DIR)), )
	make -C $(BLAS_DIR) clean
endif
#	if [ -d "./$(BLAS_DIR)" ]; then \
#		make -C $(BLAS_DIR) clean; \
#	fi
ifneq ($(wildcard $(CBLAS_DIR)), )
	make -C $(CBLAS_DIR) cleanall
endif
ifneq ($(wildcard $(LAPACK_DIR)), )
	make -C $(LAPACK_DIR) clean
endif


deep_clean:
	rm -rf $(BLAS_DIR)
	rm -rf blas-$(BLAS_VERSION).tgz
	rm -rf $(CBLAS_DIR)
	rm -rf cblas.tgz
	rm -rf $(LAPACK_DIR)
	rm -rf lapack-$(LAPACK_VERSION).tar.gz


.PHONY: obj_blas obj_cblas obj_lapack clean deep_clean obj_lapacke
