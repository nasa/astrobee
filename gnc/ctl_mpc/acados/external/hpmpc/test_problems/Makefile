###################################################################################################
#                                                                                                 #
# This file is part of HPMPC.                                                                     #
#                                                                                                 #
# HPMPC -- Library for High-Performance implementation of solvers for MPC.                        #
# Copyright (C) 2014-2015 by Technical University of Denmark. All rights reserved.                #
#                                                                                                 #
# HPMPC is free software; you can redistribute it and/or                                          #
# modify it under the terms of the GNU Lesser General Public                                      #
# License as published by the Free Software Foundation; either                                    #
# version 2.1 of the License, or (at your option) any later version.                              #
#                                                                                                 #
# HPMPC is distributed in the hope that it will be useful,                                        #
# but WITHOUT ANY WARRANTY; without even the implied warranty of                                  #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                                            #
# See the GNU Lesser General Public License for more details.                                     #
#                                                                                                 #
# You should have received a copy of the GNU Lesser General Public                                #
# License along with HPMPC; if not, write to the Free Software                                    #
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA                  #
#                                                                                                 #
# Author: Gianluca Frison, giaf (at) dtu.dk                                                       #
#                                                                                                 #
###################################################################################################

include ../Makefile.rule

LIBS = 

ifeq ($(USE_BLASFEO), 1)
LIBS += $(BLASFEO_PATH)/lib/libblasfeo.a
endif

ifeq ($(REF_BLAS), 0)
LIBS += -lm 
endif
ifeq ($(REF_BLAS), OPENBLAS)
LIBS += /opt/openblas/lib/libopenblas.a -pthread -lm
endif
ifeq ($(REF_BLAS), BLIS)
LIBS += -lblis -lm -fopenmp
endif
ifeq ($(REF_BLAS), NETLIB)
LIBS += /opt/netlib/liblapack.a /opt/netlib/libblas.a -lgfortran -lm
endif
ifeq ($(REF_BLAS), MKL)
LIBS += -Wl,--start-group /opt/intel/mkl/lib/intel64/libmkl_gf_lp64.a /opt/intel/mkl/lib/intel64/libmkl_core.a /opt/intel/mkl/lib/intel64/libmkl_sequential.a -Wl,--end-group -ldl -lpthread -lm
endif

# tests for USE_BLASFEO = 0
#OBJS_TEST = test_blas_d.o
#OBJS_TEST = tools.o test_d_ric_mpc.o
#OBJS_TEST = tools.o test_d_ip_hard.o
#OBJS_TEST = tools.o test_d_cond.o

# tests for USE_BLASFEO = 1
#OBJS_TEST = tools.o test_d_tight_ric_libstr.o
#OBJS_TEST = tools.o test_d_ric_libstr.o
OBJS_TEST = tools.o test_d_ip_hard_libstr.o
#OBJS_TEST = tools.o test_d_ip_hard_no_ext_dep.o
#OBJS_TEST = tools.o test_d_ip_hard_car_new_libstr.o
#OBJS_TEST = tools.o test_d_tree_ric_libstr.o
#OBJS_TEST = tools.o test_d_tree_ip_hard_libstr.o
#OBJS_TEST = tools.o test_d_cond_libstr.o

obj: $(OBJS_TEST)
	$(CC) -o test.out $(OBJS_TEST) -L. libhpmpc.a $(LIBS) #-pg

clean:
	rm -f *.o
	rm -f test.out
	rm -f libhpmpc.a
