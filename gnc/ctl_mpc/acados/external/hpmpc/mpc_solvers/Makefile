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

OBJS = 

ifeq ($(USE_BLASFEO), 1)
OBJS += d_ip2_res_hard_libstr.o d_tree_ip2_res_hard_libstr.o d_res_ip_res_hard_libstr.o  d_tree_res_ip_res_hard_libstr.o
else
OBJS += d_ip2_hard.o d_res_ip_hard.o d_ip2_res_hard.o d_ip2_soft.o d_res_ip_soft.o
endif

obj: $(OBJS)
ifeq ($(TARGET), X64_AVX2)
	( cd avx; $(MAKE) obj)
	( cd c99; $(MAKE) obj)
endif
ifeq ($(TARGET), X64_AVX)
	( cd avx; $(MAKE) obj)
	( cd c99; $(MAKE) obj)
endif
ifeq ($(TARGET), X64_SSE3)
	( cd c99; $(MAKE) obj)
endif
ifeq ($(TARGET), C99_4X4)
	( cd c99; $(MAKE) obj)
endif
ifeq ($(TARGET), C99_4X4_PREFETCH)
	( cd c99; $(MAKE) obj)
endif
ifeq ($(TARGET), CORTEX_A57)
	( cd c99; $(MAKE) obj)
endif
ifeq ($(TARGET), CORTEX_A15)
	( cd c99; $(MAKE) obj)
endif
ifeq ($(TARGET), CORTEX_A9)
	( cd c99; $(MAKE) obj)
endif
ifeq ($(TARGET), CORTEX_A7)
	( cd c99; $(MAKE) obj)
endif

clean:
	rm -f *.o
	make -C avx clean
	make -C c99 clean


