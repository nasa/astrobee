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

from ctypes import *
import ctypes.util 
import numpy as np


class hpipm_ocp_qcqp_dim:


	def __init__(self, N):

		# load hpipm shared library
		__hpipm   = CDLL('libhpipm.so')
		self.__hpipm = __hpipm

		# C dim struct
		dim_struct_size = __hpipm.d_ocp_qcqp_dim_strsize()
		dim_struct = cast(create_string_buffer(dim_struct_size), c_void_p)
		self.dim_struct = dim_struct

		# C dim internal memory
		dim_mem_size = __hpipm.d_ocp_qcqp_dim_memsize(N)
		dim_mem = cast(create_string_buffer(dim_mem_size), c_void_p)
		self.dim_mem = dim_mem

		# create C dim
		__hpipm.d_ocp_qcqp_dim_create(N, self.dim_struct, self.dim_mem)


	def set(self, field, value, idx_start, idx_end=None):
		self.__hpipm.d_ocp_qcqp_dim_set.argtypes = [c_char_p, c_int, c_int, c_void_p]
		field_name_b = field.encode('utf-8')
		if idx_end==None:
			self.__hpipm.d_ocp_qcqp_dim_set(c_char_p(field_name_b), idx_start, value, self.dim_struct)
		else:
			for i in range(idx_start, idx_end+1):
				self.__hpipm.d_ocp_qcqp_dim_set(c_char_p(field_name_b), i, value, self.dim_struct)
		return


	def print_C_struct(self):
		self.__hpipm.d_ocp_qcqp_dim_print(self.dim_struct)
		return 


	def codegen(self, file_name, mode):
		file_name_b = file_name.encode('utf-8')
		mode_b = mode.encode('utf-8')
		self.__hpipm.d_ocp_qcqp_dim_codegen(c_char_p(file_name_b), c_char_p(mode_b), self.dim_struct)
		return 

