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



class hpipm_ocp_qp_solver_arg:


	def __init__(self, dim, mode):

		c_mode = 0
		if(mode=='speed_abs'):
			c_mode = 0
		elif(mode=='speed'):
			c_mode = 1
		elif(mode=='balance'):
			c_mode = 2
		elif(mode=='robust'):
			c_mode = 3
		else:
			raise NameError('hpipm_ocp_qp_solver_arg: wrong mode')

		# save dim internally
		self.dim = dim

		# load hpipm shared library
		__hpipm   = CDLL('libhpipm.so')
		self.__hpipm = __hpipm

		# C qp struct
		arg_struct_size = __hpipm.d_ocp_qp_ipm_arg_strsize()
		arg_struct = cast(create_string_buffer(arg_struct_size), c_void_p)
		self.arg_struct = arg_struct

		# C qp internal memory
		arg_mem_size = __hpipm.d_ocp_qp_ipm_arg_memsize(dim.dim_struct)
		arg_mem = cast(create_string_buffer(arg_mem_size), c_void_p)
		self.arg_mem = arg_mem

		# create C qp
		__hpipm.d_ocp_qp_ipm_arg_create(dim.dim_struct, arg_struct, arg_mem)

		# initialize default arguments
		__hpipm.d_ocp_qp_ipm_arg_set_default(c_mode, arg_struct) # mode==SPEED
	

	def set(self, field, value):
		if((field=='mu0') | (field=='tol_stat') | (field=='tol_eq') | (field=='tol_ineq') | (field=='tol_comp') | (field=='reg_prim')):
			tmp_in = np.zeros((1,1))
			tmp_in[0][0] = value
			tmp = cast(tmp_in.ctypes.data, POINTER(c_double))
		elif(field=='iter_max'):
			tmp_in = np.zeros((1,1), dtype=int)
			tmp_in[0][0] = value
			tmp = cast(tmp_in.ctypes.data, POINTER(c_int))
		else:
			raise NameError('hpipm_ocp_qp_solver_arg.set: wrong field')
#		self.__hpipm.d_ocp_qp_ipm_arg_set_mu0.argtypes = [POINTER(c_double), c_void_p]
		field_name_b = field.encode('utf-8')
		self.__hpipm.d_ocp_qp_ipm_arg_set(c_char_p(field_name_b), tmp, self.arg_struct)
		return


	def codegen(self, file_name, mode):
		file_name_b = file_name.encode('utf-8')
		mode_b = mode.encode('utf-8')
		self.__hpipm.d_ocp_qp_ipm_arg_codegen(c_char_p(file_name_b), c_char_p(mode_b), self.dim.dim_struct, self.arg_struct)
		return 

