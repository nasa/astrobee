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



class hpipm_ocp_qcqp_sol:


	def __init__(self, dim):

		# save dim internally
		self.dim = dim

		# load hpipm shared library
		__hpipm   = CDLL('libhpipm.so')
		self.__hpipm = __hpipm

		# C qp struct
		qp_sol_struct_size = __hpipm.d_ocp_qcqp_sol_strsize()
		qp_sol_struct = cast(create_string_buffer(qp_sol_struct_size), c_void_p)
		self.qp_sol_struct = qp_sol_struct

		# C qp internal memory
		qp_sol_mem_size = __hpipm.d_ocp_qcqp_sol_memsize(dim.dim_struct)
		qp_sol_mem = cast(create_string_buffer(qp_sol_mem_size), c_void_p)
		self.qp_sol_mem = qp_sol_mem

		# create C qp
		__hpipm.d_ocp_qcqp_sol_create(dim.dim_struct, qp_sol_struct, qp_sol_mem)


	def get(self, field, idx_start, idx_end=None):
		if(field=='u'):
			vec = self.__get_u(idx_start, idx_end)
		elif(field=='x'):
			vec = self.__get_x(idx_start, idx_end)
		else:
			raise NameError('hpipm_ocp_qcqp_sol.get: wrong field')
		return vec


	def __get_u(self, idx_start, idx_end=None):
		# nu
		nu = np.zeros((1,1), dtype=int);
		if idx_end==None:
			# get nu at stage
			tmp_ptr = cast(nu.ctypes.data, POINTER(c_int))
			self.__hpipm.d_ocp_qcqp_dim_get_nu(self.dim.dim_struct, idx_start, tmp_ptr)
			u = np.zeros((nu[0,0], 1))
			tmp_ptr = cast(u.ctypes.data, POINTER(c_double))
			self.__hpipm.d_ocp_qcqp_sol_get_u(idx_start, self.qp_sol_struct, tmp_ptr)
		else:
			u = []
			for i in range(idx_start, idx_end+1):
				# get nu at stage
				tmp_ptr = cast(nu.ctypes.data, POINTER(c_int))
				self.__hpipm.d_ocp_qcqp_dim_get_nu(self.dim.dim_struct, i, tmp_ptr)
				u.append(np.zeros((nu[0,0], 1)))
				tmp_ptr = cast(u[i].ctypes.data, POINTER(c_double))
				self.__hpipm.d_ocp_qcqp_sol_get_u(i, self.qp_sol_struct, tmp_ptr)
		return u


	def __get_x(self, idx_start, idx_end=None):
		# nx
		nx = np.zeros((1,1), dtype=int);
		if idx_end==None:
			# get nx at stage
			tmp_ptr = cast(nx.ctypes.data, POINTER(c_int))
			self.__hpipm.d_ocp_qcqp_dim_get_nx(self.dim.dim_struct, idx_start, tmp_ptr)
			x = np.zeros((nx[0,0], 1))
			tmp_ptr = cast(x.ctypes.data, POINTER(c_double))
			self.__hpipm.d_ocp_qcqp_sol_get_x(idx_start, self.qp_sol_struct, tmp_ptr)
		else:
			x = []
			for i in range(idx_start, idx_end+1):
				# get nx at stage
				tmp_ptr = cast(nx.ctypes.data, POINTER(c_int))
				self.__hpipm.d_ocp_qcqp_dim_get_nx(self.dim.dim_struct, i, tmp_ptr)
				x.append(np.zeros((nx[0,0], 1)))
				tmp_ptr = cast(x[i].ctypes.data, POINTER(c_double))
				self.__hpipm.d_ocp_qcqp_sol_get_x(i, self.qp_sol_struct, tmp_ptr)
		return x


	def print_C_struct(self):
		self.__hpipm.d_ocp_qcqp_sol_print(self.dim.dim_struct, self.qp_sol_struct)
		return 




