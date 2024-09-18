/**************************************************************************************************
*                                                                                                 *
* This file is part of BLASFEO.                                                                   *
*                                                                                                 *
* BLASFEO -- BLAS For Embedded Optimization.                                                      *
* Copyright (C) 2020 by Gianluca Frison.                                                          *
* All rights reserved.                                                                            *
*                                                                                                 *
*                                                                                                 *
* The 2-Clause BSD License                                                                        *
*                                                                                                 *
* Redistribution and use in source and binary forms, with or without                              *
* modification, are permitted provided that the following conditions are met:                     *
*                                                                                                 *
* 1. Redistributions of source code must retain the above copyright notice, this                  *
*    list of conditions and the following disclaimer.                                             *
* 2. Redistributions in binary form must reproduce the above copyright notice,                    *
*    this list of conditions and the following disclaimer in the documentation                    *
*    and/or other materials provided with the distribution.                                       *
*                                                                                                 *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND                 *
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED                   *
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE                          *
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR                 *
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES                  *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;                    *
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND                     *
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT                      *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS                   *
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                    *
*                                                                                                 *
* Author: Gianluca Frison, gianluca.frison (at) imtek.uni-freiburg.de                             *
*                                                                                                 *
**************************************************************************************************/


#include <stdlib.h>
#include <stdio.h>

#include <blasfeo_stdlib.h>
#include <blasfeo_block_size.h>
#include <blasfeo_d_aux.h>
#include <blasfeo_s_aux.h>



static int initialized = 0;

static void *mem = NULL;



int blasfeo_is_init()
	{
	return initialized;
	}



void blasfeo_init()
	{
	size_t tmp0, tmp1;
	// compute max needed memory
	size_t size_A_double = blasfeo_pm_memsize_dmat(D_PS, D_MC, D_KC); 
	size_t size_B_double = blasfeo_pm_memsize_dmat(D_PS, D_NC, D_KC); 
	tmp0 = blasfeo_pm_memsize_dmat(D_PS, D_KC, D_KC); 
	tmp1 = blasfeo_pm_memsize_dmat(D_PS, D_NC, D_NC); 
	size_t size_T_double = tmp0>tmp1 ? tmp0 : tmp1;
	// TODO size_T_double
	size_t size_A_single = blasfeo_pm_memsize_smat(S_PS, S_MC, S_KC); 
	size_t size_B_single = blasfeo_pm_memsize_smat(S_PS, S_NC, S_KC); 
	tmp0 = blasfeo_pm_memsize_smat(S_PS, S_KC, S_KC); 
	tmp1 = blasfeo_pm_memsize_smat(S_PS, S_NC, S_NC); 
	size_t size_T_single = tmp0>tmp1 ? tmp0 : tmp1;
	// TODO size_T_single
	size_A_double = (size_A_double + 4096 - 1) / 4096 * 4096;
	size_B_double = (size_B_double + 4096 - 1) / 4096 * 4096;
	size_T_double = (size_T_double + 4096 - 1) / 4096 * 4096;
	size_A_single = (size_A_single + 4096 - 1) / 4096 * 4096;
	size_B_single = (size_B_single + 4096 - 1) / 4096 * 4096;
	size_T_single = (size_T_single + 4096 - 1) / 4096 * 4096;
	size_t size_double = size_A_double + size_B_double + size_T_double;
	size_t size_single = size_A_single + size_B_single + size_T_single;
	size_t size = size_double>=size_single ? size_double : size_single;
	// alignment
	size += 2*4096;
//	printf("\nsize %d\n", size);
	// call malloc
	mem = malloc(size);
	initialized = 1;
	}



void blasfeo_quit()
	{
	free(mem);
	initialized = 0;
	}



void *blasfeo_get_buffer()
	{
	return mem;
	}
