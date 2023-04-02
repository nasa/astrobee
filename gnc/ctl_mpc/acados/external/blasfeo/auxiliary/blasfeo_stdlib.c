/**************************************************************************************************
*                                                                                                 *
* This file is part of BLASFEO.                                                                   *
*                                                                                                 *
* BLASFEO -- BLAS For Embedded Optimization.                                                      *
* Copyright (C) 2019 by Gianluca Frison.                                                          *
* Developed at IMTEK (University of Freiburg) under the supervision of Moritz Diehl.              *
* All rights reserved.                                                                            *
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



void blasfeo_malloc(void **ptr, size_t size)
	{
	*ptr = malloc(size);
	return;
	}



// allocate memory aligned to typical cache line size (64 bytes)
void blasfeo_malloc_align(void **ptr, size_t size)
	{

#if defined(OS_WINDOWS)

	*ptr = _aligned_malloc( size, CACHE_LINE_SIZE );

#elif defined(__DSPACE__)

	// XXX fix this hack !!! (Andrea?)
	*ptr = malloc( size );

#elif(defined __XILINX_NONE_ELF__ || defined __XILINX_ULTRASCALE_NONE_ELF_JAILHOUSE__)

	*ptr = memalign( CACHE_LINE_SIZE, size );

#else

	int err = posix_memalign( ptr, CACHE_LINE_SIZE, size );
	if(err!=0)
		{
		printf("Memory allocation error");
		exit(1);
		}

#endif

	return;

	}



void blasfeo_free(void *ptr)
	{
	free(ptr);
	return;
	}



void blasfeo_free_align(void *ptr)
	{

#if defined(OS_WINDOWS)

	_aligned_free( ptr );

#else

	free( ptr );

#endif

	return;

	}
