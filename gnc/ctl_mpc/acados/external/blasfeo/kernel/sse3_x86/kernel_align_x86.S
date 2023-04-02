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

#if defined(OS_LINUX)

#define STACKSIZE 16
#define ARG1  STACKSIZE +  4(%esp)
#define ARG2  STACKSIZE +  8(%esp)
#define ARG3  STACKSIZE + 12(%esp)
#define ARG4  STACKSIZE + 16(%esp)
#define ARG5  STACKSIZE + 20(%esp)
#define ARG6  STACKSIZE + 24(%esp)
#define ARG7  STACKSIZE + 28(%esp)
#define ARG8  STACKSIZE + 32(%esp)
#define ARG9  STACKSIZE + 36(%esp)
#define ARG10 STACKSIZE + 40(%esp)
#define ARG11 STACKSIZE + 44(%esp)

#if 1

#define PROLOGUE \
	subl	$ 16, %esp; \
	movl	%ebx, 0(%esp); \
	movl	%esi, 4(%esp); \
	movl	%edi, 8(%esp); \
	movl	%ebp, 12(%esp);
#define EPILOGUE \
	movl	0(%esp), %ebx; \
	movl	4(%esp), %esi; \
	movl	8(%esp), %edi; \
	movl	12(%esp), %ebp; \
	addl	$ 16, %esp;

#else

#define PROLOGUE \
	pushl	%ebp; \
	pushl	%edi; \
	pushl	%esi; \
	pushl	%ebx;
#define EPILOGUE \
	popl	%ebx; \
	popl	%esi; \
	popl	%edi; \
	popl	%ebp;

#endif

#else

#error wrong OS

#endif



	.text





//                            1          2
// void blasfeo_align_64_byte(void *ptr, void **ptr_align);

	.align 16
#if defined(OS_LINUX)
	.globl blasfeo_align_4096_byte
	.type blasfeo_align_4096_byte, @function
blasfeo_align_4096_byte:
#elif defined(OS_MAC)
	.globl _blasfeo_align_4096_byte
_blasfeo_align_4096_byte:
#elif defined(OS_WINDOWS)
	.globl blasfeo_align_4096_byte
	.def blasfeo_align_4096_byte; .scl 2; .type 32; .endef
blasfeo_align_4096_byte:
#endif
	
	PROLOGUE

	movl	ARG1, %eax
	movl	ARG2, %ebx

	addl 	$ 4095, %eax
	movl	$ 4095, %ecx
	notl	%ecx
	andl	%ecx, %eax
	movl	%eax, (%ebx)

	EPILOGUE
	
	ret

#if defined(OS_LINUX)
	.size	blasfeo_align_4096_byte, .-blasfeo_align_4096_byte
#endif





//                    1          2
// void align_64_byte(void *ptr, void **ptr_align);

	.align 16
#if defined(OS_LINUX)
	.globl blasfeo_align_64_byte
	.type blasfeo_align_64_byte, @function
blasfeo_align_64_byte:
#elif defined(OS_MAC)
	.globl _blasfeo_align_64_byte
_blasfeo_align_64_byte:
#elif defined(OS_WINDOWS)
	.globl blasfeo_align_64_byte
	.def blasfeo_align_64_byte; .scl 2; .type 32; .endef
blasfeo_align_64_byte:
#endif
	
	PROLOGUE

	movl	ARG1, %eax
	movl	ARG2, %ebx

	addl 	$ 63, %eax
	movl	$ 63, %ecx
	notl	%ecx
	andl	%ecx, %eax
	movl	%eax, (%ebx)

	EPILOGUE
	
	ret

#if defined(OS_LINUX)
	.size	blasfeo_align_64_byte, .-blasfeo_align_64_byte
#endif



