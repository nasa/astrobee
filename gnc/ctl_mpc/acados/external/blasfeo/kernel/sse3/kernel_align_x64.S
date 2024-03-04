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

#if defined(OS_LINUX) | defined(OS_MAC)

//#define STACKSIZE 96
#define STACKSIZE 64
#define ARG1  %rdi
#define ARG2  %rsi
#define ARG3  %rdx
#define ARG4  %rcx
#define ARG5  %r8
#define ARG6  %r9
#define ARG7  STACKSIZE +  8(%rsp)
#define ARG8  STACKSIZE + 16(%rsp)
#define ARG9  STACKSIZE + 24(%rsp)
#define ARG10 STACKSIZE + 32(%rsp)
#define ARG11 STACKSIZE + 40(%rsp)
#define ARG12 STACKSIZE + 48(%rsp)
#define ARG13 STACKSIZE + 56(%rsp)
#define ARG14 STACKSIZE + 64(%rsp)
#define ARG15 STACKSIZE + 72(%rsp)
#define ARG16 STACKSIZE + 80(%rsp)
#define ARG17 STACKSIZE + 88(%rsp)
#define ARG18 STACKSIZE + 96(%rsp)
#define PROLOGUE \
	subq	$STACKSIZE, %rsp; \
	movq	%rbx,   (%rsp); \
	movq	%rbp,  8(%rsp); \
	movq	%r12, 16(%rsp); \
	movq	%r13, 24(%rsp); \
	movq	%r14, 32(%rsp); \
	movq	%r15, 40(%rsp); \
	vzeroupper;
#define EPILOGUE \
	vzeroupper; \
	movq	  (%rsp), %rbx; \
	movq	 8(%rsp), %rbp; \
	movq	16(%rsp), %r12; \
	movq	24(%rsp), %r13; \
	movq	32(%rsp), %r14; \
	movq	40(%rsp), %r15; \
	addq	$STACKSIZE, %rsp;

#elif defined(OS_WINDOWS)

#define STACKSIZE 256
#define ARG1  %rcx
#define ARG2  %rdx
#define ARG3  %r8
#define ARG4  %r9
#define ARG5  STACKSIZE + 40(%rsp)
#define ARG6  STACKSIZE + 48(%rsp)
#define ARG7  STACKSIZE + 56(%rsp)
#define ARG8  STACKSIZE + 64(%rsp)
#define ARG9  STACKSIZE + 72(%rsp)
#define ARG10 STACKSIZE + 80(%rsp)
#define ARG11 STACKSIZE + 88(%rsp)
#define ARG12 STACKSIZE + 96(%rsp)
#define ARG13 STACKSIZE + 104(%rsp)
#define ARG14 STACKSIZE + 112(%rsp)
#define ARG15 STACKSIZE + 120(%rsp)
#define ARG16 STACKSIZE + 128(%rsp)
#define ARG17 STACKSIZE + 136(%rsp)
#define ARG18 STACKSIZE + 144(%rsp)
#define PROLOGUE \
	subq	$STACKSIZE, %rsp; \
	movq	%rbx,   (%rsp); \
	movq	%rbp,  8(%rsp); \
	movq	%r12, 16(%rsp); \
	movq	%r13, 24(%rsp); \
	movq	%r14, 32(%rsp); \
	movq	%r15, 40(%rsp); \
	movq	%rdi, 48(%rsp); \
	movq	%rsi, 56(%rsp); \
	vmovups	%xmm6, 64(%rsp); \
	vmovups	%xmm7, 80(%rsp); \
	vmovups	%xmm8, 96(%rsp); \
	vmovups	%xmm9, 112(%rsp); \
	vmovups	%xmm10, 128(%rsp); \
	vmovups	%xmm11, 144(%rsp); \
	vmovups	%xmm12, 160(%rsp); \
	vmovups	%xmm13, 176(%rsp); \
	vmovups	%xmm14, 192(%rsp); \
	vmovups	%xmm15, 208(%rsp); \
	vzeroupper;
#define EPILOGUE \
	vzeroupper; \
	movq	  (%rsp), %rbx; \
	movq	 8(%rsp), %rbp; \
	movq	16(%rsp), %r12; \
	movq	24(%rsp), %r13; \
	movq	32(%rsp), %r14; \
	movq	40(%rsp), %r15; \
	movq	48(%rsp), %rdi; \
	movq	56(%rsp), %rsi; \
	vmovups	64(%rsp), %xmm6; \
	vmovups	80(%rsp), %xmm7; \
	vmovups	96(%rsp), %xmm8; \
	vmovups	112(%rsp), %xmm9; \
	vmovups	128(%rsp), %xmm10; \
	vmovups	144(%rsp), %xmm11; \
	vmovups	160(%rsp), %xmm12; \
	vmovups	176(%rsp), %xmm13; \
	vmovups	192(%rsp), %xmm14; \
	vmovups	208(%rsp), %xmm15; \
	addq	$STACKSIZE, %rsp;

#else

#error wrong OS

#endif



#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.text
#elif defined(OS_MAC)
	.section	__TEXT,__text,regular,pure_instructions
#endif





//                               1          2
// void blasafeo_align_2MB(void *ptr, void **ptr_align);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl blasfeo_align_2MB
	.type blasfeo_align_2MB, @function
blasfeo_align_2MB:
#elif defined(OS_MAC)
	.globl _blasfeo_align_2MB
_blasfeo_align_2MB:
#elif defined(OS_WINDOWS)
	.globl blasfeo_align_2MB
	.def blasfeo_align_2MB; .scl 2; .type 32; .endef
blasfeo_align_2MB:
#endif
	
	PROLOGUE

	movq	ARG1, %r10
	movq	ARG2, %r11

	addq 	$ 2097151, %r10
	movq	$ 2097151, %r12
	notq	%r12
	andq	%r12, %r10
	movq	%r10, (%r11)

	EPILOGUE
	
	ret

#if defined(OS_LINUX)
	.size	blasfeo_align_2MB, .-blasfeo_align_2MB
#endif





//                               1          2
// void blasafeo_align_4096_byte(void *ptr, void **ptr_align);

	.p2align 4,,15
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

	movq	ARG1, %r10
	movq	ARG2, %r11

	addq 	$ 4095, %r10
	movq	$ 4095, %r12
	notq	%r12
	andq	%r12, %r10
	movq	%r10, (%r11)

	EPILOGUE
	
	ret

#if defined(OS_LINUX)
	.size	blasfeo_align_4096_byte, .-blasfeo_align_4096_byte
#endif





//                            1          2
// void blasfeo_align_64_byte(void *ptr, void **ptr_align);

	.p2align 4,,15
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

	movq	ARG1, %r10
	movq	ARG2, %r11

	addq 	$ 63, %r10
	movq	$ 63, %r12
	notq	%r12
	andq	%r12, %r10
	movq	%r10, (%r11)

	EPILOGUE
	
	ret

#if defined(OS_LINUX)
	.size	blasfeo_align_64_byte, .-blasfeo_align_64_byte
#endif


