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
#define ARG2L  %esi
#define ARG6L  %r9d
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

#if defined(OS_LINUX)

#define GLOB_FUN_START(NAME) \
	.globl NAME; \
	.type NAME, @function; \
NAME:
#define FUN_START(NAME) \
	.type NAME, @function; \
NAME:
#define FUN_END(NAME) \
	.size	NAME, .-NAME
#define CALL(NAME) \
	call NAME

#else // defined(OS_MAC)

#define GLOB_FUN_START(NAME) \
	.globl _ ## NAME; \
_ ## NAME:
#define FUN_START(NAME) \
_ ## NAME:
#define FUN_END(NAME)
#define CALL(NAME) \
	callq _ ## NAME

#endif

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
#define ARG2L  %edx
#define ARG6L  STACKSIZE + 48(%rsp)
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

#define GLOB_FUN_START(NAME) \
	.globl NAME; \
	.def NAME; .scl 2; .type 32; .endef; \
NAME:
#define FUN_START(NAME) \
	.def NAME; .scl 2; .type 32; .endef; \
NAME:
#define FUN_END(NAME)
#define CALL(NAME) \
	call NAME

#else

#error wrong OS

#endif



#if defined(OS_LINUX)
	.text
#elif defined(OS_MAC)
	.section	__TEXT,__text,regular,pure_instructions
#endif





// input arguments
//
// r10	<- kmax
// r11	<- A
// r12	<- lda
// r13	<- C
//
#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACK_TN_4_P0_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpack_tn_4_p0_lib4)
#endif



	cmpl	$ 0, %r10d
	jle		0f // return

	leaq	0(%r12, %r12, 2), %r14

	cmpl	$ 8, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop

	leaq	0(%r11, %r12, 4), %r15

	prefetcht0	0(%r15)
	prefetcht0	0(%r15, %r12)
	prefetcht0	0(%r15, %r12, 2)
	prefetcht0	0(%r15, %r14)

	vmovupd		0(%r11), %xmm0
	vmovupd		0(%r11, %r12, 2), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm0, %ymm0
	vmovupd		0(%r11, %r12), %xmm1
	vmovupd		0(%r11, %r14), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm1, %ymm1
	vmovupd		16(%r11), %xmm2
	vmovupd		16(%r11, %r12, 2), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm2, %ymm2
	vmovupd		16(%r11, %r12), %xmm3
	vmovupd		16(%r11, %r14), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm3, %ymm3

	vmovupd		32+0(%r11), %xmm4
	vmovupd		32+0(%r11, %r12, 2), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm4, %ymm4
	vmovupd		32+0(%r11, %r12), %xmm5
	vmovupd		32+0(%r11, %r14), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm5, %ymm5
	vmovupd		32+16(%r11), %xmm6
	vmovupd		32+16(%r11, %r12, 2), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm6, %ymm6
	vmovupd		32+16(%r11, %r12), %xmm7
	vmovupd		32+16(%r11, %r14), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm7, %ymm7

	prefetcht0	1*256(%r13) // software prefetch
	prefetcht0	1*256+64(%r13) // software prefetch

	vunpcklpd	%ymm1, %ymm0, %ymm8
	vmovapd		%ymm8, 0(%r13)
	vunpckhpd	%ymm1, %ymm0, %ymm9
	vmovapd		%ymm9, 32(%r13)
	vunpcklpd	%ymm3, %ymm2, %ymm10
	vmovapd		%ymm10, 64(%r13)
	vunpckhpd	%ymm3, %ymm2, %ymm11
	vmovapd		%ymm11, 96(%r13)

	prefetcht0	1*256+128(%r13) // software prefetch
	prefetcht0	1*256+128+64(%r13) // software prefetch

	vunpcklpd	%ymm5, %ymm4, %ymm8
	vmovapd		%ymm8, 128+0(%r13)
	vunpckhpd	%ymm5, %ymm4, %ymm9
	vmovapd		%ymm9, 128+32(%r13)
	vunpcklpd	%ymm7, %ymm6, %ymm10
	vmovapd		%ymm10, 128+64(%r13)
	vunpckhpd	%ymm7, %ymm6, %ymm11
	vmovapd		%ymm11, 128+96(%r13)

	subl	$ 8, %r10d
	addq	$ 64, %r11
	addq	$ 256, %r13

	cmpl	$ 7, %r10d
	jg		1b

2: // consider clean-up

	cmpl	$ 7, %r10d
	jle		2f // consider clean-up loop


	vmovupd		0(%r11), %xmm0
	vmovupd		0(%r11, %r12, 2), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm0, %ymm0
	vmovupd		0(%r11, %r12), %xmm1
	vmovupd		0(%r11, %r14), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm1, %ymm1
	vmovupd		16(%r11), %xmm2
	vmovupd		16(%r11, %r12, 2), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm2, %ymm2
	vmovupd		16(%r11, %r12), %xmm3
	vmovupd		16(%r11, %r14), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm3, %ymm3

	vmovupd		32+0(%r11), %xmm4
	vmovupd		32+0(%r11, %r12, 2), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm4, %ymm4
	vmovupd		32+0(%r11, %r12), %xmm5
	vmovupd		32+0(%r11, %r14), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm5, %ymm5
	vmovupd		32+16(%r11), %xmm6
	vmovupd		32+16(%r11, %r12, 2), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm6, %ymm6
	vmovupd		32+16(%r11, %r12), %xmm7
	vmovupd		32+16(%r11, %r14), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm7, %ymm7

	vunpcklpd	%ymm1, %ymm0, %ymm8
	vmovapd		%ymm8, 0(%r13)
	vunpckhpd	%ymm1, %ymm0, %ymm9
	vmovapd		%ymm9, 32(%r13)
	vunpcklpd	%ymm3, %ymm2, %ymm10
	vmovapd		%ymm10, 64(%r13)
	vunpckhpd	%ymm3, %ymm2, %ymm11
	vmovapd		%ymm11, 96(%r13)

	vunpcklpd	%ymm5, %ymm4, %ymm8
	vmovapd		%ymm8, 128+0(%r13)
	vunpckhpd	%ymm5, %ymm4, %ymm9
	vmovapd		%ymm9, 128+32(%r13)
	vunpcklpd	%ymm7, %ymm6, %ymm10
	vmovapd		%ymm10, 128+64(%r13)
	vunpckhpd	%ymm7, %ymm6, %ymm11
	vmovapd		%ymm11, 128+96(%r13)

	subl	$ 8, %r10d
	addq	$ 64, %r11
	addq	$ 256, %r13

//	cmpl	$ 0, %r10d
	jle		0f // return


2: // consider clean-up

	cmpl	$ 3, %r10d
	jle		2f // consider clean-up loop

	leaq	0(%r11, %r12, 4), %r15

	prefetcht0	0(%r15)
	prefetcht0	0(%r15, %r12)
	prefetcht0	0(%r15, %r12, 2)
	prefetcht0	0(%r15, %r14)

	vmovupd		0(%r11), %xmm0
	vmovupd		0(%r11, %r12, 2), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm0, %ymm0
	vmovupd		0(%r11, %r12), %xmm1
	vmovupd		0(%r11, %r14), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm1, %ymm1
	vmovupd		16(%r11), %xmm2
	vmovupd		16(%r11, %r12, 2), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm2, %ymm2
	vmovupd		16(%r11, %r12), %xmm3
	vmovupd		16(%r11, %r14), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm3, %ymm3

	vunpcklpd	%ymm1, %ymm0, %ymm4
	vmovapd		%ymm4, 0(%r13)
	vunpckhpd	%ymm1, %ymm0, %ymm5
	vmovapd		%ymm5, 32(%r13)
	vunpcklpd	%ymm3, %ymm2, %ymm6
	vmovapd		%ymm6, 64(%r13)
	vunpckhpd	%ymm3, %ymm2, %ymm7
	vmovapd		%ymm7, 96(%r13)

	subl	$ 4, %r10d
	addq	$ 32, %r11
	addq	$ 128, %r13


2: // consider clean-up loop

	cmpl	$ 0, %r10d
	jle		0f // return


1: // clean-up loop
	
	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13)
	vmovsd		0(%r11, %r12), %xmm0
	vmovsd		%xmm0, 8(%r13)
	vmovsd		0(%r11, %r12, 2), %xmm0
	vmovsd		%xmm0, 16(%r13)
	vmovsd		0(%r11, %r14), %xmm0
	vmovsd		%xmm0, 24(%r13)

	subl	$ 1, %r10d
	addq	$ 8, %r11
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		1b


0: // return



#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpack_tn_4_p0_lib4)
#endif





// input arguments
//
// r10	<- kmax
// r11	<- A
// r12	<- lda
// r13	<- C
//
#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACK_TN_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpack_tn_4_lib4)
#endif



	cmpl	$ 0, %r10d
	jle		0f // return

	leaq	0(%r12, %r12, 2), %r14

	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovupd		0(%r11), %xmm0
	vmovupd		0(%r11, %r12, 2), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm0, %ymm0
	vmovupd		0(%r11, %r12), %xmm1
	vmovupd		0(%r11, %r14), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm1, %ymm1
	vmovupd		16(%r11), %xmm2
	vmovupd		16(%r11, %r12, 2), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm2, %ymm2
	vmovupd		16(%r11, %r12), %xmm3
	vmovupd		16(%r11, %r14), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm3, %ymm3

	vunpcklpd	%ymm1, %ymm0, %ymm4
	vmovapd		%ymm4, 0(%r13)
	vunpckhpd	%ymm1, %ymm0, %ymm5
	vmovapd		%ymm5, 32(%r13)
	vunpcklpd	%ymm3, %ymm2, %ymm6
	vmovapd		%ymm6, 64(%r13)
	vunpckhpd	%ymm3, %ymm2, %ymm7
	vmovapd		%ymm7, 96(%r13)

	subl	$ 4, %r10d
	addq	$ 32, %r11
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13)
	vmovsd		0(%r11, %r12), %xmm0
	vmovsd		%xmm0, 8(%r13)
	vmovsd		0(%r11, %r12, 2), %xmm0
	vmovsd		%xmm0, 16(%r13)
	vmovsd		0(%r11, %r14), %xmm0
	vmovsd		%xmm0, 24(%r13)

	subl	$ 1, %r10d
	addq	$ 8, %r11
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return



#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpack_tn_4_lib4)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACK_TN_3_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpack_tn_3_lib4)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return


	movq	%r11, %r14
	addq	%r12, %r14
	addq	%r12, %r14


	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovupd		0(%r11), %xmm0
	vmovupd		0(%r14), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm0, %ymm0
	vmovupd		0(%r11, %r12, 1), %xmm1
	vmovupd		16(%r11), %xmm2
	vmovupd		16(%r14), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm2, %ymm2
	vmovupd		16(%r11, %r12, 1), %xmm3

	vunpcklpd	%ymm1, %ymm0, %ymm4
	vmovapd		%ymm4, 0(%r13)
	vunpckhpd	%ymm1, %ymm0, %ymm5
	vmovapd		%ymm5, 32(%r13)
	vunpcklpd	%ymm3, %ymm2, %ymm6
	vmovapd		%ymm6, 64(%r13)
	vunpckhpd	%ymm3, %ymm2, %ymm7
	vmovapd		%ymm7, 96(%r13)

	subl	$ 4, %r10d
	addq	$ 32, %r11
	addq	$ 32, %r14
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13)
	vmovsd		0(%r11, %r12, 1), %xmm0
	vmovsd		%xmm0, 8(%r13)
	vmovsd		0(%r14), %xmm0
	vmovsd		%xmm0, 16(%r13)

	subl	$ 1, %r10d
	addq	$ 8, %r11
	addq	$ 8, %r14
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpack_tn_3_lib4)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACK_TN_2_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpack_tn_2_lib4)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return


	movq	%r11, %r14
	addq	%r12, %r14
	addq	%r12, %r14


	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovupd		0(%r11), %xmm0
	vmovupd		0(%r11, %r12, 1), %xmm1
	vmovupd		16(%r11), %xmm2
	vmovupd		16(%r11, %r12, 1), %xmm3

	vunpcklpd	%ymm1, %ymm0, %ymm4
	vmovapd		%ymm4, 0(%r13)
	vunpckhpd	%ymm1, %ymm0, %ymm5
	vmovapd		%ymm5, 32(%r13)
	vunpcklpd	%ymm3, %ymm2, %ymm6
	vmovapd		%ymm6, 64(%r13)
	vunpckhpd	%ymm3, %ymm2, %ymm7
	vmovapd		%ymm7, 96(%r13)

	subl	$ 4, %r10d
	addq	$ 32, %r11
	addq	$ 32, %r14
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13)
	vmovsd		0(%r11, %r12, 1), %xmm0
	vmovsd		%xmm0, 8(%r13)

	subl	$ 1, %r10d
	addq	$ 8, %r11
	addq	$ 8, %r14
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpack_tn_2_lib4)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACK_TN_1_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpack_tn_1_lib4)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return


	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13)
	vmovsd		8(%r11), %xmm0
	vmovsd		%xmm0, 32(%r13)
	vmovsd		16(%r11), %xmm0
	vmovsd		%xmm0, 64(%r13)
	vmovsd		24(%r11), %xmm0
	vmovsd		%xmm0, 96(%r13)

	subl	$ 4, %r10d
	addq	$ 32, %r11
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13)

	subl	$ 1, %r10d
	addq	$ 8, %r11
	addq	$ 8, %r14
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpack_tn_1_lib4)
#endif





// input arguments
//
// r10	<- kmax
// r11	<- A
// r12	<- lda
// r13	<- C
// r14	<- sdc
//
#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACK_TT_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpack_tt_4_lib4)
#endif



#if 1



	cmpl	$ 0, %r10d
	jle		0f // return

	leaq	0(%r12, %r12, 2), %r15

	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
//	prefetcht2	0(%r13, %r14, 4) // software prefetch
//	prefetcht2	64(%r13, %r14, 4) // software prefetch

//	prefetchnta	2*64(%r11)
//	prefetchnta	2*64(%r11, %r12)
//	prefetchnta	2*64(%r11, %r12, 2)
//	prefetchnta	2*64(%r11, %r15)

	vmovupd		0(%r11), %ymm0
	vmovupd		0(%r11, %r12), %ymm1
	vmovupd		0(%r11, %r12, 2), %ymm2
	vmovupd		0(%r11, %r15), %ymm3

	vmovapd		%ymm0, 0(%r13)
	vmovapd		%ymm1, 32(%r13)
	vmovapd		%ymm2, 64(%r13)
	vmovapd		%ymm3, 96(%r13)

	subl	$ 4, %r10d
	addq	$ 32, %r11
	addq	%r14, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13)

	vmovsd		0(%r11, %r12), %xmm0
	vmovsd		%xmm0, 32(%r13)

	vmovsd		0(%r11, %r12, 2), %xmm0
	vmovsd		%xmm0, 64(%r13)

	vmovsd		0(%r11, %r15), %xmm0
	vmovsd		%xmm0, 96(%r13)

	subl	$ 1, %r10d
	addq	$ 8, %r11
	addq	$ 8, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return




#else



	cmpl	$ 0, %r10d
	jle		0f // return

	leaq	0(%r12, %r12, 2), %r15

	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	movl	%r10d, %ecx

3:
	movl	%ecx, %r10d
	movl	$ 512, %edx
//	movl	$ 256, %edx
//	movl	$ 8, %edx
	cmpl	%edx, %r10d
	cmovgl	%edx, %r10d


// TODO block for e.g. L1 cache

	movl	%r10d, %eax
	movq	%r11, %rbx

//	.p2align 3
1: // prefetch loop
	// TODO use load instead on sandybridge ???
	prefetchnta	0(%rbx)
	prefetchnta	0(%rbx, %r12)
	prefetchnta	0(%rbx, %r12, 2)
	prefetchnta	0(%rbx, %r14)

	addq	$ 32, %rbx
	subl	$ 8, %eax

	cmpl	$ 0, %eax
	jg		1b



	// main loop
	.p2align 3
1: // main loop
	
//	prefetcht2	0(%r13, %r14, 4) // software prefetch
//	prefetcht2	64(%r13, %r14, 4) // software prefetch

	vmovupd		0(%r11), %ymm0
	vmovupd		0(%r11, %r12), %ymm1
	vmovupd		0(%r11, %r12, 2), %ymm2
	vmovupd		0(%r11, %r15), %ymm3

	vmovapd		%ymm0, 0(%r13)
	vmovapd		%ymm1, 32(%r13)
	vmovapd		%ymm2, 64(%r13)
	vmovapd		%ymm3, 96(%r13)

	subl	$ 4, %r10d
	addq	$ 32, %r11
	addq	%r14, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
//	jle		0f // return
	jle		4f // return


2: // clean-up loop
	
	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13)

	vmovsd		0(%r11, %r12), %xmm0
	vmovsd		%xmm0, 32(%r13)

	vmovsd		0(%r11, %r12, 2), %xmm0
	vmovsd		%xmm0, 64(%r13)

	vmovsd		0(%r11, %r15), %xmm0
	vmovsd		%xmm0, 96(%r13)

	subl	$ 1, %r10d
	addq	$ 8, %r11
	addq	$ 8, %r13

	cmpl	$ 0, %r10d
	jg		2b


4:
	subl	$ 512, %ecx
//	subl	$ 256, %ecx
//	subl	$ 8, %ecx
	cmpl	$ 0, %ecx
	jg		3b


0: // return



#endif



#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpack_tt_4_lib4)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACK_TT_3_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpack_tt_3_lib4)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return


	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	movq	%r11, %r15

	vmovupd		0(%r15), %ymm0
	vmovapd		%ymm0, 0(%r13)
	addq	%r12, %r15

	vmovupd		0(%r15), %ymm0
	vmovapd		%ymm0, 32(%r13)
	addq	%r12, %r15

	vmovupd		0(%r15), %ymm0
	vmovapd		%ymm0, 64(%r13)
	addq	%r12, %r15

	subl	$ 4, %r10d
	addq	$ 32, %r11
	addq	%r14, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	movq	%r11, %r15

	vmovsd		0(%r15), %xmm0
	vmovsd		%xmm0, 0(%r13)
	addq	%r12, %r15

	vmovsd		0(%r15), %xmm0
	vmovsd		%xmm0, 32(%r13)
	addq	%r12, %r15

	vmovsd		0(%r15), %xmm0
	vmovsd		%xmm0, 64(%r13)
	addq	%r12, %r15

	subl	$ 1, %r10d
	addq	$ 8, %r11
	addq	$ 8, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpack_tt_3_lib4)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACK_TT_2_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpack_tt_2_lib4)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return


	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	movq	%r11, %r15

	vmovupd		0(%r15), %ymm0
	vmovapd		%ymm0, 0(%r13)
	addq	%r12, %r15

	vmovupd		0(%r15), %ymm0
	vmovapd		%ymm0, 32(%r13)
	addq	%r12, %r15

	subl	$ 4, %r10d
	addq	$ 32, %r11
	addq	%r14, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	movq	%r11, %r15

	vmovsd		0(%r15), %xmm0
	vmovsd		%xmm0, 0(%r13)
	addq	%r12, %r15

	vmovsd		0(%r15), %xmm0
	vmovsd		%xmm0, 32(%r13)
	addq	%r12, %r15

	subl	$ 1, %r10d
	addq	$ 8, %r11
	addq	$ 8, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpack_tt_2_lib4)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACK_TT_1_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpack_tt_1_lib4)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return


	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	movq	%r11, %r15

	vmovupd		0(%r15), %ymm0
	vmovapd		%ymm0, 0(%r13)
	addq	%r12, %r15

	subl	$ 4, %r10d
	addq	$ 32, %r11
	addq	%r14, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	movq	%r11, %r15

	vmovsd		0(%r15), %xmm0
	vmovsd		%xmm0, 0(%r13)
	addq	%r12, %r15

	subl	$ 1, %r10d
	addq	$ 8, %r11
	addq	$ 8, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpack_tt_1_lib4)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DUNPACK_NT_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dunpack_nt_4_lib4)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return


	movq	%r12, %r14
	addq	%r13, %r14
	addq	%r13, %r14


	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovapd		0(%r11), %xmm0
	vmovapd		64(%r11), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm0, %ymm0
	vmovapd		32(%r11), %xmm1
	vmovapd		96(%r11), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm1, %ymm1
	vmovapd		16(%r11), %xmm2
	vmovapd		80(%r11), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm2, %ymm2
	vmovapd		48(%r11), %xmm3
	vmovapd		112(%r11), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm3, %ymm3

	vunpcklpd	%ymm1, %ymm0, %ymm4
	vmovupd		%ymm4, 0(%r12)
	vunpckhpd	%ymm1, %ymm0, %ymm5
	vmovupd		%ymm5, 0(%r12, %r13, 1)
	vunpcklpd	%ymm3, %ymm2, %ymm6
	vmovupd		%ymm6, 0(%r14)
	vunpckhpd	%ymm3, %ymm2, %ymm7
	vmovupd		%ymm7, 0(%r14, %r13, 1)

	subl	$ 4, %r10d
	addq	$ 32, %r12
	addq	$ 32, %r14
	addq	$ 128, %r11

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r12)
	vmovsd		8(%r11), %xmm0
	vmovsd		%xmm0, 0(%r12, %r13, 1)
	vmovsd		16(%r11), %xmm0
	vmovsd		%xmm0, 0(%r14)
	vmovsd		24(%r11), %xmm0
	vmovsd		%xmm0, 0(%r14, %r13, 1)

	subl	$ 1, %r10d
	addq	$ 8, %r12
	addq	$ 8, %r14
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dunpack_nt_4_lib4)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DUNPACK_NT_3_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dunpack_nt_3_lib4)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return


	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovapd		0(%r11), %xmm0
	vmovapd		64(%r11), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm0, %ymm0
	vmovapd		32(%r11), %xmm1
	vmovapd		96(%r11), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm1, %ymm1
	vmovapd		16(%r11), %xmm2
	vmovapd		80(%r11), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm2, %ymm2
	vmovapd		48(%r11), %xmm3
	vmovapd		112(%r11), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm3, %ymm3

	vunpcklpd	%ymm1, %ymm0, %ymm4
	vmovupd		%ymm4, 0(%r12)
	vunpckhpd	%ymm1, %ymm0, %ymm5
	vmovupd		%ymm5, 0(%r12, %r13, 1)
	vunpcklpd	%ymm3, %ymm2, %ymm6
	vmovupd		%ymm6, 0(%r12, %r13, 2)

	subl	$ 4, %r10d
	addq	$ 32, %r12
	addq	$ 128, %r11

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r12)
	vmovsd		8(%r11), %xmm0
	vmovsd		%xmm0, 0(%r12, %r13, 1)
	vmovsd		16(%r11), %xmm0
	vmovsd		%xmm0, 0(%r12, %r13, 2)

	subl	$ 1, %r10d
	addq	$ 8, %r12
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dunpack_nt_3_lib4)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DUNPACK_NT_2_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dunpack_nt_2_lib4)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return


	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovapd		0(%r11), %xmm0
	vmovapd		64(%r11), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm0, %ymm0
	vmovapd		32(%r11), %xmm1
	vmovapd		96(%r11), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm1, %ymm1

	vunpcklpd	%ymm1, %ymm0, %ymm4
	vmovupd		%ymm4, 0(%r12)
	vunpckhpd	%ymm1, %ymm0, %ymm5
	vmovupd		%ymm5, 0(%r12, %r13, 1)

	subl	$ 4, %r10d
	addq	$ 32, %r12
	addq	$ 128, %r11

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r12)
	vmovsd		8(%r11), %xmm0
	vmovsd		%xmm0, 0(%r12, %r13, 1)

	subl	$ 1, %r10d
	addq	$ 8, %r12
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dunpack_nt_2_lib4)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DUNPACK_NT_1_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dunpack_nt_1_lib4)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return


	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r12)
	vmovsd		32(%r11), %xmm0
	vmovsd		%xmm0, 8(%r12)
	vmovsd		64(%r11), %xmm0
	vmovsd		%xmm0, 16(%r12)
	vmovsd		96(%r11), %xmm0
	vmovsd		%xmm0, 24(%r12)

	subl	$ 4, %r10d
	addq	$ 32, %r12
	addq	$ 128, %r11

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r12)

	subl	$ 1, %r10d
	addq	$ 8, %r12
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dunpack_nt_1_lib4)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACP_NT_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpacp_nt_4_lib4)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return

	cmpl	$ 0, %r11d
	jle		3f // offset==0

	movl			$ 4, %r15d				// load 4
	subl			%r12d, %r15d			// 4-offsetC
	cmpl			%r10d, %r15d			// k > 4-offsetC
	cmovgl			%r10d, %r15d			// kend=min(k,4-offsetC)

	movl			%r12d, %eax				// load offsetC
	sall			$ 3, %eax				// offsetC*sizeof(double)
	addq			%rax, %r13				// C+offsetC*sizeof(double)

4:
	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13)
	vmovsd		8(%r11), %xmm0
	vmovsd		%xmm0, 32(%r13)
	vmovsd		16(%r11), %xmm0
	vmovsd		%xmm0, 64(%r13)
	vmovsd		24(%r11), %xmm0
	vmovsd		%xmm0, 96(%r13)

	subl			$ 1, %r10d				// k=-1
	subl			$ 1, %r15d				// k_panel=-1
	addq			$ 8, %r13				// C=+1
	addq			$ 32, %r11				// A=+bs

	cmpl			$ 0, %r15d				// if k_panel=0
	jg				4b						// loop 1

	cmpl			$ 0, %r10d				// if k=0
	jle				0f						// end

	addq			%r14, %r13				// C=Coff+sdc*bs
	subq			$ 32, %r13				// C-=4*sizeof(double) (loop+offsetC)


3:
	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
//	prefetcht0	0(%r13, %r14, 1) // software prefetch
//	prefetcht0	64(%r13, %r14, 1) // software prefetch

	vmovapd		0(%r11), %xmm0
	vmovapd		64(%r11), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm0, %ymm0
	vmovapd		32(%r11), %xmm1
	vmovapd		96(%r11), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm1, %ymm1
	vmovapd		16(%r11), %xmm2
	vmovapd		80(%r11), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm2, %ymm2
	vmovapd		48(%r11), %xmm3
	vmovapd		112(%r11), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm3, %ymm3

	vunpcklpd	%ymm1, %ymm0, %ymm4
	vmovapd		%ymm4, 0(%r13)
	vunpckhpd	%ymm1, %ymm0, %ymm5
	vmovapd		%ymm5, 32(%r13)
	vunpcklpd	%ymm3, %ymm2, %ymm6
	vmovapd		%ymm6, 64(%r13)
	vunpckhpd	%ymm3, %ymm2, %ymm7
	vmovapd		%ymm7, 96(%r13)

	subl	$ 4, %r10d
	addq	%r14, %r13
	addq	$ 128, %r11

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13)
	vmovsd		8(%r11), %xmm0
	vmovsd		%xmm0, 32(%r13)
	vmovsd		16(%r11), %xmm0
	vmovsd		%xmm0, 64(%r13)
	vmovsd		24(%r11), %xmm0
	vmovsd		%xmm0, 96(%r13)

	subl	$ 1, %r10d
	addq	$ 8, %r13
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_dpacp_nt_4_lib4, .-inner_kernel_dpacp_nt_4_lib4
#endif
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACP_TN_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpacp_tn_4_lib4)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return

	cmpl	$ 0, %r11d
	jle		3f // offset==0

	movl			$ 4, %r15d				// load 4
	subl			%r11d, %r15d			// 4-offsetA
	cmpl			%r10d, %r15d			// k > 4-offsetA
	cmovgl			%r10d, %r15d			// kend=min(k,4-offsetA)

	movl			%r11d, %eax				// load offsetA
	sall			$ 3, %eax				// offsetA*sizeof(double)
	addq			%rax, %r12				// B+offsetB*sizeof(double)

4:
	vmovsd		0(%r12), %xmm0
	vmovsd		%xmm0, 0(%r14)
	vmovsd		32(%r12), %xmm0
	vmovsd		%xmm0, 8(%r14)
	vmovsd		64(%r12), %xmm0
	vmovsd		%xmm0, 16(%r14)
	vmovsd		96(%r12), %xmm0
	vmovsd		%xmm0, 24(%r14)

	subl			$ 1, %r10d				// k=-1
	subl			$ 1, %r15d				// k_panel=-1
	addq			$ 8, %r12				// A=+1
	addq			$ 32, %r14				// C=+bs

	cmpl			$ 0, %r15d				// if k_panel=0
	jg				4b						// loop 1

	cmpl			$ 0, %r10d				// if k=0
	jle				0f						// end

	addq			%r13, %r12				// A=Boff+sdb*bs
	subq			$ 32, %r12				// A-=4*sizeof(double) (loop+offsetA)


3:
	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
//	prefetcht0	0(%r12, %r13, 1) // software prefetch
//	prefetcht0	64(%r12, %r13, 1) // software prefetch

	vmovapd		0(%r12), %xmm0
	vmovapd		64(%r12), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm0, %ymm0
	vmovapd		32(%r12), %xmm1
	vmovapd		96(%r12), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm1, %ymm1
	vmovapd		16(%r12), %xmm2
	vmovapd		80(%r12), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm2, %ymm2
	vmovapd		48(%r12), %xmm3
	vmovapd		112(%r12), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm3, %ymm3

	vunpcklpd	%ymm1, %ymm0, %ymm4
	vmovapd		%ymm4, 0(%r14)
	vunpckhpd	%ymm1, %ymm0, %ymm5
	vmovapd		%ymm5, 32(%r14)
	vunpcklpd	%ymm3, %ymm2, %ymm6
	vmovapd		%ymm6, 64(%r14)
	vunpckhpd	%ymm3, %ymm2, %ymm7
	vmovapd		%ymm7, 96(%r14)

	subl	$ 4, %r10d
	addq	%r13, %r12
	addq	$ 128, %r14

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovsd		0(%r12), %xmm0
	vmovsd		%xmm0, 0(%r14)
	vmovsd		32(%r12), %xmm0
	vmovsd		%xmm0, 8(%r14)
	vmovsd		64(%r12), %xmm0
	vmovsd		%xmm0, 16(%r14)
	vmovsd		96(%r12), %xmm0
	vmovsd		%xmm0, 24(%r14)

	subl	$ 1, %r10d
	addq	$ 8, %r12
	addq	$ 32, %r14

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpacp_tn_4_lib4)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACP_NN_12_0_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpacp_nn_12_0_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		0f // return

	cmpl	$ 3, %r10d
	jle		2f // clean-up loop

	// main loop
	.p2align 3
1:
	//
	vmovapd		0(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13)
	vmovapd		0(%r11, %r12, 1), %ymm0
	vmovapd		%ymm0, 0(%r13, %r14, 1)
	vmovapd		0(%r11, %r12, 2), %ymm0
	vmovapd		%ymm0, 0(%r13, %r14, 2)
	//
	vmovapd		32(%r11), %ymm0
	vmovapd		%ymm0, 32(%r13)
	vmovapd		32(%r11, %r12, 1), %ymm0
	vmovapd		%ymm0, 32(%r13, %r14, 1)
	vmovapd		32(%r11, %r12, 2), %ymm0
	vmovapd		%ymm0, 32(%r13, %r14, 2)
	//
	vmovapd		64(%r11), %ymm0
	vmovapd		%ymm0, 64(%r13)
	vmovapd		64(%r11, %r12, 1), %ymm0
	vmovapd		%ymm0, 64(%r13, %r14, 1)
	vmovapd		64(%r11, %r12, 2), %ymm0
	vmovapd		%ymm0, 64(%r13, %r14, 2)
	//
	vmovapd		96(%r11), %ymm0
	vmovapd		%ymm0, 96(%r13)
	vmovapd		96(%r11, %r12, 1), %ymm0
	vmovapd		%ymm0, 96(%r13, %r14, 1)
	vmovapd		96(%r11, %r12, 2), %ymm0
	vmovapd		%ymm0, 96(%r13, %r14, 2)

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovapd		0(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13)
	vmovapd		0(%r11, %r12, 1), %ymm0
	vmovapd		%ymm0, 0(%r13, %r14, 1)
	vmovapd		0(%r11, %r12, 2), %ymm0
	vmovapd		%ymm0, 0(%r13, %r14, 2)

	subl	$ 1, %r10d
	addq	$ 32, %r11
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpacp_nn_12_0_lib4)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACP_NN_12_0_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpacp_nn_12_0_vs_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		0f // return

	vcvtsi2sd	%r15d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC02(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC02(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm15

	cmpl	$ 3, %r10d
	jle		2f // clean-up loop

	// main loop
	.p2align 3
1:
	//
	vmovapd		0(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13)
	vmovapd		0(%r11, %r12, 1), %ymm0
	vmovapd		%ymm0, 0(%r13, %r14, 1)
	vmovapd		0(%r11, %r12, 2), %ymm0
	vmaskmovpd	%ymm0, %ymm15, 0(%r13, %r14, 2)
	//
	vmovapd		32(%r11), %ymm0
	vmovapd		%ymm0, 32(%r13)
	vmovapd		32(%r11, %r12, 1), %ymm0
	vmovapd		%ymm0, 32(%r13, %r14, 1)
	vmovapd		32(%r11, %r12, 2), %ymm0
	vmaskmovpd	%ymm0, %ymm15, 32(%r13, %r14, 2)
	//
	vmovapd		64(%r11), %ymm0
	vmovapd		%ymm0, 64(%r13)
	vmovapd		64(%r11, %r12, 1), %ymm0
	vmovapd		%ymm0, 64(%r13, %r14, 1)
	vmovapd		64(%r11, %r12, 2), %ymm0
	vmaskmovpd	%ymm0, %ymm15, 64(%r13, %r14, 2)
	//
	vmovapd		96(%r11), %ymm0
	vmovapd		%ymm0, 96(%r13)
	vmovapd		96(%r11, %r12, 1), %ymm0
	vmovapd		%ymm0, 96(%r13, %r14, 1)
	vmovapd		96(%r11, %r12, 2), %ymm0
	vmaskmovpd	%ymm0, %ymm15, 96(%r13, %r14, 2)

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovapd		0(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13)
	vmovapd		0(%r11, %r12, 1), %ymm0
	vmovapd		%ymm0, 0(%r13, %r14, 1)
	vmovapd		0(%r11, %r12, 2), %ymm0
	vmaskmovpd	%ymm0, %ymm15, 0(%r13, %r14, 2)

	subl	$ 1, %r10d
	addq	$ 32, %r11
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpacp_nn_12_0_vs_lib4)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACP_NN_12_1_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpacp_nn_12_1_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		0f // return

	movq	%r11, %rax
	addq	%r12, %rax

	cmpl	$ 3, %r10d
	jle		2f // clean-up loop

	// main loop
	.p2align 3
1:
	//
	vmovapd		0(%r11), %ymm0
	vmovapd		0(%r11, %r12, 1), %ymm1
	vmovapd		0(%r11, %r12, 2), %ymm2
	vmovapd		0(%rax, %r12, 2), %ymm3
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm4
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm5
	vperm2f128	$ 0x21, %ymm3, %ymm2, %ymm3
	vshufpd		$ 0x5, %ymm4, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm5, %ymm1, %ymm1
	vshufpd		$ 0x5, %ymm3, %ymm2, %ymm2
	vmovapd		%ymm0, 0(%r13)
	vmovapd		%ymm1, 0(%r13, %r14, 1)
	vmovapd		%ymm2, 0(%r13, %r14, 2)
	//
	vmovapd		32(%r11), %ymm0
	vmovapd		32(%r11, %r12, 1), %ymm1
	vmovapd		32(%r11, %r12, 2), %ymm2
	vmovapd		32(%rax, %r12, 2), %ymm3
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm4
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm5
	vperm2f128	$ 0x21, %ymm3, %ymm2, %ymm3
	vshufpd		$ 0x5, %ymm4, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm5, %ymm1, %ymm1
	vshufpd		$ 0x5, %ymm3, %ymm2, %ymm2
	vmovapd		%ymm0, 32(%r13)
	vmovapd		%ymm1, 32(%r13, %r14)
	vmovapd		%ymm2, 32(%r13, %r14, 2)
	//
	vmovapd		64(%r11), %ymm0
	vmovapd		64(%r11, %r12, 1), %ymm1
	vmovapd		64(%r11, %r12, 2), %ymm2
	vmovapd		64(%rax, %r12, 2), %ymm3
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm4
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm5
	vperm2f128	$ 0x21, %ymm3, %ymm2, %ymm3
	vshufpd		$ 0x5, %ymm4, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm5, %ymm1, %ymm1
	vshufpd		$ 0x5, %ymm3, %ymm2, %ymm2
	vmovapd		%ymm0, 64(%r13)
	vmovapd		%ymm1, 64(%r13, %r14)
	vmovapd		%ymm2, 64(%r13, %r14, 2)
	//
	vmovapd		96(%r11), %ymm0
	vmovapd		96(%r11, %r12, 1), %ymm1
	vmovapd		96(%r11, %r12, 2), %ymm2
	vmovapd		96(%rax, %r12, 2), %ymm3
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm4
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm5
	vperm2f128	$ 0x21, %ymm3, %ymm2, %ymm3
	vshufpd		$ 0x5, %ymm4, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm5, %ymm1, %ymm1
	vshufpd		$ 0x5, %ymm3, %ymm2, %ymm2
	vmovapd		%ymm0, 96(%r13)
	vmovapd		%ymm1, 96(%r13, %r14)
	vmovapd		%ymm2, 96(%r13, %r14, 2)

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %rax
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovapd		0(%r11), %ymm0
	vmovapd		0(%r11, %r12, 1), %ymm1
	vmovapd		0(%r11, %r12, 2), %ymm2
	vmovapd		0(%rax, %r12, 2), %ymm3
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm4
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm5
	vperm2f128	$ 0x21, %ymm3, %ymm2, %ymm3
	vshufpd		$ 0x5, %ymm4, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm5, %ymm1, %ymm1
	vshufpd		$ 0x5, %ymm3, %ymm2, %ymm2
	vmovapd		%ymm0, 0(%r13)
	vmovapd		%ymm1, 0(%r13, %r14)
	vmovapd		%ymm2, 0(%r13, %r14, 2)

	subl	$ 1, %r10d
	addq	$ 32, %r11
	addq	$ 32, %rax
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpacp_nn_12_1_lib4)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACP_NN_12_1_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpacp_nn_12_1_vs_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		0f // return

	vcvtsi2sd	%r15d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC02(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC02(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm15

	movq	%r11, %rax
	addq	%r12, %rax

	cmpl	$ 3, %r10d
	jle		2f // clean-up loop

	// main loop
	.p2align 3
1:
	//
	vmovapd		0(%r11), %ymm0
	vmovapd		0(%r11, %r12, 1), %ymm1
	vmovapd		0(%r11, %r12, 2), %ymm2
	vmovapd		0(%rax, %r12, 2), %ymm3
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm4
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm5
	vperm2f128	$ 0x21, %ymm3, %ymm2, %ymm3
	vshufpd		$ 0x5, %ymm4, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm5, %ymm1, %ymm1
	vshufpd		$ 0x5, %ymm3, %ymm2, %ymm2
	vmovapd		%ymm0, 0(%r13)
	vmovapd		%ymm1, 0(%r13, %r14, 1)
	vmaskmovpd	%ymm2, %ymm15, 0(%r13, %r14, 2)
	//
	vmovapd		32(%r11), %ymm0
	vmovapd		32(%r11, %r12, 1), %ymm1
	vmovapd		32(%r11, %r12, 2), %ymm2
	vmovapd		32(%rax, %r12, 2), %ymm3
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm4
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm5
	vperm2f128	$ 0x21, %ymm3, %ymm2, %ymm3
	vshufpd		$ 0x5, %ymm4, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm5, %ymm1, %ymm1
	vshufpd		$ 0x5, %ymm3, %ymm2, %ymm2
	vmovapd		%ymm0, 32(%r13)
	vmovapd		%ymm1, 32(%r13, %r14)
	vmaskmovpd	%ymm2, %ymm15, 32(%r13, %r14, 2)
	//
	vmovapd		64(%r11), %ymm0
	vmovapd		64(%r11, %r12, 1), %ymm1
	vmovapd		64(%r11, %r12, 2), %ymm2
	vmovapd		64(%rax, %r12, 2), %ymm3
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm4
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm5
	vperm2f128	$ 0x21, %ymm3, %ymm2, %ymm3
	vshufpd		$ 0x5, %ymm4, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm5, %ymm1, %ymm1
	vshufpd		$ 0x5, %ymm3, %ymm2, %ymm2
	vmovapd		%ymm0, 64(%r13)
	vmovapd		%ymm1, 64(%r13, %r14)
	vmaskmovpd	%ymm2, %ymm15, 64(%r13, %r14, 2)
	//
	vmovapd		96(%r11), %ymm0
	vmovapd		96(%r11, %r12, 1), %ymm1
	vmovapd		96(%r11, %r12, 2), %ymm2
	vmovapd		96(%rax, %r12, 2), %ymm3
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm4
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm5
	vperm2f128	$ 0x21, %ymm3, %ymm2, %ymm3
	vshufpd		$ 0x5, %ymm4, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm5, %ymm1, %ymm1
	vshufpd		$ 0x5, %ymm3, %ymm2, %ymm2
	vmovapd		%ymm0, 96(%r13)
	vmovapd		%ymm1, 96(%r13, %r14)
	vmaskmovpd	%ymm2, %ymm15, 96(%r13, %r14, 2)

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %rax
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovapd		0(%r11), %ymm0
	vmovapd		0(%r11, %r12, 1), %ymm1
	vmovapd		0(%r11, %r12, 2), %ymm2
	vmovapd		0(%rax, %r12, 2), %ymm3
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm4
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm5
	vperm2f128	$ 0x21, %ymm3, %ymm2, %ymm3
	vshufpd		$ 0x5, %ymm4, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm5, %ymm1, %ymm1
	vshufpd		$ 0x5, %ymm3, %ymm2, %ymm2
	vmovapd		%ymm0, 0(%r13)
	vmovapd		%ymm1, 0(%r13, %r14)
	vmaskmovpd	%ymm2, %ymm15, 0(%r13, %r14, 2)

	subl	$ 1, %r10d
	addq	$ 32, %r11
	addq	$ 32, %rax
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpacp_nn_12_1_vs_lib4)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACP_NN_12_2_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpacp_nn_12_2_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		0f // return

	movq	%r11, %rax
	addq	%r12, %rax

	cmpl	$ 3, %r10d
	jle		2f // clean-up loop

	// main loop
	.p2align 3
1:
	//
	vmovapd		0(%r11), %ymm0
	vmovapd		0(%r11, %r12, 1), %ymm1
	vmovapd		0(%r11, %r12, 2), %ymm2
	vmovapd		0(%rax, %r12, 2), %ymm3
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm1
	vperm2f128	$ 0x21, %ymm3, %ymm2, %ymm2
	vmovapd		%ymm0, 0(%r13)
	vmovapd		%ymm1, 0(%r13, %r14)
	vmovapd		%ymm2, 0(%r13, %r14, 2)
	//
	vmovapd		32(%r11), %ymm0
	vmovapd		32(%r11, %r12, 1), %ymm1
	vmovapd		32(%r11, %r12, 2), %ymm2
	vmovapd		32(%rax, %r12, 2), %ymm3
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm1
	vperm2f128	$ 0x21, %ymm3, %ymm2, %ymm2
	vmovapd		%ymm0, 32(%r13)
	vmovapd		%ymm1, 32(%r13, %r14)
	vmovapd		%ymm2, 32(%r13, %r14, 2)
	//
	vmovapd		64(%r11), %ymm0
	vmovapd		64(%r11, %r12, 1), %ymm1
	vmovapd		64(%r11, %r12, 2), %ymm2
	vmovapd		64(%rax, %r12, 2), %ymm3
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm1
	vperm2f128	$ 0x21, %ymm3, %ymm2, %ymm2
	vmovapd		%ymm0, 64(%r13)
	vmovapd		%ymm1, 64(%r13, %r14)
	vmovapd		%ymm2, 64(%r13, %r14, 2)
	//
	vmovapd		96(%r11), %ymm0
	vmovapd		96(%r11, %r12, 1), %ymm1
	vmovapd		96(%r11, %r12, 2), %ymm2
	vmovapd		96(%rax, %r12, 2), %ymm3
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm1
	vperm2f128	$ 0x21, %ymm3, %ymm2, %ymm2
	vmovapd		%ymm0, 96(%r13)
	vmovapd		%ymm1, 96(%r13, %r14)
	vmovapd		%ymm2, 96(%r13, %r14, 2)

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %rax
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovapd		0(%r11), %ymm0
	vmovapd		0(%r11, %r12, 1), %ymm1
	vmovapd		0(%r11, %r12, 2), %ymm2
	vmovapd		0(%rax, %r12, 2), %ymm3
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm1
	vperm2f128	$ 0x21, %ymm3, %ymm2, %ymm2
	vmovapd		%ymm0, 0(%r13)
	vmovapd		%ymm1, 0(%r13, %r14)
	vmovapd		%ymm2, 0(%r13, %r14, 2)

	subl	$ 1, %r10d
	addq	$ 32, %r11
	addq	$ 32, %rax
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpacp_nn_12_2_lib4)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACP_NN_12_2_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpacp_nn_12_2_vs_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		0f // return

	vcvtsi2sd	%r15d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC02(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC02(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm15

	movq	%r11, %rax
	addq	%r12, %rax

	cmpl	$ 3, %r10d
	jle		2f // clean-up loop

	// main loop
	.p2align 3
1:
	//
	vmovapd		0(%r11), %ymm0
	vmovapd		0(%r11, %r12, 1), %ymm1
	vmovapd		0(%r11, %r12, 2), %ymm2
	vmovapd		0(%rax, %r12, 2), %ymm3
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm1
	vperm2f128	$ 0x21, %ymm3, %ymm2, %ymm2
	vmovapd		%ymm0, 0(%r13)
	vmovapd		%ymm1, 0(%r13, %r14)
	vmaskmovpd	%ymm2, %ymm15, 0(%r13, %r14, 2)
	//
	vmovapd		32(%r11), %ymm0
	vmovapd		32(%r11, %r12, 1), %ymm1
	vmovapd		32(%r11, %r12, 2), %ymm2
	vmovapd		32(%rax, %r12, 2), %ymm3
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm1
	vperm2f128	$ 0x21, %ymm3, %ymm2, %ymm2
	vmovapd		%ymm0, 32(%r13)
	vmovapd		%ymm1, 32(%r13, %r14)
	vmaskmovpd	%ymm2, %ymm15, 32(%r13, %r14, 2)
	//
	vmovapd		64(%r11), %ymm0
	vmovapd		64(%r11, %r12, 1), %ymm1
	vmovapd		64(%r11, %r12, 2), %ymm2
	vmovapd		64(%rax, %r12, 2), %ymm3
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm1
	vperm2f128	$ 0x21, %ymm3, %ymm2, %ymm2
	vmovapd		%ymm0, 64(%r13)
	vmovapd		%ymm1, 64(%r13, %r14)
	vmaskmovpd	%ymm2, %ymm15, 64(%r13, %r14, 2)
	//
	vmovapd		96(%r11), %ymm0
	vmovapd		96(%r11, %r12, 1), %ymm1
	vmovapd		96(%r11, %r12, 2), %ymm2
	vmovapd		96(%rax, %r12, 2), %ymm3
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm1
	vperm2f128	$ 0x21, %ymm3, %ymm2, %ymm2
	vmovapd		%ymm0, 96(%r13)
	vmovapd		%ymm1, 96(%r13, %r14)
	vmaskmovpd	%ymm2, %ymm15, 96(%r13, %r14, 2)

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %rax
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovapd		0(%r11), %ymm0
	vmovapd		0(%r11, %r12, 1), %ymm1
	vmovapd		0(%r11, %r12, 2), %ymm2
	vmovapd		0(%rax, %r12, 2), %ymm3
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm1
	vperm2f128	$ 0x21, %ymm3, %ymm2, %ymm2
	vmovapd		%ymm0, 0(%r13)
	vmovapd		%ymm1, 0(%r13, %r14)
	vmaskmovpd	%ymm2, %ymm15, 0(%r13, %r14, 2)

	subl	$ 1, %r10d
	addq	$ 32, %r11
	addq	$ 32, %rax
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpacp_nn_12_2_vs_lib4)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACP_NN_12_3_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpacp_nn_12_3_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		0f // return

	movq	%r11, %rax
	addq	%r12, %rax

	cmpl	$ 3, %r10d
	jle		2f // clean-up loop

	// main loop
	.p2align 3
1:
	//
	vmovapd		0(%r11), %ymm0
	vmovapd		0(%r11, %r12, 1), %ymm1
	vmovapd		0(%r11, %r12, 2), %ymm2
	vmovapd		0(%rax, %r12, 2), %ymm3
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm4
	vperm2f128	$ 0x21, %ymm3, %ymm2, %ymm5
	vshufpd		$ 0x5, %ymm1, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm2, %ymm4, %ymm1
	vshufpd		$ 0x5, %ymm3, %ymm5, %ymm2
	vmovapd		%ymm0, 0(%r13)
	vmovapd		%ymm1, 0(%r13, %r14, 1)
	vmovapd		%ymm2, 0(%r13, %r14, 2)
	//
	vmovapd		32(%r11), %ymm0
	vmovapd		32(%r11, %r12, 1), %ymm1
	vmovapd		32(%r11, %r12, 2), %ymm2
	vmovapd		32(%rax, %r12, 2), %ymm3
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm4
	vperm2f128	$ 0x21, %ymm3, %ymm2, %ymm5
	vshufpd		$ 0x5, %ymm1, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm2, %ymm4, %ymm1
	vshufpd		$ 0x5, %ymm3, %ymm5, %ymm2
	vmovapd		%ymm0, 32(%r13)
	vmovapd		%ymm1, 32(%r13, %r14, 1)
	vmovapd		%ymm2, 32(%r13, %r14, 2)
	//
	vmovapd		64(%r11), %ymm0
	vmovapd		64(%r11, %r12, 1), %ymm1
	vmovapd		64(%r11, %r12, 2), %ymm2
	vmovapd		64(%rax, %r12, 2), %ymm3
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm4
	vperm2f128	$ 0x21, %ymm3, %ymm2, %ymm5
	vshufpd		$ 0x5, %ymm1, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm2, %ymm4, %ymm1
	vshufpd		$ 0x5, %ymm3, %ymm5, %ymm2
	vmovapd		%ymm0, 64(%r13)
	vmovapd		%ymm1, 64(%r13, %r14, 1)
	vmovapd		%ymm2, 64(%r13, %r14, 2)
	//
	vmovapd		96(%r11), %ymm0
	vmovapd		96(%r11, %r12, 1), %ymm1
	vmovapd		96(%r11, %r12, 2), %ymm2
	vmovapd		96(%rax, %r12, 2), %ymm3
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm4
	vperm2f128	$ 0x21, %ymm3, %ymm2, %ymm5
	vshufpd		$ 0x5, %ymm1, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm2, %ymm4, %ymm1
	vshufpd		$ 0x5, %ymm3, %ymm5, %ymm2
	vmovapd		%ymm0, 96(%r13)
	vmovapd		%ymm1, 96(%r13, %r14, 1)
	vmovapd		%ymm2, 96(%r13, %r14, 2)

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %rax
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovapd		0(%r11), %ymm0
	vmovapd		0(%r11, %r12, 1), %ymm1
	vmovapd		0(%r11, %r12, 2), %ymm2
	vmovapd		0(%rax, %r12, 2), %ymm3
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm4
	vperm2f128	$ 0x21, %ymm3, %ymm2, %ymm5
	vshufpd		$ 0x5, %ymm1, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm2, %ymm4, %ymm1
	vshufpd		$ 0x5, %ymm3, %ymm5, %ymm2
	vmovapd		%ymm0, 0(%r13)
	vmovapd		%ymm1, 0(%r13, %r14, 1)
	vmovapd		%ymm2, 0(%r13, %r14, 2)

	subl	$ 1, %r10d
	addq	$ 32, %r11
	addq	$ 32, %rax
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpacp_nn_12_3_lib4)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACP_NN_12_3_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpacp_nn_12_3_vs_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		0f // return

	movq	%r11, %rax
	addq	%r12, %rax

	vcvtsi2sd	%r15d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC02(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC02(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm15

	cmpl	$ 3, %r10d
	jle		2f // clean-up loop

	// main loop
	.p2align 3
1:
	//
	vmovapd		0(%r11), %ymm0
	vmovapd		0(%r11, %r12, 1), %ymm1
	vmovapd		0(%r11, %r12, 2), %ymm2
	vmovapd		0(%rax, %r12, 2), %ymm3
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm4
	vperm2f128	$ 0x21, %ymm3, %ymm2, %ymm5
	vshufpd		$ 0x5, %ymm1, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm2, %ymm4, %ymm1
	vshufpd		$ 0x5, %ymm3, %ymm5, %ymm2
	vmovapd		%ymm0, 0(%r13)
	vmovapd		%ymm1, 0(%r13, %r14, 1)
	vmaskmovpd	%ymm2, %ymm15, 0(%r13, %r14, 2)
	//
	vmovapd		32(%r11), %ymm0
	vmovapd		32(%r11, %r12, 1), %ymm1
	vmovapd		32(%r11, %r12, 2), %ymm2
	vmovapd		32(%rax, %r12, 2), %ymm3
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm4
	vperm2f128	$ 0x21, %ymm3, %ymm2, %ymm5
	vshufpd		$ 0x5, %ymm1, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm2, %ymm4, %ymm1
	vshufpd		$ 0x5, %ymm3, %ymm5, %ymm2
	vmovapd		%ymm0, 32(%r13)
	vmovapd		%ymm1, 32(%r13, %r14, 1)
	vmaskmovpd	%ymm2, %ymm15, 32(%r13, %r14, 2)
	//
	vmovapd		64(%r11), %ymm0
	vmovapd		64(%r11, %r12, 1), %ymm1
	vmovapd		64(%r11, %r12, 2), %ymm2
	vmovapd		64(%rax, %r12, 2), %ymm3
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm4
	vperm2f128	$ 0x21, %ymm3, %ymm2, %ymm5
	vshufpd		$ 0x5, %ymm1, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm2, %ymm4, %ymm1
	vshufpd		$ 0x5, %ymm3, %ymm5, %ymm2
	vmovapd		%ymm0, 64(%r13)
	vmovapd		%ymm1, 64(%r13, %r14, 1)
	vmaskmovpd	%ymm2, %ymm15, 64(%r13, %r14, 2)
	//
	vmovapd		96(%r11), %ymm0
	vmovapd		96(%r11, %r12, 1), %ymm1
	vmovapd		96(%r11, %r12, 2), %ymm2
	vmovapd		96(%rax, %r12, 2), %ymm3
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm4
	vperm2f128	$ 0x21, %ymm3, %ymm2, %ymm5
	vshufpd		$ 0x5, %ymm1, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm2, %ymm4, %ymm1
	vshufpd		$ 0x5, %ymm3, %ymm5, %ymm2
	vmovapd		%ymm0, 96(%r13)
	vmovapd		%ymm1, 96(%r13, %r14, 1)
	vmaskmovpd	%ymm2, %ymm15, 96(%r13, %r14, 2)

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %rax
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovapd		0(%r11), %ymm0
	vmovapd		0(%r11, %r12, 1), %ymm1
	vmovapd		0(%r11, %r12, 2), %ymm2
	vmovapd		0(%rax, %r12, 2), %ymm3
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm4
	vperm2f128	$ 0x21, %ymm3, %ymm2, %ymm5
	vshufpd		$ 0x5, %ymm1, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm2, %ymm4, %ymm1
	vshufpd		$ 0x5, %ymm3, %ymm5, %ymm2
	vmovapd		%ymm0, 0(%r13)
	vmovapd		%ymm1, 0(%r13, %r14, 1)
	vmaskmovpd	%ymm2, %ymm15, 0(%r13, %r14, 2)

	subl	$ 1, %r10d
	addq	$ 32, %r11
	addq	$ 32, %rax
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpacp_nn_12_3_vs_lib4)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACP_NN_8_0_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpacp_nn_8_0_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		0f // return

	cmpl	$ 3, %r10d
	jle		2f // clean-up loop

	// main loop
	.p2align 3
1:
	//
	vmovapd		0(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13)
	vmovapd		0(%r11, %r12), %ymm0
	vmovapd		%ymm0, 0(%r13, %r14)
	//
	vmovapd		32(%r11), %ymm0
	vmovapd		%ymm0, 32(%r13)
	vmovapd		32(%r11, %r12), %ymm0
	vmovapd		%ymm0, 32(%r13, %r14)
	//
	vmovapd		64(%r11), %ymm0
	vmovapd		%ymm0, 64(%r13)
	vmovapd		64(%r11, %r12), %ymm0
	vmovapd		%ymm0, 64(%r13, %r14)
	//
	vmovapd		96(%r11), %ymm0
	vmovapd		%ymm0, 96(%r13)
	vmovapd		96(%r11, %r12), %ymm0
	vmovapd		%ymm0, 96(%r13, %r14)

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovapd		0(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13)
	vmovapd		0(%r11, %r12), %ymm0
	vmovapd		%ymm0, 0(%r13, %r14)

	subl	$ 1, %r10d
	addq	$ 32, %r11
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpacp_nn_8_0_lib4)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACP_NN_8_0_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpacp_nn_8_0_vs_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		0f // return

	vcvtsi2sd	%r15d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC01(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC01(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm15

	cmpl	$ 3, %r10d
	jle		2f // clean-up loop

	// main loop
	.p2align 3
1:
	//
	vmovapd		0(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13)
	vmovapd		0(%r11, %r12), %ymm0
	vmaskmovpd	%ymm0, %ymm15, 0(%r13, %r14)
	//
	vmovapd		32(%r11), %ymm0
	vmovapd		%ymm0, 32(%r13)
	vmovapd		32(%r11, %r12), %ymm0
	vmaskmovpd	%ymm0, %ymm15, 32(%r13, %r14)
	//
	vmovapd		64(%r11), %ymm0
	vmovapd		%ymm0, 64(%r13)
	vmovapd		64(%r11, %r12), %ymm0
	vmaskmovpd	%ymm0, %ymm15, 64(%r13, %r14)
	//
	vmovapd		96(%r11), %ymm0
	vmovapd		%ymm0, 96(%r13)
	vmovapd		96(%r11, %r12), %ymm0
	vmaskmovpd	%ymm0, %ymm15, 96(%r13, %r14)

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovapd		0(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13)
	vmovapd		0(%r11, %r12), %ymm0
	vmaskmovpd	%ymm0, %ymm15, 0(%r13, %r14)

	subl	$ 1, %r10d
	addq	$ 32, %r11
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpacp_nn_8_0_vs_lib4)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACP_NN_8_1_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpacp_nn_8_1_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		0f // return

	cmpl	$ 3, %r10d
	jle		2f // clean-up loop

	// main loop
	.p2align 3
1:
	//
	vmovapd		0(%r11), %ymm0
	vmovapd		0(%r11, %r12, 1), %ymm1
	vmovapd		0(%r11, %r12, 2), %ymm2
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm3
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm2
	vshufpd		$ 0x5, %ymm3, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm2, %ymm1, %ymm1
	vmovapd		%ymm0, 0(%r13)
	vmovapd		%ymm1, 0(%r13, %r14)
	//
	vmovapd		32(%r11), %ymm0
	vmovapd		32(%r11, %r12, 1), %ymm1
	vmovapd		32(%r11, %r12, 2), %ymm2
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm3
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm2
	vshufpd		$ 0x5, %ymm3, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm2, %ymm1, %ymm1
	vmovapd		%ymm0, 32(%r13)
	vmovapd		%ymm1, 32(%r13, %r14)
	//
	vmovapd		64(%r11), %ymm0
	vmovapd		64(%r11, %r12, 1), %ymm1
	vmovapd		64(%r11, %r12, 2), %ymm2
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm3
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm2
	vshufpd		$ 0x5, %ymm3, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm2, %ymm1, %ymm1
	vmovapd		%ymm0, 64(%r13)
	vmovapd		%ymm1, 64(%r13, %r14)
	//
	vmovapd		96(%r11), %ymm0
	vmovapd		96(%r11, %r12, 1), %ymm1
	vmovapd		96(%r11, %r12, 2), %ymm2
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm3
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm2
	vshufpd		$ 0x5, %ymm3, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm2, %ymm1, %ymm1
	vmovapd		%ymm0, 96(%r13)
	vmovapd		%ymm1, 96(%r13, %r14)

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovapd		0(%r11), %ymm0
	vmovapd		0(%r11, %r12, 1), %ymm1
	vmovapd		0(%r11, %r12, 2), %ymm2
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm3
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm2
	vshufpd		$ 0x5, %ymm3, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm2, %ymm1, %ymm1
	vmovapd		%ymm0, 0(%r13)
	vmovapd		%ymm1, 0(%r13, %r14)

	subl	$ 1, %r10d
	addq	$ 32, %r11
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpacp_nn_8_1_lib4)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACP_NN_8_1_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpacp_nn_8_1_vs_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		0f // return

	vcvtsi2sd	%r15d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC01(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC01(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm15

	cmpl	$ 3, %r10d
	jle		2f // clean-up loop

	// main loop
	.p2align 3
1:
	//
	vmovapd		0(%r11), %ymm0
	vmovapd		0(%r11, %r12, 1), %ymm1
	vmovapd		0(%r11, %r12, 2), %ymm2
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm3
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm2
	vshufpd		$ 0x5, %ymm3, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm2, %ymm1, %ymm1
	vmovapd		%ymm0, 0(%r13)
	vmaskmovpd	%ymm1, %ymm15, 0(%r13, %r14)
	//
	vmovapd		32(%r11), %ymm0
	vmovapd		32(%r11, %r12, 1), %ymm1
	vmovapd		32(%r11, %r12, 2), %ymm2
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm3
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm2
	vshufpd		$ 0x5, %ymm3, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm2, %ymm1, %ymm1
	vmovapd		%ymm0, 32(%r13)
	vmaskmovpd	%ymm1, %ymm15, 32(%r13, %r14)
	//
	vmovapd		64(%r11), %ymm0
	vmovapd		64(%r11, %r12, 1), %ymm1
	vmovapd		64(%r11, %r12, 2), %ymm2
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm3
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm2
	vshufpd		$ 0x5, %ymm3, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm2, %ymm1, %ymm1
	vmovapd		%ymm0, 64(%r13)
	vmaskmovpd	%ymm1, %ymm15, 64(%r13, %r14)
	//
	vmovapd		96(%r11), %ymm0
	vmovapd		96(%r11, %r12, 1), %ymm1
	vmovapd		96(%r11, %r12, 2), %ymm2
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm3
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm2
	vshufpd		$ 0x5, %ymm3, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm2, %ymm1, %ymm1
	vmovapd		%ymm0, 96(%r13)
	vmaskmovpd	%ymm1, %ymm15, 96(%r13, %r14)

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovapd		0(%r11), %ymm0
	vmovapd		0(%r11, %r12, 1), %ymm1
	vmovapd		0(%r11, %r12, 2), %ymm2
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm3
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm2
	vshufpd		$ 0x5, %ymm3, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm2, %ymm1, %ymm1
	vmovapd		%ymm0, 0(%r13)
	vmaskmovpd	%ymm1, %ymm15, 0(%r13, %r14)

	subl	$ 1, %r10d
	addq	$ 32, %r11
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpacp_nn_8_1_vs_lib4)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACP_NN_8_2_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpacp_nn_8_2_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		0f // return

	cmpl	$ 3, %r10d
	jle		2f // clean-up loop

	// main loop
	.p2align 3
1:
	//
	vmovapd		0(%r11), %ymm0
	vmovapd		0(%r11, %r12, 1), %ymm1
	vmovapd		0(%r11, %r12, 2), %ymm2
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm1
	vmovapd		%ymm0, 0(%r13)
	vmovapd		%ymm1, 0(%r13, %r14)
	//
	vmovapd		32(%r11), %ymm0
	vmovapd		32(%r11, %r12, 1), %ymm1
	vmovapd		32(%r11, %r12, 2), %ymm2
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm1
	vmovapd		%ymm0, 32(%r13)
	vmovapd		%ymm1, 32(%r13, %r14)
	//
	vmovapd		64(%r11), %ymm0
	vmovapd		64(%r11, %r12, 1), %ymm1
	vmovapd		64(%r11, %r12, 2), %ymm2
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm1
	vmovapd		%ymm0, 64(%r13)
	vmovapd		%ymm1, 64(%r13, %r14)
	//
	vmovapd		96(%r11), %ymm0
	vmovapd		96(%r11, %r12, 1), %ymm1
	vmovapd		96(%r11, %r12, 2), %ymm2
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm1
	vmovapd		%ymm0, 96(%r13)
	vmovapd		%ymm1, 96(%r13, %r14)

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovapd		0(%r11), %ymm0
	vmovapd		0(%r11, %r12, 1), %ymm1
	vmovapd		0(%r11, %r12, 2), %ymm2
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm1
	vmovapd		%ymm0, 0(%r13)
	vmovapd		%ymm1, 0(%r13, %r14)

	subl	$ 1, %r10d
	addq	$ 32, %r11
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpacp_nn_8_2_lib4)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACP_NN_8_2_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpacp_nn_8_2_vs_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		0f // return

	vcvtsi2sd	%r15d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC01(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC01(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm15

	cmpl	$ 3, %r10d
	jle		2f // clean-up loop

	// main loop
	.p2align 3
1:
	//
	vmovapd		0(%r11), %ymm0
	vmovapd		0(%r11, %r12, 1), %ymm1
	vmovapd		0(%r11, %r12, 2), %ymm2
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm1
	vmovapd		%ymm0, 0(%r13)
	vmaskmovpd	%ymm1, %ymm15, 0(%r13, %r14)
	//
	vmovapd		32(%r11), %ymm0
	vmovapd		32(%r11, %r12, 1), %ymm1
	vmovapd		32(%r11, %r12, 2), %ymm2
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm1
	vmovapd		%ymm0, 32(%r13)
	vmaskmovpd	%ymm1, %ymm15, 32(%r13, %r14)
	//
	vmovapd		64(%r11), %ymm0
	vmovapd		64(%r11, %r12, 1), %ymm1
	vmovapd		64(%r11, %r12, 2), %ymm2
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm1
	vmovapd		%ymm0, 64(%r13)
	vmaskmovpd	%ymm1, %ymm15, 64(%r13, %r14)
	//
	vmovapd		96(%r11), %ymm0
	vmovapd		96(%r11, %r12, 1), %ymm1
	vmovapd		96(%r11, %r12, 2), %ymm2
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm1
	vmovapd		%ymm0, 96(%r13)
	vmaskmovpd	%ymm1, %ymm15, 96(%r13, %r14)

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovapd		0(%r11), %ymm0
	vmovapd		0(%r11, %r12, 1), %ymm1
	vmovapd		0(%r11, %r12, 2), %ymm2
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm1
	vmovapd		%ymm0, 0(%r13)
	vmaskmovpd	%ymm1, %ymm15, 0(%r13, %r14)

	subl	$ 1, %r10d
	addq	$ 32, %r11
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpacp_nn_8_2_vs_lib4)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACP_NN_8_3_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpacp_nn_8_3_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		0f // return

	cmpl	$ 3, %r10d
	jle		2f // clean-up loop

	// main loop
	.p2align 3
1:
	//
	vmovapd		0(%r11), %ymm0
	vmovapd		0(%r11, %r12, 1), %ymm1
	vmovapd		0(%r11, %r12, 2), %ymm2
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm3
	vshufpd		$ 0x5, %ymm1, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm2, %ymm3, %ymm1
	vmovapd		%ymm0, 0(%r13)
	vmovapd		%ymm1, 0(%r13, %r14, 1)
	//
	vmovapd		32(%r11), %ymm0
	vmovapd		32(%r11, %r12, 1), %ymm1
	vmovapd		32(%r11, %r12, 2), %ymm2
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm3
	vshufpd		$ 0x5, %ymm1, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm2, %ymm3, %ymm1
	vmovapd		%ymm0, 32(%r13)
	vmovapd		%ymm1, 32(%r13, %r14, 1)
	//
	vmovapd		64(%r11), %ymm0
	vmovapd		64(%r11, %r12, 1), %ymm1
	vmovapd		64(%r11, %r12, 2), %ymm2
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm3
	vshufpd		$ 0x5, %ymm1, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm2, %ymm3, %ymm1
	vmovapd		%ymm0, 64(%r13)
	vmovapd		%ymm1, 64(%r13, %r14, 1)
	//
	vmovapd		96(%r11), %ymm0
	vmovapd		96(%r11, %r12, 1), %ymm1
	vmovapd		96(%r11, %r12, 2), %ymm2
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm3
	vshufpd		$ 0x5, %ymm1, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm2, %ymm3, %ymm1
	vmovapd		%ymm0, 96(%r13)
	vmovapd		%ymm1, 96(%r13, %r14, 1)

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovapd		0(%r11), %ymm0
	vmovapd		0(%r11, %r12, 1), %ymm1
	vmovapd		0(%r11, %r12, 2), %ymm2
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm3
	vshufpd		$ 0x5, %ymm1, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm2, %ymm3, %ymm1
	vmovapd		%ymm0, 0(%r13)
	vmovapd		%ymm1, 0(%r13, %r14, 1)

	subl	$ 1, %r10d
	addq	$ 32, %r11
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpacp_nn_8_3_lib4)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACP_NN_8_3_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpacp_nn_8_3_vs_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		0f // return

	vcvtsi2sd	%r15d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC01(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC01(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm15

	cmpl	$ 3, %r10d
	jle		2f // clean-up loop

	// main loop
	.p2align 3
1:
	//
	vmovapd		0(%r11), %ymm0
	vmovapd		0(%r11, %r12, 1), %ymm1
	vmovapd		0(%r11, %r12, 2), %ymm2
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm3
	vshufpd		$ 0x5, %ymm1, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm2, %ymm3, %ymm1
	vmovapd		%ymm0, 0(%r13)
	vmaskmovpd	%ymm1, %ymm15, 0(%r13, %r14, 1)
	//
	vmovapd		32(%r11), %ymm0
	vmovapd		32(%r11, %r12, 1), %ymm1
	vmovapd		32(%r11, %r12, 2), %ymm2
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm3
	vshufpd		$ 0x5, %ymm1, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm2, %ymm3, %ymm1
	vmovapd		%ymm0, 32(%r13)
	vmaskmovpd	%ymm1, %ymm15, 32(%r13, %r14, 1)
	//
	vmovapd		64(%r11), %ymm0
	vmovapd		64(%r11, %r12, 1), %ymm1
	vmovapd		64(%r11, %r12, 2), %ymm2
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm3
	vshufpd		$ 0x5, %ymm1, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm2, %ymm3, %ymm1
	vmovapd		%ymm0, 64(%r13)
	vmaskmovpd	%ymm1, %ymm15, 64(%r13, %r14, 1)
	//
	vmovapd		96(%r11), %ymm0
	vmovapd		96(%r11, %r12, 1), %ymm1
	vmovapd		96(%r11, %r12, 2), %ymm2
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm3
	vshufpd		$ 0x5, %ymm1, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm2, %ymm3, %ymm1
	vmovapd		%ymm0, 96(%r13)
	vmaskmovpd	%ymm1, %ymm15, 96(%r13, %r14, 1)

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovapd		0(%r11), %ymm0
	vmovapd		0(%r11, %r12, 1), %ymm1
	vmovapd		0(%r11, %r12, 2), %ymm2
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vperm2f128	$ 0x21, %ymm2, %ymm1, %ymm3
	vshufpd		$ 0x5, %ymm1, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm2, %ymm3, %ymm1
	vmovapd		%ymm0, 0(%r13)
	vmaskmovpd	%ymm1, %ymm15, 0(%r13, %r14, 1)

	subl	$ 1, %r10d
	addq	$ 32, %r11
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpacp_nn_8_3_vs_lib4)
#endif





// input arguments:
// r10  <- kmax
// r11  <- A
// r12  <- ...*sda
// r13  <- C

#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACP_NN_4_0_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpacp_nn_4_0_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		0f // return

	cmpl	$ 3, %r10d
	jle		2f // clean-up loop

	// main loop
	.p2align 3
1:
	//
	vmovapd		0(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13)
	//
	vmovapd		32(%r11), %ymm0
	vmovapd		%ymm0, 32(%r13)
	//
	vmovapd		64(%r11), %ymm0
	vmovapd		%ymm0, 64(%r13)
	//
	vmovapd		96(%r11), %ymm0
	vmovapd		%ymm0, 96(%r13)

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovapd		0(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13)

	subl	$ 1, %r10d
	addq	$ 32, %r11
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpacp_nn_4_0_lib4)
#endif





// input arguments:
// r10  <- kmax
// r11  <- A
// r12  <- ...*sda
// r13  <- C

#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACP_NN_4_1_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpacp_nn_4_1_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		0f // return

	cmpl	$ 3, %r10d
	jle		2f // clean-up loop

	// main loop
	.p2align 3
1:
	//
	vmovapd		0(%r11), %ymm0
	vmovapd		0(%r11, %r12, 1), %ymm1
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm1
	vshufpd		$ 0x5, %ymm1, %ymm0, %ymm0
	vmovapd		%ymm0, 0(%r13)
	//
	vmovapd		32(%r11), %ymm0
	vmovapd		32(%r11, %r12, 1), %ymm1
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm1
	vshufpd		$ 0x5, %ymm1, %ymm0, %ymm0
	vmovapd		%ymm0, 32(%r13)
	//
	vmovapd		64(%r11), %ymm0
	vmovapd		64(%r11, %r12, 1), %ymm1
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm1
	vshufpd		$ 0x5, %ymm1, %ymm0, %ymm0
	vmovapd		%ymm0, 64(%r13)
	//
	vmovapd		96(%r11), %ymm0
	vmovapd		96(%r11, %r12, 1), %ymm1
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm1
	vshufpd		$ 0x5, %ymm1, %ymm0, %ymm0
	vmovapd		%ymm0, 96(%r13)

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovapd		0(%r11), %ymm0
	vmovapd		0(%r11, %r12, 1), %ymm1
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm1
	vshufpd		$ 0x5, %ymm1, %ymm0, %ymm0
	vmovapd		%ymm0, 0(%r13)

	subl	$ 1, %r10d
	addq	$ 32, %r11
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpacp_nn_4_1_lib4)
#endif





// input arguments:
// r10  <- kmax
// r11  <- A
// r12  <- ...*sda
// r13  <- C

#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACP_NN_4_2_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpacp_nn_4_2_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		0f // return

	cmpl	$ 3, %r10d
	jle		2f // clean-up loop

	// main loop
	.p2align 3
1:
	//
	vmovapd		0(%r11), %ymm0
	vmovapd		0(%r11, %r12, 1), %ymm1
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vmovapd		%ymm0, 0(%r13)
	//
	vmovapd		32(%r11), %ymm0
	vmovapd		32(%r11, %r12, 1), %ymm1
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vmovapd		%ymm0, 32(%r13)
	//
	vmovapd		64(%r11), %ymm0
	vmovapd		64(%r11, %r12, 1), %ymm1
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vmovapd		%ymm0, 64(%r13)
	//
	vmovapd		96(%r11), %ymm0
	vmovapd		96(%r11, %r12, 1), %ymm1
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vmovapd		%ymm0, 96(%r13)

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovapd		0(%r11), %ymm0
	vmovapd		0(%r11, %r12, 1), %ymm1
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vmovapd		%ymm0, 0(%r13)

	subl	$ 1, %r10d
	addq	$ 32, %r11
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpacp_nn_4_2_lib4)
#endif





// input arguments:
// r10  <- kmax
// r11  <- A
// r12  <- ...*sda
// r13  <- C

#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACP_NN_4_3_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpacp_nn_4_3_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		0f // return

	cmpl	$ 3, %r10d
	jle		2f // clean-up loop

	// main loop
	.p2align 3
1:
	//
	vmovapd		0(%r11), %ymm0
	vmovapd		0(%r11, %r12, 1), %ymm1
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm1, %ymm0, %ymm0
	vmovapd		%ymm0, 0(%r13)
	//
	vmovapd		32(%r11), %ymm0
	vmovapd		32(%r11, %r12, 1), %ymm1
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm1, %ymm0, %ymm0
	vmovapd		%ymm0, 32(%r13)
	//
	vmovapd		64(%r11), %ymm0
	vmovapd		64(%r11, %r12, 1), %ymm1
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm1, %ymm0, %ymm0
	vmovapd		%ymm0, 64(%r13)
	//
	vmovapd		96(%r11), %ymm0
	vmovapd		96(%r11, %r12, 1), %ymm1
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm1, %ymm0, %ymm0
	vmovapd		%ymm0, 96(%r13)

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovapd		0(%r11), %ymm0
	vmovapd		0(%r11, %r12, 1), %ymm1
	vperm2f128	$ 0x21, %ymm1, %ymm0, %ymm0
	vshufpd		$ 0x5, %ymm1, %ymm0, %ymm0
	vmovapd		%ymm0, 0(%r13)

	subl	$ 1, %r10d
	addq	$ 32, %r11
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpacp_nn_4_3_lib4)
#endif





// input arguments:
// r10  <- kmax
// r11  <- A
// r12  <- ...*sda
// r13  <- C

#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACP_NN_3_0_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpacp_nn_3_0_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		0f // return

	cmpl	$ 3, %r10d
	jle		2f // clean-up loop

	// main loop
	.p2align 3
1:
	//
	vmovupd		0(%r11), %xmm0
	vmovsd		16(%r11), %xmm1
	vmovupd		%xmm0, 0(%r13)
	vmovsd		%xmm1, 16(%r13)
	//
	vmovupd		32(%r11), %xmm0
	vmovsd		48(%r11), %xmm1
	vmovupd		%xmm0, 32(%r13)
	vmovsd		%xmm1, 48(%r13)
	//
	vmovupd		64(%r11), %xmm0
	vmovsd		80(%r11), %xmm1
	vmovupd		%xmm0, 64(%r13)
	vmovsd		%xmm1, 80(%r13)
	//
	vmovupd		96(%r11), %xmm0
	vmovsd		112(%r11), %xmm1
	vmovupd		%xmm0, 96(%r13)
	vmovsd		%xmm1, 112(%r13)

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovupd		0(%r11), %xmm0
	vmovsd		16(%r11), %xmm1
	vmovupd		%xmm0, 0(%r13)
	vmovsd		%xmm1, 16(%r13)

	subl	$ 1, %r10d
	addq	$ 32, %r11
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpacp_nn_3_0_lib4)
#endif





// input arguments:
// r10  <- kmax
// r11  <- A
// r12  <- ...*sda
// r13  <- C

#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACP_NN_3_2_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpacp_nn_3_2_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		0f // return

	cmpl	$ 3, %r10d
	jle		2f // clean-up loop

	// main loop
	.p2align 3
1:
	//
	vmovupd		16(%r11), %xmm0
	vmovsd		0(%r11, %r12), %xmm1
	vmovupd		%xmm0, 0(%r13)
	vmovsd		%xmm1, 16(%r13)
	//
	vmovupd		48(%r11), %xmm0
	vmovsd		32(%r11, %r12), %xmm1
	vmovupd		%xmm0, 32(%r13)
	vmovsd		%xmm1, 48(%r13)
	//
	vmovupd		80(%r11), %xmm0
	vmovsd		64(%r11, %r12), %xmm1
	vmovupd		%xmm0, 64(%r13)
	vmovsd		%xmm1, 80(%r13)
	//
	vmovupd		112(%r11), %xmm0
	vmovsd		96(%r11, %r12), %xmm1
	vmovupd		%xmm0, 96(%r13)
	vmovsd		%xmm1, 112(%r13)

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovupd		16(%r11), %xmm0
	vmovsd		0(%r11, %r12), %xmm1
	vmovupd		%xmm0, 0(%r13)
	vmovsd		%xmm1, 16(%r13)

	subl	$ 1, %r10d
	addq	$ 32, %r11
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpacp_nn_3_2_lib4)
#endif





// input arguments:
// r10  <- kmax
// r11  <- A
// r12  <- ...*sda
// r13  <- C

#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACP_NN_3_3_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpacp_nn_3_3_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		0f // return

	cmpl	$ 3, %r10d
	jle		2f // clean-up loop

	// main loop
	.p2align 3
1:
	//
	vmovsd		24(%r11), %xmm0
	vmovapd		0(%r11, %r12), %xmm1
	vmovsd		%xmm0, 0(%r13)
	vmovupd		%xmm1, 8(%r13)
	//
	vmovsd		56(%r11), %xmm0
	vmovapd		32(%r11, %r12), %xmm1
	vmovsd		%xmm0, 32(%r13)
	vmovupd		%xmm1, 40(%r13)
	//
	vmovsd		88(%r11), %xmm0
	vmovapd		64(%r11, %r12), %xmm1
	vmovsd		%xmm0, 64(%r13)
	vmovupd		%xmm1, 72(%r13)
	//
	vmovsd		120(%r11), %xmm0
	vmovapd		96(%r11, %r12), %xmm1
	vmovsd		%xmm0, 96(%r13)
	vmovupd		%xmm1, 104(%r13)

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovsd		24(%r11), %xmm0
	vmovapd		0(%r11, %r12), %xmm1
	vmovsd		%xmm0, 0(%r13)
	vmovupd		%xmm1, 8(%r13)

	subl	$ 1, %r10d
	addq	$ 32, %r11
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpacp_nn_3_3_lib4)
#endif





// input arguments:
// r10  <- kmax
// r11  <- A
// r12  <- ...*sda
// r13  <- C

#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACP_NN_2_0_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpacp_nn_2_0_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		0f // return

	cmpl	$ 3, %r10d
	jle		2f // clean-up loop

	// main loop
	.p2align 3
1:
	//
	vmovupd		0(%r11), %xmm0
	vmovupd		%xmm0, 0(%r13)
	//
	vmovupd		32(%r11), %xmm0
	vmovupd		%xmm0, 32(%r13)
	//
	vmovupd		64(%r11), %xmm0
	vmovupd		%xmm0, 64(%r13)
	//
	vmovupd		96(%r11), %xmm0
	vmovupd		%xmm0, 96(%r13)

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovupd		0(%r11), %xmm0
	vmovupd		%xmm0, 0(%r13)

	subl	$ 1, %r10d
	addq	$ 32, %r11
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpacp_nn_2_0_lib4)
#endif





// input arguments:
// r10  <- kmax
// r11  <- A
// r12  <- ...*sda
// r13  <- C

#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACP_NN_2_1_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpacp_nn_2_1_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		0f // return

	cmpl	$ 3, %r10d
	jle		2f // clean-up loop

	// main loop
	.p2align 3
1:
	//
	vmovsd		24(%r11), %xmm0
	vmovhpd		0(%r11, %r12), %xmm0, %xmm0
	vmovapd		%xmm0, 0(%r13)
	//
	vmovsd		56(%r11), %xmm0
	vmovhpd		32(%r11, %r12), %xmm0, %xmm0
	vmovapd		%xmm0, 32(%r13)
	//
	vmovsd		88(%r11), %xmm0
	vmovhpd		64(%r11, %r12), %xmm0, %xmm0
	vmovapd		%xmm0, 64(%r13)
	//
	vmovsd		120(%r11), %xmm0
	vmovhpd		96(%r11, %r12), %xmm0, %xmm0
	vmovapd		%xmm0, 96(%r13)

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovsd		24(%r11), %xmm0
	vmovhpd		0(%r11, %r12), %xmm0, %xmm0
	vmovapd		%xmm0, 0(%r13)

	subl	$ 1, %r10d
	addq	$ 32, %r11
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpacp_nn_2_1_lib4)
#endif





// input arguments:
// r10  <- kmax
// r11  <- A
// r12  <- ...*sda
// r13  <- C

#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DPACP_NN_1_0_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dpacp_nn_1_0_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		0f // return

	cmpl	$ 3, %r10d
	jle		2f // clean-up loop

	// main loop
	.p2align 3
1:
	//
	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13)
	//
	vmovsd		32(%r11), %xmm0
	vmovsd		%xmm0, 32(%r13)
	//
	vmovsd		64(%r11), %xmm0
	vmovsd		%xmm0, 64(%r13)
	//
	vmovsd		96(%r11), %xmm0
	vmovsd		%xmm0, 96(%r13)

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13)

	subl	$ 1, %r10d
	addq	$ 32, %r11
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dpacp_nn_1_0_lib4)
#endif





//                              1         2          3        4          5
// void kernel_dpack_nn_12_lib4(int kmax, double *A, int lda, double *C, int sdc)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpack_nn_12_lib4)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$ 3, %r12d
	movq	ARG4, %r13 // C
	movq	ARG5, %r14 // sdc
	sall	$ 5, %r14d


	cmpl	$ 0, %r10d
	jle		0f // return


	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovupd		0(%r11), %ymm0
	prefetcht0	0(%r11, %r12, 4)
	vmovapd		%ymm0, 0(%r13)
	vmovupd		32(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13, %r14)
	vmovupd		64(%r11), %ymm0
	prefetcht0	64(%r11, %r12, 4)
	vmovapd		%ymm0, 0(%r13, %r14, 2)
	addq	%r12, %r11

	vmovupd		0(%r11), %ymm0
	prefetcht0	0(%r11, %r12, 4)
	vmovapd		%ymm0, 32(%r13)
	vmovupd		32(%r11), %ymm0
	vmovapd		%ymm0, 32(%r13, %r14)
	vmovupd		64(%r11), %ymm0
	prefetcht0	64(%r11, %r12, 4)
	vmovapd		%ymm0, 32(%r13, %r14, 2)
	addq	%r12, %r11

	vmovupd		0(%r11), %ymm0
	prefetcht0	0(%r11, %r12, 4)
	vmovapd		%ymm0, 64(%r13)
	vmovupd		32(%r11), %ymm0
	vmovapd		%ymm0, 64(%r13, %r14)
	vmovupd		64(%r11), %ymm0
	prefetcht0	64(%r11, %r12, 4)
	vmovapd		%ymm0, 64(%r13, %r14, 2)
	addq	%r12, %r11

	vmovupd		0(%r11), %ymm0
	prefetcht0	0(%r11, %r12, 4)
	vmovapd		%ymm0, 96(%r13)
	vmovupd		32(%r11), %ymm0
	vmovapd		%ymm0, 96(%r13, %r14)
	vmovupd		64(%r11), %ymm0
	prefetcht0	64(%r11, %r12, 4)
	vmovapd		%ymm0, 96(%r13, %r14, 2)
	addq	%r12, %r11

	subl	$ 4, %r10d
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13)
	vmovupd		32(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13, %r14)
	vmovupd		64(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13, %r14, 2)
	addq	%r12, %r11

	subl	$ 1, %r10d
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

	EPILOGUE

	ret

	FUN_END(kernel_dpack_nn_12_lib4)





//                                 1         2          3        4          5        6
// void kernel_dpack_nn_12_vs_lib4(int kmax, double *A, int lda, double *C, int sdc, int m1)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpack_nn_12_vs_lib4)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$ 3, %r12d
	movq	ARG4, %r13 // C
	movq	ARG5, %r14 // sdc
	sall	$ 5, %r14d
	movq	ARG6, %r15 // m1


	cmpl	$ 0, %r10d
	jle		0f // return


	vcvtsi2sd	%r15d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC02(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC02(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm15


	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13)
	vmovupd		32(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13, %r14)
	vmaskmovpd	64(%r11), %ymm15, %ymm0
	vmovapd		%ymm0, 0(%r13, %r14, 2)
	addq	%r12, %r11

	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 32(%r13)
	vmovupd		32(%r11), %ymm0
	vmovapd		%ymm0, 32(%r13, %r14)
	vmaskmovpd	64(%r11), %ymm15, %ymm0
	vmovapd		%ymm0, 32(%r13, %r14, 2)
	addq	%r12, %r11

	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 64(%r13)
	vmovupd		32(%r11), %ymm0
	vmovapd		%ymm0, 64(%r13, %r14)
	vmaskmovpd	64(%r11), %ymm15, %ymm0
	vmovapd		%ymm0, 64(%r13, %r14, 2)
	addq	%r12, %r11

	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 96(%r13)
	vmovupd		32(%r11), %ymm0
	vmovapd		%ymm0, 96(%r13, %r14)
	vmaskmovpd	64(%r11), %ymm15, %ymm0
	vmovapd		%ymm0, 96(%r13, %r14, 2)
	addq	%r12, %r11

	subl	$ 4, %r10d
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13)
	vmovupd		32(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13, %r14)
	vmaskmovpd	64(%r11), %ymm15, %ymm0
	vmovapd		%ymm0, 0(%r13, %r14, 2)
	addq	%r12, %r11

	subl	$ 1, %r10d
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

	EPILOGUE

	ret

	FUN_END(kernel_dpack_nn_12_vs_lib4)





//                             1         2          3        4          5
// void kernel_dpack_nn_8_lib4(int kmax, double *A, int lda, double *C, int sdc)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpack_nn_8_lib4)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$ 3, %r12d
	movq	ARG4, %r13 // C
	movq	ARG5, %r14 // sdc
	sall	$ 5, %r14d


	cmpl	$ 0, %r10d
	jle		0f // return


	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovupd		0(%r11), %ymm0
	prefetcht0	0(%r11, %r12, 4)
	vmovapd		%ymm0, 0(%r13)
	vmovupd		32(%r11), %ymm0
	prefetcht0	32(%r11, %r12, 4)
	vmovapd		%ymm0, 0(%r13, %r14)
	addq	%r12, %r11

	vmovupd		0(%r11), %ymm0
	prefetcht0	0(%r11, %r12, 4)
	vmovapd		%ymm0, 32(%r13)
	vmovupd		32(%r11), %ymm0
	prefetcht0	32(%r11, %r12, 4)
	vmovapd		%ymm0, 32(%r13, %r14)
	addq	%r12, %r11

	vmovupd		0(%r11), %ymm0
	prefetcht0	0(%r11, %r12, 4)
	vmovapd		%ymm0, 64(%r13)
	vmovupd		32(%r11), %ymm0
	prefetcht0	32(%r11, %r12, 4)
	vmovapd		%ymm0, 64(%r13, %r14)
	addq	%r12, %r11

	vmovupd		0(%r11), %ymm0
	prefetcht0	0(%r11, %r12, 4)
	vmovapd		%ymm0, 96(%r13)
	vmovupd		32(%r11), %ymm0
	prefetcht0	32(%r11, %r12, 4)
	vmovapd		%ymm0, 96(%r13, %r14)
	addq	%r12, %r11

	subl	$ 4, %r10d
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13)
	vmovupd		32(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13, %r14)
	addq	%r12, %r11

	subl	$ 1, %r10d
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

	EPILOGUE

	ret

	FUN_END(kernel_dpack_nn_8_lib4)





//                                1         2          3        4          5        6
// void kernel_dpack_nn_8_vs_lib4(int kmax, double *A, int lda, double *C, int sdc, int m1)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpack_nn_8_vs_lib4)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$ 3, %r12d
	movq	ARG4, %r13 // C
	movq	ARG5, %r14 // sdc
	sall	$ 5, %r14d
	movq	ARG6, %r15 // m1


	cmpl	$ 0, %r10d
	jle		0f // return


	vcvtsi2sd	%r15d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC01(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC01(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm15


	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13)
	vmaskmovpd	32(%r11), %ymm15, %ymm0
	vmovapd		%ymm0, 0(%r13, %r14)
	addq	%r12, %r11

	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 32(%r13)
	vmaskmovpd	32(%r11), %ymm15, %ymm0
	vmovapd		%ymm0, 32(%r13, %r14)
	addq	%r12, %r11

	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 64(%r13)
	vmaskmovpd	32(%r11), %ymm15, %ymm0
	vmovapd		%ymm0, 64(%r13, %r14)
	addq	%r12, %r11

	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 96(%r13)
	vmaskmovpd	32(%r11), %ymm15, %ymm0
	vmovapd		%ymm0, 96(%r13, %r14)
	addq	%r12, %r11

	subl	$ 4, %r10d
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13)
	vmaskmovpd	32(%r11), %ymm15, %ymm0
	vmovapd		%ymm0, 0(%r13, %r14)
	addq	%r12, %r11

	subl	$ 1, %r10d
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

	EPILOGUE

	ret

	FUN_END(kernel_dpack_nn_8_vs_lib4)





//                             1         2          3        4
// void kernel_dpack_nn_4_lib4(int kmax, double *A, int lda, double *C)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpack_nn_4_lib4)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$ 3, %r12d
	movq	ARG4, %r13 // C


	cmpl	$ 0, %r10d
	jle		0f // return


	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovupd		0(%r11), %ymm0
	prefetcht0	0(%r11, %r12, 4)
	vmovapd		%ymm0, 0(%r13)
	addq	%r12, %r11

	vmovupd		0(%r11), %ymm0
	prefetcht0	0(%r11, %r12, 4)
	vmovapd		%ymm0, 32(%r13)
	addq	%r12, %r11

	vmovupd		0(%r11), %ymm0
	prefetcht0	0(%r11, %r12, 4)
	vmovapd		%ymm0, 64(%r13)
	addq	%r12, %r11

	vmovupd		0(%r11), %ymm0
	prefetcht0	0(%r11, %r12, 4)
	vmovapd		%ymm0, 96(%r13)
	addq	%r12, %r11

	subl	$ 4, %r10d
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13)
	addq	%r12, %r11

	subl	$ 1, %r10d
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

	EPILOGUE

	ret

	FUN_END(kernel_dpack_nn_4_lib4)





//                                1         2          3        4          5
// void kernel_dpack_nn_4_vs_lib4(int kmax, double *A, int lda, double *C, int m1)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpack_nn_4_vs_lib4)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$ 3, %r12d
	movq	ARG4, %r13 // C
	movq	ARG5, %r14 // m1


	cmpl	$ 0, %r10d
	jle		0f // return


	vcvtsi2sd	%r14d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC00(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC00(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm15


	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmaskmovpd	0(%r11), %ymm15, %ymm0
	vmovapd		%ymm0, 0(%r13)
	addq	%r12, %r11

	vmaskmovpd	0(%r11), %ymm15, %ymm0
	vmovapd		%ymm0, 32(%r13)
	addq	%r12, %r11

	vmaskmovpd	0(%r11), %ymm15, %ymm0
	vmovapd		%ymm0, 64(%r13)
	addq	%r12, %r11

	vmaskmovpd	0(%r11), %ymm15, %ymm0
	vmovapd		%ymm0, 96(%r13)
	addq	%r12, %r11

	subl	$ 4, %r10d
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmaskmovpd	0(%r11), %ymm15, %ymm0
	vmovapd		%ymm0, 0(%r13)
	addq	%r12, %r11

	subl	$ 1, %r10d
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

	EPILOGUE

	ret

	FUN_END(kernel_dpack_nn_4_vs_lib4)





//                             1         2          3        4
// void kernel_dpack_tn_4_lib4(int kmax, double *A, int lda, double *C)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpack_tn_4_lib4)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$ 3, %r12d
	movq	ARG4, %r13 // C


#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACK_TN_4_LIB4
#else
	CALL(inner_kernel_dpack_tn_4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dpack_tn_4_lib4)





//                             1         2          3        4
// void kernel_dpack_tn_4_lib4(int kmax, double *A, int lda, double *C)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpack_tn_4_p0_lib4)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$ 3, %r12d
	movq	ARG4, %r13 // C


#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACK_TN_4_P0_LIB4
#else
	CALL(inner_kernel_dpack_tn_4_p0_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dpack_tn_4_p0_lib4)





//                                1         2          3        4          5
// void kernel_dpack_tn_4_vs_lib4(int kmax, double *A, int lda, double *C, int m1)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpack_tn_4_vs_lib4)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$ 3, %r12d
	movq	ARG4, %r13 // C

	movq	ARG5, %r14 // m1
	cmpl	$ 1, %r14d
	jg		100f

#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACK_TN_1_LIB4
#else
	CALL(inner_kernel_dpack_tn_1_lib4)
#endif

	jmp		103f

100:

	movq	ARG5, %r14 // m1
	cmpl	$ 2, %r14d
	jg		101f

#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACK_TN_2_LIB4
#else
	CALL(inner_kernel_dpack_tn_2_lib4)
#endif
	
	jmp		103f

101:

	movq	ARG5, %r14 // m1
	cmpl	$ 3, %r14d
	jg		102f

#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACK_TN_3_LIB4
#else
	CALL(inner_kernel_dpack_tn_3_lib4)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACK_TN_4_LIB4
#else
	CALL(inner_kernel_dpack_tn_4_lib4)
#endif

103:


	EPILOGUE

	ret

	FUN_END(kernel_dpack_tn_4_vs_lib4)





//                             1         2          3        4          5
// void kernel_dpack_tt_12_lib4(int kmax, double *A, int lda, double *C, int sdc)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpack_tt_12_lib4)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$ 3, %r12d
	movq	ARG4, %r13 // C
	movq	ARG5, %r14 // sdc
	sall	$ 5, %r14d


	cmpl	$ 0, %r10d
	jle		0f // return


	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	movq	%r11, %r15

	vmovupd		0(%r15), %ymm0
	vmovapd		%ymm0, 0(%r13)
	addq	%r12, %r15

	vmovupd		0(%r15), %ymm0
	vmovapd		%ymm0, 32(%r13)
	addq	%r12, %r15

	vmovupd		0(%r15), %ymm0
	vmovapd		%ymm0, 64(%r13)
	addq	%r12, %r15

	vmovupd		0(%r15), %ymm0
	vmovapd		%ymm0, 96(%r13)
	addq	%r12, %r15

	vmovupd		0(%r15), %ymm0
	vmovapd		%ymm0, 128(%r13)
	addq	%r12, %r15

	vmovupd		0(%r15), %ymm0
	vmovapd		%ymm0, 160(%r13)
	addq	%r12, %r15

	vmovupd		0(%r15), %ymm0
	vmovapd		%ymm0, 192(%r13)
	addq	%r12, %r15

	vmovupd		0(%r15), %ymm0
	vmovapd		%ymm0, 224(%r13)
	addq	%r12, %r15

	vmovupd		0(%r15), %ymm0
	vmovapd		%ymm0, 256(%r13)
	addq	%r12, %r15

	vmovupd		0(%r15), %ymm0
	vmovapd		%ymm0, 288(%r13)
	addq	%r12, %r15

	vmovupd		0(%r15), %ymm0
	vmovapd		%ymm0, 320(%r13)
	addq	%r12, %r15

	vmovupd		0(%r15), %ymm0
	vmovapd		%ymm0, 352(%r13)
	addq	%r12, %r15

	subl	$ 4, %r10d
	addq	$ 32, %r11
	addq	%r14, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	movq	%r11, %r15

	vmovsd		0(%r15), %xmm0
	vmovsd		%xmm0, 0(%r13)
	addq	%r12, %r15

	vmovsd		0(%r15), %xmm0
	vmovsd		%xmm0, 32(%r13)
	addq	%r12, %r15

	vmovsd		0(%r15), %xmm0
	vmovsd		%xmm0, 64(%r13)
	addq	%r12, %r15

	vmovsd		0(%r15), %xmm0
	vmovsd		%xmm0, 96(%r13)
	addq	%r12, %r15

	vmovsd		0(%r15), %xmm0
	vmovsd		%xmm0, 128(%r13)
	addq	%r12, %r15

	vmovsd		0(%r15), %xmm0
	vmovsd		%xmm0, 160(%r13)
	addq	%r12, %r15

	vmovsd		0(%r15), %xmm0
	vmovsd		%xmm0, 192(%r13)
	addq	%r12, %r15

	vmovsd		0(%r15), %xmm0
	vmovsd		%xmm0, 224(%r13)
	addq	%r12, %r15

	vmovsd		0(%r15), %xmm0
	vmovsd		%xmm0, 256(%r13)
	addq	%r12, %r15

	vmovsd		0(%r15), %xmm0
	vmovsd		%xmm0, 288(%r13)
	addq	%r12, %r15

	vmovsd		0(%r15), %xmm0
	vmovsd		%xmm0, 320(%r13)
	addq	%r12, %r15

	vmovsd		0(%r15), %xmm0
	vmovsd		%xmm0, 352(%r13)
	addq	%r12, %r15

	subl	$ 1, %r10d
	addq	$ 8, %r11
	addq	$ 8, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

	EPILOGUE

	ret

	FUN_END(kernel_dpack_tt_12_lib4)





//                             1         2          3        4          5
// void kernel_dpack_tt_8_lib4(int kmax, double *A, int lda, double *C, int sdc)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpack_tt_8_lib4)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$ 3, %r12d
	movq	ARG4, %r13 // C
	movq	ARG5, %r14 // sdc
	sall	$ 5, %r14d


	cmpl	$ 0, %r10d
	jle		0f // return


	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	movq	%r11, %r15

	vmovupd		0(%r15), %ymm0
	vmovapd		%ymm0, 0(%r13)
	addq	%r12, %r15

	vmovupd		0(%r15), %ymm0
	vmovapd		%ymm0, 32(%r13)
	addq	%r12, %r15

	vmovupd		0(%r15), %ymm0
	vmovapd		%ymm0, 64(%r13)
	addq	%r12, %r15

	vmovupd		0(%r15), %ymm0
	vmovapd		%ymm0, 96(%r13)
	addq	%r12, %r15

	vmovupd		0(%r15), %ymm0
	vmovapd		%ymm0, 128(%r13)
	addq	%r12, %r15

	vmovupd		0(%r15), %ymm0
	vmovapd		%ymm0, 160(%r13)
	addq	%r12, %r15

	vmovupd		0(%r15), %ymm0
	vmovapd		%ymm0, 192(%r13)
	addq	%r12, %r15

	vmovupd		0(%r15), %ymm0
	vmovapd		%ymm0, 224(%r13)
	addq	%r12, %r15

	subl	$ 4, %r10d
	addq	$ 32, %r11
	addq	%r14, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	movq	%r11, %r15

	vmovsd		0(%r15), %xmm0
	vmovsd		%xmm0, 0(%r13)
	addq	%r12, %r15

	vmovsd		0(%r15), %xmm0
	vmovsd		%xmm0, 32(%r13)
	addq	%r12, %r15

	vmovsd		0(%r15), %xmm0
	vmovsd		%xmm0, 64(%r13)
	addq	%r12, %r15

	vmovsd		0(%r15), %xmm0
	vmovsd		%xmm0, 96(%r13)
	addq	%r12, %r15

	vmovsd		0(%r15), %xmm0
	vmovsd		%xmm0, 128(%r13)
	addq	%r12, %r15

	vmovsd		0(%r15), %xmm0
	vmovsd		%xmm0, 160(%r13)
	addq	%r12, %r15

	vmovsd		0(%r15), %xmm0
	vmovsd		%xmm0, 192(%r13)
	addq	%r12, %r15

	vmovsd		0(%r15), %xmm0
	vmovsd		%xmm0, 224(%r13)
	addq	%r12, %r15

	subl	$ 1, %r10d
	addq	$ 8, %r11
	addq	$ 8, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

	EPILOGUE

	ret

	FUN_END(kernel_dpack_tt_8_lib4)





//                             1         2          3        4          5
// void kernel_dpack_tt_4_lib4(int kmax, double *A, int lda, double *C, int sdc)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpack_tt_4_lib4)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$ 3, %r12d
	movq	ARG4, %r13 // C
	movq	ARG5, %r14 // sdc
	sall	$ 5, %r14d


#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACK_TT_4_LIB4
#else
	CALL(inner_kernel_dpack_tt_4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dpack_tt_4_lib4)





//                                1         2          3        4          5        6
// void kernel_dpack_tt_4_vs_lib4(int kmax, double *A, int lda, double *C, int sdc, int m1)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpack_tt_4_vs_lib4)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$ 3, %r12d
	movq	ARG4, %r13 // C
	movq	ARG5, %r14 // sdc
	sall	$ 5, %r14d

	movq	ARG6, %r15 // m1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACK_TT_1_LIB4
#else
	CALL(inner_kernel_dpack_tt_1_lib4)
#endif

	jmp		103f

100:

	movq	ARG6, %r15 // m1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACK_TT_2_LIB4
#else
	CALL(inner_kernel_dpack_tt_2_lib4)
#endif
	
	jmp		103f

101:

	movq	ARG6, %r15 // m1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACK_TT_3_LIB4
#else
	CALL(inner_kernel_dpack_tt_3_lib4)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACK_TT_4_LIB4
#else
	CALL(inner_kernel_dpack_tt_4_lib4)
#endif

103:


	EPILOGUE

	ret

	FUN_END(kernel_dpack_tt_4_vs_lib4)





//                                1         2          3        4          5
// void kernel_dunpack_nn_12_lib4(int kmax, double *A, int sda, double *C, int ldc)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dunpack_nn_12_lib4)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG4, %r13 // C
	movq	ARG5, %r14 // ldc
	sall	$ 3, %r14d


	cmpl	$ 0, %r10d
	jle		0f // return


	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovapd		0(%r11), %ymm0
	vmovupd		%ymm0, 0(%r13)
	vmovapd		0(%r11, %r12, 1), %ymm0
	vmovupd		%ymm0, 32(%r13)
	vmovapd		0(%r11, %r12, 2), %ymm0
	vmovupd		%ymm0, 64(%r13)
	addq	%r14, %r13

	vmovapd		32(%r11), %ymm0
	vmovupd		%ymm0, 0(%r13)
	vmovapd		32(%r11, %r12, 1), %ymm0
	vmovupd		%ymm0, 32(%r13)
	vmovapd		32(%r11, %r12, 2), %ymm0
	vmovupd		%ymm0, 64(%r13)
	addq	%r14, %r13

	vmovapd		64(%r11), %ymm0
	vmovupd		%ymm0, 0(%r13)
	vmovapd		64(%r11, %r12, 1), %ymm0
	vmovupd		%ymm0, 32(%r13)
	vmovapd		64(%r11, %r12, 2), %ymm0
	vmovupd		%ymm0, 64(%r13)
	addq	%r14, %r13

	vmovapd		96(%r11), %ymm0
	vmovupd		%ymm0, 0(%r13)
	vmovapd		96(%r11, %r12, 1), %ymm0
	vmovupd		%ymm0, 32(%r13)
	vmovapd		96(%r11, %r12, 2), %ymm0
	vmovupd		%ymm0, 64(%r13)
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 128, %r11

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovapd		0(%r11), %ymm0
	vmovupd		%ymm0, 0(%r13)
	vmovapd		0(%r11, %r12, 1), %ymm0
	vmovupd		%ymm0, 32(%r13)
	vmovapd		0(%r11, %r12, 2), %ymm0
	vmovupd		%ymm0, 64(%r13)
	addq	%r14, %r13

	subl	$ 1, %r10d
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		2b


0: // return

	EPILOGUE

	ret

	FUN_END(kernel_dunpack_nn_12_lib4)





//                                   1         2          3        4          5        6
// void kernel_dunpack_nn_12_vs_lib4(int kmax, double *A, int sda, double *C, int ldc, int m1)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dunpack_nn_12_vs_lib4)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG4, %r13 // C
	movq	ARG5, %r14 // ldc
	sall	$ 3, %r14d
	movq	ARG6, %r15 // m1


	cmpl	$ 0, %r10d
	jle		0f // return


	vcvtsi2sd	%r15d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC02(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC02(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm15


	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovapd		0(%r11), %ymm0
	vmovupd		%ymm0, 0(%r13)
	vmovapd		0(%r11, %r12, 1), %ymm0
	vmovupd		%ymm0, 32(%r13)
	vmovapd		0(%r11, %r12, 2), %ymm0
	vmaskmovpd	%ymm0, %ymm15, 64(%r13)
	addq	%r14, %r13

	vmovapd		32(%r11), %ymm0
	vmovupd		%ymm0, 0(%r13)
	vmovapd		32(%r11, %r12, 1), %ymm0
	vmovupd		%ymm0, 32(%r13)
	vmovapd		32(%r11, %r12, 2), %ymm0
	vmaskmovpd	%ymm0, %ymm15, 64(%r13)
	addq	%r14, %r13

	vmovapd		64(%r11), %ymm0
	vmovupd		%ymm0, 0(%r13)
	vmovapd		64(%r11, %r12, 1), %ymm0
	vmovupd		%ymm0, 32(%r13)
	vmovapd		64(%r11, %r12, 2), %ymm0
	vmaskmovpd	%ymm0, %ymm15, 64(%r13)
	addq	%r14, %r13

	vmovapd		96(%r11), %ymm0
	vmovupd		%ymm0, 0(%r13)
	vmovapd		96(%r11, %r12, 1), %ymm0
	vmovupd		%ymm0, 32(%r13)
	vmovapd		96(%r11, %r12, 2), %ymm0
	vmaskmovpd	%ymm0, %ymm15, 64(%r13)
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 128, %r11

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovapd		0(%r11), %ymm0
	vmovupd		%ymm0, 0(%r13)
	vmovapd		0(%r11, %r12, 1), %ymm0
	vmovupd		%ymm0, 32(%r13)
	vmovapd		0(%r11, %r12, 2), %ymm0
	vmaskmovpd	%ymm0, %ymm15, 64(%r13)
	addq	%r14, %r13

	subl	$ 1, %r10d
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		2b


0: // return

	EPILOGUE

	ret

	FUN_END(kernel_dunpack_nn_12_vs_lib4)





//                               1         2          3        4          5
// void kernel_dunpack_nn_8_lib4(int kmax, double *A, int sda, double *C, int ldc)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dunpack_nn_8_lib4)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG4, %r13 // C
	movq	ARG5, %r14 // ldc
	sall	$ 3, %r14d


	cmpl	$ 0, %r10d
	jle		0f // return


	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovapd		0(%r11), %ymm0
	vmovupd		%ymm0, 0(%r13)
	vmovapd		0(%r11, %r12, 1), %ymm0
	vmovupd		%ymm0, 32(%r13)
	addq	%r14, %r13

	vmovapd		32(%r11), %ymm0
	vmovupd		%ymm0, 0(%r13)
	vmovapd		32(%r11, %r12, 1), %ymm0
	vmovupd		%ymm0, 32(%r13)
	addq	%r14, %r13

	vmovapd		64(%r11), %ymm0
	vmovupd		%ymm0, 0(%r13)
	vmovapd		64(%r11, %r12, 1), %ymm0
	vmovupd		%ymm0, 32(%r13)
	addq	%r14, %r13

	vmovapd		96(%r11), %ymm0
	vmovupd		%ymm0, 0(%r13)
	vmovapd		96(%r11, %r12, 1), %ymm0
	vmovupd		%ymm0, 32(%r13)
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 128, %r11

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovapd		0(%r11), %ymm0
	vmovupd		%ymm0, 0(%r13)
	vmovapd		0(%r11, %r12, 1), %ymm0
	vmovupd		%ymm0, 32(%r13)
	addq	%r14, %r13

	subl	$ 1, %r10d
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		2b


0: // return

	EPILOGUE

	ret

	FUN_END(kernel_dunpack_nn_8_lib4)





//                                  1         2          3        4          5        6
// void kernel_dunpack_nn_8_vs_lib4(int kmax, double *A, int sda, double *C, int ldc, int m1)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dunpack_nn_8_vs_lib4)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG4, %r13 // C
	movq	ARG5, %r14 // ldc
	sall	$ 3, %r14d
	movq	ARG6, %r15 // m1


	cmpl	$ 0, %r10d
	jle		0f // return


	vcvtsi2sd	%r15d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC01(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC01(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm15


	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovapd		0(%r11), %ymm0
	vmovupd		%ymm0, 0(%r13)
	vmovapd		0(%r11, %r12, 1), %ymm0
	vmaskmovpd	%ymm0, %ymm15, 32(%r13)
	addq	%r14, %r13

	vmovapd		32(%r11), %ymm0
	vmovupd		%ymm0, 0(%r13)
	vmovapd		32(%r11, %r12, 1), %ymm0
	vmaskmovpd	%ymm0, %ymm15, 32(%r13)
	addq	%r14, %r13

	vmovapd		64(%r11), %ymm0
	vmovupd		%ymm0, 0(%r13)
	vmovapd		64(%r11, %r12, 1), %ymm0
	vmaskmovpd	%ymm0, %ymm15, 32(%r13)
	addq	%r14, %r13

	vmovapd		96(%r11), %ymm0
	vmovupd		%ymm0, 0(%r13)
	vmovapd		96(%r11, %r12, 1), %ymm0
	vmaskmovpd	%ymm0, %ymm15, 32(%r13)
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 128, %r11

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovapd		0(%r11), %ymm0
	vmovupd		%ymm0, 0(%r13)
	vmovapd		0(%r11, %r12, 1), %ymm0
	vmaskmovpd	%ymm0, %ymm15, 32(%r13)
	addq	%r14, %r13

	subl	$ 1, %r10d
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		2b


0: // return

	EPILOGUE

	ret

	FUN_END(kernel_dunpack_nn_8_vs_lib4)





//                               1         2          3          4
// void kernel_dunpack_nn_4_lib4(int kmax, double *A, double *C, int ldc)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dunpack_nn_4_lib4)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // C
	movq	ARG4, %r13 // ldc
	sall	$ 3, %r13d


	cmpl	$ 0, %r10d
	jle		0f // return


	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovapd		0(%r11), %ymm0
	vmovupd		%ymm0, 0(%r12)
	addq	%r13, %r12

	vmovapd		32(%r11), %ymm0
	vmovupd		%ymm0, 0(%r12)
	addq	%r13, %r12

	vmovapd		64(%r11), %ymm0
	vmovupd		%ymm0, 0(%r12)
	addq	%r13, %r12

	vmovapd		96(%r11), %ymm0
	vmovupd		%ymm0, 0(%r12)
	addq	%r13, %r12

	subl	$ 4, %r10d
	addq	$ 128, %r11

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovapd		0(%r11), %ymm0
	vmovupd		%ymm0, 0(%r12)
	addq	%r13, %r12

	subl	$ 1, %r10d
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		2b


0: // return

	EPILOGUE

	ret

	FUN_END(kernel_dunpack_nn_4_lib4)





//                               1         2          3          4        5
// void kernel_dunpack_nn_4_lib4(int kmax, double *A, double *C, int ldc, int m1)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dunpack_nn_4_vs_lib4)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // C
	movq	ARG4, %r13 // ldc
	sall	$ 3, %r13d
	movq	ARG5, %r14 // m1


	cmpl	$ 0, %r10d
	jle		0f // return


	vcvtsi2sd	%r14d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC00(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC00(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm15


	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovapd		0(%r11), %ymm0
	vmaskmovpd	%ymm0, %ymm15, 0(%r12)
	addq	%r13, %r12

	vmovapd		32(%r11), %ymm0
	vmaskmovpd	%ymm0, %ymm15, 0(%r12)
	addq	%r13, %r12

	vmovapd		64(%r11), %ymm0
	vmaskmovpd	%ymm0, %ymm15, 0(%r12)
	addq	%r13, %r12

	vmovapd		96(%r11), %ymm0
	vmaskmovpd	%ymm0, %ymm15, 0(%r12)
	addq	%r13, %r12

	subl	$ 4, %r10d
	addq	$ 128, %r11

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovapd		0(%r11), %ymm0
	vmaskmovpd	%ymm0, %ymm15, 0(%r12)
	addq	%r13, %r12

	subl	$ 1, %r10d
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		2b


0: // return

	EPILOGUE

	ret

	FUN_END(kernel_dunpack_nn_4_vs_lib4)





//                               1         2          3          4
// void kernel_dunpack_nt_4_lib4(int kmax, double *C, double *A, int lda)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dunpack_nt_4_lib4)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // C
	movq	ARG3, %r12 // A
	movq	ARG4, %r13 // lda
	sall	$ 3, %r13d


#if MACRO_LEVEL>=1
	INNER_KERNEL_DUNPACK_NT_4_LIB4
#else
	CALL(inner_kernel_dunpack_nt_4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dunpack_nt_4_lib4)





//                                  1         2          3          4        5
// void kernel_dunpack_nt_4_vs_lib4(int kmax, double *C, double *A, int lda, int m1)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dunpack_nt_4_vs_lib4)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // C
	movq	ARG3, %r12 // A
	movq	ARG4, %r13 // lda
	sall	$ 3, %r13d

	movq	ARG5, %r14 // m1
	cmpl	$ 1, %r14d
	jg		100f

#if MACRO_LEVEL>=1
	INNER_KERNEL_DUNPACK_NT_1_LIB4
#else
	CALL(inner_kernel_dunpack_nt_1_lib4)
#endif

	jmp		103f

100:

	movq	ARG5, %r14 // m1
	cmpl	$ 2, %r14d
	jg		101f

#if MACRO_LEVEL>=1
	INNER_KERNEL_DUNPACK_NT_2_LIB4
#else
	CALL(inner_kernel_dunpack_nt_2_lib4)
#endif
	
	jmp		103f

101:

	movq	ARG5, %r14 // m1
	cmpl	$ 3, %r14d
	jg		102f

#if MACRO_LEVEL>=1
	INNER_KERNEL_DUNPACK_NT_3_LIB4
#else
	CALL(inner_kernel_dunpack_nt_3_lib4)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=1
	INNER_KERNEL_DUNPACK_NT_4_LIB4
#else
	CALL(inner_kernel_dunpack_nt_4_lib4)
#endif

103:


	EPILOGUE

	ret

	FUN_END(kernel_dunpack_nt_4_vs_lib4)





//                                1         2          3        4          5
// void kernel_dunpack_tt_12_lib4(int kmax, double *A, int sda, double *C, int ldc)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dunpack_tt_12_lib4)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG4, %r13 // C
	movq	ARG5, %r14 // ldc
	sall	$ 3, %r14d


	cmpl	$ 0, %r10d
	jle		0f // return


	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	movq	%r13, %r15

	vmovapd		0(%r11), %ymm0
	vmovupd		%ymm0, 0(%r15)
	addq	%r14, %r15

	vmovapd		32(%r11), %ymm0
	vmovupd		%ymm0, 0(%r15)
	addq	%r14, %r15

	vmovapd		64(%r11), %ymm0
	vmovupd		%ymm0, 0(%r15)
	addq	%r14, %r15

	vmovapd		96(%r11), %ymm0
	vmovupd		%ymm0, 0(%r15)
	addq	%r14, %r15

	vmovapd		128(%r11), %ymm0
	vmovupd		%ymm0, 0(%r15)
	addq	%r14, %r15

	vmovapd		160(%r11), %ymm0
	vmovupd		%ymm0, 0(%r15)
	addq	%r14, %r15

	vmovapd		192(%r11), %ymm0
	vmovupd		%ymm0, 0(%r15)
	addq	%r14, %r15

	vmovapd		224(%r11), %ymm0
	vmovupd		%ymm0, 0(%r15)
	addq	%r14, %r15

	vmovapd		256(%r11), %ymm0
	vmovupd		%ymm0, 0(%r15)
	addq	%r14, %r15

	vmovapd		288(%r11), %ymm0
	vmovupd		%ymm0, 0(%r15)
	addq	%r14, %r15

	vmovapd		320(%r11), %ymm0
	vmovupd		%ymm0, 0(%r15)
	addq	%r14, %r15

	vmovapd		352(%r11), %ymm0
	vmovupd		%ymm0, 0(%r15)
	addq	%r14, %r15

	subl	$ 4, %r10d
	addq	%r12, %r11
	addq	$ 32, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	movq	%r13, %r15

	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r15)
	addq	%r14, %r15

	vmovsd		32(%r11), %xmm0
	vmovsd		%xmm0, 0(%r15)
	addq	%r14, %r15

	vmovsd		64(%r11), %xmm0
	vmovsd		%xmm0, 0(%r15)
	addq	%r14, %r15

	vmovsd		96(%r11), %xmm0
	vmovsd		%xmm0, 0(%r15)
	addq	%r14, %r15

	vmovsd		128(%r11), %xmm0
	vmovsd		%xmm0, 0(%r15)
	addq	%r14, %r15

	vmovsd		160(%r11), %xmm0
	vmovsd		%xmm0, 0(%r15)
	addq	%r14, %r15

	vmovsd		192(%r11), %xmm0
	vmovsd		%xmm0, 0(%r15)
	addq	%r14, %r15

	vmovsd		224(%r11), %xmm0
	vmovsd		%xmm0, 0(%r15)
	addq	%r14, %r15

	vmovsd		256(%r11), %xmm0
	vmovsd		%xmm0, 0(%r15)
	addq	%r14, %r15

	vmovsd		288(%r11), %xmm0
	vmovsd		%xmm0, 0(%r15)
	addq	%r14, %r15

	vmovsd		320(%r11), %xmm0
	vmovsd		%xmm0, 0(%r15)
	addq	%r14, %r15

	vmovsd		352(%r11), %xmm0
	vmovsd		%xmm0, 0(%r15)
	addq	%r14, %r15

	subl	$ 1, %r10d
	addq	$ 8, %r11
	addq	$ 8, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

	EPILOGUE

	ret

	FUN_END(kernel_dunpack_tt_12_lib4)





//                               1         2          3        4          5
// void kernel_dunpack_tt_8_lib4(int kmax, double *A, int sda, double *C, int ldc)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dunpack_tt_8_lib4)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG4, %r13 // C
	movq	ARG5, %r14 // ldc
	sall	$ 3, %r14d


	cmpl	$ 0, %r10d
	jle		0f // return


	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	movq	%r13, %r15

	vmovapd		0(%r11), %ymm0
	vmovupd		%ymm0, 0(%r15)
	addq	%r14, %r15

	vmovapd		32(%r11), %ymm0
	vmovupd		%ymm0, 0(%r15)
	addq	%r14, %r15

	vmovapd		64(%r11), %ymm0
	vmovupd		%ymm0, 0(%r15)
	addq	%r14, %r15

	vmovapd		96(%r11), %ymm0
	vmovupd		%ymm0, 0(%r15)
	addq	%r14, %r15

	vmovapd		128(%r11), %ymm0
	vmovupd		%ymm0, 0(%r15)
	addq	%r14, %r15

	vmovapd		160(%r11), %ymm0
	vmovupd		%ymm0, 0(%r15)
	addq	%r14, %r15

	vmovapd		192(%r11), %ymm0
	vmovupd		%ymm0, 0(%r15)
	addq	%r14, %r15

	vmovapd		224(%r11), %ymm0
	vmovupd		%ymm0, 0(%r15)
	addq	%r14, %r15

	subl	$ 4, %r10d
	addq	%r12, %r11
	addq	$ 32, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	movq	%r13, %r15

	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r15)
	addq	%r14, %r15

	vmovsd		32(%r11), %xmm0
	vmovsd		%xmm0, 0(%r15)
	addq	%r14, %r15

	vmovsd		64(%r11), %xmm0
	vmovsd		%xmm0, 0(%r15)
	addq	%r14, %r15

	vmovsd		96(%r11), %xmm0
	vmovsd		%xmm0, 0(%r15)
	addq	%r14, %r15

	vmovsd		128(%r11), %xmm0
	vmovsd		%xmm0, 0(%r15)
	addq	%r14, %r15

	vmovsd		160(%r11), %xmm0
	vmovsd		%xmm0, 0(%r15)
	addq	%r14, %r15

	vmovsd		192(%r11), %xmm0
	vmovsd		%xmm0, 0(%r15)
	addq	%r14, %r15

	vmovsd		224(%r11), %xmm0
	vmovsd		%xmm0, 0(%r15)
	addq	%r14, %r15

	subl	$ 1, %r10d
	addq	$ 8, %r11
	addq	$ 8, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

	EPILOGUE

	ret

	FUN_END(kernel_dunpack_tt_8_lib4)





//                               1         2          3        4          5
// void kernel_dunpack_tt_4_lib4(int kmax, double *A, int sda, double *C, int ldc)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dunpack_tt_4_lib4)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG4, %r13 // C
	movq	ARG5, %r14 // ldc
	sall	$ 3, %r14d


	cmpl	$ 0, %r10d
	jle		0f // return


	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	movq	%r13, %r15

//	prefetcht0	0(%r11, %r12) // software prefetch
//	prefetcht0	64(%r11, %r12) // software prefetch

	vmovapd		0(%r11), %ymm0
	vmovupd		%ymm0, 0(%r15)
	addq	%r14, %r15

	vmovapd		32(%r11), %ymm0
	vmovupd		%ymm0, 0(%r15)
	addq	%r14, %r15

	vmovapd		64(%r11), %ymm0
	vmovupd		%ymm0, 0(%r15)
	addq	%r14, %r15

	vmovapd		96(%r11), %ymm0
	vmovupd		%ymm0, 0(%r15)
	addq	%r14, %r15

	subl	$ 4, %r10d
	addq	%r12, %r11
	addq	$ 32, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	movq	%r13, %r15

	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r15)
	addq	%r14, %r15

	vmovsd		32(%r11), %xmm0
	vmovsd		%xmm0, 0(%r15)
	addq	%r14, %r15

	vmovsd		64(%r11), %xmm0
	vmovsd		%xmm0, 0(%r15)
	addq	%r14, %r15

	vmovsd		96(%r11), %xmm0
	vmovsd		%xmm0, 0(%r15)
	addq	%r14, %r15

	subl	$ 1, %r10d
	addq	$ 8, %r11
	addq	$ 8, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

	EPILOGUE

	ret

	FUN_END(kernel_dunpack_tt_4_lib4)





//                             1         2          3            4          5
// void kernel_dpacp_nt_4_lib4(int kmax, double *A, int offsetC, double *C, int sdc)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpacp_nt_4_lib4)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // offsetC
	movq	ARG4, %r13 // C
	movq	ARG5, %r14 // sdc
	sall	$ 5, %r14d

#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACP_NT_4_LIB4
#else
	CALL(inner_kernel_dpacp_nt_4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dpacp_nt_4_lib4)





//                             1         2            3          4        5
// void kernel_dpacp_tn_4_lib4(int kmax, int offsetA, double *A, int sda, double *C)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpacp_tn_4_lib4)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // offsetA
	movq	ARG3, %r12 // A
	movq	ARG4, %r13 // sda
	sall	$ 5, %r13d
	movq	ARG5, %r14 // C

#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACP_TN_4_LIB4
#else
	CALL(inner_kernel_dpacp_tn_4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dpacp_tn_4_lib4)





//                              1         2            3          4        5          6
// void kernel_dpacp_nn_12_lib4(int kmax, int offsetA, double *A, int sda, double *C, int ldc)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpacp_nn_12_lib4)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG5, %r13 // C
	movq	ARG6, %r14 // sdc
	sall	$ 5, %r14d


	cmpl	$ 0, ARG2L
	je		100f
	cmpl	$ 1, ARG2L
	je		101f
	cmpl	$ 2, ARG2L
	je		102f
	cmpl	$ 3, ARG2L
	je		103f

	jmp		200f

100:
#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACP_NN_12_0_LIB4
#else
	CALL(inner_kernel_dpacp_nn_12_0_lib4)
#endif
	jmp		200f

101:
#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACP_NN_12_1_LIB4
#else
	CALL(inner_kernel_dpacp_nn_12_1_lib4)
#endif
	jmp		200f

102:
#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACP_NN_12_2_LIB4
#else
	CALL(inner_kernel_dpacp_nn_12_2_lib4)
#endif
	jmp		200f

103:
#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACP_NN_12_3_LIB4
#else
	CALL(inner_kernel_dpacp_nn_12_3_lib4)
#endif
	jmp		200f

200:

	EPILOGUE

	ret

	FUN_END(kernel_dpacp_nn_12_lib4)





//                                 1         2            3          4        5          6        7
// void kernel_dpacp_nn_12_vs_lib4(int kmax, int offsetA, double *A, int sda, double *C, int ldc, int m1)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpacp_nn_12_vs_lib4)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG5, %r13 // C
	movq	ARG6, %r14 // sdc
	sall	$ 5, %r14d
	movq	ARG7, %r15 // m1


	cmpl	$ 0, ARG2L
	je		100f
	cmpl	$ 1, ARG2L
	je		101f
	cmpl	$ 2, ARG2L
	je		102f
	cmpl	$ 3, ARG2L
	je		103f

	jmp		200f

100:
#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACP_NN_12_0_VS_LIB4
#else
	CALL(inner_kernel_dpacp_nn_12_0_vs_lib4)
#endif
	jmp		200f

101:
#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACP_NN_12_1_VS_LIB4
#else
	CALL(inner_kernel_dpacp_nn_12_1_vs_lib4)
#endif
	jmp		200f

102:
#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACP_NN_12_2_VS_LIB4
#else
	CALL(inner_kernel_dpacp_nn_12_2_vs_lib4)
#endif
	jmp		200f

103:
#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACP_NN_12_3_VS_LIB4
#else
	CALL(inner_kernel_dpacp_nn_12_3_vs_lib4)
#endif
	jmp		200f

200:

	EPILOGUE

	ret

	FUN_END(kernel_dpacp_nn_12_vs_lib4)





//                             1         2            3          4        5          6
// void kernel_dpacp_nn_8_lib4(int kmax, int offsetA, double *A, int sda, double *C, int ldc)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpacp_nn_8_lib4)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG5, %r13 // C
	movq	ARG6, %r14 // sdc
	sall	$ 5, %r14d


	cmpl	$ 0, ARG2L
	je		100f
	cmpl	$ 1, ARG2L
	je		101f
	cmpl	$ 2, ARG2L
	je		102f
	cmpl	$ 3, ARG2L
	je		103f

	jmp		200f

100:
#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACP_NN_8_0_LIB4
#else
	CALL(inner_kernel_dpacp_nn_8_0_lib4)
#endif
	jmp		200f

101:
#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACP_NN_8_1_LIB4
#else
	CALL(inner_kernel_dpacp_nn_8_1_lib4)
#endif
	jmp		200f

102:
#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACP_NN_8_2_LIB4
#else
	CALL(inner_kernel_dpacp_nn_8_2_lib4)
#endif
	jmp		200f

103:
#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACP_NN_8_3_LIB4
#else
	CALL(inner_kernel_dpacp_nn_8_3_lib4)
#endif
	jmp		200f

200:

	EPILOGUE

	ret

	FUN_END(kernel_dpacp_nn_8_lib4)





//                                1         2            3          4        5          6        7
// void kernel_dpacp_nn_8_vs_lib4(int kmax, int offsetA, double *A, int sda, double *C, int ldc, int m1)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpacp_nn_8_vs_lib4)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG5, %r13 // C
	movq	ARG6, %r14 // sdc
	sall	$ 5, %r14d
	movq	ARG7, %r15 // m1


	cmpl	$ 0, ARG2L
	je		100f
	cmpl	$ 1, ARG2L
	je		101f
	cmpl	$ 2, ARG2L
	je		102f
	cmpl	$ 3, ARG2L
	je		103f

	jmp		200f

100:
#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACP_NN_8_0_VS_LIB4
#else
	CALL(inner_kernel_dpacp_nn_8_0_vs_lib4)
#endif
	jmp		200f

101:
#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACP_NN_8_1_VS_LIB4
#else
	CALL(inner_kernel_dpacp_nn_8_1_vs_lib4)
#endif
	jmp		200f

102:
#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACP_NN_8_2_VS_LIB4
#else
	CALL(inner_kernel_dpacp_nn_8_2_vs_lib4)
#endif
	jmp		200f

103:
#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACP_NN_8_3_VS_LIB4
#else
	CALL(inner_kernel_dpacp_nn_8_3_vs_lib4)
#endif
	jmp		200f

200:

	EPILOGUE

	ret

	FUN_END(kernel_dpacp_nn_8_vs_lib4)





//                             1         2            3          4        5
// void kernel_dpacp_nn_4_lib4(int kmax, int offsetA, double *A, int sda, double *C)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpacp_nn_4_lib4)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG5, %r13 // C


	cmpl	$ 0, ARG2L
	je		100f
	cmpl	$ 1, ARG2L
	je		101f
	cmpl	$ 2, ARG2L
	je		102f
	cmpl	$ 3, ARG2L
	je		103f

	jmp		200f

100:
#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACP_NN_4_0_LIB4
#else
	CALL(inner_kernel_dpacp_nn_4_0_lib4)
#endif
	jmp		200f

101:
#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACP_NN_4_1_LIB4
#else
	CALL(inner_kernel_dpacp_nn_4_1_lib4)
#endif
	jmp		200f

102:
#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACP_NN_4_2_LIB4
#else
	CALL(inner_kernel_dpacp_nn_4_2_lib4)
#endif
	jmp		200f

103:
#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACP_NN_4_3_LIB4
#else
	CALL(inner_kernel_dpacp_nn_4_3_lib4)
#endif
	jmp		200f

200:

	EPILOGUE

	ret

	FUN_END(kernel_dpacp_nn_4_lib4)





//                                1         2            3          4        5          6
// void kernel_dpacp_nn_4_vs_lib4(int kmax, int offsetA, double *A, int sda, double *C, int m1)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpacp_nn_4_vs_lib4)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG5, %r13 // C


	cmpl	$ 0, ARG6L
	jle		200f
	cmpl	$ 1, ARG6L
	je		104f
	cmpl	$ 2, ARG6L
	je		105f
	cmpl	$ 3, ARG6L
	je		106f
	cmpl	$ 4, ARG6L
	jge		107f

	jmp		200f


	// m1==1
104:
	cmpl	$ 3, ARG2L
	jle		100f

	jmp		200f

100:
	movq	ARG2, %r14
	sall	$ 3, %r14d
	addq	%r14, %r11
#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACP_NN_1_0_LIB4
#else
	CALL(inner_kernel_dpacp_nn_1_0_lib4)
#endif
	jmp		200f


	// m1==1
105:
	cmpl	$ 2, ARG2L
	jle		100f
	cmpl	$ 3, ARG2L
	je		103f

	jmp		200f

100:
	movq	ARG2, %r14
	sall	$ 3, %r14d
	addq	%r14, %r11
#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACP_NN_2_0_LIB4
#else
	CALL(inner_kernel_dpacp_nn_2_0_lib4)
#endif
	jmp		200f

103:
#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACP_NN_2_1_LIB4
#else
	CALL(inner_kernel_dpacp_nn_2_1_lib4)
#endif
	jmp		200f


	// m1==3
106:
	cmpl	$ 1, ARG2L
	jle		100f
	cmpl	$ 2, ARG2L
	je		102f
	cmpl	$ 3, ARG2L
	je		103f

	jmp		200f

100:
	movq	ARG2, %r14
	sall	$ 3, %r14d
	addq	%r14, %r11
#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACP_NN_3_0_LIB4
#else
	CALL(inner_kernel_dpacp_nn_3_0_lib4)
#endif
	jmp		200f

102:
#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACP_NN_3_2_LIB4
#else
	CALL(inner_kernel_dpacp_nn_3_2_lib4)
#endif
	jmp		200f

103:
#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACP_NN_3_3_LIB4
#else
	CALL(inner_kernel_dpacp_nn_3_3_lib4)
#endif
	jmp		200f



	// m1==4
107:
	cmpl	$ 0, ARG2L
	je		100f
	cmpl	$ 1, ARG2L
	je		101f
	cmpl	$ 2, ARG2L
	je		102f
	cmpl	$ 3, ARG2L
	je		103f

	jmp		200f

100:
#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACP_NN_4_0_LIB4
#else
	CALL(inner_kernel_dpacp_nn_4_0_lib4)
#endif
	jmp		200f

101:
#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACP_NN_4_1_LIB4
#else
	CALL(inner_kernel_dpacp_nn_4_1_lib4)
#endif
	jmp		200f

102:
#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACP_NN_4_2_LIB4
#else
	CALL(inner_kernel_dpacp_nn_4_2_lib4)
#endif
	jmp		200f

103:
#if MACRO_LEVEL>=1
	INNER_KERNEL_DPACP_NN_4_3_LIB4
#else
	CALL(inner_kernel_dpacp_nn_4_3_lib4)
#endif
	jmp		200f

200:

	EPILOGUE

	ret

	FUN_END(kernel_dpacp_nn_4_vs_lib4)





	// read-only data
#if defined(OS_LINUX)
	.section	.rodata.cst32,"aM",@progbits,32
#elif defined(OS_MAC)
	.section	__TEXT,__const
#elif defined(OS_WINDOWS)
	.section .rdata,"dr"
#endif


#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC00:
#elif defined(OS_MAC)
	.align 5
LC00:
#endif
	.double 0.5
	.double 1.5
	.double 2.5
	.double 3.5


#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC01:
#elif defined(OS_MAC)
	.align 5
LC01:
#endif
	.double 4.5
	.double 5.5
	.double 6.5
	.double 7.5


#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC02:
#elif defined(OS_MAC)
	.align 5
LC02:
#endif
	.double 8.5
	.double 9.5
	.double 10.5
	.double 11.5





#if defined(OS_LINUX)
	.section	.note.GNU-stack,"",@progbits
#elif defined(OS_MAC)
	.subsections_via_symbols
#endif

