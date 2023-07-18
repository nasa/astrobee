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





#if 0 // XXX
// input arguments
//
// r10	<- kmax
// r11	<- A
// r12	<- lda
// r13	<- C
// r14	<- ldc
// r15	<- Ap
// rax	<- Bp
//
#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DGETR_NT_8_P0_LIB
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgetr_nt_8_p0_lib)
#endif



	cmpl	$ 0, %r10d
	jle		0f // return

	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	leaq	0(%r12, %r12, 2), %rbx

	prefetcht0	0(%r15)
	prefetcht0	0(%r15, %r12)
	prefetcht0	0(%r15, %r12, 2)
	prefetcht0	0(%r15, %rbx)

	vmovupd		0(%r11), %xmm0
	vmovupd		0(%r11, %r12, 2), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm0, %ymm0
	vmovupd		0(%r11, %r12), %xmm1
	vmovupd		0(%r11, %rbx), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm1, %ymm1
	vmovupd		16(%r11), %xmm2
	vmovupd		16(%r11, %r12, 2), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm2, %ymm2
	vmovupd		16(%r11, %r12), %xmm3
	vmovupd		16(%r11, %rbx), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm3, %ymm3

	prefetcht0	63(%r15)
	prefetcht0	63(%r15, %r12)
	prefetcht0	63(%r15, %r12, 2)
	prefetcht0	63(%r15, %rbx)

	vmovupd		32(%r11), %xmm4
	vmovupd		32(%r11, %r12, 2), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm4, %ymm4
	vmovupd		32(%r11, %r12), %xmm5
	vmovupd		32(%r11, %rbx), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm5, %ymm5
	vmovupd		48(%r11), %xmm6
	vmovupd		48(%r11, %r12, 2), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm6, %ymm6
	vmovupd		48(%r11, %r12), %xmm7
	vmovupd		48(%r11, %rbx), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm7, %ymm7

	leaq	0(%r14, %r14, 2), %rbx

	vunpcklpd	%ymm1, %ymm0, %ymm8
	vmovupd		%ymm8, 0(%r13)
	vunpckhpd	%ymm1, %ymm0, %ymm9
	vmovupd		%ymm9, 0(%r13, %r14)
	vunpcklpd	%ymm3, %ymm2, %ymm10
	vmovupd		%ymm10, 0(%r13, %r14, 2)
	vunpckhpd	%ymm3, %ymm2, %ymm11
	vmovupd		%ymm11, 0(%r13, %rbx)

	prefetcht0	31(%rax)
	prefetcht0	31(%rax, %r14)
	prefetcht0	31(%rax, %r14, 2)
	prefetcht0	31(%rax, %rbx)

	leaq	0(%r13, %r14, 4), %rbp

	vunpcklpd	%ymm5, %ymm4, %ymm8
	vmovupd		%ymm8, 0(%rbp)
	vunpckhpd	%ymm5, %ymm4, %ymm9
	vmovupd		%ymm9, 0(%rbp, %r14)
	vunpcklpd	%ymm7, %ymm6, %ymm10
	vmovupd		%ymm10, 0(%rbp, %r14, 2)
	vunpckhpd	%ymm7, %ymm6, %ymm11
	vmovupd		%ymm11, 0(%rbp, %rbx)

	leaq	0(%rax, %r14, 4), %rbp

	prefetcht0	31(%rbp)
	prefetcht0	31(%rbp, %r14)
	prefetcht0	31(%rbp, %r14, 2)
	prefetcht0	31(%rbp, %rbx)

	subl	$ 4, %r10d
	leaq	0(%r11, %r12, 4), %r11
	leaq	0(%r15, %r12, 4), %r15
	addq	$ 32, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	leaq	0(%r14, %r14, 2), %rbx

	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13)
	vmovsd		8(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13, %r14)
	vmovsd		16(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13, %r14, 2)
	vmovsd		24(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13, %rbx)

	leaq	0(%r13, %r14, 4), %rbp

	vmovsd		32(%r11), %xmm0
	vmovsd		%xmm0, 0(%rbp)
	vmovsd		40(%r11), %xmm0
	vmovsd		%xmm0, 0(%rbp, %r14)
	vmovsd		48(%r11), %xmm0
	vmovsd		%xmm0, 0(%rbp, %r14, 2)
	vmovsd		56(%r11), %xmm0
	vmovsd		%xmm0, 0(%rbp, %rbx)

	subl	$ 1, %r10d
	addq	%r12, %r11
	addq	$ 8, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return



#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dgetr_nt_8_p0_lib)
#endif
#endif





#if 0 // XXX
// input arguments
//
// r10	<- kmax
// r11	<- A
// r12	<- lda
// r13	<- C
// r14	<- ldc
//
#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DGETR_NT_8_LIB
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgetr_nt_8_lib)
#endif



	cmpl	$ 0, %r10d
	jle		0f // return

	leaq	0(%r12, %r12, 2), %r15
	leaq	0(%r14, %r14, 2), %rax

	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovupd		0(%r11), %xmm0
	vmovupd		0(%r11, %r12, 2), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm0, %ymm0
	vmovupd		0(%r11, %r12), %xmm1
	vmovupd		0(%r11, %r15), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm1, %ymm1
	vmovupd		16(%r11), %xmm2
	vmovupd		16(%r11, %r12, 2), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm2, %ymm2
	vmovupd		16(%r11, %r12), %xmm3
	vmovupd		16(%r11, %r15), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm3, %ymm3

	vmovupd		32(%r11), %xmm4
	vmovupd		32(%r11, %r12, 2), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm4, %ymm4
	vmovupd		32(%r11, %r12), %xmm5
	vmovupd		32(%r11, %r15), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm5, %ymm5
	vmovupd		48(%r11), %xmm6
	vmovupd		48(%r11, %r12, 2), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm6, %ymm6
	vmovupd		48(%r11, %r12), %xmm7
	vmovupd		48(%r11, %r15), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm7, %ymm7

	vunpcklpd	%ymm1, %ymm0, %ymm8
	vmovupd		%ymm8, 0(%r13)
	vunpckhpd	%ymm1, %ymm0, %ymm9
	vmovupd		%ymm9, 0(%r13, %r14)
	vunpcklpd	%ymm3, %ymm2, %ymm10
	vmovupd		%ymm10, 0(%r13, %r14, 2)
	vunpckhpd	%ymm3, %ymm2, %ymm11
	vmovupd		%ymm11, 0(%r13, %rax)

	leaq	0(%r13, %r14, 4), %rbx

	vunpcklpd	%ymm5, %ymm4, %ymm8
	vmovupd		%ymm8, 0(%rbx)
	vunpckhpd	%ymm5, %ymm4, %ymm9
	vmovupd		%ymm9, 0(%rbx, %r14)
	vunpcklpd	%ymm7, %ymm6, %ymm10
	vmovupd		%ymm10, 0(%rbx, %r14, 2)
	vunpckhpd	%ymm7, %ymm6, %ymm11
	vmovupd		%ymm11, 0(%rbx, %rax)

	subl	$ 4, %r10d
	leaq	0(%r11, %r12, 4), %r11
	addq	$ 32, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13)
	vmovsd		8(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13, %r14)
	vmovsd		16(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13, %r14, 2)
	vmovsd		24(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13, %rax)

	leaq	0(%r13, %r14, 4), %rbx

	vmovsd		32(%r11), %xmm0
	vmovsd		%xmm0, 0(%rbx)
	vmovsd		40(%r11), %xmm0
	vmovsd		%xmm0, 0(%rbx, %r14)
	vmovsd		48(%r11), %xmm0
	vmovsd		%xmm0, 0(%rbx, %r14, 2)
	vmovsd		56(%r11), %xmm0
	vmovsd		%xmm0, 0(%rbx, %rax)

	subl	$ 1, %r10d
	addq	%r12, %r11
	addq	$ 8, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return



#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dgetr_nt_8_lib)
#endif
#endif





#if 0 // XXX
// input arguments
//
// r10	<- kmax
// r11	<- A
// r12	<- lda
// r13	<- C
// r14	<- ldc
//
#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DGETR_NT_4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgetr_nt_4_lib)
#endif



	cmpl	$ 0, %r10d
	jle		0f // return

	leaq	0(%r12, %r12, 2), %r15
	leaq	0(%r14, %r14, 2), %rax

	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovupd		0(%r11), %xmm0
	vmovupd		0(%r11, %r12, 2), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm0, %ymm0
	vmovupd		0(%r11, %r12), %xmm1
	vmovupd		0(%r11, %r15), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm1, %ymm1
	vmovupd		16(%r11), %xmm2
	vmovupd		16(%r11, %r12, 2), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm2, %ymm2
	vmovupd		16(%r11, %r12), %xmm3
	vmovupd		16(%r11, %r15), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm3, %ymm3

	vunpcklpd	%ymm1, %ymm0, %ymm4
	vmovupd		%ymm4, 0(%r13)
	vunpckhpd	%ymm1, %ymm0, %ymm5
	vmovupd		%ymm5, 0(%r13, %r14)
	vunpcklpd	%ymm3, %ymm2, %ymm6
	vmovupd		%ymm6, 0(%r13, %r14, 2)
	vunpckhpd	%ymm3, %ymm2, %ymm7
	vmovupd		%ymm7, 0(%r13, %rax)

	subl	$ 4, %r10d
	leaq	0(%r11, %r12, 4), %r11
	addq	$ 32, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13)
	vmovsd		8(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13, %r14)
	vmovsd		16(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13, %r14, 2)
	vmovsd		24(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13, %rax)

	subl	$ 1, %r10d
	addq	%r12, %r11
	addq	$ 8, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return



#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dgetr_nt_4_lib)
#endif
#endif





// input arguments
//
// r10	<- kmax
// r11	<- A
// r12	<- lda
// r13	<- C
// r14	<- ldc
//
#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DGETR_TN_8_LIB
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgetr_tn_8_lib)
#endif



	cmpl	$ 0, %r10d
	jle		0f // return

	leaq	0(%r12, %r12, 2), %r15
	leaq	0(%r14, %r14, 2), %rax

	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovupd		0(%r11), %xmm0
	vmovupd		0(%r11, %r12, 2), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm0, %ymm0
	vmovupd		0(%r11, %r12), %xmm1
	vmovupd		0(%r11, %r15), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm1, %ymm1
	vmovupd		16(%r11), %xmm2
	vmovupd		16(%r11, %r12, 2), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm2, %ymm2
	vmovupd		16(%r11, %r12), %xmm3
	vmovupd		16(%r11, %r15), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm3, %ymm3

	leaq	0(%r11, %r12, 4), %rbx

	vmovupd		0(%rbx), %xmm4
	vmovupd		0(%rbx, %r12, 2), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm4, %ymm4
	vmovupd		0(%rbx, %r12), %xmm5
	vmovupd		0(%rbx, %r15), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm5, %ymm5
	vmovupd		16(%rbx), %xmm6
	vmovupd		16(%rbx, %r12, 2), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm6, %ymm6
	vmovupd		16(%rbx, %r12), %xmm7
	vmovupd		16(%rbx, %r15), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm7, %ymm7

	vunpcklpd	%ymm1, %ymm0, %ymm8
	vmovupd		%ymm8, 0(%r13)
	vunpckhpd	%ymm1, %ymm0, %ymm9
	vmovupd		%ymm9, 0(%r13, %r14)
	vunpcklpd	%ymm3, %ymm2, %ymm10
	vmovupd		%ymm10, 0(%r13, %r14, 2)
	vunpckhpd	%ymm3, %ymm2, %ymm11
	vmovupd		%ymm11, 0(%r13, %rax)

	vunpcklpd	%ymm5, %ymm4, %ymm8
	vmovupd		%ymm8, 32(%r13)
	vunpckhpd	%ymm5, %ymm4, %ymm9
	vmovupd		%ymm9, 32(%r13, %r14)
	vunpcklpd	%ymm7, %ymm6, %ymm10
	vmovupd		%ymm10, 32(%r13, %r14, 2)
	vunpckhpd	%ymm7, %ymm6, %ymm11
	vmovupd		%ymm11, 32(%r13, %rax)

	subl	$ 4, %r10d
	addq	$ 32, %r11
	leaq	0(%r13, %r14, 4), %r13

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
	vmovsd		0(%r11, %r15), %xmm0
	vmovsd		%xmm0, 24(%r13)

	leaq	0(%r11, %r12, 4), %rbx

	vmovsd		0(%rbx), %xmm0
	vmovsd		%xmm0, 32(%r13)
	vmovsd		0(%rbx, %r12), %xmm0
	vmovsd		%xmm0, 40(%r13)
	vmovsd		0(%rbx, %r12, 2), %xmm0
	vmovsd		%xmm0, 48(%r13)
	vmovsd		0(%rbx, %r15), %xmm0
	vmovsd		%xmm0, 56(%r13)

	subl	$ 1, %r10d
	addq	$ 8, %r11
	addq	%r14, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return



#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dgetr_tn_8_lib)
#endif





// input arguments
//
// r10	<- kmax
// r11	<- A
// r12	<- lda
// r13	<- C
// r14	<- ldc
//
#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DGETR_TN_4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgetr_tn_4_lib)
#endif



	cmpl	$ 0, %r10d
	jle		0f // return

	leaq	0(%r12, %r12, 2), %r15
	leaq	0(%r14, %r14, 2), %rax

	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovupd		0(%r11), %xmm0
	vmovupd		0(%r11, %r12, 2), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm0, %ymm0
	vmovupd		0(%r11, %r12), %xmm1
	vmovupd		0(%r11, %r15), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm1, %ymm1
	vmovupd		16(%r11), %xmm2
	vmovupd		16(%r11, %r12, 2), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm2, %ymm2
	vmovupd		16(%r11, %r12), %xmm3
	vmovupd		16(%r11, %r15), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm3, %ymm3

	vunpcklpd	%ymm1, %ymm0, %ymm4
	vmovupd		%ymm4, 0(%r13)
	vunpckhpd	%ymm1, %ymm0, %ymm5
	vmovupd		%ymm5, 0(%r13, %r14)
	vunpcklpd	%ymm3, %ymm2, %ymm6
	vmovupd		%ymm6, 0(%r13, %r14, 2)
	vunpckhpd	%ymm3, %ymm2, %ymm7
	vmovupd		%ymm7, 0(%r13, %rax)

	subl	$ 4, %r10d
	addq	$ 32, %r11
	leaq	0(%r13, %r14, 4), %r13

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
	vmovsd		0(%r11, %r15), %xmm0
	vmovsd		%xmm0, 24(%r13)

	subl	$ 1, %r10d
	addq	$ 8, %r11
	addq	%r14, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return



#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dgetr_tn_4_lib)
#endif





// input arguments
//
// r10	<- kmax
// r11	<- A
// r12	<- lda
// r13	<- C
// r14	<- ldc
//
#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DGETR_TN_3_LIB
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgetr_tn_3_lib)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC00(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC00(%rip), %ymm14
#endif
	leaq	0(%r14, %r14, 2), %rax

	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovupd		0(%r11), %xmm0
	vmovupd		0(%r11, %r12, 2), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm0, %ymm0
	vmovupd		0(%r11, %r12, 1), %xmm1
	vmovupd		16(%r11), %xmm2
	vmovupd		16(%r11, %r12, 2), %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm2, %ymm2
	vmovupd		16(%r11, %r12, 1), %xmm3

	vunpcklpd	%ymm1, %ymm0, %ymm4
	vmaskmovpd	%ymm4, %ymm14, 0(%r13)
	vunpckhpd	%ymm1, %ymm0, %ymm5
	vmaskmovpd	%ymm5, %ymm14, 0(%r13, %r14)
	vunpcklpd	%ymm3, %ymm2, %ymm6
	vmaskmovpd	%ymm6, %ymm14, 0(%r13, %r14, 2)
	vunpckhpd	%ymm3, %ymm2, %ymm7
	vmaskmovpd	%ymm7, %ymm14, 0(%r13, %rax)

	subl	$ 4, %r10d
	addq	$ 32, %r11
	leaq	0(%r13, %r14, 4), %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13)
	vmovsd		0(%r11, %r12, 1), %xmm0
	vmovsd		%xmm0, 8(%r13)
	vmovsd		0(%r11, %r12, 2), %xmm0
	vmovsd		%xmm0, 16(%r13)

	subl	$ 1, %r10d
	addq	$ 8, %r11
	addq	%r14, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dgetr_tn_3_lib)
#endif





// input arguments
//
// r10	<- kmax
// r11	<- A
// r12	<- lda
// r13	<- C
// r14	<- ldc
//
#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DGETR_TN_2_LIB
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgetr_tn_2_lib)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return


#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC00(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC00(%rip), %ymm14
#endif
	leaq	0(%r14, %r14, 2), %rax


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
	vmovupd		%xmm4, 0(%r13)
	vunpckhpd	%ymm1, %ymm0, %ymm5
	vmovupd		%xmm5, 0(%r13, %r14)
	vunpcklpd	%ymm3, %ymm2, %ymm6
	vmovupd		%xmm6, 0(%r13, %r14, 2)
	vunpckhpd	%ymm3, %ymm2, %ymm7
	vmovupd		%xmm7, 0(%r13, %rax)

	subl	$ 4, %r10d
	addq	$ 32, %r11
	leaq	0(%r13, %r14, 4), %r13

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
	addq	%r14, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dgetr_tn_2_lib)
#endif





// input arguments
//
// r10	<- kmax
// r11	<- A
// r12	<- lda
// r13	<- C
// r14	<- ldc
//
#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DGETR_TN_1_LIB
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgetr_tn_1_lib)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return


	leaq	0(%r14, %r14, 2), %rax


	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13)
	vmovsd		8(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13, %r14)
	vmovsd		16(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13, %r14, 2)
	vmovsd		24(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13, %rax)

	subl	$ 4, %r10d
	addq	$ 32, %r11
	leaq	0(%r13, %r14, 4), %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13)

	subl	$ 1, %r10d
	addq	$ 8, %r11
	addq	%r14, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dgetr_tn_1_lib)
#endif





#if 0 // XXX
//                               1         2          3        4          5
// void kernel_dgetr_nt_8_p0_lib(int kmax, double *A, int lda, double *C, int ldc, double *Ap, double *Bp)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgetr_nt_8_p0_lib)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$ 3, %r12d
	movq	ARG4, %r13 // C
	movq	ARG5, %r14 // ldc
	sall	$ 3, %r14d
	movq	ARG6, %r15 // Ap
	movq	ARG7, %rax // Bp


#if MACRO_LEVEL>=1
	INNER_KERNEL_DGETR_NT_8_P0_LIB
#else
	CALL(inner_kernel_dgetr_nt_8_p0_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgetr_nt_8_p0_lib)
#endif





#if 0
//                            1         2          3        4          5
// void kernel_dgetr_nt_8_lib(int kmax, double *A, int lda, double *C, int ldc)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgetr_nt_8_lib)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$ 3, %r12d
	movq	ARG4, %r13 // C
	movq	ARG5, %r14 // ldc
	sall	$ 3, %r14d


#if MACRO_LEVEL>=1
	INNER_KERNEL_DGETR_NT_8_LIB
#else
	CALL(inner_kernel_dgetr_nt_8_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgetr_nt_8_lib)
#endif





#if 0 // XXX
//                            1         2          3        4          5
// void kernel_dgetr_nt_4_lib(int kmax, double *A, int lda, double *C, int ldc)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgetr_nt_4_lib)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$ 3, %r12d
	movq	ARG4, %r13 // C
	movq	ARG5, %r14 // ldc
	sall	$ 3, %r14d


#if MACRO_LEVEL>=1
	INNER_KERNEL_DGETR_NT_4_LIB
#else
	CALL(inner_kernel_dgetr_nt_4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgetr_nt_4_lib)
#endif





//                            1         2          3        4          5
// void kernel_dgetr_tn_8_lib(int kmax, double *A, int lda, double *C, int ldc)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgetr_tn_8_lib)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$ 3, %r12d
	movq	ARG4, %r13 // C
	movq	ARG5, %r14 // ldc
	sall	$ 3, %r14d


#if MACRO_LEVEL>=1
	INNER_KERNEL_DGETR_TN_8_LIB
#else
	CALL(inner_kernel_dgetr_tn_8_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgetr_tn_8_lib)





//                            1         2          3        4          5
// void kernel_dgetr_tn_4_lib(int kmax, double *A, int lda, double *C, int ldc)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgetr_tn_4_lib)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$ 3, %r12d
	movq	ARG4, %r13 // C
	movq	ARG5, %r14 // ldc
	sall	$ 3, %r14d


#if MACRO_LEVEL>=1
	INNER_KERNEL_DGETR_TN_4_LIB
#else
	CALL(inner_kernel_dgetr_tn_4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgetr_tn_4_lib)





//                               1         2          3        4          5        6
// void kernel_dgetr_tn_4_vs_lib(int kmax, double *A, int lda, double *C, int ldc, int m1)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgetr_tn_4_vs_lib)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$ 3, %r12d
	movq	ARG4, %r13 // C
	movq	ARG5, %r14 // ldc
	sall	$ 3, %r14d

	movq	ARG6, %r15 // m1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=1
	INNER_KERNEL_DGETR_TN_1_LIB
#else
	CALL(inner_kernel_dgetr_tn_1_lib)
#endif

	jmp		103f

100:

	movq	ARG6, %r15 // m1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=1
	INNER_KERNEL_DGETR_TN_2_LIB
#else
	CALL(inner_kernel_dgetr_tn_2_lib)
#endif
	
	jmp		103f

101:

	movq	ARG6, %r15 // m1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=1
	INNER_KERNEL_DGETR_TN_3_LIB
#else
	CALL(inner_kernel_dgetr_tn_3_lib)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=1
	INNER_KERNEL_DGETR_TN_4_LIB
#else
	CALL(inner_kernel_dgetr_tn_4_lib)
#endif

103:


	EPILOGUE

	ret

	FUN_END(kernel_dgetr_tn_4_vs_lib)





#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC00:
#elif defined(OS_MAC)
	.align 5
LC00:
#endif
	.quad	-1
	.quad	-1
	.quad	-1
	.quad	1





#if defined(OS_LINUX)
	.section	.note.GNU-stack,"",@progbits
#elif defined(OS_MAC)
	.subsections_via_symbols
#endif

