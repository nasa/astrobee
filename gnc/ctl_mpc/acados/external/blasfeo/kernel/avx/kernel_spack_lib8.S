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
#define ZERO_ACC \
	vxorpd	%ymm0, %ymm0, %ymm0; \
	vmovapd	%ymm0, %ymm1; \
	vmovapd	%ymm0, %ymm2; \
	vmovapd	%ymm0, %ymm3
#define NEG_ACC \
	vmovapd		.LC11(%rip), %ymm15; \
	vxorpd		%ymm15, %ymm0, %ymm0; \
	vxorpd		%ymm15, %ymm1, %ymm1; \
	vxorpd		%ymm15, %ymm2, %ymm2; \
	vxorpd		%ymm15, %ymm3, %ymm3

#else // defined(OS_MAC)

#define GLOB_FUN_START(NAME) \
	.globl _ ## NAME; \
_ ## NAME:
#define FUN_START(NAME) \
_ ## NAME:
#define FUN_END(NAME)
#define CALL(NAME) \
	callq _ ## NAME
#define ZERO_ACC \
	vxorpd	%ymm0, %ymm0, %ymm0; \
	vmovapd	%ymm0, %ymm1; \
	vmovapd	%ymm0, %ymm2; \
	vmovapd	%ymm0, %ymm3
#define NEG_ACC \
	vmovapd		LC11(%rip), %ymm15; \
	vxorpd		%ymm15, %ymm0, %ymm0; \
	vxorpd		%ymm15, %ymm1, %ymm1; \
	vxorpd		%ymm15, %ymm2, %ymm2; \
	vxorpd		%ymm15, %ymm3, %ymm3

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
#define ZERO_ACC \
	vxorpd	%ymm0, %ymm0, %ymm0; \
	vmovapd	%ymm0, %ymm1; \
	vmovapd	%ymm0, %ymm2; \
	vmovapd	%ymm0, %ymm3
#define NEG_ACC \
	vmovapd		.LC11(%rip), %ymm15; \
	vxorpd		%ymm15, %ymm0, %ymm0; \
	vxorpd		%ymm15, %ymm1, %ymm1; \
	vxorpd		%ymm15, %ymm2, %ymm2; \
	vxorpd		%ymm15, %ymm3, %ymm3

#else

#error wrong OS

#endif



#if defined(OS_LINUX)
	.text
#elif defined(OS_MAC)
	.section	__TEXT,__text,regular,pure_instructions
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_SPACK_TN_8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_spack_tn_8_lib8)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return


	leaq	0(%r11, %r12, 4), %r14
	leaq	0(%r12, %r12, 2), %r15


	cmpl	$ 7, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovups		0(%r11), %ymm0
	vmovups		0(%r11, %r12), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm8
	vunpckhps	%ymm1, %ymm0, %ymm9
	vmovups		0(%r11, %r12, 2), %ymm0
	vmovups		0(%r11, %r15), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm10
	vunpckhps	%ymm1, %ymm0, %ymm11
	vmovups		0(%r14), %ymm0
	vmovups		0(%r14, %r12), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm12
	vunpckhps	%ymm1, %ymm0, %ymm13
	vmovups		0(%r14, %r12, 2), %ymm0
	vmovups		0(%r14, %r15), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm14
	vunpckhps	%ymm1, %ymm0, %ymm15

	vshufps		$ 0x44, %ymm10, %ymm8, %ymm0
	vshufps		$ 0x44, %ymm14, %ymm12, %ymm1
	vperm2f128	$ 0x20, %ymm1, %ymm0, %ymm2
	vperm2f128	$ 0x31, %ymm1, %ymm0, %ymm3
	vmovaps		%ymm2, 0(%r13)
	vmovaps		%ymm3, 128(%r13)
	vshufps		$ 0xee, %ymm10, %ymm8, %ymm0
	vshufps		$ 0xee, %ymm14, %ymm12, %ymm1
	vperm2f128	$ 0x20, %ymm1, %ymm0, %ymm2
	vperm2f128	$ 0x31, %ymm1, %ymm0, %ymm3
	vmovaps		%ymm2, 32(%r13)
	vmovaps		%ymm3, 160(%r13)
	vshufps		$ 0x44, %ymm11, %ymm9, %ymm0
	vshufps		$ 0x44, %ymm15, %ymm13, %ymm1
	vperm2f128	$ 0x20, %ymm1, %ymm0, %ymm2
	vperm2f128	$ 0x31, %ymm1, %ymm0, %ymm3
	vmovaps		%ymm2, 64(%r13)
	vmovaps		%ymm3, 192(%r13)
	vshufps		$ 0xee, %ymm11, %ymm9, %ymm0
	vshufps		$ 0xee, %ymm15, %ymm13, %ymm1
	vperm2f128	$ 0x20, %ymm1, %ymm0, %ymm2
	vperm2f128	$ 0x31, %ymm1, %ymm0, %ymm3
	vmovaps		%ymm2, 96(%r13)
	vmovaps		%ymm3, 224(%r13)

	subl	$ 8, %r10d
	addq	$ 32, %r11
	addq	$ 32, %r14
	addq	$ 256, %r13

	cmpl	$ 7, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovss		0(%r11), %xmm0
	vmovss		%xmm0, 0(%r13)
	vmovss		0(%r11, %r12), %xmm0
	vmovss		%xmm0, 4(%r13)
	vmovss		0(%r11, %r12, 2), %xmm0
	vmovss		%xmm0, 8(%r13)
	vmovss		0(%r11, %r15), %xmm0
	vmovss		%xmm0, 12(%r13)
	vmovss		0(%r14), %xmm0
	vmovss		%xmm0, 16(%r13)
	vmovss		0(%r14, %r12), %xmm0
	vmovss		%xmm0, 20(%r13)
	vmovss		0(%r14, %r12, 2), %xmm0
	vmovss		%xmm0, 24(%r13)
	vmovss		0(%r14, %r15), %xmm0
	vmovss		%xmm0, 28(%r13)

	subl	$ 1, %r10d
	addq	$ 4, %r11
	addq	$ 4, %r14
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_spack_tn_8_lib8)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_SPACK_TN_7_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_spack_tn_7_lib8)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return


	leaq	0(%r11, %r12, 4), %r14
	leaq	0(%r12, %r12, 2), %r15


	cmpl	$ 7, %r10d
	jle		2f // clean-up loop


#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC03(%rip), %ymm4
#elif defined(OS_MAC)
	vmovups		LC03(%rip), %ymm4
#endif
	vxorps		%ymm5, %ymm5, %ymm5
	vblendps	$ 0x80, %ymm5, %ymm4, %ymm4


	// main loop
	.p2align 3
1: // main loop
	
	vmovups		0(%r11), %ymm0
	vmovups		0(%r11, %r12), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm8
	vunpckhps	%ymm1, %ymm0, %ymm9
	vmovups		0(%r11, %r12, 2), %ymm0
	vmovups		0(%r11, %r15), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm10
	vunpckhps	%ymm1, %ymm0, %ymm11
	vmovups		0(%r14), %ymm0
	vmovups		0(%r14, %r12), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm12
	vunpckhps	%ymm1, %ymm0, %ymm13
	vmovups		0(%r14, %r12, 2), %ymm0
//	vmovups		0(%r14, %r15), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm14
	vunpckhps	%ymm1, %ymm0, %ymm15

	vshufps		$ 0x44, %ymm10, %ymm8, %ymm0
	vshufps		$ 0x44, %ymm14, %ymm12, %ymm1
	vperm2f128	$ 0x20, %ymm1, %ymm0, %ymm2
	vperm2f128	$ 0x31, %ymm1, %ymm0, %ymm3
	vmaskmovps	%ymm2, %ymm4, 0(%r13)
	vmaskmovps	%ymm3, %ymm4, 128(%r13)
	vshufps		$ 0xee, %ymm10, %ymm8, %ymm0
	vshufps		$ 0xee, %ymm14, %ymm12, %ymm1
	vperm2f128	$ 0x20, %ymm1, %ymm0, %ymm2
	vperm2f128	$ 0x31, %ymm1, %ymm0, %ymm3
	vmaskmovps	%ymm2, %ymm4, 32(%r13)
	vmaskmovps	%ymm3, %ymm4, 160(%r13)
	vshufps		$ 0x44, %ymm11, %ymm9, %ymm0
	vshufps		$ 0x44, %ymm15, %ymm13, %ymm1
	vperm2f128	$ 0x20, %ymm1, %ymm0, %ymm2
	vperm2f128	$ 0x31, %ymm1, %ymm0, %ymm3
	vmaskmovps	%ymm2, %ymm4, 64(%r13)
	vmaskmovps	%ymm3, %ymm4, 192(%r13)
	vshufps		$ 0xee, %ymm11, %ymm9, %ymm0
	vshufps		$ 0xee, %ymm15, %ymm13, %ymm1
	vperm2f128	$ 0x20, %ymm1, %ymm0, %ymm2
	vperm2f128	$ 0x31, %ymm1, %ymm0, %ymm3
	vmaskmovps	%ymm2, %ymm4, 96(%r13)
	vmaskmovps	%ymm3, %ymm4, 224(%r13)

	subl	$ 8, %r10d
	addq	$ 32, %r11
	addq	$ 32, %r14
	addq	$ 256, %r13

	cmpl	$ 7, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovss		0(%r11), %xmm0
	vmovss		%xmm0, 0(%r13)
	vmovss		0(%r11, %r12), %xmm0
	vmovss		%xmm0, 4(%r13)
	vmovss		0(%r11, %r12, 2), %xmm0
	vmovss		%xmm0, 8(%r13)
	vmovss		0(%r11, %r15), %xmm0
	vmovss		%xmm0, 12(%r13)
	vmovss		0(%r14), %xmm0
	vmovss		%xmm0, 16(%r13)
	vmovss		0(%r14, %r12), %xmm0
	vmovss		%xmm0, 20(%r13)
	vmovss		0(%r14, %r12, 2), %xmm0
	vmovss		%xmm0, 24(%r13)
//	vmovss		0(%r14, %r15), %xmm0
//	vmovss		%xmm0, 28(%r13)

	subl	$ 1, %r10d
	addq	$ 4, %r11
	addq	$ 4, %r14
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_spack_tn_7_lib8)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_SPACK_TN_6_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_spack_tn_6_lib8)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return


	leaq	0(%r11, %r12, 4), %r14
	leaq	0(%r12, %r12, 2), %r15


	cmpl	$ 7, %r10d
	jle		2f // clean-up loop


#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC03(%rip), %ymm4
#elif defined(OS_MAC)
	vmovups		LC03(%rip), %ymm4
#endif
	vxorps		%ymm5, %ymm5, %ymm5
	vblendps	$ 0xc0, %ymm5, %ymm4, %ymm4


	// main loop
	.p2align 3
1: // main loop
	
	vmovups		0(%r11), %ymm0
	vmovups		0(%r11, %r12), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm8
	vunpckhps	%ymm1, %ymm0, %ymm9
	vmovups		0(%r11, %r12, 2), %ymm0
	vmovups		0(%r11, %r15), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm10
	vunpckhps	%ymm1, %ymm0, %ymm11
	vmovups		0(%r14), %ymm0
	vmovups		0(%r14, %r12), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm12
	vunpckhps	%ymm1, %ymm0, %ymm13
//	vmovups		0(%r14, %r12, 2), %ymm0
//	vmovups		0(%r14, %r15), %ymm1
//	vunpcklps	%ymm1, %ymm0, %ymm14
//	vunpckhps	%ymm1, %ymm0, %ymm15

	vshufps		$ 0x44, %ymm10, %ymm8, %ymm0
	vshufps		$ 0x44, %ymm14, %ymm12, %ymm1
	vperm2f128	$ 0x20, %ymm1, %ymm0, %ymm2
	vperm2f128	$ 0x31, %ymm1, %ymm0, %ymm3
	vmaskmovps	%ymm2, %ymm4, 0(%r13)
	vmaskmovps	%ymm3, %ymm4, 128(%r13)
	vshufps		$ 0xee, %ymm10, %ymm8, %ymm0
	vshufps		$ 0xee, %ymm14, %ymm12, %ymm1
	vperm2f128	$ 0x20, %ymm1, %ymm0, %ymm2
	vperm2f128	$ 0x31, %ymm1, %ymm0, %ymm3
	vmaskmovps	%ymm2, %ymm4, 32(%r13)
	vmaskmovps	%ymm3, %ymm4, 160(%r13)
	vshufps		$ 0x44, %ymm11, %ymm9, %ymm0
	vshufps		$ 0x44, %ymm15, %ymm13, %ymm1
	vperm2f128	$ 0x20, %ymm1, %ymm0, %ymm2
	vperm2f128	$ 0x31, %ymm1, %ymm0, %ymm3
	vmaskmovps	%ymm2, %ymm4, 64(%r13)
	vmaskmovps	%ymm3, %ymm4, 192(%r13)
	vshufps		$ 0xee, %ymm11, %ymm9, %ymm0
	vshufps		$ 0xee, %ymm15, %ymm13, %ymm1
	vperm2f128	$ 0x20, %ymm1, %ymm0, %ymm2
	vperm2f128	$ 0x31, %ymm1, %ymm0, %ymm3
	vmaskmovps	%ymm2, %ymm4, 96(%r13)
	vmaskmovps	%ymm3, %ymm4, 224(%r13)

	subl	$ 8, %r10d
	addq	$ 32, %r11
	addq	$ 32, %r14
	addq	$ 256, %r13

	cmpl	$ 7, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovss		0(%r11), %xmm0
	vmovss		%xmm0, 0(%r13)
	vmovss		0(%r11, %r12), %xmm0
	vmovss		%xmm0, 4(%r13)
	vmovss		0(%r11, %r12, 2), %xmm0
	vmovss		%xmm0, 8(%r13)
	vmovss		0(%r11, %r15), %xmm0
	vmovss		%xmm0, 12(%r13)
	vmovss		0(%r14), %xmm0
	vmovss		%xmm0, 16(%r13)
	vmovss		0(%r14, %r12), %xmm0
	vmovss		%xmm0, 20(%r13)
//	vmovss		0(%r14, %r12, 2), %xmm0
//	vmovss		%xmm0, 24(%r13)
//	vmovss		0(%r14, %r15), %xmm0
//	vmovss		%xmm0, 28(%r13)

	subl	$ 1, %r10d
	addq	$ 4, %r11
	addq	$ 4, %r14
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_spack_tn_6_lib8)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_SPACK_TN_5_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_spack_tn_5_lib8)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return


	leaq	0(%r11, %r12, 4), %r14
	leaq	0(%r12, %r12, 2), %r15


	cmpl	$ 7, %r10d
	jle		2f // clean-up loop


#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC03(%rip), %ymm4
#elif defined(OS_MAC)
	vmovups		LC03(%rip), %ymm4
#endif
	vxorps		%ymm5, %ymm5, %ymm5
	vblendps	$ 0xe0, %ymm5, %ymm4, %ymm4


	// main loop
	.p2align 3
1: // main loop
	
	vmovups		0(%r11), %ymm0
	vmovups		0(%r11, %r12), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm8
	vunpckhps	%ymm1, %ymm0, %ymm9
	vmovups		0(%r11, %r12, 2), %ymm0
	vmovups		0(%r11, %r15), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm10
	vunpckhps	%ymm1, %ymm0, %ymm11
	vmovups		0(%r14), %ymm0
//	vmovups		0(%r14, %r12), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm12
	vunpckhps	%ymm1, %ymm0, %ymm13
//	vmovups		0(%r14, %r12, 2), %ymm0
//	vmovups		0(%r14, %r15), %ymm1
//	vunpcklps	%ymm1, %ymm0, %ymm14
//	vunpckhps	%ymm1, %ymm0, %ymm15

	vshufps		$ 0x44, %ymm10, %ymm8, %ymm0
	vshufps		$ 0x44, %ymm14, %ymm12, %ymm1
	vperm2f128	$ 0x20, %ymm1, %ymm0, %ymm2
	vperm2f128	$ 0x31, %ymm1, %ymm0, %ymm3
	vmaskmovps	%ymm2, %ymm4, 0(%r13)
	vmaskmovps	%ymm3, %ymm4, 128(%r13)
	vshufps		$ 0xee, %ymm10, %ymm8, %ymm0
	vshufps		$ 0xee, %ymm14, %ymm12, %ymm1
	vperm2f128	$ 0x20, %ymm1, %ymm0, %ymm2
	vperm2f128	$ 0x31, %ymm1, %ymm0, %ymm3
	vmaskmovps	%ymm2, %ymm4, 32(%r13)
	vmaskmovps	%ymm3, %ymm4, 160(%r13)
	vshufps		$ 0x44, %ymm11, %ymm9, %ymm0
	vshufps		$ 0x44, %ymm15, %ymm13, %ymm1
	vperm2f128	$ 0x20, %ymm1, %ymm0, %ymm2
	vperm2f128	$ 0x31, %ymm1, %ymm0, %ymm3
	vmaskmovps	%ymm2, %ymm4, 64(%r13)
	vmaskmovps	%ymm3, %ymm4, 192(%r13)
	vshufps		$ 0xee, %ymm11, %ymm9, %ymm0
	vshufps		$ 0xee, %ymm15, %ymm13, %ymm1
	vperm2f128	$ 0x20, %ymm1, %ymm0, %ymm2
	vperm2f128	$ 0x31, %ymm1, %ymm0, %ymm3
	vmaskmovps	%ymm2, %ymm4, 96(%r13)
	vmaskmovps	%ymm3, %ymm4, 224(%r13)

	subl	$ 8, %r10d
	addq	$ 32, %r11
	addq	$ 32, %r14
	addq	$ 256, %r13

	cmpl	$ 7, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovss		0(%r11), %xmm0
	vmovss		%xmm0, 0(%r13)
	vmovss		0(%r11, %r12), %xmm0
	vmovss		%xmm0, 4(%r13)
	vmovss		0(%r11, %r12, 2), %xmm0
	vmovss		%xmm0, 8(%r13)
	vmovss		0(%r11, %r15), %xmm0
	vmovss		%xmm0, 12(%r13)
	vmovss		0(%r14), %xmm0
	vmovss		%xmm0, 16(%r13)
//	vmovss		0(%r14, %r12), %xmm0
//	vmovss		%xmm0, 20(%r13)
//	vmovss		0(%r14, %r12, 2), %xmm0
//	vmovss		%xmm0, 24(%r13)
//	vmovss		0(%r14, %r15), %xmm0
//	vmovss		%xmm0, 28(%r13)

	subl	$ 1, %r10d
	addq	$ 4, %r11
	addq	$ 4, %r14
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_spack_tn_5_lib8)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_SPACK_TN_4_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_spack_tn_4_lib8)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return


	leaq	0(%r11, %r12, 4), %r14
	leaq	0(%r12, %r12, 2), %r15


	cmpl	$ 7, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovups		0(%r11), %ymm0
	vmovups		0(%r11, %r12), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm8
	vunpckhps	%ymm1, %ymm0, %ymm9
	vmovups		0(%r11, %r12, 2), %ymm0
	vmovups		0(%r11, %r15), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm10
	vunpckhps	%ymm1, %ymm0, %ymm11

	vshufps			$ 0x44, %ymm10, %ymm8, %ymm0
	vextractf128	$ 0x1, %ymm0, %xmm1
	vmovaps			%xmm0, 0(%r13)
	vmovaps			%xmm1, 128(%r13)
	vshufps			$ 0xee, %ymm10, %ymm8, %ymm0
	vextractf128	$ 0x1, %ymm0, %xmm1
	vmovaps			%xmm0, 32(%r13)
	vmovaps			%xmm1, 160(%r13)
	vshufps			$ 0x44, %ymm11, %ymm9, %ymm0
	vextractf128	$ 0x1, %ymm0, %xmm1
	vmovaps			%xmm0, 64(%r13)
	vmovaps			%xmm1, 192(%r13)
	vshufps			$ 0xee, %ymm11, %ymm9, %ymm0
	vextractf128	$ 0x1, %ymm0, %xmm1
	vmovaps			%xmm0, 96(%r13)
	vmovaps			%xmm1, 224(%r13)

	subl	$ 8, %r10d
	addq	$ 32, %r11
	addq	$ 32, %r14
	addq	$ 256, %r13

	cmpl	$ 7, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovss		0(%r11), %xmm0
	vmovss		%xmm0, 0(%r13)
	vmovss		0(%r11, %r12), %xmm0
	vmovss		%xmm0, 4(%r13)
	vmovss		0(%r11, %r12, 2), %xmm0
	vmovss		%xmm0, 8(%r13)
	vmovss		0(%r11, %r15), %xmm0
	vmovss		%xmm0, 12(%r13)

	subl	$ 1, %r10d
	addq	$ 4, %r11
	addq	$ 4, %r14
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_spack_tn_4_lib8)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_SPACK_TN_3_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_spack_tn_3_lib8)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return


	leaq	0(%r11, %r12, 4), %r14
	leaq	0(%r12, %r12, 2), %r15


	cmpl	$ 7, %r10d
	jle		2f // clean-up loop


#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC03(%rip), %ymm4
#elif defined(OS_MAC)
	vmovups		LC03(%rip), %ymm4
#endif
	vxorps		%ymm5, %ymm5, %ymm5
	vblendps	$ 0xf8, %ymm5, %ymm4, %ymm4


	// main loop
	.p2align 3
1: // main loop
	
	vmovups		0(%r11), %ymm0
	vmovups		0(%r11, %r12), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm8
	vunpckhps	%ymm1, %ymm0, %ymm9
	vmovups		0(%r11, %r12, 2), %ymm0
//	vmovups		0(%r11, %r15), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm10
	vunpckhps	%ymm1, %ymm0, %ymm11

	vshufps			$ 0x44, %ymm10, %ymm8, %ymm0
	vextractf128	$ 0x1, %ymm0, %xmm1
	vmaskmovps		%xmm0, %xmm4, 0(%r13)
	vmaskmovps		%xmm1, %xmm4, 128(%r13)
	vshufps			$ 0xee, %ymm10, %ymm8, %ymm0
	vextractf128	$ 0x1, %ymm0, %xmm1
	vmaskmovps		%xmm0, %xmm4, 32(%r13)
	vmaskmovps		%xmm1, %xmm4, 160(%r13)
	vshufps			$ 0x44, %ymm11, %ymm9, %ymm0
	vextractf128	$ 0x1, %ymm0, %xmm1
	vmaskmovps		%xmm0, %xmm4, 64(%r13)
	vmaskmovps		%xmm1, %xmm4, 192(%r13)
	vshufps			$ 0xee, %ymm11, %ymm9, %ymm0
	vextractf128	$ 0x1, %ymm0, %xmm1
	vmaskmovps		%xmm0, %xmm4, 96(%r13)
	vmaskmovps		%xmm1, %xmm4, 224(%r13)

	subl	$ 8, %r10d
	addq	$ 32, %r11
	addq	$ 32, %r14
	addq	$ 256, %r13

	cmpl	$ 7, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovss		0(%r11), %xmm0
	vmovss		%xmm0, 0(%r13)
	vmovss		0(%r11, %r12), %xmm0
	vmovss		%xmm0, 4(%r13)
	vmovss		0(%r11, %r12, 2), %xmm0
	vmovss		%xmm0, 8(%r13)
//	vmovss		0(%r11, %r15), %xmm0
//	vmovss		%xmm0, 12(%r13)

	subl	$ 1, %r10d
	addq	$ 4, %r11
	addq	$ 4, %r14
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_spack_tn_3_lib8)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_SPACK_TN_2_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_spack_tn_2_lib8)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return


	leaq	0(%r11, %r12, 4), %r14
	leaq	0(%r12, %r12, 2), %r15


	cmpl	$ 7, %r10d
	jle		2f // clean-up loop


#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC03(%rip), %ymm4
#elif defined(OS_MAC)
	vmovups		LC03(%rip), %ymm4
#endif
	vxorps		%ymm5, %ymm5, %ymm5
	vblendps	$ 0xfc, %ymm5, %ymm4, %ymm4


	// main loop
	.p2align 3
1: // main loop
	
	vmovups		0(%r11), %ymm0
	vmovups		0(%r11, %r12), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm8
	vunpckhps	%ymm1, %ymm0, %ymm9
//	vmovups		0(%r11, %r12, 2), %ymm0
//	vmovups		0(%r11, %r15), %ymm1
//	vunpcklps	%ymm1, %ymm0, %ymm10
//	vunpckhps	%ymm1, %ymm0, %ymm11

	vshufps			$ 0x44, %ymm10, %ymm8, %ymm0
	vextractf128	$ 0x1, %ymm0, %xmm1
	vmaskmovps		%xmm0, %xmm4, 0(%r13)
	vmaskmovps		%xmm1, %xmm4, 128(%r13)
	vshufps			$ 0xee, %ymm10, %ymm8, %ymm0
	vextractf128	$ 0x1, %ymm0, %xmm1
	vmaskmovps		%xmm0, %xmm4, 32(%r13)
	vmaskmovps		%xmm1, %xmm4, 160(%r13)
	vshufps			$ 0x44, %ymm11, %ymm9, %ymm0
	vextractf128	$ 0x1, %ymm0, %xmm1
	vmaskmovps		%xmm0, %xmm4, 64(%r13)
	vmaskmovps		%xmm1, %xmm4, 192(%r13)
	vshufps			$ 0xee, %ymm11, %ymm9, %ymm0
	vextractf128	$ 0x1, %ymm0, %xmm1
	vmaskmovps		%xmm0, %xmm4, 96(%r13)
	vmaskmovps		%xmm1, %xmm4, 224(%r13)

	subl	$ 8, %r10d
	addq	$ 32, %r11
	addq	$ 32, %r14
	addq	$ 256, %r13

	cmpl	$ 7, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovss		0(%r11), %xmm0
	vmovss		%xmm0, 0(%r13)
	vmovss		0(%r11, %r12), %xmm0
	vmovss		%xmm0, 4(%r13)
//	vmovss		0(%r11, %r12, 2), %xmm0
//	vmovss		%xmm0, 8(%r13)
//	vmovss		0(%r11, %r15), %xmm0
//	vmovss		%xmm0, 12(%r13)

	subl	$ 1, %r10d
	addq	$ 4, %r11
	addq	$ 4, %r14
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_spack_tn_2_lib8)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_SPACK_TN_1_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_spack_tn_1_lib8)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return


	leaq	0(%r11, %r12, 4), %r14
	leaq	0(%r12, %r12, 2), %r15


	cmpl	$ 7, %r10d
	jle		2f // clean-up loop


#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC03(%rip), %ymm4
#elif defined(OS_MAC)
	vmovups		LC03(%rip), %ymm4
#endif
	vxorps		%ymm5, %ymm5, %ymm5
	vblendps	$ 0xfe, %ymm5, %ymm4, %ymm4


	// main loop
	.p2align 3
1: // main loop
	
	vmovups		0(%r11), %ymm0
//	vmovups		0(%r11, %r12), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm8
	vunpckhps	%ymm1, %ymm0, %ymm9
//	vmovups		0(%r11, %r12, 2), %ymm0
//	vmovups		0(%r11, %r15), %ymm1
//	vunpcklps	%ymm1, %ymm0, %ymm10
//	vunpckhps	%ymm1, %ymm0, %ymm11

	vshufps			$ 0x44, %ymm10, %ymm8, %ymm0
	vextractf128	$ 0x1, %ymm0, %xmm1
	vmaskmovps		%xmm0, %xmm4, 0(%r13)
	vmaskmovps		%xmm1, %xmm4, 128(%r13)
	vshufps			$ 0xee, %ymm10, %ymm8, %ymm0
	vextractf128	$ 0x1, %ymm0, %xmm1
	vmaskmovps		%xmm0, %xmm4, 32(%r13)
	vmaskmovps		%xmm1, %xmm4, 160(%r13)
	vshufps			$ 0x44, %ymm11, %ymm9, %ymm0
	vextractf128	$ 0x1, %ymm0, %xmm1
	vmaskmovps		%xmm0, %xmm4, 64(%r13)
	vmaskmovps		%xmm1, %xmm4, 192(%r13)
	vshufps			$ 0xee, %ymm11, %ymm9, %ymm0
	vextractf128	$ 0x1, %ymm0, %xmm1
	vmaskmovps		%xmm0, %xmm4, 96(%r13)
	vmaskmovps		%xmm1, %xmm4, 224(%r13)

	subl	$ 8, %r10d
	addq	$ 32, %r11
	addq	$ 32, %r14
	addq	$ 256, %r13

	cmpl	$ 7, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovss		0(%r11), %xmm0
	vmovss		%xmm0, 0(%r13)
//	vmovss		0(%r11, %r12), %xmm0
//	vmovss		%xmm0, 4(%r13)
//	vmovss		0(%r11, %r12, 2), %xmm0
//	vmovss		%xmm0, 8(%r13)
//	vmovss		0(%r11, %r15), %xmm0
//	vmovss		%xmm0, 12(%r13)

	subl	$ 1, %r10d
	addq	$ 4, %r11
	addq	$ 4, %r14
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_spack_tn_1_lib8)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_SPACK_TT_8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_spack_tt_8_lib8)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return


	cmpl	$ 7, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	movq	%r11, %r15

	vmovups		0(%r15), %ymm0
	vmovaps		%ymm0, 0(%r13)
	addq	%r12, %r15

	vmovups		0(%r15), %ymm0
	vmovaps		%ymm0, 32(%r13)
	addq	%r12, %r15

	vmovups		0(%r15), %ymm0
	vmovaps		%ymm0, 64(%r13)
	addq	%r12, %r15

	vmovups		0(%r15), %ymm0
	vmovaps		%ymm0, 96(%r13)
	addq	%r12, %r15

	vmovups		0(%r15), %ymm0
	vmovaps		%ymm0, 128(%r13)
	addq	%r12, %r15

	vmovups		0(%r15), %ymm0
	vmovaps		%ymm0, 160(%r13)
	addq	%r12, %r15

	vmovups		0(%r15), %ymm0
	vmovaps		%ymm0, 192(%r13)
	addq	%r12, %r15

	vmovups		0(%r15), %ymm0
	vmovaps		%ymm0, 224(%r13)
	addq	%r12, %r15

	subl	$ 8, %r10d
	addq	$ 32, %r11
	addq	%r14, %r13

	cmpl	$ 7, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	movq	%r11, %r15

	vmovss		0(%r15), %xmm0
	vmovss		%xmm0, 0(%r13)
	addq	%r12, %r15

	vmovss		0(%r15), %xmm0
	vmovss		%xmm0, 32(%r13)
	addq	%r12, %r15

	vmovss		0(%r15), %xmm0
	vmovss		%xmm0, 64(%r13)
	addq	%r12, %r15

	vmovss		0(%r15), %xmm0
	vmovss		%xmm0, 96(%r13)
	addq	%r12, %r15

	vmovss		0(%r15), %xmm0
	vmovss		%xmm0, 128(%r13)
	addq	%r12, %r15

	vmovss		0(%r15), %xmm0
	vmovss		%xmm0, 160(%r13)
	addq	%r12, %r15

	vmovss		0(%r15), %xmm0
	vmovss		%xmm0, 192(%r13)
	addq	%r12, %r15

	vmovss		0(%r15), %xmm0
	vmovss		%xmm0, 224(%r13)
	addq	%r12, %r15

	subl	$ 1, %r10d
	addq	$ 4, %r11
	addq	$ 4, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_spack_tt_8_lib8)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_SPACK_TT_7_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_spack_tt_7_lib8)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return


	cmpl	$ 7, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	movq	%r11, %r15

	vmovups		0(%r15), %ymm0
	vmovaps		%ymm0, 0(%r13)
	addq	%r12, %r15

	vmovups		0(%r15), %ymm0
	vmovaps		%ymm0, 32(%r13)
	addq	%r12, %r15

	vmovups		0(%r15), %ymm0
	vmovaps		%ymm0, 64(%r13)
	addq	%r12, %r15

	vmovups		0(%r15), %ymm0
	vmovaps		%ymm0, 96(%r13)
	addq	%r12, %r15

	vmovups		0(%r15), %ymm0
	vmovaps		%ymm0, 128(%r13)
	addq	%r12, %r15

	vmovups		0(%r15), %ymm0
	vmovaps		%ymm0, 160(%r13)
	addq	%r12, %r15

	vmovups		0(%r15), %ymm0
	vmovaps		%ymm0, 192(%r13)
	addq	%r12, %r15

	subl	$ 8, %r10d
	addq	$ 32, %r11
	addq	%r14, %r13

	cmpl	$ 7, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	movq	%r11, %r15

	vmovss		0(%r15), %xmm0
	vmovss		%xmm0, 0(%r13)
	addq	%r12, %r15

	vmovss		0(%r15), %xmm0
	vmovss		%xmm0, 32(%r13)
	addq	%r12, %r15

	vmovss		0(%r15), %xmm0
	vmovss		%xmm0, 64(%r13)
	addq	%r12, %r15

	vmovss		0(%r15), %xmm0
	vmovss		%xmm0, 96(%r13)
	addq	%r12, %r15

	vmovss		0(%r15), %xmm0
	vmovss		%xmm0, 128(%r13)
	addq	%r12, %r15

	vmovss		0(%r15), %xmm0
	vmovss		%xmm0, 160(%r13)
	addq	%r12, %r15

	vmovss		0(%r15), %xmm0
	vmovss		%xmm0, 192(%r13)
	addq	%r12, %r15

	subl	$ 1, %r10d
	addq	$ 4, %r11
	addq	$ 4, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_spack_tt_7_lib8)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_SPACK_TT_6_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_spack_tt_6_lib8)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return


	cmpl	$ 7, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	movq	%r11, %r15

	vmovups		0(%r15), %ymm0
	vmovaps		%ymm0, 0(%r13)
	addq	%r12, %r15

	vmovups		0(%r15), %ymm0
	vmovaps		%ymm0, 32(%r13)
	addq	%r12, %r15

	vmovups		0(%r15), %ymm0
	vmovaps		%ymm0, 64(%r13)
	addq	%r12, %r15

	vmovups		0(%r15), %ymm0
	vmovaps		%ymm0, 96(%r13)
	addq	%r12, %r15

	vmovups		0(%r15), %ymm0
	vmovaps		%ymm0, 128(%r13)
	addq	%r12, %r15

	vmovups		0(%r15), %ymm0
	vmovaps		%ymm0, 160(%r13)
	addq	%r12, %r15

	subl	$ 8, %r10d
	addq	$ 32, %r11
	addq	%r14, %r13

	cmpl	$ 7, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	movq	%r11, %r15

	vmovss		0(%r15), %xmm0
	vmovss		%xmm0, 0(%r13)
	addq	%r12, %r15

	vmovss		0(%r15), %xmm0
	vmovss		%xmm0, 32(%r13)
	addq	%r12, %r15

	vmovss		0(%r15), %xmm0
	vmovss		%xmm0, 64(%r13)
	addq	%r12, %r15

	vmovss		0(%r15), %xmm0
	vmovss		%xmm0, 96(%r13)
	addq	%r12, %r15

	vmovss		0(%r15), %xmm0
	vmovss		%xmm0, 128(%r13)
	addq	%r12, %r15

	vmovss		0(%r15), %xmm0
	vmovss		%xmm0, 160(%r13)
	addq	%r12, %r15

	subl	$ 1, %r10d
	addq	$ 4, %r11
	addq	$ 4, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_spack_tt_6_lib8)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_SPACK_TT_5_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_spack_tt_5_lib8)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return


	cmpl	$ 7, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	movq	%r11, %r15

	vmovups		0(%r15), %ymm0
	vmovaps		%ymm0, 0(%r13)
	addq	%r12, %r15

	vmovups		0(%r15), %ymm0
	vmovaps		%ymm0, 32(%r13)
	addq	%r12, %r15

	vmovups		0(%r15), %ymm0
	vmovaps		%ymm0, 64(%r13)
	addq	%r12, %r15

	vmovups		0(%r15), %ymm0
	vmovaps		%ymm0, 96(%r13)
	addq	%r12, %r15

	vmovups		0(%r15), %ymm0
	vmovaps		%ymm0, 128(%r13)
	addq	%r12, %r15

	subl	$ 8, %r10d
	addq	$ 32, %r11
	addq	%r14, %r13

	cmpl	$ 7, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	movq	%r11, %r15

	vmovss		0(%r15), %xmm0
	vmovss		%xmm0, 0(%r13)
	addq	%r12, %r15

	vmovss		0(%r15), %xmm0
	vmovss		%xmm0, 32(%r13)
	addq	%r12, %r15

	vmovss		0(%r15), %xmm0
	vmovss		%xmm0, 64(%r13)
	addq	%r12, %r15

	vmovss		0(%r15), %xmm0
	vmovss		%xmm0, 96(%r13)
	addq	%r12, %r15

	vmovss		0(%r15), %xmm0
	vmovss		%xmm0, 128(%r13)
	addq	%r12, %r15

	subl	$ 1, %r10d
	addq	$ 4, %r11
	addq	$ 4, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_spack_tt_5_lib8)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_SPACK_TT_4_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_spack_tt_4_lib8)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return


	cmpl	$ 7, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	movq	%r11, %r15

	vmovups		0(%r15), %ymm0
	vmovaps		%ymm0, 0(%r13)
	addq	%r12, %r15

	vmovups		0(%r15), %ymm0
	vmovaps		%ymm0, 32(%r13)
	addq	%r12, %r15

	vmovups		0(%r15), %ymm0
	vmovaps		%ymm0, 64(%r13)
	addq	%r12, %r15

	vmovups		0(%r15), %ymm0
	vmovaps		%ymm0, 96(%r13)
	addq	%r12, %r15

	subl	$ 8, %r10d
	addq	$ 32, %r11
	addq	%r14, %r13

	cmpl	$ 7, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	movq	%r11, %r15

	vmovss		0(%r15), %xmm0
	vmovss		%xmm0, 0(%r13)
	addq	%r12, %r15

	vmovss		0(%r15), %xmm0
	vmovss		%xmm0, 32(%r13)
	addq	%r12, %r15

	vmovss		0(%r15), %xmm0
	vmovss		%xmm0, 64(%r13)
	addq	%r12, %r15

	vmovss		0(%r15), %xmm0
	vmovss		%xmm0, 96(%r13)
	addq	%r12, %r15

	subl	$ 1, %r10d
	addq	$ 4, %r11
	addq	$ 4, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_spack_tt_4_lib8)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_SPACK_TT_3_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_spack_tt_3_lib8)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return


	cmpl	$ 7, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	movq	%r11, %r15

	vmovups		0(%r15), %ymm0
	vmovaps		%ymm0, 0(%r13)
	addq	%r12, %r15

	vmovups		0(%r15), %ymm0
	vmovaps		%ymm0, 32(%r13)
	addq	%r12, %r15

	vmovups		0(%r15), %ymm0
	vmovaps		%ymm0, 64(%r13)
	addq	%r12, %r15

	subl	$ 8, %r10d
	addq	$ 32, %r11
	addq	%r14, %r13

	cmpl	$ 7, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	movq	%r11, %r15

	vmovss		0(%r15), %xmm0
	vmovss		%xmm0, 0(%r13)
	addq	%r12, %r15

	vmovss		0(%r15), %xmm0
	vmovss		%xmm0, 32(%r13)
	addq	%r12, %r15

	vmovss		0(%r15), %xmm0
	vmovss		%xmm0, 64(%r13)
	addq	%r12, %r15

	subl	$ 1, %r10d
	addq	$ 4, %r11
	addq	$ 4, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_spack_tt_3_lib8)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_SPACK_TT_2_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_spack_tt_2_lib8)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return


	cmpl	$ 7, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	movq	%r11, %r15

	vmovups		0(%r15), %ymm0
	vmovaps		%ymm0, 0(%r13)
	addq	%r12, %r15

	vmovups		0(%r15), %ymm0
	vmovaps		%ymm0, 32(%r13)
	addq	%r12, %r15

	subl	$ 8, %r10d
	addq	$ 32, %r11
	addq	%r14, %r13

	cmpl	$ 7, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	movq	%r11, %r15

	vmovss		0(%r15), %xmm0
	vmovss		%xmm0, 0(%r13)
	addq	%r12, %r15

	vmovss		0(%r15), %xmm0
	vmovss		%xmm0, 32(%r13)
	addq	%r12, %r15

	subl	$ 1, %r10d
	addq	$ 4, %r11
	addq	$ 4, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_spack_tt_2_lib8)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_SPACK_TT_1_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_spack_tt_1_lib8)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return


	cmpl	$ 7, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	movq	%r11, %r15

	vmovups		0(%r15), %ymm0
	vmovaps		%ymm0, 0(%r13)
	addq	%r12, %r15

	subl	$ 8, %r10d
	addq	$ 32, %r11
	addq	%r14, %r13

	cmpl	$ 7, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	movq	%r11, %r15

	vmovss		0(%r15), %xmm0
	vmovss		%xmm0, 0(%r13)
	addq	%r12, %r15

	subl	$ 1, %r10d
	addq	$ 4, %r11
	addq	$ 4, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_spack_tt_1_lib8)
#endif





//                              1         2          3        4        5
// void kernel_spack_nn_24_lib8(int kmax, float *A, int lda, float *C, int sdc)

	.p2align 4,,15
	GLOB_FUN_START(kernel_spack_nn_24_lib8)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$ 2, %r12d
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
	
	vmovups		0(%r11), %ymm0
	prefetcht0	0(%r11, %r12, 4)
	vmovaps		%ymm0, 0(%r13)
	vmovups		32(%r11), %ymm0
	prefetcht0	32(%r11, %r12, 4)
	vmovaps		%ymm0, 0(%r13, %r14)
	vmovups		64(%r11), %ymm0
	prefetcht0	64(%r11, %r12, 4)
	vmovaps		%ymm0, 0(%r13, %r14, 2)
	addq	%r12, %r11

	vmovups		0(%r11), %ymm0
	prefetcht0	0(%r11, %r12, 4)
	vmovaps		%ymm0, 32(%r13)
	vmovups		32(%r11), %ymm0
	prefetcht0	32(%r11, %r12, 4)
	vmovaps		%ymm0, 32(%r13, %r14)
	vmovups		64(%r11), %ymm0
	prefetcht0	64(%r11, %r12, 4)
	vmovaps		%ymm0, 32(%r13, %r14, 2)
	addq	%r12, %r11

	vmovups		0(%r11), %ymm0
	prefetcht0	0(%r11, %r12, 4)
	vmovaps		%ymm0, 64(%r13)
	vmovups		32(%r11), %ymm0
	prefetcht0	32(%r11, %r12, 4)
	vmovaps		%ymm0, 64(%r13, %r14)
	vmovups		64(%r11), %ymm0
	prefetcht0	64(%r11, %r12, 4)
	vmovaps		%ymm0, 64(%r13, %r14, 2)
	addq	%r12, %r11

	vmovups		0(%r11), %ymm0
	prefetcht0	0(%r11, %r12, 4)
	vmovaps		%ymm0, 96(%r13)
	vmovups		32(%r11), %ymm0
	prefetcht0	32(%r11, %r12, 4)
	vmovaps		%ymm0, 96(%r13, %r14)
	vmovups		64(%r11), %ymm0
	prefetcht0	64(%r11, %r12, 4)
	vmovaps		%ymm0, 96(%r13, %r14, 2)
	addq	%r12, %r11

	subl	$ 4, %r10d
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovups		0(%r11), %ymm0
	vmovaps		%ymm0, 0(%r13)
	vmovups		32(%r11), %ymm0
	vmovaps		%ymm0, 0(%r13, %r14)
	vmovups		64(%r11), %ymm0
	vmovaps		%ymm0, 0(%r13, %r14, 2)
	addq	%r12, %r11

	subl	$ 1, %r10d
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

	EPILOGUE

	ret

	FUN_END(kernel_spack_nn_24_lib8)





//                                 1         2         3        4          5       6
// void kernel_spack_nn_24_vs_lib8(int kmax, float *A, int lda, float *C, int sdb, int m1)

	.p2align 4,,15
	GLOB_FUN_START(kernel_spack_nn_24_vs_lib8)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$ 2, %r12d
	movq	ARG4, %r13 // C
	movq	ARG5, %r14 // sdc
	sall	$ 5, %r14d
	movq	ARG6, %r15 // m1


	cmpl	$ 0, %r10d
	jle		0f // return


	vcvtsi2ss	%r15d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC02(%rip), %ymm14
#elif defined(OS_MAC)
	vmovups		LC02(%rip), %ymm14
#endif
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%ymm15, %ymm14, %ymm15


	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovups		0(%r11), %ymm0
	vmovaps		%ymm0, 0(%r13)
	vmovups		32(%r11), %ymm0
	vmovaps		%ymm0, 0(%r13, %r14)
	vmaskmovps	64(%r11), %ymm15, %ymm0
	vmovaps		%ymm0, 0(%r13, %r14, 2)
	addq	%r12, %r11

	vmovups		0(%r11), %ymm0
	vmovaps		%ymm0, 32(%r13)
	vmovups		32(%r11), %ymm0
	vmovaps		%ymm0, 32(%r13, %r14)
	vmaskmovps	64(%r11), %ymm15, %ymm0
	vmovaps		%ymm0, 32(%r13, %r14, 2)
	addq	%r12, %r11

	vmovups		0(%r11), %ymm0
	vmovaps		%ymm0, 64(%r13)
	vmovups		32(%r11), %ymm0
	vmovaps		%ymm0, 64(%r13, %r14)
	vmaskmovps	64(%r11), %ymm15, %ymm0
	vmovaps		%ymm0, 64(%r13, %r14, 2)
	addq	%r12, %r11

	vmovups		0(%r11), %ymm0
	vmovaps		%ymm0, 96(%r13)
	vmovups		32(%r11), %ymm0
	vmovaps		%ymm0, 96(%r13, %r14)
	vmaskmovps	64(%r11), %ymm15, %ymm0
	vmovaps		%ymm0, 96(%r13, %r14, 2)
	addq	%r12, %r11

	subl	$ 4, %r10d
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovups		0(%r11), %ymm0
	vmovaps		%ymm0, 0(%r13)
	vmovups		32(%r11), %ymm0
	vmovaps		%ymm0, 0(%r13, %r14)
	vmaskmovps	64(%r11), %ymm15, %ymm0
	vmovaps		%ymm0, 0(%r13, %r14, 2)
	addq	%r12, %r11

	subl	$ 1, %r10d
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

	EPILOGUE

	ret

	FUN_END(kernel_spack_nn_24_vs_lib8)





//                              1         2          3        4        5
// void kernel_spack_nn_16_lib8(int kmax, float *A, int lda, float *C, int sdc)

	.p2align 4,,15
	GLOB_FUN_START(kernel_spack_nn_16_lib8)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$ 2, %r12d
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
	
	vmovups		0(%r11), %ymm0
	prefetcht0	0(%r11, %r12, 4)
	vmovaps		%ymm0, 0(%r13)
	vmovups		32(%r11), %ymm0
	prefetcht0	32(%r11, %r12, 4)
	vmovaps		%ymm0, 0(%r13, %r14)
	addq	%r12, %r11

	vmovups		0(%r11), %ymm0
	prefetcht0	0(%r11, %r12, 4)
	vmovaps		%ymm0, 32(%r13)
	vmovups		32(%r11), %ymm0
	prefetcht0	32(%r11, %r12, 4)
	vmovaps		%ymm0, 32(%r13, %r14)
	addq	%r12, %r11

	vmovups		0(%r11), %ymm0
	prefetcht0	0(%r11, %r12, 4)
	vmovaps		%ymm0, 64(%r13)
	vmovups		32(%r11), %ymm0
	prefetcht0	32(%r11, %r12, 4)
	vmovaps		%ymm0, 64(%r13, %r14)
	addq	%r12, %r11

	vmovups		0(%r11), %ymm0
	prefetcht0	0(%r11, %r12, 4)
	vmovaps		%ymm0, 96(%r13)
	vmovups		32(%r11), %ymm0
	prefetcht0	32(%r11, %r12, 4)
	vmovaps		%ymm0, 96(%r13, %r14)
	addq	%r12, %r11

	subl	$ 4, %r10d
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovups		0(%r11), %ymm0
	vmovaps		%ymm0, 0(%r13)
	vmovups		32(%r11), %ymm0
	vmovaps		%ymm0, 0(%r13, %r14)
	addq	%r12, %r11

	subl	$ 1, %r10d
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

	EPILOGUE

	ret

	FUN_END(kernel_spack_nn_16_lib8)





//                                 1         2         3        4          5       6
// void kernel_spack_nn_16_vs_lib8(int kmax, float *A, int lda, float *C, int sdb, int m1)

	.p2align 4,,15
	GLOB_FUN_START(kernel_spack_nn_16_vs_lib8)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$ 2, %r12d
	movq	ARG4, %r13 // C
	movq	ARG5, %r14 // sdc
	sall	$ 5, %r14d
	movq	ARG6, %r15 // m1


	cmpl	$ 0, %r10d
	jle		0f // return


	vcvtsi2ss	%r15d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC01(%rip), %ymm14
#elif defined(OS_MAC)
	vmovups		LC01(%rip), %ymm14
#endif
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%ymm15, %ymm14, %ymm15


	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovups		0(%r11), %ymm0
	vmovaps		%ymm0, 0(%r13)
	vmaskmovps	32(%r11), %ymm15, %ymm0
	vmovaps		%ymm0, 0(%r13, %r14)
	addq	%r12, %r11

	vmovups		0(%r11), %ymm0
	vmovaps		%ymm0, 32(%r13)
	vmaskmovps	32(%r11), %ymm15, %ymm0
	vmovaps		%ymm0, 32(%r13, %r14)
	addq	%r12, %r11

	vmovups		0(%r11), %ymm0
	vmovaps		%ymm0, 64(%r13)
	vmaskmovps	32(%r11), %ymm15, %ymm0
	vmovaps		%ymm0, 64(%r13, %r14)
	addq	%r12, %r11

	vmovups		0(%r11), %ymm0
	vmovaps		%ymm0, 96(%r13)
	vmaskmovps	32(%r11), %ymm15, %ymm0
	vmovaps		%ymm0, 96(%r13, %r14)
	addq	%r12, %r11

	subl	$ 4, %r10d
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovups		0(%r11), %ymm0
	vmovaps		%ymm0, 0(%r13)
	vmaskmovps	32(%r11), %ymm15, %ymm0
	vmovaps		%ymm0, 0(%r13, %r14)
	addq	%r12, %r11

	subl	$ 1, %r10d
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

	EPILOGUE

	ret

	FUN_END(kernel_spack_nn_16_vs_lib8)





//                             1         2          3        4
// void kernel_spack_nn_8_lib8(int kmax, float *A, int lda, float *C)

	.p2align 4,,15
	GLOB_FUN_START(kernel_spack_nn_8_lib8)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$ 2, %r12d
	movq	ARG4, %r13 // C


	cmpl	$ 0, %r10d
	jle		0f // return


	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovups		0(%r11), %ymm0
	prefetcht0	0(%r11, %r12, 4)
	vmovaps		%ymm0, 0(%r13)
	addq	%r12, %r11

	vmovups		0(%r11), %ymm0
	prefetcht0	0(%r11, %r12, 4)
	vmovaps		%ymm0, 32(%r13)
	addq	%r12, %r11

	vmovups		0(%r11), %ymm0
	prefetcht0	0(%r11, %r12, 4)
	vmovaps		%ymm0, 64(%r13)
	addq	%r12, %r11

	vmovups		0(%r11), %ymm0
	prefetcht0	0(%r11, %r12, 4)
	vmovaps		%ymm0, 96(%r13)
	addq	%r12, %r11

	subl	$ 4, %r10d
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovups		0(%r11), %ymm0
	vmovaps		%ymm0, 0(%r13)
	addq	%r12, %r11

	subl	$ 1, %r10d
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

	EPILOGUE

	ret

	FUN_END(kernel_spack_nn_8_lib8)





//                                1         2          3        4          5
// void kernel_spack_nn_8_vs_lib8(int kmax, float *A, int lda, float *C, int m1)

	.p2align 4,,15
	GLOB_FUN_START(kernel_spack_nn_8_vs_lib8)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$ 2, %r12d
	movq	ARG4, %r13 // C
	movq	ARG5, %r14 // m1


	cmpl	$ 0, %r10d
	jle		0f // return


	vcvtsi2ss	%r14d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %ymm14
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %ymm14
#endif
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%ymm15, %ymm14, %ymm15


	cmpl	$ 3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmaskmovps	0(%r11), %ymm15, %ymm0
	vmovaps		%ymm0, 0(%r13)
	addq	%r12, %r11

	vmaskmovps	0(%r11), %ymm15, %ymm0
	vmovaps		%ymm0, 32(%r13)
	addq	%r12, %r11

	vmaskmovps	0(%r11), %ymm15, %ymm0
	vmovaps		%ymm0, 64(%r13)
	addq	%r12, %r11

	vmaskmovps	0(%r11), %ymm15, %ymm0
	vmovaps		%ymm0, 96(%r13)
	addq	%r12, %r11

	subl	$ 4, %r10d
	addq	$ 128, %r13

	cmpl	$ 3, %r10d
	jg		1b


	cmpl	$ 0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmaskmovps	0(%r11), %ymm15, %ymm0
	vmovaps		%ymm0, 0(%r13)
	addq	%r12, %r11

	subl	$ 1, %r10d
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		2b


0: // return

	EPILOGUE

	ret

	FUN_END(kernel_spack_nn_8_vs_lib8)





//                             1         2          3        4
// void kernel_spack_tn_8_lib8(int kmax, float *A, int lda, float *C)

	.p2align 4,,15
	GLOB_FUN_START(kernel_spack_tn_8_lib8)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$ 2, %r12d
	movq	ARG4, %r13 // C


#if MACRO_LEVEL>=1
	INNER_KERNEL_SPACK_TN_8_LIB8
#else
	CALL(inner_kernel_spack_tn_8_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_spack_tn_8_lib8)





//                                1         2          3        4          5
// void kernel_spack_tn_8_vs_lib8(int kmax, float *A, int lda, float *C, int m1)

	.p2align 4,,15
	GLOB_FUN_START(kernel_spack_tn_8_vs_lib8)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$ 2, %r12d
	movq	ARG4, %r13 // C

	movq	ARG5, %r14 // m1
	cmpl	$ 1, %r14d
	jg		100f

#if MACRO_LEVEL>=1
	INNER_KERNEL_SPACK_TN_1_LIB8
#else
	CALL(inner_kernel_spack_tn_1_lib8)
#endif

	jmp		107f

100:

	movq	ARG5, %r14 // m1
	cmpl	$ 2, %r14d
	jg		101f

#if MACRO_LEVEL>=1
	INNER_KERNEL_SPACK_TN_2_LIB8
#else
	CALL(inner_kernel_spack_tn_2_lib8)
#endif
	
	jmp		107f

101:

	movq	ARG5, %r14 // m1
	cmpl	$ 3, %r14d
	jg		102f

#if MACRO_LEVEL>=1
	INNER_KERNEL_SPACK_TN_3_LIB8
#else
	CALL(inner_kernel_spack_tn_3_lib8)
#endif

	jmp		107f

102:

	movq	ARG5, %r14 // m1
	cmpl	$ 4, %r14d
	jg		103f

#if MACRO_LEVEL>=1
	INNER_KERNEL_SPACK_TN_4_LIB8
#else
	CALL(inner_kernel_spack_tn_4_lib8)
#endif

	jmp		107f

103:

	movq	ARG5, %r14 // m1
	cmpl	$ 5, %r14d
	jg		104f

#if MACRO_LEVEL>=1
	INNER_KERNEL_SPACK_TN_5_LIB8
#else
	CALL(inner_kernel_spack_tn_5_lib8)
#endif

	jmp		107f

104:

	movq	ARG5, %r14 // m1
	cmpl	$ 6, %r14d
	jg		105f

#if MACRO_LEVEL>=1
	INNER_KERNEL_SPACK_TN_6_LIB8
#else
	CALL(inner_kernel_spack_tn_6_lib8)
#endif

	jmp		107f

105:

	movq	ARG5, %r14 // m1
	cmpl	$ 7, %r14d
	jg		106f

#if MACRO_LEVEL>=1
	INNER_KERNEL_SPACK_TN_7_LIB8
#else
	CALL(inner_kernel_spack_tn_7_lib8)
#endif

	jmp		107f

106:

#if MACRO_LEVEL>=1
	INNER_KERNEL_SPACK_TN_8_LIB8
#else
	CALL(inner_kernel_spack_tn_8_lib8)
#endif

107:


	EPILOGUE

	ret

	FUN_END(kernel_spack_tn_8_vs_lib8)





//                             1         2          3        4          5
// void kernel_spack_tt_8_lib8(int kmax, float *A, int lda, float *C, int sdc)

	.p2align 4,,15
	GLOB_FUN_START(kernel_spack_tt_8_lib8)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$ 2, %r12d
	movq	ARG4, %r13 // C
	movq	ARG5, %r14 // sdc
	sall	$ 5, %r14d


#if MACRO_LEVEL>=1
	INNER_KERNEL_SPACK_TT_8_LIB8
#else
	CALL(inner_kernel_spack_tt_8_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_spack_tt_8_lib8)





//                                1         2          3        4          5        6
// void kernel_spack_tt_8_vs_lib8(int kmax, float *A, int lda, float *C, int sdc, int m1)

	.p2align 4,,15
	GLOB_FUN_START(kernel_spack_tt_8_vs_lib8)

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$ 2, %r12d
	movq	ARG4, %r13 // C
	movq	ARG5, %r14 // sdc
	sall	$ 5, %r14d

	movq	ARG6, %r15 // m1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=1
	INNER_KERNEL_SPACK_TT_1_LIB8
#else
	CALL(inner_kernel_spack_tt_1_lib8)
#endif

	jmp		107f

100:

	movq	ARG6, %r15 // m1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=1
	INNER_KERNEL_SPACK_TT_2_LIB8
#else
	CALL(inner_kernel_spack_tt_2_lib8)
#endif
	
	jmp		107f

101:

	movq	ARG6, %r15 // m1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=1
	INNER_KERNEL_SPACK_TT_3_LIB8
#else
	CALL(inner_kernel_spack_tt_3_lib8)
#endif

	jmp		107f

102:

	movq	ARG6, %r15 // m1
	cmpl	$ 4, %r15d
	jg		103f

#if MACRO_LEVEL>=1
	INNER_KERNEL_SPACK_TT_4_LIB8
#else
	CALL(inner_kernel_spack_tt_4_lib8)
#endif

	jmp		107f

103:

	movq	ARG6, %r15 // m1
	cmpl	$ 5, %r15d
	jg		104f

#if MACRO_LEVEL>=1
	INNER_KERNEL_SPACK_TT_5_LIB8
#else
	CALL(inner_kernel_spack_tt_5_lib8)
#endif

	jmp		107f

104:

	movq	ARG6, %r15 // m1
	cmpl	$ 6, %r15d
	jg		105f

#if MACRO_LEVEL>=1
	INNER_KERNEL_SPACK_TT_6_LIB8
#else
	CALL(inner_kernel_spack_tt_6_lib8)
#endif

	jmp		107f

105:

	movq	ARG6, %r15 // m1
	cmpl	$ 7, %r15d
	jg		106f

#if MACRO_LEVEL>=1
	INNER_KERNEL_SPACK_TT_7_LIB8
#else
	CALL(inner_kernel_spack_tt_7_lib8)
#endif

	jmp		107f

106:

#if MACRO_LEVEL>=1
	INNER_KERNEL_SPACK_TT_8_LIB8
#else
	CALL(inner_kernel_spack_tt_8_lib8)
#endif

107:


	EPILOGUE

	ret

	FUN_END(kernel_spack_tt_8_vs_lib8)





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
	.float 0.5
	.float 1.5
	.float 2.5
	.float 3.5
	.float 4.5
	.float 5.5
	.float 6.5
	.float 7.5


#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC01:
#elif defined(OS_MAC)
	.align 5
LC01:
#endif
	.float 8.5
	.float 9.5
	.float 10.5
	.float 11.5
	.float 12.5
	.float 13.5
	.float 14.5
	.float 15.5


#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC02:
#elif defined(OS_MAC)
	.align 5
LC02:
#endif
	.float 16.5
	.float 17.5
	.float 18.5
	.float 19.5
	.float 20.5
	.float 21.5
	.float 22.5
	.float 23.5


#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC03:
#elif defined(OS_MAC)
	.align 5
LC03:
#endif
	.float -1.0
	.float -1.0
	.float -1.0
	.float -1.0
	.float -1.0
	.float -1.0
	.float -1.0
	.float -1.0


#if defined(OS_LINUX)
	.section	.note.GNU-stack,"",@progbits
#elif defined(OS_MAC)
	.subsections_via_symbols
#endif

