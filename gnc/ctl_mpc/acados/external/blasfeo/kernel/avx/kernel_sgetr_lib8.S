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


// void INNER_KERNEL_SGETR_8_LIB8
//
// input arguments:
// r10d   <- k
// r11    <- A
// r12    <- 8*sda*sizeof(float)
// r13    <- B

#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_SGETR_8_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_sgetr_8_lib8, @function
inner_kernel_sgetr_8_lib8:
#elif defined(OS_MAC)
_inner_kernel_sgetr_8_lib8:
#elif defined(OS_WINDOWS)
	.def inner_kernel_sgetr_8_lib8; .scl 2; .type 32; .endef
inner_kernel_sgetr_8_lib8:
#endif
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	cmpl	$ 7, %r10d
	jle		0f // consider clean-up

	// main loop
	.p2align 3
1: // main loop

	vmovaps		0(%r11), %ymm0
	vmovaps		32(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm8
	vunpckhps	%ymm1, %ymm0, %ymm9
	vmovaps		64(%r11), %ymm0
	vmovaps		96(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm10
	vunpckhps	%ymm1, %ymm0, %ymm11
	vmovaps		128(%r11), %ymm0
	vmovaps		160(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm12
	vunpckhps	%ymm1, %ymm0, %ymm13
	vmovaps		192(%r11), %ymm0
	vmovaps		224(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm14
	vunpckhps	%ymm1, %ymm0, %ymm15
	subl		$ 8, %r10d
	addq		%r12, %r11

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

	addq		$ 256, %r13

	cmpl		$ 7, %r10d
	jg			1b // main loop

0: // consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

	// common
	vmovaps		0(%r11), %ymm0
	vmovaps		32(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm8
	vunpckhps	%ymm1, %ymm0, %ymm9
	vmovaps		64(%r11), %ymm0
	vmovaps		96(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm10
	vunpckhps	%ymm1, %ymm0, %ymm11
	vmovaps		128(%r11), %ymm0
	vmovaps		160(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm12
	vunpckhps	%ymm1, %ymm0, %ymm13
	vmovaps		192(%r11), %ymm0
	vmovaps		224(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm14
	vunpckhps	%ymm1, %ymm0, %ymm15
	vshufps		$ 0x44, %ymm10, %ymm8, %ymm0
	vshufps		$ 0x44, %ymm14, %ymm12, %ymm1
	vshufps		$ 0xee, %ymm10, %ymm8, %ymm2
	vshufps		$ 0xee, %ymm14, %ymm12, %ymm3
	vshufps		$ 0x44, %ymm11, %ymm9, %ymm4
	vshufps		$ 0x44, %ymm15, %ymm13, %ymm5
	vshufps		$ 0xee, %ymm11, %ymm9, %ymm6
	vshufps		$ 0xee, %ymm15, %ymm13, %ymm7

	// 0
	vperm2f128	$ 0x20, %ymm1, %ymm0, %ymm8
	vmovaps		%ymm8, 0(%r13)
	cmpl	$ 1, %r10d
	jle		3f
	// 1
	vperm2f128	$ 0x20, %ymm3, %ymm2, %ymm8
	vmovaps		%ymm8, 32(%r13)
	cmpl	$ 2, %r10d
	jle		3f
	// 2
	vperm2f128	$ 0x20, %ymm5, %ymm4, %ymm8
	vmovaps		%ymm8, 64(%r13)
	cmpl	$ 3, %r10d
	jle		3f
	// 3
	vperm2f128	$ 0x20, %ymm7, %ymm6, %ymm8
	vmovaps		%ymm8, 96(%r13)
	cmpl	$ 4, %r10d
	jle		3f
	// 4
	vperm2f128	$ 0x31, %ymm1, %ymm0, %ymm8
	vmovaps		%ymm8, 128(%r13)
	cmpl	$ 5, %r10d
	jle		3f
	// 5
	vperm2f128	$ 0x31, %ymm3, %ymm2, %ymm8
	vmovaps		%ymm8, 160(%r13)
	cmpl	$ 6, %r10d
	jle		3f
	// 6
	vperm2f128	$ 0x31, %ymm5, %ymm4, %ymm8
	vmovaps		%ymm8, 192(%r13)
//	cmpl	$ 7, %r10d
//	jle		3f
	// 7
//	vperm2f128	$ 0x31, %ymm7, %ymm6, %ymm8
//	vmovaps		%ymm8, 224(%r13)

3:
	movl	%r10d, %r14d
	sall	$ 2, %r14d // kleft*sizeof(float)
	addq	%r14, %r11 // A+kleft
	movl	%r10d, %r14d
	sall	$ 5, %r14d // kleft*bs*sizeof(float)
	addq	%r14, %r13
	movl	$ 0, %r10d

2: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_sgetr_8_lib8, .-inner_kernel_sgetr_8_lib8
#endif
#endif
// end

// void INNER_KERNEL_SGETR_8_GEN_LIB8
//
// input arguments:
// r10d   <- k
// r11    <- A
// r12    <- 8*sda*sizeof(float)
// r13    <- B
// r14d   <- m1

#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_SGETR_8_GEN_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_sgetr_8_gen_lib8, @function
inner_kernel_sgetr_8_gen_lib8:
#elif defined(OS_MAC)
_inner_kernel_sgetr_8_gen_lib8:
#elif defined(OS_WINDOWS)
	.def inner_kernel_sgetr_8_gen_lib8; .scl 2; .type 32; .endef
inner_kernel_sgetr_8_gen_lib8:
#endif
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	cmpl	$ 7, %r10d
	jle		0f // consider clean-up

	// main loop
	.p2align 3
1: // main loop

	vmovaps		0(%r11), %ymm0
	vmovaps		32(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm8
	vunpckhps	%ymm1, %ymm0, %ymm9
	vmovaps		64(%r11), %ymm0
	vmovaps		96(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm10
	vunpckhps	%ymm1, %ymm0, %ymm11
	vmovaps		128(%r11), %ymm0
	vmovaps		160(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm12
	vunpckhps	%ymm1, %ymm0, %ymm13
	vmovaps		192(%r11), %ymm0
	vmovaps		224(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm14
	vunpckhps	%ymm1, %ymm0, %ymm15
	subl		$ 8, %r10d
	addq		%r12, %r11

	vmovupd		-32(%rsp), %ymm4

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

	addq		$ 256, %r13

	cmpl		$ 7, %r10d
	jg			1b // main loop

0: // consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

	// common
	vmovaps		0(%r11), %ymm0
	vmovaps		32(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm8
	vunpckhps	%ymm1, %ymm0, %ymm9
	vmovaps		64(%r11), %ymm0
	vmovaps		96(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm10
	vunpckhps	%ymm1, %ymm0, %ymm11
	vmovaps		128(%r11), %ymm0
	vmovaps		160(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm12
	vunpckhps	%ymm1, %ymm0, %ymm13
	vmovaps		192(%r11), %ymm0
	vmovaps		224(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm14
	vunpckhps	%ymm1, %ymm0, %ymm15
	vshufps		$ 0x44, %ymm10, %ymm8, %ymm0
	vshufps		$ 0x44, %ymm14, %ymm12, %ymm1
	vshufps		$ 0xee, %ymm10, %ymm8, %ymm2
	vshufps		$ 0xee, %ymm14, %ymm12, %ymm3
	vshufps		$ 0x44, %ymm11, %ymm9, %ymm4
	vshufps		$ 0x44, %ymm15, %ymm13, %ymm5
	vshufps		$ 0xee, %ymm11, %ymm9, %ymm6
	vshufps		$ 0xee, %ymm15, %ymm13, %ymm7

	vmovupd		-32(%rsp), %ymm9

	// 0
	vperm2f128	$ 0x20, %ymm1, %ymm0, %ymm8
	vmaskmovps	%ymm8, %ymm9, 0(%r13)
	cmpl	$ 1, %r10d
	jle		3f
	// 1
	vperm2f128	$ 0x20, %ymm3, %ymm2, %ymm8
	vmaskmovps	%ymm8, %ymm9, 32(%r13)
	cmpl	$ 2, %r10d
	jle		3f
	// 2
	vperm2f128	$ 0x20, %ymm5, %ymm4, %ymm8
	vmaskmovps	%ymm8, %ymm9, 64(%r13)
	cmpl	$ 3, %r10d
	jle		3f
	// 3
	vperm2f128	$ 0x20, %ymm7, %ymm6, %ymm8
	vmaskmovps	%ymm8, %ymm9, 96(%r13)
	cmpl	$ 4, %r10d
	jle		3f
	// 4
	vperm2f128	$ 0x31, %ymm1, %ymm0, %ymm8
	vmaskmovps	%ymm8, %ymm9, 128(%r13)
	cmpl	$ 5, %r10d
	jle		3f
	// 5
	vperm2f128	$ 0x31, %ymm3, %ymm2, %ymm8
	vmaskmovps	%ymm8, %ymm9, 160(%r13)
	cmpl	$ 6, %r10d
	jle		3f
	// 6
	vperm2f128	$ 0x31, %ymm5, %ymm4, %ymm8
	vmaskmovps	%ymm8, %ymm9, 192(%r13)
//	cmpl	$ 7, %r10d
//	jle		3f
	// 7
//	vperm2f128	$ 0x31, %ymm7, %ymm6, %ymm8
//	vmaskmovps	%ymm8, %ymm9, 224(%r13)

3:
	movl	%r10d, %r14d
	sall	$ 2, %r14d // kleft*sizeof(float)
	addq	%r14, %r11 // A+kleft
	movl	%r10d, %r14d
	sall	$ 5, %r14d // kleft*bs*sizeof(float)
	addq	%r14, %r13
	movl	$ 0, %r10d

2: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_sgetr_8_gen_lib8, .-inner_kernel_sgetr_8_gen_lib8
#endif
#endif

// end


// void KERNEL_INNER_EDGE_SGETR_8_0_GEN_LIB8
//
// input arguments:
// r10d   <- k
// r11    <- A
// r12    <- 8*sda*sizeof(float)
// r13    <- B
// r14d   <- m1

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_SGETR_8_0_GEN_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_edge_sgetr_8_0_gen_lib8, @function
inner_edge_sgetr_8_0_gen_lib8:
#elif defined(OS_MAC)
_inner_edge_sgetr_8_0_gen_lib8:
#elif defined(OS_WINDOWS)
	.def inner_edge_sgetr_8_0_gen_lib8; .scl 2; .type 32; .endef
inner_edge_sgetr_8_0_gen_lib8:
#endif
#endif

	// compute mask for rows
	vcvtsi2ss	%r14d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %ymm12
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %ymm12
#endif
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%ymm15, %ymm12, %ymm15
	vmovupd		%ymm15, -32(%rsp) // spill mask to stack

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_edge_sgetr_8_0_gen_lib8, .-inner_edge_sgetr_8_0_gen_lib8
#endif
#endif





// end

// void KERNEL_INNER_EDGE_SGETR_8_1_GEN_LIB8
//
// input arguments:
// r10d   <- k
// r11    <- A
// r12    <- 8*sda*sizeof(float)
// r13    <- B

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_SGETR_8_1_GEN_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_edge_sgetr_8_1_gen_lib8, @function
inner_edge_sgetr_8_1_gen_lib8:
#elif defined(OS_MAC)
_inner_edge_sgetr_8_1_gen_lib8:
#elif defined(OS_WINDOWS)
	.def inner_edge_sgetr_8_1_gen_lib8; .scl 2; .type 32; .endef
inner_edge_sgetr_8_1_gen_lib8:
#endif
#endif

	// compute mask for rows
	vcvtsi2ss	%r14d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %ymm12
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %ymm12
#endif
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%ymm15, %ymm12, %ymm15
	vmovupd		%ymm15, -32(%rsp) // spill mask to stack

	cmpl	$ 0, %r10d
	jle		2f // return

	// common
	vmovaps		0(%r11), %ymm0
	vmovaps		32(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm8
	vunpckhps	%ymm1, %ymm0, %ymm9
	vmovaps		64(%r11), %ymm0
	vmovaps		96(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm10
	vunpckhps	%ymm1, %ymm0, %ymm11
	vmovaps		128(%r11), %ymm0
	vmovaps		160(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm12
	vunpckhps	%ymm1, %ymm0, %ymm13
	vmovaps		192(%r11), %ymm0
	vmovaps		224(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm14
	vunpckhps	%ymm1, %ymm0, %ymm15
	vshufps		$ 0x44, %ymm10, %ymm8, %ymm0
	vshufps		$ 0x44, %ymm14, %ymm12, %ymm1
	vshufps		$ 0xee, %ymm10, %ymm8, %ymm2
	vshufps		$ 0xee, %ymm14, %ymm12, %ymm3
	vshufps		$ 0x44, %ymm11, %ymm9, %ymm4
	vshufps		$ 0x44, %ymm15, %ymm13, %ymm5
	vshufps		$ 0xee, %ymm11, %ymm9, %ymm6
	vshufps		$ 0xee, %ymm15, %ymm13, %ymm7

	vmovupd		-32(%rsp), %ymm9

	// 0
	// 1
	vperm2f128	$ 0x20, %ymm3, %ymm2, %ymm8
	vmaskmovps	%ymm8, %ymm9, 0(%r13)
	subl	$ 1, %r10d
	cmpl	$ 0, %r10d
	jle		3f
	// 2
	vperm2f128	$ 0x20, %ymm5, %ymm4, %ymm8
	vmaskmovps	%ymm8, %ymm9, 32(%r13)
	subl	$ 1, %r10d
	cmpl	$ 0, %r10d
	jle		3f
	// 3
	vperm2f128	$ 0x20, %ymm7, %ymm6, %ymm8
	vmaskmovps	%ymm8, %ymm9, 64(%r13)
	subl	$ 1, %r10d
	cmpl	$ 0, %r10d
	jle		3f
	// 4
	vperm2f128	$ 0x31, %ymm1, %ymm0, %ymm8
	vmaskmovps	%ymm8, %ymm9, 96(%r13)
	subl	$ 1, %r10d
	cmpl	$ 0, %r10d
	jle		3f
	// 5
	vperm2f128	$ 0x31, %ymm3, %ymm2, %ymm8
	vmaskmovps	%ymm8, %ymm9, 128(%r13)
	subl	$ 1, %r10d
	cmpl	$ 0, %r10d
	jle		3f
	// 6
	vperm2f128	$ 0x31, %ymm5, %ymm4, %ymm8
	vmaskmovps	%ymm8, %ymm9, 160(%r13)
	subl	$ 1, %r10d
	cmpl	$ 0, %r10d
	jle		3f
	// 7
	vperm2f128	$ 0x31, %ymm7, %ymm6, %ymm8
	vmaskmovps	%ymm8, %ymm9, 192(%r13)
	subl	$ 1, %r10d

	addq	%r12, %r11 // A+bs*sda*sizeof(float)
	addq	$ 224, %r13 // B+7*bs*sizeof(float)

	jmp		2f

3:
	movl	%r10d, %r14d
	sall	$ 2, %r14d
	addq	%r14, %r11 // A+k*sizeof(float)
	movl	%r10d, %r14d
	sall	$ 5, %r14d
	addq	%r14, %r13 // B+k*bs*sizeof(float)

2: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_edge_sgetr_8_1_gen_lib8, .-inner_edge_sgetr_8_1_gen_lib8
#endif
#endif





// end

// void KERNEL_INNER_EDGE_SGETR_8_2_GEN_LIB8
//
// input arguments:
// r10d   <- k
// r11    <- A
// r12    <- 8*sda*sizeof(float)
// r13    <- B

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_SGETR_8_2_GEN_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_edge_sgetr_8_2_gen_lib8, @function
inner_edge_sgetr_8_2_gen_lib8:
#elif defined(OS_MAC)
_inner_edge_sgetr_8_2_gen_lib8:
#elif defined(OS_WINDOWS)
	.def inner_edge_sgetr_8_2_gen_lib8; .scl 2; .type 32; .endef
inner_edge_sgetr_8_2_gen_lib8:
#endif
#endif

	// compute mask for rows
	vcvtsi2ss	%r14d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %ymm12
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %ymm12
#endif
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%ymm15, %ymm12, %ymm15
	vmovupd		%ymm15, -32(%rsp) // spill mask to stack

	cmpl	$ 0, %r10d
	jle		2f // return

	// common
	vmovaps		0(%r11), %ymm0
	vmovaps		32(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm8
	vunpckhps	%ymm1, %ymm0, %ymm9
	vmovaps		64(%r11), %ymm0
	vmovaps		96(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm10
	vunpckhps	%ymm1, %ymm0, %ymm11
	vmovaps		128(%r11), %ymm0
	vmovaps		160(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm12
	vunpckhps	%ymm1, %ymm0, %ymm13
	vmovaps		192(%r11), %ymm0
	vmovaps		224(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm14
	vunpckhps	%ymm1, %ymm0, %ymm15
	vshufps		$ 0x44, %ymm10, %ymm8, %ymm0
	vshufps		$ 0x44, %ymm14, %ymm12, %ymm1
	vshufps		$ 0xee, %ymm10, %ymm8, %ymm2
	vshufps		$ 0xee, %ymm14, %ymm12, %ymm3
	vshufps		$ 0x44, %ymm11, %ymm9, %ymm4
	vshufps		$ 0x44, %ymm15, %ymm13, %ymm5
	vshufps		$ 0xee, %ymm11, %ymm9, %ymm6
	vshufps		$ 0xee, %ymm15, %ymm13, %ymm7

	vmovupd		-32(%rsp), %ymm9

	// 0
	// 1
	// 2
	vperm2f128	$ 0x20, %ymm5, %ymm4, %ymm8
	vmaskmovps	%ymm8, %ymm9, 0(%r13)
	subl	$ 1, %r10d
	cmpl	$ 0, %r10d
	jle		3f
	// 3
	vperm2f128	$ 0x20, %ymm7, %ymm6, %ymm8
	vmaskmovps	%ymm8, %ymm9, 32(%r13)
	subl	$ 1, %r10d
	cmpl	$ 0, %r10d
	jle		3f
	// 4
	vperm2f128	$ 0x31, %ymm1, %ymm0, %ymm8
	vmaskmovps	%ymm8, %ymm9, 64(%r13)
	subl	$ 1, %r10d
	cmpl	$ 0, %r10d
	jle		3f
	// 5
	vperm2f128	$ 0x31, %ymm3, %ymm2, %ymm8
	vmaskmovps	%ymm8, %ymm9, 96(%r13)
	subl	$ 1, %r10d
	cmpl	$ 0, %r10d
	jle		3f
	// 6
	vperm2f128	$ 0x31, %ymm5, %ymm4, %ymm8
	vmaskmovps	%ymm8, %ymm9, 128(%r13)
	subl	$ 1, %r10d
	cmpl	$ 0, %r10d
	jle		3f
	// 7
	vperm2f128	$ 0x31, %ymm7, %ymm6, %ymm8
	vmaskmovps	%ymm8, %ymm9, 160(%r13)
	subl	$ 1, %r10d

	addq	%r12, %r11 // A+bs*sda*sizeof(float)
	addq	$ 192, %r13 // B+6*bs*sizeof(float)

	jmp		2f

3:
	movl	%r10d, %r14d
	sall	$ 2, %r14d
	addq	%r14, %r11 // A+k*sizeof(float)
	movl	%r10d, %r14d
	sall	$ 5, %r14d
	addq	%r14, %r13 // B+k*bs*sizeof(float)

2: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_edge_sgetr_8_2_gen_lib8, .-inner_edge_sgetr_8_2_gen_lib8
#endif
#endif





// end

// void KERNEL_INNER_EDGE_SGETR_8_3_GEN_LIB8
//
// input arguments:
// r10d   <- k
// r11    <- A
// r12    <- 8*sda*sizeof(float)
// r13    <- B

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_SGETR_8_3_GEN_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_edge_sgetr_8_3_gen_lib8, @function
inner_edge_sgetr_8_3_gen_lib8:
#elif defined(OS_MAC)
_inner_edge_sgetr_8_3_gen_lib8:
#elif defined(OS_WINDOWS)
	.def inner_edge_sgetr_8_3_gen_lib8; .scl 2; .type 32; .endef
inner_edge_sgetr_8_3_gen_lib8:
#endif
#endif

	// compute mask for rows
	vcvtsi2ss	%r14d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %ymm12
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %ymm12
#endif
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%ymm15, %ymm12, %ymm15
	vmovupd		%ymm15, -32(%rsp) // spill mask to stack

	cmpl	$ 0, %r10d
	jle		2f // return

	// common
	vmovaps		0(%r11), %ymm0
	vmovaps		32(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm8
	vunpckhps	%ymm1, %ymm0, %ymm9
	vmovaps		64(%r11), %ymm0
	vmovaps		96(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm10
	vunpckhps	%ymm1, %ymm0, %ymm11
	vmovaps		128(%r11), %ymm0
	vmovaps		160(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm12
	vunpckhps	%ymm1, %ymm0, %ymm13
	vmovaps		192(%r11), %ymm0
	vmovaps		224(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm14
	vunpckhps	%ymm1, %ymm0, %ymm15
	vshufps		$ 0x44, %ymm10, %ymm8, %ymm0
	vshufps		$ 0x44, %ymm14, %ymm12, %ymm1
	vshufps		$ 0xee, %ymm10, %ymm8, %ymm2
	vshufps		$ 0xee, %ymm14, %ymm12, %ymm3
	vshufps		$ 0x44, %ymm11, %ymm9, %ymm4
	vshufps		$ 0x44, %ymm15, %ymm13, %ymm5
	vshufps		$ 0xee, %ymm11, %ymm9, %ymm6
	vshufps		$ 0xee, %ymm15, %ymm13, %ymm7

	vmovupd		-32(%rsp), %ymm9

	// 0
	// 1
	// 2
	// 3
	vperm2f128	$ 0x20, %ymm7, %ymm6, %ymm8
	vmaskmovps	%ymm8, %ymm9, 0(%r13)
	subl	$ 1, %r10d
	cmpl	$ 0, %r10d
	jle		3f
	// 4
	vperm2f128	$ 0x31, %ymm1, %ymm0, %ymm8
	vmaskmovps	%ymm8, %ymm9, 32(%r13)
	subl	$ 1, %r10d
	cmpl	$ 0, %r10d
	jle		3f
	// 5
	vperm2f128	$ 0x31, %ymm3, %ymm2, %ymm8
	vmaskmovps	%ymm8, %ymm9, 64(%r13)
	subl	$ 1, %r10d
	cmpl	$ 0, %r10d
	jle		3f
	// 6
	vperm2f128	$ 0x31, %ymm5, %ymm4, %ymm8
	vmaskmovps	%ymm8, %ymm9, 96(%r13)
	subl	$ 1, %r10d
	cmpl	$ 0, %r10d
	jle		3f
	// 7
	vperm2f128	$ 0x31, %ymm7, %ymm6, %ymm8
	vmaskmovps	%ymm8, %ymm9, 128(%r13)
	subl	$ 1, %r10d

	addq	%r12, %r11 // A+bs*sda*sizeof(float)
	addq	$ 160, %r13 // B+6*bs*sizeof(float)

	jmp		2f

3:
	movl	%r10d, %r14d
	sall	$ 2, %r14d
	addq	%r14, %r11 // A+k*sizeof(float)
	movl	%r10d, %r14d
	sall	$ 5, %r14d
	addq	%r14, %r13 // B+k*bs*sizeof(float)

2: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_edge_sgetr_8_3_gen_lib8, .-inner_edge_sgetr_8_3_gen_lib8
#endif
#endif





// end

// void KERNEL_INNER_EDGE_SGETR_8_4_GEN_LIB8
//
// input arguments:
// r10d   <- k
// r11    <- A
// r12    <- 8*sda*sizeof(float)
// r13    <- B

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_SGETR_8_4_GEN_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_edge_sgetr_8_4_gen_lib8, @function
inner_edge_sgetr_8_4_gen_lib8:
#elif defined(OS_MAC)
_inner_edge_sgetr_8_4_gen_lib8:
#elif defined(OS_WINDOWS)
	.def inner_edge_sgetr_8_4_gen_lib8; .scl 2; .type 32; .endef
inner_edge_sgetr_8_4_gen_lib8:
#endif
#endif

	// compute mask for rows
	vcvtsi2ss	%r14d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %ymm12
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %ymm12
#endif
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%ymm15, %ymm12, %ymm15
	vmovupd		%ymm15, -32(%rsp) // spill mask to stack

	cmpl	$ 0, %r10d
	jle		2f // return

	// common
	vmovaps		0(%r11), %ymm0
	vmovaps		32(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm8
	vunpckhps	%ymm1, %ymm0, %ymm9
	vmovaps		64(%r11), %ymm0
	vmovaps		96(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm10
	vunpckhps	%ymm1, %ymm0, %ymm11
	vmovaps		128(%r11), %ymm0
	vmovaps		160(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm12
	vunpckhps	%ymm1, %ymm0, %ymm13
	vmovaps		192(%r11), %ymm0
	vmovaps		224(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm14
	vunpckhps	%ymm1, %ymm0, %ymm15
	vshufps		$ 0x44, %ymm10, %ymm8, %ymm0
	vshufps		$ 0x44, %ymm14, %ymm12, %ymm1
	vshufps		$ 0xee, %ymm10, %ymm8, %ymm2
	vshufps		$ 0xee, %ymm14, %ymm12, %ymm3
	vshufps		$ 0x44, %ymm11, %ymm9, %ymm4
	vshufps		$ 0x44, %ymm15, %ymm13, %ymm5
	vshufps		$ 0xee, %ymm11, %ymm9, %ymm6
	vshufps		$ 0xee, %ymm15, %ymm13, %ymm7

	vmovupd		-32(%rsp), %ymm9

	// 0
	// 1
	// 2
	// 3
	// 4
	vperm2f128	$ 0x31, %ymm1, %ymm0, %ymm8
	vmaskmovps	%ymm8, %ymm9, 0(%r13)
	subl	$ 1, %r10d
	cmpl	$ 0, %r10d
	jle		3f
	// 5
	vperm2f128	$ 0x31, %ymm3, %ymm2, %ymm8
	vmaskmovps	%ymm8, %ymm9, 32(%r13)
	subl	$ 1, %r10d
	cmpl	$ 0, %r10d
	jle		3f
	// 6
	vperm2f128	$ 0x31, %ymm5, %ymm4, %ymm8
	vmaskmovps	%ymm8, %ymm9, 64(%r13)
	subl	$ 1, %r10d
	cmpl	$ 0, %r10d
	jle		3f
	// 7
	vperm2f128	$ 0x31, %ymm7, %ymm6, %ymm8
	vmaskmovps	%ymm8, %ymm9, 96(%r13)
	subl	$ 1, %r10d

	addq	%r12, %r11 // A+bs*sda*sizeof(float)
	addq	$ 128, %r13 // B+6*bs*sizeof(float)

	jmp		2f

3:
	movl	%r10d, %r14d
	sall	$ 2, %r14d
	addq	%r14, %r11 // A+k*sizeof(float)
	movl	%r10d, %r14d
	sall	$ 5, %r14d
	addq	%r14, %r13 // B+k*bs*sizeof(float)

2: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_edge_sgetr_8_4_gen_lib8, .-inner_edge_sgetr_8_4_gen_lib8
#endif
#endif





// end

// void KERNEL_INNER_EDGE_SGETR_8_5_GEN_LIB8
//
// input arguments:
// r10d   <- k
// r11    <- A
// r12    <- 8*sda*sizeof(float)
// r13    <- B

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_SGETR_8_5_GEN_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_edge_sgetr_8_5_gen_lib8, @function
inner_edge_sgetr_8_5_gen_lib8:
#elif defined(OS_MAC)
_inner_edge_sgetr_8_5_gen_lib8:
#elif defined(OS_WINDOWS)
	.def inner_edge_sgetr_8_5_gen_lib8; .scl 2; .type 32; .endef
inner_edge_sgetr_8_5_gen_lib8:
#endif
#endif

	// compute mask for rows
	vcvtsi2ss	%r14d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %ymm12
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %ymm12
#endif
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%ymm15, %ymm12, %ymm15
	vmovupd		%ymm15, -32(%rsp) // spill mask to stack

	cmpl	$ 0, %r10d
	jle		2f // return

	// common
	vmovaps		0(%r11), %ymm0
	vmovaps		32(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm8
	vunpckhps	%ymm1, %ymm0, %ymm9
	vmovaps		64(%r11), %ymm0
	vmovaps		96(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm10
	vunpckhps	%ymm1, %ymm0, %ymm11
	vmovaps		128(%r11), %ymm0
	vmovaps		160(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm12
	vunpckhps	%ymm1, %ymm0, %ymm13
	vmovaps		192(%r11), %ymm0
	vmovaps		224(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm14
	vunpckhps	%ymm1, %ymm0, %ymm15
	vshufps		$ 0x44, %ymm10, %ymm8, %ymm0
	vshufps		$ 0x44, %ymm14, %ymm12, %ymm1
	vshufps		$ 0xee, %ymm10, %ymm8, %ymm2
	vshufps		$ 0xee, %ymm14, %ymm12, %ymm3
	vshufps		$ 0x44, %ymm11, %ymm9, %ymm4
	vshufps		$ 0x44, %ymm15, %ymm13, %ymm5
	vshufps		$ 0xee, %ymm11, %ymm9, %ymm6
	vshufps		$ 0xee, %ymm15, %ymm13, %ymm7

	vmovupd		-32(%rsp), %ymm9

	// 0
	// 1
	// 2
	// 3
	// 4
	// 5
	vperm2f128	$ 0x31, %ymm3, %ymm2, %ymm8
	vmaskmovps	%ymm8, %ymm9, 0(%r13)
	subl	$ 1, %r10d
	cmpl	$ 0, %r10d
	jle		3f
	// 6
	vperm2f128	$ 0x31, %ymm5, %ymm4, %ymm8
	vmaskmovps	%ymm8, %ymm9, 32(%r13)
	subl	$ 1, %r10d
	cmpl	$ 0, %r10d
	jle		3f
	// 7
	vperm2f128	$ 0x31, %ymm7, %ymm6, %ymm8
	vmaskmovps	%ymm8, %ymm9, 64(%r13)
	subl	$ 1, %r10d

	addq	%r12, %r11 // A+bs*sda*sizeof(float)
	addq	$ 96, %r13 // B+6*bs*sizeof(float)

	jmp		2f

3:
	movl	%r10d, %r14d
	sall	$ 2, %r14d
	addq	%r14, %r11 // A+k*sizeof(float)
	movl	%r10d, %r14d
	sall	$ 5, %r14d
	addq	%r14, %r13 // B+k*bs*sizeof(float)

2: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_edge_sgetr_8_5_gen_lib8, .-inner_edge_sgetr_8_5_gen_lib8
#endif
#endif





// end

// void KERNEL_INNER_EDGE_SGETR_8_6_GEN_LIB8
//
// input arguments:
// r10d   <- k
// r11    <- A
// r12    <- 8*sda*sizeof(float)
// r13    <- B

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_SGETR_8_6_GEN_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_edge_sgetr_8_6_gen_lib8, @function
inner_edge_sgetr_8_6_gen_lib8:
#elif defined(OS_MAC)
_inner_edge_sgetr_8_6_gen_lib8:
#elif defined(OS_WINDOWS)
	.def inner_edge_sgetr_8_6_gen_lib8; .scl 2; .type 32; .endef
inner_edge_sgetr_8_6_gen_lib8:
#endif
#endif

	// compute mask for rows
	vcvtsi2ss	%r14d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %ymm12
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %ymm12
#endif
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%ymm15, %ymm12, %ymm15
	vmovupd		%ymm15, -32(%rsp) // spill mask to stack

	cmpl	$ 0, %r10d
	jle		2f // return

	// common
	vmovaps		0(%r11), %ymm0
	vmovaps		32(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm8
	vunpckhps	%ymm1, %ymm0, %ymm9
	vmovaps		64(%r11), %ymm0
	vmovaps		96(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm10
	vunpckhps	%ymm1, %ymm0, %ymm11
	vmovaps		128(%r11), %ymm0
	vmovaps		160(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm12
	vunpckhps	%ymm1, %ymm0, %ymm13
	vmovaps		192(%r11), %ymm0
	vmovaps		224(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm14
	vunpckhps	%ymm1, %ymm0, %ymm15
	vshufps		$ 0x44, %ymm10, %ymm8, %ymm0
	vshufps		$ 0x44, %ymm14, %ymm12, %ymm1
	vshufps		$ 0xee, %ymm10, %ymm8, %ymm2
	vshufps		$ 0xee, %ymm14, %ymm12, %ymm3
	vshufps		$ 0x44, %ymm11, %ymm9, %ymm4
	vshufps		$ 0x44, %ymm15, %ymm13, %ymm5
	vshufps		$ 0xee, %ymm11, %ymm9, %ymm6
	vshufps		$ 0xee, %ymm15, %ymm13, %ymm7

	vmovupd		-32(%rsp), %ymm9

	// 0
	// 1
	// 2
	// 3
	// 4
	// 5
	// 6
	vperm2f128	$ 0x31, %ymm5, %ymm4, %ymm8
	vmaskmovps	%ymm8, %ymm9, 0(%r13)
	subl	$ 1, %r10d
	cmpl	$ 0, %r10d
	jle		3f
	// 7
	vperm2f128	$ 0x31, %ymm7, %ymm6, %ymm8
	vmaskmovps	%ymm8, %ymm9, 32(%r13)
	subl	$ 1, %r10d

	addq	%r12, %r11 // A+bs*sda*sizeof(float)
	addq	$ 64, %r13 // B+6*bs*sizeof(float)

	jmp		2f

3:
	movl	%r10d, %r14d
	sall	$ 2, %r14d
	addq	%r14, %r11 // A+k*sizeof(float)
	movl	%r10d, %r14d
	sall	$ 5, %r14d
	addq	%r14, %r13 // B+k*bs*sizeof(float)

2: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_edge_sgetr_8_6_gen_lib8, .-inner_edge_sgetr_8_6_gen_lib8
#endif
#endif





// end

// void KERNEL_INNER_EDGE_SGETR_8_7_GEN_LIB8
//
// input arguments:
// r10d   <- k
// r11    <- A
// r12    <- 8*sda*sizeof(float)
// r13    <- B

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_SGETR_8_7_GEN_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_edge_sgetr_8_7_gen_lib8, @function
inner_edge_sgetr_8_7_gen_lib8:
#elif defined(OS_MAC)
_inner_edge_sgetr_8_7_gen_lib8:
#elif defined(OS_WINDOWS)
	.def inner_edge_sgetr_8_7_gen_lib8; .scl 2; .type 32; .endef
inner_edge_sgetr_8_7_gen_lib8:
#endif
#endif

	// compute mask for rows
	vcvtsi2ss	%r14d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %ymm12
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %ymm12
#endif
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%ymm15, %ymm12, %ymm15
	vmovupd		%ymm15, -32(%rsp) // spill mask to stack

	cmpl	$ 0, %r10d
	jle		2f // return

	// common
	vmovaps		0(%r11), %ymm0
	vmovaps		32(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm8
	vunpckhps	%ymm1, %ymm0, %ymm9
	vmovaps		64(%r11), %ymm0
	vmovaps		96(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm10
	vunpckhps	%ymm1, %ymm0, %ymm11
	vmovaps		128(%r11), %ymm0
	vmovaps		160(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm12
	vunpckhps	%ymm1, %ymm0, %ymm13
	vmovaps		192(%r11), %ymm0
	vmovaps		224(%r11), %ymm1
	vunpcklps	%ymm1, %ymm0, %ymm14
	vunpckhps	%ymm1, %ymm0, %ymm15
	vshufps		$ 0x44, %ymm10, %ymm8, %ymm0
	vshufps		$ 0x44, %ymm14, %ymm12, %ymm1
	vshufps		$ 0xee, %ymm10, %ymm8, %ymm2
	vshufps		$ 0xee, %ymm14, %ymm12, %ymm3
	vshufps		$ 0x44, %ymm11, %ymm9, %ymm4
	vshufps		$ 0x44, %ymm15, %ymm13, %ymm5
	vshufps		$ 0xee, %ymm11, %ymm9, %ymm6
	vshufps		$ 0xee, %ymm15, %ymm13, %ymm7

	vmovupd		-32(%rsp), %ymm9

	// 0
	// 1
	// 2
	// 3
	// 4
	// 5
	// 6
	// 7
	vperm2f128	$ 0x31, %ymm7, %ymm6, %ymm8
	vmaskmovps	%ymm8, %ymm9, 0(%r13)
	subl	$ 1, %r10d

	addq	%r12, %r11 // A+bs*sda*sizeof(float)
	addq	$ 32, %r13 // B+6*bs*sizeof(float)

//	jmp		2f
//
//3:
//	movl	%r10d, %r14d
//	sall	$ 2, %r14d
//	addq	%r14, %r11 // A+k*sizeof(float)
//	movl	%r10d, %r14d
//	sall	$ 5, %r14d
//	addq	%r14, %r13 // B+k*bs*sizeof(float)

2: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_edge_sgetr_8_7_gen_lib8, .-inner_edge_sgetr_8_7_gen_lib8
#endif
#endif

// end



// void kernel_sgetr_8_0_lib8(int k, float *A, int sda, float *B);
//                            rdi    rsi       rdx      rcx

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgetr_8_0_lib8
	.type kernel_sgetr_8_0_lib8, @function
kernel_sgetr_8_0_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgetr_8_0_lib8
_kernel_sgetr_8_0_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgetr_8_0_lib8
	.def kernel_sgetr_8_0_lib8; .scl 2; .type 32; .endef
kernel_sgetr_8_0_lib8:
#endif

	PROLOGUE

	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG4, %r13  // B

	// offsetA==0: no edge

#if MACRO_LEVEL>=1
	INNER_KERNEL_SGETR_8_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_sgetr_8_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_sgetr_8_lib8
#endif
#endif

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgetr_8_0_lib8, .-kernel_sgetr_8_0_lib8
#endif

// end

// void kernel_sgetr_8_0_gen_lib8(int k, float *A, int sda, float *B, int m1);
//                                rdi    rsi       rdx      rcx       r8

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgetr_8_0_gen_lib8
	.type kernel_sgetr_8_0_gen_lib8, @function
kernel_sgetr_8_0_gen_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgetr_8_0_gen_lib8
_kernel_sgetr_8_0_gen_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgetr_8_0_gen_lib8
	.def kernel_sgetr_8_0_gen_lib8; .scl 2; .type 32; .endef
kernel_sgetr_8_0_gen_lib8:
#endif

	PROLOGUE

	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG4, %r13  // B
	movq	ARG5, %r14  // m1

	// offsetA==0: edge to compute mask

#if MACRO_LEVEL>=1
	INNER_EDGE_SGETR_8_0_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_sgetr_8_0_gen_lib8
#elif defined(OS_MAC)
	callq _inner_edge_sgetr_8_0_gen_lib8
#endif
#endif

#if MACRO_LEVEL>=1
	INNER_KERNEL_SGETR_8_GEN_LIB8
#else
#if defined(OS_LINUX)
	call inner_kernel_sgetr_8_gen_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_sgetr_8_gen_lib8
#endif
#endif

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgetr_8_0_gen_lib8, .-kernel_sgetr_8_0_gen_lib8
#endif

// end


// void kernel_sgetr_8_1_lib8(int k, float *A, int sda, float *B);
//                            rdi    rsi       rdx      rcx

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgetr_8_1_lib8
	.type kernel_sgetr_8_1_lib8, @function
kernel_sgetr_8_1_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgetr_8_1_lib8
_kernel_sgetr_8_1_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgetr_8_1_lib8
	.def kernel_sgetr_8_1_lib8; .scl 2; .type 32; .endef
kernel_sgetr_8_1_lib8:
#endif

	PROLOGUE

	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG4, %r13  // B
	movq	$ 8, %r14  // m1

	// offsetA==1

#if MACRO_LEVEL>=1
	INNER_EDGE_SGETR_8_1_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_sgetr_8_1_gen_lib8
#elif defined(OS_MAC)
	callq _inner_edge_sgetr_8_1_gen_lib8
#endif
#endif

#if MACRO_LEVEL>=1
	INNER_KERNEL_SGETR_8_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_sgetr_8_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_sgetr_8_lib8
#endif
#endif

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgetr_8_1_lib8, .-kernel_sgetr_8_1_lib8
#endif


// end

// void kernel_sgetr_8_1_gen_lib8(int k, float *A, int sda, float *B, int m1);
//                                rdi    rsi       rdx      rcx       r8

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgetr_8_1_gen_lib8
	.type kernel_sgetr_8_1_gen_lib8, @function
kernel_sgetr_8_1_gen_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgetr_8_1_gen_lib8
_kernel_sgetr_8_1_gen_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgetr_8_1_gen_lib8
	.def kernel_sgetr_8_1_gen_lib8; .scl 2; .type 32; .endef
kernel_sgetr_8_1_gen_lib8:
#endif

	PROLOGUE

	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG4, %r13  // B
	movq	ARG5, %r14  // m1

	// offsetA==1

#if MACRO_LEVEL>=1
	INNER_EDGE_SGETR_8_1_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_sgetr_8_1_gen_lib8
#elif defined(OS_MAC)
	callq _inner_edge_sgetr_8_1_gen_lib8
#endif
#endif

#if MACRO_LEVEL>=1
	INNER_KERNEL_SGETR_8_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_sgetr_8_gen_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_sgetr_8_gen_lib8
#endif
#endif

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgetr_8_1_gen_lib8, .-kernel_sgetr_8_1_gen_lib8
#endif

// end


// void kernel_sgetr_8_2_lib8(int k, float *A, int sda, float *B);
//                            rdi    rsi       rdx      rcx

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgetr_8_2_lib8
	.type kernel_sgetr_8_2_lib8, @function
kernel_sgetr_8_2_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgetr_8_2_lib8
_kernel_sgetr_8_2_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgetr_8_2_lib8
	.def kernel_sgetr_8_2_lib8; .scl 2; .type 32; .endef
kernel_sgetr_8_2_lib8:
#endif

	PROLOGUE

	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG4, %r13  // B
	movq	$ 8, %r14  // m1

	// offsetA==1

#if MACRO_LEVEL>=1
	INNER_EDGE_SGETR_8_2_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_sgetr_8_2_gen_lib8
#elif defined(OS_MAC)
	callq _inner_edge_sgetr_8_2_gen_lib8
#endif
#endif

#if MACRO_LEVEL>=1
	INNER_KERNEL_SGETR_8_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_sgetr_8_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_sgetr_8_lib8
#endif
#endif

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgetr_8_2_lib8, .-kernel_sgetr_8_2_lib8
#endif

// end

// void kernel_sgetr_8_2_gen_lib8(int k, float *A, int sda, float *B, int m1);
//                                rdi    rsi       rdx      rcx       r8

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgetr_8_2_gen_lib8
	.type kernel_sgetr_8_2_gen_lib8, @function
kernel_sgetr_8_2_gen_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgetr_8_2_gen_lib8
_kernel_sgetr_8_2_gen_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgetr_8_2_gen_lib8
	.def kernel_sgetr_8_2_gen_lib8; .scl 2; .type 32; .endef
kernel_sgetr_8_2_gen_lib8:
#endif

	PROLOGUE

	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG4, %r13  // B
	movq	ARG5, %r14  // m1

	// offsetA==1

#if MACRO_LEVEL>=1
	INNER_EDGE_SGETR_8_2_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_sgetr_8_2_gen_lib8
#elif defined(OS_MAC)
	callq _inner_edge_sgetr_8_2_gen_lib8
#endif
#endif

#if MACRO_LEVEL>=1
	INNER_KERNEL_SGETR_8_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_sgetr_8_gen_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_sgetr_8_gen_lib8
#endif
#endif

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgetr_8_2_gen_lib8, .-kernel_sgetr_8_2_gen_lib8
#endif

// end


// void kernel_sgetr_8_3_lib8(int k, float *A, int sda, float *B);
//                            rdi    rsi       rdx      rcx

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgetr_8_3_lib8
	.type kernel_sgetr_8_3_lib8, @function
kernel_sgetr_8_3_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgetr_8_3_lib8
_kernel_sgetr_8_3_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgetr_8_3_lib8
	.def kernel_sgetr_8_3_lib8; .scl 2; .type 32; .endef
kernel_sgetr_8_3_lib8:
#endif

	PROLOGUE

	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG4, %r13  // B
	movq	$ 8, %r14  // m1

	// offsetA==1

#if MACRO_LEVEL>=1
	INNER_EDGE_SGETR_8_3_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_sgetr_8_3_gen_lib8
#elif defined(OS_MAC)
	callq _inner_edge_sgetr_8_3_gen_lib8
#endif
#endif

#if MACRO_LEVEL>=1
	INNER_KERNEL_SGETR_8_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_sgetr_8_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_sgetr_8_lib8
#endif
#endif

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgetr_8_3_lib8, .-kernel_sgetr_8_3_lib8
#endif

// end

// void kernel_sgetr_8_3_gen_lib8(int k, float *A, int sda, float *B, int m1);
//                                rdi    rsi       rdx      rcx       r8

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgetr_8_3_gen_lib8
	.type kernel_sgetr_8_3_gen_lib8, @function
kernel_sgetr_8_3_gen_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgetr_8_3_gen_lib8
_kernel_sgetr_8_3_gen_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgetr_8_3_gen_lib8
	.def kernel_sgetr_8_3_gen_lib8; .scl 2; .type 32; .endef
kernel_sgetr_8_3_gen_lib8:
#endif

	PROLOGUE

	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG4, %r13  // B
	movq	ARG5, %r14  // m1

	// offsetA==1

#if MACRO_LEVEL>=1
	INNER_EDGE_SGETR_8_3_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_sgetr_8_3_gen_lib8
#elif defined(OS_MAC)
	callq _inner_edge_sgetr_8_3_gen_lib8
#endif
#endif

#if MACRO_LEVEL>=1
	INNER_KERNEL_SGETR_8_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_sgetr_8_gen_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_sgetr_8_gen_lib8
#endif
#endif

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgetr_8_3_gen_lib8, .-kernel_sgetr_8_3_gen_lib8
#endif


// end


// void kernel_sgetr_8_4_lib8(int k, float *A, int sda, float *B);
//                            rdi    rsi       rdx      rcx

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgetr_8_4_lib8
	.type kernel_sgetr_8_4_lib8, @function
kernel_sgetr_8_4_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgetr_8_4_lib8
_kernel_sgetr_8_4_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgetr_8_4_lib8
	.def kernel_sgetr_8_4_lib8; .scl 2; .type 32; .endef
kernel_sgetr_8_4_lib8:
#endif

	PROLOGUE

	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG4, %r13  // B
	movq	$ 8, %r14  // m1

	// offsetA==1

#if MACRO_LEVEL>=1
	INNER_EDGE_SGETR_8_4_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_sgetr_8_4_gen_lib8
#elif defined(OS_MAC)
	callq _inner_edge_sgetr_8_4_gen_lib8
#endif
#endif

#if MACRO_LEVEL>=1
	INNER_KERNEL_SGETR_8_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_sgetr_8_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_sgetr_8_lib8
#endif
#endif

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgetr_8_4_lib8, .-kernel_sgetr_8_4_lib8
#endif


// end

// void kernel_sgetr_8_4_gen_lib8(int k, float *A, int sda, float *B, int m1);
//                                rdi    rsi       rdx      rcx       r8

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgetr_8_4_gen_lib8
	.type kernel_sgetr_8_4_gen_lib8, @function
kernel_sgetr_8_4_gen_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgetr_8_4_gen_lib8
_kernel_sgetr_8_4_gen_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgetr_8_4_gen_lib8
	.def kernel_sgetr_8_4_gen_lib8; .scl 2; .type 32; .endef
kernel_sgetr_8_4_gen_lib8:
#endif

	PROLOGUE

	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG4, %r13  // B
	movq	ARG5, %r14  // m1

	// offsetA==1

#if MACRO_LEVEL>=1
	INNER_EDGE_SGETR_8_4_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_sgetr_8_4_gen_lib8
#elif defined(OS_MAC)
	callq _inner_edge_sgetr_8_4_gen_lib8
#endif
#endif

#if MACRO_LEVEL>=1
	INNER_KERNEL_SGETR_8_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_sgetr_8_gen_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_sgetr_8_gen_lib8
#endif
#endif

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgetr_8_4_gen_lib8, .-kernel_sgetr_8_4_gen_lib8
#endif

// end


// void kernel_sgetr_8_5_lib8(int k, float *A, int sda, float *B);
//                            rdi    rsi       rdx      rcx

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgetr_8_5_lib8
	.type kernel_sgetr_8_5_lib8, @function
kernel_sgetr_8_5_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgetr_8_5_lib8
_kernel_sgetr_8_5_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgetr_8_5_lib8
	.def kernel_sgetr_8_5_lib8; .scl 2; .type 32; .endef
kernel_sgetr_8_5_lib8:
#endif

	PROLOGUE

	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG4, %r13  // B
	movq	$ 8, %r14  // m1

	// offsetA==1

#if MACRO_LEVEL>=1
	INNER_EDGE_SGETR_8_5_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_sgetr_8_5_gen_lib8
#elif defined(OS_MAC)
	callq _inner_edge_sgetr_8_5_gen_lib8
#endif
#endif

#if MACRO_LEVEL>=1
	INNER_KERNEL_SGETR_8_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_sgetr_8_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_sgetr_8_lib8
#endif
#endif

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgetr_8_5_lib8, .-kernel_sgetr_8_5_lib8
#endif

// end

// void kernel_sgetr_8_5_gen_lib8(int k, float *A, int sda, float *B, int m1);
//                                rdi    rsi       rdx      rcx       r8

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgetr_8_5_gen_lib8
	.type kernel_sgetr_8_5_gen_lib8, @function
kernel_sgetr_8_5_gen_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgetr_8_5_gen_lib8
_kernel_sgetr_8_5_gen_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgetr_8_5_gen_lib8
	.def kernel_sgetr_8_5_gen_lib8; .scl 2; .type 32; .endef
kernel_sgetr_8_5_gen_lib8:
#endif

	PROLOGUE

	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG4, %r13  // B
	movq	ARG5, %r14  // m1

	// offsetA==1

#if MACRO_LEVEL>=1
	INNER_EDGE_SGETR_8_5_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_sgetr_8_5_gen_lib8
#elif defined(OS_MAC)
	callq _inner_edge_sgetr_8_5_gen_lib8
#endif
#endif

#if MACRO_LEVEL>=1
	INNER_KERNEL_SGETR_8_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_sgetr_8_gen_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_sgetr_8_gen_lib8
#endif
#endif

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgetr_8_5_gen_lib8, .-kernel_sgetr_8_5_gen_lib8
#endif

// end


// void kernel_sgetr_8_6_lib8(int k, float *A, int sda, float *B);
//                            rdi    rsi       rdx      rcx

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgetr_8_6_lib8
	.type kernel_sgetr_8_6_lib8, @function
kernel_sgetr_8_6_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgetr_8_6_lib8
_kernel_sgetr_8_6_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgetr_8_6_lib8
	.def kernel_sgetr_8_6_lib8; .scl 2; .type 32; .endef
kernel_sgetr_8_6_lib8:
#endif

	PROLOGUE

	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG4, %r13  // B
	movq	$ 8, %r14  // m1

	// offsetA==1

#if MACRO_LEVEL>=1
	INNER_EDGE_SGETR_8_6_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_sgetr_8_6_gen_lib8
#elif defined(OS_MAC)
	callq _inner_edge_sgetr_8_6_gen_lib8
#endif
#endif

#if MACRO_LEVEL>=1
	INNER_KERNEL_SGETR_8_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_sgetr_8_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_sgetr_8_lib8
#endif
#endif

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgetr_8_6_lib8, .-kernel_sgetr_8_6_lib8
#endif


// end

// void kernel_sgetr_8_6_gen_lib8(int k, float *A, int sda, float *B, int m1);
//                                rdi    rsi       rdx      rcx       r8

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgetr_8_6_gen_lib8
	.type kernel_sgetr_8_6_gen_lib8, @function
kernel_sgetr_8_6_gen_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgetr_8_6_gen_lib8
_kernel_sgetr_8_6_gen_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgetr_8_6_gen_lib8
	.def kernel_sgetr_8_6_gen_lib8; .scl 2; .type 32; .endef
kernel_sgetr_8_6_gen_lib8:
#endif

	PROLOGUE

	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG4, %r13  // B
	movq	ARG5, %r14  // m1

	// offsetA==1

#if MACRO_LEVEL>=1
	INNER_EDGE_SGETR_8_6_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_sgetr_8_6_gen_lib8
#elif defined(OS_MAC)
	callq _inner_edge_sgetr_8_6_gen_lib8
#endif
#endif

#if MACRO_LEVEL>=1
	INNER_KERNEL_SGETR_8_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_sgetr_8_gen_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_sgetr_8_gen_lib8
#endif
#endif

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgetr_8_6_gen_lib8, .-kernel_sgetr_8_6_gen_lib8
#endif

// end


// void kernel_sgetr_8_7_lib8(int k, float *A, int sda, float *B);
//                            rdi    rsi       rdx      rcx

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgetr_8_7_lib8
	.type kernel_sgetr_8_7_lib8, @function
kernel_sgetr_8_7_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgetr_8_7_lib8
_kernel_sgetr_8_7_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgetr_8_7_lib8
	.def kernel_sgetr_8_7_lib8; .scl 2; .type 32; .endef
kernel_sgetr_8_7_lib8:
#endif

	PROLOGUE

	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG4, %r13  // B
	movq	$ 8, %r14  // m1

	// offsetA==1

#if MACRO_LEVEL>=1
	INNER_EDGE_SGETR_8_7_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_sgetr_8_7_gen_lib8
#elif defined(OS_MAC)
	callq _inner_edge_sgetr_8_7_gen_lib8
#endif
#endif

#if MACRO_LEVEL>=1
	INNER_KERNEL_SGETR_8_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_sgetr_8_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_sgetr_8_lib8
#endif
#endif

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgetr_8_7_lib8, .-kernel_sgetr_8_7_lib8
#endif

// end

// void kernel_sgetr_8_7_gen_lib8(int k, float *A, int sda, float *B, int m1);
//                                rdi    rsi       rdx      rcx       r8

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgetr_8_7_gen_lib8
	.type kernel_sgetr_8_7_gen_lib8, @function
kernel_sgetr_8_7_gen_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgetr_8_7_gen_lib8
_kernel_sgetr_8_7_gen_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgetr_8_7_gen_lib8
	.def kernel_sgetr_8_7_gen_lib8; .scl 2; .type 32; .endef
kernel_sgetr_8_7_gen_lib8:
#endif

	PROLOGUE

	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG4, %r13  // B
	movq	ARG5, %r14  // m1

	// offsetA==1

#if MACRO_LEVEL>=1
	INNER_EDGE_SGETR_8_7_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_sgetr_8_7_gen_lib8
#elif defined(OS_MAC)
	callq _inner_edge_sgetr_8_7_gen_lib8
#endif
#endif

#if MACRO_LEVEL>=1
	INNER_KERNEL_SGETR_8_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_sgetr_8_gen_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_sgetr_8_gen_lib8
#endif
#endif

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgetr_8_7_gen_lib8, .-kernel_sgetr_8_7_gen_lib8
#endif

// end



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
.LC00: // { 7.5 6.5 5.5 4.5 3.5 2.5 1.5 0.5 }
#elif defined(OS_MAC)
	.align 5
LC00: // { 7.5 6.5 5.5 4.5 3.5 2.5 1.5 0.5 }
#endif
	.long	1056964608
	.long	1069547520
	.long	1075838976
	.long	1080033280
	.long	1083179008
	.long	1085276160
	.long	1087373312
	.long	1089470464

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC01: // { 15.5 14.5 13.5 12.5 11.5 10.5 9.5 8.5 }
#elif defined(OS_MAC)
	.align 5
LC01: // { 15.5 14.5 13.5 12.5 11.5 10.5 9.5 8.5 }
#endif
	.long	1091043328
	.long	1092091904
	.long	1093140480
	.long	1094189056
	.long	1095237632
	.long	1096286208
	.long	1097334784
	.long	1098383360

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC02: // { 23.5 22.5 21.5 20.5 19.5 18.5 17.5 16.5 }
#elif defined(OS_MAC)
	.align 5
LC02: // { 23.5 22.5 21.5 20.5 19.5 18.5 17.5 16.5 }
#endif
	.long	1099169792
	.long	1099694080
	.long	1100218368
	.long	1100742656
	.long	1101266944
	.long	1101791232
	.long	1102315520
	.long	1102839808

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC03: // { -1.0 -1.0 1.0 1.0 1.0 1.0 1.0 1.0 }
#elif defined(OS_MAC)
	.align 5
LC03: // { -1.0 -1.0 1.0 1.0 1.0 1.0 1.0 1.0 }
#endif
	.long	1065353216
	.long	1065353216
	.long	1065353216
	.long	1065353216
	.long	1065353216
	.long	1065353216
	.long	3212836864
	.long	3212836864



#if defined(OS_LINUX)
	.section	.note.GNU-stack,"",@progbits
#elif defined(OS_MAC)
	.subsections_via_symbols
#endif

//EOF
