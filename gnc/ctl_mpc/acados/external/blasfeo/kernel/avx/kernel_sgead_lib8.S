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





// subroutine
//
// input arguments:
// r10d   <- k
// r12    <- A
// r13    <- B

#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_SGEAD_8_0_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_sgead_8_0_lib8, @function
inner_kernel_sgead_8_0_lib8:
#elif defined(OS_MAC)
_inner_kernel_sgead_8_0_lib8:
#elif defined(OS_WINDOWS)
	.def inner_kernel_sgead_8_0_lib8; .scl 2; .type 32; .endef
inner_kernel_sgead_8_0_lib8:
#endif
#endif
	
	vbroadcastss	0(%r11), %ymm14

	cmpl	$ 3, %r10d
	jle		0f // consider clean-up

	// main loop
	.p2align 3
1: // main loop

	vmovaps		0(%r12), %ymm0
	vmovaps		0(%r13), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, 0(%r13)
	subl		$ 4, %r10d

	vmovaps		32(%r12), %ymm0
	vmovaps		32(%r13), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, 32(%r13)
	addq		$ 128, %r12

	vmovaps		-64(%r12), %ymm0
	vmovaps		64(%r13), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, 64(%r13)
	addq		$ 128, %r13

	vmovaps		-32(%r12), %ymm0
	vmovaps		-32(%r13), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, -32(%r13)

	cmpl		$ 3, %r10d
	jg			1b // main loop 

0: // consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

3: // clean-up loop

	vmovaps		0(%r12), %ymm0
	vmovaps		0(%r13), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, 0(%r13)
	subl		$ 1, %r10d
	addq		$ 32, %r12
	addq		$ 32, %r13

	cmpl		$ 0, %r10d
	jg			3b // clean-up loop 

2: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_sgead_8_0_lib8, .-inner_kernel_sgead_8_0_lib8
#endif
#endif





// subroutine
//
// input arguments:
// r10d   <- k
// r12    <- A
// r13    <- B
// r14d   <- m1

#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_SGEAD_8_0_GEN_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_sgead_8_0_gen_lib8, @function
inner_kernel_sgead_8_0_gen_lib8:
#elif defined(OS_MAC)
_inner_kernel_sgead_8_0_gen_lib8:
#elif defined(OS_WINDOWS)
	.def inner_kernel_sgead_8_0_gen_lib8; .scl 2; .type 32; .endef
inner_kernel_sgead_8_0_gen_lib8:
#endif
#endif
	
	vbroadcastss	0(%r11), %ymm14

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

	cmpl	$ 3, %r10d
	jle		0f // consider clean-up

	// main loop
	.p2align 3
1: // main loop

	vmovups		0(%r12), %ymm0
	vmaskmovps	0(%r13), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, 0(%r13)
	subl		$ 4, %r10d

	vmovups		32(%r12), %ymm0
	vmaskmovps	32(%r13), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, 32(%r13)
	addq		$ 128, %r12

	vmovups		-64(%r12), %ymm0
	vmaskmovps	64(%r13), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, 64(%r13)
	addq		$ 128, %r13

	vmovups		-32(%r12), %ymm0
	vmaskmovps	-32(%r13), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, -32(%r13)

	cmpl		$ 3, %r10d
	jg			1b // main loop 

0: // consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

3: // clean-up loop

	vmovups		0(%r12), %ymm0
	vmaskmovps	0(%r13), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, 0(%r13)
	subl		$ 1, %r10d
	addq		$ 32, %r12
	addq		$ 32, %r13

	cmpl		$ 0, %r10d
	jg			3b // clean-up loop 

2: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_sgead_8_0_lib8, .-inner_kernel_sgead_8_0_lib8
#endif
#endif





// subroutine
//
// input arguments:
// r10d   <- k
// r12    <- A
// r13d   <- 8*sda*sizeof(float)
// r14    <- B

#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_SGEAD_8_1_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_sgead_8_1_lib8, @function
inner_kernel_sgead_8_1_lib8:
#elif defined(OS_MAC)
_inner_kernel_sgead_8_1_lib8:
#elif defined(OS_WINDOWS)
	.def inner_kernel_sgead_8_1_lib8; .scl 2; .type 32; .endef
inner_kernel_sgead_8_1_lib8:
#endif
#endif
	
	vbroadcastss	0(%r11), %ymm14

	movq	%r12, %rax // A1 <- A0
	addq	%r13, %rax // A1 <- A0 + 4*sda*sizeof(float)

	cmpl	$ 3, %r10d
	jle		0f // consider clean-up

	// main loop
	.p2align 3
1: // main loop

#if 1
	vmovaps		0(%r12), %ymm0
	vmovaps		0(%rax), %ymm1
	vblendps	$ 0x01, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x39, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x77, %ymm0, %ymm1, %ymm0
	vmovaps		0(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, 0(%r14)
	subl		$ 4, %r10d

	vmovaps		32(%r12), %ymm0
	vmovaps		32(%rax), %ymm1
	vblendps	$ 0x01, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x39, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x77, %ymm0, %ymm1, %ymm0
	vmovaps		32(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, 32(%r14)
	addq		$ 128, %r12
	addq		$ 128, %rax

	vmovaps		-64(%r12), %ymm0
	vmovaps		-64(%rax), %ymm1
	vblendps	$ 0x01, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x39, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x77, %ymm0, %ymm1, %ymm0
	vmovaps		64(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, 64(%r14)
	addq		$ 128, %r14

	vmovaps		-32(%r12), %ymm0
	vmovaps		-32(%rax), %ymm1
	vblendps	$ 0x01, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x39, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x77, %ymm0, %ymm1, %ymm0
	vmovaps		-32(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, -32(%r14)
#else
	vmovups		4(%r12), %ymm0
	vmovups		-28(%rax), %ymm1
	vblendps	$ 0x80, %ymm1, %ymm0, %ymm0
	vmovaps		0(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, 0(%r14)
	subl		$ 4, %r10d

	vmovups		36(%r12), %ymm0
	vmovups		4(%rax), %ymm1
	vblendps	$ 0x80, %ymm1, %ymm0, %ymm0
	vmovaps		32(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, 32(%r14)
	addq		$ 128, %r12
	addq		$ 128, %rax

	vmovups		-60(%r12), %ymm0
	vmovups		-92(%rax), %ymm1
	vblendps	$ 0x80, %ymm1, %ymm0, %ymm0
	vmovaps		64(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, 64(%r14)
	addq		$ 128, %r14

	vmovups		-28(%r12), %ymm0
	vmovups		-60(%rax), %ymm1
	vblendps	$ 0x80, %ymm1, %ymm0, %ymm0
	vmovaps		-32(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, -32(%r14)
#endif

	cmpl		$ 3, %r10d
	jg			1b // main loop 

0: // consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

3: // clean-up loop

	vmovaps		0(%r12), %ymm0
	vmovaps		0(%rax), %ymm1
	vblendps	$ 0x01, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x39, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x77, %ymm0, %ymm1, %ymm0
	vmovaps		0(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, 0(%r14)
	subl		$ 1, %r10d
	addq		$ 32, %r12
	addq		$ 32, %rax
	addq		$ 32, %r14

	cmpl		$ 0, %r10d
	jg			3b // clean-up loop 

2: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_sgead_8_1_lib8, .-inner_kernel_sgead_8_1_lib8
#endif
#endif





// subroutine
//
// input arguments:
// r10d   <- k
// r12    <- A
// r13d   <- 8*sda*sizeof(float)
// r14    <- B
// r15d   <- m1

#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_SGEAD_8_1_GEN_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_sgead_8_1_gen_lib8, @function
inner_kernel_sgead_8_1_gen_lib8:
#elif defined(OS_MAC)
_inner_kernel_sgead_8_1_gen_lib8:
#elif defined(OS_WINDOWS)
	.def inner_kernel_sgead_8_1_gen_lib8; .scl 2; .type 32; .endef
inner_kernel_sgead_8_1_gen_lib8:
#endif
#endif
	
	vbroadcastss	0(%r11), %ymm14

	// compute mask for rows
	vcvtsi2ss	%r15d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %ymm12
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %ymm12
#endif
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%ymm15, %ymm12, %ymm15

	movq	%r12, %rax // A1 <- A0
	addq	%r13, %rax // A1 <- A0 + 4*sda*sizeof(float)

	cmpl	$ 3, %r10d
	jle		0f // consider clean-up

	// main loop
	.p2align 3
1: // main loop

	vmovaps		0(%r12), %ymm0
	vmovaps		0(%rax), %ymm1
	vblendps	$ 0x01, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x39, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x77, %ymm0, %ymm1, %ymm0
	vmaskmovps	0(%r14), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, 0(%r14)
	subl		$ 4, %r10d

	vmovaps		32(%r12), %ymm0
	vmovaps		32(%rax), %ymm1
	vblendps	$ 0x01, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x39, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x77, %ymm0, %ymm1, %ymm0
	vmaskmovps	32(%r14), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, 32(%r14)
	addq		$ 128, %r12
	addq		$ 128, %rax

	vmovaps		-64(%r12), %ymm0
	vmovaps		-64(%rax), %ymm1
	vblendps	$ 0x01, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x39, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x77, %ymm0, %ymm1, %ymm0
	vmaskmovps	64(%r14), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, 64(%r14)
	addq		$ 128, %r14

	vmovaps		-32(%r12), %ymm0
	vmovaps		-32(%rax), %ymm1
	vblendps	$ 0x01, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x39, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x77, %ymm0, %ymm1, %ymm0
	vmaskmovps	-32(%r14), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, -32(%r14)

	cmpl		$ 3, %r10d
	jg			1b // main loop 

0: // consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

3: // clean-up loop

	vmovaps		0(%r12), %ymm0
	vmovaps		0(%rax), %ymm1
	vblendps	$ 0x01, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x39, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x77, %ymm0, %ymm1, %ymm0
	vmaskmovps	0(%r14), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, 0(%r14)
	subl		$ 1, %r10d
	addq		$ 32, %r12
	addq		$ 32, %rax
	addq		$ 32, %r14

	cmpl		$ 0, %r10d
	jg			3b // clean-up loop 

2: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_sgead_8_1_gen_lib8, .-inner_kernel_sgead_8_1_gen_lib8
#endif
#endif





// subroutine
//
// input arguments:
// r10d   <- k
// r12    <- A
// r13d   <- 8*sda*sizeof(float)
// r14    <- B

#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_SGEAD_8_2_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_sgead_8_2_lib8, @function
inner_kernel_sgead_8_2_lib8:
#elif defined(OS_MAC)
_inner_kernel_sgead_8_2_lib8:
#elif defined(OS_WINDOWS)
	.def inner_kernel_sgead_8_2_lib8; .scl 2; .type 32; .endef
inner_kernel_sgead_8_2_lib8:
#endif
#endif
	
	vbroadcastss	0(%r11), %ymm14

	movq	%r12, %rax // A1 <- A0
	addq	%r13, %rax // A1 <- A0 + 4*sda*sizeof(float)

	cmpl	$ 3, %r10d
	jle		0f // consider clean-up

	// main loop
	.p2align 3
1: // main loop

	vmovaps		0(%r12), %ymm0
	vmovaps		0(%rax), %ymm1
	vblendps	$ 0x03, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x4e, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x33, %ymm0, %ymm1, %ymm0
	vmovaps		0(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, 0(%r14)
	subl		$ 4, %r10d

	vmovaps		32(%r12), %ymm0
	vmovaps		32(%rax), %ymm1
	vblendps	$ 0x03, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x4e, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x33, %ymm0, %ymm1, %ymm0
	vmovaps		32(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, 32(%r14)
	addq		$ 128, %r12
	addq		$ 128, %rax

	vmovaps		-64(%r12), %ymm0
	vmovaps		-64(%rax), %ymm1
	vblendps	$ 0x03, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x4e, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x33, %ymm0, %ymm1, %ymm0
	vmovaps		64(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, 64(%r14)
	addq		$ 128, %r14

	vmovaps		-32(%r12), %ymm0
	vmovaps		-32(%rax), %ymm1
	vblendps	$ 0x03, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x4e, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x33, %ymm0, %ymm1, %ymm0
	vmovaps		-32(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, -32(%r14)

	cmpl		$ 3, %r10d
	jg			1b // main loop 

0: // consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

3: // clean-up loop

	vmovaps		0(%r12), %ymm0
	vmovaps		0(%rax), %ymm1
	vblendps	$ 0x03, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x4e, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x33, %ymm0, %ymm1, %ymm0
	vmovaps		0(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, 0(%r14)
	subl		$ 1, %r10d
	addq		$ 32, %r12
	addq		$ 32, %rax
	addq		$ 32, %r14

	cmpl		$ 0, %r10d
	jg			3b // clean-up loop 

2: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_sgead_8_2_lib8, .-inner_kernel_sgead_8_2_lib8
#endif
#endif





// subroutine
//
// input arguments:
// r10d   <- k
// r12    <- A
// r13d   <- 8*sda*sizeof(float)
// r14    <- B
// r15d   <- m1

#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_SGEAD_8_2_GEN_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_sgead_8_2_gen_lib8, @function
inner_kernel_sgead_8_2_gen_lib8:
#elif defined(OS_MAC)
_inner_kernel_sgead_8_2_gen_lib8:
#elif defined(OS_WINDOWS)
	.def inner_kernel_sgead_8_2_gen_lib8; .scl 2; .type 32; .endef
inner_kernel_sgead_8_2_gen_lib8:
#endif
#endif
	
	vbroadcastss	0(%r11), %ymm14

	// compute mask for rows
	vcvtsi2ss	%r15d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %ymm12
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %ymm12
#endif
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%ymm15, %ymm12, %ymm15

	movq	%r12, %rax // A1 <- A0
	addq	%r13, %rax // A1 <- A0 + 4*sda*sizeof(float)

	cmpl	$ 3, %r10d
	jle		0f // consider clean-up

	// main loop
	.p2align 3
1: // main loop

	vmovaps		0(%r12), %ymm0
	vmovaps		0(%rax), %ymm1
	vblendps	$ 0x03, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x4e, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x33, %ymm0, %ymm1, %ymm0
	vmaskmovps	0(%r14), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, 0(%r14)
	subl		$ 4, %r10d

	vmovaps		32(%r12), %ymm0
	vmovaps		32(%rax), %ymm1
	vblendps	$ 0x03, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x4e, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x33, %ymm0, %ymm1, %ymm0
	vmaskmovps	32(%r14), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, 32(%r14)
	addq		$ 128, %r12
	addq		$ 128, %rax

	vmovaps		-64(%r12), %ymm0
	vmovaps		-64(%rax), %ymm1
	vblendps	$ 0x03, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x4e, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x33, %ymm0, %ymm1, %ymm0
	vmaskmovps	64(%r14), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, 64(%r14)
	addq		$ 128, %r14

	vmovaps		-32(%r12), %ymm0
	vmovaps		-32(%rax), %ymm1
	vblendps	$ 0x03, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x4e, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x33, %ymm0, %ymm1, %ymm0
	vmaskmovps	-32(%r14), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, -32(%r14)

	cmpl		$ 3, %r10d
	jg			1b // main loop 

0: // consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

3: // clean-up loop

	vmovaps		0(%r12), %ymm0
	vmovaps		0(%rax), %ymm1
	vblendps	$ 0x03, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x4e, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x33, %ymm0, %ymm1, %ymm0
	vmaskmovps	0(%r14), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, 0(%r14)
	subl		$ 1, %r10d
	addq		$ 32, %r12
	addq		$ 32, %rax
	addq		$ 32, %r14

	cmpl		$ 0, %r10d
	jg			3b // clean-up loop 

2: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_sgead_8_2_gen_lib8, .-inner_kernel_sgead_8_2_gen_lib8
#endif
#endif





// subroutine
//
// input arguments:
// r10d   <- k
// r12    <- A
// r13d   <- 8*sda*sizeof(float)
// r14    <- B

#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_SGEAD_8_3_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_sgead_8_3_lib8, @function
inner_kernel_sgead_8_3_lib8:
#elif defined(OS_MAC)
_inner_kernel_sgead_8_3_lib8:
#elif defined(OS_WINDOWS)
	.def inner_kernel_sgead_8_3_lib8; .scl 2; .type 32; .endef
inner_kernel_sgead_8_3_lib8:
#endif
#endif
	
	vbroadcastss	0(%r11), %ymm14

	movq	%r12, %rax // A1 <- A0
	addq	%r13, %rax // A1 <- A0 + 4*sda*sizeof(float)

	cmpl	$ 3, %r10d
	jle		0f // consider clean-up

	// main loop
	.p2align 3
1: // main loop

	vmovaps		0(%r12), %ymm0
	vmovaps		0(%rax), %ymm1
	vblendps	$ 0x07, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x93, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x11, %ymm0, %ymm1, %ymm0
	vmovaps		0(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, 0(%r14)
	subl		$ 4, %r10d

	vmovaps		32(%r12), %ymm0
	vmovaps		32(%rax), %ymm1
	vblendps	$ 0x07, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x93, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x11, %ymm0, %ymm1, %ymm0
	vmovaps		32(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, 32(%r14)
	addq		$ 128, %r12
	addq		$ 128, %rax

	vmovaps		-64(%r12), %ymm0
	vmovaps		-64(%rax), %ymm1
	vblendps	$ 0x07, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x93, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x11, %ymm0, %ymm1, %ymm0
	vmovaps		64(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, 64(%r14)
	addq		$ 128, %r14

	vmovaps		-32(%r12), %ymm0
	vmovaps		-32(%rax), %ymm1
	vblendps	$ 0x07, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x93, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x11, %ymm0, %ymm1, %ymm0
	vmovaps		-32(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, -32(%r14)

	cmpl		$ 3, %r10d
	jg			1b // main loop 

0: // consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

3: // clean-up loop

	vmovaps		0(%r12), %ymm0
	vmovaps		0(%rax), %ymm1
	vblendps	$ 0x07, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x03, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x11, %ymm0, %ymm1, %ymm0
	vmovaps		0(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, 0(%r14)
	subl		$ 1, %r10d
	addq		$ 32, %r12
	addq		$ 32, %rax
	addq		$ 32, %r14

	cmpl		$ 0, %r10d
	jg			3b // clean-up loop 

2: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_sgead_8_3_lib8, .-inner_kernel_sgead_8_3_lib8
#endif
#endif





// subroutine
//
// input arguments:
// r10d   <- k
// r12    <- A
// r13d   <- 8*sda*sizeof(float)
// r14    <- B
// r15d   <- m1

#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_SGEAD_8_3_GEN_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_sgead_8_3_gen_lib8, @function
inner_kernel_sgead_8_3_gen_lib8:
#elif defined(OS_MAC)
_inner_kernel_sgead_8_3_gen_lib8:
#elif defined(OS_WINDOWS)
	.def inner_kernel_sgead_8_3_gen_lib8; .scl 2; .type 32; .endef
inner_kernel_sgead_8_3_gen_lib8:
#endif
#endif
	
	vbroadcastss	0(%r11), %ymm14

	// compute mask for rows
	vcvtsi2ss	%r15d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %ymm12
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %ymm12
#endif
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%ymm15, %ymm12, %ymm15

	movq	%r12, %rax // A1 <- A0
	addq	%r13, %rax // A1 <- A0 + 4*sda*sizeof(float)

	cmpl	$ 3, %r10d
	jle		0f // consider clean-up

	// main loop
	.p2align 3
1: // main loop

	vmovaps		0(%r12), %ymm0
	vmovaps		0(%rax), %ymm1
	vblendps	$ 0x07, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x93, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x11, %ymm0, %ymm1, %ymm0
	vmaskmovps	0(%r14), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, 0(%r14)
	subl		$ 4, %r10d

	vmovaps		32(%r12), %ymm0
	vmovaps		32(%rax), %ymm1
	vblendps	$ 0x07, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x93, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x11, %ymm0, %ymm1, %ymm0
	vmaskmovps	32(%r14), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, 32(%r14)
	addq		$ 128, %r12
	addq		$ 128, %rax

	vmovaps		-64(%r12), %ymm0
	vmovaps		-64(%rax), %ymm1
	vblendps	$ 0x07, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x93, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x11, %ymm0, %ymm1, %ymm0
	vmaskmovps	64(%r14), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, 64(%r14)
	addq		$ 128, %r14

	vmovaps		-32(%r12), %ymm0
	vmovaps		-32(%rax), %ymm1
	vblendps	$ 0x07, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x93, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x11, %ymm0, %ymm1, %ymm0
	vmaskmovps	-32(%r14), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, -32(%r14)

	cmpl		$ 3, %r10d
	jg			1b // main loop 

0: // consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

3: // clean-up loop

	vmovaps		0(%r12), %ymm0
	vmovaps		0(%rax), %ymm1
	vblendps	$ 0x07, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x93, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x11, %ymm0, %ymm1, %ymm0
	vmaskmovps	0(%r14), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, 0(%r14)
	subl		$ 1, %r10d
	addq		$ 32, %r12
	addq		$ 32, %rax
	addq		$ 32, %r14

	cmpl		$ 0, %r10d
	jg			3b // clean-up loop 

2: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_sgead_8_3_gen_lib8, .-inner_kernel_sgead_8_3_gen_lib8
#endif
#endif





// subroutine
//
// input arguments:
// r10d   <- k
// r12    <- A
// r13d   <- 8*sda*sizeof(float)
// r14    <- B

#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_SGEAD_8_4_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_sgead_8_4_lib8, @function
inner_kernel_sgead_8_4_lib8:
#elif defined(OS_MAC)
_inner_kernel_sgead_8_4_lib8:
#elif defined(OS_WINDOWS)
	.def inner_kernel_sgead_8_4_lib8; .scl 2; .type 32; .endef
inner_kernel_sgead_8_4_lib8:
#endif
#endif
	
	vbroadcastss	0(%r11), %ymm14

	movq	%r12, %rax // A1 <- A0
	addq	%r13, %rax // A1 <- A0 + 4*sda*sizeof(float)

	cmpl	$ 3, %r10d
	jle		0f // consider clean-up

	// main loop
	.p2align 3
1: // main loop

	vmovaps		16(%r12), %xmm0
	vmovaps		0(%rax), %xmm1
	vinsertf128	$ 0x01, %xmm1, %ymm0, %ymm0
	vmovaps		0(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, 0(%r14)
	subl		$ 4, %r10d

	vmovaps		48(%r12), %xmm0
	vmovaps		32(%rax), %xmm1
	vinsertf128	$ 0x01, %xmm1, %ymm0, %ymm0
	vmovaps		32(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, 32(%r14)
	addq		$ 128, %r12

	vmovaps		-48(%r12), %xmm0
	vmovaps		64(%rax), %xmm1
	vinsertf128	$ 0x01, %xmm1, %ymm0, %ymm0
	vmovaps		64(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, 64(%r14)
	addq		$ 128, %rax

	vmovaps		-16(%r12), %xmm0
	vmovaps		-32(%rax), %xmm1
	vinsertf128	$ 0x01, %xmm1, %ymm0, %ymm0
	vmovaps		96(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, 96(%r14)
	addq		$ 128, %r14

	cmpl		$ 3, %r10d
	jg			1b // main loop 

0: // consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

3: // clean-up loop

	vmovaps		16(%r12), %xmm0
	vmovaps		0(%rax), %xmm1
	vinsertf128	$ 0x01, %xmm1, %ymm0, %ymm0
	vmovaps		0(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, 0(%r14)
	subl		$ 1, %r10d
	addq		$ 32, %r12
	addq		$ 32, %rax
	addq		$ 32, %r14

	cmpl		$ 0, %r10d
	jg			3b // clean-up loop 

2: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_sgead_8_4_lib8, .-inner_kernel_sgead_8_4_lib8
#endif
#endif





// subroutine
//
// input arguments:
// r10d   <- k
// r12    <- A
// r13d   <- 8*sda*sizeof(float)
// r14    <- B
// r15d   <- m1

#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_SGEAD_8_4_GEN_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_sgead_8_4_gen_lib8, @function
inner_kernel_sgead_8_4_gen_lib8:
#elif defined(OS_MAC)
_inner_kernel_sgead_8_4_gen_lib8:
#elif defined(OS_WINDOWS)
	.def inner_kernel_sgead_8_4_gen_lib8; .scl 2; .type 32; .endef
inner_kernel_sgead_8_4_gen_lib8:
#endif
#endif
	
	vbroadcastss	0(%r11), %ymm14

	// compute mask for rows
	vcvtsi2ss	%r15d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %ymm12
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %ymm12
#endif
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%ymm15, %ymm12, %ymm15

	movq	%r12, %rax // A1 <- A0
	addq	%r13, %rax // A1 <- A0 + 4*sda*sizeof(float)

	cmpl	$ 3, %r10d
	jle		0f // consider clean-up

	// main loop
	.p2align 3
1: // main loop

	vmovaps		16(%r12), %xmm0
	vmovaps		0(%rax), %xmm1
	vinsertf128	$ 0x01, %xmm1, %ymm0, %ymm0
	vmaskmovps	0(%r14), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, 0(%r14)
	subl		$ 4, %r10d

	vmovaps		48(%r12), %xmm0
	vmovaps		32(%rax), %xmm1
	vinsertf128	$ 0x01, %xmm1, %ymm0, %ymm0
	vmaskmovps	32(%r14), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, 32(%r14)
	addq		$ 128, %r12

	vmovaps		-48(%r12), %xmm0
	vmovaps		64(%rax), %xmm1
	vinsertf128	$ 0x01, %xmm1, %ymm0, %ymm0
	vmaskmovps	64(%r14), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, 64(%r14)
	addq		$ 128, %rax

	vmovaps		-16(%r12), %xmm0
	vmovaps		-32(%rax), %xmm1
	vinsertf128	$ 0x01, %xmm1, %ymm0, %ymm0
	vmaskmovps	96(%r14), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, 96(%r14)
	addq		$ 128, %r14

	cmpl		$ 3, %r10d
	jg			1b // main loop 

0: // consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

3: // clean-up loop

	vmovaps		16(%r12), %xmm0
	vmovaps		0(%rax), %xmm1
	vinsertf128	$ 0x01, %xmm1, %ymm0, %ymm0
	vmaskmovps	0(%r14), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, 0(%r14)
	subl		$ 1, %r10d
	addq		$ 32, %r12
	addq		$ 32, %rax
	addq		$ 32, %r14

	cmpl		$ 0, %r10d
	jg			3b // clean-up loop 

2: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_sgead_8_4_gen_lib8, .-inner_kernel_sgead_8_4_gen_lib8
#endif
#endif





// subroutine
//
// input arguments:
// r10d   <- k
// r12    <- A
// r13d   <- 8*sda*sizeof(float)
// r14    <- B

#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_SGEAD_8_5_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_sgead_8_5_lib8, @function
inner_kernel_sgead_8_5_lib8:
#elif defined(OS_MAC)
_inner_kernel_sgead_8_5_lib8:
#elif defined(OS_WINDOWS)
	.def inner_kernel_sgead_8_5_lib8; .scl 2; .type 32; .endef
inner_kernel_sgead_8_5_lib8:
#endif
#endif
	
	vbroadcastss	0(%r11), %ymm14

	movq	%r12, %rax // A1 <- A0
	addq	%r13, %rax // A1 <- A0 + 4*sda*sizeof(float)

	cmpl	$ 3, %r10d
	jle		0f // consider clean-up

	// main loop
	.p2align 3
1: // main loop

	vmovaps		0(%r12), %ymm0
	vmovaps		0(%rax), %ymm1
	vblendps	$ 0x1f, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x39, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x88, %ymm0, %ymm1, %ymm0
	vmovaps		0(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, 0(%r14)
	subl		$ 4, %r10d

	vmovaps		32(%r12), %ymm0
	vmovaps		32(%rax), %ymm1
	vblendps	$ 0x1f, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x39, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x88, %ymm0, %ymm1, %ymm0
	vmovaps		32(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, 32(%r14)
	addq		$ 128, %r12
	addq		$ 128, %rax

	vmovaps		-64(%r12), %ymm0
	vmovaps		-64(%rax), %ymm1
	vblendps	$ 0x1f, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x39, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x88, %ymm0, %ymm1, %ymm0
	vmovaps		64(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, 64(%r14)
	addq		$ 128, %r14

	vmovaps		-32(%r12), %ymm0
	vmovaps		-32(%rax), %ymm1
	vblendps	$ 0x1f, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x39, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x88, %ymm0, %ymm1, %ymm0
	vmovaps		-32(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, -32(%r14)

	cmpl		$ 3, %r10d
	jg			1b // main loop 

0: // consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

3: // clean-up loop

	vmovaps		0(%r12), %ymm0
	vmovaps		0(%rax), %ymm1
	vblendps	$ 0x1f, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x39, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x88, %ymm0, %ymm1, %ymm0
	vmovaps		0(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, 0(%r14)
	subl		$ 1, %r10d
	addq		$ 32, %r12
	addq		$ 32, %rax
	addq		$ 32, %r14

	cmpl		$ 0, %r10d
	jg			3b // clean-up loop 

2: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_sgead_8_5_lib8, .-inner_kernel_sgead_8_5_lib8
#endif
#endif





// subroutine
//
// input arguments:
// r10d   <- k
// r12    <- A
// r13d   <- 8*sda*sizeof(float)
// r14    <- B
// r15d   <- m1

#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_SGEAD_8_5_GEN_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_sgead_8_5_gen_lib8, @function
inner_kernel_sgead_8_5_gen_lib8:
#elif defined(OS_MAC)
_inner_kernel_sgead_8_5_gen_lib8:
#elif defined(OS_WINDOWS)
	.def inner_kernel_sgead_8_5_gen_lib8; .scl 2; .type 32; .endef
inner_kernel_sgead_8_5_gen_lib8:
#endif
#endif
	
	vbroadcastss	0(%r11), %ymm14

	// compute mask for rows
	vcvtsi2ss	%r15d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %ymm12
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %ymm12
#endif
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%ymm15, %ymm12, %ymm15

	movq	%r12, %rax // A1 <- A0
	addq	%r13, %rax // A1 <- A0 + 4*sda*sizeof(float)

	cmpl	$ 3, %r10d
	jle		0f // consider clean-up

	// main loop
	.p2align 3
1: // main loop

	vmovaps		0(%r12), %ymm0
	vmovaps		0(%rax), %ymm1
	vblendps	$ 0x1f, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x39, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x88, %ymm0, %ymm1, %ymm0
	vmaskmovps	0(%r14), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, 0(%r14)
	subl		$ 4, %r10d

	vmovaps		32(%r12), %ymm0
	vmovaps		32(%rax), %ymm1
	vblendps	$ 0x1f, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x39, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x88, %ymm0, %ymm1, %ymm0
	vmaskmovps	32(%r14), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, 32(%r14)
	addq		$ 128, %r12
	addq		$ 128, %rax

	vmovaps		-64(%r12), %ymm0
	vmovaps		-64(%rax), %ymm1
	vblendps	$ 0x1f, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x39, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x88, %ymm0, %ymm1, %ymm0
	vmaskmovps	64(%r14), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, 64(%r14)
	addq		$ 128, %r14

	vmovaps		-32(%r12), %ymm0
	vmovaps		-32(%rax), %ymm1
	vblendps	$ 0x1f, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x39, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x88, %ymm0, %ymm1, %ymm0
	vmaskmovps	-32(%r14), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, -32(%r14)

	cmpl		$ 3, %r10d
	jg			1b // main loop 

0: // consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

3: // clean-up loop

	vmovaps		0(%r12), %ymm0
	vmovaps		0(%rax), %ymm1
	vblendps	$ 0x1f, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x39, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0x88, %ymm0, %ymm1, %ymm0
	vmaskmovps	0(%r14), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, 0(%r14)
	subl		$ 1, %r10d
	addq		$ 32, %r12
	addq		$ 32, %rax
	addq		$ 32, %r14

	cmpl		$ 0, %r10d
	jg			3b // clean-up loop 

2: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_sgead_8_5_gen_lib8, .-inner_kernel_sgead_8_5_gen_lib8
#endif
#endif





// subroutine
//
// input arguments:
// r10d   <- k
// r12    <- A
// r13d   <- 8*sda*sizeof(float)
// r14    <- B

#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_SGEAD_8_6_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_sgead_8_6_lib8, @function
inner_kernel_sgead_8_6_lib8:
#elif defined(OS_MAC)
_inner_kernel_sgead_8_6_lib8:
#elif defined(OS_WINDOWS)
	.def inner_kernel_sgead_8_6_lib8; .scl 2; .type 32; .endef
inner_kernel_sgead_8_6_lib8:
#endif
#endif
	
	vbroadcastss	0(%r11), %ymm14

	movq	%r12, %rax // A1 <- A0
	addq	%r13, %rax // A1 <- A0 + 4*sda*sizeof(float)

	cmpl	$ 3, %r10d
	jle		0f // consider clean-up

	// main loop
	.p2align 3
1: // main loop

	vmovaps		0(%r12), %ymm0
	vmovaps		0(%rax), %ymm1
	vblendps	$ 0x3f, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x4e, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0xcc, %ymm0, %ymm1, %ymm0
	vmovaps		0(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, 0(%r14)
	subl		$ 4, %r10d

	vmovaps		32(%r12), %ymm0
	vmovaps		32(%rax), %ymm1
	vblendps	$ 0x3f, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x4e, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0xcc, %ymm0, %ymm1, %ymm0
	vmovaps		32(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, 32(%r14)
	addq		$ 128, %r12
	addq		$ 128, %rax

	vmovaps		-64(%r12), %ymm0
	vmovaps		-64(%rax), %ymm1
	vblendps	$ 0x3f, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x4e, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0xcc, %ymm0, %ymm1, %ymm0
	vmovaps		64(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, 64(%r14)
	addq		$ 128, %r14

	vmovaps		-32(%r12), %ymm0
	vmovaps		-32(%rax), %ymm1
	vblendps	$ 0x3f, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x4e, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0xcc, %ymm0, %ymm1, %ymm0
	vmovaps		-32(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, -32(%r14)

	cmpl		$ 3, %r10d
	jg			1b // main loop 

0: // consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

3: // clean-up loop

	vmovaps		0(%r12), %ymm0
	vmovaps		0(%rax), %ymm1
	vblendps	$ 0x3f, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x4e, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0xcc, %ymm0, %ymm1, %ymm0
	vmovaps		0(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, 0(%r14)
	subl		$ 1, %r10d
	addq		$ 32, %r12
	addq		$ 32, %rax
	addq		$ 32, %r14

	cmpl		$ 0, %r10d
	jg			3b // clean-up loop 

2: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_sgead_8_6_lib8, .-inner_kernel_sgead_8_6_lib8
#endif
#endif





// subroutine
//
// input arguments:
// r10d   <- k
// r12    <- A
// r13d   <- 8*sda*sizeof(float)
// r14    <- B
// r15d   <- m1

#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_SGEAD_8_6_GEN_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_sgead_8_6_gen_lib8, @function
inner_kernel_sgead_8_6_gen_lib8:
#elif defined(OS_MAC)
_inner_kernel_sgead_8_6_gen_lib8:
#elif defined(OS_WINDOWS)
	.def inner_kernel_sgead_8_6_gen_lib8; .scl 2; .type 32; .endef
inner_kernel_sgead_8_6_gen_lib8:
#endif
#endif
	
	vbroadcastss	0(%r11), %ymm14

	// compute mask for rows
	vcvtsi2ss	%r15d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %ymm12
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %ymm12
#endif
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%ymm15, %ymm12, %ymm15

	movq	%r12, %rax // A1 <- A0
	addq	%r13, %rax // A1 <- A0 + 4*sda*sizeof(float)

	cmpl	$ 3, %r10d
	jle		0f // consider clean-up

	// main loop
	.p2align 3
1: // main loop

	vmovaps		0(%r12), %ymm0
	vmovaps		0(%rax), %ymm1
	vblendps	$ 0x3f, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x4e, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0xcc, %ymm0, %ymm1, %ymm0
	vmaskmovps	0(%r14), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, 0(%r14)
	subl		$ 4, %r10d

	vmovaps		32(%r12), %ymm0
	vmovaps		32(%rax), %ymm1
	vblendps	$ 0x3f, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x4e, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0xcc, %ymm0, %ymm1, %ymm0
	vmaskmovps	32(%r14), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, 32(%r14)
	addq		$ 128, %r12
	addq		$ 128, %rax

	vmovaps		-64(%r12), %ymm0
	vmovaps		-64(%rax), %ymm1
	vblendps	$ 0x3f, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x4e, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0xcc, %ymm0, %ymm1, %ymm0
	vmaskmovps	64(%r14), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, 64(%r14)
	addq		$ 128, %r14

	vmovaps		-32(%r12), %ymm0
	vmovaps		-32(%rax), %ymm1
	vblendps	$ 0x3f, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x4e, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0xcc, %ymm0, %ymm1, %ymm0
	vmaskmovps	-32(%r14), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, -32(%r14)

	cmpl		$ 3, %r10d
	jg			1b // main loop 

0: // consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

3: // clean-up loop

	vmovaps		0(%r12), %ymm0
	vmovaps		0(%rax), %ymm1
	vblendps	$ 0x3f, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x4e, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0xcc, %ymm0, %ymm1, %ymm0
	vmaskmovps	0(%r14), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, 0(%r14)
	subl		$ 1, %r10d
	addq		$ 32, %r12
	addq		$ 32, %rax
	addq		$ 32, %r14

	cmpl		$ 0, %r10d
	jg			3b // clean-up loop 

2: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_sgead_8_6_gen_lib8, .-inner_kernel_sgead_8_6_gen_lib8
#endif
#endif





// subroutine
//
// input arguments:
// r10d   <- k
// r12    <- A
// r13d   <- 8*sda*sizeof(float)
// r14    <- B

#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_SGEAD_8_7_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_sgead_8_7_lib8, @function
inner_kernel_sgead_8_7_lib8:
#elif defined(OS_MAC)
_inner_kernel_sgead_8_7_lib8:
#elif defined(OS_WINDOWS)
	.def inner_kernel_sgead_8_7_lib8; .scl 2; .type 32; .endef
inner_kernel_sgead_8_7_lib8:
#endif
#endif
	
	vbroadcastss	0(%r11), %ymm14

	movq	%r12, %rax // A1 <- A0
	addq	%r13, %rax // A1 <- A0 + 4*sda*sizeof(float)

	cmpl	$ 3, %r10d
	jle		0f // consider clean-up

	// main loop
	.p2align 3
1: // main loop

	vmovaps		0(%r12), %ymm0
	vmovaps		0(%rax), %ymm1
	vblendps	$ 0x7f, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x93, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0xee, %ymm0, %ymm1, %ymm0
	vmovaps		0(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, 0(%r14)
	subl		$ 4, %r10d

	vmovaps		32(%r12), %ymm0
	vmovaps		32(%rax), %ymm1
	vblendps	$ 0x7f, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x93, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0xee, %ymm0, %ymm1, %ymm0
	vmovaps		32(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, 32(%r14)
	addq		$ 128, %r12
	addq		$ 128, %rax

	vmovaps		-64(%r12), %ymm0
	vmovaps		-64(%rax), %ymm1
	vblendps	$ 0x7f, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x93, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0xee, %ymm0, %ymm1, %ymm0
	vmovaps		64(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, 64(%r14)
	addq		$ 128, %r14

	vmovaps		-32(%r12), %ymm0
	vmovaps		-32(%rax), %ymm1
	vblendps	$ 0x7f, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x93, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0xee, %ymm0, %ymm1, %ymm0
	vmovaps		-32(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, -32(%r14)

	cmpl		$ 3, %r10d
	jg			1b // main loop 

0: // consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

3: // clean-up loop

	vmovaps		0(%r12), %ymm0
	vmovaps		0(%rax), %ymm1
	vblendps	$ 0x7f, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x03, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0xee, %ymm0, %ymm1, %ymm0
	vmovaps		0(%r14), %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmovaps		%ymm0, 0(%r14)
	subl		$ 1, %r10d
	addq		$ 32, %r12
	addq		$ 32, %rax
	addq		$ 32, %r14

	cmpl		$ 0, %r10d
	jg			3b // clean-up loop 

2: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_sgead_8_7_lib8, .-inner_kernel_sgead_8_7_lib8
#endif
#endif





// subroutine
//
// input arguments:
// r10d   <- k
// r12    <- A
// r13d   <- 8*sda*sizeof(float)
// r14    <- B
// r15d   <- m1

#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_SGEAD_8_7_GEN_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_sgead_8_7_gen_lib8, @function
inner_kernel_sgead_8_7_gen_lib8:
#elif defined(OS_MAC)
_inner_kernel_sgead_8_7_gen_lib8:
#elif defined(OS_WINDOWS)
	.def inner_kernel_sgead_8_7_gen_lib8; .scl 2; .type 32; .endef
inner_kernel_sgead_8_7_gen_lib8:
#endif
#endif
	
	vbroadcastss	0(%r11), %ymm14

	// compute mask for rows
	vcvtsi2ss	%r15d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %ymm12
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %ymm12
#endif
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%ymm15, %ymm12, %ymm15

	movq	%r12, %rax // A1 <- A0
	addq	%r13, %rax // A1 <- A0 + 4*sda*sizeof(float)

	cmpl	$ 3, %r10d
	jle		0f // consider clean-up

	// main loop
	.p2align 3
1: // main loop

	vmovaps		0(%r12), %ymm0
	vmovaps		0(%rax), %ymm1
	vblendps	$ 0x7f, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x93, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0xee, %ymm0, %ymm1, %ymm0
	vmaskmovps	0(%r14), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, 0(%r14)
	subl		$ 4, %r10d

	vmovaps		32(%r12), %ymm0
	vmovaps		32(%rax), %ymm1
	vblendps	$ 0x7f, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x93, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0xee, %ymm0, %ymm1, %ymm0
	vmaskmovps	32(%r14), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, 32(%r14)
	addq		$ 128, %r12
	addq		$ 128, %rax

	vmovaps		-64(%r12), %ymm0
	vmovaps		-64(%rax), %ymm1
	vblendps	$ 0x7f, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x93, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0xee, %ymm0, %ymm1, %ymm0
	vmaskmovps	64(%r14), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, 64(%r14)
	addq		$ 128, %r14

	vmovaps		-32(%r12), %ymm0
	vmovaps		-32(%rax), %ymm1
	vblendps	$ 0x7f, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x93, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0xee, %ymm0, %ymm1, %ymm0
	vmaskmovps	-32(%r14), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, -32(%r14)

	cmpl		$ 3, %r10d
	jg			1b // main loop 

0: // consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

3: // clean-up loop

	vmovaps		0(%r12), %ymm0
	vmovaps		0(%rax), %ymm1
	vblendps	$ 0x7f, %ymm1, %ymm0, %ymm0
	vpermilps	$ 0x93, %ymm0, %ymm0
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm1
	vblendps	$ 0xee, %ymm0, %ymm1, %ymm0
	vmaskmovps	0(%r14), %ymm15, %ymm13
	vmulps		%ymm14, %ymm0, %ymm0
	vaddps		%ymm13, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm15, 0(%r14)
	subl		$ 1, %r10d
	addq		$ 32, %r12
	addq		$ 32, %rax
	addq		$ 32, %r14

	cmpl		$ 0, %r10d
	jg			3b // clean-up loop 

2: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_sgead_8_7_gen_lib8, .-inner_kernel_sgead_8_7_gen_lib8
#endif
#endif





//                            1      2             3         4
// void kernel_sgead_8_0_lib8(int k, float *alpha, float *A, float *B);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgead_8_0_lib8
	.type kernel_sgead_8_0_lib8, @function
kernel_sgead_8_0_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgead_8_0_lib8
_kernel_sgead_8_0_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgead_8_0_lib8
	.def kernel_sgead_8_0_lib8; .scl 2; .type 32; .endef
kernel_sgead_8_0_lib8:
#endif
	
	PROLOGUE

	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // alpha
	movq	ARG3, %r12  // A
	movq	ARG4, %r13  // B

#if MACRO_LEVEL>=1
	INNER_KERNEL_SGEAD_8_0_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_sgead_8_0_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_sgead_8_0_lib8
#endif
#endif

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgead_8_0_lib8, .-kernel_sgead_8_0_lib8
#endif





//                                  rdi    rsi       rdx       rcx
// void kernel_sgead_8_0_gen_lib8(int k, float *A, float *B, int m1);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgead_8_0_gen_lib8
	.type kernel_sgead_8_0_gen_lib8, @function
kernel_sgead_8_0_gen_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgead_8_0_gen_lib8
_kernel_sgead_8_0_gen_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgead_8_0_gen_lib8
	.def kernel_sgead_8_0_gen_lib8; .scl 2; .type 32; .endef
kernel_sgead_8_0_gen_lib8:
#endif
	
	PROLOGUE

	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // alpha
	movq	ARG3, %r12  // A
	movq	ARG4, %r13  // B
	movq	ARG5, %r14 // m1

#if MACRO_LEVEL>=1
	INNER_KERNEL_SGEAD_8_0_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_sgead_8_0_gen_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_sgead_8_0_gen_lib8
#endif
#endif

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgead_8_0_gen_lib8, .-kernel_sgead_8_0_gen_lib8
#endif





//                              rdi    rsi       rdx      rcx
// void kernel_sgead_8_1_lib8(int k, float *A, int sda, float *B);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgead_8_1_lib8
	.type kernel_sgead_8_1_lib8, @function
kernel_sgead_8_1_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgead_8_1_lib8
_kernel_sgead_8_1_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgead_8_1_lib8
	.def kernel_sgead_8_1_lib8; .scl 2; .type 32; .endef
kernel_sgead_8_1_lib8:
#endif
	
	PROLOGUE

	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // alpha
	movq	ARG3, %r12  // A
	movq	ARG4, %r13  // 8*sda*sizeof(float)
	sall	$ 5, %r13d
	movq	ARG5, %r14  // B

#if MACRO_LEVEL>=1
	INNER_KERNEL_SGEAD_8_1_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_sgead_8_1_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_sgead_8_1_lib8
#endif
#endif

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgead_8_1_lib8, .-kernel_sgead_8_1_lib8
#endif





//                                  rdi    rsi       rdx      rcx       r8
// void kernel_sgead_8_1_gen_lib8(int k, float *A, int sda, float *B, int m0);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgead_8_1_gen_lib8
	.type kernel_sgead_8_1_gen_lib8, @function
kernel_sgead_8_1_gen_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgead_8_1_gen_lib8
_kernel_sgead_8_1_gen_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgead_8_1_gen_lib8
	.def kernel_sgead_8_1_gen_lib8; .scl 2; .type 32; .endef
kernel_sgead_8_1_gen_lib8:
#endif
	
	PROLOGUE

	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // alpha
	movq	ARG3, %r12  // A
	movq	ARG4, %r13  // 8*sda*sizeof(float)
	sall	$ 5, %r13d
	movq	ARG5, %r14  // B
	movq	ARG6, %r15 // m1

#if MACRO_LEVEL>=1
	INNER_KERNEL_SGEAD_8_1_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_sgead_8_1_gen_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_sgead_8_1_gen_lib8
#endif
#endif

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgead_8_1_gen_lib8, .-kernel_sgead_8_1_gen_lib8
#endif





//                              rdi    rsi       rdx      rcx
// void kernel_sgead_8_2_lib8(int k, float *A, int sda, float *B);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgead_8_2_lib8
	.type kernel_sgead_8_2_lib8, @function
kernel_sgead_8_2_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgead_8_2_lib8
_kernel_sgead_8_2_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgead_8_2_lib8
	.def kernel_sgead_8_2_lib8; .scl 2; .type 32; .endef
kernel_sgead_8_2_lib8:
#endif
	
	PROLOGUE

	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // alpha
	movq	ARG3, %r12  // A
	movq	ARG4, %r13  // 8*sda*sizeof(float)
	sall	$ 5, %r13d
	movq	ARG5, %r14  // B

#if MACRO_LEVEL>=1
	INNER_KERNEL_SGEAD_8_2_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_sgead_8_2_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_sgead_8_2_lib8
#endif
#endif

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgead_8_2_lib8, .-kernel_sgead_8_2_lib8
#endif





//                                  rdi    rsi       rdx      rcx       r8
// void kernel_sgead_8_2_gen_lib8(int k, float *A, int sda, float *B, int m0);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgead_8_2_gen_lib8
	.type kernel_sgead_8_2_gen_lib8, @function
kernel_sgead_8_2_gen_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgead_8_2_gen_lib8
_kernel_sgead_8_2_gen_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgead_8_2_gen_lib8
	.def kernel_sgead_8_2_gen_lib8; .scl 2; .type 32; .endef
kernel_sgead_8_2_gen_lib8:
#endif
	
	PROLOGUE

	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // alpha
	movq	ARG3, %r12  // A
	movq	ARG4, %r13  // 8*sda*sizeof(float)
	sall	$ 5, %r13d
	movq	ARG5, %r14  // B
	movq	ARG6, %r15 // m1

#if MACRO_LEVEL>=1
	INNER_KERNEL_SGEAD_8_2_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_sgead_8_2_gen_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_sgead_8_2_gen_lib8
#endif
#endif

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgead_8_2_gen_lib8, .-kernel_sgead_8_2_gen_lib8
#endif





//                              rdi    rsi       rdx      rcx
// void kernel_sgead_8_3_lib8(int k, float *A, int sda, float *B);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgead_8_3_lib8
	.type kernel_sgead_8_3_lib8, @function
kernel_sgead_8_3_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgead_8_3_lib8
_kernel_sgead_8_3_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgead_8_3_lib8
	.def kernel_sgead_8_3_lib8; .scl 2; .type 32; .endef
kernel_sgead_8_3_lib8:
#endif
	
	PROLOGUE

	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // alpha
	movq	ARG3, %r12  // A
	movq	ARG4, %r13  // 8*sda*sizeof(float)
	sall	$ 5, %r13d
	movq	ARG5, %r14  // B

#if MACRO_LEVEL>=1
	INNER_KERNEL_SGEAD_8_3_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_sgead_8_3_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_sgead_8_3_lib8
#endif
#endif

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgead_8_3_lib8, .-kernel_sgead_8_3_lib8
#endif





//                                  rdi    rsi       rdx      rcx       r8
// void kernel_sgead_8_3_gen_lib8(int k, float *A, int sda, float *B, int m0);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgead_8_3_gen_lib8
	.type kernel_sgead_8_3_gen_lib8, @function
kernel_sgead_8_3_gen_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgead_8_3_gen_lib8
_kernel_sgead_8_3_gen_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgead_8_3_gen_lib8
	.def kernel_sgead_8_3_gen_lib8; .scl 2; .type 32; .endef
kernel_sgead_8_3_gen_lib8:
#endif
	
	PROLOGUE

	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // alpha
	movq	ARG3, %r12  // A
	movq	ARG4, %r13  // 8*sda*sizeof(float)
	sall	$ 5, %r13d
	movq	ARG5, %r14  // B
	movq	ARG6, %r15 // m1

#if MACRO_LEVEL>=1
	INNER_KERNEL_SGEAD_8_3_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_sgead_8_3_gen_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_sgead_8_3_gen_lib8
#endif
#endif

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgead_8_3_gen_lib8, .-kernel_sgead_8_3_gen_lib8
#endif





//                              rdi    rsi       rdx      rcx
// void kernel_sgead_8_4_lib8(int k, float *A, int sda, float *B);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgead_8_4_lib8
	.type kernel_sgead_8_4_lib8, @function
kernel_sgead_8_4_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgead_8_4_lib8
_kernel_sgead_8_4_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgead_8_4_lib8
	.def kernel_sgead_8_4_lib8; .scl 2; .type 32; .endef
kernel_sgead_8_4_lib8:
#endif
	
	PROLOGUE

	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // alpha
	movq	ARG3, %r12  // A
	movq	ARG4, %r13  // 8*sda*sizeof(float)
	sall	$ 5, %r13d
	movq	ARG5, %r14  // B

#if MACRO_LEVEL>=1
	INNER_KERNEL_SGEAD_8_4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_sgead_8_4_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_sgead_8_4_lib8
#endif
#endif

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgead_8_4_lib8, .-kernel_sgead_8_4_lib8
#endif





//                                  rdi    rsi       rdx      rcx       r8
// void kernel_sgead_8_4_gen_lib8(int k, float *A, int sda, float *B, int m0);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgead_8_4_gen_lib8
	.type kernel_sgead_8_4_gen_lib8, @function
kernel_sgead_8_4_gen_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgead_8_4_gen_lib8
_kernel_sgead_8_4_gen_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgead_8_4_gen_lib8
	.def kernel_sgead_8_4_gen_lib8; .scl 2; .type 32; .endef
kernel_sgead_8_4_gen_lib8:
#endif
	
	PROLOGUE

	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // alpha
	movq	ARG3, %r12  // A
	movq	ARG4, %r13  // 8*sda*sizeof(float)
	sall	$ 5, %r13d
	movq	ARG5, %r14  // B
	movq	ARG6, %r15 // m1

#if MACRO_LEVEL>=1
	INNER_KERNEL_SGEAD_8_4_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_sgead_8_4_gen_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_sgead_8_4_gen_lib8
#endif
#endif

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgead_8_4_gen_lib8, .-kernel_sgead_8_4_gen_lib8
#endif





//                              rdi    rsi       rdx      rcx
// void kernel_sgead_8_5_lib8(int k, float *A, int sda, float *B);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgead_8_5_lib8
	.type kernel_sgead_8_5_lib8, @function
kernel_sgead_8_5_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgead_8_5_lib8
_kernel_sgead_8_5_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgead_8_5_lib8
	.def kernel_sgead_8_5_lib8; .scl 2; .type 32; .endef
kernel_sgead_8_5_lib8:
#endif
	
	PROLOGUE

	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // alpha
	movq	ARG3, %r12  // A
	movq	ARG4, %r13  // 8*sda*sizeof(float)
	sall	$ 5, %r13d
	movq	ARG5, %r14  // B

#if MACRO_LEVEL>=1
	INNER_KERNEL_SGEAD_8_5_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_sgead_8_5_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_sgead_8_5_lib8
#endif
#endif

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgead_8_5_lib8, .-kernel_sgead_8_5_lib8
#endif





//                                  rdi    rsi       rdx      rcx       r8
// void kernel_sgead_8_5_gen_lib8(int k, float *A, int sda, float *B, int m0);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgead_8_5_gen_lib8
	.type kernel_sgead_8_5_gen_lib8, @function
kernel_sgead_8_5_gen_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgead_8_5_gen_lib8
_kernel_sgead_8_5_gen_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgead_8_5_gen_lib8
	.def kernel_sgead_8_5_gen_lib8; .scl 2; .type 32; .endef
kernel_sgead_8_5_gen_lib8:
#endif
	
	PROLOGUE

	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // alpha
	movq	ARG3, %r12  // A
	movq	ARG4, %r13  // 8*sda*sizeof(float)
	sall	$ 5, %r13d
	movq	ARG5, %r14  // B
	movq	ARG6, %r15 // m1

#if MACRO_LEVEL>=1
	INNER_KERNEL_SGEAD_8_5_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_sgead_8_5_gen_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_sgead_8_5_gen_lib8
#endif
#endif

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgead_8_5_gen_lib8, .-kernel_sgead_8_5_gen_lib8
#endif





//                              rdi    rsi       rdx      rcx
// void kernel_sgead_8_6_lib8(int k, float *A, int sda, float *B);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgead_8_6_lib8
	.type kernel_sgead_8_6_lib8, @function
kernel_sgead_8_6_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgead_8_6_lib8
_kernel_sgead_8_6_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgead_8_6_lib8
	.def kernel_sgead_8_6_lib8; .scl 2; .type 32; .endef
kernel_sgead_8_6_lib8:
#endif
	
	PROLOGUE

	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // alpha
	movq	ARG3, %r12  // A
	movq	ARG4, %r13  // 8*sda*sizeof(float)
	sall	$ 5, %r13d
	movq	ARG5, %r14  // B

#if MACRO_LEVEL>=1
	INNER_KERNEL_SGEAD_8_6_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_sgead_8_6_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_sgead_8_6_lib8
#endif
#endif

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgead_8_6_lib8, .-kernel_sgead_8_6_lib8
#endif





//                                  rdi    rsi       rdx      rcx       r8
// void kernel_sgead_8_6_gen_lib8(int k, float *A, int sda, float *B, int m0);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgead_8_6_gen_lib8
	.type kernel_sgead_8_6_gen_lib8, @function
kernel_sgead_8_6_gen_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgead_8_6_gen_lib8
_kernel_sgead_8_6_gen_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgead_8_6_gen_lib8
	.def kernel_sgead_8_6_gen_lib8; .scl 2; .type 32; .endef
kernel_sgead_8_6_gen_lib8:
#endif
	
	PROLOGUE

	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // alpha
	movq	ARG3, %r12  // A
	movq	ARG4, %r13  // 8*sda*sizeof(float)
	sall	$ 5, %r13d
	movq	ARG5, %r14  // B
	movq	ARG6, %r15 // m1

#if MACRO_LEVEL>=1
	INNER_KERNEL_SGEAD_8_6_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_sgead_8_6_gen_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_sgead_8_6_gen_lib8
#endif
#endif

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgead_8_6_gen_lib8, .-kernel_sgead_8_6_gen_lib8
#endif





//                              rdi    rsi       rdx      rcx
// void kernel_sgead_8_7_lib8(int k, float *A, int sda, float *B);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgead_8_7_lib8
	.type kernel_sgead_8_7_lib8, @function
kernel_sgead_8_7_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgead_8_7_lib8
_kernel_sgead_8_7_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgead_8_7_lib8
	.def kernel_sgead_8_7_lib8; .scl 2; .type 32; .endef
kernel_sgead_8_7_lib8:
#endif
	
	PROLOGUE

	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // alpha
	movq	ARG3, %r12  // A
	movq	ARG4, %r13  // 8*sda*sizeof(float)
	sall	$ 5, %r13d
	movq	ARG5, %r14  // B

#if MACRO_LEVEL>=1
	INNER_KERNEL_SGEAD_8_7_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_sgead_8_7_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_sgead_8_7_lib8
#endif
#endif

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgead_8_7_lib8, .-kernel_sgead_8_7_lib8
#endif





//                                  rdi    rsi       rdx      rcx       r8
// void kernel_sgead_8_7_gen_lib8(int k, float *A, int sda, float *B, int m0);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgead_8_7_gen_lib8
	.type kernel_sgead_8_7_gen_lib8, @function
kernel_sgead_8_7_gen_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgead_8_7_gen_lib8
_kernel_sgead_8_7_gen_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgead_8_7_gen_lib8
	.def kernel_sgead_8_7_gen_lib8; .scl 2; .type 32; .endef
kernel_sgead_8_7_gen_lib8:
#endif
	
	PROLOGUE

	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // alpha
	movq	ARG3, %r12  // A
	movq	ARG4, %r13  // 8*sda*sizeof(float)
	sall	$ 5, %r13d
	movq	ARG5, %r14  // B
	movq	ARG6, %r15 // m1

#if MACRO_LEVEL>=1
	INNER_KERNEL_SGEAD_8_7_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_sgead_8_7_gen_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_sgead_8_7_gen_lib8
#endif
#endif

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgead_8_7_gen_lib8, .-kernel_sgead_8_7_gen_lib8
#endif





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

