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
	vmovapd	%ymm0, %ymm3; \
	vmovapd	%ymm0, %ymm4; \
	vmovapd	%ymm0, %ymm5; \
	vmovapd	%ymm0, %ymm6; \
	vmovapd	%ymm0, %ymm7
#define NEG_ACC \
	vmovapd		.LC11(%rip), %ymm15; \
	vxorpd		%ymm15, %ymm0, %ymm0; \
	vxorpd		%ymm15, %ymm1, %ymm1; \
	vxorpd		%ymm15, %ymm2, %ymm2; \
	vxorpd		%ymm15, %ymm3, %ymm3; \
	vxorpd		%ymm15, %ymm4, %ymm4; \
	vxorpd		%ymm15, %ymm5, %ymm5; \
	vxorpd		%ymm15, %ymm6, %ymm6; \
	vxorpd		%ymm15, %ymm7, %ymm7

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
	vmovapd	%ymm0, %ymm3; \
	vmovapd	%ymm0, %ymm4; \
	vmovapd	%ymm0, %ymm5; \
	vmovapd	%ymm0, %ymm6; \
	vmovapd	%ymm0, %ymm7
#define NEG_ACC \
	vmovapd		LC11(%rip), %ymm15; \
	vxorpd		%ymm15, %ymm0, %ymm0; \
	vxorpd		%ymm15, %ymm1, %ymm1; \
	vxorpd		%ymm15, %ymm2, %ymm2; \
	vxorpd		%ymm15, %ymm3, %ymm3; \
	vxorpd		%ymm15, %ymm4, %ymm4; \
	vxorpd		%ymm15, %ymm5, %ymm5; \
	vxorpd		%ymm15, %ymm6, %ymm6; \
	vxorpd		%ymm15, %ymm7, %ymm7

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
	vmovapd	%ymm0, %ymm3; \
	vmovapd	%ymm0, %ymm4; \
	vmovapd	%ymm0, %ymm5; \
	vmovapd	%ymm0, %ymm6; \
	vmovapd	%ymm0, %ymm7
#define NEG_ACC \
	vmovapd		.LC11(%rip), %ymm15; \
	vxorpd		%ymm15, %ymm0, %ymm0; \
	vxorpd		%ymm15, %ymm1, %ymm1; \
	vxorpd		%ymm15, %ymm2, %ymm2; \
	vxorpd		%ymm15, %ymm3, %ymm3; \
	vxorpd		%ymm15, %ymm4, %ymm4; \
	vxorpd		%ymm15, %ymm5, %ymm5; \
	vxorpd		%ymm15, %ymm6, %ymm6; \
	vxorpd		%ymm15, %ymm7, %ymm7

#else

#error wrong OS

#endif



#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.text
#elif defined(OS_MAC)
	.section	__TEXT,__text,regular,pure_instructions
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- 4*sda*sizeof(double)
// r13   <- B
// ymm0  <- [d00 d11 d22 d33]
// ymm1  <- [d01 d10 d23 d32]
// ymm2  <- [d03 d12 d21 d30]
// ymm3  <- [d02 d13 d20 d31]
// ymm4  <- [d40 d51 d62 d73]
// ymm5  <- [d41 d50 d63 d72]
// ymm6  <- [d43 d52 d61 d70]
// ymm7  <- [d42 d53 d60 d71]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

//
// output arguments:
// r10d  <- 0
// r11   <- A+4*k*sizeof(double)
// r12   <- 4*sda*sizeof(double)
// r13   <- B+4*k*sizeof(double)
// ymm0  <- [d00 d11 d22 d33]
// ymm1  <- [d01 d10 d23 d32]
// ymm2  <- [d03 d12 d21 d30]
// ymm3  <- [d02 d13 d20 d31]
// ymm4  <- [d40 d51 d62 d73]
// ymm5  <- [d41 d50 d63 d72]
// ymm6  <- [d43 d52 d61 d70]
// ymm7  <- [d42 d53 d60 d71]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_8X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_8x4_lib4)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	// prefetch
	prefetcht0	0(%r13) // software prefetch
	prefetcht0	0+64(%r13) // software prefetch

	// preload
	vmovapd 0(%r11), %ymm8 // A0[0]
	vmovapd 0(%r11, %r12, 1), %ymm9 // A1[0]
	vmovapd 0(%r13), %ymm12 // B[0]

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	prefetcht0	128(%r13) // software prefetch
	prefetcht0	128+64(%r13) // software prefetch

	// unroll 0
	vmovapd 32(%r13), %ymm13 // B[4]
	vmulpd	%ymm8, %ymm12, %ymm15
	vaddpd	%ymm0, %ymm15, %ymm0
	vshufpd $ 0x5, %ymm12, %ymm12, %ymm14
	vmulpd	%ymm9, %ymm12, %ymm15
	vaddpd	%ymm4, %ymm15, %ymm4

	vperm2f128 $ 0x1, %ymm14, %ymm14, %ymm12
	vmulpd	%ymm8, %ymm14, %ymm15
	vaddpd	%ymm1, %ymm15, %ymm1
	vmovapd 32(%r11), %ymm10 // A0[4]
	vmulpd	%ymm9, %ymm14, %ymm15
	vaddpd	%ymm5, %ymm15, %ymm5

	vmovapd 32(%r11, %r12, 1), %ymm11 // A1[4]
	vmulpd	%ymm8, %ymm12, %ymm15
	vaddpd	%ymm3, %ymm15, %ymm3
	vshufpd $ 0x5, %ymm12, %ymm12, %ymm14
	vmulpd	%ymm9, %ymm12, %ymm15
	vaddpd	%ymm7, %ymm15, %ymm7

	subl	$ 4, %r10d
	vmulpd	%ymm8, %ymm14, %ymm15
	vaddpd	%ymm2, %ymm15, %ymm2
	vmulpd	%ymm9, %ymm14, %ymm15
	vaddpd	%ymm6, %ymm15, %ymm6

	// unroll 1
	vmovapd 64(%r13), %ymm12 // B[8]
	vmulpd	%ymm10, %ymm13, %ymm15
	vaddpd	%ymm0, %ymm15, %ymm0
	vshufpd $ 0x5, %ymm13, %ymm13, %ymm14
	vmulpd	%ymm11, %ymm13, %ymm15
	vaddpd	%ymm4, %ymm15, %ymm4

	vperm2f128 $ 0x1, %ymm14, %ymm14, %ymm13
	vmulpd	%ymm10, %ymm14, %ymm15
	vaddpd	%ymm1, %ymm15, %ymm1
	vmovapd 64(%r11), %ymm8 // A0[8]
	vmulpd	%ymm11, %ymm14, %ymm15
	vaddpd	%ymm5, %ymm15, %ymm5

	vmovapd 64(%r11, %r12, 1), %ymm9 // A1[8]
	vmulpd	%ymm10, %ymm13, %ymm15
	vaddpd	%ymm3, %ymm15, %ymm3
	vshufpd $ 0x5, %ymm13, %ymm13, %ymm14
	vmulpd	%ymm11, %ymm13, %ymm15
	vaddpd	%ymm7, %ymm15, %ymm7

	vmulpd	%ymm10, %ymm14, %ymm15
	vaddpd	%ymm2, %ymm15, %ymm2
	vmulpd	%ymm11, %ymm14, %ymm15
	vaddpd	%ymm6, %ymm15, %ymm6

	// unroll 2
	vmovapd 96(%r13), %ymm13 // B[12]
	vmulpd	%ymm8, %ymm12, %ymm15
	vaddpd	%ymm0, %ymm15, %ymm0
	vshufpd $ 0x5, %ymm12, %ymm12, %ymm14
	vmulpd	%ymm9, %ymm12, %ymm15
	vaddpd	%ymm4, %ymm15, %ymm4

	vperm2f128 $ 0x1, %ymm14, %ymm14, %ymm12
	vmulpd	%ymm8, %ymm14, %ymm15
	vaddpd	%ymm1, %ymm15, %ymm1
	vmovapd 96(%r11), %ymm10 // A0[12]
	vmulpd	%ymm9, %ymm14, %ymm15
	vaddpd	%ymm5, %ymm15, %ymm5

	vmovapd 96(%r11, %r12, 1), %ymm11 // A1[12]
	vmulpd	%ymm8, %ymm12, %ymm15
	vaddpd	%ymm3, %ymm15, %ymm3
	vshufpd $ 0x5, %ymm12, %ymm12, %ymm14
	addq	$ 128, %r13
	vmulpd	%ymm9, %ymm12, %ymm15
	vaddpd	%ymm7, %ymm15, %ymm7

	addq	$ 128, %r11
	vmulpd	%ymm8, %ymm14, %ymm15
	vaddpd	%ymm2, %ymm15, %ymm2
	vmulpd	%ymm9, %ymm14, %ymm15
	vaddpd	%ymm6, %ymm15, %ymm6


	// unroll 3
	vmovapd 0(%r13), %ymm12 // B[0]
	vmulpd	%ymm10, %ymm13, %ymm15
	vaddpd	%ymm0, %ymm15, %ymm0
	vshufpd $ 0x5, %ymm13, %ymm13, %ymm14
	vmulpd	%ymm11, %ymm13, %ymm15
	vaddpd	%ymm4, %ymm15, %ymm4

	vperm2f128 $ 0x1, %ymm14, %ymm14, %ymm13
	vmulpd	%ymm10, %ymm14, %ymm15
	vaddpd	%ymm1, %ymm15, %ymm1
	vmovapd 0(%r11), %ymm8 // A0[0]
	vmulpd	%ymm11, %ymm14, %ymm15
	vaddpd	%ymm5, %ymm15, %ymm5

	vmovapd 0(%r11, %r12, 1), %ymm9 // A1[0]
	vmulpd	%ymm10, %ymm13, %ymm15
	vaddpd	%ymm3, %ymm15, %ymm3
	vshufpd $ 0x5, %ymm13, %ymm13, %ymm14
	vmulpd	%ymm11, %ymm13, %ymm15
	vaddpd	%ymm7, %ymm15, %ymm7

	vmulpd	%ymm10, %ymm14, %ymm15
	vaddpd	%ymm2, %ymm15, %ymm2
	vmulpd	%ymm11, %ymm14, %ymm15
	vaddpd	%ymm6, %ymm15, %ymm6

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovapd 32(%r13), %ymm13 // B[4]
	vmulpd	%ymm8, %ymm12, %ymm15
	vaddpd	%ymm0, %ymm15, %ymm0
	vshufpd $ 0x5, %ymm12, %ymm12, %ymm14
	vmulpd	%ymm9, %ymm12, %ymm15
	vaddpd	%ymm4, %ymm15, %ymm4

	vperm2f128 $ 0x1, %ymm14, %ymm14, %ymm12
	vmulpd	%ymm8, %ymm14, %ymm15
	vaddpd	%ymm1, %ymm15, %ymm1
	vmovapd 32(%r11), %ymm10 // A0[4]
	vmulpd	%ymm9, %ymm14, %ymm15
	vaddpd	%ymm5, %ymm15, %ymm5

	vmovapd 32(%r11, %r12, 1), %ymm11 // A1[4]
	vmulpd	%ymm8, %ymm12, %ymm15
	vaddpd	%ymm3, %ymm15, %ymm3
	vshufpd $ 0x5, %ymm12, %ymm12, %ymm14
	vmulpd	%ymm9, %ymm12, %ymm15
	vaddpd	%ymm7, %ymm15, %ymm7

	subl	$ 4, %r10d
	vmulpd	%ymm8, %ymm14, %ymm15
	vaddpd	%ymm2, %ymm15, %ymm2
	vmulpd	%ymm9, %ymm14, %ymm15
	vaddpd	%ymm6, %ymm15, %ymm6

	// unroll 1
	vmovapd 64(%r13), %ymm12 // B[8]
	vmulpd	%ymm10, %ymm13, %ymm15
	vaddpd	%ymm0, %ymm15, %ymm0
	vshufpd $ 0x5, %ymm13, %ymm13, %ymm14
	vmulpd	%ymm11, %ymm13, %ymm15
	vaddpd	%ymm4, %ymm15, %ymm4

	vperm2f128 $ 0x1, %ymm14, %ymm14, %ymm13
	vmulpd	%ymm10, %ymm14, %ymm15
	vaddpd	%ymm1, %ymm15, %ymm1
	vmovapd 64(%r11), %ymm8 // A0[8]
	vmulpd	%ymm11, %ymm14, %ymm15
	vaddpd	%ymm5, %ymm15, %ymm5

	vmovapd 64(%r11, %r12, 1), %ymm9 // A1[8]
	vmulpd	%ymm10, %ymm13, %ymm15
	vaddpd	%ymm3, %ymm15, %ymm3
	vshufpd $ 0x5, %ymm13, %ymm13, %ymm14
	vmulpd	%ymm11, %ymm13, %ymm15
	vaddpd	%ymm7, %ymm15, %ymm7

	vmulpd	%ymm10, %ymm14, %ymm15
	vaddpd	%ymm2, %ymm15, %ymm2
	vmulpd	%ymm11, %ymm14, %ymm15
	vaddpd	%ymm6, %ymm15, %ymm6

	// unroll 2
	vmovapd 96(%r13), %ymm13 // B[12]
	vmulpd	%ymm8, %ymm12, %ymm15
	vaddpd	%ymm0, %ymm15, %ymm0
	vshufpd $ 0x5, %ymm12, %ymm12, %ymm14
	vmulpd	%ymm9, %ymm12, %ymm15
	vaddpd	%ymm4, %ymm15, %ymm4

	vperm2f128 $ 0x1, %ymm14, %ymm14, %ymm12
	vmulpd	%ymm8, %ymm14, %ymm15
	vaddpd	%ymm1, %ymm15, %ymm1
	vmovapd 96(%r11), %ymm10 // A0[12]
	vmulpd	%ymm9, %ymm14, %ymm15
	vaddpd	%ymm5, %ymm15, %ymm5

	vmovapd 96(%r11, %r12, 1), %ymm11 // A1[12]
	vmulpd	%ymm8, %ymm12, %ymm15
	vaddpd	%ymm3, %ymm15, %ymm3
	vshufpd $ 0x5, %ymm12, %ymm12, %ymm14
	addq	$ 128, %r13
	vmulpd	%ymm9, %ymm12, %ymm15
	vaddpd	%ymm7, %ymm15, %ymm7
	addq	$ 128, %r11

	vmulpd	%ymm8, %ymm14, %ymm15
	vaddpd	%ymm2, %ymm15, %ymm2
	vmulpd	%ymm9, %ymm14, %ymm15
	vaddpd	%ymm6, %ymm15, %ymm6


	// unroll 3
//	vmovapd 0(%r13), %ymm12 // B[0]
	vmulpd	%ymm10, %ymm13, %ymm15
	vaddpd	%ymm0, %ymm15, %ymm0
	vshufpd $ 0x5, %ymm13, %ymm13, %ymm14
	vmulpd	%ymm11, %ymm13, %ymm15
	vaddpd	%ymm4, %ymm15, %ymm4

	vperm2f128 $ 0x1, %ymm14, %ymm14, %ymm13
	vmulpd	%ymm10, %ymm14, %ymm15
	vaddpd	%ymm1, %ymm15, %ymm1
//	vmovapd 0(%r11), %ymm8 // A0[0]
	vmulpd	%ymm11, %ymm14, %ymm15
	vaddpd	%ymm5, %ymm15, %ymm5

//	vmovapd 0(%r11, %r12, 1), %ymm9 // A1[0]
	vmulpd	%ymm10, %ymm13, %ymm15
	vaddpd	%ymm3, %ymm15, %ymm3
	vshufpd $ 0x5, %ymm13, %ymm13, %ymm14
	vmulpd	%ymm11, %ymm13, %ymm15
	vaddpd	%ymm7, %ymm15, %ymm7

	vmulpd	%ymm10, %ymm14, %ymm15
	vaddpd	%ymm2, %ymm15, %ymm2
	vmulpd	%ymm11, %ymm14, %ymm15
	vaddpd	%ymm6, %ymm15, %ymm6


//	cmpl	$ 3, %r10d
	jmp		2f


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	vmovapd 0(%r13), %ymm12 // B[0]
	vmovapd 0(%r11), %ymm8 // A0[0]
	vmovapd 0(%r11, %r12, 1), %ymm9 // A1[0]
	vmulpd	%ymm8, %ymm12, %ymm15
	vaddpd	%ymm0, %ymm15, %ymm0
	vshufpd $ 0x5, %ymm12, %ymm12, %ymm14
	vmulpd	%ymm9, %ymm12, %ymm15
	vaddpd	%ymm4, %ymm15, %ymm4
	addq	$ 32, %r11

	vmulpd	%ymm8, %ymm14, %ymm15
	vaddpd	%ymm1, %ymm15, %ymm1
	addq	$ 32, %r13
	vmulpd	%ymm9, %ymm14, %ymm15
	vaddpd	%ymm5, %ymm15, %ymm5

	vperm2f128 $ 0x1, %ymm14, %ymm14, %ymm14
	vmulpd	%ymm8, %ymm14, %ymm15
	vaddpd	%ymm3, %ymm15, %ymm3
	vmulpd	%ymm9, %ymm14, %ymm15
	vaddpd	%ymm7, %ymm15, %ymm7

	vshufpd $ 0x5, %ymm14, %ymm14, %ymm14
	vmulpd	%ymm8, %ymm14, %ymm15
	vaddpd	%ymm2, %ymm15, %ymm2
	subl	$ 1, %r10d
	vmulpd	%ymm9, %ymm14, %ymm15
	vaddpd	%ymm6, %ymm15, %ymm6

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_8x4_lib4)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- 4*sda*sizeof(double)
// r13   <- B
// r14   <- 4*sdb*sizeof(double)
// r15   <- dirty
// rax   <- dirty
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

//
// output arguments:
// r10d  <- k
// r11   <- A+4*sda*sizeof(double)
// r12   <- 4*sda*sizeof(double)
// r13   <- B+(k/4)*sdb*sizeof(double)+(k%4)
// r14   <- 4*sdb*sizeof(double)
// r15   <- dirty
// rax   <- dirty
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_8X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_8x4_lib4)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	// preload
	vmovapd 0(%r11), %ymm8 // A0[0]
	vmovapd 0(%r11, %r12, 1), %ymm9 // A1[0]

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	prefetcht0	0(%r13, %r14, 2) // software prefetch
	prefetcht0	64(%r13, %r14, 2) // software prefetch

	// unroll 0
	vbroadcastsd	0(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vmovapd			32(%r11), %ymm10 // A0

	vbroadcastsd	32(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vmovapd			32(%r11, %r12, 1), %ymm11 // A1

	vbroadcastsd	64(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6

	vbroadcastsd	96(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7
	subl	$ 4, %r10d

	// unroll 1
	vbroadcastsd	8(%r13), %ymm12
	vmulpd			%ymm10, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm11, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vmovapd			64(%r11), %ymm8 // A0

	vbroadcastsd	40(%r13), %ymm12
	vmulpd			%ymm10, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm11, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vmovapd			64(%r11, %r12, 1), %ymm9 // A1

	vbroadcastsd	72(%r13), %ymm12
	vmulpd			%ymm10, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm11, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6

	vbroadcastsd	104(%r13), %ymm12
	vmulpd			%ymm10, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm11, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7

	// unroll 2
	vbroadcastsd	16(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vmovapd			96(%r11), %ymm10 // A0

	vbroadcastsd	48(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vmovapd			96(%r11, %r12, 1), %ymm11 // A1

	vbroadcastsd	80(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6

	vbroadcastsd	112(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7
	addq	$ 128, %r11

	// unroll 3
	vbroadcastsd	24(%r13), %ymm12
	vmulpd			%ymm10, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm11, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vmovapd			0(%r11), %ymm8 // A0

	vbroadcastsd	56(%r13), %ymm12
	vmulpd			%ymm10, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm11, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vmovapd			0(%r11, %r12, 1), %ymm9 // A1

	vbroadcastsd	88(%r13), %ymm12
	vmulpd			%ymm10, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm11, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6

	vbroadcastsd	120(%r13), %ymm12
	vmulpd			%ymm10, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	addq	%r14, %r13
	vmulpd			%ymm11, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7


	cmpl	$ 4, %r10d
	jg		1b // main loop

0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vmovapd			32(%r11), %ymm10 // A0

	vbroadcastsd	32(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vmovapd			32(%r11, %r12, 1), %ymm11 // A1

	vbroadcastsd	64(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6

	vbroadcastsd	96(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7
	subl	$ 4, %r10d

	// unroll 1
	vbroadcastsd	8(%r13), %ymm12
	vmulpd			%ymm10, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm11, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vmovapd			64(%r11), %ymm8 // A0

	vbroadcastsd	40(%r13), %ymm12
	vmulpd			%ymm10, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm11, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vmovapd			64(%r11, %r12, 1), %ymm9 // A1

	vbroadcastsd	72(%r13), %ymm12
	vmulpd			%ymm10, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm11, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6

	vbroadcastsd	104(%r13), %ymm12
	vmulpd			%ymm10, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm11, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7

	// unroll 2
	vbroadcastsd	16(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vmovapd			96(%r11), %ymm10 // A0

	vbroadcastsd	48(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vmovapd			96(%r11, %r12, 1), %ymm11 // A1

	vbroadcastsd	80(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6

	vbroadcastsd	112(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7
	addq	$ 128, %r11

	// unroll 3
	vbroadcastsd	24(%r13), %ymm12
	vmulpd			%ymm10, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm11, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
//	vmovapd			0(%r11), %ymm8 // A0

	vbroadcastsd	56(%r13), %ymm12
	vmulpd			%ymm10, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm11, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
//	vmovapd			0(%r11, %r12, 1), %ymm9 // A1

	vbroadcastsd	88(%r13), %ymm12
	vmulpd			%ymm10, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm11, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6

	vbroadcastsd	120(%r13), %ymm12
	vmulpd			%ymm10, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	addq	%r14, %r13
	vmulpd			%ymm11, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7


	jmp		2f


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	vmovapd			0(%r11), %ymm8 // A0[0]
	vmovapd 		0(%r11, %r12, 1), %ymm9 // A1[0]
	vbroadcastsd	0(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4

	vbroadcastsd	32(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	addq	$ 32, %r11

	vbroadcastsd	64(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6
	subl	$ 1, %r10d

	vbroadcastsd	96(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7
	addq	$ 8, %r13

	cmpl	$ 0, %r10d
	jg		3b // clean up loop

2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_8x4_lib4)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// r13   <- 4*sdb*sizeof(double)
// r14   <= dirty
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

//
// output arguments:
// r10d  <- 0
// r11   <- A+4*k*sizeof(double)
// r12   <- B+(k/4)*sdb*sizeof(double)+(k%4)
// r13   <- 4*sdb*sizeof(double)
// r14   <= dirty
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_4X8_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_4x8_lib4)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	// preload
	vmovapd 		0(%r11), %ymm13 // A

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	prefetcht0	0(%r12, %r13, 2) // software prefetch
	prefetcht0	64(%r12, %r13, 2) // software prefetch
	prefetcht0	128(%r12, %r13, 2) // software prefetch
	prefetcht0	192(%r12, %r13, 2) // software prefetch

	// unroll 0
	vbroadcastsd	0(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
	vmovapd			32(%r11), %ymm14 // A
	vbroadcastsd	32(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	vbroadcastsd	64(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	vbroadcastsd	96(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm3, %ymm15, %ymm3
	vbroadcastsd	128(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm4, %ymm15, %ymm4
	vbroadcastsd	160(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5
	vbroadcastsd	192(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6
	vbroadcastsd	224(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm7, %ymm15, %ymm7
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	8(%r12), %ymm12 // B
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
	vmovapd			64(%r11), %ymm13 // A
	vbroadcastsd	40(%r12), %ymm12 // B
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	vbroadcastsd	72(%r12), %ymm12 // B
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	vbroadcastsd	104(%r12), %ymm12 // B
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm3, %ymm15, %ymm3
	vbroadcastsd	136(%r12), %ymm12 // B
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm4, %ymm15, %ymm4
	vbroadcastsd	168(%r12), %ymm12 // B
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5
	vbroadcastsd	200(%r12), %ymm12 // B
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6
	vbroadcastsd	232(%r12), %ymm12 // B
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm7, %ymm15, %ymm7
	addq	$ 128, %r11

	// unroll 0
	vbroadcastsd	16(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
	vmovapd			-32(%r11), %ymm14 // A
	vbroadcastsd	48(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	vbroadcastsd	80(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	vbroadcastsd	112(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm3, %ymm15, %ymm3
	vbroadcastsd	144(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm4, %ymm15, %ymm4
	vbroadcastsd	176(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5
	vbroadcastsd	208(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6
	vbroadcastsd	240(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm7, %ymm15, %ymm7

	// unroll 0
	vbroadcastsd	24(%r12), %ymm12 // B
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
	vmovapd			0(%r11), %ymm13 // A
	vbroadcastsd	56(%r12), %ymm12 // B
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	vbroadcastsd	88(%r12), %ymm12 // B
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	vbroadcastsd	120(%r12), %ymm12 // B
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm3, %ymm15, %ymm3
	vbroadcastsd	152(%r12), %ymm12 // B
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm4, %ymm15, %ymm4
	vbroadcastsd	184(%r12), %ymm12 // B
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5
	vbroadcastsd	216(%r12), %ymm12 // B
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6
	vbroadcastsd	248(%r12), %ymm12 // B
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm7, %ymm15, %ymm7
	addq	%r13, %r12

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
	vmovapd			32(%r11), %ymm14 // A
	vbroadcastsd	32(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	vbroadcastsd	64(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	vbroadcastsd	96(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm3, %ymm15, %ymm3
	vbroadcastsd	128(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm4, %ymm15, %ymm4
	vbroadcastsd	160(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5
	vbroadcastsd	192(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6
	vbroadcastsd	224(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm7, %ymm15, %ymm7
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	8(%r12), %ymm12 // B
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
	vmovapd			64(%r11), %ymm13 // A
	vbroadcastsd	40(%r12), %ymm12 // B
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	vbroadcastsd	72(%r12), %ymm12 // B
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	vbroadcastsd	104(%r12), %ymm12 // B
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm3, %ymm15, %ymm3
	vbroadcastsd	136(%r12), %ymm12 // B
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm4, %ymm15, %ymm4
	vbroadcastsd	168(%r12), %ymm12 // B
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5
	vbroadcastsd	200(%r12), %ymm12 // B
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6
	vbroadcastsd	232(%r12), %ymm12 // B
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm7, %ymm15, %ymm7
	addq	$ 128, %r11

	// unroll 0
	vbroadcastsd	16(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
	vmovapd			-32(%r11), %ymm14 // A
	vbroadcastsd	48(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	vbroadcastsd	80(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	vbroadcastsd	112(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm3, %ymm15, %ymm3
	vbroadcastsd	144(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm4, %ymm15, %ymm4
	vbroadcastsd	176(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5
	vbroadcastsd	208(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6
	vbroadcastsd	240(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm7, %ymm15, %ymm7

	// unroll 0
	vbroadcastsd	24(%r12), %ymm12 // B
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
//	vmovapd			0(%r11), %ymm13 // A
	vbroadcastsd	56(%r12), %ymm12 // B
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	vbroadcastsd	88(%r12), %ymm12 // B
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	vbroadcastsd	120(%r12), %ymm12 // B
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm3, %ymm15, %ymm3
	vbroadcastsd	152(%r12), %ymm12 // B
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm4, %ymm15, %ymm4
	vbroadcastsd	184(%r12), %ymm12 // B
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5
	vbroadcastsd	216(%r12), %ymm12 // B
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6
	vbroadcastsd	248(%r12), %ymm12 // B
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm7, %ymm15, %ymm7
	addq	%r13, %r12

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
	vbroadcastsd	32(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	vbroadcastsd	64(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	vbroadcastsd	96(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm3, %ymm15, %ymm3
	vbroadcastsd	128(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm4, %ymm15, %ymm4
	vbroadcastsd	160(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5
	vbroadcastsd	192(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6
	vbroadcastsd	224(%r12), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm7, %ymm15, %ymm7

	addq	$ 32, %r11
	addq	$ 8, %r12
	subl	$ 1, %r10d

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_4x8_lib4)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- B
// r12   <- C
// r13   <- 32*sdc
// ymm0  <- [a00 a10 a20 a30]
// ymm1  <- [a01 a11 a21 a31]
// ymm2  <- [a02 a12 a22 a32]
// ymm3  <- [a03 a13 a23 a33]

//
// output arguments:
// r10d  <- 0
// r11   <- ?
// r12   <- ?
// r13   <- 32*sdc
// ymm0  <- [a00 a10 a20 a30]
// ymm1  <- [a01 a11 a21 a31]
// ymm2  <- [a02 a12 a22 a32]
// ymm3  <- [a03 a13 a23 a33]

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEBP_ADD_NN_8X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgebp_add_nn_8x4_lib4)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return

	cmpl	$ 3, %r10d
	jle		2f // cleanup loop

	// main loop
	.p2align 3
1:
	vmovapd			0(%r12), %ymm12
	vmovapd			0(%r12, %r13, 1), %ymm14
	vbroadcastsd	0(%r11), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm12, %ymm12
	vmulpd			%ymm4, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm14, %ymm14
	vbroadcastsd	8(%r11), %ymm13
	subl	$ 4, %r10d
	vmulpd			%ymm1, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm12, %ymm12
	vmulpd			%ymm5, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm14, %ymm14
	vbroadcastsd	16(%r11), %ymm13
	vmulpd			%ymm2, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm12, %ymm12
	vmulpd			%ymm6, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm14, %ymm14
	vbroadcastsd	24(%r11), %ymm13
	vmulpd			%ymm3, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm12, %ymm12
	vmulpd			%ymm7, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm14, %ymm14
	vmovapd			%ymm12, 0(%r12)
	vmovapd			%ymm14, 0(%r12, %r13, 1)

	vmovapd			32(%r12), %ymm12
	vmovapd			32(%r12, %r13, 1), %ymm14
	vbroadcastsd	32(%r11), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm12, %ymm12
	vmulpd			%ymm4, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm14, %ymm14
	vbroadcastsd	40(%r11), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm12, %ymm12
	vmulpd			%ymm5, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm14, %ymm14
	vbroadcastsd	48(%r11), %ymm13
	vmulpd			%ymm2, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm12, %ymm12
	vmulpd			%ymm6, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm14, %ymm14
	vbroadcastsd	56(%r11), %ymm13
	vmulpd			%ymm3, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm12, %ymm12
	vmulpd			%ymm7, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm14, %ymm14
	vmovapd			%ymm12, 32(%r12)
	vmovapd			%ymm14, 32(%r12, %r13, 1)

	vmovapd			64(%r12), %ymm12
	vmovapd			64(%r12, %r13, 1), %ymm14
	vbroadcastsd	64(%r11), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm12, %ymm12
	vmulpd			%ymm4, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm14, %ymm14
	vbroadcastsd	72(%r11), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm12, %ymm12
	vmulpd			%ymm5, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm14, %ymm14
	vbroadcastsd	80(%r11), %ymm13
	vmulpd			%ymm2, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm12, %ymm12
	vmulpd			%ymm6, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm14, %ymm14
	vbroadcastsd	88(%r11), %ymm13
	vmulpd			%ymm3, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm12, %ymm12
	vmulpd			%ymm7, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm14, %ymm14
	vmovapd			%ymm12, 64(%r12)
	vmovapd			%ymm14, 64(%r12, %r13, 1)

	vmovapd			96(%r12), %ymm12
	vmovapd			96(%r12, %r13, 1), %ymm14
	vbroadcastsd	96(%r11), %ymm13
	addq	$ 128, %r11
	vmulpd			%ymm0, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm12, %ymm12
	vmulpd			%ymm4, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm14, %ymm14
	vbroadcastsd	-24(%r11), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm12, %ymm12
	vmulpd			%ymm5, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm14, %ymm14
	vbroadcastsd	-16(%r11), %ymm13
	vmulpd			%ymm2, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm12, %ymm12
	vmulpd			%ymm6, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm14, %ymm14
	vbroadcastsd	-8(%r11), %ymm13
	addq	$ 128, %r12
	vmulpd			%ymm3, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm12, %ymm12
	vmulpd			%ymm7, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm14, %ymm14
	vmovapd			%ymm12, -32(%r12)
	vmovapd			%ymm14, -32(%r12, %r13, 1)

	cmpl	$ 3, %r10d
	jg		1b // main loop

	cmpl	$ 0, %r10d
	jle		0f // return

	// cleanup loop
2:
	vmovapd			0(%r12), %ymm12
	vmovapd			0(%r12, %r13, 1), %ymm14
	vbroadcastsd	0(%r11), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm12, %ymm12
	vmulpd			%ymm4, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm14, %ymm14
	vbroadcastsd	8(%r11), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm12, %ymm12
	vmulpd			%ymm5, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm14, %ymm14
	vbroadcastsd	16(%r11), %ymm13
	vmulpd			%ymm2, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm12, %ymm12
	vmulpd			%ymm6, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm14, %ymm14
	vbroadcastsd	24(%r11), %ymm13
	vmulpd			%ymm3, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm12, %ymm12
	vmulpd			%ymm7, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm14, %ymm14
	vmovapd			%ymm12, 0(%r12)
	vmovapd			%ymm14, 0(%r12, %r13, 1)

	addq	$ 32, %r11
	addq	$ 32, %r12

	subl	$ 1, %r10d
	cmpl	$ 0, %r10d
	jg		2b // main loop

	// return
0:

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgebp_add_nn_8x4_lib4)
#endif





// common inner routine with file scope
//
// edge for B unaligned
//
// input arguments:
// r10   <- k
// r11   <- A
// r12   <- bs*sda*sizeof(double)
// r13   <- B
// r14   <- bs*sdb*sizeof(double)
// r15   <- offB
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm8  <- dirty
// ymm12 <- dirty
// ymm15 <- dirty

//
// output arguments:
// r10   <- k-(4-offB)
// r11   <- A+(4-offB)*bs*sizeof(double)
// r12   <- bs*sda*sizeof(double)
// r13   <- B-offB+bs*sdb*sizeof(double)
// r14   <- bs*sdb*sizeof(double)
// r15   <- offB
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm8  <- dirty
// ymm12 <- dirty
// ymm15 <- dirty


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DGEMM_NN_8X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dgemm_nn_8x4_lib4)
#endif

	cmpl			$ 0, %r15d // offset==0
	jle				2f // end

	cmpl			$ 0, %r10d // k==0
	jle				2f // end

	movl			$ 4, %ebx
	subl			%r15d, %ebx // 4-offsetB
	cmpl			%r10d, %ebx
//	jle				0f
//	movl			%r10d, %ebx // kend=min(k,4-offsetB)
//0:
	cmovgl			%r10d, %ebx // kend=min(k,4-offsetB)

	movl			%r15d, %eax
	sall			$ 3, %eax // offsetB*sizeof(double)
	addq			%rax, %r13 // B+offsetB*sizeof(double)

1:
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r12, 1), %ymm9
	vbroadcastsd	0(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	32(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastsd	64(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6
	vbroadcastsd	96(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7

	subl			$ 1, %r10d // k-1
	subl			$ 1, %ebx // kend-1
	addq			$ 32, %r11 // A0+1*bs*sizeof(float)
	addq			$ 8, %r13 // B+1*sizeof(float)

	cmpl			$ 0, %ebx
	jg				1b

	cmpl			$ 0, %r10d
	jle				2f // end

	addq			%r14, %r13
	subq			$ 32, %r13 // B+bs*(sdb-1)*sizeof(double)

2:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dgemm_nn_8x4_lib4)
#endif





// common inner routine with file scope
//
// edge for B unaligned
//
// input arguments:
// r10   <- k
// r11   <- A
// r12   <- B
// r13   <- bs*sdb*sizeof(double)
// r14   <- offB
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm8  <- dirty
// ymm12 <- dirty
// ymm15 <- dirty

//
// output arguments:
// r10   <- k-(4-offB)
// r11   <- A+(4-offB)*bs*sizeof(double)
// r12   <- B-offB+bs*sdb*sizeof(double)
// r13   <- bs*sdb*sizeof(double)
// r14   <- offB
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm8  <- dirty
// ymm12 <- dirty
// ymm15 <- dirty


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DGEMM_NN_4X8_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dgemm_nn_4x8_lib4)
#endif

	cmpl			$ 0, %r14d // offset==0
	jle				2f // end

	cmpl			$ 0, %r10d // k==0
	jle				2f // end

	movl			$ 4, %r15d
	subl			%r14d, %r15d // 4-offsetB
	cmpl			%r10d, %r15d
//	jle				0f
//	movl			%r10d, %r15d // kend=min(k,4-offsetB)
//0:
	cmovgl			%r10d, %r15d // kend=min(k,4-offsetB)

	movl			%r14d, %eax
	sall			$ 3, %eax // offsetB*sizeof(double)
	addq			%rax, %r12 // B+offsetB*sizeof(double)

1:
	vmovapd			0(%r11), %ymm12
	vbroadcastsd	0(%r12), %ymm13
	vmulpd			%ymm12, %ymm13, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
	vbroadcastsd	32(%r12), %ymm13
	vmulpd			%ymm12, %ymm13, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	vbroadcastsd	64(%r12), %ymm13
	vmulpd			%ymm12, %ymm13, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	vbroadcastsd	96(%r12), %ymm13
	vmulpd			%ymm12, %ymm13, %ymm15
	vaddpd			%ymm3, %ymm15, %ymm3
	vbroadcastsd	128(%r12), %ymm13 // B
	vmulpd			%ymm12, %ymm13, %ymm15
	vaddpd			%ymm4, %ymm15, %ymm4
	vbroadcastsd	160(%r12), %ymm13 // B
	vmulpd			%ymm12, %ymm13, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5
	vbroadcastsd	192(%r12), %ymm13 // B
	vmulpd			%ymm12, %ymm13, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6
	vbroadcastsd	224(%r12), %ymm13 // B
	vmulpd			%ymm12, %ymm13, %ymm15
	vaddpd			%ymm7, %ymm15, %ymm7

	subl			$ 1, %r10d // k-1
	subl			$ 1, %r15d // kend-1
	addq			$ 32, %r11 // A+1*bs*sizeof(float)
	addq			$ 8, %r12 // B+1*sizeof(float)

	cmpl			$ 0, %r15d
	jg				1b

	cmpl			$ 0, %r10d
	jle				2f // end

	addq			%r13, %r12
	subq			$ 32, %r12 // B+bs*(sdb-1)*sizeof(double)

2:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dgemm_nn_4x8_lib4)
#endif





// common inner routine with file scope
//
// edge for B upper triangular
//
// input arguments:
// r10   <- kmax
// r11   <- A
// r12   <- 4*sda*sizeof(double)
// r13   <- B
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- dirty
// ymm12 <- dirty
// ymm15 <- dirty

//
// output arguments:
// r10   <- kmax+4
// r11   <- A+4*4*sizeof(double)
// r12   <- 4*sda*sizeof(double)
// r13   <- B+4*4*sizeof(double)
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- dirty
// ymm12 <- dirty
// ymm15 <- dirty


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NT_RU_8X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nt_ru_8x4_lib4)
#endif

	vbroadcastsd	0(%r13), %ymm12
	vmovapd			0(%r11), %ymm8
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
	vmovapd			0(%r11, %r12, 1), %ymm9
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm4, %ymm15, %ymm4

	vbroadcastsd	32(%r13), %ymm12
	vmovapd			32(%r11), %ymm8
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
	vmovapd			32(%r11, %r12, 1), %ymm9
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm4, %ymm15, %ymm4
	vbroadcastsd	40(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5

	vbroadcastsd	64(%r13), %ymm12
	vmovapd			64(%r11), %ymm8
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
	vmovapd			64(%r11, %r12, 1), %ymm9
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm4, %ymm15, %ymm4
	vbroadcastsd	72(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5
	vbroadcastsd	80(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6

	vbroadcastsd	96(%r13), %ymm12
	vmovapd			96(%r11), %ymm8
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
	vmovapd			96(%r11, %r12, 1), %ymm9
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm4, %ymm15, %ymm4
	vbroadcastsd	104(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5
	vbroadcastsd	112(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6
	vbroadcastsd	120(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm3, %ymm15, %ymm3
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm7, %ymm15, %ymm7

	subl			$ 4, %r10d
	addq			$ 128, %r11
	addq			$ 128, %r13

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nt_ru_8x4_lib4)
#endif





// common inner routine with file scope
//
// edge for B upper triangular
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- 4*sda*sizeof(double)
// r13   <- B
// r15   <- dirty
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- dirty
// ymm12 <- dirty
// ymm15 <- dirty

//
// output arguments:
// r10d  <- max(k-4,0)
// r11   <- A+4*4*sizeof(double)
// r12   <- 4*sda*sizeof(double)
// r13   <- B+4*4*sizeof(double)
// r15   <- dirty
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- dirty
// ymm12 <- dirty
// ymm15 <- dirty


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NT_RU_8X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nt_ru_8x4_vs_lib4)
#endif

	vbroadcastsd	0(%r13), %ymm12
	subl			$ 1, %r10d
	vmovapd			0(%r11), %ymm8
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
	vmovapd			0(%r11, %r12, 1), %ymm9
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm4, %ymm15, %ymm4
	addq			$ 32, %r11
	addq			$ 32, %r13

	cmpl	$ 0, %r10d
	jle		0f

	vbroadcastsd	0(%r13), %ymm12
	subl			$ 1, %r10d
	vmovapd			0(%r11), %ymm8
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
	vmovapd			0(%r11, %r12, 1), %ymm9
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm4, %ymm15, %ymm4
	addq			$ 32, %r11
	vbroadcastsd	8(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	addq			$ 32, %r13
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5

	cmpl	$ 0, %r10d
	jle		0f

	vbroadcastsd	0(%r13), %ymm12
	subl			$ 1, %r10d
	vmovapd			0(%r11), %ymm8
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
	vmovapd			0(%r11, %r12, 1), %ymm9
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm4, %ymm15, %ymm4
	vbroadcastsd	8(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	addq			$ 32, %r11
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5
	vbroadcastsd	16(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	addq			$ 32, %r13
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6

	cmpl	$ 0, %r10d
	jle		0f

	vbroadcastsd	0(%r13), %ymm12
	subl			$ 1, %r10d
	vmovapd			0(%r11), %ymm8
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
	vmovapd			0(%r11, %r12, 1), %ymm9
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm4, %ymm15, %ymm4
	vbroadcastsd	8(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5
	vbroadcastsd	16(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	addq			$ 32, %r11
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6
	vbroadcastsd	24(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm3, %ymm15, %ymm3
	addq			$ 32, %r13
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm7, %ymm15, %ymm7

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nt_ru_8x4_vs_lib4)
#endif





// common inner routine with file scope
//
// edge for B upper triangular
//
// input arguments:
// r10   <- kmax
// r11   <- A
// r12   <- 4*sda*sizeof(double)
// r13   <- B
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- dirty
// ymm12 <- dirty
// ymm15 <- dirty

//
// output arguments:
// r10   <- kmax+4
// r11   <- A+4*4*sizeof(double)
// r12   <- 4*sda*sizeof(double)
// r13   <- B+4*4*sizeof(double)
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- dirty
// ymm12 <- dirty
// ymm15 <- dirty


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NT_RU_ONE_8X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nt_ru_one_8x4_lib4)
#endif

	vmovapd			0(%r11), %ymm8
	vaddpd			%ymm0, %ymm8, %ymm0
	vmovapd			0(%r11, %r12, 1), %ymm9
	vaddpd			%ymm4, %ymm9, %ymm4

	vbroadcastsd	32(%r13), %ymm12
	vmovapd			32(%r11), %ymm8
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
	vmovapd			32(%r11, %r12, 1), %ymm9
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm4, %ymm15, %ymm4
	vaddpd			%ymm1, %ymm8, %ymm1
	vaddpd			%ymm5, %ymm9, %ymm5

	vbroadcastsd	64(%r13), %ymm12
	vmovapd			64(%r11), %ymm8
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
	vmovapd			64(%r11, %r12, 1), %ymm9
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm4, %ymm15, %ymm4
	vbroadcastsd	72(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5
	vaddpd			%ymm2, %ymm8, %ymm2
	vaddpd			%ymm6, %ymm9, %ymm6

	vbroadcastsd	96(%r13), %ymm12
	vmovapd			96(%r11), %ymm8
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
	vmovapd			96(%r11, %r12, 1), %ymm9
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm4, %ymm15, %ymm4
	vbroadcastsd	104(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5
	vbroadcastsd	112(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6
	vaddpd			%ymm3, %ymm8, %ymm3
	vaddpd			%ymm7, %ymm9, %ymm7

	subl			$ 4, %r10d
	addq			$ 128, %r11
	addq			$ 128, %r13

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nt_ru_one_8x4_lib4)
#endif





// common inner routine with file scope
//
// edge for B upper triangular
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- 4*sda*sizeof(double)
// r13   <- B
// r15   <- dirty
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- dirty
// ymm12 <- dirty
// ymm15 <- dirty

//
// output arguments:
// r10d  <- max(k-4,0)
// r11   <- A+4*4*sizeof(double)
// r12   <- 4*sda*sizeof(double)
// r13   <- B+4*4*sizeof(double)
// r15   <- dirty
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- dirty
// ymm12 <- dirty
// ymm15 <- dirty


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NT_RU_ONE_8X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nt_ru_one_8x4_vs_lib4)
#endif

	subl			$ 1, %r10d
	vmovapd			0(%r11), %ymm8
	vaddpd			%ymm0, %ymm8, %ymm0
	vmovapd			0(%r11, %r12, 1), %ymm9
	vaddpd			%ymm4, %ymm9, %ymm4
	addq			$ 32, %r11
	addq			$ 32, %r13

	cmpl	$ 0, %r10d
	jle		0f

	vbroadcastsd	0(%r13), %ymm12
	subl			$ 1, %r10d
	vmovapd			0(%r11), %ymm8
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
	vmovapd			0(%r11, %r12, 1), %ymm9
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm4, %ymm15, %ymm4
	addq			$ 32, %r11
	vaddpd			%ymm1, %ymm8, %ymm1
	addq			$ 32, %r13
	vaddpd			%ymm5, %ymm9, %ymm5

	cmpl	$ 0, %r10d
	jle		0f

	vbroadcastsd	0(%r13), %ymm12
	subl			$ 1, %r10d
	vmovapd			0(%r11), %ymm8
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
	vmovapd			0(%r11, %r12, 1), %ymm9
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm4, %ymm15, %ymm4
	vbroadcastsd	8(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	addq			$ 32, %r11
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5
	vaddpd			%ymm2, %ymm8, %ymm2
	addq			$ 32, %r13
	vaddpd			%ymm6, %ymm9, %ymm6

	cmpl	$ 0, %r10d
	jle		0f

	vbroadcastsd	0(%r13), %ymm12
	subl			$ 1, %r10d
	vmovapd			0(%r11), %ymm8
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
	vmovapd			0(%r11, %r12, 1), %ymm9
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm4, %ymm15, %ymm4
	vbroadcastsd	8(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5
	vbroadcastsd	16(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	addq			$ 32, %r11
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6
	vaddpd			%ymm3, %ymm8, %ymm3
	addq			$ 32, %r13
	vaddpd			%ymm7, %ymm9, %ymm7

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nt_ru_one_8x4_vs_lib4)
#endif





// common inner routine with file scope
//
// edge for B lower triangular
//
// input arguments:
// r10   <- k
// r11   <- A0
// r12   <- bs*sda*sizeof(double)
// r13   <- B
// r14   <- bs*sdb*sizeof(double)
// r15   <- offB
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm12 <- dirty
// ymm15 <- dirty

//
// output arguments:
// r10   <- k-(4-offB)
// r11   <- A+(4-offB)*bs*sizeof(double)
// r12   <- bs*sda*sizeof(double)
// r13   <- B-offB+bs*sdb*sizeof(double)
// r14   <- bs*sdb*sizeof(double)
// r15   <- offB
// rax   <- dirty
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm12 <- dirty
// ymm15 <- dirty


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NN_RL_8X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nn_rl_8x4_lib4)
#endif

	cmpl	$ 0, %r15d
	jg		0f

	// offB==0

	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r12, 1), %ymm9
	vbroadcastsd	0(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4

	vmovapd			32(%r11), %ymm8
	vmovapd			32(%r11, %r12, 1), %ymm9
	vbroadcastsd	8(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	40(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5

	vmovapd			64(%r11), %ymm8
	vmovapd			64(%r11, %r12, 1), %ymm9
	vbroadcastsd	16(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	48(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastsd	80(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6

	vmovapd			96(%r11), %ymm8
	vmovapd			96(%r11, %r12, 1), %ymm9
	vbroadcastsd	24(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	56(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastsd	88(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6
	vbroadcastsd	120(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7

	subl			$ 4, %r10d // k-4
	addq			$ 128, %r11 // A0+4*bs*sizeof(double)
	addq			%r14, %r13 // B+bs*sdb*sizeof(double)

	jmp		3f

0:
	cmpl	$ 1, %r15d
	jg		1f

	// offB==1

	addq			$ 8, %r13 // B+1*sizeof(double)

	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r12, 1), %ymm9
	vbroadcastsd	0(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4

	vmovapd			32(%r11), %ymm8
	vmovapd			32(%r11, %r12, 1), %ymm9
	vbroadcastsd	8(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	40(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5

	vmovapd			64(%r11), %ymm8
	vmovapd			64(%r11, %r12, 1), %ymm9
	vbroadcastsd	16(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	48(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastsd	80(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6

	subl			$ 3, %r10d // k-3
	addq			$ 96, %r11 // A0+3*bs*sizeof(double)
	addq			%r14, %r13
	subq			$ 8, %r13 // B+bs*sdb*sizeof(double)-1

	jmp		3f

1:
	cmpl	$ 2, %r15d
	jg		2f

	// offB==2

	addq			$ 16, %r13 // B+2*sizeof(double)

	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r12, 1), %ymm9
	vbroadcastsd	0(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4

	vmovapd			32(%r11), %ymm8
	vmovapd			32(%r11, %r12, 1), %ymm9
	vbroadcastsd	8(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	40(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5

	subl			$ 2, %r10d // k-2
	addq			$ 64, %r11 // A0+2*bs*sizeof(double)
	addq			%r14, %r13
	subq			$ 16, %r13 // B+bs*sdb*sizeof(double)-2

	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r12, 1), %ymm9
	vbroadcastsd	0(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	32(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastsd	64(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6

	vmovapd			32(%r11), %ymm8
	vmovapd			32(%r11, %r12, 1), %ymm9
	vbroadcastsd	8(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	40(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastsd	72(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6
	vbroadcastsd	104(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7

	vmovapd			64(%r11), %ymm8
	vmovapd			64(%r11, %r12, 1), %ymm9
	vbroadcastsd	16(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	48(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastsd	80(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6
	vbroadcastsd	112(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7

	vmovapd			96(%r11), %ymm8
	vmovapd			96(%r11, %r12, 1), %ymm9
	vbroadcastsd	24(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	56(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastsd	88(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6
	vbroadcastsd	120(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7

	subl			$ 4, %r10d // k-4
	addq			$ 128, %r11 // A0+4*bs*sizeof(double)
	addq			%r14, %r13 // B+bs*sdb*sizeof(double)

	jmp		3f

2:
	// offB==3

	addq			$ 24, %r13 // B+3*sizeof(double)

	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r12, 1), %ymm9
	vbroadcastsd	0(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A0+1*bs*sizeof(double)
	addq			%r14, %r13
	subq			$ 24, %r13 // B+bs*sdb*sizeof(double)-3

	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r12, 1), %ymm9
	vbroadcastsd	0(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	32(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5

	vmovapd			32(%r11), %ymm8
	vmovapd			32(%r11, %r12, 1), %ymm9
	vbroadcastsd	8(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	40(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastsd	72(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6

	vmovapd			64(%r11), %ymm8
	vmovapd			64(%r11, %r12, 1), %ymm9
	vbroadcastsd	16(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	48(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastsd	80(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6
	vbroadcastsd	112(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7

	vmovapd			96(%r11), %ymm8
	vmovapd			96(%r11, %r12, 1), %ymm9
	vbroadcastsd	24(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	56(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastsd	88(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6
	vbroadcastsd	120(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7

	subl			$ 4, %r10d // k-4
	addq			$ 128, %r11 // A0+4*bs*sizeof(double)
	addq			%r14, %r13 // B+bs*sdb*sizeof(double)

3:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nn_rl_8x4_lib4)
#endif





// common inner routine with file scope
//
// edge for B lower triangular
//
// input arguments:
// r10   <- k
// r11   <- A0
// r12   <- bs*sda*sizeof(double)
// r13   <- B
// r14   <- bs*sdb*sizeof(double)
// r15   <- offB
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm12 <- dirty
// ymm15 <- dirty

//
// output arguments:
// r10   <- k-(4-offB)
// r11   <- A+(4-offB)*bs*sizeof(double)
// r12   <- bs*sda*sizeof(double)
// r13   <- B-offB+bs*sdb*sizeof(double)
// r14   <- bs*sdb*sizeof(double)
// r15   <- offB
// rax   <- dirty
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm12 <- dirty
// ymm15 <- dirty


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NN_RL_8X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nn_rl_8x4_vs_lib4)
#endif

	cmpl			$ 0, %r10d
	jle				3f // end

	cmpl			$ 0, %r15d
	jg				0f // offB>0

	// offB==0

	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r12, 1), %ymm9
	vbroadcastsd	0(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A0+1*bs*sizeof(double)
	addq			$ 8, %r13 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r12, 1), %ymm9
	vbroadcastsd	0(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	32(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A0+1*bs*sizeof(double)
	addq			$ 8, %r13 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r12, 1), %ymm9
	vbroadcastsd	0(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	32(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastsd	64(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A0+1*bs*sizeof(double)
	addq			$ 8, %r13 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r12, 1), %ymm9
	vbroadcastsd	0(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	32(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastsd	64(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6
	vbroadcastsd	96(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A0+1*bs*sizeof(double)
	addq			%r14, %r13
	subq			$ 24, %r13 // B+bs*sdb*sizeof(double)-(bs-1)*sizeof(double)

	jmp				3f // end

0:
	cmpl			$ 1, %r15d
	jg				1f // offB>1

	// offB==1

	addq			$ 8, %r13 // B+1*sizeof(double)

	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r12, 1), %ymm9
	vbroadcastsd	0(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A0+1*bs*sizeof(double)
	addq			$ 8, %r13 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r12, 1), %ymm9
	vbroadcastsd	0(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	32(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A0+1*bs*sizeof(double)
	addq			$ 8, %r13 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r12, 1), %ymm9
	vbroadcastsd	0(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	32(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastsd	64(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A0+1*bs*sizeof(double)
	addq			%r14, %r13
	subq			$ 24, %r13 // B+bs*sdb*sizeof(double)-(bs-1)*sizeof(double)

	jmp				3f // end

1:
	cmpl			$ 2, %r15d
	jg				2f // offB>2

	// offB==2

	addq			$ 16, %r13 // B+2*sizeof(double)

	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r12, 1), %ymm9
	vbroadcastsd	0(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A0+1*bs*sizeof(double)
	addq			$ 8, %r13 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r12, 1), %ymm9
	vbroadcastsd	0(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	32(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5

	subl			$ 1, %r10d // k-2
	addq			$ 32, %r11 // A0+1*bs*sizeof(double)
	addq			%r14, %r13
	subq			$ 24, %r13 // B+bs*sdb*sizeof(double)-(bs-1)*sizeof(double)

	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r12, 1), %ymm9
	vbroadcastsd	0(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	32(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastsd	64(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A0+1*bs*sizeof(double)
	addq			$ 8, %r13 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r12, 1), %ymm9
	vbroadcastsd	0(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	32(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastsd	64(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6
	vbroadcastsd	96(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A0+1*bs*sizeof(double)
	addq			$ 8, %r13 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r12, 1), %ymm9
	vbroadcastsd	0(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	32(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastsd	64(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6
	vbroadcastsd	96(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A0+1*bs*sizeof(double)
	addq			$ 8, %r13 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r12, 1), %ymm9
	vbroadcastsd	0(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	32(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastsd	64(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6
	vbroadcastsd	96(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A0+1*bs*sizeof(double)
	addq			%r14, %r13
	subq			$ 24, %r13 // B+bs*sdb*sizeof(double)-(bs-1)*sizeof(double)

	jmp				3f

2:
	// offB==3

	addq			$ 24, %r13 // B+3*sizeof(double)

	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r12, 1), %ymm9
	vbroadcastsd	0(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A0+1*bs*sizeof(double)
	addq			%r14, %r13
	subq			$ 24, %r13 // B+bs*sdb*sizeof(double)-(bs-1)*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r12, 1), %ymm9
	vbroadcastsd	0(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	32(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A0+1*bs*sizeof(double)
	addq			$ 8, %r13 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r12, 1), %ymm9
	vbroadcastsd	0(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	32(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastsd	64(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A0+1*bs*sizeof(double)
	addq			$ 8, %r13 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r12, 1), %ymm9
	vbroadcastsd	0(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	32(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastsd	64(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6
	vbroadcastsd	96(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A0+1*bs*sizeof(double)
	addq			$ 8, %r13 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r12, 1), %ymm9
	vbroadcastsd	0(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	32(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastsd	64(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6
	vbroadcastsd	96(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7

	subl			$ 1, %r10d // k-4
	addq			$ 32, %r11 // A0+1*bs*sizeof(double)
	addq			%r14, %r13
	subq			$ 24, %r13 // B+bs*sdb*sizeof(double)-(bs-1)*sizeof(double)

3:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nn_rl_8x4_vs_lib4)
#endif





// common inner routine with file scope
//
// edge for B lower triangular
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- lda
// r13   <- B
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NT_RL_8X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nt_rl_8x4_lib4)
#endif
	
	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vmovapd			0(%r11, %r12), %ymm14 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm4, %ymm15, %ymm4
	vbroadcastsd	8(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5
	vbroadcastsd	16(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6
	vbroadcastsd	24(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm3, %ymm15, %ymm3
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm7, %ymm15, %ymm7

	// unroll 1
	vmovapd			32(%r11), %ymm13 // A
	vmovapd			32(%r11, %r12), %ymm14 // A
	vbroadcastsd	40(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5
	vbroadcastsd	48(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6
	vbroadcastsd	56(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm3, %ymm15, %ymm3
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm7, %ymm15, %ymm7

	// unroll 2
	vmovapd			64(%r11), %ymm13 // A
	vmovapd			64(%r11, %r12), %ymm14 // A
	vbroadcastsd	80(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6
	vbroadcastsd	88(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm3, %ymm15, %ymm3
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm7, %ymm15, %ymm7

	// unroll 3
	vmovapd			96(%r11), %ymm13 // A
	vmovapd			96(%r11, %r12), %ymm14 // A
	vbroadcastsd	120(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm3, %ymm15, %ymm3
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm7, %ymm15, %ymm7

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r13

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nt_rl_8x4_lib4)
#endif





// common inner routine with file scope
//
// edge for B upper triangular
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- lda
// r13   <- B
// r14   <- n1
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NT_RL_8X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nt_rl_8x4_vs_lib4)
#endif
	
	cmpl	$ 0, %r14d
	jle		0f // end

	cmpl	$ 4, %r14d
	jl		1f // end

	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vmovapd			0(%r11, %r12), %ymm14 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm4, %ymm15, %ymm4
	vbroadcastsd	8(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5
	vbroadcastsd	16(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6
	vbroadcastsd	24(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm3, %ymm15, %ymm3
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm7, %ymm15, %ymm7

	// unroll 1
	vmovapd			32(%r11), %ymm13 // A
	vmovapd			32(%r11, %r12), %ymm14 // A
	vbroadcastsd	40(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5
	vbroadcastsd	48(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6
	vbroadcastsd	56(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm3, %ymm15, %ymm3
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm7, %ymm15, %ymm7

	// unroll 2
	vmovapd			64(%r11), %ymm13 // A
	vmovapd			64(%r11, %r12), %ymm14 // A
	vbroadcastsd	80(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6
	vbroadcastsd	88(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm3, %ymm15, %ymm3
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm7, %ymm15, %ymm7

	// unroll 3
	vmovapd			96(%r11), %ymm13 // A
	vmovapd			96(%r11, %r12), %ymm14 // A
	vbroadcastsd	120(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm3, %ymm15, %ymm3
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm7, %ymm15, %ymm7

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r13

	jmp		0f

1:

	cmpl	$ 3, %r14d
	jl		2f // end

	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vmovapd			0(%r11, %r12), %ymm14 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm4, %ymm15, %ymm4
	vbroadcastsd	8(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5
	vbroadcastsd	16(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6

	// unroll 1
	vmovapd			32(%r11), %ymm13 // A
	vmovapd			32(%r11, %r12), %ymm14 // A
	vbroadcastsd	40(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5
	vbroadcastsd	48(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6

	// unroll 2
	vmovapd			64(%r11), %ymm13 // A
	vmovapd			64(%r11, %r12), %ymm14 // A
	vbroadcastsd	80(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6

	// unroll 3

	subl	$ 3, %r10d
	addq	$ 96, %r11
	addq	$ 96, %r13

	jmp		0f

2:

	cmpl	$ 2, %r14d
	jl		3f // end

	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vmovapd			0(%r11, %r12), %ymm14 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm4, %ymm15, %ymm4
	vbroadcastsd	8(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5

	// unroll 1
	vmovapd			32(%r11), %ymm13 // A
	vmovapd			32(%r11, %r12), %ymm14 // A
	vbroadcastsd	40(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5

	// unroll 2

	// unroll 3

	subl	$ 2, %r10d
	addq	$ 64, %r11
	addq	$ 64, %r13

	jmp		0f

3:

//	cmpl	$ 1, %r14d
//	jl		0f // end

	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vmovapd			0(%r11, %r12), %ymm14 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm4, %ymm15, %ymm4

	// unroll 1

	// unroll 2

	// unroll 3

	subl	$ 1, %r10d
	addq	$ 32, %r11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nt_rl_8x4_vs_lib4)
#endif





// common inner routine with file scope
//
// edge for B lower triangular
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- lda
// r13   <- B
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NT_RL_ONE_8X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nt_rl_one_8x4_lib4)
#endif
	
	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vmovapd			0(%r11, %r12), %ymm14 // A
	vaddpd			%ymm0, %ymm13, %ymm0
	vaddpd			%ymm4, %ymm14, %ymm4
	vbroadcastsd	8(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5
	vbroadcastsd	16(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6
	vbroadcastsd	24(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm3, %ymm15, %ymm3
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm7, %ymm15, %ymm7

	// unroll 1
	vmovapd			32(%r11), %ymm13 // A
	vmovapd			32(%r11, %r12), %ymm14 // A
	vaddpd			%ymm1, %ymm13, %ymm1
	vaddpd			%ymm5, %ymm14, %ymm5
	vbroadcastsd	48(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6
	vbroadcastsd	56(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm3, %ymm15, %ymm3
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm7, %ymm15, %ymm7

	// unroll 2
	vmovapd			64(%r11), %ymm13 // A
	vmovapd			64(%r11, %r12), %ymm14 // A
	vaddpd			%ymm2, %ymm13, %ymm2
	vaddpd			%ymm6, %ymm14, %ymm6
	vbroadcastsd	88(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm3, %ymm15, %ymm3
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm7, %ymm15, %ymm7

	// unroll 3
	vmovapd			96(%r11), %ymm13 // A
	vmovapd			96(%r11, %r12), %ymm14 // A
	vaddpd			%ymm3, %ymm13, %ymm3
	vaddpd			%ymm7, %ymm14, %ymm7

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r13

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nt_rl_one_8x4_lib4)
#endif





// common inner routine with file scope
//
// edge for B upper triangular
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- lda
// r13   <- B
// r14   <- n1
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NT_RL_ONE_8X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nt_rl_one_8x4_vs_lib4)
#endif
	
	cmpl	$ 0, %r14d
	jle		0f // end

	cmpl	$ 4, %r14d
	jl		1f // end

	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vmovapd			0(%r11, %r12), %ymm14 // A
	vaddpd			%ymm0, %ymm13, %ymm0
	vaddpd			%ymm4, %ymm14, %ymm4
	vbroadcastsd	8(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5
	vbroadcastsd	16(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6
	vbroadcastsd	24(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm3, %ymm15, %ymm3
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm7, %ymm15, %ymm7

	// unroll 1
	vmovapd			32(%r11), %ymm13 // A
	vmovapd			32(%r11, %r12), %ymm14 // A
	vaddpd			%ymm1, %ymm13, %ymm1
	vaddpd			%ymm5, %ymm14, %ymm5
	vbroadcastsd	48(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6
	vbroadcastsd	56(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm3, %ymm15, %ymm3
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm7, %ymm15, %ymm7

	// unroll 2
	vmovapd			64(%r11), %ymm13 // A
	vmovapd			64(%r11, %r12), %ymm14 // A
	vaddpd			%ymm2, %ymm13, %ymm2
	vaddpd			%ymm6, %ymm14, %ymm6
	vbroadcastsd	88(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm3, %ymm15, %ymm3
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm7, %ymm15, %ymm7

	// unroll 3
	vmovapd			96(%r11), %ymm13 // A
	vmovapd			96(%r11, %r12), %ymm14 // A
	vaddpd			%ymm3, %ymm13, %ymm3
	vaddpd			%ymm7, %ymm14, %ymm7

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r13

	jmp		0f

1:

	cmpl	$ 3, %r14d
	jl		2f // end

	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vmovapd			0(%r11, %r12), %ymm14 // A
	vaddpd			%ymm0, %ymm13, %ymm0
	vaddpd			%ymm4, %ymm14, %ymm4
	vbroadcastsd	8(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5
	vbroadcastsd	16(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6

	// unroll 1
	vmovapd			32(%r11), %ymm13 // A
	vmovapd			32(%r11, %r12), %ymm14 // A
	vaddpd			%ymm1, %ymm13, %ymm1
	vaddpd			%ymm5, %ymm14, %ymm5
	vbroadcastsd	48(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6

	// unroll 2
	vmovapd			64(%r11), %ymm13 // A
	vmovapd			64(%r11, %r12), %ymm14 // A
	vaddpd			%ymm2, %ymm13, %ymm2
	vaddpd			%ymm6, %ymm14, %ymm6

	// unroll 3

	subl	$ 3, %r10d
	addq	$ 96, %r11
	addq	$ 96, %r13

	jmp		0f

2:

	cmpl	$ 2, %r14d
	jl		3f // end

	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vmovapd			0(%r11, %r12), %ymm14 // A
	vaddpd			%ymm0, %ymm13, %ymm0
	vaddpd			%ymm4, %ymm14, %ymm4
	vbroadcastsd	8(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	vmulpd			%ymm14, %ymm12, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5

	// unroll 1
	vmovapd			32(%r11), %ymm13 // A
	vmovapd			32(%r11, %r12), %ymm14 // A
	vaddpd			%ymm1, %ymm13, %ymm1
	vaddpd			%ymm5, %ymm14, %ymm5

	// unroll 2

	// unroll 3

	subl	$ 2, %r10d
	addq	$ 64, %r11
	addq	$ 64, %r13

	jmp		0f

3:

//	cmpl	$ 1, %r14d
//	jl		0f // end

	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vmovapd			0(%r11, %r12), %ymm14 // A
	vaddpd			%ymm0, %ymm13, %ymm0
	vaddpd			%ymm4, %ymm14, %ymm4

	// unroll 1

	// unroll 2

	// unroll 3

	subl	$ 1, %r10d
	addq	$ 32, %r11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nt_rl_one_8x4_vs_lib4)
#endif





// common inner routine with file scope
//
// blend
//
// input arguments:
// ymm0  <- [d00 d11 d22 d33]
// ymm1  <- [d01 d10 d23 d32]
// ymm2  <- [d03 d12 d21 d30]
// ymm3  <- [d02 d13 d20 d31]
// ymm4  <- [d40 d51 d62 d73]
// ymm5  <- [d41 d50 d63 d72]
// ymm6  <- [d43 d52 d61 d70]
// ymm7  <- [d42 d53 d60 d71]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty
//
// output arguments:
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_8X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_blend_8x4_lib4)
#endif


	// tc==n
	vblendpd	$ 0xa, %ymm1, %ymm0, %ymm8
	vblendpd	$ 0x5, %ymm1, %ymm0, %ymm9
	vblendpd	$ 0xa, %ymm3, %ymm2, %ymm10
	vblendpd	$ 0x5, %ymm3, %ymm2, %ymm11

	vblendpd	$ 0xc, %ymm10, %ymm8, %ymm0
	vblendpd	$ 0x3, %ymm10, %ymm8, %ymm2
	vblendpd	$ 0xc, %ymm11, %ymm9, %ymm1
	vblendpd	$ 0x3, %ymm11, %ymm9, %ymm3

	vblendpd	$ 0xa, %ymm5, %ymm4, %ymm8
	vblendpd	$ 0x5, %ymm5, %ymm4, %ymm9
	vblendpd	$ 0xa, %ymm7, %ymm6, %ymm10
	vblendpd	$ 0x5, %ymm7, %ymm6, %ymm11

	vblendpd	$ 0xc, %ymm10, %ymm8, %ymm4
	vblendpd	$ 0x3, %ymm10, %ymm8, %ymm6
	vblendpd	$ 0xc, %ymm11, %ymm9, %ymm5
	vblendpd	$ 0x3, %ymm11, %ymm9, %ymm7

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_blend_8x4_lib4)
#endif





// common inner routine with file scope
//
// scale for alpha = 1.0 and beta
//
// input arguments:
// r10   <- beta
// r11   <- C
// r12   <- 4*sdc*sizeof(double)
// ymm0  <- [d00 d11 d22 d33]
// ymm1  <- [d01 d10 d23 d32]
// ymm2  <- [d03 d12 d21 d30]
// ymm3  <- [d02 d13 d20 d31]
// ymm4  <- [d40 d51 d62 d73]
// ymm5  <- [d41 d50 d63 d72]
// ymm6  <- [d43 d52 d61 d70]
// ymm7  <- [d42 d53 d60 d71]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10   <- beta
// r11   <- C
// r12   <- 4*sdc*sizeof(double)
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M1B_8X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_scale_m1b_8x4_lib4)
#endif

	// beta
	vbroadcastsd	0(%r10), %ymm14

	vxorpd		%ymm15, %ymm15, %ymm15 // 0.0

	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	vmovapd		0(%r11), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vsubpd		%ymm0, %ymm15, %ymm0
	vmovapd		32(%r11), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vsubpd		%ymm1, %ymm15, %ymm1
	vmovapd		64(%r11), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vsubpd		%ymm2, %ymm15, %ymm2
	vmovapd		96(%r11), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vsubpd		%ymm3, %ymm15, %ymm3

	vmovapd		0(%r11, %r12, 1), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vsubpd		%ymm4, %ymm15, %ymm4
	vmovapd		32(%r11, %r12, 1), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vsubpd		%ymm5, %ymm15, %ymm5
	vmovapd		64(%r11, %r12, 1), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vsubpd		%ymm6, %ymm15, %ymm6
	vmovapd		96(%r11, %r12, 1), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vsubpd		%ymm7, %ymm15, %ymm7

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m1b_8x4_lib4)
#endif





// common inner routine with file scope
//
// scale for alpha = 1.0 and beta = 1.0
//
// input arguments:
// r10   <- C
// r11   <- 4*sdc*sizeof(double)
// ymm0  <- [d00 d11 d22 d33]
// ymm1  <- [d01 d10 d23 d32]
// ymm2  <- [d03 d12 d21 d30]
// ymm3  <- [d02 d13 d20 d31]
// ymm4  <- [d40 d51 d62 d73]
// ymm5  <- [d41 d50 d63 d72]
// ymm6  <- [d43 d52 d61 d70]
// ymm7  <- [d42 d53 d60 d71]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10   <- C
// r11   <- 4*sdc*sizeof(double)
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M11_8X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_scale_m11_8x4_lib4)
#endif


	vmovapd		0(%r10), %ymm15
	vsubpd		%ymm0, %ymm15, %ymm0
	vmovapd		32(%r10), %ymm15
	vsubpd		%ymm1, %ymm15, %ymm1
	vmovapd		64(%r10), %ymm15
	vsubpd		%ymm2, %ymm15, %ymm2
	vmovapd		96(%r10), %ymm15
	vsubpd		%ymm3, %ymm15, %ymm3

	vmovapd		0(%r10, %r11, 1), %ymm15
	vsubpd		%ymm4, %ymm15, %ymm4
	vmovapd		32(%r10, %r11, 1), %ymm15
	vsubpd		%ymm5, %ymm15, %ymm5
	vmovapd		64(%r10, %r11, 1), %ymm15
	vsubpd		%ymm6, %ymm15, %ymm6
	vmovapd		96(%r10, %r11, 1), %ymm15
	vsubpd		%ymm7, %ymm15, %ymm7

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m11_8x4_lib4)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta=0
//
// input arguments:
// r10   <- alpha
// ymm0  <- [d00 d11 d22 d33]
// ymm1  <- [d01 d10 d23 d32]
// ymm2  <- [d03 d12 d21 d30]
// ymm3  <- [d02 d13 d20 d31]
// ymm4  <- [d40 d51 d62 d73]
// ymm5  <- [d41 d50 d63 d72]
// ymm6  <- [d43 d52 d61 d70]
// ymm7  <- [d42 d53 d60 d71]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10   <- alpha
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_A0_8X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_scale_a0_8x4_lib4)
#endif


	// alpha
	vbroadcastsd	0(%r10), %ymm15

	vmulpd		%ymm0, %ymm15, %ymm0
	vmulpd		%ymm1, %ymm15, %ymm1
	vmulpd		%ymm2, %ymm15, %ymm2
	vmulpd		%ymm3, %ymm15, %ymm3

	vmulpd		%ymm4, %ymm15, %ymm4
	vmulpd		%ymm5, %ymm15, %ymm5
	vmulpd		%ymm6, %ymm15, %ymm6
	vmulpd		%ymm7, %ymm15, %ymm7

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_a0_8x4_lib4)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- 4*sdc*sizeof(double)
// r15   <- dirty
// ymm0  <- [d00 d11 d22 d33]
// ymm1  <- [d01 d10 d23 d32]
// ymm2  <- [d03 d12 d21 d30]
// ymm3  <- [d02 d13 d20 d31]
// ymm4  <- [d40 d51 d62 d73]
// ymm5  <- [d41 d50 d63 d72]
// ymm6  <- [d43 d52 d61 d70]
// ymm7  <- [d42 d53 d60 d71]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- 4*sdc*sizeof(double)
// r15   <- dirty
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_8X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_8x4_lib4)
#endif

	// alpha
	vbroadcastsd	0(%r10), %ymm15

	vmulpd		%ymm0, %ymm15, %ymm0
	vmulpd		%ymm1, %ymm15, %ymm1
	vmulpd		%ymm2, %ymm15, %ymm2
	vmulpd		%ymm3, %ymm15, %ymm3

	vmulpd		%ymm4, %ymm15, %ymm4
	vmulpd		%ymm5, %ymm15, %ymm5
	vmulpd		%ymm6, %ymm15, %ymm6
	vmulpd		%ymm7, %ymm15, %ymm7

	// beta
	vbroadcastsd	0(%r11), %ymm14

	vxorpd		%ymm15, %ymm15, %ymm15 // 0.0

	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	vmovapd		0(%r12), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm0, %ymm15, %ymm0
	vmovapd		32(%r12), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm1, %ymm15, %ymm1
	vmovapd		64(%r12), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm2, %ymm15, %ymm2
	vmovapd		96(%r12), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm3, %ymm15, %ymm3

	vmovapd		0(%r12, %r13, 1), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm4, %ymm15, %ymm4
	vmovapd		32(%r12, %r13, 1), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm5, %ymm15, %ymm5
	vmovapd		64(%r12, %r13, 1), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm6, %ymm15, %ymm6
	vmovapd		96(%r12, %r13, 1), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm7, %ymm15, %ymm7

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_8x4_lib4)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- offset
// r13   <- C
// r14   <- 4*sdc*sizeof(double)
// r15  <- n0 // col index: start from (inc)
// ymm0  <- [d00 d11 d22 d33]
// ymm1  <- [d01 d10 d23 d32]
// ymm2  <- [d03 d12 d21 d30]
// ymm3  <- [d02 d13 d20 d31]
// ymm4  <- [d40 d51 d62 d73]
// ymm5  <- [d41 d50 d63 d72]
// ymm6  <- [d43 d52 d61 d70]
// ymm7  <- [d42 d53 d60 d71]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- offset
// r13   <- C
// r14   <- 4*sdc*sizeof(double)
// r15  <- n0 // col index: start from (inc)
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_8X4_GEN_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_8x4_gen_lib4)
#endif


	// alpha
	vbroadcastsd	0(%r10), %ymm15

	vmulpd		%ymm0, %ymm15, %ymm0
	vmulpd		%ymm1, %ymm15, %ymm1
	vmulpd		%ymm2, %ymm15, %ymm2
	vmulpd		%ymm3, %ymm15, %ymm3

	vmulpd		%ymm4, %ymm15, %ymm4
	vmulpd		%ymm5, %ymm15, %ymm5
	vmulpd		%ymm6, %ymm15, %ymm6
	vmulpd		%ymm7, %ymm15, %ymm7

	// beta
	vbroadcastsd	0(%r11), %ymm15

	vxorpd		%ymm14, %ymm14, %ymm14 // 0.0

	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			3f // end

	cmpl	$ 0, %r12d
	jg		0f

	// offset==0

	vmovapd		0(%r13), %ymm14
	vmulpd		%ymm14, %ymm15, %ymm14
	vaddpd		%ymm0, %ymm14, %ymm0
	vmovapd		32(%r13), %ymm14
	vmulpd		%ymm14, %ymm15, %ymm14
	vaddpd		%ymm1, %ymm14, %ymm1
	vmovapd		64(%r13), %ymm14
	vmulpd		%ymm14, %ymm15, %ymm14
	vaddpd		%ymm2, %ymm14, %ymm2
	vmovapd		96(%r13), %ymm14
	vmulpd		%ymm14, %ymm15, %ymm14
	vaddpd		%ymm3, %ymm14, %ymm3

	vmovapd		0(%r13, %r14, 1), %ymm14
	vmulpd		%ymm14, %ymm15, %ymm14
	vaddpd		%ymm4, %ymm14, %ymm4
	vmovapd		32(%r13, %r14, 1), %ymm14
	vmulpd		%ymm14, %ymm15, %ymm14
	vaddpd		%ymm5, %ymm14, %ymm5
	vmovapd		64(%r13, %r14, 1), %ymm14
	vmulpd		%ymm14, %ymm15, %ymm14
	vaddpd		%ymm6, %ymm14, %ymm6
	vmovapd		96(%r13, %r14, 1), %ymm14
	vmulpd		%ymm14, %ymm15, %ymm14
	vaddpd		%ymm7, %ymm14, %ymm7

	jmp		3f

0:

	cmpl	$ 1, %r12d
	jg		1f

	// offset==1

	vmovapd		0(%r13), %ymm12
	vmovapd		0(%r13, %r14, 1), %ymm13
	vmovapd		0(%r13, %r14, 2), %ymm14
	vblendpd	$ 0x1, %ymm13, %ymm12, %ymm12
	vblendpd	$ 0x1, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm14
	vshufpd		$ 0x5, %ymm14, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm14
	vshufpd		$ 0x5, %ymm14, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm0, %ymm12, %ymm0
	vmulpd		%ymm13, %ymm15, %ymm13
	vaddpd		%ymm4, %ymm13, %ymm4

	vmovapd		32(%r13), %ymm12
	vmovapd		32(%r13, %r14, 1), %ymm13
	vmovapd		32(%r13, %r14, 2), %ymm14
	vblendpd	$ 0x1, %ymm13, %ymm12, %ymm12
	vblendpd	$ 0x1, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm14
	vshufpd		$ 0x5, %ymm14, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm14
	vshufpd		$ 0x5, %ymm14, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm1, %ymm12, %ymm1
	vmulpd		%ymm13, %ymm15, %ymm13
	vaddpd		%ymm5, %ymm13, %ymm5

	vmovapd		64(%r13), %ymm12
	vmovapd		64(%r13, %r14, 1), %ymm13
	vmovapd		64(%r13, %r14, 2), %ymm14
	vblendpd	$ 0x1, %ymm13, %ymm12, %ymm12
	vblendpd	$ 0x1, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm14
	vshufpd		$ 0x5, %ymm14, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm14
	vshufpd		$ 0x5, %ymm14, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm2, %ymm12, %ymm2
	vmulpd		%ymm13, %ymm15, %ymm13
	vaddpd		%ymm6, %ymm13, %ymm6

	vmovapd		96(%r13), %ymm12
	vmovapd		96(%r13, %r14, 1), %ymm13
	vmovapd		96(%r13, %r14, 2), %ymm14
	vblendpd	$ 0x1, %ymm13, %ymm12, %ymm12
	vblendpd	$ 0x1, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm14
	vshufpd		$ 0x5, %ymm14, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm14
	vshufpd		$ 0x5, %ymm14, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm3, %ymm12, %ymm3
	vmulpd		%ymm13, %ymm15, %ymm13
	vaddpd		%ymm7, %ymm13, %ymm7

	jmp		3f

1:

	cmpl	$ 2, %r12d
	jg		2f

	// offset==2

	vmovapd		0(%r13), %ymm12
	vmovapd		0(%r13, %r14, 1), %ymm13
	vmovapd		0(%r13, %r14, 2), %ymm14
	vblendpd	$ 0x3, %ymm13, %ymm12, %ymm12
	vblendpd	$ 0x3, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm0, %ymm12, %ymm0
	vmulpd		%ymm13, %ymm15, %ymm13
	vaddpd		%ymm4, %ymm13, %ymm4

	vmovapd		32(%r13), %ymm12
	vmovapd		32(%r13, %r14, 1), %ymm13
	vmovapd		32(%r13, %r14, 2), %ymm14
	vblendpd	$ 0x3, %ymm13, %ymm12, %ymm12
	vblendpd	$ 0x3, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm1, %ymm12, %ymm1
	vmulpd		%ymm13, %ymm15, %ymm13
	vaddpd		%ymm5, %ymm13, %ymm5

	vmovapd		64(%r13), %ymm12
	vmovapd		64(%r13, %r14, 1), %ymm13
	vmovapd		64(%r13, %r14, 2), %ymm14
	vblendpd	$ 0x3, %ymm13, %ymm12, %ymm12
	vblendpd	$ 0x3, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm2, %ymm12, %ymm2
	vmulpd		%ymm13, %ymm15, %ymm13
	vaddpd		%ymm6, %ymm13, %ymm6

	vmovapd		96(%r13), %ymm12
	vmovapd		96(%r13, %r14, 1), %ymm13
	vmovapd		96(%r13, %r14, 2), %ymm14
	vblendpd	$ 0x3, %ymm13, %ymm12, %ymm12
	vblendpd	$ 0x3, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm3, %ymm12, %ymm3
	vmulpd		%ymm13, %ymm15, %ymm13
	vaddpd		%ymm7, %ymm13, %ymm7


	jmp		3f

2:

	// offset==3

	vmovapd		0(%r13), %ymm12
	vmovapd		0(%r13, %r14, 1), %ymm13
	vmovapd		0(%r13, %r14, 2), %ymm14
	vblendpd	$ 0x7, %ymm13, %ymm12, %ymm12
	vblendpd	$ 0x7, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm14
	vshufpd		$ 0x5, %ymm12, %ymm14, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm14
	vshufpd		$ 0x5, %ymm13, %ymm14, %ymm13
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm0, %ymm12, %ymm0
	vmulpd		%ymm13, %ymm15, %ymm13
	vaddpd		%ymm4, %ymm13, %ymm4

	vmovapd		32(%r13), %ymm12
	vmovapd		32(%r13, %r14, 1), %ymm13
	vmovapd		32(%r13, %r14, 2), %ymm14
	vblendpd	$ 0x7, %ymm13, %ymm12, %ymm12
	vblendpd	$ 0x7, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm14
	vshufpd		$ 0x5, %ymm12, %ymm14, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm14
	vshufpd		$ 0x5, %ymm13, %ymm14, %ymm13
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm1, %ymm12, %ymm1
	vmulpd		%ymm13, %ymm15, %ymm13
	vaddpd		%ymm5, %ymm13, %ymm5

	vmovapd		64(%r13), %ymm12
	vmovapd		64(%r13, %r14, 1), %ymm13
	vmovapd		64(%r13, %r14, 2), %ymm14
	vblendpd	$ 0x7, %ymm13, %ymm12, %ymm12
	vblendpd	$ 0x7, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm14
	vshufpd		$ 0x5, %ymm12, %ymm14, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm14
	vshufpd		$ 0x5, %ymm13, %ymm14, %ymm13
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm2, %ymm12, %ymm2
	vmulpd		%ymm13, %ymm15, %ymm13
	vaddpd		%ymm6, %ymm13, %ymm6

	vmovapd		96(%r13), %ymm12
	vmovapd		96(%r13, %r14, 1), %ymm13
	vmovapd		96(%r13, %r14, 2), %ymm14
	vblendpd	$ 0x7, %ymm13, %ymm12, %ymm12
	vblendpd	$ 0x7, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm14
	vshufpd		$ 0x5, %ymm12, %ymm14, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm14
	vshufpd		$ 0x5, %ymm13, %ymm14, %ymm13
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm3, %ymm12, %ymm3
	vmulpd		%ymm13, %ymm15, %ymm13
	vaddpd		%ymm7, %ymm13, %ymm7


3:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_8x4_gen_lib4)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12  <- offset
// r13   <- C
// r14  <- 4*sdc*sizeof(double)
// ymm0 <- [d00 d11 d22 d33]
// ymm1 <- [d01 d10 d23 d32]
// ymm2 <- [d03 d12 d21 d30]
// ymm3 <- [d02 d13 d20 d31]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10   <- alpha
// r11   <- beta
// r12  <- offset
// r13   <- C
// r14  <- 4*sdc*sizeof(double)
// r15  <- n0 // col index: start from (inc)
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_4X8_GEN_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_4x8_gen_lib4)
#endif

	// alpha
	vbroadcastsd	0(%r10), %ymm15

	vmulpd		%ymm0, %ymm15, %ymm0
	vmulpd		%ymm1, %ymm15, %ymm1
	vmulpd		%ymm2, %ymm15, %ymm2
	vmulpd		%ymm3, %ymm15, %ymm3
	vmulpd		%ymm4, %ymm15, %ymm4
	vmulpd		%ymm5, %ymm15, %ymm5
	vmulpd		%ymm6, %ymm15, %ymm6
	vmulpd		%ymm7, %ymm15, %ymm7

	// beta
	vbroadcastsd	0(%r11), %ymm15

	vxorpd		%ymm14, %ymm14, %ymm14 // 0.0

	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			3f // end

	cmpl	$ 0, %r12d
	jg		0f

	// offset==0

	vmovapd		0(%r13), %ymm12
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm0, %ymm12, %ymm0
	vmovapd		32(%r13), %ymm12
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm1, %ymm12, %ymm1
	vmovapd		64(%r13), %ymm12
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm2, %ymm12, %ymm2
	vmovapd		96(%r13), %ymm12
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm3, %ymm12, %ymm3
	vmovapd		128(%r13), %ymm12
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm4, %ymm12, %ymm4
	vmovapd		160(%r13), %ymm12
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm5, %ymm12, %ymm5
	vmovapd		192(%r13), %ymm12
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm6, %ymm12, %ymm6
	vmovapd		224(%r13), %ymm12
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm7, %ymm12, %ymm7

	jmp		3f

0:

	movq	%r13, %r15 // C0
	addq	%r14, %r15 // C1 <- C0 + 4*sdc*sizeof(double)

	cmpl	$ 1, %r12d
	jg		1f

	// offset==1

	vmovapd		0(%r13), %ymm12
	vmovapd		0(%r15), %ymm14
	vblendpd	$ 0x1, %ymm14, %ymm12, %ymm12
	vmovapd		32(%r13), %ymm13
	vmovapd		32(%r15), %ymm14
	vblendpd	$ 0x1, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm14
	vshufpd		$ 0x5, %ymm14, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm14
	vshufpd		$ 0x5, %ymm14, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm0, %ymm12, %ymm0
	vmulpd		%ymm13, %ymm15, %ymm13
	vaddpd		%ymm1, %ymm13, %ymm1

	vmovapd		64(%r13), %ymm12
	vmovapd		64(%r15), %ymm14
	vblendpd	$ 0x1, %ymm14, %ymm12, %ymm12
	vmovapd		96(%r13), %ymm13
	vmovapd		96(%r15), %ymm14
	vblendpd	$ 0x1, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm14
	vshufpd		$ 0x5, %ymm14, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm14
	vshufpd		$ 0x5, %ymm14, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm2, %ymm12, %ymm2
	vmulpd		%ymm13, %ymm15, %ymm13
	vaddpd		%ymm3, %ymm13, %ymm3

	vmovapd		128(%r13), %ymm12
	vmovapd		128(%r15), %ymm14
	vblendpd	$ 0x1, %ymm14, %ymm12, %ymm12
	vmovapd		160(%r13), %ymm13
	vmovapd		160(%r15), %ymm14
	vblendpd	$ 0x1, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm14
	vshufpd		$ 0x5, %ymm14, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm14
	vshufpd		$ 0x5, %ymm14, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm4, %ymm12, %ymm4
	vmulpd		%ymm13, %ymm15, %ymm13
	vaddpd		%ymm5, %ymm13, %ymm5

	vmovapd		192(%r13), %ymm12
	vmovapd		192(%r15), %ymm14
	vblendpd	$ 0x1, %ymm14, %ymm12, %ymm12
	vmovapd		224(%r13), %ymm13
	vmovapd		224(%r15), %ymm14
	vblendpd	$ 0x1, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm14
	vshufpd		$ 0x5, %ymm14, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm14
	vshufpd		$ 0x5, %ymm14, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm6, %ymm12, %ymm6
	vmulpd		%ymm13, %ymm15, %ymm13
	vaddpd		%ymm7, %ymm13, %ymm7

	jmp		3f

1:

	cmpl	$ 2, %r12d
	jg		2f

	// offset==2

	vmovapd		0(%r13), %ymm12
	vmovapd		0(%r15), %ymm14
	vblendpd	$ 0x3, %ymm14, %ymm12, %ymm12
	vmovapd		32(%r13), %ymm13
	vmovapd		32(%r15), %ymm14
	vblendpd	$ 0x3, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm0, %ymm12, %ymm0
	vmulpd		%ymm13, %ymm15, %ymm13
	vaddpd		%ymm1, %ymm13, %ymm1

	vmovapd		64(%r13), %ymm12
	vmovapd		64(%r15), %ymm14
	vblendpd	$ 0x3, %ymm14, %ymm12, %ymm12
	vmovapd		96(%r13), %ymm13
	vmovapd		96(%r15), %ymm14
	vblendpd	$ 0x3, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm2, %ymm12, %ymm2
	vmulpd		%ymm13, %ymm15, %ymm13
	vaddpd		%ymm3, %ymm13, %ymm3

	vmovapd		128(%r13), %ymm12
	vmovapd		128(%r15), %ymm14
	vblendpd	$ 0x3, %ymm14, %ymm12, %ymm12
	vmovapd		160(%r13), %ymm13
	vmovapd		160(%r15), %ymm14
	vblendpd	$ 0x3, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm4, %ymm12, %ymm4
	vmulpd		%ymm13, %ymm15, %ymm13
	vaddpd		%ymm5, %ymm13, %ymm5

	vmovapd		192(%r13), %ymm12
	vmovapd		192(%r15), %ymm14
	vblendpd	$ 0x3, %ymm14, %ymm12, %ymm12
	vmovapd		224(%r13), %ymm13
	vmovapd		224(%r15), %ymm14
	vblendpd	$ 0x3, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm6, %ymm12, %ymm6
	vmulpd		%ymm13, %ymm15, %ymm13
	vaddpd		%ymm7, %ymm13, %ymm7

	jmp		3f

2:

	// offset==3

	vmovapd		0(%r13), %ymm12
	vmovapd		0(%r15), %ymm14
	vblendpd	$ 0x7, %ymm14, %ymm12, %ymm12
	vmovapd		32(%r13), %ymm13
	vmovapd		32(%r15), %ymm14
	vblendpd	$ 0x7, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm14
	vshufpd		$ 0x5, %ymm12, %ymm14, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm14
	vshufpd		$ 0x5, %ymm13, %ymm14, %ymm13
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm0, %ymm12, %ymm0
	vmulpd		%ymm13, %ymm15, %ymm13
	vaddpd		%ymm1, %ymm13, %ymm1

	vmovapd		64(%r13), %ymm12
	vmovapd		64(%r15), %ymm14
	vblendpd	$ 0x7, %ymm14, %ymm12, %ymm12
	vmovapd		96(%r13), %ymm13
	vmovapd		96(%r15), %ymm14
	vblendpd	$ 0x7, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm14
	vshufpd		$ 0x5, %ymm12, %ymm14, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm14
	vshufpd		$ 0x5, %ymm13, %ymm14, %ymm13
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm2, %ymm12, %ymm2
	vmulpd		%ymm13, %ymm15, %ymm13
	vaddpd		%ymm3, %ymm13, %ymm3

	vmovapd		128(%r13), %ymm12
	vmovapd		128(%r15), %ymm14
	vblendpd	$ 0x7, %ymm14, %ymm12, %ymm12
	vmovapd		160(%r13), %ymm13
	vmovapd		160(%r15), %ymm14
	vblendpd	$ 0x7, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm14
	vshufpd		$ 0x5, %ymm12, %ymm14, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm14
	vshufpd		$ 0x5, %ymm13, %ymm14, %ymm13
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm4, %ymm12, %ymm4
	vmulpd		%ymm13, %ymm15, %ymm13
	vaddpd		%ymm5, %ymm13, %ymm5

	vmovapd		192(%r13), %ymm12
	vmovapd		192(%r15), %ymm14
	vblendpd	$ 0x7, %ymm14, %ymm12, %ymm12
	vmovapd		224(%r13), %ymm13
	vmovapd		224(%r15), %ymm14
	vblendpd	$ 0x7, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm14
	vshufpd		$ 0x5, %ymm12, %ymm14, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm14
	vshufpd		$ 0x5, %ymm13, %ymm14, %ymm13
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm6, %ymm12, %ymm6
	vmulpd		%ymm13, %ymm15, %ymm13
	vaddpd		%ymm7, %ymm13, %ymm7

3:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_4x8_gen_lib4)
#endif





// common inner routine with file scope
//
// blend for generic alpha and beta
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- 4*sdc*sizeof(double)
// r15   <- dirty
// ymm0  <- [d00 d11 d22 d33]
// ymm1  <- [d01 d10 d23 d32]
// ymm2  <- [d03 d12 d21 d30]
// ymm3  <- [d02 d13 d20 d31]
// ymm4  <- [d40 d51 d62 d73]
// ymm5  <- [d41 d50 d63 d72]
// ymm6  <- [d43 d52 d61 d70]
// ymm7  <- [d42 d53 d60 d71]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- 4*sdc*sizeof(double)
// r15   <- dirty
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_SCALE_AB_8X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_blend_scale_ab_8x4_lib4)
#endif


	// alpha
	vbroadcastsd	0(%r10), %ymm15

	// tc==n
	vblendpd	$ 0xa, %ymm1, %ymm0, %ymm8
	vblendpd	$ 0x5, %ymm1, %ymm0, %ymm9
	vblendpd	$ 0xa, %ymm3, %ymm2, %ymm10
	vblendpd	$ 0x5, %ymm3, %ymm2, %ymm11

	vblendpd	$ 0xc, %ymm10, %ymm8, %ymm0
	vblendpd	$ 0x3, %ymm10, %ymm8, %ymm2
	vblendpd	$ 0xc, %ymm11, %ymm9, %ymm1
	vblendpd	$ 0x3, %ymm11, %ymm9, %ymm3

	vmulpd		%ymm0, %ymm15, %ymm0
	vmulpd		%ymm1, %ymm15, %ymm1
	vmulpd		%ymm2, %ymm15, %ymm2
	vmulpd		%ymm3, %ymm15, %ymm3

	vblendpd	$ 0xa, %ymm5, %ymm4, %ymm8
	vblendpd	$ 0x5, %ymm5, %ymm4, %ymm9
	vblendpd	$ 0xa, %ymm7, %ymm6, %ymm10
	vblendpd	$ 0x5, %ymm7, %ymm6, %ymm11

	vblendpd	$ 0xc, %ymm10, %ymm8, %ymm4
	vblendpd	$ 0x3, %ymm10, %ymm8, %ymm6
	vblendpd	$ 0xc, %ymm11, %ymm9, %ymm5
	vblendpd	$ 0x3, %ymm11, %ymm9, %ymm7

	vmulpd		%ymm4, %ymm15, %ymm4
	vmulpd		%ymm5, %ymm15, %ymm5
	vmulpd		%ymm6, %ymm15, %ymm6
	vmulpd		%ymm7, %ymm15, %ymm7

	// beta
	vbroadcastsd	0(%r11), %ymm14

	vxorpd		%ymm15, %ymm15, %ymm15 // 0.0

	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	// alg==1
	vmovapd		0(%r12), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm0, %ymm15, %ymm0
	vmovapd		32(%r12), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm1, %ymm15, %ymm1
	vmovapd		64(%r12), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm2, %ymm15, %ymm2
	vmovapd		96(%r12), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm3, %ymm15, %ymm3

	vmovapd		0(%r12, %r13, 1), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm4, %ymm15, %ymm4
	vmovapd		32(%r12, %r13, 1), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm5, %ymm15, %ymm5
	vmovapd		64(%r12, %r13, 1), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm6, %ymm15, %ymm6
	vmovapd		96(%r12, %r13, 1), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm7, %ymm15, %ymm7

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_blend_scale_ab_8x4_lib4)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// ymm0 <- [d00 d11 d22 d33]
// ymm1 <- [d01 d10 d23 d32]
// ymm2 <- [d03 d12 d21 d30]
// ymm3 <- [d02 d13 d20 d31]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10   <- alpha
// r11   <- beta
// r10   <- C
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_4X8_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_4x8_lib4)
#endif

	// alpha
	vbroadcastsd	0(%r10), %ymm15

	vmulpd		%ymm0, %ymm15, %ymm0
	vmulpd		%ymm1, %ymm15, %ymm1
	vmulpd		%ymm2, %ymm15, %ymm2
	vmulpd		%ymm3, %ymm15, %ymm3
	vmulpd		%ymm4, %ymm15, %ymm4
	vmulpd		%ymm5, %ymm15, %ymm5
	vmulpd		%ymm6, %ymm15, %ymm6
	vmulpd		%ymm7, %ymm15, %ymm7

	// beta
	vbroadcastsd	0(%r11), %ymm14

	vxorpd		%ymm15, %ymm15, %ymm15 // 0.0

	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	vmovapd		0(%r12), %ymm15
	vmulpd		%ymm14, %ymm15, %ymm15
	vaddpd		%ymm15, %ymm0, %ymm0
	vmovapd		32(%r12), %ymm15
	vmulpd		%ymm14, %ymm15, %ymm15
	vaddpd		%ymm15, %ymm1, %ymm1
	vmovapd		64(%r12), %ymm15
	vmulpd		%ymm14, %ymm15, %ymm15
	vaddpd		%ymm15, %ymm2, %ymm2
	vmovapd		96(%r12), %ymm15
	vmulpd		%ymm14, %ymm15, %ymm15
	vaddpd		%ymm15, %ymm3, %ymm3
	vmovapd		128(%r12), %ymm15
	vmulpd		%ymm14, %ymm15, %ymm15
	vaddpd		%ymm15, %ymm4, %ymm4
	vmovapd		160(%r12), %ymm15
	vmulpd		%ymm14, %ymm15, %ymm15
	vaddpd		%ymm15, %ymm5, %ymm5
	vmovapd		192(%r12), %ymm15
	vmulpd		%ymm14, %ymm15, %ymm15
	vaddpd		%ymm15, %ymm6, %ymm6
	vmovapd		224(%r12), %ymm15
	vmulpd		%ymm14, %ymm15, %ymm15
	vaddpd		%ymm15, %ymm7, %ymm7

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_4x8_lib4)
#endif





// common inner routine with file scope
//
// transpose and scale for generic alpha and beta
//
// input arguments:
// r10   <- &alpha
// r11   <- &beta
// r12   <- C
// r13   <- 4*sdc*sizeof(double)
// r14   <- dirty
// r15   <- dirty
// ymm0  <- [d00 d11 d22 d33]
// ymm1  <- [d01 d10 d23 d32]
// ymm2  <- [d03 d12 d21 d30]
// ymm3  <- [d02 d13 d20 d31]
// ymm4  <- [d40 d51 d62 d73]
// ymm5  <- [d41 d50 d63 d72]
// ymm6  <- [d43 d52 d61 d70]
// ymm7  <- [d42 d53 d60 d71]
// ymm8  <- [d80 d91 da2 db3]
// ymm9  <- [d81 d90 da3 db2]
// ymm10 <- [d83 d92 da1 db0]
// ymm11 <- [d82 d93 da0 db1]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10   <- &alpha
// r11   <- &beta
// r12   <- C
// r13   <- 4*sdc*sizeof(double)
// r14   <- dirty
// r15   <- dirty
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- [d80 d90 da0 db0]
// ymm9  <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_TRAN_SCALE_AB_4X8_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_tran_scale_ab_4x8_lib4)
#endif

	vunpcklpd	%ymm1, %ymm0, %ymm12
	vunpckhpd	%ymm0, %ymm1, %ymm13
	vunpcklpd	%ymm3, %ymm2, %ymm14
	vunpckhpd	%ymm2, %ymm3, %ymm15

	vperm2f128	$ 0x20, %ymm14, %ymm12, %ymm0
	vperm2f128	$ 0x31, %ymm12, %ymm14, %ymm2
	vperm2f128	$ 0x20, %ymm15, %ymm13, %ymm1
	vperm2f128	$ 0x31, %ymm13, %ymm15, %ymm3

	vbroadcastsd 0(%r10), %ymm15 // alpha

	vmulpd		%ymm0, %ymm15, %ymm0
	vmulpd		%ymm1, %ymm15, %ymm1
	vmulpd		%ymm2, %ymm15, %ymm2
	vmulpd		%ymm3, %ymm15, %ymm3

	vunpcklpd	%ymm5, %ymm4, %ymm12
	vunpckhpd	%ymm4, %ymm5, %ymm13
	vunpcklpd	%ymm7, %ymm6, %ymm14
	vunpckhpd	%ymm6, %ymm7, %ymm15

	vperm2f128	$ 0x20, %ymm14, %ymm12, %ymm4
	vperm2f128	$ 0x31, %ymm12, %ymm14, %ymm6
	vperm2f128	$ 0x20, %ymm15, %ymm13, %ymm5
	vperm2f128	$ 0x31, %ymm13, %ymm15, %ymm7

	vbroadcastsd 0(%r10), %ymm15 // alpha

	vmulpd		%ymm4, %ymm15, %ymm4
	vmulpd		%ymm5, %ymm15, %ymm5
	vmulpd		%ymm6, %ymm15, %ymm6
	vmulpd		%ymm7, %ymm15, %ymm7

	vbroadcastsd 0(%r11), %ymm14 // beta

	vxorpd		%ymm15, %ymm15, %ymm15 // 0.0

	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	vmovapd		0(%r12), %ymm15
	vmulpd		%ymm14, %ymm15, %ymm15
	vaddpd		%ymm15, %ymm0, %ymm0
	vmovapd		32(%r12), %ymm15
	vmulpd		%ymm14, %ymm15, %ymm15
	vaddpd		%ymm15, %ymm1, %ymm1
	vmovapd		64(%r12), %ymm15
	vmulpd		%ymm14, %ymm15, %ymm15
	vaddpd		%ymm15, %ymm2, %ymm2
	vmovapd		96(%r12), %ymm15
	vmulpd		%ymm14, %ymm15, %ymm15
	vaddpd		%ymm15, %ymm3, %ymm3
	vmovapd		128(%r12), %ymm15
	vmulpd		%ymm14, %ymm15, %ymm15
	vaddpd		%ymm15, %ymm4, %ymm4
	vmovapd		160(%r12), %ymm15
	vmulpd		%ymm14, %ymm15, %ymm15
	vaddpd		%ymm15, %ymm5, %ymm5
	vmovapd		192(%r12), %ymm15
	vmulpd		%ymm14, %ymm15, %ymm15
	vaddpd		%ymm15, %ymm6, %ymm6
	vmovapd		224(%r12), %ymm15
	vmulpd		%ymm14, %ymm15, %ymm15
	vaddpd		%ymm15, %ymm7, %ymm7

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_tran_scale_ab_4x8_lib4)
#endif





// common inner routine with file scope
//
// blend for generic alpha and beta
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- offset
// r13   <- C
// r14   <- 4*sdc*sizeof(double)
// r15  <- n0 // col index: start from (inc)
// ymm0  <- [d00 d11 d22 d33]
// ymm1  <- [d01 d10 d23 d32]
// ymm2  <- [d03 d12 d21 d30]
// ymm3  <- [d02 d13 d20 d31]
// ymm4  <- [d40 d51 d62 d73]
// ymm5  <- [d41 d50 d63 d72]
// ymm6  <- [d43 d52 d61 d70]
// ymm7  <- [d42 d53 d60 d71]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- offset
// r13   <- C
// r14   <- 4*sdc*sizeof(double)
// r15  <- n0 // col index: start from (inc)
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_SCALE_AB_8X4_GEN_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_blend_scale_ab_8x4_gen_lib4)
#endif


	// alpha
	vbroadcastsd	0(%r10), %ymm15

	// tc==n
	vblendpd	$ 0xa, %ymm1, %ymm0, %ymm8
	vblendpd	$ 0x5, %ymm1, %ymm0, %ymm9
	vblendpd	$ 0xa, %ymm3, %ymm2, %ymm10
	vblendpd	$ 0x5, %ymm3, %ymm2, %ymm11

	vblendpd	$ 0xc, %ymm10, %ymm8, %ymm0
	vblendpd	$ 0x3, %ymm10, %ymm8, %ymm2
	vblendpd	$ 0xc, %ymm11, %ymm9, %ymm1
	vblendpd	$ 0x3, %ymm11, %ymm9, %ymm3

	vmulpd		%ymm0, %ymm15, %ymm0
	vmulpd		%ymm1, %ymm15, %ymm1
	vmulpd		%ymm2, %ymm15, %ymm2
	vmulpd		%ymm3, %ymm15, %ymm3

	vblendpd	$ 0xa, %ymm5, %ymm4, %ymm8
	vblendpd	$ 0x5, %ymm5, %ymm4, %ymm9
	vblendpd	$ 0xa, %ymm7, %ymm6, %ymm10
	vblendpd	$ 0x5, %ymm7, %ymm6, %ymm11

	vblendpd	$ 0xc, %ymm10, %ymm8, %ymm4
	vblendpd	$ 0x3, %ymm10, %ymm8, %ymm6
	vblendpd	$ 0xc, %ymm11, %ymm9, %ymm5
	vblendpd	$ 0x3, %ymm11, %ymm9, %ymm7

	vmulpd		%ymm4, %ymm15, %ymm4
	vmulpd		%ymm5, %ymm15, %ymm5
	vmulpd		%ymm6, %ymm15, %ymm6
	vmulpd		%ymm7, %ymm15, %ymm7

	// beta
	vbroadcastsd	0(%r11), %ymm15

	vxorpd		%ymm14, %ymm14, %ymm14 // 0.0

	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			3f // end

	cmpl	$ 0, %r12d
	jg		0f

	// offset==0

	vmovapd		0(%r13), %ymm14
	vmulpd		%ymm14, %ymm15, %ymm14
	vaddpd		%ymm0, %ymm14, %ymm0
	vmovapd		32(%r13), %ymm14
	vmulpd		%ymm14, %ymm15, %ymm14
	vaddpd		%ymm1, %ymm14, %ymm1
	vmovapd		64(%r13), %ymm14
	vmulpd		%ymm14, %ymm15, %ymm14
	vaddpd		%ymm2, %ymm14, %ymm2
	vmovapd		96(%r13), %ymm14
	vmulpd		%ymm14, %ymm15, %ymm14
	vaddpd		%ymm3, %ymm14, %ymm3

	vmovapd		0(%r13, %r14, 1), %ymm14
	vmulpd		%ymm14, %ymm15, %ymm14
	vaddpd		%ymm4, %ymm14, %ymm4
	vmovapd		32(%r13, %r14, 1), %ymm14
	vmulpd		%ymm14, %ymm15, %ymm14
	vaddpd		%ymm5, %ymm14, %ymm5
	vmovapd		64(%r13, %r14, 1), %ymm14
	vmulpd		%ymm14, %ymm15, %ymm14
	vaddpd		%ymm6, %ymm14, %ymm6
	vmovapd		96(%r13, %r14, 1), %ymm14
	vmulpd		%ymm14, %ymm15, %ymm14
	vaddpd		%ymm7, %ymm14, %ymm7

	jmp		3f

0:

	cmpl	$ 1, %r12d
	jg		1f

	// offset==1

	vmovapd		0(%r13), %ymm12
	vmovapd		0(%r13, %r14, 1), %ymm13
	vmovapd		0(%r13, %r14, 2), %ymm14
	vblendpd	$ 0x1, %ymm13, %ymm12, %ymm12
	vblendpd	$ 0x1, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm14
	vshufpd		$ 0x5, %ymm14, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm14
	vshufpd		$ 0x5, %ymm14, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm0, %ymm12, %ymm0
	vmulpd		%ymm13, %ymm15, %ymm13
	vaddpd		%ymm4, %ymm13, %ymm4

	vmovapd		32(%r13), %ymm12
	vmovapd		32(%r13, %r14, 1), %ymm13
	vmovapd		32(%r13, %r14, 2), %ymm14
	vblendpd	$ 0x1, %ymm13, %ymm12, %ymm12
	vblendpd	$ 0x1, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm14
	vshufpd		$ 0x5, %ymm14, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm14
	vshufpd		$ 0x5, %ymm14, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm1, %ymm12, %ymm1
	vmulpd		%ymm13, %ymm15, %ymm13
	vaddpd		%ymm5, %ymm13, %ymm5

	vmovapd		64(%r13), %ymm12
	vmovapd		64(%r13, %r14, 1), %ymm13
	vmovapd		64(%r13, %r14, 2), %ymm14
	vblendpd	$ 0x1, %ymm13, %ymm12, %ymm12
	vblendpd	$ 0x1, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm14
	vshufpd		$ 0x5, %ymm14, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm14
	vshufpd		$ 0x5, %ymm14, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm2, %ymm12, %ymm2
	vmulpd		%ymm13, %ymm15, %ymm13
	vaddpd		%ymm6, %ymm13, %ymm6

	vmovapd		96(%r13), %ymm12
	vmovapd		96(%r13, %r14, 1), %ymm13
	vmovapd		96(%r13, %r14, 2), %ymm14
	vblendpd	$ 0x1, %ymm13, %ymm12, %ymm12
	vblendpd	$ 0x1, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm14
	vshufpd		$ 0x5, %ymm14, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm14
	vshufpd		$ 0x5, %ymm14, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm3, %ymm12, %ymm3
	vmulpd		%ymm13, %ymm15, %ymm13
	vaddpd		%ymm7, %ymm13, %ymm7

	jmp		3f

1:

	cmpl	$ 2, %r12d
	jg		2f

	// offset==2

	vmovapd		0(%r13), %ymm12
	vmovapd		0(%r13, %r14, 1), %ymm13
	vmovapd		0(%r13, %r14, 2), %ymm14
	vblendpd	$ 0x3, %ymm13, %ymm12, %ymm12
	vblendpd	$ 0x3, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm0, %ymm12, %ymm0
	vmulpd		%ymm13, %ymm15, %ymm13
	vaddpd		%ymm4, %ymm13, %ymm4

	vmovapd		32(%r13), %ymm12
	vmovapd		32(%r13, %r14, 1), %ymm13
	vmovapd		32(%r13, %r14, 2), %ymm14
	vblendpd	$ 0x3, %ymm13, %ymm12, %ymm12
	vblendpd	$ 0x3, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm1, %ymm12, %ymm1
	vmulpd		%ymm13, %ymm15, %ymm13
	vaddpd		%ymm5, %ymm13, %ymm5

	vmovapd		64(%r13), %ymm12
	vmovapd		64(%r13, %r14, 1), %ymm13
	vmovapd		64(%r13, %r14, 2), %ymm14
	vblendpd	$ 0x3, %ymm13, %ymm12, %ymm12
	vblendpd	$ 0x3, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm2, %ymm12, %ymm2
	vmulpd		%ymm13, %ymm15, %ymm13
	vaddpd		%ymm6, %ymm13, %ymm6

	vmovapd		96(%r13), %ymm12
	vmovapd		96(%r13, %r14, 1), %ymm13
	vmovapd		96(%r13, %r14, 2), %ymm14
	vblendpd	$ 0x3, %ymm13, %ymm12, %ymm12
	vblendpd	$ 0x3, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm3, %ymm12, %ymm3
	vmulpd		%ymm13, %ymm15, %ymm13
	vaddpd		%ymm7, %ymm13, %ymm7


	jmp		3f

2:

	// offset==3

	vmovapd		0(%r13), %ymm12
	vmovapd		0(%r13, %r14, 1), %ymm13
	vmovapd		0(%r13, %r14, 2), %ymm14
	vblendpd	$ 0x7, %ymm13, %ymm12, %ymm12
	vblendpd	$ 0x7, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm14
	vshufpd		$ 0x5, %ymm12, %ymm14, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm14
	vshufpd		$ 0x5, %ymm13, %ymm14, %ymm13
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm0, %ymm12, %ymm0
	vmulpd		%ymm13, %ymm15, %ymm13
	vaddpd		%ymm4, %ymm13, %ymm4

	vmovapd		32(%r13), %ymm12
	vmovapd		32(%r13, %r14, 1), %ymm13
	vmovapd		32(%r13, %r14, 2), %ymm14
	vblendpd	$ 0x7, %ymm13, %ymm12, %ymm12
	vblendpd	$ 0x7, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm14
	vshufpd		$ 0x5, %ymm12, %ymm14, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm14
	vshufpd		$ 0x5, %ymm13, %ymm14, %ymm13
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm1, %ymm12, %ymm1
	vmulpd		%ymm13, %ymm15, %ymm13
	vaddpd		%ymm5, %ymm13, %ymm5

	vmovapd		64(%r13), %ymm12
	vmovapd		64(%r13, %r14, 1), %ymm13
	vmovapd		64(%r13, %r14, 2), %ymm14
	vblendpd	$ 0x7, %ymm13, %ymm12, %ymm12
	vblendpd	$ 0x7, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm14
	vshufpd		$ 0x5, %ymm12, %ymm14, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm14
	vshufpd		$ 0x5, %ymm13, %ymm14, %ymm13
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm2, %ymm12, %ymm2
	vmulpd		%ymm13, %ymm15, %ymm13
	vaddpd		%ymm6, %ymm13, %ymm6

	vmovapd		96(%r13), %ymm12
	vmovapd		96(%r13, %r14, 1), %ymm13
	vmovapd		96(%r13, %r14, 2), %ymm14
	vblendpd	$ 0x7, %ymm13, %ymm12, %ymm12
	vblendpd	$ 0x7, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm14
	vshufpd		$ 0x5, %ymm12, %ymm14, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm14
	vshufpd		$ 0x5, %ymm13, %ymm14, %ymm13
	vmulpd		%ymm12, %ymm15, %ymm12
	vaddpd		%ymm3, %ymm12, %ymm3
	vmulpd		%ymm13, %ymm15, %ymm13
	vaddpd		%ymm7, %ymm13, %ymm7


3:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_blend_scale_ab_8x4_gen_lib4)
#endif





// common inner routine with file scope
//
// scale for alpha = 1.0 and beta
//
// input arguments:
// r10   <- beta
// r11   <- C
// r12   <- 4*sdc*sizeof(double)
// ymm0  <- [d00 d11 d22 d33]
// ymm1  <- [d01 d10 d23 d32]
// ymm2  <- [d03 d12 d21 d30]
// ymm3  <- [d02 d13 d20 d31]
// ymm4  <- [d40 d51 d62 d73]
// ymm5  <- [d41 d50 d63 d72]
// ymm6  <- [d43 d52 d61 d70]
// ymm7  <- [d42 d53 d60 d71]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10   <- beta
// r11   <- C
// r12   <- 4*sdc*sizeof(double)
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_SCALE_M1B_8X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_blend_scale_m1b_8x4_lib4)
#endif

	vblendpd	$ 0xa, %ymm1, %ymm0, %ymm8
	vblendpd	$ 0x5, %ymm1, %ymm0, %ymm9
	vblendpd	$ 0xa, %ymm3, %ymm2, %ymm10
	vblendpd	$ 0x5, %ymm3, %ymm2, %ymm11

	vblendpd	$ 0xc, %ymm10, %ymm8, %ymm0
	vblendpd	$ 0x3, %ymm10, %ymm8, %ymm2
	vblendpd	$ 0xc, %ymm11, %ymm9, %ymm1
	vblendpd	$ 0x3, %ymm11, %ymm9, %ymm3

	vblendpd	$ 0xa, %ymm5, %ymm4, %ymm8
	vblendpd	$ 0x5, %ymm5, %ymm4, %ymm9
	vblendpd	$ 0xa, %ymm7, %ymm6, %ymm10
	vblendpd	$ 0x5, %ymm7, %ymm6, %ymm11

	vblendpd	$ 0xc, %ymm10, %ymm8, %ymm4
	vblendpd	$ 0x3, %ymm10, %ymm8, %ymm6
	vblendpd	$ 0xc, %ymm11, %ymm9, %ymm5
	vblendpd	$ 0x3, %ymm11, %ymm9, %ymm7

	// beta
	vbroadcastsd	0(%r10), %ymm14

	vxorpd		%ymm15, %ymm15, %ymm15 // 0.0

	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	vmovapd		0(%r11), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vsubpd		%ymm0, %ymm15, %ymm0
	vmovapd		32(%r11), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vsubpd		%ymm1, %ymm15, %ymm1
	vmovapd		64(%r11), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vsubpd		%ymm2, %ymm15, %ymm2
	vmovapd		96(%r11), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vsubpd		%ymm3, %ymm15, %ymm3

	vmovapd		0(%r11, %r12, 1), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vsubpd		%ymm4, %ymm15, %ymm4
	vmovapd		32(%r11, %r12, 1), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vsubpd		%ymm5, %ymm15, %ymm5
	vmovapd		64(%r11, %r12, 1), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vsubpd		%ymm6, %ymm15, %ymm6
	vmovapd		96(%r11, %r12, 1), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vsubpd		%ymm7, %ymm15, %ymm7

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_blend_scale_m1b_8x4_lib4)
#endif





// common inner routine with file scope
//
// blender for alpha = 1.0 and beta = 1.0
//
// input arguments:
// r10   <- C
// r11   <- 4*sdc*sizeof(double)
// r15   <- dirty
// ymm0  <- [d00 d11 d22 d33]
// ymm1  <- [d01 d10 d23 d32]
// ymm2  <- [d03 d12 d21 d30]
// ymm3  <- [d02 d13 d20 d31]
// ymm4  <- [d40 d51 d62 d73]
// ymm5  <- [d41 d50 d63 d72]
// ymm6  <- [d43 d52 d61 d70]
// ymm7  <- [d42 d53 d60 d71]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10   <- C
// r11   <- 4*sdc*sizeof(double)
// r15   <- dirty
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_SCALE_M11_8X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_blend_scale_m11_8x4_lib4)
#endif


	// tc==n
	vblendpd	$ 0xa, %ymm1, %ymm0, %ymm8
	vblendpd	$ 0x5, %ymm1, %ymm0, %ymm9
	vblendpd	$ 0xa, %ymm3, %ymm2, %ymm10
	vblendpd	$ 0x5, %ymm3, %ymm2, %ymm11

	vblendpd	$ 0xc, %ymm10, %ymm8, %ymm0
	vblendpd	$ 0x3, %ymm10, %ymm8, %ymm2
	vblendpd	$ 0xc, %ymm11, %ymm9, %ymm1
	vblendpd	$ 0x3, %ymm11, %ymm9, %ymm3

	vblendpd	$ 0xa, %ymm5, %ymm4, %ymm8
	vblendpd	$ 0x5, %ymm5, %ymm4, %ymm9
	vblendpd	$ 0xa, %ymm7, %ymm6, %ymm10
	vblendpd	$ 0x5, %ymm7, %ymm6, %ymm11

	vblendpd	$ 0xc, %ymm10, %ymm8, %ymm4
	vblendpd	$ 0x3, %ymm10, %ymm8, %ymm6
	vblendpd	$ 0xc, %ymm11, %ymm9, %ymm5
	vblendpd	$ 0x3, %ymm11, %ymm9, %ymm7

	vmovapd		0(%r10), %ymm15
	vsubpd		%ymm0, %ymm15, %ymm0
	vmovapd		32(%r10), %ymm15
	vsubpd		%ymm1, %ymm15, %ymm1
	vmovapd		64(%r10), %ymm15
	vsubpd		%ymm2, %ymm15, %ymm2
	vmovapd		96(%r10), %ymm15
	vsubpd		%ymm3, %ymm15, %ymm3

	vmovapd		0(%r10, %r11, 1), %ymm15
	vsubpd		%ymm4, %ymm15, %ymm4
	vmovapd		32(%r10, %r11, 1), %ymm15
	vsubpd		%ymm5, %ymm15, %ymm5
	vmovapd		64(%r10, %r11, 1), %ymm15
	vsubpd		%ymm6, %ymm15, %ymm6
	vmovapd		96(%r10, %r11, 1), %ymm15
	vsubpd		%ymm7, %ymm15, %ymm7

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_blend_scale_m11_8x4_lib4)
#endif





// common inner routine with file scope
//
// cholesky factorization
//
// input arguments:
// r10  <- inv_diag_E
// ymm0 <- [d00 d11 d22 d33]
// ymm1 <- [d01 d10 d23 d32]
// ymm2 <- [d03 d12 d21 d30]
// ymm3 <- [d02 d13 d20 d31]
// ymm4 <- [d40 d51 d62 d73]
// ymm5 <- [d41 d50 d63 d72]
// ymm6 <- [d43 d52 d61 d70]
// ymm7 <- [d42 d53 d60 d71]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10  <- inv_diag_E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d51 d62 d73]
// ymm5 <- [d41 d50 d63 d72]
// ymm6 <- [d43 d52 d61 d70]
// ymm7 <- [d42 d53 d60 d71]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DPOTRF_8X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dpotrf_8x4_lib4)
#endif

	vxorpd	%ymm15, %ymm15, %ymm15 // 0.0
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd	.LC04(%rip), %xmm14 // 1.0
#elif defined(OS_MAC)
	vmovsd	LC04(%rip), %xmm14 // 1.0
#endif

	vmovsd		%xmm0, %xmm0, %xmm13
	vucomisd	%xmm15, %xmm13 // d_00 > 0.0 ?
	jbe			1f
	vsqrtsd		%xmm13, %xmm13, %xmm13
	vdivsd		%xmm13, %xmm14, %xmm13
2:
	vmovsd		%xmm13, 0(%r10)
	vmovddup	%xmm13, %xmm13
	vperm2f128	$ 0x00, %ymm13, %ymm13, %ymm13
	vmulpd		%ymm0, %ymm13, %ymm0
	vmulpd		%ymm4, %ymm13, %ymm4
	vperm2f128	$ 0x00, %ymm0, %ymm0, %ymm11
	vpermilpd	$ 0xf, %ymm11, %ymm13
	vmulpd		%ymm0, %ymm13, %ymm12
	vsubpd		%ymm12, %ymm1, %ymm1
	vmulpd		%ymm4, %ymm13, %ymm12
	vsubpd		%ymm12, %ymm5, %ymm5
	vperm2f128	$ 0x11, %ymm0, %ymm0, %ymm11
	vpermilpd	$ 0x0, %ymm11, %ymm13
	vmulpd		%ymm0, %ymm13, %ymm12
	vsubpd		%ymm12, %ymm2, %ymm2
	vmulpd		%ymm4, %ymm13, %ymm12
	vsubpd		%ymm12, %ymm6, %ymm6
	vpermilpd	$ 0xf, %ymm11, %ymm13
	vmulpd		%ymm0, %ymm13, %ymm12
	vsubpd		%ymm12, %ymm3, %ymm3
	vmulpd		%ymm4, %ymm13, %ymm12
	vsubpd		%ymm12, %ymm7, %ymm7


	vpermilpd	$ 0x3, %xmm1, %xmm13
	vucomisd	%xmm15, %xmm13 // d_11 > 0.0 ?
	jbe			3f
	vsqrtsd		%xmm13, %xmm13, %xmm13
	vdivsd		%xmm13, %xmm14, %xmm13
4:
	vmovsd		%xmm13, 8(%r10)
	vmovddup	%xmm13, %xmm13
	vperm2f128	$ 0x00, %ymm13, %ymm13, %ymm13
	vmulpd		%ymm1, %ymm13, %ymm1
	vmulpd		%ymm5, %ymm13, %ymm5
	vperm2f128	$ 0x11, %ymm1, %ymm1, %ymm11
	vpermilpd	$ 0x0, %ymm11, %ymm13
	vmulpd		%ymm1, %ymm13, %ymm12
	vsubpd		%ymm12, %ymm2, %ymm2
	vmulpd		%ymm5, %ymm13, %ymm12
	vsubpd		%ymm12, %ymm6, %ymm6
	vpermilpd	$ 0xf, %ymm11, %ymm13
	vmulpd		%ymm1, %ymm13, %ymm12
	vsubpd		%ymm12, %ymm3, %ymm3
	vmulpd		%ymm5, %ymm13, %ymm12
	vsubpd		%ymm12, %ymm7, %ymm7


	vextractf128	$ 0x1, %ymm2, %xmm13
	vucomisd	%xmm15, %xmm13 // d_22 > 0.0 ?
	jbe			5f
	vsqrtsd		%xmm13, %xmm13, %xmm13
	vdivsd		%xmm13, %xmm14, %xmm13
6:
	vmovsd		%xmm13, 16(%r10)
	vmovddup	%xmm13, %xmm13
	vperm2f128	$ 0x00, %ymm13, %ymm13, %ymm13
	vmulpd		%ymm2, %ymm13, %ymm2
	vmulpd		%ymm6, %ymm13, %ymm6
	vperm2f128	$ 0x11, %ymm2, %ymm2, %ymm11
	vpermilpd	$ 0xf, %ymm11, %ymm13
	vmulpd		%ymm2, %ymm13, %ymm12
	vsubpd		%ymm12, %ymm3, %ymm3
	vmulpd		%ymm6, %ymm13, %ymm12
	vsubpd		%ymm12, %ymm7, %ymm7

	vextractf128	$ 0x1, %ymm3, %xmm13
	vpermilpd	$ 0x3, %xmm13, %xmm13
	vucomisd	%xmm15, %xmm13 // d_33 > 0.0 ?
	jbe			7f
	vsqrtsd		%xmm13, %xmm13, %xmm13
	vdivsd		%xmm13, %xmm14, %xmm13
8:
	vmovsd		%xmm13, 24(%r10)
	vmovddup	%xmm13, %xmm13
	vperm2f128	$ 0x00, %ymm13, %ymm13, %ymm13
	vmulpd		%ymm3, %ymm13, %ymm3
	vmulpd		%ymm7, %ymm13, %ymm7

	jmp				0f

1:
	vxorpd	%ymm13, %ymm13, %ymm13
	jmp		2b

3:
	vxorpd	%ymm13, %ymm13, %ymm13
	jmp		4b

5:
	vxorpd	%ymm13, %ymm13, %ymm13
	jmp		6b

7:
	vxorpd	%ymm13, %ymm13, %ymm13
	jmp		8b

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dpotrf_8x4_lib4)
#endif





// common inner routine with file scope
//
// cholesky factorization vs
//
// input arguments:
// r10  <- inv_diag_E
// r11d <- kn
// ymm0 <- [d00 d11 d22 d33]
// ymm1 <- [d01 d10 d23 d32]
// ymm2 <- [d03 d12 d21 d30]
// ymm3 <- [d02 d13 d20 d31]
// ymm4 <- [d40 d51 d62 d73]
// ymm5 <- [d41 d50 d63 d72]
// ymm6 <- [d43 d52 d61 d70]
// ymm7 <- [d42 d53 d60 d71]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10  <- inv_diag_E
// r11d <- kn
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d51 d62 d73]
// ymm5 <- [d41 d50 d63 d72]
// ymm6 <- [d43 d52 d61 d70]
// ymm7 <- [d42 d53 d60 d71]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DPOTRF_8X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dpotrf_8x4_vs_lib4)
#endif

	vxorpd	%ymm15, %ymm15, %ymm15 // 0.0
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd	.LC04(%rip), %xmm14 // 1.0
#elif defined(OS_MAC)
	vmovsd	LC04(%rip), %xmm14 // 1.0
#endif

	vmovsd		%xmm0, %xmm0, %xmm13
	vucomisd	%xmm15, %xmm13 // d_00 > 0.0 ?
	jbe			1f
	vsqrtsd		%xmm13, %xmm13, %xmm13
	vdivsd		%xmm13, %xmm14, %xmm13
2:
	vmovsd		%xmm13, 0(%r10)
	vmovddup	%xmm13, %xmm13
	vperm2f128	$ 0x00, %ymm13, %ymm13, %ymm13
	vmulpd		%ymm0, %ymm13, %ymm0
	vmulpd		%ymm4, %ymm13, %ymm4
	vperm2f128	$ 0x00, %ymm0, %ymm0, %ymm11
	vpermilpd	$ 0xf, %ymm11, %ymm13
	vmulpd		%ymm0, %ymm13, %ymm12
	vsubpd		%ymm12, %ymm1, %ymm1
	vmulpd		%ymm4, %ymm13, %ymm12
	vsubpd		%ymm12, %ymm5, %ymm5
	vperm2f128	$ 0x11, %ymm0, %ymm0, %ymm11
	vpermilpd	$ 0x0, %ymm11, %ymm13
	vmulpd		%ymm0, %ymm13, %ymm12
	vsubpd		%ymm12, %ymm2, %ymm2
	vmulpd		%ymm4, %ymm13, %ymm12
	vsubpd		%ymm12, %ymm6, %ymm6
	vpermilpd	$ 0xf, %ymm11, %ymm13
	vmulpd		%ymm0, %ymm13, %ymm12
	vsubpd		%ymm12, %ymm3, %ymm3
	vmulpd		%ymm4, %ymm13, %ymm12
	vsubpd		%ymm12, %ymm7, %ymm7

	cmpl		$ 2, %r11d
	jl			0f // ret

	vpermilpd	$ 0x3, %xmm1, %xmm13
	vucomisd	%xmm15, %xmm13 // d_11 > 0.0 ?
	jbe			3f
	vsqrtsd		%xmm13, %xmm13, %xmm13
	vdivsd		%xmm13, %xmm14, %xmm13
4:
	vmovsd		%xmm13, 8(%r10)
	vmovddup	%xmm13, %xmm13
	vperm2f128	$ 0x00, %ymm13, %ymm13, %ymm13
	vmulpd		%ymm1, %ymm13, %ymm1
	vmulpd		%ymm5, %ymm13, %ymm5
	vperm2f128	$ 0x11, %ymm1, %ymm1, %ymm11
	vpermilpd	$ 0x0, %ymm11, %ymm13
	vmulpd		%ymm1, %ymm13, %ymm12
	vsubpd		%ymm12, %ymm2, %ymm2
	vmulpd		%ymm5, %ymm13, %ymm12
	vsubpd		%ymm12, %ymm6, %ymm6
	vpermilpd	$ 0xf, %ymm11, %ymm13
	vmulpd		%ymm1, %ymm13, %ymm12
	vsubpd		%ymm12, %ymm3, %ymm3
	vmulpd		%ymm5, %ymm13, %ymm12
	vsubpd		%ymm12, %ymm7, %ymm7

	cmpl		$ 3, %r11d
	jl			0f // ret

	vextractf128	$ 0x1, %ymm2, %xmm13
	vucomisd	%xmm15, %xmm13 // d_22 > 0.0 ?
	jbe			5f
	vsqrtsd		%xmm13, %xmm13, %xmm13
	vdivsd		%xmm13, %xmm14, %xmm13
6:
	vmovsd		%xmm13, 16(%r10)
	vmovddup	%xmm13, %xmm13
	vperm2f128	$ 0x00, %ymm13, %ymm13, %ymm13
	vmulpd		%ymm2, %ymm13, %ymm2
	vmulpd		%ymm6, %ymm13, %ymm6
	vperm2f128	$ 0x11, %ymm2, %ymm2, %ymm11
	vpermilpd	$ 0xf, %ymm11, %ymm13
	vmulpd		%ymm2, %ymm13, %ymm12
	vsubpd		%ymm12, %ymm3, %ymm3
	vmulpd		%ymm6, %ymm13, %ymm12
	vsubpd		%ymm12, %ymm7, %ymm7

	cmpl		$ 4, %r11d
	jl			0f // ret

	vextractf128	$ 0x1, %ymm3, %xmm13
	vpermilpd	$ 0x3, %xmm13, %xmm13
	vucomisd	%xmm15, %xmm13 // d_33 > 0.0 ?
	jbe			7f
	vsqrtsd		%xmm13, %xmm13, %xmm13
	vdivsd		%xmm13, %xmm14, %xmm13
8:
	vmovsd		%xmm13, 24(%r10)
	vmovddup	%xmm13, %xmm13
	vperm2f128	$ 0x00, %ymm13, %ymm13, %ymm13
	vmulpd		%ymm3, %ymm13, %ymm3
	vmulpd		%ymm7, %ymm13, %ymm7

	jmp				0f

1:
	vxorpd	%ymm13, %ymm13, %ymm13
	jmp		2b

3:
	vxorpd	%ymm13, %ymm13, %ymm13
	jmp		4b

5:
	vxorpd	%ymm13, %ymm13, %ymm13
	jmp		6b

7:
	vxorpd	%ymm13, %ymm13, %ymm13
	jmp		8b

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dpotrf_8x4_vs_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution for cholesky factorization
//
// input arguments:
// r10  <- E
// r11  <- inv_diag_E
// ymm0 <- [d00 d11 d22 d33]
// ymm1 <- [d01 d10 d23 d32]
// ymm2 <- [d03 d12 d21 d30]
// ymm3 <- [d02 d13 d20 d31]
// ymm4 <- [d40 d51 d62 d73]
// ymm5 <- [d41 d50 d63 d72]
// ymm6 <- [d43 d52 d61 d70]
// ymm7 <- [d42 d53 d60 d71]
// ymm12 <- dirty
// ymm13 <- dirty
//
// output arguments:
// r10  <- E
// r11  <- inv_diag_E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d51 d62 d73]
// ymm5 <- [d41 d50 d63 d72]
// ymm6 <- [d43 d52 d61 d70]
// ymm7 <- [d42 d53 d60 d71]
// ymm12 <- dirty
// ymm13 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RLT_INV_8X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rlt_inv_8x4_lib4)
#endif

	vbroadcastsd	0(%r11), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm0
	vmulpd			%ymm4, %ymm13, %ymm4
	vbroadcastsd	8(%r10), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm1, %ymm1
	vmulpd			%ymm4, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm5, %ymm5
	vbroadcastsd	16(%r10), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm2, %ymm2
	vmulpd			%ymm4, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm6, %ymm6
	vbroadcastsd	24(%r10), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm3, %ymm3
	vmulpd			%ymm4, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm7, %ymm7

	vbroadcastsd	8(%r11), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm1
	vmulpd			%ymm5, %ymm13, %ymm5
	vbroadcastsd	48(%r10), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm2, %ymm2
	vmulpd			%ymm5, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm6, %ymm6
	vbroadcastsd	56(%r10), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm3, %ymm3
	vmulpd			%ymm5, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm7, %ymm7

	vbroadcastsd	16(%r11), %ymm13
	vmulpd			%ymm2, %ymm13, %ymm2
	vmulpd			%ymm6, %ymm13, %ymm6
	vbroadcastsd	88(%r10), %ymm13
	vmulpd			%ymm2, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm3, %ymm3
	vmulpd			%ymm6, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm7, %ymm7

	vbroadcastsd	24(%r11), %ymm13
	vmulpd			%ymm3, %ymm13, %ymm3
	vmulpd			%ymm7, %ymm13, %ymm7

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rlt_inv_8x4_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution for cholesky factorization
//
// input arguments:
// r10  <- D
// r11  <- inv_diag_D
// r12d <- kn
// ymm0 <- [d00 d11 d22 d33]
// ymm1 <- [d01 d10 d23 d32]
// ymm2 <- [d03 d12 d21 d30]
// ymm3 <- [d02 d13 d20 d31]
// ymm4 <- [d40 d51 d62 d73]
// ymm5 <- [d41 d50 d63 d72]
// ymm6 <- [d43 d52 d61 d70]
// ymm7 <- [d42 d53 d60 d71]
// ymm12 <- dirty
// ymm13 <- dirty
//
// output arguments:
// r10  <- D
// r11  <- inv_diag_D
// r12d <- kn
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d51 d62 d73]
// ymm5 <- [d41 d50 d63 d72]
// ymm6 <- [d43 d52 d61 d70]
// ymm7 <- [d42 d53 d60 d71]
// ymm12 <- dirty
// ymm13 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RLT_INV_8X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rlt_inv_8x4_vs_lib4)
#endif

	vbroadcastsd	0(%r11), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm0
	vmulpd			%ymm4, %ymm13, %ymm4
	vbroadcastsd	8(%r10), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm1, %ymm1
	vmulpd			%ymm4, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm5, %ymm5
	vbroadcastsd	16(%r10), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm2, %ymm2
	vmulpd			%ymm4, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm6, %ymm6
	vbroadcastsd	24(%r10), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm3, %ymm3
	vmulpd			%ymm4, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm7, %ymm7

	cmpl			$ 2, %r12d
	jl				0f // ret

	vbroadcastsd	8(%r11), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm1
	vmulpd			%ymm5, %ymm13, %ymm5
	vbroadcastsd	48(%r10), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm2, %ymm2
	vmulpd			%ymm5, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm6, %ymm6
	vbroadcastsd	56(%r10), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm3, %ymm3
	vmulpd			%ymm5, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm7, %ymm7

	cmpl			$ 3, %r12d
	jl				0f // ret

	vbroadcastsd	16(%r11), %ymm13
	vmulpd			%ymm2, %ymm13, %ymm2
	vmulpd			%ymm6, %ymm13, %ymm6
	vbroadcastsd	88(%r10), %ymm13
	vmulpd			%ymm2, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm3, %ymm3
	vmulpd			%ymm6, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm7, %ymm7

	cmpl			$ 4, %r12d
	jl				0f // ret

	vbroadcastsd	24(%r11), %ymm13
	vmulpd			%ymm3, %ymm13, %ymm3
	vmulpd			%ymm7, %ymm13, %ymm7

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rlt_inv_8x4_vs_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = right
// uplo = lower
// tran = transposed
// unit diagonal
//
// input arguments:
// r10  <- E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d51 d62 d73]
// ymm5 <- [d41 d50 d63 d72]
// ymm6 <- [d43 d52 d61 d70]
// ymm7 <- [d42 d53 d60 d71]
// ymm12 <- dirty
// ymm13 <- dirty
//
// output arguments:
// r10  <- E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d51 d62 d73]
// ymm5 <- [d41 d50 d63 d72]
// ymm6 <- [d43 d52 d61 d70]
// ymm7 <- [d42 d53 d60 d71]
// ymm12 <- dirty
// ymm13 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RLT_ONE_8X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rlt_one_8x4_lib4)
#endif

	vbroadcastsd	8(%r10), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm1, %ymm1
	vmulpd			%ymm4, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm5, %ymm5

	vbroadcastsd	16(%r10), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm2, %ymm2
	vmulpd			%ymm4, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm6, %ymm6
	vbroadcastsd	48(%r10), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm2, %ymm2
	vmulpd			%ymm5, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm6, %ymm6

	vbroadcastsd	24(%r10), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm3, %ymm3
	vmulpd			%ymm4, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm7, %ymm7
	vbroadcastsd	56(%r10), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm3, %ymm3
	vmulpd			%ymm5, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm7, %ymm7
	vbroadcastsd	88(%r10), %ymm13
	vmulpd			%ymm2, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm3, %ymm3
	vmulpd			%ymm6, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm7, %ymm7

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rlt_one_8x4_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = right
// uplo = lower
// tran = transposed
// unit diagonal
//
// input arguments:
// r10  <- D
// r11d <- kn
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d51 d62 d73]
// ymm5 <- [d41 d50 d63 d72]
// ymm6 <- [d43 d52 d61 d70]
// ymm7 <- [d42 d53 d60 d71]
// ymm12 <- dirty
// ymm13 <- dirty
//
// output arguments:
// r10  <- D
// r11d <- kn
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d51 d62 d73]
// ymm5 <- [d41 d50 d63 d72]
// ymm6 <- [d43 d52 d61 d70]
// ymm7 <- [d42 d53 d60 d71]
// ymm12 <- dirty
// ymm13 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RLT_ONE_8X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rlt_one_8x4_vs_lib4)
#endif

	cmpl			$ 2, %r11d

	jl				0f // ret

	vbroadcastsd	8(%r10), %ymm13
	cmpl			$ 3, %r11d
	vmulpd			%ymm0, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm1, %ymm1
	vmulpd			%ymm4, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm5, %ymm5

	jl				0f // ret

	vbroadcastsd	16(%r10), %ymm13
	cmpl			$ 4, %r11d
	vmulpd			%ymm0, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm2, %ymm2
	vmulpd			%ymm4, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm6, %ymm6
	vbroadcastsd	48(%r10), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm2, %ymm2
	vmulpd			%ymm5, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm6, %ymm6

	jl				0f // ret

	vbroadcastsd	24(%r10), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm3, %ymm3
	vmulpd			%ymm4, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm7, %ymm7
	vbroadcastsd	56(%r10), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm3, %ymm3
	vmulpd			%ymm5, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm7, %ymm7
	vbroadcastsd	88(%r10), %ymm13
	vmulpd			%ymm2, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm3, %ymm3
	vmulpd			%ymm6, %ymm13, %ymm12
	vsubpd			%ymm12, %ymm7, %ymm7

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rlt_one_8x4_vs_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = right
// uplo = upper
// tran = transposed
// requires explicit inverse of diagonal
//
// input arguments:
// r10  <- E
// r11  <- inv_diag_E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d51 d62 d73]
// ymm5 <- [d41 d50 d63 d72]
// ymm6 <- [d43 d52 d61 d70]
// ymm7 <- [d42 d53 d60 d71]
// ymm12 <- dirty
// ymm13 <- dirty
//
// output arguments:
// r10  <- E
// r11  <- inv_diag_E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d51 d62 d73]
// ymm5 <- [d41 d50 d63 d72]
// ymm6 <- [d43 d52 d61 d70]
// ymm7 <- [d42 d53 d60 d71]
// ymm12 <- dirty
// ymm13 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RUT_INV_8X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rut_inv_8x4_lib4)
#endif

	vbroadcastsd	24(%r11), %ymm12
	vmulpd			%ymm3, %ymm12, %ymm3
	vmulpd			%ymm7, %ymm12, %ymm7
	vbroadcastsd	112(%r10), %ymm12
	vmulpd			%ymm3, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm7, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm6, %ymm6
	vbroadcastsd	104(%r10), %ymm12
	vmulpd			%ymm3, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm7, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm5, %ymm5
	vbroadcastsd	96(%r10), %ymm12
	vmulpd			%ymm3, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm7, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm4, %ymm4

	vbroadcastsd	16(%r11), %ymm12
	vmulpd			%ymm2, %ymm12, %ymm2
	vmulpd			%ymm6, %ymm12, %ymm6
	vbroadcastsd	72(%r10), %ymm12
	vmulpd			%ymm2, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm6, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm5, %ymm5
	vbroadcastsd	64(%r10), %ymm12
	vmulpd			%ymm2, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm6, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm4, %ymm4

	vbroadcastsd	8(%r11), %ymm12
	vmulpd			%ymm1, %ymm12, %ymm1
	vmulpd			%ymm5, %ymm12, %ymm5
	vbroadcastsd	32(%r10), %ymm12
	vmulpd			%ymm1, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm5, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm4, %ymm4

	vbroadcastsd	0(%r11), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm0
	vmulpd			%ymm4, %ymm12, %ymm4

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rut_inv_8x4_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = right
// uplo = lower
// tran = transposed
// requires explicit inverse of diagonal
//
// input arguments:
// r10  <- D
// r11  <- inv_diag_D
// r12d <- kn
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d51 d62 d73]
// ymm5 <- [d41 d50 d63 d72]
// ymm6 <- [d43 d52 d61 d70]
// ymm7 <- [d42 d53 d60 d71]
// ymm12 <- dirty
// ymm13 <- dirty
//
// output arguments:
// r10  <- D
// r11  <- inv_diag_D
// r12d <- kn
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d51 d62 d73]
// ymm5 <- [d41 d50 d63 d72]
// ymm6 <- [d43 d52 d61 d70]
// ymm7 <- [d42 d53 d60 d71]
// ymm12 <- dirty
// ymm13 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RUT_INV_8X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rut_inv_8x4_vs_lib4)
#endif

	cmpl			$ 3, %r12d
	jle				0f

	vbroadcastsd	24(%r11), %ymm12
	vmulpd			%ymm3, %ymm12, %ymm3
	vmulpd			%ymm7, %ymm12, %ymm7
	vbroadcastsd	112(%r10), %ymm12
	vmulpd			%ymm3, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm7, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm6, %ymm6
	vbroadcastsd	104(%r10), %ymm12
	vmulpd			%ymm3, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm7, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm5, %ymm5
	vbroadcastsd	96(%r10), %ymm12
	vmulpd			%ymm3, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm7, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm4, %ymm4

0:
	cmpl			$ 2, %r12d
	jle				1f

	vbroadcastsd	16(%r11), %ymm12
	vmulpd			%ymm2, %ymm12, %ymm2
	vmulpd			%ymm6, %ymm12, %ymm6
	vbroadcastsd	72(%r10), %ymm12
	vmulpd			%ymm2, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm6, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm5, %ymm5
	vbroadcastsd	64(%r10), %ymm12
	vmulpd			%ymm2, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm6, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm4, %ymm4

1:
	cmpl			$ 1, %r12d
	jle				2f

	vbroadcastsd	8(%r11), %ymm12
	vmulpd			%ymm1, %ymm12, %ymm1
	vmulpd			%ymm5, %ymm12, %ymm5
	vbroadcastsd	32(%r10), %ymm12
	vmulpd			%ymm1, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm5, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm4, %ymm4

2:

	vbroadcastsd	0(%r11), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm0
	vmulpd			%ymm4, %ymm12, %ymm4

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rut_inv_8x4_vs_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = right
// uplo = upper
// tran = transposed
// unit diagonal
//
// input arguments:
// r10  <- E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d51 d62 d73]
// ymm5 <- [d41 d50 d63 d72]
// ymm6 <- [d43 d52 d61 d70]
// ymm7 <- [d42 d53 d60 d71]
// ymm12 <- dirty
// ymm13 <- dirty
//
// output arguments:
// r10  <- E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d51 d62 d73]
// ymm5 <- [d41 d50 d63 d72]
// ymm6 <- [d43 d52 d61 d70]
// ymm7 <- [d42 d53 d60 d71]
// ymm12 <- dirty
// ymm13 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RUT_ONE_8X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rut_one_8x4_lib4)
#endif

	vbroadcastsd	112(%r10), %ymm12
	vmulpd			%ymm3, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm7, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm6, %ymm6
	vbroadcastsd	104(%r10), %ymm12
	vmulpd			%ymm3, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm7, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm5, %ymm5
	vbroadcastsd	96(%r10), %ymm12
	vmulpd			%ymm3, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm7, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm4, %ymm4

	vbroadcastsd	72(%r10), %ymm12
	vmulpd			%ymm2, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm6, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm5, %ymm5
	vbroadcastsd	64(%r10), %ymm12
	vmulpd			%ymm2, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm6, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm4, %ymm4

	vbroadcastsd	32(%r10), %ymm12
	vmulpd			%ymm1, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm5, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm4, %ymm4

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rut_one_8x4_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = right
// uplo = lower
// tran = transposed
// unit diagonal
//
// input arguments:
// r10  <- D
// r11d <- kn
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d51 d62 d73]
// ymm5 <- [d41 d50 d63 d72]
// ymm6 <- [d43 d52 d61 d70]
// ymm7 <- [d42 d53 d60 d71]
// ymm12 <- dirty
// ymm13 <- dirty
//
// output arguments:
// r10  <- D
// r11d <- kn
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d51 d62 d73]
// ymm5 <- [d41 d50 d63 d72]
// ymm6 <- [d43 d52 d61 d70]
// ymm7 <- [d42 d53 d60 d71]
// ymm12 <- dirty
// ymm13 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RUT_ONE_8X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rut_one_8x4_vs_lib4)
#endif

	cmpl			$ 3, %r11d
	jle				0f

	vbroadcastsd	112(%r10), %ymm12
	vmulpd			%ymm3, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm7, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm6, %ymm6
	vbroadcastsd	104(%r10), %ymm12
	vmulpd			%ymm3, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm7, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm5, %ymm5
	vbroadcastsd	96(%r10), %ymm12
	vmulpd			%ymm3, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm7, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm4, %ymm4

0:
	cmpl			$ 2, %r11d
	jle				1f

	vbroadcastsd	72(%r10), %ymm12
	vmulpd			%ymm2, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm6, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm5, %ymm5
	vbroadcastsd	64(%r10), %ymm12
	vmulpd			%ymm2, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm6, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm4, %ymm4

1:
	cmpl			$ 1, %r11d
	jle				2f

	vbroadcastsd	32(%r10), %ymm12
	vmulpd			%ymm1, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm5, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm4, %ymm4

2:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rut_one_8x4_vs_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = right
// uplo = up
// tran = normal
// requires explicit inverse of diagonal
//
// input arguments:
// r10  <- E
// r11  <- inv_diag_E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d50 d60 d70]
// ymm5 <- [d41 d51 d61 d71]
// ymm6 <- [d42 d52 d62 d72]
// ymm7 <- [d43 d53 d63 d73]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10  <- E
// r11  <- inv_diag_E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d50 d60 d70]
// ymm5 <- [d41 d51 d61 d71]
// ymm6 <- [d42 d52 d62 d72]
// ymm7 <- [d43 d53 d63 d73]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RUN_INV_8X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_run_inv_8x4_lib4)
#endif

	// first column
	vbroadcastsd	0(%r11), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm0
	vmulpd			%ymm4, %ymm12, %ymm4

	// second column
	vbroadcastsd	32(%r10), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm4, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm5, %ymm5
	vbroadcastsd	8(%r11), %ymm12
	vmulpd			%ymm1, %ymm12, %ymm1
	vmulpd			%ymm5, %ymm12, %ymm5

	// third column
	vbroadcastsd	64(%r10), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm4, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm6, %ymm6
	vbroadcastsd	72(%r10), %ymm12
	vmulpd			%ymm1, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm5, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm6, %ymm6
	vbroadcastsd	16(%r11), %ymm12
	vmulpd			%ymm2, %ymm12, %ymm2
	vmulpd			%ymm6, %ymm12, %ymm6

	// fourth column
	vbroadcastsd	96(%r10), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm4, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm7, %ymm7
	vbroadcastsd	104(%r10), %ymm12
	vmulpd			%ymm1, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm5, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm7, %ymm7
	vbroadcastsd	112(%r10), %ymm12
	vmulpd			%ymm2, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm6, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm7, %ymm7
	vbroadcastsd	24(%r11), %ymm12
	vmulpd			%ymm3, %ymm12, %ymm3
	vmulpd			%ymm7, %ymm12, %ymm7

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_run_inv_8x4_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = left
// uplo = lower
// tran = normal
// not-unit diagonal
//
// input arguments:
// r10  <- E0
// r11  <- 4*sde*sizeof(double)
// r12  <- inv_diag_E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d50 d60 d70]
// ymm5 <- [d41 d51 d61 d71]
// ymm6 <- [d42 d52 d62 d72]
// ymm7 <- [d43 d53 d63 d73]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10  <- E0
// r11  <- 4*sde*sizeof(double)
// r12  <- inv_diag_E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d50 d60 d70]
// ymm5 <- [d41 d51 d61 d71]
// ymm6 <- [d42 d52 d62 d72]
// ymm7 <- [d43 d53 d63 d73]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_LLN_INV_8X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_lln_inv_8x4_lib4)
#endif

	// solve top-left

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC04(%rip), %ymm15 // 1.0
#elif defined(OS_MAC)
	vmovapd			LC04(%rip), %ymm15 // 1.0
#endif
	vbroadcastsd	0(%r12), %ymm12
	vblendpd		$ 0x1, %ymm12, %ymm15, %ymm12
	vmulpd			%ymm12, %ymm0, %ymm0
	vmulpd			%ymm12, %ymm1, %ymm1
	vmulpd			%ymm12, %ymm2, %ymm2
	vmulpd			%ymm12, %ymm3, %ymm3
	vxorpd		%ymm14, %ymm14, %ymm14
	vmovapd		0(%r10), %ymm12
	vblendpd	$ 0x1, %ymm14, %ymm12, %ymm12
	vmovapd		0(%r10, %r11, 1), %ymm14
	vperm2f128	$ 0x00, %ymm0, %ymm0, %ymm13
	vpermilpd	$ 0x0, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm0, %ymm0
	vmulpd		%ymm14, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm4, %ymm4
	vperm2f128	$ 0x00, %ymm1, %ymm1, %ymm13
	vpermilpd	$ 0x0, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm1, %ymm1
	vmulpd		%ymm14, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm5, %ymm5
	vperm2f128	$ 0x00, %ymm2, %ymm2, %ymm13
	vpermilpd	$ 0x0, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm2, %ymm2
	vmulpd		%ymm14, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm6, %ymm6
	vperm2f128	$ 0x00, %ymm3, %ymm3, %ymm13
	vpermilpd	$ 0x0, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm3, %ymm3
	vmulpd		%ymm14, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm7, %ymm7

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC04(%rip), %ymm15 // 1.0
#elif defined(OS_MAC)
	vmovapd			LC04(%rip), %ymm15 // 1.0
#endif
	vbroadcastsd	8(%r12), %ymm12
	vblendpd		$ 0x2, %ymm12, %ymm15, %ymm12
	vmulpd			%ymm12, %ymm0, %ymm0
	vmulpd			%ymm12, %ymm1, %ymm1
	vmulpd			%ymm12, %ymm2, %ymm2
	vmulpd			%ymm12, %ymm3, %ymm3
	vxorpd		%ymm14, %ymm14, %ymm14
	vmovapd		32(%r10), %ymm12
	vblendpd	$ 0x3, %ymm14, %ymm12, %ymm12
	vmovapd		32(%r10, %r11, 1), %ymm14
	vperm2f128	$ 0x00, %ymm0, %ymm0, %ymm13
	vpermilpd	$ 0xf, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm0, %ymm0
	vmulpd		%ymm14, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm4, %ymm4
	vperm2f128	$ 0x00, %ymm1, %ymm1, %ymm13
	vpermilpd	$ 0xf, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm1, %ymm1
	vmulpd		%ymm14, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm5, %ymm5
	vperm2f128	$ 0x00, %ymm2, %ymm2, %ymm13
	vpermilpd	$ 0xf, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm2, %ymm2
	vmulpd		%ymm14, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm6, %ymm6
	vperm2f128	$ 0x00, %ymm3, %ymm3, %ymm13
	vpermilpd	$ 0xf, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm3, %ymm3
	vmulpd		%ymm14, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm7, %ymm7

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC04(%rip), %ymm15 // 1.0
#elif defined(OS_MAC)
	vmovapd			LC04(%rip), %ymm15 // 1.0
#endif
	vbroadcastsd	16(%r12), %ymm12
	vblendpd		$ 0x4, %ymm12, %ymm15, %ymm12
	vmulpd			%ymm12, %ymm0, %ymm0
	vmulpd			%ymm12, %ymm1, %ymm1
	vmulpd			%ymm12, %ymm2, %ymm2
	vmulpd			%ymm12, %ymm3, %ymm3
	vxorpd		%ymm14, %ymm14, %ymm14
	vmovapd		64(%r10), %ymm12
	vblendpd	$ 0x7, %ymm14, %ymm12, %ymm12
	vmovapd		64(%r10, %r11, 1), %ymm14
	vperm2f128	$ 0x11, %ymm0, %ymm0, %ymm13
	vpermilpd	$ 0x0, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm0, %ymm0
	vmulpd		%ymm14, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm4, %ymm4
	vperm2f128	$ 0x11, %ymm1, %ymm1, %ymm13
	vpermilpd	$ 0x0, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm1, %ymm1
	vmulpd		%ymm14, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm5, %ymm5
	vperm2f128	$ 0x11, %ymm2, %ymm2, %ymm13
	vpermilpd	$ 0x0, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm2, %ymm2
	vmulpd		%ymm14, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm6, %ymm6
	vperm2f128	$ 0x11, %ymm3, %ymm3, %ymm13
	vpermilpd	$ 0x0, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm3, %ymm3
	vmulpd		%ymm14, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm7, %ymm7

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC04(%rip), %ymm15 // 1.0
#elif defined(OS_MAC)
	vmovapd			LC04(%rip), %ymm15 // 1.0
#endif
	vbroadcastsd	24(%r12), %ymm12
	vblendpd		$ 0x8, %ymm12, %ymm15, %ymm12
	vmulpd			%ymm12, %ymm0, %ymm0
	vmulpd			%ymm12, %ymm1, %ymm1
	vmulpd			%ymm12, %ymm2, %ymm2
	vmulpd			%ymm12, %ymm3, %ymm3
	vmovapd		96(%r10, %r11, 1), %ymm14
	vperm2f128	$ 0x11, %ymm0, %ymm0, %ymm13
	vpermilpd	$ 0xf, %ymm13, %ymm13
	vmulpd		%ymm14, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm4, %ymm4
	vperm2f128	$ 0x11, %ymm1, %ymm1, %ymm13
	vpermilpd	$ 0xf, %ymm13, %ymm13
	vmulpd		%ymm14, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm5, %ymm5
	vperm2f128	$ 0x11, %ymm2, %ymm2, %ymm13
	vpermilpd	$ 0xf, %ymm13, %ymm13
	vmulpd		%ymm14, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm6, %ymm6
	vperm2f128	$ 0x11, %ymm3, %ymm3, %ymm13
	vpermilpd	$ 0xf, %ymm13, %ymm13
	vmulpd		%ymm14, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm7, %ymm7

	addq		$ 128, %r10


	// solve top-left
	vxorpd		%ymm14, %ymm14, %ymm14

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC04(%rip), %ymm15 // 1.0
#elif defined(OS_MAC)
	vmovapd			LC04(%rip), %ymm15 // 1.0
#endif
	vbroadcastsd	32(%r12), %ymm12
	vblendpd		$ 0x1, %ymm12, %ymm15, %ymm12
	vmulpd			%ymm12, %ymm4, %ymm4
	vmulpd			%ymm12, %ymm5, %ymm5
	vmulpd			%ymm12, %ymm6, %ymm6
	vmulpd			%ymm12, %ymm7, %ymm7
	vmovapd		0(%r10, %r11, 1), %ymm12
	vblendpd	$ 0x1, %ymm14, %ymm12, %ymm12
	vperm2f128	$ 0x00, %ymm4, %ymm4, %ymm13
	vpermilpd	$ 0x0, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm4, %ymm4
	vperm2f128	$ 0x00, %ymm5, %ymm5, %ymm13
	vpermilpd	$ 0x0, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm5, %ymm5
	vperm2f128	$ 0x00, %ymm6, %ymm6, %ymm13
	vpermilpd	$ 0x0, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm6, %ymm6
	vperm2f128	$ 0x00, %ymm7, %ymm7, %ymm13
	vpermilpd	$ 0x0, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm7, %ymm7

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC04(%rip), %ymm15 // 1.0
#elif defined(OS_MAC)
	vmovapd			LC04(%rip), %ymm15 // 1.0
#endif
	vbroadcastsd	40(%r12), %ymm12
	vblendpd		$ 0x2, %ymm12, %ymm15, %ymm12
	vmulpd			%ymm12, %ymm4, %ymm4
	vmulpd			%ymm12, %ymm5, %ymm5
	vmulpd			%ymm12, %ymm6, %ymm6
	vmulpd			%ymm12, %ymm7, %ymm7
	vmovapd		32(%r10, %r11, 1), %ymm12
	vblendpd	$ 0x3, %ymm14, %ymm12, %ymm12
	vperm2f128	$ 0x00, %ymm4, %ymm4, %ymm13
	vpermilpd	$ 0xf, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm4, %ymm4
	vperm2f128	$ 0x00, %ymm5, %ymm5, %ymm13
	vpermilpd	$ 0xf, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm5, %ymm5
	vperm2f128	$ 0x00, %ymm6, %ymm6, %ymm13
	vpermilpd	$ 0xf, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm6, %ymm6
	vperm2f128	$ 0x00, %ymm7, %ymm7, %ymm13
	vpermilpd	$ 0xf, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm7, %ymm7

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC04(%rip), %ymm15 // 1.0
#elif defined(OS_MAC)
	vmovapd			LC04(%rip), %ymm15 // 1.0
#endif
	vbroadcastsd	48(%r12), %ymm12
	vblendpd		$ 0x4, %ymm12, %ymm15, %ymm12
	vmulpd			%ymm12, %ymm4, %ymm4
	vmulpd			%ymm12, %ymm5, %ymm5
	vmulpd			%ymm12, %ymm6, %ymm6
	vmulpd			%ymm12, %ymm7, %ymm7
	vmovapd		64(%r10, %r11, 1), %ymm12
	vblendpd	$ 0x7, %ymm14, %ymm12, %ymm12
	vperm2f128	$ 0x11, %ymm4, %ymm4, %ymm13
	vpermilpd	$ 0x0, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm4, %ymm4
	vperm2f128	$ 0x11, %ymm5, %ymm5, %ymm13
	vpermilpd	$ 0x0, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm5, %ymm5
	vperm2f128	$ 0x11, %ymm6, %ymm6, %ymm13
	vpermilpd	$ 0x0, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm6, %ymm6
	vperm2f128	$ 0x11, %ymm7, %ymm7, %ymm13
	vpermilpd	$ 0x0, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm7, %ymm7

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC04(%rip), %ymm15 // 1.0
#elif defined(OS_MAC)
	vmovapd			LC04(%rip), %ymm15 // 1.0
#endif
	vbroadcastsd	56(%r12), %ymm12
	vblendpd		$ 0x8, %ymm12, %ymm15, %ymm12
	vmulpd			%ymm12, %ymm4, %ymm4
	vmulpd			%ymm12, %ymm5, %ymm5
	vmulpd			%ymm12, %ymm6, %ymm6
	vmulpd			%ymm12, %ymm7, %ymm7

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_lln_inv_8x4_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = left
// uplo = lower
// tran = normal
// unit diagonal
//
// input arguments:
// r10  <- E0
// r11  <- 4*sde*sizeof(double)
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d50 d60 d70]
// ymm5 <- [d41 d51 d61 d71]
// ymm6 <- [d42 d52 d62 d72]
// ymm7 <- [d43 d53 d63 d73]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10  <- E0
// r11  <- 4*sde*sizeof(double)
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d50 d60 d70]
// ymm5 <- [d41 d51 d61 d71]
// ymm6 <- [d42 d52 d62 d72]
// ymm7 <- [d43 d53 d63 d73]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_LLN_ONE_8X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_lln_one_8x4_lib4)
#endif

	// solve top-left
	vxorpd		%ymm14, %ymm14, %ymm14

	vmovapd		0(%r10), %ymm12
	vxorpd		%ymm14, %ymm14, %ymm14
	vblendpd	$ 0x1, %ymm14, %ymm12, %ymm12
	vmovapd		0(%r10, %r11, 1), %ymm14
	vperm2f128	$ 0x00, %ymm0, %ymm0, %ymm13
	vpermilpd	$ 0x0, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm0, %ymm0
	vmulpd		%ymm14, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm4, %ymm4
	vperm2f128	$ 0x00, %ymm1, %ymm1, %ymm13
	vpermilpd	$ 0x0, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm1, %ymm1
	vmulpd		%ymm14, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm5, %ymm5
	vperm2f128	$ 0x00, %ymm2, %ymm2, %ymm13
	vpermilpd	$ 0x0, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm2, %ymm2
	vmulpd		%ymm14, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm6, %ymm6
	vperm2f128	$ 0x00, %ymm3, %ymm3, %ymm13
	vpermilpd	$ 0x0, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm3, %ymm3
	vmulpd		%ymm14, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm7, %ymm7

	vmovapd		32(%r10), %ymm12
	vxorpd		%ymm14, %ymm14, %ymm14
	vblendpd	$ 0x3, %ymm14, %ymm12, %ymm12
	vmovapd		32(%r10, %r11, 1), %ymm14
	vperm2f128	$ 0x00, %ymm0, %ymm0, %ymm13
	vpermilpd	$ 0xf, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm0, %ymm0
	vmulpd		%ymm14, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm4, %ymm4
	vperm2f128	$ 0x00, %ymm1, %ymm1, %ymm13
	vpermilpd	$ 0xf, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm1, %ymm1
	vmulpd		%ymm14, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm5, %ymm5
	vperm2f128	$ 0x00, %ymm2, %ymm2, %ymm13
	vpermilpd	$ 0xf, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm2, %ymm2
	vmulpd		%ymm14, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm6, %ymm6
	vperm2f128	$ 0x00, %ymm3, %ymm3, %ymm13
	vpermilpd	$ 0xf, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm3, %ymm3
	vmulpd		%ymm14, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm7, %ymm7

	vmovapd		64(%r10), %ymm12
	vxorpd		%ymm14, %ymm14, %ymm14
	vblendpd	$ 0x7, %ymm14, %ymm12, %ymm12
	vmovapd		64(%r10, %r11, 1), %ymm14
	vperm2f128	$ 0x11, %ymm0, %ymm0, %ymm13
	vpermilpd	$ 0x0, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm0, %ymm0
	vmulpd		%ymm14, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm4, %ymm4
	vperm2f128	$ 0x11, %ymm1, %ymm1, %ymm13
	vpermilpd	$ 0x0, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm1, %ymm1
	vmulpd		%ymm14, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm5, %ymm5
	vperm2f128	$ 0x11, %ymm2, %ymm2, %ymm13
	vpermilpd	$ 0x0, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm2, %ymm2
	vmulpd		%ymm14, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm6, %ymm6
	vperm2f128	$ 0x11, %ymm3, %ymm3, %ymm13
	vpermilpd	$ 0x0, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm3, %ymm3
	vmulpd		%ymm14, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm7, %ymm7

	vmovapd		96(%r10, %r11, 1), %ymm14
	vperm2f128	$ 0x11, %ymm0, %ymm0, %ymm13
	vpermilpd	$ 0xf, %ymm13, %ymm13
	vmulpd		%ymm14, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm4, %ymm4
	vperm2f128	$ 0x11, %ymm1, %ymm1, %ymm13
	vpermilpd	$ 0xf, %ymm13, %ymm13
	vmulpd		%ymm14, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm5, %ymm5
	vperm2f128	$ 0x11, %ymm2, %ymm2, %ymm13
	vpermilpd	$ 0xf, %ymm13, %ymm13
	vmulpd		%ymm14, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm6, %ymm6
	vperm2f128	$ 0x11, %ymm3, %ymm3, %ymm13
	vpermilpd	$ 0xf, %ymm13, %ymm13
	vmulpd		%ymm14, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm7, %ymm7

	addq		$ 128, %r10


	// solve top-left
	vxorpd		%ymm14, %ymm14, %ymm14

	vmovapd		0(%r10, %r11, 1), %ymm12
	vblendpd	$ 0x1, %ymm14, %ymm12, %ymm12
	vperm2f128	$ 0x00, %ymm4, %ymm4, %ymm13
	vpermilpd	$ 0x0, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm4, %ymm4
	vperm2f128	$ 0x00, %ymm5, %ymm5, %ymm13
	vpermilpd	$ 0x0, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm5, %ymm5
	vperm2f128	$ 0x00, %ymm6, %ymm6, %ymm13
	vpermilpd	$ 0x0, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm6, %ymm6
	vperm2f128	$ 0x00, %ymm7, %ymm7, %ymm13
	vpermilpd	$ 0x0, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm7, %ymm7

	vmovapd		32(%r10, %r11, 1), %ymm12
	vblendpd	$ 0x3, %ymm14, %ymm12, %ymm12
	vperm2f128	$ 0x00, %ymm4, %ymm4, %ymm13
	vpermilpd	$ 0xf, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm4, %ymm4
	vperm2f128	$ 0x00, %ymm5, %ymm5, %ymm13
	vpermilpd	$ 0xf, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm5, %ymm5
	vperm2f128	$ 0x00, %ymm6, %ymm6, %ymm13
	vpermilpd	$ 0xf, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm6, %ymm6
	vperm2f128	$ 0x00, %ymm7, %ymm7, %ymm13
	vpermilpd	$ 0xf, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm7, %ymm7

	vmovapd		64(%r10, %r11, 1), %ymm12
	vblendpd	$ 0x7, %ymm14, %ymm12, %ymm12
	vperm2f128	$ 0x11, %ymm4, %ymm4, %ymm13
	vpermilpd	$ 0x0, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm4, %ymm4
	vperm2f128	$ 0x11, %ymm5, %ymm5, %ymm13
	vpermilpd	$ 0x0, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm5, %ymm5
	vperm2f128	$ 0x11, %ymm6, %ymm6, %ymm13
	vpermilpd	$ 0x0, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm6, %ymm6
	vperm2f128	$ 0x11, %ymm7, %ymm7, %ymm13
	vpermilpd	$ 0x0, %ymm13, %ymm13
	vmulpd		%ymm12, %ymm13, %ymm15
	vsubpd		%ymm15, %ymm7, %ymm7

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_lln_one_8x4_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = left
// uplo = upper
// tran = normal
// requires explicit inverse of diagonal
//
// input arguments:
// r10  <- E
// r11  <- 4*sde*sizeof(double)
// r12  <- inv_diag_E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d50 d60 d70]
// ymm5 <- [d41 d51 d61 d71]
// ymm6 <- [d42 d52 d62 d72]
// ymm7 <- [d43 d53 d63 d73]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10  <- E
// r11  <- 4*sde*sizeof(double)
// r12  <- inv_diag_E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d50 d60 d70]
// ymm5 <- [d41 d51 d61 d71]
// ymm6 <- [d42 d52 d62 d72]
// ymm7 <- [d43 d53 d63 d73]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_LUN_INV_8X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_lun_inv_8x4_lib4)
#endif

	// bottom-right

	vmovapd			224(%r10, %r11, 1), %ymm13
	vxorpd			%ymm14, %ymm14, %ymm14 // 0.0
	vblendpd		$ 0x7, %ymm13, %ymm14, %ymm13
	vbroadcastsd	56(%r12), %ymm12
	vmovapd			224(%r10), %ymm11

	vperm2f128		$ 0x11, %ymm4, %ymm4, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm4, %ymm4
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm4, %ymm4
	vmulpd			%ymm11, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0

	vperm2f128		$ 0x11, %ymm5, %ymm5, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm5, %ymm5
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm5, %ymm5
	vmulpd			%ymm11, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm1, %ymm1

	vperm2f128		$ 0x11, %ymm6, %ymm6, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm6, %ymm6
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm6, %ymm6
	vmulpd			%ymm11, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm2, %ymm2

	vperm2f128		$ 0x11, %ymm7, %ymm7, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm7, %ymm7
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm7, %ymm7
	vmulpd			%ymm11, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm3, %ymm3


	vxorpd			%ymm13, %ymm13, %ymm13 // 0.0 // ?????????????
	vmovapd			192(%r10, %r11, 1), %xmm13
	vbroadcastsd	48(%r12), %ymm12
	vmovapd			192(%r10), %ymm11

	vperm2f128		$ 0x11, %ymm4, %ymm4, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm4, %ymm4
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm4, %ymm4
	vmulpd			%ymm11, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0

	vperm2f128		$ 0x11, %ymm5, %ymm5, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm5, %ymm5
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm5, %ymm5
	vmulpd			%ymm11, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm1, %ymm1

	vperm2f128		$ 0x11, %ymm6, %ymm6, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm6, %ymm6
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm6, %ymm6
	vmulpd			%ymm11, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm2, %ymm2

	vperm2f128		$ 0x11, %ymm7, %ymm7, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm7, %ymm7
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm7, %ymm7
	vmulpd			%ymm11, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm3, %ymm3


	vxorpd			%ymm13, %ymm13, %ymm13 // 0.0
	vmovsd			160(%r10, %r11, 1), %xmm13
	vbroadcastsd	40(%r12), %ymm12
	vmovapd			160(%r10), %ymm11

	vperm2f128		$ 0x00, %ymm4, %ymm4, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm4, %ymm4
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm4, %ymm4
	vmulpd			%ymm11, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0

	vperm2f128		$ 0x00, %ymm5, %ymm5, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm5, %ymm5
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm5, %ymm5
	vmulpd			%ymm11, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm1, %ymm1

	vperm2f128		$ 0x00, %ymm6, %ymm6, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm6, %ymm6
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm6, %ymm6
	vmulpd			%ymm11, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm2, %ymm2

	vperm2f128		$ 0x00, %ymm7, %ymm7, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm7, %ymm7
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm7, %ymm7
	vmulpd			%ymm11, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm3, %ymm3


	vbroadcastsd	32(%r12), %ymm12
	vmovapd			128(%r10), %ymm11

	vperm2f128		$ 0x00, %ymm4, %ymm4, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm4, %ymm4
	vmulpd			%ymm11, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0

	vperm2f128		$ 0x00, %ymm5, %ymm5, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm5, %ymm5
	vmulpd			%ymm11, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm1, %ymm1

	vperm2f128		$ 0x00, %ymm6, %ymm6, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm6, %ymm6
	vmulpd			%ymm11, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm2, %ymm2

	vperm2f128		$ 0x00, %ymm7, %ymm7, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm7, %ymm7
	vmulpd			%ymm11, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm3, %ymm3


	// top-left

	vmovapd			96(%r10), %ymm13
	vxorpd			%ymm14, %ymm14, %ymm14 // 0.0
	vblendpd		$ 0x7, %ymm13, %ymm14, %ymm13
	vbroadcastsd	24(%r12), %ymm12

	vperm2f128		$ 0x11, %ymm0, %ymm0, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm0, %ymm0
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0

	vperm2f128		$ 0x11, %ymm1, %ymm1, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm1, %ymm1
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm1, %ymm1

	vperm2f128		$ 0x11, %ymm2, %ymm2, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm2, %ymm2
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm2, %ymm2

	vperm2f128		$ 0x11, %ymm3, %ymm3, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm3, %ymm3
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm3, %ymm3


	vxorpd			%ymm13, %ymm13, %ymm13 // 0.0
	vmovapd			64(%r10), %xmm13
	vbroadcastsd	16(%r12), %ymm12

	vperm2f128		$ 0x11, %ymm0, %ymm0, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm0, %ymm0
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0

	vperm2f128		$ 0x11, %ymm1, %ymm1, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm1, %ymm1
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm1, %ymm1

	vperm2f128		$ 0x11, %ymm2, %ymm2, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm2, %ymm2
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm2, %ymm2

	vperm2f128		$ 0x11, %ymm3, %ymm3, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm3, %ymm3
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm3, %ymm3


	vxorpd			%ymm13, %ymm13, %ymm13 // 0.0
	vmovsd			32(%r10), %xmm13
	vbroadcastsd	8(%r12), %ymm12

	vpermilpd		$ 0xf, %ymm0, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm0, %ymm0
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0

	vpermilpd		$ 0xf, %ymm1, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm1, %ymm1
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm1, %ymm1

	vpermilpd		$ 0xf, %ymm2, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm2, %ymm2
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm2, %ymm2

	vpermilpd		$ 0xf, %ymm3, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm3, %ymm3
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm3, %ymm3


	vbroadcastsd	0(%r12), %ymm12

	vmulpd			%ymm0, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm0, %ymm0

	vmulpd			%ymm1, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm1, %ymm1

	vmulpd			%ymm2, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm2, %ymm2

	vmulpd			%ymm3, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm3, %ymm3

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_lun_inv_8x4_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = left
// uplo = upper
// tran = normal
// requires explicit inverse of diagonal
//
// input arguments:
// r10  <- E
// r11  <- 4*sde*sizeof(double)
// r12  <- inv_diag_E
// r13  <- km
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d50 d60 d70]
// ymm5 <- [d41 d51 d61 d71]
// ymm6 <- [d42 d52 d62 d72]
// ymm7 <- [d43 d53 d63 d73]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10  <- E
// r11  <- 4*sde*sizeof(double)
// r12  <- inv_diag_E
// r13  <- km
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d50 d60 d70]
// ymm5 <- [d41 d51 d61 d71]
// ymm6 <- [d42 d52 d62 d72]
// ymm7 <- [d43 d53 d63 d73]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_LUN_INV_8X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_lun_inv_8x4_vs_lib4)
#endif

	// bottom-right

	cmpl	$ 7, %r13d
	jle		0f

	vmovapd			224(%r10, %r11, 1), %ymm13
	vxorpd			%ymm14, %ymm14, %ymm14 // 0.0
	vblendpd		$ 0x7, %ymm13, %ymm14, %ymm13
	vbroadcastsd	56(%r12), %ymm12
	vmovapd			224(%r10), %ymm11

	vperm2f128		$ 0x11, %ymm4, %ymm4, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm4, %ymm4
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm4, %ymm4
	vmulpd			%ymm11, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0

	vperm2f128		$ 0x11, %ymm5, %ymm5, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm5, %ymm5
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm5, %ymm5
	vmulpd			%ymm11, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm1, %ymm1

	vperm2f128		$ 0x11, %ymm6, %ymm6, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm6, %ymm6
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm6, %ymm6
	vmulpd			%ymm11, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm2, %ymm2

	vperm2f128		$ 0x11, %ymm7, %ymm7, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm7, %ymm7
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm7, %ymm7
	vmulpd			%ymm11, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm3, %ymm3

0:
	cmpl	$ 6, %r13d
	jle		1f

	vxorpd			%ymm13, %ymm13, %ymm13 // 0.0 // ?????????????
	vmovapd			192(%r10, %r11, 1), %xmm13
	vbroadcastsd	48(%r12), %ymm12
	vmovapd			192(%r10), %ymm11

	vperm2f128		$ 0x11, %ymm4, %ymm4, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm4, %ymm4
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm4, %ymm4
	vmulpd			%ymm11, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0

	vperm2f128		$ 0x11, %ymm5, %ymm5, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm5, %ymm5
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm5, %ymm5
	vmulpd			%ymm11, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm1, %ymm1

	vperm2f128		$ 0x11, %ymm6, %ymm6, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm6, %ymm6
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm6, %ymm6
	vmulpd			%ymm11, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm2, %ymm2

	vperm2f128		$ 0x11, %ymm7, %ymm7, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm7, %ymm7
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm7, %ymm7
	vmulpd			%ymm11, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm3, %ymm3

1:
	cmpl	$ 5, %r13d
	jle		2f

	vxorpd			%ymm13, %ymm13, %ymm13 // 0.0
	vmovsd			160(%r10, %r11, 1), %xmm13
	vbroadcastsd	40(%r12), %ymm12
	vmovapd			160(%r10), %ymm11

	vperm2f128		$ 0x00, %ymm4, %ymm4, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm4, %ymm4
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm4, %ymm4
	vmulpd			%ymm11, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0

	vperm2f128		$ 0x00, %ymm5, %ymm5, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm5, %ymm5
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm5, %ymm5
	vmulpd			%ymm11, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm1, %ymm1

	vperm2f128		$ 0x00, %ymm6, %ymm6, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm6, %ymm6
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm6, %ymm6
	vmulpd			%ymm11, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm2, %ymm2

	vperm2f128		$ 0x00, %ymm7, %ymm7, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm7, %ymm7
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm7, %ymm7
	vmulpd			%ymm11, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm3, %ymm3

2:

	vbroadcastsd	32(%r12), %ymm12
	vmovapd			128(%r10), %ymm11

	vperm2f128		$ 0x00, %ymm4, %ymm4, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm4, %ymm4
	vmulpd			%ymm11, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0

	vperm2f128		$ 0x00, %ymm5, %ymm5, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm5, %ymm5
	vmulpd			%ymm11, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm1, %ymm1

	vperm2f128		$ 0x00, %ymm6, %ymm6, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm6, %ymm6
	vmulpd			%ymm11, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm2, %ymm2

	vperm2f128		$ 0x00, %ymm7, %ymm7, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm7, %ymm7
	vmulpd			%ymm11, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm3, %ymm3


	// top-left

	vmovapd			96(%r10), %ymm13
	vxorpd			%ymm14, %ymm14, %ymm14 // 0.0
	vblendpd		$ 0x7, %ymm13, %ymm14, %ymm13
	vbroadcastsd	24(%r12), %ymm12

	vperm2f128		$ 0x11, %ymm0, %ymm0, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm0, %ymm0
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0

	vperm2f128		$ 0x11, %ymm1, %ymm1, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm1, %ymm1
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm1, %ymm1

	vperm2f128		$ 0x11, %ymm2, %ymm2, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm2, %ymm2
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm2, %ymm2

	vperm2f128		$ 0x11, %ymm3, %ymm3, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm3, %ymm3
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm3, %ymm3


	vxorpd			%ymm13, %ymm13, %ymm13 // 0.0
	vmovapd			64(%r10), %xmm13
	vbroadcastsd	16(%r12), %ymm12

	vperm2f128		$ 0x11, %ymm0, %ymm0, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm0, %ymm0
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0

	vperm2f128		$ 0x11, %ymm1, %ymm1, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm1, %ymm1
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm1, %ymm1

	vperm2f128		$ 0x11, %ymm2, %ymm2, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm2, %ymm2
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm2, %ymm2

	vperm2f128		$ 0x11, %ymm3, %ymm3, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm3, %ymm3
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm3, %ymm3


	vxorpd			%ymm13, %ymm13, %ymm13 // 0.0
	vmovsd			32(%r10), %xmm13
	vbroadcastsd	8(%r12), %ymm12

	vpermilpd		$ 0xf, %ymm0, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm0, %ymm0
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0

	vpermilpd		$ 0xf, %ymm1, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm1, %ymm1
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm1, %ymm1

	vpermilpd		$ 0xf, %ymm2, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm2, %ymm2
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm2, %ymm2

	vpermilpd		$ 0xf, %ymm3, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm3, %ymm3
	vmulpd			%ymm13, %ymm14, %ymm15
	vsubpd			%ymm15, %ymm3, %ymm3


	vbroadcastsd	0(%r12), %ymm12

	vmulpd			%ymm0, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm0, %ymm0

	vmulpd			%ymm1, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm1, %ymm1

	vmulpd			%ymm2, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm2, %ymm2

	vmulpd			%ymm3, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm3, %ymm3

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_lun_inv_8x4_vs_lib4)
#endif





// common inner routine with file scope
//
// LU factorization without pivoting
// left kernel
//
// input arguments:
// r10  <- inv_diag_E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d50 d60 d70]
// ymm5 <- [d41 d51 d61 d71]
// ymm6 <- [d42 d52 d62 d72]
// ymm7 <- [d43 d53 d63 d73]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10  <- inv_diag_E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d50 d60 d70]
// ymm5 <- [d41 d51 d61 d71]
// ymm6 <- [d42 d52 d62 d72]
// ymm7 <- [d43 d53 d63 d73]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DGETRF_L_8X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dgetrf_l_8x4_lib4)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC04(%rip), %xmm14 // 1.0
#elif defined(OS_MAC)
	vmovsd			LC04(%rip), %xmm14 // 1.0
#endif
//	vmovddup		%xmm14, %xmm14

	// first column
//	vblendpd		$ 0x1, %ymm0, %ymm12, %ymm12
	vmovapd			%ymm0, %ymm12
	vdivsd			%xmm0, %xmm14, %xmm13
//	vpermpd			$ 0x00, %ymm13, %ymm13
	vmovddup		%xmm13, %xmm13
	vperm2f128		$ 0x00, %ymm13, %ymm13, %ymm13
	vmovsd			%xmm13, 0(%r10)
	vmulpd			%ymm0, %ymm13, %ymm0
	vmulpd			%ymm4, %ymm13, %ymm4
	vblendpd		$ 0x1, %ymm12, %ymm0, %ymm0

	// second column
//	vpermpd			$ 0x00, %ymm1, %ymm13
	vmovddup		%xmm1, %xmm13
	vperm2f128		$ 0x00, %ymm13, %ymm13, %ymm13
	vmulpd			%ymm0, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm4, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm5, %ymm5
	vblendpd		$ 0x2, %ymm1, %ymm13, %ymm12

	vpermilpd		$ 0x3, %xmm1, %xmm13
	vdivsd			%xmm13, %xmm14, %xmm13
//	vpermpd			$ 0x00, %ymm13, %ymm13
	vmovddup		%xmm13, %xmm13
	vperm2f128		$ 0x00, %ymm13, %ymm13, %ymm13
	vmovsd			%xmm13, 8(%r10)
	vmulpd			%ymm1, %ymm13, %ymm1
	vmulpd			%ymm5, %ymm13, %ymm5
	vblendpd		$ 0x3, %ymm12, %ymm1, %ymm1

	// third column
//	vpermpd			$ 0x00, %ymm2, %ymm13
	vmovddup		%xmm2, %xmm13
	vperm2f128		$ 0x00, %ymm13, %ymm13, %ymm13
	vmulpd			%ymm0, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm4, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm6, %ymm6
	vblendpd		$ 0x2, %ymm2, %ymm13, %ymm12

//	vpermpd			$ 0x55, %ymm2, %ymm13
	vperm2f128		$ 0x00, %ymm2, %ymm2, %ymm13
	vpermilpd		$ 0xf, %ymm13, %ymm13
	vmulpd			%ymm1, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm5, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm6, %ymm6
	vblendpd		$ 0x4, %ymm2, %ymm12, %ymm12

//	vpermpd			$ 0xaa, %ymm2, %ymm13
	vperm2f128		$ 0x11, %ymm2, %ymm2, %ymm13
	vpermilpd		$ 0x0, %ymm13, %ymm13
	vdivsd			%xmm13, %xmm14, %xmm13
//	vpermpd			$ 0x00, %ymm13, %ymm13
	vmovddup		%xmm13, %xmm13
	vperm2f128		$ 0x00, %ymm13, %ymm13, %ymm13
	vmovsd			%xmm13, 16(%r10)
	vmulpd			%ymm2, %ymm13, %ymm2
	vmulpd			%ymm6, %ymm13, %ymm6
	vblendpd		$ 0x7, %ymm12, %ymm2, %ymm2

	// fourth column
//	vpermpd			$ 0x00, %ymm3, %ymm13
	vmovddup		%xmm3, %xmm13
	vperm2f128		$ 0x00, %ymm13, %ymm13, %ymm13
	vmulpd			%ymm0, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm4, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm7, %ymm7
	vblendpd		$ 0x2, %ymm3, %ymm13, %ymm12

//	vpermpd			$ 0x55, %ymm3, %ymm13
	vperm2f128		$ 0x00, %ymm3, %ymm3, %ymm13
	vpermilpd		$ 0xf, %ymm13, %ymm13
	vmulpd			%ymm1, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm5, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm7, %ymm7
	vblendpd		$ 0x4, %ymm3, %ymm12, %ymm12

//	vpermpd			$ 0xaa, %ymm3, %ymm13
	vperm2f128		$ 0x11, %ymm3, %ymm3, %ymm11
	vpermilpd		$ 0x0, %ymm11, %ymm13
	vmulpd			%ymm2, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm6, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm7, %ymm7
	vblendpd		$ 0x8, %ymm3, %ymm12, %ymm12

//	vpermpd			$ 0xff, %ymm3, %ymm13
	vperm2f128		$ 0x11, %ymm3, %ymm3, %ymm11
	vpermilpd		$ 0xf, %ymm11, %ymm13
	vdivsd			%xmm13, %xmm14, %xmm13
//	vpermpd			$ 0x00, %ymm13, %ymm13
	vmovddup		%xmm13, %xmm13
	vperm2f128		$ 0x00, %ymm13, %ymm13, %ymm13
	vmovsd			%xmm13, 24(%r10)
//	vmulpd			%ymm3, %ymm13, %ymm3
	vmulpd			%ymm7, %ymm13, %ymm7
	vblendpd		$ 0x7, %ymm12, %ymm3, %ymm3

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dgetrf_l_8x4_lib4)
#endif





// common inner routine with file scope
//
// LU factorization without pivoting
// right kernel
//
// input arguments:
// r10  <- E
// r11  <- sde
// r12  <- inv_diag_D
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d50 d60 d70]
// ymm5 <- [d41 d51 d61 d71]
// ymm6 <- [d42 d52 d62 d72]
// ymm7 <- [d43 d53 d63 d73]
//
// output arguments:
// r10  <- E
// r11  <- sde
// r12  <- inv_diag_D
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d50 d60 d70]
// ymm5 <- [d41 d51 d61 d71]
// ymm6 <- [d42 d52 d62 d72]
// ymm7 <- [d43 d53 d63 d73]

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DGETRF_R_8X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dgetrf_r_8x4_lib4)
#endif
	
	// solve upper 4x4 & correct lower 4x4

	vxorpd			%ymm15, %ymm15, %ymm15
	vmovapd			0(%r10), %ymm12
	vblendpd		$ 0x1, %ymm15, %ymm12, %ymm12
	vmovapd			0(%r10, %r11), %ymm14
//	vpermpd			$ 0x00, %ymm0, %ymm13
	vmovddup		%xmm0, %xmm13
	vperm2f128		$ 0x00, %ymm13, %ymm13, %ymm13
	vmulpd			%ymm12, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm14, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm4, %ymm4
//	vpermpd			$ 0x00, %ymm1, %ymm13
	vmovddup		%xmm1, %xmm13
	vperm2f128		$ 0x00, %ymm13, %ymm13, %ymm13
	vmulpd			%ymm12, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm14, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm5, %ymm5
//	vpermpd			$ 0x00, %ymm2, %ymm13
	vmovddup		%xmm2, %xmm13
	vperm2f128		$ 0x00, %ymm13, %ymm13, %ymm13
	vmulpd			%ymm12, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm14, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm6, %ymm6
//	vpermpd			$ 0x00, %ymm3, %ymm13
	vmovddup		%xmm3, %xmm13
	vperm2f128		$ 0x00, %ymm13, %ymm13, %ymm13
	vmulpd			%ymm12, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm14, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm7, %ymm7

	vxorpd			%ymm15, %ymm15, %ymm15
	vmovapd			32(%r10), %ymm12
	vblendpd		$ 0x3, %ymm15, %ymm12, %ymm12
	vmovapd			32(%r10, %r11), %ymm14
//	vpermpd			$ 0x55, %ymm0, %ymm13
	vperm2f128		$ 0x00, %ymm0, %ymm0, %ymm13
	vpermilpd		$ 0xf, %ymm13, %ymm13
	vmulpd			%ymm12, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm14, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm4, %ymm4
//	vpermpd			$ 0x55, %ymm1, %ymm13
	vperm2f128		$ 0x00, %ymm1, %ymm1, %ymm13
	vpermilpd		$ 0xf, %ymm13, %ymm13
	vmulpd			%ymm12, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm14, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm5, %ymm5
//	vpermpd			$ 0x55, %ymm2, %ymm13
	vperm2f128		$ 0x00, %ymm2, %ymm2, %ymm13
	vpermilpd		$ 0xf, %ymm13, %ymm13
	vmulpd			%ymm12, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm14, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm6, %ymm6
//	vpermpd			$ 0x55, %ymm3, %ymm13
	vperm2f128		$ 0x00, %ymm3, %ymm3, %ymm13
	vpermilpd		$ 0xf, %ymm13, %ymm13
	vmulpd			%ymm12, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm14, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm7, %ymm7

	vxorpd			%ymm15, %ymm15, %ymm15
	vmovapd			64(%r10), %ymm12
	vblendpd		$ 0x7, %ymm15, %ymm12, %ymm12
	vmovapd			64(%r10, %r11), %ymm14
//	vpermpd			$ 0xaa, %ymm0, %ymm13
	vperm2f128		$ 0x11, %ymm0, %ymm0, %ymm13
	vpermilpd		$ 0x0, %ymm13, %ymm13
	vmulpd			%ymm12, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm14, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm4, %ymm4
//	vpermpd			$ 0xaa, %ymm1, %ymm13
	vperm2f128		$ 0x11, %ymm1, %ymm1, %ymm13
	vpermilpd		$ 0x0, %ymm13, %ymm13
	vmulpd			%ymm12, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm14, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm5, %ymm5
//	vpermpd			$ 0xaa, %ymm2, %ymm13
	vperm2f128		$ 0x11, %ymm2, %ymm2, %ymm13
	vpermilpd		$ 0x0, %ymm13, %ymm13
	vmulpd			%ymm12, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm14, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm6, %ymm6
//	vpermpd			$ 0xaa, %ymm3, %ymm13
	vperm2f128		$ 0x11, %ymm3, %ymm3, %ymm13
	vpermilpd		$ 0x0, %ymm13, %ymm13
	vmulpd			%ymm12, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm14, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm7, %ymm7

	vmovapd			96(%r10, %r11), %ymm14
//	vpermpd			$ 0xff, %ymm0, %ymm13
	vperm2f128		$ 0x11, %ymm0, %ymm0, %ymm13
	vpermilpd		$ 0xf, %ymm13, %ymm13
	vmulpd			%ymm14, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm4, %ymm4
//	vpermpd			$ 0xff, %ymm1, %ymm13
	vperm2f128		$ 0x11, %ymm1, %ymm1, %ymm13
	vpermilpd		$ 0xf, %ymm13, %ymm13
	vmulpd			%ymm14, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm5, %ymm5
//	vpermpd			$ 0xff, %ymm2, %ymm13
	vperm2f128		$ 0x11, %ymm2, %ymm2, %ymm13
	vpermilpd		$ 0xf, %ymm13, %ymm13
	vmulpd			%ymm14, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm6, %ymm6
//	vpermpd			$ 0xff, %ymm3, %ymm13
	vperm2f128		$ 0x11, %ymm3, %ymm3, %ymm13
	vpermilpd		$ 0xf, %ymm13, %ymm13
	vmulpd			%ymm14, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm7, %ymm7


	// factorize lower 4x4

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC04(%rip), %xmm14 // 1.0
#elif defined(OS_MAC)
	vmovsd			LC04(%rip), %xmm14 // 1.0
#endif
//	vmovddup		%xmm14, %xmm14

	// first column
//	vblendpd		$ 0x1, %ymm4, %ymm12, %ymm12
	vmovapd			%ymm4, %ymm12
	vdivsd			%xmm4, %xmm14, %xmm13
//	vpermpd			$ 0x00, %ymm13, %ymm13
	vmovddup		%xmm13, %xmm13
	vperm2f128		$ 0x00, %ymm13, %ymm13, %ymm13
	vmovsd			%xmm13, 0(%r12)
	vmulpd			%ymm4, %ymm13, %ymm4
	vblendpd		$ 0x1, %ymm12, %ymm4, %ymm4

	// second column
//	vpermpd			$ 0x00, %ymm5, %ymm13
	vmovddup		%xmm5, %xmm13
	vperm2f128		$ 0x00, %ymm13, %ymm13, %ymm13
	vmulpd			%ymm4, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm5, %ymm5
	vblendpd		$ 0x2, %ymm5, %ymm13, %ymm12

	vpermilpd		$ 0x3, %xmm5, %xmm13
	vdivsd			%xmm13, %xmm14, %xmm13
//	vpermpd			$ 0x00, %ymm13, %ymm13
	vmovddup		%xmm13, %xmm13
	vperm2f128		$ 0x00, %ymm13, %ymm13, %ymm13
	vmovsd			%xmm13, 8(%r12)
	vmulpd			%ymm5, %ymm13, %ymm5
	vblendpd		$ 0x3, %ymm12, %ymm5, %ymm5

	// third column
//	vpermpd			$ 0x00, %ymm6, %ymm13
	vmovddup		%xmm6, %xmm13
	vperm2f128		$ 0x00, %ymm13, %ymm13, %ymm13
	vmulpd			%ymm4, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm6, %ymm6
	vblendpd		$ 0x2, %ymm6, %ymm13, %ymm12

//	vpermpd			$ 0x55, %ymm6, %ymm13
	vperm2f128		$ 0x00, %ymm6, %ymm6, %ymm13
	vpermilpd		$ 0xf, %ymm13, %ymm13
	vmulpd			%ymm5, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm6, %ymm6
	vblendpd		$ 0x4, %ymm6, %ymm12, %ymm12

//	vpermpd			$ 0xaa, %ymm6, %ymm13
	vperm2f128		$ 0x11, %ymm6, %ymm6, %ymm13
	vpermilpd		$ 0x0, %ymm13, %ymm13
	vdivsd			%xmm13, %xmm14, %xmm13
//	vpermpd			$ 0x00, %ymm13, %ymm13
	vmovddup		%xmm13, %xmm13
	vperm2f128		$ 0x00, %ymm13, %ymm13, %ymm13
	vmovsd			%xmm13, 16(%r12)
	vmulpd			%ymm6, %ymm13, %ymm6
	vblendpd		$ 0x7, %ymm12, %ymm6, %ymm6

	// fourth column
//	vpermpd			$ 0x00, %ymm7, %ymm13
	vmovddup		%xmm7, %xmm13
	vperm2f128		$ 0x00, %ymm13, %ymm13, %ymm13
	vmulpd			%ymm4, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm7, %ymm7
	vblendpd		$ 0x2, %ymm7, %ymm13, %ymm12

//	vpermpd			$ 0x55, %ymm7, %ymm13
	vperm2f128		$ 0x00, %ymm7, %ymm7, %ymm13
	vpermilpd		$ 0xf, %ymm13, %ymm13
	vmulpd			%ymm5, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm7, %ymm7
	vblendpd		$ 0x4, %ymm7, %ymm12, %ymm12

//	vpermpd			$ 0xaa, %ymm7, %ymm13
	vperm2f128		$ 0x11, %ymm7, %ymm7, %ymm11
	vpermilpd		$ 0x0, %ymm11, %ymm13
	vmulpd			%ymm6, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm7, %ymm7
	vblendpd		$ 0x8, %ymm7, %ymm12, %ymm12
	
//	vpermpd			$ 0xff, %ymm7, %ymm13
	vperm2f128		$ 0x11, %ymm7, %ymm7, %ymm11
	vpermilpd		$ 0xf, %ymm11, %ymm13
	vdivsd			%xmm13, %xmm14, %xmm13
//	vpermpd			$ 0x00, %ymm13, %ymm13
	vmovddup		%xmm13, %xmm13
	vperm2f128		$ 0x00, %ymm13, %ymm13, %ymm13
	vmovsd			%xmm13, 24(%r12)
//	vmulpd			%ymm7, %ymm13, %ymm7
	vblendpd		$ 0x7, %ymm12, %ymm7, %ymm7

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dgetrf_r_8x4_lib4)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// r11  <- 4*sdd*sizeof(double)
// r15  <- dirty
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d50 d60 d70]
// ymm5 <- [d41 d51 d61 d71]
// ymm6 <- [d42 d52 d62 d72]
// ymm7 <- [d43 d53 d63 d73]
//
// output arguments:
// r10  <- D
// r11  <- 4*sdd*sizeof(double)
// r15  <- dirty
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d50 d60 d70]
// ymm5 <- [d41 d51 d61 d71]
// ymm6 <- [d42 d52 d62 d72]
// ymm7 <- [d43 d53 d63 d73]

#if MACRO_LEVEL>=1
	.macro INNER_STORE_8X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_8x4_lib4)
#endif

	vmovapd %ymm0,  0(%r10)
	vmovapd %ymm1, 32(%r10)
	vmovapd %ymm2, 64(%r10)
	vmovapd %ymm3, 96(%r10)

	vmovapd %ymm4,  0(%r10, %r11, 1)
	vmovapd %ymm5, 32(%r10, %r11, 1)
	vmovapd %ymm6, 64(%r10, %r11, 1)
	vmovapd %ymm7, 96(%r10, %r11, 1)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_8x4_lib4)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10   <- D
// r11   <- 4*sdd*sizeof(double)
// r14   <- dirty
// r15   <- dirty
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- [d80 d90 da0 db0]
// ymm9  <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]
//
// output arguments:
// r10   <- D
// r11   <- 4*sdd*sizeof(double)
// r14   <- dirty
// r15   <- dirty
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- [d80 d90 da0 db0]
// ymm9  <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X8_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_4x8_lib4)
#endif

	vmovapd %ymm0,   0(%r10)
	vmovapd %ymm1,  32(%r10)
	vmovapd %ymm2,  64(%r10)
	vmovapd %ymm3,  96(%r10)

	vmovapd %ymm4, 128(%r10)
	vmovapd %ymm5, 160(%r10)
	vmovapd %ymm6, 192(%r10)
	vmovapd %ymm7, 224(%r10)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4x8_lib4)
#endif





// common inner routine with file scope
//
// store lower n
//
// input arguments:
// r10  <- D
// r11  <- 4*sdd*sizeof(double)
// r15  <- dirty
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d50 d60 d70]
// ymm5 <- [d41 d50 d61 d71]
// ymm6 <- [d42 d52 d62 d72]
// ymm7 <- [d43 d53 d63 d73]
// ymm14 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10  <- D
// r11  <- 4*sdd*sizeof(double)
// r15  <- dirty
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d50 d60 d70]
// ymm5 <- [d41 d51 d61 d71]
// ymm6 <- [d42 d52 d62 d72]
// ymm7 <- [d43 d53 d63 d73]
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_8X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_l_8x4_lib4)
#endif

	vmovapd		%ymm0,0(%r10)
	vmovapd		32(%r10), %ymm14
	vblendpd	$ 0x1, %ymm14, %ymm1, %ymm1
	vmovapd		%ymm1, 32(%r10)
	vmovapd		64(%r10), %ymm14
	vblendpd	$ 0x3, %ymm14, %ymm2, %ymm2
	vmovapd		%ymm2, 64(%r10)
	vmovapd		96(%r10), %ymm14
	vblendpd	$ 0x7, %ymm14, %ymm3, %ymm3
	vmovapd		%ymm3, 96(%r10)

	vmovapd		%ymm4, 0(%r10, %r11, 1)
	vmovapd		%ymm5, 32(%r10, %r11, 1)
	vmovapd		%ymm6, 64(%r10, %r11, 1)
	vmovapd		%ymm7, 96(%r10, %r11, 1)

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_8x4_lib4)
#endif





// common inner routine with file scope
//
// store n upper triangular
//
// input arguments:
// r10   <- D
// r11  <- 4*sdd*sizeof(double)
// ymm0  <- [d00 d11 d22 d33]
// ymm1  <- [d01 d10 d23 d32]
// ymm2  <- [d03 d12 d21 d30]
// ymm3  <- [d02 d13 d20 d31]
// ymm4 <- [d40 d50 d60 d70]
// ymm5 <- [d41 d50 d61 d71]
// ymm6 <- [d42 d52 d62 d72]
// ymm7 <- [d43 d53 d63 d73]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_U_8X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_u_8x4_lib4)
#endif
	
	//
	vmovapd		0(%r10, %r11), %ymm14
	vblendpd	$ 0x1, %ymm4, %ymm14, %ymm4
	vmovapd		%ymm0, 0(%r10)
	vmovapd		%ymm4, 0(%r10, %r11)
	//
	vmovapd		32(%r10, %r11), %ymm14
	vblendpd	$ 0x3, %ymm5, %ymm14, %ymm5
	vmovapd		%ymm1, 32(%r10)
	vmovapd		%ymm5, 32(%r10, %r11)
	//
	vmovapd		64(%r10, %r11), %ymm14
	vblendpd	$ 0x7, %ymm6, %ymm14, %ymm6
	vmovapd		%ymm2, 64(%r10)
	vmovapd		%ymm6, 64(%r10, %r11)
	//
	vmovapd		%ymm3, 96(%r10)
	vmovapd		%ymm7, 96(%r10, %r11)

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_u_8x4_lib4)
#endif





// common inner routine with file scope
//
// store n vs
//
// input arguments:
// r10  <- D
// r11  <- 4*sdd*sizeof(double)
// r12d  <- km
// r13d  <- kn
// r15  <- dirty
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d50 d60 d70]
// ymm5 <- [d41 d50 d61 d71]
// ymm6 <- [d42 d52 d62 d72]
// ymm7 <- [d43 d53 d63 d73]
// ymm14 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10  <- D
// r11  <- 4*sdd*sizeof(double)
// r12d  <- km
// r13d  <- kn
// r15  <- dirty
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d50 d60 d70]
// ymm5 <- [d41 d51 d61 d71]
// ymm6 <- [d42 d52 d62 d72]
// ymm7 <- [d43 d53 d63 d73]
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_STORE_8X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_8x4_vs_lib4)
#endif

	vcvtsi2sd	%r12d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC03(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC03(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm15

	cmpl		$ 2, %r13d
	vmovapd		%ymm0, 0(%r10)
	vmaskmovpd	%ymm4, %ymm15,  0(%r10, %r11, 1)
	jl			0f // end
	cmpl		$ 3, %r13d
	vmovapd		%ymm1, 32(%r10)
	vmaskmovpd	%ymm5, %ymm15, 32(%r10, %r11, 1)
	jl			0f // end
	vmovapd		%ymm2, 64(%r10)
	vmaskmovpd	%ymm6, %ymm15, 64(%r10, %r11, 1)
	je			0f // end
	vmovapd		%ymm3, 96(%r10)
	vmaskmovpd	%ymm7, %ymm15, 96(%r10, %r11, 1)

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_8x4_vs_lib4)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10   <- D
// r11   <- m1
// r12   <- n1
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- [d80 d90 da0 db0]
// ymm9  <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]
//
// output arguments:
// r10   <- D
// r11   <- m1
// r12   <- n1
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- [d80 d90 da0 db0]
// ymm9  <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X8_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_4x8_vs_lib4)
#endif

	vcvtsi2sd	%r11d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC02(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC02(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm15

	vmaskmovpd	%ymm0, %ymm15,   0(%r10)
	vmaskmovpd	%ymm1, %ymm15,  32(%r10)
	vmaskmovpd	%ymm2, %ymm15,  64(%r10)
	vmaskmovpd	%ymm3, %ymm15,  96(%r10)

	vmaskmovpd	%ymm4, %ymm15, 128(%r10)
	cmpl		$ 6, %r12d
	jl			0f // end
	vmaskmovpd	%ymm5, %ymm15, 160(%r10)
	cmpl		$ 7, %r12d
	jl			0f // end
	vmaskmovpd	%ymm6, %ymm15, 192(%r10)
	je			0f // end
	vmaskmovpd	%ymm7, %ymm15, 224(%r10)

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4x8_vs_lib4)
#endif





// common inner routine with file scope
//
// store lower n vs
//
// input arguments:
// r10  <- D
// r11  <- 4*sdd*sizeof(double)
// r12d  <- km
// r13d  <- kn
// r15  <- dirty
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d50 d60 d70]
// ymm5 <- [d41 d50 d61 d71]
// ymm6 <- [d42 d52 d62 d72]
// ymm7 <- [d43 d53 d63 d73]
// ymm14 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10  <- D
// r11  <- 4*sdd*sizeof(double)
// r12d  <- km
// r13d  <- kn
// r15  <- dirty
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d50 d60 d70]
// ymm5 <- [d41 d51 d61 d71]
// ymm6 <- [d42 d52 d62 d72]
// ymm7 <- [d43 d53 d63 d73]
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_8X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_l_8x4_vs_lib4)
#endif

	vcvtsi2sd	%r12d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC03(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC03(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm15

	cmpl		$ 2, %r13d
	vmovapd		%ymm0, 0(%r10)
	vmaskmovpd	%ymm4, %ymm15,  0(%r10, %r11, 1)
	jl			0f // end
	cmpl		$ 3, %r13d
	vmovapd		32(%r10), %ymm14
	vblendpd	$ 0x1, %ymm14, %ymm1, %ymm1
	vmovapd		%ymm1, 32(%r10)
	vmaskmovpd	%ymm5, %ymm15, 32(%r10, %r11, 1)
	jl			0f // end
	vmovapd		64(%r10), %ymm14
	vblendpd	$ 0x3, %ymm14, %ymm2, %ymm2
	vmovapd		%ymm2, 64(%r10)
	vmaskmovpd	%ymm6, %ymm15, 64(%r10, %r11, 1)
	je			0f // end
	vmovapd		96(%r10), %ymm14
	vblendpd	$ 0x7, %ymm14, %ymm3, %ymm3
	vmovapd		%ymm3, 96(%r10)
	vmaskmovpd	%ymm7, %ymm15, 96(%r10, %r11, 1)
0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_8x4_vs_lib4)
#endif





// common inner routine with file scope
//
// store upper n vs
//
// input arguments:
// r10  <- D
// r11  <- 4*sdd*sizeof(double)
// r12d  <- km
// r13d  <- kn
// r15  <- dirty
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d50 d60 d70]
// ymm5 <- [d41 d50 d61 d71]
// ymm6 <- [d42 d52 d62 d72]
// ymm7 <- [d43 d53 d63 d73]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_U_8X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_u_8x4_vs_lib4)
#endif
	
	vcvtsi2sd	%r12d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC03(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC03(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm15

	vmovapd		0(%r10, %r11), %ymm14
	vblendpd	$ 0x1, %ymm4, %ymm14, %ymm4
	vmovapd		%ymm0, 0(%r10)
	vmaskmovpd	%ymm4, %ymm15,  0(%r10, %r11, 1)
	cmpl		$ 2, %r13d
	jl			0f // end
	vmovapd		32(%r10, %r11), %ymm14
	vblendpd	$ 0x3, %ymm5, %ymm14, %ymm5
	vmovapd		%ymm1, 32(%r10)
	vmaskmovpd	%ymm5, %ymm15, 32(%r10, %r11, 1)
	cmpl		$ 3, %r13d
	jl			0f // end
	vmovapd		64(%r10, %r11), %ymm14
	vblendpd	$ 0x7, %ymm6, %ymm14, %ymm6
	vmovapd		%ymm2, 64(%r10)
	vmaskmovpd	%ymm6, %ymm15, 64(%r10, %r11, 1)
	je			0f // end
	vmovapd		%ymm3, 96(%r10)
	vmaskmovpd	%ymm7, %ymm15, 96(%r10, %r11, 1)


0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_u_8x4_vs_lib4)
#endif





// common inner routine with file scope
//
// store n generalized
//
// input arguments:
// r10  <- offset
// r11  <- D
// r12  <- 4*sdd*sizeof(double)
// r13  <- m0 // row index: start from (inc)
// r14  <- m1 // row index: up to (exc)
// r15  <- n0 // col index: start from (inc)
// rax  <- n1 // col index: up to (exc)
// rbx  <- dirty
// rbp  <- dirty
// ymm0 <- [d00 d11 d22 d33]
// ymm1 <- [d01 d10 d23 d32]
// ymm2 <- [d03 d12 d21 d30]
// ymm3 <- [d02 d13 d20 d31]
// ymm4 <- [d40 d50 d60 d70]
// ymm5 <- [d41 d51 d61 d71]
// ymm6 <- [d42 d52 d62 d72]
// ymm7 <- [d43 d53 d63 d73]
//
// output arguments:
// r10  <- offset
// r11  <- D
// r12  <- 4*sdd*sizeof(double)
// r13  <- m0 // row index: start from (inc)
// r14  <- m1 // row index: up to (exc)
// r15  <- n1-n0
// rax  <- n1-n0
// rbx  <- dirty
// rbp  <- dirty
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d50 d60 d70]
// ymm5 <- [d41 d51 d61 d71]
// ymm6 <- [d42 d52 d62 d72]
// ymm7 <- [d43 d53 d63 d73]

#if MACRO_LEVEL>=1
	.macro INNER_STORE_8X4_GEN_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_8x4_gen_lib4)
#endif

	// compute mask for rows
	vcvtsi2sd	%r13d, %xmm14, %xmm14
	vcvtsi2sd	%r14d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC02(%rip), %ymm12
	vmovupd		.LC03(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC02(%rip), %ymm12
	vmovupd		LC03(%rip), %ymm13
#endif
	vmovddup	%xmm14, %xmm14
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm14, %ymm14, %ymm14
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm12, %ymm14, %ymm14
	vsubpd		%ymm15, %ymm13, %ymm15

	// shift D and sol for cols
	cmpl	$ 0, %r15d
	jle		0f

	vmovapd		%ymm1, %ymm0
	vmovapd		%ymm5, %ymm4
	vmovapd		%ymm2, %ymm1
	vmovapd		%ymm6, %ymm5
	vmovapd		%ymm3, %ymm2
	vmovapd		%ymm7, %ymm6
	addq		$ 32, %r11

	cmpl	$ 1, %r15d
	jle		0f

	vmovapd		%ymm1, %ymm0
	vmovapd		%ymm5, %ymm4
	vmovapd		%ymm2, %ymm1
	vmovapd		%ymm6, %ymm5
	addq		$ 32, %r11

	cmpl	$ 2, %r15d
	jle		0f

	vmovapd		%ymm1, %ymm0
	vmovapd		%ymm5, %ymm4
	addq		$ 32, %r11

0:

	// compute number of cols
	cmpl	$ 4, %eax
	jle		0f
	movl	$ 4, %eax
0:
	subl	%r15d, %eax
	movl	%eax, %r15d

	cmpl	$ 0, %r10d
	jg		0f

	// offset==0

	cmpl		$ 2, %r15d
	vmaskmovpd	%ymm0, %ymm14,  0(%r11)
	vmaskmovpd	%ymm4, %ymm15,  0(%r11, %r12, 1)
	jl			4f // end
	cmpl		$ 3, %r15d
	vmaskmovpd	%ymm1, %ymm14, 32(%r11)
	vmaskmovpd	%ymm5, %ymm15, 32(%r11, %r12, 1)
	jl			4f // end
	vmaskmovpd	%ymm2, %ymm14, 64(%r11)
	vmaskmovpd	%ymm6, %ymm15, 64(%r11, %r12, 1)
	je			4f // end
	vmaskmovpd	%ymm3, %ymm14, 96(%r11)
	vmaskmovpd	%ymm7, %ymm15, 96(%r11, %r12, 1)

	jmp		4f

0:

	cmpl	$ 1, %r10d
	jg		1f

	// offset==1

	vmovapd		%ymm0, %ymm13
	vperm2f128	$ 0x03, %ymm4, %ymm0, %ymm12
	vshufpd		$ 0x5, %ymm0, %ymm12, %ymm0
	vperm2f128	$ 0x03, %ymm13, %ymm4, %ymm12
	vshufpd		$ 0x5, %ymm4, %ymm12, %ymm4

	vmovapd		%ymm1, %ymm13
	vperm2f128	$ 0x03, %ymm5, %ymm1, %ymm12
	vshufpd		$ 0x5, %ymm1, %ymm12, %ymm1
	vperm2f128	$ 0x03, %ymm13, %ymm5, %ymm12
	vshufpd		$ 0x5, %ymm5, %ymm12, %ymm5

	vmovapd		%ymm2, %ymm13
	vperm2f128	$ 0x03, %ymm6, %ymm2, %ymm12
	vshufpd		$ 0x5, %ymm2, %ymm12, %ymm2
	vperm2f128	$ 0x03, %ymm13, %ymm6, %ymm12
	vshufpd		$ 0x5, %ymm6, %ymm12, %ymm6

	vmovapd		%ymm3, %ymm13
	vperm2f128	$ 0x03, %ymm7, %ymm3, %ymm12
	vshufpd		$ 0x5, %ymm3, %ymm12, %ymm3
	vperm2f128	$ 0x03, %ymm13, %ymm7, %ymm12
	vshufpd		$ 0x5, %ymm7, %ymm12, %ymm7

	vperm2f128	$ 0x01, %ymm15, %ymm15, %ymm12
	vshufpd		$ 0x5, %ymm15, %ymm12, %ymm15
	vperm2f128	$ 0x01, %ymm14, %ymm14, %ymm12
	vshufpd		$ 0x5, %ymm14, %ymm12, %ymm14

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC08(%rip), %ymm12
	vmovupd		.LC05(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC08(%rip), %ymm12
	vmovupd		LC05(%rip), %ymm13
#endif
	vandpd		%ymm12, %ymm14, %ymm12
	vandpd		%ymm13, %ymm15, %ymm13

	vblendpd	$ 0x1, %ymm14, %ymm15, %ymm14

	jmp		3f

1:

	cmpl	$ 2, %r10d
	jg		2f

	// offset==2

	vmovapd		%ymm0, %ymm13
	vperm2f128	$ 0x03, %ymm4, %ymm0, %ymm0
	vperm2f128	$ 0x03, %ymm13, %ymm4, %ymm4

	vmovapd		%ymm1, %ymm13
	vperm2f128	$ 0x03, %ymm5, %ymm1, %ymm1
	vperm2f128	$ 0x03, %ymm13, %ymm5, %ymm5

	vmovapd		%ymm2, %ymm13
	vperm2f128	$ 0x03, %ymm6, %ymm2, %ymm2
	vperm2f128	$ 0x03, %ymm13, %ymm6, %ymm6

	vmovapd		%ymm3, %ymm13
	vperm2f128	$ 0x03, %ymm7, %ymm3, %ymm3
	vperm2f128	$ 0x03, %ymm13, %ymm7, %ymm7

	vperm2f128	$ 0x01, %ymm14, %ymm14, %ymm14
	vperm2f128	$ 0x01, %ymm15, %ymm15, %ymm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC09(%rip), %ymm12
	vmovupd		.LC06(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC09(%rip), %ymm12
	vmovupd		LC06(%rip), %ymm13
#endif
	vandpd		%ymm12, %ymm14, %ymm12
	vandpd		%ymm13, %ymm15, %ymm13

	vblendpd	$ 0x3, %ymm14, %ymm15, %ymm14

	jmp		3f

2:

	// offset==3

	vmovapd		%ymm0, %ymm13
	vperm2f128	$ 0x21, %ymm0, %ymm4, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm4, %ymm0
	vperm2f128	$ 0x21, %ymm4, %ymm13, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm13, %ymm4

	vmovapd		%ymm1, %ymm13
	vperm2f128	$ 0x21, %ymm1, %ymm5, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm5, %ymm1
	vperm2f128	$ 0x21, %ymm5, %ymm13, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm13, %ymm5

	vmovapd		%ymm2, %ymm13
	vperm2f128	$ 0x21, %ymm2, %ymm6, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm6, %ymm2
	vperm2f128	$ 0x21, %ymm6, %ymm13, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm13, %ymm6

	vmovapd		%ymm3, %ymm13
	vperm2f128	$ 0x21, %ymm3, %ymm7, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm7, %ymm3
	vperm2f128	$ 0x21, %ymm7, %ymm13, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm13, %ymm7

	vperm2f128	$ 0x01, %ymm14, %ymm14, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm14, %ymm14
	vperm2f128	$ 0x01, %ymm15, %ymm15, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm15, %ymm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC10(%rip), %ymm12
	vmovupd		.LC07(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC10(%rip), %ymm12
	vmovupd		LC07(%rip), %ymm13
#endif
	vandpd		%ymm12, %ymm14, %ymm12
	vandpd		%ymm13, %ymm15, %ymm13

	vblendpd	$ 0x7, %ymm14, %ymm15, %ymm14

3:

	cmpl		$ 2, %r15d
	vmaskmovpd	%ymm0, %ymm12, 0(%r11)
	vmaskmovpd	%ymm4, %ymm14, 0(%r11, %r12, 1)
	vmaskmovpd	%ymm0, %ymm13, 0(%r11, %r12, 2)
	jl			4f // end
	cmpl		$ 3, %r15d
	vmaskmovpd	%ymm1, %ymm12, 32(%r11)
	vmaskmovpd	%ymm5, %ymm14, 32(%r11, %r12, 1)
	vmaskmovpd	%ymm1, %ymm13, 32(%r11, %r12, 2)
	jl			4f // end
	vmaskmovpd	%ymm2, %ymm12, 64(%r11)
	vmaskmovpd	%ymm6, %ymm14, 64(%r11, %r12, 1)
	vmaskmovpd	%ymm2, %ymm13, 64(%r11, %r12, 2)
	je			4f // end
	vmaskmovpd	%ymm3, %ymm12, 96(%r11)
	vmaskmovpd	%ymm7, %ymm14, 96(%r11, %r12, 1)
	vmaskmovpd	%ymm3, %ymm13, 96(%r11, %r12, 2)

4:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_8x4_gen_lib4)
#endif





// common inner routine with file scope
//
// store n generalized
//
// input arguments:
// r10  <- offset
// r11  <- D
// r12  <- 4*sdd*sizeof(double)
// r13  <- m0 // row index: start from (inc)
// r14  <- m1 // row index: up to (exc)
// r15  <- n0 // col index: start from (inc)
// rax  <- n1 // col index: up to (exc)
// rbx  <- dirty
// ymm0 <- [d00 d11 d22 d33]
// ymm1 <- [d01 d10 d23 d32]
// ymm2 <- [d03 d12 d21 d30]
// ymm3 <- [d02 d13 d20 d31]
//
// output arguments:
// r10  <- offset
// r11  <- D
// r12  <- 4*sdd*sizeof(double)
// r13  <- m0 // row index: start from (inc)
// r14  <- m1 // row index: up to (exc)
// r15  <- n1-n0
// rax  <- n1-n0
// rbx  <- dirty
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X8_GEN_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_4x8_gen_lib4)
#endif

	// compute mask for rows
	vcvtsi2sd	%r13d, %xmm14, %xmm14
	vcvtsi2sd	%r14d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC02(%rip), %ymm12
#elif defined(OS_MAC)
	vmovupd		LC02(%rip), %ymm12
#endif
	vmovddup	%xmm14, %xmm14
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm14, %ymm14, %ymm14
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm12, %ymm14, %ymm14
	vsubpd		%ymm15, %ymm12, %ymm15
	vandpd		%ymm14, %ymm15, %ymm15

	// shift D and sol for cols
	cmpl	$ 0, %r15d
	jle		0f

	vmovapd		%ymm1, %ymm0
	vmovapd		%ymm2, %ymm1
	vmovapd		%ymm3, %ymm2
	vmovapd		%ymm4, %ymm3
	vmovapd		%ymm5, %ymm4
	vmovapd		%ymm6, %ymm5
	vmovapd		%ymm7, %ymm6
	addq		$ 32, %r11

	cmpl	$ 1, %r15d
	jle		0f

	vmovapd		%ymm1, %ymm0
	vmovapd		%ymm2, %ymm1
	vmovapd		%ymm3, %ymm2
	vmovapd		%ymm4, %ymm3
	vmovapd		%ymm5, %ymm4
	vmovapd		%ymm6, %ymm5
	addq		$ 32, %r11

	cmpl	$ 2, %r15d
	jle		0f

	vmovapd		%ymm1, %ymm0
	vmovapd		%ymm2, %ymm1
	vmovapd		%ymm3, %ymm2
	vmovapd		%ymm4, %ymm3
	vmovapd		%ymm5, %ymm4
	addq		$ 32, %r11

0:

	// compute number of cols
	cmpl	$ 8, %eax
	jle		0f
	movl	$ 8, %eax
0:
	subl	%r15d, %eax
	movl	%eax, %r15d

	cmpl	$ 0, %r15d
	jle		4f


	cmpl	$ 0, %r10d
	jg		0f

	// offset==0

	vmaskmovpd	%ymm0, %ymm15, 0(%r11)
	cmpl		$ 2, %r15d
	jl			4f // end
	vmaskmovpd	%ymm1, %ymm15, 32(%r11)
	cmpl		$ 3, %r15d
	jl			4f // end
	vmaskmovpd	%ymm2, %ymm15, 64(%r11)
	cmpl		$ 4, %r15d
	jl			4f // end
	vmaskmovpd	%ymm3, %ymm15, 96(%r11)
	cmpl		$ 5, %r15d
	jl			4f // end
	vmaskmovpd	%ymm4, %ymm15, 128(%r11)
	cmpl		$ 6, %r15d
	jl			4f // end
	vmaskmovpd	%ymm5, %ymm15, 160(%r11)
	cmpl		$ 7, %r15d
	jl			4f // end
	vmaskmovpd	%ymm6, %ymm15, 192(%r11)
	je			4f // end
	vmaskmovpd	%ymm7, %ymm15, 224(%r11)

	jmp		4f

0:
	
	cmpl	$ 1, %r10d
	jg		1f

	// offset==1

	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm12
	vshufpd		$ 0x5, %ymm0, %ymm12, %ymm0

	vperm2f128	$ 0x01, %ymm1, %ymm1, %ymm12
	vshufpd		$ 0x5, %ymm1, %ymm12, %ymm1

	vperm2f128	$ 0x01, %ymm2, %ymm2, %ymm12
	vshufpd		$ 0x5, %ymm2, %ymm12, %ymm2

	vperm2f128	$ 0x01, %ymm3, %ymm3, %ymm12
	vshufpd		$ 0x5, %ymm3, %ymm12, %ymm3

	vperm2f128	$ 0x01, %ymm4, %ymm4, %ymm12
	vshufpd		$ 0x5, %ymm4, %ymm12, %ymm4

	vperm2f128	$ 0x01, %ymm5, %ymm5, %ymm12
	vshufpd		$ 0x5, %ymm5, %ymm12, %ymm5

	vperm2f128	$ 0x01, %ymm6, %ymm6, %ymm12
	vshufpd		$ 0x5, %ymm6, %ymm12, %ymm6

	vperm2f128	$ 0x01, %ymm7, %ymm7, %ymm12
	vshufpd		$ 0x5, %ymm7, %ymm12, %ymm7

	vperm2f128	$ 0x01, %ymm15, %ymm15, %ymm12
	vshufpd		$ 0x5, %ymm15, %ymm12, %ymm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC08(%rip), %ymm12
	vmovupd		.LC05(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC08(%rip), %ymm12
	vmovupd		LC05(%rip), %ymm13
#endif
	vandpd		%ymm12, %ymm15, %ymm12
	vandpd		%ymm13, %ymm15, %ymm13

	jmp		3f

1:

	cmpl	$ 2, %r10d
	jg		2f

	// offset==2

	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm0

	vperm2f128	$ 0x01, %ymm1, %ymm1, %ymm1

	vperm2f128	$ 0x01, %ymm2, %ymm2, %ymm2

	vperm2f128	$ 0x01, %ymm3, %ymm3, %ymm3

	vperm2f128	$ 0x01, %ymm4, %ymm4, %ymm4

	vperm2f128	$ 0x01, %ymm5, %ymm5, %ymm5

	vperm2f128	$ 0x01, %ymm6, %ymm6, %ymm6

	vperm2f128	$ 0x01, %ymm7, %ymm7, %ymm7

	vperm2f128	$ 0x01, %ymm15, %ymm15, %ymm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC09(%rip), %ymm12
	vmovupd		.LC06(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC09(%rip), %ymm12
	vmovupd		LC06(%rip), %ymm13
#endif
	vandpd		%ymm12, %ymm15, %ymm12
	vandpd		%ymm13, %ymm15, %ymm13

	jmp		3f

2:

	// offset==3

	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm0, %ymm0

	vperm2f128	$ 0x01, %ymm1, %ymm1, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm1, %ymm1

	vperm2f128	$ 0x01, %ymm2, %ymm2, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm2, %ymm2

	vperm2f128	$ 0x01, %ymm3, %ymm3, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm3, %ymm3

	vperm2f128	$ 0x01, %ymm4, %ymm4, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm4, %ymm4

	vperm2f128	$ 0x01, %ymm5, %ymm5, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm5, %ymm5

	vperm2f128	$ 0x01, %ymm6, %ymm6, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm6, %ymm6

	vperm2f128	$ 0x01, %ymm7, %ymm7, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm7, %ymm7

	vperm2f128	$ 0x01, %ymm15, %ymm15, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm15, %ymm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC10(%rip), %ymm12
	vmovupd		.LC07(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC10(%rip), %ymm12
	vmovupd		LC07(%rip), %ymm13
#endif
	vandpd		%ymm12, %ymm15, %ymm12
	vandpd		%ymm13, %ymm15, %ymm13

3:

	vmaskmovpd	%ymm0, %ymm12, 0(%r11)
	vmaskmovpd	%ymm0, %ymm13, 0(%r11, %r12)
	cmpl		$ 2, %r15d
	jl			4f // end
	vmaskmovpd	%ymm1, %ymm12, 32(%r11)
	vmaskmovpd	%ymm1, %ymm13, 32(%r11, %r12)
	cmpl		$ 3, %r15d
	jl			4f // end
	vmaskmovpd	%ymm2, %ymm12, 64(%r11)
	vmaskmovpd	%ymm2, %ymm13, 64(%r11, %r12)
	cmpl		$ 4, %r15d
	jl			4f // end
	vmaskmovpd	%ymm3, %ymm12, 96(%r11)
	vmaskmovpd	%ymm3, %ymm13, 96(%r11, %r12)
	cmpl		$ 5, %r15d
	jl			4f // end
	vmaskmovpd	%ymm4, %ymm12, 128(%r11)
	vmaskmovpd	%ymm4, %ymm13, 128(%r11, %r12)
	cmpl		$ 6, %r15d
	jl			4f // end
	vmaskmovpd	%ymm5, %ymm12, 160(%r11)
	vmaskmovpd	%ymm5, %ymm13, 160(%r11, %r12)
	cmpl		$ 7, %r15d
	jl			4f // end
	vmaskmovpd	%ymm6, %ymm12, 192(%r11)
	vmaskmovpd	%ymm6, %ymm13, 192(%r11, %r12)
	je			4f // end
	vmaskmovpd	%ymm7, %ymm12, 224(%r11)
	vmaskmovpd	%ymm7, %ymm13, 224(%r11, %r12)

4:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4x8_gen_lib4)
#endif





// common inner routine with file scope
//
// store l generalized
//
// input arguments:
// r10  <- offset
// r11  <- D
// r12  <- 4*sdd*sizeof(double)
// r13  <- m0 // row index: start from (inc)
// r14  <- m1 // row index: up to (exc)
// r15  <- n0 // col index: start from (inc)
// rax  <- n1 // col index: up to (exc)
// rbx  <- dirty
// rbp  <- dirty
// ymm0 <- [d00 d11 d22 d33]
// ymm1 <- [d01 d10 d23 d32]
// ymm2 <- [d03 d12 d21 d30]
// ymm3 <- [d02 d13 d20 d31]
// ymm4 <- [d40 d50 d60 d70]
// ymm5 <- [d41 d51 d61 d71]
// ymm6 <- [d42 d52 d62 d72]
// ymm7 <- [d43 d53 d63 d73]
//
// output arguments:
// r10  <- offset
// r11  <- D
// r12  <- 4*sdd*sizeof(double)
// r13  <- m0 // row index: start from (inc)
// r14  <- m1 // row index: up to (exc)
// r15  <- n1-n0
// rax  <- n1-n0
// rbx  <- dirty
// rbp  <- dirty
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d50 d60 d70]
// ymm5 <- [d41 d51 d61 d71]
// ymm6 <- [d42 d52 d62 d72]
// ymm7 <- [d43 d53 d63 d73]

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_8X4_GEN_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_l_8x4_gen_lib4)
#endif
	
	// compute mask for rows
	vcvtsi2sd	%r14d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC03(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC03(%rip), %ymm13
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm13, %ymm15


	// shift D and sol for cols
	cmpl	$ 0, %r15d
	jle		0f

	vmovapd		%ymm1, %ymm0
	vmovapd		%ymm5, %ymm4
	vmovapd		%ymm2, %ymm1
	vmovapd		%ymm6, %ymm5
	vmovapd		%ymm3, %ymm2
	vmovapd		%ymm7, %ymm6
	addq		$ 32, %r11

	cmpl	$ 1, %r15d
	jle		0f

	vmovapd		%ymm1, %ymm0
	vmovapd		%ymm5, %ymm4
	vmovapd		%ymm2, %ymm1
	vmovapd		%ymm6, %ymm5
	addq		$ 32, %r11

	cmpl	$ 2, %r15d
	jle		0f

	vmovapd		%ymm1, %ymm0
	vmovapd		%ymm5, %ymm4
	addq		$ 32, %r11

0:

	// compute number of cols
	cmpl	$ 4, %eax
	jle		0f
	movl	$ 4, %eax
0:
	subl	%r15d, %eax
	movl	%eax, %r15d

	cmpl	$ 0, %r15d
	jle		3f


	// select offset
	cmpl	$ 0, %r10d
	je		10f
	cmpl	$ 1, %r10d
	je		11f
	cmpl	$ 2, %r10d
	je		12f
	cmpl	$ 3, %r10d
	je		13f

	jmp 3f


	// offset==0
10:
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd		.LC04(%rip), %ymm13
	vmovapd		.LC12(%rip), %ymm14
#elif defined(OS_MAC)
	vmovapd		LC04(%rip), %ymm13
	vmovapd		LC12(%rip), %ymm14
#endif

	// select m0
	cmpl	$ 0, %r13d
	jle		20f
	cmpl	$ 1, %r13d
	je		21f
	cmpl	$ 2, %r13d
	je		22f
	cmpl	$ 3, %r13d
	je		23f

	jmp 3f

20:
	vmaskmovpd	%ymm0, %ymm14,  0(%r11)
	vmaskmovpd	%ymm4, %ymm15,  0(%r11, %r12, 1)
	cmpl		$ 2, %r15d
	jl			3f // end
	vblendpd	$ 0x1, %ymm13, %ymm14, %ymm14
	vmaskmovpd	%ymm1, %ymm14, 32(%r11)
	vmaskmovpd	%ymm5, %ymm15, 32(%r11, %r12, 1)
	cmpl		$ 3, %r15d
	jl			3f // end
	vblendpd	$ 0x2, %ymm13, %ymm14, %ymm14
	vmaskmovpd	%ymm2, %ymm14, 64(%r11)
	vmaskmovpd	%ymm6, %ymm15, 64(%r11, %r12, 1)
	cmpl		$ 4, %r15d
	jl			3f // end
	vblendpd	$ 0x4, %ymm13, %ymm14, %ymm14
	vmaskmovpd	%ymm3, %ymm14, 96(%r11)
	vmaskmovpd	%ymm7, %ymm15, 96(%r11, %r12, 1)

	jmp 3f

21:
	vblendpd	$ 0x1, %ymm13, %ymm14, %ymm14
	vmaskmovpd	%ymm0, %ymm14,  0(%r11)
	vmaskmovpd	%ymm4, %ymm15,  0(%r11, %r12, 1)
	cmpl		$ 2, %r15d
	jl			3f // end
	vblendpd	$ 0x2, %ymm13, %ymm14, %ymm14
	vmaskmovpd	%ymm1, %ymm14, 32(%r11)
	vmaskmovpd	%ymm5, %ymm15, 32(%r11, %r12, 1)
	cmpl		$ 3, %r15d
	jl			3f // end
	vblendpd	$ 0x4, %ymm13, %ymm14, %ymm14
	vmaskmovpd	%ymm2, %ymm14, 64(%r11)
	vmaskmovpd	%ymm6, %ymm15, 64(%r11, %r12, 1)
	cmpl		$ 4, %r15d
	jl			3f // end
	vmaskmovpd	%ymm7, %ymm15, 96(%r11, %r12, 1)

	jmp 3f

22:
	vblendpd	$ 0x3, %ymm13, %ymm14, %ymm14
	vmaskmovpd	%ymm0, %ymm14,  0(%r11)
	vmaskmovpd	%ymm4, %ymm15,  0(%r11, %r12, 1)
	cmpl		$ 2, %r15d
	jl			3f // end
	vblendpd	$ 0x4, %ymm13, %ymm14, %ymm14
	vmaskmovpd	%ymm1, %ymm14, 32(%r11)
	vmaskmovpd	%ymm5, %ymm15, 32(%r11, %r12, 1)
	cmpl		$ 3, %r15d
	jl			3f // end
	vmaskmovpd	%ymm6, %ymm15, 64(%r11, %r12, 1)
	cmpl		$ 4, %r15d
	jl			3f // end
	vblendpd	$ 0x1, %ymm13, %ymm15, %ymm15
	vmaskmovpd	%ymm7, %ymm15, 96(%r11, %r12, 1)

	jmp 3f

23:
	vblendpd	$ 0x7, %ymm13, %ymm14, %ymm14
	vmaskmovpd	%ymm0, %ymm14,  0(%r11)
	vmaskmovpd	%ymm4, %ymm15,  0(%r11, %r12, 1)
	cmpl		$ 2, %r15d
	jl			3f // end
	vmaskmovpd	%ymm5, %ymm15, 32(%r11, %r12, 1)
	cmpl		$ 3, %r15d
	jl			3f // end
	vblendpd	$ 0x1, %ymm13, %ymm15, %ymm15
	vmaskmovpd	%ymm6, %ymm15, 64(%r11, %r12, 1)
	cmpl		$ 4, %r15d
	jl			3f // end
	vblendpd	$ 0x2, %ymm13, %ymm15, %ymm15
	vmaskmovpd	%ymm7, %ymm15, 96(%r11, %r12, 1)

	jmp 3f


	// offset==1
11:
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd		.LC12(%rip), %ymm14
#elif defined(OS_MAC)
	vmovapd		LC12(%rip), %ymm14
#endif

	vmovapd		%ymm0, %ymm13
	vperm2f128	$ 0x03, %ymm4, %ymm0, %ymm12
	vshufpd		$ 0x5, %ymm0, %ymm12, %ymm0
	vperm2f128	$ 0x03, %ymm13, %ymm4, %ymm12
	vshufpd		$ 0x5, %ymm4, %ymm12, %ymm4

	vmovapd		%ymm1, %ymm13
	vperm2f128	$ 0x03, %ymm5, %ymm1, %ymm12
	vshufpd		$ 0x5, %ymm1, %ymm12, %ymm1
	vperm2f128	$ 0x03, %ymm13, %ymm5, %ymm12
	vshufpd		$ 0x5, %ymm5, %ymm12, %ymm5

	vmovapd		%ymm2, %ymm13
	vperm2f128	$ 0x03, %ymm6, %ymm2, %ymm12
	vshufpd		$ 0x5, %ymm2, %ymm12, %ymm2
	vperm2f128	$ 0x03, %ymm13, %ymm6, %ymm12
	vshufpd		$ 0x5, %ymm6, %ymm12, %ymm6

	vmovapd		%ymm3, %ymm13
	vperm2f128	$ 0x03, %ymm7, %ymm3, %ymm12
	vshufpd		$ 0x5, %ymm3, %ymm12, %ymm3
	vperm2f128	$ 0x03, %ymm13, %ymm7, %ymm12
	vshufpd		$ 0x5, %ymm7, %ymm12, %ymm7

	vperm2f128	$ 0x01, %ymm15, %ymm15, %ymm12
	vshufpd		$ 0x5, %ymm15, %ymm12, %ymm15
//	vperm2f128	$ 0x01, %ymm14, %ymm14, %ymm12
//	vshufpd		$ 0x5, %ymm14, %ymm12, %ymm14

#if defined(OS_LINUX) | defined(OS_WINDOWS)
//	vmovupd		.LC08(%rip), %ymm12
	vmovupd		.LC05(%rip), %ymm13
#elif defined(OS_MAC)
//	vmovupd		LC08(%rip), %ymm12
	vmovupd		LC05(%rip), %ymm13
#endif
//	vandpd		%ymm12, %ymm14, %ymm12
	vandpd		%ymm13, %ymm15, %ymm13

	vblendpd	$ 0x1, %ymm14, %ymm15, %ymm14


#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd		.LC04(%rip), %ymm15
	vmovapd		.LC12(%rip), %ymm12
#elif defined(OS_MAC)
	vmovapd		LC04(%rip), %ymm15
	vmovapd		LC12(%rip), %ymm12
#endif

	// select m0
	cmpl	$ 0, %r13d
	jle		20f
	cmpl	$ 1, %r13d
	je		21f
	cmpl	$ 2, %r13d
	je		22f
	cmpl	$ 3, %r13d
	je		23f

	jmp 3f

20:
	vblendpd	$ 0x1, %ymm15, %ymm12, %ymm12
	vmaskmovpd	%ymm0, %ymm12, 0(%r11)
	vmaskmovpd	%ymm4, %ymm14, 0(%r11, %r12, 1)
	vmaskmovpd	%ymm0, %ymm13, 0(%r11, %r12, 2)
	cmpl		$ 2, %eax
	jl			3f // end
	vblendpd	$ 0x2, %ymm15, %ymm12, %ymm12
	vmaskmovpd	%ymm1, %ymm12, 32(%r11)
	vmaskmovpd	%ymm5, %ymm14, 32(%r11, %r12, 1)
	vmaskmovpd	%ymm1, %ymm13, 32(%r11, %r12, 2)
	cmpl		$ 3, %eax
	jl			3f // end
	vblendpd	$ 0x4, %ymm15, %ymm12, %ymm12
	vmaskmovpd	%ymm2, %ymm12, 64(%r11)
	vmaskmovpd	%ymm6, %ymm14, 64(%r11, %r12, 1)
	vmaskmovpd	%ymm2, %ymm13, 64(%r11, %r12, 2)
	cmpl		$ 4, %eax
	jl			3f // end
	vmaskmovpd	%ymm7, %ymm14, 96(%r11, %r12, 1)
	vmaskmovpd	%ymm3, %ymm13, 96(%r11, %r12, 2)

	jmp 3f

21:
	vblendpd	$ 0x3, %ymm15, %ymm12, %ymm12
	vmaskmovpd	%ymm0, %ymm12, 0(%r11)
	vmaskmovpd	%ymm4, %ymm14, 0(%r11, %r12, 1)
	vmaskmovpd	%ymm0, %ymm13, 0(%r11, %r12, 2)
	cmpl		$ 2, %eax
	jl			3f // end
	vblendpd	$ 0x4, %ymm15, %ymm12, %ymm12
	vmaskmovpd	%ymm1, %ymm12, 32(%r11)
	vmaskmovpd	%ymm5, %ymm14, 32(%r11, %r12, 1)
	vmaskmovpd	%ymm1, %ymm13, 32(%r11, %r12, 2)
	cmpl		$ 3, %eax
	jl			3f // end
	vmaskmovpd	%ymm6, %ymm14, 64(%r11, %r12, 1)
	vmaskmovpd	%ymm2, %ymm13, 64(%r11, %r12, 2)
	cmpl		$ 4, %eax
	jl			3f // end
	vblendpd	$ 0x1, %ymm15, %ymm14, %ymm14
	vmaskmovpd	%ymm7, %ymm14, 96(%r11, %r12, 1)
	vmaskmovpd	%ymm3, %ymm13, 96(%r11, %r12, 2)

	jmp 3f

22:
	vblendpd	$ 0x7, %ymm15, %ymm12, %ymm12
	vmaskmovpd	%ymm0, %ymm12, 0(%r11)
	vmaskmovpd	%ymm4, %ymm14, 0(%r11, %r12, 1)
	vmaskmovpd	%ymm0, %ymm13, 0(%r11, %r12, 2)
	cmpl		$ 2, %eax
	jl			3f // end
	vmaskmovpd	%ymm5, %ymm14, 32(%r11, %r12, 1)
	vmaskmovpd	%ymm1, %ymm13, 32(%r11, %r12, 2)
	cmpl		$ 3, %eax
	jl			3f // end
	vblendpd	$ 0x1, %ymm15, %ymm14, %ymm14
	vmaskmovpd	%ymm6, %ymm14, 64(%r11, %r12, 1)
	vmaskmovpd	%ymm2, %ymm13, 64(%r11, %r12, 2)
	cmpl		$ 4, %eax
	jl			3f // end
	vblendpd	$ 0x2, %ymm15, %ymm14, %ymm14
	vmaskmovpd	%ymm7, %ymm14, 96(%r11, %r12, 1)
	vmaskmovpd	%ymm3, %ymm13, 96(%r11, %r12, 2)

	jmp 3f

23:
	vmaskmovpd	%ymm4, %ymm14, 0(%r11, %r12, 1)
	vmaskmovpd	%ymm0, %ymm13, 0(%r11, %r12, 2)
	cmpl		$ 2, %eax
	jl			3f // end
	vblendpd	$ 0x1, %ymm15, %ymm14, %ymm14
	vmaskmovpd	%ymm5, %ymm14, 32(%r11, %r12, 1)
	vmaskmovpd	%ymm1, %ymm13, 32(%r11, %r12, 2)
	cmpl		$ 3, %eax
	jl			3f // end
	vblendpd	$ 0x2, %ymm15, %ymm14, %ymm14
	vmaskmovpd	%ymm6, %ymm14, 64(%r11, %r12, 1)
	vmaskmovpd	%ymm2, %ymm13, 64(%r11, %r12, 2)
	cmpl		$ 4, %eax
	jl			3f // end
	vblendpd	$ 0x4, %ymm15, %ymm14, %ymm14
	vmaskmovpd	%ymm7, %ymm14, 96(%r11, %r12, 1)
	vmaskmovpd	%ymm3, %ymm13, 96(%r11, %r12, 2)

	jmp 3f


	// offset==2
12:
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd		.LC12(%rip), %ymm14
#elif defined(OS_MAC)
	vmovapd		LC12(%rip), %ymm14
#endif

	vmovapd		%ymm0, %ymm13
	vperm2f128	$ 0x03, %ymm4, %ymm0, %ymm0
	vperm2f128	$ 0x03, %ymm13, %ymm4, %ymm4

	vmovapd		%ymm1, %ymm13
	vperm2f128	$ 0x03, %ymm5, %ymm1, %ymm1
	vperm2f128	$ 0x03, %ymm13, %ymm5, %ymm5

	vmovapd		%ymm2, %ymm13
	vperm2f128	$ 0x03, %ymm6, %ymm2, %ymm2
	vperm2f128	$ 0x03, %ymm13, %ymm6, %ymm6

	vmovapd		%ymm3, %ymm13
	vperm2f128	$ 0x03, %ymm7, %ymm3, %ymm3
	vperm2f128	$ 0x03, %ymm13, %ymm7, %ymm7

//	vperm2f128	$ 0x01, %ymm14, %ymm14, %ymm14
	vperm2f128	$ 0x01, %ymm15, %ymm15, %ymm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
//	vmovupd		.LC09(%rip), %ymm12
	vmovupd		.LC06(%rip), %ymm13
#elif defined(OS_MAC)
//	vmovupd		LC09(%rip), %ymm12
	vmovupd		LC06(%rip), %ymm13
#endif
//	vandpd		%ymm12, %ymm14, %ymm12
	vandpd		%ymm13, %ymm15, %ymm13

	vblendpd	$ 0x3, %ymm14, %ymm15, %ymm14

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd		.LC04(%rip), %ymm15
	vmovapd		.LC12(%rip), %ymm12
#elif defined(OS_MAC)
	vmovapd		LC04(%rip), %ymm15
	vmovapd		LC12(%rip), %ymm12
#endif

	// select m0
	cmpl	$ 0, %r13d
	jle		20f
	cmpl	$ 1, %r13d
	je		21f
	cmpl	$ 2, %r13d
	je		22f
	cmpl	$ 3, %r13d
	je		23f

	jmp 3f

20:
	vblendpd	$ 0x3, %ymm15, %ymm12, %ymm12
	vmaskmovpd	%ymm0, %ymm12, 0(%r11)
	vmaskmovpd	%ymm4, %ymm14, 0(%r11, %r12, 1)
	vmaskmovpd	%ymm0, %ymm13, 0(%r11, %r12, 2)
	cmpl		$ 2, %r15d
	jl			3f // end
	vblendpd	$ 0x4, %ymm15, %ymm12, %ymm12
	vmaskmovpd	%ymm1, %ymm12, 32(%r11)
	vmaskmovpd	%ymm5, %ymm14, 32(%r11, %r12, 1)
	vmaskmovpd	%ymm1, %ymm13, 32(%r11, %r12, 2)
	cmpl		$ 3, %r15d
	jl			3f // end
	vmaskmovpd	%ymm6, %ymm14, 64(%r11, %r12, 1)
	vmaskmovpd	%ymm2, %ymm13, 64(%r11, %r12, 2)
	cmpl		$ 4, %r15d
	jl			3f // end
	vblendpd	$ 0x1, %ymm15, %ymm14, %ymm14
	vmaskmovpd	%ymm7, %ymm14, 96(%r11, %r12, 1)
	vmaskmovpd	%ymm3, %ymm13, 96(%r11, %r12, 2)

	jmp 3f

21:
	vblendpd	$ 0x7, %ymm15, %ymm12, %ymm12
	vmaskmovpd	%ymm0, %ymm12, 0(%r11)
	vmaskmovpd	%ymm4, %ymm14, 0(%r11, %r12, 1)
	vmaskmovpd	%ymm0, %ymm13, 0(%r11, %r12, 2)
	cmpl		$ 2, %r15d
	jl			3f // end
	vmaskmovpd	%ymm5, %ymm14, 32(%r11, %r12, 1)
	vmaskmovpd	%ymm1, %ymm13, 32(%r11, %r12, 2)
	cmpl		$ 3, %r15d
	jl			3f // end
	vblendpd	$ 0x1, %ymm15, %ymm14, %ymm14
	vmaskmovpd	%ymm6, %ymm14, 64(%r11, %r12, 1)
	vmaskmovpd	%ymm2, %ymm13, 64(%r11, %r12, 2)
	cmpl		$ 4, %r15d
	jl			3f // end
	vblendpd	$ 0x2, %ymm15, %ymm14, %ymm14
	vmaskmovpd	%ymm7, %ymm14, 96(%r11, %r12, 1)
	vmaskmovpd	%ymm3, %ymm13, 96(%r11, %r12, 2)

	jmp 3f

22:
	vmaskmovpd	%ymm4, %ymm14, 0(%r11, %r12, 1)
	vmaskmovpd	%ymm0, %ymm13, 0(%r11, %r12, 2)
	cmpl		$ 2, %r15d
	jl			3f // end
	vblendpd	$ 0x1, %ymm15, %ymm14, %ymm14
	vmaskmovpd	%ymm5, %ymm14, 32(%r11, %r12, 1)
	vmaskmovpd	%ymm1, %ymm13, 32(%r11, %r12, 2)
	cmpl		$ 3, %r15d
	jl			3f // end
	vblendpd	$ 0x2, %ymm15, %ymm14, %ymm14
	vmaskmovpd	%ymm6, %ymm14, 64(%r11, %r12, 1)
	vmaskmovpd	%ymm2, %ymm13, 64(%r11, %r12, 2)
	cmpl		$ 4, %r15d
	jl			3f // end
	vblendpd	$ 0x4, %ymm15, %ymm14, %ymm14
	vmaskmovpd	%ymm7, %ymm14, 96(%r11, %r12, 1)
	vmaskmovpd	%ymm3, %ymm13, 96(%r11, %r12, 2)

	jmp 3f

23:
	vblendpd	$ 0x1, %ymm15, %ymm14, %ymm14
	vmaskmovpd	%ymm4, %ymm14, 0(%r11, %r12, 1)
	vmaskmovpd	%ymm0, %ymm13, 0(%r11, %r12, 2)
	cmpl		$ 2, %r15d
	jl			3f // end
	vblendpd	$ 0x2, %ymm15, %ymm14, %ymm14
	vmaskmovpd	%ymm5, %ymm14, 32(%r11, %r12, 1)
	vmaskmovpd	%ymm1, %ymm13, 32(%r11, %r12, 2)
	cmpl		$ 3, %r15d
	jl			3f // end
	vblendpd	$ 0x4, %ymm15, %ymm14, %ymm14
	vmaskmovpd	%ymm6, %ymm14, 64(%r11, %r12, 1)
	vmaskmovpd	%ymm2, %ymm13, 64(%r11, %r12, 2)
	cmpl		$ 4, %r15d
	jl			3f // end
	vmaskmovpd	%ymm3, %ymm13, 96(%r11, %r12, 2)

	jmp 3f


	// offset==3
13:
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd		.LC12(%rip), %ymm14
#elif defined(OS_MAC)
	vmovapd		LC12(%rip), %ymm14
#endif

	vmovapd		%ymm0, %ymm13
	vperm2f128	$ 0x21, %ymm0, %ymm4, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm4, %ymm0
	vperm2f128	$ 0x21, %ymm4, %ymm13, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm13, %ymm4

	vmovapd		%ymm1, %ymm13
	vperm2f128	$ 0x21, %ymm1, %ymm5, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm5, %ymm1
	vperm2f128	$ 0x21, %ymm5, %ymm13, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm13, %ymm5

	vmovapd		%ymm2, %ymm13
	vperm2f128	$ 0x21, %ymm2, %ymm6, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm6, %ymm2
	vperm2f128	$ 0x21, %ymm6, %ymm13, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm13, %ymm6

	vmovapd		%ymm3, %ymm13
	vperm2f128	$ 0x21, %ymm3, %ymm7, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm7, %ymm3
	vperm2f128	$ 0x21, %ymm7, %ymm13, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm13, %ymm7

//	vperm2f128	$ 0x01, %ymm14, %ymm14, %ymm12
//	vshufpd		$ 0x5, %ymm12, %ymm14, %ymm14
	vperm2f128	$ 0x01, %ymm15, %ymm15, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm15, %ymm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
//	vmovupd		.LC10(%rip), %ymm12
	vmovupd		.LC07(%rip), %ymm13
#elif defined(OS_MAC)
//	vmovupd		LC10(%rip), %ymm12
	vmovupd		LC07(%rip), %ymm13
#endif
//	vandpd		%ymm12, %ymm14, %ymm12
	vandpd		%ymm13, %ymm15, %ymm13

	vblendpd	$ 0x7, %ymm14, %ymm15, %ymm14

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd		.LC04(%rip), %ymm15
	vmovapd		.LC12(%rip), %ymm12
#elif defined(OS_MAC)
	vmovapd		LC04(%rip), %ymm15
	vmovapd		LC12(%rip), %ymm12
#endif

	// select m0
	cmpl	$ 0, %r13d
	jle		20f
	cmpl	$ 1, %r13d
	je		21f
	cmpl	$ 2, %r13d
	je		22f
	cmpl	$ 3, %r13d
	je		23f

	jmp 3f

20:
	vblendpd	$ 0x7, %ymm15, %ymm12, %ymm12
	vmaskmovpd	%ymm0, %ymm12, 0(%r11)
	vmaskmovpd	%ymm4, %ymm14, 0(%r11, %r12, 1)
	vmaskmovpd	%ymm0, %ymm13, 0(%r11, %r12, 2)
	cmpl		$ 2, %r15d
	jl			3f // end
	vmaskmovpd	%ymm5, %ymm14, 32(%r11, %r12, 1)
	vmaskmovpd	%ymm1, %ymm13, 32(%r11, %r12, 2)
	cmpl		$ 3, %r15d
	jl			3f // end
	vblendpd	$ 0x1, %ymm15, %ymm14, %ymm14
	vmaskmovpd	%ymm6, %ymm14, 64(%r11, %r12, 1)
	vmaskmovpd	%ymm2, %ymm13, 64(%r11, %r12, 2)
	cmpl		$ 4, %r15d
	jl			3f // end
	vblendpd	$ 0x2, %ymm15, %ymm14, %ymm14
	vmaskmovpd	%ymm7, %ymm14, 96(%r11, %r12, 1)
	vmaskmovpd	%ymm3, %ymm13, 96(%r11, %r12, 2)

	jmp 3f

21:
	vmaskmovpd	%ymm4, %ymm14, 0(%r11, %r12, 1)
	vmaskmovpd	%ymm0, %ymm13, 0(%r11, %r12, 2)
	cmpl		$ 2, %r15d
	jl			3f // end
	vblendpd	$ 0x1, %ymm15, %ymm14, %ymm14
	vmaskmovpd	%ymm5, %ymm14, 32(%r11, %r12, 1)
	vmaskmovpd	%ymm1, %ymm13, 32(%r11, %r12, 2)
	cmpl		$ 3, %r15d
	jl			3f // end
	vblendpd	$ 0x2, %ymm15, %ymm14, %ymm14
	vmaskmovpd	%ymm6, %ymm14, 64(%r11, %r12, 1)
	vmaskmovpd	%ymm2, %ymm13, 64(%r11, %r12, 2)
	cmpl		$ 4, %r15d
	jl			3f // end
	vblendpd	$ 0x4, %ymm15, %ymm14, %ymm14
	vmaskmovpd	%ymm7, %ymm14, 96(%r11, %r12, 1)
	vmaskmovpd	%ymm3, %ymm13, 96(%r11, %r12, 2)

	jmp 3f

22:
	vblendpd	$ 0x1, %ymm15, %ymm14, %ymm14
	vmaskmovpd	%ymm4, %ymm14, 0(%r11, %r12, 1)
	vmaskmovpd	%ymm0, %ymm13, 0(%r11, %r12, 2)
	cmpl		$ 2, %r15d
	jl			3f // end
	vblendpd	$ 0x2, %ymm15, %ymm14, %ymm14
	vmaskmovpd	%ymm5, %ymm14, 32(%r11, %r12, 1)
	vmaskmovpd	%ymm1, %ymm13, 32(%r11, %r12, 2)
	cmpl		$ 3, %r15d
	jl			3f // end
	vblendpd	$ 0x4, %ymm15, %ymm14, %ymm14
	vmaskmovpd	%ymm6, %ymm14, 64(%r11, %r12, 1)
	vmaskmovpd	%ymm2, %ymm13, 64(%r11, %r12, 2)
	cmpl		$ 4, %r15d
	jl			3f // end
	vmaskmovpd	%ymm3, %ymm13, 96(%r11, %r12, 2)

	jmp 3f

23:
	vblendpd	$ 0x3, %ymm15, %ymm14, %ymm14
	vmaskmovpd	%ymm4, %ymm14, 0(%r11, %r12, 1)
	vmaskmovpd	%ymm0, %ymm13, 0(%r11, %r12, 2)
	cmpl		$ 2, %r15d
	jl			3f // end
	vblendpd	$ 0x4, %ymm15, %ymm14, %ymm14
	vmaskmovpd	%ymm5, %ymm14, 32(%r11, %r12, 1)
	vmaskmovpd	%ymm1, %ymm13, 32(%r11, %r12, 2)
	cmpl		$ 3, %r15d
	jl			3f // end
	vmaskmovpd	%ymm2, %ymm13, 64(%r11, %r12, 2)
	cmpl		$ 4, %r15d
	jl			3f // end
	vblendpd	$ 0x1, %ymm15, %ymm13, %ymm13
	vmaskmovpd	%ymm3, %ymm13, 96(%r11, %r12, 2)

	jmp 3f

3:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_8x4_gen_lib4)
#endif





// common inner routine with file scope
//
// transpose
//
// input arguments:
// ymm0 <- [d00 d11 d22 d33]
// ymm1 <- [d01 d10 d23 d32]
// ymm2 <- [d03 d12 d21 d30]
// ymm3 <- [d02 d13 d20 d31]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_TRAN_8X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_tran_8x4_lib4)
#endif

	vunpcklpd	%ymm1, %ymm0, %ymm12
	vunpckhpd	%ymm1, %ymm0, %ymm13
	vunpcklpd	%ymm3, %ymm2, %ymm14
	vunpckhpd	%ymm3, %ymm2, %ymm15

	vperm2f128	$ 0x20, %ymm14, %ymm12, %ymm0
	vperm2f128	$ 0x31, %ymm14, %ymm12, %ymm2
	vperm2f128	$ 0x20, %ymm15, %ymm13, %ymm1
	vperm2f128	$ 0x31, %ymm15, %ymm13, %ymm3

	vunpcklpd	%ymm5, %ymm4, %ymm12
	vunpckhpd	%ymm5, %ymm4, %ymm13
	vunpcklpd	%ymm7, %ymm6, %ymm14
	vunpckhpd	%ymm7, %ymm6, %ymm15

	vperm2f128	$ 0x20, %ymm14, %ymm12, %ymm4
	vperm2f128	$ 0x31, %ymm14, %ymm12, %ymm6
	vperm2f128	$ 0x20, %ymm15, %ymm13, %ymm5
	vperm2f128	$ 0x31, %ymm15, %ymm13, %ymm7

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_tran_8x4_lib4)
#endif





//                               1      2              3          4        5          6             7          8        9          10
// void kernel_dgemm_nt_8x4_lib4(int k, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_8x4_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x4_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12 // C
	movq	ARG8, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_AB_8X4_LIB4
#else
	CALL(inner_blend_scale_ab_8x4_lib4)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB4
#else
	CALL(inner_store_8x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_8x4_lib4)





//                               1      2              3          4          5        6             7          8
// void kernel_dgemm_nt_4x8_lib4(int k, double *alpha, double *A, double *B, int sdb, double *beta, double *C, double *D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_4x8_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG4, %r11 // B
	movq	ARG5, %r12 // sdb
	sall	$ 5, %r12d // 4*sdb*sizeof(double)
	movq	ARG3, %r13 // A

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x4_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12 // C

#if MACRO_LEVEL>=1
	INNER_TRAN_SCALE_AB_4X8_LIB4
#else
	CALL(inner_tran_scale_ab_4x8_lib4)
#endif


	// store n

	movq	ARG8, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_LIB4
#else
	CALL(inner_store_4x8_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_4x8_lib4)





//                                  rdi     rsi            rdx        rcx      r8         r9            rsp+8      rsp+16   rsp+24     rsp+32   rsp+40  rsp+48
// void kernel_dgemm_nt_8x4_vs_lib4(int km, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_8x4_vs_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x4_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12 // C
	movq	ARG8, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_AB_8X4_LIB4
#else
	CALL(inner_blend_scale_ab_8x4_lib4)
#endif


	// store n

	movq	ARG9, %r10 // store address D
	movq	ARG10, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG11, %r12 // km
	movq	ARG12, %r13 // kn

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_VS_LIB4
#else
	CALL(inner_store_8x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_8x4_vs_lib4)





//                                  1      2              3          4          5        6             7          8          9       10
// void kernel_dgemm_nt_4x8_vs_lib4(int k, double *alpha, double *A, double *B, int sdb, double *beta, double *C, double *D, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_4x8_vs_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG4, %r11 // B
	movq	ARG5, %r12 // sdb
	sall	$ 5, %r12d // 4*sdb*sizeof(double)
	movq	ARG3, %r13 // A

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x4_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12 // C

#if MACRO_LEVEL>=1
	INNER_TRAN_SCALE_AB_4X8_LIB4
#else
	CALL(inner_tran_scale_ab_4x8_lib4)
#endif


	// store n

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // km
	movq	ARG10, %r12 // kn

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_VS_LIB4
#else
	CALL(inner_store_4x8_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_4x8_vs_lib4)





//                                   rdi    rsi            rdx        rcx      r8         r9            rsp+8        rsp+16     rsp+24   rsp+32       rsp+40     rsp+48   rsp+56  rsp+64  rsp+72  rsp+80
// void kernel_dgemm_nt_8x4_gen_lib4(int k, double *alpha, double *A, int sda, double *B, double *beta, int offsetC, double *C, int sdc, int offsetD, double *D, int sdd, int m0, int m1, int n0, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_8x4_gen_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x4_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12 // offsetC
	movq	ARG8, %r13 // C
	movq	ARG9, %r14 // sdc
	sall	$ 5, %r14d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_AB_8X4_GEN_LIB4
#else
	CALL(inner_blend_scale_ab_8x4_gen_lib4)
#endif


	// store n

	movq	ARG10, %r10 // offsetD
	movq	ARG11, %r11 // D
	movq	ARG12, %r12 // sdd
	sall	$ 5, %r12d // 4*sdb*sizeof(double)
	movq	ARG13, %r13 // m0
	movq	ARG14, %r14 // m1
	movq	ARG15, %r15 // n0
	movq	ARG16, %rax // n1

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_GEN_LIB4
#else
	CALL(inner_store_8x4_gen_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_8x4_gen_lib4)





//                               1      2              3          4        5            6          7        8             9          10       11         12
// void kernel_dgemm_nn_8x4_lib4(int k, double *alpha, double *A, int sda, int offsetB, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nn_8x4_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG6, %r13 // B
	movq	ARG7, %r14 // sdb
	sall	$ 5, %r14d // 4*sdb*sizeof(double)
	movq	ARG5, %r15 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_DGEMM_NN_8X4_LIB4
#else
	CALL(inner_edge_dgemm_nn_8x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_8x4_lib4)
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12 // C
	movq	ARG10, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB4
#else
	CALL(inner_scale_ab_8x4_lib4)
#endif


	// store n

	movq	ARG11, %r10 // D
	movq	ARG12, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB4
#else
	CALL(inner_store_8x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nn_8x4_lib4)





//                               1      2              3          4            5          6        7             8          9
// void kernel_dgemm_nn_4x8_lib4(int k, double *alpha, double *A, int offsetB, double *B, int sdb, double *beta, double *C, double *D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nn_4x8_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG5, %r12  // B
	movq	ARG6, %r13 // sdb
	sall	$ 5, %r13d // 4*sdb*sizeof(double)
	movq	ARG4, %r14 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_DGEMM_NN_4X8_LIB4
#else
	CALL(inner_edge_dgemm_nn_4x8_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X8_LIB4
#else
	CALL(inner_kernel_dgemm_nn_4x8_lib4)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X8_LIB4
#else
	CALL(inner_scale_ab_4x8_lib4)
#endif


	// store n

	movq	ARG9, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_LIB4
#else
	CALL(inner_store_4x8_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nn_4x8_lib4)





//                                  1      2              3          4        5            6          7        8             9          10       11         12       13      14
// void kernel_dgemm_nn_8x4_vs_lib4(int k, double *alpha, double *A, int sda, int offsetB, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nn_8x4_vs_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG6, %r13 // B
	movq	ARG7, %r14 // sdb
	sall	$ 5, %r14d // 4*sdb*sizeof(double)
	movq	ARG5, %r15 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_DGEMM_NN_8X4_LIB4
#else
	CALL(inner_edge_dgemm_nn_8x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_8x4_lib4)
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12 // C
	movq	ARG10, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB4
#else
	CALL(inner_scale_ab_8x4_lib4)
#endif


	// store n

	movq	ARG11, %r10 // D
	movq	ARG12, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG13, %r12 // km
	movq	ARG14, %r13 // kn

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_VS_LIB4
#else
	CALL(inner_store_8x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nn_8x4_vs_lib4)





//                                  1      2              3          4            5          6        7             8          9          10     11
// void kernel_dgemm_nn_4x8_vs_lib4(int k, double *alpha, double *A, int offsetB, double *B, int sdb, double *beta, double *C, double *D, int m1 int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nn_4x8_vs_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG5, %r12  // B
	movq	ARG6, %r13 // sdb
	sall	$ 5, %r13d // 4*sdb*sizeof(double)
	movq	ARG4, %r14 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_DGEMM_NN_4X8_LIB4
#else
	CALL(inner_edge_dgemm_nn_4x8_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X8_LIB4
#else
	CALL(inner_kernel_dgemm_nn_4x8_lib4)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X8_LIB4
#else
	CALL(inner_scale_ab_4x8_lib4)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // m1
	movq	ARG11, %r12 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_VS_LIB4
#else
	CALL(inner_store_4x8_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nn_4x8_vs_lib4)





//                                   1      2              3          4        5         6          7        8             9         10         11       12        13         14       15      16      17      18
// void kernel_dgemm_nn_8x4_gen_lib4(int k, double *alpha, double *A, int sda, int offB, double *B, int sdb, double *beta, int offC, double *C, int sdc, int offD, double *D, int sdd, int m0, int m1, int n0, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nn_8x4_gen_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG6, %r13  // B
	movq	ARG7, %r14 // sdb
	sall	$ 5, %r14d // 4*sdb*sizeof(double)
	movq	ARG5, %r15 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_DGEMM_NN_8X4_LIB4
#else
	CALL(inner_edge_dgemm_nn_8x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_8x4_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12 // offsetC
	movq	ARG10, %r13 // C
	movq	ARG11, %r14 // sdc
	sall	$ 5, %r14d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_GEN_LIB4
#else
	CALL(inner_scale_ab_8x4_gen_lib4)
#endif


	// store n gen

	movq	ARG12, %r10 // offsetD
	movq	ARG13, %r11 // D
	movq	ARG14, %r12 // sdd
	sall	$ 5, %r12d // 4*sdb*sizeof(double)
	movq	ARG15, %r13 // m0
	movq	ARG16, %r14 // m1
	movq	ARG17, %r15 // n0
	movq	ARG18, %rax // n1

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_GEN_LIB4
#else
	CALL(inner_store_8x4_gen_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nn_8x4_gen_lib4)





//                               1      2              3          4        5            6          7        8             9          10
// void kernel_dgemm_tt_4x8_lib4(int k, double *alpha, int offsetA, double *A, int sda, double *B, int sdb, double *beta, double *C, double *D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_tt_4x8_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG6, %r11 // B
	movq	ARG7, %r12 // sdb
	sall	$ 5, %r12d // 4*sdb*sizeof(double)
	movq	ARG4, %r13 // A
	movq	ARG5, %r14 // sda
	sall	$ 5, %r14d // 4*sda*sizeof(double)
	movq	ARG3, %r15 // offsetA

#if MACRO_LEVEL>=1
	INNER_EDGE_DGEMM_NN_8X4_LIB4
#else
	CALL(inner_edge_dgemm_nn_8x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_8x4_lib4)
#endif

#if MACRO_LEVEL>=1
	INNER_TRAN_8X4_LIB4
#else
	CALL(inner_tran_8x4_lib4)
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X8_LIB4
#else
	CALL(inner_scale_ab_4x8_lib4)
#endif


	// store n

	movq	ARG10, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_LIB4
#else
	CALL(inner_store_4x8_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dgemm_tt_4x8_lib4)





//                               1      2              3          4            5          6        7             8          9        10         11
// void kernel_dgemm_tt_8x4_lib4(int k, double *alpha, int offsetA, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_tt_8x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG6, %r11  // B
	movq	ARG4, %r12  // A
	movq	ARG5, %r13 // sda
	sall	$ 5, %r13d // 4*sda*sizeof(double)
	movq	ARG3, %r14 // offsetA

#if MACRO_LEVEL>=1
	INNER_EDGE_DGEMM_NN_4X8_LIB4
#else
	CALL(inner_edge_dgemm_nn_4x8_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X8_LIB4
#else
	CALL(inner_kernel_dgemm_nn_4x8_lib4)
#endif

#if MACRO_LEVEL>=1
	INNER_TRAN_8X4_LIB4
#else
	CALL(inner_tran_8x4_lib4)
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB4
#else
	CALL(inner_scale_ab_8x4_lib4)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB4
#else
	CALL(inner_store_8x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_tt_8x4_lib4)





//                                  1      2              3          4        5            6          7        8             9          10         11      12
// void kernel_dgemm_tt_4x8_vs_lib4(int k, double *alpha, int offsetA, double *A, int sda, double *B, int sdb, double *beta, double *C, double *D, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_tt_4x8_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG6, %r11 // B
	movq	ARG7, %r12 // sdb
	sall	$ 5, %r12d // 4*sdb*sizeof(double)
	movq	ARG4, %r13 // A
	movq	ARG5, %r14 // sda
	sall	$ 5, %r14d // 4*sda*sizeof(double)
	movq	ARG3, %r15 // offsetA

#if MACRO_LEVEL>=1
	INNER_EDGE_DGEMM_NN_8X4_LIB4
#else
	CALL(inner_edge_dgemm_nn_8x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_8x4_lib4)
#endif

#if MACRO_LEVEL>=1
	INNER_TRAN_8X4_LIB4
#else
	CALL(inner_tran_8x4_lib4)
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X8_LIB4
#else
	CALL(inner_scale_ab_4x8_lib4)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // km 
	movq	ARG12, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_VS_LIB4
#else
	CALL(inner_store_4x8_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dgemm_tt_4x8_vs_lib4)





//                                  1      2              3          4            5          6        7             8          9          10      11        12     13
// void kernel_dgemm_tt_8x4_vs_lib4(int k, double *alpha, int offsetA, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_tt_8x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG6, %r11  // B
	movq	ARG4, %r12  // A
	movq	ARG5, %r13 // sda
	sall	$ 5, %r13d // 4*sda*sizeof(double)
	movq	ARG3, %r14 // offsetA

#if MACRO_LEVEL>=1
	INNER_EDGE_DGEMM_NN_4X8_LIB4
#else
	CALL(inner_edge_dgemm_nn_4x8_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X8_LIB4
#else
	CALL(inner_kernel_dgemm_nn_4x8_lib4)
#endif

#if MACRO_LEVEL>=1
	INNER_TRAN_8X4_LIB4
#else
	CALL(inner_tran_8x4_lib4)
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB4
#else
	CALL(inner_scale_ab_8x4_lib4)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_VS_LIB4
#else
	CALL(inner_store_8x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_tt_8x4_vs_lib4)





//                                   1      2              3          4        5         6          7        8             9         10         11       12        13         14       15      16      17      18
// void kernel_dgemm_tt_4x8_gen_lib4(int k, double *alpha, it offsetA, double *A, int sda, double *B, int sdb, double *beta, int offC, double *C, int sdc, int offD, double *D, int sdd, int m0, int m1, int n0, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_tt_4x8_gen_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG6, %r11  // B
	movq	ARG7, %r12 // sdb
	sall	$ 5, %r12d // 4*sdb*sizeof(double)
	movq	ARG4, %r13  // A
	movq	ARG5, %r14 // sda
	sall	$ 5, %r14d // 4*sda*sizeof(double)
	movq	ARG3, %r15 // offsetA

#if MACRO_LEVEL>=1
	INNER_EDGE_DGEMM_NN_8X4_LIB4
#else
	CALL(inner_edge_dgemm_nn_8x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_8x4_lib4)
#endif

#if MACRO_LEVEL>=1
	INNER_TRAN_8X4_LIB4
#else
	CALL(inner_tran_8x4_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12 // offsetC
	movq	ARG10, %r13 // C
	movq	ARG11, %r14 // sdc
	sall	$ 5, %r14d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X8_GEN_LIB4
#else
	CALL(inner_scale_ab_4x8_gen_lib4)
#endif


	// store n gen

	movq	ARG12, %r10 // offsetD
	movq	ARG13, %r11 // D
	movq	ARG14, %r12 // sdd
	sall	$ 5, %r12d // 4*sdb*sizeof(double)
	movq	ARG15, %r13 // m0
	movq	ARG16, %r14 // m1
	movq	ARG17, %r15 // n0
	movq	ARG18, %rax // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_GEN_LIB4
#else
	CALL(inner_store_4x8_gen_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dgemm_tt_4x8_gen_lib4)





//                                 1      2              3          4        5            6          7        8             9          10       11         12
// void kernel_dsyrk_nn_u_8x4_lib4(int k, double *alpha, double *A, int sda, int offsetB, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nn_u_8x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG6, %r13 // B
	movq	ARG7, %r14 // sdb
	sall	$ 5, %r14d // 4*sdb*sizeof(double)
	movq	ARG5, %r15 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_DGEMM_NN_8X4_LIB4
#else
	CALL(inner_edge_dgemm_nn_8x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_8x4_lib4)
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12 // C
	movq	ARG10, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB4
#else
	CALL(inner_scale_ab_8x4_lib4)
#endif


	// store n

	movq	ARG11, %r10 // D
	movq	ARG12, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_U_8X4_LIB4
#else
	CALL(inner_store_u_8x4_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dsyrk_nn_u_8x4_lib4)





//                                    1      2              3          4        5            6          7        8             9          10       11         12       13      14
// void kernel_dsyrk_nn_u_8x4_vs_lib4(int k, double *alpha, double *A, int sda, int offsetB, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nn_u_8x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG6, %r13 // B
	movq	ARG7, %r14 // sdb
	sall	$ 5, %r14d // 4*sdb*sizeof(double)
	movq	ARG5, %r15 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_DGEMM_NN_8X4_LIB4
#else
	CALL(inner_edge_dgemm_nn_8x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_8x4_lib4)
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12 // C
	movq	ARG10, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB4
#else
	CALL(inner_scale_ab_8x4_lib4)
#endif


	// store n

	movq	ARG11, %r10 // D
	movq	ARG12, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG13, %r12 // km 
	movq	ARG14, %r13 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_U_8X4_VS_LIB4
#else
	CALL(inner_store_u_8x4_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dsyrk_nn_u_8x4_vs_lib4)





//                                 rdi     rsi            rdx        rcx      r8         r9            rsp+8      rsp+16   rsp+24     rsp+32
// void kernel_dsyrk_nt_l_8x4_lib4(int km, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nt_l_8x4_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x4_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12 // C
	movq	ARG8, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_AB_8X4_LIB4
#else
	CALL(inner_blend_scale_ab_8x4_lib4)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X4_LIB4
#else
	CALL(inner_store_l_8x4_lib4)
#endif

	EPILOGUE

	ret

	FUN_END(kernel_dsyrk_nt_l_8x4_lib4)





//                                    rdi     rsi            rdx        rcx      r8         r9            rsp+8      rsp+16   rsp+24     rsp+32   rsp+40  rsp+48
// void kernel_dsyrk_nt_l_8x4_vs_lib4(int km, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nt_l_8x4_vs_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x4_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12 // C
	movq	ARG8, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_AB_8X4_LIB4
#else
	CALL(inner_blend_scale_ab_8x4_lib4)
#endif


	// store n

	movq	ARG9, %r10 // store address D
	movq	ARG10, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

	movq	ARG11, %r12 // km
	movq	ARG12, %r13 // kn

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X4_VS_LIB4
#else
	CALL(inner_store_l_8x4_vs_lib4)
#endif

	EPILOGUE

	ret

	FUN_END(kernel_dsyrk_nt_l_8x4_vs_lib4)





//                                     rdi    rsi            rdx        rcx      r8         r9            rsp+8        rsp+16     rsp+24   rsp+32       rsp+40     rsp+48   rsp+56  rsp+64  rsp+72  rsp+80
// void kernel_dsyrk_nt_l_8x4_gen_lib4(int k, double *alpha, double *A, int sda, double *B, double *beta, int offsetC, double *C, int sdc, int offsetD, double *D, int sdd, int m0, int m1, int n0, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nt_l_8x4_gen_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x4_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12 // offsetC
	movq	ARG8, %r13 // C
	movq	ARG9, %r14 // sdc
	sall	$ 5, %r14d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_AB_8X4_GEN_LIB4
#else
	CALL(inner_blend_scale_ab_8x4_gen_lib4)
#endif


	// store n

	movq	ARG10, %r10 // offsetD
	movq	ARG11, %r11 // D
	movq	ARG12, %r12 // sdd
	sall	$ 5, %r12d // 4*sdb*sizeof(double)
	movq	ARG13, %r13 // m0
	movq	ARG14, %r14 // m1
	movq	ARG15, %r15 // n0
	movq	ARG16, %rax // n1

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X4_GEN_LIB4
#else
	CALL(inner_store_l_8x4_gen_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dsyrk_nt_l_8x4_gen_lib4)





//                                  rdi    rsi            rdx        rcx      r8           r9         rsp+8    rsp+16     rsp+24
// void kernel_dtrmm_nn_rl_8x4_lib4(int k, double *alpha, double *A, int sda, int offsetB, double *B, int sdb, double *D, int sdd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nn_rl_8x4_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// initial triangle

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sdb
	sall	$ 5, %r12d // 4*sdb*sizeof(double)
	movq	ARG6, %r13 // B
	movq	ARG7, %r14 // sdb
	sall	$ 5, %r14d // 4*sdb*sizeof(double)
	movq	ARG5, %r15 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NN_RL_8X4_LIB4
#else
	CALL(inner_edge_dtrmm_nn_rl_8x4_lib4)
#endif

	// call inner dgemm kernel nt after initial triangle

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_8x4_lib4)
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha

#if MACRO_LEVEL>=1
	INNER_SCALE_A0_8X4_LIB4
#else
	CALL(inner_scale_a0_8x4_lib4)
#endif


	// store n

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB4
#else
	CALL(inner_store_8x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nn_rl_8x4_lib4)





//                                     1      2              3          4        5            6          7        8          9        10      11
// void kernel_dtrmm_nn_rl_8x4_vs_lib4(int k, double *alpha, double *A, int sda, int offsetB, double *B, int sdb, double *D, int sdd, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nn_rl_8x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// initial triangle

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sdb
	sall	$ 5, %r12d // 4*sdb*sizeof(double)
	movq	ARG6, %r13 // B
	movq	ARG7, %r14 // sdb
	sall	$ 5, %r14d // 4*sdb*sizeof(double)
	movq	ARG5, %r15 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NN_RL_8X4_VS_LIB4
#else
	CALL(inner_edge_dtrmm_nn_rl_8x4_vs_lib4)
#endif

	// call inner dgemm kernel nt after initial triangle

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_8x4_lib4)
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha

#if MACRO_LEVEL>=1
	INNER_SCALE_A0_8X4_LIB4
#else
	CALL(inner_scale_a0_8x4_lib4)
#endif


	// store n

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdb*sizeof(double)
	movq	ARG10, %r12 // km
	movq	ARG11, %r13 // kn

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_VS_LIB4
#else
	CALL(inner_store_8x4_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dtrmm_nn_rl_8x4_vs_lib4)





//                                      rdi    rsi            rdx        rcx      r8           r9         rsp+8    rsp+16       rsp+24     rsp+32   rsp+40  rsp+48  rsp+56  rsp+64
// void kernel_dtrmm_nn_rl_8x4_gen_lib4(int k, double *alpha, double *A, int sda, int offsetB, double *B, int sdb, int offsetD, double *D, int sdd, int m0, int m1, int n0, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nn_rl_8x4_gen_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// initial triangle

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sdb
	sall	$ 5, %r12d // 4*sdb*sizeof(double)
	movq	ARG6, %r13 // B
	movq	ARG7, %r14 // sdb
	sall	$ 5, %r14d // 4*sdb*sizeof(double)
	movq	ARG5, %r15 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NN_RL_8X4_VS_LIB4
#else
	CALL(inner_edge_dtrmm_nn_rl_8x4_vs_lib4)
#endif

	// call inner dgemm kernel nt after initial triangle

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_8x4_lib4)
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha

#if MACRO_LEVEL>=1
	INNER_SCALE_A0_8X4_LIB4
#else
	CALL(inner_scale_a0_8x4_lib4)
#endif


	// store n

	movq	ARG8, %r10 // offsetD
	movq	ARG9, %r11 // D
	movq	ARG10, %r12 // sdd
	sall	$ 5, %r12d // 4*sdb*sizeof(double)
	movq	ARG11, %r13 // m0
	movq	ARG12, %r14 // m1
	movq	ARG13, %r15 // n0
	movq	ARG14, %rax // n1

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_GEN_LIB4
#else
	CALL(inner_store_8x4_gen_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nn_rl_8x4_gen_lib4)





//                                  1      2              3          4        5          6          7
// void kernel_dtrmm_nt_ru_8x4_lib4(int k, double *alpha, double *A, int sda, double *B, double *D, int sdd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_ru_8x4_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt after initial triangle

	movq	ARG1, %r10 // k
	subl	$ 4, %r10d //k-4
	movq	ARG3, %r11 // A
	addq	$ 128, %r11 // A+4*bs
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13 // B
	addq	$ 128, %r13 // B+4*bs


#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x4_lib4)
#endif


	// call inner blend

#if MACRO_LEVEL>=1
	INNER_BLEND_8X4_LIB4
#else
	CALL(inner_blend_8x4_lib4)
#endif


	// initial triangle

	movq	ARG1, %r10 // A
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13 // B

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RU_8X4_LIB4
#else
	CALL(inner_edge_dtrmm_nt_ru_8x4_lib4)
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha

#if MACRO_LEVEL>=1
	INNER_SCALE_A0_8X4_LIB4
#else
	CALL(inner_scale_a0_8x4_lib4)
#endif


	// store n

	movq	ARG6, %r10 // D
	movq	ARG7, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB4
#else
	CALL(inner_store_8x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_ru_8x4_lib4)





//                                  1      2              3          4        5          6          7        8       9
// void kernel_dtrmm_nt_ru_8x4_lib4(int k, double *alpha, double *A, int sda, double *B, double *D, int sdd, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_ru_8x4_vs_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt after initial triangle

	movq	ARG1, %r10 // k
	subl	$ 4, %r10d //k-4
	movq	ARG3, %r11 // A
	addq	$ 128, %r11 // A+4*bs
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13 // B
	addq	$ 128, %r13 // B+4*bs


#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x4_lib4)
#endif


	// call inner blender nn

#if MACRO_LEVEL>=1
	INNER_BLEND_8X4_LIB4
#else
	CALL(inner_blend_8x4_lib4)
#endif


	// initial triangle

	movq	ARG1, %r10
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13 // B

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RU_8X4_VS_LIB4
#else
	CALL(inner_edge_dtrmm_nt_ru_8x4_vs_lib4)
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB4
#else
	CALL(inner_scale_ab_8x4_lib4)
#endif


// store n

	movq	ARG6, %r10 // store address D
	movq	ARG7, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG8, %r12 // km 
	movq	ARG9, %r13 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_VS_LIB4
#else
	CALL(inner_store_8x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_ru_8x4_vs_lib4)





//                                  rdi    rsi        rdx      rcx        r8         r9       rsp+8      rsp+16   rsp+24
// void kernel_dpotrf_nt_l_8x4_lib4(int k, double *A, int sda, double *B, double *C, int sdc, double *D, int sdd, double *inv_diag_D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpotrf_nt_l_8x4_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x4_lib4)
#endif


	// call inner blender nn

	movq	ARG5, %r10 // C
	movq	ARG6, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_M11_8X4_LIB4
#else
	CALL(inner_blend_scale_m11_8x4_lib4)
#endif


	// factorization

	movq	ARG9, %r10  // inv_diag_D

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_8X4_LIB4
#else
	CALL(inner_edge_dpotrf_8x4_lib4)
#endif


	// store n

	movq	ARG7, %r10 // store address D
	movq	ARG8, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X4_LIB4
#else
	CALL(inner_store_l_8x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dpotrf_nt_l_8x4_lib4)





//                                     rdi    rsi        rdx      rcx        r8         r9       rsp+8      rsp+16   rsp+24              rsp+32  rsp+40
// void kernel_dpotrf_nt_l_8x4_vs_lib4(int k, double *A, int sda, double *B, double *C, int sdc, double *D, int sdd, double *inv_diag_D, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpotrf_nt_l_8x4_vs_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x4_lib4)
#endif


	// call inner blender nn

	movq	ARG5, %r10 // C
	movq	ARG6, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_M11_8X4_LIB4
#else
	CALL(inner_blend_scale_m11_8x4_lib4)
#endif


	// factorization

	movq	ARG9, %r10  // inv_diag_D
	movq	ARG11, %r11 // kn

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_8X4_VS_LIB4
#else
	CALL(inner_edge_dpotrf_8x4_vs_lib4)
#endif


	// store n

	movq	ARG7, %r10 // store address D
	movq	ARG8, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

	movq	ARG10, %r12 // km
	movq	ARG11, %r13 // kn

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X4_VS_LIB4
#else
	CALL(inner_store_l_8x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dpotrf_nt_l_8x4_vs_lib4)





//                                        rdi     rsi         rdx       rcx         r8      r9          rsp+8     rsp+16      rsp+24     rsp+32   rsp+40     rsp+48   rsp+56
// void kernel_dsyrk_dpotrf_nt_l_8x4_lib4(int kp, double *Ap, int sdap, double *Bp, int km, double *Am, int sdam, double *Bm, double *C, int sdc, double *D, int sdd, double *inv_diag_D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_dpotrf_nt_l_8x4_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG2, %r11  // Ap
	movq	ARG3, %r12 // sdap
	sall	$ 5, %r12d   // 4*sdap*sizeof(double)
	movq	ARG4, %r13  // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x4_lib4)
#endif


	// change sign
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd		.LC11(%rip), %ymm15
#elif defined(OS_MAC)
	vmovapd		LC11(%rip), %ymm15
#endif
	vxorpd		%ymm15, %ymm0, %ymm0
	vxorpd		%ymm15, %ymm1, %ymm1
	vxorpd		%ymm15, %ymm2, %ymm2
	vxorpd		%ymm15, %ymm3, %ymm3
	vxorpd		%ymm15, %ymm4, %ymm4
	vxorpd		%ymm15, %ymm5, %ymm5
	vxorpd		%ymm15, %ymm6, %ymm6
	vxorpd		%ymm15, %ymm7, %ymm7

	// call inner dgemm kernel nt sub

	movq	ARG5, %r10                 // km
	movq	ARG6, %r11                   // Am
	movq	ARG7, %r12 // sdam
	sall	$ 5, %r12d                   // 4*sdam*sizeof(double)
	movq	ARG8, %r13  // Bm

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x4_lib4)
#endif


	// call inner blender nn

	movq	ARG9, %r10 // C
	movq	ARG10, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_M11_8X4_LIB4
#else
	CALL(inner_blend_scale_m11_8x4_lib4)
#endif


	// factorization

	movq	ARG13, %r10  // inv_diag_D

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_8X4_LIB4
#else
	CALL(inner_edge_dpotrf_8x4_lib4)
#endif


	// store n

	movq	ARG11, %r10 // store address D
	movq	ARG12, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X4_LIB4
#else
	CALL(inner_store_l_8x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dsyrk_dpotrf_nt_l_8x4_lib4)





//                                           rdi     rsi         rdx       rcx         r8      r9          rsp+8     rsp+16      rsp+24     rsp+32   rsp+40     rsp+48   rsp+56              rsp+64  rsp+72
// void kernel_dsyrk_dpotrf_nt_l_8x4_vs_lib4(int kp, double *Ap, int sdap, double *Bp, int km, double *Am, int sdam, double *Bm, double *C, int sdc, double *D, int sdd, double *inv_diag_D, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_dpotrf_nt_l_8x4_vs_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG2, %r11  // Ap
	movq	ARG3, %r12 // sdap
	sall	$ 5, %r12d   // 4*sdap*sizeof(double)
	movq	ARG4, %r13  // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x4_lib4)
#endif


	// change sign
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd		.LC11(%rip), %ymm15
#elif defined(OS_MAC)
	vmovapd		LC11(%rip), %ymm15
#endif
	vxorpd		%ymm15, %ymm0, %ymm0
	vxorpd		%ymm15, %ymm1, %ymm1
	vxorpd		%ymm15, %ymm2, %ymm2
	vxorpd		%ymm15, %ymm3, %ymm3
	vxorpd		%ymm15, %ymm4, %ymm4
	vxorpd		%ymm15, %ymm5, %ymm5
	vxorpd		%ymm15, %ymm6, %ymm6
	vxorpd		%ymm15, %ymm7, %ymm7

	// call inner dgemm kernel nt sub

	movq	ARG5, %r10                 // km
	movq	ARG6, %r11                   // Am
	movq	ARG7, %r12 // sdam
	sall	$ 5, %r12d                   // 4*sdam*sizeof(double)
	movq	ARG8, %r13  // Bm

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x4_lib4)
#endif


	// call inner blender nn

	movq	ARG9, %r10 // C
	movq	ARG10, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_M11_8X4_LIB4
#else
	CALL(inner_blend_scale_m11_8x4_lib4)
#endif


	// factorization

	movq	ARG13, %r10  // inv_diag_D
	movq	ARG15, %r11 // kn

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_8X4_VS_LIB4
#else
	CALL(inner_edge_dpotrf_8x4_vs_lib4)
#endif


	// store n

	movq	ARG11, %r10 // store address D
	movq	ARG12, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

	movq	ARG14, %r12 // km
	movq	ARG15, %r13 // kn

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X4_VS_LIB4
#else
	CALL(inner_store_l_8x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dsyrk_dpotrf_nt_l_8x4_vs_lib4)





//                                      1      2          3        4          5             6          7        8          9        10         11
// void kernel_dtrsm_nt_rl_inv_8x4_lib4(int k, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd, double *E, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_inv_8x4_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x4_lib4)
#endif


	// call inner blender nn

	movq	ARG5, %r10 // beta
	movq	ARG6, %r11 // C
	movq	ARG7, %r12 // sdc
	sall	$ 5, %r12d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_M1B_8X4_LIB4
#else
	CALL(inner_blend_scale_m1b_8x4_lib4)
#endif


	// solve

	movq	ARG10, %r10  // E
	movq	ARG11, %r11  // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_8X4_LIB4
#else
	CALL(inner_edge_dtrsm_rlt_inv_8x4_lib4)
#endif


	// store n

	movq	ARG8, %r10 // store address D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB4
#else
	CALL(inner_store_8x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_8x4_lib4)





//                                         1      2          3        4          5             6          7        8          9        10         11                  12      13
// void kernel_dtrsm_nt_rl_inv_8x4_vs_lib4(int k, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd, double *E, double *inv_diag_E, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_inv_8x4_vs_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x4_lib4)
#endif


	// call inner blender nn

	movq	ARG5, %r10 // beta
	movq	ARG6, %r11 // C
	movq	ARG7, %r12 // sdc
	sall	$ 5, %r12d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_M1B_8X4_LIB4
#else
	CALL(inner_blend_scale_m1b_8x4_lib4)
#endif


	// solve

	movq	ARG10, %r10  // E
	movq	ARG11, %r11  // inv_diag_E
	movq	ARG13, %r12 // kn

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_8X4_VS_LIB4
#else
	CALL(inner_edge_dtrsm_rlt_inv_8x4_vs_lib4)
#endif


	// store n

	movq	ARG8, %r10 // store address D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

	movq	ARG12, %r12 // km
	movq	ARG13, %r13 // kn

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_VS_LIB4
#else
	CALL(inner_store_8x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_8x4_vs_lib4)





//                                               rdi     rsi         rdx       rcx          r8     r9          rsp+8     rsp+16      rsp+24     rsp+32   rsp+40     rsp+48   rsp+56     rsp+64              rsp+72  rsp+80
// void kernel_dgemm_dtrsm_nt_rl_inv_8x4_vs_lib4(int kp, double *Ap, int sdap, double *Bp, int km, double *Am, int sdam, double *Bm, double *C, int sdc, double *D, int sdd, double *E, double *inv_diag_E, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_dtrsm_nt_rl_inv_8x4_vs_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG2, %r11  // Ap
	movq	ARG3, %r12 // sdap
	sall	$ 5, %r12d   // 4*sdap*sizeof(double)
	movq	ARG4, %r13  // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x4_lib4)
#endif


	// change sign
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd		.LC11(%rip), %ymm15
#elif defined(OS_MAC)
	vmovapd		LC11(%rip), %ymm15
#endif
	vxorpd		%ymm15, %ymm0, %ymm0
	vxorpd		%ymm15, %ymm1, %ymm1
	vxorpd		%ymm15, %ymm2, %ymm2
	vxorpd		%ymm15, %ymm3, %ymm3
	vxorpd		%ymm15, %ymm4, %ymm4
	vxorpd		%ymm15, %ymm5, %ymm5
	vxorpd		%ymm15, %ymm6, %ymm6
	vxorpd		%ymm15, %ymm7, %ymm7

	// call inner dgemm kernel nt sub

	movq	ARG5, %r10                 // km
	movq	ARG6, %r11                   // Am
	movq	ARG7, %r12 // sdam
	sall	$ 5, %r12d                   // 4*sda*sizeof(double)
	movq	ARG8, %r13  // Bm

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x4_lib4)
#endif


	// call inner blender nn

	movq	ARG9, %r10  // C
	movq	ARG10, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_M11_8X4_LIB4
#else
	CALL(inner_blend_scale_m11_8x4_lib4)
#endif


	// solve

	movq	ARG13, %r10  // E
	movq	ARG14, %r11  // inv_diag_E
	movq	ARG16, %r12 // kn

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_8X4_VS_LIB4
#else
	CALL(inner_edge_dtrsm_rlt_inv_8x4_vs_lib4)
#endif


	// store n

	movq	ARG11, %r10 // store address D
	movq	ARG12, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

	movq	ARG15, %r12 // km
	movq	ARG16, %r13 // kn

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_VS_LIB4
#else
	CALL(inner_store_8x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_dtrsm_nt_rl_inv_8x4_vs_lib4)





//                                            rdi     rsi         rdx       rcx         r8      r9          rsp+8     rsp+16      rsp+24     rsp+32   rsp+40     rsp+48   rsp+56     rsp+64
// void kernel_dgemm_dtrsm_nt_rl_inv_8x4_lib4(int kp, double *Ap, int sdap, double *Bp, int km, double *Am, int sdam, double *Bm, double *C, int sdc, double *D, int sdd, double *E, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_dtrsm_nt_rl_inv_8x4_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG2, %r11  // Ap
	movq	ARG3, %r12 // sdap
	sall	$ 5, %r12d   // 4*sdap*sizeof(double)
	movq	ARG4, %r13  // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x4_lib4)
#endif


	// change sign
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd		.LC11(%rip), %ymm15
#elif defined(OS_MAC)
	vmovapd		LC11(%rip), %ymm15
#endif
	vxorpd		%ymm15, %ymm0, %ymm0
	vxorpd		%ymm15, %ymm1, %ymm1
	vxorpd		%ymm15, %ymm2, %ymm2
	vxorpd		%ymm15, %ymm3, %ymm3
	vxorpd		%ymm15, %ymm4, %ymm4
	vxorpd		%ymm15, %ymm5, %ymm5
	vxorpd		%ymm15, %ymm6, %ymm6
	vxorpd		%ymm15, %ymm7, %ymm7

	// call inner dgemm kernel nt sub

	movq	ARG5, %r10                 // km
	movq	ARG6, %r11                   // Am
	movq	ARG7, %r12 // sdam
	sall	$ 5, %r12d                   // 4*sda*sizeof(double)
	movq	ARG8, %r13  // Bm

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x4_lib4)
#endif


	// call inner blender nn

	movq	ARG9, %r10  // C
	movq	ARG10, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_M11_8X4_LIB4
#else
	CALL(inner_blend_scale_m11_8x4_lib4)
#endif


	// solve

	movq	ARG13, %r10  // E
	movq	ARG14, %r11  // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_8X4_LIB4
#else
	CALL(inner_edge_dtrsm_rlt_inv_8x4_lib4)
#endif


	// store n

	movq	ARG11, %r10 // store address D
	movq	ARG12, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB4
#else
	CALL(inner_store_8x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_dtrsm_nt_rl_inv_8x4_lib4)





//                                      1      2          3        4          5             6          7        8          9        10
// void kernel_dtrsm_nt_rl_one_8x4_lib4(int k, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd, double *E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_one_8x4_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x4_lib4)
#endif


	// call inner blender nn

	movq	ARG5, %r10 // beta
	movq	ARG6, %r11 // C
	movq	ARG7, %r12 // sdc
	sall	$ 5, %r12d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_M1B_8X4_LIB4
#else
	CALL(inner_blend_scale_m1b_8x4_lib4)
#endif


	// solve

	movq	ARG10, %r10  // E

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_ONE_8X4_LIB4
#else
	CALL(inner_edge_dtrsm_rlt_one_8x4_lib4)
#endif


	// store n

	movq	ARG8, %r10 // store address D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB4
#else
	CALL(inner_store_8x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_rl_one_8x4_lib4)





//                                         1      2          3        4          5             6          7        8          9        10         11      12
// void kernel_dtrsm_nt_rl_one_8x4_vs_lib4(int k, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd, double *E, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_one_8x4_vs_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x4_lib4)
#endif


	// call inner blender nn

	movq	ARG5, %r10 // beta
	movq	ARG6, %r11 // C
	movq	ARG7, %r12 // sdc
	sall	$ 5, %r12d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_M1B_8X4_LIB4
#else
	CALL(inner_blend_scale_m1b_8x4_lib4)
#endif


	// solve

	movq	ARG10, %r10  // E
	movq	ARG12, %r11 // kn

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_ONE_8X4_VS_LIB4
#else
	CALL(inner_edge_dtrsm_rlt_one_8x4_vs_lib4)
#endif


	// store n

	movq	ARG8, %r10 // store address D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

	movq	ARG11, %r12 // km
	movq	ARG12, %r13 // kn

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_VS_LIB4
#else
	CALL(inner_store_8x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_rl_one_8x4_vs_lib4)





//                                      1      2          3        4          5             6          7        8          9        10         11
// void kernel_dtrsm_nt_ru_inv_8x4_lib4(int k, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd, double *E, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_ru_inv_8x4_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x4_lib4)
#endif


	// call inner blender nn

	movq	ARG5, %r10 // beta
	movq	ARG6, %r11 // C
	movq	ARG7, %r12 // sdc
	sall	$ 5, %r12d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_M1B_8X4_LIB4
#else
	CALL(inner_blend_scale_m1b_8x4_lib4)
#endif


	// solve

	movq	ARG10, %r10  // E
	movq	ARG11, %r11  // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUT_INV_8X4_LIB4
#else
	CALL(inner_edge_dtrsm_rut_inv_8x4_lib4)
#endif


	// store n

	movq	ARG8, %r10 // store address D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB4
#else
	CALL(inner_store_8x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_ru_inv_8x4_lib4)





//                                         1      2          3        4          5             6          7        8          9        10         11                  12      13
// void kernel_dtrsm_nt_ru_inv_8x4_vs_lib4(int k, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd, double *E, double *inv_diag_E, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_ru_inv_8x4_vs_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x4_lib4)
#endif


	// call inner blender nn

	movq	ARG5, %r10 // beta
	movq	ARG6, %r11 // C
	movq	ARG7, %r12 // sdc
	sall	$ 5, %r12d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_M1B_8X4_LIB4
#else
	CALL(inner_blend_scale_m1b_8x4_lib4)
#endif


	// solve

	movq	ARG10, %r10  // E 
	movq	ARG11, %r11  // inv_diag_E 
	movq	ARG13, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUT_INV_8X4_VS_LIB4
#else
	CALL(inner_edge_dtrsm_rut_inv_8x4_vs_lib4)
#endif


	// store n

	movq	ARG8, %r10 // store address D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG12, %r12 // km 
	movq	ARG13, %r13 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_VS_LIB4
#else
	CALL(inner_store_8x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_ru_inv_8x4_vs_lib4)





//                                      1      2          3        4          5             6          7        8          9        10
// void kernel_dtrsm_nt_ru_one_8x4_lib4(int k, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd, double *E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_ru_one_8x4_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x4_lib4)
#endif


	// call inner blender nn

	movq	ARG5, %r10 // beta
	movq	ARG6, %r11 // C
	movq	ARG7, %r12 // sdc
	sall	$ 5, %r12d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_M1B_8X4_LIB4
#else
	CALL(inner_blend_scale_m1b_8x4_lib4)
#endif


	// solve

	movq	ARG10, %r10  // E

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUT_ONE_8X4_LIB4
#else
	CALL(inner_edge_dtrsm_rut_one_8x4_lib4)
#endif


	// store n

	movq	ARG8, %r10 // store address D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB4
#else
	CALL(inner_store_8x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_ru_one_8x4_lib4)





//                                         1      2          3        4          5             6          7        8          9        10         11      12
// void kernel_dtrsm_nt_ru_one_8x4_vs_lib4(int k, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd, double *E, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_ru_one_8x4_vs_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x4_lib4)
#endif


	// call inner blender nn

	movq	ARG5, %r10 // beta
	movq	ARG6, %r11 // C
	movq	ARG7, %r12 // sdc
	sall	$ 5, %r12d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_M1B_8X4_LIB4
#else
	CALL(inner_blend_scale_m1b_8x4_lib4)
#endif


	// solve

	movq	ARG10, %r10  // E 
	movq	ARG12, %r11 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUT_ONE_8X4_VS_LIB4
#else
	CALL(inner_edge_dtrsm_rut_one_8x4_vs_lib4)
#endif


	// store n

	movq	ARG8, %r10 // store address D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG11, %r12 // km 
	movq	ARG12, %r13 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_VS_LIB4
#else
	CALL(inner_store_8x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_ru_one_8x4_vs_lib4)





//                                      1      2          3        4          5        6             7          8        9          10       11         12
// void kernel_dtrsm_nn_ru_inv_8x4_lib4(int k, double *A, int sda, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd, double *E, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_ru_inv_8x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // sda
	sall	$ 5, %r14d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_8x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // sdc
	sall	$ 5, %r12d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_8X4_LIB4
#else
	CALL(inner_scale_m1b_8x4_lib4)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUN_INV_8X4_LIB4
#else
	CALL(inner_edge_dtrsm_run_inv_8x4_lib4)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB4
#else
	CALL(inner_store_8x4_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dtrsm_nn_ru_inv_8x4_lib4)





//                                         1      2          3        4          5        6             7          8        9          10       11         12                  13      14
// void kernel_dtrsm_nn_ru_inv_8x4_vs_lib4(int k, double *A, int sda, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd, double *E, double *inv_diag_E, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_ru_inv_8x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // sda
	sall	$ 5, %r14d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_8x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // sdc
	sall	$ 5, %r12d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_8X4_LIB4
#else
	CALL(inner_scale_m1b_8x4_lib4)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUN_INV_8X4_LIB4
#else
	CALL(inner_edge_dtrsm_run_inv_8x4_lib4)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

	movq	ARG13, %r12 // km
	movq	ARG14, %r13 // kn

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_VS_LIB4
#else
	CALL(inner_store_8x4_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dtrsm_nn_ru_inv_8x4_vs_lib4)





//                                      1      2          3        4          5        6             7          8        9          10       11         12       13
// void kernel_dtrsm_nn_ll_inv_8x4_lib4(int k, double *A, int sda, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd, double *E, int sde, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_ll_inv_8x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // sda
	sall	$ 5, %r14d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_8x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // sdc
	sall	$ 5, %r12d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_8X4_LIB4
#else
	CALL(inner_scale_m1b_8x4_lib4)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // sde
	sall	$ 5, %r11d // 4*sde*sizeof(double)
	movq	ARG13, %r12  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_LLN_INV_8X4_LIB4
#else
	CALL(inner_edge_dtrsm_lln_inv_8x4_lib4)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB4
#else
	CALL(inner_store_8x4_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dtrsm_nn_ll_inv_8x4_lib4)





//                                         1      2          3        4          5        6             7          8        9          10       11         12        13                 14      15
// void kernel_dtrsm_nn_ll_inv_8x4_vs_lib4(int k, double *A, int sda, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd, double *E, int sde, double *inv_diga_E, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_ll_inv_8x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // sda
	sall	$ 5, %r14d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_8x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // sdc
	sall	$ 5, %r12d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_8X4_LIB4
#else
	CALL(inner_scale_m1b_8x4_lib4)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // sde
	sall	$ 5, %r11d // 4*sde*sizeof(double)
	movq	ARG13, %r12  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_LLN_INV_8X4_LIB4
#else
	CALL(inner_edge_dtrsm_lln_inv_8x4_lib4)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

	movq	ARG14, %r12 // km
	movq	ARG15, %r13 // kn

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_VS_LIB4
#else
	CALL(inner_store_8x4_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dtrsm_nn_ll_inv_8x4_vs_lib4)





//                                      1      2          3        4          5        6             7          8        9          10       11         12
// void kernel_dtrsm_nn_ll_one_8x4_lib4(int k, double *A, int sda, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd, double *E, int sde);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_ll_one_8x4_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // sda
	sall	$ 5, %r14d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_8x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // sdc
	sall	$ 5, %r12d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_8X4_LIB4
#else
	CALL(inner_scale_m1b_8x4_lib4)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // sde
	sall	$ 5, %r11d // 4*sde*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_LLN_ONE_8X4_LIB4
#else
	CALL(inner_edge_dtrsm_lln_one_8x4_lib4)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB4
#else
	CALL(inner_store_8x4_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dtrsm_nn_ll_one_8x4_lib4)





//                                         1      2          3        4          5        6             7          8        9          10       11         12        13     14
// void kernel_dtrsm_nn_ll_one_8x4_vs_lib4(int k, double *A, int sda, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd, double *E, int sde, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_ll_one_8x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // sda
	sall	$ 5, %r14d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_8x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // sdc
	sall	$ 5, %r12d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_8X4_LIB4
#else
	CALL(inner_scale_m1b_8x4_lib4)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // sde
	sall	$ 5, %r11d // 4*sde*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_LLN_ONE_8X4_LIB4
#else
	CALL(inner_edge_dtrsm_lln_one_8x4_lib4)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

	movq	ARG13, %r12 // km
	movq	ARG14, %r13 // kn

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_VS_LIB4
#else
	CALL(inner_store_8x4_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dtrsm_nn_ll_one_8x4_vs_lib4)





//                                      edi    rsi        rdx      ecx        r8       r9         rsp+8    rsp+16     rsp+24   rsp+32     rsp+40   rsp+48
// void kernel_dtrsm_nn_lu_inv_8x4_lib4(int k, double *A, int sda, double *B, int sdb, double *C, int sdc, double *D, int sdd, double *E, int sde, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_lu_inv_8x4_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // sda
	sall	$ 5, %r14d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_8x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // C
	movq	ARG7, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_8X4_LIB4
#else
	CALL(inner_scale_m11_8x4_lib4)
#endif


	// solve

	movq	ARG10, %r10  // E
	movq	ARG11, %r11 // sde
	sall	$ 5, %r11d // 4*sde*sizeof(double)
	movq	ARG12, %r12  // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_LUN_INV_8X4_LIB4
#else
	CALL(inner_edge_dtrsm_lun_inv_8x4_lib4)
#endif


	// store

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB4
#else
	CALL(inner_store_8x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nn_lu_inv_8x4_lib4)





//                                         edi    rsi        rdx      ecx        r8       r9         rsp+8    rsp+16     rsp+24   rsp+32     rsp+40   rsp+48              rsp+56  rsp+64
// void kernel_dtrsm_nn_lu_inv_8x4_vs_lib4(int k, double *A, int sda, double *B, int sdb, double *C, int sdc, double *D, int sdd, double *E, int sde, double *inv_diag_E, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_lu_inv_8x4_vs_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // sda
	sall	$ 5, %r14d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_8x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // C
	movq	ARG7, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_8X4_LIB4
#else
	CALL(inner_scale_m11_8x4_lib4)
#endif


	// solve

	movq	ARG10, %r10  // E
	movq	ARG11, %r11 // sde
	sall	$ 5, %r11d // 4*sde*sizeof(double)
	movq	ARG12, %r12  // inv_diag_E
	movq	ARG13, %r13  // km

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_LUN_INV_8X4_VS_LIB4
#else
	CALL(inner_edge_dtrsm_lun_inv_8x4_vs_lib4)
#endif


	// store

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG13, %r12  // km
	movq	ARG14, %r13  // km

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_VS_LIB4
#else
	CALL(inner_store_8x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nn_lu_inv_8x4_vs_lib4)





//                                edi    rsi        rdx      rcx        r8       r9         rsp+8    rsp+16     rsp+24   rsp+32
// void kernel_dgetrf_nn_l_8x4_lib4(int k, double *A, int sda, double *B, int sdb, double *C, int sdc, double *D, int sdd, double *inv_diag_D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgetrf_nn_l_8x4_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // sda
	sall	$ 5, %r14d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_8x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // C
	movq	ARG7, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_8X4_LIB4
#else
	CALL(inner_scale_m11_8x4_lib4)
#endif


	// factorization

	movq	ARG10, %r10  // inv_diag_D

#if MACRO_LEVEL>=1
	INNER_EDGE_DGETRF_L_8X4_LIB4
#else
	CALL(inner_edge_dgetrf_l_8x4_lib4)
#endif


	// store

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB4
#else
	CALL(inner_store_8x4_lib4)
#endif


	// epilogue

	EPILOGUE

	ret

	FUN_END(kernel_dgetrf_nn_l_8x4_lib4)





//                                   edi    rsi        rdx      rcx        r8       r9         rsp+8    rsp+16     rsp+24   rsp+32              rsp+40  rsp+48
// void kernel_dgetrf_nn_l_8x4_vs_lib4(int k, double *A, int sda, double *B, int sdb, double *C, int sdc, double *D, int sdd, double *inv_diag_D, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgetrf_nn_l_8x4_vs_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // sda
	sall	$ 5, %r14d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_8x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // C
	movq	ARG7, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_8X4_LIB4
#else
	CALL(inner_scale_m11_8x4_lib4)
#endif


	// factorization

	movq	ARG10, %r10  // inv_diag_D

#if MACRO_LEVEL>=1
	INNER_EDGE_DGETRF_L_8X4_LIB4
#else
	CALL(inner_edge_dgetrf_l_8x4_lib4)
#endif


	// store

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG11, %r12  // km
	movq	ARG12, %r13  // km

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_VS_LIB4
#else
	CALL(inner_store_8x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgetrf_nn_l_8x4_vs_lib4)





//                                  1      2          3        4          5        6          7        8          9        10
// void kernel_dgetrf_nn_r 8x4_lib4(int k, double *A, int sda, double *B, int sdb, double *C, int sdc, double *D, int sdd, double *inv_diag_D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgetrf_nn_r_8x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // sda
	sall	$ 5, %r14d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_8x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // C
	movq	ARG7, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_8X4_LIB4
#else
	CALL(inner_scale_m11_8x4_lib4)
#endif


	// factorization

//	movq	ARG10, %r10  // inv_diag_D 
	movq	ARG8, %r10 // D
	subq	$ 128, %r10 // E
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG10, %r12  // inv_diag_D 

#if MACRO_LEVEL>=1
	INNER_EDGE_DGETRF_R_8X4_LIB4
#else
	CALL(inner_edge_dgetrf_r_8x4_lib4)
#endif


	// store

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB4
#else
	CALL(inner_store_8x4_lib4)
#endif


	// epilogue

	EPILOGUE

	ret

	FUN_END(kernel_dgetrf_nn_r_8x4_lib4)





//                                     1      2          3        4          5        6          7        8          9        10                  11      12
// void kernel_dgetrf_nn_r_8x4_vs_lib4(int k, double *A, int sda, double *B, int sdb, double *C, int sdc, double *D, int sdd, double *inv_diag_D, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgetrf_nn_r_8x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // sda
	sall	$ 5, %r14d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_8x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // C
	movq	ARG7, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_8X4_LIB4
#else
	CALL(inner_scale_m11_8x4_lib4)
#endif


	// factorization

//	movq	ARG10, %r10  // inv_diag_D 
	movq	ARG8, %r10 // D
	subq	$ 128, %r10 // E
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG10, %r12  // inv_diag_D 

#if MACRO_LEVEL>=1
	INNER_EDGE_DGETRF_R_8X4_LIB4
#else
	CALL(inner_edge_dgetrf_r_8x4_lib4)
#endif


	// store

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG11, %r12  // km
	movq	ARG12, %r13  // km

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_VS_LIB4
#else
	CALL(inner_store_8x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgetrf_nn_r_8x4_vs_lib4)





//                             1         2           3           4           5
// void kernel_dlarfb4_rn_8_lib4(int kmax, double *pV, double *pT, double *pD, int sdd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dlarfb4_rn_8_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC

	movq	ARG1, %r10 // k
	movq	ARG4, %r11 // D
	movq	ARG5, %r12 // sdd
	sall	$ 5, %r12d
	movq	ARG2, %r13 // V

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r13

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x4_lib4)
#endif

#if MACRO_LEVEL>=1
	INNER_BLEND_8X4_LIB4
#else
	CALL(inner_blend_8x4_lib4)
#endif

	movq	ARG1, %r10 // k
	movq	ARG4, %r11 // D
	movq	ARG5, %r12 // sdd
	sall	$ 5, %r12d
	movq	ARG2, %r13 // V

	//
	vmovapd			0(%r11), %ymm12
	vmovapd			0(%r11, %r12, 1), %ymm14
	vaddpd			%ymm12, %ymm0, %ymm0
	vaddpd			%ymm14, %ymm4, %ymm4
	//
	vmovapd			32(%r11), %ymm12
	vmovapd			32(%r11, %r12, 1), %ymm14
	vaddpd			%ymm12, %ymm1, %ymm1
	vaddpd			%ymm14, %ymm5, %ymm5
	vbroadcastsd	32(%r13), %ymm13
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	//
	vmovapd			64(%r11), %ymm12
	vmovapd			64(%r11, %r12, 1), %ymm14
	vaddpd			%ymm12, %ymm2, %ymm2
	vaddpd			%ymm14, %ymm6, %ymm6
	vbroadcastsd	64(%r13), %ymm13
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	72(%r13), %ymm13
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	//
	vmovapd			96(%r11), %ymm12
	vmovapd			96(%r11, %r12, 1), %ymm14
	vaddpd			%ymm12, %ymm3, %ymm3
	vaddpd			%ymm14, %ymm7, %ymm7
	vbroadcastsd	96(%r13), %ymm13
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	104(%r13), %ymm13
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastsd	112(%r13), %ymm13
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6

	movq	ARG3, %r10 // T

	//
	vbroadcastsd	120(%r10), %ymm12
	vmulpd			%ymm3, %ymm12, %ymm3
	vmulpd			%ymm7, %ymm12, %ymm7
	//
	vbroadcastsd	112(%r10), %ymm12
	vmulpd			%ymm2, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm6, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7
	vbroadcastsd	80(%r10), %ymm12
	vmulpd			%ymm2, %ymm12, %ymm2
	vmulpd			%ymm6, %ymm12, %ymm6
	//
	vbroadcastsd	104(%r10), %ymm12
	vmulpd			%ymm1, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm5, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7
	vbroadcastsd	72(%r10), %ymm12
	vmulpd			%ymm1, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm5, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6
	vbroadcastsd	40(%r10), %ymm12
	vmulpd			%ymm1, %ymm12, %ymm1
	vmulpd			%ymm5, %ymm12, %ymm5
	//
	vbroadcastsd	96(%r10), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm4, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7
	vbroadcastsd	64(%r10), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm4, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6
	vbroadcastsd	32(%r10), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm4, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastsd	0(%r10), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm0
	vmulpd			%ymm4, %ymm12, %ymm4

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // V
	movq	ARG4, %r12 // D
	movq	ARG5, %r13 // sdd
	sall	$ 5, %r13d

	//
	vmovapd			0(%r12), %ymm12
	vmovapd			0(%r12, %r13, 1), %ymm14
	vaddpd			%ymm12, %ymm0, %ymm12
	vaddpd			%ymm14, %ymm4, %ymm14
	vmovapd			%ymm12, 0(%r12)
	vmovapd			%ymm14, 0(%r12, %r13, 1)
	//
	vmovapd			32(%r12), %ymm12
	vmovapd			32(%r12, %r13, 1), %ymm14
	vbroadcastsd	32(%r11), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm12, %ymm12
	vmulpd			%ymm4, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm14, %ymm14
	vaddpd			%ymm12, %ymm1, %ymm12
	vaddpd			%ymm14, %ymm5, %ymm14
	vmovapd			%ymm12, 32(%r12)
	vmovapd			%ymm14, 32(%r12, %r13, 1)
	//
	vmovapd			64(%r12), %ymm12
	vmovapd			64(%r12, %r13, 1), %ymm14
	vbroadcastsd	64(%r11), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm12, %ymm12
	vmulpd			%ymm4, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm14, %ymm14
	vbroadcastsd	72(%r11), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm12, %ymm12
	vmulpd			%ymm5, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm14, %ymm14
	vaddpd			%ymm12, %ymm2, %ymm12
	vaddpd			%ymm14, %ymm6, %ymm14
	vmovapd			%ymm12, 64(%r12)
	vmovapd			%ymm14, 64(%r12, %r13, 1)
	//
	vmovapd			96(%r12), %ymm12
	vmovapd			96(%r12, %r13, 1), %ymm14
	vbroadcastsd	96(%r11), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm12, %ymm12
	vmulpd			%ymm4, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm14, %ymm14
	vbroadcastsd	104(%r11), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm12, %ymm12
	vmulpd			%ymm5, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm14, %ymm14
	vbroadcastsd	112(%r11), %ymm13
	vmulpd			%ymm2, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm12, %ymm12
	vmulpd			%ymm6, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm14, %ymm14
	vaddpd			%ymm12, %ymm3, %ymm12
	vaddpd			%ymm14, %ymm7, %ymm14
	vmovapd			%ymm12, 96(%r12)
	vmovapd			%ymm14, 96(%r12, %r13, 1)

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r12

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEBP_ADD_NN_8X4_LIB4
#else
	CALL(inner_kernel_dgebp_add_nn_8x4_lib4)
#endif

	EPILOGUE

	ret

	FUN_END(kernel_dlarfb4_rn_8_lib4)





//                                 1         2           3           4           5        6           7
// void kernel_dlarfb4_rn_8_la_lib4(int kmax, double *pV, double *pT, double *pD, int sdd, double *pA, int sda);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dlarfb4_rn_8_la_lib4)

		PROLOGUE

	// zero accumulation registers

	ZERO_ACC

	movq	ARG1, %r10 // k
	movq	ARG6, %r11 // A
	movq	ARG7, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG2, %r13 // V

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x4_lib4)
#endif

#if MACRO_LEVEL>=1
	INNER_BLEND_8X4_LIB4
#else
	CALL(inner_blend_8x4_lib4)
#endif

	movq	ARG4, %r10 // D
	movq	ARG5, %r11 // sdd
	sall	$ 5, %r11d

	//
	vmovapd			0(%r10), %ymm12
	vmovapd			0(%r10, %r11, 1), %ymm14
	vaddpd			%ymm12, %ymm0, %ymm0
	vaddpd			%ymm14, %ymm4, %ymm4
	//
	vmovapd			32(%r10), %ymm12
	vmovapd			32(%r10, %r11, 1), %ymm14
	vaddpd			%ymm12, %ymm1, %ymm1
	vaddpd			%ymm14, %ymm5, %ymm5
	//
	vmovapd			64(%r10), %ymm12
	vmovapd			64(%r10, %r11, 1), %ymm14
	vaddpd			%ymm12, %ymm2, %ymm2
	vaddpd			%ymm14, %ymm6, %ymm6
	//
	vmovapd			96(%r10), %ymm12
	vmovapd			96(%r10, %r11, 1), %ymm14
	vaddpd			%ymm12, %ymm3, %ymm3
	vaddpd			%ymm14, %ymm7, %ymm7

	movq	ARG3, %r10 // T

	//
	vbroadcastsd	120(%r10), %ymm12
	vmulpd			%ymm3, %ymm12, %ymm3
	vmulpd			%ymm7, %ymm12, %ymm7
	//
	vbroadcastsd	112(%r10), %ymm12
	vmulpd			%ymm2, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm6, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7
	vbroadcastsd	80(%r10), %ymm12
	vmulpd			%ymm2, %ymm12, %ymm2
	vmulpd			%ymm6, %ymm12, %ymm6
	//
	vbroadcastsd	104(%r10), %ymm12
	vmulpd			%ymm1, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm5, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7
	vbroadcastsd	72(%r10), %ymm12
	vmulpd			%ymm1, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm5, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6
	vbroadcastsd	40(%r10), %ymm12
	vmulpd			%ymm1, %ymm12, %ymm1
	vmulpd			%ymm5, %ymm12, %ymm5
	//
	vbroadcastsd	96(%r10), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm4, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7
	vbroadcastsd	64(%r10), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm4, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6
	vbroadcastsd	32(%r10), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm4, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastsd	0(%r10), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm0
	vmulpd			%ymm4, %ymm12, %ymm4

	movq	ARG4, %r10 // D
	movq	ARG5, %r11 // sdd
	sall	$ 5, %r11d

	//
	vmovapd			0(%r10), %ymm12
	vmovapd			0(%r10, %r11, 1), %ymm14
	vaddpd			%ymm12, %ymm0, %ymm12
	vaddpd			%ymm14, %ymm4, %ymm14
	vmovapd			%ymm12, 0(%r10)
	vmovapd			%ymm14, 0(%r10, %r11, 1)
	//
	vmovapd			32(%r10), %ymm12
	vmovapd			32(%r10, %r11, 1), %ymm14
	vaddpd			%ymm12, %ymm1, %ymm12
	vaddpd			%ymm14, %ymm5, %ymm14
	vmovapd			%ymm12, 32(%r10)
	vmovapd			%ymm14, 32(%r10, %r11, 1)
	//
	vmovapd			64(%r10), %ymm12
	vmovapd			64(%r10, %r11, 1), %ymm14
	vaddpd			%ymm12, %ymm2, %ymm12
	vaddpd			%ymm14, %ymm6, %ymm14
	vmovapd			%ymm12, 64(%r10)
	vmovapd			%ymm14, 64(%r10, %r11, 1)
	//
	vmovapd			96(%r10), %ymm12
	vmovapd			96(%r10, %r11, 1), %ymm14
	vaddpd			%ymm12, %ymm3, %ymm12
	vaddpd			%ymm14, %ymm7, %ymm14
	vmovapd			%ymm12, 96(%r10)
	vmovapd			%ymm14, 96(%r10, %r11, 1)

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // V
	movq	ARG6, %r12 // A
	movq	ARG7, %r13 // sda
	sall	$ 5, %r13d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEBP_ADD_NN_8X4_LIB4
#else
	CALL(inner_kernel_dgebp_add_nn_8x4_lib4)
#endif

	EPILOGUE

	ret

	FUN_END(kernel_dlarfb4_rn_8_la_lib4)





//                                  1       2       3            4            5           6           7        8           9        10          11
// void kernel_dlarfb4_rn_4_lla_lib4(int n0, int n1, double *pVL, double *pVA, double *pT, double *pD, int sdd, double *pL, int sdl, double *pA, int sda);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dlarfb4_rn_8_lla_lib4)

		PROLOGUE

	// zero accumulation registers

	ZERO_ACC

	// L

	movq	ARG1, %r10 // n0
	movq	ARG8, %r11 // L
	movq	ARG9, %r12 // sdl
	sall	$ 5, %r12d
	movq	ARG3, %r13 // VL

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x4_lib4)
#endif

	// A

	movq	ARG2, %r10 // n1
	movq	ARG10, %r11 // A
	movq	ARG11, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG4, %r13 // VA

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x4_lib4)
#endif

#if MACRO_LEVEL>=1
	INNER_BLEND_8X4_LIB4
#else
	CALL(inner_blend_8x4_lib4)
#endif

	// L final 4x4 lower triangular
	movq	ARG1, %r10 // n0
	sall	$ 5, %r10d // n0*ps*sizeof(double)
	movq	ARG8, %r11 // L
	addq	%r10, %r11 // L+n0*ps
	movq	ARG9, %r12 // sdl
	sall	$ 5, %r12d
	movq	ARG3, %r13 // VL
	addq	%r10, %r13 // VL+n0*ps

	// 4
	vmovapd			0(%r11), %ymm12
	vmovapd			0(%r11, %r12, 1), %ymm13
	vbroadcastsd	0(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm4, %ymm15, %ymm4
	vbroadcastsd	8(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5
	vbroadcastsd	16(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6
	vbroadcastsd	24(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm3, %ymm15, %ymm3
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm7, %ymm15, %ymm7
	// 3
	vmovapd			32(%r11), %ymm12
	vmovapd			32(%r11, %r12, 1), %ymm13
	vbroadcastsd	40(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5
	vbroadcastsd	48(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6
	vbroadcastsd	56(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm3, %ymm15, %ymm3
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm7, %ymm15, %ymm7
	// 2
	vmovapd			64(%r11), %ymm12
	vmovapd			64(%r11, %r12, 1), %ymm13
	vbroadcastsd	80(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6
	vbroadcastsd	88(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm3, %ymm15, %ymm3
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm7, %ymm15, %ymm7
	// 1
	vmovapd			96(%r11), %ymm12
	vmovapd			96(%r11, %r12, 1), %ymm13
	vbroadcastsd	120(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm3, %ymm15, %ymm3
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm7, %ymm15, %ymm7

	// D

	movq	ARG6, %r10 // D
	movq	ARG7, %r11 // sdd
	sall	$ 5, %r11d

	//
	vmovapd			0(%r10), %ymm12
	vmovapd			0(%r10, %r11, 1), %ymm14
	vaddpd			%ymm12, %ymm0, %ymm0
	vaddpd			%ymm14, %ymm4, %ymm4
	//
	vmovapd			32(%r10), %ymm12
	vmovapd			32(%r10, %r11, 1), %ymm14
	vaddpd			%ymm12, %ymm1, %ymm1
	vaddpd			%ymm14, %ymm5, %ymm5
	//
	vmovapd			64(%r10), %ymm12
	vmovapd			64(%r10, %r11, 1), %ymm14
	vaddpd			%ymm12, %ymm2, %ymm2
	vaddpd			%ymm14, %ymm6, %ymm6
	//
	vmovapd			96(%r10), %ymm12
	vmovapd			96(%r10, %r11, 1), %ymm14
	vaddpd			%ymm12, %ymm3, %ymm3
	vaddpd			%ymm14, %ymm7, %ymm7

	// T

	movq	ARG5, %r10 // T

	//
	vbroadcastsd	120(%r10), %ymm12
	vmulpd			%ymm3, %ymm12, %ymm3
	vmulpd			%ymm7, %ymm12, %ymm7
	//
	vbroadcastsd	112(%r10), %ymm12
	vmulpd			%ymm2, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm6, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7
	vbroadcastsd	80(%r10), %ymm12
	vmulpd			%ymm2, %ymm12, %ymm2
	vmulpd			%ymm6, %ymm12, %ymm6
	//
	vbroadcastsd	104(%r10), %ymm12
	vmulpd			%ymm1, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm5, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7
	vbroadcastsd	72(%r10), %ymm12
	vmulpd			%ymm1, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm5, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6
	vbroadcastsd	40(%r10), %ymm12
	vmulpd			%ymm1, %ymm12, %ymm1
	vmulpd			%ymm5, %ymm12, %ymm5
	//
	vbroadcastsd	96(%r10), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm4, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7
	vbroadcastsd	64(%r10), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm4, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6
	vbroadcastsd	32(%r10), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm4, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastsd	0(%r10), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm0
	vmulpd			%ymm4, %ymm12, %ymm4

	// D

	movq	ARG6, %r10 // D
	movq	ARG7, %r11 // sdd
	sall	$ 5, %r11d

	//
	vmovapd			0(%r10), %ymm12
	vmovapd			0(%r10, %r11, 1), %ymm14
	vaddpd			%ymm12, %ymm0, %ymm12
	vaddpd			%ymm14, %ymm4, %ymm14
	vmovapd			%ymm12, 0(%r10)
	vmovapd			%ymm14, 0(%r10, %r11, 1)
	//
	vmovapd			32(%r10), %ymm12
	vmovapd			32(%r10, %r11, 1), %ymm14
	vaddpd			%ymm12, %ymm1, %ymm12
	vaddpd			%ymm14, %ymm5, %ymm14
	vmovapd			%ymm12, 32(%r10)
	vmovapd			%ymm14, 32(%r10, %r11, 1)
	//
	vmovapd			64(%r10), %ymm12
	vmovapd			64(%r10, %r11, 1), %ymm14
	vaddpd			%ymm12, %ymm2, %ymm12
	vaddpd			%ymm14, %ymm6, %ymm14
	vmovapd			%ymm12, 64(%r10)
	vmovapd			%ymm14, 64(%r10, %r11, 1)
	//
	vmovapd			96(%r10), %ymm12
	vmovapd			96(%r10, %r11, 1), %ymm14
	vaddpd			%ymm12, %ymm3, %ymm12
	vaddpd			%ymm14, %ymm7, %ymm14
	vmovapd			%ymm12, 96(%r10)
	vmovapd			%ymm14, 96(%r10, %r11, 1)

	// L

	movq	ARG1, %r10 // n0
	movq	ARG3, %r11 // VL
	movq	ARG8, %r12 // L
	movq	ARG9, %r13 // sdl
	sall	$ 5, %r13d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEBP_ADD_NN_8X4_LIB4
#else
	CALL(inner_kernel_dgebp_add_nn_8x4_lib4)
#endif

	// L final 4x4 lower triangular

	movq	ARG1, %r10 // n0
	sall	$ 5, %r10d // n0*ps*sizeof(double)
	movq	ARG3, %r11 // VL
	addq	%r10, %r11
	movq	ARG8, %r12 // L
	addq	%r10, %r12
	movq	ARG9, %r13 // sdl
	sall	$ 5, %r13d

	// 4
	vmovapd			0(%r12), %ymm12
	vmovapd			0(%r12, %r13, 1), %ymm13
	vbroadcastsd	0(%r11), %ymm14
	vmulpd			%ymm0, %ymm14, %ymm15
	vaddpd			%ymm12, %ymm15, %ymm12
	vmulpd			%ymm4, %ymm14, %ymm15
	vaddpd			%ymm13, %ymm15, %ymm13
	vbroadcastsd	8(%r11), %ymm14
	vmulpd			%ymm1, %ymm14, %ymm15
	vaddpd			%ymm12, %ymm15, %ymm12
	vmulpd			%ymm5, %ymm14, %ymm15
	vaddpd			%ymm13, %ymm15, %ymm13
	vbroadcastsd	16(%r11), %ymm14
	vmulpd			%ymm2, %ymm14, %ymm15
	vaddpd			%ymm12, %ymm15, %ymm12
	vmulpd			%ymm6, %ymm14, %ymm15
	vaddpd			%ymm13, %ymm15, %ymm13
	vbroadcastsd	24(%r11), %ymm14
	vmulpd			%ymm3, %ymm14, %ymm15
	vaddpd			%ymm12, %ymm15, %ymm12
	vmulpd			%ymm7, %ymm14, %ymm15
	vaddpd			%ymm13, %ymm15, %ymm13
	vmovapd			%ymm12, 0(%r12)
	vmovapd			%ymm13, 0(%r12, %r13, 1)
	// 3
	vmovapd			32(%r12), %ymm12
	vmovapd			32(%r12, %r13, 1), %ymm13
	vbroadcastsd	40(%r11), %ymm14
	vmulpd			%ymm1, %ymm14, %ymm15
	vaddpd			%ymm12, %ymm15, %ymm12
	vmulpd			%ymm5, %ymm14, %ymm15
	vaddpd			%ymm13, %ymm15, %ymm13
	vbroadcastsd	48(%r11), %ymm14
	vmulpd			%ymm2, %ymm14, %ymm15
	vaddpd			%ymm12, %ymm15, %ymm12
	vmulpd			%ymm6, %ymm14, %ymm15
	vaddpd			%ymm13, %ymm15, %ymm13
	vbroadcastsd	56(%r11), %ymm14
	vmulpd			%ymm3, %ymm14, %ymm15
	vaddpd			%ymm12, %ymm15, %ymm12
	vmulpd			%ymm7, %ymm14, %ymm15
	vaddpd			%ymm13, %ymm15, %ymm13
	vmovapd			%ymm12, 32(%r12)
	vmovapd			%ymm13, 32(%r12, %r13, 1)
	// 2
	vmovapd			64(%r12), %ymm12
	vmovapd			64(%r12, %r13, 1), %ymm13
	vbroadcastsd	80(%r11), %ymm14
	vmulpd			%ymm2, %ymm14, %ymm15
	vaddpd			%ymm12, %ymm15, %ymm12
	vmulpd			%ymm6, %ymm14, %ymm15
	vaddpd			%ymm13, %ymm15, %ymm13
	vbroadcastsd	88(%r11), %ymm14
	vmulpd			%ymm3, %ymm14, %ymm15
	vaddpd			%ymm12, %ymm15, %ymm12
	vmulpd			%ymm7, %ymm14, %ymm15
	vaddpd			%ymm13, %ymm15, %ymm13
	vmovapd			%ymm12, 64(%r12)
	vmovapd			%ymm13, 64(%r12, %r13, 1)
	// 3
	vmovapd			96(%r12), %ymm12
	vmovapd			96(%r12, %r13, 1), %ymm13
	vbroadcastsd	120(%r11), %ymm14
	vmulpd			%ymm3, %ymm14, %ymm15
	vaddpd			%ymm12, %ymm15, %ymm12
	vmulpd			%ymm7, %ymm14, %ymm15
	vaddpd			%ymm13, %ymm15, %ymm13
	vmovapd			%ymm12, 96(%r12)
	vmovapd			%ymm13, 96(%r12, %r13, 1)

	// A

	movq	ARG2, %r10 // n1
	movq	ARG4, %r11 // VA
	movq	ARG10, %r12 // A
	movq	ARG11, %r13 // sda
	sall	$ 5, %r13d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEBP_ADD_NN_8X4_LIB4
#else
	CALL(inner_kernel_dgebp_add_nn_8x4_lib4)
#endif

	EPILOGUE

	ret

	FUN_END(kernel_dlarfb4_rn_8_lla_lib4)





//#if defined(BLAS_API)
#if ( defined(BLAS_API) | ( defined(LA_HIGH_PERFORMANCE) & defined(MF_COLMAJ) ) )

#include "kernel_dgemm_8x4_lib.S"

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
.LC00: // { -1 -1 -1 1 }
#elif defined(OS_MAC)
LC00: // { -1 -1 -1 1 }
	.align 5
#endif
	.quad	-1
	.quad	-1
	.quad	-1
	.quad	1

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC01: // { -1 -1 -1 -1 }
#elif defined(OS_MAC)
LC01: // { -1 -1 -1 -1 }
	.align 5
#endif
	.quad	-1
	.quad	-1
	.quad	-1
	.quad	-1

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC02: // { 3.5 2.5 1.5 0.5 }
#elif defined(OS_MAC)
LC02: // { 3.5 2.5 1.5 0.5 }
	.align 5
#endif
	.long	0
	.long	1071644672
	.long	0
	.long	1073217536
	.long	0
	.long	1074003968
	.long	0
	.long	1074528256

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC03: // { 7.5 6.5 5.5 4.5 }
#elif defined(OS_MAC)
LC03: // { 7.5 6.5 5.5 4.5 }
	.align 5
#endif
	.long	0
	.long	1074921472
	.long	0
	.long	1075183616
	.long	0
	.long	1075445760
	.long	0
	.long	1075707904

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC04: // { 1.0 1.0 1.0 1.0 }
#elif defined(OS_MAC)
LC04: // { 1.0 1.0 1.0 1.0 }
	.align 5
#endif
	.long	0
	.long	1072693248
	.long	0
	.long	1072693248
	.long	0
	.long	1072693248
	.long	0
	.long	1072693248

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC05: // { 1.0 1.0 1.0 -1.0 }
#elif defined(OS_MAC)
	.align 5
LC05: // { 1.0 1.0 1.0 -1.0 }
#endif
	.long	0
	.long	-1074790400
	.long	0
	.long	1072693248
	.long	0
	.long	1072693248
	.long	0
	.long	1072693248

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC06: // { 1.0 1.0 -1.0 -1.0 }
#elif defined(OS_MAC)
	.align 5
LC06: // { 1.0 1.0 -1.0 -1.0 }
#endif
	.long	0
	.long	-1074790400
	.long	0
	.long	-1074790400
	.long	0
	.long	1072693248
	.long	0
	.long	1072693248

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC07: // { 1.0 -1.0 -1.0 -1.0 }
#elif defined(OS_MAC)
	.align 5
LC07: // { 1.0 -1.0 -1.0 -1.0 }
#endif
	.long	0
	.long	-1074790400
	.long	0
	.long	-1074790400
	.long	0
	.long	-1074790400
	.long	0
	.long	1072693248

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC08: // { -1.0 -1.0 -1.0 1.0 }
#elif defined(OS_MAC)
	.align 5
LC08: // { -1.0 -1.0 -1.0 1.0 }
#endif
	.long	0
	.long	1072693248
	.long	0
	.long	-1074790400
	.long	0
	.long	-1074790400
	.long	0
	.long	-1074790400

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC09: // { -1.0 -1.0 1.0 1.0 }
#elif defined(OS_MAC)
	.align 5
LC09: // { -1.0 -1.0 1.0 1.0 }
#endif
	.long	0
	.long	1072693248
	.long	0
	.long	1072693248
	.long	0
	.long	-1074790400
	.long	0
	.long	-1074790400

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC10: // { -1.0 1.0 1.0 1.0 }
#elif defined(OS_MAC)
	.align 5
LC10: // { -1.0 1.0 1.0 1.0 }
#endif
	.long	0
	.long	1072693248
	.long	0
	.long	1072693248
	.long	0
	.long	1072693248
	.long	0
	.long	-1074790400

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC11:
#elif defined(OS_MAC)
	.align 5
LC11:
#endif
	.long	0x0
	.long	0x80000000
	.long	0x0
	.long	0x80000000
	.long	0x0
	.long	0x80000000
	.long	0x0
	.long	0x80000000
	.long	0x0
	.long	0x80000000

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC12:
#elif defined(OS_MAC)
	.align 5
LC12:
#endif
	.double	-1.0
	.double	-1.0
	.double	-1.0
	.double	-1.0




#if defined(OS_LINUX)
	.section	.note.GNU-stack,"",@progbits
#elif defined(OS_MAC)
	.subsections_via_symbols
#endif

