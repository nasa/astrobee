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
	movq	%r15, 40(%rsp);
#define EPILOGUE \
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
	xorpd	%xmm0, %xmm0; \
	movapd	%xmm0, %xmm1; \
	movapd	%xmm0, %xmm2; \
	movapd	%xmm0, %xmm3; \
	movapd	%xmm0, %xmm4; \
	movapd	%xmm0, %xmm5; \
	movapd	%xmm0, %xmm6; \
	movapd	%xmm0, %xmm7
#define NEG_ACC \
	movapd		.LC11(%rip), %xmm15; \
	xorpd		%xmm15, %xmm0; \
	xorpd		%xmm15, %xmm1; \
	xorpd		%xmm15, %xmm2; \
	xorpd		%xmm15, %xmm3; \
	xorpd		%xmm15, %xmm4; \
	xorpd		%xmm15, %xmm5; \
	xorpd		%xmm15, %xmm6; \
	xorpd		%xmm15, %xmm7

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
	xorpd	%xmm0, %xmm0; \
	movapd	%xmm0, %xmm1; \
	movapd	%xmm0, %xmm2; \
	movapd	%xmm0, %xmm3; \
	movapd	%xmm0, %xmm4; \
	movapd	%xmm0, %xmm5; \
	movapd	%xmm0, %xmm6; \
	movapd	%xmm0, %xmm7
#define NEG_ACC \
	movapd		LC11(%rip), %xmm15; \
	xorpd		%xmm15, %xmm0; \
	xorpd		%xmm15, %xmm1; \
	xorpd		%xmm15, %xmm2; \
	xorpd		%xmm15, %xmm3; \
	xorpd		%xmm15, %xmm4; \
	xorpd		%xmm15, %xmm5; \
	xorpd		%xmm15, %xmm6; \
	xorpd		%xmm15, %xmm7

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
	movups	%xmm6, 64(%rsp); \
	movups	%xmm7, 80(%rsp); \
	movups	%xmm8, 96(%rsp); \
	movups	%xmm9, 112(%rsp); \
	movups	%xmm10, 128(%rsp); \
	movups	%xmm11, 144(%rsp); \
	movups	%xmm12, 160(%rsp); \
	movups	%xmm13, 176(%rsp); \
	movups	%xmm14, 192(%rsp); \
	movups	%xmm15, 208(%rsp);
#define EPILOGUE \
	movq	  (%rsp), %rbx; \
	movq	 8(%rsp), %rbp; \
	movq	16(%rsp), %r12; \
	movq	24(%rsp), %r13; \
	movq	32(%rsp), %r14; \
	movq	40(%rsp), %r15; \
	movq	48(%rsp), %rdi; \
	movq	56(%rsp), %rsi; \
	movups	64(%rsp), %xmm6; \
	movups	80(%rsp), %xmm7; \
	movups	96(%rsp), %xmm8; \
	movups	112(%rsp), %xmm9; \
	movups	128(%rsp), %xmm10; \
	movups	144(%rsp), %xmm11; \
	movups	160(%rsp), %xmm12; \
	movups	176(%rsp), %xmm13; \
	movups	192(%rsp), %xmm14; \
	movups	208(%rsp), %xmm15; \
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
	xorpd	%xmm0, %xmm0; \
	movapd	%xmm0, %xmm1; \
	movapd	%xmm0, %xmm2; \
	movapd	%xmm0, %xmm3; \
	movapd	%xmm0, %xmm4; \
	movapd	%xmm0, %xmm5; \
	movapd	%xmm0, %xmm6; \
	movapd	%xmm0, %xmm7
#define NEG_ACC \
	movapd		.LC11(%rip), %xmm15; \
	xorpd		%xmm15, %xmm0; \
	xorpd		%xmm15, %xmm1; \
	xorpd		%xmm15, %xmm2; \
	xorpd		%xmm15, %xmm3; \
	xorpd		%xmm15, %xmm4; \
	xorpd		%xmm15, %xmm5; \
	xorpd		%xmm15, %xmm6; \
	xorpd		%xmm15, %xmm7

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
// r10d   <- k
// r11   <- A
// r12   <- B
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

//
// output arguments:
// r10d  <- 0
// r11   <- A+4*k*sizeof(double)
// r12   <- B+4*k*sizeof(double)
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_4x4_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	// prefetch
	movapd		0(%r11), %xmm8 // A[0]
	movapd 		16(%r11), %xmm9 // A[2]
	movapd		0(%r12), %xmm10 // B[0]

	xorpd		%xmm11, %xmm11
	movapd		%xmm11, %xmm12
	movapd		%xmm11, %xmm13
	movapd		%xmm11, %xmm14
	movapd		%xmm11, %xmm15


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	addpd		%xmm14, %xmm3
	movapd		16(%r12), %xmm14 // B[2]
	addpd		%xmm11, %xmm7
	movapd		%xmm10, %xmm11
	pshufd		$ 0x4e, %xmm10, %xmm15
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11

	addpd		%xmm12, %xmm2
	addpd		%xmm13, %xmm6
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13

	addpd		%xmm10, %xmm1
	movapd		32(%r12), %xmm10 // B[4]
	addpd		%xmm11, %xmm5
	movapd		%xmm14, %xmm11
	pshufd		$ 0x4e, %xmm14, %xmm12
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11

	addpd		%xmm15, %xmm0
	addpd		%xmm13, %xmm4
	movapd		%xmm12, %xmm13
	mulpd		%xmm8, %xmm12
	movapd 		32(%r11), %xmm8 // A[4]
	mulpd		%xmm9, %xmm13
	movapd 		48(%r11), %xmm9 // A[6]


	// unroll 1
	addpd		%xmm14, %xmm3
	movapd		48(%r12), %xmm14 // B[6]
	addpd		%xmm11, %xmm7
	movapd		%xmm10, %xmm11
	pshufd		$ 0x4e, %xmm10, %xmm15
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11

	addpd		%xmm12, %xmm2
	addpd		%xmm13, %xmm6
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13

	addpd		%xmm10, %xmm1
	movapd		64(%r12), %xmm10 // B[8]
	addpd		%xmm11, %xmm5
	movapd		%xmm14, %xmm11
	pshufd		$ 0x4e, %xmm14, %xmm12
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11

	addpd		%xmm15, %xmm0
	addpd		%xmm13, %xmm4
	movapd		%xmm12, %xmm13
	mulpd		%xmm8, %xmm12
	movapd 		64(%r11), %xmm8 // A[8]
	mulpd		%xmm9, %xmm13
	movapd 		80(%r11), %xmm9 // A[10]


	// unroll 2
	addpd		%xmm14, %xmm3
	movapd		80(%r12), %xmm14 // B[10]
	addpd		%xmm11, %xmm7
	movapd		%xmm10, %xmm11
	pshufd		$ 0x4e, %xmm10, %xmm15
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	subl		$ 4, %r10d

	addpd		%xmm12, %xmm2
	addpd		%xmm13, %xmm6
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13

	addpd		%xmm10, %xmm1
	movapd		96(%r12), %xmm10 // B[12]
	addpd		%xmm11, %xmm5
	movapd		%xmm14, %xmm11
	pshufd		$ 0x4e, %xmm14, %xmm12
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11

	addpd		%xmm15, %xmm0
	addpd		%xmm13, %xmm4
	movapd		%xmm12, %xmm13
	mulpd		%xmm8, %xmm12
	movapd 		96(%r11), %xmm8 // A[12]
	mulpd		%xmm9, %xmm13
	movapd 		112(%r11), %xmm9 // A[14]
	

	// unroll 3
	addpd		%xmm14, %xmm3
	movapd		112(%r12), %xmm14 // B[14]
	addpd		%xmm11, %xmm7
	movapd		%xmm10, %xmm11
	pshufd		$ 0x4e, %xmm10, %xmm15
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addq		$ 128, %r12 // B += 16

	addpd		%xmm12, %xmm2
	addpd		%xmm13, %xmm6
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13
	addq		$ 128, %r11 // A += 16

	addpd		%xmm10, %xmm1
	movapd		0(%r12), %xmm10 // B[0]
	addpd		%xmm11, %xmm5
	movapd		%xmm14, %xmm11
	pshufd		$ 0x4e, %xmm14, %xmm12
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11
	cmpl		$ 4, %r10d

	addpd		%xmm15, %xmm0
	addpd		%xmm13, %xmm4
	movapd		%xmm12, %xmm13
	mulpd		%xmm8, %xmm12
	movapd 		0(%r11), %xmm8 // A[0]
	mulpd		%xmm9, %xmm13
	movapd 		16(%r11), %xmm9 // A[2]


	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1


	// unroll 0
	addpd		%xmm14, %xmm3
	movapd		16(%r12), %xmm14 // B[2]
	addpd		%xmm11, %xmm7
	movapd		%xmm10, %xmm11
	pshufd		$ 0x4e, %xmm10, %xmm15
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11

	addpd		%xmm12, %xmm2
	addpd		%xmm13, %xmm6
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13

	addpd		%xmm10, %xmm1
	movapd		32(%r12), %xmm10 // B[4]
	addpd		%xmm11, %xmm5
	movapd		%xmm14, %xmm11
	pshufd		$ 0x4e, %xmm14, %xmm12
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11

	addpd		%xmm15, %xmm0
	addpd		%xmm13, %xmm4
	movapd		%xmm12, %xmm13
	mulpd		%xmm8, %xmm12
	movapd 		32(%r11), %xmm8 // A[4]
	mulpd		%xmm9, %xmm13
	movapd 		48(%r11), %xmm9 // A[6]


	// unroll 1
	addpd		%xmm14, %xmm3
	movapd		48(%r12), %xmm14 // B[6]
	addpd		%xmm11, %xmm7
	movapd		%xmm10, %xmm11
	pshufd		$ 0x4e, %xmm10, %xmm15
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11

	addpd		%xmm12, %xmm2
	addpd		%xmm13, %xmm6
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13

	addpd		%xmm10, %xmm1
	movapd		64(%r12), %xmm10 // B[8]
	addpd		%xmm11, %xmm5
	movapd		%xmm14, %xmm11
	pshufd		$ 0x4e, %xmm14, %xmm12
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11

	addpd		%xmm15, %xmm0
	addpd		%xmm13, %xmm4
	movapd		%xmm12, %xmm13
	mulpd		%xmm8, %xmm12
	movapd 		64(%r11), %xmm8 // A[8]
	mulpd		%xmm9, %xmm13
	movapd 		80(%r11), %xmm9 // A[10]


	// unroll 2
	addpd		%xmm14, %xmm3
	movapd		80(%r12), %xmm14 // B[10]
	addpd		%xmm11, %xmm7
	movapd		%xmm10, %xmm11
	pshufd		$ 0x4e, %xmm10, %xmm15
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	subl		$ 4, %r10d

	addpd		%xmm12, %xmm2
	addpd		%xmm13, %xmm6
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13

	addpd		%xmm10, %xmm1
	movapd		96(%r12), %xmm10 // B[12]
	addpd		%xmm11, %xmm5
	movapd		%xmm14, %xmm11
	pshufd		$ 0x4e, %xmm14, %xmm12
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11

	addpd		%xmm15, %xmm0
	addpd		%xmm13, %xmm4
	movapd		%xmm12, %xmm13
	mulpd		%xmm8, %xmm12
	movapd 		96(%r11), %xmm8 // A[12]
	mulpd		%xmm9, %xmm13
	movapd 		112(%r11), %xmm9 // A[14]
	

	// unroll 3
	addpd		%xmm14, %xmm3
	movapd		112(%r12), %xmm14 // B[14]
	addpd		%xmm11, %xmm7
	movapd		%xmm10, %xmm11
	pshufd		$ 0x4e, %xmm10, %xmm15
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addq		$ 128, %r12 // B += 16

	addpd		%xmm12, %xmm2
	addpd		%xmm13, %xmm6
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13
	addq		$ 128, %r11 // A += 16

	addpd		%xmm10, %xmm1
//	movapd		0(%r12), %xmm10 // B[0]
	addpd		%xmm11, %xmm5
	movapd		%xmm14, %xmm11
	pshufd		$ 0x4e, %xmm14, %xmm12
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11
//	cmpl		$ 4, %r10d

	addpd		%xmm15, %xmm0
	addpd		%xmm13, %xmm4
	movapd		%xmm12, %xmm13
	mulpd		%xmm8, %xmm12
//	movapd 		0(%r11), %xmm8 // A[0]
	mulpd		%xmm9, %xmm13
//	movapd 		16(%r11), %xmm9 // A[2]


	// clean accumulators
	addpd		%xmm14, %xmm3
	addpd		%xmm11, %xmm7
	addpd		%xmm12, %xmm2
	addpd		%xmm13, %xmm6


	jmp		2f


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	

	// unroll 0
	movapd		0(%r11), %xmm8 // A[0]
	movapd 		16(%r11), %xmm9 // A[2]
	movapd		0(%r12), %xmm10 // B[0]
	addpd		%xmm14, %xmm3
	movapd		16(%r12), %xmm14 // B[2]
	addpd		%xmm11, %xmm7
	movapd		%xmm10, %xmm11
	pshufd		$ 0x4e, %xmm10, %xmm15
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	subl	$ 1, %r10d

	addpd		%xmm12, %xmm2
	addpd		%xmm13, %xmm6
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13
	addq	$ 32, %r12

	addpd		%xmm10, %xmm1
	addpd		%xmm11, %xmm5
	movapd		%xmm14, %xmm11
	pshufd		$ 0x4e, %xmm14, %xmm12
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11
	addq	$ 32, %r11

	addpd		%xmm15, %xmm0
	addpd		%xmm13, %xmm4
	movapd		%xmm12, %xmm13
	mulpd		%xmm8, %xmm12
	mulpd		%xmm9, %xmm13

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


	// clean accumulators
	addpd		%xmm14, %xmm3
	addpd		%xmm11, %xmm7
	addpd		%xmm12, %xmm2
	addpd		%xmm13, %xmm6


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_4x4_lib4)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d   <- k
// r11   <- A
// r12   <- B
// r13   <- 4*sdb*sizeof(double)
// xmm0  <- [d00 d10]
// xmm1  <- [d01 d11]
// xmm2  <- [d02 d12]
// xmm3  <- [d03 d13]
// xmm4  <- [d20 d30]
// xmm5  <- [d21 d31]
// xmm6  <- [d22 d32]
// xmm7  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

//
// output arguments:
// r10d  <- 0
// r11   <- A+4*k*sizeof(double)
// r12   <- B+(k/4)*sdb*sizeof(double)+(k%4)
// r13   <- 4*sdb*sizeof(double)
// xmm0  <- [d00 d10]
// xmm1  <- [d01 d11]
// xmm2  <- [d02 d12]
// xmm3  <- [d03 d13]
// xmm4  <- [d20 d30]
// xmm5  <- [d21 d31]
// xmm6  <- [d22 d32]
// xmm7  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_4x4_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	// prefetch
	movapd		0(%r11), %xmm8 // A[0]
	movapd 		16(%r11), %xmm9 // A[2]

	xorpd		%xmm11, %xmm11
	movapd		%xmm11, %xmm12
	movapd		%xmm11, %xmm13
	movapd		%xmm11, %xmm14
	movapd		%xmm11, %xmm15


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	prefetcht0	0(%r12, %r13, 2) // software prefetch
	prefetcht0	64(%r12, %r13, 2) // software prefetch

	// unroll 0
	movddup		0(%r12), %xmm10 // B[0]
	addpd		%xmm14, %xmm2
	addpd		%xmm11, %xmm6
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11

	movddup		32(%r12), %xmm15 // B[4]
	addpd		%xmm12, %xmm3
	addpd		%xmm13, %xmm7
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13

	movddup		64(%r12), %xmm14 // B[8]
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4
	movapd		%xmm14, %xmm11
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11

	movddup		96(%r12), %xmm12 // B[12]
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5
	movapd		%xmm12, %xmm13
	mulpd		%xmm8, %xmm12
	movapd 		32(%r11), %xmm8 // A[4]
	mulpd		%xmm9, %xmm13
	movapd 		48(%r11), %xmm9 // A[6]


	// unroll 1
	movddup		8(%r12), %xmm10 // B[1]
	addpd		%xmm14, %xmm2
	addpd		%xmm11, %xmm6
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11

	movddup		40(%r12), %xmm15 // B[5]
	addpd		%xmm12, %xmm3
	addpd		%xmm13, %xmm7
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13

	movddup		72(%r12), %xmm14 // B[9]
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4
	movapd		%xmm14, %xmm11
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11

	movddup		104(%r12), %xmm12 // B[13]
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5
	movapd		%xmm12, %xmm13
	mulpd		%xmm8, %xmm12
	movapd 		64(%r11), %xmm8 // A[8]
	mulpd		%xmm9, %xmm13
	movapd 		80(%r11), %xmm9 // A[10]


	// unroll 2
	movddup		16(%r12), %xmm10 // B[2]
	addpd		%xmm14, %xmm2
	addpd		%xmm11, %xmm6
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	subl		$ 4, %r10d

	movddup		48(%r12), %xmm15 // B[6]
	addpd		%xmm12, %xmm3
	addpd		%xmm13, %xmm7
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13

	movddup		80(%r12), %xmm14 // B[10]
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4
	movapd		%xmm14, %xmm11
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11

	movddup		112(%r12), %xmm12 // B[14]
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5
	movapd		%xmm12, %xmm13
	mulpd		%xmm8, %xmm12
	movapd 		96(%r11), %xmm8 // A[12]
	mulpd		%xmm9, %xmm13
	movapd 		112(%r11), %xmm9 // A[14]
	

	// unroll 3
	movddup		24(%r12), %xmm10 // B[3]
	addpd		%xmm14, %xmm2
	addpd		%xmm11, %xmm6
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11

	movddup		56(%r12), %xmm15 // B[7]
	addpd		%xmm12, %xmm3
	addpd		%xmm13, %xmm7
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13
	addq		$ 128, %r11 // A += 16

	movddup		88(%r12), %xmm14 // B[11]
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4
	movapd		%xmm14, %xmm11
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11

	movddup		120(%r12), %xmm12 // B[15]
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5
	movapd		%xmm12, %xmm13
	mulpd		%xmm8, %xmm12
	movapd 		0(%r11), %xmm8 // A[0]
	mulpd		%xmm9, %xmm13
	movapd 		16(%r11), %xmm9 // A[2]
	addq		%r13, %r12 // B += ...


	cmpl		$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1


	// unroll 0
	movddup		0(%r12), %xmm10 // B[0]
	addpd		%xmm14, %xmm2
	addpd		%xmm11, %xmm6
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11

	movddup		32(%r12), %xmm15 // B[4]
	addpd		%xmm12, %xmm3
	addpd		%xmm13, %xmm7
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13

	movddup		64(%r12), %xmm14 // B[8]
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4
	movapd		%xmm14, %xmm11
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11

	movddup		96(%r12), %xmm12 // B[12]
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5
	movapd		%xmm12, %xmm13
	mulpd		%xmm8, %xmm12
	movapd 		32(%r11), %xmm8 // A[4]
	mulpd		%xmm9, %xmm13
	movapd 		48(%r11), %xmm9 // A[6]


	// unroll 1
	movddup		8(%r12), %xmm10 // B[1]
	addpd		%xmm14, %xmm2
	addpd		%xmm11, %xmm6
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11

	movddup		40(%r12), %xmm15 // B[5]
	addpd		%xmm12, %xmm3
	addpd		%xmm13, %xmm7
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13

	movddup		72(%r12), %xmm14 // B[9]
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4
	movapd		%xmm14, %xmm11
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11

	movddup		104(%r12), %xmm12 // B[13]
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5
	movapd		%xmm12, %xmm13
	mulpd		%xmm8, %xmm12
	movapd 		64(%r11), %xmm8 // A[8]
	mulpd		%xmm9, %xmm13
	movapd 		80(%r11), %xmm9 // A[10]


	// unroll 2
	movddup		16(%r12), %xmm10 // B[2]
	addpd		%xmm14, %xmm2
	addpd		%xmm11, %xmm6
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	subl		$ 4, %r10d

	movddup		48(%r12), %xmm15 // B[6]
	addpd		%xmm12, %xmm3
	addpd		%xmm13, %xmm7
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13

	movddup		80(%r12), %xmm14 // B[10]
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4
	movapd		%xmm14, %xmm11
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11

	movddup		112(%r12), %xmm12 // B[14]
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5
	movapd		%xmm12, %xmm13
	mulpd		%xmm8, %xmm12
	movapd 		96(%r11), %xmm8 // A[12]
	mulpd		%xmm9, %xmm13
	movapd 		112(%r11), %xmm9 // A[14]
	

	// unroll 3
	movddup		24(%r12), %xmm10 // B[3]
	addpd		%xmm14, %xmm2
	addpd		%xmm11, %xmm6
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11

	movddup		56(%r12), %xmm15 // B[7]
	addpd		%xmm12, %xmm3
	addpd		%xmm13, %xmm7
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13
	addq		$ 128, %r11 // A += 16

	movddup		88(%r12), %xmm14 // B[11]
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4
	movapd		%xmm14, %xmm11
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11

	movddup		120(%r12), %xmm12 // B[15]
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5
	movapd		%xmm12, %xmm13
	mulpd		%xmm8, %xmm12
//	movapd 		0(%r11), %xmm8 // A[0]
	mulpd		%xmm9, %xmm13
//	movapd 		16(%r11), %xmm9 // A[2]
	addq		%r13, %r12 // B += ...


	// clean accumulators
	addpd		%xmm14, %xmm2
	addpd		%xmm11, %xmm6
	addpd		%xmm12, %xmm3
	addpd		%xmm13, %xmm7


	jmp		2f


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	

	// unroll 0
	movapd 		0(%r11), %xmm8 // A[0]
	movapd 		16(%r11), %xmm9 // A[2]

	movddup		0(%r12), %xmm10 // B[0]
	addpd		%xmm14, %xmm2
	addpd		%xmm11, %xmm6
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	subl	$ 1, %r10d

	movddup		32(%r12), %xmm15 // B[4]
	addpd		%xmm12, %xmm3
	addpd		%xmm13, %xmm7
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13

	movddup		64(%r12), %xmm14 // B[8]
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4
	movapd		%xmm14, %xmm11
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11
	addq	$ 32, %r11

	movddup		96(%r12), %xmm12 // B[12]
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5
	movapd		%xmm12, %xmm13
	mulpd		%xmm8, %xmm12
	mulpd		%xmm9, %xmm13
	addq	$ 8, %r12

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


	// clean accumulators
	addpd		%xmm14, %xmm2
	addpd		%xmm11, %xmm6
	addpd		%xmm12, %xmm3
	addpd		%xmm13, %xmm7


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_4x4_lib4)
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
// xmm0  <- [d00 d10]
// xmm1  <- [d01 d11]
// xmm2  <- [d02 d12]
// xmm3  <- [d03 d13]
// xmm4  <- [d20 d30]
// xmm5  <- [d21 d31]
// xmm6  <- [d22 d32]
// xmm7  <- [d23 d33]
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
// xmm0  <- [d00 d10]
// xmm1  <- [d01 d11]
// xmm2  <- [d02 d12]
// xmm3  <- [d03 d13]
// xmm4  <- [d20 d30]
// xmm5  <- [d21 d31]
// xmm6  <- [d22 d32]
// xmm7  <- [d23 d33]
// ymm8  <- dirty
// ymm12 <- dirty
// ymm15 <- dirty


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DGEMM_NN_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dgemm_nn_4x4_lib4)
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
	movapd 		0(%r11), %xmm8 // A[0]
	movapd 		16(%r11), %xmm9 // A[2]

	movddup		0(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	movddup		32(%r12), %xmm15 // B[4]
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5

	movddup		64(%r12), %xmm14 // B[8]
	movapd		%xmm14, %xmm11
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11
	addpd		%xmm14, %xmm2
	addpd		%xmm11, %xmm6

	movddup		96(%r12), %xmm12 // B[12]
	movapd		%xmm12, %xmm13
	mulpd		%xmm8, %xmm12
	mulpd		%xmm9, %xmm13
	addpd		%xmm12, %xmm3
	addpd		%xmm13, %xmm7

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

	FUN_END(inner_edge_dgemm_nn_4x4_lib4)
#endif





// common inner routine with file scope
//
// edge for B lower triangular
//
// input arguments:
// r10   <- k
// r11   <- A
// r12   <- B
// r13   <- bs*sdb*sizeof(double)
// r14   <- offB
// xmm0  <- [d00 d10]
// xmm1  <- [d01 d11]
// xmm2  <- [d02 d12]
// xmm3  <- [d03 d13]
// xmm4  <- [d20 d30]
// xmm5  <- [d21 d31]
// xmm6  <- [d22 d32]
// xmm7  <- [d23 d33]
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
// xmm0  <- [d00 d10]
// xmm1  <- [d01 d11]
// xmm2  <- [d02 d12]
// xmm3  <- [d03 d13]
// xmm4  <- [d20 d30]
// xmm5  <- [d21 d31]
// xmm6  <- [d22 d32]
// xmm7  <- [d23 d33]
// ymm8  <- dirty
// ymm12 <- dirty
// ymm15 <- dirty


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NN_RL_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nn_rl_4x4_lib4)
#endif
	
	cmpl	$ 0, %r14d
	jg		0f

	// offB==0

	// unroll 0
	movapd 		0(%r11), %xmm8 // A[0]
	movapd 		16(%r11), %xmm9 // A[2]

	movddup		0(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	// unroll 1
	movapd 		32(%r11), %xmm8 // A[0]
	movapd 		48(%r11), %xmm9 // A[2]

	movddup		8(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	movddup		40(%r12), %xmm15 // B[4]
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5

	// unroll 2
	movapd 		64(%r11), %xmm8 // A[0]
	movapd 		80(%r11), %xmm9 // A[2]

	movddup		16(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	movddup		48(%r12), %xmm15 // B[4]
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5

	movddup		80(%r12), %xmm14 // B[8]
	movapd		%xmm14, %xmm11
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11
	addpd		%xmm14, %xmm2
	addpd		%xmm11, %xmm6

	// unroll 3
	movapd 		96(%r11), %xmm8 // A[0]
	movapd 		112(%r11), %xmm9 // A[2]

	movddup		24(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	movddup		56(%r12), %xmm15 // B[4]
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5

	movddup		88(%r12), %xmm14 // B[8]
	movapd		%xmm14, %xmm11
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11
	addpd		%xmm14, %xmm2
	addpd		%xmm11, %xmm6

	movddup		120(%r12), %xmm12 // B[12]
	movapd		%xmm12, %xmm13
	mulpd		%xmm8, %xmm12
	mulpd		%xmm9, %xmm13
	addpd		%xmm12, %xmm3
	addpd		%xmm13, %xmm7

	subl	$ 4, %r10d // k-4
	addq	$ 128, %r11 // A+4*bs*sizeof(double)
	addq	%r13, %r12 // B+bs*sdb*sizeof(double)

	jmp		3f

0:
	cmpl	$ 1, %r14d
	jg		1f

	// offB==1

	addq			$ 8, %r12 // B+1*sizeof(double)

	// unroll 0
	movapd 		0(%r11), %xmm8 // A[0]
	movapd 		16(%r11), %xmm9 // A[2]

	movddup		0(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	// unroll 1
	movapd 		32(%r11), %xmm8 // A[0]
	movapd 		48(%r11), %xmm9 // A[2]

	movddup		8(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	movddup		40(%r12), %xmm15 // B[4]
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5

	// unroll 2
	movapd 		64(%r11), %xmm8 // A[0]
	movapd 		80(%r11), %xmm9 // A[2]

	movddup		16(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	movddup		48(%r12), %xmm15 // B[4]
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5

	movddup		80(%r12), %xmm14 // B[8]
	movapd		%xmm14, %xmm11
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11
	addpd		%xmm14, %xmm2
	addpd		%xmm11, %xmm6

	subl	$ 3, %r10d // k-3
	addq	$ 96, %r11 // A+3*bs*sizeof(double)
	addq	%r13, %r12
	subq			$ 8, %r12 // B+bs*sdb*sizeof(double)-1

	jmp		3f

1:
	cmpl	$ 2, %r14d
	jg		2f

	// offB==2

	addq	$ 16, %r12 // B+2*sizeof(double)

	// unroll 0
	movapd 		0(%r11), %xmm8 // A[0]
	movapd 		16(%r11), %xmm9 // A[2]

	movddup		0(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	// unroll 1
	movapd 		32(%r11), %xmm8 // A[0]
	movapd 		48(%r11), %xmm9 // A[2]

	movddup		8(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	movddup		40(%r12), %xmm15 // B[4]
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5

	subl	$ 2, %r10d // k-2
	addq	$ 64, %r11 // A+2*bs*sizeof(double)
	addq	%r13, %r12
	subq	$ 16, %r12 // B+bs*sdb*sizeof(double)-2

	// unroll 2
	movapd 		0(%r11), %xmm8 // A[0]
	movapd 		16(%r11), %xmm9 // A[2]

	movddup		0(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	movddup		32(%r12), %xmm15 // B[4]
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5

	movddup		64(%r12), %xmm14 // B[8]
	movapd		%xmm14, %xmm11
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11
	addpd		%xmm14, %xmm2
	addpd		%xmm11, %xmm6

	// unroll 3
	movapd 		32(%r11), %xmm8 // A[0]
	movapd 		48(%r11), %xmm9 // A[2]

	movddup		8(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	movddup		40(%r12), %xmm15 // B[4]
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5

	movddup		72(%r12), %xmm14 // B[8]
	movapd		%xmm14, %xmm11
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11
	addpd		%xmm14, %xmm2
	addpd		%xmm11, %xmm6

	movddup		104(%r12), %xmm12 // B[12]
	movapd		%xmm12, %xmm13
	mulpd		%xmm8, %xmm12
	mulpd		%xmm9, %xmm13
	addpd		%xmm12, %xmm3
	addpd		%xmm13, %xmm7

	// unroll 4
	movapd 		64(%r11), %xmm8 // A[0]
	movapd 		80(%r11), %xmm9 // A[2]

	movddup		16(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	movddup		48(%r12), %xmm15 // B[4]
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5

	movddup		80(%r12), %xmm14 // B[8]
	movapd		%xmm14, %xmm11
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11
	addpd		%xmm14, %xmm2
	addpd		%xmm11, %xmm6

	movddup		112(%r12), %xmm12 // B[12]
	movapd		%xmm12, %xmm13
	mulpd		%xmm8, %xmm12
	mulpd		%xmm9, %xmm13
	addpd		%xmm12, %xmm3
	addpd		%xmm13, %xmm7

	// unroll 5
	movapd 		96(%r11), %xmm8 // A[0]
	movapd 		112(%r11), %xmm9 // A[2]

	movddup		24(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	movddup		56(%r12), %xmm15 // B[4]
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5

	movddup		88(%r12), %xmm14 // B[8]
	movapd		%xmm14, %xmm11
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11
	addpd		%xmm14, %xmm2
	addpd		%xmm11, %xmm6

	movddup		120(%r12), %xmm12 // B[12]
	movapd		%xmm12, %xmm13
	mulpd		%xmm8, %xmm12
	mulpd		%xmm9, %xmm13
	addpd		%xmm12, %xmm3
	addpd		%xmm13, %xmm7

	subl	$ 4, %r10d // k-4
	addq	$ 128, %r11 // A+4*bs*sizeof(double)
	addq	%r13, %r12 // B+bs*sdb*sizeof(double)

	jmp		3f

2:
	// offB==3

	addq	$ 24, %r12 // B+3*sizeof(double)

	// unroll 0
	movapd 		0(%r11), %xmm8 // A[0]
	movapd 		16(%r11), %xmm9 // A[2]

	movddup		0(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	subl	$ 1, %r10d // k-1
	addq	$ 32, %r11 // A+1*bs*sizeof(double)
	addq	%r13, %r12
	subq	$ 24, %r12 // B+bs*sdb*sizeof(double)-3

	// unroll 1
	movapd 		0(%r11), %xmm8 // A[0]
	movapd 		16(%r11), %xmm9 // A[2]

	movddup		0(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	movddup		32(%r12), %xmm15 // B[4]
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5

	// unroll 2
	movapd 		32(%r11), %xmm8 // A[0]
	movapd 		48(%r11), %xmm9 // A[2]

	movddup		8(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	movddup		40(%r12), %xmm15 // B[4]
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5

	movddup		72(%r12), %xmm14 // B[8]
	movapd		%xmm14, %xmm11
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11
	addpd		%xmm14, %xmm2
	addpd		%xmm11, %xmm6

	// unroll 3
	movapd 		64(%r11), %xmm8 // A[0]
	movapd 		80(%r11), %xmm9 // A[2]

	movddup		16(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	movddup		48(%r12), %xmm15 // B[4]
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5

	movddup		80(%r12), %xmm14 // B[8]
	movapd		%xmm14, %xmm11
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11
	addpd		%xmm14, %xmm2
	addpd		%xmm11, %xmm6

	movddup		112(%r12), %xmm12 // B[12]
	movapd		%xmm12, %xmm13
	mulpd		%xmm8, %xmm12
	mulpd		%xmm9, %xmm13
	addpd		%xmm12, %xmm3
	addpd		%xmm13, %xmm7

	// unroll 4
	movapd 		96(%r11), %xmm8 // A[0]
	movapd 		112(%r11), %xmm9 // A[2]

	movddup		24(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	movddup		56(%r12), %xmm15 // B[4]
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5

	movddup		88(%r12), %xmm14 // B[8]
	movapd		%xmm14, %xmm11
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11
	addpd		%xmm14, %xmm2
	addpd		%xmm11, %xmm6

	movddup		120(%r12), %xmm12 // B[12]
	movapd		%xmm12, %xmm13
	mulpd		%xmm8, %xmm12
	mulpd		%xmm9, %xmm13
	addpd		%xmm12, %xmm3
	addpd		%xmm13, %xmm7

	subl	$ 4, %r10d // k-4
	addq	$ 128, %r11 // A+4*bs*sizeof(double)
	addq	%r13, %r12 // B+bs*sdb*sizeof(double)

3:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nn_rl_4x4_lib4)
#endif





// common inner routine with file scope
//
// edge for B lower triangular
//
// input arguments:
// r10   <- k
// r11   <- A
// r12   <- B
// r13   <- bs*sdb*sizeof(double)
// r14   <- offB

//
// output arguments:
// r10   <- k-(4-offB)
// r11   <- A+(4-offB)*bs*sizeof(double)
// r12   <- B-offB+bs*sdb*sizeof(double)
// r13   <- bs*sdb*sizeof(double)
// r14   <- offB


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NN_RL_4X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nn_rl_4x4_vs_lib4)
#endif
	
	cmpl			$ 0, %r10d
	jle				3f // end

	cmpl			$ 0, %r14d
	jg				0f // offB>0

	// offB==0

	movapd 		0(%r11), %xmm8 // A[0]
	movapd 		16(%r11), %xmm9 // A[2]

	movddup		0(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A+1*bs*sizeof(double)
	addq			$ 8, %r12 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	movapd 		0(%r11), %xmm8 // A[0]
	movapd 		16(%r11), %xmm9 // A[2]

	movddup		0(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	movddup		32(%r12), %xmm15 // B[4]
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A+1*bs*sizeof(double)
	addq			$ 8, %r12 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	movapd 		0(%r11), %xmm8 // A[0]
	movapd 		16(%r11), %xmm9 // A[2]

	movddup		0(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	movddup		32(%r12), %xmm15 // B[4]
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5

	movddup		64(%r12), %xmm14 // B[8]
	movapd		%xmm14, %xmm11
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11
	addpd		%xmm14, %xmm2
	addpd		%xmm11, %xmm6

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A+1*bs*sizeof(double)
	addq			$ 8, %r12 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	movapd 		0(%r11), %xmm8 // A[0]
	movapd 		16(%r11), %xmm9 // A[2]

	movddup		0(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	movddup		32(%r12), %xmm15 // B[4]
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5

	movddup		64(%r12), %xmm14 // B[8]
	movapd		%xmm14, %xmm11
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11
	addpd		%xmm14, %xmm2
	addpd		%xmm11, %xmm6

	movddup		96(%r12), %xmm12 // B[12]
	movapd		%xmm12, %xmm13
	mulpd		%xmm8, %xmm12
	mulpd		%xmm9, %xmm13
	addpd		%xmm12, %xmm3
	addpd		%xmm13, %xmm7

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A+1*bs*sizeof(double)
	addq			%r13, %r12
	subq			$ 24, %r12 // B+bs*sdb*sizeof(double)-(bs-1)*sizeof(double)

	jmp				3f // end

0:
	cmpl			$ 1, %r14d
	jg				1f // offB>1

	// offB==1

	addq			$ 8, %r12 // B+1*sizeof(double)

	movapd 		0(%r11), %xmm8 // A[0]
	movapd 		16(%r11), %xmm9 // A[2]

	movddup		0(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A+1*bs*sizeof(double)
	addq			$ 8, %r12 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	movapd 		0(%r11), %xmm8 // A[0]
	movapd 		16(%r11), %xmm9 // A[2]

	movddup		0(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	movddup		32(%r12), %xmm15 // B[4]
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A+1*bs*sizeof(double)
	addq			$ 8, %r12 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	movapd 		0(%r11), %xmm8 // A[0]
	movapd 		16(%r11), %xmm9 // A[2]

	movddup		0(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	movddup		32(%r12), %xmm15 // B[4]
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5

	movddup		64(%r12), %xmm14 // B[8]
	movapd		%xmm14, %xmm11
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11
	addpd		%xmm14, %xmm2
	addpd		%xmm11, %xmm6

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A+1*bs*sizeof(double)
	addq			%r13, %r12
	subq			$ 24, %r12 // B+bs*sdb*sizeof(double)-(bs-1)*sizeof(double)

	jmp				3f // end

1:
	cmpl			$ 2, %r14d
	jg				2f // offB>2

	// offB==2

	addq			$ 16, %r12 // B+2*sizeof(double)

	movapd 		0(%r11), %xmm8 // A[0]
	movapd 		16(%r11), %xmm9 // A[2]

	movddup		0(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A+1*bs*sizeof(double)
	addq			$ 8, %r12 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	movapd 		0(%r11), %xmm8 // A[0]
	movapd 		16(%r11), %xmm9 // A[2]

	movddup		0(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	movddup		32(%r12), %xmm15 // B[4]
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5

	subl			$ 1, %r10d // k-2
	addq			$ 32, %r11 // A+2*bs*sizeof(double)
	addq			%r13, %r12
	subq			$ 24, %r12 // B+bs*sdb*sizeof(double)-(bs-1)*sizeof(double)

	movapd 		0(%r11), %xmm8 // A[0]
	movapd 		16(%r11), %xmm9 // A[2]

	movddup		0(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	movddup		32(%r12), %xmm15 // B[4]
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5

	movddup		64(%r12), %xmm14 // B[8]
	movapd		%xmm14, %xmm11
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11
	addpd		%xmm14, %xmm2
	addpd		%xmm11, %xmm6

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A+1*bs*sizeof(double)
	addq			$ 8, %r12 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	movapd 		0(%r11), %xmm8 // A[0]
	movapd 		16(%r11), %xmm9 // A[2]

	movddup		0(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	movddup		32(%r12), %xmm15 // B[4]
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5

	movddup		64(%r12), %xmm14 // B[8]
	movapd		%xmm14, %xmm11
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11
	addpd		%xmm14, %xmm2
	addpd		%xmm11, %xmm6

	movddup		96(%r12), %xmm12 // B[12]
	movapd		%xmm12, %xmm13
	mulpd		%xmm8, %xmm12
	mulpd		%xmm9, %xmm13
	addpd		%xmm12, %xmm3
	addpd		%xmm13, %xmm7

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A+1*bs*sizeof(double)
	addq			$ 8, %r12 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	movapd 		0(%r11), %xmm8 // A[0]
	movapd 		16(%r11), %xmm9 // A[2]

	movddup		0(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	movddup		32(%r12), %xmm15 // B[4]
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5

	movddup		64(%r12), %xmm14 // B[8]
	movapd		%xmm14, %xmm11
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11
	addpd		%xmm14, %xmm2
	addpd		%xmm11, %xmm6

	movddup		96(%r12), %xmm12 // B[12]
	movapd		%xmm12, %xmm13
	mulpd		%xmm8, %xmm12
	mulpd		%xmm9, %xmm13
	addpd		%xmm12, %xmm3
	addpd		%xmm13, %xmm7

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A+1*bs*sizeof(double)
	addq			$ 8, %r12 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	movapd 		0(%r11), %xmm8 // A[0]
	movapd 		16(%r11), %xmm9 // A[2]

	movddup		0(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	movddup		32(%r12), %xmm15 // B[4]
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5

	movddup		64(%r12), %xmm14 // B[8]
	movapd		%xmm14, %xmm11
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11
	addpd		%xmm14, %xmm2
	addpd		%xmm11, %xmm6

	movddup		96(%r12), %xmm12 // B[12]
	movapd		%xmm12, %xmm13
	mulpd		%xmm8, %xmm12
	mulpd		%xmm9, %xmm13
	addpd		%xmm12, %xmm3
	addpd		%xmm13, %xmm7

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A+1*bs*sizeof(double)
	addq			%r13, %r12
	subq			$ 24, %r12 // B+bs*sdb*sizeof(double)-(bs-1)*sizeof(double)

	jmp				3f

2:
	// offB==3

	addq			$ 24, %r12 // B+3*sizeof(double)

	movapd 		0(%r11), %xmm8 // A[0]
	movapd 		16(%r11), %xmm9 // A[2]

	movddup		0(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A+1*bs*sizeof(double)
	addq			%r13, %r12
	subq			$ 24, %r12 // B+bs*sdb*sizeof(double)-(bs-1)*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	movapd 		0(%r11), %xmm8 // A[0]
	movapd 		16(%r11), %xmm9 // A[2]

	movddup		0(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	movddup		32(%r12), %xmm15 // B[4]
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A+1*bs*sizeof(double)
	addq			$ 8, %r12 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	movapd 		0(%r11), %xmm8 // A[0]
	movapd 		16(%r11), %xmm9 // A[2]

	movddup		0(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	movddup		32(%r12), %xmm15 // B[4]
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5

	movddup		64(%r12), %xmm14 // B[8]
	movapd		%xmm14, %xmm11
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11
	addpd		%xmm14, %xmm2
	addpd		%xmm11, %xmm6

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A+1*bs*sizeof(double)
	addq			$ 8, %r12 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	movapd 		0(%r11), %xmm8 // A[0]
	movapd 		16(%r11), %xmm9 // A[2]

	movddup		0(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	movddup		32(%r12), %xmm15 // B[4]
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5

	movddup		64(%r12), %xmm14 // B[8]
	movapd		%xmm14, %xmm11
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11
	addpd		%xmm14, %xmm2
	addpd		%xmm11, %xmm6

	movddup		96(%r12), %xmm12 // B[12]
	movapd		%xmm12, %xmm13
	mulpd		%xmm8, %xmm12
	mulpd		%xmm9, %xmm13
	addpd		%xmm12, %xmm3
	addpd		%xmm13, %xmm7

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A+1*bs*sizeof(double)
	addq			$ 8, %r12 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	movapd 		0(%r11), %xmm8 // A[0]
	movapd 		16(%r11), %xmm9 // A[2]

	movddup		0(%r12), %xmm10 // B[0]
	movapd		%xmm10, %xmm11
	mulpd		%xmm8, %xmm10
	mulpd		%xmm9, %xmm11
	addpd		%xmm10, %xmm0
	addpd		%xmm11, %xmm4

	movddup		32(%r12), %xmm15 // B[4]
	movapd		%xmm15, %xmm13
	mulpd		%xmm8, %xmm15
	mulpd		%xmm9, %xmm13
	addpd		%xmm15, %xmm1
	addpd		%xmm13, %xmm5

	movddup		64(%r12), %xmm14 // B[8]
	movapd		%xmm14, %xmm11
	mulpd		%xmm8, %xmm14
	mulpd		%xmm9, %xmm11
	addpd		%xmm14, %xmm2
	addpd		%xmm11, %xmm6

	movddup		96(%r12), %xmm12 // B[12]
	movapd		%xmm12, %xmm13
	mulpd		%xmm8, %xmm12
	mulpd		%xmm9, %xmm13
	addpd		%xmm12, %xmm3
	addpd		%xmm13, %xmm7

	subl			$ 1, %r10d // k-4
	addq			$ 32, %r11 // A+4*bs*sizeof(double)
	addq			%r13, %r12
	subq			$ 24, %r12 // B+bs*sdb*sizeof(double)-(bs-1)*sizeof(double)

3:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nn_rl_4x4_vs_lib4)
#endif





// common inner routine with file scope
//
// edge for B upper triangular
//
// input arguments:
// r10   <- A
// r11   <- B
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

//
// output arguments:
// r10   <- A+4*4*sizeof(double)
// r11   <- B+4*4*sizeof(double)
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NT_RU_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nt_ru_4x4_lib4)
#endif
	
	movapd			0(%r10), %xmm8
	movapd			16(%r10), %xmm9
	movddup			0(%r11), %xmm12
	movapd			%xmm12, %xmm13
	mulpd			%xmm8, %xmm12
	mulpd			%xmm9, %xmm13
	addpd			%xmm12, %xmm0
	addpd			%xmm13, %xmm4

	movapd			32(%r10), %xmm8
	movapd			48(%r10), %xmm9
	movddup			32(%r11), %xmm12
	movapd			%xmm12, %xmm13
	mulpd			%xmm8, %xmm12
	mulpd			%xmm9, %xmm13
	addpd			%xmm12, %xmm0
	addpd			%xmm13, %xmm4
	movddup			40(%r11), %xmm12
	movapd			%xmm12, %xmm13
	mulpd			%xmm8, %xmm12
	mulpd			%xmm9, %xmm13
	addpd			%xmm12, %xmm1
	addpd			%xmm13, %xmm5

	movapd			64(%r10), %xmm8
	movapd			80(%r10), %xmm9
	movddup			64(%r11), %xmm12
	movapd			%xmm12, %xmm13
	mulpd			%xmm8, %xmm12
	mulpd			%xmm9, %xmm13
	addpd			%xmm12, %xmm0
	addpd			%xmm13, %xmm4
	movddup			72(%r11), %xmm12
	movapd			%xmm12, %xmm13
	mulpd			%xmm8, %xmm12
	mulpd			%xmm9, %xmm13
	addpd			%xmm12, %xmm1
	addpd			%xmm13, %xmm5
	movddup			80(%r11), %xmm12
	movapd			%xmm12, %xmm13
	mulpd			%xmm8, %xmm12
	mulpd			%xmm9, %xmm13
	addpd			%xmm12, %xmm2
	addpd			%xmm13, %xmm6

	movapd			96(%r10), %xmm8
	movapd			112(%r10), %xmm9
	movddup			96(%r11), %xmm12
	movapd			%xmm12, %xmm13
	mulpd			%xmm8, %xmm12
	mulpd			%xmm9, %xmm13
	addpd			%xmm12, %xmm0
	addpd			%xmm13, %xmm4
	movddup			104(%r11), %xmm12
	movapd			%xmm12, %xmm13
	mulpd			%xmm8, %xmm12
	mulpd			%xmm9, %xmm13
	addpd			%xmm12, %xmm1
	addpd			%xmm13, %xmm5
	movddup			112(%r11), %xmm12
	movapd			%xmm12, %xmm13
	mulpd			%xmm8, %xmm12
	mulpd			%xmm9, %xmm13
	addpd			%xmm12, %xmm2
	addpd			%xmm13, %xmm6
	movddup			120(%r11), %xmm12
	movapd			%xmm12, %xmm13
	mulpd			%xmm8, %xmm12
	mulpd			%xmm9, %xmm13
	addpd			%xmm12, %xmm3
	addpd			%xmm13, %xmm7

	addq			$ 128, %r10
	addq			$ 128, %r11

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nt_ru_4x4_lib4)
#endif





// common inner routine with file scope
//
// edge for B upper triangular
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

//
// output arguments:
// r10d  <- max(k-4,0)
// r11   <- A+4*4*sizeof(double)
// r12   <- B+4*4*sizeof(double)
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NT_RU_4X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nt_ru_4x4_vs_lib4)
#endif
	
	movapd			0(%r11), %xmm8
	movapd			16(%r11), %xmm9
	subl			$ 1, %r10d
	movddup			0(%r12), %xmm12
	movapd			%xmm12, %xmm13
	mulpd			%xmm8, %xmm12
	mulpd			%xmm9, %xmm13
	addpd			%xmm12, %xmm0
	addpd			%xmm13, %xmm4
	addq			$ 32, %r11
	addq			$ 32, %r12

	cmpl	$ 0, %r10d
	jle		0f

	movapd			0(%r11), %xmm8
	movapd			16(%r11), %xmm9
	subl			$ 1, %r10d
	movddup			0(%r12), %xmm12
	movapd			%xmm12, %xmm13
	mulpd			%xmm8, %xmm12
	mulpd			%xmm9, %xmm13
	addpd			%xmm12, %xmm0
	addpd			%xmm13, %xmm4
	addq			$ 32, %r11
	movddup			8(%r12), %xmm12
	movapd			%xmm12, %xmm13
	mulpd			%xmm8, %xmm12
	mulpd			%xmm9, %xmm13
	addpd			%xmm12, %xmm1
	addpd			%xmm13, %xmm5
	addq			$ 32, %r12

	cmpl	$ 0, %r10d
	jle		0f

	movapd			0(%r11), %xmm8
	movapd			16(%r11), %xmm9
	subl			$ 1, %r10d
	movddup			0(%r12), %xmm12
	movapd			%xmm12, %xmm13
	mulpd			%xmm8, %xmm12
	mulpd			%xmm9, %xmm13
	addpd			%xmm12, %xmm0
	addpd			%xmm13, %xmm4
	movddup			8(%r12), %xmm12
	movapd			%xmm12, %xmm13
	mulpd			%xmm8, %xmm12
	mulpd			%xmm9, %xmm13
	addpd			%xmm12, %xmm1
	addpd			%xmm13, %xmm5
	addq			$ 32, %r11
	movddup			16(%r12), %xmm12
	movapd			%xmm12, %xmm13
	mulpd			%xmm8, %xmm12
	mulpd			%xmm9, %xmm13
	addpd			%xmm12, %xmm2
	addpd			%xmm13, %xmm6
	addq			$ 32, %r12

	cmpl	$ 0, %r10d
	jle		0f

	movapd			0(%r11), %xmm8
	movapd			16(%r11), %xmm9
	subl			$ 1, %r10d
	movddup			0(%r12), %xmm12
	movapd			%xmm12, %xmm13
	mulpd			%xmm8, %xmm12
	mulpd			%xmm9, %xmm13
	addpd			%xmm12, %xmm0
	addpd			%xmm13, %xmm4
	movddup			8(%r12), %xmm12
	movapd			%xmm12, %xmm13
	mulpd			%xmm8, %xmm12
	mulpd			%xmm9, %xmm13
	addpd			%xmm12, %xmm1
	addpd			%xmm13, %xmm5
	movddup			16(%r12), %xmm12
	movapd			%xmm12, %xmm13
	mulpd			%xmm8, %xmm12
	mulpd			%xmm9, %xmm13
	addpd			%xmm12, %xmm2
	addpd			%xmm13, %xmm6
	addq			$ 32, %r11
	movddup			24(%r12), %xmm12
	movapd			%xmm12, %xmm13
	mulpd			%xmm8, %xmm12
	mulpd			%xmm9, %xmm13
	addpd			%xmm12, %xmm3
	addpd			%xmm13, %xmm7
	addq			$ 32, %r12

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nt_ru_4x4_vs_lib4)
#endif





// common inner routine with file scope
//
// blend
//
// input arguments:
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

// output arguments:
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_blend_4x4_lib4)
#endif
	
	movapd	%xmm0, %xmm8
	movsd	%xmm1, %xmm0
	movsd	%xmm8, %xmm1

	movapd	%xmm2, %xmm8
	movsd	%xmm3, %xmm2
	movsd	%xmm8, %xmm3

	movapd	%xmm4, %xmm8
	movsd	%xmm5, %xmm4
	movsd	%xmm8, %xmm5

	movapd	%xmm6, %xmm8
	movsd	%xmm7, %xmm6
	movsd	%xmm8, %xmm7

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_blend_4x4_lib4)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

// output arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_4x4_lib4)
#endif
	
	// alpha
	movddup	0(%r10), %xmm15

	mulpd	%xmm15, %xmm0
	mulpd	%xmm15, %xmm1
	mulpd	%xmm15, %xmm2
	mulpd	%xmm15, %xmm3
	mulpd	%xmm15, %xmm4
	mulpd	%xmm15, %xmm5
	mulpd	%xmm15, %xmm6
	mulpd	%xmm15, %xmm7


	// beta
	movddup	0(%r11), %xmm14

	movapd		0(%r12), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm0
	movapd		16(%r12), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm4
	movapd		32(%r12), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm1
	movapd		48(%r12), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm5
	movapd		64(%r12), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm2
	movapd		80(%r12), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm6
	movapd		96(%r12), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm3
	movapd		112(%r12), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm7

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_4x4_lib4)
#endif





// common inner routine with file scope
//
// blend for generic alpha and beta
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12  <- offset
// r13   <- C
// r14  <- 4*sdc*sizeof(double)
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
//
// output arguments:
// r10   <- alpha
// r11   <- beta
// r12  <- offset
// r13   <- C
// r14  <- 4*sdc*sizeof(double)
// r15  <- n0 // col index: start from (inc)
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_4X4_GEN_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_4x4_gen_lib4)
#endif
	
	// alpha
	movddup	0(%r10), %xmm15

	mulpd	%xmm15, %xmm0
	mulpd	%xmm15, %xmm1
	mulpd	%xmm15, %xmm2
	mulpd	%xmm15, %xmm3
	mulpd	%xmm15, %xmm4
	mulpd	%xmm15, %xmm5
	mulpd	%xmm15, %xmm6
	mulpd	%xmm15, %xmm7


	// beta
	movddup	0(%r11), %xmm14

	xorpd		%xmm15, %xmm15 // 0.0

	ucomisd		%xmm14, %xmm15 // beta==0.0 ?
	je			3f // end

	cmpl	$ 0, %r12d
	jg		0f

	// offset==0

	movapd		0(%r13), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm0
	movapd		16(%r13), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm4
	movapd		32(%r13), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm1
	movapd		48(%r13), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm5
	movapd		64(%r13), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm2
	movapd		80(%r13), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm6
	movapd		96(%r13), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm3
	movapd		112(%r13), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm7


	jmp		3f

0:

	movq	%r13, %r15 // C0
	addq	%r14, %r15 // C1 <- C0 + 4*sdc*sizeof(double)

	cmpl	$ 1, %r12d
	jg		1f

	// offset==1

	movupd		8(%r13), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm0
	movlpd		24(%r13), %xmm15
	movhpd		0(%r15), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm4
	movupd		40(%r13), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm1
	movlpd		56(%r13), %xmm15
	movhpd		32(%r15), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm5
	movupd		72(%r13), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm2
	movlpd		88(%r13), %xmm15
	movhpd		64(%r15), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm6
	movupd		104(%r13), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm3
	movlpd		120(%r13), %xmm15
	movhpd		96(%r15), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm7

	jmp		3f

1:

	cmpl	$ 2, %r12d
	jg		2f

	// offset==2

	movapd		16(%r13), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm0
	movapd		0(%r15), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm4
	movapd		48(%r13), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm1
	movapd		32(%r15), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm5
	movapd		80(%r13), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm2
	movapd		64(%r15), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm6
	movapd		112(%r13), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm3
	movapd		96(%r15), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm7


	jmp		3f

2:

	// offset==3

	movlpd		24(%r13), %xmm15
	movhpd		0(%r15), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm0
	movupd		8(%r15), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm4
	movlpd		56(%r13), %xmm15
	movhpd		32(%r15), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm1
	movupd		40(%r15), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm5
	movlpd		88(%r13), %xmm15
	movhpd		64(%r15), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm2
	movupd		72(%r15), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm6
	movlpd		120(%r13), %xmm15
	movhpd		96(%r15), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm3
	movupd		104(%r15), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm7


3:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_4x4_gen_lib4)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta=0.0
//
// input arguments:
// r10   <- alpha
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

// output arguments:
// r10   <- alpha
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_A0_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_scale_a0_4x4_lib4)
#endif
	
	// alpha
	movddup	0(%r10), %xmm15

	mulpd	%xmm15, %xmm0
	mulpd	%xmm15, %xmm1
	mulpd	%xmm15, %xmm2
	mulpd	%xmm15, %xmm3
	mulpd	%xmm15, %xmm4
	mulpd	%xmm15, %xmm5
	mulpd	%xmm15, %xmm6
	mulpd	%xmm15, %xmm7

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_a0_4x4_lib4)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

// output arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_SCALE_AB_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_blend_scale_ab_4x4_lib4)
#endif
	
	movapd	%xmm0, %xmm8
	movsd	%xmm1, %xmm0
	movsd	%xmm8, %xmm1

	movapd	%xmm2, %xmm8
	movsd	%xmm3, %xmm2
	movsd	%xmm8, %xmm3

	movapd	%xmm4, %xmm8
	movsd	%xmm5, %xmm4
	movsd	%xmm8, %xmm5

	movapd	%xmm6, %xmm8
	movsd	%xmm7, %xmm6
	movsd	%xmm8, %xmm7

	// alpha
	movddup	0(%r10), %xmm15

	mulpd	%xmm15, %xmm0
	mulpd	%xmm15, %xmm1
	mulpd	%xmm15, %xmm2
	mulpd	%xmm15, %xmm3
	mulpd	%xmm15, %xmm4
	mulpd	%xmm15, %xmm5
	mulpd	%xmm15, %xmm6
	mulpd	%xmm15, %xmm7


	// beta
	movddup	0(%r11), %xmm14

	xorpd		%xmm15, %xmm15 // 0.0
	ucomisd		%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	movapd		0(%r12), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm0
	movapd		16(%r12), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm4
	movapd		32(%r12), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm1
	movapd		48(%r12), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm5
	movapd		64(%r12), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm2
	movapd		80(%r12), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm6
	movapd		96(%r12), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm3
	movapd		112(%r12), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm7

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_blend_scale_ab_4x4_lib4)
#endif





// common inner routine with file scope
//
// blend for generic alpha and beta
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12  <- offset
// r13   <- C
// r14  <- 4*sdc*sizeof(double)
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
//
// output arguments:
// r10   <- alpha
// r11   <- beta
// r12  <- offset
// r13   <- C
// r14  <- 4*sdc*sizeof(double)
// r15  <- n0 // col index: start from (inc)
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_SCALE_AB_4X4_GEN_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_blend_scale_ab_4x4_gen_lib4)
#endif
	
	movapd	%xmm0, %xmm8
	movsd	%xmm1, %xmm0
	movsd	%xmm8, %xmm1

	movapd	%xmm2, %xmm8
	movsd	%xmm3, %xmm2
	movsd	%xmm8, %xmm3

	movapd	%xmm4, %xmm8
	movsd	%xmm5, %xmm4
	movsd	%xmm8, %xmm5

	movapd	%xmm6, %xmm8
	movsd	%xmm7, %xmm6
	movsd	%xmm8, %xmm7

	// alpha
	movddup	0(%r10), %xmm15

	mulpd	%xmm15, %xmm0
	mulpd	%xmm15, %xmm1
	mulpd	%xmm15, %xmm2
	mulpd	%xmm15, %xmm3
	mulpd	%xmm15, %xmm4
	mulpd	%xmm15, %xmm5
	mulpd	%xmm15, %xmm6
	mulpd	%xmm15, %xmm7


	// beta
	movddup	0(%r11), %xmm14

	xorpd		%xmm15, %xmm15 // 0.0

	ucomisd		%xmm14, %xmm15 // beta==0.0 ?
	je			3f // end

	cmpl	$ 0, %r12d
	jg		0f

	// offset==0

	movapd		0(%r13), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm0
	movapd		16(%r13), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm4
	movapd		32(%r13), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm1
	movapd		48(%r13), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm5
	movapd		64(%r13), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm2
	movapd		80(%r13), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm6
	movapd		96(%r13), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm3
	movapd		112(%r13), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm7


	jmp		3f

0:

	movq	%r13, %r15 // C0
	addq	%r14, %r15 // C1 <- C0 + 4*sdc*sizeof(double)

	cmpl	$ 1, %r12d
	jg		1f

	// offset==1

	movupd		8(%r13), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm0
	movlpd		24(%r13), %xmm15
	movhpd		0(%r15), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm4
	movupd		40(%r13), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm1
	movlpd		56(%r13), %xmm15
	movhpd		32(%r15), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm5
	movupd		72(%r13), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm2
	movlpd		88(%r13), %xmm15
	movhpd		64(%r15), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm6
	movupd		104(%r13), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm3
	movlpd		120(%r13), %xmm15
	movhpd		96(%r15), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm7

	jmp		3f

1:

	cmpl	$ 2, %r12d
	jg		2f

	// offset==2

	movapd		16(%r13), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm0
	movapd		0(%r15), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm4
	movapd		48(%r13), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm1
	movapd		32(%r15), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm5
	movapd		80(%r13), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm2
	movapd		64(%r15), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm6
	movapd		112(%r13), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm3
	movapd		96(%r15), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm7


	jmp		3f

2:

	// offset==3

	movlpd		24(%r13), %xmm15
	movhpd		0(%r15), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm0
	movupd		8(%r15), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm4
	movlpd		56(%r13), %xmm15
	movhpd		32(%r15), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm1
	movupd		40(%r15), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm5
	movlpd		88(%r13), %xmm15
	movhpd		64(%r15), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm2
	movupd		72(%r15), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm6
	movlpd		120(%r13), %xmm15
	movhpd		96(%r15), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm3
	movupd		104(%r15), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm7


3:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_blend_scale_ab_4x4_gen_lib4)
#endif





// common inner routine with file scope
//
// blender for alpha = 1.0 and beta
//
// input arguments:
// r10   <- beta
// r11   <- C
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

// output arguments:
// r10   <- beta
// r11   <- C
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_SCALE_M1B_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_blend_scale_m1b_4x4_lib4)
#endif	
	
	movapd	%xmm0, %xmm8
	movsd	%xmm1, %xmm0
	movsd	%xmm8, %xmm1

	movapd	%xmm2, %xmm8
	movsd	%xmm3, %xmm2
	movsd	%xmm8, %xmm3

	movapd	%xmm4, %xmm8
	movsd	%xmm5, %xmm4
	movsd	%xmm8, %xmm5

	movapd	%xmm6, %xmm8
	movsd	%xmm7, %xmm6
	movsd	%xmm8, %xmm7


	// beta
	movddup	0(%r10), %xmm14

	xorpd		%xmm15, %xmm15 // 0.0
	ucomisd		%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	movapd		0(%r11), %xmm15
	mulpd		%xmm14, %xmm15
	subpd		%xmm0, %xmm15
	movapd		%xmm15, %xmm0
	movapd		16(%r11), %xmm15
	mulpd		%xmm14, %xmm15
	subpd		%xmm4, %xmm15
	movapd		%xmm15, %xmm4
	movapd		32(%r11), %xmm15
	mulpd		%xmm14, %xmm15
	subpd		%xmm1, %xmm15
	movapd		%xmm15, %xmm1
	movapd		48(%r11), %xmm15
	mulpd		%xmm14, %xmm15
	subpd		%xmm5, %xmm15
	movapd		%xmm15, %xmm5
	movapd		64(%r11), %xmm15
	mulpd		%xmm14, %xmm15
	subpd		%xmm2, %xmm15
	movapd		%xmm15, %xmm2
	movapd		80(%r11), %xmm15
	mulpd		%xmm14, %xmm15
	subpd		%xmm6, %xmm15
	movapd		%xmm15, %xmm6
	movapd		96(%r11), %xmm15
	mulpd		%xmm14, %xmm15
	subpd		%xmm3, %xmm15
	movapd		%xmm15, %xmm3
	movapd		112(%r11), %xmm15
	mulpd		%xmm14, %xmm15
	subpd		%xmm7, %xmm15
	movapd		%xmm15, %xmm7

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_blend_scale_m1b_4x4_lib4)
#endif





// common inner routine with file scope
//
// blender for alpha = 1.0 and beta = 1.0
//
// input arguments:
// r10   <- C
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

// output arguments:
// r10   <- C
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_SCALE_M11_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_blend_scale_m11_4x4_lib4)
#endif	
	
	movapd	%xmm0, %xmm8
	movsd	%xmm1, %xmm0
	movsd	%xmm8, %xmm1

	movapd	%xmm2, %xmm8
	movsd	%xmm3, %xmm2
	movsd	%xmm8, %xmm3

	movapd	%xmm4, %xmm8
	movsd	%xmm5, %xmm4
	movsd	%xmm8, %xmm5

	movapd	%xmm6, %xmm8
	movsd	%xmm7, %xmm6
	movsd	%xmm8, %xmm7


	movapd		0(%r10), %xmm15
	subpd		%xmm0, %xmm15
	movapd		%xmm15, %xmm0
	movapd		16(%r10), %xmm15
	subpd		%xmm4, %xmm15
	movapd		%xmm15, %xmm4
	movapd		32(%r10), %xmm15
	subpd		%xmm1, %xmm15
	movapd		%xmm15, %xmm1
	movapd		48(%r10), %xmm15
	subpd		%xmm5, %xmm15
	movapd		%xmm15, %xmm5
	movapd		64(%r10), %xmm15
	subpd		%xmm2, %xmm15
	movapd		%xmm15, %xmm2
	movapd		80(%r10), %xmm15
	subpd		%xmm6, %xmm15
	movapd		%xmm15, %xmm6
	movapd		96(%r10), %xmm15
	subpd		%xmm3, %xmm15
	movapd		%xmm15, %xmm3
	movapd		112(%r10), %xmm15
	subpd		%xmm7, %xmm15
	movapd		%xmm15, %xmm7

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_blend_scale_m11_4x4_lib4)
#endif





// common inner routine with file scope
//
// cholesky factorization 
//
// input arguments:
// r10  <- inv_diag_E
// r11d <- kn
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty
//
// output arguments:
// r10  <- inv_diag_E
// r11d <- kn
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DPOTRF_4X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dpotrf_4x4_vs_lib4)
#endif
	
	xorpd			%xmm15, %xmm15 // 0.0

	movsd			%xmm0, %xmm13
	ucomisd			%xmm15, %xmm13 // d_00 > 0.0 ?
	jbe				1f
	sqrtsd			%xmm13, %xmm13
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	movsd			.LC04(%rip), %xmm12 // 1.0
#elif defined(OS_MAC)
	movsd			LC04(%rip), %xmm12 // 1.0
#endif
	divsd			%xmm13, %xmm12
2:
	cmpl			$ 2, %r11d
	movsd			%xmm12, 0(%r10)
	movddup			%xmm12, %xmm12
	mulpd			%xmm12, %xmm0
	mulpd			%xmm12, %xmm4

	jl				0f // ret

	movapd			%xmm0, %xmm12
	shufpd			$ 0x3, %xmm12, %xmm12
	movapd			%xmm12, %xmm13
	mulpd			%xmm0, %xmm12
	mulpd			%xmm4, %xmm13
	subpd			%xmm12, %xmm1
	subpd			%xmm13, %xmm5
	movapd			%xmm1, %xmm13
	shufpd			$ 0x3, %xmm13, %xmm13 // 0x1 ???
	ucomisd			%xmm15, %xmm13 // d_11 > 0.0 ?
	jbe				3f
	sqrtsd			%xmm13, %xmm13
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	movsd			.LC04(%rip), %xmm12 // 1.0
#elif defined(OS_MAC)
	movsd			LC04(%rip), %xmm12 // 1.0
#endif
	divsd			%xmm13, %xmm12
4:
	cmpl			$ 3, %r11d
	movsd			%xmm12, 8(%r10)
	movddup			%xmm12, %xmm12
	mulpd			%xmm12, %xmm1
	mulpd			%xmm12, %xmm5

	jl				0f // ret

	movddup			%xmm4, %xmm12
	movddup			%xmm5, %xmm13
	mulpd			%xmm4, %xmm12
	mulpd			%xmm5, %xmm13
	subpd			%xmm12, %xmm6
	subpd			%xmm13, %xmm6
	movsd			%xmm6, %xmm13
	ucomisd			%xmm15, %xmm13 // d_22 > 0.0 ?
	jbe				5f
	sqrtsd			%xmm13, %xmm13
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	movsd			.LC04(%rip), %xmm12 // 1.0
#elif defined(OS_MAC)
	movsd			LC04(%rip), %xmm12 // 1.0
#endif
	divsd			%xmm13, %xmm12
6:
	cmpl			$ 4, %r11d
	movsd			%xmm12, 16(%r10)
	movddup			%xmm12, %xmm12
	mulpd			%xmm12, %xmm6

	jl				0f // ret

	movapd			%xmm4, %xmm12
	movapd			%xmm5, %xmm13
	movapd			%xmm6, %xmm14
	shufpd			$ 0x3, %xmm12, %xmm12
	shufpd			$ 0x3, %xmm13, %xmm13
	shufpd			$ 0x3, %xmm14, %xmm14
	mulpd			%xmm4, %xmm12
	mulpd			%xmm5, %xmm13
	mulpd			%xmm6, %xmm14
	subpd			%xmm12, %xmm7
	subpd			%xmm13, %xmm7
	subpd			%xmm14, %xmm7
	movapd			%xmm7, %xmm13
	shufpd			$ 0x3, %xmm13, %xmm13
	ucomisd			%xmm15, %xmm13 // d_33 > 0.0 ?
	jbe				7f
	sqrtsd			%xmm13, %xmm13
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	movsd			.LC04(%rip), %xmm12 // 1.0
#elif defined(OS_MAC)
	movsd			LC04(%rip), %xmm12 // 1.0
#endif
	divsd			%xmm13, %xmm12
8:
	movsd			%xmm12, 24(%r10)
	movddup			%xmm12, %xmm12
	mulpd			%xmm12, %xmm7

	jmp		0f
	
1:
	xorpd	%xmm12, %xmm12
	jmp		2b

3:
	xorpd	%xmm12, %xmm12
	jmp		4b

5:
	xorpd	%xmm12, %xmm12
	jmp		6b

7:
	xorpd	%xmm12, %xmm12
	jmp		8b

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dpotrf_4x4_vs_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution for cholesky factorization 
//
// input arguments:
// r10  <- E
// r11  <- inv_diag_E
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty
//
// output arguments:
// r10  <- E
// r11  <- inv_diag_E
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RLT_INV_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rlt_inv_4x4_lib4)
#endif
	
	movddup			0(%r11), %xmm13
	mulpd			%xmm13, %xmm0
	mulpd			%xmm13, %xmm4
	movddup			8(%r10), %xmm13
	movapd			%xmm13, %xmm12
	mulpd			%xmm0, %xmm13
	mulpd			%xmm4, %xmm12
	subpd			%xmm13, %xmm1
	subpd			%xmm12, %xmm5
	movddup			16(%r10), %xmm13
	movapd			%xmm13, %xmm12
	mulpd			%xmm0, %xmm12
	mulpd			%xmm4, %xmm13
	subpd			%xmm12, %xmm2
	subpd			%xmm13, %xmm6
	movddup			24(%r10), %xmm13
	movapd			%xmm13, %xmm12
	mulpd			%xmm0, %xmm12
	mulpd			%xmm4, %xmm13
	subpd			%xmm12, %xmm3
	subpd			%xmm13, %xmm7

	movddup			8(%r11), %xmm13
	mulpd			%xmm13, %xmm1
	mulpd			%xmm13, %xmm5
	movddup			48(%r10), %xmm13
	movapd			%xmm13, %xmm12
	mulpd			%xmm1, %xmm12
	mulpd			%xmm5, %xmm13
	subpd			%xmm12, %xmm2
	subpd			%xmm13, %xmm6
	movddup			56(%r10), %xmm13
	movapd			%xmm13, %xmm12
	mulpd			%xmm1, %xmm12
	mulpd			%xmm5, %xmm13
	subpd			%xmm12, %xmm3
	subpd			%xmm13, %xmm7

	movddup			16(%r11), %xmm13
	mulpd			%xmm13, %xmm2
	mulpd			%xmm13, %xmm6
	movddup			88(%r10), %xmm13
	movapd			%xmm13, %xmm12
	mulpd			%xmm2, %xmm12
	mulpd			%xmm6, %xmm13
	subpd			%xmm12, %xmm3
	subpd			%xmm13, %xmm7

	movddup			24(%r11), %xmm13
	mulpd			%xmm13, %xmm3
	mulpd			%xmm13, %xmm7

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rlt_inv_4x4_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution for cholesky factorization 
//
// input arguments:
// r10  <- D
// r11  <- inv_diag_D
// r12d <- kn
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty
//
// output arguments:
// r10  <- D
// r11  <- inv_diag_D
// r12d <- kn
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RLT_INV_4X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rlt_inv_4x4_vs_lib4)
#endif
	
	movddup			0(%r11), %xmm13
	cmpl			$ 2, %r12d
	mulpd			%xmm13, %xmm0
	mulpd			%xmm13, %xmm4

	jl				0f // ret

	movddup			8(%r10), %xmm13
	cmpl			$ 3, %r12d
	movapd			%xmm13, %xmm12
	mulpd			%xmm0, %xmm13
	mulpd			%xmm4, %xmm12
	subpd			%xmm13, %xmm1
	subpd			%xmm12, %xmm5
	movddup			8(%r11), %xmm13
	mulpd			%xmm13, %xmm1
	mulpd			%xmm13, %xmm5

	jl				0f // ret

	movddup			16(%r10), %xmm13
	cmpl			$ 4, %r12d
	movapd			%xmm13, %xmm12
	mulpd			%xmm0, %xmm12
	mulpd			%xmm4, %xmm13
	subpd			%xmm12, %xmm2
	subpd			%xmm13, %xmm6
	movddup			48(%r10), %xmm13
	movapd			%xmm13, %xmm12
	mulpd			%xmm1, %xmm12
	mulpd			%xmm5, %xmm13
	subpd			%xmm12, %xmm2
	subpd			%xmm13, %xmm6
	movddup			16(%r11), %xmm13
	mulpd			%xmm13, %xmm2
	mulpd			%xmm13, %xmm6

	jl				0f // ret

	movddup			24(%r10), %xmm13
	movapd			%xmm13, %xmm12
	mulpd			%xmm0, %xmm12
	mulpd			%xmm4, %xmm13
	subpd			%xmm12, %xmm3
	subpd			%xmm13, %xmm7
	movddup			56(%r10), %xmm13
	movapd			%xmm13, %xmm12
	mulpd			%xmm1, %xmm12
	mulpd			%xmm5, %xmm13
	subpd			%xmm12, %xmm3
	subpd			%xmm13, %xmm7
	movddup			88(%r10), %xmm13
	movapd			%xmm13, %xmm12
	mulpd			%xmm2, %xmm12
	mulpd			%xmm6, %xmm13
	subpd			%xmm12, %xmm3
	subpd			%xmm13, %xmm7
	movddup			24(%r11), %xmm13
	mulpd			%xmm13, %xmm3
	mulpd			%xmm13, %xmm7

0:
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rlt_inv_4x4_vs_lib4)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty
//
// output arguments:
// r10  <- D
// xmm0  <- [d00 d10]
// xmm1  <- [d01 d11]
// xmm2  <- [d02 d12]
// xmm3  <- [d03 d13]
// xmm4  <- [d20 d30]
// xmm5  <- [d21 d31]
// xmm6  <- [d22 d32]
// xmm7  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_4x4_lib4)
#endif
	
	movapd %xmm0,   0(%r10)
	movapd %xmm4,  16(%r10)
	movapd %xmm1,  32(%r10)
	movapd %xmm5,  48(%r10)
	movapd %xmm2,  64(%r10)
	movapd %xmm6,  80(%r10)
	movapd %xmm3,  96(%r10)
	movapd %xmm7, 112(%r10)
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4x4_lib4)
#endif





// common inner routine with file scope
//
// store n vs
//
// input arguments:
// r10   <- D
// r11d   <- km
// r12d   <- kn
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty
//
// output arguments:
// r10   <- D
// r11d   <- km
// r12d   <- kn
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_4x4_vs_lib4)
#endif
	
	cmpl		$ 2, %r11d
	jg			1f
	je			0f

	// km==1
	movsd		%xmm0,  0(%r10)
	cmpl		$ 2, %r12d
	jl			4f // end
	movsd		%xmm1, 32(%r10)
	cmpl		$ 3, %r12d
	jl			4f // end
	movsd		%xmm2, 64(%r10)
	je			4f // end
	movsd		%xmm3, 96(%r10)

	jmp		4f

0:
	// km==2
	movapd		%xmm0,  0(%r10)
	cmpl		$ 2, %r12d
	jl			4f // end
	movapd		%xmm1, 32(%r10)
	cmpl		$ 3, %r12d
	jl			4f // end
	movapd		%xmm2, 64(%r10)
	je			4f // end
	movapd		%xmm3, 96(%r10)

	jmp		4f

1:
	cmpl		$ 3, %r11d
	jg			2f

	// km==3
	movapd		%xmm0,   0(%r10)
	movsd		%xmm4,  16(%r10)
	cmpl		$ 2, %r12d
	jl			4f // end
	movapd		%xmm1,  32(%r10)
	movsd		%xmm5,  48(%r10)
	cmpl		$ 3, %r12d
	jl			4f // end
	movapd		%xmm2,  64(%r10)
	movsd		%xmm6,  80(%r10)
	je			4f // end
	movapd		%xmm3,  96(%r10)
	movsd		%xmm7, 112(%r10)

	jmp		4f

2:
	// km==4
	movapd		%xmm0,   0(%r10)
	movapd		%xmm4,  16(%r10)
	cmpl		$ 2, %r12d
	jl			4f // end
	movapd		%xmm1,  32(%r10)
	movapd		%xmm5,  48(%r10)
	cmpl		$ 3, %r12d
	jl			4f // end
	movapd		%xmm2,  64(%r10)
	movapd		%xmm6,  80(%r10)
	je			4f // end
	movapd		%xmm3,  96(%r10)
	movapd		%xmm7, 112(%r10)

4:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4x4_vs_lib4)
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
// xmm0 <-
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
// xmm0 <-

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X4_GEN_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_4x4_gen_lib4)
#endif
	
	// masks computation ???

	// shift D and sol for cols
	cmpl	$ 0, %r15d
	jle		0f

	movapd		%xmm1, %xmm0
	movapd		%xmm5, %xmm4
	movapd		%xmm2, %xmm1
	movapd		%xmm6, %xmm5
	movapd		%xmm3, %xmm2
	movapd		%xmm7, %xmm6
	addq		$ 32, %r11

	cmpl	$ 1, %r15d
	jle		0f

	movapd		%xmm1, %xmm0
	movapd		%xmm5, %xmm4
	movapd		%xmm2, %xmm1
	movapd		%xmm6, %xmm5
	addq		$ 32, %r11

	cmpl	$ 2, %r15d
	jle		0f

	movapd		%xmm1, %xmm0
	movapd		%xmm5, %xmm4
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

	///////////////
	// offset==0 //
	///////////////

	cmpl	$ 0, %r13d
	jle		4f

	cmpl	$ 1, %r13d
	jg		5f

	movlpd	0(%r11), %xmm0
	movlpd	32(%r11), %xmm1
	movlpd	64(%r11), %xmm2
	movlpd	96(%r11), %xmm3

	jmp		4f

5:

	cmpl	$ 2, %r13d
	jg		5f

	movapd	0(%r11), %xmm0
	movapd	32(%r11), %xmm1
	movapd	64(%r11), %xmm2
	movapd	96(%r11), %xmm3

	jmp		4f

5:

	cmpl	$ 3, %r13d
	jg		5f

	movapd	0(%r11), %xmm0
	movlpd	16(%r11), %xmm4
	movapd	32(%r11), %xmm1
	movlpd	48(%r11), %xmm5
	movapd	64(%r11), %xmm2
	movlpd	80(%r11), %xmm6
	movapd	96(%r11), %xmm3
	movlpd	112(%r11), %xmm7

	jmp		4f

5:

	// XXX needed ???
	movapd	0(%r11), %xmm0
	movapd	16(%r11), %xmm4
	movapd	32(%r11), %xmm1
	movapd	48(%r11), %xmm5
	movapd	64(%r11), %xmm2
	movapd	80(%r11), %xmm6
	movapd	96(%r11), %xmm3
	movapd	112(%r11), %xmm7

4:
	cmpl		$ 2, %r14d
	jg			5f
	je			4f

	// km==1
	movsd		%xmm0,  0(%r11)
	cmpl		$ 2, %r15d
	jl			3f // end
	movsd		%xmm1, 32(%r11)
	cmpl		$ 3, %r15d
	jl			3f // end
	movsd		%xmm2, 64(%r11)
	je			3f // end
	movsd		%xmm3, 96(%r11)

	jmp		3f

4:
	// km==2
	movapd		%xmm0,  0(%r11)
	cmpl		$ 2, %r15d
	jl			3f // end
	movapd		%xmm1, 32(%r11)
	cmpl		$ 3, %r15d
	jl			3f // end
	movapd		%xmm2, 64(%r11)
	je			3f // end
	movapd		%xmm3, 96(%r11)

	jmp		3f

5:
	cmpl		$ 3, %r14d
	jg			6f

	// km==3
	movapd		%xmm0,   0(%r11)
	movsd		%xmm4,  16(%r11)
	cmpl		$ 2, %r15d
	jl			3f // end
	movapd		%xmm1,  32(%r11)
	movsd		%xmm5,  48(%r11)
	cmpl		$ 3, %r15d
	jl			3f // end
	movapd		%xmm2,  64(%r11)
	movsd		%xmm6,  80(%r11)
	je			3f // end
	movapd		%xmm3,  96(%r11)
	movsd		%xmm7, 112(%r11)

	jmp		3f

6:
	// km==4
	movapd		%xmm0,   0(%r11)
	movapd		%xmm4,  16(%r11)
	cmpl		$ 2, %r15d
	jl			3f // end
	movapd		%xmm1,  32(%r11)
	movapd		%xmm5,  48(%r11)
	cmpl		$ 3, %r15d
	jl			3f // end
	movapd		%xmm2,  64(%r11)
	movapd		%xmm6,  80(%r11)
	je			3f // end
	movapd		%xmm3,  96(%r11)
	movapd		%xmm7, 112(%r11)

	jmp		3f

0:
	
	movq	%r11, %rbx // D0
	addq	%r12, %rbx // D1 <- D0 + 4*sdd*sizeof(double)

	cmpl	$ 1, %r10d
	jg		1f

	///////////////
	// offset==1 //
	///////////////

	cmpl	$ 0, %r13d
	jle		4f

	cmpl	$ 1, %r13d
	jg		5f

	movlpd	8(%r11), %xmm0
	movlpd	40(%r11), %xmm1
	movlpd	72(%r11), %xmm2
	movlpd	104(%r11), %xmm3

	jmp		4f

5:

	cmpl	$ 2, %r13d
	jg		5f

	movupd	8(%r11), %xmm0
	movupd	40(%r11), %xmm1
	movupd	72(%r11), %xmm2
	movupd	104(%r11), %xmm3

	jmp		4f

5:

	cmpl	$ 3, %r13d
	jg		5f

	movupd	8(%r11), %xmm0
	movupd	40(%r11), %xmm1
	movupd	72(%r11), %xmm2
	movupd	104(%r11), %xmm3
	movlpd	24(%r11), %xmm4
	movlpd	56(%r11), %xmm5
	movlpd	88(%r11), %xmm6
	movlpd	120(%r11), %xmm7

	jmp		4f

5:

	// XXX needed ???
	movupd	8(%r11), %xmm0
	movupd	40(%r11), %xmm1
	movupd	72(%r11), %xmm2
	movupd	104(%r11), %xmm3
	movlpd	24(%r11), %xmm4
	movlpd	56(%r11), %xmm5
	movlpd	88(%r11), %xmm6
	movlpd	120(%r11), %xmm7
	movhpd	0(%rbx), %xmm4
	movhpd	32(%rbx), %xmm5
	movhpd	64(%rbx), %xmm6
	movhpd	96(%rbx), %xmm7

4:
	cmpl		$ 2, %r14d
	jg			5f
	je			4f

	// km==1
	movsd		%xmm0,   8(%r11)
	cmpl		$ 2, %r15d
	jl			3f // end
	movsd		%xmm1,  40(%r11)
	cmpl		$ 3, %r15d
	jl			3f // end
	movsd		%xmm2,  72(%r11)
	je			3f // end
	movsd		%xmm3, 104(%r11)

	jmp		3f

4:
	// km==2
	movupd		%xmm0,   8(%r11)
	cmpl		$ 2, %r15d
	jl			3f // end
	movupd		%xmm1,  40(%r11)
	cmpl		$ 3, %r15d
	jl			3f // end
	movupd		%xmm2,  72(%r11)
	je			3f // end
	movupd		%xmm3, 104(%r11)

	jmp		3f

5:
	cmpl		$ 3, %r14d
	jg			6f

	// km==3
	movupd		%xmm0,   8(%r11)
	movsd		%xmm4,  24(%r11)
	cmpl		$ 2, %r15d
	jl			3f // end
	movupd		%xmm1,  40(%r11)
	movsd		%xmm5,  56(%r11)
	cmpl		$ 3, %r15d
	jl			3f // end
	movupd		%xmm2,  72(%r11)
	movsd		%xmm6,  88(%r11)
	je			3f // end
	movupd		%xmm3, 104(%r11)
	movsd		%xmm7, 120(%r11)

	jmp		3f

6:
	// km==4
	movupd		%xmm0,   8(%r11)
	movsd		%xmm4,  24(%r11)
	movhpd		%xmm4,   0(%rbx)
	cmpl		$ 2, %r15d
	jl			3f // end
	movupd		%xmm1,  40(%r11)
	movsd		%xmm5,  56(%r11)
	movhpd		%xmm5,  32(%rbx)
	cmpl		$ 3, %r15d
	jl			3f // end
	movupd		%xmm2,  72(%r11)
	movsd		%xmm6,  88(%r11)
	movhpd		%xmm6,  64(%rbx)
	je			3f // end
	movupd		%xmm3, 104(%r11)
	movsd		%xmm7, 120(%r11)
	movhpd		%xmm7,  96(%rbx)


	jmp		3f

1:

	cmpl	$ 2, %r10d
	jg		2f

	///////////////
	// offset==2 //
	///////////////

	cmpl	$ 0, %r13d
	jle		4f

	cmpl	$ 1, %r13d
	jg		5f

	movlpd	16(%r11), %xmm0
	movlpd	48(%r11), %xmm1
	movlpd	80(%r11), %xmm2
	movlpd	112(%r11), %xmm3

	jmp		4f

5:

	cmpl	$ 2, %r13d
	jg		5f

	movapd	16(%r11), %xmm0
	movapd	48(%r11), %xmm1
	movapd	80(%r11), %xmm2
	movapd	112(%r11), %xmm3

	jmp		4f

5:

	cmpl	$ 3, %r13d
	jg		5f

	movapd	16(%r11), %xmm0
	movlpd	0(%rbx), %xmm4
	movapd	48(%r11), %xmm1
	movlpd	32(%rbx), %xmm5
	movapd	80(%r11), %xmm2
	movlpd	64(%rbx), %xmm6
	movapd	112(%r11), %xmm3
	movlpd	96(%rbx), %xmm7

	jmp		4f

5:

	// XXX needed ???
	movapd	16(%r11), %xmm0
	movapd	0(%rbx), %xmm4
	movapd	48(%r11), %xmm1
	movapd	32(%rbx), %xmm5
	movapd	80(%r11), %xmm2
	movapd	64(%rbx), %xmm6
	movapd	112(%r11), %xmm3
	movapd	96(%rbx), %xmm7

4:
	cmpl		$ 2, %r14d
	jg			5f
	je			4f

	// km==1
	movsd		%xmm0, 16(%r11)
	cmpl		$ 2, %r15d
	jl			3f // end
	movsd		%xmm1, 48(%r11)
	cmpl		$ 3, %r15d
	jl			3f // end
	movsd		%xmm2, 80(%r11)
	je			3f // end
	movsd		%xmm3, 112(%r11)

	jmp		3f

4:
	// km==2
	movapd		%xmm0, 16(%r11)
	cmpl		$ 2, %r15d
	jl			3f // end
	movapd		%xmm1, 48(%r11)
	cmpl		$ 3, %r15d
	jl			3f // end
	movapd		%xmm2, 80(%r11)
	je			3f // end
	movapd		%xmm3, 112(%r11)

	jmp		3f

5:
	cmpl		$ 3, %r14d
	jg			6f

	// km==3
	movapd		%xmm0,  16(%r11)
	movsd		%xmm4,   0(%rbx)
	cmpl		$ 2, %r15d
	jl			3f // end
	movapd		%xmm1,  48(%r11)
	movsd		%xmm5,  32(%rbx)
	cmpl		$ 3, %r15d
	jl			3f // end
	movapd		%xmm2,  80(%r11)
	movsd		%xmm6,  64(%rbx)
	je			3f // end
	movapd		%xmm3, 112(%r11)
	movsd		%xmm7,  96(%rbx)

	jmp		3f

6:
	// km==4
	movapd		%xmm0,  16(%r11)
	movapd		%xmm4,   0(%rbx)
	cmpl		$ 2, %r15d
	jl			3f // end
	movapd		%xmm1,  48(%r11)
	movapd		%xmm5,  32(%rbx)
	cmpl		$ 3, %r15d
	jl			3f // end
	movapd		%xmm2,  80(%r11)
	movapd		%xmm6,  64(%rbx)
	je			3f // end
	movapd		%xmm3, 112(%r11)
	movapd		%xmm7,  96(%rbx)


	jmp		3f

2:

	///////////////
	// offset==3 //
	///////////////

	cmpl	$ 0, %r13d
	jle		4f

	cmpl	$ 1, %r13d
	jg		5f

	movlpd	24(%r11), %xmm0
	movlpd	56(%r11), %xmm1
	movlpd	88(%r11), %xmm2
	movlpd	120(%r11), %xmm3

	jmp		4f

5:

	cmpl	$ 2, %r13d
	jg		5f

	movlpd	24(%r11), %xmm0
	movlpd	56(%r11), %xmm1
	movlpd	88(%r11), %xmm2
	movlpd	120(%r11), %xmm3
	movhpd	0(%rbx), %xmm0
	movhpd	32(%rbx), %xmm1
	movhpd	64(%rbx), %xmm2
	movhpd	96(%rbx), %xmm3

	jmp		4f

5:

	cmpl	$ 3, %r13d
	jg		5f

	movlpd	24(%r11), %xmm0
	movlpd	56(%r11), %xmm1
	movlpd	88(%r11), %xmm2
	movlpd	120(%r11), %xmm3
	movhpd	0(%rbx), %xmm0
	movhpd	32(%rbx), %xmm1
	movhpd	64(%rbx), %xmm2
	movhpd	96(%rbx), %xmm3
	movlpd	8(%rbx), %xmm4
	movlpd	40(%rbx), %xmm5
	movlpd	72(%rbx), %xmm6
	movlpd	104(%rbx), %xmm7

	jmp		4f

5:

	movlpd	24(%r11), %xmm0
	movlpd	56(%r11), %xmm1
	movlpd	88(%r11), %xmm2
	movlpd	120(%r11), %xmm3
	movhpd	0(%rbx), %xmm0
	movhpd	32(%rbx), %xmm1
	movhpd	64(%rbx), %xmm2
	movhpd	96(%rbx), %xmm3
	movupd	8(%rbx), %xmm4
	movupd	40(%rbx), %xmm5
	movupd	72(%rbx), %xmm6
	movupd	104(%rbx), %xmm7

4:
	cmpl		$ 2, %r14d
	jg			5f
	je			4f

	// km==1
	movsd		%xmm0,  24(%r11)
	cmpl		$ 2, %r15d
	jl			3f // end
	movsd		%xmm1,  56(%r11)
	cmpl		$ 3, %r15d
	jl			3f // end
	movsd		%xmm2,  88(%r11)
	je			3f // end
	movsd		%xmm3, 120(%r11)

	jmp		3f

4:
	// km==2
	movsd		%xmm0,  24(%r11)
	movhpd		%xmm0,   0(%rbx)
	cmpl		$ 2, %r15d
	jl			3f // end
	movsd		%xmm1,  56(%r11)
	movhpd		%xmm1,  32(%rbx)
	cmpl		$ 3, %r15d
	jl			3f // end
	movsd		%xmm2,  88(%r11)
	movhpd		%xmm2,  64(%rbx)
	je			3f // end
	movsd		%xmm3, 120(%r11)
	movhpd		%xmm3,  96(%rbx)

	jmp		3f

5:
	cmpl		$ 3, %r14d
	jg			6f

	// km==3
	movsd		%xmm0,  24(%r11)
	movhpd		%xmm0,   0(%rbx)
	movsd		%xmm4,   8(%rbx)
	cmpl		$ 2, %r15d
	jl			3f // end
	movsd		%xmm1,  56(%r11)
	movhpd		%xmm1,  32(%rbx)
	movsd		%xmm5,  40(%rbx)
	cmpl		$ 3, %r15d
	jl			3f // end
	movsd		%xmm2,  88(%r11)
	movhpd		%xmm2,  64(%rbx)
	movsd		%xmm6,  72(%rbx)
	je			3f // end
	movsd		%xmm3, 120(%r11)
	movhpd		%xmm3,  96(%rbx)
	movsd		%xmm7, 104(%rbx)

	jmp		3f

6:
	// km==4
	movsd		%xmm0,  24(%r11)
	movhpd		%xmm0,   0(%rbx)
	movupd		%xmm4,   8(%rbx)
	cmpl		$ 2, %r15d
	jl			3f // end
	movsd		%xmm1,  56(%r11)
	movhpd		%xmm1,  32(%rbx)
	movupd		%xmm5,  40(%rbx)
	cmpl		$ 3, %r15d
	jl			3f // end
	movsd		%xmm2,  88(%r11)
	movhpd		%xmm2,  64(%rbx)
	movupd		%xmm6,  72(%rbx)
	je			3f // end
	movsd		%xmm3, 120(%r11)
	movhpd		%xmm3,  96(%rbx)
	movupd		%xmm7, 104(%rbx)


3:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4x4_gen_lib4)
#endif





// common inner routine with file scope
//
// store n lower triangular
//
// input arguments:
// r10   <- D
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty
//
// output arguments:
// r10   <- D
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_l_4x4_lib4)
#endif
	
	movapd		%xmm0,   0(%r10)
	movapd		%xmm4,  16(%r10)
	movhpd		%xmm1,  40(%r10)
	movapd		%xmm5,  48(%r10)
//	movapd		%xmm2,  64(%r10)
	movapd		%xmm6,  80(%r10)
//	movapd		%xmm3,  96(%r10)
	movhpd		%xmm7, 120(%r10)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_4x4_lib4)
#endif





// common inner routine with file scope
//
// store n vs lower triangular
//
// input arguments:
// r10   <- D
// r11d   <- km
// r12d   <- kn
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty
//
// output arguments:
// r10   <- D
// r11d   <- km
// r12d   <- kn
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_4X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_l_4x4_vs_lib4)
#endif
	
	cmpl		$ 2, %r11d
	jg			1f
	je			0f

	// km==1
	movsd		%xmm0,  0(%r10)

	jmp		3f

0:
	// km==2
	cmpl		$ 2, %r12d
	movapd		%xmm0,  0(%r10)
	jl			3f // end
	movhpd		%xmm1, 40(%r10)

	jmp		3f

1:
	cmpl		$ 3, %r11d
	jg			2f

	// km==3
	cmpl		$ 2, %r12d
	movapd		%xmm0,   0(%r10)
	movsd		%xmm4,  16(%r10)
	jl			3f // end
	cmpl		$ 3, %r12d
	movhpd		%xmm1,  40(%r10)
	movsd		%xmm5,  48(%r10)
	jl			3f // end
//	movapd		%xmm2,  64(%r10)
	movsd		%xmm6,  80(%r10)

	jmp		3f

2:
	// km==3
	cmpl		$ 2, %r12d
	movapd		%xmm0,   0(%r10)
	movapd		%xmm4,  16(%r10)
	jl			3f // end
	cmpl		$ 3, %r12d
	movhpd		%xmm1,  40(%r10)
	movapd		%xmm5,  48(%r10)
	jl			3f // end
//	movapd		%xmm2,  64(%r10)
	movapd		%xmm6,  80(%r10)
	je			3f // end
//	movapd		%xmm3,  96(%r10)
	movhpd		%xmm7, 120(%r10)

3:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_4x4_vs_lib4)
#endif





#if 0

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
// xmm0 <-
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
// xmm0 <-

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_4X4_GEN_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_l_4x4_gen_lib4)
#endif
	
	// masks computation ???

	// shift D and sol for cols
	cmpl	$ 0, %r15d
	jle		0f

	movapd		%xmm1, %xmm0
	movapd		%xmm5, %xmm4
	movapd		%xmm2, %xmm1
	movapd		%xmm6, %xmm5
	movapd		%xmm3, %xmm2
	movapd		%xmm7, %xmm6
	addq		$ 32, %r11

	cmpl	$ 1, %r15d
	jle		0f

	movapd		%xmm1, %xmm0
	movapd		%xmm5, %xmm4
	movapd		%xmm2, %xmm1
	movapd		%xmm6, %xmm5
	addq		$ 32, %r11

	cmpl	$ 2, %r15d
	jle		0f

	movapd		%xmm1, %xmm0
	movapd		%xmm5, %xmm4
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

	///////////////
	// offset==0 //
	///////////////

	cmpl	$ 0, %r13d
	jle		4f

	cmpl	$ 1, %r13d
	jg		5f

	movlpd	0(%r11), %xmm0

	jmp		4f

5:

	cmpl	$ 2, %r13d
	jg		5f

	movapd	0(%r11), %xmm0
	movhpd	40(%r11), %xmm1

	jmp		4f

5:

	cmpl	$ 3, %r13d
	jg		5f

	movapd	0(%r11), %xmm0
	movlpd	16(%r11), %xmm4
	movhpd	40(%r11), %xmm1
	movlpd	48(%r11), %xmm5
	movlpd	80(%r11), %xmm6

	jmp		4f

5:

	// XXX needed ???
	movapd	0(%r11), %xmm0
	movapd	16(%r11), %xmm4
	movhpd	40(%r11), %xmm1
	movapd	48(%r11), %xmm5
	movapd	80(%r11), %xmm6
	movhpd	120(%r11), %xmm7

4:
	cmpl		$ 2, %r14d
	jg			5f
	je			4f

	// km==1
	movsd		%xmm0,  0(%r11)

	jmp		3f

4:
	// km==2
	movapd		%xmm0,  0(%r11)
	cmpl		$ 2, %r15d
	jl			3f // end
	movhpd		%xmm1, 40(%r11)

	jmp		3f

5:
	cmpl		$ 3, %r14d
	jg			6f

	// km==3
	movapd		%xmm0,   0(%r11)
	movsd		%xmm4,  16(%r11)
	cmpl		$ 2, %r15d
	jl			3f // end
	movhpd		%xmm1,  40(%r11)
	movsd		%xmm5,  48(%r11)
	cmpl		$ 3, %r15d
	jl			3f // end
	movsd		%xmm6,  80(%r11)

	jmp		3f

6:
	// km==4
	movapd		%xmm0,   0(%r11)
	movapd		%xmm4,  16(%r11)
	cmpl		$ 2, %r15d
	jl			3f // end
	movhpd		%xmm1,  40(%r11)
	movapd		%xmm5,  48(%r11)
	cmpl		$ 3, %r15d
	jl			3f // end
	movapd		%xmm6,  80(%r11)
	je			3f // end
	movhpd		%xmm7, 120(%r11)

	jmp		3f

0:
	
	movq	%r11, %rbx // D0
	addq	%r12, %rbx // D1 <- D0 + 4*sdd*sizeof(double)

	cmpl	$ 1, %r10d
	jg		1f

	///////////////
	// offset==1 //
	///////////////

	cmpl	$ 0, %r13d
	jle		4f

	cmpl	$ 1, %r13d
	jg		5f

	movlpd	8(%r11), %xmm0

	jmp		4f

5:

	cmpl	$ 2, %r13d
	jg		5f

	movupd	8(%r11), %xmm0
	movhpd	48(%r11), %xmm1

	jmp		4f

5:

	cmpl	$ 3, %r13d
	jg		5f

	movupd	8(%r11), %xmm0
	movlpd	24(%r11), %xmm4
	movhpd	48(%r11), %xmm1
	movlpd	56(%r11), %xmm5
	movlpd	88(%r11), %xmm6

	jmp		4f

5:

	// XXX needed ???
	movupd	8(%r11), %xmm0
	movlpd	24(%r11), %xmm4
	movhpd	0(%rbx), %xmm4
	movhpd	48(%r11), %xmm1
	movlpd	56(%r11), %xmm5
	movhpd	32(%rbx), %xmm5
	movlpd	88(%r11), %xmm6
	movhpd	64(%rbx), %xmm6
	movhpd	96(%rbx), %xmm7

4:
	cmpl		$ 2, %r14d
	jg			5f
	je			4f

	// km==1
	movsd		%xmm0,   8(%r11)

	jmp		3f

4:
	// km==2
	movupd		%xmm0,   8(%r11)
	cmpl		$ 2, %r15d
	jl			3f // end
	movhpd		%xmm1,  48(%r11)

	jmp		3f

5:
	cmpl		$ 3, %r14d
	jg			6f

	// km==3
	movupd		%xmm0,   8(%r11)
	movsd		%xmm4,  24(%r11)
	cmpl		$ 2, %r15d
	jl			3f // end
	movhpd		%xmm1,  48(%r11)
	movsd		%xmm5,  56(%r11)
	cmpl		$ 3, %r15d
	jl			3f // end
	movsd		%xmm6,  88(%r11)

	jmp		3f

6:
	// km==4
	movupd		%xmm0,   8(%r11)
	movsd		%xmm4,  24(%r11)
	movhpd		%xmm4,   0(%rbx)
	cmpl		$ 2, %r15d
	jl			3f // end
	movhpd		%xmm1,  48(%r11)
	movsd		%xmm5,  56(%r11)
	movhpd		%xmm5,  32(%rbx)
	cmpl		$ 3, %r15d
	jl			3f // end
	movsd		%xmm6,  88(%r11)
	movhpd		%xmm6,  64(%rbx)
	je			3f // end
	movhpd		%xmm7,  96(%rbx)


	jmp		3f

1:

	cmpl	$ 2, %r10d
	jg		2f

	///////////////
	// offset==2 //
	///////////////

	cmpl	$ 0, %r13d
	jle		4f

	cmpl	$ 1, %r13d
	jg		5f

	movlpd	16(%r11), %xmm0

	jmp		4f

5:

	cmpl	$ 2, %r13d
	jg		5f

	movapd	16(%r11), %xmm0
	movhpd	56(%r11), %xmm1

	jmp		4f

5:

	cmpl	$ 3, %r13d
	jg		5f

	movapd	16(%r11), %xmm0
	movlpd	0(%rbx), %xmm4
	movhpd	56(%r11), %xmm1
	movlpd	32(%rbx), %xmm5
	movlpd	64(%rbx), %xmm6

	jmp		4f

5:

	// XXX needed ???
	movapd	16(%r11), %xmm0
	movapd	0(%rbx), %xmm4
	movhpd	56(%r11), %xmm1
	movapd	32(%rbx), %xmm5
	movapd	64(%rbx), %xmm6
	movhpd	104(%rbx), %xmm7

4:
	cmpl		$ 2, %r14d
	jg			5f
	je			4f

	// km==1
	movsd		%xmm0, 16(%r11)

	jmp		3f

4:
	// km==2
	movapd		%xmm0, 16(%r11)
	cmpl		$ 2, %r15d
	jl			3f // end
	movhpd		%xmm1, 56(%r11)

	jmp		3f

5:
	cmpl		$ 3, %r14d
	jg			6f

	// km==3
	movapd		%xmm0,  16(%r11)
	movsd		%xmm4,   0(%rbx)
	cmpl		$ 2, %r15d
	jl			3f // end
	movhpd		%xmm1,  56(%r11)
	movsd		%xmm5,  32(%rbx)
	cmpl		$ 3, %r15d
	jl			3f // end
	movsd		%xmm6,  64(%rbx)

	jmp		3f

6:
	// km==4
	movapd		%xmm0,  16(%r11)
	movapd		%xmm4,   0(%rbx)
	cmpl		$ 2, %r15d
	jl			3f // end
	movhpd		%xmm1,  56(%r11)
	movapd		%xmm5,  32(%rbx)
	cmpl		$ 3, %r15d
	jl			3f // end
	movapd		%xmm6,  64(%rbx)
	je			3f // end
	movhpd		%xmm7, 104(%rbx)


	jmp		3f

2:

	///////////////
	// offset==3 //
	///////////////

	cmpl	$ 0, %r13d
	jle		4f

	cmpl	$ 1, %r13d
	jg		5f

	movlpd	24(%r11), %xmm0

	jmp		4f

5:

	cmpl	$ 2, %r13d
	jg		5f

	movlpd	24(%r11), %xmm0
	movhpd	0(%rbx), %xmm0
	movhpd	32(%rbx), %xmm1

	jmp		4f

5:

	cmpl	$ 3, %r13d
	jg		5f

	movlpd	24(%r11), %xmm0
	movhpd	0(%rbx), %xmm0
	movlpd	8(%r11), %xmm4
	movhpd	32(%rbx), %xmm1
	movlpd	40(%r11), %xmm5
	movlpd	72(%r11), %xmm6

	jmp		4f

5:

	movlpd	24(%r11), %xmm0
	movhpd	0(%rbx), %xmm0
	movupd	8(%r11), %xmm4
	movhpd	32(%rbx), %xmm1
	movupd	40(%r11), %xmm5
	movupd	72(%r11), %xmm6
	movhpd	112(%r11), %xmm7

4:
	cmpl		$ 2, %r14d
	jg			5f
	je			4f

	// km==1
	movsd		%xmm0,  24(%r11)

	jmp		3f

4:
	// km==2
	movsd		%xmm0,  24(%r11)
	movhpd		%xmm0,   0(%rbx)
	cmpl		$ 2, %r15d
	jl			3f // end
	movhpd		%xmm1,  32(%rbx)

	jmp		3f

5:
	cmpl		$ 3, %r14d
	jg			6f

	// km==3
	movsd		%xmm0,  24(%r11)
	movhpd		%xmm0,   0(%rbx)
	movsd		%xmm4,   8(%rbx)
	cmpl		$ 2, %r15d
	jl			3f // end
	movhpd		%xmm1,  32(%rbx)
	movsd		%xmm5,  40(%rbx)
	cmpl		$ 3, %r15d
	jl			3f // end
	movsd		%xmm6,  72(%rbx)

	jmp		3f

6:
	// km==4
	movsd		%xmm0,  24(%r11)
	movhpd		%xmm0,   0(%rbx)
	movupd		%xmm4,   8(%rbx)
	cmpl		$ 2, %r15d
	jl			3f // end
	movhpd		%xmm1,  32(%rbx)
	movupd		%xmm5,  40(%rbx)
	cmpl		$ 3, %r15d
	jl			3f // end
	movupd		%xmm6,  72(%rbx)
	je			3f // end
	movhpd		%xmm7, 112(%rbx)


3:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_4x4_gen_lib4)
#endif

#endif





// common inner routine with file scope
//
// transpose
//
// input arguments:
// r10  <- D
// r11  <- ldd
// ymm0 <- [d00 d11 d22 d33]
// ymm1 <- [d01 d10 d23 d32]
// ymm2 <- [d03 d12 d21 d30]
// ymm3 <- [d02 d13 d20 d31]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_TRAN_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_tran_4x4_lib4)
#endif
	
	movapd		%xmm0, %xmm12
	unpckhpd	%xmm1, %xmm12
	unpcklpd	%xmm1, %xmm0
	movapd		%xmm12, %xmm1

	movapd		%xmm6, %xmm12
	unpckhpd	%xmm7, %xmm12
	unpcklpd	%xmm7, %xmm6
	movapd		%xmm12, %xmm7

	movapd		%xmm2, %xmm12
	movapd		%xmm2, %xmm13
	unpckhpd	%xmm3, %xmm12
	unpcklpd	%xmm3, %xmm13

	movapd		%xmm4, %xmm2
	unpckhpd	%xmm5, %xmm4
	unpcklpd	%xmm5, %xmm2
	movapd		%xmm4, %xmm3

	movapd		%xmm12, %xmm5
	movapd		%xmm13, %xmm4

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_tran_4x4_lib4)
#endif





//                               1      2              3          4          5             6          7
// void kernel_dgemm_nt_4x4_lib4(int k, double *alpha, double *A, double *B, double *beta, double *C, double *D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_4x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12   // C

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_AB_4X4_LIB4
#else
	CALL(inner_blend_scale_ab_4x4_lib4)
#endif


	// store n

	movq	ARG7, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
	CALL(inner_store_4x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_4x4_lib4)





//                                  rdi    rsi            rdx        rcx        r8            r9         rsp+8     rsp+16   rsp+24
// void kernel_dgemm_nt_4x4_vs_lib4(int k, double *alpha, double *A, double *B, double *beta, double *C, double *D, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_4x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12   // C

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_AB_4X4_LIB4
#else
	CALL(inner_blend_scale_ab_4x4_lib4)
#endif


	// store n

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // km 
	movq	ARG9, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB4
#else
	CALL(inner_store_4x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_4x4_vs_lib4)






//                                   1      2              3          4          5             6            7          8        9            10         11       12      13      14      15
// void kernel_dgemm_nt_4x4_gen_lib4(int k, double *alpha, double *A, double *B, double *beta, int offsetC, double *C, int sdc, int offsetD, double *D, int sdd, int m0, int m1, int n0, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_4x4_gen_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12 // offsetC
	movq	ARG7, %r13 // C
	movq	ARG8, %r14 // sdc
	sall	$ 5, %r14d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_AB_4X4_GEN_LIB4
#else
	CALL(inner_blend_scale_ab_4x4_gen_lib4)
#endif

	// store n gen

	movq	ARG9, %r10 // offsetD
	movq	ARG10, %r11 // D
	movq	ARG11, %r12 // sdd
	sall	$ 5, %r12d // 4*sdb*sizeof(double)
	movq	ARG12, %r13 // m0
	movq	ARG13, %r14 // m1
	movq	ARG14, %r15 // n0
	movq	ARG15, %rax // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_GEN_LIB4
#else
	CALL(inner_store_4x4_gen_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_4x4_gen_lib4)





//                               1      2              3          4            5          6        7             8          9
// void kernel_dgemm_nn_4x4_lib4(int k, double *alpha, double *A, int offsetB, double *B, int sdb, double *beta, double *C, double *D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nn_4x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG5, %r12  // B
	movq	ARG6, %r13 // sdb
	sall	$ 5, %r13d // 4*sdb*sizeof(double)
	movq	ARG4, %r14 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_edge_dgemm_nn_4x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_4x4_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
	CALL(inner_scale_ab_4x4_lib4)
#endif


	// store n

	movq	ARG9, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
	CALL(inner_store_4x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nn_4x4_lib4)





//                                  1      2              3          4            5          6        7             8          9          10      11
// void kernel_dgemm_nn_4x4_vs_lib4(int k, double *alpha, double *A, int offsetB, double *B, int sdb, double *beta, double *C, double *D, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nn_4x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG5, %r12  // B
	movq	ARG6, %r13 // sdb
	sall	$ 5, %r13d // 4*sdb*sizeof(double)
	movq	ARG4, %r14 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_edge_dgemm_nn_4x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_4x4_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
	CALL(inner_scale_ab_4x4_lib4)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // km 
	movq	ARG11, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB4
#else
	CALL(inner_store_4x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nn_4x4_vs_lib4)





//                                   1      2              3          4         5          6        7             8         9          10       11        12         13       14      15      16      17
// void kernel_dgemm_nn_4x4_gen_lib4(int k, double *alpha, double *A, int offB, double *B, int sdb, double *beta, int offC, double *C, int sdc, int offD, double *D, int sdd, int m0, int m1, int n0, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nn_4x4_gen_lib4)
	
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
	INNER_EDGE_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_edge_dgemm_nn_4x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_4x4_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12 // offsetC
	movq	ARG9, %r13 // C
	movq	ARG10, %r14 // sdc
	sall	$ 5, %r14d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_GEN_LIB4
#else
	CALL(inner_scale_ab_4x4_gen_lib4)
#endif


	// store n gen

	movq	ARG11, %r10 // offsetD
	movq	ARG12, %r11 // D
	movq	ARG13, %r12 // sdd
	sall	$ 5, %r12d // 4*sdb*sizeof(double)
	movq	ARG14, %r13 // m0
	movq	ARG15, %r14 // m1
	movq	ARG16, %r15 // n0
	movq	ARG17, %rax // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_GEN_LIB4
#else
	CALL(inner_store_4x4_gen_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nn_4x4_gen_lib4)





//                               1      2              3          4            5          6        7             8          9
// void kernel_dgemm_tt_4x4_lib4(int k, double *alpha, int offsetA, double *A, int sda, double *B, double *beta, double *C, double *D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_tt_4x4_lib4)
	
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
	INNER_EDGE_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_edge_dgemm_nn_4x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_4x4_lib4)
#endif

#if MACRO_LEVEL>=1
	INNER_TRAN_4X4_LIB4
#else
	CALL(inner_tran_4x4_lib4)
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
	CALL(inner_scale_ab_4x4_lib4)
#endif


	// store n

	movq	ARG9, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
	CALL(inner_store_4x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_tt_4x4_lib4)





//                                  1      2              3          4            5          6        7             8          9          10      11
// void kernel_dgemm_tt_4x4_vs_lib4(int k, double *alpha, int offsetA, double *A, int sda, double *B, double *beta, double *C, double *D, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_tt_4x4_vs_lib4)
	
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
	INNER_EDGE_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_edge_dgemm_nn_4x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_4x4_lib4)
#endif

#if MACRO_LEVEL>=1
	INNER_TRAN_4X4_LIB4
#else
	CALL(inner_tran_4x4_lib4)
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
	CALL(inner_scale_ab_4x4_lib4)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // m1
	movq	ARG11, %r12 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB4
#else
	CALL(inner_store_4x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_tt_4x4_vs_lib4)





//                                   1      2              3          4         5          6        7             8         9          10       11        12         13       14      15      16      17
// void kernel_dgemm_tt_4x4_gen_lib4(int k, double *alpha, int offsetA, double *A, int sda, double *B, double *beta, int offC, double *C, int sdc, int offD, double *D, int sdd, int m0, int m1, int n0, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_tt_4x4_gen_lib4)
	
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
	INNER_EDGE_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_edge_dgemm_nn_4x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_4x4_lib4)
#endif

#if MACRO_LEVEL>=1
	INNER_TRAN_4X4_LIB4
#else
	CALL(inner_tran_4x4_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12 // offsetC
	movq	ARG9, %r13 // C
	movq	ARG10, %r14 // sdc
	sall	$ 5, %r14d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_GEN_LIB4
#else
	CALL(inner_scale_ab_4x4_gen_lib4)
#endif


	// store n gen

	movq	ARG11, %r10 // offsetD
	movq	ARG12, %r11 // D
	movq	ARG13, %r12 // sdd
	sall	$ 5, %r12d // 4*sdb*sizeof(double)
	movq	ARG14, %r13 // m0
	movq	ARG15, %r14 // m1
	movq	ARG16, %r15 // n0
	movq	ARG17, %rax // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_GEN_LIB4
#else
	CALL(inner_store_4x4_gen_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_tt_4x4_gen_lib4)





//                                 rdi    rsi            rdx        rcx        r8            r9         rsp+8
// void kernel_dsyrk_nt_l_4x4_lib4(int k, double *alpha, double *A, double *B, double *beta, double *C, double *D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nt_l_4x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12   // C

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_AB_4X4_LIB4
#else
	CALL(inner_blend_scale_ab_4x4_lib4)
#endif


	// store n

	movq	ARG7, %r10 // D


#if MACRO_LEVEL>=1
	INNER_STORE_L_4X4_LIB4
#else
	CALL(inner_store_l_4x4_lib4)
#endif



	EPILOGUE

	ret

	FUN_END(kernel_dsyrk_nt_l_4x4_lib4)





//                                    rdi    rsi            rdx        rcx        r8            r9         rsp+8     rsp+16   rsp+24
// void kernel_dsyrk_nt_l_4x4_vs_lib4(int k, double *alpha, double *A, double *B, double *beta, double *C, double *D, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nt_l_4x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12   // C

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_AB_4X4_LIB4
#else
	CALL(inner_blend_scale_ab_4x4_lib4)
#endif


	// store n

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // km 
	movq	ARG9, %r12 // kn 


#if MACRO_LEVEL>=1
	INNER_STORE_L_4X4_VS_LIB4
#else
	CALL(inner_store_l_4x4_vs_lib4)
#endif



	EPILOGUE

	ret

	FUN_END(kernel_dsyrk_nt_l_4x4_vs_lib4)




#if 0

//                                     1      2              3          4          5             6            7          8        9            10         11       12      13      14      15
// void kernel_dsyrk_nt_l_4x4_gen_lib4(int k, double *alpha, double *A, double *B, double *beta, int offsetC, double *C, int sdc, int offsetD, double *D, int sdd, int m0, int m1, int n0, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nt_l_4x4_gen_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12 // offsetC
	movq	ARG7, %r13 // C
	movq	ARG8, %r14 // sdc
	sall	$ 5, %r14d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_AB_4X4_GEN_LIB4
#else
	CALL(inner_blend_scale_ab_4x4_gen_lib4)
#endif


	// store n gen

	movq	ARG9, %r10 // offsetD
	movq	ARG10, %r11 // D
	movq	ARG11, %r12 // sdd
	sall	$ 5, %r12d // 4*sdb*sizeof(double)
	movq	ARG12, %r13 // m0
	movq	ARG13, %r14 // m1
	movq	ARG14, %r15 // n0
	movq	ARG15, %rax // n1

#if MACRO_LEVEL>=1
	INNER_STORE_L_4X4_GEN_LIB4
#else
	CALL(inner_store_l_4x4_gen_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dsyrk_nt_l_4x4_gen_lib4)

#endif





//                                  1      2              3          4          5
// void kernel_dtrmm_nt_ru_4x4_lib4(int k, double *alpha, double *A, double *B, double *D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_ru_4x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt after initial triangle

	movq	ARG1, %r10 // k
	subl	$ 4, %r10d // k-4
	movq	ARG3, %r11 // A
	addq	$ 128, %r11 // A+4*bs
	movq	ARG4, %r12 // B
	addq	$ 128, %r12 // B+4*bs

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blend

#if MACRO_LEVEL>=1
	INNER_BLEND_4X4_LIB4
#else
	CALL(inner_blend_4x4_lib4)
#endif


	// initial triangle

	movq	ARG3, %r10
	movq	ARG4, %r11

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RU_4X4_LIB4
#else
	CALL(inner_edge_dtrmm_nt_ru_4x4_lib4)
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha

#if MACRO_LEVEL>=1
	INNER_SCALE_A0_4X4_LIB4
#else
	CALL(inner_scale_a0_4x4_lib4)
#endif


	// store n

	movq	ARG5, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
	CALL(inner_store_4x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_ru_4x4_lib4)





//                                     1      2              3          4          5          6       7
// void kernel_dtrmm_nt_ru_4x4_vs_lib4(int k, double *alpha, double *A, double *B, double *D, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_ru_4x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt after initial triangle

	movq	ARG1, %r10 // k
	subl	$ 4, %r10d // k-4
	movq	ARG3, %r11 // A
	addq	$ 128, %r11 // A+4*bs
	movq	ARG4, %r12 // B
	addq	$ 128, %r12 // B+4*bs

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blender nn

#if MACRO_LEVEL>=1
	INNER_BLEND_4X4_LIB4
#else
	CALL(inner_blend_4x4_lib4)
#endif


	// initial triangle

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // B

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RU_4X4_VS_LIB4
#else
	CALL(inner_edge_dtrmm_nt_ru_4x4_vs_lib4)
#endif


	// call inner loader nn

	movq	ARG2, %r10 // alpha

#if MACRO_LEVEL>=1
	INNER_SCALE_A0_4X4_LIB4
#else
	CALL(inner_scale_a0_4x4_lib4)
#endif


	// store n

	movq	ARG5, %r10 // D
	movq	ARG6, %r11 // km 
	movq	ARG7, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB4
#else
	CALL(inner_store_4x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_ru_4x4_vs_lib4)





//                                  1      2          3          4          5          6
// void kernel_dpotrf_nt_l_4x4_lib4(int k, double *A, double *B, double *C, double *D, double *inv_diag_D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpotrf_nt_l_4x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG4, %r10 // C

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_M11_4X4_LIB4
#else
	CALL(inner_blend_scale_m11_4x4_lib4)
#endif


	// factorization

	movq	ARG6, %r10  // inv_diag_D 
	movl	$ 4, %r11d // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_4X4_VS_LIB4
#else
	CALL(inner_edge_dpotrf_4x4_vs_lib4)
#endif


	// store

	movq	ARG5, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_L_4X4_LIB4
#else
	CALL(inner_store_l_4x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dpotrf_nt_l_4x4_lib4)





//                                     edi    rsi        rdx        ecx        r8         r9                  rsp+8   rsp+16
// void kernel_dpotrf_nt_l_4x4_vs_lib4(int k, double *A, double *B, double *C, double *D, double *inv_diag_D, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpotrf_nt_l_4x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG4, %r10 // C

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_M11_4X4_LIB4
#else
	CALL(inner_blend_scale_m11_4x4_lib4)
#endif


	// factorization

	movq	ARG6, %r10  // inv_diag_D 
	movq	ARG8, %r11 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_4X4_VS_LIB4
#else
	CALL(inner_edge_dpotrf_4x4_vs_lib4)
#endif


	// store

	movq	ARG5, %r10 // D
	movq	ARG7, %r11 // km 
	movq	ARG8, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_L_4X4_VS_LIB4
#else
	CALL(inner_store_l_4x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dpotrf_nt_l_4x4_vs_lib4)





//                                        edi     rsi         rdx         ecx     r8          r9          rsp+8      rsp+16     rsp+24
// void kernel_dsyrk_dpotrf_nt_l_4x4_lib4(int kp, double *Ap, double *Bp, int km, double *Am, double *Bm, double *C, double *D, double *inv_diag_D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_dpotrf_nt_l_4x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG2, %r11  // Ap
	movq	ARG3, %r12  // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// change sign
	NEG_ACC


	// call inner dgemm kernel nt sub

	movq	ARG4, %r10 // km
	movq	ARG5, %r11   // Am
	movq	ARG6, %r12   // Bm

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG7, %r10   // C

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_M11_4X4_LIB4
#else
	CALL(inner_blend_scale_m11_4x4_lib4)
#endif


	// factorization

	movq	ARG9, %r10  // inv_diag_D 
	movl	$ 4, %r11d

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_4X4_VS_LIB4
#else
	CALL(inner_edge_dpotrf_4x4_vs_lib4)
#endif


	// store

	movq	ARG8, %r10  // D 

#if MACRO_LEVEL>=1
	INNER_STORE_L_4X4_LIB4
#else
	CALL(inner_store_l_4x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dsyrk_dpotrf_nt_l_4x4_lib4)





//                                           edi     rsi         rdx         ecx     r8          r9          rsp+8      rsp+16     rsp+24              rsp+32  rsp+40
// void kernel_dsyrk_dpotrf_nt_l_4x4_vs_lib4(int kp, double *Ap, double *Bp, int km, double *Am, double *Bm, double *C, double *D, double *inv_diag_D, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_dpotrf_nt_l_4x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG2, %r11  // Ap
	movq	ARG3, %r12  // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// change sign
	NEG_ACC


	// call inner dgemm kernel nt sub

	movq	ARG4, %r10 // km
	movq	ARG5, %r11   // Am
	movq	ARG6, %r12   // Bm

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG7, %r10   // C

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_M11_4X4_LIB4
#else
	CALL(inner_blend_scale_m11_4x4_lib4)
#endif


	// factorization

	movq	ARG9, %r10  // inv_diag_D 
	movq	ARG11, %r11 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_4X4_VS_LIB4
#else
	CALL(inner_edge_dpotrf_4x4_vs_lib4)
#endif


	// store

	movq	ARG8, %r10  // D 
	movq	ARG10, %r11 // km 
	movq	ARG11, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_L_4X4_VS_LIB4
#else
	CALL(inner_store_l_4x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dsyrk_dpotrf_nt_l_4x4_vs_lib4)





//                                      1      2          3          4             5          6          7          8
// void kernel_dtrsm_nt_rl_inv_4x4_lib4(int k, double *A, double *B, double *beta, double *C, double *D, double *E, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_inv_4x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG4, %r10 // beta
	movq	ARG5, %r11 // C

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_M1B_4X4_LIB4
#else
	CALL(inner_blend_scale_m1b_4x4_lib4)
#endif


	// solve

	movq	ARG7, %r10  // E 
	movq	ARG8, %r11  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_4X4_LIB4
#else
	CALL(inner_edge_dtrsm_rlt_inv_4x4_lib4)
#endif


	// store

	movq	ARG6, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
	CALL(inner_store_4x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_4x4_lib4)





//                                            1       2           3           4       5           6           7          8          9          10
// void kernel_dgemm_dtrsm_nt_rl_inv_4x4_lib4(int kp, double *Ap, double *Bp, int km, double *Am, double *Bm, double *C, double *D, double *E, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_dtrsm_nt_rl_inv_4x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG2, %r11  // Ap
	movq	ARG3, %r12  // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// change sign
	NEG_ACC


	// call inner dgemm kernel nt sub

	movq	ARG4, %r10 // km
	movq	ARG5, %r11   // Am
	movq	ARG6, %r12   // Bm

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG7, %r10   // C

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_M11_4X4_LIB4
#else
	CALL(inner_blend_scale_m11_4x4_lib4)
#endif


	// solve

	movq	ARG9, %r10  // E 
	movq	ARG10, %r11  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_4X4_LIB4
#else
	CALL(inner_edge_dtrsm_rlt_inv_4x4_lib4)
#endif


	// store

	movq	ARG8, %r10   // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
	CALL(inner_store_4x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_dtrsm_nt_rl_inv_4x4_lib4)





//                                         1      2          3          4             5          6          7          8                   9       10
// void kernel_dtrsm_nt_rl_inv_4x4_vs_lib4(int k, double *A, double *B, double *beta, double *C, double *D, double *E, double *inv_diag_E, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_inv_4x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blender_loader nn // TODO scale gen

	movq	ARG4, %r10 // beta
	movq	ARG5, %r11 // C

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_M1B_4X4_LIB4
#else
	CALL(inner_blend_scale_m1b_4x4_lib4)
#endif


	// solve

	movq	ARG7, %r10  // E 
	movq	ARG8, %r11  // inv_diag_E 
	movq	ARG10, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_4X4_VS_LIB4
#else
	CALL(inner_edge_dtrsm_rlt_inv_4x4_vs_lib4)
#endif


	// store

	movq	ARG6, %r10 // D
	movq	ARG9, %r11 // km 
	movq	ARG10, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB4
#else
	CALL(inner_store_4x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_4x4_vs_lib4)





//                                               edi     rsi         rdx         ecx     r8          r9          rsp+8      rsp+16     rsp+24     rsp+32              rsp+40  rsp+48
// void kernel_dgemm_dtrsm_nt_rl_inv_4x4_vs_lib4(int kp, double *Ap, double *Bp, int km, double *Am, double *Bm, double *C, double *D, double *E, double *inv_diag_E, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_dtrsm_nt_rl_inv_4x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG2, %r11  // Ap
	movq	ARG3, %r12  // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// change sign
	NEG_ACC


	// call inner dgemm kernel nt sub

	movq	ARG4, %r10 // km
	movq	ARG5, %r11   // Am
	movq	ARG6, %r12   // Bm

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG7, %r10  // C 

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_M11_4X4_LIB4
#else
	CALL(inner_blend_scale_m11_4x4_lib4)
#endif


	// solve

	movq	ARG9, %r10  // E 
	movq	ARG10, %r11  // inv_diag_E 
	movq	ARG12, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_4X4_VS_LIB4
#else
	CALL(inner_edge_dtrsm_rlt_inv_4x4_vs_lib4)
#endif


	// store

	movq	ARG8, %r10 // D 
	movq	ARG11, %r11 // km 
	movq	ARG12, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB4
#else
	CALL(inner_store_4x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_dtrsm_nt_rl_inv_4x4_vs_lib4)





//                                  1      2              3          4            5          6        7
// void kernel_dtrmm_nn_rl_4x4_lib4(int k, double *alpha, double *A, int offsetB, double *B, int sdb, double *D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nn_rl_4x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC



	// initial triangle

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG5, %r12 // B
	movq	ARG6, %r13 // sdb
	sall	$ 5, %r13d // 4*sdb*sizeof(double)
	movq	ARG4, %r14 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NN_RL_4X4_LIB4
#else
	CALL(inner_edge_dtrmm_nn_rl_4x4_lib4)
#endif

	// call inner dgemm kernel nt after initial triangle

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_4x4_lib4)
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha

#if MACRO_LEVEL>=1
	INNER_SCALE_A0_4X4_LIB4
#else
	CALL(inner_scale_a0_4x4_lib4)
#endif


	// store n

	movq	ARG7, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
	CALL(inner_store_4x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nn_rl_4x4_lib4)





//                                     1      2              3          4            5          6        7          8       9
// void kernel_dtrmm_nn_rl_4x4_vs_lib4(int k, double *alpha, double *A, int offsetB, double *B, int sdb, double *D, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nn_rl_4x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC



	// initial triangle

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG5, %r12 // B
	movq	ARG6, %r13 // sdb
	sall	$ 5, %r13d // 4*sdb*sizeof(double)
	movq	ARG4, %r14 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NN_RL_4X4_VS_LIB4
#else
	CALL(inner_edge_dtrmm_nn_rl_4x4_vs_lib4)
#endif

	// call inner dgemm kernel nt after initial triangle

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_4x4_lib4)
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha

#if MACRO_LEVEL>=1
	INNER_SCALE_A0_4X4_LIB4
#else
	CALL(inner_scale_a0_4x4_lib4)
#endif


	// store n

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // m1
	movq	ARG9, %r12 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB4
#else
	CALL(inner_store_4x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nn_rl_4x4_vs_lib4)





//                                      1      2              3          4            5          6        7            8          9        10      11      12      13
// void kernel_dtrmm_nn_rl_4x4_gen_lib4(int k, double *alpha, double *A, int offsetB, double *B, int sdb, int offsetD, double *D, int sdd, int m0, int m1, int n0, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nn_rl_4x4_gen_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// initial triangle

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG5, %r12 // B
	movq	ARG6, %r13 // sdb
	sall	$ 5, %r13d // 4*sdb*sizeof(double)
	movq	ARG4, %r14 // B

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NN_RL_4X4_VS_LIB4
#else
	CALL(inner_edge_dtrmm_nn_rl_4x4_vs_lib4)
#endif

	// call inner dgemm kernel nt after initial triangle

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_4x4_lib4)
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha

#if MACRO_LEVEL>=1
	INNER_SCALE_A0_4X4_LIB4
#else
	CALL(inner_scale_a0_4x4_lib4)
#endif


	// store n

	movq	ARG7, %r10 // offsetD
	movq	ARG8, %r11 // D
	movq	ARG9, %r12 // sdd
	sall	$ 5, %r12d // 4*sdb*sizeof(double)
	movq	ARG10, %r13 // m0
	movq	ARG11, %r14 // m1
	movq	ARG12, %r15 // n0
	movq	ARG13, %rax // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_GEN_LIB4
#else
	CALL(inner_store_4x4_gen_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nn_rl_4x4_gen_lib4)





//#if defined(BLAS_API)
#if ( defined(BLAS_API) | ( defined(LA_HIGH_PERFORMANCE) & defined(MF_COLMAJ) ) )

#include "kernel_dgemm_4x4_lib.S"

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


#if defined(OS_LINUX)
	.section	.note.GNU-stack,"",@progbits
#elif defined(OS_MAC)
	.subsections_via_symbols
#endif

