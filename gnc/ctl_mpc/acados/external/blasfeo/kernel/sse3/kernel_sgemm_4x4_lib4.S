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
	movapd	%xmm0, %xmm3
//#define NEG_ACC \
//	movapd		.LC11(%rip), %xmm15; \
//	xorpd		%xmm15, %xmm0; \
//	xorpd		%xmm15, %xmm1; \
//	xorpd		%xmm15, %xmm2; \
//	xorpd		%xmm15, %xmm3; \
//	xorpd		%xmm15, %xmm4; \
//	xorpd		%xmm15, %xmm5; \
//	xorpd		%xmm15, %xmm6; \
//	xorpd		%xmm15, %xmm7

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
	movapd	%xmm0, %xmm3
//#define NEG_ACC \
//	movapd		LC11(%rip), %xmm15; \
//	xorpd		%xmm15, %xmm0; \
//	xorpd		%xmm15, %xmm1; \
//	xorpd		%xmm15, %xmm2; \
//	xorpd		%xmm15, %xmm3; \
//	xorpd		%xmm15, %xmm4; \
//	xorpd		%xmm15, %xmm5; \
//	xorpd		%xmm15, %xmm6; \
//	xorpd		%xmm15, %xmm7

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
	movapd	%xmm0, %xmm3
//#define NEG_ACC \
//	movapd		.LC11(%rip), %xmm15; \
//	xorpd		%xmm15, %xmm0; \
//	xorpd		%xmm15, %xmm1; \
//	xorpd		%xmm15, %xmm2; \
//	xorpd		%xmm15, %xmm3; \
//	xorpd		%xmm15, %xmm4; \
//	xorpd		%xmm15, %xmm5; \
//	xorpd		%xmm15, %xmm6; \
//	xorpd		%xmm15, %xmm7

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
// xmm0  <- []
// xmm1  <- []
// xmm2  <- []
// xmm3  <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+4*k*sizeof(float)
// r12   <- B+4*k*sizeof(float)
// xmm0  <- []
// xmm1  <- []
// xmm2  <- []
// xmm3  <- []

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_SGEMM_NT_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_sgemm_nt_4x4_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	// zero temp accumulators
	xorpd		%xmm10, %xmm10
	movapd		%xmm10, %xmm11

	// preload
	movaps		0(%r11), %xmm12
	movaps		16(%r11), %xmm13
	movaps		0(%r12), %xmm8


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	addps		%xmm10, %xmm2
	pshufd		$ 0x39, %xmm8, %xmm9
	mulps		%xmm12, %xmm8

	addps		%xmm11, %xmm3
	pshufd		$ 0x39, %xmm9, %xmm10
	mulps		%xmm12, %xmm9

	addps		%xmm8, %xmm0
	movaps		16(%r12), %xmm8
	pshufd		$ 0x39, %xmm10, %xmm11
	mulps		%xmm12, %xmm10

	addps		%xmm9, %xmm1
	mulps		%xmm12, %xmm11
	movaps		32(%r11), %xmm12


	// unroll 1
	addps		%xmm10, %xmm2
	pshufd		$ 0x39, %xmm8, %xmm9
	mulps		%xmm13, %xmm8
	subl		$ 4, %r10d

	addps		%xmm11, %xmm3
	pshufd		$ 0x39, %xmm9, %xmm10
	mulps		%xmm13, %xmm9

	addps		%xmm8, %xmm0
	movaps		32(%r12), %xmm8
	pshufd		$ 0x39, %xmm10, %xmm11
	mulps		%xmm13, %xmm10

	addps		%xmm9, %xmm1
	mulps		%xmm13, %xmm11
	movaps		48(%r11), %xmm13


	// unroll 2
	addps		%xmm10, %xmm2
	pshufd		$ 0x39, %xmm8, %xmm9
	mulps		%xmm12, %xmm8

	addps		%xmm11, %xmm3
	pshufd		$ 0x39, %xmm9, %xmm10
	mulps		%xmm12, %xmm9

	addps		%xmm8, %xmm0
	movaps		48(%r12), %xmm8
	pshufd		$ 0x39, %xmm10, %xmm11
	mulps		%xmm12, %xmm10

	addps		%xmm9, %xmm1
	mulps		%xmm12, %xmm11
	movaps		64(%r11), %xmm12


	// unroll 3
	addps		%xmm10, %xmm2
	pshufd		$ 0x39, %xmm8, %xmm9
	mulps		%xmm13, %xmm8

	addps		%xmm11, %xmm3
	pshufd		$ 0x39, %xmm9, %xmm10
	mulps		%xmm13, %xmm9

	addps		%xmm8, %xmm0
	movaps		64(%r12), %xmm8
	pshufd		$ 0x39, %xmm10, %xmm11
	mulps		%xmm13, %xmm10

	addps		%xmm9, %xmm1
	mulps		%xmm13, %xmm11
	movaps		80(%r11), %xmm13

	addq		$ 64, %r11 // A += 16
	addq		$ 64, %r12 // B += 16

	cmpl		$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1


	// unroll 0
	addps		%xmm10, %xmm2
	pshufd		$ 0x39, %xmm8, %xmm9
	mulps		%xmm12, %xmm8

	addps		%xmm11, %xmm3
	pshufd		$ 0x39, %xmm9, %xmm10
	mulps		%xmm12, %xmm9

	addps		%xmm8, %xmm0
	movaps		16(%r12), %xmm8
	pshufd		$ 0x39, %xmm10, %xmm11
	mulps		%xmm12, %xmm10

	addps		%xmm9, %xmm1
	mulps		%xmm12, %xmm11
	movaps		32(%r11), %xmm12


	// unroll 1
	addps		%xmm10, %xmm2
	pshufd		$ 0x39, %xmm8, %xmm9
	mulps		%xmm13, %xmm8
	subl		$ 4, %r10d

	addps		%xmm11, %xmm3
	pshufd		$ 0x39, %xmm9, %xmm10
	mulps		%xmm13, %xmm9

	addps		%xmm8, %xmm0
	movaps		32(%r12), %xmm8
	pshufd		$ 0x39, %xmm10, %xmm11
	mulps		%xmm13, %xmm10

	addps		%xmm9, %xmm1
	mulps		%xmm13, %xmm11
	movaps		48(%r11), %xmm13


	// unroll 2
	addps		%xmm10, %xmm2
	pshufd		$ 0x39, %xmm8, %xmm9
	mulps		%xmm12, %xmm8

	addps		%xmm11, %xmm3
	pshufd		$ 0x39, %xmm9, %xmm10
	mulps		%xmm12, %xmm9

	addps		%xmm8, %xmm0
	movaps		48(%r12), %xmm8
	pshufd		$ 0x39, %xmm10, %xmm11
	mulps		%xmm12, %xmm10

	addps		%xmm9, %xmm1
	mulps		%xmm12, %xmm11
//	movaps		64(%r11), %xmm12


	// unroll 3
	addps		%xmm10, %xmm2
	pshufd		$ 0x39, %xmm8, %xmm9
	mulps		%xmm13, %xmm8

	addps		%xmm11, %xmm3
	pshufd		$ 0x39, %xmm9, %xmm10
	mulps		%xmm13, %xmm9

	addps		%xmm8, %xmm0
//	movaps		64(%r12), %xmm8
	pshufd		$ 0x39, %xmm10, %xmm11
	mulps		%xmm13, %xmm10

	addps		%xmm9, %xmm1
	mulps		%xmm13, %xmm11
//	movaps		80(%r11), %xmm13

	addq		$ 64, %r11 // A += 16
	addq		$ 64, %r12 // B += 16

	// clean accumulators
	addps		%xmm10, %xmm2
	addps		%xmm11, %xmm3


	jmp		2f


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	

	// unroll 0
	movaps		0(%r11), %xmm12
	movaps		0(%r12), %xmm8

	addps		%xmm10, %xmm2
	pshufd		$ 0x39, %xmm8, %xmm9
	mulps		%xmm12, %xmm8

	addps		%xmm11, %xmm3
	pshufd		$ 0x39, %xmm9, %xmm10
	mulps		%xmm12, %xmm9

	addps		%xmm8, %xmm0
	pshufd		$ 0x39, %xmm10, %xmm11
	mulps		%xmm12, %xmm10

	addps		%xmm9, %xmm1
	mulps		%xmm12, %xmm11

	subl	$ 1, %r10d
	addq	$ 16, %r12
	addq	$ 16, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


	// clean accumulators
	addps		%xmm10, %xmm2
	addps		%xmm11, %xmm3


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_sgemm_nt_4x4_lib4)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d   <- k
// r11   <- A
// r12   <- B
// r13   <- 4*sdb*sizeof(float)
// xmm0  <- [d00 d10 d20 d30]
// xmm1  <- [d01 d11 d21 d31]
// xmm2  <- [d02 d12 d22 d32]
// xmm3  <- [d03 d13 d23 d33]

//
// output arguments:
// r10d  <- 0
// r11   <- A+4*k*sizeof(float)
// r12   <- B+(k/4)*sdb*sizeof(float)+(k%4)
// r13   <- 4*sdb*sizeof(float)
// xmm0  <- [d00 d10 d20 d30]
// xmm1  <- [d01 d11 d21 d31]
// xmm2  <- [d02 d12 d22 d32]
// xmm3  <- [d03 d13 d23 d33]

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_SGEMM_NN_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_sgemm_nn_4x4_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	// prefetch


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	prefetcht0	0(%r12, %r13, 2) // software prefetch

	// unroll 0
	movaps		0(%r11), %xmm12

	movaps		0(%r12), %xmm8
	pshufd		$ 0x00, %xmm8, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm0

	movaps		16(%r12), %xmm9
	pshufd		$ 0x00, %xmm9, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm1

	movaps		32(%r12), %xmm10
	pshufd		$ 0x00, %xmm10, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm2

	movaps		48(%r12), %xmm11
	pshufd		$ 0x00, %xmm11, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm3


	// unroll 1
	movaps		16(%r11), %xmm12

	pshufd		$ 0x55, %xmm8, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm0

	pshufd		$ 0x55, %xmm9, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm1

	pshufd		$ 0x55, %xmm10, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm2

	pshufd		$ 0x55, %xmm11, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm3


	// unroll 2
	movaps		32(%r11), %xmm12

	pshufd		$ 0xaa, %xmm8, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm0

	pshufd		$ 0xaa, %xmm9, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm1

	pshufd		$ 0xaa, %xmm10, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm2

	pshufd		$ 0xaa, %xmm11, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm3

	
	// unroll 3
	movaps		48(%r11), %xmm12

	pshufd		$ 0xff, %xmm8, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm0

	pshufd		$ 0xff, %xmm9, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm1

	pshufd		$ 0xff, %xmm10, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm2

	pshufd		$ 0xff, %xmm11, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm3


	subl	$ 4, %r10d
	addq	$ 64, %r11 // B += ...
	addq	%r13, %r12 // B += ...

	cmpl		$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	movaps		0(%r11), %xmm12

	movaps		0(%r12), %xmm8
	pshufd		$ 0x00, %xmm8, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm0

	movaps		16(%r12), %xmm9
	pshufd		$ 0x00, %xmm9, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm1

	movaps		32(%r12), %xmm10
	pshufd		$ 0x00, %xmm10, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm2

	movaps		48(%r12), %xmm11
	pshufd		$ 0x00, %xmm11, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm3


	// unroll 1
	movaps		16(%r11), %xmm12

	pshufd		$ 0x55, %xmm8, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm0

	pshufd		$ 0x55, %xmm9, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm1

	pshufd		$ 0x55, %xmm10, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm2

	pshufd		$ 0x55, %xmm11, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm3


	// unroll 2
	movaps		32(%r11), %xmm12

	pshufd		$ 0xaa, %xmm8, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm0

	pshufd		$ 0xaa, %xmm9, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm1

	pshufd		$ 0xaa, %xmm10, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm2

	pshufd		$ 0xaa, %xmm11, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm3

	
	// unroll 3
	movaps		48(%r11), %xmm12

	pshufd		$ 0xff, %xmm8, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm0

	pshufd		$ 0xff, %xmm9, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm1

	pshufd		$ 0xff, %xmm10, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm2

	pshufd		$ 0xff, %xmm11, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm3


	subl	$ 4, %r10d
	addq	$ 64, %r11 // B += ...
	addq	%r13, %r12 // B += ...

	// clean accumulators


	jmp		2f


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	

	// unroll 0
	movaps		0(%r11), %xmm12

	movss		0(%r12), %xmm8
	pshufd		$ 0x00, %xmm8, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm0

	movss		16(%r12), %xmm9
	pshufd		$ 0x00, %xmm9, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm1

	movss		32(%r12), %xmm10
	pshufd		$ 0x00, %xmm10, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm2

	movss		48(%r12), %xmm11
	pshufd		$ 0x00, %xmm11, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm3


	subl	$ 1, %r10d
	addq	$ 16, %r11
	addq	$ 4, %r12

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


	// clean accumulators


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_sgemm_nn_4x4_lib4)
#endif





// common inner routine with file scope
//
// edge for B unaligned
//
// input arguments:
// r10   <- k
// r11   <- A
// r12   <- B
// r13   <- bs*sdb*sizeof(float)
// r14   <- offB
// xmm0  <- [d00 d10 d20 d30]
// xmm1  <- [d01 d11 d21 d31]
// xmm2  <- [d02 d12 d22 d32]
// xmm3  <- [d03 d13 d23 d33]

//
// output arguments:
// r10   <- k-(4-offB)
// r11   <- A+(4-offB)*bs*sizeof(float)
// r12   <- B-offB+bs*sdb*sizeof(float)
// r13   <- bs*sdb*sizeof(float)
// r14   <- offB
// xmm0  <- [d00 d10 d20 d30]
// xmm1  <- [d01 d11 d21 d31]
// xmm2  <- [d02 d12 d22 d32]
// xmm3  <- [d03 d13 d23 d33]


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_SGEMM_NN_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_sgemm_nn_4x4_lib4)
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
	sall			$ 2, %eax // offsetB*sizeof(float)
	addq			%rax, %r12 // B+offsetB*sizeof(float)

1:
	movaps		0(%r11), %xmm12

	movss		0(%r12), %xmm8
	pshufd		$ 0x00, %xmm8, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm0

	movss		16(%r12), %xmm9
	pshufd		$ 0x00, %xmm9, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm1

	movss		32(%r12), %xmm10
	pshufd		$ 0x00, %xmm10, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm2

	movss		48(%r12), %xmm11
	pshufd		$ 0x00, %xmm11, %xmm14
	mulps		%xmm12, %xmm14
	addps		%xmm14, %xmm3


	subl			$ 1, %r10d // k-1
	subl			$ 1, %r15d // kend-1
	addq			$ 16, %r11 // A+1*bs*sizeof(float)
	addq			$ 4, %r12 // B+1*sizeof(float)

	cmpl			$ 0, %r15d
	jg				1b

	cmpl			$ 0, %r10d
	jle				2f // end

	addq			%r13, %r12
	subq			$ 16, %r12 // B+bs*(sdb-1)*sizeof(float)

2:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_sgemm_nn_4x4_lib4)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// xmm0  <- []
// xmm1  <- []
// xmm2  <- []
// xmm3  <- []

// output arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// xmm0  <- [d00 d10 d20 d30]
// xmm1  <- [d01 d11 d21 d31]
// xmm2  <- [d02 d12 d22 d32]
// xmm3  <- [d03 d13 d23 d33]

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_SCALE_AB_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_blend_scale_ab_4x4_lib4)
#endif

	movaps		%xmm1, %xmm12
	shufps		$ 0xd8, %xmm0, %xmm1
	shufps		$ 0xd8, %xmm3, %xmm0
	shufps		$ 0xd8, %xmm2, %xmm3
	shufps		$ 0xd8, %xmm12, %xmm2

	movaps		%xmm0, %xmm12
	shufps		$ 0xd8, %xmm2, %xmm0
	shufps		$ 0xd8, %xmm12, %xmm2
	movaps		%xmm1, %xmm12
	shufps		$ 0xd8, %xmm3, %xmm1
	shufps		$ 0xd8, %xmm12, %xmm3

	// alpha
	movss		0(%r10), %xmm15
	shufps		$ 0x00, %xmm15, %xmm15

	mulps		%xmm15, %xmm0
	mulps		%xmm15, %xmm1
	mulps		%xmm15, %xmm2
	mulps		%xmm15, %xmm3

	// beta
	movss		0(%r11), %xmm14
	shufps		$ 0x00, %xmm14, %xmm14

	xorps		%xmm15, %xmm15 // 0.0
	ucomiss		%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	movaps		0(%r12), %xmm15
	mulps		%xmm14, %xmm15
	addps		%xmm15, %xmm0
	movaps		16(%r12), %xmm15
	mulps		%xmm14, %xmm15
	addps		%xmm15, %xmm1
	movaps		32(%r12), %xmm15
	mulps		%xmm14, %xmm15
	addps		%xmm15, %xmm2
	movaps		48(%r12), %xmm15
	mulps		%xmm14, %xmm15
	addps		%xmm15, %xmm3

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_blend_scale_ab_4x4_lib4)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// xmm0  <- [d00 d10 d20 d30]
// xmm1  <- [d01 d11 d21 d31]
// xmm2  <- [d02 d12 d22 d32]
// xmm3  <- [d03 d13 d23 d33]

// output arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// xmm0  <- [d00 d10 d20 d30]
// xmm1  <- [d01 d11 d21 d31]
// xmm2  <- [d02 d12 d22 d32]
// xmm3  <- [d03 d13 d23 d33]

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_4x4_lib4)
#endif

	// alpha
	movss		0(%r10), %xmm15
	shufps		$ 0x00, %xmm15, %xmm15

	mulps		%xmm15, %xmm0
	mulps		%xmm15, %xmm1
	mulps		%xmm15, %xmm2
	mulps		%xmm15, %xmm3

	// beta
	movss		0(%r11), %xmm14
	shufps		$ 0x00, %xmm14, %xmm14

	xorps		%xmm15, %xmm15 // 0.0
	ucomiss		%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	movaps		0(%r12), %xmm15
	mulps		%xmm14, %xmm15
	addps		%xmm15, %xmm0
	movaps		16(%r12), %xmm15
	mulps		%xmm14, %xmm15
	addps		%xmm15, %xmm1
	movaps		32(%r12), %xmm15
	mulps		%xmm14, %xmm15
	addps		%xmm15, %xmm2
	movaps		48(%r12), %xmm15
	mulps		%xmm14, %xmm15
	addps		%xmm15, %xmm3

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_4x4_lib4)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// xmm0  <- [d00 d10 d20 d30]
// xmm1  <- [d01 d11 d21 d31]
// xmm2  <- [d02 d12 d22 d32]
// xmm3  <- [d03 d13 d23 d33]
//
// output arguments:
// r10  <- D
// xmm0  <- [d00 d10 d20 d30]
// xmm1  <- [d01 d11 d21 d31]
// xmm2  <- [d02 d12 d22 d32]
// xmm3  <- [d03 d13 d23 d33]

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_4x4_lib4)
#endif
	
	movapd %xmm0,   0(%r10)
	movapd %xmm1,  16(%r10)
	movapd %xmm2,  32(%r10)
	movapd %xmm3,  48(%r10)
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4x4_lib4)
#endif





//                               1      2             3         4         5            6         7
// void kernel_sgemm_nt_4x4_lib4(int k, float *alpha, float *A, float *B, float *beta, float *C, float *D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_nt_4x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_SGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_sgemm_nt_4x4_lib4)
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

	FUN_END(kernel_sgemm_nt_4x4_lib4)





//                               1      2              3          4            5          6        7             8          9
// void kernel_sgemm_nn_4x4_lib4(int k, float *alpha, float *A, int offsetB, float *B, int sdb, float *beta, float *C, float *D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_nn_4x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG5, %r12  // B
	movq	ARG6, %r13 // sdb
	sall	$ 4, %r13d // 4*sdb*sizeof(float)
	movq	ARG4, %r14 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_SGEMM_NN_4X4_LIB4
#else
	CALL(inner_edge_sgemm_nn_4x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_SGEMM_NN_4X4_LIB4
#else
	CALL(inner_kernel_sgemm_nn_4x4_lib4)
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

	FUN_END(kernel_sgemm_nn_4x4_lib4)





//#if defined(BLAS_API)
#if ( defined(BLAS_API) | ( defined(LA_HIGH_PERFORMANCE) & defined(MF_COLMAJ) ) )

#include "kernel_sgemm_4x4_lib.S"

#endif






