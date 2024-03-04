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
// r10d  <- k
// r11   <- A
// r12   <- x
// xmm0 <- [z0 z1]_a
// xmm1 <- [z2 z3]_a
// xmm2 <- [z0 z1]_b
// xmm3 <- [z2 z3]_b

//
// output arguments:
// r10d  <- 0
// r11   <- A+4*k*sizeof(double)
// r12   <- x+k*sizeof(double)
// xmm0 <- [z0 z1]_a
// xmm1 <- [z2 z3]_a
// xmm2 <- [z0 z1]_b
// xmm3 <- [z2 z3]_b

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMV_ADD_N_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemv_add_n_4_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	cmpl	$ 4, %r10d
	jl		0f // clean-up loop

	// main loop
	.p2align 3
1: // main loop
	
	movddup	0(%r12), %xmm12
	movapd	0(%r11), %xmm8
	mulpd	%xmm12, %xmm8
	addpd	%xmm8, %xmm0
	movapd	16(%r11), %xmm8
	mulpd	%xmm12, %xmm8
	addpd	%xmm8, %xmm1
	subl	$ 4, %r10d

	movddup	8(%r12), %xmm12
	movapd	32(%r11), %xmm8
	mulpd	%xmm12, %xmm8
	addpd	%xmm8, %xmm2
	movapd	48(%r11), %xmm8
	mulpd	%xmm12, %xmm8
	addpd	%xmm8, %xmm3
	
	movddup	16(%r12), %xmm12
	movapd	64(%r11), %xmm8
	mulpd	%xmm12, %xmm8
	addpd	%xmm8, %xmm0
	movapd	80(%r11), %xmm8
	mulpd	%xmm12, %xmm8
	addpd	%xmm8, %xmm1

	movddup	24(%r12), %xmm12
	movapd	96(%r11), %xmm8
	mulpd	%xmm12, %xmm8
	addpd	%xmm8, %xmm2
	movapd	112(%r11), %xmm8
	mulpd	%xmm12, %xmm8
	addpd	%xmm8, %xmm3
	
	addq	$ 128, %r11
	addq	$ 32, %r12
	
	cmpl	$ 3, %r10d

	jg		1b // main loop 


	// consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

0: // clean-up
	
	movddup	0(%r12), %xmm12
	movapd	0(%r11), %xmm8
	mulpd	%xmm12, %xmm8
	addpd	%xmm8, %xmm0
	movapd	16(%r11), %xmm8
	mulpd	%xmm12, %xmm8
	addpd	%xmm8, %xmm1

	addq	$ 32, %r11
	addq	$ 8, %r12
	
	subl	$ 1, %r10d
	cmpl	$ 0, %r10d

	jg		0b // clean

2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemv_add_n_4_lib4)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- bs*sda*sizeof(double) = 32*sda
// r13   <- x
// xmm0  <- [z0a z0b]
// xmm1  <- [z1a z1b]
// xmm2  <- [z2a z2b]
// xmm3  <- [z3a z3b]

//
// output arguments:
// r10d  <- 0
// r11   <- A+4*k*sizeof(double)
// r12   <- bs*sda*sizeof(double) = 32*sda
// r13   <- x+k*sizeof(double)
// xmm0  <- [z0a z0b]
// xmm1  <- [z1a z1b]
// xmm2  <- [z2a z2b]
// xmm3  <- [z3a z3b]

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMV_ADD_T_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemv_add_t_4_lib4)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	cmpl	$ 4, %r10d
	jl		0f // clean-up loop

	// main loop
	.p2align 3
1: // main loop
	
	movupd	0(%r13), %xmm12

	movapd	0(%r11), %xmm8
	mulpd	%xmm12, %xmm8
	addpd	%xmm8, %xmm0
	subl	$ 4, %r10d

	movapd	32(%r11), %xmm8
	mulpd	%xmm12, %xmm8
	addpd	%xmm8, %xmm1

	movapd	64(%r11), %xmm8
	mulpd	%xmm12, %xmm8
	addpd	%xmm8, %xmm2

	movapd	96(%r11), %xmm8
	mulpd	%xmm12, %xmm8
	addpd	%xmm8, %xmm3

	movupd	16(%r13), %xmm12

	movapd	16(%r11), %xmm8
	mulpd	%xmm12, %xmm8
	addpd	%xmm8, %xmm0

	movapd	48(%r11), %xmm8
	mulpd	%xmm12, %xmm8
	addpd	%xmm8, %xmm1

	movapd	80(%r11), %xmm8
	mulpd	%xmm12, %xmm8
	addpd	%xmm8, %xmm2

	movapd	112(%r11), %xmm8
	mulpd	%xmm12, %xmm8
	addpd	%xmm8, %xmm3

	addq	%r12, %r11
	addq	$ 32, %r13
	
	cmpl	$ 3, %r10d
	jg		1b // main loop 


	// consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

0: // clean-up
	
	movsd	0(%r13), %xmm12

	movsd	0(%r11), %xmm8
	mulsd	%xmm12, %xmm8
	addsd	%xmm8, %xmm0
	subl	$ 1, %r10d

	movsd	32(%r11), %xmm8
	mulsd	%xmm12, %xmm8
	addsd	%xmm8, %xmm1

	movsd	64(%r11), %xmm8
	mulsd	%xmm12, %xmm8
	addsd	%xmm8, %xmm2

	movsd	96(%r11), %xmm8
	mulsd	%xmm12, %xmm8
	addsd	%xmm8, %xmm3

	addq	$ 8, %r11
	addq	$ 8, %r13
	
	cmpl	$ 0, %r10d
	jg		0b // main loop 

	
2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemv_add_t_4_lib4)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- bs*sda*sizeof(double) = 32*sda
// r13   <- x_t
// r14   <- z_n
// xmm0  <- [z_t_0a z_t_0b]
// xmm1  <- [z_t_1a z_t_1b]
// xmm2  <- [z_t_2a z_t_2b]
// xmm3  <- [z_t_3a z_t_3b]
// xmm4  <- x_n_0
// xmm5  <- x_n_1
// xmm6  <- x_n_2
// xmm7  <- x_n_3

//
// output arguments:
// r10d  <- 0
// r11   <- A+4*k*sizeof(double)
// r12   <- bs*sda*sizeof(double) = 32*sda
// r13   <- x_t+k*sizeof(double)
// r14   <- z_n+k*sizeof(double)
// xmm0  <- [z_t_0a z_t_0b]
// xmm1  <- [z_t_1a z_t_1b]
// xmm2  <- [z_t_2a z_t_2b]
// xmm3  <- [z_t_3a z_t_3b]
// xmm4  <- x_n_0
// xmm5  <- x_n_1
// xmm6  <- x_n_2
// xmm7  <- x_n_3

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMV_ADD_NT_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemv_add_nt_4_lib4)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	cmpl	$ 4, %r10d
	jl		0f // clean-up loop

	// main loop
	.p2align 3
1: // main loop
	
	movupd	0(%r13), %xmm9
	movupd	16(%r13), %xmm10
	movupd	0(%r14), %xmm11
	movupd	16(%r14), %xmm12

	subl	$ 4, %r10d

	movapd	0(%r11), %xmm14
	movapd	%xmm14, %xmm15
	mulpd	%xmm9, %xmm14
	addpd	%xmm14, %xmm0
	mulpd	%xmm4, %xmm15
	addpd	%xmm15, %xmm11

	movapd	16(%r11), %xmm14
	movapd	%xmm14, %xmm15
	mulpd	%xmm10, %xmm14
	addpd	%xmm14, %xmm0
	mulpd	%xmm4, %xmm15
	addpd	%xmm15, %xmm12

	movapd	32(%r11), %xmm14
	movapd	%xmm14, %xmm15
	mulpd	%xmm9, %xmm14
	addpd	%xmm14, %xmm1
	mulpd	%xmm5, %xmm15
	addpd	%xmm15, %xmm11

	movapd	48(%r11), %xmm14
	movapd	%xmm14, %xmm15
	mulpd	%xmm10, %xmm14
	addpd	%xmm14, %xmm1
	mulpd	%xmm5, %xmm15
	addpd	%xmm15, %xmm12

	movapd	64(%r11), %xmm14
	movapd	%xmm14, %xmm15
	mulpd	%xmm9, %xmm14
	addpd	%xmm14, %xmm2
	mulpd	%xmm6, %xmm15
	addpd	%xmm15, %xmm11

	movapd	80(%r11), %xmm14
	movapd	%xmm14, %xmm15
	mulpd	%xmm10, %xmm14
	addpd	%xmm14, %xmm2
	mulpd	%xmm6, %xmm15
	addpd	%xmm15, %xmm12

	movapd	96(%r11), %xmm14
	movapd	%xmm14, %xmm15
	mulpd	%xmm9, %xmm14
	addpd	%xmm14, %xmm3
	mulpd	%xmm7, %xmm15
	addpd	%xmm15, %xmm11

	movapd	112(%r11), %xmm14
	movapd	%xmm14, %xmm15
	mulpd	%xmm10, %xmm14
	addpd	%xmm14, %xmm3
	mulpd	%xmm7, %xmm15
	addpd	%xmm15, %xmm12

	movupd	%xmm11, 0(%r14) 
	movupd	%xmm12, 16(%r14) 

	addq	%r12, %r11
	addq	$ 32, %r13
	addq	$ 32, %r14
	
	cmpl	$ 3, %r10d
	jg		1b // main loop 


	// consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

0: // clean-up
	
	movsd	0(%r13), %xmm9
	movsd	0(%r14), %xmm11

	subl	$ 1, %r10d

	movsd	0(%r11), %xmm14
	movsd	%xmm14, %xmm15
	mulsd	%xmm9, %xmm14
	addsd	%xmm14, %xmm0
	mulsd	%xmm4, %xmm15
	addsd	%xmm15, %xmm11

	movsd	32(%r11), %xmm14
	movsd	%xmm14, %xmm15
	mulsd	%xmm9, %xmm14
	addsd	%xmm14, %xmm1
	mulsd	%xmm5, %xmm15
	addsd	%xmm15, %xmm11

	movsd	64(%r11), %xmm14
	movsd	%xmm14, %xmm15
	mulsd	%xmm9, %xmm14
	addsd	%xmm14, %xmm2
	mulsd	%xmm6, %xmm15
	addsd	%xmm15, %xmm11

	movsd	96(%r11), %xmm14
	movsd	%xmm14, %xmm15
	mulsd	%xmm9, %xmm14
	addsd	%xmm14, %xmm3
	mulsd	%xmm7, %xmm15
	addsd	%xmm15, %xmm11

	movsd	%xmm11, 0(%r14) 

	addq	$ 8, %r11
	addq	$ 8, %r13
	addq	$ 8, %r14
	
	cmpl	$ 0, %r10d
	jg		0b // main loop 

2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemv_add_nt_4_lib4)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- bs*sda*sizeof(double) = 32*sda
// r13   <- x
// r14d  <- offA
// xmm0  <- [z0a z0b]
// xmm1  <- [z1a z1b]
// xmm2  <- [z2a z2b]
// xmm3  <- [z3a z3b]

//
// output arguments:
// r10d  <- 
// r11   <- 
// r12   <- 
// r13   <- 
// r14d  <- offA
// xmm0  <- [z0a z0b]
// xmm1  <- [z1a z1b]
// xmm2  <- [z2a z2b]
// xmm3  <- [z3a z3b]

#if MACRO_LEVEL>=2
	.macro INNER_EDGE_GEMV_ADD_T_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dgemv_add_t_4_lib4)
#endif

	cmpl			$ 0, %r14d				// offset==0
	jle				2f						// end

	cmpl			$ 0, %r10d				// k==0
	jle				2f						// end

	movl			$ 4, %r15d				// load 4
	subl			%r14d, %r15d			// 4-offsetA
	cmpl			%r10d, %r15d			// k > 4-offsetA
	cmovgl			%r10d, %r15d			// kend=min(k,4-offsetA)

//	movl			%r14d, %eax				// load offsetA
//	sall			$ 3, %eax				// offsetA*sizeof(double)
//	addq			%rax, %r11				// A+offsetA*sizeof(double)

1:
	movsd	0(%r13), %xmm12

	movsd	0(%r11), %xmm8
	mulsd	%xmm12, %xmm8
	addsd	%xmm8, %xmm0
	subl	$ 1, %r10d

	movsd	32(%r11), %xmm8
	mulsd	%xmm12, %xmm8
	addsd	%xmm8, %xmm1

	movsd	64(%r11), %xmm8
	mulsd	%xmm12, %xmm8
	addsd	%xmm8, %xmm2

	movsd	96(%r11), %xmm8
	mulsd	%xmm12, %xmm8
	addsd	%xmm8, %xmm3

	subl	$ 1, %r10d				// k=-1
	subl	$ 1, %r15d				// k_panel=-1
	addq	$ 8, %r11				// A=+bs
	addq	$ 8, %r13				// x=+1

	cmpl	$ 0, %r15d				// if k_panel=0
	jg		1b						// loop 1

	cmpl	$ 0, %r10d				// if k=0
	jle		2f						// end

	addq	%r12, %r11				// B=Boff+sdb*bs
	subq	$ 32, %r11				// B-=4*sizeof(double) (loop+offsetB)

2:

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_edge_dgemv_add_t_4_lib4)
#endif





// common inner routine with file scope
//
// input arguments:
// r10   <- kmax
// r11   <- A
// r12   <- bs*sda*sizeof(double) = 32*sda
// r13   <- x_t
// r14   <- z_n
// xmm0  <- [z_t_0a z_t_0b]
// xmm1  <- [z_t_1a z_t_1b]
// xmm2  <- [z_t_2a z_t_2b]
// xmm3  <- [z_t_3a z_t_3b]
// xmm4  <- x_n_0
// xmm5  <- x_n_1
// xmm6  <- x_n_2
// xmm7  <- x_n_3

//
// output arguments:
// r10   <- kmax-4
// r11   <- A+4*k*sizeof(double)
// r12   <- bs*sda*sizeof(double) = 32*sda
// r13   <- x_t+k*sizeof(double)
// r14   <- z_n+k*sizeof(double)
// xmm0  <- [z_t_0a z_t_0b]
// xmm1  <- [z_t_1a z_t_1b]
// xmm2  <- [z_t_2a z_t_2b]
// xmm3  <- [z_t_3a z_t_3b]
// xmm4  <- x_n_0
// xmm5  <- x_n_1
// xmm6  <- x_n_2
// xmm7  <- x_n_3

#if MACRO_LEVEL>=2
	.macro INNER_EDGE_DSYMV_ADD_NT_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dsymv_add_nt_4_lib4)
#endif

	xorpd	%xmm13, %xmm13

	movupd	0(%r13), %xmm9
	movupd	16(%r13), %xmm10
	movupd	0(%r14), %xmm11
	movupd	16(%r14), %xmm12

	// 0
	movapd	0(%r11), %xmm14
	movapd	%xmm14, %xmm15
	mulpd	%xmm9, %xmm14
	addpd	%xmm14, %xmm0
	movsd	%xmm13, %xmm15 //
	mulpd	%xmm4, %xmm15
	addpd	%xmm15, %xmm11

	movapd	16(%r11), %xmm14
	movapd	%xmm14, %xmm15
	mulpd	%xmm10, %xmm14
	addpd	%xmm14, %xmm0
	mulpd	%xmm4, %xmm15
	addpd	%xmm15, %xmm12

	// 1
	movapd	32(%r11), %xmm14
	movapd	%xmm14, %xmm15
	movsd	%xmm13, %xmm14 //
	mulpd	%xmm9, %xmm14
	addpd	%xmm14, %xmm1
//	movapd	%xmm13, %xmm15 //
//	mulpd	%xmm5, %xmm15
//	addpd	%xmm15, %xmm11

	movapd	48(%r11), %xmm14
	movapd	%xmm14, %xmm15
	mulpd	%xmm10, %xmm14
	addpd	%xmm14, %xmm1
	mulpd	%xmm5, %xmm15
	addpd	%xmm15, %xmm12

	// 2
//	movapd	64(%r11), %xmm14
//	movapd	%xmm14, %xmm15
//	movapd	%xmm13, %xmm14 //
//	mulpd	%xmm9, %xmm14
//	addpd	%xmm14, %xmm2
//	movapd	%xmm13, %xmm15 //
//	mulpd	%xmm6, %xmm15
//	addpd	%xmm15, %xmm11

	movapd	80(%r11), %xmm14
	movapd	%xmm14, %xmm15
	mulpd	%xmm10, %xmm14
	addpd	%xmm14, %xmm2
	movsd	%xmm13, %xmm15 //
	mulpd	%xmm6, %xmm15
	addpd	%xmm15, %xmm12

	// 3
//	movapd	96(%r11), %xmm14
//	movapd	%xmm14, %xmm15
//	movapd	%xmm13, %xmm14 //
//	mulpd	%xmm9, %xmm14
//	addpd	%xmm14, %xmm3
//	movapd	%xmm13, %xmm15 //
//	mulpd	%xmm7, %xmm15
//	addpd	%xmm15, %xmm11

	movapd	112(%r11), %xmm14
	movapd	%xmm14, %xmm15
	movsd	%xmm13, %xmm14 //
	mulpd	%xmm10, %xmm14
	addpd	%xmm14, %xmm3
//	movapd	%xmm13, %xmm15 //
//	mulpd	%xmm7, %xmm15
//	addpd	%xmm15, %xmm12

	movupd	%xmm11, 0(%r14) 
	movupd	%xmm12, 16(%r14) 

	addq	%r12, %r11
	addq	$ 32, %r13
	addq	$ 32, %r14
	
	subq	$ 4, %r10

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_edge_dsymv_add_nt_4_lib4)
#endif






#if 0

// common inner routine with file scope
//
// triangular substitution with vector RHS
//
// input arguments:
// r10  <- E
// r11  <- inv_diag_E
// xmm0 <- [z0 z1]
// xmm1 <- [z2 z3]
//
// output arguments:
// r10  <- E
// r11  <- inv_diag_E
// xmm0 <- [z0 z1]
// xmm1 <- [z2 z3]

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSV_LN_INV_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsv_ln_inv_4_lib4)
#endif
	
	xorpd			%xmm14, %xmm14

	movddup			0(%r11), %xmm12
	mulpd			%xmm0, %xmm12
	movsd			%xmm12, %xmm0

	movapd			0(%r10), %xmm13
	movsd			%xmm14, %xmm13
	movddup			%xmm0, %xmm12
	mulpd			%xmm13, %xmm12
	subpd			%xmm12, %xmm0
	movddup			8(%r11), %xmm12
	mulpd			%xmm0, %xmm12
	movhpd			%xmm12, %xmm0

	movapd			32(%r10), %ymm13
	vblendpd		$ 0x3, %ymm14, %ymm13, %ymm13
	vpermilpd		$ 0x3, %ymm0, %ymm12
	vperm2f128		$ 0x00, %ymm12, %ymm12, %ymm12
	vmulpd			%ymm13, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0
	vbroadcastsd	16(%r11), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm1
	vblendpd		$ 0x4, %ymm1, %ymm0, %ymm0

	vmovapd			64(%r10), %ymm13
	vblendpd		$ 0x7, %ymm14, %ymm13, %ymm13
	vpermilpd		$ 0x0, %ymm0, %ymm12
	vperm2f128		$ 0x11, %ymm12, %ymm12, %ymm12
	vmulpd			%ymm13, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0
	vbroadcastsd	24(%r11), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm1
	vblendpd		$ 0x8, %ymm1, %ymm0, %ymm0

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsv_ln_inv_4_lib4)
#endif

#endif




// common inner routine with file scope
//
// blend for ta==n, scale for generic alpha and beta
//
// input arguments:
// r10  <- alpha
// r11  <- beta
// r12  <- y
// xmm0 <- [z0 z1]_a
// xmm1 <- [z2 z3]_a
// xmm2 <- [z0 z1]_b
// xmm3 <- [z2 z3]_b
//
// output arguments:
// r10  <- alpha
// r11  <- beta
// r12  <- y
// xmm0 <- [z0 z1]
// xmm1 <- [z2 z3]

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_N_SCALE_AB_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_blend_n_scale_ab_4_lib4)
#endif

	// reduction
	addpd	%xmm2, %xmm0
	addpd	%xmm3, %xmm1

	// alpha
	movddup	0(%r10), %xmm15
	mulpd	%xmm15, %xmm0
	mulpd	%xmm15, %xmm1

	// beta
	movddup	0(%r11), %xmm15

	vxorpd		%xmm14, %xmm14, %xmm14 // 0.0
	vucomisd	%xmm14, %xmm15 // beta==0.0 ?
	je			0f // end

	movupd	0(%r12), %xmm14
	mulpd	%xmm15, %xmm14
	addpd	%xmm14, %xmm0
	movupd	16(%r12), %xmm14
	mulpd	%xmm15, %xmm14
	addpd	%xmm14, %xmm1

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret
	
	FUN_END(inner_blend_n_scale_ab_4_lib4)
#endif





// common inner routine with file scope
//
// blend for ta==t, scale for generic alpha and beta
//
// input arguments:
// r10  <- alpha
// r11  <- beta
// r12  <- y
// xmm0 <- [z0a z0b]
// xmm1 <- [z1a z1b]
// xmm2 <- [z2a z2b]
// xmm3 <- [z3a z3b]
//
// output arguments:
// r10  <- alpha
// r11  <- beta
// r12  <- y
// xmm0 <- [z0 z1]
// xmm1 <- [z2 z3]

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_T_SCALE_AB_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_blend_t_scale_ab_4_lib4)
#endif

	// reduction
	haddpd	%xmm1, %xmm0
	haddpd	%xmm3, %xmm2
	movapd	%xmm2, %xmm1

	// alpha
	movddup	0(%r10), %xmm15
	mulpd	%xmm15, %xmm0
	mulpd	%xmm15, %xmm1

	// beta
	movddup	0(%r11), %xmm15

	vxorpd		%xmm14, %xmm14, %xmm14 // 0.0
	vucomisd	%xmm14, %xmm15 // beta==0.0 ?
	je			0f // end

	movupd	0(%r12), %xmm14
	mulpd	%xmm15, %xmm14
	addpd	%xmm14, %xmm0
	movupd	16(%r12), %xmm14
	mulpd	%xmm15, %xmm14
	addpd	%xmm14, %xmm1

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret
	
	FUN_END(inner_blend_t_scale_ab_4_lib4)
#endif





// common inner routine with file scope
//
// blend for ta==t, scale for generic alpha and beta=1.0
//
// input arguments:
// r10  <- alpha
// r11  <- y
// xmm0 <- [z0a z0b]
// xmm1 <- [z1a z1b]
// xmm2 <- [z2a z2b]
// xmm3 <- [z3a z3b]
//
// output arguments:
// r10  <- alpha
// r11  <- y
// xmm0 <- [z0 z1]
// xmm1 <- [z2 z3]

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_T_SCALE_A1_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_blend_t_scale_a1_4_lib4)
#endif

	// reduction
	haddpd	%xmm1, %xmm0
	haddpd	%xmm3, %xmm2
	movapd	%xmm2, %xmm1

	// alpha
	movddup	0(%r10), %xmm15
	mulpd	%xmm15, %xmm0
	mulpd	%xmm15, %xmm1

	// beta
	movupd	0(%r11), %xmm14
	addpd	%xmm14, %xmm0
	movupd	16(%r11), %xmm14
	addpd	%xmm14, %xmm1
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret
	
	FUN_END(inner_blend_t_scale_a1_4_lib4)
#endif





// common inner routine with file scope
//
// store 
//
// input arguments:
// r10  <- z
// xmm0 <- [z0 z1]
// xmm1 <- [z2 z3]
//
// output arguments:
// r10  <- z
// xmm0 <- [z0 z1]
// xmm1 <- [z2 z3]

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_4_lib4)
#endif
	
	movupd %xmm0,  0(%r10)
	movupd %xmm1, 16(%r10)
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4_lib4)
#endif





// common inner routine with file scope
//
// store vs
//
// input arguments:
// r10   <- D
// r11d   <- km
// xmm0 <- [z0 z1]
// xmm1 <- [z2 z3]
//
// output arguments:
// r10   <- D
// r11d   <- km
// xmm0 <- [z0 z1]
// xmm1 <- [z2 z3]

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_4_vs_lib4)
#endif
	
	cmpl	$ 0, %r11d
	jle		0f // return

	movsd 	%xmm0, 0(%r10)

	cmpl	$ 1, %r11d
	jle		0f // return

	movhpd 	%xmm0, 8(%r10)

	cmpl	$ 2, %r11d
	jle		0f // return

	movsd 	%xmm1, 16(%r10)

	cmpl	$ 3, %r11d
	jle		0f // return

	movhpd 	%xmm1, 24(%r10)

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4_vs_lib4)
#endif





//                            1      2              3          4          5             6          7
// void kernel_dgemv_n_4_lib4(int k, double *alpha, double *A, double *x, double *beta, double *y, double *z);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemv_n_4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // x

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMV_ADD_N_4_LIB4
#else
	CALL(inner_kernel_dgemv_add_n_4_lib4)
#endif


	// call inner blend n scale ab

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11   // beta
	movq	ARG6, %r12   // y

#if MACRO_LEVEL>=1
	INNER_BLEND_N_SCALE_AB_4_LIB4
#else
	CALL(inner_blend_n_scale_ab_4_lib4)
#endif


	// store

	movq	ARG7, %r10 // z 

#if MACRO_LEVEL>=1
	INNER_STORE_4_LIB4
#else
	CALL(inner_store_4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemv_n_4_lib4)





//                               1      2              3          4          5             6          7          8
// void kernel_dgemv_n_4_vs_lib4(int k, double *alpha, double *A, double *x, double *beta, double *y, double *z, int k1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemv_n_4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // x

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMV_ADD_N_4_LIB4
#else
	CALL(inner_kernel_dgemv_add_n_4_lib4)
#endif


	// call inner blend n scale ab

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11   // beta
	movq	ARG6, %r12   // y

#if MACRO_LEVEL>=1
	INNER_BLEND_N_SCALE_AB_4_LIB4
#else
	CALL(inner_blend_n_scale_ab_4_lib4)
#endif


	// store

	movq	ARG7, %r10 // z 
	movq	ARG8, %r11 // k1

#if MACRO_LEVEL>=1
	INNER_STORE_4_VS_LIB4
#else
	CALL(inner_store_4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemv_n_4_vs_lib4)





//                            1      2              3          4        5          6             7         8          9
// void kernel_dgemv_t_4_lib4(int k, double *alpha, int offa, double *A, int sda, double *x, double *beta, double *y, double *z);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemv_t_4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG4, %r11  // A
	movq	ARG5, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
//	movslq	%r12d, %r12
	movq	ARG6, %r13  // x
	movq	ARG3, %r14 // offA

#if MACRO_LEVEL>=2
	INNER_EDGE_GEMV_ADD_T_4_LIB4
#else
	CALL(inner_edge_dgemv_add_t_4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMV_ADD_T_4_LIB4
#else
	CALL(inner_kernel_dgemv_add_t_4_lib4)
#endif


	// call inner blender t

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11   // beta
	movq	ARG8, %r12 // y 

#if MACRO_LEVEL>=1
	INNER_BLEND_T_SCALE_AB_4_LIB4
#else
	CALL(inner_blend_t_scale_ab_4_lib4)
#endif


	// store

	movq	ARG9, %r10 // z 

#if MACRO_LEVEL>=1
	INNER_STORE_4_LIB4
#else
	CALL(inner_store_4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemv_t_4_lib4)





//                               1      2              3          4        5          6             7         8           9         10
// void kernel_dgemv_t_4_vs_lib4(int k, double *alpha, int offA, double *A, int sda, double *x, double *beta, double *y, double *z, int km);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemv_t_4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG4, %r11  // A
	movq	ARG5, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
//	movslq	%r12d, %r12
	movq	ARG6, %r13  // x
	movq	ARG3, %r14 // offA

#if MACRO_LEVEL>=2
	INNER_EDGE_GEMV_ADD_T_4_LIB4
#else
	CALL(inner_edge_dgemv_add_t_4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMV_ADD_T_4_LIB4
#else
	CALL(inner_kernel_dgemv_add_t_4_lib4)
#endif


	// call inner blender t

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11   // beta
	movq	ARG8, %r12 // y 

#if MACRO_LEVEL>=1
	INNER_BLEND_T_SCALE_AB_4_LIB4
#else
	CALL(inner_blend_t_scale_ab_4_lib4)
#endif


	// store

	movq	ARG9, %r10 // z 
	movq	ARG10, %r11 // km 

#if MACRO_LEVEL>=1
	INNER_STORE_4_VS_LIB4
#else
	CALL(inner_store_4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemv_t_4_vs_lib4)





//                             1      2                3                4          5        6            7            8               9            10           11
// void kernel_dgemv_nt_4_lib4(int k, double *alpha_n, double *alpha_t, double *A, int sda, double *x_n, double *x_t, double *beta_t, double *y_t, double *z_n, double *z_t);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemv_nt_4_lib4)
	
	PROLOGUE

	// zero accumulation registers y_t

	ZERO_ACC

	// initialize x_n
	movq	ARG2, %r10 // alpha_n
	movddup 0(%r10), %xmm15

	movq	ARG6, %r10 // x_n

	movddup 0(%r10), %xmm4
	mulpd	%xmm15, %xmm4
	movddup 8(%r10), %xmm5
	mulpd	%xmm15, %xmm5
	movddup 16(%r10), %xmm6
	mulpd	%xmm15, %xmm6
	movddup 24(%r10), %xmm7
	mulpd	%xmm15, %xmm7


	// inner kernel dgemv nt

	movq	ARG1, %r10 // k
	movq	ARG4, %r11  // A
	movq	ARG5, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
//	movslq	%r12d, %r12
	movq	ARG7, %r13  // x_t
	movq	ARG10, %r14  // z_n

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMV_ADD_NT_4_LIB4
#else
	CALL(inner_kernel_dgemv_add_nt_4_lib4)
#endif


	// inner blend n scale ab

	movq	ARG3, %r10 // alpha_t
	movq	ARG8, %r11   // beta_t
	movq	ARG9, %r12   // y_t

#if MACRO_LEVEL>=1
	INNER_BLEND_T_SCALE_AB_4_LIB4
#else
	CALL(inner_blend_t_scale_ab_4_lib4)
#endif


	// store

	movq	ARG11, %r10 // z_t 

#if MACRO_LEVEL>=1
	INNER_STORE_4_LIB4
#else
	CALL(inner_store_4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemv_nt_4_lib4)





//                                1      2                3                4          5        6            7            8               9            10           11           12
// void kernel_dgemv_nt_4_vs_lib4(int k, double *alpha_n, double *alpha_t, double *A, int sda, double *x_n, double *x_t, double *beta_t, double *y_t, double *z_n, double *z_t, int km);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemv_nt_4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers y_t

	xorpd	%xmm0, %xmm0
	movapd	%xmm0, %xmm1
	movapd	%xmm0, %xmm2
	movapd	%xmm0, %xmm3

	movapd	%xmm0, %xmm4
	movapd	%xmm0, %xmm5
	movapd	%xmm0, %xmm6
	movapd	%xmm0, %xmm7

	// initialize x_n
	movq	ARG2, %r10 // alpha_n
	movddup 0(%r10), %xmm15

	movq	ARG6, %r10 // x_n
	movq	ARG12, %r11 // km

	movddup 0(%r10), %xmm4
	mulpd	%xmm15, %xmm4
	cmpl	$ 2, %r11d
	jl		0f
	movddup 8(%r10), %xmm5
	mulpd	%xmm15, %xmm5
	cmpl	$ 3, %r11d
	jl		0f
	movddup 16(%r10), %xmm6
	mulpd	%xmm15, %xmm6
	je		0f
	movddup 24(%r10), %xmm7
	mulpd	%xmm15, %xmm7
0:

	// inner kernel dgemv nt

	movq	ARG1, %r10 // k
	movq	ARG4, %r11  // A
	movq	ARG5, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
//	movslq	%r12d, %r12
	movq	ARG7, %r13  // x_t
	movq	ARG10, %r14  // z_n

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMV_ADD_NT_4_LIB4
#else
	CALL(inner_kernel_dgemv_add_nt_4_lib4)
#endif


	// inner blend n scale ab

	movq	ARG3, %r10 // alpha_t
	movq	ARG8, %r11   // beta_t
	movq	ARG9, %r12   // y_t

#if MACRO_LEVEL>=1
	INNER_BLEND_T_SCALE_AB_4_LIB4
#else
	CALL(inner_blend_t_scale_ab_4_lib4)
#endif


	// store

	movq	ARG11, %r10 // z_t 
	movq	ARG12, %r11 // km 

#if MACRO_LEVEL>=1
	INNER_STORE_4_VS_LIB4
#else
	CALL(inner_store_4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemv_nt_4_vs_lib4)





//                            1      2              3          4        5           6
// void kernel_dsymv_l_4_lib4(int k, double *alpha, double *A, int sda, double *x, double *z);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsymv_l_4_lib4)
	
	PROLOGUE

	// zero accumulation registers y_t

	ZERO_ACC

	// initialize x_n
	movq	ARG2, %r10 // alpha
	movddup 0(%r10), %xmm15

	movq	ARG5, %r10 // x_n

	movddup 0(%r10), %xmm4
	mulpd	%xmm15, %xmm4
	movddup 8(%r10), %xmm5
	mulpd	%xmm15, %xmm5
	movddup 16(%r10), %xmm6
	mulpd	%xmm15, %xmm6
	movddup 24(%r10), %xmm7
	mulpd	%xmm15, %xmm7


	// inner edge dsyrk & kernel dgemv nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13  // x_t
	movq	ARG6, %r14  // z_n

#if MACRO_LEVEL>=2
	INNER_EDGE_DSYMV_ADD_NT_4_LIB4
#else
	CALL(inner_edge_dsymv_add_nt_4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMV_ADD_NT_4_LIB4
#else
	CALL(inner_kernel_dgemv_add_nt_4_lib4)
#endif


	// call inner blend n scale ab

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11   // z_t

#if MACRO_LEVEL>=1
	INNER_BLEND_T_SCALE_A1_4_LIB4
#else
	CALL(inner_blend_t_scale_a1_4_lib4)
#endif


	// store

	movq	ARG6, %r10 // z_t 

#if MACRO_LEVEL>=1
	INNER_STORE_4_LIB4
#else
	CALL(inner_store_4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dsymv_l_4_lib4)






