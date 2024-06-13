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
#define ARG19 STACKSIZE + 104(%rsp)
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
	vmovapd	%ymm0, %ymm7; \
	vmovapd	%ymm0, %ymm8; \
	vmovapd	%ymm0, %ymm9; \
	vmovapd	%ymm0, %ymm10; \
	vmovapd	%ymm0, %ymm11
#define NEG_ACC \
	vmovapd		.LC11(%rip), %ymm15; \
	vxorpd		%ymm15, %ymm0, %ymm0; \
	vxorpd		%ymm15, %ymm1, %ymm1; \
	vxorpd		%ymm15, %ymm2, %ymm2; \
	vxorpd		%ymm15, %ymm3, %ymm3; \
	vxorpd		%ymm15, %ymm4, %ymm4; \
	vxorpd		%ymm15, %ymm5, %ymm5; \
	vxorpd		%ymm15, %ymm6, %ymm6; \
	vxorpd		%ymm15, %ymm7, %ymm7; \
	vxorpd		%ymm15, %ymm8, %ymm8; \
	vxorpd		%ymm15, %ymm9, %ymm9; \
	vxorpd		%ymm15, %ymm10, %ymm10; \
	vxorpd		%ymm15, %ymm11, %ymm11

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
	vmovapd	%ymm0, %ymm7; \
	vmovapd	%ymm0, %ymm8; \
	vmovapd	%ymm0, %ymm9; \
	vmovapd	%ymm0, %ymm10; \
	vmovapd	%ymm0, %ymm11
#define NEG_ACC \
	vmovapd		LC11(%rip), %ymm15; \
	vxorpd		%ymm15, %ymm0, %ymm0; \
	vxorpd		%ymm15, %ymm1, %ymm1; \
	vxorpd		%ymm15, %ymm2, %ymm2; \
	vxorpd		%ymm15, %ymm3, %ymm3; \
	vxorpd		%ymm15, %ymm4, %ymm4; \
	vxorpd		%ymm15, %ymm5, %ymm5; \
	vxorpd		%ymm15, %ymm6, %ymm6; \
	vxorpd		%ymm15, %ymm7, %ymm7; \
	vxorpd		%ymm15, %ymm8, %ymm8; \
	vxorpd		%ymm15, %ymm9, %ymm9; \
	vxorpd		%ymm15, %ymm10, %ymm10; \
	vxorpd		%ymm15, %ymm11, %ymm11

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
#define ARG19 STACKSIZE + 152(%rsp)
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
	vmovapd	%ymm0, %ymm7; \
	vmovapd	%ymm0, %ymm8; \
	vmovapd	%ymm0, %ymm9; \
	vmovapd	%ymm0, %ymm10; \
	vmovapd	%ymm0, %ymm11
#define NEG_ACC \
	vmovapd		.LC11(%rip), %ymm15; \
	vxorpd		%ymm15, %ymm0, %ymm0; \
	vxorpd		%ymm15, %ymm1, %ymm1; \
	vxorpd		%ymm15, %ymm2, %ymm2; \
	vxorpd		%ymm15, %ymm3, %ymm3; \
	vxorpd		%ymm15, %ymm4, %ymm4; \
	vxorpd		%ymm15, %ymm5, %ymm5; \
	vxorpd		%ymm15, %ymm6, %ymm6; \
	vxorpd		%ymm15, %ymm7, %ymm7; \
	vxorpd		%ymm15, %ymm8, %ymm8; \
	vxorpd		%ymm15, %ymm9, %ymm9; \
	vxorpd		%ymm15, %ymm10, %ymm10; \
	vxorpd		%ymm15, %ymm11, %ymm11

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
// r14   <- 4*sdb*sizeof(double)
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
// r14   <- 4*sdb*sizeof(double)
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
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_8X8_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_8x8_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	// preload
	vmovapd			0(%r11), %ymm12
	vmovapd			0(%r11, %r12, 1), %ymm13
	vbroadcastsd	0(%r13), %ymm14
	vbroadcastsd 	0(%r13, %r14, 1), %ymm15

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop
	
	// unroll 0
	vfmadd231pd		%ymm12, %ymm14, %ymm0
	subl	$ 4, %r10d
	vfmadd231pd		%ymm13, %ymm14, %ymm4
	vbroadcastsd	8(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm8
	vbroadcastsd	8(%r13, %r14, 1), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm1
	vfmadd231pd		%ymm13, %ymm14, %ymm5
	vbroadcastsd	16(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm9
	vbroadcastsd	16(%r13, %r14, 1), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm2
	vfmadd231pd		%ymm13, %ymm14, %ymm6
	vbroadcastsd	24(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm10
	vbroadcastsd	24(%r13, %r14, 1), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm3
	vmovapd			32(%r11), %ymm12
	vfmadd231pd		%ymm13, %ymm14, %ymm7
	vbroadcastsd	32(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm11
	vmovapd			32(%r11, %r12, 1), %ymm13
	vbroadcastsd	32(%r13, %r14, 1), %ymm15

	// unroll 1
	vfmadd231pd		%ymm12, %ymm14, %ymm0
	vfmadd231pd		%ymm13, %ymm14, %ymm4
	vbroadcastsd	40(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm8
	vbroadcastsd	40(%r13, %r14, 1), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm1
	vfmadd231pd		%ymm13, %ymm14, %ymm5
	vbroadcastsd	48(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm9
	vbroadcastsd	48(%r13, %r14, 1), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm2
	vfmadd231pd		%ymm13, %ymm14, %ymm6
	vbroadcastsd	56(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm10
	vbroadcastsd	56(%r13, %r14, 1), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm3
	vmovapd			64(%r11), %ymm12
	vfmadd231pd		%ymm13, %ymm14, %ymm7
	vbroadcastsd	64(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm11
	vmovapd			64(%r11, %r12, 1), %ymm13
	vbroadcastsd	64(%r13, %r14, 1), %ymm15

	// unroll 2
	vfmadd231pd		%ymm12, %ymm14, %ymm0
	vfmadd231pd		%ymm13, %ymm14, %ymm4
	vbroadcastsd	72(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm8
	vbroadcastsd	72(%r13, %r14, 1), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm1
	vfmadd231pd		%ymm13, %ymm14, %ymm5
	vbroadcastsd	80(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm9
	vbroadcastsd	80(%r13, %r14, 1), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm2
	vfmadd231pd		%ymm13, %ymm14, %ymm6
	vbroadcastsd	88(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm10
	vbroadcastsd	88(%r13, %r14, 1), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm3
	vmovapd			96(%r11), %ymm12
	vfmadd231pd		%ymm13, %ymm14, %ymm7
	vbroadcastsd	96(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm11
	vmovapd			96(%r11, %r12, 1), %ymm13
	vbroadcastsd	96(%r13, %r14, 1), %ymm15

	// unroll 3
	vfmadd231pd		%ymm12, %ymm14, %ymm0
	vfmadd231pd		%ymm13, %ymm14, %ymm4
	vbroadcastsd	104(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm8
	vbroadcastsd	104(%r13, %r14, 1), %ymm15
	addq	$ 128, %r11

	vfmadd231pd		%ymm12, %ymm14, %ymm1
	vfmadd231pd		%ymm13, %ymm14, %ymm5
	vbroadcastsd	112(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm9
	vbroadcastsd	112(%r13, %r14, 1), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm2
	vfmadd231pd		%ymm13, %ymm14, %ymm6
	vbroadcastsd	120(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm10
	vbroadcastsd	120(%r13, %r14, 1), %ymm15
	addq	$ 128, %r13

	vfmadd231pd		%ymm12, %ymm14, %ymm3
	vmovapd			0(%r11), %ymm12
	vfmadd231pd		%ymm13, %ymm14, %ymm7
	vbroadcastsd	0(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm11
	vmovapd			0(%r11, %r12, 1), %ymm13
	vbroadcastsd	0(%r13, %r14, 1), %ymm15

	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vfmadd231pd		%ymm12, %ymm14, %ymm0
	subl	$ 4, %r10d
	vfmadd231pd		%ymm13, %ymm14, %ymm4
	vbroadcastsd	8(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm8
	vbroadcastsd	8(%r13, %r14, 1), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm1
	vfmadd231pd		%ymm13, %ymm14, %ymm5
	vbroadcastsd	16(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm9
	vbroadcastsd	16(%r13, %r14, 1), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm2
	vfmadd231pd		%ymm13, %ymm14, %ymm6
	vbroadcastsd	24(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm10
	vbroadcastsd	24(%r13, %r14, 1), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm3
	vmovapd			32(%r11), %ymm12
	vfmadd231pd		%ymm13, %ymm14, %ymm7
	vbroadcastsd	32(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm11
	vmovapd			32(%r11, %r12, 1), %ymm13
	vbroadcastsd	32(%r13, %r14, 1), %ymm15

	// unroll 1
	vfmadd231pd		%ymm12, %ymm14, %ymm0
	vfmadd231pd		%ymm13, %ymm14, %ymm4
	vbroadcastsd	40(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm8
	vbroadcastsd	40(%r13, %r14, 1), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm1
	vfmadd231pd		%ymm13, %ymm14, %ymm5
	vbroadcastsd	48(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm9
	vbroadcastsd	48(%r13, %r14, 1), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm2
	vfmadd231pd		%ymm13, %ymm14, %ymm6
	vbroadcastsd	56(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm10
	vbroadcastsd	56(%r13, %r14, 1), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm3
	vmovapd			64(%r11), %ymm12
	vfmadd231pd		%ymm13, %ymm14, %ymm7
	vbroadcastsd	64(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm11
	vmovapd			64(%r11, %r12, 1), %ymm13
	vbroadcastsd	64(%r13, %r14, 1), %ymm15

	// unroll 2
	vfmadd231pd		%ymm12, %ymm14, %ymm0
	vfmadd231pd		%ymm13, %ymm14, %ymm4
	vbroadcastsd	72(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm8
	vbroadcastsd	72(%r13, %r14, 1), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm1
	vfmadd231pd		%ymm13, %ymm14, %ymm5
	vbroadcastsd	80(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm9
	vbroadcastsd	80(%r13, %r14, 1), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm2
	vfmadd231pd		%ymm13, %ymm14, %ymm6
	vbroadcastsd	88(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm10
	vbroadcastsd	88(%r13, %r14, 1), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm3
	vmovapd			96(%r11), %ymm12
	vfmadd231pd		%ymm13, %ymm14, %ymm7
	vbroadcastsd	96(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm11
	vmovapd			96(%r11, %r12, 1), %ymm13
	vbroadcastsd	96(%r13, %r14, 1), %ymm15

	// unroll 3
	vfmadd231pd		%ymm12, %ymm14, %ymm0
	vfmadd231pd		%ymm13, %ymm14, %ymm4
	vbroadcastsd	104(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm8
	vbroadcastsd	104(%r13, %r14, 1), %ymm15
	addq	$ 128, %r11

	vfmadd231pd		%ymm12, %ymm14, %ymm1
	vfmadd231pd		%ymm13, %ymm14, %ymm5
	vbroadcastsd	112(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm9
	vbroadcastsd	112(%r13, %r14, 1), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm2
	vfmadd231pd		%ymm13, %ymm14, %ymm6
	vbroadcastsd	120(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm10
	vbroadcastsd	120(%r13, %r14, 1), %ymm15
	addq	$ 128, %r13

	vfmadd231pd		%ymm12, %ymm14, %ymm3
//	vmovapd			0(%r11), %ymm12
	vfmadd231pd		%ymm13, %ymm14, %ymm7
//	vbroadcastsd	0(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm11
//	vmovapd			0(%r11, %r12, 1), %ymm13
//	vbroadcastsd	0(%r13, %r14, 1), %ymm15

	jmp		2f


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovapd			0(%r11), %ymm12
	vmovapd			0(%r11, %r12, 1), %ymm13
	vbroadcastsd	0(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm14, %ymm0
	vfmadd231pd		%ymm13, %ymm14, %ymm4
	vbroadcastsd	0(%r13, %r14, 1), %ymm15
	vfmadd231pd		%ymm13, %ymm15, %ymm8
	subl	$ 1, %r10d

	vbroadcastsd	8(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm14, %ymm1
	vfmadd231pd		%ymm13, %ymm14, %ymm5
	vbroadcastsd	8(%r13, %r14, 1), %ymm15
	vfmadd231pd		%ymm13, %ymm15, %ymm9
	addq		$ 32, %r11

	vbroadcastsd	16(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm14, %ymm2
	vfmadd231pd		%ymm13, %ymm14, %ymm6
	vbroadcastsd	16(%r13, %r14, 1), %ymm15
	vfmadd231pd		%ymm13, %ymm15, %ymm10
	addq		$ 32, %r13

	vbroadcastsd	-8(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm14, %ymm3
	vfmadd231pd		%ymm13, %ymm14, %ymm7
	vbroadcastsd	-8(%r13, %r14, 1), %ymm15
	vfmadd231pd		%ymm13, %ymm15, %ymm11

	cmpl		$ 0, %r10d
	jg		3b // clean up loop 


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_8x8_lib4)
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
// r14   <- 4*sdb*sizeof(double)
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
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_8X8L_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_8x8l_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	// preload
	vmovapd			0(%r11), %ymm12
	vmovapd			0(%r11, %r12, 1), %ymm13
	vbroadcastsd	0(%r13), %ymm14
	vbroadcastsd 	128(%r13), %ymm15

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop
	
	// unroll 0
	vfmadd231pd		%ymm12, %ymm14, %ymm0
	subl	$ 4, %r10d
	vfmadd231pd		%ymm13, %ymm14, %ymm4
	vbroadcastsd	32(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm8
	vbroadcastsd	160(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm1
	vfmadd231pd		%ymm13, %ymm14, %ymm5
	vbroadcastsd	64(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm9
	vbroadcastsd	192(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm2
	vfmadd231pd		%ymm13, %ymm14, %ymm6
	vbroadcastsd	96(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm10
	vbroadcastsd	224(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm3
	vmovapd			32(%r11), %ymm12
	vfmadd231pd		%ymm13, %ymm14, %ymm7
	vbroadcastsd	8(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm11
	vmovapd			32(%r11, %r12, 1), %ymm13
	vbroadcastsd	136(%r13), %ymm15

	// unroll 1
	vfmadd231pd		%ymm12, %ymm14, %ymm0
	vfmadd231pd		%ymm13, %ymm14, %ymm4
	vbroadcastsd	40(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm8
	vbroadcastsd	168(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm1
	vfmadd231pd		%ymm13, %ymm14, %ymm5
	vbroadcastsd	72(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm9
	vbroadcastsd	200(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm2
	vfmadd231pd		%ymm13, %ymm14, %ymm6
	vbroadcastsd	104(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm10
	vbroadcastsd	232(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm3
	vmovapd			64(%r11), %ymm12
	vfmadd231pd		%ymm13, %ymm14, %ymm7
	vbroadcastsd	16(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm11
	vmovapd			64(%r11, %r12, 1), %ymm13
	vbroadcastsd	144(%r13), %ymm15

	// unroll 2
	vfmadd231pd		%ymm12, %ymm14, %ymm0
	vfmadd231pd		%ymm13, %ymm14, %ymm4
	vbroadcastsd	48(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm8
	vbroadcastsd	176(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm1
	vfmadd231pd		%ymm13, %ymm14, %ymm5
	vbroadcastsd	80(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm9
	vbroadcastsd	208(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm2
	vfmadd231pd		%ymm13, %ymm14, %ymm6
	vbroadcastsd	112(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm10
	vbroadcastsd	240(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm3
	vmovapd			96(%r11), %ymm12
	vfmadd231pd		%ymm13, %ymm14, %ymm7
	vbroadcastsd	24(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm11
	vmovapd			96(%r11, %r12, 1), %ymm13
	vbroadcastsd	152(%r13), %ymm15

	// unroll 3
	vfmadd231pd		%ymm12, %ymm14, %ymm0
	vfmadd231pd		%ymm13, %ymm14, %ymm4
	vbroadcastsd	56(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm8
	vbroadcastsd	184(%r13), %ymm15
	addq	$ 128, %r11

	vfmadd231pd		%ymm12, %ymm14, %ymm1
	vfmadd231pd		%ymm13, %ymm14, %ymm5
	vbroadcastsd	88(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm9
	vbroadcastsd	216(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm2
	vfmadd231pd		%ymm13, %ymm14, %ymm6
	vbroadcastsd	120(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm10
	vbroadcastsd	248(%r13), %ymm15
	addq	%r14, %r13

	vfmadd231pd		%ymm12, %ymm14, %ymm3
	vmovapd			0(%r11), %ymm12
	vfmadd231pd		%ymm13, %ymm14, %ymm7
	vbroadcastsd	0(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm11
	vmovapd			0(%r11, %r12, 1), %ymm13
	vbroadcastsd	128(%r13), %ymm15

	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vfmadd231pd		%ymm12, %ymm14, %ymm0
	subl	$ 4, %r10d
	vfmadd231pd		%ymm13, %ymm14, %ymm4
	vbroadcastsd	32(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm8
	vbroadcastsd	160(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm1
	vfmadd231pd		%ymm13, %ymm14, %ymm5
	vbroadcastsd	64(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm9
	vbroadcastsd	192(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm2
	vfmadd231pd		%ymm13, %ymm14, %ymm6
	vbroadcastsd	96(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm10
	vbroadcastsd	224(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm3
	vmovapd			32(%r11), %ymm12
	vfmadd231pd		%ymm13, %ymm14, %ymm7
	vbroadcastsd	8(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm11
	vmovapd			32(%r11, %r12, 1), %ymm13
	vbroadcastsd	136(%r13), %ymm15

	// unroll 1
	vfmadd231pd		%ymm12, %ymm14, %ymm0
	vfmadd231pd		%ymm13, %ymm14, %ymm4
	vbroadcastsd	40(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm8
	vbroadcastsd	168(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm1
	vfmadd231pd		%ymm13, %ymm14, %ymm5
	vbroadcastsd	72(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm9
	vbroadcastsd	200(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm2
	vfmadd231pd		%ymm13, %ymm14, %ymm6
	vbroadcastsd	104(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm10
	vbroadcastsd	232(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm3
	vmovapd			64(%r11), %ymm12
	vfmadd231pd		%ymm13, %ymm14, %ymm7
	vbroadcastsd	16(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm11
	vmovapd			64(%r11, %r12, 1), %ymm13
	vbroadcastsd	144(%r13), %ymm15

	// unroll 2
	vfmadd231pd		%ymm12, %ymm14, %ymm0
	vfmadd231pd		%ymm13, %ymm14, %ymm4
	vbroadcastsd	48(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm8
	vbroadcastsd	176(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm1
	vfmadd231pd		%ymm13, %ymm14, %ymm5
	vbroadcastsd	80(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm9
	vbroadcastsd	208(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm2
	vfmadd231pd		%ymm13, %ymm14, %ymm6
	vbroadcastsd	112(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm10
	vbroadcastsd	240(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm3
	vmovapd			96(%r11), %ymm12
	vfmadd231pd		%ymm13, %ymm14, %ymm7
	vbroadcastsd	24(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm11
	vmovapd			96(%r11, %r12, 1), %ymm13
	vbroadcastsd	152(%r13), %ymm15

	// unroll 3
	vfmadd231pd		%ymm12, %ymm14, %ymm0
	vfmadd231pd		%ymm13, %ymm14, %ymm4
	vbroadcastsd	56(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm8
	vbroadcastsd	184(%r13), %ymm15
	addq	$ 128, %r11

	vfmadd231pd		%ymm12, %ymm14, %ymm1
	vfmadd231pd		%ymm13, %ymm14, %ymm5
	vbroadcastsd	88(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm9
	vbroadcastsd	216(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm2
	vfmadd231pd		%ymm13, %ymm14, %ymm6
	vbroadcastsd	120(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm10
	vbroadcastsd	248(%r13), %ymm15
	addq	%r14, %r13

	vfmadd231pd		%ymm12, %ymm14, %ymm3
//	vmovapd			0(%r11), %ymm12
	vfmadd231pd		%ymm13, %ymm14, %ymm7
//	vbroadcastsd	0(%r13), %ymm14
	vfmadd231pd		%ymm13, %ymm15, %ymm11
//	vmovapd			0(%r11, %r12, 1), %ymm13
//	vbroadcastsd	128(%r13), %ymm15

	jmp		2f


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovapd			0(%r11), %ymm12
	vmovapd			0(%r11, %r12, 1), %ymm13
	vbroadcastsd	0(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm14, %ymm0
	vfmadd231pd		%ymm13, %ymm14, %ymm4
	vbroadcastsd	128(%r13), %ymm15
	vfmadd231pd		%ymm13, %ymm15, %ymm8
	subl	$ 1, %r10d

	vbroadcastsd	32(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm14, %ymm1
	vfmadd231pd		%ymm13, %ymm14, %ymm5
	vbroadcastsd	160(%r13), %ymm15
	vfmadd231pd		%ymm13, %ymm15, %ymm9
	addq		$ 32, %r11

	vbroadcastsd	64(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm14, %ymm2
	vfmadd231pd		%ymm13, %ymm14, %ymm6
	vbroadcastsd	192(%r13), %ymm15
	vfmadd231pd		%ymm13, %ymm15, %ymm10

	vbroadcastsd	96(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm14, %ymm3
	vfmadd231pd		%ymm13, %ymm14, %ymm7
	vbroadcastsd	224(%r13), %ymm15
	vfmadd231pd		%ymm13, %ymm15, %ymm11
	addq		$ 8, %r13

	cmpl		$ 0, %r10d
	jg		3b // clean up loop 


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_8x8l_lib4)
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
// r14   <- 4*sdb*sizeof(double)
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
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_8X8U_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_8x8u_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	// preload
	vmovapd			0(%r11), %ymm12
	vmovapd			0(%r11, %r12, 1), %ymm13
	vbroadcastsd	0(%r13), %ymm14
	vbroadcastsd 	128(%r13), %ymm15

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop
	
	// unroll 0
	vfmadd231pd		%ymm12, %ymm14, %ymm0
	subl	$ 4, %r10d
	vbroadcastsd	32(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm15, %ymm4
	vfmadd231pd		%ymm13, %ymm15, %ymm8
	vbroadcastsd	160(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm1
	vbroadcastsd	64(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm15, %ymm5
	vfmadd231pd		%ymm13, %ymm15, %ymm9
	vbroadcastsd	192(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm2
	vbroadcastsd	96(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm15, %ymm6
	vfmadd231pd		%ymm13, %ymm15, %ymm10
	vbroadcastsd	224(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm3
	vbroadcastsd	8(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm15, %ymm7
	vmovapd			32(%r11), %ymm12
	vfmadd231pd		%ymm13, %ymm15, %ymm11
	vmovapd			32(%r11, %r12, 1), %ymm13
	vbroadcastsd	136(%r13), %ymm15

	// unroll 1
	vfmadd231pd		%ymm12, %ymm14, %ymm0
	vbroadcastsd	40(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm15, %ymm4
	vfmadd231pd		%ymm13, %ymm15, %ymm8
	vbroadcastsd	168(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm1
	vbroadcastsd	72(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm15, %ymm5
	vfmadd231pd		%ymm13, %ymm15, %ymm9
	vbroadcastsd	200(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm2
	vbroadcastsd	104(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm15, %ymm6
	vfmadd231pd		%ymm13, %ymm15, %ymm10
	vbroadcastsd	232(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm3
	vbroadcastsd	16(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm15, %ymm7
	vmovapd			64(%r11), %ymm12
	vfmadd231pd		%ymm13, %ymm15, %ymm11
	vmovapd			64(%r11, %r12, 1), %ymm13
	vbroadcastsd	144(%r13), %ymm15

	// unroll 2
	vfmadd231pd		%ymm12, %ymm14, %ymm0
	vbroadcastsd	48(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm15, %ymm4
	vfmadd231pd		%ymm13, %ymm15, %ymm8
	vbroadcastsd	176(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm1
	vbroadcastsd	80(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm15, %ymm5
	vfmadd231pd		%ymm13, %ymm15, %ymm9
	vbroadcastsd	208(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm2
	vbroadcastsd	112(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm15, %ymm6
	vfmadd231pd		%ymm13, %ymm15, %ymm10
	vbroadcastsd	240(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm3
	vbroadcastsd	24(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm15, %ymm7
	vmovapd			96(%r11), %ymm12
	vfmadd231pd		%ymm13, %ymm15, %ymm11
	vmovapd			96(%r11, %r12, 1), %ymm13
	vbroadcastsd	152(%r13), %ymm15

	// unroll 3
	vfmadd231pd		%ymm12, %ymm14, %ymm0
	vbroadcastsd	56(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm15, %ymm4
	vfmadd231pd		%ymm13, %ymm15, %ymm8
	vbroadcastsd	184(%r13), %ymm15
	addq	$ 128, %r11

	vfmadd231pd		%ymm12, %ymm14, %ymm1
	vbroadcastsd	88(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm15, %ymm5
	vfmadd231pd		%ymm13, %ymm15, %ymm9
	vbroadcastsd	216(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm2
	vbroadcastsd	120(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm15, %ymm6
	vfmadd231pd		%ymm13, %ymm15, %ymm10
	vbroadcastsd	248(%r13), %ymm15
	addq	%r14, %r13

	vfmadd231pd		%ymm12, %ymm14, %ymm3
	vbroadcastsd	0(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm15, %ymm7
	vmovapd			0(%r11), %ymm12
	vfmadd231pd		%ymm13, %ymm15, %ymm11
	vmovapd			0(%r11, %r12, 1), %ymm13
	vbroadcastsd	128(%r13), %ymm15

	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vfmadd231pd		%ymm12, %ymm14, %ymm0
	subl	$ 4, %r10d
	vbroadcastsd	32(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm15, %ymm4
	vfmadd231pd		%ymm13, %ymm15, %ymm8
	vbroadcastsd	160(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm1
	vbroadcastsd	64(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm15, %ymm5
	vfmadd231pd		%ymm13, %ymm15, %ymm9
	vbroadcastsd	192(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm2
	vbroadcastsd	96(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm15, %ymm6
	vfmadd231pd		%ymm13, %ymm15, %ymm10
	vbroadcastsd	224(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm3
	vbroadcastsd	8(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm15, %ymm7
	vmovapd			32(%r11), %ymm12
	vfmadd231pd		%ymm13, %ymm15, %ymm11
	vmovapd			32(%r11, %r12, 1), %ymm13
	vbroadcastsd	136(%r13), %ymm15

	// unroll 1
	vfmadd231pd		%ymm12, %ymm14, %ymm0
	vbroadcastsd	40(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm15, %ymm4
	vfmadd231pd		%ymm13, %ymm15, %ymm8
	vbroadcastsd	168(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm1
	vbroadcastsd	72(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm15, %ymm5
	vfmadd231pd		%ymm13, %ymm15, %ymm9
	vbroadcastsd	200(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm2
	vbroadcastsd	104(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm15, %ymm6
	vfmadd231pd		%ymm13, %ymm15, %ymm10
	vbroadcastsd	232(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm3
	vbroadcastsd	16(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm15, %ymm7
	vmovapd			64(%r11), %ymm12
	vfmadd231pd		%ymm13, %ymm15, %ymm11
	vmovapd			64(%r11, %r12, 1), %ymm13
	vbroadcastsd	144(%r13), %ymm15

	// unroll 2
	vfmadd231pd		%ymm12, %ymm14, %ymm0
	vbroadcastsd	48(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm15, %ymm4
	vfmadd231pd		%ymm13, %ymm15, %ymm8
	vbroadcastsd	176(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm1
	vbroadcastsd	80(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm15, %ymm5
	vfmadd231pd		%ymm13, %ymm15, %ymm9
	vbroadcastsd	208(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm2
	vbroadcastsd	112(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm15, %ymm6
	vfmadd231pd		%ymm13, %ymm15, %ymm10
	vbroadcastsd	240(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm3
	vbroadcastsd	24(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm15, %ymm7
	vmovapd			96(%r11), %ymm12
	vfmadd231pd		%ymm13, %ymm15, %ymm11
	vmovapd			96(%r11, %r12, 1), %ymm13
	vbroadcastsd	152(%r13), %ymm15

	// unroll 3
	vfmadd231pd		%ymm12, %ymm14, %ymm0
	vbroadcastsd	56(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm15, %ymm4
	vfmadd231pd		%ymm13, %ymm15, %ymm8
	vbroadcastsd	184(%r13), %ymm15
	addq	$ 128, %r11

	vfmadd231pd		%ymm12, %ymm14, %ymm1
	vbroadcastsd	88(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm15, %ymm5
	vfmadd231pd		%ymm13, %ymm15, %ymm9
	vbroadcastsd	216(%r13), %ymm15

	vfmadd231pd		%ymm12, %ymm14, %ymm2
	vbroadcastsd	120(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm15, %ymm6
	vfmadd231pd		%ymm13, %ymm15, %ymm10
	vbroadcastsd	248(%r13), %ymm15
	addq	%r14, %r13

	vfmadd231pd		%ymm12, %ymm14, %ymm3
//	vbroadcastsd	0(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm15, %ymm7
//	vmovapd			0(%r11), %ymm12
	vfmadd231pd		%ymm13, %ymm15, %ymm11
//	vmovapd			0(%r11, %r12, 1), %ymm13
//	vbroadcastsd	128(%r13), %ymm15

	jmp		2f


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovapd			0(%r11), %ymm12
	vmovapd			0(%r11, %r12, 1), %ymm13
	vbroadcastsd	0(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm14, %ymm0
	vbroadcastsd	128(%r13), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm4
	vfmadd231pd		%ymm13, %ymm15, %ymm8
	subl	$ 1, %r10d

	vbroadcastsd	32(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm14, %ymm1
	vbroadcastsd	160(%r13), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm5
	vfmadd231pd		%ymm13, %ymm15, %ymm9
	addq		$ 32, %r11

	vbroadcastsd	64(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm14, %ymm2
	vbroadcastsd	192(%r13), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm6
	vfmadd231pd		%ymm13, %ymm15, %ymm10

	vbroadcastsd	96(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm14, %ymm3
	vbroadcastsd	224(%r13), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm7
	vfmadd231pd		%ymm13, %ymm15, %ymm11
	addq		$ 8, %r13

	cmpl		$ 0, %r10d
	jg		3b // clean up loop 


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_8x8u_lib4)
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

//
// output arguments:


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DGEMM_NN_8X8L_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dgemm_nn_8x8l_lib4)
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
	vmovapd			0(%r11), %ymm12
	vmovapd			0(%r11, %r12, 1), %ymm13
	vbroadcastsd	0(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm14, %ymm0
	vfmadd231pd		%ymm13, %ymm14, %ymm4
	vbroadcastsd	128(%r13), %ymm15
	vfmadd231pd		%ymm13, %ymm15, %ymm8
	vbroadcastsd	32(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm14, %ymm1
	vfmadd231pd		%ymm13, %ymm14, %ymm5
	vbroadcastsd	160(%r13), %ymm15
	vfmadd231pd		%ymm13, %ymm15, %ymm9
	vbroadcastsd	64(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm14, %ymm2
	vfmadd231pd		%ymm13, %ymm14, %ymm6
	vbroadcastsd	192(%r13), %ymm15
	vfmadd231pd		%ymm13, %ymm15, %ymm10
	vbroadcastsd	96(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm14, %ymm3
	vfmadd231pd		%ymm13, %ymm14, %ymm7
	vbroadcastsd	224(%r13), %ymm15
	vfmadd231pd		%ymm13, %ymm15, %ymm11

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

	FUN_END(inner_edge_dgemm_nn_8x8l_lib4)
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

//
// output arguments:


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DGEMM_NN_8X8U_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dgemm_nn_8x8u_lib4)
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
	vmovapd			0(%r11), %ymm12
	vmovapd			0(%r11, %r12, 1), %ymm13
	vbroadcastsd	0(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm14, %ymm0
	vbroadcastsd	128(%r13), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm4
	vfmadd231pd		%ymm13, %ymm15, %ymm8

	vbroadcastsd	32(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm14, %ymm1
	vbroadcastsd	160(%r13), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm5
	vfmadd231pd		%ymm13, %ymm15, %ymm9

	vbroadcastsd	64(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm14, %ymm2
	vbroadcastsd	192(%r13), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm6
	vfmadd231pd		%ymm13, %ymm15, %ymm10

	vbroadcastsd	96(%r13), %ymm14
	vfmadd231pd		%ymm12, %ymm14, %ymm3
	vbroadcastsd	224(%r13), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm7
	vfmadd231pd		%ymm13, %ymm15, %ymm11

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

	FUN_END(inner_edge_dgemm_nn_8x8u_lib4)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta
//
// input arguments:
// r10   <- &alpha
// r11   <- &beta
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
// ymm8  <- [d80 d91 da2 db3]
// ymm9  <- [d81 d90 da3 db2]
// ymm10 <- [d83 d92 da1 db0]
// ymm11 <- [d82 d93 da0 db1]
//
// output arguments:
// r10   <- &alpha
// r11   <- &beta
// r12   <- C
// r13   <- 4*sdc*sizeof(double)
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
	.macro INNER_SCALE_AB_8X8L_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_8x8l_lib4)
#endif
		

	vbroadcastsd 0(%r10), %ymm15 // alpha

	vmulpd		%ymm0, %ymm15, %ymm0
	vmulpd		%ymm1, %ymm15, %ymm1
	vmulpd		%ymm2, %ymm15, %ymm2
	vmulpd		%ymm3, %ymm15, %ymm3

	vmulpd		%ymm4, %ymm15, %ymm4
	vmulpd		%ymm5, %ymm15, %ymm5
	vmulpd		%ymm6, %ymm15, %ymm6
	vmulpd		%ymm7, %ymm15, %ymm7

	vmulpd		%ymm8, %ymm15, %ymm8
	vmulpd		%ymm9, %ymm15, %ymm9
	vmulpd		%ymm10, %ymm15, %ymm10
	vmulpd		%ymm11, %ymm15, %ymm11

	vbroadcastsd 0(%r11), %ymm14 // beta

	vxorpd		%ymm15, %ymm15, %ymm15 // 0.0

	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	vmovapd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm0
	vmovapd		32(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm1
	vmovapd		64(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm2
	vmovapd		96(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm3

	vmovapd		0(%r12, %r13, 1), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm4
	vmovapd		32(%r12, %r13, 1), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm5
	vmovapd		64(%r12, %r13, 1), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm6
	vmovapd		96(%r12, %r13, 1), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm7

	vmovapd		128(%r12, %r13, 1), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm8
	vmovapd		160(%r12, %r13, 1), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm9
	vmovapd		192(%r12, %r13, 1), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm10
	vmovapd		224(%r12, %r13, 1), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_8x8l_lib4)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta
//
// input arguments:
// r10   <- &alpha
// r11   <- &beta
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
// ymm8  <- [d80 d91 da2 db3]
// ymm9  <- [d81 d90 da3 db2]
// ymm10 <- [d83 d92 da1 db0]
// ymm11 <- [d82 d93 da0 db1]
//
// output arguments:
// r10   <- &alpha
// r11   <- &beta
// r12   <- C
// r13   <- 4*sdc*sizeof(double)
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
	.macro INNER_SCALE_AB_8X8U_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_8x8u_lib4)
#endif
		

	vbroadcastsd 0(%r10), %ymm15 // alpha

	vmulpd		%ymm0, %ymm15, %ymm0
	vmulpd		%ymm1, %ymm15, %ymm1
	vmulpd		%ymm2, %ymm15, %ymm2
	vmulpd		%ymm3, %ymm15, %ymm3

	vmulpd		%ymm4, %ymm15, %ymm4
	vmulpd		%ymm5, %ymm15, %ymm5
	vmulpd		%ymm6, %ymm15, %ymm6
	vmulpd		%ymm7, %ymm15, %ymm7

	vmulpd		%ymm8, %ymm15, %ymm8
	vmulpd		%ymm9, %ymm15, %ymm9
	vmulpd		%ymm10, %ymm15, %ymm10
	vmulpd		%ymm11, %ymm15, %ymm11

	vbroadcastsd 0(%r11), %ymm14 // beta

	vxorpd		%ymm15, %ymm15, %ymm15 // 0.0

	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	vmovapd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm0
	vmovapd		32(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm1
	vmovapd		64(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm2
	vmovapd		96(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm3

	vmovapd		128(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm4
	vmovapd		160(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm5
	vmovapd		192(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm6
	vmovapd		224(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm7

	vmovapd		128(%r12, %r13, 1), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm8
	vmovapd		160(%r12, %r13, 1), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm9
	vmovapd		192(%r12, %r13, 1), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm10
	vmovapd		224(%r12, %r13, 1), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_8x8u_lib4)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta
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
	.macro INNER_TRAN_SCALE_AB_8X8_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_tran_scale_ab_8x8_lib4)
#endif
		

	vunpcklpd	%ymm1, %ymm0, %ymm12
	vunpckhpd	%ymm1, %ymm0, %ymm13
	vunpcklpd	%ymm3, %ymm2, %ymm14
	vunpckhpd	%ymm3, %ymm2, %ymm15

	vperm2f128	$ 0x20, %ymm14, %ymm12, %ymm0
	vperm2f128	$ 0x31, %ymm14, %ymm12, %ymm2
	vperm2f128	$ 0x20, %ymm15, %ymm13, %ymm1
	vperm2f128	$ 0x31, %ymm15, %ymm13, %ymm3

	vbroadcastsd 0(%r10), %ymm15 // alpha

	vmulpd		%ymm0, %ymm15, %ymm0
	vmulpd		%ymm1, %ymm15, %ymm1
	vmulpd		%ymm2, %ymm15, %ymm2
	vmulpd		%ymm3, %ymm15, %ymm3

	vunpcklpd	%ymm5, %ymm4, %ymm12
	vunpckhpd	%ymm5, %ymm4, %ymm13
	vunpcklpd	%ymm7, %ymm6, %ymm14
	vunpckhpd	%ymm7, %ymm6, %ymm15

	vperm2f128	$ 0x20, %ymm14, %ymm12, %ymm4
	vperm2f128	$ 0x31, %ymm14, %ymm12, %ymm6
	vperm2f128	$ 0x20, %ymm15, %ymm13, %ymm5
	vperm2f128	$ 0x31, %ymm15, %ymm13, %ymm7

	vbroadcastsd 0(%r10), %ymm15 // alpha

	vmulpd		%ymm4, %ymm15, %ymm4
	vmulpd		%ymm5, %ymm15, %ymm5
	vmulpd		%ymm6, %ymm15, %ymm6
	vmulpd		%ymm7, %ymm15, %ymm7

	vunpcklpd	%ymm9, %ymm8, %ymm12
	vunpckhpd	%ymm9, %ymm8, %ymm13
	vunpcklpd	%ymm11, %ymm10, %ymm14
	vunpckhpd	%ymm11, %ymm10, %ymm15

	vperm2f128	$ 0x20, %ymm14, %ymm12, %ymm8
	vperm2f128	$ 0x31, %ymm14, %ymm12, %ymm10
	vperm2f128	$ 0x20, %ymm15, %ymm13, %ymm9
	vperm2f128	$ 0x31, %ymm15, %ymm13, %ymm11

	vbroadcastsd 0(%r10), %ymm15 // alpha

	vmulpd		%ymm8, %ymm15, %ymm8
	vmulpd		%ymm9, %ymm15, %ymm9
	vmulpd		%ymm10, %ymm15, %ymm10
	vmulpd		%ymm11, %ymm15, %ymm11

	vbroadcastsd 0(%r11), %ymm14 // beta

	vxorpd		%ymm15, %ymm15, %ymm15 // 0.0

	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	vmovapd		0(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm0
	vmovapd		32(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm1
	vmovapd		64(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm2
	vmovapd		96(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm3

	vmovapd		128(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm4
	vmovapd		160(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm5
	vmovapd		192(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm6
	vmovapd		224(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm7

	vmovapd		128(%r12, %r13, 1), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm8
	vmovapd		160(%r12, %r13, 1), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm9
	vmovapd		192(%r12, %r13, 1), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm10
	vmovapd		224(%r12, %r13, 1), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_tran_scale_ab_8x8_lib4)
#endif





// common inner routine with file scope
//
// scale for alpha=1.0 and beta=1.0
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
// ymm8  <- [d80 d90 da0 db0]
// ymm9  <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M11_8X8_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_scale_m11_8x8_lib4)
#endif
		
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd		.LC04(%rip), %ymm14 // beta=1.0
#else
	vmovapd		LC04(%rip), %ymm14 // beta=1.0
#endif

	vmovapd		0(%r10), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm0
	vmovapd		32(%r10), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm1
	vmovapd		64(%r10), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm2
	vmovapd		96(%r10), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm3

	vmovapd		0(%r10, %r11, 1), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm4
	vmovapd		32(%r10, %r11, 1), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm5
	vmovapd		64(%r10, %r11, 1), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm6
	vmovapd		96(%r10, %r11, 1), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm7

	vmovapd		128(%r10, %r11, 1), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm8
	vmovapd		160(%r10, %r11, 1), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm9
	vmovapd		192(%r10, %r11, 1), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm10
	vmovapd		224(%r10, %r11, 1), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m11_8x8_lib4)
#endif





// common inner routine with file scope
//
// scale for alpha=1.0 and beta=1.0
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
// ymm8  <- [d80 d90 da0 db0]
// ymm9  <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_TRAN_SCALE_M11_8X8_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_tran_scale_m11_8x8_lib4)
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

	vunpcklpd	%ymm9, %ymm8, %ymm12
	vunpckhpd	%ymm9, %ymm8, %ymm13
	vunpcklpd	%ymm11, %ymm10, %ymm14
	vunpckhpd	%ymm11, %ymm10, %ymm15

	vperm2f128	$ 0x20, %ymm14, %ymm12, %ymm8
	vperm2f128	$ 0x31, %ymm14, %ymm12, %ymm10
	vperm2f128	$ 0x20, %ymm15, %ymm13, %ymm9
	vperm2f128	$ 0x31, %ymm15, %ymm13, %ymm11

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd		.LC04(%rip), %ymm14 // beta=1.0
#else
	vmovapd		LC04(%rip), %ymm14 // beta=1.0
#endif

	vmovapd		0(%r10), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm0
	vmovapd		32(%r10), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm1
	vmovapd		64(%r10), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm2
	vmovapd		96(%r10), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm3

	vmovapd		128(%r10), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm4
	vmovapd		160(%r10), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm5
	vmovapd		192(%r10), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm6
	vmovapd		224(%r10), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm7

	vmovapd		128(%r10, %r11, 1), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm8
	vmovapd		160(%r10, %r11, 1), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm9
	vmovapd		192(%r10, %r11, 1), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm10
	vmovapd		224(%r10, %r11, 1), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_tran_scale_m11_8x8_lib4)
#endif





// common inner routine with file scope
//
// cholesky factorization 
//
// input arguments:
// r10  <- inv_diag_E
// r11d <- kn
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10  <- inv_diag_E
// r11d <- kn
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DPOTRF_8X8_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dpotrf_8x8_vs_lib4)
#endif
	
	vxorpd			%ymm15, %ymm15, %ymm15 // 0.0
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC04(%rip), %xmm14 // 1.0
#elif defined(OS_MAC)
	vmovsd			LC04(%rip), %xmm14 // 1.0
#endif

	vmovsd			%xmm0, %xmm0, %xmm13
	vucomisd		%xmm15, %xmm13 // d_00 > 0.0 ?
	jbe				1f
	vsqrtsd			%xmm13, %xmm13, %xmm13
	vdivsd			%xmm13, %xmm14, %xmm13
2:
	vmovsd			%xmm13, 0(%r10)
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmulpd			%ymm0, %ymm13, %ymm0
	vmulpd			%ymm4, %ymm13, %ymm4
	vpermpd			$ 0x55, %ymm0, %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm1
	vfnmadd231pd	%ymm4, %ymm13, %ymm5
	vperm2f128		$ 0x11, %ymm0, %ymm0, %ymm12
	vpermilpd		$ 0x0, %ymm12, %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm2
	vfnmadd231pd	%ymm4, %ymm13, %ymm6
	vpermilpd		$ 0xf, %ymm12, %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm3
	vfnmadd231pd	%ymm4, %ymm13, %ymm7
	vperm2f128		$ 0x00, %ymm4, %ymm4, %ymm12
	vpermilpd		$ 0x0, %ymm12, %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm8
	vpermilpd		$ 0xf, %ymm12, %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm9
	vperm2f128		$ 0x11, %ymm4, %ymm4, %ymm12
	vpermilpd		$ 0x0, %ymm12, %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm10
	vpermilpd		$ 0xf, %ymm12, %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm11

	vpermilpd		$ 0x3, %xmm1, %xmm13
	vucomisd		%xmm15, %xmm13 // d_11 > 0.0 ?
	jbe				3f
	vsqrtsd			%xmm13, %xmm13, %xmm13
	vdivsd			%xmm13, %xmm14, %xmm13
4:
	vmovsd			%xmm13, 8(%r10)
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmulpd			%ymm1, %ymm13, %ymm1
	vmulpd			%ymm5, %ymm13, %ymm5
	vperm2f128		$ 0x11, %ymm1, %ymm1, %ymm12
	vpermilpd		$ 0x0, %ymm12, %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm2
	vfnmadd231pd	%ymm5, %ymm13, %ymm6
	vpermilpd		$ 0xf, %ymm12, %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm3
	vfnmadd231pd	%ymm5, %ymm13, %ymm7
	vperm2f128		$ 0x00, %ymm5, %ymm5, %ymm12
	vpermilpd		$ 0x0, %ymm12, %ymm13
	vfnmadd231pd	%ymm5, %ymm13, %ymm8
	vpermilpd		$ 0xf, %ymm12, %ymm13
	vfnmadd231pd	%ymm5, %ymm13, %ymm9
	vperm2f128		$ 0x11, %ymm5, %ymm5, %ymm12
	vpermilpd		$ 0x0, %ymm12, %ymm13
	vfnmadd231pd	%ymm5, %ymm13, %ymm10
	vpermilpd		$ 0xf, %ymm12, %ymm13
	vfnmadd231pd	%ymm5, %ymm13, %ymm11

	vextractf128	$ 0x1, %ymm2, %xmm13
	vucomisd		%xmm15, %xmm13 // d_22 > 0.0 ?
	jbe				5f
	vsqrtsd			%xmm13, %xmm13, %xmm13
	vdivsd			%xmm13, %xmm14, %xmm13
6:
	vmovsd			%xmm13, 16(%r10)
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmulpd			%ymm2, %ymm13, %ymm2
	vmulpd			%ymm6, %ymm13, %ymm6
	vpermpd			$ 0xff, %ymm2, %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm3
	vfnmadd231pd	%ymm6, %ymm13, %ymm7
	vperm2f128		$ 0x00, %ymm6, %ymm6, %ymm12
	vpermilpd		$ 0x0, %ymm12, %ymm13
	vfnmadd231pd	%ymm6, %ymm13, %ymm8
	vpermilpd		$ 0xf, %ymm12, %ymm13
	vfnmadd231pd	%ymm6, %ymm13, %ymm9
	vperm2f128		$ 0x11, %ymm6, %ymm6, %ymm12
	vpermilpd		$ 0x0, %ymm12, %ymm13
	vfnmadd231pd	%ymm6, %ymm13, %ymm10
	vpermilpd		$ 0xf, %ymm12, %ymm13
	vfnmadd231pd	%ymm6, %ymm13, %ymm11

	vpermpd			$ 0xff, %ymm3, %ymm13
	vucomisd		%xmm15, %xmm13 // d_33 > 0.0 ?
	jbe				7f
	vsqrtsd			%xmm13, %xmm13, %xmm13
	vdivsd			%xmm13, %xmm14, %xmm13
8:
	vmovsd			%xmm13, 24(%r10)
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmulpd			%ymm3, %ymm13, %ymm3
	vmulpd			%ymm7, %ymm13, %ymm7
	vperm2f128		$ 0x00, %ymm7, %ymm7, %ymm12
	vpermilpd		$ 0x0, %ymm12, %ymm13
	vfnmadd231pd	%ymm7, %ymm13, %ymm8
	vpermilpd		$ 0xf, %ymm12, %ymm13
	vfnmadd231pd	%ymm7, %ymm13, %ymm9
	vperm2f128		$ 0x11, %ymm7, %ymm7, %ymm12
	vpermilpd		$ 0x0, %ymm12, %ymm13
	vfnmadd231pd	%ymm7, %ymm13, %ymm10
	vpermilpd		$ 0xf, %ymm12, %ymm13
	vfnmadd231pd	%ymm7, %ymm13, %ymm11

	vmovsd			%xmm8, %xmm8, %xmm13
	vucomisd		%xmm15, %xmm13 // d_00 > 0.0 ?
	jbe				9f
	vsqrtsd			%xmm13, %xmm13, %xmm13
	vdivsd			%xmm13, %xmm14, %xmm13
10:
	vmovsd			%xmm13, 32(%r10)
//	vmovddup		%xmm13, %xmm13
//	vperm2f128		$ 0x00, %ymm13, %ymm13, %ymm13
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmulpd			%ymm8, %ymm13, %ymm8
	cmpl			$ 6, %r11d
	jl				0f // ret
//	vperm2f128		$ 0x00, %ymm8, %ymm8, %ymm12
//	vpermilpd		$ 0xf, %ymm12, %ymm13
	vpermpd			$ 0x55, %ymm8, %ymm13
	vfnmadd231pd	%ymm8, %ymm13, %ymm9
	vperm2f128		$ 0x11, %ymm8, %ymm8, %ymm12
	vpermilpd		$ 0x0, %ymm12, %ymm13
	vfnmadd231pd	%ymm8, %ymm13, %ymm10
	vpermilpd		$ 0xf, %ymm12, %ymm13
	vfnmadd231pd	%ymm8, %ymm13, %ymm11

	vpermilpd		$ 0x3, %xmm9, %xmm13
	vucomisd		%xmm15, %xmm13 // d_11 > 0.0 ?
	jbe				11f
	vsqrtsd			%xmm13, %xmm13, %xmm13
	vdivsd			%xmm13, %xmm14, %xmm13
12:
	vmovsd			%xmm13, 40(%r10)
//	vmovddup		%xmm13, %xmm13
//	vperm2f128		$ 0x00, %ymm13, %ymm13, %ymm13
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmulpd			%ymm9, %ymm13, %ymm9
	cmpl			$ 7, %r11d
	jl				0f // ret
	vperm2f128		$ 0x11, %ymm9, %ymm9, %ymm12
	vpermilpd		$ 0x0, %ymm12, %ymm13
	vfnmadd231pd	%ymm9, %ymm13, %ymm10
	vpermilpd		$ 0xf, %ymm12, %ymm13
	vfnmadd231pd	%ymm9, %ymm13, %ymm11

	vextractf128	$ 0x1, %ymm10, %xmm13
	vucomisd		%xmm15, %xmm13 // d_22 > 0.0 ?
	jbe				13f
	vsqrtsd			%xmm13, %xmm13, %xmm13
	vdivsd			%xmm13, %xmm14, %xmm13
14:
	vmovsd			%xmm13, 48(%r10)
//	vmovddup		%xmm13, %xmm13
//	vperm2f128		$ 0x00, %ymm13, %ymm13, %ymm13
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmulpd			%ymm10, %ymm13, %ymm10
	cmpl			$ 8, %r11d
	jl				0f // ret
//	vperm2f128		$ 0x11, %ymm10, %ymm10, %ymm12
//	vpermilpd		$ 0xf, %ymm12, %ymm13
	vpermpd			$ 0xff, %ymm10, %ymm13
	vfnmadd231pd	%ymm10, %ymm13, %ymm11

//	vextractf128	$ 0x1, %ymm11, %xmm13
//	vpermilpd		$ 0x3, %xmm13, %xmm13
	vpermpd			$ 0xff, %ymm11, %ymm13
	vucomisd		%xmm15, %xmm13 // d_33 > 0.0 ?
	jbe				15f
	vsqrtsd			%xmm13, %xmm13, %xmm13
	vdivsd			%xmm13, %xmm14, %xmm13
16:
	vmovsd			%xmm13, 56(%r10)
//	vmovddup		%xmm13, %xmm13
//	vperm2f128		$ 0x00, %ymm13, %ymm13, %ymm13
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmulpd			%ymm11, %ymm13, %ymm11



	jmp				0f

1:
	vxorpd			%ymm13, %ymm13, %ymm13
	jmp				2b

3:
	vxorpd			%ymm13, %ymm13, %ymm13
	jmp				4b

5:
	vxorpd			%ymm13, %ymm13, %ymm13
	jmp				6b

7:
	vxorpd			%ymm13, %ymm13, %ymm13
	jmp				8b

9:
	vxorpd			%ymm13, %ymm13, %ymm13
	jmp				10b

11:
	vxorpd			%ymm13, %ymm13, %ymm13
	jmp				12b

13:
	vxorpd			%ymm13, %ymm13, %ymm13
	jmp				14b

15:
	vxorpd			%ymm13, %ymm13, %ymm13
	jmp				16b

0:
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dpotrf_8x8_vs_lib4)
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
	.macro INNER_EDGE_DTRSM_RLT_INV_8X8L_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rlt_inv_8x8l_lib4)
#endif
	
	vbroadcastsd	0(%r12), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm0
	vmulpd			%ymm4, %ymm13, %ymm4
	vbroadcastsd	8(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm1
	vfnmadd231pd	%ymm4, %ymm13, %ymm5
	vbroadcastsd	16(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm2
	vfnmadd231pd	%ymm4, %ymm13, %ymm6
	vbroadcastsd	24(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm3
	vfnmadd231pd	%ymm4, %ymm13, %ymm7
	vbroadcastsd	0(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm8
	vbroadcastsd	8(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm9
	vbroadcastsd	16(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm10
	vbroadcastsd	24(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm11

	vbroadcastsd	8(%r12), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm1
	vmulpd			%ymm5, %ymm13, %ymm5
	vbroadcastsd	48(%r10), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm2
	vfnmadd231pd	%ymm5, %ymm13, %ymm6
	vbroadcastsd	56(%r10), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm3
	vfnmadd231pd	%ymm5, %ymm13, %ymm7
	vbroadcastsd	32(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm5, %ymm13, %ymm8
	vbroadcastsd	40(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm5, %ymm13, %ymm9
	vbroadcastsd	48(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm5, %ymm13, %ymm10
	vbroadcastsd	56(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm5, %ymm13, %ymm11

	vbroadcastsd	16(%r12), %ymm13
	vmulpd			%ymm2, %ymm13, %ymm2
	vmulpd			%ymm6, %ymm13, %ymm6
	vbroadcastsd	88(%r10), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm3
	vfnmadd231pd	%ymm6, %ymm13, %ymm7
	vbroadcastsd	64(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm6, %ymm13, %ymm8
	vbroadcastsd	72(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm6, %ymm13, %ymm9
	vbroadcastsd	80(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm6, %ymm13, %ymm10
	vbroadcastsd	88(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm6, %ymm13, %ymm11

	vbroadcastsd	24(%r12), %ymm13
	vmulpd			%ymm3, %ymm13, %ymm3
	vmulpd			%ymm7, %ymm13, %ymm7
	vbroadcastsd	96(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm7, %ymm13, %ymm8
	vbroadcastsd	104(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm7, %ymm13, %ymm9
	vbroadcastsd	112(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm7, %ymm13, %ymm10
	vbroadcastsd	120(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm7, %ymm13, %ymm11
	addq	$ 128, %r10

	vbroadcastsd	32(%r12), %ymm13
	vmulpd			%ymm8, %ymm13, %ymm8
	vbroadcastsd	8(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm8, %ymm13, %ymm9
	vbroadcastsd	16(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm8, %ymm13, %ymm10
	vbroadcastsd	24(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm8, %ymm13, %ymm11

	vbroadcastsd	40(%r12), %ymm13
	vmulpd			%ymm9, %ymm13, %ymm9
	vbroadcastsd	48(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm9, %ymm13, %ymm10
	vbroadcastsd	56(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm9, %ymm13, %ymm11

	vbroadcastsd	48(%r12), %ymm13
	vmulpd			%ymm10, %ymm13, %ymm10
	vbroadcastsd	88(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm10, %ymm13, %ymm11

	vbroadcastsd	56(%r12), %ymm13
	vmulpd			%ymm11, %ymm13, %ymm11

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rlt_inv_8x8l_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution for cholesky factorization 
//
// input arguments:
// r10  <- E
// r11  <- sde
// r12  <- inv_diag_E
// r13  <- D
// r14  <- sdd
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
// r11  <- sde
// r12  <- inv_diag_E
// r13  <- D
// r14  <- sdd
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
	.macro INNER_EDGE_DTRSM_RLT_INV_8X8U_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rlt_inv_8x8u_lib4)
#endif
	
	vbroadcastsd	0(%r12), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm0
	vbroadcastsd	8(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm1
	vbroadcastsd	16(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm2
	vbroadcastsd	24(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm3

	vmovapd			0(%r13, %r14, 1), %ymm12
	vbroadcastsd	0(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm4
	vfnmadd231pd	%ymm12, %ymm13, %ymm8
	vbroadcastsd	8(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm5
	vfnmadd231pd	%ymm12, %ymm13, %ymm9
	vbroadcastsd	16(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm6
	vfnmadd231pd	%ymm12, %ymm13, %ymm10
	vbroadcastsd	24(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm7
	vfnmadd231pd	%ymm12, %ymm13, %ymm11


	vbroadcastsd	8(%r12), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm1
	vbroadcastsd	48(%r10), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm2
	vbroadcastsd	56(%r10), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm3

	vmovapd			32(%r13, %r14, 1), %ymm12
	vbroadcastsd	32(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm4
	vfnmadd231pd	%ymm12, %ymm13, %ymm8
	vbroadcastsd	40(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm5
	vfnmadd231pd	%ymm12, %ymm13, %ymm9
	vbroadcastsd	48(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm6
	vfnmadd231pd	%ymm12, %ymm13, %ymm10
	vbroadcastsd	56(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm7
	vfnmadd231pd	%ymm12, %ymm13, %ymm11


	vbroadcastsd	16(%r12), %ymm13
	vmulpd			%ymm2, %ymm13, %ymm2
	vbroadcastsd	88(%r10), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm3

	vmovapd			64(%r13, %r14, 1), %ymm12
	vbroadcastsd	64(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm4
	vfnmadd231pd	%ymm12, %ymm13, %ymm8
	vbroadcastsd	72(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm5
	vfnmadd231pd	%ymm12, %ymm13, %ymm9
	vbroadcastsd	80(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm6
	vfnmadd231pd	%ymm12, %ymm13, %ymm10
	vbroadcastsd	88(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm7
	vfnmadd231pd	%ymm12, %ymm13, %ymm11


	vbroadcastsd	24(%r12), %ymm13
	vmulpd			%ymm3, %ymm13, %ymm3

	vmovapd			96(%r13, %r14, 1), %ymm12
	vbroadcastsd	96(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm4
	vfnmadd231pd	%ymm12, %ymm13, %ymm8
	vbroadcastsd	104(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm5
	vfnmadd231pd	%ymm12, %ymm13, %ymm9
	vbroadcastsd	112(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm6
	vfnmadd231pd	%ymm12, %ymm13, %ymm10
	vbroadcastsd	120(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm7
	vfnmadd231pd	%ymm12, %ymm13, %ymm11

	addq	$ 128, %r10

	vbroadcastsd	32(%r12), %ymm13
	vmulpd			%ymm4, %ymm13, %ymm4
	vmulpd			%ymm8, %ymm13, %ymm8
	vbroadcastsd	8(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm5
	vfnmadd231pd	%ymm8, %ymm13, %ymm9
	vbroadcastsd	16(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm6
	vfnmadd231pd	%ymm8, %ymm13, %ymm10
	vbroadcastsd	24(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm7
	vfnmadd231pd	%ymm8, %ymm13, %ymm11

	vbroadcastsd	40(%r12), %ymm13
	vmulpd			%ymm5, %ymm13, %ymm5
	vmulpd			%ymm9, %ymm13, %ymm9
	vbroadcastsd	48(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm5, %ymm13, %ymm6
	vfnmadd231pd	%ymm9, %ymm13, %ymm10
	vbroadcastsd	56(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm5, %ymm13, %ymm7
	vfnmadd231pd	%ymm9, %ymm13, %ymm11

	vbroadcastsd	48(%r12), %ymm13
	vmulpd			%ymm6, %ymm13, %ymm6
	vmulpd			%ymm10, %ymm13, %ymm10
	vbroadcastsd	88(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm6, %ymm13, %ymm7
	vfnmadd231pd	%ymm10, %ymm13, %ymm11

	vbroadcastsd	56(%r12), %ymm13
	vmulpd			%ymm7, %ymm13, %ymm7
	vmulpd			%ymm11, %ymm13, %ymm11

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rlt_inv_8x8u_lib4)
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
	.macro INNER_EDGE_DTRSM_RLT_INV_8X8L_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rlt_inv_8x8l_vs_lib4)
#endif
	
	vbroadcastsd	0(%r12), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm0
	vmulpd			%ymm4, %ymm13, %ymm4
	vbroadcastsd	8(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm1
	vfnmadd231pd	%ymm4, %ymm13, %ymm5
	vbroadcastsd	16(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm2
	vfnmadd231pd	%ymm4, %ymm13, %ymm6
	vbroadcastsd	24(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm3
	vfnmadd231pd	%ymm4, %ymm13, %ymm7
	vbroadcastsd	0(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm8
	vbroadcastsd	8(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm9
	vbroadcastsd	16(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm10
	vbroadcastsd	24(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm11

	vbroadcastsd	8(%r12), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm1
	vmulpd			%ymm5, %ymm13, %ymm5
	vbroadcastsd	48(%r10), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm2
	vfnmadd231pd	%ymm5, %ymm13, %ymm6
	vbroadcastsd	56(%r10), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm3
	vfnmadd231pd	%ymm5, %ymm13, %ymm7
	vbroadcastsd	32(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm5, %ymm13, %ymm8
	vbroadcastsd	40(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm5, %ymm13, %ymm9
	vbroadcastsd	48(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm5, %ymm13, %ymm10
	vbroadcastsd	56(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm5, %ymm13, %ymm11

	vbroadcastsd	16(%r12), %ymm13
	vmulpd			%ymm2, %ymm13, %ymm2
	vmulpd			%ymm6, %ymm13, %ymm6
	vbroadcastsd	88(%r10), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm3
	vfnmadd231pd	%ymm6, %ymm13, %ymm7
	vbroadcastsd	64(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm6, %ymm13, %ymm8
	vbroadcastsd	72(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm6, %ymm13, %ymm9
	vbroadcastsd	80(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm6, %ymm13, %ymm10
	vbroadcastsd	88(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm6, %ymm13, %ymm11

	vbroadcastsd	24(%r12), %ymm13
	vmulpd			%ymm3, %ymm13, %ymm3
	vmulpd			%ymm7, %ymm13, %ymm7
	vbroadcastsd	96(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm7, %ymm13, %ymm8
	vbroadcastsd	104(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm7, %ymm13, %ymm9
	vbroadcastsd	112(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm7, %ymm13, %ymm10
	vbroadcastsd	120(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm7, %ymm13, %ymm11
	addq	$ 128, %r10

	vbroadcastsd	32(%r12), %ymm13
	vmulpd			%ymm8, %ymm13, %ymm8
	cmpl			$ 6, %r13d
	jl				0f // ret
	vbroadcastsd	8(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm8, %ymm13, %ymm9
	vbroadcastsd	16(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm8, %ymm13, %ymm10
	vbroadcastsd	24(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm8, %ymm13, %ymm11

	vbroadcastsd	40(%r12), %ymm13
	vmulpd			%ymm9, %ymm13, %ymm9
	cmpl			$ 7, %r13d
	jl				0f // ret
	vbroadcastsd	48(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm9, %ymm13, %ymm10
	vbroadcastsd	56(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm9, %ymm13, %ymm11

	vbroadcastsd	48(%r12), %ymm13
	vmulpd			%ymm10, %ymm13, %ymm10
	cmpl			$ 8, %r13d
	jl				0f // ret
	vbroadcastsd	88(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm10, %ymm13, %ymm11

	vbroadcastsd	56(%r12), %ymm13
	vmulpd			%ymm11, %ymm13, %ymm11

0:
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rlt_inv_8x8l_vs_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution for cholesky factorization 
//
// input arguments:
// r10  <- E
// r11  <- sde
// r12  <- inv_diag_E
// r13  <- D
// r14  <- sdd
// r15d <- kn
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
// r11  <- sde
// r12  <- inv_diag_E
// r13  <- D
// r14  <- sdd
// r15d <- kn
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
	.macro INNER_EDGE_DTRSM_RLT_INV_8X8U_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rlt_inv_8x8u_vs_lib4)
#endif
	
	vbroadcastsd	0(%r12), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm0
	vbroadcastsd	8(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm1
	vbroadcastsd	16(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm2
	vbroadcastsd	24(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm3

	vmovapd			0(%r13, %r14, 1), %ymm12
	vbroadcastsd	0(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm4
	vfnmadd231pd	%ymm12, %ymm13, %ymm8
	vbroadcastsd	8(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm5
	vfnmadd231pd	%ymm12, %ymm13, %ymm9
	vbroadcastsd	16(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm6
	vfnmadd231pd	%ymm12, %ymm13, %ymm10
	vbroadcastsd	24(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm7
	vfnmadd231pd	%ymm12, %ymm13, %ymm11


	vbroadcastsd	8(%r12), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm1
	vbroadcastsd	48(%r10), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm2
	vbroadcastsd	56(%r10), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm3

	vmovapd			32(%r13, %r14, 1), %ymm12
	vbroadcastsd	32(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm4
	vfnmadd231pd	%ymm12, %ymm13, %ymm8
	vbroadcastsd	40(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm5
	vfnmadd231pd	%ymm12, %ymm13, %ymm9
	vbroadcastsd	48(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm6
	vfnmadd231pd	%ymm12, %ymm13, %ymm10
	vbroadcastsd	56(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm7
	vfnmadd231pd	%ymm12, %ymm13, %ymm11


	vbroadcastsd	16(%r12), %ymm13
	vmulpd			%ymm2, %ymm13, %ymm2
	vbroadcastsd	88(%r10), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm3

	vmovapd			64(%r13, %r14, 1), %ymm12
	vbroadcastsd	64(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm4
	vfnmadd231pd	%ymm12, %ymm13, %ymm8
	vbroadcastsd	72(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm5
	vfnmadd231pd	%ymm12, %ymm13, %ymm9
	vbroadcastsd	80(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm6
	vfnmadd231pd	%ymm12, %ymm13, %ymm10
	vbroadcastsd	88(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm7
	vfnmadd231pd	%ymm12, %ymm13, %ymm11


	vbroadcastsd	24(%r12), %ymm13
	vmulpd			%ymm3, %ymm13, %ymm3

	vmovapd			96(%r13, %r14, 1), %ymm12
	vbroadcastsd	96(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm4
	vfnmadd231pd	%ymm12, %ymm13, %ymm8
	vbroadcastsd	104(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm5
	vfnmadd231pd	%ymm12, %ymm13, %ymm9
	vbroadcastsd	112(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm6
	vfnmadd231pd	%ymm12, %ymm13, %ymm10
	vbroadcastsd	120(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm7
	vfnmadd231pd	%ymm12, %ymm13, %ymm11

	addq	$ 128, %r10

	vbroadcastsd	32(%r12), %ymm13
	vmulpd			%ymm4, %ymm13, %ymm4
	vmulpd			%ymm8, %ymm13, %ymm8
	cmpl			$ 6, %r15d
	jl				0f // ret
	vbroadcastsd	8(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm5
	vfnmadd231pd	%ymm8, %ymm13, %ymm9
	vbroadcastsd	16(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm6
	vfnmadd231pd	%ymm8, %ymm13, %ymm10
	vbroadcastsd	24(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm7
	vfnmadd231pd	%ymm8, %ymm13, %ymm11

	vbroadcastsd	40(%r12), %ymm13
	vmulpd			%ymm5, %ymm13, %ymm5
	vmulpd			%ymm9, %ymm13, %ymm9
	cmpl			$ 7, %r15d
	jl				0f // ret
	vbroadcastsd	48(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm5, %ymm13, %ymm6
	vfnmadd231pd	%ymm9, %ymm13, %ymm10
	vbroadcastsd	56(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm5, %ymm13, %ymm7
	vfnmadd231pd	%ymm9, %ymm13, %ymm11

	vbroadcastsd	48(%r12), %ymm13
	vmulpd			%ymm6, %ymm13, %ymm6
	vmulpd			%ymm10, %ymm13, %ymm10
	cmpl			$ 8, %r15d
	jl				0f // ret
	vbroadcastsd	88(%r10, %r11, 1), %ymm13
	vfnmadd231pd	%ymm6, %ymm13, %ymm7
	vfnmadd231pd	%ymm10, %ymm13, %ymm11

	vbroadcastsd	56(%r12), %ymm13
	vmulpd			%ymm7, %ymm13, %ymm7
	vmulpd			%ymm11, %ymm13, %ymm11



//	subq	$ 128, %r10
//	vmovapd	0(%r10, %r11, 1), %ymm4
//	vmovapd	32(%r10, %r11, 1), %ymm5
//	vmovapd	64(%r10, %r11, 1), %ymm6
//	vmovapd	96(%r10, %r11, 1), %ymm7



0:
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rlt_inv_8x8u_vs_lib4)
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
	.macro INNER_STORE_8X8L_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_8x8l_lib4)
#endif
	
	vmovapd %ymm0,  0(%r10)
	vmovapd %ymm1, 32(%r10)
	vmovapd %ymm2, 64(%r10)
	vmovapd %ymm3, 96(%r10)

	vmovapd %ymm4,  0(%r10, %r11, 1)
	vmovapd %ymm5, 32(%r10, %r11, 1)
	vmovapd %ymm6, 64(%r10, %r11, 1)
	vmovapd %ymm7, 96(%r10, %r11, 1)

	vmovapd %ymm8,  128(%r10, %r11, 1)
	vmovapd %ymm9,  160(%r10, %r11, 1)
	vmovapd %ymm10, 192(%r10, %r11, 1)
	vmovapd %ymm11, 224(%r10, %r11, 1)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_8x8l_lib4)
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
	.macro INNER_STORE_8X8U_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_8x8u_lib4)
#endif
	
	vmovapd %ymm0,  0(%r10)
	vmovapd %ymm1, 32(%r10)
	vmovapd %ymm2, 64(%r10)
	vmovapd %ymm3, 96(%r10)

	vmovapd %ymm4, 128(%r10)
	vmovapd %ymm5, 160(%r10)
	vmovapd %ymm6, 192(%r10)
	vmovapd %ymm7, 224(%r10)

	vmovapd %ymm8,  128(%r10, %r11, 1)
	vmovapd %ymm9,  160(%r10, %r11, 1)
	vmovapd %ymm10, 192(%r10, %r11, 1)
	vmovapd %ymm11, 224(%r10, %r11, 1)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_8x8u_lib4)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10   <- D
// r11   <- 4*sdd*sizeof(double)
// r12d  <- km
// r13d  <- kn
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
// r12d  <- km
// r13d  <- kn
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
	.macro INNER_STORE_8X8L_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_8x8l_vs_lib4)
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

	vmovapd %ymm0,  0(%r10)
	vmovapd %ymm1, 32(%r10)
	vmovapd %ymm2, 64(%r10)
	vmovapd %ymm3, 96(%r10)

	vmaskmovpd	%ymm4, %ymm15,  0(%r10, %r11, 1)
	vmaskmovpd	%ymm5, %ymm15, 32(%r10, %r11, 1)
	vmaskmovpd	%ymm6, %ymm15, 64(%r10, %r11, 1)
	vmaskmovpd	%ymm7, %ymm15, 96(%r10, %r11, 1)

	vmaskmovpd	%ymm8, %ymm15, 128(%r10, %r11, 1)
	cmpl		$ 6, %r13d
	jl			0f // end
	vmaskmovpd	%ymm9, %ymm15, 160(%r10, %r11, 1)
	cmpl		$ 7, %r13d
	jl			0f // end
	vmaskmovpd	%ymm10, %ymm15, 192(%r10, %r11, 1)
	je			0f // end
	vmaskmovpd	%ymm11, %ymm15, 224(%r10, %r11, 1)

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_8x8l_vs_lib4)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10   <- D
// r11   <- 4*sdd*sizeof(double)
// r12d  <- km
// r13d  <- kn
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
// r12d  <- km
// r13d  <- kn
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
	.macro INNER_STORE_8X8U_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_8x8u_vs_lib4)
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

	vmovapd %ymm0,  0(%r10)
	vmovapd %ymm1, 32(%r10)
	vmovapd %ymm2, 64(%r10)
	vmovapd %ymm3, 96(%r10)


	vmovapd		%ymm4, 128(%r10)
	vmaskmovpd	%ymm8, %ymm15, 128(%r10, %r11, 1)
	cmpl		$ 6, %r13d
	jl			0f // end
	vmovapd		%ymm5, 160(%r10)
	vmaskmovpd	%ymm9, %ymm15, 160(%r10, %r11, 1)
	cmpl		$ 7, %r13d
	jl			0f // end
	vmovapd		%ymm6, 192(%r10)
	vmaskmovpd	%ymm10, %ymm15, 192(%r10, %r11, 1)
	je			0f // end
	vmovapd		%ymm7, 224(%r10)
	vmaskmovpd	%ymm11, %ymm15, 224(%r10, %r11, 1)

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_8x8u_vs_lib4)
#endif





// common inner routine with file scope
//
// store lower n
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
// ymm5  <- [d41 d50 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- [d80 d90 da0 db0]
// ymm9  <- [d81 d90 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]
// ymm14 <- dirty
// ymm15 <- dirty
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
// ymm9  <- [d81 d90 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_8X8_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_l_8x8_lib4)
#endif
	
	vmovapd		%ymm0, 0(%r10)
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

	vmovapd		%ymm8, 128(%r10, %r11, 1)
	vmovapd		160(%r10, %r11, 1), %ymm14
	vblendpd	$ 0x1, %ymm14, %ymm9, %ymm9
	vmovapd		%ymm9, 160(%r10, %r11, 1)
	vmovapd		192(%r10, %r11, 1), %ymm14
	vblendpd	$ 0x3, %ymm14, %ymm10, %ymm10
	vmovapd		%ymm10, 192(%r10, %r11, 1)
	vmovapd		224(%r10, %r11, 1), %ymm14
	vblendpd	$ 0x7, %ymm14, %ymm11, %ymm11
	vmovapd		%ymm11, 224(%r10, %r11, 1)

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_8x8_lib4)
#endif





// common inner routine with file scope
//
// store lower n
//
// input arguments:
// r10   <- D
// r11   <- 4*sdd*sizeof(double)
// r12d  <- km
// r13d  <- kn
// r14   <- dirty
// r15   <- dirty
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d50 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- [d80 d90 da0 db0]
// ymm9  <- [d81 d90 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]
// ymm14 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10   <- D
// r11   <- 4*sdd*sizeof(double)
// r12d  <- km
// r13d  <- kn
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
// ymm9  <- [d81 d90 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_8X8_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_l_8x8_vs_lib4)
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

	vmovapd		%ymm0, 0(%r10)
	vmovapd		32(%r10), %ymm14
	vblendpd	$ 0x1, %ymm14, %ymm1, %ymm1	
	vmovapd		%ymm1, 32(%r10)
	vmovapd		64(%r10), %ymm14
	vblendpd	$ 0x3, %ymm14, %ymm2, %ymm2	
	vmovapd		%ymm2, 64(%r10)
	vmovapd		96(%r10), %ymm14
	vblendpd	$ 0x7, %ymm14, %ymm3, %ymm3	
	vmovapd		%ymm3, 96(%r10)

	vmaskmovpd	%ymm4, %ymm15,  0(%r10, %r11, 1)
	vmaskmovpd	%ymm5, %ymm15, 32(%r10, %r11, 1)
	vmaskmovpd	%ymm6, %ymm15, 64(%r10, %r11, 1)
	vmaskmovpd	%ymm7, %ymm15, 96(%r10, %r11, 1)

	vmaskmovpd	%ymm8, %ymm15, 128(%r10, %r11, 1)
	cmpl		$ 6, %r13d
	jl			0f // end
	vmovapd		160(%r10, %r11, 1), %ymm14
	vblendpd	$ 0x1, %ymm14, %ymm9, %ymm9
	vmaskmovpd	%ymm9, %ymm15, 160(%r10, %r11, 1)
	cmpl		$ 7, %r13d
	jl			0f // end
	vmovapd		192(%r10, %r11, 1), %ymm14
	vblendpd	$ 0x3, %ymm14, %ymm10, %ymm10
	vmaskmovpd	%ymm10, %ymm15, 192(%r10, %r11, 1)
	je			0f // end
	vmovapd		224(%r10, %r11, 1), %ymm14
	vblendpd	$ 0x7, %ymm14, %ymm11, %ymm11
	vmaskmovpd	%ymm11, %ymm15, 224(%r10, %r11, 1)

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_8x8_vs_lib4)
#endif





// common inner routine with file scope
//
// store upper n
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
// ymm5  <- [d41 d50 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- [d80 d90 da0 db0]
// ymm9  <- [d81 d90 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]
// ymm14 <- dirty
// ymm15 <- dirty
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
// ymm9  <- [d81 d90 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_STORE_U_8X8_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_u_8x8_lib4)
#endif
	
	//
	vmovapd		0(%r10), %ymm14
	vblendpd	$ 0x1, %ymm0, %ymm14, %ymm0	
	vmovapd		%ymm0, 0(%r10)
	//
	vmovapd		32(%r10), %ymm14
	vblendpd	$ 0x3, %ymm1, %ymm14, %ymm1	
	vmovapd		%ymm1, 32(%r10)
	//
	vmovapd		64(%r10), %ymm14
	vblendpd	$ 0x7, %ymm2, %ymm14, %ymm2	
	vmovapd		%ymm2, 64(%r10)
	//
	vmovapd		%ymm3, 96(%r10)

	//
	vmovapd		%ymm4, 128(%r10)
	//
	vmovapd		%ymm5, 160(%r10)
	//
	vmovapd		%ymm6, 192(%r10)
	//
	vmovapd		%ymm7, 224(%r10)

	//
	vmovapd		128(%r10, %r11, 1), %ymm14
	vblendpd	$ 0x1, %ymm8, %ymm14, %ymm8
	vmovapd		%ymm8, 128(%r10, %r11, 1)
	//
	vmovapd		160(%r10, %r11, 1), %ymm14
	vblendpd	$ 0x3, %ymm9, %ymm14, %ymm9
	vmovapd		%ymm9, 160(%r10, %r11, 1)
	//
	vmovapd		192(%r10, %r11, 1), %ymm14
	vblendpd	$ 0x7, %ymm10, %ymm14, %ymm10
	vmovapd		%ymm10, 192(%r10, %r11, 1)
	//
	vmovapd		%ymm11, 224(%r10, %r11, 1)

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_u_8x8_lib4)
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
	.macro INNER_STORE_U_8X8_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_u_8x8_vs_lib4)
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

	//
	vmovapd		0(%r10), %ymm14
	vblendpd	$ 0x1, %ymm0, %ymm14, %ymm0	
	vmovapd		%ymm0,  0(%r10)
	//
	vmovapd		32(%r10), %ymm14
	vblendpd	$ 0x3, %ymm1, %ymm14, %ymm1	
	vmovapd		%ymm1, 32(%r10)
	//
	vmovapd		64(%r10), %ymm14
	vblendpd	$ 0x7, %ymm2, %ymm14, %ymm2	
	vmovapd		%ymm2, 64(%r10)
	//
	vmovapd		%ymm3, 96(%r10)


	//
	vmovapd		128(%r10, %r11, 1), %ymm14
	vblendpd	$ 0x1, %ymm8, %ymm14, %ymm8
	vmovapd		%ymm4, 128(%r10)
	vmaskmovpd	%ymm8, %ymm15, 128(%r10, %r11, 1)
	cmpl		$ 6, %r13d
	jl			0f // end
	//
	vmovapd		160(%r10, %r11, 1), %ymm14
	vblendpd	$ 0x3, %ymm9, %ymm14, %ymm9
	vmovapd		%ymm5, 160(%r10)
	vmaskmovpd	%ymm9, %ymm15, 160(%r10, %r11, 1)
	cmpl		$ 7, %r13d
	jl			0f // end
	//
	vmovapd		192(%r10, %r11, 1), %ymm14
	vblendpd	$ 0x7, %ymm10, %ymm14, %ymm10
	vmovapd		%ymm6, 192(%r10)
	vmaskmovpd	%ymm10, %ymm15, 192(%r10, %r11, 1)
	je			0f // end
	//
	vmovapd		%ymm7, 224(%r10)
	vmaskmovpd	%ymm11, %ymm15, 224(%r10, %r11, 1)

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_u_8x8_vs_lib4)
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
	.macro INNER_STORE_8X8_GEN_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_8x8_gen_lib4)
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
	vmovapd		%ymm8, %ymm7
	vmovapd		%ymm9, %ymm8
	vmovapd		%ymm10, %ymm9
	vmovapd		%ymm11, %ymm10
	addq		$ 32, %r11

	cmpl	$ 1, %r15d
	jle		0f

	vmovapd		%ymm1, %ymm0
	vmovapd		%ymm5, %ymm4
	vmovapd		%ymm2, %ymm1
	vmovapd		%ymm6, %ymm5
	vmovapd		%ymm7, %ymm6
	vmovapd		%ymm8, %ymm7
	vmovapd		%ymm9, %ymm8
	vmovapd		%ymm10, %ymm9
	addq		$ 32, %r11

	cmpl	$ 2, %r15d
	jle		0f

	vmovapd		%ymm1, %ymm0
	vmovapd		%ymm5, %ymm4
	vmovapd		%ymm6, %ymm5
	vmovapd		%ymm7, %ymm6
	vmovapd		%ymm8, %ymm7
	vmovapd		%ymm9, %ymm8
	addq		$ 32, %r11

0:

	// compute number of cols
	cmpl	$ 8, %eax
	jle		0f
	movl	$ 8, %eax
0:
	subl	%r15d, %eax
	movl	%eax, %r15d

	cmpl	$ 0, %r10d
	jg		0f

	// offset==0

	vmaskmovpd	%ymm0, %ymm14,  0(%r11)
	vmaskmovpd	%ymm1, %ymm14, 32(%r11)
	vmaskmovpd	%ymm2, %ymm14, 64(%r11)
	vmaskmovpd	%ymm3, %ymm14, 96(%r11)

	vmaskmovpd	%ymm4, %ymm15,  0(%r11, %r12, 1)
	vmaskmovpd	%ymm5, %ymm15, 32(%r11, %r12, 1)
	vmaskmovpd	%ymm6, %ymm15, 64(%r11, %r12, 1)
	vmaskmovpd	%ymm7, %ymm15, 96(%r11, %r12, 1)

	vmaskmovpd	%ymm8, %ymm15, 128(%r11, %r12, 1)
	cmpl		$ 6, %r15d
	jl			4f // end
	vmaskmovpd	%ymm9, %ymm15, 160(%r11, %r12, 1)
	cmpl		$ 7, %r15d
	jl			4f // end
	vmaskmovpd	%ymm10, %ymm15, 192(%r11, %r12, 1)
	je			4f // end
	vmaskmovpd	%ymm11, %ymm15, 224(%r11, %r12, 1)

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

	vperm2f128	$ 0x01, %ymm8, %ymm8, %ymm12
	vshufpd		$ 0x5, %ymm8, %ymm12, %ymm8

	vperm2f128	$ 0x01, %ymm9, %ymm9, %ymm12
	vshufpd		$ 0x5, %ymm9, %ymm12, %ymm9

	vperm2f128	$ 0x01, %ymm10, %ymm10, %ymm12
	vshufpd		$ 0x5, %ymm10, %ymm12, %ymm10

	vperm2f128	$ 0x01, %ymm11, %ymm11, %ymm12
	vshufpd		$ 0x5, %ymm11, %ymm12, %ymm11

	vperm2f128	$ 0x01, %ymm15, %ymm15, %ymm12
	vshufpd		$ 0x5, %ymm15, %ymm12, %ymm15
	vperm2f128	$ 0x01, %ymm14, %ymm14, %ymm12
	vshufpd		$ 0x5, %ymm14, %ymm12, %ymm14

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vandpd		.LC08(%rip), %ymm14, %ymm12
	vandpd		.LC05(%rip), %ymm15, %ymm13
#elif defined(OS_MAC)
	vandpd		LC08(%rip), %ymm14, %ymm12
	vandpd		LC05(%rip), %ymm15, %ymm13
#endif

	vblendpd	$ 0x1, %ymm14, %ymm15, %ymm14

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vandpd		.LC08(%rip), %ymm15, %ymm15
#elif defined(OS_MAC)
	vandpd		LC08(%rip), %ymm15, %ymm15
#endif

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

	vperm2f128	$ 0x01, %ymm8, %ymm8, %ymm8

	vperm2f128	$ 0x01, %ymm9, %ymm9, %ymm9

	vperm2f128	$ 0x01, %ymm10, %ymm10, %ymm10

	vperm2f128	$ 0x01, %ymm11, %ymm11, %ymm11

	vperm2f128	$ 0x01, %ymm14, %ymm14, %ymm14
	vperm2f128	$ 0x01, %ymm15, %ymm15, %ymm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vandpd		.LC09(%rip), %ymm14, %ymm12
	vandpd		.LC06(%rip), %ymm15, %ymm13
#elif defined(OS_MAC)
	vandpd		LC09(%rip), %ymm14, %ymm12
	vandpd		LC06(%rip), %ymm15, %ymm13
#endif

	vblendpd	$ 0x3, %ymm14, %ymm15, %ymm14

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vandpd		.LC09(%rip), %ymm15, %ymm15
#elif defined(OS_MAC)
	vandpd		LC09(%rip), %ymm15, %ymm15
#endif

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

	vperm2f128	$ 0x01, %ymm8, %ymm8, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm8, %ymm8

	vperm2f128	$ 0x01, %ymm9, %ymm9, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm9, %ymm9

	vperm2f128	$ 0x01, %ymm10, %ymm10, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm10, %ymm10

	vperm2f128	$ 0x01, %ymm11, %ymm11, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm11, %ymm11

	vperm2f128	$ 0x01, %ymm14, %ymm14, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm14, %ymm14
	vperm2f128	$ 0x01, %ymm15, %ymm15, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm15, %ymm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vandpd		.LC10(%rip), %ymm14, %ymm12
	vandpd		.LC07(%rip), %ymm15, %ymm13
#elif defined(OS_MAC)
	vandpd		LC10(%rip), %ymm14, %ymm12
	vandpd		LC07(%rip), %ymm15, %ymm13
#endif

	vblendpd	$ 0x7, %ymm14, %ymm15, %ymm14

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vandpd		.LC10(%rip), %ymm15, %ymm15
#elif defined(OS_MAC)
	vandpd		LC10(%rip), %ymm15, %ymm15
#endif

3:

	vmaskmovpd	%ymm0, %ymm12, 0(%r11)
	vmaskmovpd	%ymm4, %ymm14, 0(%r11, %r12, 1)
	vmaskmovpd	%ymm0, %ymm13, 0(%r11, %r12, 2)
	vmaskmovpd	%ymm1, %ymm12, 32(%r11)
	vmaskmovpd	%ymm5, %ymm14, 32(%r11, %r12, 1)
	vmaskmovpd	%ymm1, %ymm13, 32(%r11, %r12, 2)
	vmaskmovpd	%ymm2, %ymm12, 64(%r11)
	vmaskmovpd	%ymm6, %ymm14, 64(%r11, %r12, 1)
	vmaskmovpd	%ymm2, %ymm13, 64(%r11, %r12, 2)
	vmaskmovpd	%ymm3, %ymm12, 96(%r11)
	vmaskmovpd	%ymm7, %ymm14, 96(%r11, %r12, 1)
	vmaskmovpd	%ymm3, %ymm13, 96(%r11, %r12, 2)

	vmaskmovpd	%ymm8, %ymm15, 128(%r11, %r12, 1)
	vmaskmovpd	%ymm8, %ymm13, 128(%r11, %r12, 2)
	cmpl		$ 6, %r15d
	jl			4f // end
	vmaskmovpd	%ymm9, %ymm15, 160(%r11, %r12, 1)
	vmaskmovpd	%ymm9, %ymm13, 160(%r11, %r12, 2)
	cmpl		$ 7, %r15d
	jl			4f // end
	vmaskmovpd	%ymm10, %ymm15, 192(%r11, %r12, 1)
	vmaskmovpd	%ymm10, %ymm13, 192(%r11, %r12, 2)
	je			4f // end
	vmaskmovpd	%ymm11, %ymm15, 224(%r11, %r12, 1)
	vmaskmovpd	%ymm11, %ymm13, 224(%r11, %r12, 2)

4:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_8x8_gen_lib4)
#endif





//                               1      2              3          4        5          6        7             8          9        10         11
// void kernel_dgemm_nt_8x8l_lib4(int k, double *alpha, double *A, int sda, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_8x8l_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13 // B
	movq	ARG6, %r14 // sdb
	sall	$ 5, %r14d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12 // C
	movq	ARG9, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X8L_LIB4
#else
	CALL(inner_scale_ab_8x8l_lib4)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_8X8L_LIB4
#else
	CALL(inner_store_8x8l_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dgemm_nt_8x8l_lib4)





//                               1      2              3          4        5          6        7             8          9        10         11
// void kernel_dgemm_nt_8x8u_lib4(int k, double *alpha, double *A, int sda, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_8x8u_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG5, %r11 // B
	movq	ARG6, %r12 // sdb
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG3, %r13 // A
	movq	ARG4, %r14 // sda
	sall	$ 5, %r14d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12 // C
	movq	ARG9, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_TRAN_SCALE_AB_8X8_LIB4
#else
	CALL(inner_tran_scale_ab_8x8_lib4)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_8X8U_LIB4
#else
	CALL(inner_store_8x8u_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dgemm_nt_8x8u_lib4)





//                                   1      2              3          4        5          6        7             8          9        10         11       12      13
// void kernel_dgemm_nt_8x8l_vs_lib4(int k, double *alpha, double *A, int sda, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_8x8l_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13 // B
	movq	ARG6, %r14 // sdb
	sall	$ 5, %r14d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12 // C
	movq	ARG9, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X8L_LIB4
#else
	CALL(inner_scale_ab_8x8l_lib4)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG12, %r12 // km 
	movq	ARG13, %r13 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_8X8L_VS_LIB4
#else
	CALL(inner_store_8x8l_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dgemm_nt_8x8l_vs_lib4)





//                                   1      2              3          4        5          6        7             8          9        10         11       12      13
// void kernel_dgemm_nt_8x8u_vs_lib4(int k, double *alpha, double *A, int sda, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_8x8u_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG5, %r11 // B
	movq	ARG6, %r12 // sdb
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG3, %r13 // A
	movq	ARG4, %r14 // sdb
	sall	$ 5, %r14d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12 // C
	movq	ARG9, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_TRAN_SCALE_AB_8X8_LIB4
#else
	CALL(inner_tran_scale_ab_8x8_lib4)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG12, %r12 // km 
	movq	ARG13, %r13 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_8X8U_VS_LIB4
#else
	CALL(inner_store_8x8u_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dgemm_nt_8x8u_vs_lib4)





#if 0
//                                   1      2              3          4        5          6        7             8         9          10       11        12         13       14      15      16      17
// void kernel_dgemm_nt_8x8_gen_lib4(int k, double *alpha, double *A, int sda, double *B, int sdb, double *beta, int offC, double *C, int sdc, int offD, double *D, int sdd, int m0, int m1, int n0, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_8x8_gen_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13 // B
	movq	ARG6, %r14 // sdb
	sall	$ 5, %r14d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12 // C
	movq	ARG9, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X8L_LIB4
#else
	CALL(inner_scale_ab_8x8l_lib4)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_8X8L_LIB4
#else
	CALL(inner_store_8x8l_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dgemm_nt_8x8_gen_lib4)
#endif





//                                 1      2              3          4        5            6          7        8             9          10       11         12
// void kernel_dsyrk_nn_u_8x8_lib4(int k, double *alpha, double *A, int sda, int offsetB, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nn_u_8x8_lib4)
	
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
	INNER_EDGE_DGEMM_NN_8X8U_LIB4
#else
	CALL(inner_edge_dgemm_nn_8x8u_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_8X8U_LIB4
#else
	CALL(inner_kernel_dgemm_nn_8x8u_lib4)
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12 // C
	movq	ARG10, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X8U_LIB4
#else
	CALL(inner_scale_ab_8x8u_lib4)
#endif


	// store n

	movq	ARG11, %r10 // D
	movq	ARG12, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_U_8X8_LIB4
#else
	CALL(inner_store_u_8x8_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dsyrk_nn_u_8x8_lib4)





//                                    1      2              3          4        5            6          7        8             9          10       11         12       13      14
// void kernel_dsyrk_nn_u_8x8_vs_lib4(int k, double *alpha, double *A, int sda, int offsetB, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nn_u_8x8_vs_lib4)
	
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
	INNER_EDGE_DGEMM_NN_8X8U_LIB4
#else
	CALL(inner_edge_dgemm_nn_8x8u_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_8X8U_LIB4
#else
	CALL(inner_kernel_dgemm_nn_8x8u_lib4)
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12 // C
	movq	ARG10, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X8U_LIB4
#else
	CALL(inner_scale_ab_8x8u_lib4)
#endif


	// store n

	movq	ARG11, %r10 // D
	movq	ARG12, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG13, %r12 // m1
	movq	ARG14, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_U_8X8_VS_LIB4
#else
	CALL(inner_store_u_8x8_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dsyrk_nn_u_8x8_vs_lib4)





//                               1      2              3          4        5          6        7             8          9        10         11
// void kernel_dsyrk_nt_8x8_lib4(int k, double *alpha, double *A, int sda, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nt_l_8x8_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13 // B
	movq	ARG6, %r14 // sdb
	sall	$ 5, %r14d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12 // C
	movq	ARG9, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X8L_LIB4
#else
	CALL(inner_scale_ab_8x8l_lib4)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X8_LIB4
#else
	CALL(inner_store_l_8x8_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dsyrk_nt_l_8x8_lib4)






//                                  1      2              3          4        5          6        7             8          9        10         11       12      13
// void kernel_dsyrk_nt_8x8_vs_lib4(int k, double *alpha, double *A, int sda, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nt_l_8x8_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13 // B
	movq	ARG6, %r14 // sdb
	sall	$ 5, %r14d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12 // C
	movq	ARG9, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X8L_LIB4
#else
	CALL(inner_scale_ab_8x8l_lib4)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG12, %r12 // D
	movq	ARG13, %r13 // D

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X8_VS_LIB4
#else
	CALL(inner_store_l_8x8_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dsyrk_nt_l_8x8_vs_lib4)






//                                  1      2          3        4          5        6          7        8          9        10
// void kernel_dpotrf_nt_l_8x8_lib4(int k, double *A, int sda, double *B, int sdb, double *C, int sdc, double *D, int sdd, double *inv_diag_D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpotrf_nt_l_8x8_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // sdb
	sall	$ 5, %r14d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib4)
#endif


	// call inner blender nn

	movq	ARG6, %r10 // C
	movq	ARG7, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_8X8_LIB4
#else
	CALL(inner_scale_m11_8x8_lib4)
#endif


	// factorization

	movq	ARG10, %r10  // inv_diag_D 
	movl	$ 8, %r11d

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_8X8_VS_LIB4
#else
	CALL(inner_edge_dpotrf_8x8_vs_lib4)
#endif


	// store n

	movq	ARG8, %r10 // store address D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X8_LIB4
#else
	CALL(inner_store_l_8x8_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dpotrf_nt_l_8x8_lib4)





//                                     1      2          3        4          5        6          7        8          9        10                  11      12
// void kernel_dpotrf_nt_l_8x8_vs_lib4(int k, double *A, int sda, double *B, int sdb, double *C, int sdc, double *D, int sdd, double *inv_diag_D, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpotrf_nt_l_8x8_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // sdb
	sall	$ 5, %r14d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib4)
#endif


	// call inner blender nn

	movq	ARG6, %r10 // C
	movq	ARG7, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_8X8_LIB4
#else
	CALL(inner_scale_m11_8x8_lib4)
#endif


	// factorization

	movq	ARG10, %r10  // inv_diag_D 
	movq	ARG12, %r11 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_8X8_VS_LIB4
#else
	CALL(inner_edge_dpotrf_8x8_vs_lib4)
#endif


	// store n

	movq	ARG8, %r10 // store address D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

	movq	ARG11, %r12 // km 
	movq	ARG12, %r13 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X8_VS_LIB4
#else
	CALL(inner_store_l_8x8_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dpotrf_nt_l_8x8_vs_lib4)





//                                        1       2           3         4           5         6       7           8         9           10        11         12       13         14       15
// void kernel_dsyrk_dpotrf_nt_l_8x8_lib4(int kp, double *Ap, int sdap, double *Bp, int sdbp, int km, double *Am, int sdam, double *Bm, int sdbm, double *C, int sdc, double *D, int sdd, double *inv_diag_D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_dpotrf_nt_l_8x8_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG2, %r11 // Ap
	movq	ARG3, %r12 // sdap
	sall	$ 5, %r12d // 4*sdap*sizeof(double)
	movq	ARG4, %r13 // Bp
	movq	ARG5, %r14 // sdbp
	sall	$ 5, %r14d // 4*sdbp*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib4)
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
	vxorpd		%ymm15, %ymm8, %ymm8
	vxorpd		%ymm15, %ymm9, %ymm9
	vxorpd		%ymm15, %ymm10, %ymm10
	vxorpd		%ymm15, %ymm11, %ymm11


	movq	ARG6, %r10 // km
	movq	ARG7, %r11 // Am
	movq	ARG8, %r12 // sdam
	sall	$ 5, %r12d // 4*sdam*sizeof(double)
	movq	ARG9, %r13 // Bm
	movq	ARG10, %r14 // sdbm
	sall	$ 5, %r14d // 4*sdbm*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib4)
#endif


	// call inner blender nn

	movq	ARG11, %r10 // C
	movq	ARG12, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_8X8_LIB4
#else
	CALL(inner_scale_m11_8x8_lib4)
#endif


	// factorization

	movq	ARG15, %r10  // inv_diag_D 
	movl	$ 8, %r11d

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_8X8_VS_LIB4
#else
	CALL(inner_edge_dpotrf_8x8_vs_lib4)
#endif


	// store n

	movq	ARG13, %r10 // store address D
	movq	ARG14, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X8_LIB4
#else
	CALL(inner_store_l_8x8_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dsyrk_dpotrf_nt_l_8x8_lib4)





//                                           1       2           3         4           5         6       7           8         9           10        11         12       13         14       15                  16      17
// void kernel_dsyrk_dpotrf_nt_l_8x8_vs_lib4(int kp, double *Ap, int sdap, double *Bp, int sdbp, int km, double *Am, int sdam, double *Bm, int sdbm, double *C, int sdc, double *D, int sdd, double *inv_diag_D, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_dpotrf_nt_l_8x8_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG2, %r11 // Ap
	movq	ARG3, %r12 // sdap
	sall	$ 5, %r12d // 4*sdap*sizeof(double)
	movq	ARG4, %r13 // Bp
	movq	ARG5, %r14 // sdbp
	sall	$ 5, %r14d // 4*sdbp*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib4)
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
	vxorpd		%ymm15, %ymm8, %ymm8
	vxorpd		%ymm15, %ymm9, %ymm9
	vxorpd		%ymm15, %ymm10, %ymm10
	vxorpd		%ymm15, %ymm11, %ymm11


	// call inner dgemm kernel nt sub

	movq	ARG6, %r10 // km
	movq	ARG7, %r11 // Am
	movq	ARG8, %r12 // sdam
	sall	$ 5, %r12d // 4*sdam*sizeof(double)
	movq	ARG9, %r13 // Bm
	movq	ARG10, %r14 // sdbm
	sall	$ 5, %r14d // 4*sdbm*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib4)
#endif


	// call inner blender nn

	movq	ARG11, %r10 // C
	movq	ARG12, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_8X8_LIB4
#else
	CALL(inner_scale_m11_8x8_lib4)
#endif


	// factorization

	movq	ARG15, %r10  // inv_diag_D 
	movq	ARG17, %r11 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_8X8_VS_LIB4
#else
	CALL(inner_edge_dpotrf_8x8_vs_lib4)
#endif


	// store n

	movq	ARG13, %r10 // store address D
	movq	ARG14, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

	movq	ARG16, %r12 // km 
	movq	ARG17, %r13 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X8_VS_LIB4
#else
	CALL(inner_store_l_8x8_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dsyrk_dpotrf_nt_l_8x8_vs_lib4)





//                                       1      2          3        4          5        6          7        8          9        10         11       12
// void kernel_dtrsm_nt_rl_inv_8x8l_lib4(int k, double *A, int sda, double *B, int sdb, double *C, int sdc, double *D, int sdd, double *E, int sde, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_inv_8x8l_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13
	movq	ARG5, %r14
	sall	$ 5, %r14d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib4)
#endif


	// call inner blender nn

	movq	ARG6, %r10
	movq	ARG7, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_8X8_LIB4
#else
	CALL(inner_scale_m11_8x8_lib4)
#endif


	// solve

	movq	ARG10, %r10  // E 
	movq	ARG11, %r11  // sde 
	sall	$ 5, %r11d // 4*sde*sizeof(double)
	movq	ARG12, %r12  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_8X8L_LIB4
#else
	CALL(inner_edge_dtrsm_rlt_inv_8x8l_lib4)
#endif


	// store n

	movq	ARG8, %r10 // store address D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_8X8L_LIB4
#else
	CALL(inner_store_8x8l_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_8x8l_lib4)





//                                       1      2          3        4          5        6          7        8          9        10         11       12
// void kernel_dtrsm_nt_rl_inv_8x8u_lib4(int k, double *A, int sda, double *B, int sdb, double *C, int sdc, double *D, int sdd, double *E, int sde, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_inv_8x8u_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10
	movq	ARG4, %r11
	movq	ARG5, %r12
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG2, %r13
	movq	ARG3, %r14
	sall	$ 5, %r14d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib4)
#endif


	// call inner blender nn

	movq	ARG6, %r10
	movq	ARG7, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_TRAN_SCALE_M11_8X8_LIB4
#else
	CALL(inner_tran_scale_m11_8x8_lib4)
#endif


	// solve

	movq	ARG10, %r10  // E 
	movq	ARG11, %r11  // sde 
	sall	$ 5, %r11d // 4*sde*sizeof(double)
	movq	ARG12, %r12  // inv_diag_E 
	movq	ARG8, %r13 // D
	movq	ARG9, %r14 // sdd

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_8X8U_LIB4
#else
	CALL(inner_edge_dtrsm_rlt_inv_8x8u_lib4)
#endif


	// store n

	movq	ARG8, %r10 // store address D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_8X8U_LIB4
#else
	CALL(inner_store_8x8u_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_8x8u_lib4)





//                                          1      2          3        4          5        6          7        8          9        10         11       12                  13      14
// void kernel_dtrsm_nt_rl_inv_8x8l_vs_lib4(int k, double *A, int sda, double *B, int sdb, double *C, int sdc, double *D, int sdd, double *E, int sde, double *inv_diag_E, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_inv_8x8l_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13
	movq	ARG5, %r14
	sall	$ 5, %r14d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib4)
#endif


	// call inner blender nn

	movq	ARG6, %r10
	movq	ARG7, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_8X8_LIB4
#else
	CALL(inner_scale_m11_8x8_lib4)
#endif


	// solve

	movq	ARG10, %r10  // E 
	movq	ARG11, %r11  // sde 
	sall	$ 5, %r11d // 4*sde*sizeof(double)
	movq	ARG12, %r12  // inv_diag_E 
	movq	ARG14, %r13 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_8X8L_VS_LIB4
#else
	CALL(inner_edge_dtrsm_rlt_inv_8x8l_vs_lib4)
#endif


	// store n

	movq	ARG8, %r10 // store address D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

	movq	ARG13, %r12 // km 
	movq	ARG14, %r13 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_8X8L_VS_LIB4
#else
	CALL(inner_store_8x8l_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_8x8l_vs_lib4)





//                                          1      2          3        4          5        6          7        8          9        10         11       12                  13      14
// void kernel_dtrsm_nt_rl_inv_8x8u_vs_lib4(int k, double *A, int sda, double *B, int sdb, double *C, int sdc, double *D, int sdd, double *E, int sde, double *inv_diag_E, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_inv_8x8u_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10
	movq	ARG4, %r11
	movq	ARG5, %r12
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG2, %r13
	movq	ARG3, %r14
	sall	$ 5, %r14d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib4)
#endif


	// call inner blender nn

	movq	ARG6, %r10 // C
	movq	ARG7, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_TRAN_SCALE_M11_8X8_LIB4
#else
	CALL(inner_tran_scale_m11_8x8_lib4)
#endif


	// solve

	movq	ARG10, %r10  // E 
	movq	ARG11, %r11  // sde 
	sall	$ 5, %r11d // 4*sde*sizeof(double)
	movq	ARG12, %r12  // inv_diag_E 
	movq	ARG8, %r13 // D
	movq	ARG9, %r14 // sdd
	sall	$ 5, %r14d // 4*sdc*sizeof(double)
	movq	ARG14, %r15 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_8X8U_VS_LIB4
#else
	CALL(inner_edge_dtrsm_rlt_inv_8x8u_vs_lib4)
#endif


	// store n

	movq	ARG8, %r10 // store address D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

	movq	ARG13, %r12 // km 
	movq	ARG14, %r13 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_8X8U_VS_LIB4
#else
	CALL(inner_store_8x8u_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_8x8u_vs_lib4)





//                                                1       2           3         4           5         6       7           8          9          10        11         12       13         14       15         16       17                  18      19
// void kernel_dgemm_dtrsm_nt_rl_inv_8x8l_vs_lib4(int kp, double *Ap, int sdap, double *Bp, int sdbp, int km, double *Am, int sdam, double *Bm, int sdbm, double *C, int sdc, double *D, int sdd, double *E, int sde, double *inv_diag_E, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_dtrsm_nt_rl_inv_8x8l_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13
	movq	ARG5, %r14
	sall	$ 5, %r14d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib4)
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
	vxorpd		%ymm15, %ymm8, %ymm8
	vxorpd		%ymm15, %ymm9, %ymm9
	vxorpd		%ymm15, %ymm10, %ymm10
	vxorpd		%ymm15, %ymm11, %ymm11


	movq	ARG6, %r10
	movq	ARG7, %r11
	movq	ARG8, %r12
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG9, %r13
	movq	ARG10, %r14
	sall	$ 5, %r14d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib4)
#endif


	// call inner blender nn

	movq	ARG11, %r10
	movq	ARG12, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_8X8_LIB4
#else
	CALL(inner_scale_m11_8x8_lib4)
#endif


	// solve

	movq	ARG15, %r10  // E 
	movq	ARG16, %r11  // sde 
	sall	$ 5, %r11d // 4*sde*sizeof(double)
	movq	ARG17, %r12  // inv_diag_E 
	movq	ARG19, %r13 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_8X8L_VS_LIB4
#else
	CALL(inner_edge_dtrsm_rlt_inv_8x8l_vs_lib4)
#endif


	// store n

	movq	ARG13, %r10 // store address D
	movq	ARG14, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

	movq	ARG18, %r12 // km 
	movq	ARG19, %r13 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_8X8L_VS_LIB4
#else
	CALL(inner_store_8x8l_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dgemm_dtrsm_nt_rl_inv_8x8l_vs_lib4)





//                                                1       2           3         4           5         6       7           8          9          10        11         12       13         14       15         16       17                  18      19
// void kernel_dgemm_dtrsm_nt_rl_inv_8x8u_vs_lib4(int kp, double *Ap, int sdap, double *Bp, int sdbp, int km, double *Am, int sdam, double *Bm, int sdbm, double *C, int sdc, double *D, int sdd, double *E, int sde, double *inv_diag_E, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_dtrsm_nt_rl_inv_8x8u_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10
	movq	ARG4, %r11
	movq	ARG5, %r12
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG2, %r13
	movq	ARG3, %r14
	sall	$ 5, %r14d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib4)
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
	vxorpd		%ymm15, %ymm8, %ymm8
	vxorpd		%ymm15, %ymm9, %ymm9
	vxorpd		%ymm15, %ymm10, %ymm10
	vxorpd		%ymm15, %ymm11, %ymm11


	movq	ARG6, %r10
	movq	ARG9, %r11
	movq	ARG10, %r12
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG7, %r13
	movq	ARG8, %r14
	sall	$ 5, %r14d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB4
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib4)
#endif


	// call inner blender nn

	movq	ARG11, %r10 // C
	movq	ARG12, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_TRAN_SCALE_M11_8X8_LIB4
#else
	CALL(inner_tran_scale_m11_8x8_lib4)
#endif


	// solve

	movq	ARG15, %r10  // E 
	movq	ARG16, %r11  // sde 
	sall	$ 5, %r11d // 4*sde*sizeof(double)
	movq	ARG17, %r12  // inv_diag_E 
	movq	ARG13, %r13 // D
	movq	ARG14, %r14 // sdd
	sall	$ 5, %r14d // 4*sdc*sizeof(double)
	movq	ARG19, %r15 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_8X8U_VS_LIB4
#else
	CALL(inner_edge_dtrsm_rlt_inv_8x8u_vs_lib4)
#endif


	// store n

	movq	ARG13, %r10 // store address D
	movq	ARG14, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

	movq	ARG18, %r12 // km 
	movq	ARG19, %r13 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_8X8U_VS_LIB4
#else
	CALL(inner_store_8x8u_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dgemm_dtrsm_nt_rl_inv_8x8u_vs_lib4)





//#if defined(BLAS_API)
#if ( defined(BLAS_API) | ( defined(LA_HIGH_PERFORMANCE) & defined(MF_COLMAJ) ) )

#include "kernel_dgemm_8x8_lib.S"

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
	.double 0.5
	.double 1.5
	.double 2.5
	.double 3.5

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC03: // { 7.5 6.5 5.5 4.5 }
#elif defined(OS_MAC)
LC03: // { 7.5 6.5 5.5 4.5 }
	.align 5
#endif
	.double 4.5
	.double 5.5
	.double 6.5
	.double 7.5

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
.LC11: // { -1.0 1.0 1.0 1.0 }
#elif defined(OS_MAC)
	.align 5
LC11: // { -1.0 1.0 1.0 1.0 }
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

