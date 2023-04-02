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
	vmovapd	%ymm0, %ymm7; \
	vmovapd	%ymm0, %ymm8; \
	vmovapd	%ymm0, %ymm9; \
	vmovapd	%ymm0, %ymm10; \
	vmovapd	%ymm0, %ymm11
#define NEG_ACC \
	vmovapd		.LC13(%rip), %ymm15; \
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
	vmovapd		LC13(%rip), %ymm15; \
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
	vmovapd		.LC13(%rip), %ymm15; \
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
// r10d  <- 0
// r11   <- A+4*k*sizeof(double)
// r12   <- 4*sda*sizeof(double)
// r13   <- B+4*k*sizeof(double)
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
// rbx   <- dirty
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
// rbx   <- dirty
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
	.macro INNER_KERNEL_DGEMM_ADD_NN_12X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_add_nn_12x4_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	// preload
	vmovapd 0(%r11), %ymm12 // A0[0]
	vmovapd 0(%r11, %r12, 1), %ymm13 // A1[0]

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	prefetcht0	0(%r13, %r14, 2) // software prefetch
	prefetcht0	64(%r13, %r14, 2) // software prefetch

	// unroll 0
	vbroadcastsd	0(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vmulpd			0(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm8, %ymm8

	vbroadcastsd	32(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vmulpd			0(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm9, %ymm9
	subl	$ 4, %r10d

	vbroadcastsd	64(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6
	vmulpd			0(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm10, %ymm10

	vbroadcastsd	96(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vmovapd			32(%r11), %ymm12 // A0
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm13, %ymm14, %ymm15
	vmovapd			32(%r11, %r12, 1), %ymm13 // A1
	vaddpd			%ymm15, %ymm7, %ymm7
	vmulpd			0(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm11, %ymm11

	// unroll 1
	vbroadcastsd	8(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vmulpd			32(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm8, %ymm8

	vbroadcastsd	40(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vmulpd			32(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm9, %ymm9

	vbroadcastsd	72(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6
	vmulpd			32(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm10, %ymm10

	vbroadcastsd	104(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vmovapd			64(%r11), %ymm12 // A0
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm13, %ymm14, %ymm15
	vmovapd			64(%r11, %r12, 1), %ymm13 // A1
	vaddpd			%ymm15, %ymm7, %ymm7
	vmulpd			32(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm11, %ymm11

	// unroll 2
	vbroadcastsd	16(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vmulpd			64(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm8, %ymm8

	vbroadcastsd	48(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vmulpd			64(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm9, %ymm9

	vbroadcastsd	80(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6
	vmulpd			64(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm10, %ymm10

	vbroadcastsd	112(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vmovapd			96(%r11), %ymm12 // A0
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm13, %ymm14, %ymm15
	vmovapd			96(%r11, %r12, 1), %ymm13 // A1
	vaddpd			%ymm15, %ymm7, %ymm7
	vmulpd			64(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm11, %ymm11
	addq	$ 128, %r11

	// unroll 3
	vbroadcastsd	24(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vmulpd			-32(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm8, %ymm8

	vbroadcastsd	56(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vmulpd			-32(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm9, %ymm9

	vbroadcastsd	88(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6
	vmulpd			-32(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm10, %ymm10

	vbroadcastsd	120(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vmovapd			0(%r11), %ymm12 // A0
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm13, %ymm14, %ymm15
	vmovapd			0(%r11, %r12, 1), %ymm13 // A1
	vaddpd			%ymm15, %ymm7, %ymm7
	vmulpd			-32(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm11, %ymm11
	addq	%r14, %r13

	cmpl	$ 4, %r10d
	jg		1b // main loop 

0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vmulpd			0(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm8, %ymm8

	vbroadcastsd	32(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vmulpd			0(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm9, %ymm9
	subl	$ 4, %r10d

	vbroadcastsd	64(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6
	vmulpd			0(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm10, %ymm10

	vbroadcastsd	96(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vmovapd			32(%r11), %ymm12 // A0
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm13, %ymm14, %ymm15
	vmovapd			32(%r11, %r12, 1), %ymm13 // A1
	vaddpd			%ymm15, %ymm7, %ymm7
	vmulpd			0(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm11, %ymm11

	// unroll 1
	vbroadcastsd	8(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vmulpd			32(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm8, %ymm8

	vbroadcastsd	40(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vmulpd			32(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm9, %ymm9

	vbroadcastsd	72(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6
	vmulpd			32(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm10, %ymm10

	vbroadcastsd	104(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vmovapd			64(%r11), %ymm12 // A0
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm13, %ymm14, %ymm15
	vmovapd			64(%r11, %r12, 1), %ymm13 // A1
	vaddpd			%ymm15, %ymm7, %ymm7
	vmulpd			32(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm11, %ymm11

	// unroll 2
	vbroadcastsd	16(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vmulpd			64(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm8, %ymm8

	vbroadcastsd	48(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vmulpd			64(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm9, %ymm9

	vbroadcastsd	80(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6
	vmulpd			64(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm10, %ymm10

	vbroadcastsd	112(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vmovapd			96(%r11), %ymm12 // A0
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm13, %ymm14, %ymm15
	vmovapd			96(%r11, %r12, 1), %ymm13 // A1
	vaddpd			%ymm15, %ymm7, %ymm7
	vmulpd			64(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm11, %ymm11
	addq	$ 128, %r11

	// unroll 3
	vbroadcastsd	24(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vmulpd			-32(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm8, %ymm8

	vbroadcastsd	56(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vmulpd			-32(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm9, %ymm9

	vbroadcastsd	88(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6
	vmulpd			-32(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm10, %ymm10

	vbroadcastsd	120(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
//	vmovapd			0(%r11), %ymm12 // A0
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm13, %ymm14, %ymm15
//	vmovapd			0(%r11, %r12, 1), %ymm13 // A1
	vaddpd			%ymm15, %ymm7, %ymm7
	vmulpd			-32(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm11, %ymm11
	addq	%r14, %r13


	jmp		2f


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	vmovapd			0(%r11), %ymm12 // A0[0]
	vmovapd 		0(%r11, %r12, 1), %ymm13 // A1[0]

	vbroadcastsd	0(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vmulpd			0(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm8, %ymm8

	vbroadcastsd	32(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vmulpd			0(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm9, %ymm9
	subl	$ 1, %r10d

	vbroadcastsd	64(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6
	vmulpd			0(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm10, %ymm10

	vbroadcastsd	96(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7
	vmulpd			0(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm11, %ymm11

	addq	$ 32, %r11
	addq	$ 8, %r13

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 

2: // return


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_add_nn_12x4_lib4)
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
// r10   <- k-(4-offB)
// r11   <- A+(4-offB)*bs*sizeof(double)
// r12   <- bs*sda*sizeof(double)
// r13   <- B-offB+bs*sdb*sizeof(double)
// r14   <- bs*sdb*sizeof(double)
// r15   <- offB


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DGEMM_ADD_NN_12X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dgemm_add_nn_12x4_lib4)
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
	vmovapd			0(%r11), %ymm12 // A0[0]
	vmovapd 		0(%r11, %r12, 1), %ymm13 // A1[0]

	vbroadcastsd	0(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vmulpd			0(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm8, %ymm8

	vbroadcastsd	32(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vmulpd			0(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm9, %ymm9

	vbroadcastsd	64(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6
	vmulpd			0(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm10, %ymm10

	vbroadcastsd	96(%r13), %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm13, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7
	vmulpd			0(%r11, %r12, 2), %ymm14, %ymm15
	vaddpd			%ymm15, %ymm11, %ymm11

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

	FUN_END(inner_edge_dgemm_add_nn_12x4_lib4)
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

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_12X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_12x4_lib4)
#endif
		
	vbroadcastsd 0(%r10), %ymm15 // beta

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

	movq	%r12, %r15 // C1 <- C0
	addq	%r13, %r15 // C1 <- C0 + 4*sdc*sizeof(double)

	movq	%r15, %r14 // C2 <- C1
	addq	%r13, %r14 // C2 <- C1 + 4*sdc*sizeof(double)

	vbroadcastsd 0(%r11), %ymm14 // beta

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

	vmovapd		0(%r15), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm4, %ymm15, %ymm4
	vmovapd		32(%r15), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm5, %ymm15, %ymm5
	vmovapd		64(%r15), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm6, %ymm15, %ymm6
	vmovapd		96(%r15), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm7, %ymm15, %ymm7

	vmovapd		0(%r14), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm8, %ymm15, %ymm8
	vmovapd		32(%r14), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm9, %ymm15, %ymm9
	vmovapd		64(%r14), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm10, %ymm15, %ymm10
	vmovapd		96(%r14), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm11, %ymm15, %ymm11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_12x4_lib4)
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
	.macro INNER_STORE_12X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_12x4_lib4)
#endif
	
	vmovapd %ymm0,  0(%r10)
	vmovapd %ymm1, 32(%r10)
	vmovapd %ymm2, 64(%r10)
	vmovapd %ymm3, 96(%r10)

	vmovapd %ymm4,  0(%r10, %r11, 1)
	vmovapd %ymm5, 32(%r10, %r11, 1)
	vmovapd %ymm6, 64(%r10, %r11, 1)
	vmovapd %ymm7, 96(%r10, %r11, 1)

	vmovapd %ymm8,   0(%r10, %r11, 2)
	vmovapd %ymm9,  32(%r10, %r11, 2)
	vmovapd %ymm10, 64(%r10, %r11, 2)
	vmovapd %ymm11, 96(%r10, %r11, 2)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_12x4_lib4)
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
// r14  <- dirty
// r15  <- dirty
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d50 d60 d70]
// ymm5 <- [d41 d50 d61 d71]
// ymm6 <- [d42 d52 d62 d72]
// ymm7 <- [d43 d53 d63 d73]
// ymm8  <- [d80 d90 da0 db0]
// ymm9  <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]
//
// output arguments:
// r10  <- D
// r11  <- 4*sdd*sizeof(double)
// r12d  <- km
// r13d  <- kn
// r14  <- dirty
// r15  <- dirty
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm4 <- [d40 d50 d60 d70]
// ymm5 <- [d41 d51 d61 d71]
// ymm6 <- [d42 d52 d62 d72]
// ymm7 <- [d43 d53 d63 d73]
// ymm8  <- [d80 d90 da0 db0]
// ymm9  <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]

#if MACRO_LEVEL>=1
	.macro INNER_STORE_12X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_12x4_vs_lib4)
#endif
	
	vcvtsi2sd	%r12d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC04(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC04(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm15

	vmovapd		%ymm0, 0(%r10)
	vmovapd		%ymm4, 0(%r10, %r11, 1)
	vmaskmovpd	%ymm8, %ymm15,  0(%r10, %r11, 2)
	cmpl		$ 2, %r13d
	jl			0f // end
	vmovapd		%ymm1, 32(%r10)
	vmovapd		%ymm5, 32(%r10, %r11, 1)
	vmaskmovpd	%ymm9, %ymm15, 32(%r10, %r11, 2)
	cmpl		$ 3, %r13d
	jl			0f // end
	vmovapd		%ymm2, 64(%r10)
	vmovapd		%ymm6, 64(%r10, %r11, 1)
	vmaskmovpd	%ymm10, %ymm15, 64(%r10, %r11, 2)
	je			0f // end
	vmovapd		%ymm3, 96(%r10)
	vmovapd		%ymm7, 96(%r10, %r11, 1)
	vmaskmovpd	%ymm11, %ymm15, 96(%r10, %r11, 2)

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_12x4_vs_lib4)
#endif





//                                1      2              3          4        5            6          7        8             9          10       11         12
// void kernel_dgemm_nn_12x4_lib4(int k, double *alpha, double *A, int sda, int offsetB, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nn_12x4_lib4)
	
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
	INNER_EDGE_DGEMM_ADD_NN_12X4_LIB4
#else
	CALL(inner_edge_dgemm_add_nn_12x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_ADD_NN_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_add_nn_12x4_lib4)
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12 // C
	movq	ARG10, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif


	// store n

	movq	ARG11, %r10 // D
	movq	ARG12, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dgemm_nn_12x4_lib4)





//                                   1      2              3          4        5            6          7        8             9          10       11         12       13      14
// void kernel_dgemm_nn_12x4_vs_lib4(int k, double *alpha, double *A, int sda, int offsetB, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nn_12x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG6, %r13 // B
	movq	ARG7, %r14 // sda
	sall	$ 5, %r14d // 4*sda*sizeof(double)
	movq	ARG5, %r15 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_DGEMM_ADD_NN_12X4_LIB4
#else
	CALL(inner_edge_dgemm_add_nn_12x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_ADD_NN_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_add_nn_12x4_lib4)
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12 // C
	movq	ARG10, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif


	// store n

	movq	ARG11, %r10 // store address D
	movq	ARG12, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

	movq	ARG13, %r12 // km 
	movq	ARG14, %r13 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB4
#else
	CALL(inner_store_12x4_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dgemm_nn_12x4_vs_lib4)





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
.LC04: // { 11.5 10.5 9.5 8.5 }
#elif defined(OS_MAC)
LC04: // { 11.5 10.5 9.5 8.5 }
	.align 5
#endif
	.long	0
	.long	1075904512
	.long	0
	.long	1076035584
	.long	0
	.long	1076166656
 	.long	0
	.long	1076297728

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC05: // { 1.0 1.0 1.0 1.0 }
#elif defined(OS_MAC)
LC05: // { 1.0 1.0 1.0 1.0 }
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



#if defined(OS_LINUX)
	.section	.note.GNU-stack,"",@progbits
#elif defined(OS_MAC)
	.subsections_via_symbols
#endif

