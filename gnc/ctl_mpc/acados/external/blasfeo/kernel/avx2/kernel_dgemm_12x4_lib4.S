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



#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_12x4_lib4)
#endif
	
// broadcast scheme
#if 1

	cmpl	$ 0, %r10d
	jle		2f // return

	// prefetch
	prefetcht0	0(%r13) // software prefetch
	prefetcht0	0+64(%r13) // software prefetch

	// preload
	vmovapd 0(%r11), %ymm13 // A0[0]
	vmovapd 0(%r11, %r12), %ymm14 // A1[0]
	vmovapd 0(%r11, %r12, 2), %ymm15 // A1[0]

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	prefetcht0	128(%r13) // software prefetch
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	prefetcht0	128+64(%r13) // software prefetch
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	8(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	subl	$ 4, %r10d

	vbroadcastsd	16(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	24(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			32(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			32(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			32(%r11, %r12, 2), %ymm15 // A1

	// unroll 1
	vbroadcastsd	32(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	40(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	48(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	56(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			64(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			64(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			64(%r11, %r12, 2), %ymm15 // A1

	// unroll 2
	vbroadcastsd	64(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	72(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	80(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	88(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			96(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			96(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			96(%r11, %r12, 2), %ymm15 // A1

	// unroll 3
	vbroadcastsd	96(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	addq	$ 128, %r11

	vbroadcastsd	104(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	112(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	120(%r13), %ymm12
	addq	$ 128, %r13
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			0(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			0(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			0(%r11, %r12, 2), %ymm15 // A1


	cmpl	$ 4, %r10d
	jg		1b // main loop 

0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	8(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	subl	$ 4, %r10d

	vbroadcastsd	16(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	24(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			32(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			32(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			32(%r11, %r12, 2), %ymm15 // A1

	// unroll 1
	vbroadcastsd	32(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	40(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	48(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	56(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			64(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			64(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			64(%r11, %r12, 2), %ymm15 // A1

	// unroll 2
	vbroadcastsd	64(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	72(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	80(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	88(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			96(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			96(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			96(%r11, %r12, 2), %ymm15 // A1

	// unroll 3
	vbroadcastsd	96(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	addq	$ 128, %r11

	vbroadcastsd	104(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	112(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	120(%r13), %ymm12
	addq	$ 128, %r13
	vfmadd231pd		%ymm13, %ymm12, %ymm3
//	vmovapd			0(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
//	vmovapd			0(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
//	vmovapd			0(%r11, %r12, 2), %ymm15 // A1



	jmp		2f


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	vmovapd			0(%r11), %ymm13 // A0[0]
	vmovapd 		0(%r11, %r12), %ymm14 // A1[0]
	vmovapd 		0(%r11, %r12, 2), %ymm15 // A2[0]
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	8(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	addq	$ 32, %r11

	vbroadcastsd	16(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	subl	$ 1, %r10d

	vbroadcastsd	24(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

// shuffle scheme
#else

	cmpl	$ 0, %r10d
	jle		2f // return

	// prefetch
	vmovapd 0(%r11), %ymm12 // A0[0]
	vmovapd 0(%r13), %ymm15 // B[0]
	vmovapd 0(%r11, %r12), %ymm13 // A1[0]
	vmovapd 0(%r11, %r12, 2), %ymm14 // A2[0]

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop
	
	// unroll 0
	vfmadd231pd	%ymm12, %ymm15, %ymm0
	subl		$ 4, %r10d
	vfmadd231pd	%ymm13, %ymm15, %ymm4
	vfmadd231pd	%ymm14, %ymm15, %ymm8

	vshufpd		$ 0x5, %ymm15, %ymm15, %ymm15
	vfmadd231pd	%ymm12, %ymm15, %ymm1
	vfmadd231pd	%ymm13, %ymm15, %ymm5
	vfmadd231pd	%ymm14, %ymm15, %ymm9

	vperm2f128	$ 0x1, %ymm15, %ymm15, %ymm15
	vfmadd231pd	%ymm12, %ymm15, %ymm3
	vfmadd231pd	%ymm13, %ymm15, %ymm7
	vfmadd231pd	%ymm14, %ymm15, %ymm11

	vshufpd		$ 0x5, %ymm15, %ymm15, %ymm15
	vfmadd231pd	%ymm12, %ymm15, %ymm2
	vmovapd		32(%r11), %ymm12 // A0[4]
	vfmadd231pd	%ymm13, %ymm15, %ymm6
	vmovapd		32(%r11, %r12), %ymm13 // A1[4]
	vfmadd231pd	%ymm14, %ymm15, %ymm10
	vmovapd		32(%r13), %ymm15 // B[4]
	vmovapd		32(%r11, %r12, 2), %ymm14 // A2[4]

	// unroll 1
	vfmadd231pd	%ymm12, %ymm15, %ymm0
	vfmadd231pd	%ymm13, %ymm15, %ymm4
	vfmadd231pd	%ymm14, %ymm15, %ymm8

	vshufpd 	$ 0x5, %ymm15, %ymm15, %ymm15
	vfmadd231pd	%ymm12, %ymm15, %ymm1
	vfmadd231pd	%ymm13, %ymm15, %ymm5
	vfmadd231pd	%ymm14, %ymm15, %ymm9

	vperm2f128 	$ 0x1, %ymm15, %ymm15, %ymm15
	vfmadd231pd	%ymm12, %ymm15, %ymm3
	vfmadd231pd	%ymm13, %ymm15, %ymm7
	vfmadd231pd	%ymm14, %ymm15, %ymm11

	vshufpd 	$ 0x5, %ymm15, %ymm15, %ymm15
	vfmadd231pd	%ymm12, %ymm15, %ymm2
	vmovapd 	64(%r11), %ymm12 // A0[8]
	vfmadd231pd	%ymm13, %ymm15, %ymm6
	vmovapd 	64(%r11, %r12), %ymm13 // A1[8]
	vfmadd231pd	%ymm14, %ymm15, %ymm10
	vmovapd 	64(%r13), %ymm15 // B[8]
	vmovapd 	64(%r11, %r12, 2), %ymm14 // A2[8]


	// unroll 2
	vfmadd231pd	%ymm12, %ymm15, %ymm0
	vfmadd231pd	%ymm13, %ymm15, %ymm4
	vfmadd231pd	%ymm14, %ymm15, %ymm8

	vshufpd 	$ 0x5, %ymm15, %ymm15, %ymm15
	vfmadd231pd	%ymm12, %ymm15, %ymm1
	vfmadd231pd	%ymm13, %ymm15, %ymm5
	vfmadd231pd	%ymm14, %ymm15, %ymm9

	vperm2f128 	$ 0x1, %ymm15, %ymm15, %ymm15
	vfmadd231pd	%ymm12, %ymm15, %ymm3
	vfmadd231pd	%ymm13, %ymm15, %ymm7
	vfmadd231pd	%ymm14, %ymm15, %ymm11

	vshufpd 	$ 0x5, %ymm15, %ymm15, %ymm15
	vfmadd231pd	%ymm12, %ymm15, %ymm2
	vmovapd 	96(%r11), %ymm12 // A0[12]
	vfmadd231pd	%ymm13, %ymm15, %ymm6
	vmovapd 	96(%r11, %r12), %ymm13 // A1[12]
	vfmadd231pd	%ymm14, %ymm15, %ymm10
	vmovapd 	96(%r13), %ymm15 // B[12]
	vmovapd 	96(%r11, %r12, 2), %ymm14 // A2[12]


	// unroll 3
	vfmadd231pd	%ymm12, %ymm15, %ymm0
	addq		$ 128, %r11
	vfmadd231pd	%ymm13, %ymm15, %ymm4
	vfmadd231pd	%ymm14, %ymm15, %ymm8

	vshufpd 	$ 0x5, %ymm15, %ymm15, %ymm15
	vfmadd231pd	%ymm12, %ymm15, %ymm1
	addq		$ 128, %r13
	vfmadd231pd	%ymm13, %ymm15, %ymm5
	vfmadd231pd	%ymm14, %ymm15, %ymm9

	vperm2f128 $ 0x1, %ymm15, %ymm15, %ymm15
	vfmadd231pd	%ymm12, %ymm15, %ymm3
	vfmadd231pd	%ymm13, %ymm15, %ymm7
	vfmadd231pd	%ymm14, %ymm15, %ymm11

	vshufpd 	$ 0x5, %ymm15, %ymm15, %ymm15
	vfmadd231pd	%ymm12, %ymm15, %ymm2
	vmovapd 	0(%r11), %ymm12 // A0[0]
	vfmadd231pd	%ymm13, %ymm15, %ymm6
	vmovapd 	0(%r11, %r12), %ymm13 // A1[0]
	vfmadd231pd	%ymm14, %ymm15, %ymm10
	vmovapd 	0(%r13), %ymm15 // B[0]
	vmovapd 	0(%r11, %r12, 2), %ymm14 // A2[0]


	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1


	// unroll 0
	vfmadd231pd	%ymm12, %ymm15, %ymm0
	subl		$ 4, %r10d
	vfmadd231pd	%ymm13, %ymm15, %ymm4
	vfmadd231pd	%ymm14, %ymm15, %ymm8

	vshufpd		$ 0x5, %ymm15, %ymm15, %ymm15
	vfmadd231pd	%ymm12, %ymm15, %ymm1
	vfmadd231pd	%ymm13, %ymm15, %ymm5
	vfmadd231pd	%ymm14, %ymm15, %ymm9

	vperm2f128	$ 0x1, %ymm15, %ymm15, %ymm15
	vfmadd231pd	%ymm12, %ymm15, %ymm3
	vfmadd231pd	%ymm13, %ymm15, %ymm7
	vfmadd231pd	%ymm14, %ymm15, %ymm11

	vshufpd		$ 0x5, %ymm15, %ymm15, %ymm15
	vfmadd231pd	%ymm12, %ymm15, %ymm2
	vmovapd		32(%r11), %ymm12 // A0[4]
	vfmadd231pd	%ymm13, %ymm15, %ymm6
	vmovapd		32(%r11, %r12), %ymm13 // A1[4]
	vfmadd231pd	%ymm14, %ymm15, %ymm10
	vmovapd		32(%r13), %ymm15 // B[4]
	vmovapd		32(%r11, %r12, 2), %ymm14 // A2[4]

	// unroll 1
	vfmadd231pd	%ymm12, %ymm15, %ymm0
	vfmadd231pd	%ymm13, %ymm15, %ymm4
	vfmadd231pd	%ymm14, %ymm15, %ymm8

	vshufpd 	$ 0x5, %ymm15, %ymm15, %ymm15
	vfmadd231pd	%ymm12, %ymm15, %ymm1
	vfmadd231pd	%ymm13, %ymm15, %ymm5
	vfmadd231pd	%ymm14, %ymm15, %ymm9

	vperm2f128 	$ 0x1, %ymm15, %ymm15, %ymm15
	vfmadd231pd	%ymm12, %ymm15, %ymm3
	vfmadd231pd	%ymm13, %ymm15, %ymm7
	vfmadd231pd	%ymm14, %ymm15, %ymm11

	vshufpd 	$ 0x5, %ymm15, %ymm15, %ymm15
	vfmadd231pd	%ymm12, %ymm15, %ymm2
	vmovapd 	64(%r11), %ymm12 // A0[8]
	vfmadd231pd	%ymm13, %ymm15, %ymm6
	vmovapd 	64(%r11, %r12), %ymm13 // A1[8]
	vfmadd231pd	%ymm14, %ymm15, %ymm10
	vmovapd 	64(%r13), %ymm15 // B[8]
	vmovapd 	64(%r11, %r12, 2), %ymm14 // A2[8]


	// unroll 2
	vfmadd231pd	%ymm12, %ymm15, %ymm0
	vfmadd231pd	%ymm13, %ymm15, %ymm4
	vfmadd231pd	%ymm14, %ymm15, %ymm8

	vshufpd 	$ 0x5, %ymm15, %ymm15, %ymm15
	vfmadd231pd	%ymm12, %ymm15, %ymm1
	vfmadd231pd	%ymm13, %ymm15, %ymm5
	vfmadd231pd	%ymm14, %ymm15, %ymm9

	vperm2f128 	$ 0x1, %ymm15, %ymm15, %ymm15
	vfmadd231pd	%ymm12, %ymm15, %ymm3
	vfmadd231pd	%ymm13, %ymm15, %ymm7
	vfmadd231pd	%ymm14, %ymm15, %ymm11

	vshufpd 	$ 0x5, %ymm15, %ymm15, %ymm15
	vfmadd231pd	%ymm12, %ymm15, %ymm2
	vmovapd 	96(%r11), %ymm12 // A0[12]
	vfmadd231pd	%ymm13, %ymm15, %ymm6
	vmovapd 	96(%r11, %r12), %ymm13 // A1[12]
	vfmadd231pd	%ymm14, %ymm15, %ymm10
	vmovapd 	96(%r13), %ymm15 // B[12]
	vmovapd 	96(%r11, %r12, 2), %ymm14 // A2[12]


	// unroll 3
	vfmadd231pd	%ymm12, %ymm15, %ymm0
	addq		$ 128, %r11
	vfmadd231pd	%ymm13, %ymm15, %ymm4
	vfmadd231pd	%ymm14, %ymm15, %ymm8

	vshufpd 	$ 0x5, %ymm15, %ymm15, %ymm15
	vfmadd231pd	%ymm12, %ymm15, %ymm1
	addq		$ 128, %r13
	vfmadd231pd	%ymm13, %ymm15, %ymm5
	vfmadd231pd	%ymm14, %ymm15, %ymm9

	vperm2f128 $ 0x1, %ymm15, %ymm15, %ymm15
	vfmadd231pd	%ymm12, %ymm15, %ymm3
//	cmpl		$ 4, %r10d
	vfmadd231pd	%ymm13, %ymm15, %ymm7
	vfmadd231pd	%ymm14, %ymm15, %ymm11

	vshufpd 	$ 0x5, %ymm15, %ymm15, %ymm15
	vfmadd231pd	%ymm12, %ymm15, %ymm2
//	vmovapd 	0(%r11), %ymm12 // A0[0]
	vfmadd231pd	%ymm13, %ymm15, %ymm6
//	vmovapd 	0(%r11, %r12), %ymm13 // A1[0]
	vfmadd231pd	%ymm14, %ymm15, %ymm10
//	vmovapd 	0(%r13), %ymm15 // B[0]
//	vmovapd 	0(%r11, %r12, 2), %ymm14 // A2[0]


	jmp		2f


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	vmovapd 	0(%r11), %ymm12 // A0[4]
	vmovapd 	0(%r11, %r12), %ymm13 // A1[4]
	vmovapd 	0(%r13), %ymm15 // B[4]
	vmovapd 	0(%r11, %r12, 2), %ymm14 // A2[4]
	vfmadd231pd	%ymm12, %ymm15, %ymm0
	addq		$ 32, %r11
	vfmadd231pd	%ymm13, %ymm15, %ymm4
	addq		$ 32, %r13
	vfmadd231pd	%ymm14, %ymm15, %ymm8

	vshufpd 	$ 0x5, %ymm15, %ymm15, %ymm15
	vfmadd231pd	%ymm12, %ymm15, %ymm1
	vfmadd231pd	%ymm13, %ymm15, %ymm5
	vfmadd231pd	%ymm14, %ymm15, %ymm9

	vperm2f128 	$ 0x1, %ymm15, %ymm15, %ymm15
	vfmadd231pd	%ymm12, %ymm15, %ymm3
	subl		$ 1, %r10d
	vfmadd231pd	%ymm13, %ymm15, %ymm7
	vfmadd231pd	%ymm14, %ymm15, %ymm11

	vshufpd 	$ 0x5, %ymm15, %ymm15, %ymm15
	vfmadd231pd	%ymm12, %ymm15, %ymm2
	vfmadd231pd	%ymm13, %ymm15, %ymm6
	vfmadd231pd	%ymm14, %ymm15, %ymm10


	cmpl		$ 0, %r10d
	jg		3b // clean up loop 


2: // return

#endif

#if MACRO_LEVEL>=2
	.endm
#else
	ret

FUN_END(inner_kernel_dgemm_nt_12x4_lib4)
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
	.macro INNER_KERNEL_DGEMM_NN_12X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_12x4_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	// preload
	vmovapd 0(%r11), %ymm13 // A0[0]
	vmovapd 0(%r11, %r12), %ymm14 // A1[0]
	vmovapd 0(%r11, %r12, 2), %ymm15 // A1[0]

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	prefetcht0	 0(%r13, %r14, 2) // software prefetch
	prefetcht0	64(%r13, %r14, 2) // software prefetch

	// unroll 0
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	32(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	subl	$ 4, %r10d

	vbroadcastsd	64(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	96(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			32(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			32(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			32(%r11, %r12, 2), %ymm15 // A1

	// unroll 1
	vbroadcastsd	8(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	40(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	72(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	104(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			64(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			64(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			64(%r11, %r12, 2), %ymm15 // A1

	// unroll 2
	vbroadcastsd	16(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	48(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	80(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	112(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			96(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			96(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			96(%r11, %r12, 2), %ymm15 // A1

	// unroll 3
	vbroadcastsd	24(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	addq	$ 128, %r11

	vbroadcastsd	56(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	88(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	120(%r13), %ymm12
	addq	%r14, %r13
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			0(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			0(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			0(%r11, %r12, 2), %ymm15 // A1

	cmpl	$ 4, %r10d
	jg		1b // main loop 

0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	32(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	subl	$ 4, %r10d

	vbroadcastsd	64(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	96(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			32(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			32(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			32(%r11, %r12, 2), %ymm15 // A1

	// unroll 1
	vbroadcastsd	8(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	40(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	72(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	104(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			64(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			64(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			64(%r11, %r12, 2), %ymm15 // A1

	// unroll 2
	vbroadcastsd	16(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	48(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	80(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	112(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			96(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			96(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			96(%r11, %r12, 2), %ymm15 // A1

	// unroll 3
	vbroadcastsd	24(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	addq	$ 128, %r11

	vbroadcastsd	56(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	88(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	120(%r13), %ymm12
	addq	%r14, %r13
	vfmadd231pd		%ymm13, %ymm12, %ymm3
//	vmovapd			0(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
//	vmovapd			0(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
//	vmovapd			0(%r11, %r12, 2), %ymm15 // A1

	jmp		2f


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	vmovapd			0(%r11), %ymm13 // A0[0]
	vmovapd 		0(%r11, %r12), %ymm14 // A1[0]
	vmovapd 		0(%r11, %r12, 2), %ymm15 // A2[0]
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	32(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	addq	$ 32, %r11

	vbroadcastsd	64(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	subl	$ 1, %r10d

	vbroadcastsd	96(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	addq	$ 8, %r13

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_12x4_lib4)
#endif





#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEBP_ADD_NN_12X12_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgebp_add_nn_12x12_lib4)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	cmpl	$ 3, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// preload
	vmovapd 0(%r11), %ymm13 // A0[0]
	vmovapd 0(%r11, %r12), %ymm14 // A1[0]
	vmovapd 0(%r11, %r12, 2), %ymm15 // A1[0]

	// rank 4

//	prefetcht0	128(%r13) // software prefetch
//	prefetcht0	128+64(%r13) // software prefetch
	prefetcht0	0(%r13, %r14) // software prefetch
	prefetcht0	0+64(%r13, %r14) // software prefetch

	// unroll 0 & initial load
	vbroadcastsd	0(%r13), %ymm12
	vmovapd			0(%r15), %ymm0
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			0(%r15, %r14), %ymm4
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			0(%r15, %r14, 2), %ymm8
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	32(%r13), %ymm12
	vmovapd			32(%r15), %ymm1
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vmovapd			32(%r15, %r14), %ymm5
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vmovapd			32(%r15, %r14, 2), %ymm9
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	64(%r13), %ymm12
	vmovapd			64(%r15), %ymm2
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vmovapd			64(%r15, %r14), %ymm6
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vmovapd			64(%r15, %r14, 2), %ymm10
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	96(%r13), %ymm12
	vmovapd			96(%r15), %ymm3
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			96(%r15, %r14), %ymm7
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			96(%r15, %r14, 2), %ymm11
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 1
	vbroadcastsd	8(%r13), %ymm12
	vmovapd			32(%r11), %ymm13 // A0
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			32(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			32(%r11, %r12, 2), %ymm15 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	40(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	72(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	104(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			64(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			64(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			64(%r11, %r12, 2), %ymm15 // A1

	// unroll 2
	vbroadcastsd	16(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	48(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	80(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	112(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			96(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			96(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			96(%r11, %r12, 2), %ymm15 // A1

	// unroll 3
	vbroadcastsd	24(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	56(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	88(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	120(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			128+0(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			128+0(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			128+0(%r11, %r12, 2), %ymm15 // A1


	// rank 8

//	prefetcht0	128(%r13, %r14) // software prefetch
//	prefetcht0	128+64(%r13, %r14) // software prefetch
	prefetcht0	0(%r13, %r14, 2) // software prefetch
	prefetcht0	0+64(%r13, %r14, 2) // software prefetch

	// unroll 0
	vbroadcastsd	0(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	32(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	64(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	96(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			128+32(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			128+32(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			128+32(%r11, %r12, 2), %ymm15 // A1

	// unroll 1
	vbroadcastsd	8(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	40(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	72(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	104(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			128+64(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			128+64(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			128+64(%r11, %r12, 2), %ymm15 // A1

	// unroll 2
	vbroadcastsd	16(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	48(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	80(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	112(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			128+96(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			128+96(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			128+96(%r11, %r12, 2), %ymm15 // A1

	// unroll 3
	vbroadcastsd	24(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	56(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	88(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	120(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			256+0(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			256+0(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			256+0(%r11, %r12, 2), %ymm15 // A1


	// rank 12

	prefetcht0	128(%r13) // software prefetch
	prefetcht0	128+64(%r13) // software prefetch

	// unroll 0
	vbroadcastsd	0(%r13, %r14, 2), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	32(%r13, %r14, 2), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	64(%r13, %r14, 2), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	96(%r13, %r14, 2), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			256+32(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			256+32(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			256+32(%r11, %r12, 2), %ymm15 // A1

	// unroll 1
	vbroadcastsd	8(%r13, %r14, 2), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	40(%r13, %r14, 2), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	72(%r13, %r14, 2), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	104(%r13, %r14, 2), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			256+64(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			256+64(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			256+64(%r11, %r12, 2), %ymm15 // A1

	// unroll 2
	vbroadcastsd	16(%r13, %r14, 2), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	48(%r13, %r14, 2), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	80(%r13, %r14, 2), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	112(%r13, %r14, 2), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			256+96(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			256+96(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			256+96(%r11, %r12, 2), %ymm15 // A1

	// unroll 3
	vbroadcastsd	24(%r13, %r14, 2), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			%ymm0, 0(%r15)
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			%ymm4, 0(%r15, %r14)
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vmovapd			%ymm8, 0(%r15, %r14, 2)
	subl	$ 4, %r10d

	vbroadcastsd	56(%r13, %r14, 2), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vmovapd			%ymm1, 32(%r15)
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vmovapd			%ymm5, 32(%r15, %r14)
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vmovapd			%ymm9, 32(%r15, %r14, 2)

	vbroadcastsd	88(%r13, %r14, 2), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vmovapd			%ymm2, 64(%r15)
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vmovapd			%ymm6, 64(%r15, %r14)
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vmovapd			%ymm10, 64(%r15, %r14, 2)

	vbroadcastsd	120(%r13, %r14, 2), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			%ymm3, 96(%r15)
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			%ymm7, 96(%r15, %r14)
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			%ymm11, 96(%r15, %r14, 2)
	addq	$ 128, %r13
	addq	$ 128, %r15

//	prefetcht0	128(%r15) // software prefetch
//	prefetcht0	128+64(%r15) // software prefetch

	cmpl	$ 3, %r10d
	jg		1b // main loop 

	jmp		2f


0: // clean up loop

	// preload
	vmovapd 0(%r11), %ymm13 // A0[0]
	vmovapd 0(%r11, %r12), %ymm14 // A1[0]
	vmovapd 0(%r11, %r12, 2), %ymm15 // A1[0]

	// rank 4

	// unroll 0 & initial load
	vbroadcastsd	0(%r13), %ymm12
	vmovapd			0(%r15), %ymm0
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			0(%r15, %r14), %ymm4
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			0(%r15, %r14, 2), %ymm8
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	// unroll 1
	vbroadcastsd	8(%r13), %ymm12
	vmovapd			32(%r11), %ymm13 // A0
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			32(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			32(%r11, %r12, 2), %ymm15 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	// unroll 2
	vbroadcastsd	16(%r13), %ymm12
	vmovapd			64(%r11), %ymm13 // A0
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			64(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			64(%r11, %r12, 2), %ymm15 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	// unroll 3
	vbroadcastsd	24(%r13), %ymm12
	vmovapd			96(%r11), %ymm13 // A0
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			96(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			96(%r11, %r12, 2), %ymm15 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm8


	// rank 8

	// unroll 0
	vbroadcastsd	0(%r13, %r14), %ymm12
	vmovapd			128+0(%r11), %ymm13 // A0
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			128+0(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			128+0(%r11, %r12, 2), %ymm15 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	// unroll 1
	vbroadcastsd	8(%r13, %r14), %ymm12
	vmovapd			128+32(%r11), %ymm13 // A0
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			128+32(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			128+32(%r11, %r12, 2), %ymm15 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	// unroll 2
	vbroadcastsd	16(%r13, %r14), %ymm12
	vmovapd			128+64(%r11), %ymm13 // A0
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			128+64(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			128+64(%r11, %r12, 2), %ymm15 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	// unroll 3
	vbroadcastsd	24(%r13, %r14), %ymm12
	vmovapd			128+96(%r11), %ymm13 // A0
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			128+96(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			128+96(%r11, %r12, 2), %ymm15 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm8


	// rank 12

	// unroll 0
	vbroadcastsd	0(%r13, %r14, 2), %ymm12
	vmovapd			256+0(%r11), %ymm13 // A0
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			256+0(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			256+0(%r11, %r12, 2), %ymm15 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm8


	// unroll 1
	vbroadcastsd	8(%r13, %r14, 2), %ymm12
	vmovapd			256+32(%r11), %ymm13 // A0
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			256+32(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			256+32(%r11, %r12, 2), %ymm15 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	// unroll 2
	vbroadcastsd	16(%r13, %r14, 2), %ymm12
	vmovapd			256+64(%r11), %ymm13 // A0
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			256+64(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			256+64(%r11, %r12, 2), %ymm15 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	subl	$ 1, %r10d

	// unroll 3
	vbroadcastsd	24(%r13, %r14, 2), %ymm12
	vmovapd			256+96(%r11), %ymm13 // A0
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			%ymm0, 0(%r15)
	vmovapd			256+96(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			%ymm4, 0(%r15, %r14)
	vmovapd			256+96(%r11, %r12, 2), %ymm15 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vmovapd			%ymm8, 0(%r15, %r14, 2)
	addq	$ 32, %r13
	addq	$ 32, %r15

	cmpl	$ 0, %r10d
	jg		0b // main loop 


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgebp_add_nn_12x12_lib4)
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
	.macro INNER_KERNEL_DGEMM_NN_4X12_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_4x12_lib4)
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
	prefetcht0	256(%r12, %r13, 2) // software prefetch
	prefetcht0	320(%r12, %r13, 2) // software prefetch

	// unroll 0
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			32(%r11), %ymm14 // A
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	96(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	128(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	160(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	192(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	224(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	256(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	288(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	320(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	352(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm0
	vmovapd			64(%r11), %ymm13 // A
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm1
	vbroadcastsd	72(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm2
	vbroadcastsd	104(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm3
	vbroadcastsd	136(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vbroadcastsd	168(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vbroadcastsd	200(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vbroadcastsd	232(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vbroadcastsd	264(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm8
	vbroadcastsd	296(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm9
	vbroadcastsd	328(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm10
	vbroadcastsd	360(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm11
	addq	$ 128, %r11

	// unroll 0
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			-32(%r11), %ymm14 // A
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	80(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	112(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	144(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	176(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	208(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	240(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	272(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	304(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	336(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	368(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11

	// unroll 0
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm0
	vmovapd			0(%r11), %ymm13 // A
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm1
	vbroadcastsd	88(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm2
	vbroadcastsd	120(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm3
	vbroadcastsd	152(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vbroadcastsd	184(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vbroadcastsd	216(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vbroadcastsd	248(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vbroadcastsd	280(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm8
	vbroadcastsd	312(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm9
	vbroadcastsd	344(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm10
	vbroadcastsd	376(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm11
	addq	%r13, %r12

	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			32(%r11), %ymm14 // A
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	96(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	128(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	160(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	192(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	224(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	256(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	288(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	320(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	352(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm0
	vmovapd			64(%r11), %ymm13 // A
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm1
	vbroadcastsd	72(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm2
	vbroadcastsd	104(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm3
	vbroadcastsd	136(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vbroadcastsd	168(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vbroadcastsd	200(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vbroadcastsd	232(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vbroadcastsd	264(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm8
	vbroadcastsd	296(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm9
	vbroadcastsd	328(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm10
	vbroadcastsd	360(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm11
	addq	$ 128, %r11

	// unroll 0
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			-32(%r11), %ymm14 // A
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	80(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	112(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	144(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	176(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	208(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	240(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	272(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	304(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	336(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	368(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11

	// unroll 0
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm0
//	vmovapd			0(%r11), %ymm13 // A
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm1
	vbroadcastsd	88(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm2
	vbroadcastsd	120(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm3
	vbroadcastsd	152(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vbroadcastsd	184(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vbroadcastsd	216(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vbroadcastsd	248(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vbroadcastsd	280(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm8
	vbroadcastsd	312(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm9
	vbroadcastsd	344(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm10
	vbroadcastsd	376(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm11
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
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	96(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vbroadcastsd	128(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm4
	vbroadcastsd	160(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm5
	vbroadcastsd	192(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm6
	vbroadcastsd	224(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm7
	vbroadcastsd	256(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm8
	vbroadcastsd	288(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm9
	vbroadcastsd	320(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm10
	vbroadcastsd	352(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm11

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

	FUN_END(inner_kernel_dgemm_nn_4x12_lib4)
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
// ymm4  <-
// ymm5  <-
// ymm6  <-
// ymm7  <-
// ymm8  <-
// ymm9  <-
// ymm10 <-
// ymm11 <-

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
// ymm4  <-
// ymm5  <-
// ymm6  <-
// ymm7  <-
// ymm8  <-
// ymm9  <-
// ymm10 <-
// ymm11 <-

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEBP_ADD_NN_12X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgebp_add_nn_12x4_lib4)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return

	cmpl	$ 3, %r10d
	jle		2f // cleanup loop

	// main loop
	.p2align 3
1:
	vmovapd			0(%r12), %ymm12
	vmovapd			0(%r12, %r13), %ymm14
	vmovapd			0(%r12, %r13, 2), %ymm15
	vbroadcastsd	0(%r11), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm12
	vfmadd231pd		%ymm4, %ymm13, %ymm14
	vfmadd231pd		%ymm8, %ymm13, %ymm15
	vbroadcastsd	8(%r11), %ymm13
	subl	$ 4, %r10d
	vfmadd231pd		%ymm1, %ymm13, %ymm12
	vfmadd231pd		%ymm5, %ymm13, %ymm14
	vfmadd231pd		%ymm9, %ymm13, %ymm15
	vbroadcastsd	16(%r11), %ymm13
	vfmadd231pd		%ymm2, %ymm13, %ymm12
	vfmadd231pd		%ymm6, %ymm13, %ymm14
	vfmadd231pd		%ymm10, %ymm13, %ymm15
	vbroadcastsd	24(%r11), %ymm13
	vfmadd231pd		%ymm3, %ymm13, %ymm12
	vfmadd231pd		%ymm7, %ymm13, %ymm14
	vfmadd231pd		%ymm11, %ymm13, %ymm15
	vmovapd			%ymm12, 0(%r12)
	vmovapd			%ymm14, 0(%r12, %r13)
	vmovapd			%ymm15, 0(%r12, %r13, 2)

	vmovapd			32(%r12), %ymm12
	vmovapd			32(%r12, %r13), %ymm14
	vmovapd			32(%r12, %r13, 2), %ymm15
	vbroadcastsd	32(%r11), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm12
	vfmadd231pd		%ymm4, %ymm13, %ymm14
	vfmadd231pd		%ymm8, %ymm13, %ymm15
	vbroadcastsd	40(%r11), %ymm13
	vfmadd231pd		%ymm1, %ymm13, %ymm12
	vfmadd231pd		%ymm5, %ymm13, %ymm14
	vfmadd231pd		%ymm9, %ymm13, %ymm15
	vbroadcastsd	48(%r11), %ymm13
	vfmadd231pd		%ymm2, %ymm13, %ymm12
	vfmadd231pd		%ymm6, %ymm13, %ymm14
	vfmadd231pd		%ymm10, %ymm13, %ymm15
	vbroadcastsd	56(%r11), %ymm13
	vfmadd231pd		%ymm3, %ymm13, %ymm12
	vfmadd231pd		%ymm7, %ymm13, %ymm14
	vfmadd231pd		%ymm11, %ymm13, %ymm15
	vmovapd			%ymm12, 32(%r12)
	vmovapd			%ymm14, 32(%r12, %r13)
	vmovapd			%ymm15, 32(%r12, %r13, 2)

	vmovapd			64(%r12), %ymm12
	vmovapd			64(%r12, %r13), %ymm14
	vmovapd			64(%r12, %r13, 2), %ymm15
	vbroadcastsd	64(%r11), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm12
	vfmadd231pd		%ymm4, %ymm13, %ymm14
	vfmadd231pd		%ymm8, %ymm13, %ymm15
	vbroadcastsd	72(%r11), %ymm13
	vfmadd231pd		%ymm1, %ymm13, %ymm12
	vfmadd231pd		%ymm5, %ymm13, %ymm14
	vfmadd231pd		%ymm9, %ymm13, %ymm15
	vbroadcastsd	80(%r11), %ymm13
	vfmadd231pd		%ymm2, %ymm13, %ymm12
	vfmadd231pd		%ymm6, %ymm13, %ymm14
	vfmadd231pd		%ymm10, %ymm13, %ymm15
	vbroadcastsd	88(%r11), %ymm13
	vfmadd231pd		%ymm3, %ymm13, %ymm12
	vfmadd231pd		%ymm7, %ymm13, %ymm14
	vfmadd231pd		%ymm11, %ymm13, %ymm15
	vmovapd			%ymm12, 64(%r12)
	vmovapd			%ymm14, 64(%r12, %r13)
	vmovapd			%ymm15, 64(%r12, %r13, 2)

	vmovapd			96(%r12), %ymm12
	vmovapd			96(%r12, %r13), %ymm14
	vmovapd			96(%r12, %r13, 2), %ymm15
	vbroadcastsd	96(%r11), %ymm13
	addq	$ 128, %r11
	vfmadd231pd		%ymm0, %ymm13, %ymm12
	vfmadd231pd		%ymm4, %ymm13, %ymm14
	vfmadd231pd		%ymm8, %ymm13, %ymm15
	vbroadcastsd	-24(%r11), %ymm13
	vfmadd231pd		%ymm1, %ymm13, %ymm12
	vfmadd231pd		%ymm5, %ymm13, %ymm14
	vfmadd231pd		%ymm9, %ymm13, %ymm15
	vbroadcastsd	-16(%r11), %ymm13
	vfmadd231pd		%ymm2, %ymm13, %ymm12
	vfmadd231pd		%ymm6, %ymm13, %ymm14
	vfmadd231pd		%ymm10, %ymm13, %ymm15
	vbroadcastsd	-8(%r11), %ymm13
	addq	$ 128, %r12
	vfmadd231pd		%ymm3, %ymm13, %ymm12
	vfmadd231pd		%ymm7, %ymm13, %ymm14
	vfmadd231pd		%ymm11, %ymm13, %ymm15
	vmovapd			%ymm12, -32(%r12)
	vmovapd			%ymm14, -32(%r12, %r13)
	vmovapd			%ymm15, -32(%r12, %r13, 2)

	cmpl	$ 3, %r10d
	jg		1b // main loop

	cmpl	$ 0, %r10d
	jle		0f // return

	// cleanup loop
2:
	vmovapd			0(%r12), %ymm12
	vmovapd			0(%r12, %r13), %ymm14
	vmovapd			0(%r12, %r13, 2), %ymm15
	vbroadcastsd	0(%r11), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm12
	vfmadd231pd		%ymm4, %ymm13, %ymm14
	vfmadd231pd		%ymm8, %ymm13, %ymm15
	vbroadcastsd	8(%r11), %ymm13
	vfmadd231pd		%ymm1, %ymm13, %ymm12
	vfmadd231pd		%ymm5, %ymm13, %ymm14
	vfmadd231pd		%ymm9, %ymm13, %ymm15
	vbroadcastsd	16(%r11), %ymm13
	vfmadd231pd		%ymm2, %ymm13, %ymm12
	vfmadd231pd		%ymm6, %ymm13, %ymm14
	vfmadd231pd		%ymm10, %ymm13, %ymm15
	vbroadcastsd	24(%r11), %ymm13
	vfmadd231pd		%ymm3, %ymm13, %ymm12
	vfmadd231pd		%ymm7, %ymm13, %ymm14
	vfmadd231pd		%ymm11, %ymm13, %ymm15
	vmovapd			%ymm12, 0(%r12)
	vmovapd			%ymm14, 0(%r12, %r13)
	vmovapd			%ymm15, 0(%r12, %r13, 2)

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

	FUN_END(inner_kernel_dgebp_add_nn_12x4_lib4)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// r13   <- 32*sdb
// r14   <- C
// ymm0  <- []
// ymm1  <- []
// ymm2  <- []
// ymm3  <- []
// ymm4  <-
// ymm5  <-
// ymm6  <-
// ymm7  <-
// ymm8  <-
// ymm9  <-
// ymm10 <-
// ymm11 <-

//
// output arguments:
// r10d  <- 0
// r11   <- A
// r12   <- B+?
// r13   <- 32*sdb
// r14   <- C+?
// ymm0  <- []
// ymm1  <- []
// ymm2  <- []
// ymm3  <- []
// ymm4  <-
// ymm5  <-
// ymm6  <-
// ymm7  <-
// ymm8  <-
// ymm9  <-
// ymm10 <-
// ymm11 <-

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEBP_ADD_NN_4X12_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgebp_add_nn_4x12_lib4)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return

	cmpl	$ 11, %r10d
	jle		2f // cleanup loop

	// main loop
	.p2align 3
1:
	// load block from C
	vmovapd	0(%r14), %ymm0
	vmovapd	32(%r14), %ymm1
	vmovapd	64(%r14), %ymm2
	vmovapd	96(%r14), %ymm3
	vmovapd	128(%r14), %ymm4
	vmovapd	160(%r14), %ymm5
	vmovapd	192(%r14), %ymm6
	vmovapd	224(%r14), %ymm7
	vmovapd	256(%r14), %ymm8
	vmovapd	288(%r14), %ymm9
	vmovapd	320(%r14), %ymm10
	vmovapd	352(%r14), %ymm11

	// 0
	vmovapd			0(%r11), %ymm12
	vbroadcastsd	0(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	32(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	64(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	96(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	128(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	160(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	192(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	224(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	256(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	288(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	320(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	352(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11

	// 1
	vmovapd			32(%r11), %ymm12
	vbroadcastsd	8(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	40(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	72(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	104(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	136(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	168(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	200(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	232(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	264(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	296(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	328(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	360(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11

	// 2
	vmovapd			64(%r11), %ymm12
	vbroadcastsd	16(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	48(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	80(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	112(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	144(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	176(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	208(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	240(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	272(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	304(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	336(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	368(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11

	// 3
	vmovapd			96(%r11), %ymm12
	vbroadcastsd	24(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	56(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	88(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	120(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	152(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	184(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	216(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	248(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	280(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	312(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	344(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	376(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11

	movq		%r12, %r15
	addq		%r13, %r15

	// 4
	vmovapd			128(%r11), %ymm12
	vbroadcastsd	0(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	32(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	64(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	96(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	128(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	160(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	192(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	224(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	256(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	288(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	320(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	352(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11

	// 5
	vmovapd			160(%r11), %ymm12
	vbroadcastsd	8(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	40(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	72(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	104(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	136(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	168(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	200(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	232(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	264(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	296(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	328(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	360(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11

	// 6
	vmovapd			192(%r11), %ymm12
	vbroadcastsd	16(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	48(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	80(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	112(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	144(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	176(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	208(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	240(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	272(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	304(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	336(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	368(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11

	// 7
	vmovapd			224(%r11), %ymm12
	vbroadcastsd	24(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	56(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	88(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	120(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	152(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	184(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	216(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	248(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	280(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	312(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	344(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	376(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11

	addq		%r13, %r15

	// 8
	vmovapd			256(%r11), %ymm12
	vbroadcastsd	0(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	32(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	64(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	96(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	128(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	160(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	192(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	224(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	256(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	288(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	320(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	352(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11

	// 9
	vmovapd			288(%r11), %ymm12
	vbroadcastsd	8(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	40(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	72(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	104(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	136(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	168(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	200(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	232(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	264(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	296(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	328(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	360(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11

	// 10
	vmovapd			320(%r11), %ymm12
	vbroadcastsd	16(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	48(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	80(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	112(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	144(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	176(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	208(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	240(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	272(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	304(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	336(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	368(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11

	// 11
	vmovapd			352(%r11), %ymm12
	vbroadcastsd	24(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	56(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	88(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	120(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	152(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	184(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	216(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	248(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	280(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	312(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	344(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	376(%r15), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11

	// store block to C
	vmovapd	%ymm0, 0(%r14)
	vmovapd	%ymm1, 32(%r14)
	vmovapd	%ymm2, 64(%r14)
	vmovapd	%ymm3, 96(%r14)
	vmovapd	%ymm4, 128(%r14)
	vmovapd	%ymm5, 160(%r14)
	vmovapd	%ymm6, 192(%r14)
	vmovapd	%ymm7, 224(%r14)
	vmovapd	%ymm8, 256(%r14)
	vmovapd	%ymm9, 288(%r14)
	vmovapd	%ymm10, 320(%r14)
	vmovapd	%ymm11, 352(%r14)

	addq	$ 384, %r12
	addq	$ 384, %r14
	subl	$ 12, %r10d

	cmpl	$ 11, %r10d
	jg		1b // main loop

2:
	cmpl	$ 3, %r10d
	jle		2f // return

	// cleanup loop
1:
	// load block from C
	vmovapd	0(%r14), %ymm0
	vmovapd	32(%r14), %ymm1
	vmovapd	64(%r14), %ymm2
	vmovapd	96(%r14), %ymm3

	// 0
	vmovapd			0(%r11), %ymm12
	vbroadcastsd	0(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	32(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	64(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	96(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3

	// 1
	vmovapd			32(%r11), %ymm12
	vbroadcastsd	8(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	40(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	72(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	104(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3

	// 2
	vmovapd			64(%r11), %ymm12
	vbroadcastsd	16(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	48(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	80(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	112(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3

	// 3
	vmovapd			96(%r11), %ymm12
	vbroadcastsd	24(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	56(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	88(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	120(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3

	// 4
	vmovapd			128(%r11), %ymm12
	vbroadcastsd	0(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	32(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	64(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	96(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3

	// 5
	vmovapd			160(%r11), %ymm12
	vbroadcastsd	8(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	40(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	72(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	104(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3

	// 6
	vmovapd			192(%r11), %ymm12
	vbroadcastsd	16(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	48(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	80(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	112(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3

	// 7
	vmovapd			224(%r11), %ymm12
	vbroadcastsd	24(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	56(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	88(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	120(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3

	// 8
	vmovapd			256(%r11), %ymm12
	vbroadcastsd	0(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	32(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	64(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	96(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3

	// 9
	vmovapd			288(%r11), %ymm12
	vbroadcastsd	8(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	40(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	72(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	104(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3

	// 10
	vmovapd			320(%r11), %ymm12
	vbroadcastsd	16(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	48(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	80(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	112(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3

	// 11
	vmovapd			352(%r11), %ymm12
	vbroadcastsd	24(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	56(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	88(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	120(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3

	// store block to C
	vmovapd	%ymm0, 0(%r14)
	vmovapd	%ymm1, 32(%r14)
	vmovapd	%ymm2, 64(%r14)
	vmovapd	%ymm3, 96(%r14)

	addq	$ 128, %r12
	addq	$ 128, %r14
	subl	$ 4, %r10d

	cmpl	$ 3, %r10d
	jg		1b // main loop

2:
	cmpl	$ 0, %r10d
	jle		0f // return

	// cleanup loop
1:
	// load block from C
	vmovapd	0(%r14), %ymm0

	// 0
	vmovapd			0(%r11), %ymm12
	vbroadcastsd	0(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0

	// 1
	vmovapd			32(%r11), %ymm12
	vbroadcastsd	8(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0

	// 2
	vmovapd			64(%r11), %ymm12
	vbroadcastsd	16(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0

	// 3
	vmovapd			96(%r11), %ymm12
	vbroadcastsd	24(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0

	// 4
	vmovapd			128(%r11), %ymm12
	vbroadcastsd	0(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0

	// 5
	vmovapd			160(%r11), %ymm12
	vbroadcastsd	8(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0

	// 6
	vmovapd			192(%r11), %ymm12
	vbroadcastsd	16(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0

	// 7
	vmovapd			224(%r11), %ymm12
	vbroadcastsd	24(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0

	// 8
	vmovapd			256(%r11), %ymm12
	vbroadcastsd	0(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0

	// 9
	vmovapd			288(%r11), %ymm12
	vbroadcastsd	8(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0

	// 10
	vmovapd			320(%r11), %ymm12
	vbroadcastsd	16(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0

	// 11
	vmovapd			352(%r11), %ymm12
	vbroadcastsd	24(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0

	// store block to C
	vmovapd	%ymm0, 0(%r14)

	addq	$ 32, %r12
	addq	$ 32, %r14
	subl	$ 1, %r10d

	cmpl	$ 0, %r10d
	jg		1b // main loop

	// return
0:

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgebp_add_nn_4x12_lib4)
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
	.macro INNER_EDGE_DGEMM_NN_12X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dgemm_nn_12x4_lib4)
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
	vmovapd			0(%r11, %r12), %ymm14 // A1[0]
	vmovapd			0(%r11, %r12, 2), %ymm15 // A2[0]
	vbroadcastsd	0(%r13), %ymm13 // B[0]
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vfmadd231pd		%ymm14, %ymm13, %ymm4
	vfmadd231pd		%ymm15, %ymm13, %ymm8
	vbroadcastsd	32(%r13), %ymm13 // B[1]
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vfmadd231pd		%ymm14, %ymm13, %ymm5
	vfmadd231pd		%ymm15, %ymm13, %ymm9
	vbroadcastsd	64(%r13), %ymm13 // B[2]
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vfmadd231pd		%ymm14, %ymm13, %ymm6
	vfmadd231pd		%ymm15, %ymm13, %ymm10
	vbroadcastsd	96(%r13), %ymm13 // B[3]
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vfmadd231pd		%ymm14, %ymm13, %ymm7
	vfmadd231pd		%ymm15, %ymm13, %ymm11

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

	FUN_END(inner_edge_dgemm_nn_12x4_lib4)
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
	.macro INNER_EDGE_DGEMM_NN_4X12_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dgemm_nn_4x12_lib4)
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
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	32(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	64(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	96(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	128(%r12), %ymm13 // B
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	160(%r12), %ymm13 // B
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	192(%r12), %ymm13 // B
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	224(%r12), %ymm13 // B
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	256(%r12), %ymm13 // B
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	288(%r12), %ymm13 // B
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	320(%r12), %ymm13 // B
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	352(%r12), %ymm13 // B
	vfmadd231pd		%ymm12, %ymm13, %ymm11

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

	FUN_END(inner_edge_dgemm_nn_4x12_lib4)
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
// ymm8  <- [d80 d90 da0 db0]
// ymm9  <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

//
// output arguments:
// r10   <- kmax
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
// ymm8  <- [d80 d90 da0 db0]
// ymm9  <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NT_RU_12X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nt_ru_12x4_lib4)
#endif
	
	vbroadcastsd	0(%r13), %ymm12
	vmovapd			0(%r11), %ymm13
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			0(%r11, %r12), %ymm14
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			0(%r11, %r12, 2), %ymm15
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	32(%r13), %ymm12
	vmovapd			32(%r11), %ymm13
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			32(%r11, %r12), %ymm14
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			32(%r11, %r12, 2), %ymm15
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	40(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	64(%r13), %ymm12
	vmovapd			64(%r11), %ymm13
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			64(%r11, %r12), %ymm14
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			64(%r11, %r12, 2), %ymm15
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	72(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	80(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	96(%r13), %ymm12
	vmovapd			96(%r11), %ymm13
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			96(%r11, %r12), %ymm14
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			96(%r11, %r12, 2), %ymm15
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	104(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	112(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	120(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	subl			$ 4, %r10d
	addq			$ 128, %r11
	addq			$ 128, %r13

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nt_ru_12x4_lib4)
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

//
// output arguments:
// r10d  <- max(k-4,0)
// r11   <- A+4*4*sizeof(double)
// r12   <- 4*sda*sizeof(double)
// r13   <- B+4*4*sizeof(double)
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
	.macro INNER_EDGE_DTRMM_NT_RU_12X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nt_ru_12x4_vs_lib4)
#endif
	
	vbroadcastsd	0(%r13), %ymm12
	vmovapd			0(%r11), %ymm13
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			0(%r11, %r12), %ymm14
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			0(%r11, %r12, 2), %ymm15
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	subl			$ 1, %r10d
	addq			$ 32, %r13
	addq			$ 32, %r11

	cmpl	$ 0, %r10d
	jle		0f

	vbroadcastsd	0(%r13), %ymm12
	vmovapd			0(%r11), %ymm13
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			0(%r11, %r12), %ymm14
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			0(%r11, %r12, 2), %ymm15
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	subl			$ 1, %r10d
	addq			$ 32, %r11
	addq			$ 32, %r13

	cmpl	$ 0, %r10d
	jle		0f

	vbroadcastsd	0(%r13), %ymm12
	vmovapd			0(%r11), %ymm13
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			0(%r11, %r12), %ymm14
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			0(%r11, %r12, 2), %ymm15
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	subl			$ 1, %r10d
	addq			$ 32, %r11
	addq			$ 32, %r13

	cmpl	$ 0, %r10d
	jle		0f

	vbroadcastsd	0(%r13), %ymm12
	vmovapd			0(%r11), %ymm13
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			0(%r11, %r12), %ymm14
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			0(%r11, %r12, 2), %ymm15
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	subl			$ 1, %r10d
	addq			$ 32, %r11
	addq			$ 32, %r13

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nt_ru_12x4_vs_lib4)
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
// ymm8  <- [d80 d90 da0 db0]
// ymm9  <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

//
// output arguments:
// r10   <- kmax
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
// ymm8  <- [d80 d90 da0 db0]
// ymm9  <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NT_RU_ONE_12X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nt_ru_one_12x4_lib4)
#endif
	
	vmovapd			0(%r11), %ymm13
	vaddpd			%ymm0, %ymm13, %ymm0
	vmovapd			0(%r11, %r12), %ymm14
	vaddpd			%ymm4, %ymm14, %ymm4
	vmovapd			0(%r11, %r12, 2), %ymm15
	vaddpd			%ymm8, %ymm15, %ymm8

	vbroadcastsd	32(%r13), %ymm12
	vmovapd			32(%r11), %ymm13
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			32(%r11, %r12), %ymm14
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			32(%r11, %r12, 2), %ymm15
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vaddpd			%ymm1, %ymm13, %ymm1
	vaddpd			%ymm5, %ymm14, %ymm5
	vaddpd			%ymm9, %ymm15, %ymm9

	vbroadcastsd	64(%r13), %ymm12
	vmovapd			64(%r11), %ymm13
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			64(%r11, %r12), %ymm14
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			64(%r11, %r12, 2), %ymm15
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	72(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vaddpd			%ymm2, %ymm13, %ymm2
	vaddpd			%ymm6, %ymm14, %ymm6
	vaddpd			%ymm10, %ymm15, %ymm10

	vbroadcastsd	96(%r13), %ymm12
	vmovapd			96(%r11), %ymm13
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			96(%r11, %r12), %ymm14
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			96(%r11, %r12, 2), %ymm15
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	104(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	112(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vaddpd			%ymm3, %ymm13, %ymm3
	vaddpd			%ymm7, %ymm14, %ymm7
	vaddpd			%ymm11, %ymm15, %ymm11

	subl			$ 4, %r10d
	addq			$ 128, %r11
	addq			$ 128, %r13

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nt_ru_one_12x4_lib4)
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

//
// output arguments:
// r10d  <- max(k-4,0)
// r11   <- A+4*4*sizeof(double)
// r12   <- 4*sda*sizeof(double)
// r13   <- B+4*4*sizeof(double)
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
	.macro INNER_EDGE_DTRMM_NT_RU_ONE_12X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nt_ru_one_12x4_vs_lib4)
#endif
	
	vmovapd			0(%r11), %ymm13
	vaddpd			%ymm0, %ymm13, %ymm0
	vmovapd			0(%r11, %r12), %ymm14
	vaddpd			%ymm4, %ymm14, %ymm4
	vmovapd			0(%r11, %r12, 2), %ymm15
	vaddpd			%ymm8, %ymm15, %ymm8
	subl			$ 1, %r10d
	addq			$ 32, %r13
	addq			$ 32, %r11

	cmpl	$ 0, %r10d
	jle		0f

	vbroadcastsd	0(%r13), %ymm12
	vmovapd			0(%r11), %ymm13
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			0(%r11, %r12), %ymm14
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			0(%r11, %r12, 2), %ymm15
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vaddpd			%ymm1, %ymm13, %ymm1
	vaddpd			%ymm5, %ymm14, %ymm5
	vaddpd			%ymm9, %ymm15, %ymm9
	subl			$ 1, %r10d
	addq			$ 32, %r11
	addq			$ 32, %r13

	cmpl	$ 0, %r10d
	jle		0f

	vbroadcastsd	0(%r13), %ymm12
	vmovapd			0(%r11), %ymm13
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			0(%r11, %r12), %ymm14
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			0(%r11, %r12, 2), %ymm15
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vaddpd			%ymm2, %ymm13, %ymm2
	vaddpd			%ymm6, %ymm14, %ymm6
	vaddpd			%ymm10, %ymm15, %ymm10
	subl			$ 1, %r10d
	addq			$ 32, %r11
	addq			$ 32, %r13

	cmpl	$ 0, %r10d
	jle		0f

	vbroadcastsd	0(%r13), %ymm12
	vmovapd			0(%r11), %ymm13
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			0(%r11, %r12), %ymm14
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			0(%r11, %r12, 2), %ymm15
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vaddpd			%ymm3, %ymm13, %ymm3
	vaddpd			%ymm7, %ymm14, %ymm7
	vaddpd			%ymm11, %ymm15, %ymm11
	subl			$ 1, %r10d
	addq			$ 32, %r11
	addq			$ 32, %r13

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nt_ru_one_12x4_vs_lib4)
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
// ymm8  <- [d40 d50 d60 d70]
// ymm9  <- [d41 d51 d61 d71]
// ymm10  <- [d42 d52 d62 d72]
// ymm11 <- [d43 d53 d63 d73]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
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
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NN_RL_12X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nn_rl_12x4_lib4)
#endif
	
	cmpl	$ 0, %r15d
	jg		0f

	// offB==0

	vmovapd			0(%r11), %ymm13
	vmovapd			0(%r11, %r12), %ymm14
	vmovapd			0(%r11, %r12, 2), %ymm15
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vmovapd			32(%r11), %ymm13
	vmovapd			32(%r11, %r12), %ymm14
	vmovapd			32(%r11, %r12, 2), %ymm15
	vbroadcastsd	8(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	40(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vmovapd			64(%r11), %ymm13
	vmovapd			64(%r11, %r12), %ymm14
	vmovapd			64(%r11, %r12, 2), %ymm15
	vbroadcastsd	16(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	48(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	80(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vmovapd			96(%r11), %ymm13
	vmovapd			96(%r11, %r12), %ymm14
	vmovapd			96(%r11, %r12, 2), %ymm15
	vbroadcastsd	24(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	56(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	88(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	120(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	subl			$ 4, %r10d // k-4
	addq			$ 128, %r11 // A0+4*bs*sizeof(double)
	addq			%r14, %r13 // B+bs*sdb*sizeof(double)

	jmp		3f

0:
	cmpl	$ 1, %r15d
	jg		1f

	// offB==1

	addq			$ 8, %r13 // B+1*sizeof(double)

	vmovapd			0(%r11), %ymm13
	vmovapd			0(%r11, %r12), %ymm14
	vmovapd			0(%r11, %r12, 2), %ymm15
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vmovapd			32(%r11), %ymm13
	vmovapd			32(%r11, %r12), %ymm14
	vmovapd			32(%r11, %r12, 2), %ymm15
	vbroadcastsd	8(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	40(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vmovapd			64(%r11), %ymm13
	vmovapd			64(%r11, %r12), %ymm14
	vmovapd			64(%r11, %r12, 2), %ymm15
	vbroadcastsd	16(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	48(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	80(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

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

	vmovapd			0(%r11), %ymm13
	vmovapd			0(%r11, %r12), %ymm14
	vmovapd			0(%r11, %r12, 2), %ymm15
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vmovapd			32(%r11), %ymm13
	vmovapd			32(%r11, %r12), %ymm14
	vmovapd			32(%r11, %r12, 2), %ymm15
	vbroadcastsd	8(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	40(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	subl			$ 2, %r10d // k-2
	addq			$ 64, %r11 // A0+2*bs*sizeof(double)
	addq			%r14, %r13
	subq			$ 16, %r13 // B+bs*sdb*sizeof(double)-2

	vmovapd			0(%r11), %ymm13
	vmovapd			0(%r11, %r12), %ymm14
	vmovapd			0(%r11, %r12, 2), %ymm15
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	32(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	64(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vmovapd			32(%r11), %ymm13
	vmovapd			32(%r11, %r12), %ymm14
	vmovapd			32(%r11, %r12, 2), %ymm15
	vbroadcastsd	8(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	40(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	72(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	104(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	vmovapd			64(%r11), %ymm13
	vmovapd			64(%r11, %r12), %ymm14
	vmovapd			64(%r11, %r12, 2), %ymm15
	vbroadcastsd	16(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	48(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	80(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	112(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	vmovapd			96(%r11), %ymm13
	vmovapd			96(%r11, %r12), %ymm14
	vmovapd			96(%r11, %r12, 2), %ymm15
	vbroadcastsd	24(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	56(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	88(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	120(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	subl			$ 4, %r10d // k-4
	addq			$ 128, %r11 // A0+4*bs*sizeof(double)
	addq			%r14, %r13 // B+bs*sdb*sizeof(double)

	jmp		3f

2:
	// offB==3

	addq			$ 24, %r13 // B+3*sizeof(double)

	vmovapd			0(%r11), %ymm13
	vmovapd			0(%r11, %r12), %ymm14
	vmovapd			0(%r11, %r12, 2), %ymm15
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A0+1*bs*sizeof(double)
	addq			%r14, %r13
	subq			$ 24, %r13 // B+bs*sdb*sizeof(double)-3

	vmovapd			0(%r11), %ymm13
	vmovapd			0(%r11, %r12), %ymm14
	vmovapd			0(%r11, %r12, 2), %ymm15
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	32(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vmovapd			32(%r11), %ymm13
	vmovapd			32(%r11, %r12), %ymm14
	vmovapd			32(%r11, %r12, 2), %ymm15
	vbroadcastsd	8(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	40(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	72(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vmovapd			64(%r11), %ymm13
	vmovapd			64(%r11, %r12), %ymm14
	vmovapd			64(%r11, %r12, 2), %ymm15
	vbroadcastsd	16(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	48(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	80(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	112(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	vmovapd			96(%r11), %ymm13
	vmovapd			96(%r11, %r12), %ymm14
	vmovapd			96(%r11, %r12, 2), %ymm15
	vbroadcastsd	24(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	56(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	88(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	120(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	subl			$ 4, %r10d // k-4
	addq			$ 128, %r11 // A0+4*bs*sizeof(double)
	addq			%r14, %r13 // B+bs*sdb*sizeof(double)

3:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nn_rl_12x4_lib4)
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
	.macro INNER_EDGE_DTRMM_NN_RL_12X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nn_rl_12x4_vs_lib4)
#endif
	
	cmpl			$ 0, %r10d
	jle				3f // end

	cmpl			$ 0, %r15d
	jg				0f // offB>0

	// offB==0

	vmovapd			0(%r11), %ymm13
	vmovapd			0(%r11, %r12), %ymm14
	vmovapd			0(%r11, %r12, 2), %ymm15
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A0+1*bs*sizeof(double)
	addq			$ 8, %r13 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm13
	vmovapd			0(%r11, %r12), %ymm14
	vmovapd			0(%r11, %r12, 2), %ymm15
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	32(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A0+1*bs*sizeof(double)
	addq			$ 8, %r13 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm13
	vmovapd			0(%r11, %r12), %ymm14
	vmovapd			0(%r11, %r12, 2), %ymm15
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	32(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	64(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A0+1*bs*sizeof(double)
	addq			$ 8, %r13 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm13
	vmovapd			0(%r11, %r12), %ymm14
	vmovapd			0(%r11, %r12, 2), %ymm15
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	32(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	64(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	96(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

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

	vmovapd			0(%r11), %ymm13
	vmovapd			0(%r11, %r12), %ymm14
	vmovapd			0(%r11, %r12, 2), %ymm15
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A0+1*bs*sizeof(double)
	addq			$ 8, %r13 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm13
	vmovapd			0(%r11, %r12), %ymm14
	vmovapd			0(%r11, %r12, 2), %ymm15
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	32(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A0+1*bs*sizeof(double)
	addq			$ 8, %r13 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm13
	vmovapd			0(%r11, %r12), %ymm14
	vmovapd			0(%r11, %r12, 2), %ymm15
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	32(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	64(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

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

	vmovapd			0(%r11), %ymm13
	vmovapd			0(%r11, %r12), %ymm14
	vmovapd			0(%r11, %r12, 2), %ymm15
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A0+1*bs*sizeof(double)
	addq			$ 8, %r13 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm13
	vmovapd			0(%r11, %r12), %ymm14
	vmovapd			0(%r11, %r12, 2), %ymm15
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	32(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	subl			$ 1, %r10d // k-2
	addq			$ 32, %r11 // A0+1*bs*sizeof(double)
	addq			%r14, %r13
	subq			$ 24, %r13 // B+bs*sdb*sizeof(double)-(bs-1)*sizeof(double)

	vmovapd			0(%r11), %ymm13
	vmovapd			0(%r11, %r12), %ymm14
	vmovapd			0(%r11, %r12, 2), %ymm15
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	32(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	64(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A0+1*bs*sizeof(double)
	addq			$ 8, %r13 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm13
	vmovapd			0(%r11, %r12), %ymm14
	vmovapd			0(%r11, %r12, 2), %ymm15
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	32(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	64(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	96(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A0+1*bs*sizeof(double)
	addq			$ 8, %r13 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm13
	vmovapd			0(%r11, %r12), %ymm14
	vmovapd			0(%r11, %r12, 2), %ymm15
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	32(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	64(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	96(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A0+1*bs*sizeof(double)
	addq			$ 8, %r13 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm13
	vmovapd			0(%r11, %r12), %ymm14
	vmovapd			0(%r11, %r12, 2), %ymm15
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	32(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	64(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	96(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A0+1*bs*sizeof(double)
	addq			%r14, %r13
	subq			$ 24, %r13 // B+bs*sdb*sizeof(double)-(bs-1)*sizeof(double)

	jmp				3f

2:
	// offB==3

	addq			$ 24, %r13 // B+3*sizeof(double)

	vmovapd			0(%r11), %ymm13
	vmovapd			0(%r11, %r12), %ymm14
	vmovapd			0(%r11, %r12, 2), %ymm15
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A0+1*bs*sizeof(double)
	addq			%r14, %r13
	subq			$ 24, %r13 // B+bs*sdb*sizeof(double)-(bs-1)*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm13
	vmovapd			0(%r11, %r12), %ymm14
	vmovapd			0(%r11, %r12, 2), %ymm15
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	32(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A0+1*bs*sizeof(double)
	addq			$ 8, %r13 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm13
	vmovapd			0(%r11, %r12), %ymm14
	vmovapd			0(%r11, %r12, 2), %ymm15
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	32(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	64(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A0+1*bs*sizeof(double)
	addq			$ 8, %r13 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm13
	vmovapd			0(%r11, %r12), %ymm14
	vmovapd			0(%r11, %r12, 2), %ymm15
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	32(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	64(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	96(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A0+1*bs*sizeof(double)
	addq			$ 8, %r13 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm13
	vmovapd			0(%r11, %r12), %ymm14
	vmovapd			0(%r11, %r12, 2), %ymm15
	vbroadcastsd	0(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	32(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	64(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	96(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	subl			$ 1, %r10d // k-4
	addq			$ 32, %r11 // A0+1*bs*sizeof(double)
	addq			%r14, %r13
	subq			$ 24, %r13 // B+bs*sdb*sizeof(double)-(bs-1)*sizeof(double)

3:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nn_rl_12x4_vs_lib4)
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
	.macro INNER_EDGE_DTRMM_NT_RL_12X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nt_rl_12x4_lib4)
#endif
	
	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vmovapd			0(%r11, %r12), %ymm14 // A
	vmovapd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 1
	vmovapd			32(%r11), %ymm13 // A
	vmovapd			32(%r11, %r12), %ymm14 // A
	vmovapd			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	40(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	48(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	56(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 2
	vmovapd			64(%r11), %ymm13 // A
	vmovapd			64(%r11, %r12), %ymm14 // A
	vmovapd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	80(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	88(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 3
	vmovapd			96(%r11), %ymm13 // A
	vmovapd			96(%r11, %r12), %ymm14 // A
	vmovapd			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	120(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r13

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nt_rl_12x4_lib4)
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
	.macro INNER_EDGE_DTRMM_NT_RL_12X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nt_rl_12x4_vs_lib4)
#endif
	
	cmpl	$ 0, %r14d
	jle		0f // end

	cmpl	$ 4, %r14d
	jl		1f // end

	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vmovapd			0(%r11, %r12), %ymm14 // A
	vmovapd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 1
	vmovapd			32(%r11), %ymm13 // A
	vmovapd			32(%r11, %r12), %ymm14 // A
	vmovapd			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	40(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	48(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	56(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 2
	vmovapd			64(%r11), %ymm13 // A
	vmovapd			64(%r11, %r12), %ymm14 // A
	vmovapd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	80(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	88(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 3
	vmovapd			96(%r11), %ymm13 // A
	vmovapd			96(%r11, %r12), %ymm14 // A
	vmovapd			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	120(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

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
	vmovapd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	// unroll 1
	vmovapd			32(%r11), %ymm13 // A
	vmovapd			32(%r11, %r12), %ymm14 // A
	vmovapd			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	40(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	48(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	// unroll 2
	vmovapd			64(%r11), %ymm13 // A
	vmovapd			64(%r11, %r12), %ymm14 // A
	vmovapd			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	80(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

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
	vmovapd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	// unroll 1
	vmovapd			32(%r11), %ymm13 // A
	vmovapd			32(%r11, %r12), %ymm14 // A
	vmovapd			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	40(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

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
	vmovapd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

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

	FUN_END(inner_edge_dtrmm_nt_rl_12x4_vs_lib4)
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
	.macro INNER_EDGE_DTRMM_NT_RL_ONE_12X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nt_rl_one_12x4_lib4)
#endif
	
	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vmovapd			0(%r11, %r12), %ymm14 // A
	vmovapd			0(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm0, %ymm13, %ymm0
	vaddpd			%ymm4, %ymm14, %ymm4
	vaddpd			%ymm8, %ymm15, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 1
	vmovapd			32(%r11), %ymm13 // A
	vmovapd			32(%r11, %r12), %ymm14 // A
	vmovapd			32(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm1, %ymm13, %ymm1
	vaddpd			%ymm5, %ymm14, %ymm5
	vaddpd			%ymm9, %ymm15, %ymm9
	vbroadcastsd	48(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	56(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 2
	vmovapd			64(%r11), %ymm13 // A
	vmovapd			64(%r11, %r12), %ymm14 // A
	vmovapd			64(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm2, %ymm13, %ymm2
	vaddpd			%ymm6, %ymm14, %ymm6
	vaddpd			%ymm10, %ymm15, %ymm10
	vbroadcastsd	88(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 3
	vmovapd			96(%r11), %ymm13 // A
	vmovapd			96(%r11, %r12), %ymm14 // A
	vmovapd			96(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm3, %ymm13, %ymm3
	vaddpd			%ymm7, %ymm14, %ymm7
	vaddpd			%ymm11, %ymm15, %ymm11

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r13

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nt_rl_one_12x4_lib4)
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
	.macro INNER_EDGE_DTRMM_NT_RL_ONE_12X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nt_rl_one_12x4_vs_lib4)
#endif
	
	cmpl	$ 0, %r14d
	jle		0f // end

	cmpl	$ 4, %r14d
	jl		1f // end

	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vmovapd			0(%r11, %r12), %ymm14 // A
	vmovapd			0(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm0, %ymm13, %ymm0
	vaddpd			%ymm4, %ymm14, %ymm4
	vaddpd			%ymm8, %ymm15, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	24(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 1
	vmovapd			32(%r11), %ymm13 // A
	vmovapd			32(%r11, %r12), %ymm14 // A
	vmovapd			32(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm1, %ymm13, %ymm1
	vaddpd			%ymm5, %ymm14, %ymm5
	vaddpd			%ymm9, %ymm15, %ymm9
	vbroadcastsd	48(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10
	vbroadcastsd	56(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 2
	vmovapd			64(%r11), %ymm13 // A
	vmovapd			64(%r11, %r12), %ymm14 // A
	vmovapd			64(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm2, %ymm13, %ymm2
	vaddpd			%ymm6, %ymm14, %ymm6
	vaddpd			%ymm10, %ymm15, %ymm10
	vbroadcastsd	88(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 3
	vmovapd			96(%r11), %ymm13 // A
	vmovapd			96(%r11, %r12), %ymm14 // A
	vmovapd			96(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm3, %ymm13, %ymm3
	vaddpd			%ymm7, %ymm14, %ymm7
	vaddpd			%ymm11, %ymm15, %ymm11

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
	vmovapd			0(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm0, %ymm13, %ymm0
	vaddpd			%ymm4, %ymm14, %ymm4
	vaddpd			%ymm8, %ymm15, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9
	vbroadcastsd	16(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	// unroll 1
	vmovapd			32(%r11), %ymm13 // A
	vmovapd			32(%r11, %r12), %ymm14 // A
	vmovapd			32(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm1, %ymm13, %ymm1
	vaddpd			%ymm5, %ymm14, %ymm5
	vaddpd			%ymm9, %ymm15, %ymm9
	vbroadcastsd	48(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	// unroll 2
	vmovapd			64(%r11), %ymm13 // A
	vmovapd			64(%r11, %r12), %ymm14 // A
	vmovapd			64(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm2, %ymm13, %ymm2
	vaddpd			%ymm6, %ymm14, %ymm6
	vaddpd			%ymm10, %ymm15, %ymm10

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
	vmovapd			0(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm0, %ymm13, %ymm0
	vaddpd			%ymm4, %ymm14, %ymm4
	vaddpd			%ymm8, %ymm15, %ymm8
	vbroadcastsd	8(%r13), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	// unroll 1
	vmovapd			32(%r11), %ymm13 // A
	vmovapd			32(%r11, %r12), %ymm14 // A
	vmovapd			32(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm1, %ymm13, %ymm1
	vaddpd			%ymm5, %ymm14, %ymm5
	vaddpd			%ymm9, %ymm15, %ymm9

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
	vmovapd			0(%r11, %r12, 2), %ymm15 // A
	vaddpd			%ymm0, %ymm13, %ymm0
	vaddpd			%ymm4, %ymm14, %ymm4
	vaddpd			%ymm8, %ymm15, %ymm8

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

	FUN_END(inner_edge_dtrmm_nt_rl_one_12x4_vs_lib4)
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
	.macro INNER_BLEND_12X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_blend_12x4_lib4)
#endif
	
	vblendpd	$ 0xa, %ymm1, %ymm0, %ymm12
	vblendpd	$ 0x5, %ymm1, %ymm0, %ymm13
	vblendpd	$ 0xa, %ymm3, %ymm2, %ymm14
	vblendpd	$ 0x5, %ymm3, %ymm2, %ymm15

	vblendpd	$ 0xc, %ymm14, %ymm12, %ymm0
	vblendpd	$ 0x3, %ymm14, %ymm12, %ymm2
	vblendpd	$ 0xc, %ymm15, %ymm13, %ymm1
	vblendpd	$ 0x3, %ymm15, %ymm13, %ymm3


	vblendpd	$ 0xa, %ymm5, %ymm4, %ymm12
	vblendpd	$ 0x5, %ymm5, %ymm4, %ymm13
	vblendpd	$ 0xa, %ymm7, %ymm6, %ymm14
	vblendpd	$ 0x5, %ymm7, %ymm6, %ymm15

	vblendpd	$ 0xc, %ymm14, %ymm12, %ymm4
	vblendpd	$ 0x3, %ymm14, %ymm12, %ymm6
	vblendpd	$ 0xc, %ymm15, %ymm13, %ymm5
	vblendpd	$ 0x3, %ymm15, %ymm13, %ymm7


	vblendpd	$ 0xa, %ymm9, %ymm8, %ymm12
	vblendpd	$ 0x5, %ymm9, %ymm8, %ymm13
	vblendpd	$ 0xa, %ymm11, %ymm10, %ymm14
	vblendpd	$ 0x5, %ymm11, %ymm10, %ymm15

	vblendpd	$ 0xc, %ymm14, %ymm12, %ymm8
	vblendpd	$ 0x3, %ymm14, %ymm12, %ymm10
	vblendpd	$ 0xc, %ymm15, %ymm13, %ymm9
	vblendpd	$ 0x3, %ymm15, %ymm13, %ymm11

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_blend_12x4_lib4)
#endif





// common inner routine with file scope
//
// transpose and scale for generic alpha and beta
//
// input arguments:
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
	.macro INNER_TRAN_12X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_tran_12x4_lib4)
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

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_tran_12x4_lib4)
#endif





// common inner routine with file scope
//
// scale for alpha = 1.0 and beta
//
// input arguments:
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
// ymm8  <- [d80 d90 da0 db0]
// ymm9  <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 d63 db3]
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
// ymm8  <- [d80 d90 da0 db0]
// ymm9  <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 d63 db3]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M1B_12X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_scale_m1b_12x4_lib4)
#endif
	
	// beta
	vbroadcastsd	0(%r10), %ymm14

	vxorpd		%ymm15, %ymm15, %ymm15 // 0.0

	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	vmovapd		0(%r11), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm0
	vmovapd		32(%r11), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm1
	vmovapd		64(%r11), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm2
	vmovapd		96(%r11), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm3

	vmovapd		0(%r11, %r12), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm4
	vmovapd		32(%r11, %r12), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm5
	vmovapd		64(%r11, %r12), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm6
	vmovapd		96(%r11, %r12), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm7

	vmovapd		0(%r11, %r12, 2), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm8
	vmovapd		32(%r11, %r12, 2), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm9
	vmovapd		64(%r11, %r12, 2), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm10
	vmovapd		96(%r11, %r12, 2), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m1b_12x4_lib4)
#endif





// common inner routine with file scope
//
// scale for alpha = 1.0 and beta = 1.0
//
// input arguments:
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
// ymm11 <- [d83 d93 d63 db3]
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
// ymm8  <- [d80 d90 da0 db0]
// ymm9  <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 d63 db3]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M11_12X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_scale_m11_12x4_lib4)
#endif
	

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd		.LC05(%rip), %ymm14 // beta=1.0
#else
	vmovapd		LC05(%rip), %ymm14 // beta=1.0
#endif

	vmovapd		0(%r10), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm0
	vmovapd		32(%r10), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm1
	vmovapd		64(%r10), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm2
	vmovapd		96(%r10), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm3

	vmovapd		0(%r10, %r11), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm4
	vmovapd		32(%r10, %r11), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm5
	vmovapd		64(%r10, %r11), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm6
	vmovapd		96(%r10, %r11), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm7

	vmovapd		0(%r10, %r11, 2), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm8
	vmovapd		32(%r10, %r11, 2), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm9
	vmovapd		64(%r10, %r11, 2), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm10
	vmovapd		96(%r10, %r11, 2), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm11

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m11_12x4_lib4)
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
	vfmadd231pd	%ymm14, %ymm15, %ymm0
	vmovapd		32(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm1
	vmovapd		64(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm2
	vmovapd		96(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm3

	vmovapd		0(%r15), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm4
	vmovapd		32(%r15), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm5
	vmovapd		64(%r15), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm6
	vmovapd		96(%r15), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm7

	vmovapd		0(%r14), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm8
	vmovapd		32(%r14), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm9
	vmovapd		64(%r14), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm10
	vmovapd		96(%r14), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_12x4_lib4)
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
// ymm8  <- [d80 d90 da0 db0]
// ymm9  <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_12X4_GEN_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_12x4_gen_lib4)
#endif
	

	// alpha
	vbroadcastsd	0(%r10), %ymm15

	vmulpd			%ymm0, %ymm15, %ymm0
	vmulpd			%ymm1, %ymm15, %ymm1
	vmulpd			%ymm2, %ymm15, %ymm2
	vmulpd			%ymm3, %ymm15, %ymm3

	vmulpd			%ymm4, %ymm15, %ymm4
	vmulpd			%ymm5, %ymm15, %ymm5
	vmulpd			%ymm6, %ymm15, %ymm6
	vmulpd			%ymm7, %ymm15, %ymm7

	vmulpd			%ymm8, %ymm15, %ymm8
	vmulpd			%ymm9, %ymm15, %ymm9
	vmulpd			%ymm10, %ymm15, %ymm10
	vmulpd			%ymm11, %ymm15, %ymm11

	movq	%r13, %rax // C1 <- C0
	addq	%r14, %rax // C1 <- C0 + 4*sdc*sizeof(double)

	// beta
	vbroadcastsd	0(%r11), %ymm15

	vxorpd			%ymm14, %ymm14, %ymm14 // 0.0

	vucomisd		%xmm15, %xmm14 // beta==0.0 ?
	je			3f // end

	cmpl	$ 0, %r12d
	jg		0f

	// offset==0

	vmovapd			0(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm15, %ymm0
	vmovapd			32(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm15, %ymm1
	vmovapd			64(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm15, %ymm2
	vmovapd			96(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm15, %ymm3

	vmovapd			0(%r13, %r14), %ymm14
	vfmadd231pd		%ymm14, %ymm15, %ymm4
	vmovapd			32(%r13, %r14), %ymm14
	vfmadd231pd		%ymm14, %ymm15, %ymm5
	vmovapd			64(%r13, %r14), %ymm14
	vfmadd231pd		%ymm14, %ymm15, %ymm6
	vmovapd			96(%r13, %r14), %ymm14
	vfmadd231pd		%ymm14, %ymm15, %ymm7

	vmovapd			0(%r13, %r14, 2), %ymm14
	vfmadd231pd		%ymm14, %ymm15, %ymm8
	vmovapd			32(%r13, %r14, 2), %ymm14
	vfmadd231pd		%ymm14, %ymm15, %ymm9
	vmovapd			64(%r13, %r14, 2), %ymm14
	vfmadd231pd		%ymm14, %ymm15, %ymm10
	vmovapd			96(%r13, %r14, 2), %ymm14
	vfmadd231pd		%ymm14, %ymm15, %ymm11

	jmp		3f

0:

	cmpl	$ 1, %r12d
	jg		1f

	// offset==1

	vmovapd			0(%r13), %ymm12
	vmovapd			0(%r13, %r14), %ymm13
	vmovapd			0(%r13, %r14, 2), %ymm14
	vmovapd			0(%rax, %r14, 2), %ymm15
	vblendpd		$ 0x1, %ymm13, %ymm12, %ymm12
	vblendpd		$ 0x1, %ymm14, %ymm13, %ymm13
	vblendpd		$ 0x1, %ymm15, %ymm14, %ymm14
	vperm2f128		$ 0x01, %ymm12, %ymm12, %ymm15
	vshufpd			$ 0x5, %ymm15, %ymm12, %ymm12
	vperm2f128		$ 0x01, %ymm13, %ymm13, %ymm15
	vshufpd			$ 0x5, %ymm15, %ymm13, %ymm13
	vperm2f128		$ 0x01, %ymm14, %ymm14, %ymm15
	vshufpd			$ 0x5, %ymm15, %ymm14, %ymm14
	vbroadcastsd	0(%r11), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm0
	vfmadd231pd		%ymm13, %ymm15, %ymm4
	vfmadd231pd		%ymm14, %ymm15, %ymm8

	vmovapd			32(%r13), %ymm12
	vmovapd			32(%r13, %r14), %ymm13
	vmovapd			32(%r13, %r14, 2), %ymm14
	vmovapd			32(%rax, %r14, 2), %ymm15
	vblendpd		$ 0x1, %ymm13, %ymm12, %ymm12
	vblendpd		$ 0x1, %ymm14, %ymm13, %ymm13
	vblendpd		$ 0x1, %ymm15, %ymm14, %ymm14
	vperm2f128		$ 0x01, %ymm12, %ymm12, %ymm15
	vshufpd			$ 0x5, %ymm15, %ymm12, %ymm12
	vperm2f128		$ 0x01, %ymm13, %ymm13, %ymm15
	vshufpd			$ 0x5, %ymm15, %ymm13, %ymm13
	vperm2f128		$ 0x01, %ymm14, %ymm14, %ymm15
	vshufpd			$ 0x5, %ymm15, %ymm14, %ymm14
	vbroadcastsd	0(%r11), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm1
	vfmadd231pd		%ymm13, %ymm15, %ymm5
	vfmadd231pd		%ymm14, %ymm15, %ymm9

	vmovapd			64(%r13), %ymm12
	vmovapd			64(%r13, %r14), %ymm13
	vmovapd			64(%r13, %r14, 2), %ymm14
	vmovapd			64(%rax, %r14, 2), %ymm15
	vblendpd		$ 0x1, %ymm13, %ymm12, %ymm12
	vblendpd		$ 0x1, %ymm14, %ymm13, %ymm13
	vblendpd		$ 0x1, %ymm15, %ymm14, %ymm14
	vperm2f128		$ 0x01, %ymm12, %ymm12, %ymm15
	vshufpd			$ 0x5, %ymm15, %ymm12, %ymm12
	vperm2f128		$ 0x01, %ymm13, %ymm13, %ymm15
	vshufpd			$ 0x5, %ymm15, %ymm13, %ymm13
	vperm2f128		$ 0x01, %ymm14, %ymm14, %ymm15
	vshufpd			$ 0x5, %ymm15, %ymm14, %ymm14
	vbroadcastsd	0(%r11), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm2
	vfmadd231pd		%ymm13, %ymm15, %ymm6
	vfmadd231pd		%ymm14, %ymm15, %ymm10

	vmovapd			96(%r13), %ymm12
	vmovapd			96(%r13, %r14), %ymm13
	vmovapd			96(%r13, %r14, 2), %ymm14
	vmovapd			96(%rax, %r14, 2), %ymm15
	vblendpd		$ 0x1, %ymm13, %ymm12, %ymm12
	vblendpd		$ 0x1, %ymm14, %ymm13, %ymm13
	vblendpd		$ 0x1, %ymm15, %ymm14, %ymm14
	vperm2f128		$ 0x01, %ymm12, %ymm12, %ymm15
	vshufpd			$ 0x5, %ymm15, %ymm12, %ymm12
	vperm2f128		$ 0x01, %ymm13, %ymm13, %ymm15
	vshufpd			$ 0x5, %ymm15, %ymm13, %ymm13
	vperm2f128		$ 0x01, %ymm14, %ymm14, %ymm15
	vshufpd			$ 0x5, %ymm15, %ymm14, %ymm14
	vbroadcastsd	0(%r11), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm3
	vfmadd231pd		%ymm13, %ymm15, %ymm7
	vfmadd231pd		%ymm14, %ymm15, %ymm11

	jmp		3f

1:

	cmpl	$ 2, %r12d
	jg		2f

	// offset==2

	vmovapd			0(%r13), %ymm12
	vmovapd			0(%r13, %r14), %ymm13
	vmovapd			0(%r13, %r14, 2), %ymm14
	vmovapd			0(%rax, %r14, 2), %ymm15
	vblendpd		$ 0x3, %ymm13, %ymm12, %ymm12
	vblendpd		$ 0x3, %ymm14, %ymm13, %ymm13
	vblendpd		$ 0x3, %ymm15, %ymm14, %ymm14
	vperm2f128		$ 0x01, %ymm12, %ymm12, %ymm12
	vperm2f128		$ 0x01, %ymm13, %ymm13, %ymm13
	vperm2f128		$ 0x01, %ymm14, %ymm14, %ymm14
	vbroadcastsd	0(%r11), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm0
	vfmadd231pd		%ymm13, %ymm15, %ymm4
	vfmadd231pd		%ymm14, %ymm15, %ymm8

	vmovapd			32(%r13), %ymm12
	vmovapd			32(%r13, %r14), %ymm13
	vmovapd			32(%r13, %r14, 2), %ymm14
	vmovapd			32(%rax, %r14, 2), %ymm15
	vblendpd		$ 0x3, %ymm13, %ymm12, %ymm12
	vblendpd		$ 0x3, %ymm14, %ymm13, %ymm13
	vblendpd		$ 0x3, %ymm15, %ymm14, %ymm14
	vperm2f128		$ 0x01, %ymm12, %ymm12, %ymm12
	vperm2f128		$ 0x01, %ymm13, %ymm13, %ymm13
	vperm2f128		$ 0x01, %ymm14, %ymm14, %ymm14
	vbroadcastsd	0(%r11), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm1
	vfmadd231pd		%ymm13, %ymm15, %ymm5
	vfmadd231pd		%ymm14, %ymm15, %ymm9

	vmovapd			64(%r13), %ymm12
	vmovapd			64(%r13, %r14), %ymm13
	vmovapd			64(%r13, %r14, 2), %ymm14
	vmovapd			64(%rax, %r14, 2), %ymm15
	vblendpd		$ 0x3, %ymm13, %ymm12, %ymm12
	vblendpd		$ 0x3, %ymm14, %ymm13, %ymm13
	vblendpd		$ 0x3, %ymm15, %ymm14, %ymm14
	vperm2f128		$ 0x01, %ymm12, %ymm12, %ymm12
	vperm2f128		$ 0x01, %ymm13, %ymm13, %ymm13
	vperm2f128		$ 0x01, %ymm14, %ymm14, %ymm14
	vbroadcastsd	0(%r11), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm2
	vfmadd231pd		%ymm13, %ymm15, %ymm6
	vfmadd231pd		%ymm14, %ymm15, %ymm10

	vmovapd			96(%r13), %ymm12
	vmovapd			96(%r13, %r14), %ymm13
	vmovapd			96(%r13, %r14, 2), %ymm14
	vmovapd			96(%rax, %r14, 2), %ymm15
	vblendpd		$ 0x3, %ymm13, %ymm12, %ymm12
	vblendpd		$ 0x3, %ymm14, %ymm13, %ymm13
	vblendpd		$ 0x3, %ymm15, %ymm14, %ymm14
	vperm2f128		$ 0x01, %ymm12, %ymm12, %ymm12
	vperm2f128		$ 0x01, %ymm13, %ymm13, %ymm13
	vperm2f128		$ 0x01, %ymm14, %ymm14, %ymm14
	vbroadcastsd	0(%r11), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm3
	vfmadd231pd		%ymm13, %ymm15, %ymm7
	vfmadd231pd		%ymm14, %ymm15, %ymm11


	jmp		3f

2:

	// offset==3

	vmovapd			0(%r13), %ymm12
	vmovapd			0(%r13, %r14), %ymm13
	vmovapd			0(%r13, %r14, 2), %ymm14
	vmovapd			0(%rax, %r14, 2), %ymm15
	vblendpd		$ 0x7, %ymm13, %ymm12, %ymm12
	vblendpd		$ 0x7, %ymm14, %ymm13, %ymm13
	vblendpd		$ 0x7, %ymm15, %ymm14, %ymm14
	vperm2f128		$ 0x01, %ymm12, %ymm12, %ymm15
	vshufpd			$ 0x5, %ymm12, %ymm15, %ymm12
	vperm2f128		$ 0x01, %ymm13, %ymm13, %ymm15
	vshufpd			$ 0x5, %ymm13, %ymm15, %ymm13
	vperm2f128		$ 0x01, %ymm14, %ymm14, %ymm15
	vshufpd			$ 0x5, %ymm14, %ymm15, %ymm14
	vbroadcastsd	0(%r11), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm0
	vfmadd231pd		%ymm13, %ymm15, %ymm4
	vfmadd231pd		%ymm14, %ymm15, %ymm8

	vmovapd			32(%r13), %ymm12
	vmovapd			32(%r13, %r14), %ymm13
	vmovapd			32(%r13, %r14, 2), %ymm14
	vmovapd			32(%rax, %r14, 2), %ymm15
	vblendpd		$ 0x7, %ymm13, %ymm12, %ymm12
	vblendpd		$ 0x7, %ymm14, %ymm13, %ymm13
	vblendpd		$ 0x7, %ymm15, %ymm14, %ymm14
	vperm2f128		$ 0x01, %ymm12, %ymm12, %ymm15
	vshufpd			$ 0x5, %ymm12, %ymm15, %ymm12
	vperm2f128		$ 0x01, %ymm13, %ymm13, %ymm15
	vshufpd			$ 0x5, %ymm13, %ymm15, %ymm13
	vperm2f128		$ 0x01, %ymm14, %ymm14, %ymm15
	vshufpd			$ 0x5, %ymm14, %ymm15, %ymm14
	vbroadcastsd	0(%r11), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm1
	vfmadd231pd		%ymm13, %ymm15, %ymm5
	vfmadd231pd		%ymm14, %ymm15, %ymm9

	vmovapd			64(%r13), %ymm12
	vmovapd			64(%r13, %r14), %ymm13
	vmovapd			64(%r13, %r14, 2), %ymm14
	vmovapd			64(%rax, %r14, 2), %ymm15
	vblendpd		$ 0x7, %ymm13, %ymm12, %ymm12
	vblendpd		$ 0x7, %ymm14, %ymm13, %ymm13
	vblendpd		$ 0x7, %ymm15, %ymm14, %ymm14
	vperm2f128		$ 0x01, %ymm12, %ymm12, %ymm15
	vshufpd			$ 0x5, %ymm12, %ymm15, %ymm12
	vperm2f128		$ 0x01, %ymm13, %ymm13, %ymm15
	vshufpd			$ 0x5, %ymm13, %ymm15, %ymm13
	vperm2f128		$ 0x01, %ymm14, %ymm14, %ymm15
	vshufpd			$ 0x5, %ymm14, %ymm15, %ymm14
	vbroadcastsd	0(%r11), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm2
	vfmadd231pd		%ymm13, %ymm15, %ymm6
	vfmadd231pd		%ymm14, %ymm15, %ymm10

	vmovapd			96(%r13), %ymm12
	vmovapd			96(%r13, %r14), %ymm13
	vmovapd			96(%r13, %r14, 2), %ymm14
	vmovapd			96(%rax, %r14, 2), %ymm15
	vblendpd		$ 0x7, %ymm13, %ymm12, %ymm12
	vblendpd		$ 0x7, %ymm14, %ymm13, %ymm13
	vblendpd		$ 0x7, %ymm15, %ymm14, %ymm14
	vperm2f128		$ 0x01, %ymm12, %ymm12, %ymm15
	vshufpd			$ 0x5, %ymm12, %ymm15, %ymm12
	vperm2f128		$ 0x01, %ymm13, %ymm13, %ymm15
	vshufpd			$ 0x5, %ymm13, %ymm15, %ymm13
	vperm2f128		$ 0x01, %ymm14, %ymm14, %ymm15
	vshufpd			$ 0x5, %ymm14, %ymm15, %ymm14
	vbroadcastsd	0(%r11), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm3
	vfmadd231pd		%ymm13, %ymm15, %ymm7
	vfmadd231pd		%ymm14, %ymm15, %ymm11


3:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_12x4_gen_lib4)
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
// r15  <- dirty
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
	.macro INNER_SCALE_AB_4X12_GEN_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_4x12_gen_lib4)
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
	vmulpd		%ymm8, %ymm15, %ymm8
	vmulpd		%ymm9, %ymm15, %ymm9
	vmulpd		%ymm10, %ymm15, %ymm10
	vmulpd		%ymm11, %ymm15, %ymm11

	// beta
	vbroadcastsd	0(%r11), %ymm15

	vxorpd		%ymm14, %ymm14, %ymm14 // 0.0

	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			3f // end

	cmpl	$ 0, %r12d
	jg		0f

	// offset==0

	vmovapd		0(%r13), %ymm12
	vfmadd231pd	%ymm12, %ymm15, %ymm0
	vmovapd		32(%r13), %ymm12
	vfmadd231pd	%ymm12, %ymm15, %ymm1
	vmovapd		64(%r13), %ymm12
	vfmadd231pd	%ymm12, %ymm15, %ymm2
	vmovapd		96(%r13), %ymm12
	vfmadd231pd	%ymm12, %ymm15, %ymm3
	vmovapd		128(%r13), %ymm12
	vfmadd231pd	%ymm12, %ymm15, %ymm4
	vmovapd		160(%r13), %ymm12
	vfmadd231pd	%ymm12, %ymm15, %ymm5
	vmovapd		192(%r13), %ymm12
	vfmadd231pd	%ymm12, %ymm15, %ymm6
	vmovapd		224(%r13), %ymm12
	vfmadd231pd	%ymm12, %ymm15, %ymm7
	vmovapd		256(%r13), %ymm12
	vfmadd231pd	%ymm12, %ymm15, %ymm8
	vmovapd		288(%r13), %ymm12
	vfmadd231pd	%ymm12, %ymm15, %ymm9
	vmovapd		320(%r13), %ymm12
	vfmadd231pd	%ymm12, %ymm15, %ymm10
	vmovapd		352(%r13), %ymm12
	vfmadd231pd	%ymm12, %ymm15, %ymm11

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
	vfmadd231pd	%ymm12, %ymm15, %ymm0
	vfmadd231pd	%ymm13, %ymm15, %ymm1

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
	vfmadd231pd	%ymm12, %ymm15, %ymm2
	vfmadd231pd	%ymm13, %ymm15, %ymm3

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
	vfmadd231pd	%ymm12, %ymm15, %ymm4
	vfmadd231pd	%ymm13, %ymm15, %ymm5

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
	vfmadd231pd	%ymm12, %ymm15, %ymm6
	vfmadd231pd	%ymm13, %ymm15, %ymm7

	vmovapd		256(%r13), %ymm12
	vmovapd		256(%r15), %ymm14
	vblendpd	$ 0x1, %ymm14, %ymm12, %ymm12
	vmovapd		288(%r13), %ymm13
	vmovapd		288(%r15), %ymm14
	vblendpd	$ 0x1, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm14
	vshufpd		$ 0x5, %ymm14, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm14
	vshufpd		$ 0x5, %ymm14, %ymm13, %ymm13
	vfmadd231pd	%ymm12, %ymm15, %ymm8
	vfmadd231pd	%ymm13, %ymm15, %ymm9

	vmovapd		320(%r13), %ymm12
	vmovapd		320(%r15), %ymm14
	vblendpd	$ 0x1, %ymm14, %ymm12, %ymm12
	vmovapd		352(%r13), %ymm13
	vmovapd		352(%r15), %ymm14
	vblendpd	$ 0x1, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm14
	vshufpd		$ 0x5, %ymm14, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm14
	vshufpd		$ 0x5, %ymm14, %ymm13, %ymm13
	vfmadd231pd	%ymm12, %ymm15, %ymm10
	vfmadd231pd	%ymm13, %ymm15, %ymm11

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
	vfmadd231pd	%ymm12, %ymm15, %ymm0
	vfmadd231pd	%ymm13, %ymm15, %ymm1

	vmovapd		64(%r13), %ymm12
	vmovapd		64(%r15), %ymm14
	vblendpd	$ 0x3, %ymm14, %ymm12, %ymm12
	vmovapd		96(%r13), %ymm13
	vmovapd		96(%r15), %ymm14
	vblendpd	$ 0x3, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm13
	vfmadd231pd	%ymm12, %ymm15, %ymm2
	vfmadd231pd	%ymm13, %ymm15, %ymm3

	vmovapd		128(%r13), %ymm12
	vmovapd		128(%r15), %ymm14
	vblendpd	$ 0x3, %ymm14, %ymm12, %ymm12
	vmovapd		160(%r13), %ymm13
	vmovapd		160(%r15), %ymm14
	vblendpd	$ 0x3, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm13
	vfmadd231pd	%ymm12, %ymm15, %ymm4
	vfmadd231pd	%ymm13, %ymm15, %ymm5

	vmovapd		192(%r13), %ymm12
	vmovapd		192(%r15), %ymm14
	vblendpd	$ 0x3, %ymm14, %ymm12, %ymm12
	vmovapd		224(%r13), %ymm13
	vmovapd		224(%r15), %ymm14
	vblendpd	$ 0x3, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm13
	vfmadd231pd	%ymm12, %ymm15, %ymm6
	vfmadd231pd	%ymm13, %ymm15, %ymm7

	vmovapd		256(%r13), %ymm12
	vmovapd		256(%r15), %ymm14
	vblendpd	$ 0x3, %ymm14, %ymm12, %ymm12
	vmovapd		288(%r13), %ymm13
	vmovapd		288(%r15), %ymm14
	vblendpd	$ 0x3, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm13
	vfmadd231pd	%ymm12, %ymm15, %ymm8
	vfmadd231pd	%ymm13, %ymm15, %ymm9

	vmovapd		320(%r13), %ymm12
	vmovapd		320(%r15), %ymm14
	vblendpd	$ 0x3, %ymm14, %ymm12, %ymm12
	vmovapd		352(%r13), %ymm13
	vmovapd		352(%r15), %ymm14
	vblendpd	$ 0x3, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm13
	vfmadd231pd	%ymm12, %ymm15, %ymm10
	vfmadd231pd	%ymm13, %ymm15, %ymm11

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
	vfmadd231pd	%ymm12, %ymm15, %ymm0
	vfmadd231pd	%ymm13, %ymm15, %ymm1

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
	vfmadd231pd	%ymm12, %ymm15, %ymm2
	vfmadd231pd	%ymm13, %ymm15, %ymm3

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
	vfmadd231pd	%ymm12, %ymm15, %ymm4
	vfmadd231pd	%ymm13, %ymm15, %ymm5

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
	vfmadd231pd	%ymm12, %ymm15, %ymm6
	vfmadd231pd	%ymm13, %ymm15, %ymm7

	vmovapd		256(%r13), %ymm12
	vmovapd		256(%r15), %ymm14
	vblendpd	$ 0x7, %ymm14, %ymm12, %ymm12
	vmovapd		288(%r13), %ymm13
	vmovapd		288(%r15), %ymm14
	vblendpd	$ 0x7, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm14
	vshufpd		$ 0x5, %ymm12, %ymm14, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm14
	vshufpd		$ 0x5, %ymm13, %ymm14, %ymm13
	vfmadd231pd	%ymm12, %ymm15, %ymm8
	vfmadd231pd	%ymm13, %ymm15, %ymm9

	vmovapd		320(%r13), %ymm12
	vmovapd		320(%r15), %ymm14
	vblendpd	$ 0x7, %ymm14, %ymm12, %ymm12
	vmovapd		352(%r13), %ymm13
	vmovapd		352(%r15), %ymm14
	vblendpd	$ 0x7, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm14
	vshufpd		$ 0x5, %ymm12, %ymm14, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm14
	vshufpd		$ 0x5, %ymm13, %ymm14, %ymm13
	vfmadd231pd	%ymm12, %ymm15, %ymm10
	vfmadd231pd	%ymm13, %ymm15, %ymm11

3:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_4x12_gen_lib4)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta=0
//
// input arguments:
// r10   <- &alpha
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
	.macro INNER_SCALE_A0_12X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_scale_a0_12x4_lib4)
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

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_a0_12x4_lib4)
#endif





// common inner routine with file scope
//
// blend and scale for generic alpha and beta
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
	.macro INNER_BLEND_SCALE_AB_12X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_blend_scale_ab_12x4_lib4)
#endif
		

	vblendpd	$ 0xa, %ymm1, %ymm0, %ymm12
	vblendpd	$ 0x5, %ymm1, %ymm0, %ymm13
	vblendpd	$ 0xa, %ymm3, %ymm2, %ymm14
	vblendpd	$ 0x5, %ymm3, %ymm2, %ymm15

	vblendpd	$ 0xc, %ymm14, %ymm12, %ymm0
	vblendpd	$ 0x3, %ymm14, %ymm12, %ymm2
	vblendpd	$ 0xc, %ymm15, %ymm13, %ymm1
	vblendpd	$ 0x3, %ymm15, %ymm13, %ymm3

	vbroadcastsd 0(%r10), %ymm15 // alpha

	vmulpd		%ymm0, %ymm15, %ymm0
	vmulpd		%ymm1, %ymm15, %ymm1
	vmulpd		%ymm2, %ymm15, %ymm2
	vmulpd		%ymm3, %ymm15, %ymm3

	vblendpd	$ 0xa, %ymm5, %ymm4, %ymm12
	vblendpd	$ 0x5, %ymm5, %ymm4, %ymm13
	vblendpd	$ 0xa, %ymm7, %ymm6, %ymm14
	vblendpd	$ 0x5, %ymm7, %ymm6, %ymm15

	vblendpd	$ 0xc, %ymm14, %ymm12, %ymm4
	vblendpd	$ 0x3, %ymm14, %ymm12, %ymm6
	vblendpd	$ 0xc, %ymm15, %ymm13, %ymm5
	vblendpd	$ 0x3, %ymm15, %ymm13, %ymm7

	vbroadcastsd 0(%r10), %ymm15 // alpha

	vmulpd		%ymm4, %ymm15, %ymm4
	vmulpd		%ymm5, %ymm15, %ymm5
	vmulpd		%ymm6, %ymm15, %ymm6
	vmulpd		%ymm7, %ymm15, %ymm7

	vblendpd	$ 0xa, %ymm9, %ymm8, %ymm12
	vblendpd	$ 0x5, %ymm9, %ymm8, %ymm13
	vblendpd	$ 0xa, %ymm11, %ymm10, %ymm14
	vblendpd	$ 0x5, %ymm11, %ymm10, %ymm15

	vblendpd	$ 0xc, %ymm14, %ymm12, %ymm8
	vblendpd	$ 0x3, %ymm14, %ymm12, %ymm10
	vblendpd	$ 0xc, %ymm15, %ymm13, %ymm9
	vblendpd	$ 0x3, %ymm15, %ymm13, %ymm11

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

	vmovapd		0(%r12, %r13), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm4
	vmovapd		32(%r12, %r13), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm5
	vmovapd		64(%r12, %r13), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm6
	vmovapd		96(%r12, %r13), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm7

	vmovapd		0(%r12, %r13, 2), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm8
	vmovapd		32(%r12, %r13, 2), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm9
	vmovapd		64(%r12, %r13, 2), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm10
	vmovapd		96(%r12, %r13, 2), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_blend_scale_ab_12x4_lib4)
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
	.macro INNER_SCALE_AB_4X12_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_4x12_lib4)
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
	vmulpd		%ymm8, %ymm15, %ymm8
	vmulpd		%ymm9, %ymm15, %ymm9
	vmulpd		%ymm10, %ymm15, %ymm10
	vmulpd		%ymm11, %ymm15, %ymm11

	// beta
	vbroadcastsd	0(%r11), %ymm14

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
	vmovapd		256(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm8
	vmovapd		288(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm9
	vmovapd		320(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm10
	vmovapd		352(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_4x12_lib4)
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
	.macro INNER_TRAN_SCALE_AB_4X12_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_tran_scale_ab_4x12_lib4)
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

	vmovapd		256(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm8
	vmovapd		288(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm9
	vmovapd		320(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm10
	vmovapd		352(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_tran_scale_ab_4x12_lib4)
#endif





// common inner routine with file scope
//
// blender for alpha = 1.0 and beta = 1.0
//
// input arguments:
// r10   <- C
// r11   <- 4*sdc*sizeof(double)
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
// r10   <- C
// r11   <- 4*sdc*sizeof(double)
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
	.macro INNER_BLEND_SCALE_M11_12X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_blend_scale_m11_12x4_lib4)
#endif
	
	vblendpd	$ 0xa, %ymm1, %ymm0, %ymm12
	vblendpd	$ 0x5, %ymm1, %ymm0, %ymm13
	vblendpd	$ 0xa, %ymm3, %ymm2, %ymm14
	vblendpd	$ 0x5, %ymm3, %ymm2, %ymm15

	vblendpd	$ 0xc, %ymm14, %ymm12, %ymm0
	vblendpd	$ 0x3, %ymm14, %ymm12, %ymm2
	vblendpd	$ 0xc, %ymm15, %ymm13, %ymm1
	vblendpd	$ 0x3, %ymm15, %ymm13, %ymm3

	vblendpd	$ 0xa, %ymm5, %ymm4, %ymm12
	vblendpd	$ 0x5, %ymm5, %ymm4, %ymm13
	vblendpd	$ 0xa, %ymm7, %ymm6, %ymm14
	vblendpd	$ 0x5, %ymm7, %ymm6, %ymm15

	vblendpd	$ 0xc, %ymm14, %ymm12, %ymm4
	vblendpd	$ 0x3, %ymm14, %ymm12, %ymm6
	vblendpd	$ 0xc, %ymm15, %ymm13, %ymm5
	vblendpd	$ 0x3, %ymm15, %ymm13, %ymm7

	vblendpd	$ 0xa, %ymm9, %ymm8, %ymm12
	vblendpd	$ 0x5, %ymm9, %ymm8, %ymm13
	vblendpd	$ 0xa, %ymm11, %ymm10, %ymm14
	vblendpd	$ 0x5, %ymm11, %ymm10, %ymm15

	vblendpd	$ 0xc, %ymm14, %ymm12, %ymm8
	vblendpd	$ 0x3, %ymm14, %ymm12, %ymm10
	vblendpd	$ 0xc, %ymm15, %ymm13, %ymm9
	vblendpd	$ 0x3, %ymm15, %ymm13, %ymm11

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd		.LC05(%rip), %ymm14 // beta=1.0
#else
	vmovapd		LC05(%rip), %ymm14 // beta=1.0
#endif

	vmovapd		0(%r10), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm0
	vmovapd		32(%r10), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm1
	vmovapd		64(%r10), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm2
	vmovapd		96(%r10), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm3

	vmovapd		0(%r10, %r11), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm4
	vmovapd		32(%r10, %r11), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm5
	vmovapd		64(%r10, %r11), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm6
	vmovapd		96(%r10, %r11), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm7

	vmovapd		0(%r10, %r11, 2), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm8
	vmovapd		32(%r10, %r11, 2), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm9
	vmovapd		64(%r10, %r11, 2), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm10
	vmovapd		96(%r10, %r11, 2), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm11

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_blend_scale_m11_12x4_lib4)
#endif





// common inner routine with file scope
//
// transpose and scale for alpha=1.0 and beta=1.0
//
// input arguments:
// r10   <- C
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
	.macro INNER_TRAN_SCALE_M11_4X12_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_tran_scale_m11_4x12_lib4)
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
	vmovapd		.LC05(%rip), %ymm14 // beta=1.0
#else
	vmovapd		LC05(%rip), %ymm14 // beta=1.0
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

	vmovapd		256(%r10), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm8
	vmovapd		288(%r10), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm9
	vmovapd		320(%r10), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm10
	vmovapd		352(%r10), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_tran_scale_m11_4x12_lib4)
#endif





// common inner routine with file scope
//
// cholesky factorization 
//
// input arguments:
// r10   <- inv_diag_E
// r11d  <- kn
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
// ymm8  <- [d80 d91 da2 db3]
// ymm9  <- [d81 d90 da3 db2]
// ymm10 <- [d83 d92 da1 db0]
// ymm11 <- [d82 d93 da0 db1]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DPOTRF_12X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dpotrf_12x4_vs_lib4)
#endif
	
	vxorpd			%ymm15, %ymm15, %ymm15 // 0.0
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC05(%rip), %xmm14 // 1.0
#elif defined(OS_MAC)
	vmovsd			LC05(%rip), %xmm14 // 1.0
#endif

	vmovsd			%xmm0, %xmm0, %xmm13
	vucomisd		%xmm15, %xmm13 // d_00 > 0.0 ?
	jbe				1f
	vsqrtsd			%xmm13, %xmm13, %xmm13
	vdivsd			%xmm13, %xmm14, %xmm13
2:
	vmovsd			%xmm13, 0(%r10)
//	vmovddup		%xmm13, %xmm13
//	vperm2f128		$ 0x00, %ymm13, %ymm13, %ymm13
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmulpd			%ymm0, %ymm13, %ymm0
	vmulpd			%ymm4, %ymm13, %ymm4
	vmulpd			%ymm8, %ymm13, %ymm8
	cmpl			$ 2, %r11d
	jl				0f // ret
//	vperm2f128		$ 0x00, %ymm0, %ymm0, %ymm12
//	vpermilpd		$ 0xf, %ymm12, %ymm13
	vpermpd			$ 0x55, %ymm0, %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm1
	vfnmadd231pd	%ymm4, %ymm13, %ymm5
	vfnmadd231pd	%ymm8, %ymm13, %ymm9
	vperm2f128		$ 0x11, %ymm0, %ymm0, %ymm12
	vpermilpd		$ 0x0, %ymm12, %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm2
	vfnmadd231pd	%ymm4, %ymm13, %ymm6
	vfnmadd231pd	%ymm8, %ymm13, %ymm10
	vpermilpd		$ 0xf, %ymm12, %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm3
	vfnmadd231pd	%ymm4, %ymm13, %ymm7
	vfnmadd231pd	%ymm8, %ymm13, %ymm11

	vpermilpd		$ 0x3, %xmm1, %xmm13
	vucomisd		%xmm15, %xmm13 // d_11 > 0.0 ?
	jbe				3f
	vsqrtsd			%xmm13, %xmm13, %xmm13
	vdivsd			%xmm13, %xmm14, %xmm13
4:
	vmovsd			%xmm13, 8(%r10)
//	vmovddup		%xmm13, %xmm13
//	vperm2f128		$ 0x00, %ymm13, %ymm13, %ymm13
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmulpd			%ymm1, %ymm13, %ymm1
	vmulpd			%ymm5, %ymm13, %ymm5
	vmulpd			%ymm9, %ymm13, %ymm9
	cmpl			$ 3, %r11d
	jl				0f // ret
	vperm2f128		$ 0x11, %ymm1, %ymm1, %ymm12
	vpermilpd		$ 0x0, %ymm12, %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm2
	vfnmadd231pd	%ymm5, %ymm13, %ymm6
	vfnmadd231pd	%ymm9, %ymm13, %ymm10
	vpermilpd		$ 0xf, %ymm12, %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm3
	vfnmadd231pd	%ymm5, %ymm13, %ymm7
	vfnmadd231pd	%ymm9, %ymm13, %ymm11

	vextractf128	$ 0x1, %ymm2, %xmm13
	vucomisd		%xmm15, %xmm13 // d_22 > 0.0 ?
	jbe				5f
	vsqrtsd			%xmm13, %xmm13, %xmm13
	vdivsd			%xmm13, %xmm14, %xmm13
6:
	vmovsd			%xmm13, 16(%r10)
//	vmovddup		%xmm13, %xmm13
//	vperm2f128		$ 0x00, %ymm13, %ymm13, %ymm13
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmulpd			%ymm2, %ymm13, %ymm2
	vmulpd			%ymm6, %ymm13, %ymm6
	vmulpd			%ymm10, %ymm13, %ymm10
	cmpl			$ 4, %r11d
	jl				0f // ret
//	vperm2f128		$ 0x11, %ymm2, %ymm2, %ymm12
//	vpermilpd		$ 0xf, %ymm12, %ymm13
	vpermpd			$ 0xff, %ymm2, %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm3
	vfnmadd231pd	%ymm6, %ymm13, %ymm7
	vfnmadd231pd	%ymm10, %ymm13, %ymm11

//	vextractf128	$ 0x1, %ymm3, %xmm13
//	vpermilpd		$ 0x3, %xmm13, %xmm13
	vpermpd			$ 0xff, %ymm3, %ymm13
	vucomisd		%xmm15, %xmm13 // d_33 > 0.0 ?
	jbe				7f
	vsqrtsd			%xmm13, %xmm13, %xmm13
	vdivsd			%xmm13, %xmm14, %xmm13
8:
	vmovsd			%xmm13, 24(%r10)
//	vmovddup		%xmm13, %xmm13
//	vperm2f128		$ 0x00, %ymm13, %ymm13, %ymm13
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmulpd			%ymm3, %ymm13, %ymm3
	vmulpd			%ymm7, %ymm13, %ymm7
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

0:
	#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dpotrf_12x4_vs_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution for cholesky factorization 
//
// input arguments:
// r10   <- E
// r11   <- inv_diag_E
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
// ymm11 <- [d82 d93 d60 db1]
// ymm12 <- dirty
// ymm13 <- dirty
//
// output arguments:
// r10   <- E
// r11   <- inv_diag_E
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm4  <- [d40 d51 d62 d73]
// ymm5  <- [d41 d50 d63 d72]
// ymm6  <- [d43 d52 d61 d70]
// ymm7  <- [d42 d53 d60 d71]
// ymm8  <- [d80 d91 da2 db3]
// ymm9  <- [d81 d90 da3 db2]
// ymm10 <- [d83 d92 da1 db0]
// ymm11 <- [d82 d93 d60 db1]
// ymm12 <- dirty
// ymm13 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RLT_INV_12X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rlt_inv_12x4_lib4)
#endif
	
	vbroadcastsd	0(%r11), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm0
	vmulpd			%ymm4, %ymm13, %ymm4
	vmulpd			%ymm8, %ymm13, %ymm8
	vbroadcastsd	8(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm1
	vfnmadd231pd	%ymm4, %ymm13, %ymm5
	vfnmadd231pd	%ymm8, %ymm13, %ymm9
	vbroadcastsd	16(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm2
	vfnmadd231pd	%ymm4, %ymm13, %ymm6
	vfnmadd231pd	%ymm8, %ymm13, %ymm10
	vbroadcastsd	24(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm3
	vfnmadd231pd	%ymm4, %ymm13, %ymm7
	vfnmadd231pd	%ymm8, %ymm13, %ymm11

	vbroadcastsd	8(%r11), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm1
	vmulpd			%ymm5, %ymm13, %ymm5
	vmulpd			%ymm9, %ymm13, %ymm9
	vbroadcastsd	48(%r10), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm2
	vfnmadd231pd	%ymm5, %ymm13, %ymm6
	vfnmadd231pd	%ymm9, %ymm13, %ymm10
	vbroadcastsd	56(%r10), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm3
	vfnmadd231pd	%ymm5, %ymm13, %ymm7
	vfnmadd231pd	%ymm9, %ymm13, %ymm11

	vbroadcastsd	16(%r11), %ymm13
	vmulpd			%ymm2, %ymm13, %ymm2
	vmulpd			%ymm6, %ymm13, %ymm6
	vmulpd			%ymm10, %ymm13, %ymm10
	vbroadcastsd	88(%r10), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm3
	vfnmadd231pd	%ymm6, %ymm13, %ymm7
	vfnmadd231pd	%ymm10, %ymm13, %ymm11

	vbroadcastsd	24(%r11), %ymm13
	vmulpd			%ymm3, %ymm13, %ymm3
	vmulpd			%ymm7, %ymm13, %ymm7
	vmulpd			%ymm11, %ymm13, %ymm11

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rlt_inv_12x4_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution for cholesky factorization 
//
// input arguments:
// r10  <- D
// r11  <- sdd
// r12  <- inv_diag_D
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
// r11  <- sdd
// r12  <- inv_diag_D
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
	.macro INNER_EDGE_DTRSM_RLT_INV_4X12_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rlt_inv_4x12_lib4)
#endif
	
	vbroadcastsd	0(%r12), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm0
	vbroadcastsd	8(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm1
	vbroadcastsd	16(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm2
	vbroadcastsd	24(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm3
	vbroadcastsd	0(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm4
	vbroadcastsd	8(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm5
	vbroadcastsd	16(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm6
	vbroadcastsd	24(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm7
	vbroadcastsd	0(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm8
	vbroadcastsd	8(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm9
	vbroadcastsd	16(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm10
	vbroadcastsd	24(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm11

	vbroadcastsd	8(%r12), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm1
	vbroadcastsd	48(%r10), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm2
	vbroadcastsd	56(%r10), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm3
	vbroadcastsd	32(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm4
	vbroadcastsd	40(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm5
	vbroadcastsd	48(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm6
	vbroadcastsd	56(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm7
	vbroadcastsd	32(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm8
	vbroadcastsd	40(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm9
	vbroadcastsd	48(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm10
	vbroadcastsd	56(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm11

	vbroadcastsd	16(%r12), %ymm13
	vmulpd			%ymm2, %ymm13, %ymm2
	vbroadcastsd	88(%r10), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm3
	vbroadcastsd	64(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm4
	vbroadcastsd	72(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm5
	vbroadcastsd	80(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm6
	vbroadcastsd	88(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm7
	vbroadcastsd	64(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm8
	vbroadcastsd	72(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm9
	vbroadcastsd	80(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm10
	vbroadcastsd	88(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm11

	vbroadcastsd	24(%r12), %ymm13
	vmulpd			%ymm3, %ymm13, %ymm3
	vbroadcastsd	96(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm4
	vbroadcastsd	104(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm5
	vbroadcastsd	112(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm6
	vbroadcastsd	120(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm7
	vbroadcastsd	96(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm8
	vbroadcastsd	104(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm9
	vbroadcastsd	112(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm10
	vbroadcastsd	120(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm11

	addq	$ 128, %r10

	vbroadcastsd	32(%r12), %ymm13
	vmulpd			%ymm4, %ymm13, %ymm4
	vbroadcastsd	8(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm5
	vbroadcastsd	16(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm6
	vbroadcastsd	24(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm7
	vbroadcastsd	0(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm8
	vbroadcastsd	8(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm9
	vbroadcastsd	16(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm10
	vbroadcastsd	24(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm11

	vbroadcastsd	40(%r12), %ymm13
	vmulpd			%ymm5, %ymm13, %ymm5
	vbroadcastsd	48(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm5, %ymm13, %ymm6
	vbroadcastsd	56(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm5, %ymm13, %ymm7
	vbroadcastsd	32(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm5, %ymm13, %ymm8
	vbroadcastsd	40(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm5, %ymm13, %ymm9
	vbroadcastsd	48(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm5, %ymm13, %ymm10
	vbroadcastsd	56(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm5, %ymm13, %ymm11

	vbroadcastsd	48(%r12), %ymm13
	vmulpd			%ymm6, %ymm13, %ymm6
	vbroadcastsd	88(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm6, %ymm13, %ymm7
	vbroadcastsd	64(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm6, %ymm13, %ymm8
	vbroadcastsd	72(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm6, %ymm13, %ymm9
	vbroadcastsd	80(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm6, %ymm13, %ymm10
	vbroadcastsd	88(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm6, %ymm13, %ymm11

	vbroadcastsd	56(%r12), %ymm13
	vmulpd			%ymm7, %ymm13, %ymm7
	vbroadcastsd	96(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm7, %ymm13, %ymm8
	vbroadcastsd	104(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm7, %ymm13, %ymm9
	vbroadcastsd	112(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm7, %ymm13, %ymm10
	vbroadcastsd	120(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm7, %ymm13, %ymm11

	addq	$ 128, %r10

	vbroadcastsd	64(%r12), %ymm13
	vmulpd			%ymm8, %ymm13, %ymm8
	vbroadcastsd	8(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm8, %ymm13, %ymm9
	vbroadcastsd	16(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm8, %ymm13, %ymm10
	vbroadcastsd	24(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm8, %ymm13, %ymm11

	vbroadcastsd	72(%r12), %ymm13
	vmulpd			%ymm9, %ymm13, %ymm9
	vbroadcastsd	48(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm9, %ymm13, %ymm10
	vbroadcastsd	56(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm9, %ymm13, %ymm11

	vbroadcastsd	80(%r12), %ymm13
	vmulpd			%ymm10, %ymm13, %ymm10
	vbroadcastsd	88(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm10, %ymm13, %ymm11

	vbroadcastsd	88(%r12), %ymm13
	vmulpd			%ymm11, %ymm13, %ymm11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rlt_inv_4x12_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution for cholesky factorization 
//
// input arguments:
// r10   <- D
// r11   <- inv_diag_D
// r12d  <- kn
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
// ymm11 <- [d82 d93 d60 db1]
// ymm12 <- dirty
// ymm13 <- dirty
//
// output arguments:
// r10   <- D
// r11   <- inv_diag_D
// r12d  <- kn
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm4  <- [d40 d51 d62 d73]
// ymm5  <- [d41 d50 d63 d72]
// ymm6  <- [d43 d52 d61 d70]
// ymm7  <- [d42 d53 d60 d71]
// ymm8  <- [d80 d91 da2 db3]
// ymm9  <- [d81 d90 da3 db2]
// ymm10 <- [d83 d92 da1 db0]
// ymm11 <- [d82 d93 d60 db1]
// ymm12 <- dirty
// ymm13 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RLT_INV_12X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rlt_inv_12x4_vs_lib4)
#endif
	
	vbroadcastsd	0(%r11), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm0
	vmulpd			%ymm4, %ymm13, %ymm4
	vmulpd			%ymm8, %ymm13, %ymm8
	cmpl			$ 2, %r12d
	jl				0f // ret
	vbroadcastsd	8(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm1
	vfnmadd231pd	%ymm4, %ymm13, %ymm5
	vfnmadd231pd	%ymm8, %ymm13, %ymm9
	vbroadcastsd	16(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm2
	vfnmadd231pd	%ymm4, %ymm13, %ymm6
	vfnmadd231pd	%ymm8, %ymm13, %ymm10
	vbroadcastsd	24(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm3
	vfnmadd231pd	%ymm4, %ymm13, %ymm7
	vfnmadd231pd	%ymm8, %ymm13, %ymm11

	vbroadcastsd	8(%r11), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm1
	vmulpd			%ymm5, %ymm13, %ymm5
	vmulpd			%ymm9, %ymm13, %ymm9
	cmpl			$ 3, %r12d
	jl				0f // ret
	vbroadcastsd	48(%r10), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm2
	vfnmadd231pd	%ymm5, %ymm13, %ymm6
	vfnmadd231pd	%ymm9, %ymm13, %ymm10
	vbroadcastsd	56(%r10), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm3
	vfnmadd231pd	%ymm5, %ymm13, %ymm7
	vfnmadd231pd	%ymm9, %ymm13, %ymm11

	vbroadcastsd	16(%r11), %ymm13
	vmulpd			%ymm2, %ymm13, %ymm2
	vmulpd			%ymm6, %ymm13, %ymm6
	vmulpd			%ymm10, %ymm13, %ymm10
	cmpl			$ 4, %r12d
	jl				0f // ret
	vbroadcastsd	88(%r10), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm3
	vfnmadd231pd	%ymm6, %ymm13, %ymm7
	vfnmadd231pd	%ymm10, %ymm13, %ymm11

	vbroadcastsd	24(%r11), %ymm13
	vmulpd			%ymm3, %ymm13, %ymm3
	vmulpd			%ymm7, %ymm13, %ymm7
	vmulpd			%ymm11, %ymm13, %ymm11

0:
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rlt_inv_12x4_vs_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution for cholesky factorization 
//
// input arguments:
// r10  <- D
// r11  <- sdd
// r12  <- inv_diag_D
// r13d <- kn
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
// r11  <- sdd
// r12  <- inv_diag_D
// r13d <- kn
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
	.macro INNER_EDGE_DTRSM_RLT_INV_4X12_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rlt_inv_4x12_vs_lib4)
#endif
	
	vbroadcastsd	0(%r12), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm0
	vbroadcastsd	8(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm1
	vbroadcastsd	16(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm2
	vbroadcastsd	24(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm3
	vbroadcastsd	0(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm4
	vbroadcastsd	8(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm5
	vbroadcastsd	16(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm6
	vbroadcastsd	24(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm7
	vbroadcastsd	0(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm8
	vbroadcastsd	8(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm9
	vbroadcastsd	16(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm10
	vbroadcastsd	24(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm11

	vbroadcastsd	8(%r12), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm1
	vbroadcastsd	48(%r10), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm2
	vbroadcastsd	56(%r10), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm3
	vbroadcastsd	32(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm4
	vbroadcastsd	40(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm5
	vbroadcastsd	48(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm6
	vbroadcastsd	56(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm7
	vbroadcastsd	32(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm8
	vbroadcastsd	40(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm9
	vbroadcastsd	48(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm10
	vbroadcastsd	56(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm11

	vbroadcastsd	16(%r12), %ymm13
	vmulpd			%ymm2, %ymm13, %ymm2
	vbroadcastsd	88(%r10), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm3
	vbroadcastsd	64(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm4
	vbroadcastsd	72(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm5
	vbroadcastsd	80(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm6
	vbroadcastsd	88(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm7
	vbroadcastsd	64(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm8
	vbroadcastsd	72(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm9
	vbroadcastsd	80(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm10
	vbroadcastsd	88(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm11

	vbroadcastsd	24(%r12), %ymm13
	vmulpd			%ymm3, %ymm13, %ymm3
	vbroadcastsd	96(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm4
	vbroadcastsd	104(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm5
	vbroadcastsd	112(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm6
	vbroadcastsd	120(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm7
	vbroadcastsd	96(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm8
	vbroadcastsd	104(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm9
	vbroadcastsd	112(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm10
	vbroadcastsd	120(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm3, %ymm13, %ymm11

	addq	$ 128, %r10

	vbroadcastsd	32(%r12), %ymm13
	vmulpd			%ymm4, %ymm13, %ymm4
	vbroadcastsd	8(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm5
	vbroadcastsd	16(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm6
	vbroadcastsd	24(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm7
	vbroadcastsd	0(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm8
	vbroadcastsd	8(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm9
	vbroadcastsd	16(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm10
	vbroadcastsd	24(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm11

	vbroadcastsd	40(%r12), %ymm13
	vmulpd			%ymm5, %ymm13, %ymm5
	vbroadcastsd	48(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm5, %ymm13, %ymm6
	vbroadcastsd	56(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm5, %ymm13, %ymm7
	vbroadcastsd	32(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm5, %ymm13, %ymm8
	vbroadcastsd	40(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm5, %ymm13, %ymm9
	vbroadcastsd	48(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm5, %ymm13, %ymm10
	vbroadcastsd	56(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm5, %ymm13, %ymm11

	vbroadcastsd	48(%r12), %ymm13
	vmulpd			%ymm6, %ymm13, %ymm6
	vbroadcastsd	88(%r10, %r11), %ymm13
	vfnmadd231pd	%ymm6, %ymm13, %ymm7
	vbroadcastsd	64(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm6, %ymm13, %ymm8
	vbroadcastsd	72(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm6, %ymm13, %ymm9
	vbroadcastsd	80(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm6, %ymm13, %ymm10
	vbroadcastsd	88(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm6, %ymm13, %ymm11

	vbroadcastsd	56(%r12), %ymm13
	vmulpd			%ymm7, %ymm13, %ymm7
	vbroadcastsd	96(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm7, %ymm13, %ymm8
	vbroadcastsd	104(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm7, %ymm13, %ymm9
	vbroadcastsd	112(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm7, %ymm13, %ymm10
	vbroadcastsd	120(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm7, %ymm13, %ymm11

	addq	$ 128, %r10

	vbroadcastsd	64(%r12), %ymm13
	vmulpd			%ymm8, %ymm13, %ymm8
	cmpl			$ 10, %r13d
	jl				0f // ret
	vbroadcastsd	8(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm8, %ymm13, %ymm9
	vbroadcastsd	16(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm8, %ymm13, %ymm10
	vbroadcastsd	24(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm8, %ymm13, %ymm11

	vbroadcastsd	72(%r12), %ymm13
	vmulpd			%ymm9, %ymm13, %ymm9
	cmpl			$ 11, %r13d
	jl				0f // ret
	vbroadcastsd	48(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm9, %ymm13, %ymm10
	vbroadcastsd	56(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm9, %ymm13, %ymm11

	vbroadcastsd	80(%r12), %ymm13
	vmulpd			%ymm10, %ymm13, %ymm10
	cmpl			$ 12, %r13d
	jl				0f // ret
	vbroadcastsd	88(%r10, %r11, 2), %ymm13
	vfnmadd231pd	%ymm10, %ymm13, %ymm11

	vbroadcastsd	88(%r12), %ymm13
	vmulpd			%ymm11, %ymm13, %ymm11

0:
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rlt_inv_4x12_vs_lib4)
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
// ymm8  <- [d80 d91 da2 db3]
// ymm9  <- [d81 d90 da3 db2]
// ymm10 <- [d83 d92 da1 db0]
// ymm11 <- [d82 d93 d60 db1]
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
// ymm8  <- [d80 d91 da2 db3]
// ymm9  <- [d81 d90 da3 db2]
// ymm10 <- [d83 d92 da1 db0]
// ymm11 <- [d82 d93 d60 db1]
// ymm12 <- dirty
// ymm13 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RLT_ONE_12X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rlt_one_12x4_lib4)
#endif
	
	vbroadcastsd	8(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm1
	vfnmadd231pd	%ymm4, %ymm13, %ymm5
	vfnmadd231pd	%ymm8, %ymm13, %ymm9

	vbroadcastsd	16(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm2
	vfnmadd231pd	%ymm4, %ymm13, %ymm6
	vfnmadd231pd	%ymm8, %ymm13, %ymm10
	vbroadcastsd	48(%r10), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm2
	vfnmadd231pd	%ymm5, %ymm13, %ymm6
	vfnmadd231pd	%ymm9, %ymm13, %ymm10

	vbroadcastsd	24(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm3
	vfnmadd231pd	%ymm4, %ymm13, %ymm7
	vfnmadd231pd	%ymm8, %ymm13, %ymm11
	vbroadcastsd	56(%r10), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm3
	vfnmadd231pd	%ymm5, %ymm13, %ymm7
	vfnmadd231pd	%ymm9, %ymm13, %ymm11
	vbroadcastsd	88(%r10), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm3
	vfnmadd231pd	%ymm6, %ymm13, %ymm7
	vfnmadd231pd	%ymm10, %ymm13, %ymm11

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rlt_one_12x4_lib4)
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
// ymm8  <- [d80 d91 da2 db3]
// ymm9  <- [d81 d90 da3 db2]
// ymm10 <- [d83 d92 da1 db0]
// ymm11 <- [d82 d93 d60 db1]
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
// ymm8  <- [d80 d91 da2 db3]
// ymm9  <- [d81 d90 da3 db2]
// ymm10 <- [d83 d92 da1 db0]
// ymm11 <- [d82 d93 d60 db1]
// ymm12 <- dirty
// ymm13 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RLT_ONE_12X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rlt_one_12x4_vs_lib4)
#endif
	
	cmpl			$ 2, %r11d
	jl				0f // ret

	vbroadcastsd	8(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm1
	vfnmadd231pd	%ymm4, %ymm13, %ymm5
	vfnmadd231pd	%ymm8, %ymm13, %ymm9

	cmpl			$ 3, %r11d
	jl				0f // ret

	vbroadcastsd	16(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm2
	vfnmadd231pd	%ymm4, %ymm13, %ymm6
	vfnmadd231pd	%ymm8, %ymm13, %ymm10
	vbroadcastsd	48(%r10), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm2
	vfnmadd231pd	%ymm5, %ymm13, %ymm6
	vfnmadd231pd	%ymm9, %ymm13, %ymm10

	cmpl			$ 4, %r11d
	jl				0f // ret

	vbroadcastsd	24(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm3
	vfnmadd231pd	%ymm4, %ymm13, %ymm7
	vfnmadd231pd	%ymm8, %ymm13, %ymm11
	vbroadcastsd	56(%r10), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm3
	vfnmadd231pd	%ymm5, %ymm13, %ymm7
	vfnmadd231pd	%ymm9, %ymm13, %ymm11
	vbroadcastsd	88(%r10), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm3
	vfnmadd231pd	%ymm6, %ymm13, %ymm7
	vfnmadd231pd	%ymm10, %ymm13, %ymm11

0:
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rlt_one_12x4_vs_lib4)
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
// ymm8  <- [d80 d91 da2 db3]
// ymm9  <- [d81 d90 da3 db2]
// ymm10 <- [d83 d92 da1 db0]
// ymm11 <- [d82 d93 d60 db1]
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
// ymm8  <- [d80 d91 da2 db3]
// ymm9  <- [d81 d90 da3 db2]
// ymm10 <- [d83 d92 da1 db0]
// ymm11 <- [d82 d93 d60 db1]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RUN_INV_12X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_run_inv_12x4_lib4)
#endif

	// first column
	vbroadcastsd	0(%r11), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm0
	vmulpd			%ymm4, %ymm12, %ymm4
	vmulpd			%ymm8, %ymm12, %ymm8
	vbroadcastsd	32(%r10), %ymm12
	vfnmadd231pd	%ymm0, %ymm12, %ymm1
	vfnmadd231pd	%ymm4, %ymm12, %ymm5
	vfnmadd231pd	%ymm8, %ymm12, %ymm9
	vbroadcastsd	64(%r10), %ymm12
	vfnmadd231pd	%ymm0, %ymm12, %ymm2
	vfnmadd231pd	%ymm4, %ymm12, %ymm6
	vfnmadd231pd	%ymm8, %ymm12, %ymm10
	vbroadcastsd	96(%r10), %ymm12
	vfnmadd231pd	%ymm0, %ymm12, %ymm3
	vfnmadd231pd	%ymm4, %ymm12, %ymm7
	vfnmadd231pd	%ymm8, %ymm12, %ymm11

	// second column
	vbroadcastsd	8(%r11), %ymm12
	vmulpd			%ymm1, %ymm12, %ymm1
	vmulpd			%ymm5, %ymm12, %ymm5
	vmulpd			%ymm9, %ymm12, %ymm9
	vbroadcastsd	72(%r10), %ymm12
	vfnmadd231pd	%ymm1, %ymm12, %ymm2
	vfnmadd231pd	%ymm5, %ymm12, %ymm6
	vfnmadd231pd	%ymm9, %ymm12, %ymm10
	vbroadcastsd	104(%r10), %ymm12
	vfnmadd231pd	%ymm1, %ymm12, %ymm3
	vfnmadd231pd	%ymm5, %ymm12, %ymm7
	vfnmadd231pd	%ymm9, %ymm12, %ymm11

	// third column
	vbroadcastsd	16(%r11), %ymm12
	vmulpd			%ymm2, %ymm12, %ymm2
	vmulpd			%ymm6, %ymm12, %ymm6
	vmulpd			%ymm10, %ymm12, %ymm10
	vbroadcastsd	112(%r10), %ymm12
	vfnmadd231pd	%ymm2, %ymm12, %ymm3
	vfnmadd231pd	%ymm6, %ymm12, %ymm7
	vfnmadd231pd	%ymm10, %ymm12, %ymm11

	// fourth column
	vbroadcastsd	24(%r11), %ymm12
	vmulpd			%ymm3, %ymm12, %ymm3
	vmulpd			%ymm7, %ymm12, %ymm7
	vmulpd			%ymm11, %ymm12, %ymm11

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_run_inv_12x4_lib4)
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
// ymm8  <- [d80 d91 da2 db3]
// ymm9  <- [d81 d90 da3 db2]
// ymm10 <- [d83 d92 da1 db0]
// ymm11 <- [d82 d93 d60 db1]
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
// ymm8  <- [d80 d91 da2 db3]
// ymm9  <- [d81 d90 da3 db2]
// ymm10 <- [d83 d92 da1 db0]
// ymm11 <- [d82 d93 d60 db1]
// ymm12 <- dirty
// ymm13 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RUT_INV_12X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rut_inv_12x4_lib4)
#endif
	
	vbroadcastsd	24(%r11), %ymm12
	vmulpd			%ymm3, %ymm12, %ymm3
	vmulpd			%ymm7, %ymm12, %ymm7
	vmulpd			%ymm11, %ymm12, %ymm11
	vbroadcastsd	112(%r10), %ymm12
	vfnmadd231pd	%ymm3, %ymm12, %ymm2
	vfnmadd231pd	%ymm7, %ymm12, %ymm6
	vfnmadd231pd	%ymm11, %ymm12, %ymm10
	vbroadcastsd	104(%r10), %ymm12
	vfnmadd231pd	%ymm3, %ymm12, %ymm1
	vfnmadd231pd	%ymm7, %ymm12, %ymm5
	vfnmadd231pd	%ymm11, %ymm12, %ymm9
	vbroadcastsd	96(%r10), %ymm12
	vfnmadd231pd	%ymm3, %ymm12, %ymm0
	vfnmadd231pd	%ymm7, %ymm12, %ymm4
	vfnmadd231pd	%ymm11, %ymm12, %ymm8

	vbroadcastsd	16(%r11), %ymm12
	vmulpd			%ymm2, %ymm12, %ymm2
	vmulpd			%ymm6, %ymm12, %ymm6
	vmulpd			%ymm10, %ymm12, %ymm10
	vbroadcastsd	72(%r10), %ymm12
	vfnmadd231pd	%ymm2, %ymm12, %ymm1
	vfnmadd231pd	%ymm6, %ymm12, %ymm5
	vfnmadd231pd	%ymm10, %ymm12, %ymm9
	vbroadcastsd	64(%r10), %ymm12
	vfnmadd231pd	%ymm2, %ymm12, %ymm0
	vfnmadd231pd	%ymm6, %ymm12, %ymm4
	vfnmadd231pd	%ymm10, %ymm12, %ymm8

	vbroadcastsd	8(%r11), %ymm12
	vmulpd			%ymm1, %ymm12, %ymm1
	vmulpd			%ymm5, %ymm12, %ymm5
	vmulpd			%ymm9, %ymm12, %ymm9
	vbroadcastsd	32(%r10), %ymm12
	vfnmadd231pd	%ymm1, %ymm12, %ymm0
	vfnmadd231pd	%ymm5, %ymm12, %ymm4
	vfnmadd231pd	%ymm9, %ymm12, %ymm8

	vbroadcastsd	0(%r11), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm0
	vmulpd			%ymm4, %ymm12, %ymm4
	vmulpd			%ymm8, %ymm12, %ymm8

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rut_inv_12x4_lib4)
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
// ymm8  <- [d80 d91 da2 db3]
// ymm9  <- [d81 d90 da3 db2]
// ymm10 <- [d83 d92 da1 db0]
// ymm11 <- [d82 d93 d60 db1]
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
// ymm8  <- [d80 d91 da2 db3]
// ymm9  <- [d81 d90 da3 db2]
// ymm10 <- [d83 d92 da1 db0]
// ymm11 <- [d82 d93 d60 db1]
// ymm12 <- dirty
// ymm13 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RUT_INV_12X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rut_inv_12x4_vs_lib4)
#endif
	
	cmpl			$ 3, %r12d
	jle				0f

	vbroadcastsd	24(%r11), %ymm12
	vmulpd			%ymm3, %ymm12, %ymm3
	vmulpd			%ymm7, %ymm12, %ymm7
	vmulpd			%ymm11, %ymm12, %ymm11
	vbroadcastsd	112(%r10), %ymm12
	vfnmadd231pd	%ymm3, %ymm12, %ymm2
	vfnmadd231pd	%ymm7, %ymm12, %ymm6
	vfnmadd231pd	%ymm11, %ymm12, %ymm10
	vbroadcastsd	104(%r10), %ymm12
	vfnmadd231pd	%ymm3, %ymm12, %ymm1
	vfnmadd231pd	%ymm7, %ymm12, %ymm5
	vfnmadd231pd	%ymm11, %ymm12, %ymm9
	vbroadcastsd	96(%r10), %ymm12
	vfnmadd231pd	%ymm3, %ymm12, %ymm0
	vfnmadd231pd	%ymm7, %ymm12, %ymm4
	vfnmadd231pd	%ymm11, %ymm12, %ymm8

0:
	cmpl			$ 2, %r12d
	jle				1f

	vbroadcastsd	16(%r11), %ymm12
	vmulpd			%ymm2, %ymm12, %ymm2
	vmulpd			%ymm6, %ymm12, %ymm6
	vmulpd			%ymm10, %ymm12, %ymm10
	vbroadcastsd	72(%r10), %ymm12
	vfnmadd231pd	%ymm2, %ymm12, %ymm1
	vfnmadd231pd	%ymm6, %ymm12, %ymm5
	vfnmadd231pd	%ymm10, %ymm12, %ymm9
	vbroadcastsd	64(%r10), %ymm12
	vfnmadd231pd	%ymm2, %ymm12, %ymm0
	vfnmadd231pd	%ymm6, %ymm12, %ymm4
	vfnmadd231pd	%ymm10, %ymm12, %ymm8

1:
	cmpl			$ 1, %r12d
	jle				2f

	vbroadcastsd	8(%r11), %ymm12
	vmulpd			%ymm1, %ymm12, %ymm1
	vmulpd			%ymm5, %ymm12, %ymm5
	vmulpd			%ymm9, %ymm12, %ymm9
	vbroadcastsd	32(%r10), %ymm12
	vfnmadd231pd	%ymm1, %ymm12, %ymm0
	vfnmadd231pd	%ymm5, %ymm12, %ymm4
	vfnmadd231pd	%ymm9, %ymm12, %ymm8

2:

	vbroadcastsd	0(%r11), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm0
	vmulpd			%ymm4, %ymm12, %ymm4
	vmulpd			%ymm8, %ymm12, %ymm8

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rut_inv_12x4_vs_lib4)
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
// ymm8  <- [d80 d91 da2 db3]
// ymm9  <- [d81 d90 da3 db2]
// ymm10 <- [d83 d92 da1 db0]
// ymm11 <- [d82 d93 d60 db1]
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
// ymm8  <- [d80 d91 da2 db3]
// ymm9  <- [d81 d90 da3 db2]
// ymm10 <- [d83 d92 da1 db0]
// ymm11 <- [d82 d93 d60 db1]
// ymm12 <- dirty
// ymm13 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RUT_ONE_12X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rut_one_12x4_lib4)
#endif
	
	vbroadcastsd	112(%r10), %ymm12
	vfnmadd231pd	%ymm3, %ymm12, %ymm2
	vfnmadd231pd	%ymm7, %ymm12, %ymm6
	vfnmadd231pd	%ymm11, %ymm12, %ymm10
	vbroadcastsd	104(%r10), %ymm12
	vfnmadd231pd	%ymm3, %ymm12, %ymm1
	vfnmadd231pd	%ymm7, %ymm12, %ymm5
	vfnmadd231pd	%ymm11, %ymm12, %ymm9
	vbroadcastsd	96(%r10), %ymm12
	vfnmadd231pd	%ymm3, %ymm12, %ymm0
	vfnmadd231pd	%ymm7, %ymm12, %ymm4
	vfnmadd231pd	%ymm11, %ymm12, %ymm8

	vbroadcastsd	72(%r10), %ymm12
	vfnmadd231pd	%ymm2, %ymm12, %ymm1
	vfnmadd231pd	%ymm6, %ymm12, %ymm5
	vfnmadd231pd	%ymm10, %ymm12, %ymm9
	vbroadcastsd	64(%r10), %ymm12
	vfnmadd231pd	%ymm2, %ymm12, %ymm0
	vfnmadd231pd	%ymm6, %ymm12, %ymm4
	vfnmadd231pd	%ymm10, %ymm12, %ymm8

	vbroadcastsd	32(%r10), %ymm12
	vfnmadd231pd	%ymm1, %ymm12, %ymm0
	vfnmadd231pd	%ymm5, %ymm12, %ymm4
	vfnmadd231pd	%ymm9, %ymm12, %ymm8

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rut_one_12x4_lib4)
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
// ymm8  <- [d80 d91 da2 db3]
// ymm9  <- [d81 d90 da3 db2]
// ymm10 <- [d83 d92 da1 db0]
// ymm11 <- [d82 d93 d60 db1]
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
// ymm8  <- [d80 d91 da2 db3]
// ymm9  <- [d81 d90 da3 db2]
// ymm10 <- [d83 d92 da1 db0]
// ymm11 <- [d82 d93 d60 db1]
// ymm12 <- dirty
// ymm13 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RUT_ONE_12X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rut_one_12x4_vs_lib4)
#endif
	
	cmpl			$ 3, %r11d
	jle				0f

	vbroadcastsd	112(%r10), %ymm12
	vfnmadd231pd	%ymm3, %ymm12, %ymm2
	vfnmadd231pd	%ymm7, %ymm12, %ymm6
	vfnmadd231pd	%ymm11, %ymm12, %ymm10
	vbroadcastsd	104(%r10), %ymm12
	vfnmadd231pd	%ymm3, %ymm12, %ymm1
	vfnmadd231pd	%ymm7, %ymm12, %ymm5
	vfnmadd231pd	%ymm11, %ymm12, %ymm9
	vbroadcastsd	96(%r10), %ymm12
	vfnmadd231pd	%ymm3, %ymm12, %ymm0
	vfnmadd231pd	%ymm7, %ymm12, %ymm4
	vfnmadd231pd	%ymm11, %ymm12, %ymm8

0:
	cmpl			$ 2, %r11d
	jle				1f

	vbroadcastsd	72(%r10), %ymm12
	vfnmadd231pd	%ymm2, %ymm12, %ymm1
	vfnmadd231pd	%ymm6, %ymm12, %ymm5
	vfnmadd231pd	%ymm10, %ymm12, %ymm9
	vbroadcastsd	64(%r10), %ymm12
	vfnmadd231pd	%ymm2, %ymm12, %ymm0
	vfnmadd231pd	%ymm6, %ymm12, %ymm4
	vfnmadd231pd	%ymm10, %ymm12, %ymm8

1:
	cmpl			$ 1, %r11d
	jle				2f

	vbroadcastsd	32(%r10), %ymm12
	vfnmadd231pd	%ymm1, %ymm12, %ymm0
	vfnmadd231pd	%ymm5, %ymm12, %ymm4
	vfnmadd231pd	%ymm9, %ymm12, %ymm8

2:


#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rut_one_12x4_vs_lib4)
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
// r10   <- E0
// r11   <- 4*sde*sizeof(double)
// r12  <- inv_diag_E
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
//
// output arguments:
// r10   <- E0
// r11   <- 4*sde*sizeof(double)
// r12  <- inv_diag_E
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
	.macro INNER_EDGE_DTRSM_LLN_INV_12X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_lln_inv_12x4_lib4)
#endif


	// left block-column

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC05(%rip), %ymm15 // 1.0
#elif defined(OS_MAC)
	vmovapd			LC05(%rip), %ymm15 // 1.0
#endif
	vbroadcastsd	0(%r12), %ymm12
	vblendpd		$ 0x1, %ymm12, %ymm15, %ymm12
	vmulpd			%ymm12, %ymm0, %ymm0
	vmulpd			%ymm12, %ymm1, %ymm1
	vmulpd			%ymm12, %ymm2, %ymm2
	vmulpd			%ymm12, %ymm3, %ymm3
	vxorpd			%ymm15, %ymm15, %ymm15
	vmovapd			0(%r10), %ymm12
	vblendpd		$ 0x1, %ymm15, %ymm12, %ymm12
	vmovapd			0(%r10, %r11), %ymm14
	vmovapd			0(%r10, %r11, 2), %ymm15
	vpermpd			$ 0x00, %ymm0, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm0
	vfnmadd231pd	%ymm14, %ymm13, %ymm4
	vfnmadd231pd	%ymm15, %ymm13, %ymm8
	vpermpd			$ 0x00, %ymm1, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm1
	vfnmadd231pd	%ymm14, %ymm13, %ymm5
	vfnmadd231pd	%ymm15, %ymm13, %ymm9
	vpermpd			$ 0x00, %ymm2, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm2
	vfnmadd231pd	%ymm14, %ymm13, %ymm6
	vfnmadd231pd	%ymm15, %ymm13, %ymm10
	vpermpd			$ 0x00, %ymm3, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm3
	vfnmadd231pd	%ymm14, %ymm13, %ymm7
	vfnmadd231pd	%ymm15, %ymm13, %ymm11

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC05(%rip), %ymm15 // 1.0
#elif defined(OS_MAC)
	vmovapd			LC05(%rip), %ymm15 // 1.0
#endif
	vbroadcastsd	8(%r12), %ymm12
	vblendpd		$ 0x2, %ymm12, %ymm15, %ymm12
	vmulpd			%ymm12, %ymm0, %ymm0
	vmulpd			%ymm12, %ymm1, %ymm1
	vmulpd			%ymm12, %ymm2, %ymm2
	vmulpd			%ymm12, %ymm3, %ymm3
	vxorpd			%ymm15, %ymm15, %ymm15
	vmovapd			32(%r10), %ymm12
	vblendpd		$ 0x3, %ymm15, %ymm12, %ymm12
	vmovapd			32(%r10, %r11), %ymm14
	vmovapd			32(%r10, %r11, 2), %ymm15
	vpermpd			$ 0x55, %ymm0, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm0
	vfnmadd231pd	%ymm14, %ymm13, %ymm4
	vfnmadd231pd	%ymm15, %ymm13, %ymm8
	vpermpd			$ 0x55, %ymm1, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm1
	vfnmadd231pd	%ymm14, %ymm13, %ymm5
	vfnmadd231pd	%ymm15, %ymm13, %ymm9
	vpermpd			$ 0x55, %ymm2, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm2
	vfnmadd231pd	%ymm14, %ymm13, %ymm6
	vfnmadd231pd	%ymm15, %ymm13, %ymm10
	vpermpd			$ 0x55, %ymm3, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm3
	vfnmadd231pd	%ymm14, %ymm13, %ymm7
	vfnmadd231pd	%ymm15, %ymm13, %ymm11

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC05(%rip), %ymm15 // 1.0
#elif defined(OS_MAC)
	vmovapd			LC05(%rip), %ymm15 // 1.0
#endif
	vbroadcastsd	16(%r12), %ymm12
	vblendpd		$ 0x4, %ymm12, %ymm15, %ymm12
	vmulpd			%ymm12, %ymm0, %ymm0
	vmulpd			%ymm12, %ymm1, %ymm1
	vmulpd			%ymm12, %ymm2, %ymm2
	vmulpd			%ymm12, %ymm3, %ymm3
	vxorpd			%ymm15, %ymm15, %ymm15
	vmovapd			64(%r10), %ymm12
	vblendpd		$ 0x7, %ymm15, %ymm12, %ymm12
	vmovapd			64(%r10, %r11), %ymm14
	vmovapd			64(%r10, %r11, 2), %ymm15
	vpermpd			$ 0xaa, %ymm0, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm0
	vfnmadd231pd	%ymm14, %ymm13, %ymm4
	vfnmadd231pd	%ymm15, %ymm13, %ymm8
	vpermpd			$ 0xaa, %ymm1, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm1
	vfnmadd231pd	%ymm14, %ymm13, %ymm5
	vfnmadd231pd	%ymm15, %ymm13, %ymm9
	vpermpd			$ 0xaa, %ymm2, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm2
	vfnmadd231pd	%ymm14, %ymm13, %ymm6
	vfnmadd231pd	%ymm15, %ymm13, %ymm10
	vpermpd			$ 0xaa, %ymm3, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm3
	vfnmadd231pd	%ymm14, %ymm13, %ymm7
	vfnmadd231pd	%ymm15, %ymm13, %ymm11

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC05(%rip), %ymm15 // 1.0
#elif defined(OS_MAC)
	vmovapd			LC05(%rip), %ymm15 // 1.0
#endif
	vbroadcastsd	24(%r12), %ymm12
	vblendpd		$ 0x8, %ymm12, %ymm15, %ymm12
	vmulpd			%ymm12, %ymm0, %ymm0
	vmulpd			%ymm12, %ymm1, %ymm1
	vmulpd			%ymm12, %ymm2, %ymm2
	vmulpd			%ymm12, %ymm3, %ymm3
	vmovapd			96(%r10, %r11), %ymm14
	vmovapd			96(%r10, %r11, 2), %ymm15
	vpermpd			$ 0xff, %ymm0, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm4
	vfnmadd231pd	%ymm15, %ymm13, %ymm8
	vpermpd			$ 0xff, %ymm1, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm5
	vfnmadd231pd	%ymm15, %ymm13, %ymm9
	vpermpd			$ 0xff, %ymm2, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm6
	vfnmadd231pd	%ymm15, %ymm13, %ymm10
	vpermpd			$ 0xff, %ymm3, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm7
	vfnmadd231pd	%ymm15, %ymm13, %ymm11

	addq		$ 128, %r10


#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC05(%rip), %ymm15 // 1.0
#elif defined(OS_MAC)
	vmovapd			LC05(%rip), %ymm15 // 1.0
#endif
	// middle block-column

	vbroadcastsd	32(%r12), %ymm12
	vblendpd		$ 0x1, %ymm12, %ymm15, %ymm12
	vmulpd			%ymm12, %ymm4, %ymm4
	vmulpd			%ymm12, %ymm5, %ymm5
	vmulpd			%ymm12, %ymm6, %ymm6
	vmulpd			%ymm12, %ymm7, %ymm7
	vxorpd			%ymm14, %ymm14, %ymm14
	vmovapd			0(%r10, %r11), %ymm12
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm12
	vmovapd			0(%r10, %r11, 2), %ymm14
	vpermpd			$ 0x00, %ymm4, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm4
	vfnmadd231pd	%ymm14, %ymm13, %ymm8
	vpermpd			$ 0x00, %ymm5, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm5
	vfnmadd231pd	%ymm14, %ymm13, %ymm9
	vpermpd			$ 0x00, %ymm6, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm6
	vfnmadd231pd	%ymm14, %ymm13, %ymm10
	vpermpd			$ 0x00, %ymm7, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm7
	vfnmadd231pd	%ymm14, %ymm13, %ymm11

	vbroadcastsd	40(%r12), %ymm12
	vblendpd		$ 0x2, %ymm12, %ymm15, %ymm12
	vmulpd			%ymm12, %ymm4, %ymm4
	vmulpd			%ymm12, %ymm5, %ymm5
	vmulpd			%ymm12, %ymm6, %ymm6
	vmulpd			%ymm12, %ymm7, %ymm7
	vxorpd			%ymm14, %ymm14, %ymm14
	vmovapd			32(%r10, %r11), %ymm12
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm12
	vmovapd			32(%r10, %r11, 2), %ymm14
	vpermpd			$ 0x55, %ymm4, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm4
	vfnmadd231pd	%ymm14, %ymm13, %ymm8
	vpermpd			$ 0x55, %ymm5, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm5
	vfnmadd231pd	%ymm14, %ymm13, %ymm9
	vpermpd			$ 0x55, %ymm6, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm6
	vfnmadd231pd	%ymm14, %ymm13, %ymm10
	vpermpd			$ 0x55, %ymm7, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm7
	vfnmadd231pd	%ymm14, %ymm13, %ymm11

	vbroadcastsd	48(%r12), %ymm12
	vblendpd		$ 0x4, %ymm12, %ymm15, %ymm12
	vmulpd			%ymm12, %ymm4, %ymm4
	vmulpd			%ymm12, %ymm5, %ymm5
	vmulpd			%ymm12, %ymm6, %ymm6
	vmulpd			%ymm12, %ymm7, %ymm7
	vxorpd			%ymm14, %ymm14, %ymm14
	vmovapd			64(%r10, %r11), %ymm12
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm12
	vmovapd			64(%r10, %r11, 2), %ymm14
	vpermpd			$ 0xaa, %ymm4, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm4
	vfnmadd231pd	%ymm14, %ymm13, %ymm8
	vpermpd			$ 0xaa, %ymm5, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm5
	vfnmadd231pd	%ymm14, %ymm13, %ymm9
	vpermpd			$ 0xaa, %ymm6, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm6
	vfnmadd231pd	%ymm14, %ymm13, %ymm10
	vpermpd			$ 0xaa, %ymm7, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm7
	vfnmadd231pd	%ymm14, %ymm13, %ymm11

	vbroadcastsd	56(%r12), %ymm12
	vblendpd		$ 0x8, %ymm12, %ymm15, %ymm12
	vmulpd			%ymm12, %ymm4, %ymm4
	vmulpd			%ymm12, %ymm5, %ymm5
	vmulpd			%ymm12, %ymm6, %ymm6
	vmulpd			%ymm12, %ymm7, %ymm7
	vmovapd			96(%r10, %r11, 2), %ymm14
	vpermpd			$ 0xff, %ymm4, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm8
	vpermpd			$ 0xff, %ymm5, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm9
	vpermpd			$ 0xff, %ymm6, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm10
	vpermpd			$ 0xff, %ymm7, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm11


	addq		$ 128, %r10


	// right block-column

	vxorpd			%ymm14, %ymm14, %ymm14

	vbroadcastsd	64(%r12), %ymm12
	vblendpd		$ 0x1, %ymm12, %ymm15, %ymm12
	vmulpd			%ymm12, %ymm8, %ymm8
	vmulpd			%ymm12, %ymm9, %ymm9
	vmulpd			%ymm12, %ymm10, %ymm10
	vmulpd			%ymm12, %ymm11, %ymm11
	vmovapd			0(%r10, %r11, 2), %ymm12
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm12
	vpermpd			$ 0x00, %ymm8, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm8
	vpermpd			$ 0x00, %ymm9, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm9
	vpermpd			$ 0x00, %ymm10, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm10
	vpermpd			$ 0x00, %ymm11, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm11

	vbroadcastsd	72(%r12), %ymm12
	vblendpd		$ 0x2, %ymm12, %ymm15, %ymm12
	vmulpd			%ymm12, %ymm8, %ymm8
	vmulpd			%ymm12, %ymm9, %ymm9
	vmulpd			%ymm12, %ymm10, %ymm10
	vmulpd			%ymm12, %ymm11, %ymm11
	vmovapd			32(%r10, %r11, 2), %ymm12
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm12
	vpermpd			$ 0x55, %ymm8, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm8
	vpermpd			$ 0x55, %ymm9, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm9
	vpermpd			$ 0x55, %ymm10, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm10
	vpermpd			$ 0x55, %ymm11, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm11

	vbroadcastsd	80(%r12), %ymm12
	vblendpd		$ 0x4, %ymm12, %ymm15, %ymm12
	vmulpd			%ymm12, %ymm8, %ymm8
	vmulpd			%ymm12, %ymm9, %ymm9
	vmulpd			%ymm12, %ymm10, %ymm10
	vmulpd			%ymm12, %ymm11, %ymm11
	vmovapd			64(%r10, %r11, 2), %ymm12
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm12
	vpermpd			$ 0xaa, %ymm8, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm8
	vpermpd			$ 0xaa, %ymm9, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm9
	vpermpd			$ 0xaa, %ymm10, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm10
	vpermpd			$ 0xaa, %ymm11, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm11

	vbroadcastsd	88(%r12), %ymm12
	vblendpd		$ 0x8, %ymm12, %ymm15, %ymm12
	vmulpd			%ymm12, %ymm8, %ymm8
	vmulpd			%ymm12, %ymm9, %ymm9
	vmulpd			%ymm12, %ymm10, %ymm10
	vmulpd			%ymm12, %ymm11, %ymm11

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_lln_inv_12x4_lib4)
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
// r10   <- E0
// r11   <- 4*sde*sizeof(double)
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
//
// output arguments:
// r10   <- E0
// r11   <- 4*sde*sizeof(double)
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
	.macro INNER_EDGE_DTRSM_LLN_ONE_12X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_lln_one_12x4_lib4)
#endif

	// left block-column

	vxorpd			%ymm15, %ymm15, %ymm15
	vmovapd			0(%r10), %ymm12
	vblendpd		$ 0x1, %ymm15, %ymm12, %ymm12
	vmovapd			0(%r10, %r11), %ymm14
	vmovapd			0(%r10, %r11, 2), %ymm15
	vpermpd			$ 0x00, %ymm0, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm0
	vfnmadd231pd	%ymm14, %ymm13, %ymm4
	vfnmadd231pd	%ymm15, %ymm13, %ymm8
	vpermpd			$ 0x00, %ymm1, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm1
	vfnmadd231pd	%ymm14, %ymm13, %ymm5
	vfnmadd231pd	%ymm15, %ymm13, %ymm9
	vpermpd			$ 0x00, %ymm2, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm2
	vfnmadd231pd	%ymm14, %ymm13, %ymm6
	vfnmadd231pd	%ymm15, %ymm13, %ymm10
	vpermpd			$ 0x00, %ymm3, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm3
	vfnmadd231pd	%ymm14, %ymm13, %ymm7
	vfnmadd231pd	%ymm15, %ymm13, %ymm11

	vxorpd			%ymm15, %ymm15, %ymm15
	vmovapd			32(%r10), %ymm12
	vblendpd		$ 0x3, %ymm15, %ymm12, %ymm12
	vmovapd			32(%r10, %r11), %ymm14
	vmovapd			32(%r10, %r11, 2), %ymm15
	vpermpd			$ 0x55, %ymm0, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm0
	vfnmadd231pd	%ymm14, %ymm13, %ymm4
	vfnmadd231pd	%ymm15, %ymm13, %ymm8
	vpermpd			$ 0x55, %ymm1, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm1
	vfnmadd231pd	%ymm14, %ymm13, %ymm5
	vfnmadd231pd	%ymm15, %ymm13, %ymm9
	vpermpd			$ 0x55, %ymm2, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm2
	vfnmadd231pd	%ymm14, %ymm13, %ymm6
	vfnmadd231pd	%ymm15, %ymm13, %ymm10
	vpermpd			$ 0x55, %ymm3, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm3
	vfnmadd231pd	%ymm14, %ymm13, %ymm7
	vfnmadd231pd	%ymm15, %ymm13, %ymm11

	vxorpd			%ymm15, %ymm15, %ymm15
	vmovapd			64(%r10), %ymm12
	vblendpd		$ 0x7, %ymm15, %ymm12, %ymm12
	vmovapd			64(%r10, %r11), %ymm14
	vmovapd			64(%r10, %r11, 2), %ymm15
	vpermpd			$ 0xaa, %ymm0, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm0
	vfnmadd231pd	%ymm14, %ymm13, %ymm4
	vfnmadd231pd	%ymm15, %ymm13, %ymm8
	vpermpd			$ 0xaa, %ymm1, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm1
	vfnmadd231pd	%ymm14, %ymm13, %ymm5
	vfnmadd231pd	%ymm15, %ymm13, %ymm9
	vpermpd			$ 0xaa, %ymm2, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm2
	vfnmadd231pd	%ymm14, %ymm13, %ymm6
	vfnmadd231pd	%ymm15, %ymm13, %ymm10
	vpermpd			$ 0xaa, %ymm3, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm3
	vfnmadd231pd	%ymm14, %ymm13, %ymm7
	vfnmadd231pd	%ymm15, %ymm13, %ymm11

	vmovapd			96(%r10, %r11), %ymm14
	vmovapd			96(%r10, %r11, 2), %ymm15
	vpermpd			$ 0xff, %ymm0, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm4
	vfnmadd231pd	%ymm15, %ymm13, %ymm8
	vpermpd			$ 0xff, %ymm1, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm5
	vfnmadd231pd	%ymm15, %ymm13, %ymm9
	vpermpd			$ 0xff, %ymm2, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm6
	vfnmadd231pd	%ymm15, %ymm13, %ymm10
	vpermpd			$ 0xff, %ymm3, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm7
	vfnmadd231pd	%ymm15, %ymm13, %ymm11

	addq		$ 128, %r10


	// middle block-column

	vxorpd			%ymm14, %ymm14, %ymm14
	vmovapd			0(%r10, %r11), %ymm12
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm12
	vmovapd			0(%r10, %r11, 2), %ymm14
	vpermpd			$ 0x00, %ymm4, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm4
	vfnmadd231pd	%ymm14, %ymm13, %ymm8
	vpermpd			$ 0x00, %ymm5, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm5
	vfnmadd231pd	%ymm14, %ymm13, %ymm9
	vpermpd			$ 0x00, %ymm6, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm6
	vfnmadd231pd	%ymm14, %ymm13, %ymm10
	vpermpd			$ 0x00, %ymm7, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm7
	vfnmadd231pd	%ymm14, %ymm13, %ymm11

	vxorpd			%ymm14, %ymm14, %ymm14
	vmovapd			32(%r10, %r11), %ymm12
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm12
	vmovapd			32(%r10, %r11, 2), %ymm14
	vpermpd			$ 0x55, %ymm4, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm4
	vfnmadd231pd	%ymm14, %ymm13, %ymm8
	vpermpd			$ 0x55, %ymm5, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm5
	vfnmadd231pd	%ymm14, %ymm13, %ymm9
	vpermpd			$ 0x55, %ymm6, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm6
	vfnmadd231pd	%ymm14, %ymm13, %ymm10
	vpermpd			$ 0x55, %ymm7, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm7
	vfnmadd231pd	%ymm14, %ymm13, %ymm11

	vxorpd			%ymm14, %ymm14, %ymm14
	vmovapd			64(%r10, %r11), %ymm12
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm12
	vmovapd			64(%r10, %r11, 2), %ymm14
	vpermpd			$ 0xaa, %ymm4, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm4
	vfnmadd231pd	%ymm14, %ymm13, %ymm8
	vpermpd			$ 0xaa, %ymm5, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm5
	vfnmadd231pd	%ymm14, %ymm13, %ymm9
	vpermpd			$ 0xaa, %ymm6, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm6
	vfnmadd231pd	%ymm14, %ymm13, %ymm10
	vpermpd			$ 0xaa, %ymm7, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm7
	vfnmadd231pd	%ymm14, %ymm13, %ymm11

	vmovapd			96(%r10, %r11, 2), %ymm14
	vpermpd			$ 0xff, %ymm4, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm8
	vpermpd			$ 0xff, %ymm5, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm9
	vpermpd			$ 0xff, %ymm6, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm10
	vpermpd			$ 0xff, %ymm7, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm11


	addq		$ 128, %r10


	// right block-column

	vxorpd			%ymm14, %ymm14, %ymm14

	vmovapd			0(%r10, %r11, 2), %ymm12
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm12
	vpermpd			$ 0x00, %ymm8, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm8
	vpermpd			$ 0x00, %ymm9, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm9
	vpermpd			$ 0x00, %ymm10, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm10
	vpermpd			$ 0x00, %ymm11, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm11

	vmovapd			32(%r10, %r11, 2), %ymm12
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm12
	vpermpd			$ 0x55, %ymm8, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm8
	vpermpd			$ 0x55, %ymm9, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm9
	vpermpd			$ 0x55, %ymm10, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm10
	vpermpd			$ 0x55, %ymm11, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm11

	vmovapd			64(%r10, %r11, 2), %ymm12
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm12
	vpermpd			$ 0xaa, %ymm8, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm8
	vpermpd			$ 0xaa, %ymm9, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm9
	vpermpd			$ 0xaa, %ymm10, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm10
	vpermpd			$ 0xaa, %ymm11, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm11

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_lln_one_12x4_lib4)
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
// ymm8 <- [d80 d90 da0 db0]
// ymm9 <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]
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
// ymm8 <- [d80 d90 da0 db0]
// ymm9 <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_LUN_INV_12X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_lun_inv_12x4_lib4)
#endif
	
	movq	%r10, %r13 // E1 <- E0
	addq	%r11, %r13 // E1 <- E0 + 4*sde*sizeof(double)
	movq	%r13, %r14 // E2 <- E1
	addq	%r11, %r14 // E2 <- E1 + 4*sde*sizeof(double)

	// bottom-right

	vmovapd			352(%r14), %ymm13
	vxorpd			%ymm14, %ymm14, %ymm14 // 0.0
	vblendpd		$ 0x7, %ymm13, %ymm14, %ymm13
	vbroadcastsd	88(%r12), %ymm12
	vmovapd			352(%r13), %ymm15
//	vmovapd			352(%r10), %ymm11

	vpermpd			$ 0xff, %ymm8, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm8, %ymm8
	vfnmadd231pd	%ymm13, %ymm14, %ymm8
	vfnmadd231pd	%ymm15, %ymm14, %ymm4
	vfnmadd231pd	352(%r10), %ymm14, %ymm0

	vpermpd			$ 0xff, %ymm9, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm9, %ymm9
	vfnmadd231pd	%ymm13, %ymm14, %ymm9
	vfnmadd231pd	%ymm15, %ymm14, %ymm5
	vfnmadd231pd	352(%r10), %ymm14, %ymm1

	vpermpd			$ 0xff, %ymm10, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm10, %ymm10
	vfnmadd231pd	%ymm13, %ymm14, %ymm10
	vfnmadd231pd	%ymm15, %ymm14, %ymm6
	vfnmadd231pd	352(%r10), %ymm14, %ymm2

	vpermpd			$ 0xff, %ymm11, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm11, %ymm11
	vfnmadd231pd	%ymm13, %ymm14, %ymm11
	vfnmadd231pd	%ymm15, %ymm14, %ymm7
	vfnmadd231pd	352(%r10), %ymm14, %ymm3


	vxorpd			%ymm13, %ymm13, %ymm13 // 0.0 // ?????????????
	vmovapd			320(%r14), %xmm13
	vbroadcastsd	80(%r12), %ymm12
	vmovapd			320(%r13), %ymm15
//	vmovapd			320(%r10), %ymm11

	vpermpd			$ 0xaa, %ymm8, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm8, %ymm8
	vfnmadd231pd	%ymm13, %ymm14, %ymm8
	vfnmadd231pd	%ymm15, %ymm14, %ymm4
	vfnmadd231pd	320(%r10), %ymm14, %ymm0

	vpermpd			$ 0xaa, %ymm9, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm9, %ymm9
	vfnmadd231pd	%ymm13, %ymm14, %ymm9
	vfnmadd231pd	%ymm15, %ymm14, %ymm5
	vfnmadd231pd	320(%r10), %ymm14, %ymm1

	vpermpd			$ 0xaa, %ymm10, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm10, %ymm10
	vfnmadd231pd	%ymm13, %ymm14, %ymm10
	vfnmadd231pd	%ymm15, %ymm14, %ymm6
	vfnmadd231pd	320(%r10), %ymm14, %ymm2

	vpermpd			$ 0xaa, %ymm11, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm11, %ymm11
	vfnmadd231pd	%ymm13, %ymm14, %ymm11
	vfnmadd231pd	%ymm15, %ymm14, %ymm7
	vfnmadd231pd	320(%r10), %ymm14, %ymm3


	vxorpd			%ymm13, %ymm13, %ymm13 // 0.0
	vmovsd			288(%r14), %xmm13
	vbroadcastsd	72(%r12), %ymm12
	vmovapd			288(%r13), %ymm15
//	vmovapd			288(%r10), %ymm11

	vpermpd			$ 0x55, %ymm8, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm8, %ymm8
	vfnmadd231pd	%ymm13, %ymm14, %ymm8
	vfnmadd231pd	%ymm15, %ymm14, %ymm4
	vfnmadd231pd	288(%r10), %ymm14, %ymm0

	vpermpd			$ 0x55, %ymm9, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm9, %ymm9
	vfnmadd231pd	%ymm13, %ymm14, %ymm9
	vfnmadd231pd	%ymm15, %ymm14, %ymm5
	vfnmadd231pd	288(%r10), %ymm14, %ymm1

	vpermpd			$ 0x55, %ymm10, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm10, %ymm10
	vfnmadd231pd	%ymm13, %ymm14, %ymm10
	vfnmadd231pd	%ymm15, %ymm14, %ymm6
	vfnmadd231pd	288(%r10), %ymm14, %ymm2

	vpermpd			$ 0x55, %ymm11, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm11, %ymm11
	vfnmadd231pd	%ymm13, %ymm14, %ymm11
	vfnmadd231pd	%ymm15, %ymm14, %ymm7
	vfnmadd231pd	288(%r10), %ymm14, %ymm3


	vbroadcastsd	64(%r12), %ymm12
	vmovapd			256(%r13), %ymm15
//	vmovapd			256(%r10), %ymm11

	vpermpd			$ 0x00, %ymm8, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm8, %ymm8
	vfnmadd231pd	%ymm15, %ymm14, %ymm4
	vfnmadd231pd	256(%r10), %ymm14, %ymm0

	vpermpd			$ 0x00, %ymm9, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm9, %ymm9
	vfnmadd231pd	%ymm15, %ymm14, %ymm5
	vfnmadd231pd	256(%r10), %ymm14, %ymm1

	vpermpd			$ 0x00, %ymm10, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm10, %ymm10
	vfnmadd231pd	%ymm15, %ymm14, %ymm6
	vfnmadd231pd	256(%r10), %ymm14, %ymm2

	vpermpd			$ 0x00, %ymm11, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm11, %ymm11
	vfnmadd231pd	%ymm15, %ymm14, %ymm7
	vfnmadd231pd	256(%r10), %ymm14, %ymm3


	// middle-middle

	vmovapd			224(%r13), %ymm13
	vxorpd			%ymm14, %ymm14, %ymm14 // 0.0
	vblendpd		$ 0x7, %ymm13, %ymm14, %ymm13
	vbroadcastsd	56(%r12), %ymm12
	vmovapd			224(%r10), %ymm15

	vpermpd			$ 0xff, %ymm4, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm4, %ymm4
	vfnmadd231pd	%ymm13, %ymm14, %ymm4
	vfnmadd231pd	%ymm15, %ymm14, %ymm0

	vpermpd			$ 0xff, %ymm5, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm5, %ymm5
	vfnmadd231pd	%ymm13, %ymm14, %ymm5
	vfnmadd231pd	%ymm15, %ymm14, %ymm1

	vpermpd			$ 0xff, %ymm6, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm6, %ymm6
	vfnmadd231pd	%ymm13, %ymm14, %ymm6
	vfnmadd231pd	%ymm15, %ymm14, %ymm2

	vpermpd			$ 0xff, %ymm7, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm7, %ymm7
	vfnmadd231pd	%ymm13, %ymm14, %ymm7
	vfnmadd231pd	%ymm15, %ymm14, %ymm3


	vxorpd			%ymm13, %ymm13, %ymm13 // 0.0 // ?????????????
	vmovapd			192(%r13), %xmm13
	vbroadcastsd	48(%r12), %ymm12
	vmovapd			192(%r10), %ymm15

	vpermpd			$ 0xaa, %ymm4, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm4, %ymm4
	vfnmadd231pd	%ymm13, %ymm14, %ymm4
	vfnmadd231pd	%ymm15, %ymm14, %ymm0

	vpermpd			$ 0xaa, %ymm5, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm5, %ymm5
	vfnmadd231pd	%ymm13, %ymm14, %ymm5
	vfnmadd231pd	%ymm15, %ymm14, %ymm1

	vpermpd			$ 0xaa, %ymm6, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm6, %ymm6
	vfnmadd231pd	%ymm13, %ymm14, %ymm6
	vfnmadd231pd	%ymm15, %ymm14, %ymm2

	vpermpd			$ 0xaa, %ymm7, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm7, %ymm7
	vfnmadd231pd	%ymm13, %ymm14, %ymm7
	vfnmadd231pd	%ymm15, %ymm14, %ymm3


	vxorpd			%ymm13, %ymm13, %ymm13 // 0.0
	vmovsd			160(%r13), %xmm13
	vbroadcastsd	40(%r12), %ymm12
	vmovapd			160(%r10), %ymm15

	vpermpd			$ 0x55, %ymm4, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm4, %ymm4
	vfnmadd231pd	%ymm13, %ymm14, %ymm4
	vfnmadd231pd	%ymm15, %ymm14, %ymm0

	vpermpd			$ 0x55, %ymm5, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm5, %ymm5
	vfnmadd231pd	%ymm13, %ymm14, %ymm5
	vfnmadd231pd	%ymm15, %ymm14, %ymm1

	vpermpd			$ 0x55, %ymm6, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm6, %ymm6
	vfnmadd231pd	%ymm13, %ymm14, %ymm6
	vfnmadd231pd	%ymm15, %ymm14, %ymm2

	vpermpd			$ 0x55, %ymm7, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm7, %ymm7
	vfnmadd231pd	%ymm13, %ymm14, %ymm7
	vfnmadd231pd	%ymm15, %ymm14, %ymm3


	vbroadcastsd	32(%r12), %ymm12
	vmovapd			128(%r10), %ymm15

	vpermpd			$ 0x00, %ymm4, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm4, %ymm4
	vfnmadd231pd	%ymm15, %ymm14, %ymm0

	vpermpd			$ 0x00, %ymm5, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm5, %ymm5
	vfnmadd231pd	%ymm15, %ymm14, %ymm1

	vpermpd			$ 0x00, %ymm6, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm6, %ymm6
	vfnmadd231pd	%ymm15, %ymm14, %ymm2

	vpermpd			$ 0x00, %ymm7, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm7, %ymm7
	vfnmadd231pd	%ymm15, %ymm14, %ymm3


	// top-left

	vmovapd			96(%r10), %ymm13
	vxorpd			%ymm14, %ymm14, %ymm14 // 0.0
	vblendpd		$ 0x7, %ymm13, %ymm14, %ymm13
	vbroadcastsd	24(%r12), %ymm12

	vpermpd			$ 0xff, %ymm0, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm0, %ymm0
	vfnmadd231pd	%ymm13, %ymm14, %ymm0

	vpermpd			$ 0xff, %ymm1, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm1, %ymm1
	vfnmadd231pd	%ymm13, %ymm14, %ymm1

	vpermpd			$ 0xff, %ymm2, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm2, %ymm2
	vfnmadd231pd	%ymm13, %ymm14, %ymm2

	vpermpd			$ 0xff, %ymm3, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm3, %ymm3
	vfnmadd231pd	%ymm13, %ymm14, %ymm3


	vxorpd			%ymm13, %ymm13, %ymm13 // 0.0
	vmovapd			64(%r10), %xmm13
	vbroadcastsd	16(%r12), %ymm12

	vpermpd			$ 0xaa, %ymm0, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm0, %ymm0
	vfnmadd231pd	%ymm13, %ymm14, %ymm0

	vpermpd			$ 0xaa, %ymm1, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm1, %ymm1
	vfnmadd231pd	%ymm13, %ymm14, %ymm1

	vpermpd			$ 0xaa, %ymm2, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm2, %ymm2
	vfnmadd231pd	%ymm13, %ymm14, %ymm2

	vpermpd			$ 0xaa, %ymm3, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm3, %ymm3
	vfnmadd231pd	%ymm13, %ymm14, %ymm3


	vxorpd			%ymm13, %ymm13, %ymm13 // 0.0
	vmovsd			32(%r10), %xmm13
	vbroadcastsd	8(%r12), %ymm12

	vpermilpd		$ 0xf, %ymm0, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm0, %ymm0
	vfnmadd231pd	%ymm13, %ymm14, %ymm0

	vpermilpd		$ 0xf, %ymm1, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm1, %ymm1
	vfnmadd231pd	%ymm13, %ymm14, %ymm1

	vpermilpd		$ 0xf, %ymm2, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm2, %ymm2
	vfnmadd231pd	%ymm13, %ymm14, %ymm2

	vpermilpd		$ 0xf, %ymm3, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm3, %ymm3
	vfnmadd231pd	%ymm13, %ymm14, %ymm3


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

	FUN_END(inner_edge_dtrsm_lun_inv_12x4_lib4)
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
// ymm8 <- [d80 d90 da0 db0]
// ymm9 <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]
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
// ymm8 <- [d80 d90 da0 db0]
// ymm9 <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_LUN_INV_12X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_lun_inv_12x4_vs_lib4)
#endif
	
	movq	%r10, %r15 // E1 <- E0
	addq	%r11, %r15 // E1 <- E0 + 4*sde*sizeof(double)
	movq	%r15, %r14 // E2 <- E1
	addq	%r11, %r14 // E2 <- E1 + 4*sde*sizeof(double)

	// bottom-right

	cmpl	$ 11, %r13d
	jle		0f

	vmovapd			352(%r14), %ymm13
	vxorpd			%ymm14, %ymm14, %ymm14 // 0.0
	vblendpd		$ 0x7, %ymm13, %ymm14, %ymm13
	vbroadcastsd	88(%r12), %ymm12
	vmovapd			352(%r15), %ymm15
//	vmovapd			352(%r10), %ymm11

	vpermpd			$ 0xff, %ymm8, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm8, %ymm8
	vfnmadd231pd	%ymm13, %ymm14, %ymm8
	vfnmadd231pd	%ymm15, %ymm14, %ymm4
	vfnmadd231pd	352(%r10), %ymm14, %ymm0

	vpermpd			$ 0xff, %ymm9, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm9, %ymm9
	vfnmadd231pd	%ymm13, %ymm14, %ymm9
	vfnmadd231pd	%ymm15, %ymm14, %ymm5
	vfnmadd231pd	352(%r10), %ymm14, %ymm1

	vpermpd			$ 0xff, %ymm10, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm10, %ymm10
	vfnmadd231pd	%ymm13, %ymm14, %ymm10
	vfnmadd231pd	%ymm15, %ymm14, %ymm6
	vfnmadd231pd	352(%r10), %ymm14, %ymm2

	vpermpd			$ 0xff, %ymm11, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm11, %ymm11
	vfnmadd231pd	%ymm13, %ymm14, %ymm11
	vfnmadd231pd	%ymm15, %ymm14, %ymm7
	vfnmadd231pd	352(%r10), %ymm14, %ymm3

0:
	cmpl	$ 10, %r13d
	jle		1f

	vxorpd			%ymm13, %ymm13, %ymm13 // 0.0 // ?????????????
	vmovapd			320(%r14), %xmm13
	vbroadcastsd	80(%r12), %ymm12
	vmovapd			320(%r15), %ymm15
//	vmovapd			320(%r10), %ymm11

	vpermpd			$ 0xaa, %ymm8, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm8, %ymm8
	vfnmadd231pd	%ymm13, %ymm14, %ymm8
	vfnmadd231pd	%ymm15, %ymm14, %ymm4
	vfnmadd231pd	320(%r10), %ymm14, %ymm0

	vpermpd			$ 0xaa, %ymm9, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm9, %ymm9
	vfnmadd231pd	%ymm13, %ymm14, %ymm9
	vfnmadd231pd	%ymm15, %ymm14, %ymm5
	vfnmadd231pd	320(%r10), %ymm14, %ymm1

	vpermpd			$ 0xaa, %ymm10, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm10, %ymm10
	vfnmadd231pd	%ymm13, %ymm14, %ymm10
	vfnmadd231pd	%ymm15, %ymm14, %ymm6
	vfnmadd231pd	320(%r10), %ymm14, %ymm2

	vpermpd			$ 0xaa, %ymm11, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm11, %ymm11
	vfnmadd231pd	%ymm13, %ymm14, %ymm11
	vfnmadd231pd	%ymm15, %ymm14, %ymm7
	vfnmadd231pd	320(%r10), %ymm14, %ymm3

1:
	cmpl	$ 9, %r13d
	jle		2f

	vxorpd			%ymm13, %ymm13, %ymm13 // 0.0
	vmovsd			288(%r14), %xmm13
	vbroadcastsd	72(%r12), %ymm12
	vmovapd			288(%r15), %ymm15
//	vmovapd			288(%r10), %ymm11

	vpermpd			$ 0x55, %ymm8, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm8, %ymm8
	vfnmadd231pd	%ymm13, %ymm14, %ymm8
	vfnmadd231pd	%ymm15, %ymm14, %ymm4
	vfnmadd231pd	288(%r10), %ymm14, %ymm0

	vpermpd			$ 0x55, %ymm9, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm9, %ymm9
	vfnmadd231pd	%ymm13, %ymm14, %ymm9
	vfnmadd231pd	%ymm15, %ymm14, %ymm5
	vfnmadd231pd	288(%r10), %ymm14, %ymm1

	vpermpd			$ 0x55, %ymm10, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm10, %ymm10
	vfnmadd231pd	%ymm13, %ymm14, %ymm10
	vfnmadd231pd	%ymm15, %ymm14, %ymm6
	vfnmadd231pd	288(%r10), %ymm14, %ymm2

	vpermpd			$ 0x55, %ymm11, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm11, %ymm11
	vfnmadd231pd	%ymm13, %ymm14, %ymm11
	vfnmadd231pd	%ymm15, %ymm14, %ymm7
	vfnmadd231pd	288(%r10), %ymm14, %ymm3

2:

	vbroadcastsd	64(%r12), %ymm12
	vmovapd			256(%r15), %ymm15
//	vmovapd			256(%r10), %ymm11

	vpermpd			$ 0x00, %ymm8, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm8, %ymm8
	vfnmadd231pd	%ymm15, %ymm14, %ymm4
	vfnmadd231pd	256(%r10), %ymm14, %ymm0

	vpermpd			$ 0x00, %ymm9, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm9, %ymm9
	vfnmadd231pd	%ymm15, %ymm14, %ymm5
	vfnmadd231pd	256(%r10), %ymm14, %ymm1

	vpermpd			$ 0x00, %ymm10, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm10, %ymm10
	vfnmadd231pd	%ymm15, %ymm14, %ymm6
	vfnmadd231pd	256(%r10), %ymm14, %ymm2

	vpermpd			$ 0x00, %ymm11, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm11, %ymm11
	vfnmadd231pd	%ymm15, %ymm14, %ymm7
	vfnmadd231pd	256(%r10), %ymm14, %ymm3


	// middle-middle

	vmovapd			224(%r15), %ymm13
	vxorpd			%ymm14, %ymm14, %ymm14 // 0.0
	vblendpd		$ 0x7, %ymm13, %ymm14, %ymm13
	vbroadcastsd	56(%r12), %ymm12
	vmovapd			224(%r10), %ymm15

	vpermpd			$ 0xff, %ymm4, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm4, %ymm4
	vfnmadd231pd	%ymm13, %ymm14, %ymm4
	vfnmadd231pd	%ymm15, %ymm14, %ymm0

	vpermpd			$ 0xff, %ymm5, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm5, %ymm5
	vfnmadd231pd	%ymm13, %ymm14, %ymm5
	vfnmadd231pd	%ymm15, %ymm14, %ymm1

	vpermpd			$ 0xff, %ymm6, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm6, %ymm6
	vfnmadd231pd	%ymm13, %ymm14, %ymm6
	vfnmadd231pd	%ymm15, %ymm14, %ymm2

	vpermpd			$ 0xff, %ymm7, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm7, %ymm7
	vfnmadd231pd	%ymm13, %ymm14, %ymm7
	vfnmadd231pd	%ymm15, %ymm14, %ymm3


	vxorpd			%ymm13, %ymm13, %ymm13 // 0.0 // ?????????????
	vmovapd			192(%r15), %xmm13
	vbroadcastsd	48(%r12), %ymm12
	vmovapd			192(%r10), %ymm15

	vpermpd			$ 0xaa, %ymm4, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm4, %ymm4
	vfnmadd231pd	%ymm13, %ymm14, %ymm4
	vfnmadd231pd	%ymm15, %ymm14, %ymm0

	vpermpd			$ 0xaa, %ymm5, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm5, %ymm5
	vfnmadd231pd	%ymm13, %ymm14, %ymm5
	vfnmadd231pd	%ymm15, %ymm14, %ymm1

	vpermpd			$ 0xaa, %ymm6, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm6, %ymm6
	vfnmadd231pd	%ymm13, %ymm14, %ymm6
	vfnmadd231pd	%ymm15, %ymm14, %ymm2

	vpermpd			$ 0xaa, %ymm7, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm7, %ymm7
	vfnmadd231pd	%ymm13, %ymm14, %ymm7
	vfnmadd231pd	%ymm15, %ymm14, %ymm3


	vxorpd			%ymm13, %ymm13, %ymm13 // 0.0
	vmovsd			160(%r15), %xmm13
	vbroadcastsd	40(%r12), %ymm12
	vmovapd			160(%r10), %ymm15

	vpermpd			$ 0x55, %ymm4, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm4, %ymm4
	vfnmadd231pd	%ymm13, %ymm14, %ymm4
	vfnmadd231pd	%ymm15, %ymm14, %ymm0

	vpermpd			$ 0x55, %ymm5, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm5, %ymm5
	vfnmadd231pd	%ymm13, %ymm14, %ymm5
	vfnmadd231pd	%ymm15, %ymm14, %ymm1

	vpermpd			$ 0x55, %ymm6, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm6, %ymm6
	vfnmadd231pd	%ymm13, %ymm14, %ymm6
	vfnmadd231pd	%ymm15, %ymm14, %ymm2

	vpermpd			$ 0x55, %ymm7, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm7, %ymm7
	vfnmadd231pd	%ymm13, %ymm14, %ymm7
	vfnmadd231pd	%ymm15, %ymm14, %ymm3


	vbroadcastsd	32(%r12), %ymm12
	vmovapd			128(%r10), %ymm15

	vpermpd			$ 0x00, %ymm4, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm4, %ymm4
	vfnmadd231pd	%ymm15, %ymm14, %ymm0

	vpermpd			$ 0x00, %ymm5, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm5, %ymm5
	vfnmadd231pd	%ymm15, %ymm14, %ymm1

	vpermpd			$ 0x00, %ymm6, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm6, %ymm6
	vfnmadd231pd	%ymm15, %ymm14, %ymm2

	vpermpd			$ 0x00, %ymm7, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm7, %ymm7
	vfnmadd231pd	%ymm15, %ymm14, %ymm3


	// top-left

	vmovapd			96(%r10), %ymm13
	vxorpd			%ymm14, %ymm14, %ymm14 // 0.0
	vblendpd		$ 0x7, %ymm13, %ymm14, %ymm13
	vbroadcastsd	24(%r12), %ymm12

	vpermpd			$ 0xff, %ymm0, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm0, %ymm0
	vfnmadd231pd	%ymm13, %ymm14, %ymm0

	vpermpd			$ 0xff, %ymm1, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm1, %ymm1
	vfnmadd231pd	%ymm13, %ymm14, %ymm1

	vpermpd			$ 0xff, %ymm2, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm2, %ymm2
	vfnmadd231pd	%ymm13, %ymm14, %ymm2

	vpermpd			$ 0xff, %ymm3, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm3, %ymm3
	vfnmadd231pd	%ymm13, %ymm14, %ymm3


	vxorpd			%ymm13, %ymm13, %ymm13 // 0.0
	vmovapd			64(%r10), %xmm13
	vbroadcastsd	16(%r12), %ymm12

	vpermpd			$ 0xaa, %ymm0, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm0, %ymm0
	vfnmadd231pd	%ymm13, %ymm14, %ymm0

	vpermpd			$ 0xaa, %ymm1, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm1, %ymm1
	vfnmadd231pd	%ymm13, %ymm14, %ymm1

	vpermpd			$ 0xaa, %ymm2, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm2, %ymm2
	vfnmadd231pd	%ymm13, %ymm14, %ymm2

	vpermpd			$ 0xaa, %ymm3, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm3, %ymm3
	vfnmadd231pd	%ymm13, %ymm14, %ymm3


	vxorpd			%ymm13, %ymm13, %ymm13 // 0.0
	vmovsd			32(%r10), %xmm13
	vbroadcastsd	8(%r12), %ymm12

	vpermilpd		$ 0xf, %ymm0, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm0, %ymm0
	vfnmadd231pd	%ymm13, %ymm14, %ymm0

	vpermilpd		$ 0xf, %ymm1, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm1, %ymm1
	vfnmadd231pd	%ymm13, %ymm14, %ymm1

	vpermilpd		$ 0xf, %ymm2, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm2, %ymm2
	vfnmadd231pd	%ymm13, %ymm14, %ymm2

	vpermilpd		$ 0xf, %ymm3, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm3, %ymm3
	vfnmadd231pd	%ymm13, %ymm14, %ymm3


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

	FUN_END(inner_edge_dtrsm_lun_inv_12x4_vs_lib4)
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
// ymm8 <- [d80 d90 da0 db0]
// ymm9 <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]
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
// ymm8 <- [d80 d90 da0 db0]
// ymm9 <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DGETRF_L_12X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dgetrf_l_12x4_lib4)
#endif
	
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC05(%rip), %xmm14 // 1.0
#elif defined(OS_MAC)
	vmovsd			LC05(%rip), %xmm14 // 1.0
#endif
//	vmovddup		%xmm14, %xmm14

	// first column
//	vblendpd		$ 0x1, %ymm0, %ymm12, %ymm12
	vmovapd			%ymm0, %ymm12
	vdivsd			%xmm0, %xmm14, %xmm13
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmovsd			%xmm13, 0(%r10)
	vmulpd			%ymm0, %ymm13, %ymm0
	vmulpd			%ymm4, %ymm13, %ymm4
	vmulpd			%ymm8, %ymm13, %ymm8
	vblendpd		$ 0x1, %ymm12, %ymm0, %ymm0

	// second column
	vpermpd			$ 0x00, %ymm1, %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm1
	vfnmadd231pd	%ymm4, %ymm13, %ymm5
	vfnmadd231pd	%ymm8, %ymm13, %ymm9
	vblendpd		$ 0x2, %ymm1, %ymm13, %ymm12

	vpermilpd		$ 0x3, %xmm1, %xmm13
	vdivsd			%xmm13, %xmm14, %xmm13
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmovsd			%xmm13, 8(%r10)
	vmulpd			%ymm1, %ymm13, %ymm1
	vmulpd			%ymm5, %ymm13, %ymm5
	vmulpd			%ymm9, %ymm13, %ymm9
	vblendpd		$ 0x3, %ymm12, %ymm1, %ymm1

	// third column
	vpermpd			$ 0x00, %ymm2, %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm2
	vfnmadd231pd	%ymm4, %ymm13, %ymm6
	vfnmadd231pd	%ymm8, %ymm13, %ymm10
	vblendpd		$ 0x2, %ymm2, %ymm13, %ymm12

	vpermpd			$ 0x55, %ymm2, %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm2
	vfnmadd231pd	%ymm5, %ymm13, %ymm6
	vfnmadd231pd	%ymm9, %ymm13, %ymm10
	vblendpd		$ 0x4, %ymm2, %ymm12, %ymm12

	vpermpd			$ 0xaa, %ymm2, %ymm13
	vdivsd			%xmm13, %xmm14, %xmm13
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmovsd			%xmm13, 16(%r10)
	vmulpd			%ymm2, %ymm13, %ymm2
	vmulpd			%ymm6, %ymm13, %ymm6
	vmulpd			%ymm10, %ymm13, %ymm10
	vblendpd		$ 0x7, %ymm12, %ymm2, %ymm2

	// fourth column
	vpermpd			$ 0x00, %ymm3, %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm3
	vfnmadd231pd	%ymm4, %ymm13, %ymm7
	vfnmadd231pd	%ymm8, %ymm13, %ymm11
	vblendpd		$ 0x2, %ymm3, %ymm13, %ymm12

	vpermpd			$ 0x55, %ymm3, %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm3
	vfnmadd231pd	%ymm5, %ymm13, %ymm7
	vfnmadd231pd	%ymm9, %ymm13, %ymm11
	vblendpd		$ 0x4, %ymm3, %ymm12, %ymm12

	vpermpd			$ 0xaa, %ymm3, %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm3
	vfnmadd231pd	%ymm6, %ymm13, %ymm7
	vfnmadd231pd	%ymm10, %ymm13, %ymm11
	vblendpd		$ 0x8, %ymm3, %ymm12, %ymm12
	
	vpermpd			$ 0xff, %ymm3, %ymm13
	vdivsd			%xmm13, %xmm14, %xmm13
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmovsd			%xmm13, 24(%r10)
//	vmulpd			%ymm3, %ymm13, %ymm3
	vmulpd			%ymm7, %ymm13, %ymm7
	vmulpd			%ymm11, %ymm13, %ymm11
	vblendpd		$ 0x7, %ymm12, %ymm3, %ymm3

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dgetrf_l_12x4_lib4)
#endif





// common inner routine with file scope
//
// LU factorization without pivoting
// middle kernel
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
// ymm8 <- [d80 d90 da0 db0]
// ymm9 <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty
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
// ymm8 <- [d80 d90 da0 db0]
// ymm9 <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DGETRF_M_12X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dgetrf_m_12x4_lib4)
#endif
	
	// solve upper 4x4 & correct lower 8x4

	vxorpd			%ymm15, %ymm15, %ymm15
	vmovapd			0(%r10), %ymm12
	vblendpd		$ 0x1, %ymm15, %ymm12, %ymm12
	vmovapd			0(%r10, %r11), %ymm14
	vmovapd			0(%r10, %r11, 2), %ymm15
	vpermpd			$ 0x00, %ymm0, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm0
	vfnmadd231pd	%ymm14, %ymm13, %ymm4
	vfnmadd231pd	%ymm15, %ymm13, %ymm8
	vpermpd			$ 0x00, %ymm1, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm1
	vfnmadd231pd	%ymm14, %ymm13, %ymm5
	vfnmadd231pd	%ymm15, %ymm13, %ymm9
	vpermpd			$ 0x00, %ymm2, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm2
	vfnmadd231pd	%ymm14, %ymm13, %ymm6
	vfnmadd231pd	%ymm15, %ymm13, %ymm10
	vpermpd			$ 0x00, %ymm3, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm3
	vfnmadd231pd	%ymm14, %ymm13, %ymm7
	vfnmadd231pd	%ymm15, %ymm13, %ymm11

	vxorpd			%ymm15, %ymm15, %ymm15
	vmovapd			32(%r10), %ymm12
	vblendpd		$ 0x3, %ymm15, %ymm12, %ymm12
	vmovapd			32(%r10, %r11), %ymm14
	vmovapd			32(%r10, %r11, 2), %ymm15
	vpermpd			$ 0x55, %ymm0, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm0
	vfnmadd231pd	%ymm14, %ymm13, %ymm4
	vfnmadd231pd	%ymm15, %ymm13, %ymm8
	vpermpd			$ 0x55, %ymm1, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm1
	vfnmadd231pd	%ymm14, %ymm13, %ymm5
	vfnmadd231pd	%ymm15, %ymm13, %ymm9
	vpermpd			$ 0x55, %ymm2, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm2
	vfnmadd231pd	%ymm14, %ymm13, %ymm6
	vfnmadd231pd	%ymm15, %ymm13, %ymm10
	vpermpd			$ 0x55, %ymm3, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm3
	vfnmadd231pd	%ymm14, %ymm13, %ymm7
	vfnmadd231pd	%ymm15, %ymm13, %ymm11

	vxorpd			%ymm15, %ymm15, %ymm15
	vmovapd			64(%r10), %ymm12
	vblendpd		$ 0x7, %ymm15, %ymm12, %ymm12
	vmovapd			64(%r10, %r11), %ymm14
	vmovapd			64(%r10, %r11, 2), %ymm15
	vpermpd			$ 0xaa, %ymm0, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm0
	vfnmadd231pd	%ymm14, %ymm13, %ymm4
	vfnmadd231pd	%ymm15, %ymm13, %ymm8
	vpermpd			$ 0xaa, %ymm1, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm1
	vfnmadd231pd	%ymm14, %ymm13, %ymm5
	vfnmadd231pd	%ymm15, %ymm13, %ymm9
	vpermpd			$ 0xaa, %ymm2, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm2
	vfnmadd231pd	%ymm14, %ymm13, %ymm6
	vfnmadd231pd	%ymm15, %ymm13, %ymm10
	vpermpd			$ 0xaa, %ymm3, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm3
	vfnmadd231pd	%ymm14, %ymm13, %ymm7
	vfnmadd231pd	%ymm15, %ymm13, %ymm11

	vmovapd			96(%r10, %r11), %ymm14
	vmovapd			96(%r10, %r11, 2), %ymm15
	vpermpd			$ 0xff, %ymm0, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm4
	vfnmadd231pd	%ymm15, %ymm13, %ymm8
	vpermpd			$ 0xff, %ymm1, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm5
	vfnmadd231pd	%ymm15, %ymm13, %ymm9
	vpermpd			$ 0xff, %ymm2, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm6
	vfnmadd231pd	%ymm15, %ymm13, %ymm10
	vpermpd			$ 0xff, %ymm3, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm7
	vfnmadd231pd	%ymm15, %ymm13, %ymm11


	// factorize lower 8x4

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC05(%rip), %xmm14 // 1.0
#elif defined(OS_MAC)
	vmovsd			LC05(%rip), %xmm14 // 1.0
#endif
//	vmovddup		%xmm14, %xmm14

	// first column
//	vblendpd		$ 0x1, %ymm4, %ymm12, %ymm12
	vmovapd			%ymm4, %ymm12
	vdivsd			%xmm4, %xmm14, %xmm13
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmovsd			%xmm13, 0(%r12)
	vmulpd			%ymm4, %ymm13, %ymm4
	vmulpd			%ymm8, %ymm13, %ymm8
	vblendpd		$ 0x1, %ymm12, %ymm4, %ymm4

	// second column
	vpermpd			$ 0x00, %ymm5, %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm5
	vfnmadd231pd	%ymm8, %ymm13, %ymm9
	vblendpd		$ 0x2, %ymm5, %ymm13, %ymm12

	vpermilpd		$ 0x3, %xmm5, %xmm13
	vdivsd			%xmm13, %xmm14, %xmm13
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmovsd			%xmm13, 8(%r12)
	vmulpd			%ymm5, %ymm13, %ymm5
	vmulpd			%ymm9, %ymm13, %ymm9
	vblendpd		$ 0x3, %ymm12, %ymm5, %ymm5

	// third column
	vpermpd			$ 0x00, %ymm6, %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm6
	vfnmadd231pd	%ymm8, %ymm13, %ymm10
	vblendpd		$ 0x2, %ymm6, %ymm13, %ymm12

	vpermpd			$ 0x55, %ymm6, %ymm13
	vfnmadd231pd	%ymm5, %ymm13, %ymm6
	vfnmadd231pd	%ymm9, %ymm13, %ymm10
	vblendpd		$ 0x4, %ymm6, %ymm12, %ymm12

	vpermpd			$ 0xaa, %ymm6, %ymm13
	vdivsd			%xmm13, %xmm14, %xmm13
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmovsd			%xmm13, 16(%r12)
	vmulpd			%ymm6, %ymm13, %ymm6
	vmulpd			%ymm10, %ymm13, %ymm10
	vblendpd		$ 0x7, %ymm12, %ymm6, %ymm6

	// fourth column
	vpermpd			$ 0x00, %ymm7, %ymm13
	vfnmadd231pd	%ymm4, %ymm13, %ymm7
	vfnmadd231pd	%ymm8, %ymm13, %ymm11
	vblendpd		$ 0x2, %ymm7, %ymm13, %ymm12

	vpermpd			$ 0x55, %ymm7, %ymm13
	vfnmadd231pd	%ymm5, %ymm13, %ymm7
	vfnmadd231pd	%ymm9, %ymm13, %ymm11
	vblendpd		$ 0x4, %ymm7, %ymm12, %ymm12

	vpermpd			$ 0xaa, %ymm7, %ymm13
	vfnmadd231pd	%ymm6, %ymm13, %ymm7
	vfnmadd231pd	%ymm10, %ymm13, %ymm11
	vblendpd		$ 0x8, %ymm7, %ymm12, %ymm12
	
	vpermpd			$ 0xff, %ymm7, %ymm13
	vdivsd			%xmm13, %xmm14, %xmm13
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmovsd			%xmm13, 24(%r12)
//	vmulpd			%ymm7, %ymm13, %ymm7
	vmulpd			%ymm11, %ymm13, %ymm11
	vblendpd		$ 0x7, %ymm12, %ymm7, %ymm7

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dgetrf_m_12x4_lib4)
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
// ymm8 <- [d80 d90 da0 db0]
// ymm9 <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty
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
// ymm8 <- [d80 d90 da0 db0]
// ymm9 <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DGETRF_R_12X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dgetrf_r_12x4_lib4)
#endif
	
	// solve upper 8x4 & correct lower 4x4

	vxorpd			%ymm15, %ymm15, %ymm15
	vmovapd			0(%r10), %ymm12
	vblendpd		$ 0x1, %ymm15, %ymm12, %ymm12
	vmovapd			0(%r10, %r11), %ymm14
	vmovapd			0(%r10, %r11, 2), %ymm15
	vpermpd			$ 0x00, %ymm0, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm0
	vfnmadd231pd	%ymm14, %ymm13, %ymm4
	vfnmadd231pd	%ymm15, %ymm13, %ymm8
	vpermpd			$ 0x00, %ymm1, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm1
	vfnmadd231pd	%ymm14, %ymm13, %ymm5
	vfnmadd231pd	%ymm15, %ymm13, %ymm9
	vpermpd			$ 0x00, %ymm2, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm2
	vfnmadd231pd	%ymm14, %ymm13, %ymm6
	vfnmadd231pd	%ymm15, %ymm13, %ymm10
	vpermpd			$ 0x00, %ymm3, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm3
	vfnmadd231pd	%ymm14, %ymm13, %ymm7
	vfnmadd231pd	%ymm15, %ymm13, %ymm11

	vxorpd			%ymm15, %ymm15, %ymm15
	vmovapd			32(%r10), %ymm12
	vblendpd		$ 0x3, %ymm15, %ymm12, %ymm12
	vmovapd			32(%r10, %r11), %ymm14
	vmovapd			32(%r10, %r11, 2), %ymm15
	vpermpd			$ 0x55, %ymm0, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm0
	vfnmadd231pd	%ymm14, %ymm13, %ymm4
	vfnmadd231pd	%ymm15, %ymm13, %ymm8
	vpermpd			$ 0x55, %ymm1, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm1
	vfnmadd231pd	%ymm14, %ymm13, %ymm5
	vfnmadd231pd	%ymm15, %ymm13, %ymm9
	vpermpd			$ 0x55, %ymm2, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm2
	vfnmadd231pd	%ymm14, %ymm13, %ymm6
	vfnmadd231pd	%ymm15, %ymm13, %ymm10
	vpermpd			$ 0x55, %ymm3, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm3
	vfnmadd231pd	%ymm14, %ymm13, %ymm7
	vfnmadd231pd	%ymm15, %ymm13, %ymm11

	vxorpd			%ymm15, %ymm15, %ymm15
	vmovapd			64(%r10), %ymm12
	vblendpd		$ 0x7, %ymm15, %ymm12, %ymm12
	vmovapd			64(%r10, %r11), %ymm14
	vmovapd			64(%r10, %r11, 2), %ymm15
	vpermpd			$ 0xaa, %ymm0, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm0
	vfnmadd231pd	%ymm14, %ymm13, %ymm4
	vfnmadd231pd	%ymm15, %ymm13, %ymm8
	vpermpd			$ 0xaa, %ymm1, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm1
	vfnmadd231pd	%ymm14, %ymm13, %ymm5
	vfnmadd231pd	%ymm15, %ymm13, %ymm9
	vpermpd			$ 0xaa, %ymm2, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm2
	vfnmadd231pd	%ymm14, %ymm13, %ymm6
	vfnmadd231pd	%ymm15, %ymm13, %ymm10
	vpermpd			$ 0xaa, %ymm3, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm3
	vfnmadd231pd	%ymm14, %ymm13, %ymm7
	vfnmadd231pd	%ymm15, %ymm13, %ymm11

	vmovapd			96(%r10, %r11), %ymm14
	vmovapd			96(%r10, %r11, 2), %ymm15
	vpermpd			$ 0xff, %ymm0, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm4
	vfnmadd231pd	%ymm15, %ymm13, %ymm8
	vpermpd			$ 0xff, %ymm1, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm5
	vfnmadd231pd	%ymm15, %ymm13, %ymm9
	vpermpd			$ 0xff, %ymm2, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm6
	vfnmadd231pd	%ymm15, %ymm13, %ymm10
	vpermpd			$ 0xff, %ymm3, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm7
	vfnmadd231pd	%ymm15, %ymm13, %ymm11

	addq		$ 128, %r10


	vxorpd			%ymm14, %ymm14, %ymm14
	vmovapd			0(%r10, %r11), %ymm12
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm12
	vmovapd			0(%r10, %r11, 2), %ymm14
	vpermpd			$ 0x00, %ymm4, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm4
	vfnmadd231pd	%ymm14, %ymm13, %ymm8
	vpermpd			$ 0x00, %ymm5, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm5
	vfnmadd231pd	%ymm14, %ymm13, %ymm9
	vpermpd			$ 0x00, %ymm6, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm6
	vfnmadd231pd	%ymm14, %ymm13, %ymm10
	vpermpd			$ 0x00, %ymm7, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm7
	vfnmadd231pd	%ymm14, %ymm13, %ymm11

	vxorpd			%ymm14, %ymm14, %ymm14
	vmovapd			32(%r10, %r11), %ymm12
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm12
	vmovapd			32(%r10, %r11, 2), %ymm14
	vpermpd			$ 0x55, %ymm4, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm4
	vfnmadd231pd	%ymm14, %ymm13, %ymm8
	vpermpd			$ 0x55, %ymm5, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm5
	vfnmadd231pd	%ymm14, %ymm13, %ymm9
	vpermpd			$ 0x55, %ymm6, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm6
	vfnmadd231pd	%ymm14, %ymm13, %ymm10
	vpermpd			$ 0x55, %ymm7, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm7
	vfnmadd231pd	%ymm14, %ymm13, %ymm11

	vxorpd			%ymm14, %ymm14, %ymm14
	vmovapd			64(%r10, %r11), %ymm12
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm12
	vmovapd			64(%r10, %r11, 2), %ymm14
	vpermpd			$ 0xaa, %ymm4, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm4
	vfnmadd231pd	%ymm14, %ymm13, %ymm8
	vpermpd			$ 0xaa, %ymm5, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm5
	vfnmadd231pd	%ymm14, %ymm13, %ymm9
	vpermpd			$ 0xaa, %ymm6, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm6
	vfnmadd231pd	%ymm14, %ymm13, %ymm10
	vpermpd			$ 0xaa, %ymm7, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm7
	vfnmadd231pd	%ymm14, %ymm13, %ymm11

	vmovapd			96(%r10, %r11, 2), %ymm14
	vpermpd			$ 0xff, %ymm4, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm8
	vpermpd			$ 0xff, %ymm5, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm9
	vpermpd			$ 0xff, %ymm6, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm10
	vpermpd			$ 0xff, %ymm7, %ymm13
	vfnmadd231pd	%ymm14, %ymm13, %ymm11



	// factorize lower 8x4

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC05(%rip), %xmm14 // 1.0
#elif defined(OS_MAC)
	vmovsd			LC05(%rip), %xmm14 // 1.0
#endif
//	vmovddup		%xmm14, %xmm14

	// first column
//	vblendpd		$ 0x1, %ymm8, %ymm12, %ymm12
	vmovapd			%ymm8, %ymm12
	vdivsd			%xmm8, %xmm14, %xmm13
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmovsd			%xmm13, 0(%r12)
	vmulpd			%ymm8, %ymm13, %ymm8
	vblendpd		$ 0x1, %ymm12, %ymm8, %ymm8

	// second column
	vpermpd			$ 0x00, %ymm9, %ymm13
	vfnmadd231pd	%ymm8, %ymm13, %ymm9
	vblendpd		$ 0x2, %ymm9, %ymm13, %ymm12

	vpermilpd		$ 0x3, %xmm9, %xmm13
	vdivsd			%xmm13, %xmm14, %xmm13
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmovsd			%xmm13, 8(%r12)
	vmulpd			%ymm9, %ymm13, %ymm9
	vblendpd		$ 0x3, %ymm12, %ymm9, %ymm9

	// third column
	vpermpd			$ 0x00, %ymm10, %ymm13
	vfnmadd231pd	%ymm8, %ymm13, %ymm10
	vblendpd		$ 0x2, %ymm10, %ymm13, %ymm12

	vpermpd			$ 0x55, %ymm10, %ymm13
	vfnmadd231pd	%ymm9, %ymm13, %ymm10
	vblendpd		$ 0x4, %ymm10, %ymm12, %ymm12

	vpermpd			$ 0xaa, %ymm10, %ymm13
	vdivsd			%xmm13, %xmm14, %xmm13
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmovsd			%xmm13, 16(%r12)
	vmulpd			%ymm10, %ymm13, %ymm10
	vblendpd		$ 0x7, %ymm12, %ymm10, %ymm10

	// fourth column
	vpermpd			$ 0x00, %ymm11, %ymm13
	vfnmadd231pd	%ymm8, %ymm13, %ymm11
	vblendpd		$ 0x2, %ymm11, %ymm13, %ymm12

	vpermpd			$ 0x55, %ymm11, %ymm13
	vfnmadd231pd	%ymm9, %ymm13, %ymm11
	vblendpd		$ 0x4, %ymm11, %ymm12, %ymm12

	vpermpd			$ 0xaa, %ymm11, %ymm13
	vfnmadd231pd	%ymm10, %ymm13, %ymm11
	vblendpd		$ 0x8, %ymm11, %ymm12, %ymm12
	
	vpermpd			$ 0xff, %ymm11, %ymm13
	vdivsd			%xmm13, %xmm14, %xmm13
//	vpermpd			$ 0x00, %ymm13, %ymm13
	vmovsd			%xmm13, 24(%r12)
//	vmulpd			%ymm11, %ymm13, %ymm11
	vblendpd		$ 0x7, %ymm12, %ymm11, %ymm11

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dgetrf_r_12x4_lib4)
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

	vmovapd %ymm4,  0(%r10, %r11)
	vmovapd %ymm5, 32(%r10, %r11)
	vmovapd %ymm6, 64(%r10, %r11)
	vmovapd %ymm7, 96(%r10, %r11)

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
	.macro INNER_STORE_4X12_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_4x12_lib4)
#endif
	
	vmovapd %ymm0,   0(%r10)
	vmovapd %ymm1,  32(%r10)
	vmovapd %ymm2,  64(%r10)
	vmovapd %ymm3,  96(%r10)

	vmovapd %ymm4, 128(%r10)
	vmovapd %ymm5, 160(%r10)
	vmovapd %ymm6, 192(%r10)
	vmovapd %ymm7, 224(%r10)

	vmovapd %ymm8, 256(%r10)
	vmovapd %ymm9, 288(%r10)
	vmovapd %ymm10, 320(%r10)
	vmovapd %ymm11, 352(%r10)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4x12_lib4)
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
// ymm14 <- dirty
// ymm15 <- dirty
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
// ymm14 <- dirty
// ymm15 <- dirty

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
	vmovapd		%ymm4, 0(%r10, %r11)
	vmaskmovpd	%ymm8, %ymm15,  0(%r10, %r11, 2)
	cmpl		$ 2, %r13d
	jl			0f // end
	vmovapd		%ymm1, 32(%r10)
	vmovapd		%ymm5, 32(%r10, %r11)
	vmaskmovpd	%ymm9, %ymm15, 32(%r10, %r11, 2)
	cmpl		$ 3, %r13d
	jl			0f // end
	vmovapd		%ymm2, 64(%r10)
	vmovapd		%ymm6, 64(%r10, %r11)
	vmaskmovpd	%ymm10, %ymm15, 64(%r10, %r11, 2)
	je			0f // end
	vmovapd		%ymm3, 96(%r10)
	vmovapd		%ymm7, 96(%r10, %r11)
	vmaskmovpd	%ymm11, %ymm15, 96(%r10, %r11, 2)

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_12x4_vs_lib4)
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
	.macro INNER_STORE_4X12_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_4x12_vs_lib4)
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
	vmaskmovpd	%ymm5, %ymm15, 160(%r10)
	vmaskmovpd	%ymm6, %ymm15, 192(%r10)
	vmaskmovpd	%ymm7, %ymm15, 224(%r10)

	vmaskmovpd	%ymm8, %ymm15, 256(%r10)
	cmpl		$ 10, %r12d
	jl			0f // end
	vmaskmovpd	%ymm9, %ymm15, 288(%r10)
	cmpl		$ 11, %r12d
	jl			0f // end
	vmaskmovpd	%ymm10, %ymm15, 320(%r10)
	je			0f // end
	vmaskmovpd	%ymm11, %ymm15, 352(%r10)

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4x12_vs_lib4)
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
// ymm8  <- [d80 d90 da0 db0]
// ymm9  <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]
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
// ymm8  <- [d80 d90 da0 db0]
// ymm9  <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]

#if MACRO_LEVEL>=1
	.macro INNER_STORE_12X4_GEN_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_12x4_gen_lib4)
#endif
	
	// compute mask for rows
	vcvtsi2sd	%r13d, %xmm14, %xmm14
	vcvtsi2sd	%r14d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC02(%rip), %ymm12
	vmovupd		.LC04(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC02(%rip), %ymm12
	vmovupd		LC04(%rip), %ymm13
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
	vmovapd		%ymm9, %ymm8
	vmovapd		%ymm2, %ymm1
	vmovapd		%ymm6, %ymm5
	vmovapd		%ymm10, %ymm9
	vmovapd		%ymm3, %ymm2
	vmovapd		%ymm7, %ymm6
	vmovapd		%ymm11, %ymm10
	addq		$ 32, %r11

	cmpl	$ 1, %r15d
	jle		0f

	vmovapd		%ymm1, %ymm0
	vmovapd		%ymm5, %ymm4
	vmovapd		%ymm9, %ymm8
	vmovapd		%ymm2, %ymm1
	vmovapd		%ymm6, %ymm5
	vmovapd		%ymm10, %ymm9
	addq		$ 32, %r11

	cmpl	$ 2, %r15d
	jle		0f

	vmovapd		%ymm1, %ymm0
	vmovapd		%ymm5, %ymm4
	vmovapd		%ymm9, %ymm8
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

	vmaskmovpd	%ymm0, %ymm14,  0(%r11)
	vmovapd		%ymm4, 0(%r11, %r12)
	vmaskmovpd	%ymm8, %ymm15,  0(%r11, %r12, 2)
	cmpl		$ 2, %r15d
	jl			4f // end
	vmaskmovpd	%ymm1, %ymm14, 32(%r11)
	vmovapd		%ymm5, 32(%r11, %r12)
	vmaskmovpd	%ymm9, %ymm15, 32(%r11, %r12, 2)
	cmpl		$ 3, %r15d
	jl			4f // end
	vmaskmovpd	%ymm2, %ymm14, 64(%r11)
	vmovapd		%ymm6, 64(%r11, %r12)
	vmaskmovpd	%ymm10, %ymm15, 64(%r11, %r12, 2)
	je			4f // end
	vmaskmovpd	%ymm3, %ymm14, 96(%r11)
	vmovapd		%ymm7, 96(%r11, %r12)
	vmaskmovpd	%ymm11, %ymm15, 96(%r11, %r12, 2)

	jmp		4f

0:
	
	cmpl	$ 1, %r10d
	jg		1f

	// offset==1

	vmovapd		%ymm0, %ymm13
	vperm2f128	$ 0x03, %ymm8, %ymm0, %ymm12
	vshufpd		$ 0x5, %ymm0, %ymm12, %ymm0
	vperm2f128	$ 0x03, %ymm4, %ymm8, %ymm12
	vshufpd		$ 0x5, %ymm8, %ymm12, %ymm8
	vperm2f128	$ 0x03, %ymm13, %ymm4, %ymm12
	vshufpd		$ 0x5, %ymm4, %ymm12, %ymm4

	vmovapd		%ymm1, %ymm13
	vperm2f128	$ 0x03, %ymm9, %ymm1, %ymm12
	vshufpd		$ 0x5, %ymm1, %ymm12, %ymm1
	vperm2f128	$ 0x03, %ymm5, %ymm9, %ymm12
	vshufpd		$ 0x5, %ymm9, %ymm12, %ymm9
	vperm2f128	$ 0x03, %ymm13, %ymm5, %ymm12
	vshufpd		$ 0x5, %ymm5, %ymm12, %ymm5

	vmovapd		%ymm2, %ymm13
	vperm2f128	$ 0x03, %ymm10, %ymm2, %ymm12
	vshufpd		$ 0x5, %ymm2, %ymm12, %ymm2
	vperm2f128	$ 0x03, %ymm6, %ymm10, %ymm12
	vshufpd		$ 0x5, %ymm10, %ymm12, %ymm10
	vperm2f128	$ 0x03, %ymm13, %ymm6, %ymm12
	vshufpd		$ 0x5, %ymm6, %ymm12, %ymm6

	vmovapd		%ymm3, %ymm13
	vperm2f128	$ 0x03, %ymm11, %ymm3, %ymm12
	vshufpd		$ 0x5, %ymm3, %ymm12, %ymm3
	vperm2f128	$ 0x03, %ymm7, %ymm11, %ymm12
	vshufpd		$ 0x5, %ymm11, %ymm12, %ymm11
	vperm2f128	$ 0x03, %ymm13, %ymm7, %ymm12
	vshufpd		$ 0x5, %ymm7, %ymm12, %ymm7

	vperm2f128	$ 0x01, %ymm15, %ymm15, %ymm12
	vshufpd		$ 0x5, %ymm15, %ymm12, %ymm15
	vperm2f128	$ 0x01, %ymm14, %ymm14, %ymm12
	vshufpd		$ 0x5, %ymm14, %ymm12, %ymm14

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC10(%rip), %ymm12
	vmovupd		.LC06(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC10(%rip), %ymm12
	vmovupd		LC06(%rip), %ymm13
#endif
	vandpd		%ymm12, %ymm14, %ymm12
	vandpd		%ymm13, %ymm15, %ymm13

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vblendpd	$ 0xe, .LC09(%rip), %ymm14, %ymm14
	vblendpd	$ 0x1, .LC09(%rip), %ymm15, %ymm15
#elif defined(OS_MAC)
	vblendpd	$ 0xe, LC09(%rip), %ymm14, %ymm14
	vblendpd	$ 0x1, LC09(%rip), %ymm15, %ymm15
#endif


	jmp		3f

1:

	cmpl	$ 2, %r10d
	jg		2f

	// offset==2

	vmovapd		%ymm0, %ymm13
	vperm2f128	$ 0x03, %ymm8, %ymm0, %ymm0
	vperm2f128	$ 0x03, %ymm4, %ymm8, %ymm8
	vperm2f128	$ 0x03, %ymm13, %ymm4, %ymm4

	vmovapd		%ymm1, %ymm13
	vperm2f128	$ 0x03, %ymm9, %ymm1, %ymm1
	vperm2f128	$ 0x03, %ymm5, %ymm9, %ymm9
	vperm2f128	$ 0x03, %ymm13, %ymm5, %ymm5

	vmovapd		%ymm2, %ymm13
	vperm2f128	$ 0x03, %ymm10, %ymm2, %ymm2
	vperm2f128	$ 0x03, %ymm6, %ymm10, %ymm10
	vperm2f128	$ 0x03, %ymm13, %ymm6, %ymm6

	vmovapd		%ymm3, %ymm13
	vperm2f128	$ 0x03, %ymm11, %ymm3, %ymm3
	vperm2f128	$ 0x03, %ymm7, %ymm11, %ymm11
	vperm2f128	$ 0x03, %ymm13, %ymm7, %ymm7

	vperm2f128	$ 0x01, %ymm14, %ymm14, %ymm14
	vperm2f128	$ 0x01, %ymm15, %ymm15, %ymm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC11(%rip), %ymm12
	vmovupd		.LC07(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC11(%rip), %ymm12
	vmovupd		LC07(%rip), %ymm13
#endif
	vandpd		%ymm12, %ymm14, %ymm12
	vandpd		%ymm13, %ymm15, %ymm13

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vblendpd	$ 0xc, .LC09(%rip), %ymm14, %ymm14
	vblendpd	$ 0x3, .LC09(%rip), %ymm15, %ymm15
#elif defined(OS_MAC)
	vblendpd	$ 0xc, LC09(%rip), %ymm14, %ymm14
	vblendpd	$ 0x3, LC09(%rip), %ymm15, %ymm15
#endif

	jmp		3f

2:

	// offset==3

	vmovapd		%ymm0, %ymm13
	vperm2f128	$ 0x21, %ymm0, %ymm8, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm8, %ymm0
	vperm2f128	$ 0x21, %ymm8, %ymm4, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm4, %ymm8
	vperm2f128	$ 0x21, %ymm4, %ymm13, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm13, %ymm4

	vmovapd		%ymm1, %ymm13
	vperm2f128	$ 0x21, %ymm1, %ymm9, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm9, %ymm1
	vperm2f128	$ 0x21, %ymm9, %ymm5, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm5, %ymm9
	vperm2f128	$ 0x21, %ymm5, %ymm13, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm13, %ymm5

	vmovapd		%ymm2, %ymm13
	vperm2f128	$ 0x21, %ymm2, %ymm10, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm10, %ymm2
	vperm2f128	$ 0x21, %ymm10, %ymm6, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm6, %ymm10
	vperm2f128	$ 0x21, %ymm6, %ymm13, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm13, %ymm6

	vmovapd		%ymm3, %ymm13
	vperm2f128	$ 0x21, %ymm3, %ymm11, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm11, %ymm3
	vperm2f128	$ 0x21, %ymm11, %ymm7, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm7, %ymm11
	vperm2f128	$ 0x21, %ymm7, %ymm13, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm13, %ymm7

	vperm2f128	$ 0x01, %ymm14, %ymm14, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm14, %ymm14
	vperm2f128	$ 0x01, %ymm15, %ymm15, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm15, %ymm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC12(%rip), %ymm12
	vmovupd		.LC08(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC12(%rip), %ymm12
	vmovupd		LC08(%rip), %ymm13
#endif
	vandpd		%ymm12, %ymm14, %ymm12
	vandpd		%ymm13, %ymm15, %ymm13

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vblendpd	$ 0x8, .LC09(%rip), %ymm14, %ymm14
	vblendpd	$ 0x7, .LC09(%rip), %ymm15, %ymm15
#elif defined(OS_MAC)
	vblendpd	$ 0x8, LC09(%rip), %ymm14, %ymm14
	vblendpd	$ 0x7, LC09(%rip), %ymm15, %ymm15
#endif


3:

	movq	%r11, %rax
	addq	%r12, %rax

	vmaskmovpd	%ymm0, %ymm12, 0(%r11)
	vmaskmovpd	%ymm4, %ymm14, 0(%r11, %r12)
	vmaskmovpd	%ymm8, %ymm15, 0(%r11, %r12, 2)
	vmaskmovpd	%ymm0, %ymm13, 0(%rax, %r12, 2)
	cmpl		$ 2, %r15d
	jl			4f // end
	vmaskmovpd	%ymm1, %ymm12, 32(%r11)
	vmaskmovpd	%ymm5, %ymm14, 32(%r11, %r12)
	vmaskmovpd	%ymm9, %ymm15, 32(%r11, %r12, 2)
	vmaskmovpd	%ymm1, %ymm13, 32(%rax, %r12, 2)
	cmpl		$ 3, %r15d
	jl			4f // end
	vmaskmovpd	%ymm2, %ymm12, 64(%r11)
	vmaskmovpd	%ymm6, %ymm14, 64(%r11, %r12)
	vmaskmovpd	%ymm10, %ymm15, 64(%r11, %r12, 2)
	vmaskmovpd	%ymm2, %ymm13, 64(%rax, %r12, 2)
	je			4f // end
	vmaskmovpd	%ymm3, %ymm12, 96(%r11)
	vmaskmovpd	%ymm7, %ymm14, 96(%r11, %r12)
	vmaskmovpd	%ymm11, %ymm15, 96(%r11, %r12, 2)
	vmaskmovpd	%ymm3, %ymm13, 96(%rax, %r12, 2)

4:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_12x4_gen_lib4)
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
	.macro INNER_STORE_4X12_GEN_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_4x12_gen_lib4)
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
	vmovapd		%ymm8, %ymm7
	vmovapd		%ymm9, %ymm8
	vmovapd		%ymm10, %ymm9
	vmovapd		%ymm11, %ymm10
	addq		$ 32, %r11

	cmpl	$ 1, %r15d
	jle		0f

	vmovapd		%ymm1, %ymm0
	vmovapd		%ymm2, %ymm1
	vmovapd		%ymm3, %ymm2
	vmovapd		%ymm4, %ymm3
	vmovapd		%ymm5, %ymm4
	vmovapd		%ymm6, %ymm5
	vmovapd		%ymm7, %ymm6
	vmovapd		%ymm8, %ymm7
	vmovapd		%ymm9, %ymm8
	vmovapd		%ymm10, %ymm9
	addq		$ 32, %r11

	cmpl	$ 2, %r15d
	jle		0f

	vmovapd		%ymm1, %ymm0
	vmovapd		%ymm2, %ymm1
	vmovapd		%ymm3, %ymm2
	vmovapd		%ymm4, %ymm3
	vmovapd		%ymm5, %ymm4
	vmovapd		%ymm6, %ymm5
	vmovapd		%ymm7, %ymm6
	vmovapd		%ymm8, %ymm7
	vmovapd		%ymm9, %ymm8
	addq		$ 32, %r11

0:

	// compute number of cols
	cmpl	$ 12, %eax
	jle		0f
	movl	$ 12, %eax
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
	cmpl		$ 8, %r15d
	jl			4f // end
	vmaskmovpd	%ymm7, %ymm15, 224(%r11)
	cmpl		$ 9, %r15d
	jl			4f // end
	vmaskmovpd	%ymm8, %ymm15, 256(%r11)
	cmpl		$ 10, %r15d
	jl			4f // end
	vmaskmovpd	%ymm9, %ymm15, 288(%r11)
	cmpl		$ 11, %r15d
	jl			4f // end
	vmaskmovpd	%ymm10, %ymm15, 320(%r11)
	je			4f // end
	vmaskmovpd	%ymm11, %ymm15, 352(%r11)

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

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC10(%rip), %ymm12
	vmovupd		.LC06(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC10(%rip), %ymm12
	vmovupd		LC06(%rip), %ymm13
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

	vperm2f128	$ 0x01, %ymm8, %ymm8, %ymm8

	vperm2f128	$ 0x01, %ymm9, %ymm9, %ymm9

	vperm2f128	$ 0x01, %ymm10, %ymm10, %ymm10

	vperm2f128	$ 0x01, %ymm11, %ymm11, %ymm11

	vperm2f128	$ 0x01, %ymm15, %ymm15, %ymm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC11(%rip), %ymm12
	vmovupd		.LC07(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC11(%rip), %ymm12
	vmovupd		LC07(%rip), %ymm13
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

	vperm2f128	$ 0x01, %ymm8, %ymm8, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm8, %ymm8

	vperm2f128	$ 0x01, %ymm9, %ymm9, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm9, %ymm9

	vperm2f128	$ 0x01, %ymm10, %ymm10, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm10, %ymm10

	vperm2f128	$ 0x01, %ymm11, %ymm11, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm11, %ymm11

	vperm2f128	$ 0x01, %ymm15, %ymm15, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm15, %ymm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC12(%rip), %ymm12
	vmovupd		.LC08(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC12(%rip), %ymm12
	vmovupd		LC08(%rip), %ymm13
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
	cmpl		$ 8, %r15d
	jl			4f // end
	vmaskmovpd	%ymm7, %ymm12, 224(%r11)
	vmaskmovpd	%ymm7, %ymm13, 224(%r11, %r12)
	cmpl		$ 9, %r15d
	jl			4f // end
	vmaskmovpd	%ymm8, %ymm12, 256(%r11)
	vmaskmovpd	%ymm8, %ymm13, 256(%r11, %r12)
	cmpl		$ 10, %r15d
	jl			4f // end
	vmaskmovpd	%ymm9, %ymm12, 288(%r11)
	vmaskmovpd	%ymm9, %ymm13, 288(%r11, %r12)
	cmpl		$ 11, %r15d
	jl			4f // end
	vmaskmovpd	%ymm10, %ymm12, 320(%r11)
	vmaskmovpd	%ymm10, %ymm13, 320(%r11, %r12)
	je			4f // end
	vmaskmovpd	%ymm11, %ymm12, 352(%r11)
	vmaskmovpd	%ymm11, %ymm13, 352(%r11, %r12)

4:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4x12_gen_lib4)
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
	.macro INNER_STORE_L_12X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_l_12x4_lib4)
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

	vmovapd		%ymm4, 0(%r10, %r11)
	vmovapd		%ymm5, 32(%r10, %r11)
	vmovapd		%ymm6, 64(%r10, %r11)
	vmovapd		%ymm7, 96(%r10, %r11)

	vmovapd		%ymm8, 0(%r10, %r11, 2)
	vmovapd		%ymm9, 32(%r10, %r11, 2)
	vmovapd		%ymm10, 64(%r10, %r11, 2)
	vmovapd		%ymm11, 96(%r10, %r11, 2)

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_12x4_lib4)
#endif





// common inner routine with file scope
//
// store lower n vs
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
	.macro INNER_STORE_L_12X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_l_12x4_vs_lib4)
#endif
	
	movq	%r10, %r15 // D1 <- D0
	addq	%r11, %r15 // D1 <- D0 + 4*sdd*sizeof(double)

	movq	%r15, %r14 // D2 <- D1
	addq	%r11, %r14 // D2 <- D1 + 4*sdd*sizeof(double)

	vcvtsi2sd	%r12d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC04(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC04(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm15

	cmpl		$ 2, %r13d
	vmovapd		%ymm0, 0(%r10)
	vmovapd		%ymm4, 0(%r15)
	vmaskmovpd	%ymm8, %ymm15,  0(%r14)
	jl			0f // end
	cmpl		$ 3, %r13d
	vmovapd		32(%r10), %ymm14
	vblendpd	$ 0x1, %ymm14, %ymm1, %ymm1	
	vmovapd		%ymm1, 32(%r10)
	vmovapd		%ymm5, 32(%r15)
	vmaskmovpd	%ymm9, %ymm15, 32(%r14)
	jl			0f // end
	vmovapd		64(%r10), %ymm14
	vblendpd	$ 0x3, %ymm14, %ymm2, %ymm2	
	vmovapd		%ymm2, 64(%r10)
	vmovapd		%ymm6, 64(%r15)
	vmaskmovpd	%ymm10, %ymm15, 64(%r14)
	je			0f // end
	vmovapd		96(%r10), %ymm14
	vblendpd	$ 0x7, %ymm14, %ymm3, %ymm3	
	vmovapd		%ymm3, 96(%r10)
	vmovapd		%ymm7, 96(%r15)
	vmaskmovpd	%ymm11, %ymm15, 96(%r14)

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_12x4_vs_lib4)
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
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d50 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
// ymm8  <- [d80 d90 da0 db0]
// ymm9  <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_U_12X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_u_12x4_lib4)
#endif
	
	//
	vmovapd		0(%r10, %r11, 2), %ymm14
	vblendpd	$ 0x1, %ymm8, %ymm14, %ymm8
	vmovapd		%ymm0, 0(%r10)
	vmovapd		%ymm4, 0(%r10, %r11)
	vmovapd		%ymm8, 0(%r10, %r11, 2)
	//
	vmovapd		32(%r10, %r11, 2), %ymm14
	vblendpd	$ 0x3, %ymm9, %ymm14, %ymm9
	vmovapd		%ymm1, 32(%r10)
	vmovapd		%ymm5, 32(%r10, %r11)
	vmovapd		%ymm9, 32(%r10, %r11, 2)
	//
	vmovapd		64(%r10, %r11, 2), %ymm14
	vblendpd	$ 0x7, %ymm10, %ymm14, %ymm10
	vmovapd		%ymm2, 64(%r10)
	vmovapd		%ymm6, 64(%r10, %r11)
	vmovapd		%ymm10, 64(%r10, %r11, 2)
	//
	vmovapd		%ymm3, 96(%r10)
	vmovapd		%ymm7, 96(%r10, %r11)
	vmovapd		%ymm11, 96(%r10, %r11, 2)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_u_12x4_lib4)
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
// ymm8  <- [d80 d90 da0 db0]
// ymm9  <- [d81 d91 da1 db1]
// ymm10 <- [d82 d92 da2 db2]
// ymm11 <- [d83 d93 da3 db3]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_U_12X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_u_12x4_vs_lib4)
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

	vmovapd		0(%r10, %r11, 2), %ymm14
	vblendpd	$ 0x1, %ymm8, %ymm14, %ymm8
	vmovapd		%ymm0, 0(%r10)
	vmovapd		%ymm4, 0(%r10, %r11)
	vmaskmovpd	%ymm8, %ymm15,  0(%r10, %r11, 2)
	cmpl		$ 2, %r13d
	jl			0f // end
	vmovapd		32(%r10, %r11, 2), %ymm14
	vblendpd	$ 0x3, %ymm9, %ymm14, %ymm9
	vmovapd		%ymm1, 32(%r10)
	vmovapd		%ymm5, 32(%r10, %r11)
	vmaskmovpd	%ymm9, %ymm15, 32(%r10, %r11, 2)
	cmpl		$ 3, %r13d
	jl			0f // end
	vmovapd		64(%r10, %r11, 2), %ymm14
	vblendpd	$ 0x7, %ymm10, %ymm14, %ymm10
	vmovapd		%ymm2, 64(%r10)
	vmovapd		%ymm6, 64(%r10, %r11)
	vmaskmovpd	%ymm10, %ymm15, 64(%r10, %r11, 2)
	je			0f // end
	vmovapd		%ymm3, 96(%r10)
	vmovapd		%ymm7, 96(%r10, %r11)
	vmaskmovpd	%ymm11, %ymm15, 96(%r10, %r11, 2)

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_u_12x4_vs_lib4)
#endif





// common inner routine with file scope
//
// prefetch
//
// input arguments:
// r10  <- D
// r11  <- 4*sdd*sizeof(double)
// ymm0 <- [d00 d11 d22 d33]
// ymm1 <- [d01 d10 d23 d32]
// ymm2 <- [d03 d12 d21 d30]
// ymm3 <- [d02 d13 d20 d31]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_PREFETCH1_12X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_prefetch1_12x4_lib4)
#endif

	prefetcht1	0(%r10)
	prefetcht1	0(%r10, %r11)
	prefetcht1	0(%r10, %r11, 2)
	prefetcht1	64(%r10)
	prefetcht1	64(%r10, %r11)
	prefetcht1	64(%r10, %r11, 2)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_prefetch1_12x4_lib4)
#endif





//                                1      2              3          4        5          6             7          8        9          10
// void kernel_dgemm_nt_12x4_lib4(int k, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_12x4_lib4)
	
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
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12 // C
	movq	ARG8, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dgemm_nt_12x4_lib4)





//                                1      2              3          4          5        6             7          8
// void kernel_dgemm_nt_4x12_lib4(int k, double *alpha, double *A, double *B, int sdb, double *beta, double *C, double *D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_4x12_lib4)
	
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
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12 // C

#if MACRO_LEVEL>=1
	INNER_TRAN_SCALE_AB_4X12_LIB4
#else
	CALL(inner_tran_scale_ab_4x12_lib4)
#endif


	// store n

	movq	ARG8, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_LIB4
#else
	CALL(inner_store_4x12_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dgemm_nt_4x12_lib4)





//                                   rdi     rsi            rdx        rcx      r8         r9            rsp+8      rsp+16   rsp+24     rsp+32   rsp+40  rsp+48
// void kernel_dgemm_nt_12x4_vs_lib4(int km, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_12x4_vs_lib4)
	
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
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12 // C
	movq	ARG8, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

	movq	ARG11, %r12 // km 
	movq	ARG12, %r13 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB4
#else
	CALL(inner_store_12x4_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dgemm_nt_12x4_vs_lib4)





//                                   1      2              3          4          5        6             7          8          9       10
// void kernel_dgemm_nt_4x12_vs_lib4(int k, double *alpha, double *A, double *B, int sdb, double *beta, double *C, double *D, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_4x12_vs_lib4)
	
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
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12 // C

#if MACRO_LEVEL>=1
	INNER_TRAN_SCALE_AB_4X12_LIB4
#else
	CALL(inner_tran_scale_ab_4x12_lib4)
#endif


	// store n

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // km
	movq	ARG10, %r12 // kn

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_VS_LIB4
#else
	CALL(inner_store_4x12_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dgemm_nt_4x12_vs_lib4)





//                                    1      2              3          4        5          6             7            8          9        10           11         12       13      14      15     16
// void kernel_dgemm_nt_12x4_gen_lib4(int k, double *alpha, double *A, int sda, double *B, double *beta, int offsetC, double *C, int sdc, int offsetD, double *D, int sdd, int m0, int m1, int n0, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_12x4_gen_lib4)
	
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
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12 // offsetC
	movq	ARG8, %r13 // C
	movq	ARG9, %r14 // sdc
	sall	$ 5, %r14d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_GEN_LIB4
#else
	CALL(inner_scale_ab_12x4_gen_lib4)
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
	INNER_STORE_12X4_GEN_LIB4
#else
	CALL(inner_store_12x4_gen_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dgemm_nt_12x4_gen_lib4)





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
	movq	ARG7, %r14 // sda
	sall	$ 5, %r14d // 4*sdb*sizeof(double)
	movq	ARG5, %r15 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_DGEMM_NN_12X4_LIB4
#else
	CALL(inner_edge_dgemm_nn_12x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4)
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





//                                1      2              3          4            5          6        7             8          9
// void kernel_dgemm_nn_4x12_lib4(int k, double *alpha, double *A, int offsetB, double *B, int sdb, double *beta, double *C, double *D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nn_4x12_lib4)
	
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
	INNER_EDGE_DGEMM_NN_4X12_LIB4
#else
	CALL(inner_edge_dgemm_nn_4x12_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X12_LIB4
#else
	CALL(inner_kernel_dgemm_nn_4x12_lib4)
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X12_LIB4
#else
	CALL(inner_scale_ab_4x12_lib4)
#endif


	// store n

	movq	ARG9, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_LIB4
#else
	CALL(inner_store_4x12_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nn_4x12_lib4)





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
	INNER_EDGE_DGEMM_NN_12X4_LIB4
#else
	CALL(inner_edge_dgemm_nn_12x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4)
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





//                                   1      2              3          4            5          6        7             8          9          10      11
// void kernel_dgemm_nn_4x12_vs_lib4(int k, double *alpha, double *A, int offsetB, double *B, int sdb, double *beta, double *C, double *D, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nn_4x12_vs_lib4)
	
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
	INNER_EDGE_DGEMM_NN_4X12_LIB4
#else
	CALL(inner_edge_dgemm_nn_4x12_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X12_LIB4
#else
	CALL(inner_kernel_dgemm_nn_4x12_lib4)
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X12_LIB4
#else
	CALL(inner_scale_ab_4x12_lib4)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // m1
	movq	ARG11, %r12 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_VS_LIB4
#else
	CALL(inner_store_4x12_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nn_4x12_vs_lib4)





//                                    1      2              3          4        5         6          7        8             9         10         11       12        13         14       15      16      17      18
// void kernel_dgemm_nn_12x4_gen_lib4(int k, double *alpha, double *A, int sda, int offB, double *B, int sdb, double *beta, int offC, double *C, int sdc, int offD, double *D, int sdd, int m0, int m1, int n0, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nn_12x4_gen_lib4)
	
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
	INNER_EDGE_DGEMM_NN_12X4_LIB4
#else
	CALL(inner_edge_dgemm_nn_12x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12 // offsetC
	movq	ARG10, %r13 // C
	movq	ARG11, %r14 // sdc
	sall	$ 5, %r14d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_GEN_LIB4
#else
	CALL(inner_scale_ab_12x4_gen_lib4)
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
	INNER_STORE_12X4_GEN_LIB4
#else
	CALL(inner_store_12x4_gen_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dgemm_nn_12x4_gen_lib4)





//                               1      2              3          4        5            6          7        8             9          10
// void kernel_dgemm_tt_4x12_lib4(int k, double *alpha, int offsetA, double *A, int sda, double *B, int sdb, double *beta, double *C, double *D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_tt_4x12_lib4)
	
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
	INNER_EDGE_DGEMM_NN_12X4_LIB4
#else
	CALL(inner_edge_dgemm_nn_12x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4)
#endif

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB4
#else
	CALL(inner_tran_12x4_lib4)
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X12_LIB4
#else
	CALL(inner_scale_ab_4x12_lib4)
#endif


	// store n

	movq	ARG10, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_LIB4
#else
	CALL(inner_store_4x12_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dgemm_tt_4x12_lib4)





//                               1      2              3          4            5          6        7             8          9        10         11
// void kernel_dgemm_tt_12x4_lib4(int k, double *alpha, int offsetA, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_tt_12x4_lib4)
	
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
	INNER_EDGE_DGEMM_NN_4X12_LIB4
#else
	CALL(inner_edge_dgemm_nn_4x12_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X12_LIB4
#else
	CALL(inner_kernel_dgemm_nn_4x12_lib4)
#endif

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB4
#else
	CALL(inner_tran_12x4_lib4)
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_tt_12x4_lib4)





//                                  1      2              3          4        5            6          7        8             9          10         11      12
// void kernel_dgemm_tt_4x12_vs_lib4(int k, double *alpha, int offsetA, double *A, int sda, double *B, int sdb, double *beta, double *C, double *D, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_tt_4x12_vs_lib4)
	
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
	INNER_EDGE_DGEMM_NN_12X4_LIB4
#else
	CALL(inner_edge_dgemm_nn_12x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4)
#endif

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB4
#else
	CALL(inner_tran_12x4_lib4)
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X12_LIB4
#else
	CALL(inner_scale_ab_4x12_lib4)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // km 
	movq	ARG12, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_VS_LIB4
#else
	CALL(inner_store_4x12_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dgemm_tt_4x12_vs_lib4)





//                                  1      2              3          4            5          6        7             8          9          10      11        12     13
// void kernel_dgemm_tt_12x4_vs_lib4(int k, double *alpha, int offsetA, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_tt_12x4_vs_lib4)
	
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
	INNER_EDGE_DGEMM_NN_4X12_LIB4
#else
	CALL(inner_edge_dgemm_nn_4x12_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X12_LIB4
#else
	CALL(inner_kernel_dgemm_nn_4x12_lib4)
#endif

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB4
#else
	CALL(inner_tran_12x4_lib4)
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB4
#else
	CALL(inner_store_12x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_tt_12x4_vs_lib4)





//                                   1      2              3          4        5         6          7        8             9         10         11       12        13         14       15      16      17      18
// void kernel_dgemm_tt_4x12_gen_lib4(int k, double *alpha, it offsetA, double *A, int sda, double *B, int sdb, double *beta, int offC, double *C, int sdc, int offD, double *D, int sdd, int m0, int m1, int n0, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_tt_4x12_gen_lib4)
	
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
	INNER_EDGE_DGEMM_NN_12X4_LIB4
#else
	CALL(inner_edge_dgemm_nn_12x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4)
#endif

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB4
#else
	CALL(inner_tran_12x4_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12 // offsetC
	movq	ARG10, %r13 // C
	movq	ARG11, %r14 // sdc
	sall	$ 5, %r14d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X12_GEN_LIB4
#else
	CALL(inner_scale_ab_4x12_gen_lib4)
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
	INNER_STORE_4X12_GEN_LIB4
#else
	CALL(inner_store_4x12_gen_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dgemm_tt_4x12_gen_lib4)





//                                  1      2              3          4        5            6          7        8             9          10       11         12
// void kernel_dsyrk_nn_u_12x4_lib4(int k, double *alpha, double *A, int sda, int offsetB, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nn_u_12x4_lib4)
	
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
	INNER_EDGE_DGEMM_NN_12X4_LIB4
#else
	CALL(inner_edge_dgemm_nn_12x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4)
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
	INNER_STORE_U_12X4_LIB4
#else
	CALL(inner_store_u_12x4_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dsyrk_nn_u_12x4_lib4)





//                                     1      2              3          4        5            6          7        8             9          10       11         12       13      14
// void kernel_dsyrk_nn_u_12x4_vs_lib4(int k, double *alpha, double *A, int sda, int offsetB, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nn_u_12x4_vs_lib4)
	
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
	INNER_EDGE_DGEMM_NN_12X4_LIB4
#else
	CALL(inner_edge_dgemm_nn_12x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4)
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
	movq	ARG13, %r12 // km 
	movq	ARG14, %r13 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_U_12X4_VS_LIB4
#else
	CALL(inner_store_u_12x4_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dsyrk_nn_u_12x4_vs_lib4)





//                                  rdi     rsi            rdx        rcx      r8         r9            rsp+8      rsp+16   rsp+24     rsp+32
// void kernel_dsyrk_nt_l_12x4_lib4(int km, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nt_l_12x4_lib4)
	
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
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12 // C
	movq	ARG8, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_L_12X4_LIB4
#else
	CALL(inner_store_l_12x4_lib4)
#endif

	EPILOGUE
	
	ret

	FUN_END(kernel_dsyrk_nt_l_12x4_lib4)





//                                     rdi     rsi            rdx        rcx      r8         r9            rsp+8      rsp+16   rsp+24     rsp+32   rsp+40  rsp+48
// void kernel_dsyrk_nt_l_12x4_vs_lib4(int km, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nt_l_12x4_vs_lib4)
	
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
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12 // C
	movq	ARG8, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4
#else
	CALL(inner_scale_ab_12x4_lib4)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

	movq	ARG11, %r12 // km 
	movq	ARG12, %r13 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_L_12X4_VS_LIB4
#else
	CALL(inner_store_l_12x4_vs_lib4)
#endif

	EPILOGUE
	
	ret

	FUN_END(kernel_dsyrk_nt_l_12x4_vs_lib4)





//                                   rdi    rsi            rdx        rcx      r8           r9         rsp+8    rsp+16     rsp+24
// void kernel_dtrmm_nn_rl_12x4_lib4(int k, double *alpha, double *A, int sda, int offsetB, double *B, int sdb, double *D, int sdd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nn_rl_12x4_lib4)
	
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
	INNER_EDGE_DTRMM_NN_RL_12X4_LIB4
#else
	CALL(inner_edge_dtrmm_nn_rl_12x4_lib4)
#endif

	// call inner dgemm kernel nt after initial triangle

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4)
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha

#if MACRO_LEVEL>=1
	INNER_SCALE_A0_12X4_LIB4
#else
	CALL(inner_scale_a0_12x4_lib4)
#endif


	// store n

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dtrmm_nn_rl_12x4_lib4)





//                                      1      2              3          4        5            6          7        8          9        10      11
// void kernel_dtrmm_nn_rl_12x4_vs_lib4(int k, double *alpha, double *A, int sda, int offsetB, double *B, int sdb, double *D, int sdd, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nn_rl_12x4_vs_lib4)
	
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
	INNER_EDGE_DTRMM_NN_RL_12X4_VS_LIB4
#else
	CALL(inner_edge_dtrmm_nn_rl_12x4_vs_lib4)
#endif

	// call inner dgemm kernel nt after initial triangle

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4)
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha

#if MACRO_LEVEL>=1
	INNER_SCALE_A0_12X4_LIB4
#else
	CALL(inner_scale_a0_12x4_lib4)
#endif


	// store n

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdb*sizeof(double)
	movq	ARG10, %r12 // km
	movq	ARG11, %r13 // kn

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB4
#else
	CALL(inner_store_12x4_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dtrmm_nn_rl_12x4_vs_lib4)





//                                  1      2              3          4        5          6          7
// void kernel_dtrmm_nt_ru_8x4_lib4(int k, double *alpha, double *A, int sda, double *B, double *D, int sdd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_ru_12x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt after initial triangle

	movq	ARG1, %r10 // k
//	subl	$ 4, %r10d //k-4
	movq	ARG3, %r11 // A
//	addq	$ 128, %r11 // A+4*bs
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13 // B
//	addq	$ 128, %r13 // B+4*bs

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RU_12X4_LIB4
#else
	CALL(inner_edge_dtrmm_nt_ru_12x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


// call inner blend

#if MACRO_LEVEL>=1
//	INNER_BLEND_12X4_LIB4
#else
//	CALL(inner_blend_12x4_lib4)
#endif


	// initial triangle

//	movq	ARG1, %r10 // kmax
//	movq	ARG3, %r11 // A
//	movq	ARG4, %r12 // sda
//	sall	$ 5, %r12d // 4*sda*sizeof(double)
//	movq	ARG5, %r13 // B

#if MACRO_LEVEL>=1
//	INNER_EDGE_DTRMM_NT_RU_12X4_LIB4
#else
//	CALL(inner_edge_dtrmm_nt_ru_12x4_lib4)
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha

#if MACRO_LEVEL>=1
	INNER_SCALE_A0_12X4_LIB4
#else
	CALL(inner_scale_a0_12x4_lib4)
#endif


	// store n

	movq	ARG6, %r10 // D
	movq	ARG7, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dtrmm_nt_ru_12x4_lib4)





//                                   1      2              3          4        5          6          7        8       9
// void kernel_dtrmm_nt_ru_12x4_lib4(int k, double *alpha, double *A, int sda, double *B, double *D, int sdd, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_ru_12x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt after initial triangle

	movq	ARG1, %r10 // k
//	subl	$ 4, %r10d //k-4
	movq	ARG3, %r11 // A
//	addq	$ 128, %r11 // A+4*bs
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG5, %r13 // B
//	addq	$ 128, %r13 // B+4*bs

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RU_12X4_LIB4
#else
	CALL(inner_edge_dtrmm_nt_ru_12x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


// call inner blend

#if MACRO_LEVEL>=1
//	INNER_BLEND_12X4_LIB4
#else
//	CALL(inner_blend_12x4_lib4)
#endif


	// initial triangle

//	movq	ARG3, %r10 // A
//	movq	ARG4, %r11 // sda
//	sall	$ 5, %r11d // 4*sda*sizeof(double)
//	movq	ARG5, %r12 // B

#if MACRO_LEVEL>=1
//	INNER_EDGE_DTRMM_NT_RU_12X4_LIB4
#else
//	CALL(inner_edge_dtrmm_nt_ru_12x4_lib4)
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha

#if MACRO_LEVEL>=1
	INNER_SCALE_A0_12X4_LIB4
#else
	CALL(inner_scale_a0_12x4_lib4)
#endif


	// store n

	movq	ARG6, %r10 // store address D
	movq	ARG7, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG8, %r12 // km 
	movq	ARG9, %r13 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB4
#else
	CALL(inner_store_12x4_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dtrmm_nt_ru_12x4_vs_lib4)





//                                   rdi    rsi        rdx      rcx        r8         r9       rsp+8      rsp+16   rsp+24
// void kernel_dpotrf_nt_l_12x4_lib4(int k, double *A, int sda, double *B, double *C, int sdc, double *D, int sdd, double *inv_diag_D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpotrf_nt_l_12x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender nn

	movq	ARG5, %r10 // C
	movq	ARG6, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_LIB4
#else
	CALL(inner_scale_m11_12x4_lib4)
#endif


	// factorization

	movq	ARG9, %r10  // inv_diag_D 
	movl	$ 4, %r11d

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_12X4_VS_LIB4
#else
	CALL(inner_edge_dpotrf_12x4_vs_lib4)
#endif


	// store n

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_L_12X4_LIB4
#else
	CALL(inner_store_l_12x4_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dpotrf_nt_l_12x4_lib4)





//                                      rdi    rsi        rdx      rcx        r8         r9       rsp+8      rsp+16   rsp+24              rsp+32  rsp+40 
// void kernel_dpotrf_nt_l_12x4_vs_lib4(int k, double *A, int sda, double *B, double *C, int sdc, double *D, int sdd, double *inv_diag_D, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpotrf_nt_l_12x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender nn

	movq	ARG5, %r10 // C
	movq	ARG6, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_LIB4
#else
	CALL(inner_scale_m11_12x4_lib4)
#endif


	// factorization

	movq	ARG9, %r10  // inv_diag_D 
	movq	ARG11, %r11 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_12X4_VS_LIB4
#else
	CALL(inner_edge_dpotrf_12x4_vs_lib4)
#endif


	// store n

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

	movq	ARG10, %r12 // km 
	movq	ARG11, %r13 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_L_12X4_VS_LIB4
#else
	CALL(inner_store_l_12x4_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dpotrf_nt_l_12x4_vs_lib4)





//                                         rdi     rsi         rdx       rcx         r8      r9          rsp+8     rsp+16      rsp+24     rsp+32   rsp+40     rsp+48   rsp+56
// void kernel_dsyrk_dpotrf_nt_l_12x4_lib4(int kp, double *Ap, int sdap, double *Bp, int km, double *Am, int sdam, double *Bm, double *C, int sdc, double *D, int sdd, double *inv_diag_D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_dpotrf_nt_l_12x4_lib4)
	
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
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// change sign
	NEG_ACC


	// call inner dgemm kernel nt sub

	movq	ARG5, %r10                 // km
	movq	ARG6, %r11                   // Am
	movq	ARG7, %r12 // sdam
	sall	$ 5, %r12d                   // 4*sdam*sizeof(double)
	movq	ARG8, %r13  // Bm

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender nn

	movq	ARG9, %r10 // C
	movq	ARG10, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_LIB4
#else
	CALL(inner_scale_m11_12x4_lib4)
#endif


	// factorization

	movq	ARG13, %r10  // inv_diag_D 
	movl	$ 4, %r11d

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_12X4_VS_LIB4
#else
	CALL(inner_edge_dpotrf_12x4_vs_lib4)
#endif


	// store n

	movq	ARG11, %r10 // D
	movq	ARG12, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_L_12X4_LIB4
#else
	CALL(inner_store_l_12x4_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dsyrk_dpotrf_nt_l_12x4_lib4)





//                                            rdi     rsi         rdx       rcx         r8      r9          rsp+8     rsp+16      rsp+24     rsp+32   rsp+40     rsp+48   rsp+56              rsp+64  rsp+72
// void kernel_dsyrk_dpotrf_nt_l_12x4_vs_lib4(int kp, double *Ap, int sdap, double *Bp, int km, double *Am, int sdam, double *Bm, double *C, int sdc, double *D, int sdd, double *inv_diag_D, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_dpotrf_nt_l_12x4_vs_lib4)
	
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
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// change sign
	NEG_ACC


	// call inner dgemm kernel nt sub

	movq	ARG5, %r10                 // km
	movq	ARG6, %r11                   // Am
	movq	ARG7, %r12 // sdam
	sall	$ 5, %r12d                   // 4*sdam*sizeof(double)
	movq	ARG8, %r13  // Bm

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender nn

	movq	ARG9, %r10 // C
	movq	ARG10, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_LIB4
#else
	CALL(inner_scale_m11_12x4_lib4)
#endif


	// factorization

	movq	ARG13, %r10  // inv_diag_D 
	movq	ARG15, %r11 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_12X4_VS_LIB4
#else
	CALL(inner_edge_dpotrf_12x4_vs_lib4)
#endif


	// store n

	movq	ARG11, %r10 // D
	movq	ARG12, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

	movq	ARG14, %r12 // km 
	movq	ARG15, %r13 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_L_12X4_VS_LIB4
#else
	CALL(inner_store_l_12x4_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dsyrk_dpotrf_nt_l_12x4_vs_lib4)





//                                       1      2          3        4          5             6          7        8          9        10         11
// void kernel_dtrsm_nt_rl_inv_12x4_lib4(int k, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd, double *E, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_inv_12x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender nn

	movq	ARG5, %r10 // beta
	movq	ARG6, %r11 // C
	movq	ARG7, %r12 // sdc
	sall	$ 5, %r12d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4
#else
	CALL(inner_scale_m1b_12x4_lib4)
#endif


	// solve

	movq	ARG10, %r10  // E 
	movq	ARG11, %r11  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_12X4_LIB4
#else
	CALL(inner_edge_dtrsm_rlt_inv_12x4_lib4)
#endif


	// store n

	movq	ARG8, %r10 // store address D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_12x4_lib4)





//                                       1      2          3        4          5             6          7        8          9        10         11       12      13
// void kernel_dtrsm_nt_rl_inv_12x4_vs_lib4(int k, double *A, int sda, double *B, double *C, int sdc, double *D, int sdd, double *E, double *inv_diag_E, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_inv_12x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender nn

	movq	ARG5, %r10 // beta
	movq	ARG6, %r11 // C
	movq	ARG7, %r12 // sdc
	sall	$ 5, %r12d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4
#else
	CALL(inner_scale_m1b_12x4_lib4)
#endif


	// solve

	movq	ARG10, %r10  // E 
	movq	ARG11, %r11  // inv_diag_E 
	movq	ARG13, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_12X4_VS_LIB4
#else
	CALL(inner_edge_dtrsm_rlt_inv_12x4_vs_lib4)
#endif


	// store n

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

	movq	ARG12, %r12 // km 
	movq	ARG13, %r13 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB4
#else
	CALL(inner_store_12x4_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_12x4_vs_lib4)





//                                          1      2          3          4        5          6          7          8        9                   10      11
// void kernel_dtrsm_nt_rl_inv_4x12_vs_lib4(int k, double *A, double *B, int sdb, double *C, double *D, double *E, int sde, double *inv_diag_E, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_inv_4x12_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // B
	movq	ARG4, %r12 // sdb
	sall	$ 5, %r12d // 32*sdb
	movq	ARG2, %r13 // A

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender nn

	movq	ARG5, %r10

#if MACRO_LEVEL>=1
	INNER_TRAN_SCALE_M11_4X12_LIB4
#else
	CALL(inner_tran_scale_m11_4x12_lib4)
#endif


	// solve

	movq	ARG7, %r10  // E 
	movq	ARG8, %r11  // sde 
	sall	$ 5, %r11d // 4*sde*sizeof(double)
	movq	ARG9, %r12  // inv_diag_E 
	movq	ARG11, %r13 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_4X12_VS_LIB4
#else
	CALL(inner_edge_dtrsm_rlt_inv_4x12_vs_lib4)
#endif


	// store n

	movq	ARG6, %r10 // store address D
	movq	ARG10, %r11 // km 
	movq	ARG11, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_VS_LIB4
#else
	CALL(inner_store_4x12_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_4x12_vs_lib4)





//                                                1       2           3         4           5       6           7         8           9          10       11         12       13         14                  15      16
// void kernel_dgemm_dtrsm_nt_rl_inv_12x4_vs_lib4(int kp, double *Ap, int sdap, double *Bp, int km, double *Am, int sdam, double *Bm, double *C, int sdc, double *D, int sdd, double *E, double *inv_diag_E, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_dtrsm_nt_rl_inv_12x4_vs_lib4)
	
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
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// change sign
	NEG_ACC


	// call inner dgemm kernel nt sub

	movq	ARG5, %r10                 // km
	movq	ARG6, %r11                   // Am
	movq	ARG7, %r12 // sdam
	sall	$ 5, %r12d                   // 4*sda*sizeof(double)
	movq	ARG8, %r13  // Bm

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender nn

	movq	ARG9, %r10  // C
	movq	ARG10, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_LIB4
#else
	CALL(inner_scale_m11_12x4_lib4)
#endif


	// solve

	movq	ARG13, %r10  // E 
	movq	ARG14, %r11  // inv_diag_E 
	movq	ARG16, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_12X4_VS_LIB4
#else
	CALL(inner_edge_dtrsm_rlt_inv_12x4_vs_lib4)
#endif


	// store n

	movq	ARG11, %r10 // D
	movq	ARG12, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

	movq	ARG15, %r12 // km 
	movq	ARG16, %r13 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB4
#else
	CALL(inner_store_12x4_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dgemm_dtrsm_nt_rl_inv_12x4_vs_lib4)





//                                                1       2           3           4         5       6           7           8         9          10         11         12       13                 14       15
// void kernel_dgemm_dtrsm_nt_rl_inv_4x12_vs_lib4(int kp, double *Ap, double *Bp, int sdbp, int km, double *Am, double *Bm, int sdbm, double *C, double *D, double *E, int sde, double *inv_diag_E, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_dtrsm_nt_rl_inv_4x12_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG3, %r11  // Bp
	movq	ARG4, %r12 // sdbp
	sall	$ 5, %r12d   // 32*sdbp
	movq	ARG2, %r13  // Ap

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// change sign
	NEG_ACC


	// call inner dgemm kernel nt sub

	movq	ARG5, %r10 // km
	movq	ARG7, %r11 // Bm
	movq	ARG8, %r12 // sdbm
	sall	$ 5, %r12d // 32*sdbm
	movq	ARG6, %r13 // Am

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender nn

	movq	ARG9, %r10

#if MACRO_LEVEL>=1
	INNER_TRAN_SCALE_M11_4X12_LIB4
#else
	CALL(inner_tran_scale_m11_4x12_lib4)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11  // sde 
	sall	$ 5, %r11d // 4*sde*sizeof(double)
	movq	ARG13, %r12  // inv_diag_E 
	movq	ARG15, %r13 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_4X12_VS_LIB4
#else
	CALL(inner_edge_dtrsm_rlt_inv_4x12_vs_lib4)
#endif


	// store n

	movq	ARG10, %r10 // store address D
	movq	ARG14, %r11 // km 
	movq	ARG15, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_VS_LIB4
#else
	CALL(inner_store_4x12_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dgemm_dtrsm_nt_rl_inv_4x12_vs_lib4)





//                                       1      2          3          4        5          6          7          8        9
// void kernel_dtrsm_nt_rl_inv_4x12_lib4(int k, double *A, double *B, int sdb, double *C, double *D, double *E, int sde, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_inv_4x12_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10
	movq	ARG3, %r11
	movq	ARG4, %r12
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG2, %r13

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender nn

	movq	ARG5, %r10

#if MACRO_LEVEL>=1
	INNER_TRAN_SCALE_M11_4X12_LIB4
#else
	CALL(inner_tran_scale_m11_4x12_lib4)
#endif


	// solve

	movq	ARG7, %r10  // E 
	movq	ARG8, %r11  // sde 
	sall	$ 5, %r11d // 4*sde*sizeof(double)
	movq	ARG9, %r12  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_4X12_LIB4
#else
	CALL(inner_edge_dtrsm_rlt_inv_4x12_lib4)
#endif


	// store n

	movq	ARG6, %r10 // store address D

#if MACRO_LEVEL>=1
	INNER_STORE_4X12_LIB4
#else
	CALL(inner_store_4x12_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_4x12_lib4)





//                                             rdi     rsi         rdx       rcx         r8      r9          rsp+8     rsp+16      rsp+24     rsp+32   rsp+40     rsp+48   rsp+56     rsp+64
// void kernel_dgemm_dtrsm_nt_rl_inv_12x4_lib4(int kp, double *Ap, int sdap, double *Bp, int km, double *Am, int sdam, double *Bm, double *C, int sdc, double *D, int sdd, double *E, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_dtrsm_nt_rl_inv_12x4_lib4)
	
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
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// change sign
	NEG_ACC


	// call inner dgemm kernel nt sub

	movq	ARG5, %r10                 // km
	movq	ARG6, %r11                   // Am
	movq	ARG7, %r12 // sdam
	sall	$ 5, %r12d                   // 4*sda*sizeof(double)
	movq	ARG8, %r13  // Bm

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender nn

	movq	ARG9, %r10  // C
	movq	ARG10, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_LIB4
#else
	CALL(inner_scale_m11_12x4_lib4)
#endif


	// solve

	movq	ARG13, %r10  // E 
	movq	ARG14, %r11  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_12X4_LIB4
#else
	CALL(inner_edge_dtrsm_rlt_inv_12x4_lib4)
#endif


	// store n

	movq	ARG11, %r10 // store address D
	movq	ARG12, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dgemm_dtrsm_nt_rl_inv_12x4_lib4)





//                                       1      2          3        4          5             6          7        8          9        10
// void kernel_dtrsm_nt_rl_one_12x4_lib4(int k, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd, double *E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_one_12x4_lib4)
	
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
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender nn

	movq	ARG5, %r10 // beta
	movq	ARG6, %r11 // C
	movq	ARG7, %r12 // sdc
	sall	$ 5, %r12d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4
#else
	CALL(inner_scale_m1b_12x4_lib4)
#endif


	// solve

	movq	ARG10, %r10  // E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_ONE_12X4_LIB4
#else
	CALL(inner_edge_dtrsm_rlt_one_12x4_lib4)
#endif


	// store n

	movq	ARG8, %r10 // store address D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dtrsm_nt_rl_one_12x4_lib4)





//                                          1      2          3        4          5             6          7        8          9        10         11      12
// void kernel_dtrsm_nt_rl_one_12x4_vs_lib4(int k, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd, double *E, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_one_12x4_vs_lib4)
	
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
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender nn

	movq	ARG5, %r10 // beta
	movq	ARG6, %r11 // C
	movq	ARG7, %r12 // sdc
	sall	$ 5, %r12d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4
#else
	CALL(inner_scale_m1b_12x4_lib4)
#endif


	// solve

	movq	ARG10, %r10  // E 
	movq	ARG12, %r11 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_ONE_12X4_VS_LIB4
#else
	CALL(inner_edge_dtrsm_rlt_one_12x4_vs_lib4)
#endif


	// store n

	movq	ARG8, %r10 // store address D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

	movq	ARG11, %r12 // km 
	movq	ARG12, %r13 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB4
#else
	CALL(inner_store_12x4_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dtrsm_nt_rl_one_12x4_vs_lib4)





//                                       1      2          3        4          5             6          7        8          9        10         11
// void kernel_dtrsm_nt_ru_inv_12x4_lib4(int k, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd, double *E, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_ru_inv_12x4_lib4)
	
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
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender nn

	movq	ARG5, %r10 // beta
	movq	ARG6, %r11 // C
	movq	ARG7, %r12 // sdc
	sall	$ 5, %r12d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4
#else
	CALL(inner_scale_m1b_12x4_lib4)
#endif


	// solve

	movq	ARG10, %r10  // E 
	movq	ARG11, %r11  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUT_INV_12X4_LIB4
#else
	CALL(inner_edge_dtrsm_rut_inv_12x4_lib4)
#endif


	// store n

	movq	ARG8, %r10 // store address D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dtrsm_nt_ru_inv_12x4_lib4)





//                                          1      2          3        4          5             6          7        8          9        10         11                  12      13
// void kernel_dtrsm_nt_ru_inv_12x4_vs_lib4(int k, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd, double *E, double *inv_diag_E, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_ru_inv_12x4_vs_lib4)
	
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
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender nn

	movq	ARG5, %r10 // beta
	movq	ARG6, %r11 // C
	movq	ARG7, %r12 // sdc
	sall	$ 5, %r12d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4
#else
	CALL(inner_scale_m1b_12x4_lib4)
#endif


	// solve

	movq	ARG10, %r10  // E 
	movq	ARG11, %r11  // inv_diag_E 
	movq	ARG13, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUT_INV_12X4_VS_LIB4
#else
	CALL(inner_edge_dtrsm_rut_inv_12x4_vs_lib4)
#endif


	// store n

	movq	ARG8, %r10 // store address D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG12, %r12 // km 
	movq	ARG13, %r13 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB4
#else
	CALL(inner_store_12x4_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dtrsm_nt_ru_inv_12x4_vs_lib4)





//                                       1      2          3        4          5             6          7        8          9        10
// void kernel_dtrsm_nt_ru_one_12x4_lib4(int k, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd, double *E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_ru_one_12x4_lib4)
	
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
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender nn

	movq	ARG5, %r10 // beta
	movq	ARG6, %r11 // C
	movq	ARG7, %r12 // sdc
	sall	$ 5, %r12d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4
#else
	CALL(inner_scale_m1b_12x4_lib4)
#endif


	// solve

	movq	ARG10, %r10  // E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUT_ONE_12X4_LIB4
#else
	CALL(inner_edge_dtrsm_rut_one_12x4_lib4)
#endif


	// store n

	movq	ARG8, %r10 // store address D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dtrsm_nt_ru_one_12x4_lib4)





//                                          1      2          3        4          5             6          7        8          9        10         11      12
// void kernel_dtrsm_nt_ru_one_12x4_vs_lib4(int k, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd, double *E, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_ru_one_12x4_vs_lib4)
	
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
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender nn

	movq	ARG5, %r10 // beta
	movq	ARG6, %r11 // C
	movq	ARG7, %r12 // sdc
	sall	$ 5, %r12d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4
#else
	CALL(inner_scale_m1b_12x4_lib4)
#endif


	// solve

	movq	ARG10, %r10  // E 
	movq	ARG12, %r11 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUT_ONE_12X4_VS_LIB4
#else
	CALL(inner_edge_dtrsm_rut_one_12x4_vs_lib4)
#endif


	// store n

	movq	ARG8, %r10 // store address D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG11, %r12 // km 
	movq	ARG12, %r13 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB4
#else
	CALL(inner_store_12x4_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dtrsm_nt_ru_one_12x4_vs_lib4)





//                                       1      2          3        4          5        6             7          8        9          10       11         12
// void kernel_dtrsm_nn_ru_inv_12x4_lib4(int k, double *A, int sda, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd, double *E, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_ru_inv_12x4_lib4)
	
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
	INNER_KERNEL_DGEMM_NN_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // sdc
	sall	$ 5, %r12d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4
#else
	CALL(inner_scale_m1b_12x4_lib4)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUN_INV_12X4_LIB4
#else
	CALL(inner_edge_dtrsm_run_inv_12x4_lib4)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dtrsm_nn_ru_inv_12x4_lib4)





//                                          1      2          3        4          5        6             7          8        9          10       11         12                  13      14
// void kernel_dtrsm_nn_ru_inv_12x4_vs_lib4(int k, double *A, int sda, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd, double *E, double *inv_diag_E, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_ru_inv_12x4_vs_lib4)
	
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
	INNER_KERNEL_DGEMM_NN_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // sdc
	sall	$ 5, %r12d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4
#else
	CALL(inner_scale_m1b_12x4_lib4)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUN_INV_12X4_LIB4
#else
	CALL(inner_edge_dtrsm_run_inv_12x4_lib4)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // sdd
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

	FUN_END(kernel_dtrsm_nn_ru_inv_12x4_vs_lib4)





//                                       1      2          3        4          5        6             7          8        9          10       11         12       13
// void kernel_dtrsm_nn_ll_inv_12x4_lib4(int k, double *A, int sda, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd, double *E, int sde, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_ll_inv_12x4_lib4)
	
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
	INNER_KERNEL_DGEMM_NN_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // sdc
	sall	$ 5, %r12d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4
#else
	CALL(inner_scale_m1b_12x4_lib4)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // sde
	sall	$ 5, %r11d // 4*sde*sizeof(double)
	movq	ARG13, %r12  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_LLN_INV_12X4_LIB4
#else
	CALL(inner_edge_dtrsm_lln_inv_12x4_lib4)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dtrsm_nn_ll_inv_12x4_lib4)





//                                          1      2          3        4          5        6             7          8        9          10       11         12        13                 14      15
// void kernel_dtrsm_nn_ll_inv_12x4_vs_lib4(int k, double *A, int sda, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd, double *E, int sde, double *inv_diga_E, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_ll_inv_12x4_vs_lib4)
	
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
	INNER_KERNEL_DGEMM_NN_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // sdc
	sall	$ 5, %r12d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4
#else
	CALL(inner_scale_m1b_12x4_lib4)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // sde
	sall	$ 5, %r11d // 4*sde*sizeof(double)
	movq	ARG13, %r12  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_LLN_INV_12X4_LIB4
#else
	CALL(inner_edge_dtrsm_lln_inv_12x4_lib4)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

	movq	ARG14, %r12 // km
	movq	ARG15, %r13 // kn

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB4
#else
	CALL(inner_store_12x4_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dtrsm_nn_ll_inv_12x4_vs_lib4)





//                                       1      2          3        4          5        6             7          8        9          10       11         12
// void kernel_dtrsm_nn_ll_one_12x4_lib4(int k, double *A, int sda, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd, double *E, int sde);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_ll_one_12x4_lib4)
	
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
	INNER_KERNEL_DGEMM_NN_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // sdc
	sall	$ 5, %r12d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4
#else
	CALL(inner_scale_m1b_12x4_lib4)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // sde
	sall	$ 5, %r11d // 4*sde*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_LLN_ONE_12X4_LIB4
#else
	CALL(inner_edge_dtrsm_lln_one_12x4_lib4)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dtrsm_nn_ll_one_12x4_lib4)





//                                          1      2          3        4          5        6             7          8        9          10       11         12        13     14
// void kernel_dtrsm_nn_ll_one_12x4_vs_lib4(int k, double *A, int sda, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd, double *E, int sde, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_ll_one_12x4_vs_lib4)
	
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
	INNER_KERNEL_DGEMM_NN_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // beta
	movq	ARG7, %r11 // C
	movq	ARG8, %r12 // sdc
	sall	$ 5, %r12d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4
#else
	CALL(inner_scale_m1b_12x4_lib4)
#endif


	// solve

	movq	ARG11, %r10  // E 
	movq	ARG12, %r11 // sde
	sall	$ 5, %r11d // 4*sde*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_LLN_ONE_12X4_LIB4
#else
	CALL(inner_edge_dtrsm_lln_one_12x4_lib4)
#endif


	// store

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // sdd
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

	FUN_END(kernel_dtrsm_nn_ll_one_12x4_vs_lib4)





//                                       edi    rsi        rdx      ecx        r8       r9         rsp+8    rsp+16     rsp+24   rsp+32     rsp+40   rsp+48
// void kernel_dtrsm_nn_lu_inv_12x4_lib4(int k, double *A, int sda, double *B, int sdb, double *C, int sdc, double *D, int sdd, double *E, int sde, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_lu_inv_12x4_lib4)
	
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
	INNER_KERNEL_DGEMM_NN_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // C
	movq	ARG7, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_LIB4
#else
	CALL(inner_scale_m11_12x4_lib4)
#endif


	// solve

	movq	ARG10, %r10  // E 
	movq	ARG11, %r11 // sde
	sall	$ 5, %r11d // 4*sde*sizeof(double)
	movq	ARG12, %r12  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_LUN_INV_12X4_LIB4
#else
	CALL(inner_edge_dtrsm_lun_inv_12x4_lib4)
#endif


	// store

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dtrsm_nn_lu_inv_12x4_lib4)





//                                          edi    rsi        rdx      ecx        r8       r9         rsp+8    rsp+16     rsp+24   rsp+32     rsp+40   rsp+48               rsp+56  rsp+64
// void kernel_dtrsm_nn_lu_inv_12x4_vs_lib4(int k, double *A, int sda, double *B, int sdb, double *C, int sdc, double *D, int sdd, double *E, int sde, double *inv_diag_E), int km, int kn;

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_lu_inv_12x4_vs_lib4)
	
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
	INNER_KERNEL_DGEMM_NN_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // C
	movq	ARG7, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_LIB4
#else
	CALL(inner_scale_m11_12x4_lib4)
#endif


	// solve

	movq	ARG10, %r10  // E 
	movq	ARG11, %r11 // sde
	sall	$ 5, %r11d // 4*sde*sizeof(double)
	movq	ARG12, %r12  // inv_diag_E 
	movq	ARG13, %r13  // km 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_LUN_INV_12X4_VS_LIB4
#else
	CALL(inner_edge_dtrsm_lun_inv_12x4_vs_lib4)
#endif


	// store

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG13, %r12  // km 
	movq	ARG14, %r13  // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB4
#else
	CALL(inner_store_12x4_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dtrsm_nn_lu_inv_12x4_vs_lib4)





//                                   edi    rsi        rdx      rcx        r8       r9         rsp+8    rsp+16     rsp+24   rsp+32
// void kernel_dgetrf_nn_l_12x4_lib4(int k, double *A, int sda, double *B, int sdb, double *C, int sdc, double *D, int sdd, double *inv_diag_D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgetrf_nn_l_12x4_lib4)
	
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
	INNER_KERNEL_DGEMM_NN_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // C
	movq	ARG7, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_LIB4
#else
	CALL(inner_scale_m11_12x4_lib4)
#endif


	// factorization

	movq	ARG10, %r10  // inv_diag_D 

#if MACRO_LEVEL>=1
	INNER_EDGE_DGETRF_L_12X4_LIB4
#else
	CALL(inner_edge_dgetrf_l_12x4_lib4)
#endif


	// store

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dgetrf_nn_l_12x4_lib4)





//                                      edi    rsi        rdx      rcx        r8       r9         rsp+8    rsp+16     rsp+24   rsp+32              rsp+40  rsp+48
// void kernel_dgetrf_nn_l_12x4_vs_lib4(int k, double *A, int sda, double *B, int sdb, double *C, int sdc, double *D, int sdd, double *inv_diag_D, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgetrf_nn_l_12x4_vs_lib4)
	
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
	INNER_KERNEL_DGEMM_NN_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // C
	movq	ARG7, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_LIB4
#else
	CALL(inner_scale_m11_12x4_lib4)
#endif


	// factorization

	movq	ARG10, %r10  // inv_diag_D 

#if MACRO_LEVEL>=1
	INNER_EDGE_DGETRF_L_12X4_LIB4
#else
	CALL(inner_edge_dgetrf_l_12x4_lib4)
#endif


	// store

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG11, %r12  // km
	movq	ARG12, %r13  // km

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB4
#else
	CALL(inner_store_12x4_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dgetrf_nn_l_12x4_vs_lib4)





//                                   edi    rsi        rdx      rcx        r8       r9         rsp+8    rsp+16     rsp+24   rsp+32
// void kernel_dgetrf_nn_m_12x4_lib4(int k, double *A, int sda, double *B, int sdb, double *C, int sdc, double *D, int sdd, double *inv_diag_D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgetrf_nn_m_12x4_lib4)
	
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
	INNER_KERNEL_DGEMM_NN_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // C
	movq	ARG7, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_LIB4
#else
	CALL(inner_scale_m11_12x4_lib4)
#endif


	// factorization

	movq	ARG8, %r10 // D
	subq	$ 128, %r10 // E
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG10, %r12  // inv_diag_D 

#if MACRO_LEVEL>=1
	INNER_EDGE_DGETRF_M_12X4_LIB4
#else
	CALL(inner_edge_dgetrf_m_12x4_lib4)
#endif


	// store

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dgetrf_nn_m_12x4_lib4)





//                                      edi    rsi        rdx      rcx        r8       r9         rsp+8    rsp+16     rsp+24   rsp+32              rsp+40  rsp+48
// void kernel_dgetrf_nn_m_12x4_vs_lib4(int k, double *A, int sda, double *B, int sdb, double *C, int sdc, double *D, int sdd, double *inv_diag_D, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgetrf_nn_m_12x4_vs_lib4)
	
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
	INNER_KERNEL_DGEMM_NN_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // C
	movq	ARG7, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_LIB4
#else
	CALL(inner_scale_m11_12x4_lib4)
#endif


	// factorization

	movq	ARG8, %r10 // D
	subq	$ 128, %r10 // E
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG10, %r12  // inv_diag_D 

#if MACRO_LEVEL>=1
	INNER_EDGE_DGETRF_M_12X4_LIB4
#else
	CALL(inner_edge_dgetrf_m_12x4_lib4)
#endif


	// store

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG11, %r12  // km
	movq	ARG12, %r13  // km

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB4
#else
	CALL(inner_store_12x4_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dgetrf_nn_m_12x4_vs_lib4)





//                                   edi    rsi        rdx      rcx        r8       r9         rsp+8    rsp+16     rsp+24   rsp+32
// void kernel_dgetrf_nn_r_12x4_lib4(int k, double *A, int sda, double *B, int sdb, double *C, int sdc, double *D, int sdd, double *inv_diag_D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgetrf_nn_r_12x4_lib4)
	
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
	INNER_KERNEL_DGEMM_NN_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // C
	movq	ARG7, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_LIB4
#else
	CALL(inner_scale_m11_12x4_lib4)
#endif


	// factorization

	movq	ARG8, %r10 // D
	subq	$ 256, %r10 // E
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG10, %r12  // inv_diag_D 

#if MACRO_LEVEL>=1
	INNER_EDGE_DGETRF_R_12X4_LIB4
#else
	CALL(inner_edge_dgetrf_r_12x4_lib4)
#endif


	// store

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dgetrf_nn_r_12x4_lib4)





//                                      edi    rsi        rdx      rcx        r8       r9         rsp+8    rsp+16     rsp+24   rsp+32              rsp+40  rsp+48
// void kernel_dgetrf_nn_r_12x4_vs_lib4(int k, double *A, int sda, double *B, int sdb, double *C, int sdc, double *D, int sdd, double *inv_diag_D, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgetrf_nn_r_12x4_vs_lib4)
	
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
	INNER_KERNEL_DGEMM_NN_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_12x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG6, %r10 // C
	movq	ARG7, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_LIB4
#else
	CALL(inner_scale_m11_12x4_lib4)
#endif


	// factorization

	movq	ARG8, %r10 // D
	subq	$ 256, %r10 // E
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG10, %r12  // inv_diag_D 

#if MACRO_LEVEL>=1
	INNER_EDGE_DGETRF_R_12X4_LIB4
#else
	CALL(inner_edge_dgetrf_r_12x4_lib4)
#endif


	// store

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG11, %r12  // km
	movq	ARG12, %r13  // km

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB4
#else
	CALL(inner_store_12x4_vs_lib4)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dgetrf_nn_r_12x4_vs_lib4)





//                                   1      2          3        4          5          6        7          8        9
// void kernel_dgetrf_nt_l_12x4_lib4(int k, double *A, int sda, double *B, double *C, int sdc, double *D, int sdd, double *inv_diag_D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgetrf_nt_l_12x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // C
	movq	ARG6, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_LIB4
#else
	CALL(inner_scale_m11_12x4_lib4)
#endif


	// factorization

	movq	ARG9, %r10  // inv_diag_D 

#if MACRO_LEVEL>=1
	INNER_EDGE_DGETRF_L_12X4_LIB4
#else
	CALL(inner_edge_dgetrf_l_12x4_lib4)
#endif


	// store

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif


	// epilogue

	EPILOGUE

	ret

	FUN_END(kernel_dgetrf_nt_l_12x4_lib4)





//                                      1      2          3        4          5          6        7          8        9                   10      11
// void kernel_dgetrf_nt_l_12x4_vs_lib4(int k, double *A, int sda, double *B, double *C, int sdc, double *D, int sdd, double *inv_diag_D, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgetrf_nt_l_12x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // C
	movq	ARG6, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_LIB4
#else
	CALL(inner_scale_m11_12x4_lib4)
#endif


	// factorization

	movq	ARG9, %r10  // inv_diag_D 

#if MACRO_LEVEL>=1
	INNER_EDGE_DGETRF_L_12X4_LIB4
#else
	CALL(inner_edge_dgetrf_l_12x4_lib4)
#endif


	// store

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG10, %r12  // km
	movq	ARG11, %r13  // km

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB4
#else
	CALL(inner_store_12x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgetrf_nt_l_12x4_vs_lib4)





//                                   1      2          3        4          5          6        7           8       9
// void kernel_dgetrf_nt_m 12x4_lib4(int k, double *A, int sda, double *B, double *C, int sdc, double *D, int sdd, double *inv_diag_D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgetrf_nt_m_12x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // C
	movq	ARG6, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_LIB4
#else
	CALL(inner_scale_m11_12x4_lib4)
#endif


	// factorization

//	movq	ARG10, %r10  // inv_diag_D 
	movq	ARG7, %r10 // D
	subq	$ 128, %r10 // E
	movq	ARG8, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG9, %r12  // inv_diag_D 

#if MACRO_LEVEL>=1
	INNER_EDGE_DGETRF_M_12X4_LIB4
#else
	CALL(inner_edge_dgetrf_m_12x4_lib4)
#endif


	// store

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif


	// epilogue

	EPILOGUE

	ret

	FUN_END(kernel_dgetrf_nt_m_12x4_lib4)





//                                      1      2          3        4          5          6        7          8        9                   10      11
// void kernel_dgetrf_nt_m_12x4_vs_lib4(int k, double *A, int sda, double *B, double *C, int sdc, double *D, int sdd, double *inv_diag_D, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgetrf_nt_m_12x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // C
	movq	ARG6, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_LIB4
#else
	CALL(inner_scale_m11_12x4_lib4)
#endif


	// factorization

//	movq	ARG10, %r10  // inv_diag_D 
	movq	ARG7, %r10 // D
	subq	$ 128, %r10 // E
	movq	ARG8, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG9, %r12  // inv_diag_D 

#if MACRO_LEVEL>=1
	INNER_EDGE_DGETRF_M_12X4_LIB4
#else
	CALL(inner_edge_dgetrf_m_12x4_lib4)
#endif


	// store

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG10, %r12  // km
	movq	ARG11, %r13  // km

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB4
#else
	CALL(inner_store_12x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgetrf_nt_m_12x4_vs_lib4)





//                                   1      2          3        4          5          6        7           8       9
// void kernel_dgetrf_nt_r 12x4_lib4(int k, double *A, int sda, double *B, double *C, int sdc, double *D, int sdd, double *inv_diag_D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgetrf_nt_r_12x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // C
	movq	ARG6, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_LIB4
#else
	CALL(inner_scale_m11_12x4_lib4)
#endif


	// factorization

//	movq	ARG10, %r10  // inv_diag_D 
	movq	ARG7, %r10 // D
	subq	$ 256, %r10 // E
	movq	ARG8, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG9, %r12  // inv_diag_D 

#if MACRO_LEVEL>=1
	INNER_EDGE_DGETRF_R_12X4_LIB4
#else
	CALL(inner_edge_dgetrf_r_12x4_lib4)
#endif


	// store

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif


	// epilogue

	EPILOGUE

	ret

	FUN_END(kernel_dgetrf_nt_r_12x4_lib4)





//                                      1      2          3        4          5          6        7          8        9                   10      11
// void kernel_dgetrf_nt_r_12x4_vs_lib4(int k, double *A, int sda, double *B, double *C, int sdc, double *D, int sdd, double *inv_diag_D, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgetrf_nt_r_12x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // C
	movq	ARG6, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_LIB4
#else
	CALL(inner_scale_m11_12x4_lib4)
#endif


	// factorization

//	movq	ARG10, %r10  // inv_diag_D 
	movq	ARG7, %r10 // D
	subq	$ 256, %r10 // E
	movq	ARG8, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG9, %r12  // inv_diag_D 

#if MACRO_LEVEL>=1
	INNER_EDGE_DGETRF_R_12X4_LIB4
#else
	CALL(inner_edge_dgetrf_r_12x4_lib4)
#endif


	// store

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG10, %r12  // km
	movq	ARG11, %r13  // km

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_VS_LIB4
#else
	CALL(inner_store_12x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgetrf_nt_r_12x4_vs_lib4)





//                                1         2           3         4           5          6
// void kernel_dlarfb12_rn_12_lib4(int kmax, double *pV, int sdd, double *pT, double *pD, double *pK);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dlarfb12_rn_12_lib4)
	
	PROLOGUE

	// zero accumulation registers

//	ZERO_ACC
	

	// left 12x4 of K

	movq	ARG1, %r10 // k
	movq	ARG5, %r11 // D
	movq	ARG3, %r12 // sdd
	sall	$ 5, %r12d
	movq	ARG2, %r13 // V

	//
	vmovapd			0(%r11), %ymm12
	vmovapd			0(%r11, %r12), %ymm13
	vmovapd			0(%r11, %r12, 2), %ymm14
	vmovapd			%ymm12, %ymm0
	vmovapd			%ymm13, %ymm4
	vmovapd			%ymm14, %ymm8
	//
	vmovapd			32(%r11), %ymm12
	vmovapd			32(%r11, %r12), %ymm13
	vmovapd			32(%r11, %r12, 2), %ymm14
	vbroadcastsd	32(%r13), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm0
	vfmadd231pd		%ymm13, %ymm15, %ymm4
	vfmadd231pd		%ymm14, %ymm15, %ymm8
	vmovapd			%ymm12, %ymm1
	vmovapd			%ymm13, %ymm5
	vmovapd			%ymm14, %ymm9
	//
	vmovapd			64(%r11), %ymm12
	vmovapd			64(%r11, %r12), %ymm13
	vmovapd			64(%r11, %r12, 2), %ymm14
	vbroadcastsd	64(%r13), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm0
	vfmadd231pd		%ymm13, %ymm15, %ymm4
	vfmadd231pd		%ymm14, %ymm15, %ymm8
	vbroadcastsd	72(%r13), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm1
	vfmadd231pd		%ymm13, %ymm15, %ymm5
	vfmadd231pd		%ymm14, %ymm15, %ymm9
	vmovapd			%ymm12, %ymm2
	vmovapd			%ymm13, %ymm6
	vmovapd			%ymm14, %ymm10
	//
	vmovapd			96(%r11), %ymm12
	vmovapd			96(%r11, %r12), %ymm13
	vmovapd			96(%r11, %r12, 2), %ymm14
	vbroadcastsd	96(%r13), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm0
	vfmadd231pd		%ymm13, %ymm15, %ymm4
	vfmadd231pd		%ymm14, %ymm15, %ymm8
	vbroadcastsd	104(%r13), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm1
	vfmadd231pd		%ymm13, %ymm15, %ymm5
	vfmadd231pd		%ymm14, %ymm15, %ymm9
	vbroadcastsd	112(%r13), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm2
	vfmadd231pd		%ymm13, %ymm15, %ymm6
	vfmadd231pd		%ymm14, %ymm15, %ymm10
	vmovapd			%ymm12, %ymm3
	vmovapd			%ymm13, %ymm7
	vmovapd			%ymm14, %ymm11

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r13

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif

	movq	ARG6, %r10 // K
	movq	$ 384, %r11 // sdk

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif


	// middle 12x4 of K

	movq	ARG1, %r10 // k
	movq	ARG5, %r11 // D
	movq	ARG3, %r12 // sdd
	sall	$ 5, %r12d
	movq	ARG2, %r13 // V

	addq	%r12, %r13

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r13

	//
	vmovapd			0(%r11), %ymm12
	vmovapd			0(%r11, %r12), %ymm13
	vmovapd			0(%r11, %r12, 2), %ymm14
	vmovapd			%ymm12, %ymm0
	vmovapd			%ymm13, %ymm4
	vmovapd			%ymm14, %ymm8
	//
	vmovapd			32(%r11), %ymm12
	vmovapd			32(%r11, %r12), %ymm13
	vmovapd			32(%r11, %r12, 2), %ymm14
	vbroadcastsd	32(%r13), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm0
	vfmadd231pd		%ymm13, %ymm15, %ymm4
	vfmadd231pd		%ymm14, %ymm15, %ymm8
	vmovapd			%ymm12, %ymm1
	vmovapd			%ymm13, %ymm5
	vmovapd			%ymm14, %ymm9
	//
	vmovapd			64(%r11), %ymm12
	vmovapd			64(%r11, %r12), %ymm13
	vmovapd			64(%r11, %r12, 2), %ymm14
	vbroadcastsd	64(%r13), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm0
	vfmadd231pd		%ymm13, %ymm15, %ymm4
	vfmadd231pd		%ymm14, %ymm15, %ymm8
	vbroadcastsd	72(%r13), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm1
	vfmadd231pd		%ymm13, %ymm15, %ymm5
	vfmadd231pd		%ymm14, %ymm15, %ymm9
	vmovapd			%ymm12, %ymm2
	vmovapd			%ymm13, %ymm6
	vmovapd			%ymm14, %ymm10
	//
	vmovapd			96(%r11), %ymm12
	vmovapd			96(%r11, %r12), %ymm13
	vmovapd			96(%r11, %r12, 2), %ymm14
	vbroadcastsd	96(%r13), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm0
	vfmadd231pd		%ymm13, %ymm15, %ymm4
	vfmadd231pd		%ymm14, %ymm15, %ymm8
	vbroadcastsd	104(%r13), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm1
	vfmadd231pd		%ymm13, %ymm15, %ymm5
	vfmadd231pd		%ymm14, %ymm15, %ymm9
	vbroadcastsd	112(%r13), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm2
	vfmadd231pd		%ymm13, %ymm15, %ymm6
	vfmadd231pd		%ymm14, %ymm15, %ymm10
	vmovapd			%ymm12, %ymm3
	vmovapd			%ymm13, %ymm7
	vmovapd			%ymm14, %ymm11

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r13

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif

	movq	ARG6, %r10 // K
	movq	$ 384, %r11 // sdk

	addq	$ 128, %r10

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif


	// right 12x4 of K

	movq	ARG1, %r10 // k
	movq	ARG5, %r11 // D
	movq	ARG3, %r12 // sdd
	sall	$ 5, %r12d
	movq	ARG2, %r13 // V

	addq	%r12, %r13

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r13

	addq	%r12, %r13

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r13

	//
	vmovapd			0(%r11), %ymm12
	vmovapd			0(%r11, %r12), %ymm13
	vmovapd			0(%r11, %r12, 2), %ymm14
	vmovapd			%ymm12, %ymm0
	vmovapd			%ymm13, %ymm4
	vmovapd			%ymm14, %ymm8
	//
	vmovapd			32(%r11), %ymm12
	vmovapd			32(%r11, %r12), %ymm13
	vmovapd			32(%r11, %r12, 2), %ymm14
	vbroadcastsd	32(%r13), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm0
	vfmadd231pd		%ymm13, %ymm15, %ymm4
	vfmadd231pd		%ymm14, %ymm15, %ymm8
	vmovapd			%ymm12, %ymm1
	vmovapd			%ymm13, %ymm5
	vmovapd			%ymm14, %ymm9
	//
	vmovapd			64(%r11), %ymm12
	vmovapd			64(%r11, %r12), %ymm13
	vmovapd			64(%r11, %r12, 2), %ymm14
	vbroadcastsd	64(%r13), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm0
	vfmadd231pd		%ymm13, %ymm15, %ymm4
	vfmadd231pd		%ymm14, %ymm15, %ymm8
	vbroadcastsd	72(%r13), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm1
	vfmadd231pd		%ymm13, %ymm15, %ymm5
	vfmadd231pd		%ymm14, %ymm15, %ymm9
	vmovapd			%ymm12, %ymm2
	vmovapd			%ymm13, %ymm6
	vmovapd			%ymm14, %ymm10
	//
	vmovapd			96(%r11), %ymm12
	vmovapd			96(%r11, %r12), %ymm13
	vmovapd			96(%r11, %r12, 2), %ymm14
	vbroadcastsd	96(%r13), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm0
	vfmadd231pd		%ymm13, %ymm15, %ymm4
	vfmadd231pd		%ymm14, %ymm15, %ymm8
	vbroadcastsd	104(%r13), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm1
	vfmadd231pd		%ymm13, %ymm15, %ymm5
	vfmadd231pd		%ymm14, %ymm15, %ymm9
	vbroadcastsd	112(%r13), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm2
	vfmadd231pd		%ymm13, %ymm15, %ymm6
	vfmadd231pd		%ymm14, %ymm15, %ymm10
	vmovapd			%ymm12, %ymm3
	vmovapd			%ymm13, %ymm7
	vmovapd			%ymm14, %ymm11

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r13

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif

	movq	ARG6, %r10 // K
	movq	$ 384, %r11 // sdk

	addq	$ 128, %r10

	addq	$ 128, %r10

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif


	// T to right 12x4 of K

	movq	ARG6, %r10 // K
	movq	$ 384, %r11 // sdk
	movq	ARG4, %r12 // T
	movq	$ 384, %r13 // sdt

	addq 	$ 128, %r10
	addq 	$ 128, %r12
	addq	%r13, %r12

	addq 	$ 128, %r10
	addq 	$ 128, %r12
	addq	%r13, %r12

	//
	vmovapd			96(%r10), %ymm3
	vmovapd			96(%r10, %r11), %ymm7
	vmovapd			96(%r10, %r11, 2), %ymm11
	vbroadcastsd	120(%r12), %ymm13
	vmulpd			%ymm3, %ymm13, %ymm3
	vmulpd			%ymm7, %ymm13, %ymm7
	vmulpd			%ymm11, %ymm13, %ymm11
	//
	vmovapd			64(%r10), %ymm2
	vmovapd			64(%r10, %r11), %ymm6
	vmovapd			64(%r10, %r11, 2), %ymm10
	vbroadcastsd	112(%r12), %ymm13
	vfmadd231pd		%ymm2, %ymm13, %ymm3
	vfmadd231pd		%ymm6, %ymm13, %ymm7
	vfmadd231pd		%ymm10, %ymm13, %ymm11
	vbroadcastsd	80(%r12), %ymm13
	vmulpd			%ymm2, %ymm13, %ymm2
	vmulpd			%ymm6, %ymm13, %ymm6
	vmulpd			%ymm10, %ymm13, %ymm10
	//
	vmovapd			32(%r10), %ymm1
	vmovapd			32(%r10, %r11), %ymm5
	vmovapd			32(%r10, %r11, 2), %ymm9
	vbroadcastsd	104(%r12), %ymm13
	vfmadd231pd		%ymm1, %ymm13, %ymm3
	vfmadd231pd		%ymm5, %ymm13, %ymm7
	vfmadd231pd		%ymm9, %ymm13, %ymm11
	vbroadcastsd	72(%r12), %ymm13
	vfmadd231pd		%ymm1, %ymm13, %ymm2
	vfmadd231pd		%ymm5, %ymm13, %ymm6
	vfmadd231pd		%ymm9, %ymm13, %ymm10
	vbroadcastsd	40(%r12), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm1
	vmulpd			%ymm5, %ymm13, %ymm5
	vmulpd			%ymm9, %ymm13, %ymm9
	//
	vmovapd			0(%r10), %ymm0
	vmovapd			0(%r10, %r11), %ymm4
	vmovapd			0(%r10, %r11, 2), %ymm8
	vbroadcastsd	96(%r12), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm3
	vfmadd231pd		%ymm4, %ymm13, %ymm7
	vfmadd231pd		%ymm8, %ymm13, %ymm11
	vbroadcastsd	64(%r12), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm2
	vfmadd231pd		%ymm4, %ymm13, %ymm6
	vfmadd231pd		%ymm8, %ymm13, %ymm10
	vbroadcastsd	32(%r12), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm1
	vfmadd231pd		%ymm4, %ymm13, %ymm5
	vfmadd231pd		%ymm8, %ymm13, %ymm9
	vbroadcastsd	0(%r12), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm0
	vmulpd			%ymm4, %ymm13, %ymm4
	vmulpd			%ymm8, %ymm13, %ymm8

	subq 	$ 128, %r10
//	subq 	$ 128, %r12
	subq	%r13, %r12

	//
	vmovapd			0(%r10), %ymm12
	vmovapd			0(%r10, %r11), %ymm13
	vmovapd			0(%r10, %r11, 2), %ymm14
	vbroadcastsd	96(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm3
	vfmadd231pd		%ymm13, %ymm15, %ymm7
	vfmadd231pd		%ymm14, %ymm15, %ymm11
	vbroadcastsd	64(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm2
	vfmadd231pd		%ymm13, %ymm15, %ymm6
	vfmadd231pd		%ymm14, %ymm15, %ymm10
	vbroadcastsd	32(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm1
	vfmadd231pd		%ymm13, %ymm15, %ymm5
	vfmadd231pd		%ymm14, %ymm15, %ymm9
	vbroadcastsd	0(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm0
	vfmadd231pd		%ymm13, %ymm15, %ymm4
	vfmadd231pd		%ymm14, %ymm15, %ymm8
	//
	vmovapd			32(%r10), %ymm12
	vmovapd			32(%r10, %r11), %ymm13
	vmovapd			32(%r10, %r11, 2), %ymm14
	vbroadcastsd	104(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm3
	vfmadd231pd		%ymm13, %ymm15, %ymm7
	vfmadd231pd		%ymm14, %ymm15, %ymm11
	vbroadcastsd	72(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm2
	vfmadd231pd		%ymm13, %ymm15, %ymm6
	vfmadd231pd		%ymm14, %ymm15, %ymm10
	vbroadcastsd	40(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm1
	vfmadd231pd		%ymm13, %ymm15, %ymm5
	vfmadd231pd		%ymm14, %ymm15, %ymm9
	vbroadcastsd	8(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm0
	vfmadd231pd		%ymm13, %ymm15, %ymm4
	vfmadd231pd		%ymm14, %ymm15, %ymm8
	//
	vmovapd			64(%r10), %ymm12
	vmovapd			64(%r10, %r11), %ymm13
	vmovapd			64(%r10, %r11, 2), %ymm14
	vbroadcastsd	112(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm3
	vfmadd231pd		%ymm13, %ymm15, %ymm7
	vfmadd231pd		%ymm14, %ymm15, %ymm11
	vbroadcastsd	80(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm2
	vfmadd231pd		%ymm13, %ymm15, %ymm6
	vfmadd231pd		%ymm14, %ymm15, %ymm10
	vbroadcastsd	48(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm1
	vfmadd231pd		%ymm13, %ymm15, %ymm5
	vfmadd231pd		%ymm14, %ymm15, %ymm9
	vbroadcastsd	16(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm0
	vfmadd231pd		%ymm13, %ymm15, %ymm4
	vfmadd231pd		%ymm14, %ymm15, %ymm8
	//
	vmovapd			96(%r10), %ymm12
	vmovapd			96(%r10, %r11), %ymm13
	vmovapd			96(%r10, %r11, 2), %ymm14
	vbroadcastsd	120(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm3
	vfmadd231pd		%ymm13, %ymm15, %ymm7
	vfmadd231pd		%ymm14, %ymm15, %ymm11
	vbroadcastsd	88(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm2
	vfmadd231pd		%ymm13, %ymm15, %ymm6
	vfmadd231pd		%ymm14, %ymm15, %ymm10
	vbroadcastsd	56(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm1
	vfmadd231pd		%ymm13, %ymm15, %ymm5
	vfmadd231pd		%ymm14, %ymm15, %ymm9
	vbroadcastsd	24(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm0
	vfmadd231pd		%ymm13, %ymm15, %ymm4
	vfmadd231pd		%ymm14, %ymm15, %ymm8


	subq 	$ 128, %r10
//	subq 	$ 128, %r12
	subq	%r13, %r12

	//
	vmovapd			0(%r10), %ymm12
	vmovapd			0(%r10, %r11), %ymm13
	vmovapd			0(%r10, %r11, 2), %ymm14
	vbroadcastsd	96(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm3
	vfmadd231pd		%ymm13, %ymm15, %ymm7
	vfmadd231pd		%ymm14, %ymm15, %ymm11
	vbroadcastsd	64(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm2
	vfmadd231pd		%ymm13, %ymm15, %ymm6
	vfmadd231pd		%ymm14, %ymm15, %ymm10
	vbroadcastsd	32(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm1
	vfmadd231pd		%ymm13, %ymm15, %ymm5
	vfmadd231pd		%ymm14, %ymm15, %ymm9
	vbroadcastsd	0(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm0
	vfmadd231pd		%ymm13, %ymm15, %ymm4
	vfmadd231pd		%ymm14, %ymm15, %ymm8
	//
	vmovapd			32(%r10), %ymm12
	vmovapd			32(%r10, %r11), %ymm13
	vmovapd			32(%r10, %r11, 2), %ymm14
	vbroadcastsd	104(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm3
	vfmadd231pd		%ymm13, %ymm15, %ymm7
	vfmadd231pd		%ymm14, %ymm15, %ymm11
	vbroadcastsd	72(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm2
	vfmadd231pd		%ymm13, %ymm15, %ymm6
	vfmadd231pd		%ymm14, %ymm15, %ymm10
	vbroadcastsd	40(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm1
	vfmadd231pd		%ymm13, %ymm15, %ymm5
	vfmadd231pd		%ymm14, %ymm15, %ymm9
	vbroadcastsd	8(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm0
	vfmadd231pd		%ymm13, %ymm15, %ymm4
	vfmadd231pd		%ymm14, %ymm15, %ymm8
	//
	vmovapd			64(%r10), %ymm12
	vmovapd			64(%r10, %r11), %ymm13
	vmovapd			64(%r10, %r11, 2), %ymm14
	vbroadcastsd	112(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm3
	vfmadd231pd		%ymm13, %ymm15, %ymm7
	vfmadd231pd		%ymm14, %ymm15, %ymm11
	vbroadcastsd	80(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm2
	vfmadd231pd		%ymm13, %ymm15, %ymm6
	vfmadd231pd		%ymm14, %ymm15, %ymm10
	vbroadcastsd	48(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm1
	vfmadd231pd		%ymm13, %ymm15, %ymm5
	vfmadd231pd		%ymm14, %ymm15, %ymm9
	vbroadcastsd	16(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm0
	vfmadd231pd		%ymm13, %ymm15, %ymm4
	vfmadd231pd		%ymm14, %ymm15, %ymm8
	//
	vmovapd			96(%r10), %ymm12
	vmovapd			96(%r10, %r11), %ymm13
	vmovapd			96(%r10, %r11, 2), %ymm14
	vbroadcastsd	120(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm3
	vfmadd231pd		%ymm13, %ymm15, %ymm7
	vfmadd231pd		%ymm14, %ymm15, %ymm11
	vbroadcastsd	88(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm2
	vfmadd231pd		%ymm13, %ymm15, %ymm6
	vfmadd231pd		%ymm14, %ymm15, %ymm10
	vbroadcastsd	56(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm1
	vfmadd231pd		%ymm13, %ymm15, %ymm5
	vfmadd231pd		%ymm14, %ymm15, %ymm9
	vbroadcastsd	24(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm0
	vfmadd231pd		%ymm13, %ymm15, %ymm4
	vfmadd231pd		%ymm14, %ymm15, %ymm8


	movq	ARG6, %r10 // K
	movq	$ 384, %r11 // sdk

	addq	$ 128, %r10
	addq	$ 128, %r10

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif


	// T to middle 12x4 of K

	movq	ARG6, %r10 // K
	movq	$ 384, %r11 // sdk
	movq	ARG4, %r12 // T
	movq	$ 384, %r13 // sdt

	addq 	$ 128, %r10
	addq 	$ 128, %r12
	addq	%r13, %r12

	//
	vmovapd			96(%r10), %ymm3
	vmovapd			96(%r10, %r11), %ymm7
	vmovapd			96(%r10, %r11, 2), %ymm11
	vbroadcastsd	120(%r12), %ymm13
	vmulpd			%ymm3, %ymm13, %ymm3
	vmulpd			%ymm7, %ymm13, %ymm7
	vmulpd			%ymm11, %ymm13, %ymm11
	//
	vmovapd			64(%r10), %ymm2
	vmovapd			64(%r10, %r11), %ymm6
	vmovapd			64(%r10, %r11, 2), %ymm10
	vbroadcastsd	112(%r12), %ymm13
	vfmadd231pd		%ymm2, %ymm13, %ymm3
	vfmadd231pd		%ymm6, %ymm13, %ymm7
	vfmadd231pd		%ymm10, %ymm13, %ymm11
	vbroadcastsd	80(%r12), %ymm13
	vmulpd			%ymm2, %ymm13, %ymm2
	vmulpd			%ymm6, %ymm13, %ymm6
	vmulpd			%ymm10, %ymm13, %ymm10
	//
	vmovapd			32(%r10), %ymm1
	vmovapd			32(%r10, %r11), %ymm5
	vmovapd			32(%r10, %r11, 2), %ymm9
	vbroadcastsd	104(%r12), %ymm13
	vfmadd231pd		%ymm1, %ymm13, %ymm3
	vfmadd231pd		%ymm5, %ymm13, %ymm7
	vfmadd231pd		%ymm9, %ymm13, %ymm11
	vbroadcastsd	72(%r12), %ymm13
	vfmadd231pd		%ymm1, %ymm13, %ymm2
	vfmadd231pd		%ymm5, %ymm13, %ymm6
	vfmadd231pd		%ymm9, %ymm13, %ymm10
	vbroadcastsd	40(%r12), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm1
	vmulpd			%ymm5, %ymm13, %ymm5
	vmulpd			%ymm9, %ymm13, %ymm9
	//
	vmovapd			0(%r10), %ymm0
	vmovapd			0(%r10, %r11), %ymm4
	vmovapd			0(%r10, %r11, 2), %ymm8
	vbroadcastsd	96(%r12), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm3
	vfmadd231pd		%ymm4, %ymm13, %ymm7
	vfmadd231pd		%ymm8, %ymm13, %ymm11
	vbroadcastsd	64(%r12), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm2
	vfmadd231pd		%ymm4, %ymm13, %ymm6
	vfmadd231pd		%ymm8, %ymm13, %ymm10
	vbroadcastsd	32(%r12), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm1
	vfmadd231pd		%ymm4, %ymm13, %ymm5
	vfmadd231pd		%ymm8, %ymm13, %ymm9
	vbroadcastsd	0(%r12), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm0
	vmulpd			%ymm4, %ymm13, %ymm4
	vmulpd			%ymm8, %ymm13, %ymm8

	subq 	$ 128, %r10
//	subq 	$ 128, %r12
	subq	%r13, %r12

	//
	vmovapd			0(%r10), %ymm12
	vmovapd			0(%r10, %r11), %ymm13
	vmovapd			0(%r10, %r11, 2), %ymm14
	vbroadcastsd	96(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm3
	vfmadd231pd		%ymm13, %ymm15, %ymm7
	vfmadd231pd		%ymm14, %ymm15, %ymm11
	vbroadcastsd	64(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm2
	vfmadd231pd		%ymm13, %ymm15, %ymm6
	vfmadd231pd		%ymm14, %ymm15, %ymm10
	vbroadcastsd	32(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm1
	vfmadd231pd		%ymm13, %ymm15, %ymm5
	vfmadd231pd		%ymm14, %ymm15, %ymm9
	vbroadcastsd	0(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm0
	vfmadd231pd		%ymm13, %ymm15, %ymm4
	vfmadd231pd		%ymm14, %ymm15, %ymm8
	//
	vmovapd			32(%r10), %ymm12
	vmovapd			32(%r10, %r11), %ymm13
	vmovapd			32(%r10, %r11, 2), %ymm14
	vbroadcastsd	104(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm3
	vfmadd231pd		%ymm13, %ymm15, %ymm7
	vfmadd231pd		%ymm14, %ymm15, %ymm11
	vbroadcastsd	72(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm2
	vfmadd231pd		%ymm13, %ymm15, %ymm6
	vfmadd231pd		%ymm14, %ymm15, %ymm10
	vbroadcastsd	40(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm1
	vfmadd231pd		%ymm13, %ymm15, %ymm5
	vfmadd231pd		%ymm14, %ymm15, %ymm9
	vbroadcastsd	8(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm0
	vfmadd231pd		%ymm13, %ymm15, %ymm4
	vfmadd231pd		%ymm14, %ymm15, %ymm8
	//
	vmovapd			64(%r10), %ymm12
	vmovapd			64(%r10, %r11), %ymm13
	vmovapd			64(%r10, %r11, 2), %ymm14
	vbroadcastsd	112(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm3
	vfmadd231pd		%ymm13, %ymm15, %ymm7
	vfmadd231pd		%ymm14, %ymm15, %ymm11
	vbroadcastsd	80(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm2
	vfmadd231pd		%ymm13, %ymm15, %ymm6
	vfmadd231pd		%ymm14, %ymm15, %ymm10
	vbroadcastsd	48(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm1
	vfmadd231pd		%ymm13, %ymm15, %ymm5
	vfmadd231pd		%ymm14, %ymm15, %ymm9
	vbroadcastsd	16(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm0
	vfmadd231pd		%ymm13, %ymm15, %ymm4
	vfmadd231pd		%ymm14, %ymm15, %ymm8
	//
	vmovapd			96(%r10), %ymm12
	vmovapd			96(%r10, %r11), %ymm13
	vmovapd			96(%r10, %r11, 2), %ymm14
	vbroadcastsd	120(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm3
	vfmadd231pd		%ymm13, %ymm15, %ymm7
	vfmadd231pd		%ymm14, %ymm15, %ymm11
	vbroadcastsd	88(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm2
	vfmadd231pd		%ymm13, %ymm15, %ymm6
	vfmadd231pd		%ymm14, %ymm15, %ymm10
	vbroadcastsd	56(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm1
	vfmadd231pd		%ymm13, %ymm15, %ymm5
	vfmadd231pd		%ymm14, %ymm15, %ymm9
	vbroadcastsd	24(%r12), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm0
	vfmadd231pd		%ymm13, %ymm15, %ymm4
	vfmadd231pd		%ymm14, %ymm15, %ymm8


	movq	ARG6, %r10 // K
	movq	$ 384, %r11 // sdk

	addq	$ 128, %r10

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif


	// T to left 12x4 of K

	movq	ARG6, %r10 // K
	movq	$ 384, %r11 // sdk
	movq	ARG4, %r12 // T
	movq	$ 384, %r13 // sdt

	//
	vmovapd			96(%r10), %ymm3
	vmovapd			96(%r10, %r11), %ymm7
	vmovapd			96(%r10, %r11, 2), %ymm11
	vbroadcastsd	120(%r12), %ymm13
	vmulpd			%ymm3, %ymm13, %ymm3
	vmulpd			%ymm7, %ymm13, %ymm7
	vmulpd			%ymm11, %ymm13, %ymm11
	//
	vmovapd			64(%r10), %ymm2
	vmovapd			64(%r10, %r11), %ymm6
	vmovapd			64(%r10, %r11, 2), %ymm10
	vbroadcastsd	112(%r12), %ymm13
	vfmadd231pd		%ymm2, %ymm13, %ymm3
	vfmadd231pd		%ymm6, %ymm13, %ymm7
	vfmadd231pd		%ymm10, %ymm13, %ymm11
	vbroadcastsd	80(%r12), %ymm13
	vmulpd			%ymm2, %ymm13, %ymm2
	vmulpd			%ymm6, %ymm13, %ymm6
	vmulpd			%ymm10, %ymm13, %ymm10
	//
	vmovapd			32(%r10), %ymm1
	vmovapd			32(%r10, %r11), %ymm5
	vmovapd			32(%r10, %r11, 2), %ymm9
	vbroadcastsd	104(%r12), %ymm13
	vfmadd231pd		%ymm1, %ymm13, %ymm3
	vfmadd231pd		%ymm5, %ymm13, %ymm7
	vfmadd231pd		%ymm9, %ymm13, %ymm11
	vbroadcastsd	72(%r12), %ymm13
	vfmadd231pd		%ymm1, %ymm13, %ymm2
	vfmadd231pd		%ymm5, %ymm13, %ymm6
	vfmadd231pd		%ymm9, %ymm13, %ymm10
	vbroadcastsd	40(%r12), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm1
	vmulpd			%ymm5, %ymm13, %ymm5
	vmulpd			%ymm9, %ymm13, %ymm9
	//
	vmovapd			0(%r10), %ymm0
	vmovapd			0(%r10, %r11), %ymm4
	vmovapd			0(%r10, %r11, 2), %ymm8
	vbroadcastsd	96(%r12), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm3
	vfmadd231pd		%ymm4, %ymm13, %ymm7
	vfmadd231pd		%ymm8, %ymm13, %ymm11
	vbroadcastsd	64(%r12), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm2
	vfmadd231pd		%ymm4, %ymm13, %ymm6
	vfmadd231pd		%ymm8, %ymm13, %ymm10
	vbroadcastsd	32(%r12), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm1
	vfmadd231pd		%ymm4, %ymm13, %ymm5
	vfmadd231pd		%ymm8, %ymm13, %ymm9
	vbroadcastsd	0(%r12), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm0
	vmulpd			%ymm4, %ymm13, %ymm4
	vmulpd			%ymm8, %ymm13, %ymm8

	movq	ARG6, %r10 // K
	movq	$ 384, %r11 // sdk

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
	CALL(inner_store_12x4_lib4)
#endif



	movq	ARG1, %r10
	movq	ARG6, %r11 // K
	movq	$ 384, %r12
	movq	ARG2, %r13 // V
	movq	ARG3, %r14 // sdd
	sall	$ 5, %r14d
	movq	ARG5, %r15 // D


	// left 12x4

	// preload
	vmovapd 0(%r11), %ymm13 // A0[0]
	vmovapd 0(%r11, %r12), %ymm14 // A1[0]
	vmovapd 0(%r11, %r12, 2), %ymm15 // A2[0]

	// rank 4

//	prefetcht0	 0(%r13, %r14, 2) // software prefetch
//	prefetcht0	64(%r13, %r14, 2) // software prefetch

	// unroll 0
	vmovapd			0(%r15), %ymm0
	vaddpd			%ymm13, %ymm0, %ymm0
	vmovapd			%ymm0, 0(%r15)
	vmovapd			0(%r15, %r14), %ymm4
	vaddpd			%ymm14, %ymm4, %ymm4
	vmovapd			%ymm4, 0(%r15, %r14)
	vmovapd			0(%r15, %r14, 2), %ymm8
	vaddpd			%ymm15, %ymm8, %ymm8
	vmovapd			%ymm8, 0(%r15, %r14, 2)

	vbroadcastsd	32(%r13), %ymm12
	vmovapd			32(%r15), %ymm1
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vmovapd			32(%r15, %r14), %ymm5
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vmovapd			32(%r15, %r14, 2), %ymm9
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	64(%r13), %ymm12
	vmovapd			64(%r15), %ymm2
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vmovapd			64(%r15, %r14), %ymm6
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vmovapd			64(%r15, %r14, 2), %ymm10
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	96(%r13), %ymm12
	vmovapd			96(%r15), %ymm3
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			96(%r15, %r14), %ymm7
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			96(%r15, %r14, 2), %ymm11
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 1
	vmovapd			32(%r11), %ymm13 // A0
	vaddpd			%ymm13, %ymm1, %ymm1
	vmovapd			%ymm1, 32(%r15)
	vmovapd			32(%r11, %r12), %ymm14 // A1
	vaddpd			%ymm14, %ymm5, %ymm5
	vmovapd			%ymm5, 32(%r15, %r14)
	vmovapd			32(%r11, %r12, 2), %ymm15 // A1
	vaddpd			%ymm15, %ymm9, %ymm9
	vmovapd			%ymm9, 32(%r15, %r14, 2)

	vbroadcastsd	72(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	104(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			64(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			64(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			64(%r11, %r12, 2), %ymm15 // A1

	// unroll 2
	vaddpd			%ymm13, %ymm2, %ymm2
	vmovapd			%ymm2, 64(%r15)
	vaddpd			%ymm14, %ymm6, %ymm6
	vmovapd			%ymm6, 64(%r15, %r14)
	vaddpd			%ymm15, %ymm10, %ymm10
	vmovapd			%ymm10, 64(%r15, %r14, 2)

	vbroadcastsd	112(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			96(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			96(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			96(%r11, %r12, 2), %ymm15 // A1

	// unroll 3
	vaddpd			%ymm13, %ymm3, %ymm3
	vmovapd			%ymm3, 96(%r15)
	vaddpd			%ymm14, %ymm7, %ymm7
	vmovapd			%ymm7, 96(%r15, %r14)
	vaddpd			%ymm15, %ymm11, %ymm11
	vmovapd			%ymm11, 96(%r15, %r14, 2)

	addq	$ 128, %r13
	addq	$ 128, %r15
	subl	$ 4, %r10d


	// middle 12x4

	// preload
	vmovapd 0(%r11), %ymm13 // A0[0]
	vmovapd 0(%r11, %r12), %ymm14 // A1[0]
	vmovapd 0(%r11, %r12, 2), %ymm15 // A1[0]

	// rank 4

//	prefetcht0	 0(%r13, %r14, 2) // software prefetch
//	prefetcht0	64(%r13, %r14, 2) // software prefetch

	// unroll 0
	vbroadcastsd	0(%r13), %ymm12
	vmovapd			0(%r15), %ymm0
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			0(%r15, %r14), %ymm4
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			0(%r15, %r14, 2), %ymm8
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	32(%r13), %ymm12
	vmovapd			32(%r15), %ymm1
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vmovapd			32(%r15, %r14), %ymm5
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vmovapd			32(%r15, %r14, 2), %ymm9
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	64(%r13), %ymm12
	vmovapd			64(%r15), %ymm2
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vmovapd			64(%r15, %r14), %ymm6
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vmovapd			64(%r15, %r14, 2), %ymm10
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	96(%r13), %ymm12
	vmovapd			96(%r15), %ymm3
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			96(%r15, %r14), %ymm7
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			96(%r15, %r14, 2), %ymm11
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 1
	vbroadcastsd	8(%r13), %ymm12
	vmovapd			32(%r11), %ymm13 // A0
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			32(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			32(%r11, %r12, 2), %ymm15 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	40(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	72(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	104(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			64(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			64(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			64(%r11, %r12, 2), %ymm15 // A1

	// unroll 2
	vbroadcastsd	16(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	48(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	80(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	112(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			96(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			96(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			96(%r11, %r12, 2), %ymm15 // A1

	// unroll 3
	vbroadcastsd	24(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	56(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	88(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	120(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			128+0(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			128+0(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			128+0(%r11, %r12, 2), %ymm15 // A1


	// rank 8

//	prefetcht0	 0(%r13, %r14, 2) // software prefetch
//	prefetcht0	64(%r13, %r14, 2) // software prefetch

	// unroll 0
	vaddpd			%ymm13, %ymm0, %ymm0
	vmovapd			%ymm0, 0(%r15)
	vaddpd			%ymm14, %ymm4, %ymm4
	vmovapd			%ymm4, 0(%r15, %r14)
	vaddpd			%ymm15, %ymm8, %ymm8
	vmovapd			%ymm8, 0(%r15, %r14, 2)

	vbroadcastsd	32(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	64(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	96(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			128+32(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			128+32(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			128+32(%r11, %r12, 2), %ymm15 // A1

	// unroll 1
	vaddpd			%ymm13, %ymm1, %ymm1
	vmovapd			%ymm1, 32(%r15)
	vaddpd			%ymm14, %ymm5, %ymm5
	vmovapd			%ymm5, 32(%r15, %r14)
	vaddpd			%ymm15, %ymm9, %ymm9
	vmovapd			%ymm9, 32(%r15, %r14, 2)

	vbroadcastsd	72(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	104(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			128+64(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			128+64(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			128+64(%r11, %r12, 2), %ymm15 // A1

	// unroll 2
	vaddpd			%ymm13, %ymm2, %ymm2
	vmovapd			%ymm2, 64(%r15)
	vaddpd			%ymm14, %ymm6, %ymm6
	vmovapd			%ymm6, 64(%r15, %r14)
	vaddpd			%ymm15, %ymm10, %ymm10
	vmovapd			%ymm10, 64(%r15, %r14, 2)

	vbroadcastsd	112(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			128+96(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			128+96(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			128+96(%r11, %r12, 2), %ymm15 // A1

	// unroll 3
	vaddpd			%ymm13, %ymm3, %ymm3
	vmovapd			%ymm3, 96(%r15)
	vaddpd			%ymm14, %ymm7, %ymm7
	vmovapd			%ymm7, 96(%r15, %r14)
	vaddpd			%ymm15, %ymm11, %ymm11
	vmovapd			%ymm11, 96(%r15, %r14, 2)

	addq	$ 128, %r13
	addq	$ 128, %r15
	subl	$ 4, %r10d


	// right 12x4

	// preload
	vmovapd 0(%r11), %ymm13 // A0[0]
	vmovapd 0(%r11, %r12), %ymm14 // A1[0]
	vmovapd 0(%r11, %r12, 2), %ymm15 // A1[0]

	// rank 4

//	prefetcht0	 0(%r13, %r14, 2) // software prefetch
//	prefetcht0	64(%r13, %r14, 2) // software prefetch

	// unroll 0
	vbroadcastsd	0(%r13), %ymm12
	vmovapd			0(%r15), %ymm0
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			0(%r15, %r14), %ymm4
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			0(%r15, %r14, 2), %ymm8
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	32(%r13), %ymm12
	vmovapd			32(%r15), %ymm1
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vmovapd			32(%r15, %r14), %ymm5
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vmovapd			32(%r15, %r14, 2), %ymm9
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	64(%r13), %ymm12
	vmovapd			64(%r15), %ymm2
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vmovapd			64(%r15, %r14), %ymm6
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vmovapd			64(%r15, %r14, 2), %ymm10
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	96(%r13), %ymm12
	vmovapd			96(%r15), %ymm3
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			96(%r15, %r14), %ymm7
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			96(%r15, %r14, 2), %ymm11
	vfmadd231pd		%ymm15, %ymm12, %ymm11

	// unroll 1
	vbroadcastsd	8(%r13), %ymm12
	vmovapd			32(%r11), %ymm13 // A0
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			32(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			32(%r11, %r12, 2), %ymm15 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	40(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	72(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	104(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			64(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			64(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			64(%r11, %r12, 2), %ymm15 // A1

	// unroll 2
	vbroadcastsd	16(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	48(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	80(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	112(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			96(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			96(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			96(%r11, %r12, 2), %ymm15 // A1

	// unroll 3
	vbroadcastsd	24(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	56(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	88(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	120(%r13), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			128+0(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			128+0(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			128+0(%r11, %r12, 2), %ymm15 // A1


	// rank 8

//	prefetcht0	 0(%r13, %r14, 2) // software prefetch
//	prefetcht0	64(%r13, %r14, 2) // software prefetch

	// unroll 0
	vbroadcastsd	0(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	32(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	64(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	96(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			128+32(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			128+32(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			128+32(%r11, %r12, 2), %ymm15 // A1

	// unroll 1
	vbroadcastsd	8(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	40(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	72(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	104(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			128+64(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			128+64(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			128+64(%r11, %r12, 2), %ymm15 // A1

	// unroll 2
	vbroadcastsd	16(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	48(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	80(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	112(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			128+96(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			128+96(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			128+96(%r11, %r12, 2), %ymm15 // A1

	// unroll 3
	vbroadcastsd	24(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vfmadd231pd		%ymm15, %ymm12, %ymm8

	vbroadcastsd	56(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	88(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	120(%r13, %r14), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			256+0(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			256+0(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			256+0(%r11, %r12, 2), %ymm15 // A1


	// rank 12

//	prefetcht0	 0(%r13, %r14, 2) // software prefetch
//	prefetcht0	64(%r13, %r14, 2) // software prefetch

	// unroll 0
	vaddpd			%ymm13, %ymm0, %ymm0
	vmovapd			%ymm0, 0(%r15)
	vaddpd			%ymm14, %ymm4, %ymm4
	vmovapd			%ymm4, 0(%r15, %r14)
	vaddpd			%ymm15, %ymm8, %ymm8
	vmovapd			%ymm8, 0(%r15, %r14, 2)

	vbroadcastsd	32(%r13, %r14, 2), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vfmadd231pd		%ymm15, %ymm12, %ymm9

	vbroadcastsd	64(%r13, %r14, 2), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	96(%r13, %r14, 2), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			256+32(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			256+32(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			256+32(%r11, %r12, 2), %ymm15 // A1

	// unroll 1
	vaddpd			%ymm13, %ymm1, %ymm1
	vmovapd			%ymm1, 32(%r15)
	vaddpd			%ymm14, %ymm5, %ymm5
	vmovapd			%ymm5, 32(%r15, %r14)
	vaddpd			%ymm15, %ymm9, %ymm9
	vmovapd			%ymm9, 32(%r15, %r14, 2)

	vbroadcastsd	72(%r13, %r14, 2), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vfmadd231pd		%ymm15, %ymm12, %ymm10

	vbroadcastsd	104(%r13, %r14, 2), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			256+64(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			256+64(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			256+64(%r11, %r12, 2), %ymm15 // A1

	// unroll 2
	vaddpd			%ymm13, %ymm2, %ymm2
	vmovapd			%ymm2, 64(%r15)
	vaddpd			%ymm14, %ymm6, %ymm6
	vmovapd			%ymm6, 64(%r15, %r14)
	vaddpd			%ymm15, %ymm10, %ymm10
	vmovapd			%ymm10, 64(%r15, %r14, 2)

	vbroadcastsd	112(%r13, %r14, 2), %ymm12
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	vmovapd			256+96(%r11), %ymm13 // A0
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	vmovapd			256+96(%r11, %r12), %ymm14 // A1
	vfmadd231pd		%ymm15, %ymm12, %ymm11
	vmovapd			256+96(%r11, %r12, 2), %ymm15 // A1

	// unroll 3
	vaddpd			%ymm13, %ymm3, %ymm3
	vmovapd			%ymm3, 96(%r15)
	vaddpd			%ymm14, %ymm7, %ymm7
	vmovapd			%ymm7, 96(%r15, %r14)
	vaddpd			%ymm15, %ymm11, %ymm11
	vmovapd			%ymm11, 96(%r15, %r14, 2)

	addq	$ 128, %r13
	addq	$ 128, %r15
	subl	$ 4, %r10d



#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEBP_ADD_NN_12X12_LIB4
#else
	CALL(inner_kernel_dgebp_add_nn_12x12_lib4)
#endif


100:
	EPILOGUE

	ret

	FUN_END(kernel_dlarfb12_rn_12_lib4)





//                               1         2           3         4           5          6           7
// void kernel_dlarfb12_rn_4_lib4(int kmax, double *pV, int sdd, double *pT, double *pD, double *pK, int km);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dlarfb12_rn_4_lib4)
	
	PROLOGUE

	// zero accumulation registers

//	ZERO_ACC
	
	movq	ARG1, %r10 // k
	movq	ARG5, %r11 // D
	movq	ARG2, %r12 // V
	movq	ARG3, %r13 // sdd
	sall	$ 5, %r13d

	//
	vmovapd			0(%r11), %ymm12
	vmovapd			%ymm12, %ymm0
	//
	vmovapd			32(%r11), %ymm12
	vbroadcastsd	32(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vmovapd			%ymm12, %ymm1
	//
	vmovapd			64(%r11), %ymm12
	vbroadcastsd	64(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	72(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vmovapd			%ymm12, %ymm2
	//
	vmovapd			96(%r11), %ymm12
	vbroadcastsd	96(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	104(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	112(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vmovapd			%ymm12, %ymm3
	//
	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r12
	//
	vmovapd			0(%r11), %ymm12
	vbroadcastsd	0(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	8(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	16(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	24(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vmovapd			%ymm12, %ymm4
	//
	vmovapd			32(%r11), %ymm12
	vbroadcastsd	32(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	40(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	48(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	56(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	32(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vmovapd			%ymm12, %ymm5
	//
	vmovapd			64(%r11), %ymm12
	vbroadcastsd	64(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	72(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	80(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	88(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	64(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	72(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vmovapd			%ymm12, %ymm6
	//
	vmovapd			96(%r11), %ymm12
	vbroadcastsd	96(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	104(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	112(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	120(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	96(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	104(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	112(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vmovapd			%ymm12, %ymm7
	//
	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r12
	//
	vmovapd			0(%r11), %ymm12
	vbroadcastsd	0(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	8(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	16(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	24(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	0(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	8(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	16(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	24(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vmovapd			%ymm12, %ymm8
	//
	vmovapd			32(%r11), %ymm12
	vbroadcastsd	32(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	40(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	48(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	56(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	32(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	40(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	48(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	56(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	32(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vmovapd			%ymm12, %ymm9
	//
	vmovapd			64(%r11), %ymm12
	vbroadcastsd	64(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	72(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	80(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	88(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	64(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	72(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	80(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	88(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	64(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	72(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vmovapd			%ymm12, %ymm10
	//
	vmovapd			96(%r11), %ymm12
	vbroadcastsd	96(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	104(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	112(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	120(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	96(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	104(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	112(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	120(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	96(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	104(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	112(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vmovapd			%ymm12, %ymm11
	//
	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r12

	movq	%r11, %r14
	movq	%r12, %r11
	movq	%r13, %r12
	movq	%r14, %r13

#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB4
#else
	CALL(inner_tran_12x4_lib4)
#endif
#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif
#if MACRO_LEVEL>=1
	INNER_TRAN_12X4_LIB4
#else
	CALL(inner_tran_12x4_lib4)
#endif

	movq	ARG4, %r11 // T
	movq	$ 384, %r12 // sdt !!!!!!!!!!!!!!!!!!!!!!!!!

	//
	vbroadcastsd	376(%r11, %r12, 2), %ymm13
	vmulpd			%ymm11, %ymm13, %ymm11
	//
	vbroadcastsd	368(%r11, %r12, 2), %ymm13
	vfmadd231pd		%ymm10, %ymm13, %ymm11
	vbroadcastsd	336(%r11, %r12, 2), %ymm13
	vmulpd			%ymm10, %ymm13, %ymm10
	//
	vbroadcastsd	360(%r11, %r12, 2), %ymm13
	vfmadd231pd		%ymm9, %ymm13, %ymm11
	vbroadcastsd	328(%r11, %r12, 2), %ymm13
	vfmadd231pd		%ymm9, %ymm13, %ymm10
	vbroadcastsd	296(%r11, %r12, 2), %ymm13
	vmulpd			%ymm9, %ymm13, %ymm9
	//
	vbroadcastsd	352(%r11, %r12, 2), %ymm13
	vfmadd231pd		%ymm8, %ymm13, %ymm11
	vbroadcastsd	320(%r11, %r12, 2), %ymm13
	vfmadd231pd		%ymm8, %ymm13, %ymm10
	vbroadcastsd	288(%r11, %r12, 2), %ymm13
	vfmadd231pd		%ymm8, %ymm13, %ymm9
	vbroadcastsd	256(%r11, %r12, 2), %ymm13
	vmulpd			%ymm8, %ymm13, %ymm8
	//
	vbroadcastsd	376(%r11, %r12), %ymm13
	vfmadd231pd		%ymm7, %ymm13, %ymm11
	vbroadcastsd	344(%r11, %r12), %ymm13
	vfmadd231pd		%ymm7, %ymm13, %ymm10
	vbroadcastsd	312(%r11, %r12), %ymm13
	vfmadd231pd		%ymm7, %ymm13, %ymm9
	vbroadcastsd	280(%r11, %r12), %ymm13
	vfmadd231pd		%ymm7, %ymm13, %ymm8
	vbroadcastsd	248(%r11, %r12), %ymm13
	vmulpd			%ymm7, %ymm13, %ymm7
	//
	vbroadcastsd	368(%r11, %r12), %ymm13
	vfmadd231pd		%ymm6, %ymm13, %ymm11
	vbroadcastsd	336(%r11, %r12), %ymm13
	vfmadd231pd		%ymm6, %ymm13, %ymm10
	vbroadcastsd	304(%r11, %r12), %ymm13
	vfmadd231pd		%ymm6, %ymm13, %ymm9
	vbroadcastsd	272(%r11, %r12), %ymm13
	vfmadd231pd		%ymm6, %ymm13, %ymm8
	vbroadcastsd	240(%r11, %r12), %ymm13
	vfmadd231pd		%ymm6, %ymm13, %ymm7
	vbroadcastsd	208(%r11, %r12), %ymm13
	vmulpd			%ymm6, %ymm13, %ymm6
	//
	vbroadcastsd	360(%r11, %r12), %ymm13
	vfmadd231pd		%ymm5, %ymm13, %ymm11
	vbroadcastsd	328(%r11, %r12), %ymm13
	vfmadd231pd		%ymm5, %ymm13, %ymm10
	vbroadcastsd	296(%r11, %r12), %ymm13
	vfmadd231pd		%ymm5, %ymm13, %ymm9
	vbroadcastsd	264(%r11, %r12), %ymm13
	vfmadd231pd		%ymm5, %ymm13, %ymm8
	vbroadcastsd	232(%r11, %r12), %ymm13
	vfmadd231pd		%ymm5, %ymm13, %ymm7
	vbroadcastsd	200(%r11, %r12), %ymm13
	vfmadd231pd		%ymm5, %ymm13, %ymm6
	vbroadcastsd	168(%r11, %r12), %ymm13
	vmulpd			%ymm5, %ymm13, %ymm5
	//
	vbroadcastsd	352(%r11, %r12), %ymm13
	vfmadd231pd		%ymm4, %ymm13, %ymm11
	vbroadcastsd	320(%r11, %r12), %ymm13
	vfmadd231pd		%ymm4, %ymm13, %ymm10
	vbroadcastsd	288(%r11, %r12), %ymm13
	vfmadd231pd		%ymm4, %ymm13, %ymm9
	vbroadcastsd	256(%r11, %r12), %ymm13
	vfmadd231pd		%ymm4, %ymm13, %ymm8
	vbroadcastsd	224(%r11, %r12), %ymm13
	vfmadd231pd		%ymm4, %ymm13, %ymm7
	vbroadcastsd	192(%r11, %r12), %ymm13
	vfmadd231pd		%ymm4, %ymm13, %ymm6
	vbroadcastsd	160(%r11, %r12), %ymm13
	vfmadd231pd		%ymm4, %ymm13, %ymm5
	vbroadcastsd	128(%r11, %r12), %ymm13
	vmulpd			%ymm4, %ymm13, %ymm4
	//
	vbroadcastsd	376(%r11), %ymm13
	vfmadd231pd		%ymm3, %ymm13, %ymm11
	vbroadcastsd	344(%r11), %ymm13
	vfmadd231pd		%ymm3, %ymm13, %ymm10
	vbroadcastsd	312(%r11), %ymm13
	vfmadd231pd		%ymm3, %ymm13, %ymm9
	vbroadcastsd	280(%r11), %ymm13
	vfmadd231pd		%ymm3, %ymm13, %ymm8
	vbroadcastsd	248(%r11), %ymm13
	vfmadd231pd		%ymm3, %ymm13, %ymm7
	vbroadcastsd	216(%r11), %ymm13
	vfmadd231pd		%ymm3, %ymm13, %ymm6
	vbroadcastsd	184(%r11), %ymm13
	vfmadd231pd		%ymm3, %ymm13, %ymm5
	vbroadcastsd	152(%r11), %ymm13
	vfmadd231pd		%ymm3, %ymm13, %ymm4
	vbroadcastsd	120(%r11), %ymm13
	vmulpd			%ymm3, %ymm13, %ymm3
	//
	vbroadcastsd	368(%r11), %ymm13
	vfmadd231pd		%ymm2, %ymm13, %ymm11
	vbroadcastsd	336(%r11), %ymm13
	vfmadd231pd		%ymm2, %ymm13, %ymm10
	vbroadcastsd	304(%r11), %ymm13
	vfmadd231pd		%ymm2, %ymm13, %ymm9
	vbroadcastsd	272(%r11), %ymm13
	vfmadd231pd		%ymm2, %ymm13, %ymm8
	vbroadcastsd	240(%r11), %ymm13
	vfmadd231pd		%ymm2, %ymm13, %ymm7
	vbroadcastsd	208(%r11), %ymm13
	vfmadd231pd		%ymm2, %ymm13, %ymm6
	vbroadcastsd	176(%r11), %ymm13
	vfmadd231pd		%ymm2, %ymm13, %ymm5
	vbroadcastsd	144(%r11), %ymm13
	vfmadd231pd		%ymm2, %ymm13, %ymm4
	vbroadcastsd	112(%r11), %ymm13
	vfmadd231pd		%ymm2, %ymm13, %ymm3
	vbroadcastsd	80(%r11), %ymm13
	vmulpd			%ymm2, %ymm13, %ymm2
	//
	vbroadcastsd	360(%r11), %ymm13
	vfmadd231pd		%ymm1, %ymm13, %ymm11
	vbroadcastsd	328(%r11), %ymm13
	vfmadd231pd		%ymm1, %ymm13, %ymm10
	vbroadcastsd	296(%r11), %ymm13
	vfmadd231pd		%ymm1, %ymm13, %ymm9
	vbroadcastsd	264(%r11), %ymm13
	vfmadd231pd		%ymm1, %ymm13, %ymm8
	vbroadcastsd	232(%r11), %ymm13
	vfmadd231pd		%ymm1, %ymm13, %ymm7
	vbroadcastsd	200(%r11), %ymm13
	vfmadd231pd		%ymm1, %ymm13, %ymm6
	vbroadcastsd	168(%r11), %ymm13
	vfmadd231pd		%ymm1, %ymm13, %ymm5
	vbroadcastsd	136(%r11), %ymm13
	vfmadd231pd		%ymm1, %ymm13, %ymm4
	vbroadcastsd	104(%r11), %ymm13
	vfmadd231pd		%ymm1, %ymm13, %ymm3
	vbroadcastsd	72(%r11), %ymm13
	vfmadd231pd		%ymm1, %ymm13, %ymm2
	vbroadcastsd	40(%r11), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm1
	//
	vbroadcastsd	352(%r11), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm11
	vbroadcastsd	320(%r11), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm10
	vbroadcastsd	288(%r11), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm9
	vbroadcastsd	256(%r11), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm8
	vbroadcastsd	224(%r11), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm7
	vbroadcastsd	192(%r11), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm6
	vbroadcastsd	160(%r11), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm5
	vbroadcastsd	128(%r11), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm4
	vbroadcastsd	96(%r11), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm3
	vbroadcastsd	64(%r11), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm2
	vbroadcastsd	32(%r11), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm1
	vbroadcastsd	0(%r11), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm0

	movq	ARG6, %r10 // K
	movq	ARG7, %r11 // km

	cmpl	$ 4, %r11d
	jge		0f

	vcvtsi2sd	%r11d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC02(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC02(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm15

	vxorpd		%ymm14, %ymm14, %ymm14
	vblendvpd	%ymm15, %ymm11, %ymm14, %ymm11
	vblendvpd	%ymm15, %ymm10, %ymm14, %ymm10
	vblendvpd	%ymm15, %ymm9, %ymm14, %ymm9
	vblendvpd	%ymm15, %ymm8, %ymm14, %ymm8
	vblendvpd	%ymm15, %ymm7, %ymm14, %ymm7
	vblendvpd	%ymm15, %ymm6, %ymm14, %ymm6
	vblendvpd	%ymm15, %ymm5, %ymm14, %ymm5
	vblendvpd	%ymm15, %ymm4, %ymm14, %ymm4
	vblendvpd	%ymm15, %ymm3, %ymm14, %ymm3
	vblendvpd	%ymm15, %ymm2, %ymm14, %ymm2
	vblendvpd	%ymm15, %ymm1, %ymm14, %ymm1
	vblendvpd	%ymm15, %ymm0, %ymm14, %ymm0

0:
	vmovapd			%ymm11, 352(%r10)
	vmovapd			%ymm10, 320(%r10)
	vmovapd			%ymm9, 288(%r10)
	vmovapd			%ymm8, 256(%r10)
	vmovapd			%ymm7, 224(%r10)
	vmovapd			%ymm6, 192(%r10)
	vmovapd			%ymm5, 160(%r10)
	vmovapd			%ymm4, 128(%r10)
	vmovapd			%ymm3, 96(%r10)
	vmovapd			%ymm2, 64(%r10)
	vmovapd			%ymm1, 32(%r10)
	vmovapd			%ymm0, 0(%r10)

	movq	ARG1, %r10 // n
	movq	ARG6, %r11 // K
	movq	ARG2, %r12 // V
	movq	ARG3, %r13 // sdd
	sall	$ 5, %r13d
	movq	ARG5, %r14 // D

	// load block from C
	vmovapd	0(%r14), %ymm0
	vmovapd	32(%r14), %ymm1
	vmovapd	64(%r14), %ymm2
	vmovapd	96(%r14), %ymm3
	vmovapd	128(%r14), %ymm4
	vmovapd	160(%r14), %ymm5
	vmovapd	192(%r14), %ymm6
	vmovapd	224(%r14), %ymm7
	vmovapd	256(%r14), %ymm8
	vmovapd	288(%r14), %ymm9
	vmovapd	320(%r14), %ymm10
	vmovapd	352(%r14), %ymm11

	// 0
	vmovapd			0(%r11), %ymm12
	vaddpd			%ymm12, %ymm0, %ymm0
	vbroadcastsd	32(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	64(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	96(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	128(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	160(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	192(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	224(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	256(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	288(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	320(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	352(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11
	// 1
	vmovapd			32(%r11), %ymm12
	vaddpd			%ymm12, %ymm1, %ymm1
	vbroadcastsd	72(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	104(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	136(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	168(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	200(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	232(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	264(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	296(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	328(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	360(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11
	// 2
	vmovapd			64(%r11), %ymm12
	vaddpd			%ymm12, %ymm2, %ymm2
	vbroadcastsd	112(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	144(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	176(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	208(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	240(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	272(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	304(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	336(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	368(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11
	// 3
	vmovapd			96(%r11), %ymm12
	vaddpd			%ymm12, %ymm3, %ymm3
	vbroadcastsd	152(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	184(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	216(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	248(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	280(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	312(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	344(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	376(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11
	// 4
	vmovapd			128(%r11), %ymm12
	vaddpd			%ymm12, %ymm4, %ymm4
	vbroadcastsd	160(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	192(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	224(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	256(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	288(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	320(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	352(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11
	// 5
	vmovapd			160(%r11), %ymm12
	vaddpd			%ymm12, %ymm5, %ymm5
	vbroadcastsd	200(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	232(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	264(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	296(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	328(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	360(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11
	// 6
	vmovapd			192(%r11), %ymm12
	vaddpd			%ymm12, %ymm6, %ymm6
	vbroadcastsd	240(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	272(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	304(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	336(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	368(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11
	// 7
	vmovapd			224(%r11), %ymm12
	vaddpd			%ymm12, %ymm7, %ymm7
	vbroadcastsd	280(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	312(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	344(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	376(%r12, %r13), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11
	// 8
	vmovapd			256(%r11), %ymm12
	vaddpd			%ymm12, %ymm8, %ymm8
	vbroadcastsd	288(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	320(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	352(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11
	// 9
	vmovapd			288(%r11), %ymm12
	vaddpd			%ymm12, %ymm9, %ymm9
	vbroadcastsd	328(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	360(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11
	// 10
	vmovapd			320(%r11), %ymm12
	vaddpd			%ymm12, %ymm10, %ymm10
	vbroadcastsd	368(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11
	// 11
	vmovapd			352(%r11), %ymm12
	vaddpd			%ymm12, %ymm11, %ymm11

	// store block to C
	vmovapd	%ymm0, 0(%r14)
	vmovapd	%ymm1, 32(%r14)
	vmovapd	%ymm2, 64(%r14)
	vmovapd	%ymm3, 96(%r14)
	vmovapd	%ymm4, 128(%r14)
	vmovapd	%ymm5, 160(%r14)
	vmovapd	%ymm6, 192(%r14)
	vmovapd	%ymm7, 224(%r14)
	vmovapd	%ymm8, 256(%r14)
	vmovapd	%ymm9, 288(%r14)
	vmovapd	%ymm10, 320(%r14)
	vmovapd	%ymm11, 352(%r14)

	subl	$ 12, %r10d
	addq	$ 384, %r12
	addq	$ 384, %r14

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEBP_ADD_NN_4X12_LIB4
#else
	CALL(inner_kernel_dgebp_add_nn_4x12_lib4)
#endif

100:
	EPILOGUE

	ret

	FUN_END(kernel_dlarfb12_rn_4_lib4)





	//                             1         2           3           4           5
// void kernel_dlarfb4_rn_12_lib4(int kmax, double *pV, double *pT, double *pD, int sdd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dlarfb4_rn_12_lib4)
	
	PROLOGUE

	// zero accumulation registers

//	ZERO_ACC
	
	movq	ARG1, %r10 // k
	movq	ARG4, %r11 // D
	movq	ARG5, %r12 // sdd
	sall	$ 5, %r12d
	movq	ARG2, %r13 // V

	//
	vmovapd			0(%r11), %ymm0
	vmovapd			0(%r11, %r12), %ymm4
	vmovapd			0(%r11, %r12, 2), %ymm8
	//
	vmovapd			32(%r11), %ymm1
	vmovapd			32(%r11, %r12), %ymm5
	vmovapd			32(%r11, %r12, 2), %ymm9
	vbroadcastsd	32(%r13), %ymm13
	vfmadd231pd		%ymm13, %ymm1, %ymm0
	vfmadd231pd		%ymm13, %ymm5, %ymm4
	vfmadd231pd		%ymm13, %ymm9, %ymm8
	//
	vmovapd			64(%r11), %ymm2
	vmovapd			64(%r11, %r12), %ymm6
	vmovapd			64(%r11, %r12, 2), %ymm10
	vbroadcastsd	64(%r13), %ymm13
	vfmadd231pd		%ymm13, %ymm2, %ymm0
	vfmadd231pd		%ymm13, %ymm6, %ymm4
	vfmadd231pd		%ymm13, %ymm10, %ymm8
	vbroadcastsd	72(%r13), %ymm13
	vfmadd231pd		%ymm13, %ymm2, %ymm1
	vfmadd231pd		%ymm13, %ymm6, %ymm5
	vfmadd231pd		%ymm13, %ymm10, %ymm9
	//
	vmovapd			96(%r11), %ymm3
	vmovapd			96(%r11, %r12), %ymm7
	vmovapd			96(%r11, %r12, 2), %ymm11
	vbroadcastsd	96(%r13), %ymm13
	vfmadd231pd		%ymm13, %ymm3, %ymm0
	vfmadd231pd		%ymm13, %ymm7, %ymm4
	vfmadd231pd		%ymm13, %ymm11, %ymm8
	vbroadcastsd	104(%r13), %ymm13
	vfmadd231pd		%ymm13, %ymm3, %ymm1
	vfmadd231pd		%ymm13, %ymm7, %ymm5
	vfmadd231pd		%ymm13, %ymm11, %ymm9
	vbroadcastsd	112(%r13), %ymm13
	vfmadd231pd		%ymm13, %ymm3, %ymm2
	vfmadd231pd		%ymm13, %ymm7, %ymm6
	vfmadd231pd		%ymm13, %ymm11, %ymm10

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r13

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif

	movq	ARG3, %r10 // T

	//
	vbroadcastsd	120(%r10), %ymm12
	vmulpd			%ymm3, %ymm12, %ymm3
	vmulpd			%ymm7, %ymm12, %ymm7
	vmulpd			%ymm11, %ymm12, %ymm11
	//
	vbroadcastsd	112(%r10), %ymm12
	vfmadd231pd		%ymm2, %ymm12, %ymm3
	vfmadd231pd		%ymm6, %ymm12, %ymm7
	vfmadd231pd		%ymm10, %ymm12, %ymm11
	vbroadcastsd	80(%r10), %ymm12
	vmulpd			%ymm2, %ymm12, %ymm2
	vmulpd			%ymm6, %ymm12, %ymm6
	vmulpd			%ymm10, %ymm12, %ymm10
	//
	vbroadcastsd	104(%r10), %ymm12
	vfmadd231pd		%ymm1, %ymm12, %ymm3
	vfmadd231pd		%ymm5, %ymm12, %ymm7
	vfmadd231pd		%ymm9, %ymm12, %ymm11
	vbroadcastsd	72(%r10), %ymm12
	vfmadd231pd		%ymm1, %ymm12, %ymm2
	vfmadd231pd		%ymm5, %ymm12, %ymm6
	vfmadd231pd		%ymm9, %ymm12, %ymm10
	vbroadcastsd	40(%r10), %ymm12
	vmulpd			%ymm1, %ymm12, %ymm1
	vmulpd			%ymm5, %ymm12, %ymm5
	vmulpd			%ymm9, %ymm12, %ymm9
	//
	vbroadcastsd	96(%r10), %ymm12
	vfmadd231pd		%ymm0, %ymm12, %ymm3
	vfmadd231pd		%ymm4, %ymm12, %ymm7
	vfmadd231pd		%ymm8, %ymm12, %ymm11
	vbroadcastsd	64(%r10), %ymm12
	vfmadd231pd		%ymm0, %ymm12, %ymm2
	vfmadd231pd		%ymm4, %ymm12, %ymm6
	vfmadd231pd		%ymm8, %ymm12, %ymm10
	vbroadcastsd	32(%r10), %ymm12
	vfmadd231pd		%ymm0, %ymm12, %ymm1
	vfmadd231pd		%ymm4, %ymm12, %ymm5
	vfmadd231pd		%ymm8, %ymm12, %ymm9
	vbroadcastsd	0(%r10), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm0
	vmulpd			%ymm4, %ymm12, %ymm4
	vmulpd			%ymm8, %ymm12, %ymm8

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // V
	movq	ARG4, %r12 // D
	movq	ARG5, %r13 // sdd
	sall	$ 5, %r13d

	//
	vmovapd			0(%r12), %ymm12
	vmovapd			0(%r12, %r13), %ymm14
	vmovapd			0(%r12, %r13, 2), %ymm15
	vaddpd			%ymm12, %ymm0, %ymm12
	vaddpd			%ymm14, %ymm4, %ymm14
	vaddpd			%ymm15, %ymm8, %ymm15
	vmovapd			%ymm12, 0(%r12)
	vmovapd			%ymm14, 0(%r12, %r13)
	vmovapd			%ymm15, 0(%r12, %r13, 2)
	//
	vmovapd			32(%r12), %ymm12
	vmovapd			32(%r12, %r13), %ymm14
	vmovapd			32(%r12, %r13, 2), %ymm15
	vbroadcastsd	32(%r11), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm12
	vfmadd231pd		%ymm4, %ymm13, %ymm14
	vfmadd231pd		%ymm8, %ymm13, %ymm15
	vaddpd			%ymm12, %ymm1, %ymm12
	vaddpd			%ymm14, %ymm5, %ymm14
	vaddpd			%ymm15, %ymm9, %ymm15
	vmovapd			%ymm12, 32(%r12)
	vmovapd			%ymm14, 32(%r12, %r13)
	vmovapd			%ymm15, 32(%r12, %r13, 2)
	//
	vmovapd			64(%r12), %ymm12
	vmovapd			64(%r12, %r13), %ymm14
	vmovapd			64(%r12, %r13, 2), %ymm15
	vbroadcastsd	64(%r11), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm12
	vfmadd231pd		%ymm4, %ymm13, %ymm14
	vfmadd231pd		%ymm8, %ymm13, %ymm15
	vbroadcastsd	72(%r11), %ymm13
	vfmadd231pd		%ymm1, %ymm13, %ymm12
	vfmadd231pd		%ymm5, %ymm13, %ymm14
	vfmadd231pd		%ymm9, %ymm13, %ymm15
	vaddpd			%ymm12, %ymm2, %ymm12
	vaddpd			%ymm14, %ymm6, %ymm14
	vaddpd			%ymm15, %ymm10, %ymm15
	vmovapd			%ymm12, 64(%r12)
	vmovapd			%ymm14, 64(%r12, %r13)
	vmovapd			%ymm15, 64(%r12, %r13, 2)
	//
	vmovapd			96(%r12), %ymm12
	vmovapd			96(%r12, %r13), %ymm14
	vmovapd			96(%r12, %r13, 2), %ymm15
	vbroadcastsd	96(%r11), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm12
	vfmadd231pd		%ymm4, %ymm13, %ymm14
	vfmadd231pd		%ymm8, %ymm13, %ymm15
	vbroadcastsd	104(%r11), %ymm13
	vfmadd231pd		%ymm1, %ymm13, %ymm12
	vfmadd231pd		%ymm5, %ymm13, %ymm14
	vfmadd231pd		%ymm9, %ymm13, %ymm15
	vbroadcastsd	112(%r11), %ymm13
	vfmadd231pd		%ymm2, %ymm13, %ymm12
	vfmadd231pd		%ymm6, %ymm13, %ymm14
	vfmadd231pd		%ymm10, %ymm13, %ymm15
	vaddpd			%ymm12, %ymm3, %ymm12
	vaddpd			%ymm14, %ymm7, %ymm14
	vaddpd			%ymm15, %ymm11, %ymm15
	vmovapd			%ymm12, 96(%r12)
	vmovapd			%ymm14, 96(%r12, %r13)
	vmovapd			%ymm15, 96(%r12, %r13, 2)

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r12

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEBP_ADD_NN_12X4_LIB4
#else
	CALL(inner_kernel_dgebp_add_nn_12x4_lib4)
#endif

	EPILOGUE

	ret

	FUN_END(kernel_dlarfb4_rn_12_lib4)





//                                  1         2           3           4           5        6           7
// void kernel_dlarfb4_rn_12_la_lib4(int kmax, double *pV, double *pT, double *pD, int sdd, double *pA, int sda);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dlarfb4_rn_12_la_lib4)
	
	PROLOGUE

	// zero accumulation registers

//	ZERO_ACC
	
	// D

	movq	ARG4, %r10 // D
	movq	ARG5, %r11 // sdd
	sall	$ 5, %r11d

	//
	vmovapd			0(%r10), %ymm0
	vmovapd			0(%r10, %r11), %ymm4
	vmovapd			0(%r10, %r11, 2), %ymm8
	//
	vmovapd			32(%r10), %ymm1
	vmovapd			32(%r10, %r11), %ymm5
	vmovapd			32(%r10, %r11, 2), %ymm9
	//
	vmovapd			64(%r10), %ymm2
	vmovapd			64(%r10, %r11), %ymm6
	vmovapd			64(%r10, %r11, 2), %ymm10
	//
	vmovapd			96(%r10), %ymm3
	vmovapd			96(%r10, %r11), %ymm7
	vmovapd			96(%r10, %r11, 2), %ymm11

	// A

	movq	ARG1, %r10 // k
	movq	ARG6, %r11 // A
	movq	ARG7, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG2, %r13 // VA

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif

	// T

	movq	ARG3, %r10 // T

	//
	vbroadcastsd	120(%r10), %ymm12
	vmulpd			%ymm3, %ymm12, %ymm3
	vmulpd			%ymm7, %ymm12, %ymm7
	vmulpd			%ymm11, %ymm12, %ymm11
	//
	vbroadcastsd	112(%r10), %ymm12
	vfmadd231pd		%ymm2, %ymm12, %ymm3
	vfmadd231pd		%ymm6, %ymm12, %ymm7
	vfmadd231pd		%ymm10, %ymm12, %ymm11
	vbroadcastsd	80(%r10), %ymm12
	vmulpd			%ymm2, %ymm12, %ymm2
	vmulpd			%ymm6, %ymm12, %ymm6
	vmulpd			%ymm10, %ymm12, %ymm10
	//
	vbroadcastsd	104(%r10), %ymm12
	vfmadd231pd		%ymm1, %ymm12, %ymm3
	vfmadd231pd		%ymm5, %ymm12, %ymm7
	vfmadd231pd		%ymm9, %ymm12, %ymm11
	vbroadcastsd	72(%r10), %ymm12
	vfmadd231pd		%ymm1, %ymm12, %ymm2
	vfmadd231pd		%ymm5, %ymm12, %ymm6
	vfmadd231pd		%ymm9, %ymm12, %ymm10
	vbroadcastsd	40(%r10), %ymm12
	vmulpd			%ymm1, %ymm12, %ymm1
	vmulpd			%ymm5, %ymm12, %ymm5
	vmulpd			%ymm9, %ymm12, %ymm9
	//
	vbroadcastsd	96(%r10), %ymm12
	vfmadd231pd		%ymm0, %ymm12, %ymm3
	vfmadd231pd		%ymm4, %ymm12, %ymm7
	vfmadd231pd		%ymm8, %ymm12, %ymm11
	vbroadcastsd	64(%r10), %ymm12
	vfmadd231pd		%ymm0, %ymm12, %ymm2
	vfmadd231pd		%ymm4, %ymm12, %ymm6
	vfmadd231pd		%ymm8, %ymm12, %ymm10
	vbroadcastsd	32(%r10), %ymm12
	vfmadd231pd		%ymm0, %ymm12, %ymm1
	vfmadd231pd		%ymm4, %ymm12, %ymm5
	vfmadd231pd		%ymm8, %ymm12, %ymm9
	vbroadcastsd	0(%r10), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm0
	vmulpd			%ymm4, %ymm12, %ymm4
	vmulpd			%ymm8, %ymm12, %ymm8

	// D

	movq	ARG4, %r10 // D
	movq	ARG5, %r11 // sdd
	sall	$ 5, %r11d

	//
	vmovapd			0(%r10), %ymm12
	vmovapd			0(%r10, %r11), %ymm14
	vmovapd			0(%r10, %r11, 2), %ymm15
	vaddpd			%ymm12, %ymm0, %ymm12
	vaddpd			%ymm14, %ymm4, %ymm14
	vaddpd			%ymm15, %ymm8, %ymm15
	vmovapd			%ymm12, 0(%r10)
	vmovapd			%ymm14, 0(%r10, %r11)
	vmovapd			%ymm15, 0(%r10, %r11, 2)
	//
	vmovapd			32(%r10), %ymm12
	vmovapd			32(%r10, %r11), %ymm14
	vmovapd			32(%r10, %r11, 2), %ymm15
	vaddpd			%ymm12, %ymm1, %ymm12
	vaddpd			%ymm14, %ymm5, %ymm14
	vaddpd			%ymm15, %ymm9, %ymm15
	vmovapd			%ymm12, 32(%r10)
	vmovapd			%ymm14, 32(%r10, %r11)
	vmovapd			%ymm15, 32(%r10, %r11, 2)
	//
	vmovapd			64(%r10), %ymm12
	vmovapd			64(%r10, %r11), %ymm14
	vmovapd			64(%r10, %r11, 2), %ymm15
	vaddpd			%ymm12, %ymm2, %ymm12
	vaddpd			%ymm14, %ymm6, %ymm14
	vaddpd			%ymm15, %ymm10, %ymm15
	vmovapd			%ymm12, 64(%r10)
	vmovapd			%ymm14, 64(%r10, %r11)
	vmovapd			%ymm15, 64(%r10, %r11, 2)
	//
	vmovapd			96(%r10), %ymm12
	vmovapd			96(%r10, %r11), %ymm14
	vmovapd			96(%r10, %r11, 2), %ymm15
	vaddpd			%ymm12, %ymm3, %ymm12
	vaddpd			%ymm14, %ymm7, %ymm14
	vaddpd			%ymm15, %ymm11, %ymm15
	vmovapd			%ymm12, 96(%r10)
	vmovapd			%ymm14, 96(%r10, %r11)
	vmovapd			%ymm15, 96(%r10, %r11, 2)

	// A

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // VA
	movq	ARG6, %r12 // A
	movq	ARG7, %r13 // sda
	sall	$ 5, %r13d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEBP_ADD_NN_12X4_LIB4
#else
	CALL(inner_kernel_dgebp_add_nn_12x4_lib4)
#endif

	EPILOGUE

	ret

	FUN_END(kernel_dlarfb4_rn_12_la_lib4)





//                                   1       2       3            4            5           6           7        8           9        10          11
// void kernel_dlarfb4_rn_12_lla_lib4(int n0, int n1, double *pVL, double *pVA, double *pT, double *pD, int sdd, double *pL, int sdl, double *pA, int sda);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dlarfb4_rn_12_lla_lib4)

	PROLOGUE

	// zero accumulation registers

//	ZERO_ACC
	
	// D

	movq	ARG6, %r10 // D
	movq	ARG7, %r11 // sdd
	sall	$ 5, %r11d

	//
	vmovapd			0(%r10), %ymm0
	vmovapd			0(%r10, %r11), %ymm4
	vmovapd			0(%r10, %r11, 2), %ymm8
	//
	vmovapd			32(%r10), %ymm1
	vmovapd			32(%r10, %r11), %ymm5
	vmovapd			32(%r10, %r11, 2), %ymm9
	//
	vmovapd			64(%r10), %ymm2
	vmovapd			64(%r10, %r11), %ymm6
	vmovapd			64(%r10, %r11, 2), %ymm10
	//
	vmovapd			96(%r10), %ymm3
	vmovapd			96(%r10, %r11), %ymm7
	vmovapd			96(%r10, %r11, 2), %ymm11

	// L

	movq	ARG1, %r10 // n0
	movq	ARG8, %r11 // L
	movq	ARG9, %r12 // sdl
	sall	$ 5, %r12d
	movq	ARG3, %r13 // VL

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
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
	vmovapd			0(%r11, %r12), %ymm13
	vmovapd			0(%r11, %r12, 2), %ymm14
	vbroadcastsd	0(%r13), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm0
	vfmadd231pd		%ymm13, %ymm15, %ymm4
	vfmadd231pd		%ymm14, %ymm15, %ymm8
	vbroadcastsd	8(%r13), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm1
	vfmadd231pd		%ymm13, %ymm15, %ymm5
	vfmadd231pd		%ymm14, %ymm15, %ymm9
	vbroadcastsd	16(%r13), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm2
	vfmadd231pd		%ymm13, %ymm15, %ymm6
	vfmadd231pd		%ymm14, %ymm15, %ymm10
	vbroadcastsd	24(%r13), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm3
	vfmadd231pd		%ymm13, %ymm15, %ymm7
	vfmadd231pd		%ymm14, %ymm15, %ymm11
	// 3
	vmovapd			32(%r11), %ymm12
	vmovapd			32(%r11, %r12), %ymm13
	vmovapd			32(%r11, %r12, 2), %ymm14
	vbroadcastsd	40(%r13), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm1
	vfmadd231pd		%ymm13, %ymm15, %ymm5
	vfmadd231pd		%ymm14, %ymm15, %ymm9
	vbroadcastsd	48(%r13), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm2
	vfmadd231pd		%ymm13, %ymm15, %ymm6
	vfmadd231pd		%ymm14, %ymm15, %ymm10
	vbroadcastsd	56(%r13), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm3
	vfmadd231pd		%ymm13, %ymm15, %ymm7
	vfmadd231pd		%ymm14, %ymm15, %ymm11
	// 2
	vmovapd			64(%r11), %ymm12
	vmovapd			64(%r11, %r12), %ymm13
	vmovapd			64(%r11, %r12, 2), %ymm14
	vbroadcastsd	80(%r13), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm2
	vfmadd231pd		%ymm13, %ymm15, %ymm6
	vfmadd231pd		%ymm14, %ymm15, %ymm10
	vbroadcastsd	88(%r13), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm3
	vfmadd231pd		%ymm13, %ymm15, %ymm7
	vfmadd231pd		%ymm14, %ymm15, %ymm11
	// 1
	vmovapd			96(%r11), %ymm12
	vmovapd			96(%r11, %r12), %ymm13
	vmovapd			96(%r11, %r12, 2), %ymm14
	vbroadcastsd	120(%r13), %ymm15
	vfmadd231pd		%ymm12, %ymm15, %ymm3
	vfmadd231pd		%ymm13, %ymm15, %ymm7
	vfmadd231pd		%ymm14, %ymm15, %ymm11

	// A

	movq	ARG2, %r10 // n1
	movq	ARG10, %r11 // A
	movq	ARG11, %r12 // sda
	sall	$ 5, %r12d
	movq	ARG4, %r13 // VA

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_12X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_12x4_lib4)
#endif

	// T

	movq	ARG5, %r10 // T

	vbroadcastsd	120(%r10), %ymm12
	vmulpd			%ymm3, %ymm12, %ymm3
	vmulpd			%ymm7, %ymm12, %ymm7
	vmulpd			%ymm11, %ymm12, %ymm11
	//
	vbroadcastsd	112(%r10), %ymm12
	vfmadd231pd		%ymm2, %ymm12, %ymm3
	vfmadd231pd		%ymm6, %ymm12, %ymm7
	vfmadd231pd		%ymm10, %ymm12, %ymm11
	vbroadcastsd	80(%r10), %ymm12
	vmulpd			%ymm2, %ymm12, %ymm2
	vmulpd			%ymm6, %ymm12, %ymm6
	vmulpd			%ymm10, %ymm12, %ymm10
	//
	vbroadcastsd	104(%r10), %ymm12
	vfmadd231pd		%ymm1, %ymm12, %ymm3
	vfmadd231pd		%ymm5, %ymm12, %ymm7
	vfmadd231pd		%ymm9, %ymm12, %ymm11
	vbroadcastsd	72(%r10), %ymm12
	vfmadd231pd		%ymm1, %ymm12, %ymm2
	vfmadd231pd		%ymm5, %ymm12, %ymm6
	vfmadd231pd		%ymm9, %ymm12, %ymm10
	vbroadcastsd	40(%r10), %ymm12
	vmulpd			%ymm1, %ymm12, %ymm1
	vmulpd			%ymm5, %ymm12, %ymm5
	vmulpd			%ymm9, %ymm12, %ymm9
	//
	vbroadcastsd	96(%r10), %ymm12
	vfmadd231pd		%ymm0, %ymm12, %ymm3
	vfmadd231pd		%ymm4, %ymm12, %ymm7
	vfmadd231pd		%ymm8, %ymm12, %ymm11
	vbroadcastsd	64(%r10), %ymm12
	vfmadd231pd		%ymm0, %ymm12, %ymm2
	vfmadd231pd		%ymm4, %ymm12, %ymm6
	vfmadd231pd		%ymm8, %ymm12, %ymm10
	vbroadcastsd	32(%r10), %ymm12
	vfmadd231pd		%ymm0, %ymm12, %ymm1
	vfmadd231pd		%ymm4, %ymm12, %ymm5
	vfmadd231pd		%ymm8, %ymm12, %ymm9
	vbroadcastsd	0(%r10), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm0
	vmulpd			%ymm4, %ymm12, %ymm4
	vmulpd			%ymm8, %ymm12, %ymm8

	// D

	movq	ARG6, %r10 // D
	movq	ARG7, %r11 // sdd
	sall	$ 5, %r11d

	//
	vmovapd			0(%r10), %ymm12
	vmovapd			0(%r10, %r11), %ymm14
	vmovapd			0(%r10, %r11, 2), %ymm15
	vaddpd			%ymm12, %ymm0, %ymm12
	vaddpd			%ymm14, %ymm4, %ymm14
	vaddpd			%ymm15, %ymm8, %ymm15
	vmovapd			%ymm12, 0(%r10)
	vmovapd			%ymm14, 0(%r10, %r11)
	vmovapd			%ymm15, 0(%r10, %r11, 2)
	//
	vmovapd			32(%r10), %ymm12
	vmovapd			32(%r10, %r11), %ymm14
	vmovapd			32(%r10, %r11, 2), %ymm15
	vaddpd			%ymm12, %ymm1, %ymm12
	vaddpd			%ymm14, %ymm5, %ymm14
	vaddpd			%ymm15, %ymm9, %ymm15
	vmovapd			%ymm12, 32(%r10)
	vmovapd			%ymm14, 32(%r10, %r11)
	vmovapd			%ymm15, 32(%r10, %r11, 2)
	//
	vmovapd			64(%r10), %ymm12
	vmovapd			64(%r10, %r11), %ymm14
	vmovapd			64(%r10, %r11, 2), %ymm15
	vaddpd			%ymm12, %ymm2, %ymm12
	vaddpd			%ymm14, %ymm6, %ymm14
	vaddpd			%ymm15, %ymm10, %ymm15
	vmovapd			%ymm12, 64(%r10)
	vmovapd			%ymm14, 64(%r10, %r11)
	vmovapd			%ymm15, 64(%r10, %r11, 2)
	//
	vmovapd			96(%r10), %ymm12
	vmovapd			96(%r10, %r11), %ymm14
	vmovapd			96(%r10, %r11, 2), %ymm15
	vaddpd			%ymm12, %ymm3, %ymm12
	vaddpd			%ymm14, %ymm7, %ymm14
	vaddpd			%ymm15, %ymm11, %ymm15
	vmovapd			%ymm12, 96(%r10)
	vmovapd			%ymm14, 96(%r10, %r11)
	vmovapd			%ymm15, 96(%r10, %r11, 2)

	// L

	movq	ARG1, %r10 // n0
	movq	ARG3, %r11 // VL
	movq	ARG8, %r12 // L
	movq	ARG9, %r13 // sdl
	sall	$ 5, %r13d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEBP_ADD_NN_12X4_LIB4
#else
	CALL(inner_kernel_dgebp_add_nn_12x4_lib4)
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
	vmovapd			0(%r12, %r13), %ymm13
	vmovapd			0(%r12, %r13, 2), %ymm14
	vbroadcastsd	0(%r11), %ymm15
	vfmadd231pd		%ymm0, %ymm15, %ymm12
	vfmadd231pd		%ymm4, %ymm15, %ymm13
	vfmadd231pd		%ymm8, %ymm15, %ymm14
	vbroadcastsd	8(%r11), %ymm15
	vfmadd231pd		%ymm1, %ymm15, %ymm12
	vfmadd231pd		%ymm5, %ymm15, %ymm13
	vfmadd231pd		%ymm9, %ymm15, %ymm14
	vbroadcastsd	16(%r11), %ymm15
	vfmadd231pd		%ymm2, %ymm15, %ymm12
	vfmadd231pd		%ymm6, %ymm15, %ymm13
	vfmadd231pd		%ymm10, %ymm15, %ymm14
	vbroadcastsd	24(%r11), %ymm15
	vfmadd231pd		%ymm3, %ymm15, %ymm12
	vfmadd231pd		%ymm7, %ymm15, %ymm13
	vfmadd231pd		%ymm11, %ymm15, %ymm14
	vmovapd			%ymm12, 0(%r12)
	vmovapd			%ymm13, 0(%r12, %r13)
	vmovapd			%ymm14, 0(%r12, %r13, 2)
	// 3
	vmovapd			32(%r12), %ymm12
	vmovapd			32(%r12, %r13), %ymm13
	vmovapd			32(%r12, %r13, 2), %ymm14
	vbroadcastsd	40(%r11), %ymm15
	vfmadd231pd		%ymm1, %ymm15, %ymm12
	vfmadd231pd		%ymm5, %ymm15, %ymm13
	vfmadd231pd		%ymm9, %ymm15, %ymm14
	vbroadcastsd	48(%r11), %ymm15
	vfmadd231pd		%ymm2, %ymm15, %ymm12
	vfmadd231pd		%ymm6, %ymm15, %ymm13
	vfmadd231pd		%ymm10, %ymm15, %ymm14
	vbroadcastsd	56(%r11), %ymm15
	vfmadd231pd		%ymm3, %ymm15, %ymm12
	vfmadd231pd		%ymm7, %ymm15, %ymm13
	vfmadd231pd		%ymm11, %ymm15, %ymm14
	vmovapd			%ymm12, 32(%r12)
	vmovapd			%ymm13, 32(%r12, %r13)
	vmovapd			%ymm14, 32(%r12, %r13, 2)
	// 2
	vmovapd			64(%r12), %ymm12
	vmovapd			64(%r12, %r13), %ymm13
	vmovapd			64(%r12, %r13, 2), %ymm14
	vbroadcastsd	80(%r11), %ymm15
	vfmadd231pd		%ymm2, %ymm15, %ymm12
	vfmadd231pd		%ymm6, %ymm15, %ymm13
	vfmadd231pd		%ymm10, %ymm15, %ymm14
	vbroadcastsd	88(%r11), %ymm15
	vfmadd231pd		%ymm3, %ymm15, %ymm12
	vfmadd231pd		%ymm7, %ymm15, %ymm13
	vfmadd231pd		%ymm11, %ymm15, %ymm14
	vmovapd			%ymm12, 64(%r12)
	vmovapd			%ymm13, 64(%r12, %r13)
	vmovapd			%ymm14, 64(%r12, %r13, 2)
	// 3
	vmovapd			96(%r12), %ymm12
	vmovapd			96(%r12, %r13), %ymm13
	vmovapd			96(%r12, %r13, 2), %ymm14
	vbroadcastsd	120(%r11), %ymm15
	vfmadd231pd		%ymm3, %ymm15, %ymm12
	vfmadd231pd		%ymm7, %ymm15, %ymm13
	vfmadd231pd		%ymm11, %ymm15, %ymm14
	vmovapd			%ymm12, 96(%r12)
	vmovapd			%ymm13, 96(%r12, %r13)
	vmovapd			%ymm14, 96(%r12, %r13, 2)

	// A

	movq	ARG2, %r10 // n1
	movq	ARG4, %r11 // VA
	movq	ARG10, %r12 // A
	movq	ARG11, %r13 // sda
	sall	$ 5, %r13d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEBP_ADD_NN_12X4_LIB4
#else
	CALL(inner_kernel_dgebp_add_nn_12x4_lib4)
#endif

	EPILOGUE

	ret

	FUN_END(kernel_dlarfb4_rn_12_lla_lib4)





//#if defined(BLAS_API)
#if ( defined(BLAS_API) | ( defined(LA_HIGH_PERFORMANCE) & defined(MF_COLMAJ) ) )

#include "kernel_dgemm_12x4_lib.S"
//#include "archive/kernel_dgemm_12x4_lib.S"

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
.LC04: // { 11.5 10.5 9.5 8.5 }
#elif defined(OS_MAC)
LC04: // { 11.5 10.5 9.5 8.5 }
	.align 5
#endif
	.double 8.5
	.double 9.5
	.double 10.5
	.double 11.5

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC05: // { 1.0 1.0 1.0 1.0 }
#elif defined(OS_MAC)
LC05: // { 1.0 1.0 1.0 1.0 }
	.align 5
#endif
	.double	1.0
	.double	1.0
	.double	1.0
	.double	1.0

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC06: // { 1.0 1.0 1.0 -1.0 }
#elif defined(OS_MAC)
	.align 5
LC06: // { 1.0 1.0 1.0 -1.0 }
#endif
	.double	-1.0
	.double	1.0
	.double	1.0
	.double	1.0

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC07: // { 1.0 1.0 -1.0 -1.0 }
#elif defined(OS_MAC)
	.align 5
LC07: // { 1.0 1.0 -1.0 -1.0 }
#endif
	.double	-1.0
	.double	-1.0
	.double	1.0
	.double	1.0

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC08: // { 1.0 -1.0 -1.0 -1.0 }
#elif defined(OS_MAC)
	.align 5
LC08: // { 1.0 -1.0 -1.0 -1.0 }
#endif
	.double	-1.0
	.double	-1.0
	.double	-1.0
	.double	1.0

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC09: // { -1.0 -1.0 -1.0 -1.0 }
#elif defined(OS_MAC)
	.align 5
LC09: // { -1.0 -1.0 -1.0 -1.0 }
#endif
	.double	-1.0
	.double	-1.0
	.double	-1.0
	.double	-1.0

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC10: // { -1.0 -1.0 -1.0 1.0 }
#elif defined(OS_MAC)
	.align 5
LC10: // { -1.0 -1.0 -1.0 1.0 }
#endif
	.double	1.0
	.double	-1.0
	.double	-1.0
	.double	-1.0

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC11: // { -1.0 -1.0 1.0 1.0 }
#elif defined(OS_MAC)
	.align 5
LC11: // { -1.0 -1.0 1.0 1.0 }
#endif
	.double	1.0
	.double	1.0
	.double	-1.0
	.double	-1.0

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC12: // { -1.0 1.0 1.0 1.0 }
#elif defined(OS_MAC)
	.align 5
LC12: // { -1.0 1.0 1.0 1.0 }
#endif
	.double	1.0
	.double	1.0
	.double	1.0
	.double	-1.0

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC13:
#elif defined(OS_MAC)
	.align 5
LC13:
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

