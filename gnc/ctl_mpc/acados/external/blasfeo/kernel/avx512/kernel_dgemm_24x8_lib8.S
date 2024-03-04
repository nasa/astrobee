/**************************************************************************************************
*                                                                                                 *
* This file is part of BLASFEO.                                                                   *
*                                                                                                 *
* BLASFEO -- BLAS For Embedded Optimization.                                                      *
* Copyright (C) 2021 by Gianluca Frison.                                                          *
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
	vpxord	%zmm0, %zmm0, %zmm0; \
	vmovapd	%zmm0, %zmm1; \
	vmovapd	%zmm0, %zmm2; \
	vmovapd	%zmm0, %zmm3; \
	vmovapd	%zmm0, %zmm4; \
	vmovapd	%zmm0, %zmm5; \
	vmovapd	%zmm0, %zmm6; \
	vmovapd	%zmm0, %zmm7; \
	vmovapd	%zmm0, %zmm8; \
	vmovapd	%zmm0, %zmm9; \
	vmovapd	%zmm0, %zmm10; \
	vmovapd	%zmm0, %zmm11; \
	vmovapd	%zmm0, %zmm12; \
	vmovapd	%zmm0, %zmm13; \
	vmovapd	%zmm0, %zmm14; \
	vmovapd	%zmm0, %zmm15; \
	vmovapd	%zmm0, %zmm16; \
	vmovapd	%zmm0, %zmm17; \
	vmovapd	%zmm0, %zmm18; \
	vmovapd	%zmm0, %zmm19; \
	vmovapd	%zmm0, %zmm20; \
	vmovapd	%zmm0, %zmm21; \
	vmovapd	%zmm0, %zmm22; \
	vmovapd	%zmm0, %zmm23;
#define NEG_ACC \
	vmovapd		.LC03(%rip), %zmm24; \
	vxorpd		%zmm24, %zmm0, %zmm0; \
	vxorpd		%zmm24, %zmm1, %zmm1; \
	vxorpd		%zmm24, %zmm2, %zmm2; \
	vxorpd		%zmm24, %zmm3, %zmm3; \
	vxorpd		%zmm24, %zmm4, %zmm4; \
	vxorpd		%zmm24, %zmm5, %zmm5; \
	vxorpd		%zmm24, %zmm6, %zmm6; \
	vxorpd		%zmm24, %zmm7, %zmm7; \
	vxorpd		%zmm24, %zmm8, %zmm8; \
	vxorpd		%zmm24, %zmm9, %zmm9; \
	vxorpd		%zmm24, %zmm10, %zmm10; \
	vxorpd		%zmm24, %zmm11, %zmm11; \
	vxorpd		%zmm24, %zmm12, %zmm12; \
	vxorpd		%zmm24, %zmm13, %zmm13; \
	vxorpd		%zmm24, %zmm14, %zmm14; \
	vxorpd		%zmm24, %zmm15, %zmm15; \
	vxorpd		%zmm24, %zmm16, %zmm16; \
	vxorpd		%zmm24, %zmm17, %zmm17; \
	vxorpd		%zmm24, %zmm18, %zmm18; \
	vxorpd		%zmm24, %zmm19, %zmm19; \
	vxorpd		%zmm24, %zmm20, %zmm20; \
	vxorpd		%zmm24, %zmm21, %zmm21; \
	vxorpd		%zmm24, %zmm22, %zmm22; \
	vxorpd		%zmm24, %zmm23, %zmm23;

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
	vpxord	%zmm0, %zmm0, %zmm0; \
	vmovapd	%zmm0, %zmm1; \
	vmovapd	%zmm0, %zmm2; \
	vmovapd	%zmm0, %zmm3; \
	vmovapd	%zmm0, %zmm4; \
	vmovapd	%zmm0, %zmm5; \
	vmovapd	%zmm0, %zmm6; \
	vmovapd	%zmm0, %zmm7; \
	vmovapd	%zmm0, %zmm8; \
	vmovapd	%zmm0, %zmm9; \
	vmovapd	%zmm0, %zmm10; \
	vmovapd	%zmm0, %zmm11; \
	vmovapd	%zmm0, %zmm12; \
	vmovapd	%zmm0, %zmm13; \
	vmovapd	%zmm0, %zmm14; \
	vmovapd	%zmm0, %zmm15; \
	vmovapd	%zmm0, %zmm16; \
	vmovapd	%zmm0, %zmm17; \
	vmovapd	%zmm0, %zmm18; \
	vmovapd	%zmm0, %zmm19; \
	vmovapd	%zmm0, %zmm20; \
	vmovapd	%zmm0, %zmm21; \
	vmovapd	%zmm0, %zmm22; \
	vmovapd	%zmm0, %zmm23;
#define NEG_ACC \
	vmovapd		LC03(%rip), %zmm24; \
	vxorpd		%zmm24, %zmm0, %zmm0; \
	vxorpd		%zmm24, %zmm1, %zmm1; \
	vxorpd		%zmm24, %zmm2, %zmm2; \
	vxorpd		%zmm24, %zmm3, %zmm3; \
	vxorpd		%zmm24, %zmm4, %zmm4; \
	vxorpd		%zmm24, %zmm5, %zmm5; \
	vxorpd		%zmm24, %zmm6, %zmm6; \
	vxorpd		%zmm24, %zmm7, %zmm7; \
	vxorpd		%zmm24, %zmm8, %zmm8; \
	vxorpd		%zmm24, %zmm9, %zmm9; \
	vxorpd		%zmm24, %zmm10, %zmm10; \
	vxorpd		%zmm24, %zmm11, %zmm11; \
	vxorpd		%zmm24, %zmm12, %zmm12; \
	vxorpd		%zmm24, %zmm13, %zmm13; \
	vxorpd		%zmm24, %zmm14, %zmm14; \
	vxorpd		%zmm24, %zmm15, %zmm15; \
	vxorpd		%zmm24, %zmm16, %zmm16; \
	vxorpd		%zmm24, %zmm17, %zmm17; \
	vxorpd		%zmm24, %zmm18, %zmm18; \
	vxorpd		%zmm24, %zmm19, %zmm19; \
	vxorpd		%zmm24, %zmm20, %zmm20; \
	vxorpd		%zmm24, %zmm21, %zmm21; \
	vxorpd		%zmm24, %zmm22, %zmm22; \
	vxorpd		%zmm24, %zmm23, %zmm23;

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
	vpxord	%zmm0, %zmm0, %zmm0; \
	vmovapd	%zmm0, %zmm1; \
	vmovapd	%zmm0, %zmm2; \
	vmovapd	%zmm0, %zmm3; \
	vmovapd	%zmm0, %zmm4; \
	vmovapd	%zmm0, %zmm5; \
	vmovapd	%zmm0, %zmm6; \
	vmovapd	%zmm0, %zmm7; \
	vmovapd	%zmm0, %zmm8; \
	vmovapd	%zmm0, %zmm9; \
	vmovapd	%zmm0, %zmm10; \
	vmovapd	%zmm0, %zmm11; \
	vmovapd	%zmm0, %zmm12; \
	vmovapd	%zmm0, %zmm13; \
	vmovapd	%zmm0, %zmm14; \
	vmovapd	%zmm0, %zmm15; \
	vmovapd	%zmm0, %zmm16; \
	vmovapd	%zmm0, %zmm17; \
	vmovapd	%zmm0, %zmm18; \
	vmovapd	%zmm0, %zmm19; \
	vmovapd	%zmm0, %zmm20; \
	vmovapd	%zmm0, %zmm21; \
	vmovapd	%zmm0, %zmm22; \
	vmovapd	%zmm0, %zmm23;
#define NEG_ACC \
	vmovapd		.LC03(%rip), %zmm24; \
	vxorpd		%zmm24, %zmm0, %zmm0; \
	vxorpd		%zmm24, %zmm1, %zmm1; \
	vxorpd		%zmm24, %zmm2, %zmm2; \
	vxorpd		%zmm24, %zmm3, %zmm3; \
	vxorpd		%zmm24, %zmm4, %zmm4; \
	vxorpd		%zmm24, %zmm5, %zmm5; \
	vxorpd		%zmm24, %zmm6, %zmm6; \
	vxorpd		%zmm24, %zmm7, %zmm7; \
	vxorpd		%zmm24, %zmm8, %zmm8; \
	vxorpd		%zmm24, %zmm9, %zmm9; \
	vxorpd		%zmm24, %zmm10, %zmm10; \
	vxorpd		%zmm24, %zmm11, %zmm11; \
	vxorpd		%zmm24, %zmm12, %zmm12; \
	vxorpd		%zmm24, %zmm13, %zmm13; \
	vxorpd		%zmm24, %zmm14, %zmm14; \
	vxorpd		%zmm24, %zmm15, %zmm15; \
	vxorpd		%zmm24, %zmm16, %zmm16; \
	vxorpd		%zmm24, %zmm17, %zmm17; \
	vxorpd		%zmm24, %zmm18, %zmm18; \
	vxorpd		%zmm24, %zmm19, %zmm19; \
	vxorpd		%zmm24, %zmm20, %zmm20; \
	vxorpd		%zmm24, %zmm21, %zmm21; \
	vxorpd		%zmm24, %zmm22, %zmm22; \
	vxorpd		%zmm24, %zmm23, %zmm23;

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
// zmm0  <- []
// ...
// zmm23  <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+8*k*sizeof(double)
// r12   <- 4*sda*sizeof(double)
// r13   <- B+8*k*sizeof(double)
// zmm0  <- []
// ...
// zmm23  <- []

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_24X8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_24x8_lib8)
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	// prefetch
	prefetcht0	0+0*64(%r13) // software prefetch
	prefetcht0	0+1*64(%r13) // software prefetch
	prefetcht0	0+2*64(%r13) // software prefetch
	prefetcht0	0+3*64(%r13) // software prefetch

	// preload
	vmovapd 		0(%r11), %zmm25 // A0
	vmovapd 		0(%r11, %r12), %zmm27 // A1
	vmovapd 		0(%r11, %r12, 2), %zmm29 // A2

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vmovapd			64(%r11), %zmm26 // A
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vmovapd			64(%r11, %r12), %zmm28 // A
	prefetcht0	256+0*64(%r13) // software prefetch
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm27, %zmm24, %zmm10
	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vmovapd			64(%r11, %r12, 2), %zmm30 // A
	prefetcht0	256+1*64(%r13) // software prefetch
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm27, %zmm24, %zmm11
	vfmadd231pd		%zmm29, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm27, %zmm24, %zmm12
	vfmadd231pd		%zmm29, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm27, %zmm24, %zmm13
	vfmadd231pd		%zmm29, %zmm24, %zmm21
	vbroadcastsd	48(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm27, %zmm24, %zmm14
	vfmadd231pd		%zmm29, %zmm24, %zmm22
	vbroadcastsd	56(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm27, %zmm24, %zmm15
	vfmadd231pd		%zmm29, %zmm24, %zmm23
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	0+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vfmadd231pd		%zmm28, %zmm24, %zmm8
	vfmadd231pd		%zmm30, %zmm24, %zmm16
	vmovapd			128(%r11), %zmm25 // A
	vbroadcastsd	8+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vfmadd231pd		%zmm28, %zmm24, %zmm9
	vfmadd231pd		%zmm30, %zmm24, %zmm17
	vmovapd			128(%r11, %r12), %zmm27 // A
	vbroadcastsd	16+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vfmadd231pd		%zmm28, %zmm24, %zmm10
	vfmadd231pd		%zmm30, %zmm24, %zmm18
	vmovapd			128(%r11, %r12, 2), %zmm29 // A
	vbroadcastsd	24+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vfmadd231pd		%zmm28, %zmm24, %zmm11
	vfmadd231pd		%zmm30, %zmm24, %zmm19
	vbroadcastsd	32+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vfmadd231pd		%zmm28, %zmm24, %zmm12
	vfmadd231pd		%zmm30, %zmm24, %zmm20
	vbroadcastsd	40+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vfmadd231pd		%zmm28, %zmm24, %zmm13
	vfmadd231pd		%zmm30, %zmm24, %zmm21
	vbroadcastsd	48+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vfmadd231pd		%zmm28, %zmm24, %zmm14
	vfmadd231pd		%zmm30, %zmm24, %zmm22
	vbroadcastsd	56+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7
	vfmadd231pd		%zmm28, %zmm24, %zmm15
	vfmadd231pd		%zmm30, %zmm24, %zmm23

	// unroll 0
	vbroadcastsd	0+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vmovapd			192(%r11), %zmm26 // A
	vbroadcastsd	8+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vmovapd			192(%r11, %r12), %zmm28 // A
	prefetcht0	256+2*64(%r13) // software prefetch
	vbroadcastsd	16+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm27, %zmm24, %zmm10
	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vmovapd			192(%r11, %r12, 2), %zmm30 // A
	prefetcht0	256+3*64(%r13) // software prefetch
	vbroadcastsd	24+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm27, %zmm24, %zmm11
	vfmadd231pd		%zmm29, %zmm24, %zmm19
	vbroadcastsd	32+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm27, %zmm24, %zmm12
	vfmadd231pd		%zmm29, %zmm24, %zmm20
	vbroadcastsd	40+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm27, %zmm24, %zmm13
	vfmadd231pd		%zmm29, %zmm24, %zmm21
	vbroadcastsd	48+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm27, %zmm24, %zmm14
	vfmadd231pd		%zmm29, %zmm24, %zmm22
	vbroadcastsd	56+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm27, %zmm24, %zmm15
	vfmadd231pd		%zmm29, %zmm24, %zmm23
	addq	$ 256, %r11

	// unroll 0
	vbroadcastsd	0+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vfmadd231pd		%zmm28, %zmm24, %zmm8
	vfmadd231pd		%zmm30, %zmm24, %zmm16
	vmovapd			0(%r11), %zmm25 // A
	vbroadcastsd	8+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vfmadd231pd		%zmm28, %zmm24, %zmm9
	vfmadd231pd		%zmm30, %zmm24, %zmm17
	vmovapd			0(%r11, %r12), %zmm27 // A
	vbroadcastsd	16+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vfmadd231pd		%zmm28, %zmm24, %zmm10
	vfmadd231pd		%zmm30, %zmm24, %zmm18
	vmovapd			0(%r11, %r12, 2), %zmm29 // A
	vbroadcastsd	24+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vfmadd231pd		%zmm28, %zmm24, %zmm11
	vfmadd231pd		%zmm30, %zmm24, %zmm19
	vbroadcastsd	32+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vfmadd231pd		%zmm28, %zmm24, %zmm12
	vfmadd231pd		%zmm30, %zmm24, %zmm20
	vbroadcastsd	40+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vfmadd231pd		%zmm28, %zmm24, %zmm13
	vfmadd231pd		%zmm30, %zmm24, %zmm21
	vbroadcastsd	48+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vfmadd231pd		%zmm28, %zmm24, %zmm14
	vfmadd231pd		%zmm30, %zmm24, %zmm22
	vbroadcastsd	56+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7
	vfmadd231pd		%zmm28, %zmm24, %zmm15
	vfmadd231pd		%zmm30, %zmm24, %zmm23
	addq	$ 256, %r13

	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vmovapd			64(%r11), %zmm26 // A
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vmovapd			64(%r11, %r12), %zmm28 // A
//	prefetcht0	128(%r13) // software prefetch
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm27, %zmm24, %zmm10
	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vmovapd			64(%r11, %r12, 2), %zmm30 // A
//	prefetcht0	128+64(%r13) // software prefetch
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm27, %zmm24, %zmm11
	vfmadd231pd		%zmm29, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm27, %zmm24, %zmm12
	vfmadd231pd		%zmm29, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm27, %zmm24, %zmm13
	vfmadd231pd		%zmm29, %zmm24, %zmm21
	vbroadcastsd	48(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm27, %zmm24, %zmm14
	vfmadd231pd		%zmm29, %zmm24, %zmm22
	vbroadcastsd	56(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm27, %zmm24, %zmm15
	vfmadd231pd		%zmm29, %zmm24, %zmm23
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	0+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vfmadd231pd		%zmm28, %zmm24, %zmm8
	vfmadd231pd		%zmm30, %zmm24, %zmm16
	vmovapd			128(%r11), %zmm25 // A
	vbroadcastsd	8+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vfmadd231pd		%zmm28, %zmm24, %zmm9
	vfmadd231pd		%zmm30, %zmm24, %zmm17
	vmovapd			128(%r11, %r12), %zmm27 // A
	vbroadcastsd	16+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vfmadd231pd		%zmm28, %zmm24, %zmm10
	vfmadd231pd		%zmm30, %zmm24, %zmm18
	vmovapd			128(%r11, %r12, 2), %zmm29 // A
	vbroadcastsd	24+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vfmadd231pd		%zmm28, %zmm24, %zmm11
	vfmadd231pd		%zmm30, %zmm24, %zmm19
	vbroadcastsd	32+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vfmadd231pd		%zmm28, %zmm24, %zmm12
	vfmadd231pd		%zmm30, %zmm24, %zmm20
	vbroadcastsd	40+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vfmadd231pd		%zmm28, %zmm24, %zmm13
	vfmadd231pd		%zmm30, %zmm24, %zmm21
	vbroadcastsd	48+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vfmadd231pd		%zmm28, %zmm24, %zmm14
	vfmadd231pd		%zmm30, %zmm24, %zmm22
	vbroadcastsd	56+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7
	vfmadd231pd		%zmm28, %zmm24, %zmm15
	vfmadd231pd		%zmm30, %zmm24, %zmm23

	// unroll 0
	vbroadcastsd	0+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vmovapd			192(%r11), %zmm26 // A
	vbroadcastsd	8+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vmovapd			192(%r11, %r12), %zmm28 // A
//	prefetcht0	128(%r13) // software prefetch
	vbroadcastsd	16+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm27, %zmm24, %zmm10
	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vmovapd			192(%r11, %r12, 2), %zmm30 // A
//	prefetcht0	128+64(%r13) // software prefetch
	vbroadcastsd	24+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm27, %zmm24, %zmm11
	vfmadd231pd		%zmm29, %zmm24, %zmm19
	vbroadcastsd	32+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm27, %zmm24, %zmm12
	vfmadd231pd		%zmm29, %zmm24, %zmm20
	vbroadcastsd	40+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm27, %zmm24, %zmm13
	vfmadd231pd		%zmm29, %zmm24, %zmm21
	vbroadcastsd	48+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm27, %zmm24, %zmm14
	vfmadd231pd		%zmm29, %zmm24, %zmm22
	vbroadcastsd	56+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm27, %zmm24, %zmm15
	vfmadd231pd		%zmm29, %zmm24, %zmm23
	addq	$ 256, %r11

	// unroll 0
	vbroadcastsd	0+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vfmadd231pd		%zmm28, %zmm24, %zmm8
	vfmadd231pd		%zmm30, %zmm24, %zmm16
//	vmovapd			0(%r11), %zmm25 // A
	vbroadcastsd	8+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vfmadd231pd		%zmm28, %zmm24, %zmm9
	vfmadd231pd		%zmm30, %zmm24, %zmm17
//	vmovapd			0(%r11, %r12), %zmm27 // A
	vbroadcastsd	16+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vfmadd231pd		%zmm28, %zmm24, %zmm10
	vfmadd231pd		%zmm30, %zmm24, %zmm18
//	vmovapd			0(%r11, %r12, 2), %zmm29 // A
	vbroadcastsd	24+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vfmadd231pd		%zmm28, %zmm24, %zmm11
	vfmadd231pd		%zmm30, %zmm24, %zmm19
	vbroadcastsd	32+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vfmadd231pd		%zmm28, %zmm24, %zmm12
	vfmadd231pd		%zmm30, %zmm24, %zmm20
	vbroadcastsd	40+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vfmadd231pd		%zmm28, %zmm24, %zmm13
	vfmadd231pd		%zmm30, %zmm24, %zmm21
	vbroadcastsd	48+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vfmadd231pd		%zmm28, %zmm24, %zmm14
	vfmadd231pd		%zmm30, %zmm24, %zmm22
	vbroadcastsd	56+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7
	vfmadd231pd		%zmm28, %zmm24, %zmm15
	vfmadd231pd		%zmm30, %zmm24, %zmm23
	addq	$ 256, %r13

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovapd			0(%r11), %zmm25 // A
	vmovapd			0(%r11, %r12), %zmm27 // A
	vmovapd			0(%r11, %r12, 2), %zmm29 // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm27, %zmm24, %zmm10
	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm27, %zmm24, %zmm11
	vfmadd231pd		%zmm29, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm27, %zmm24, %zmm12
	vfmadd231pd		%zmm29, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm27, %zmm24, %zmm13
	vfmadd231pd		%zmm29, %zmm24, %zmm21
	vbroadcastsd	48(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm27, %zmm24, %zmm14
	vfmadd231pd		%zmm29, %zmm24, %zmm22
	vbroadcastsd	56(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm27, %zmm24, %zmm15
	vfmadd231pd		%zmm29, %zmm24, %zmm23

	addq	$ 64, %r11
	addq	$ 64, %r13
	subl	$ 1, %r10d

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_24x8_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- 4*sda*sizeof(double)
// r13   <- B
// zmm0  <- []
// ...
// zmm23  <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+8*k*sizeof(double)
// r12   <- 4*sda*sizeof(double)
// r13   <- B+8*k*sizeof(double)
// zmm0  <- []
// ...
// zmm23  <- []

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_3VX8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_3vx8_lib8)
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	// prefetch

	// preload
	vmovapd 		0(%r11), %zmm25 // A0
	vmovapd 		0(%r11, %r12), %zmm27 // A1
	vmovapd 		0(%r11, %r12, 2), %zmm29 {%k1}{z} // A1

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vmovapd			64(%r11), %zmm26 // A
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vmovapd			64(%r11, %r12), %zmm28 // A
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm27, %zmm24, %zmm10
	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vmovapd			64(%r11, %r12, 2), %zmm30 {%k1}{z} // A
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm27, %zmm24, %zmm11
	vfmadd231pd		%zmm29, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm27, %zmm24, %zmm12
	vfmadd231pd		%zmm29, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm27, %zmm24, %zmm13
	vfmadd231pd		%zmm29, %zmm24, %zmm21
	vbroadcastsd	48(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm27, %zmm24, %zmm14
	vfmadd231pd		%zmm29, %zmm24, %zmm22
	vbroadcastsd	56(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm27, %zmm24, %zmm15
	vfmadd231pd		%zmm29, %zmm24, %zmm23
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	0+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vfmadd231pd		%zmm28, %zmm24, %zmm8
	vfmadd231pd		%zmm30, %zmm24, %zmm16
	vmovapd			128(%r11), %zmm25 // A
	vbroadcastsd	8+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vfmadd231pd		%zmm28, %zmm24, %zmm9
	vfmadd231pd		%zmm30, %zmm24, %zmm17
	vmovapd			128(%r11, %r12), %zmm27 // A
	vbroadcastsd	16+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vfmadd231pd		%zmm28, %zmm24, %zmm10
	vfmadd231pd		%zmm30, %zmm24, %zmm18
	vmovapd			128(%r11, %r12, 2), %zmm29 {%k1}{z} // A
	vbroadcastsd	24+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vfmadd231pd		%zmm28, %zmm24, %zmm11
	vfmadd231pd		%zmm30, %zmm24, %zmm19
	vbroadcastsd	32+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vfmadd231pd		%zmm28, %zmm24, %zmm12
	vfmadd231pd		%zmm30, %zmm24, %zmm20
	vbroadcastsd	40+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vfmadd231pd		%zmm28, %zmm24, %zmm13
	vfmadd231pd		%zmm30, %zmm24, %zmm21
	vbroadcastsd	48+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vfmadd231pd		%zmm28, %zmm24, %zmm14
	vfmadd231pd		%zmm30, %zmm24, %zmm22
	vbroadcastsd	56+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7
	vfmadd231pd		%zmm28, %zmm24, %zmm15
	vfmadd231pd		%zmm30, %zmm24, %zmm23

	// unroll 0
	vbroadcastsd	0+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vmovapd			192(%r11), %zmm26 // A
	vbroadcastsd	8+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vmovapd			192(%r11, %r12), %zmm28 // A
	vbroadcastsd	16+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm27, %zmm24, %zmm10
	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vmovapd			192(%r11, %r12, 2), %zmm30 {%k1}{z} // A
	vbroadcastsd	24+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm27, %zmm24, %zmm11
	vfmadd231pd		%zmm29, %zmm24, %zmm19
	vbroadcastsd	32+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm27, %zmm24, %zmm12
	vfmadd231pd		%zmm29, %zmm24, %zmm20
	vbroadcastsd	40+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm27, %zmm24, %zmm13
	vfmadd231pd		%zmm29, %zmm24, %zmm21
	vbroadcastsd	48+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm27, %zmm24, %zmm14
	vfmadd231pd		%zmm29, %zmm24, %zmm22
	vbroadcastsd	56+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm27, %zmm24, %zmm15
	vfmadd231pd		%zmm29, %zmm24, %zmm23
	addq	$ 256, %r11

	// unroll 0
	vbroadcastsd	0+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vfmadd231pd		%zmm28, %zmm24, %zmm8
	vfmadd231pd		%zmm30, %zmm24, %zmm16
	vmovapd			0(%r11), %zmm25 // A
	vbroadcastsd	8+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vfmadd231pd		%zmm28, %zmm24, %zmm9
	vfmadd231pd		%zmm30, %zmm24, %zmm17
	vmovapd			0(%r11, %r12), %zmm27 // A
	vbroadcastsd	16+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vfmadd231pd		%zmm28, %zmm24, %zmm10
	vfmadd231pd		%zmm30, %zmm24, %zmm18
	vmovapd			0(%r11, %r12, 2), %zmm29 {%k1}{z} // A
	vbroadcastsd	24+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vfmadd231pd		%zmm28, %zmm24, %zmm11
	vfmadd231pd		%zmm30, %zmm24, %zmm19
	vbroadcastsd	32+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vfmadd231pd		%zmm28, %zmm24, %zmm12
	vfmadd231pd		%zmm30, %zmm24, %zmm20
	vbroadcastsd	40+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vfmadd231pd		%zmm28, %zmm24, %zmm13
	vfmadd231pd		%zmm30, %zmm24, %zmm21
	vbroadcastsd	48+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vfmadd231pd		%zmm28, %zmm24, %zmm14
	vfmadd231pd		%zmm30, %zmm24, %zmm22
	vbroadcastsd	56+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7
	vfmadd231pd		%zmm28, %zmm24, %zmm15
	vfmadd231pd		%zmm30, %zmm24, %zmm23
	addq	$ 256, %r13

	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vmovapd			64(%r11), %zmm26 // A
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vmovapd			64(%r11, %r12), %zmm28 // A
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm27, %zmm24, %zmm10
	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vmovapd			64(%r11, %r12, 2), %zmm30 {%k1}{z} // A
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm27, %zmm24, %zmm11
	vfmadd231pd		%zmm29, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm27, %zmm24, %zmm12
	vfmadd231pd		%zmm29, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm27, %zmm24, %zmm13
	vfmadd231pd		%zmm29, %zmm24, %zmm21
	vbroadcastsd	48(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm27, %zmm24, %zmm14
	vfmadd231pd		%zmm29, %zmm24, %zmm22
	vbroadcastsd	56(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm27, %zmm24, %zmm15
	vfmadd231pd		%zmm29, %zmm24, %zmm23
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	0+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vfmadd231pd		%zmm28, %zmm24, %zmm8
	vfmadd231pd		%zmm30, %zmm24, %zmm16
	vmovapd			128(%r11), %zmm25 // A
	vbroadcastsd	8+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vfmadd231pd		%zmm28, %zmm24, %zmm9
	vfmadd231pd		%zmm30, %zmm24, %zmm17
	vmovapd			128(%r11, %r12), %zmm27 // A
	vbroadcastsd	16+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vfmadd231pd		%zmm28, %zmm24, %zmm10
	vfmadd231pd		%zmm30, %zmm24, %zmm18
	vmovapd			128(%r11, %r12, 2), %zmm29 {%k1}{z} // A
	vbroadcastsd	24+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vfmadd231pd		%zmm28, %zmm24, %zmm11
	vfmadd231pd		%zmm30, %zmm24, %zmm19
	vbroadcastsd	32+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vfmadd231pd		%zmm28, %zmm24, %zmm12
	vfmadd231pd		%zmm30, %zmm24, %zmm20
	vbroadcastsd	40+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vfmadd231pd		%zmm28, %zmm24, %zmm13
	vfmadd231pd		%zmm30, %zmm24, %zmm21
	vbroadcastsd	48+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vfmadd231pd		%zmm28, %zmm24, %zmm14
	vfmadd231pd		%zmm30, %zmm24, %zmm22
	vbroadcastsd	56+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7
	vfmadd231pd		%zmm28, %zmm24, %zmm15
	vfmadd231pd		%zmm30, %zmm24, %zmm23

	// unroll 0
	vbroadcastsd	0+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vmovapd			192(%r11), %zmm26 // A
	vbroadcastsd	8+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vmovapd			192(%r11, %r12), %zmm28 // A
	vbroadcastsd	16+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm27, %zmm24, %zmm10
	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vmovapd			192(%r11, %r12, 2), %zmm30 {%k1}{z} // A
	vbroadcastsd	24+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm27, %zmm24, %zmm11
	vfmadd231pd		%zmm29, %zmm24, %zmm19
	vbroadcastsd	32+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm27, %zmm24, %zmm12
	vfmadd231pd		%zmm29, %zmm24, %zmm20
	vbroadcastsd	40+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm27, %zmm24, %zmm13
	vfmadd231pd		%zmm29, %zmm24, %zmm21
	vbroadcastsd	48+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm27, %zmm24, %zmm14
	vfmadd231pd		%zmm29, %zmm24, %zmm22
	vbroadcastsd	56+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm27, %zmm24, %zmm15
	vfmadd231pd		%zmm29, %zmm24, %zmm23
	addq	$ 256, %r11

	// unroll 0
	vbroadcastsd	0+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vfmadd231pd		%zmm28, %zmm24, %zmm8
	vfmadd231pd		%zmm30, %zmm24, %zmm16
//	vmovapd			0(%r11), %zmm25 // A
	vbroadcastsd	8+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vfmadd231pd		%zmm28, %zmm24, %zmm9
	vfmadd231pd		%zmm30, %zmm24, %zmm17
//	vmovapd			0(%r11, %r12), %zmm27 // A
	vbroadcastsd	16+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vfmadd231pd		%zmm28, %zmm24, %zmm10
	vfmadd231pd		%zmm30, %zmm24, %zmm18
//	vmovapd			0(%r11, %r12, 2), %zmm29 {%k1}{z} // A
	vbroadcastsd	24+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vfmadd231pd		%zmm28, %zmm24, %zmm11
	vfmadd231pd		%zmm30, %zmm24, %zmm19
	vbroadcastsd	32+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vfmadd231pd		%zmm28, %zmm24, %zmm12
	vfmadd231pd		%zmm30, %zmm24, %zmm20
	vbroadcastsd	40+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vfmadd231pd		%zmm28, %zmm24, %zmm13
	vfmadd231pd		%zmm30, %zmm24, %zmm21
	vbroadcastsd	48+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vfmadd231pd		%zmm28, %zmm24, %zmm14
	vfmadd231pd		%zmm30, %zmm24, %zmm22
	vbroadcastsd	56+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7
	vfmadd231pd		%zmm28, %zmm24, %zmm15
	vfmadd231pd		%zmm30, %zmm24, %zmm23
	addq	$ 256, %r13

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovapd			0(%r11), %zmm25 // A
	vmovapd			0(%r11, %r12), %zmm27 // A
	vmovapd			0(%r11, %r12, 2), %zmm29 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm27, %zmm24, %zmm10
	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm27, %zmm24, %zmm11
	vfmadd231pd		%zmm29, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm27, %zmm24, %zmm12
	vfmadd231pd		%zmm29, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm27, %zmm24, %zmm13
	vfmadd231pd		%zmm29, %zmm24, %zmm21
	vbroadcastsd	48(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm27, %zmm24, %zmm14
	vfmadd231pd		%zmm29, %zmm24, %zmm22
	vbroadcastsd	56(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm27, %zmm24, %zmm15
	vfmadd231pd		%zmm29, %zmm24, %zmm23

	addq	$ 64, %r11
	addq	$ 64, %r13
	subl	$ 1, %r10d

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_3vx8_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- 4*sda*sizeof(double)
// r13   <- B
// zmm0  <- []
// ...
// zmm23  <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+8*k*sizeof(double)
// r12   <- 4*sda*sizeof(double)
// r13   <- B+8*k*sizeof(double)
// zmm0  <- []
// ...
// zmm23  <- []

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_3VX7_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_3vx7_lib8)
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	// prefetch

	// preload
	vmovapd 		0(%r11), %zmm25 // A0
	vmovapd 		0(%r11, %r12), %zmm27 // A1
	vmovapd 		0(%r11, %r12, 2), %zmm29 {%k1}{z} // A1

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vmovapd			64(%r11), %zmm26 // A
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vmovapd			64(%r11, %r12), %zmm28 // A
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm27, %zmm24, %zmm10
	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vmovapd			64(%r11, %r12, 2), %zmm30 {%k1}{z} // A
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm27, %zmm24, %zmm11
	vfmadd231pd		%zmm29, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm27, %zmm24, %zmm12
	vfmadd231pd		%zmm29, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm27, %zmm24, %zmm13
	vfmadd231pd		%zmm29, %zmm24, %zmm21
	vbroadcastsd	48(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm27, %zmm24, %zmm14
	vfmadd231pd		%zmm29, %zmm24, %zmm22
//	vbroadcastsd	56(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm27, %zmm24, %zmm15
//	vfmadd231pd		%zmm29, %zmm24, %zmm23
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	0+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vfmadd231pd		%zmm28, %zmm24, %zmm8
	vfmadd231pd		%zmm30, %zmm24, %zmm16
	vmovapd			128(%r11), %zmm25 // A
	vbroadcastsd	8+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vfmadd231pd		%zmm28, %zmm24, %zmm9
	vfmadd231pd		%zmm30, %zmm24, %zmm17
	vmovapd			128(%r11, %r12), %zmm27 // A
	vbroadcastsd	16+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vfmadd231pd		%zmm28, %zmm24, %zmm10
	vfmadd231pd		%zmm30, %zmm24, %zmm18
	vmovapd			128(%r11, %r12, 2), %zmm29 {%k1}{z} // A
	vbroadcastsd	24+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vfmadd231pd		%zmm28, %zmm24, %zmm11
	vfmadd231pd		%zmm30, %zmm24, %zmm19
	vbroadcastsd	32+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vfmadd231pd		%zmm28, %zmm24, %zmm12
	vfmadd231pd		%zmm30, %zmm24, %zmm20
	vbroadcastsd	40+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vfmadd231pd		%zmm28, %zmm24, %zmm13
	vfmadd231pd		%zmm30, %zmm24, %zmm21
	vbroadcastsd	48+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vfmadd231pd		%zmm28, %zmm24, %zmm14
	vfmadd231pd		%zmm30, %zmm24, %zmm22
//	vbroadcastsd	56+64(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7
//	vfmadd231pd		%zmm28, %zmm24, %zmm15
//	vfmadd231pd		%zmm30, %zmm24, %zmm23

	// unroll 0
	vbroadcastsd	0+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vmovapd			192(%r11), %zmm26 // A
	vbroadcastsd	8+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vmovapd			192(%r11, %r12), %zmm28 // A
	vbroadcastsd	16+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm27, %zmm24, %zmm10
	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vmovapd			192(%r11, %r12, 2), %zmm30 {%k1}{z} // A
	vbroadcastsd	24+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm27, %zmm24, %zmm11
	vfmadd231pd		%zmm29, %zmm24, %zmm19
	vbroadcastsd	32+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm27, %zmm24, %zmm12
	vfmadd231pd		%zmm29, %zmm24, %zmm20
	vbroadcastsd	40+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm27, %zmm24, %zmm13
	vfmadd231pd		%zmm29, %zmm24, %zmm21
	vbroadcastsd	48+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm27, %zmm24, %zmm14
	vfmadd231pd		%zmm29, %zmm24, %zmm22
//	vbroadcastsd	56+128(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm27, %zmm24, %zmm15
//	vfmadd231pd		%zmm29, %zmm24, %zmm23
	addq	$ 256, %r11

	// unroll 0
	vbroadcastsd	0+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vfmadd231pd		%zmm28, %zmm24, %zmm8
	vfmadd231pd		%zmm30, %zmm24, %zmm16
	vmovapd			0(%r11), %zmm25 // A
	vbroadcastsd	8+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vfmadd231pd		%zmm28, %zmm24, %zmm9
	vfmadd231pd		%zmm30, %zmm24, %zmm17
	vmovapd			0(%r11, %r12), %zmm27 // A
	vbroadcastsd	16+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vfmadd231pd		%zmm28, %zmm24, %zmm10
	vfmadd231pd		%zmm30, %zmm24, %zmm18
	vmovapd			0(%r11, %r12, 2), %zmm29 {%k1}{z} // A
	vbroadcastsd	24+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vfmadd231pd		%zmm28, %zmm24, %zmm11
	vfmadd231pd		%zmm30, %zmm24, %zmm19
	vbroadcastsd	32+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vfmadd231pd		%zmm28, %zmm24, %zmm12
	vfmadd231pd		%zmm30, %zmm24, %zmm20
	vbroadcastsd	40+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vfmadd231pd		%zmm28, %zmm24, %zmm13
	vfmadd231pd		%zmm30, %zmm24, %zmm21
	vbroadcastsd	48+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vfmadd231pd		%zmm28, %zmm24, %zmm14
	vfmadd231pd		%zmm30, %zmm24, %zmm22
//	vbroadcastsd	56+192(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7
//	vfmadd231pd		%zmm28, %zmm24, %zmm15
//	vfmadd231pd		%zmm30, %zmm24, %zmm23
	addq	$ 256, %r13

	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vmovapd			64(%r11), %zmm26 // A
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vmovapd			64(%r11, %r12), %zmm28 // A
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm27, %zmm24, %zmm10
	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vmovapd			64(%r11, %r12, 2), %zmm30 {%k1}{z} // A
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm27, %zmm24, %zmm11
	vfmadd231pd		%zmm29, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm27, %zmm24, %zmm12
	vfmadd231pd		%zmm29, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm27, %zmm24, %zmm13
	vfmadd231pd		%zmm29, %zmm24, %zmm21
	vbroadcastsd	48(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm27, %zmm24, %zmm14
	vfmadd231pd		%zmm29, %zmm24, %zmm22
//	vbroadcastsd	56(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm27, %zmm24, %zmm15
//	vfmadd231pd		%zmm29, %zmm24, %zmm23
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	0+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vfmadd231pd		%zmm28, %zmm24, %zmm8
	vfmadd231pd		%zmm30, %zmm24, %zmm16
	vmovapd			128(%r11), %zmm25 // A
	vbroadcastsd	8+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vfmadd231pd		%zmm28, %zmm24, %zmm9
	vfmadd231pd		%zmm30, %zmm24, %zmm17
	vmovapd			128(%r11, %r12), %zmm27 // A
	vbroadcastsd	16+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vfmadd231pd		%zmm28, %zmm24, %zmm10
	vfmadd231pd		%zmm30, %zmm24, %zmm18
	vmovapd			128(%r11, %r12, 2), %zmm29 {%k1}{z} // A
	vbroadcastsd	24+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vfmadd231pd		%zmm28, %zmm24, %zmm11
	vfmadd231pd		%zmm30, %zmm24, %zmm19
	vbroadcastsd	32+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vfmadd231pd		%zmm28, %zmm24, %zmm12
	vfmadd231pd		%zmm30, %zmm24, %zmm20
	vbroadcastsd	40+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vfmadd231pd		%zmm28, %zmm24, %zmm13
	vfmadd231pd		%zmm30, %zmm24, %zmm21
	vbroadcastsd	48+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vfmadd231pd		%zmm28, %zmm24, %zmm14
	vfmadd231pd		%zmm30, %zmm24, %zmm22
//	vbroadcastsd	56+64(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7
//	vfmadd231pd		%zmm28, %zmm24, %zmm15
//	vfmadd231pd		%zmm30, %zmm24, %zmm23

	// unroll 0
	vbroadcastsd	0+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vmovapd			192(%r11), %zmm26 // A
	vbroadcastsd	8+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vmovapd			192(%r11, %r12), %zmm28 // A
	vbroadcastsd	16+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm27, %zmm24, %zmm10
	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vmovapd			192(%r11, %r12, 2), %zmm30 {%k1}{z} // A
	vbroadcastsd	24+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm27, %zmm24, %zmm11
	vfmadd231pd		%zmm29, %zmm24, %zmm19
	vbroadcastsd	32+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm27, %zmm24, %zmm12
	vfmadd231pd		%zmm29, %zmm24, %zmm20
	vbroadcastsd	40+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm27, %zmm24, %zmm13
	vfmadd231pd		%zmm29, %zmm24, %zmm21
	vbroadcastsd	48+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm27, %zmm24, %zmm14
	vfmadd231pd		%zmm29, %zmm24, %zmm22
//	vbroadcastsd	56+128(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm27, %zmm24, %zmm15
//	vfmadd231pd		%zmm29, %zmm24, %zmm23
	addq	$ 256, %r11

	// unroll 0
	vbroadcastsd	0+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vfmadd231pd		%zmm28, %zmm24, %zmm8
	vfmadd231pd		%zmm30, %zmm24, %zmm16
//	vmovapd			0(%r11), %zmm25 // A
	vbroadcastsd	8+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vfmadd231pd		%zmm28, %zmm24, %zmm9
	vfmadd231pd		%zmm30, %zmm24, %zmm17
//	vmovapd			0(%r11, %r12), %zmm27 // A
	vbroadcastsd	16+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vfmadd231pd		%zmm28, %zmm24, %zmm10
	vfmadd231pd		%zmm30, %zmm24, %zmm18
//	vmovapd			0(%r11, %r12, 2), %zmm29 {%k1}{z} // A
	vbroadcastsd	24+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vfmadd231pd		%zmm28, %zmm24, %zmm11
	vfmadd231pd		%zmm30, %zmm24, %zmm19
	vbroadcastsd	32+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vfmadd231pd		%zmm28, %zmm24, %zmm12
	vfmadd231pd		%zmm30, %zmm24, %zmm20
	vbroadcastsd	40+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vfmadd231pd		%zmm28, %zmm24, %zmm13
	vfmadd231pd		%zmm30, %zmm24, %zmm21
	vbroadcastsd	48+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vfmadd231pd		%zmm28, %zmm24, %zmm14
	vfmadd231pd		%zmm30, %zmm24, %zmm22
//	vbroadcastsd	56+192(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7
//	vfmadd231pd		%zmm28, %zmm24, %zmm15
//	vfmadd231pd		%zmm30, %zmm24, %zmm23
	addq	$ 256, %r13

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovapd			0(%r11), %zmm25 // A
	vmovapd			0(%r11, %r12), %zmm27 // A
	vmovapd			0(%r11, %r12, 2), %zmm29 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm27, %zmm24, %zmm10
	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm27, %zmm24, %zmm11
	vfmadd231pd		%zmm29, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm27, %zmm24, %zmm12
	vfmadd231pd		%zmm29, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm27, %zmm24, %zmm13
	vfmadd231pd		%zmm29, %zmm24, %zmm21
	vbroadcastsd	48(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm27, %zmm24, %zmm14
	vfmadd231pd		%zmm29, %zmm24, %zmm22
//	vbroadcastsd	56(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm27, %zmm24, %zmm15
//	vfmadd231pd		%zmm29, %zmm24, %zmm23

	addq	$ 64, %r11
	addq	$ 64, %r13
	subl	$ 1, %r10d

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_3vx7_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- 4*sda*sizeof(double)
// r13   <- B
// zmm0  <- []
// ...
// zmm23  <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+8*k*sizeof(double)
// r12   <- 4*sda*sizeof(double)
// r13   <- B+8*k*sizeof(double)
// zmm0  <- []
// ...
// zmm23  <- []

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_3VX6_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_3vx6_lib8)
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	// prefetch

	// preload
	vmovapd 		0(%r11), %zmm25 // A0
	vmovapd 		0(%r11, %r12), %zmm27 // A1
	vmovapd 		0(%r11, %r12, 2), %zmm29 {%k1}{z} // A1

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vmovapd			64(%r11), %zmm26 // A
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vmovapd			64(%r11, %r12), %zmm28 // A
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm27, %zmm24, %zmm10
	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vmovapd			64(%r11, %r12, 2), %zmm30 {%k1}{z} // A
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm27, %zmm24, %zmm11
	vfmadd231pd		%zmm29, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm27, %zmm24, %zmm12
	vfmadd231pd		%zmm29, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm27, %zmm24, %zmm13
	vfmadd231pd		%zmm29, %zmm24, %zmm21
//	vbroadcastsd	48(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm27, %zmm24, %zmm14
//	vfmadd231pd		%zmm29, %zmm24, %zmm22
//	vbroadcastsd	56(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm27, %zmm24, %zmm15
//	vfmadd231pd		%zmm29, %zmm24, %zmm23
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	0+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vfmadd231pd		%zmm28, %zmm24, %zmm8
	vfmadd231pd		%zmm30, %zmm24, %zmm16
	vmovapd			128(%r11), %zmm25 // A
	vbroadcastsd	8+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vfmadd231pd		%zmm28, %zmm24, %zmm9
	vfmadd231pd		%zmm30, %zmm24, %zmm17
	vmovapd			128(%r11, %r12), %zmm27 // A
	vbroadcastsd	16+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vfmadd231pd		%zmm28, %zmm24, %zmm10
	vfmadd231pd		%zmm30, %zmm24, %zmm18
	vmovapd			128(%r11, %r12, 2), %zmm29 {%k1}{z} // A
	vbroadcastsd	24+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vfmadd231pd		%zmm28, %zmm24, %zmm11
	vfmadd231pd		%zmm30, %zmm24, %zmm19
	vbroadcastsd	32+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vfmadd231pd		%zmm28, %zmm24, %zmm12
	vfmadd231pd		%zmm30, %zmm24, %zmm20
	vbroadcastsd	40+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vfmadd231pd		%zmm28, %zmm24, %zmm13
	vfmadd231pd		%zmm30, %zmm24, %zmm21
//	vbroadcastsd	48+64(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vfmadd231pd		%zmm28, %zmm24, %zmm14
//	vfmadd231pd		%zmm30, %zmm24, %zmm22
//	vbroadcastsd	56+64(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7
//	vfmadd231pd		%zmm28, %zmm24, %zmm15
//	vfmadd231pd		%zmm30, %zmm24, %zmm23

	// unroll 0
	vbroadcastsd	0+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vmovapd			192(%r11), %zmm26 // A
	vbroadcastsd	8+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vmovapd			192(%r11, %r12), %zmm28 // A
	vbroadcastsd	16+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm27, %zmm24, %zmm10
	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vmovapd			192(%r11, %r12, 2), %zmm30 {%k1}{z} // A
	vbroadcastsd	24+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm27, %zmm24, %zmm11
	vfmadd231pd		%zmm29, %zmm24, %zmm19
	vbroadcastsd	32+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm27, %zmm24, %zmm12
	vfmadd231pd		%zmm29, %zmm24, %zmm20
	vbroadcastsd	40+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm27, %zmm24, %zmm13
	vfmadd231pd		%zmm29, %zmm24, %zmm21
//	vbroadcastsd	48+128(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm27, %zmm24, %zmm14
//	vfmadd231pd		%zmm29, %zmm24, %zmm22
//	vbroadcastsd	56+128(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm27, %zmm24, %zmm15
//	vfmadd231pd		%zmm29, %zmm24, %zmm23
	addq	$ 256, %r11

	// unroll 0
	vbroadcastsd	0+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vfmadd231pd		%zmm28, %zmm24, %zmm8
	vfmadd231pd		%zmm30, %zmm24, %zmm16
	vmovapd			0(%r11), %zmm25 // A
	vbroadcastsd	8+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vfmadd231pd		%zmm28, %zmm24, %zmm9
	vfmadd231pd		%zmm30, %zmm24, %zmm17
	vmovapd			0(%r11, %r12), %zmm27 // A
	vbroadcastsd	16+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vfmadd231pd		%zmm28, %zmm24, %zmm10
	vfmadd231pd		%zmm30, %zmm24, %zmm18
	vmovapd			0(%r11, %r12, 2), %zmm29 {%k1}{z} // A
	vbroadcastsd	24+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vfmadd231pd		%zmm28, %zmm24, %zmm11
	vfmadd231pd		%zmm30, %zmm24, %zmm19
	vbroadcastsd	32+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vfmadd231pd		%zmm28, %zmm24, %zmm12
	vfmadd231pd		%zmm30, %zmm24, %zmm20
	vbroadcastsd	40+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vfmadd231pd		%zmm28, %zmm24, %zmm13
	vfmadd231pd		%zmm30, %zmm24, %zmm21
//	vbroadcastsd	48+192(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vfmadd231pd		%zmm28, %zmm24, %zmm14
//	vfmadd231pd		%zmm30, %zmm24, %zmm22
//	vbroadcastsd	56+192(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7
//	vfmadd231pd		%zmm28, %zmm24, %zmm15
//	vfmadd231pd		%zmm30, %zmm24, %zmm23
	addq	$ 256, %r13

	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vmovapd			64(%r11), %zmm26 // A
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vmovapd			64(%r11, %r12), %zmm28 // A
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm27, %zmm24, %zmm10
	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vmovapd			64(%r11, %r12, 2), %zmm30 {%k1}{z} // A
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm27, %zmm24, %zmm11
	vfmadd231pd		%zmm29, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm27, %zmm24, %zmm12
	vfmadd231pd		%zmm29, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm27, %zmm24, %zmm13
	vfmadd231pd		%zmm29, %zmm24, %zmm21
//	vbroadcastsd	48(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm27, %zmm24, %zmm14
//	vfmadd231pd		%zmm29, %zmm24, %zmm22
//	vbroadcastsd	56(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm27, %zmm24, %zmm15
//	vfmadd231pd		%zmm29, %zmm24, %zmm23
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	0+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vfmadd231pd		%zmm28, %zmm24, %zmm8
	vfmadd231pd		%zmm30, %zmm24, %zmm16
	vmovapd			128(%r11), %zmm25 // A
	vbroadcastsd	8+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vfmadd231pd		%zmm28, %zmm24, %zmm9
	vfmadd231pd		%zmm30, %zmm24, %zmm17
	vmovapd			128(%r11, %r12), %zmm27 // A
	vbroadcastsd	16+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vfmadd231pd		%zmm28, %zmm24, %zmm10
	vfmadd231pd		%zmm30, %zmm24, %zmm18
	vmovapd			128(%r11, %r12, 2), %zmm29 {%k1}{z} // A
	vbroadcastsd	24+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vfmadd231pd		%zmm28, %zmm24, %zmm11
	vfmadd231pd		%zmm30, %zmm24, %zmm19
	vbroadcastsd	32+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vfmadd231pd		%zmm28, %zmm24, %zmm12
	vfmadd231pd		%zmm30, %zmm24, %zmm20
	vbroadcastsd	40+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vfmadd231pd		%zmm28, %zmm24, %zmm13
	vfmadd231pd		%zmm30, %zmm24, %zmm21
//	vbroadcastsd	48+64(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vfmadd231pd		%zmm28, %zmm24, %zmm14
//	vfmadd231pd		%zmm30, %zmm24, %zmm22
//	vbroadcastsd	56+64(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7
//	vfmadd231pd		%zmm28, %zmm24, %zmm15
//	vfmadd231pd		%zmm30, %zmm24, %zmm23

	// unroll 0
	vbroadcastsd	0+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vmovapd			192(%r11), %zmm26 // A
	vbroadcastsd	8+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vmovapd			192(%r11, %r12), %zmm28 // A
	vbroadcastsd	16+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm27, %zmm24, %zmm10
	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vmovapd			192(%r11, %r12, 2), %zmm30 {%k1}{z} // A
	vbroadcastsd	24+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm27, %zmm24, %zmm11
	vfmadd231pd		%zmm29, %zmm24, %zmm19
	vbroadcastsd	32+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm27, %zmm24, %zmm12
	vfmadd231pd		%zmm29, %zmm24, %zmm20
	vbroadcastsd	40+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm27, %zmm24, %zmm13
	vfmadd231pd		%zmm29, %zmm24, %zmm21
//	vbroadcastsd	48+128(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm27, %zmm24, %zmm14
//	vfmadd231pd		%zmm29, %zmm24, %zmm22
//	vbroadcastsd	56+128(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm27, %zmm24, %zmm15
//	vfmadd231pd		%zmm29, %zmm24, %zmm23
	addq	$ 256, %r11

	// unroll 0
	vbroadcastsd	0+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vfmadd231pd		%zmm28, %zmm24, %zmm8
	vfmadd231pd		%zmm30, %zmm24, %zmm16
//	vmovapd			0(%r11), %zmm25 // A
	vbroadcastsd	8+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vfmadd231pd		%zmm28, %zmm24, %zmm9
	vfmadd231pd		%zmm30, %zmm24, %zmm17
//	vmovapd			0(%r11, %r12), %zmm27 // A
	vbroadcastsd	16+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vfmadd231pd		%zmm28, %zmm24, %zmm10
	vfmadd231pd		%zmm30, %zmm24, %zmm18
//	vmovapd			0(%r11, %r12, 2), %zmm29 {%k1}{z} // A
	vbroadcastsd	24+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vfmadd231pd		%zmm28, %zmm24, %zmm11
	vfmadd231pd		%zmm30, %zmm24, %zmm19
	vbroadcastsd	32+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vfmadd231pd		%zmm28, %zmm24, %zmm12
	vfmadd231pd		%zmm30, %zmm24, %zmm20
	vbroadcastsd	40+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vfmadd231pd		%zmm28, %zmm24, %zmm13
	vfmadd231pd		%zmm30, %zmm24, %zmm21
//	vbroadcastsd	48+192(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vfmadd231pd		%zmm28, %zmm24, %zmm14
//	vfmadd231pd		%zmm30, %zmm24, %zmm22
//	vbroadcastsd	56+192(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7
//	vfmadd231pd		%zmm28, %zmm24, %zmm15
//	vfmadd231pd		%zmm30, %zmm24, %zmm23
	addq	$ 256, %r13

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovapd			0(%r11), %zmm25 // A
	vmovapd			0(%r11, %r12), %zmm27 // A
	vmovapd			0(%r11, %r12, 2), %zmm29 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm27, %zmm24, %zmm10
	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm27, %zmm24, %zmm11
	vfmadd231pd		%zmm29, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm27, %zmm24, %zmm12
	vfmadd231pd		%zmm29, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm27, %zmm24, %zmm13
	vfmadd231pd		%zmm29, %zmm24, %zmm21
//	vbroadcastsd	48(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm27, %zmm24, %zmm14
//	vfmadd231pd		%zmm29, %zmm24, %zmm22
//	vbroadcastsd	56(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm27, %zmm24, %zmm15
//	vfmadd231pd		%zmm29, %zmm24, %zmm23

	addq	$ 64, %r11
	addq	$ 64, %r13
	subl	$ 1, %r10d

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_3vx6_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- 4*sda*sizeof(double)
// r13   <- B
// zmm0  <- []
// ...
// zmm23  <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+8*k*sizeof(double)
// r12   <- 4*sda*sizeof(double)
// r13   <- B+8*k*sizeof(double)
// zmm0  <- []
// ...
// zmm23  <- []

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_3VX5_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_3vx5_lib8)
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	// prefetch

	// preload
	vmovapd 		0(%r11), %zmm25 // A0
	vmovapd 		0(%r11, %r12), %zmm27 // A1
	vmovapd 		0(%r11, %r12, 2), %zmm29 {%k1}{z} // A1

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vmovapd			64(%r11), %zmm26 // A
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vmovapd			64(%r11, %r12), %zmm28 // A
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm27, %zmm24, %zmm10
	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vmovapd			64(%r11, %r12, 2), %zmm30 {%k1}{z} // A
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm27, %zmm24, %zmm11
	vfmadd231pd		%zmm29, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm27, %zmm24, %zmm12
	vfmadd231pd		%zmm29, %zmm24, %zmm20
//	vbroadcastsd	40(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vfmadd231pd		%zmm27, %zmm24, %zmm13
//	vfmadd231pd		%zmm29, %zmm24, %zmm21
//	vbroadcastsd	48(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm27, %zmm24, %zmm14
//	vfmadd231pd		%zmm29, %zmm24, %zmm22
//	vbroadcastsd	56(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm27, %zmm24, %zmm15
//	vfmadd231pd		%zmm29, %zmm24, %zmm23
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	0+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vfmadd231pd		%zmm28, %zmm24, %zmm8
	vfmadd231pd		%zmm30, %zmm24, %zmm16
	vmovapd			128(%r11), %zmm25 // A
	vbroadcastsd	8+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vfmadd231pd		%zmm28, %zmm24, %zmm9
	vfmadd231pd		%zmm30, %zmm24, %zmm17
	vmovapd			128(%r11, %r12), %zmm27 // A
	vbroadcastsd	16+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vfmadd231pd		%zmm28, %zmm24, %zmm10
	vfmadd231pd		%zmm30, %zmm24, %zmm18
	vmovapd			128(%r11, %r12, 2), %zmm29 {%k1}{z} // A
	vbroadcastsd	24+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vfmadd231pd		%zmm28, %zmm24, %zmm11
	vfmadd231pd		%zmm30, %zmm24, %zmm19
	vbroadcastsd	32+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vfmadd231pd		%zmm28, %zmm24, %zmm12
	vfmadd231pd		%zmm30, %zmm24, %zmm20
//	vbroadcastsd	40+64(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vfmadd231pd		%zmm28, %zmm24, %zmm13
//	vfmadd231pd		%zmm30, %zmm24, %zmm21
//	vbroadcastsd	48+64(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vfmadd231pd		%zmm28, %zmm24, %zmm14
//	vfmadd231pd		%zmm30, %zmm24, %zmm22
//	vbroadcastsd	56+64(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7
//	vfmadd231pd		%zmm28, %zmm24, %zmm15
//	vfmadd231pd		%zmm30, %zmm24, %zmm23

	// unroll 0
	vbroadcastsd	0+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vmovapd			192(%r11), %zmm26 // A
	vbroadcastsd	8+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vmovapd			192(%r11, %r12), %zmm28 // A
	vbroadcastsd	16+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm27, %zmm24, %zmm10
	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vmovapd			192(%r11, %r12, 2), %zmm30 {%k1}{z} // A
	vbroadcastsd	24+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm27, %zmm24, %zmm11
	vfmadd231pd		%zmm29, %zmm24, %zmm19
	vbroadcastsd	32+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm27, %zmm24, %zmm12
	vfmadd231pd		%zmm29, %zmm24, %zmm20
//	vbroadcastsd	40+128(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vfmadd231pd		%zmm27, %zmm24, %zmm13
//	vfmadd231pd		%zmm29, %zmm24, %zmm21
//	vbroadcastsd	48+128(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm27, %zmm24, %zmm14
//	vfmadd231pd		%zmm29, %zmm24, %zmm22
//	vbroadcastsd	56+128(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm27, %zmm24, %zmm15
//	vfmadd231pd		%zmm29, %zmm24, %zmm23
	addq	$ 256, %r11

	// unroll 0
	vbroadcastsd	0+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vfmadd231pd		%zmm28, %zmm24, %zmm8
	vfmadd231pd		%zmm30, %zmm24, %zmm16
	vmovapd			0(%r11), %zmm25 // A
	vbroadcastsd	8+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vfmadd231pd		%zmm28, %zmm24, %zmm9
	vfmadd231pd		%zmm30, %zmm24, %zmm17
	vmovapd			0(%r11, %r12), %zmm27 // A
	vbroadcastsd	16+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vfmadd231pd		%zmm28, %zmm24, %zmm10
	vfmadd231pd		%zmm30, %zmm24, %zmm18
	vmovapd			0(%r11, %r12, 2), %zmm29 {%k1}{z} // A
	vbroadcastsd	24+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vfmadd231pd		%zmm28, %zmm24, %zmm11
	vfmadd231pd		%zmm30, %zmm24, %zmm19
	vbroadcastsd	32+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vfmadd231pd		%zmm28, %zmm24, %zmm12
	vfmadd231pd		%zmm30, %zmm24, %zmm20
//	vbroadcastsd	40+192(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vfmadd231pd		%zmm28, %zmm24, %zmm13
//	vfmadd231pd		%zmm30, %zmm24, %zmm21
//	vbroadcastsd	48+192(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vfmadd231pd		%zmm28, %zmm24, %zmm14
//	vfmadd231pd		%zmm30, %zmm24, %zmm22
//	vbroadcastsd	56+192(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7
//	vfmadd231pd		%zmm28, %zmm24, %zmm15
//	vfmadd231pd		%zmm30, %zmm24, %zmm23
	addq	$ 256, %r13

	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vmovapd			64(%r11), %zmm26 // A
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vmovapd			64(%r11, %r12), %zmm28 // A
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm27, %zmm24, %zmm10
	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vmovapd			64(%r11, %r12, 2), %zmm30 {%k1}{z} // A
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm27, %zmm24, %zmm11
	vfmadd231pd		%zmm29, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm27, %zmm24, %zmm12
	vfmadd231pd		%zmm29, %zmm24, %zmm20
//	vbroadcastsd	40(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vfmadd231pd		%zmm27, %zmm24, %zmm13
//	vfmadd231pd		%zmm29, %zmm24, %zmm21
//	vbroadcastsd	48(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm27, %zmm24, %zmm14
//	vfmadd231pd		%zmm29, %zmm24, %zmm22
//	vbroadcastsd	56(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm27, %zmm24, %zmm15
//	vfmadd231pd		%zmm29, %zmm24, %zmm23
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	0+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vfmadd231pd		%zmm28, %zmm24, %zmm8
	vfmadd231pd		%zmm30, %zmm24, %zmm16
	vmovapd			128(%r11), %zmm25 // A
	vbroadcastsd	8+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vfmadd231pd		%zmm28, %zmm24, %zmm9
	vfmadd231pd		%zmm30, %zmm24, %zmm17
	vmovapd			128(%r11, %r12), %zmm27 // A
	vbroadcastsd	16+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vfmadd231pd		%zmm28, %zmm24, %zmm10
	vfmadd231pd		%zmm30, %zmm24, %zmm18
	vmovapd			128(%r11, %r12, 2), %zmm29 {%k1}{z} // A
	vbroadcastsd	24+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vfmadd231pd		%zmm28, %zmm24, %zmm11
	vfmadd231pd		%zmm30, %zmm24, %zmm19
	vbroadcastsd	32+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vfmadd231pd		%zmm28, %zmm24, %zmm12
	vfmadd231pd		%zmm30, %zmm24, %zmm20
//	vbroadcastsd	40+64(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vfmadd231pd		%zmm28, %zmm24, %zmm13
//	vfmadd231pd		%zmm30, %zmm24, %zmm21
//	vbroadcastsd	48+64(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vfmadd231pd		%zmm28, %zmm24, %zmm14
//	vfmadd231pd		%zmm30, %zmm24, %zmm22
//	vbroadcastsd	56+64(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7
//	vfmadd231pd		%zmm28, %zmm24, %zmm15
//	vfmadd231pd		%zmm30, %zmm24, %zmm23

	// unroll 0
	vbroadcastsd	0+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vmovapd			192(%r11), %zmm26 // A
	vbroadcastsd	8+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vmovapd			192(%r11, %r12), %zmm28 // A
	vbroadcastsd	16+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm27, %zmm24, %zmm10
	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vmovapd			192(%r11, %r12, 2), %zmm30 {%k1}{z} // A
	vbroadcastsd	24+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm27, %zmm24, %zmm11
	vfmadd231pd		%zmm29, %zmm24, %zmm19
	vbroadcastsd	32+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm27, %zmm24, %zmm12
	vfmadd231pd		%zmm29, %zmm24, %zmm20
//	vbroadcastsd	40+128(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vfmadd231pd		%zmm27, %zmm24, %zmm13
//	vfmadd231pd		%zmm29, %zmm24, %zmm21
//	vbroadcastsd	48+128(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm27, %zmm24, %zmm14
//	vfmadd231pd		%zmm29, %zmm24, %zmm22
//	vbroadcastsd	56+128(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm27, %zmm24, %zmm15
//	vfmadd231pd		%zmm29, %zmm24, %zmm23
	addq	$ 256, %r11

	// unroll 0
	vbroadcastsd	0+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vfmadd231pd		%zmm28, %zmm24, %zmm8
	vfmadd231pd		%zmm30, %zmm24, %zmm16
//	vmovapd			0(%r11), %zmm25 // A
	vbroadcastsd	8+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vfmadd231pd		%zmm28, %zmm24, %zmm9
	vfmadd231pd		%zmm30, %zmm24, %zmm17
//	vmovapd			0(%r11, %r12), %zmm27 // A
	vbroadcastsd	16+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vfmadd231pd		%zmm28, %zmm24, %zmm10
	vfmadd231pd		%zmm30, %zmm24, %zmm18
//	vmovapd			0(%r11, %r12, 2), %zmm29 {%k1}{z} // A
	vbroadcastsd	24+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vfmadd231pd		%zmm28, %zmm24, %zmm11
	vfmadd231pd		%zmm30, %zmm24, %zmm19
	vbroadcastsd	32+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vfmadd231pd		%zmm28, %zmm24, %zmm12
	vfmadd231pd		%zmm30, %zmm24, %zmm20
//	vbroadcastsd	40+192(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vfmadd231pd		%zmm28, %zmm24, %zmm13
//	vfmadd231pd		%zmm30, %zmm24, %zmm21
//	vbroadcastsd	48+192(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vfmadd231pd		%zmm28, %zmm24, %zmm14
//	vfmadd231pd		%zmm30, %zmm24, %zmm22
//	vbroadcastsd	56+192(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7
//	vfmadd231pd		%zmm28, %zmm24, %zmm15
//	vfmadd231pd		%zmm30, %zmm24, %zmm23
	addq	$ 256, %r13

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovapd			0(%r11), %zmm25 // A
	vmovapd			0(%r11, %r12), %zmm27 // A
	vmovapd			0(%r11, %r12, 2), %zmm29 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm27, %zmm24, %zmm10
	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm27, %zmm24, %zmm11
	vfmadd231pd		%zmm29, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm27, %zmm24, %zmm12
	vfmadd231pd		%zmm29, %zmm24, %zmm20
//	vbroadcastsd	40(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vfmadd231pd		%zmm27, %zmm24, %zmm13
//	vfmadd231pd		%zmm29, %zmm24, %zmm21
//	vbroadcastsd	48(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm27, %zmm24, %zmm14
//	vfmadd231pd		%zmm29, %zmm24, %zmm22
//	vbroadcastsd	56(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm27, %zmm24, %zmm15
//	vfmadd231pd		%zmm29, %zmm24, %zmm23

	addq	$ 64, %r11
	addq	$ 64, %r13
	subl	$ 1, %r10d

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_3vx5_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- 4*sda*sizeof(double)
// r13   <- B
// zmm0  <- []
// ...
// zmm23  <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+8*k*sizeof(double)
// r12   <- 4*sda*sizeof(double)
// r13   <- B+8*k*sizeof(double)
// zmm0  <- []
// ...
// zmm23  <- []

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_3VX4_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_3vx4_lib8)
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	// prefetch

	// preload
	vmovapd 		0(%r11), %zmm25 // A0
	vmovapd 		0(%r11, %r12), %zmm27 // A1
	vmovapd 		0(%r11, %r12, 2), %zmm29 {%k1}{z} // A1

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vmovapd			64(%r11), %zmm26 // A
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vmovapd			64(%r11, %r12), %zmm28 // A
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm27, %zmm24, %zmm10
	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vmovapd			64(%r11, %r12, 2), %zmm30 {%k1}{z} // A
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm27, %zmm24, %zmm11
	vfmadd231pd		%zmm29, %zmm24, %zmm19
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	0+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vfmadd231pd		%zmm28, %zmm24, %zmm8
	vfmadd231pd		%zmm30, %zmm24, %zmm16
	vmovapd			128(%r11), %zmm25 // A
	vbroadcastsd	8+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vfmadd231pd		%zmm28, %zmm24, %zmm9
	vfmadd231pd		%zmm30, %zmm24, %zmm17
	vmovapd			128(%r11, %r12), %zmm27 // A
	vbroadcastsd	16+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vfmadd231pd		%zmm28, %zmm24, %zmm10
	vfmadd231pd		%zmm30, %zmm24, %zmm18
	vmovapd			128(%r11, %r12, 2), %zmm29 {%k1}{z} // A
	vbroadcastsd	24+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vfmadd231pd		%zmm28, %zmm24, %zmm11
	vfmadd231pd		%zmm30, %zmm24, %zmm19

	// unroll 0
	vbroadcastsd	0+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vmovapd			192(%r11), %zmm26 // A
	vbroadcastsd	8+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vmovapd			192(%r11, %r12), %zmm28 // A
	vbroadcastsd	16+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm27, %zmm24, %zmm10
	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vmovapd			192(%r11, %r12, 2), %zmm30 {%k1}{z} // A
	vbroadcastsd	24+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm27, %zmm24, %zmm11
	vfmadd231pd		%zmm29, %zmm24, %zmm19
	addq	$ 256, %r11

	// unroll 0
	vbroadcastsd	0+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vfmadd231pd		%zmm28, %zmm24, %zmm8
	vfmadd231pd		%zmm30, %zmm24, %zmm16
	vmovapd			0(%r11), %zmm25 // A
	vbroadcastsd	8+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vfmadd231pd		%zmm28, %zmm24, %zmm9
	vfmadd231pd		%zmm30, %zmm24, %zmm17
	vmovapd			0(%r11, %r12), %zmm27 // A
	vbroadcastsd	16+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vfmadd231pd		%zmm28, %zmm24, %zmm10
	vfmadd231pd		%zmm30, %zmm24, %zmm18
	vmovapd			0(%r11, %r12, 2), %zmm29 {%k1}{z} // A
	vbroadcastsd	24+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vfmadd231pd		%zmm28, %zmm24, %zmm11
	vfmadd231pd		%zmm30, %zmm24, %zmm19
	addq	$ 256, %r13

	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vmovapd			64(%r11), %zmm26 // A
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vmovapd			64(%r11, %r12), %zmm28 // A
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm27, %zmm24, %zmm10
	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vmovapd			64(%r11, %r12, 2), %zmm30 {%k1}{z} // A
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm27, %zmm24, %zmm11
	vfmadd231pd		%zmm29, %zmm24, %zmm19
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	0+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vfmadd231pd		%zmm28, %zmm24, %zmm8
	vfmadd231pd		%zmm30, %zmm24, %zmm16
	vmovapd			128(%r11), %zmm25 // A
	vbroadcastsd	8+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vfmadd231pd		%zmm28, %zmm24, %zmm9
	vfmadd231pd		%zmm30, %zmm24, %zmm17
	vmovapd			128(%r11, %r12), %zmm27 // A
	vbroadcastsd	16+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vfmadd231pd		%zmm28, %zmm24, %zmm10
	vfmadd231pd		%zmm30, %zmm24, %zmm18
	vmovapd			128(%r11, %r12, 2), %zmm29 {%k1}{z} // A
	vbroadcastsd	24+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vfmadd231pd		%zmm28, %zmm24, %zmm11
	vfmadd231pd		%zmm30, %zmm24, %zmm19

	// unroll 0
	vbroadcastsd	0+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vmovapd			192(%r11), %zmm26 // A
	vbroadcastsd	8+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vmovapd			192(%r11, %r12), %zmm28 // A
	vbroadcastsd	16+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm27, %zmm24, %zmm10
	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vmovapd			192(%r11, %r12, 2), %zmm30 {%k1}{z} // A
	vbroadcastsd	24+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm27, %zmm24, %zmm11
	vfmadd231pd		%zmm29, %zmm24, %zmm19
	addq	$ 256, %r11

	// unroll 0
	vbroadcastsd	0+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vfmadd231pd		%zmm28, %zmm24, %zmm8
	vfmadd231pd		%zmm30, %zmm24, %zmm16
//	vmovapd			0(%r11), %zmm25 // A
	vbroadcastsd	8+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vfmadd231pd		%zmm28, %zmm24, %zmm9
	vfmadd231pd		%zmm30, %zmm24, %zmm17
//	vmovapd			0(%r11, %r12), %zmm27 // A
	vbroadcastsd	16+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vfmadd231pd		%zmm28, %zmm24, %zmm10
	vfmadd231pd		%zmm30, %zmm24, %zmm18
//	vmovapd			0(%r11, %r12, 2), %zmm29 {%k1}{z} // A
	vbroadcastsd	24+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vfmadd231pd		%zmm28, %zmm24, %zmm11
	vfmadd231pd		%zmm30, %zmm24, %zmm19
	addq	$ 256, %r13

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovapd			0(%r11), %zmm25 // A
	vmovapd			0(%r11, %r12), %zmm27 // A
	vmovapd			0(%r11, %r12, 2), %zmm29 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm27, %zmm24, %zmm10
	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm27, %zmm24, %zmm11
	vfmadd231pd		%zmm29, %zmm24, %zmm19

	addq	$ 64, %r11
	addq	$ 64, %r13
	subl	$ 1, %r10d

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_3vx4_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- 4*sda*sizeof(double)
// r13   <- B
// zmm0  <- []
// ...
// zmm23  <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+8*k*sizeof(double)
// r12   <- 4*sda*sizeof(double)
// r13   <- B+8*k*sizeof(double)
// zmm0  <- []
// ...
// zmm23  <- []

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_3VX3_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_3vx3_lib8)
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	// prefetch

	// preload
	vmovapd 		0(%r11), %zmm25 // A0
	vmovapd 		0(%r11, %r12), %zmm27 // A1
	vmovapd 		0(%r11, %r12, 2), %zmm29 {%k1}{z} // A1

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vmovapd			64(%r11), %zmm26 // A
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vmovapd			64(%r11, %r12), %zmm28 // A
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm27, %zmm24, %zmm10
	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vmovapd			64(%r11, %r12, 2), %zmm30 {%k1}{z} // A
//	vbroadcastsd	24(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm27, %zmm24, %zmm11
//	vfmadd231pd		%zmm29, %zmm24, %zmm19
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	0+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vfmadd231pd		%zmm28, %zmm24, %zmm8
	vfmadd231pd		%zmm30, %zmm24, %zmm16
	vmovapd			128(%r11), %zmm25 // A
	vbroadcastsd	8+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vfmadd231pd		%zmm28, %zmm24, %zmm9
	vfmadd231pd		%zmm30, %zmm24, %zmm17
	vmovapd			128(%r11, %r12), %zmm27 // A
	vbroadcastsd	16+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vfmadd231pd		%zmm28, %zmm24, %zmm10
	vfmadd231pd		%zmm30, %zmm24, %zmm18
	vmovapd			128(%r11, %r12, 2), %zmm29 {%k1}{z} // A
//	vbroadcastsd	24+64(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm3
//	vfmadd231pd		%zmm28, %zmm24, %zmm11
//	vfmadd231pd		%zmm30, %zmm24, %zmm19

	// unroll 0
	vbroadcastsd	0+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vmovapd			192(%r11), %zmm26 // A
	vbroadcastsd	8+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vmovapd			192(%r11, %r12), %zmm28 // A
	vbroadcastsd	16+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm27, %zmm24, %zmm10
	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vmovapd			192(%r11, %r12, 2), %zmm30 {%k1}{z} // A
//	vbroadcastsd	24+128(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm27, %zmm24, %zmm11
//	vfmadd231pd		%zmm29, %zmm24, %zmm19
	addq	$ 256, %r11

	// unroll 0
	vbroadcastsd	0+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vfmadd231pd		%zmm28, %zmm24, %zmm8
	vfmadd231pd		%zmm30, %zmm24, %zmm16
	vmovapd			0(%r11), %zmm25 // A
	vbroadcastsd	8+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vfmadd231pd		%zmm28, %zmm24, %zmm9
	vfmadd231pd		%zmm30, %zmm24, %zmm17
	vmovapd			0(%r11, %r12), %zmm27 // A
	vbroadcastsd	16+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vfmadd231pd		%zmm28, %zmm24, %zmm10
	vfmadd231pd		%zmm30, %zmm24, %zmm18
	vmovapd			0(%r11, %r12, 2), %zmm29 {%k1}{z} // A
//	vbroadcastsd	24+192(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm3
//	vfmadd231pd		%zmm28, %zmm24, %zmm11
//	vfmadd231pd		%zmm30, %zmm24, %zmm19
	addq	$ 256, %r13

	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vmovapd			64(%r11), %zmm26 // A
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vmovapd			64(%r11, %r12), %zmm28 // A
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm27, %zmm24, %zmm10
	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vmovapd			64(%r11, %r12, 2), %zmm30 {%k1}{z} // A
//	vbroadcastsd	24(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm27, %zmm24, %zmm11
//	vfmadd231pd		%zmm29, %zmm24, %zmm19
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	0+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vfmadd231pd		%zmm28, %zmm24, %zmm8
	vfmadd231pd		%zmm30, %zmm24, %zmm16
	vmovapd			128(%r11), %zmm25 // A
	vbroadcastsd	8+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vfmadd231pd		%zmm28, %zmm24, %zmm9
	vfmadd231pd		%zmm30, %zmm24, %zmm17
	vmovapd			128(%r11, %r12), %zmm27 // A
	vbroadcastsd	16+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vfmadd231pd		%zmm28, %zmm24, %zmm10
	vfmadd231pd		%zmm30, %zmm24, %zmm18
	vmovapd			128(%r11, %r12, 2), %zmm29 {%k1}{z} // A
//	vbroadcastsd	24+64(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm3
//	vfmadd231pd		%zmm28, %zmm24, %zmm11
//	vfmadd231pd		%zmm30, %zmm24, %zmm19

	// unroll 0
	vbroadcastsd	0+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vmovapd			192(%r11), %zmm26 // A
	vbroadcastsd	8+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vmovapd			192(%r11, %r12), %zmm28 // A
	vbroadcastsd	16+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm27, %zmm24, %zmm10
	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vmovapd			192(%r11, %r12, 2), %zmm30 {%k1}{z} // A
//	vbroadcastsd	24+128(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm27, %zmm24, %zmm11
//	vfmadd231pd		%zmm29, %zmm24, %zmm19
	addq	$ 256, %r11

	// unroll 0
	vbroadcastsd	0+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vfmadd231pd		%zmm28, %zmm24, %zmm8
	vfmadd231pd		%zmm30, %zmm24, %zmm16
//	vmovapd			0(%r11), %zmm25 // A
	vbroadcastsd	8+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vfmadd231pd		%zmm28, %zmm24, %zmm9
	vfmadd231pd		%zmm30, %zmm24, %zmm17
//	vmovapd			0(%r11, %r12), %zmm27 // A
	vbroadcastsd	16+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vfmadd231pd		%zmm28, %zmm24, %zmm10
	vfmadd231pd		%zmm30, %zmm24, %zmm18
//	vmovapd			0(%r11, %r12, 2), %zmm29 {%k1}{z} // A
//	vbroadcastsd	24+192(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm3
//	vfmadd231pd		%zmm28, %zmm24, %zmm11
//	vfmadd231pd		%zmm30, %zmm24, %zmm19
	addq	$ 256, %r13

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovapd			0(%r11), %zmm25 // A
	vmovapd			0(%r11, %r12), %zmm27 // A
	vmovapd			0(%r11, %r12, 2), %zmm29 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm27, %zmm24, %zmm10
	vfmadd231pd		%zmm29, %zmm24, %zmm18
//	vbroadcastsd	24(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm27, %zmm24, %zmm11
//	vfmadd231pd		%zmm29, %zmm24, %zmm19

	addq	$ 64, %r11
	addq	$ 64, %r13
	subl	$ 1, %r10d

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_3vx3_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- 4*sda*sizeof(double)
// r13   <- B
// zmm0  <- []
// ...
// zmm23  <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+8*k*sizeof(double)
// r12   <- 4*sda*sizeof(double)
// r13   <- B+8*k*sizeof(double)
// zmm0  <- []
// ...
// zmm23  <- []

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_3VX2_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_3vx2_lib8)
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	// prefetch

	// preload
	vmovapd 		0(%r11), %zmm25 // A0
	vmovapd 		0(%r11, %r12), %zmm27 // A1
	vmovapd 		0(%r11, %r12, 2), %zmm29 {%k1}{z} // A1

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vmovapd			64(%r11), %zmm26 // A
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vmovapd			64(%r11, %r12), %zmm28 // A
//	vbroadcastsd	16(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm27, %zmm24, %zmm10
//	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vmovapd			64(%r11, %r12, 2), %zmm30 {%k1}{z} // A
//	vbroadcastsd	24(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm27, %zmm24, %zmm11
//	vfmadd231pd		%zmm29, %zmm24, %zmm19
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	0+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vfmadd231pd		%zmm28, %zmm24, %zmm8
	vfmadd231pd		%zmm30, %zmm24, %zmm16
	vmovapd			128(%r11), %zmm25 // A
	vbroadcastsd	8+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vfmadd231pd		%zmm28, %zmm24, %zmm9
	vfmadd231pd		%zmm30, %zmm24, %zmm17
	vmovapd			128(%r11, %r12), %zmm27 // A
//	vbroadcastsd	16+64(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm2
//	vfmadd231pd		%zmm28, %zmm24, %zmm10
//	vfmadd231pd		%zmm30, %zmm24, %zmm18
	vmovapd			128(%r11, %r12, 2), %zmm29 {%k1}{z} // A
//	vbroadcastsd	24+64(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm3
//	vfmadd231pd		%zmm28, %zmm24, %zmm11
//	vfmadd231pd		%zmm30, %zmm24, %zmm19

	// unroll 0
	vbroadcastsd	0+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vmovapd			192(%r11), %zmm26 // A
	vbroadcastsd	8+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vmovapd			192(%r11, %r12), %zmm28 // A
//	vbroadcastsd	16+128(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm27, %zmm24, %zmm10
//	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vmovapd			192(%r11, %r12, 2), %zmm30 {%k1}{z} // A
//	vbroadcastsd	24+128(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm27, %zmm24, %zmm11
//	vfmadd231pd		%zmm29, %zmm24, %zmm19
	addq	$ 256, %r11

	// unroll 0
	vbroadcastsd	0+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vfmadd231pd		%zmm28, %zmm24, %zmm8
	vfmadd231pd		%zmm30, %zmm24, %zmm16
	vmovapd			0(%r11), %zmm25 // A
	vbroadcastsd	8+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vfmadd231pd		%zmm28, %zmm24, %zmm9
	vfmadd231pd		%zmm30, %zmm24, %zmm17
	vmovapd			0(%r11, %r12), %zmm27 // A
//	vbroadcastsd	16+192(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm2
//	vfmadd231pd		%zmm28, %zmm24, %zmm10
//	vfmadd231pd		%zmm30, %zmm24, %zmm18
	vmovapd			0(%r11, %r12, 2), %zmm29 {%k1}{z} // A
//	vbroadcastsd	24+192(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm3
//	vfmadd231pd		%zmm28, %zmm24, %zmm11
//	vfmadd231pd		%zmm30, %zmm24, %zmm19
	addq	$ 256, %r13

	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vmovapd			64(%r11), %zmm26 // A
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vmovapd			64(%r11, %r12), %zmm28 // A
//	vbroadcastsd	16(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm27, %zmm24, %zmm10
//	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vmovapd			64(%r11, %r12, 2), %zmm30 {%k1}{z} // A
//	vbroadcastsd	24(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm27, %zmm24, %zmm11
//	vfmadd231pd		%zmm29, %zmm24, %zmm19
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	0+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vfmadd231pd		%zmm28, %zmm24, %zmm8
	vfmadd231pd		%zmm30, %zmm24, %zmm16
	vmovapd			128(%r11), %zmm25 // A
	vbroadcastsd	8+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vfmadd231pd		%zmm28, %zmm24, %zmm9
	vfmadd231pd		%zmm30, %zmm24, %zmm17
	vmovapd			128(%r11, %r12), %zmm27 // A
//	vbroadcastsd	16+64(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm2
//	vfmadd231pd		%zmm28, %zmm24, %zmm10
//	vfmadd231pd		%zmm30, %zmm24, %zmm18
	vmovapd			128(%r11, %r12, 2), %zmm29 {%k1}{z} // A
//	vbroadcastsd	24+64(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm3
//	vfmadd231pd		%zmm28, %zmm24, %zmm11
//	vfmadd231pd		%zmm30, %zmm24, %zmm19

	// unroll 0
	vbroadcastsd	0+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vmovapd			192(%r11), %zmm26 // A
	vbroadcastsd	8+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vmovapd			192(%r11, %r12), %zmm28 // A
//	vbroadcastsd	16+128(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm27, %zmm24, %zmm10
//	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vmovapd			192(%r11, %r12, 2), %zmm30 {%k1}{z} // A
//	vbroadcastsd	24+128(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm27, %zmm24, %zmm11
//	vfmadd231pd		%zmm29, %zmm24, %zmm19
	addq	$ 256, %r11

	// unroll 0
	vbroadcastsd	0+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vfmadd231pd		%zmm28, %zmm24, %zmm8
	vfmadd231pd		%zmm30, %zmm24, %zmm16
//	vmovapd			0(%r11), %zmm25 // A
	vbroadcastsd	8+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vfmadd231pd		%zmm28, %zmm24, %zmm9
	vfmadd231pd		%zmm30, %zmm24, %zmm17
//	vmovapd			0(%r11, %r12), %zmm27 // A
//	vbroadcastsd	16+192(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm2
//	vfmadd231pd		%zmm28, %zmm24, %zmm10
//	vfmadd231pd		%zmm30, %zmm24, %zmm18
//	vmovapd			0(%r11, %r12, 2), %zmm29 {%k1}{z} // A
//	vbroadcastsd	24+192(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm3
//	vfmadd231pd		%zmm28, %zmm24, %zmm11
//	vfmadd231pd		%zmm30, %zmm24, %zmm19
	addq	$ 256, %r13

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovapd			0(%r11), %zmm25 // A
	vmovapd			0(%r11, %r12), %zmm27 // A
	vmovapd			0(%r11, %r12, 2), %zmm29 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm27, %zmm24, %zmm9
	vfmadd231pd		%zmm29, %zmm24, %zmm17
//	vbroadcastsd	16(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm27, %zmm24, %zmm10
//	vfmadd231pd		%zmm29, %zmm24, %zmm18
//	vbroadcastsd	24(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm27, %zmm24, %zmm11
//	vfmadd231pd		%zmm29, %zmm24, %zmm19

	addq	$ 64, %r11
	addq	$ 64, %r13
	subl	$ 1, %r10d

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_3vx2_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- 4*sda*sizeof(double)
// r13   <- B
// zmm0  <- []
// ...
// zmm23  <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+8*k*sizeof(double)
// r12   <- 4*sda*sizeof(double)
// r13   <- B+8*k*sizeof(double)
// zmm0  <- []
// ...
// zmm23  <- []

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_3VX1_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_3vx1_lib8)
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	// prefetch

	// preload
	vmovapd 		0(%r11), %zmm25 // A0
	vmovapd 		0(%r11, %r12), %zmm27 // A1
	vmovapd 		0(%r11, %r12, 2), %zmm29 {%k1}{z} // A1

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vmovapd			64(%r11), %zmm26 // A
//	vbroadcastsd	8(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vfmadd231pd		%zmm27, %zmm24, %zmm9
//	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vmovapd			64(%r11, %r12), %zmm28 // A
//	vbroadcastsd	16(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm27, %zmm24, %zmm10
//	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vmovapd			64(%r11, %r12, 2), %zmm30 {%k1}{z} // A
//	vbroadcastsd	24(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm27, %zmm24, %zmm11
//	vfmadd231pd		%zmm29, %zmm24, %zmm19
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	0+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vfmadd231pd		%zmm28, %zmm24, %zmm8
	vfmadd231pd		%zmm30, %zmm24, %zmm16
	vmovapd			128(%r11), %zmm25 // A
//	vbroadcastsd	8+64(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm1
//	vfmadd231pd		%zmm28, %zmm24, %zmm9
//	vfmadd231pd		%zmm30, %zmm24, %zmm17
	vmovapd			128(%r11, %r12), %zmm27 // A
//	vbroadcastsd	16+64(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm2
//	vfmadd231pd		%zmm28, %zmm24, %zmm10
//	vfmadd231pd		%zmm30, %zmm24, %zmm18
	vmovapd			128(%r11, %r12, 2), %zmm29 {%k1}{z} // A
//	vbroadcastsd	24+64(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm3
//	vfmadd231pd		%zmm28, %zmm24, %zmm11
//	vfmadd231pd		%zmm30, %zmm24, %zmm19

	// unroll 0
	vbroadcastsd	0+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vmovapd			192(%r11), %zmm26 // A
//	vbroadcastsd	8+128(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vfmadd231pd		%zmm27, %zmm24, %zmm9
//	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vmovapd			192(%r11, %r12), %zmm28 // A
//	vbroadcastsd	16+128(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm27, %zmm24, %zmm10
//	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vmovapd			192(%r11, %r12, 2), %zmm30 {%k1}{z} // A
//	vbroadcastsd	24+128(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm27, %zmm24, %zmm11
//	vfmadd231pd		%zmm29, %zmm24, %zmm19
	addq	$ 256, %r11

	// unroll 0
	vbroadcastsd	0+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vfmadd231pd		%zmm28, %zmm24, %zmm8
	vfmadd231pd		%zmm30, %zmm24, %zmm16
	vmovapd			0(%r11), %zmm25 // A
//	vbroadcastsd	8+192(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm1
//	vfmadd231pd		%zmm28, %zmm24, %zmm9
//	vfmadd231pd		%zmm30, %zmm24, %zmm17
	vmovapd			0(%r11, %r12), %zmm27 // A
//	vbroadcastsd	16+192(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm2
//	vfmadd231pd		%zmm28, %zmm24, %zmm10
//	vfmadd231pd		%zmm30, %zmm24, %zmm18
	vmovapd			0(%r11, %r12, 2), %zmm29 {%k1}{z} // A
//	vbroadcastsd	24+192(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm3
//	vfmadd231pd		%zmm28, %zmm24, %zmm11
//	vfmadd231pd		%zmm30, %zmm24, %zmm19
	addq	$ 256, %r13

	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vmovapd			64(%r11), %zmm26 // A
//	vbroadcastsd	8(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vfmadd231pd		%zmm27, %zmm24, %zmm9
//	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vmovapd			64(%r11, %r12), %zmm28 // A
//	vbroadcastsd	16(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm27, %zmm24, %zmm10
//	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vmovapd			64(%r11, %r12, 2), %zmm30 {%k1}{z} // A
//	vbroadcastsd	24(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm27, %zmm24, %zmm11
//	vfmadd231pd		%zmm29, %zmm24, %zmm19
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	0+64(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vfmadd231pd		%zmm28, %zmm24, %zmm8
	vfmadd231pd		%zmm30, %zmm24, %zmm16
	vmovapd			128(%r11), %zmm25 // A
//	vbroadcastsd	8+64(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm1
//	vfmadd231pd		%zmm28, %zmm24, %zmm9
//	vfmadd231pd		%zmm30, %zmm24, %zmm17
	vmovapd			128(%r11, %r12), %zmm27 // A
//	vbroadcastsd	16+64(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm2
//	vfmadd231pd		%zmm28, %zmm24, %zmm10
//	vfmadd231pd		%zmm30, %zmm24, %zmm18
	vmovapd			128(%r11, %r12, 2), %zmm29 {%k1}{z} // A
//	vbroadcastsd	24+64(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm3
//	vfmadd231pd		%zmm28, %zmm24, %zmm11
//	vfmadd231pd		%zmm30, %zmm24, %zmm19

	// unroll 0
	vbroadcastsd	0+128(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
	vmovapd			192(%r11), %zmm26 // A
//	vbroadcastsd	8+128(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vfmadd231pd		%zmm27, %zmm24, %zmm9
//	vfmadd231pd		%zmm29, %zmm24, %zmm17
	vmovapd			192(%r11, %r12), %zmm28 // A
//	vbroadcastsd	16+128(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm27, %zmm24, %zmm10
//	vfmadd231pd		%zmm29, %zmm24, %zmm18
	vmovapd			192(%r11, %r12, 2), %zmm30 {%k1}{z} // A
//	vbroadcastsd	24+128(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm27, %zmm24, %zmm11
//	vfmadd231pd		%zmm29, %zmm24, %zmm19
	addq	$ 256, %r11

	// unroll 0
	vbroadcastsd	0+192(%r13), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vfmadd231pd		%zmm28, %zmm24, %zmm8
	vfmadd231pd		%zmm30, %zmm24, %zmm16
//	vmovapd			0(%r11), %zmm25 // A
//	vbroadcastsd	8+192(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm1
//	vfmadd231pd		%zmm28, %zmm24, %zmm9
//	vfmadd231pd		%zmm30, %zmm24, %zmm17
//	vmovapd			0(%r11, %r12), %zmm27 // A
//	vbroadcastsd	16+192(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm2
//	vfmadd231pd		%zmm28, %zmm24, %zmm10
//	vfmadd231pd		%zmm30, %zmm24, %zmm18
//	vmovapd			0(%r11, %r12, 2), %zmm29 {%k1}{z} // A
//	vbroadcastsd	24+192(%r13), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm3
//	vfmadd231pd		%zmm28, %zmm24, %zmm11
//	vfmadd231pd		%zmm30, %zmm24, %zmm19
	addq	$ 256, %r13

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovapd			0(%r11), %zmm25 // A
	vmovapd			0(%r11, %r12), %zmm27 // A
	vmovapd			0(%r11, %r12, 2), %zmm29 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm27, %zmm24, %zmm8
	vfmadd231pd		%zmm29, %zmm24, %zmm16
//	vbroadcastsd	8(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vfmadd231pd		%zmm27, %zmm24, %zmm9
//	vfmadd231pd		%zmm29, %zmm24, %zmm17
//	vbroadcastsd	16(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm27, %zmm24, %zmm10
//	vfmadd231pd		%zmm29, %zmm24, %zmm18
//	vbroadcastsd	24(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm27, %zmm24, %zmm11
//	vfmadd231pd		%zmm29, %zmm24, %zmm19

	addq	$ 64, %r11
	addq	$ 64, %r13
	subl	$ 1, %r10d

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_3vx1_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- 4*sda*sizeof(double)
// r13   <- B
// r14   <- m1
// r15   <- n1
// zmm0  <- []
// ...
// zmm7  <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+8*k*sizeof(double)
// r13   <- B+8*k*sizeof(double)
// r14   <- m1
// r15   <- n1
// zmm0  <- []
// ...
// zmm7  <- []

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_24X8_VS_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_24x8_vs_lib8)
#endif

	// compute mask for rows
	vcvtsi2sd	%r14d, %xmm25, %xmm25
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC02(%rip), %zmm24
#elif defined(OS_MAC)
	vmovupd		LC02(%rip), %zmm24
#endif
	vbroadcastsd	%xmm25, %zmm25
	vsubpd		%zmm25, %zmm24, %zmm25
	vpmovq2m	%zmm25, %k1

	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_3VX1_LIB8
#else
	CALL(inner_kernel_dgemm_nt_3vx1_lib8)
#endif
	
	jmp		107f

100:

	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_3VX2_LIB8
#else
	CALL(inner_kernel_dgemm_nt_3vx2_lib8)
#endif
	
	jmp		107f

101:

	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_3VX3_LIB8
#else
	CALL(inner_kernel_dgemm_nt_3vx3_lib8)
#endif
	
	jmp		107f

102:

	cmpl	$ 4, %r15d
	jg		103f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_3VX4_LIB8
#else
	CALL(inner_kernel_dgemm_nt_3vx4_lib8)
#endif
	
	jmp		107f

103:

	cmpl	$ 5, %r15d
	jg		104f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_3VX5_LIB8
#else
	CALL(inner_kernel_dgemm_nt_3vx5_lib8)
#endif
	
	jmp		107f

104:

	cmpl	$ 6, %r15d
	jg		105f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_3VX6_LIB8
#else
	CALL(inner_kernel_dgemm_nt_3vx6_lib8)
#endif
	
	jmp		107f

105:

	cmpl	$ 7, %r15d
	jg		106f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_3VX7_LIB8
#else
	CALL(inner_kernel_dgemm_nt_3vx7_lib8)
#endif
	
	jmp		107f

106:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_3VX8_LIB8
#else
	CALL(inner_kernel_dgemm_nt_3vx8_lib8)
#endif

107:

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_24x8_vs_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- B
// r12   <- C
// r13   <- 64*sdc
// zmm0  <- [a00 ... a70]
// ...
// zmm7  <- [a07 ... a77]
// zmm8  <- [a87 ... af7]
// ...
// zmm15 <- [a87 ... af7]
// zmm16 <- [ag7 ... an7]
// ...
// zmm23 <- [ag7 ... an7]

//
// output arguments:
// r10d  <- 0
// r11   <- ?
// r12   <- ?
// r13   <- 64*sdc
// zmm0  <- [a00 ... a70]
// ...
// zmm7  <- [a07 ... a77]
// zmm8  <- [a87 ... af7]
// ...
// zmm15 <- [a87 ... af7]
// zmm16 <- [ag7 ... an7]
// ...
// zmm23 <- [ag7 ... an7]

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEBP_NN_24X8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgebp_nn_24x8_lib8)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return

	cmpl	$ 3, %r10d
	jle		2f // cleanup loop

	// main loop
	.p2align 
1:
	// merged k-iter 0-3 to have more independent acc
	// 0-1
	vmovapd			0*64(%r12), %zmm28
	vmovapd			0*64(%r12, %r13), %zmm24
	vmovapd			0*64(%r12, %r13, 2), %zmm26
	vbroadcastsd	0+0*64(%r11), %zmm31
	vfmadd231pd		%zmm0, %zmm31, %zmm28
	vfmadd231pd		%zmm8, %zmm31, %zmm24
	vfmadd231pd		%zmm16, %zmm31, %zmm26
	vmovapd			1*64(%r12), %zmm29
	vmovapd			1*64(%r12, %r13), %zmm25
	vmovapd			1*64(%r12, %r13, 2), %zmm27
	vbroadcastsd	0+1*64(%r11), %zmm31
	vfmadd231pd		%zmm0, %zmm31, %zmm29
	vfmadd231pd		%zmm8, %zmm31, %zmm25
	vfmadd231pd		%zmm16, %zmm31, %zmm27

	vbroadcastsd	8+0*64(%r11), %zmm31
	vfmadd231pd		%zmm1, %zmm31, %zmm28
	vfmadd231pd		%zmm9, %zmm31, %zmm24
	vfmadd231pd		%zmm17, %zmm31, %zmm26
	vbroadcastsd	8+1*64(%r11), %zmm31
	vfmadd231pd		%zmm1, %zmm31, %zmm29
	vfmadd231pd		%zmm9, %zmm31, %zmm25
	vfmadd231pd		%zmm17, %zmm31, %zmm27

	vbroadcastsd	16+0*64(%r11), %zmm31
	vfmadd231pd		%zmm2, %zmm31, %zmm28
	vfmadd231pd		%zmm10, %zmm31, %zmm24
	vfmadd231pd		%zmm18, %zmm31, %zmm26
	vbroadcastsd	16+1*64(%r11), %zmm31
	vfmadd231pd		%zmm2, %zmm31, %zmm29
	vfmadd231pd		%zmm10, %zmm31, %zmm25
	vfmadd231pd		%zmm18, %zmm31, %zmm27

	vbroadcastsd	24+0*64(%r11), %zmm31
	vfmadd231pd		%zmm3, %zmm31, %zmm28
	vfmadd231pd		%zmm11, %zmm31, %zmm24
	vfmadd231pd		%zmm19, %zmm31, %zmm26
	vbroadcastsd	24+1*64(%r11), %zmm31
	vfmadd231pd		%zmm3, %zmm31, %zmm29
	vfmadd231pd		%zmm11, %zmm31, %zmm25
	vfmadd231pd		%zmm19, %zmm31, %zmm27

	vbroadcastsd	32+0*64(%r11), %zmm31
	vfmadd231pd		%zmm4, %zmm31, %zmm28
	vfmadd231pd		%zmm12, %zmm31, %zmm24
	vfmadd231pd		%zmm20, %zmm31, %zmm26
	vbroadcastsd	32+1*64(%r11), %zmm31
	vfmadd231pd		%zmm4, %zmm31, %zmm29
	vfmadd231pd		%zmm12, %zmm31, %zmm25
	vfmadd231pd		%zmm20, %zmm31, %zmm27

	vbroadcastsd	40+0*64(%r11), %zmm31
	vfmadd231pd		%zmm5, %zmm31, %zmm28
	vfmadd231pd		%zmm13, %zmm31, %zmm24
	vfmadd231pd		%zmm21, %zmm31, %zmm26
	vbroadcastsd	40+1*64(%r11), %zmm31
	vfmadd231pd		%zmm5, %zmm31, %zmm29
	vfmadd231pd		%zmm13, %zmm31, %zmm25
	vfmadd231pd		%zmm21, %zmm31, %zmm27

	vbroadcastsd	48+0*64(%r11), %zmm31
	vfmadd231pd		%zmm6, %zmm31, %zmm28
	vfmadd231pd		%zmm14, %zmm31, %zmm24
	vfmadd231pd		%zmm22, %zmm31, %zmm26
	vbroadcastsd	48+1*64(%r11), %zmm31
	vfmadd231pd		%zmm6, %zmm31, %zmm29
	vfmadd231pd		%zmm14, %zmm31, %zmm25
	vfmadd231pd		%zmm22, %zmm31, %zmm27

	vbroadcastsd	56+0*64(%r11), %zmm31
	vfmadd231pd		%zmm7, %zmm31, %zmm28
	vfmadd231pd		%zmm15, %zmm31, %zmm24
	vfmadd231pd		%zmm23, %zmm31, %zmm26
	vmovapd			%zmm28, 0*64(%r12)
	vmovapd			%zmm24, 0*64(%r12, %r13)
	vmovapd			%zmm26, 0*64(%r12, %r13, 2)
	vbroadcastsd	56+1*64(%r11), %zmm31
	vfmadd231pd		%zmm7, %zmm31, %zmm29
	vfmadd231pd		%zmm15, %zmm31, %zmm25
	vfmadd231pd		%zmm23, %zmm31, %zmm27
	vmovapd			%zmm29, 1*64(%r12)
	vmovapd			%zmm25, 1*64(%r12, %r13)
	vmovapd			%zmm27, 1*64(%r12, %r13, 2)


	// 2-3
	vmovapd			2*64(%r12), %zmm28
	vmovapd			2*64(%r12, %r13), %zmm24
	vmovapd			2*64(%r12, %r13, 2), %zmm26
	vbroadcastsd	0+2*64(%r11), %zmm31
	vfmadd231pd		%zmm0, %zmm31, %zmm28
	vfmadd231pd		%zmm8, %zmm31, %zmm24
	vfmadd231pd		%zmm16, %zmm31, %zmm26
	vmovapd			3*64(%r12), %zmm29
	vmovapd			3*64(%r12, %r13), %zmm25
	vmovapd			3*64(%r12, %r13, 2), %zmm27
	vbroadcastsd	0+3*64(%r11), %zmm31
	vfmadd231pd		%zmm0, %zmm31, %zmm29
	vfmadd231pd		%zmm8, %zmm31, %zmm25
	vfmadd231pd		%zmm16, %zmm31, %zmm27

	vbroadcastsd	8+2*64(%r11), %zmm31
	vfmadd231pd		%zmm1, %zmm31, %zmm28
	vfmadd231pd		%zmm9, %zmm31, %zmm24
	vfmadd231pd		%zmm17, %zmm31, %zmm26
	vbroadcastsd	8+3*64(%r11), %zmm31
	vfmadd231pd		%zmm1, %zmm31, %zmm29
	vfmadd231pd		%zmm9, %zmm31, %zmm25
	vfmadd231pd		%zmm17, %zmm31, %zmm27

	vbroadcastsd	16+2*64(%r11), %zmm31
	vfmadd231pd		%zmm2, %zmm31, %zmm28
	vfmadd231pd		%zmm10, %zmm31, %zmm24
	vfmadd231pd		%zmm18, %zmm31, %zmm26
	vbroadcastsd	16+3*64(%r11), %zmm31
	vfmadd231pd		%zmm2, %zmm31, %zmm29
	vfmadd231pd		%zmm10, %zmm31, %zmm25
	vfmadd231pd		%zmm18, %zmm31, %zmm27

	vbroadcastsd	24+2*64(%r11), %zmm31
	vfmadd231pd		%zmm3, %zmm31, %zmm28
	vfmadd231pd		%zmm11, %zmm31, %zmm24
	vfmadd231pd		%zmm19, %zmm31, %zmm26
	vbroadcastsd	24+3*64(%r11), %zmm31
	vfmadd231pd		%zmm3, %zmm31, %zmm29
	vfmadd231pd		%zmm11, %zmm31, %zmm25
	vfmadd231pd		%zmm19, %zmm31, %zmm27

	vbroadcastsd	32+2*64(%r11), %zmm31
	vfmadd231pd		%zmm4, %zmm31, %zmm28
	vfmadd231pd		%zmm12, %zmm31, %zmm24
	vfmadd231pd		%zmm20, %zmm31, %zmm26
	vbroadcastsd	32+3*64(%r11), %zmm31
	vfmadd231pd		%zmm4, %zmm31, %zmm29
	vfmadd231pd		%zmm12, %zmm31, %zmm25
	vfmadd231pd		%zmm20, %zmm31, %zmm27

	vbroadcastsd	40+2*64(%r11), %zmm31
	vfmadd231pd		%zmm5, %zmm31, %zmm28
	vfmadd231pd		%zmm13, %zmm31, %zmm24
	vfmadd231pd		%zmm21, %zmm31, %zmm26
	vbroadcastsd	40+3*64(%r11), %zmm31
	vfmadd231pd		%zmm5, %zmm31, %zmm29
	vfmadd231pd		%zmm13, %zmm31, %zmm25
	vfmadd231pd		%zmm21, %zmm31, %zmm27

	vbroadcastsd	48+2*64(%r11), %zmm31
	vfmadd231pd		%zmm6, %zmm31, %zmm28
	vfmadd231pd		%zmm14, %zmm31, %zmm24
	vfmadd231pd		%zmm22, %zmm31, %zmm26
	vbroadcastsd	48+3*64(%r11), %zmm31
	vfmadd231pd		%zmm6, %zmm31, %zmm29
	vfmadd231pd		%zmm14, %zmm31, %zmm25
	vfmadd231pd		%zmm22, %zmm31, %zmm27

	vbroadcastsd	56+2*64(%r11), %zmm31
	vfmadd231pd		%zmm7, %zmm31, %zmm28
	vfmadd231pd		%zmm15, %zmm31, %zmm24
	vfmadd231pd		%zmm23, %zmm31, %zmm26
	vmovapd			%zmm28, 2*64(%r12)
	vmovapd			%zmm24, 2*64(%r12, %r13)
	vmovapd			%zmm26, 2*64(%r12, %r13, 2)
	vbroadcastsd	56+3*64(%r11), %zmm31
	vfmadd231pd		%zmm7, %zmm31, %zmm29
	vfmadd231pd		%zmm15, %zmm31, %zmm25
	vfmadd231pd		%zmm23, %zmm31, %zmm27
	vmovapd			%zmm29, 3*64(%r12)
	vmovapd			%zmm25, 3*64(%r12, %r13)
	vmovapd			%zmm27, 3*64(%r12, %r13, 2)

	addq	$ 256, %r11
	addq	$ 256, %r12
	subl	$ 4, %r10d

	cmpl	$ 3, %r10d
	jg		1b // main loop

	cmpl	$ 0, %r10d
	jle		0f // return

	// cleanup loop
2:
	vmovapd			0*64(%r12), %zmm28
	vmovapd			0*64(%r12, %r13), %zmm24
	vmovapd			0*64(%r12, %r13, 2), %zmm26
	vbroadcastsd	0+0*64(%r11), %zmm23
	vfmadd231pd		%zmm0, %zmm23, %zmm28
	vfmadd231pd		%zmm8, %zmm23, %zmm24
	vfmadd231pd		%zmm16, %zmm23, %zmm26
	vbroadcastsd	8+0*64(%r11), %zmm23
	vfmadd231pd		%zmm1, %zmm23, %zmm28
	vfmadd231pd		%zmm9, %zmm23, %zmm24
	vfmadd231pd		%zmm17, %zmm23, %zmm26
	vbroadcastsd	16+0*64(%r11), %zmm23
	vfmadd231pd		%zmm2, %zmm23, %zmm28
	vfmadd231pd		%zmm10, %zmm23, %zmm24
	vfmadd231pd		%zmm18, %zmm23, %zmm26
	vbroadcastsd	24+0*64(%r11), %zmm23
	vfmadd231pd		%zmm3, %zmm23, %zmm28
	vfmadd231pd		%zmm11, %zmm23, %zmm24
	vfmadd231pd		%zmm19, %zmm23, %zmm26
	vbroadcastsd	32+0*64(%r11), %zmm23
	vfmadd231pd		%zmm4, %zmm23, %zmm28
	vfmadd231pd		%zmm12, %zmm23, %zmm24
	vfmadd231pd		%zmm20, %zmm23, %zmm26
	vbroadcastsd	40+0*64(%r11), %zmm23
	vfmadd231pd		%zmm5, %zmm23, %zmm28
	vfmadd231pd		%zmm13, %zmm23, %zmm24
	vfmadd231pd		%zmm21, %zmm23, %zmm26
	vbroadcastsd	48+0*64(%r11), %zmm23
	vfmadd231pd		%zmm6, %zmm23, %zmm28
	vfmadd231pd		%zmm14, %zmm23, %zmm24
	vfmadd231pd		%zmm22, %zmm23, %zmm26
	vbroadcastsd	56+0*64(%r11), %zmm23
	vfmadd231pd		%zmm7, %zmm23, %zmm28
	vfmadd231pd		%zmm15, %zmm23, %zmm24
	vfmadd231pd		%zmm23, %zmm23, %zmm26
	vmovapd			%zmm28, 0*64(%r12)
	vmovapd			%zmm24, 0*64(%r12, %r13)
	vmovapd			%zmm26, 0*64(%r12, %r13, 2)

	subl	$ 1, %r10d
	addq	$ 64, %r11
	addq	$ 64, %r12

	cmpl	$ 0, %r10d
	jg		2b // main loop

	// return
0:

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgebp_nn_24x8_lib8)
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
// r10  <- E
// r11  <- inv_diag_E
// zmm0 <- []
// ...
// zmm23 <- []
//
// output arguments:
// r10  <- E
// r11  <- inv_diag_E
// zmm0 <- []
// ...
// zmm23 <- []

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RLT_INV_24X8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rlt_inv_24x8_lib8)
#endif
	
	vbroadcastsd	0(%r11), %zmm24
	vmulpd			%zmm0, %zmm24, %zmm0
	vmulpd			%zmm8, %zmm24, %zmm8
	vmulpd			%zmm16, %zmm24, %zmm16
	vbroadcastsd	8+0*64(%r10), %zmm24
	vfnmadd231pd	%zmm0, %zmm24, %zmm1
	vfnmadd231pd	%zmm8, %zmm24, %zmm9
	vfnmadd231pd	%zmm16, %zmm24, %zmm17
	vbroadcastsd	16+0*64(%r10), %zmm24
	vfnmadd231pd	%zmm0, %zmm24, %zmm2
	vfnmadd231pd	%zmm8, %zmm24, %zmm10
	vfnmadd231pd	%zmm16, %zmm24, %zmm18
	vbroadcastsd	24+0*64(%r10), %zmm24
	vfnmadd231pd	%zmm0, %zmm24, %zmm3
	vfnmadd231pd	%zmm8, %zmm24, %zmm11
	vfnmadd231pd	%zmm16, %zmm24, %zmm19
	vbroadcastsd	32+0*64(%r10), %zmm24
	vfnmadd231pd	%zmm0, %zmm24, %zmm4
	vfnmadd231pd	%zmm8, %zmm24, %zmm12
	vfnmadd231pd	%zmm16, %zmm24, %zmm20
	vbroadcastsd	40+0*64(%r10), %zmm24
	vfnmadd231pd	%zmm0, %zmm24, %zmm5
	vfnmadd231pd	%zmm8, %zmm24, %zmm13
	vfnmadd231pd	%zmm16, %zmm24, %zmm21
	vbroadcastsd	48+0*64(%r10), %zmm24
	vfnmadd231pd	%zmm0, %zmm24, %zmm6
	vfnmadd231pd	%zmm8, %zmm24, %zmm14
	vfnmadd231pd	%zmm16, %zmm24, %zmm22
	vbroadcastsd	56+0*64(%r10), %zmm24
	vfnmadd231pd	%zmm0, %zmm24, %zmm7
	vfnmadd231pd	%zmm8, %zmm24, %zmm15
	vfnmadd231pd	%zmm16, %zmm24, %zmm23

	vbroadcastsd	8(%r11), %zmm24
	vmulpd			%zmm1, %zmm24, %zmm1
	vmulpd			%zmm9, %zmm24, %zmm9
	vmulpd			%zmm17, %zmm24, %zmm17
	vbroadcastsd	16+1*64(%r10), %zmm24
	vfnmadd231pd	%zmm1, %zmm24, %zmm2
	vfnmadd231pd	%zmm9, %zmm24, %zmm10
	vfnmadd231pd	%zmm17, %zmm24, %zmm18
	vbroadcastsd	24+1*64(%r10), %zmm24
	vfnmadd231pd	%zmm1, %zmm24, %zmm3
	vfnmadd231pd	%zmm9, %zmm24, %zmm11
	vfnmadd231pd	%zmm17, %zmm24, %zmm19
	vbroadcastsd	32+1*64(%r10), %zmm24
	vfnmadd231pd	%zmm1, %zmm24, %zmm4
	vfnmadd231pd	%zmm9, %zmm24, %zmm12
	vfnmadd231pd	%zmm17, %zmm24, %zmm20
	vbroadcastsd	40+1*64(%r10), %zmm24
	vfnmadd231pd	%zmm1, %zmm24, %zmm5
	vfnmadd231pd	%zmm9, %zmm24, %zmm13
	vfnmadd231pd	%zmm17, %zmm24, %zmm21
	vbroadcastsd	48+1*64(%r10), %zmm24
	vfnmadd231pd	%zmm1, %zmm24, %zmm6
	vfnmadd231pd	%zmm9, %zmm24, %zmm14
	vfnmadd231pd	%zmm17, %zmm24, %zmm22
	vbroadcastsd	56+1*64(%r10), %zmm24
	vfnmadd231pd	%zmm1, %zmm24, %zmm7
	vfnmadd231pd	%zmm9, %zmm24, %zmm15
	vfnmadd231pd	%zmm17, %zmm24, %zmm23

	vbroadcastsd	16(%r11), %zmm24
	vmulpd			%zmm2, %zmm24, %zmm2
	vmulpd			%zmm10, %zmm24, %zmm10
	vmulpd			%zmm18, %zmm24, %zmm18
	vbroadcastsd	24+2*64(%r10), %zmm24
	vfnmadd231pd	%zmm2, %zmm24, %zmm3
	vfnmadd231pd	%zmm10, %zmm24, %zmm11
	vfnmadd231pd	%zmm18, %zmm24, %zmm19
	vbroadcastsd	32+2*64(%r10), %zmm24
	vfnmadd231pd	%zmm2, %zmm24, %zmm4
	vfnmadd231pd	%zmm10, %zmm24, %zmm12
	vfnmadd231pd	%zmm18, %zmm24, %zmm20
	vbroadcastsd	40+2*64(%r10), %zmm24
	vfnmadd231pd	%zmm2, %zmm24, %zmm5
	vfnmadd231pd	%zmm10, %zmm24, %zmm13
	vfnmadd231pd	%zmm18, %zmm24, %zmm21
	vbroadcastsd	48+2*64(%r10), %zmm24
	vfnmadd231pd	%zmm2, %zmm24, %zmm6
	vfnmadd231pd	%zmm10, %zmm24, %zmm14
	vfnmadd231pd	%zmm18, %zmm24, %zmm22
	vbroadcastsd	56+2*64(%r10), %zmm24
	vfnmadd231pd	%zmm2, %zmm24, %zmm7
	vfnmadd231pd	%zmm10, %zmm24, %zmm15
	vfnmadd231pd	%zmm18, %zmm24, %zmm23

	vbroadcastsd	24(%r11), %zmm24
	vmulpd			%zmm3, %zmm24, %zmm3
	vmulpd			%zmm11, %zmm24, %zmm11
	vmulpd			%zmm19, %zmm24, %zmm19
	vbroadcastsd	32+3*64(%r10), %zmm24
	vfnmadd231pd	%zmm3, %zmm24, %zmm4
	vfnmadd231pd	%zmm11, %zmm24, %zmm12
	vfnmadd231pd	%zmm19, %zmm24, %zmm20
	vbroadcastsd	40+3*64(%r10), %zmm24
	vfnmadd231pd	%zmm3, %zmm24, %zmm5
	vfnmadd231pd	%zmm11, %zmm24, %zmm13
	vfnmadd231pd	%zmm19, %zmm24, %zmm21
	vbroadcastsd	48+3*64(%r10), %zmm24
	vfnmadd231pd	%zmm3, %zmm24, %zmm6
	vfnmadd231pd	%zmm11, %zmm24, %zmm14
	vfnmadd231pd	%zmm19, %zmm24, %zmm22
	vbroadcastsd	56+3*64(%r10), %zmm24
	vfnmadd231pd	%zmm3, %zmm24, %zmm7
	vfnmadd231pd	%zmm11, %zmm24, %zmm15
	vfnmadd231pd	%zmm19, %zmm24, %zmm23

	vbroadcastsd	32(%r11), %zmm24
	vmulpd			%zmm4, %zmm24, %zmm4
	vmulpd			%zmm12, %zmm24, %zmm12
	vmulpd			%zmm20, %zmm24, %zmm20
	vbroadcastsd	40+4*64(%r10), %zmm24
	vfnmadd231pd	%zmm4, %zmm24, %zmm5
	vfnmadd231pd	%zmm12, %zmm24, %zmm13
	vfnmadd231pd	%zmm20, %zmm24, %zmm21
	vbroadcastsd	48+4*64(%r10), %zmm24
	vfnmadd231pd	%zmm4, %zmm24, %zmm6
	vfnmadd231pd	%zmm12, %zmm24, %zmm14
	vfnmadd231pd	%zmm20, %zmm24, %zmm22
	vbroadcastsd	56+4*64(%r10), %zmm24
	vfnmadd231pd	%zmm4, %zmm24, %zmm7
	vfnmadd231pd	%zmm12, %zmm24, %zmm15
	vfnmadd231pd	%zmm20, %zmm24, %zmm23

	vbroadcastsd	40(%r11), %zmm24
	vmulpd			%zmm5, %zmm24, %zmm5
	vmulpd			%zmm13, %zmm24, %zmm13
	vmulpd			%zmm21, %zmm24, %zmm21
	vbroadcastsd	48+5*64(%r10), %zmm24
	vfnmadd231pd	%zmm5, %zmm24, %zmm6
	vfnmadd231pd	%zmm13, %zmm24, %zmm14
	vfnmadd231pd	%zmm21, %zmm24, %zmm22
	vbroadcastsd	56+5*64(%r10), %zmm24
	vfnmadd231pd	%zmm5, %zmm24, %zmm7
	vfnmadd231pd	%zmm13, %zmm24, %zmm15
	vfnmadd231pd	%zmm21, %zmm24, %zmm23

	vbroadcastsd	48(%r11), %zmm24
	vmulpd			%zmm6, %zmm24, %zmm6
	vmulpd			%zmm14, %zmm24, %zmm14
	vmulpd			%zmm22, %zmm24, %zmm22
	vbroadcastsd	56+6*64(%r10), %zmm24
	vfnmadd231pd	%zmm6, %zmm24, %zmm7
	vfnmadd231pd	%zmm14, %zmm24, %zmm15
	vfnmadd231pd	%zmm22, %zmm24, %zmm23

	vbroadcastsd	56(%r11), %zmm24
	vmulpd			%zmm7, %zmm24, %zmm7
	vmulpd			%zmm15, %zmm24, %zmm15
	vmulpd			%zmm23, %zmm24, %zmm23

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rlt_inv_16x8_lib8)
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
// r12d <- n1
// zmm0 <- []
// ...
// zmm23 <- []
//
// output arguments:
// r10  <- D
// r11  <- inv_diag_D
// r12d <- n1
// zmm0 <- []
// ...
// zmm23 <- []

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RLT_INV_24X8_VS_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rlt_inv_24x8_vs_lib8)
#endif
	
	vbroadcastsd	0(%r11), %zmm24
	vmulpd			%zmm0, %zmm24, %zmm0
	vmulpd			%zmm8, %zmm24, %zmm8
	vmulpd			%zmm16, %zmm24, %zmm16

	cmpl			$ 2, %r12d
	jl				0f // ret

	vbroadcastsd	8+0*64(%r10), %zmm24
	vfnmadd231pd	%zmm0, %zmm24, %zmm1
	vfnmadd231pd	%zmm8, %zmm24, %zmm9
	vfnmadd231pd	%zmm16, %zmm24, %zmm17
	vbroadcastsd	8(%r11), %zmm24
	vmulpd			%zmm1, %zmm24, %zmm1
	vmulpd			%zmm9, %zmm24, %zmm9
	vmulpd			%zmm17, %zmm24, %zmm17

	cmpl			$ 3, %r12d
	jl				0f // ret

	vbroadcastsd	16+0*64(%r10), %zmm24
	vfnmadd231pd	%zmm0, %zmm24, %zmm2
	vfnmadd231pd	%zmm8, %zmm24, %zmm10
	vfnmadd231pd	%zmm16, %zmm24, %zmm18
	vbroadcastsd	16+1*64(%r10), %zmm24
	vfnmadd231pd	%zmm1, %zmm24, %zmm2
	vfnmadd231pd	%zmm9, %zmm24, %zmm10
	vfnmadd231pd	%zmm17, %zmm24, %zmm18
	vbroadcastsd	16(%r11), %zmm24
	vmulpd			%zmm2, %zmm24, %zmm2
	vmulpd			%zmm10, %zmm24, %zmm10
	vmulpd			%zmm18, %zmm24, %zmm18

	cmpl			$ 4, %r12d
	jl				0f // ret

	vbroadcastsd	24+0*64(%r10), %zmm24
	vfnmadd231pd	%zmm0, %zmm24, %zmm3
	vfnmadd231pd	%zmm8, %zmm24, %zmm11
	vfnmadd231pd	%zmm16, %zmm24, %zmm19
	vbroadcastsd	24+1*64(%r10), %zmm24
	vfnmadd231pd	%zmm1, %zmm24, %zmm3
	vfnmadd231pd	%zmm9, %zmm24, %zmm11
	vfnmadd231pd	%zmm17, %zmm24, %zmm19
	vbroadcastsd	24+2*64(%r10), %zmm24
	vfnmadd231pd	%zmm2, %zmm24, %zmm3
	vfnmadd231pd	%zmm10, %zmm24, %zmm11
	vfnmadd231pd	%zmm18, %zmm24, %zmm19
	vbroadcastsd	24(%r11), %zmm24
	vmulpd			%zmm3, %zmm24, %zmm3
	vmulpd			%zmm11, %zmm24, %zmm11
	vmulpd			%zmm19, %zmm24, %zmm19

	cmpl			$ 5, %r12d
	jl				0f // ret

	vbroadcastsd	32+0*64(%r10), %zmm24
	vfnmadd231pd	%zmm0, %zmm24, %zmm4
	vfnmadd231pd	%zmm8, %zmm24, %zmm12
	vfnmadd231pd	%zmm16, %zmm24, %zmm20
	vbroadcastsd	32+1*64(%r10), %zmm24
	vfnmadd231pd	%zmm1, %zmm24, %zmm4
	vfnmadd231pd	%zmm9, %zmm24, %zmm12
	vfnmadd231pd	%zmm17, %zmm24, %zmm20
	vbroadcastsd	32+2*64(%r10), %zmm24
	vfnmadd231pd	%zmm2, %zmm24, %zmm4
	vfnmadd231pd	%zmm10, %zmm24, %zmm12
	vfnmadd231pd	%zmm18, %zmm24, %zmm20
	vbroadcastsd	32+3*64(%r10), %zmm24
	vfnmadd231pd	%zmm3, %zmm24, %zmm4
	vfnmadd231pd	%zmm11, %zmm24, %zmm12
	vfnmadd231pd	%zmm19, %zmm24, %zmm20
	vbroadcastsd	32(%r11), %zmm24
	vmulpd			%zmm4, %zmm24, %zmm4
	vmulpd			%zmm12, %zmm24, %zmm12
	vmulpd			%zmm20, %zmm24, %zmm20

	cmpl			$ 6, %r12d
	jl				0f // ret

	vbroadcastsd	40+0*64(%r10), %zmm24
	vfnmadd231pd	%zmm0, %zmm24, %zmm5
	vfnmadd231pd	%zmm8, %zmm24, %zmm13
	vfnmadd231pd	%zmm16, %zmm24, %zmm21
	vbroadcastsd	40+1*64(%r10), %zmm24
	vfnmadd231pd	%zmm1, %zmm24, %zmm5
	vfnmadd231pd	%zmm9, %zmm24, %zmm13
	vfnmadd231pd	%zmm17, %zmm24, %zmm21
	vbroadcastsd	40+2*64(%r10), %zmm24
	vfnmadd231pd	%zmm2, %zmm24, %zmm5
	vfnmadd231pd	%zmm10, %zmm24, %zmm13
	vfnmadd231pd	%zmm18, %zmm24, %zmm21
	vbroadcastsd	40+3*64(%r10), %zmm24
	vfnmadd231pd	%zmm3, %zmm24, %zmm5
	vfnmadd231pd	%zmm11, %zmm24, %zmm13
	vfnmadd231pd	%zmm19, %zmm24, %zmm21
	vbroadcastsd	40+4*64(%r10), %zmm24
	vfnmadd231pd	%zmm4, %zmm24, %zmm5
	vfnmadd231pd	%zmm12, %zmm24, %zmm13
	vfnmadd231pd	%zmm20, %zmm24, %zmm21
	vbroadcastsd	40(%r11), %zmm24
	vmulpd			%zmm5, %zmm24, %zmm5
	vmulpd			%zmm13, %zmm24, %zmm13
	vmulpd			%zmm21, %zmm24, %zmm21

	cmpl			$ 7, %r12d
	jl				0f // ret

	vbroadcastsd	48+0*64(%r10), %zmm24
	vfnmadd231pd	%zmm0, %zmm24, %zmm6
	vfnmadd231pd	%zmm8, %zmm24, %zmm14
	vfnmadd231pd	%zmm16, %zmm24, %zmm22
	vbroadcastsd	48+1*64(%r10), %zmm24
	vfnmadd231pd	%zmm1, %zmm24, %zmm6
	vfnmadd231pd	%zmm9, %zmm24, %zmm14
	vfnmadd231pd	%zmm17, %zmm24, %zmm22
	vbroadcastsd	48+2*64(%r10), %zmm24
	vfnmadd231pd	%zmm2, %zmm24, %zmm6
	vfnmadd231pd	%zmm10, %zmm24, %zmm14
	vfnmadd231pd	%zmm18, %zmm24, %zmm22
	vbroadcastsd	48+3*64(%r10), %zmm24
	vfnmadd231pd	%zmm3, %zmm24, %zmm6
	vfnmadd231pd	%zmm11, %zmm24, %zmm14
	vfnmadd231pd	%zmm19, %zmm24, %zmm22
	vbroadcastsd	48+4*64(%r10), %zmm24
	vfnmadd231pd	%zmm4, %zmm24, %zmm6
	vfnmadd231pd	%zmm12, %zmm24, %zmm14
	vfnmadd231pd	%zmm20, %zmm24, %zmm22
	vbroadcastsd	48+5*64(%r10), %zmm24
	vfnmadd231pd	%zmm5, %zmm24, %zmm6
	vfnmadd231pd	%zmm13, %zmm24, %zmm14
	vfnmadd231pd	%zmm21, %zmm24, %zmm22
	vbroadcastsd	48(%r11), %zmm24
	vmulpd			%zmm6, %zmm24, %zmm6
	vmulpd			%zmm14, %zmm24, %zmm14
	vmulpd			%zmm22, %zmm24, %zmm22

	cmpl			$ 8, %r12d
	jl				0f // ret

	vbroadcastsd	56+0*64(%r10), %zmm24
	vfnmadd231pd	%zmm0, %zmm24, %zmm7
	vfnmadd231pd	%zmm8, %zmm24, %zmm15
	vfnmadd231pd	%zmm16, %zmm24, %zmm23
	vbroadcastsd	56+1*64(%r10), %zmm24
	vfnmadd231pd	%zmm1, %zmm24, %zmm7
	vfnmadd231pd	%zmm9, %zmm24, %zmm15
	vfnmadd231pd	%zmm17, %zmm24, %zmm23
	vbroadcastsd	56+2*64(%r10), %zmm24
	vfnmadd231pd	%zmm2, %zmm24, %zmm7
	vfnmadd231pd	%zmm10, %zmm24, %zmm15
	vfnmadd231pd	%zmm18, %zmm24, %zmm23
	vbroadcastsd	56+3*64(%r10), %zmm24
	vfnmadd231pd	%zmm3, %zmm24, %zmm7
	vfnmadd231pd	%zmm11, %zmm24, %zmm15
	vfnmadd231pd	%zmm19, %zmm24, %zmm23
	vbroadcastsd	56+4*64(%r10), %zmm24
	vfnmadd231pd	%zmm4, %zmm24, %zmm7
	vfnmadd231pd	%zmm12, %zmm24, %zmm15
	vfnmadd231pd	%zmm20, %zmm24, %zmm23
	vbroadcastsd	56+5*64(%r10), %zmm24
	vfnmadd231pd	%zmm5, %zmm24, %zmm7
	vfnmadd231pd	%zmm13, %zmm24, %zmm15
	vfnmadd231pd	%zmm21, %zmm24, %zmm23
	vbroadcastsd	56+6*64(%r10), %zmm24
	vfnmadd231pd	%zmm6, %zmm24, %zmm7
	vfnmadd231pd	%zmm14, %zmm24, %zmm15
	vfnmadd231pd	%zmm22, %zmm24, %zmm23
	vbroadcastsd	56(%r11), %zmm24
	vmulpd			%zmm7, %zmm24, %zmm7
	vmulpd			%zmm15, %zmm24, %zmm15
	vmulpd			%zmm23, %zmm24, %zmm23

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rlt_inv_24x8_vs_lib8)
#endif





// common inner routine with file scope
//
// cholesky factorization 
//
// input arguments:
// r10  <- inv_diag_E
// zmm0 <- []
// ...
// ymm23 <- []
//
// output arguments:
// r10  <- inv_diag_E
// zmm0 <- []
// ...
// ymm23 <- []

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DPOTRF_24X8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_dpotrf_24x8_lib8)
#endif

	vxorpd	%ymm25, %ymm25, %ymm25 // 0.0
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd	.LC04(%rip), %xmm24 // 1.0
#elif defined(OS_MAC)
	vmovsd	LC04(%rip), %xmm24 // 1.0
#endif

	// 0
	vmovsd			%xmm0, %xmm0, %xmm26
	vucomisd		%xmm25, %xmm26 // d_00 > 0.0 ?
	jbe				1f
	vsqrtsd			%xmm26, %xmm26, %xmm26
	vdivsd			%xmm26, %xmm24, %xmm26
2:
	vmovsd			%xmm26, 0(%r10)
	vbroadcastsd	%xmm26, %zmm26
	vmulpd			%zmm0, %zmm26, %zmm0
	vmulpd			%zmm8, %zmm26, %zmm8
	vmulpd			%zmm16, %zmm26, %zmm16
	vmovapd			%zmm1, %zmm26
	vfnmadd231pd	%zmm0, %zmm0, %zmm26

	// 1
//	vpermilpd		$ 0x3, %xmm1, %xmm26
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd			%zmm26, %zmm27, %zmm26
	vucomisd		%xmm25, %xmm26 // d_11 > 0.0 ?
	jbe				3f
	vsqrtsd			%xmm26, %xmm26, %xmm26
	vdivsd			%xmm26, %xmm24, %xmm26
4:
#if defined(OS_LINUX) | defined(OS_WINDOWS)
//	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
//	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm27
	vfnmadd231pd	%zmm0, %zmm27, %zmm1
	vfnmadd231pd	%zmm8, %zmm27, %zmm9
	vfnmadd231pd	%zmm16, %zmm27, %zmm17
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm27
	vfnmadd231pd	%zmm0, %zmm27, %zmm2
	vfnmadd231pd	%zmm8, %zmm27, %zmm10
	vfnmadd231pd	%zmm16, %zmm27, %zmm18
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm27
	vfnmadd231pd	%zmm0, %zmm27, %zmm3
	vfnmadd231pd	%zmm8, %zmm27, %zmm11
	vfnmadd231pd	%zmm16, %zmm27, %zmm19
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm27
	vfnmadd231pd	%zmm0, %zmm27, %zmm4
	vfnmadd231pd	%zmm8, %zmm27, %zmm12
	vfnmadd231pd	%zmm16, %zmm27, %zmm20
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm27
	vfnmadd231pd	%zmm0, %zmm27, %zmm5
	vfnmadd231pd	%zmm8, %zmm27, %zmm13
	vfnmadd231pd	%zmm16, %zmm27, %zmm21
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm27
	vfnmadd231pd	%zmm0, %zmm27, %zmm6
	vfnmadd231pd	%zmm8, %zmm27, %zmm14
	vfnmadd231pd	%zmm16, %zmm27, %zmm22
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm27
	vfnmadd231pd	%zmm0, %zmm27, %zmm7
	vfnmadd231pd	%zmm8, %zmm27, %zmm15
	vfnmadd231pd	%zmm16, %zmm27, %zmm23
	vmovsd			%xmm26, 8(%r10)
	vbroadcastsd	%xmm26, %zmm26
	vmulpd			%zmm1, %zmm26, %zmm1
	vmulpd			%zmm9, %zmm26, %zmm9
	vmulpd			%zmm17, %zmm26, %zmm17
	vmovapd			%zmm2, %zmm26
	vfnmadd231pd	%zmm1, %zmm1, %zmm26

	// 2
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd			%zmm26, %zmm27, %zmm26
	vucomisd		%xmm25, %xmm26 // d_22 > 0.0 ?
	jbe				5f
	vsqrtsd			%xmm26, %xmm26, %xmm26
	vdivsd			%xmm26, %xmm24, %xmm26
6:
#if defined(OS_LINUX) | defined(OS_WINDOWS)
//	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
//	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd			%zmm1, %zmm27, %zmm27
	vfnmadd231pd	%zmm1, %zmm27, %zmm2
	vfnmadd231pd	%zmm9, %zmm27, %zmm10
	vfnmadd231pd	%zmm17, %zmm27, %zmm18
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd			%zmm1, %zmm27, %zmm27
	vfnmadd231pd	%zmm1, %zmm27, %zmm3
	vfnmadd231pd	%zmm9, %zmm27, %zmm11
	vfnmadd231pd	%zmm17, %zmm27, %zmm19
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd			%zmm1, %zmm27, %zmm27
	vfnmadd231pd	%zmm1, %zmm27, %zmm4
	vfnmadd231pd	%zmm9, %zmm27, %zmm12
	vfnmadd231pd	%zmm17, %zmm27, %zmm20
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd			%zmm1, %zmm27, %zmm27
	vfnmadd231pd	%zmm1, %zmm27, %zmm5
	vfnmadd231pd	%zmm9, %zmm27, %zmm13
	vfnmadd231pd	%zmm17, %zmm27, %zmm21
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd			%zmm1, %zmm27, %zmm27
	vfnmadd231pd	%zmm1, %zmm27, %zmm6
	vfnmadd231pd	%zmm9, %zmm27, %zmm14
	vfnmadd231pd	%zmm17, %zmm27, %zmm22
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd			%zmm1, %zmm27, %zmm27
	vfnmadd231pd	%zmm1, %zmm27, %zmm7
	vfnmadd231pd	%zmm9, %zmm27, %zmm15
	vfnmadd231pd	%zmm17, %zmm27, %zmm23
	vmovsd			%xmm26, 16(%r10)
	vbroadcastsd	%xmm26, %zmm26
	vmulpd			%zmm2, %zmm26, %zmm2
	vmulpd			%zmm10, %zmm26, %zmm10
	vmulpd			%zmm18, %zmm26, %zmm18
	vmovapd			%zmm3, %zmm26
	vfnmadd231pd	%zmm2, %zmm2, %zmm26

	// 3
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd			%zmm26, %zmm27, %zmm26
	vucomisd		%xmm25, %xmm26 // d_22 > 0.0 ?
	jbe				7f
	vsqrtsd			%xmm26, %xmm26, %xmm26
	vdivsd			%xmm26, %xmm24, %xmm26
8:
#if defined(OS_LINUX) | defined(OS_WINDOWS)
//	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
//	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd			%zmm2, %zmm27, %zmm27
	vfnmadd231pd	%zmm2, %zmm27, %zmm3
	vfnmadd231pd	%zmm10, %zmm27, %zmm11
	vfnmadd231pd	%zmm18, %zmm27, %zmm19
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd			%zmm2, %zmm27, %zmm27
	vfnmadd231pd	%zmm2, %zmm27, %zmm4
	vfnmadd231pd	%zmm10, %zmm27, %zmm12
	vfnmadd231pd	%zmm18, %zmm27, %zmm20
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd			%zmm2, %zmm27, %zmm27
	vfnmadd231pd	%zmm2, %zmm27, %zmm5
	vfnmadd231pd	%zmm10, %zmm27, %zmm13
	vfnmadd231pd	%zmm18, %zmm27, %zmm21
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd			%zmm2, %zmm27, %zmm27
	vfnmadd231pd	%zmm2, %zmm27, %zmm6
	vfnmadd231pd	%zmm10, %zmm27, %zmm14
	vfnmadd231pd	%zmm18, %zmm27, %zmm22
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd			%zmm2, %zmm27, %zmm27
	vfnmadd231pd	%zmm2, %zmm27, %zmm7
	vfnmadd231pd	%zmm10, %zmm27, %zmm15
	vfnmadd231pd	%zmm18, %zmm27, %zmm23
	vmovsd			%xmm26, 24(%r10)
	vbroadcastsd	%xmm26, %zmm26
	vmulpd			%zmm3, %zmm26, %zmm3
	vmulpd			%zmm11, %zmm26, %zmm11
	vmulpd			%zmm19, %zmm26, %zmm19
	vmovapd			%zmm4, %zmm26
	vfnmadd231pd	%zmm3, %zmm3, %zmm26

	// 4
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd			%zmm26, %zmm27, %zmm26
	vucomisd		%xmm25, %xmm26 // d_22 > 0.0 ?
	jbe				9f
	vsqrtsd			%xmm26, %xmm26, %xmm26
	vdivsd			%xmm26, %xmm24, %xmm26
10:
#if defined(OS_LINUX) | defined(OS_WINDOWS)
//	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
//	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd			%zmm3, %zmm27, %zmm27
	vfnmadd231pd	%zmm3, %zmm27, %zmm4
	vfnmadd231pd	%zmm11, %zmm27, %zmm12
	vfnmadd231pd	%zmm19, %zmm27, %zmm20
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd			%zmm3, %zmm27, %zmm27
	vfnmadd231pd	%zmm3, %zmm27, %zmm5
	vfnmadd231pd	%zmm11, %zmm27, %zmm13
	vfnmadd231pd	%zmm19, %zmm27, %zmm21
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd			%zmm3, %zmm27, %zmm27
	vfnmadd231pd	%zmm3, %zmm27, %zmm6
	vfnmadd231pd	%zmm11, %zmm27, %zmm14
	vfnmadd231pd	%zmm19, %zmm27, %zmm22
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd			%zmm3, %zmm27, %zmm27
	vfnmadd231pd	%zmm3, %zmm27, %zmm7
	vfnmadd231pd	%zmm11, %zmm27, %zmm15
	vfnmadd231pd	%zmm19, %zmm27, %zmm23
	vmovsd			%xmm26, 32(%r10)
	vbroadcastsd	%xmm26, %zmm26
	vmulpd			%zmm4, %zmm26, %zmm4
	vmulpd			%zmm12, %zmm26, %zmm12
	vmulpd			%zmm20, %zmm26, %zmm20
	vmovapd			%zmm5, %zmm26
	vfnmadd231pd	%zmm4, %zmm4, %zmm26

	// 5
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd			%zmm26, %zmm27, %zmm26
	vucomisd		%xmm25, %xmm26 // d_22 > 0.0 ?
	jbe				11f
	vsqrtsd			%xmm26, %xmm26, %xmm26
	vdivsd			%xmm26, %xmm24, %xmm26
12:
#if defined(OS_LINUX) | defined(OS_WINDOWS)
//	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
//	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd			%zmm4, %zmm27, %zmm27
	vfnmadd231pd	%zmm4, %zmm27, %zmm5
	vfnmadd231pd	%zmm12, %zmm27, %zmm13
	vfnmadd231pd	%zmm20, %zmm27, %zmm21
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd			%zmm4, %zmm27, %zmm27
	vfnmadd231pd	%zmm4, %zmm27, %zmm6
	vfnmadd231pd	%zmm12, %zmm27, %zmm14
	vfnmadd231pd	%zmm20, %zmm27, %zmm22
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd			%zmm4, %zmm27, %zmm27
	vfnmadd231pd	%zmm4, %zmm27, %zmm7
	vfnmadd231pd	%zmm12, %zmm27, %zmm15
	vfnmadd231pd	%zmm20, %zmm27, %zmm23
	vmovsd			%xmm26, 40(%r10)
	vbroadcastsd	%xmm26, %zmm26
	vmulpd			%zmm5, %zmm26, %zmm5
	vmulpd			%zmm13, %zmm26, %zmm13
	vmulpd			%zmm21, %zmm26, %zmm21
	vmovapd			%zmm6, %zmm26
	vfnmadd231pd	%zmm5, %zmm5, %zmm26

	// 6
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd			%zmm26, %zmm27, %zmm26
	vucomisd		%xmm25, %xmm26 // d_22 > 0.0 ?
	jbe				13f
	vsqrtsd			%xmm26, %xmm26, %xmm26
	vdivsd			%xmm26, %xmm24, %xmm26
14:
#if defined(OS_LINUX) | defined(OS_WINDOWS)
//	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
//	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd			%zmm5, %zmm27, %zmm27
	vfnmadd231pd	%zmm5, %zmm27, %zmm6
	vfnmadd231pd	%zmm13, %zmm27, %zmm14
	vfnmadd231pd	%zmm21, %zmm27, %zmm22
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd			%zmm5, %zmm27, %zmm27
	vfnmadd231pd	%zmm5, %zmm27, %zmm7
	vfnmadd231pd	%zmm13, %zmm27, %zmm15
	vfnmadd231pd	%zmm21, %zmm27, %zmm23
	vmovsd			%xmm26, 48(%r10)
	vbroadcastsd	%xmm26, %zmm26
	vmulpd			%zmm6, %zmm26, %zmm6
	vmulpd			%zmm14, %zmm26, %zmm14
	vmulpd			%zmm22, %zmm26, %zmm22
	vmovapd			%zmm7, %zmm26
	vfnmadd231pd	%zmm6, %zmm6, %zmm26

	// 7
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd			%zmm26, %zmm27, %zmm26
	vucomisd		%xmm25, %xmm26 // d_22 > 0.0 ?
	jbe				15f
	vsqrtsd			%xmm26, %xmm26, %xmm26
	vdivsd			%xmm26, %xmm24, %xmm26
16:
#if defined(OS_LINUX) | defined(OS_WINDOWS)
//	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
//	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd			%zmm6, %zmm27, %zmm27
	vfnmadd231pd	%zmm6, %zmm27, %zmm7
	vfnmadd231pd	%zmm14, %zmm27, %zmm15
	vfnmadd231pd	%zmm22, %zmm27, %zmm23
	vmovsd			%xmm26, 56(%r10)
	vbroadcastsd	%xmm26, %zmm26
	vmulpd			%zmm7, %zmm26, %zmm7
	vmulpd			%zmm15, %zmm26, %zmm15
	vmulpd			%zmm23, %zmm26, %zmm23

	jmp				0f

1:
	vxorpd			%zmm26, %zmm26, %zmm26
	jmp				2b

3:
	vxorpd			%zmm26, %zmm26, %zmm26
	jmp				4b

5:
	vxorpd			%zmm26, %zmm26, %zmm26
	jmp				6b

7:
	vxorpd			%zmm26, %zmm26, %zmm26
	jmp				8b

9:
	vxorpd			%zmm26, %zmm26, %zmm26
	jmp				10b

11:
	vxorpd			%zmm26, %zmm26, %zmm26
	jmp				12b

13:
	vxorpd			%zmm26, %zmm26, %zmm26
	jmp				14b

15:
	vxorpd			%zmm26, %zmm26, %zmm26
	jmp				16b

0:
	#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dpotrf_24x8_lib8)
#endif





// common inner routine with file scope
//
// cholesky factorization 
//
// input arguments:
// r10  <- inv_diag_E
// r11d <- m1
// r12d <- n1
// zmm0 <- []
// ...
// ymm23 <- []
//
// output arguments:
// r10  <- inv_diag_E
// r11d <- m1
// r12d <- n1
// zmm0 <- []
// ...
// ymm23 <- []

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DPOTRF_24X8_VS_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_dpotrf_24x8_vs_lib8)
#endif

	// compute mask for rows
	vcvtsi2sd	%r11d, %xmm25, %xmm25
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC02(%rip), %zmm24
#elif defined(OS_MAC)
	vmovupd		LC02(%rip), %zmm24
#endif
	vbroadcastsd	%xmm25, %zmm25
	vsubpd		%zmm25, %zmm24, %zmm25
	vpmovq2m	%zmm25, %k1

	vmovapd		%zmm16, %zmm16 {%k1}{z}
	vmovapd		%zmm17, %zmm17 {%k1}{z}
	vmovapd		%zmm18, %zmm18 {%k1}{z}
	vmovapd		%zmm19, %zmm19 {%k1}{z}
	vmovapd		%zmm20, %zmm20 {%k1}{z}
	vmovapd		%zmm21, %zmm21 {%k1}{z}
	vmovapd		%zmm22, %zmm22 {%k1}{z}
	vmovapd		%zmm23, %zmm23 {%k1}{z}

	vxorpd	%ymm25, %ymm25, %ymm25 // 0.0
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd	.LC04(%rip), %xmm24 // 1.0
#elif defined(OS_MAC)
	vmovsd	LC04(%rip), %xmm24 // 1.0
#endif

	// 0
	vmovsd			%xmm0, %xmm0, %xmm26
	vucomisd		%xmm25, %xmm26 // d_00 > 0.0 ?
	jbe				1f
	vsqrtsd			%xmm26, %xmm26, %xmm26
	vdivsd			%xmm26, %xmm24, %xmm26
2:
	vmovsd			%xmm26, 0(%r10)
	vbroadcastsd	%xmm26, %zmm26
	vmulpd			%zmm0, %zmm26, %zmm0
	vmulpd			%zmm8, %zmm26, %zmm8
	vmulpd			%zmm16, %zmm26, %zmm16
	cmpl			$ 2, %r12d
	jl				0f // ret
	vmovapd			%zmm1, %zmm26
	vfnmadd231pd	%zmm0, %zmm0, %zmm26

	// 1
//	vpermilpd		$ 0x3, %xmm1, %xmm26
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd			%zmm26, %zmm27, %zmm26
	vucomisd		%xmm25, %xmm26 // d_11 > 0.0 ?
	jbe				3f
	vsqrtsd			%xmm26, %xmm26, %xmm26
	vdivsd			%xmm26, %xmm24, %xmm26
4:
#if defined(OS_LINUX) | defined(OS_WINDOWS)
//	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
//	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm27
	vfnmadd231pd	%zmm0, %zmm27, %zmm1
	vfnmadd231pd	%zmm8, %zmm27, %zmm9
	vfnmadd231pd	%zmm16, %zmm27, %zmm17
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm27
	vfnmadd231pd	%zmm0, %zmm27, %zmm2
	vfnmadd231pd	%zmm8, %zmm27, %zmm10
	vfnmadd231pd	%zmm16, %zmm27, %zmm18
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm27
	vfnmadd231pd	%zmm0, %zmm27, %zmm3
	vfnmadd231pd	%zmm8, %zmm27, %zmm11
	vfnmadd231pd	%zmm16, %zmm27, %zmm19
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm27
	vfnmadd231pd	%zmm0, %zmm27, %zmm4
	vfnmadd231pd	%zmm8, %zmm27, %zmm12
	vfnmadd231pd	%zmm16, %zmm27, %zmm20
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm27
	vfnmadd231pd	%zmm0, %zmm27, %zmm5
	vfnmadd231pd	%zmm8, %zmm27, %zmm13
	vfnmadd231pd	%zmm16, %zmm27, %zmm21
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm27
	vfnmadd231pd	%zmm0, %zmm27, %zmm6
	vfnmadd231pd	%zmm8, %zmm27, %zmm14
	vfnmadd231pd	%zmm16, %zmm27, %zmm22
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm27
	vfnmadd231pd	%zmm0, %zmm27, %zmm7
	vfnmadd231pd	%zmm8, %zmm27, %zmm15
	vfnmadd231pd	%zmm16, %zmm27, %zmm23
	vmovsd			%xmm26, 8(%r10)
	vbroadcastsd	%xmm26, %zmm26
	vmulpd			%zmm1, %zmm26, %zmm1
	vmulpd			%zmm9, %zmm26, %zmm9
	vmulpd			%zmm17, %zmm26, %zmm17
	cmpl			$ 3, %r12d
	jl				0f // ret
	vmovapd			%zmm2, %zmm26
	vfnmadd231pd	%zmm1, %zmm1, %zmm26

	// 2
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd			%zmm26, %zmm27, %zmm26
	vucomisd		%xmm25, %xmm26 // d_22 > 0.0 ?
	jbe				5f
	vsqrtsd			%xmm26, %xmm26, %xmm26
	vdivsd			%xmm26, %xmm24, %xmm26
6:
#if defined(OS_LINUX) | defined(OS_WINDOWS)
//	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
//	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd			%zmm1, %zmm27, %zmm27
	vfnmadd231pd	%zmm1, %zmm27, %zmm2
	vfnmadd231pd	%zmm9, %zmm27, %zmm10
	vfnmadd231pd	%zmm17, %zmm27, %zmm18
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd			%zmm1, %zmm27, %zmm27
	vfnmadd231pd	%zmm1, %zmm27, %zmm3
	vfnmadd231pd	%zmm9, %zmm27, %zmm11
	vfnmadd231pd	%zmm17, %zmm27, %zmm19
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd			%zmm1, %zmm27, %zmm27
	vfnmadd231pd	%zmm1, %zmm27, %zmm4
	vfnmadd231pd	%zmm9, %zmm27, %zmm12
	vfnmadd231pd	%zmm17, %zmm27, %zmm20
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd			%zmm1, %zmm27, %zmm27
	vfnmadd231pd	%zmm1, %zmm27, %zmm5
	vfnmadd231pd	%zmm9, %zmm27, %zmm13
	vfnmadd231pd	%zmm17, %zmm27, %zmm21
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd			%zmm1, %zmm27, %zmm27
	vfnmadd231pd	%zmm1, %zmm27, %zmm6
	vfnmadd231pd	%zmm9, %zmm27, %zmm14
	vfnmadd231pd	%zmm17, %zmm27, %zmm22
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd			%zmm1, %zmm27, %zmm27
	vfnmadd231pd	%zmm1, %zmm27, %zmm7
	vfnmadd231pd	%zmm9, %zmm27, %zmm15
	vfnmadd231pd	%zmm17, %zmm27, %zmm23
	vmovsd			%xmm26, 16(%r10)
	vbroadcastsd	%xmm26, %zmm26
	vmulpd			%zmm2, %zmm26, %zmm2
	vmulpd			%zmm10, %zmm26, %zmm10
	vmulpd			%zmm18, %zmm26, %zmm18
	cmpl			$ 4, %r12d
	jl				0f // ret
	vmovapd			%zmm3, %zmm26
	vfnmadd231pd	%zmm2, %zmm2, %zmm26

	// 3
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd			%zmm26, %zmm27, %zmm26
	vucomisd		%xmm25, %xmm26 // d_22 > 0.0 ?
	jbe				7f
	vsqrtsd			%xmm26, %xmm26, %xmm26
	vdivsd			%xmm26, %xmm24, %xmm26
8:
#if defined(OS_LINUX) | defined(OS_WINDOWS)
//	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
//	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd			%zmm2, %zmm27, %zmm27
	vfnmadd231pd	%zmm2, %zmm27, %zmm3
	vfnmadd231pd	%zmm10, %zmm27, %zmm11
	vfnmadd231pd	%zmm18, %zmm27, %zmm19
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd			%zmm2, %zmm27, %zmm27
	vfnmadd231pd	%zmm2, %zmm27, %zmm4
	vfnmadd231pd	%zmm10, %zmm27, %zmm12
	vfnmadd231pd	%zmm18, %zmm27, %zmm20
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd			%zmm2, %zmm27, %zmm27
	vfnmadd231pd	%zmm2, %zmm27, %zmm5
	vfnmadd231pd	%zmm10, %zmm27, %zmm13
	vfnmadd231pd	%zmm18, %zmm27, %zmm21
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd			%zmm2, %zmm27, %zmm27
	vfnmadd231pd	%zmm2, %zmm27, %zmm6
	vfnmadd231pd	%zmm10, %zmm27, %zmm14
	vfnmadd231pd	%zmm18, %zmm27, %zmm22
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd			%zmm2, %zmm27, %zmm27
	vfnmadd231pd	%zmm2, %zmm27, %zmm7
	vfnmadd231pd	%zmm10, %zmm27, %zmm15
	vfnmadd231pd	%zmm18, %zmm27, %zmm23
	vmovsd			%xmm26, 24(%r10)
	vbroadcastsd	%xmm26, %zmm26
	vmulpd			%zmm3, %zmm26, %zmm3
	vmulpd			%zmm11, %zmm26, %zmm11
	vmulpd			%zmm19, %zmm26, %zmm19
	cmpl			$ 5, %r12d
	jl				0f // ret
	vmovapd			%zmm4, %zmm26
	vfnmadd231pd	%zmm3, %zmm3, %zmm26

	// 4
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd			%zmm26, %zmm27, %zmm26
	vucomisd		%xmm25, %xmm26 // d_22 > 0.0 ?
	jbe				9f
	vsqrtsd			%xmm26, %xmm26, %xmm26
	vdivsd			%xmm26, %xmm24, %xmm26
10:
#if defined(OS_LINUX) | defined(OS_WINDOWS)
//	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
//	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd			%zmm3, %zmm27, %zmm27
	vfnmadd231pd	%zmm3, %zmm27, %zmm4
	vfnmadd231pd	%zmm11, %zmm27, %zmm12
	vfnmadd231pd	%zmm19, %zmm27, %zmm20
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd			%zmm3, %zmm27, %zmm27
	vfnmadd231pd	%zmm3, %zmm27, %zmm5
	vfnmadd231pd	%zmm11, %zmm27, %zmm13
	vfnmadd231pd	%zmm19, %zmm27, %zmm21
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd			%zmm3, %zmm27, %zmm27
	vfnmadd231pd	%zmm3, %zmm27, %zmm6
	vfnmadd231pd	%zmm11, %zmm27, %zmm14
	vfnmadd231pd	%zmm19, %zmm27, %zmm22
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd			%zmm3, %zmm27, %zmm27
	vfnmadd231pd	%zmm3, %zmm27, %zmm7
	vfnmadd231pd	%zmm11, %zmm27, %zmm15
	vfnmadd231pd	%zmm19, %zmm27, %zmm23
	vmovsd			%xmm26, 32(%r10)
	vbroadcastsd	%xmm26, %zmm26
	vmulpd			%zmm4, %zmm26, %zmm4
	vmulpd			%zmm12, %zmm26, %zmm12
	vmulpd			%zmm20, %zmm26, %zmm20
	cmpl			$ 6, %r12d
	jl				0f // ret
	vmovapd			%zmm5, %zmm26
	vfnmadd231pd	%zmm4, %zmm4, %zmm26

	// 5
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd			%zmm26, %zmm27, %zmm26
	vucomisd		%xmm25, %xmm26 // d_22 > 0.0 ?
	jbe				11f
	vsqrtsd			%xmm26, %xmm26, %xmm26
	vdivsd			%xmm26, %xmm24, %xmm26
12:
#if defined(OS_LINUX) | defined(OS_WINDOWS)
//	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
//	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd			%zmm4, %zmm27, %zmm27
	vfnmadd231pd	%zmm4, %zmm27, %zmm5
	vfnmadd231pd	%zmm12, %zmm27, %zmm13
	vfnmadd231pd	%zmm20, %zmm27, %zmm21
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd			%zmm4, %zmm27, %zmm27
	vfnmadd231pd	%zmm4, %zmm27, %zmm6
	vfnmadd231pd	%zmm12, %zmm27, %zmm14
	vfnmadd231pd	%zmm20, %zmm27, %zmm22
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd			%zmm4, %zmm27, %zmm27
	vfnmadd231pd	%zmm4, %zmm27, %zmm7
	vfnmadd231pd	%zmm12, %zmm27, %zmm15
	vfnmadd231pd	%zmm20, %zmm27, %zmm23
	vmovsd			%xmm26, 40(%r10)
	vbroadcastsd	%xmm26, %zmm26
	vmulpd			%zmm5, %zmm26, %zmm5
	vmulpd			%zmm13, %zmm26, %zmm13
	vmulpd			%zmm21, %zmm26, %zmm21
	cmpl			$ 7, %r12d
	jl				0f // ret
	vmovapd			%zmm6, %zmm26
	vfnmadd231pd	%zmm5, %zmm5, %zmm26

	// 6
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd			%zmm26, %zmm27, %zmm26
	vucomisd		%xmm25, %xmm26 // d_22 > 0.0 ?
	jbe				13f
	vsqrtsd			%xmm26, %xmm26, %xmm26
	vdivsd			%xmm26, %xmm24, %xmm26
14:
#if defined(OS_LINUX) | defined(OS_WINDOWS)
//	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
//	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd			%zmm5, %zmm27, %zmm27
	vfnmadd231pd	%zmm5, %zmm27, %zmm6
	vfnmadd231pd	%zmm13, %zmm27, %zmm14
	vfnmadd231pd	%zmm21, %zmm27, %zmm22
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd			%zmm5, %zmm27, %zmm27
	vfnmadd231pd	%zmm5, %zmm27, %zmm7
	vfnmadd231pd	%zmm13, %zmm27, %zmm15
	vfnmadd231pd	%zmm21, %zmm27, %zmm23
	vmovsd			%xmm26, 48(%r10)
	vbroadcastsd	%xmm26, %zmm26
	vmulpd			%zmm6, %zmm26, %zmm6
	vmulpd			%zmm14, %zmm26, %zmm14
	vmulpd			%zmm22, %zmm26, %zmm22
	cmpl			$ 8, %r12d
	jl				0f // ret
	vmovapd			%zmm7, %zmm26
	vfnmadd231pd	%zmm6, %zmm6, %zmm26

	// 7
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd			%zmm26, %zmm27, %zmm26
	vucomisd		%xmm25, %xmm26 // d_22 > 0.0 ?
	jbe				15f
	vsqrtsd			%xmm26, %xmm26, %xmm26
	vdivsd			%xmm26, %xmm24, %xmm26
16:
#if defined(OS_LINUX) | defined(OS_WINDOWS)
//	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
//	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd			%zmm6, %zmm27, %zmm27
	vfnmadd231pd	%zmm6, %zmm27, %zmm7
	vfnmadd231pd	%zmm14, %zmm27, %zmm15
	vfnmadd231pd	%zmm22, %zmm27, %zmm23
	vmovsd			%xmm26, 56(%r10)
	vbroadcastsd	%xmm26, %zmm26
	vmulpd			%zmm7, %zmm26, %zmm7
	vmulpd			%zmm15, %zmm26, %zmm15
	vmulpd			%zmm23, %zmm26, %zmm23

	jmp				0f

1:
	vxorpd			%zmm26, %zmm26, %zmm26
	jmp				2b

3:
	vxorpd			%zmm26, %zmm26, %zmm26
	jmp				4b

5:
	vxorpd			%zmm26, %zmm26, %zmm26
	jmp				6b

7:
	vxorpd			%zmm26, %zmm26, %zmm26
	jmp				8b

9:
	vxorpd			%zmm26, %zmm26, %zmm26
	jmp				10b

11:
	vxorpd			%zmm26, %zmm26, %zmm26
	jmp				12b

13:
	vxorpd			%zmm26, %zmm26, %zmm26
	jmp				14b

15:
	vxorpd			%zmm26, %zmm26, %zmm26
	jmp				16b

0:
	#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dpotrf_24x8_vs_lib8)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- 8*sdc*sizeof(double)
// zmm0 <- []
// ...
// zmm23 <- []
//
// output arguments:
// r10   <- alpha
// r11   <- beta
// r10   <- C
// r13   <- 8*sdc*sizeof(double)
// zmm0 <- []
// ...
// zmm23 <- []

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_24X8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_24x8_lib8)
#endif
	
	// alpha
	vbroadcastsd	0(%r10), %zmm25

	vmulpd		%zmm0, %zmm25, %zmm0
	vmulpd		%zmm1, %zmm25, %zmm1
	vmulpd		%zmm2, %zmm25, %zmm2
	vmulpd		%zmm3, %zmm25, %zmm3
	vmulpd		%zmm4, %zmm25, %zmm4
	vmulpd		%zmm5, %zmm25, %zmm5
	vmulpd		%zmm6, %zmm25, %zmm6
	vmulpd		%zmm7, %zmm25, %zmm7
	vmulpd		%zmm8, %zmm25, %zmm8
	vmulpd		%zmm9, %zmm25, %zmm9
	vmulpd		%zmm10, %zmm25, %zmm10
	vmulpd		%zmm11, %zmm25, %zmm11
	vmulpd		%zmm12, %zmm25, %zmm12
	vmulpd		%zmm13, %zmm25, %zmm13
	vmulpd		%zmm14, %zmm25, %zmm14
	vmulpd		%zmm15, %zmm25, %zmm15
	vmulpd		%zmm16, %zmm25, %zmm16
	vmulpd		%zmm17, %zmm25, %zmm17
	vmulpd		%zmm18, %zmm25, %zmm18
	vmulpd		%zmm19, %zmm25, %zmm19
	vmulpd		%zmm20, %zmm25, %zmm20
	vmulpd		%zmm21, %zmm25, %zmm21
	vmulpd		%zmm22, %zmm25, %zmm22
	vmulpd		%zmm23, %zmm25, %zmm23

	// beta
	vbroadcastsd	0(%r11), %zmm24

	vxorpd		%zmm25, %zmm25, %zmm25 // 0.0

	vucomisd	%xmm25, %xmm24 // beta==0.0 ?
	je			0f // end

	vmovapd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm0
	vmovapd		64(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm1
	vmovapd		128(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm2
	vmovapd		192(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm3
	vmovapd		0+256(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm4
	vmovapd		64+256(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm5
	vmovapd		128+256(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm6
	vmovapd		192+256(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm7

	vmovapd		0(%r12, %r13), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm8
	vmovapd		64(%r12, %r13), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm9
	vmovapd		128(%r12, %r13), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm10
	vmovapd		192(%r12, %r13), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm11
	vmovapd		0+256(%r12, %r13), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm12
	vmovapd		64+256(%r12, %r13), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm13
	vmovapd		128+256(%r12, %r13), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm14
	vmovapd		192+256(%r12, %r13), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm15

	vmovapd		0(%r12, %r13, 2), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm16
	vmovapd		64(%r12, %r13, 2), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm17
	vmovapd		128(%r12, %r13, 2), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm18
	vmovapd		192(%r12, %r13, 2), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm19
	vmovapd		0+256(%r12, %r13, 2), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm20
	vmovapd		64+256(%r12, %r13, 2), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm21
	vmovapd		128+256(%r12, %r13, 2), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm22
	vmovapd		192+256(%r12, %r13, 2), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm23

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_24x8_lib8)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- 8*sdc*sizeof(double)
// r14   <- m1
// r15   <- n1
// zmm0 <- []
// ...
// zmm23 <- []
//
// output arguments:
// r10   <- alpha
// r11   <- beta
// r10   <- C
// r13   <- 8*sdc*sizeof(double)
// r14   <- m1
// r15   <- n1
// zmm0 <- []
// ...
// zmm23 <- []

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_24X8_VS_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_24x8_vs_lib8)
#endif
	
	// alpha
	vbroadcastsd	0(%r10), %zmm25

	vmulpd		%zmm0, %zmm25, %zmm0
	vmulpd		%zmm1, %zmm25, %zmm1
	vmulpd		%zmm2, %zmm25, %zmm2
	vmulpd		%zmm3, %zmm25, %zmm3
	vmulpd		%zmm4, %zmm25, %zmm4
	vmulpd		%zmm5, %zmm25, %zmm5
	vmulpd		%zmm6, %zmm25, %zmm6
	vmulpd		%zmm7, %zmm25, %zmm7
	vmulpd		%zmm8, %zmm25, %zmm8
	vmulpd		%zmm9, %zmm25, %zmm9
	vmulpd		%zmm10, %zmm25, %zmm10
	vmulpd		%zmm11, %zmm25, %zmm11
	vmulpd		%zmm12, %zmm25, %zmm12
	vmulpd		%zmm13, %zmm25, %zmm13
	vmulpd		%zmm14, %zmm25, %zmm14
	vmulpd		%zmm15, %zmm25, %zmm15
	vmulpd		%zmm16, %zmm25, %zmm16
	vmulpd		%zmm17, %zmm25, %zmm17
	vmulpd		%zmm18, %zmm25, %zmm18
	vmulpd		%zmm19, %zmm25, %zmm19
	vmulpd		%zmm20, %zmm25, %zmm20
	vmulpd		%zmm21, %zmm25, %zmm21
	vmulpd		%zmm22, %zmm25, %zmm22
	vmulpd		%zmm23, %zmm25, %zmm23

	// beta
	vbroadcastsd	0(%r11), %zmm24

	vxorpd		%zmm25, %zmm25, %zmm25 // 0.0

	vucomisd	%xmm25, %xmm24 // beta==0.0 ?
	je			0f // end

	// compute mask for rows
	vcvtsi2sd	%r14d, %xmm25, %xmm25
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC02(%rip), %zmm26
#elif defined(OS_MAC)
	vmovupd		LC02(%rip), %zmm26
#endif
	vbroadcastsd	%xmm25, %zmm25
	vsubpd		%zmm25, %zmm26, %zmm25
	vpmovq2m	%zmm25, %k1

	vmovapd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm0
	vmovapd		0(%r12, %r13), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm8
	vmovapd		0(%r12, %r13, 2), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm16 {%k1}
	cmpl	$ 2, %r15d
	jl		0f // end
	vmovapd		64(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm1
	vmovapd		64(%r12, %r13), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm9
	vmovapd		64(%r12, %r13, 2), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm17 {%k1}
	cmpl	$ 3, %r15d
	jl		0f // end
	vmovapd		128(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm2
	vmovapd		128(%r12, %r13), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm10
	vmovapd		128(%r12, %r13, 2), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm18 {%k1}
	cmpl	$ 4, %r15d
	jl		0f // end
	vmovapd		192(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm3
	vmovapd		192(%r12, %r13), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm11
	vmovapd		192(%r12, %r13, 2), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm19 {%k1}
	cmpl	$ 5, %r15d
	jl		0f // end
	vmovapd		0+256(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm4
	vmovapd		0+256(%r12, %r13), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm12
	vmovapd		0+256(%r12, %r13, 2), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm20 {%k1}
	cmpl	$ 6, %r15d
	jl		0f // end
	vmovapd		64+256(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm5
	vmovapd		64+256(%r12, %r13), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm13
	vmovapd		64+256(%r12, %r13, 2), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm21 {%k1}
	cmpl	$ 7, %r15d
	jl		0f // end
	vmovapd		128+256(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm6
	vmovapd		128+256(%r12, %r13), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm14
	vmovapd		128+256(%r12, %r13, 2), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm22 {%k1}
	je		0f // end
	vmovapd		192+256(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm7
	vmovapd		192+256(%r12, %r13), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm15
	vmovapd		192+256(%r12, %r13, 2), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm23 {%k1}

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_24x8_vs_lib8)
#endif





// common inner routine with file scope
//
// scale for alpha = -1.0 and beta
//
// input arguments:
// r10   <- beta
// r11   <- C
// r12   <- sdc
// zmm0 <- []
// ...
// zmm23 <- []
//
// output arguments:
// r10   <- beta
// r11   <- C
// r12   <- sdc
// zmm0 <- []
// ...
// zmm23 <- []

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M1B_24X8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_scale_m1b_24x8_lib8)
#endif	
	
	// beta
	vbroadcastsd	0(%r10), %zmm24

	vxorpd		%zmm25, %zmm25, %zmm25 // 0.0

	vucomisd	%xmm25, %xmm24 // beta==0.0 ?
	je			0f // end

	vmovapd		0(%r11), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm0
	vmovapd		1*64(%r11), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm1
	vmovapd		2*64(%r11), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm2
	vmovapd		3*64(%r11), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm3
	vmovapd		4*64(%r11), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm4
	vmovapd		5*64(%r11), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm5
	vmovapd		6*64(%r11), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm6
	vmovapd		7*64(%r11), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm7

	vmovapd		0(%r11, %r12), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm8
	vmovapd		1*64(%r11, %r12), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm9
	vmovapd		2*64(%r11, %r12), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm10
	vmovapd		3*64(%r11, %r12), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm11
	vmovapd		4*64(%r11, %r12), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm12
	vmovapd		5*64(%r11, %r12), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm13
	vmovapd		6*64(%r11, %r12), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm14
	vmovapd		7*64(%r11, %r12), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm15

	vmovapd		0(%r11, %r12, 2), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm16
	vmovapd		1*64(%r11, %r12, 2), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm17
	vmovapd		2*64(%r11, %r12, 2), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm18
	vmovapd		3*64(%r11, %r12, 2), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm19
	vmovapd		4*64(%r11, %r12, 2), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm20
	vmovapd		5*64(%r11, %r12, 2), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm21
	vmovapd		6*64(%r11, %r12, 2), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm22
	vmovapd		7*64(%r11, %r12, 2), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm23

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m1b_24x8_lib8)
#endif





// common inner routine with file scope
//
// scale for alpha = -1.0 and beta
//
// input arguments:
// r10   <- beta
// r11   <- C
// r12   <- sdc
// r13d  <- m1
// r14d  <- n1
// zmm0 <- []
// ...
// zmm23 <- []
//
// output arguments:
// r10   <- beta
// r11   <- C
// r12   <- sdc
// r13d  <- m1
// r14d  <- n1
// zmm0 <- []
// ...
// zmm23 <- []

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M1B_24X8_VS_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_scale_m1b_24x8_vs_lib8)
#endif	
	
	// beta
	vbroadcastsd	0(%r10), %zmm24

	vxorpd		%zmm25, %zmm25, %zmm25 // 0.0

	vucomisd	%xmm25, %xmm24 // beta==0.0 ?
	je			0f // end

	// compute mask for rows
	vcvtsi2sd	%r13d, %xmm25, %xmm25
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC02(%rip), %zmm26
#elif defined(OS_MAC)
	vmovupd		LC02(%rip), %zmm26
#endif
	vbroadcastsd	%xmm25, %zmm25
	vsubpd		%zmm25, %zmm26, %zmm25
	vpmovq2m	%zmm25, %k1

	vmovapd		0(%r11), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm0
	vmovapd		0(%r11, %r12), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm8
	vmovapd		0(%r11, %r12, 2), %zmm25 {%k1}{z}
	vfmsub231pd	%zmm24, %zmm25, %zmm16 {%k1}
	cmpl	$ 2, %r14d
	jl		0f // end
	vmovapd		1*64(%r11), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm1
	vmovapd		1*64(%r11, %r12), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm9
	vmovapd		1*64(%r11, %r12, 2), %zmm25 {%k1}{z}
	vfmsub231pd	%zmm24, %zmm25, %zmm17 {%k1}
	cmpl	$ 3, %r14d
	jl		0f // end
	vmovapd		2*64(%r11), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm2
	vmovapd		2*64(%r11, %r12), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm10
	vmovapd		2*64(%r11, %r12, 2), %zmm25 {%k1}{z}
	vfmsub231pd	%zmm24, %zmm25, %zmm18 {%k1}
	cmpl	$ 4, %r14d
	jl		0f // end
	vmovapd		3*64(%r11), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm3
	vmovapd		3*64(%r11, %r12), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm11
	vmovapd		3*64(%r11, %r12, 2), %zmm25 {%k1}{z}
	vfmsub231pd	%zmm24, %zmm25, %zmm19 {%k1}
	cmpl	$ 5, %r14d
	jl		0f // end
	vmovapd		4*64(%r11), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm4
	vmovapd		4*64(%r11, %r12), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm12
	vmovapd		4*64(%r11, %r12, 2), %zmm25 {%k1}{z}
	vfmsub231pd	%zmm24, %zmm25, %zmm20 {%k1}
	cmpl	$ 6, %r14d
	jl		0f // end
	vmovapd		5*64(%r11), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm5
	vmovapd		5*64(%r11, %r12), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm13
	vmovapd		5*64(%r11, %r12, 2), %zmm25 {%k1}{z}
	vfmsub231pd	%zmm24, %zmm25, %zmm21 {%k1}
	cmpl	$ 7, %r14d
	jl		0f // end
	vmovapd		6*64(%r11), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm6
	vmovapd		6*64(%r11, %r12), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm14
	vmovapd		6*64(%r11, %r12, 2), %zmm25 {%k1}{z}
	vfmsub231pd	%zmm24, %zmm25, %zmm22 {%k1}
	je		0f // end
	vmovapd		7*64(%r11), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm7
	vmovapd		7*64(%r11, %r12), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm15
	vmovapd		7*64(%r11, %r12, 2), %zmm25 {%k1}{z}
	vfmsub231pd	%zmm24, %zmm25, %zmm23 {%k1}

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m1b_24x8_vs_lib8)
#endif





// common inner routine with file scope
//
// scale for alpha = -1.0 and beta=1.0
//
// input arguments:
// r10   <- C
// r11   <- sdc
// zmm0 <- []
// ...
// zmm23 <- []
//
// output arguments:
// r10   <- C
// r11   <- sdc
// zmm0 <- []
// ...
// zmm23 <- []

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M11_24X8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_scale_m11_24x8_lib8)
#endif	
	
	vmovapd		0(%r10), %zmm24
	vsubpd		%zmm0, %zmm24, %zmm0
	vmovapd		1*64(%r10), %zmm24
	vsubpd		%zmm1, %zmm24, %zmm1
	vmovapd		2*64(%r10), %zmm24
	vsubpd		%zmm2, %zmm24, %zmm2
	vmovapd		3*64(%r10), %zmm24
	vsubpd		%zmm3, %zmm24, %zmm3
	vmovapd		4*64(%r10), %zmm24
	vsubpd		%zmm4, %zmm24, %zmm4
	vmovapd		5*64(%r10), %zmm24
	vsubpd		%zmm5, %zmm24, %zmm5
	vmovapd		6*64(%r10), %zmm24
	vsubpd		%zmm6, %zmm24, %zmm6
	vmovapd		7*64(%r10), %zmm24
	vsubpd		%zmm7, %zmm24, %zmm7

	vmovapd		0(%r10, %r11), %zmm24
	vsubpd		%zmm8, %zmm24, %zmm8
	vmovapd		1*64(%r10, %r11), %zmm24
	vsubpd		%zmm9, %zmm24, %zmm9
	vmovapd		2*64(%r10, %r11), %zmm24
	vsubpd		%zmm10, %zmm24, %zmm10
	vmovapd		3*64(%r10, %r11), %zmm24
	vsubpd		%zmm11, %zmm24, %zmm11
	vmovapd		4*64(%r10, %r11), %zmm24
	vsubpd		%zmm12, %zmm24, %zmm12
	vmovapd		5*64(%r10, %r11), %zmm24
	vsubpd		%zmm13, %zmm24, %zmm13
	vmovapd		6*64(%r10, %r11), %zmm24
	vsubpd		%zmm14, %zmm24, %zmm14
	vmovapd		7*64(%r10, %r11), %zmm24
	vsubpd		%zmm15, %zmm24, %zmm15

	vmovapd		0(%r10, %r11, 2), %zmm24
	vsubpd		%zmm16, %zmm24, %zmm16
	vmovapd		1*64(%r10, %r11, 2), %zmm24
	vsubpd		%zmm17, %zmm24, %zmm17
	vmovapd		2*64(%r10, %r11, 2), %zmm24
	vsubpd		%zmm18, %zmm24, %zmm18
	vmovapd		3*64(%r10, %r11, 2), %zmm24
	vsubpd		%zmm19, %zmm24, %zmm19
	vmovapd		4*64(%r10, %r11, 2), %zmm24
	vsubpd		%zmm20, %zmm24, %zmm20
	vmovapd		5*64(%r10, %r11, 2), %zmm24
	vsubpd		%zmm21, %zmm24, %zmm21
	vmovapd		6*64(%r10, %r11, 2), %zmm24
	vsubpd		%zmm22, %zmm24, %zmm22
	vmovapd		7*64(%r10, %r11, 2), %zmm24
	vsubpd		%zmm23, %zmm24, %zmm23

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m11_24x8_lib8)
#endif





// common inner routine with file scope
//
// scale for alpha = -1.0 and beta=1.0
//
// input arguments:
// r10   <- C
// r11   <- sdc
// r12   <- m1
// r13   <- n1
// zmm0 <- []
// ...
// zmm23 <- []
//
// output arguments:
// r10   <- C
// r11   <- sdc
// r12   <- m1
// r13   <- n1
// zmm0 <- []
// ...
// zmm23 <- []

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M11_24X8_VS_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_scale_m11_24x8_vs_lib8)
#endif	
	
	// compute mask for rows
	vcvtsi2sd	%r12d, %xmm25, %xmm25
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC02(%rip), %zmm26
#elif defined(OS_MAC)
	vmovupd		LC02(%rip), %zmm26
#endif
	vbroadcastsd	%xmm25, %zmm25
	vsubpd		%zmm25, %zmm26, %zmm25
	vpmovq2m	%zmm25, %k1

	vmovapd		0(%r10), %zmm24
	vsubpd		%zmm0, %zmm24, %zmm0
	vmovapd		0(%r10, %r11), %zmm24
	vsubpd		%zmm8, %zmm24, %zmm8
	vmovapd		0(%r10, %r11, 2), %zmm24 {%k1}{z}
	vsubpd		%zmm16, %zmm24, %zmm16 {%k1}
	cmpl	$ 2, %r13d
	jl		0f // end
	vmovapd		1*64(%r10), %zmm24
	vsubpd		%zmm1, %zmm24, %zmm1
	vmovapd		1*64(%r10, %r11), %zmm24
	vsubpd		%zmm9, %zmm24, %zmm9
	vmovapd		1*64(%r10, %r11, 2), %zmm24 {%k1}{z}
	vsubpd		%zmm17, %zmm24, %zmm17 {%k1}
	cmpl	$ 3, %r13d
	jl		0f // end
	vmovapd		2*64(%r10), %zmm24
	vsubpd		%zmm2, %zmm24, %zmm2
	vmovapd		2*64(%r10, %r11), %zmm24
	vsubpd		%zmm10, %zmm24, %zmm10
	vmovapd		2*64(%r10, %r11, 2), %zmm24 {%k1}{z}
	vsubpd		%zmm18, %zmm24, %zmm18 {%k1}
	cmpl	$ 4, %r13d
	jl		0f // end
	vmovapd		3*64(%r10), %zmm24
	vsubpd		%zmm3, %zmm24, %zmm3
	vmovapd		3*64(%r10, %r11), %zmm24
	vsubpd		%zmm11, %zmm24, %zmm11
	vmovapd		3*64(%r10, %r11, 2), %zmm24 {%k1}{z}
	vsubpd		%zmm19, %zmm24, %zmm19 {%k1}
	cmpl	$ 5, %r13d
	jl		0f // end
	vmovapd		4*64(%r10), %zmm24
	vsubpd		%zmm4, %zmm24, %zmm4
	vmovapd		4*64(%r10, %r11), %zmm24
	vsubpd		%zmm12, %zmm24, %zmm12
	vmovapd		4*64(%r10, %r11, 2), %zmm24 {%k1}{z}
	vsubpd		%zmm20, %zmm24, %zmm20 {%k1}
	cmpl	$ 6, %r13d
	jl		0f // end
	vmovapd		5*64(%r10), %zmm24
	vsubpd		%zmm5, %zmm24, %zmm5
	vmovapd		5*64(%r10, %r11), %zmm24
	vsubpd		%zmm13, %zmm24, %zmm13
	vmovapd		5*64(%r10, %r11, 2), %zmm24 {%k1}{z}
	vsubpd		%zmm21, %zmm24, %zmm21 {%k1}
	cmpl	$ 7, %r13d
	jl		0f // end
	vmovapd		6*64(%r10), %zmm24
	vsubpd		%zmm6, %zmm24, %zmm6
	vmovapd		6*64(%r10, %r11), %zmm24
	vsubpd		%zmm14, %zmm24, %zmm14
	vmovapd		6*64(%r10, %r11, 2), %zmm24 {%k1}{z}
	vsubpd		%zmm22, %zmm24, %zmm22 {%k1}
	je		0f // end
	vmovapd		7*64(%r10), %zmm24
	vsubpd		%zmm7, %zmm24, %zmm7
	vmovapd		7*64(%r10, %r11), %zmm24
	vsubpd		%zmm15, %zmm24, %zmm15
	vmovapd		7*64(%r10, %r11, 2), %zmm24 {%k1}{z}
	vsubpd		%zmm23, %zmm24, %zmm23 {%k1}

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m11_24x8_vs_lib8)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// r11  <- 8*sdd*sizeof(double)
// zmm0 <- []
// ...
// zmm23 <- []
//
// output arguments:
// r10  <- D
// r11  <- 8*sdd*sizeof(double)
// zmm0 <- []
// ...
// zmm23 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_24X8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_store_24x8_lib8)
#endif
	
	vmovapd %zmm0,   0(%r10)
	vmovapd %zmm1,  64(%r10)
	vmovapd %zmm2, 128(%r10)
	vmovapd %zmm3, 192(%r10)
	vmovapd %zmm4,   0+256(%r10)
	vmovapd %zmm5,  64+256(%r10)
	vmovapd %zmm6, 128+256(%r10)
	vmovapd %zmm7, 192+256(%r10)

	vmovapd %zmm8,   0(%r10, %r11)
	vmovapd %zmm9,  64(%r10, %r11)
	vmovapd %zmm10, 128(%r10, %r11)
	vmovapd %zmm11, 192(%r10, %r11)
	vmovapd %zmm12,   0+256(%r10, %r11)
	vmovapd %zmm13,  64+256(%r10, %r11)
	vmovapd %zmm14, 128+256(%r10, %r11)
	vmovapd %zmm15, 192+256(%r10, %r11)

	vmovapd %zmm16,   0(%r10, %r11, 2)
	vmovapd %zmm17,  64(%r10, %r11, 2)
	vmovapd %zmm18, 128(%r10, %r11, 2)
	vmovapd %zmm19, 192(%r10, %r11, 2)
	vmovapd %zmm20,   0+256(%r10, %r11, 2)
	vmovapd %zmm21,  64+256(%r10, %r11, 2)
	vmovapd %zmm22, 128+256(%r10, %r11, 2)
	vmovapd %zmm23, 192+256(%r10, %r11, 2)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_24x8_lib8)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// r11  <- 8*sdd*sizeof(double)
// r12  <- m1
// r13  <- n1
// zmm0 <- []
// ...
// zmm23 <- []
//
// output arguments:
// r10  <- D
// r11  <- 8*sdd*sizeof(double)
// r12  <- m1
// r13  <- n1
// zmm0 <- []
// ...
// zmm23 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_24X8_VS_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_store_24x8_vs_lib8)
#endif
	
	// compute mask for rows
	vcvtsi2sd	%r12d, %xmm25, %xmm25
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC02(%rip), %zmm24
#elif defined(OS_MAC)
	vmovupd		LC02(%rip), %zmm24
#endif
	vbroadcastsd	%xmm25, %zmm25
	vsubpd		%zmm25, %zmm24, %zmm25

	vpmovq2m	%zmm25, %k1

	vmovapd %zmm0,   0(%r10)
	vmovapd %zmm8,   0(%r10, %r11)
	vmovapd %zmm16,  0(%r10, %r11, 2) {%k1}
	cmpl	$ 2, %r13d
	jl		0f // end
	vmovapd %zmm1,  64(%r10)
	vmovapd %zmm9,  64(%r10, %r11)
	vmovapd %zmm17, 64(%r10, %r11, 2) {%k1}
	cmpl	$ 3, %r13d
	jl		0f // end
	vmovapd %zmm2, 128(%r10)
	vmovapd %zmm10, 128(%r10, %r11)
	vmovapd %zmm18, 128(%r10, %r11, 2) {%k1}
	cmpl	$ 4, %r13d
	jl		0f // end
	vmovapd %zmm3, 192(%r10)
	vmovapd %zmm11, 192(%r10, %r11)
	vmovapd %zmm19, 192(%r10, %r11, 2) {%k1}
	cmpl	$ 5, %r13d
	jl		0f // end
	vmovapd %zmm4,   0+256(%r10)
	vmovapd %zmm12,   0+256(%r10, %r11)
	vmovapd %zmm20,   0+256(%r10, %r11, 2) {%k1}
	cmpl	$ 6, %r13d
	jl		0f // end
	vmovapd %zmm5,  64+256(%r10)
	vmovapd %zmm13,  64+256(%r10, %r11)
	vmovapd %zmm21,  64+256(%r10, %r11, 2) {%k1}
	cmpl	$ 7, %r13d
	jl		0f // end
	vmovapd %zmm6, 128+256(%r10)
	vmovapd %zmm14, 128+256(%r10, %r11)
	vmovapd %zmm22, 128+256(%r10, %r11, 2) {%k1}
	je		0f // end
	vmovapd %zmm7, 192+256(%r10)
	vmovapd %zmm15, 192+256(%r10, %r11)
	vmovapd %zmm23, 192+256(%r10, %r11, 2) {%k1}

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_24x8_vs_lib8)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// r11  <- 8*sdd*sizeof(double)
// zmm0 <- []
// ...
// zmm23 <- []
//
// output arguments:
// r10  <- D
// r11  <- 8*sdd*sizeof(double)
// zmm0 <- []
// ...
// zmm23 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_24X8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_store_l_24x8_lib8)
#endif
	
	//
	vmovapd %zmm0,   0(%r10)
	//
	movl	$ 0xfe, %r12d
	kmovd	%r12d, %k1
	vmovapd %zmm1,  64(%r10) {%k1}
	//
	movl	$ 0xfc, %r12d
	kmovd	%r12d, %k1
	vmovapd %zmm2, 128(%r10) {%k1}
	//
	movl	$ 0xf8, %r12d
	kmovd	%r12d, %k1
	vmovapd %zmm3, 192(%r10) {%k1}
	//
	movl	$ 0xf0, %r12d
	kmovd	%r12d, %k1
	vmovapd %zmm4,   0+256(%r10) {%k1}
	//
	movl	$ 0xe0, %r12d
	kmovd	%r12d, %k1
	vmovapd %zmm5,  64+256(%r10) {%k1}
	//
	movl	$ 0xc0, %r12d
	kmovd	%r12d, %k1
	vmovapd %zmm6, 128+256(%r10) {%k1}
	//
	movl	$ 0x80, %r12d
	kmovd	%r12d, %k1
	vmovapd %zmm7, 192+256(%r10) {%k1}

	vmovapd %zmm8,   0(%r10, %r11)
	vmovapd %zmm9,  64(%r10, %r11)
	vmovapd %zmm10, 128(%r10, %r11)
	vmovapd %zmm11, 192(%r10, %r11)
	vmovapd %zmm12,   0+256(%r10, %r11)
	vmovapd %zmm13,  64+256(%r10, %r11)
	vmovapd %zmm14, 128+256(%r10, %r11)
	vmovapd %zmm15, 192+256(%r10, %r11)

	vmovapd %zmm16,   0(%r10, %r11, 2)
	vmovapd %zmm17,  64(%r10, %r11, 2)
	vmovapd %zmm18, 128(%r10, %r11, 2)
	vmovapd %zmm19, 192(%r10, %r11, 2)
	vmovapd %zmm20,   0+256(%r10, %r11, 2)
	vmovapd %zmm21,  64+256(%r10, %r11, 2)
	vmovapd %zmm22, 128+256(%r10, %r11, 2)
	vmovapd %zmm23, 192+256(%r10, %r11, 2)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_24x8_lib8)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// r11  <- 8*sdd*sizeof(double)
// r12  <- m1
// r13  <- n1
// zmm0 <- []
// ...
// zmm23 <- []
//
// output arguments:
// r10  <- D
// r11  <- 8*sdd*sizeof(double)
// r12  <- m1
// r13  <- n1
// zmm0 <- []
// ...
// zmm23 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_24X8_VS_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_store_l_24x8_vs_lib8)
#endif
	
	// compute mask for rows
	vcvtsi2sd	%r12d, %xmm25, %xmm25
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC02(%rip), %zmm24
#elif defined(OS_MAC)
	vmovupd		LC02(%rip), %zmm24
#endif
	vbroadcastsd	%xmm25, %zmm25
	vsubpd		%zmm25, %zmm24, %zmm25

	vpmovq2m	%zmm25, %k1

	vmovapd %zmm0,   0(%r10)
	vmovapd %zmm8,   0(%r10, %r11)
	vmovapd %zmm16,   0(%r10, %r11, 2) {%k1}
	cmpl	$ 2, %r13d
	jl		0f // end
	movl	$ 0xfe, %r14d
	kmovd	%r14d, %k2
	vmovapd %zmm1,  64(%r10) {%k2}
	vmovapd %zmm9,  64(%r10, %r11)
	vmovapd %zmm17,  64(%r10, %r11, 2) {%k1}
	cmpl	$ 3, %r13d
	jl		0f // end
	movl	$ 0xfc, %r14d
	kmovd	%r14d, %k2
	vmovapd %zmm2, 128(%r10) {%k2}
	vmovapd %zmm10, 128(%r10, %r11)
	vmovapd %zmm18, 128(%r10, %r11, 2) {%k1}
	cmpl	$ 4, %r13d
	jl		0f // end
	movl	$ 0xf8, %r14d
	kmovd	%r14d, %k2
	vmovapd %zmm3, 192(%r10) {%k2}
	vmovapd %zmm11, 192(%r10, %r11)
	vmovapd %zmm19, 192(%r10, %r11, 2) {%k1}
	cmpl	$ 5, %r13d
	jl		0f // end
	movl	$ 0xf0, %r14d
	kmovd	%r14d, %k2
	vmovapd %zmm4,   0+256(%r10) {%k2}
	vmovapd %zmm12,   0+256(%r10, %r11)
	vmovapd %zmm20,   0+256(%r10, %r11, 2) {%k1}
	cmpl	$ 6, %r13d
	jl		0f // end
	movl	$ 0xe0, %r14d
	kmovd	%r14d, %k2
	vmovapd %zmm5,  64+256(%r10) {%k2}
	vmovapd %zmm13,  64+256(%r10, %r11)
	vmovapd %zmm21,  64+256(%r10, %r11, 2) {%k1}
	cmpl	$ 7, %r13d
	jl		0f // end
	movl	$ 0xc0, %r14d
	kmovd	%r14d, %k2
	vmovapd %zmm6, 128+256(%r10) {%k2}
	vmovapd %zmm14, 128+256(%r10, %r11)
	vmovapd %zmm22, 128+256(%r10, %r11, 2) {%k1}
	cmpl	$ 7, %r13d
	je		0f // end
	movl	$ 0x80, %r14d
	kmovd	%r14d, %k2
	vmovapd %zmm7, 192+256(%r10) {%k2}
	vmovapd %zmm15, 192+256(%r10, %r11)
	vmovapd %zmm23, 192+256(%r10, %r11, 2) {%k1}

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_24x8_vs_lib8)
#endif





// common inner routine with file scope
//
// transpose
//
// input arguments:
// zmm0  <- []
// ...
// zmm15  <- []
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_TRAN_24X8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_tran_24x8_lib8)
#endif

	movl	$ 0xcc, %r10d
	kmovd	%r10d, %k1
	movl	$ 0x33, %r10d
	kmovd	%r10d, %k2


	vunpcklpd	%zmm1, %zmm0, %zmm24
	vunpckhpd	%zmm1, %zmm0, %zmm25
	vunpcklpd	%zmm3, %zmm2, %zmm26
	vunpckhpd	%zmm3, %zmm2, %zmm27
	vunpcklpd	%zmm5, %zmm4, %zmm28
	vunpckhpd	%zmm5, %zmm4, %zmm29
	vunpcklpd	%zmm7, %zmm6, %zmm30
	vunpckhpd	%zmm7, %zmm6, %zmm31

	vmovapd		%zmm24, %zmm0
	vpermq		$ 0x40, %zmm26, %zmm24 {%k1}
	vpermq		$ 0x0e, %zmm0, %zmm26 {%k2}
	vmovapd		%zmm25, %zmm0
	vpermq		$ 0x40, %zmm27, %zmm25 {%k1}
	vpermq		$ 0x0e, %zmm0, %zmm27 {%k2}
	vmovapd		%zmm28, %zmm0
	vpermq		$ 0x40, %zmm30, %zmm28 {%k1}
	vpermq		$ 0x0e, %zmm0, %zmm30 {%k2}
	vmovapd		%zmm29, %zmm0
	vpermq		$ 0x40, %zmm31, %zmm29 {%k1}
	vpermq		$ 0x0e, %zmm0, %zmm31 {%k2}

	vshuff64x2	$ 0x44, %zmm28, %zmm24, %zmm0
	vshuff64x2	$ 0xee, %zmm28, %zmm24, %zmm4
	vshuff64x2	$ 0x44, %zmm29, %zmm25, %zmm1
	vshuff64x2	$ 0xee, %zmm29, %zmm25, %zmm5
	vshuff64x2	$ 0x44, %zmm30, %zmm26, %zmm2
	vshuff64x2	$ 0xee, %zmm30, %zmm26, %zmm6
	vshuff64x2	$ 0x44, %zmm31, %zmm27, %zmm3
	vshuff64x2	$ 0xee, %zmm31, %zmm27, %zmm7


	vunpcklpd	%zmm9, %zmm8, %zmm24
	vunpckhpd	%zmm9, %zmm8, %zmm25
	vunpcklpd	%zmm11, %zmm10, %zmm26
	vunpckhpd	%zmm11, %zmm10, %zmm27
	vunpcklpd	%zmm13, %zmm12, %zmm28
	vunpckhpd	%zmm13, %zmm12, %zmm29
	vunpcklpd	%zmm15, %zmm14, %zmm30
	vunpckhpd	%zmm15, %zmm14, %zmm31

	vmovapd		%zmm24, %zmm8
	vpermq		$ 0x40, %zmm26, %zmm24 {%k1}
	vpermq		$ 0x0e, %zmm8, %zmm26 {%k2}
	vmovapd		%zmm25, %zmm8
	vpermq		$ 0x40, %zmm27, %zmm25 {%k1}
	vpermq		$ 0x0e, %zmm8, %zmm27 {%k2}
	vmovapd		%zmm28, %zmm8
	vpermq		$ 0x40, %zmm30, %zmm28 {%k1}
	vpermq		$ 0x0e, %zmm8, %zmm30 {%k2}
	vmovapd		%zmm29, %zmm8
	vpermq		$ 0x40, %zmm31, %zmm29 {%k1}
	vpermq		$ 0x0e, %zmm8, %zmm31 {%k2}

	vshuff64x2	$ 0x44, %zmm28, %zmm24, %zmm8
	vshuff64x2	$ 0xee, %zmm28, %zmm24, %zmm12
	vshuff64x2	$ 0x44, %zmm29, %zmm25, %zmm9
	vshuff64x2	$ 0xee, %zmm29, %zmm25, %zmm13
	vshuff64x2	$ 0x44, %zmm30, %zmm26, %zmm10
	vshuff64x2	$ 0xee, %zmm30, %zmm26, %zmm14
	vshuff64x2	$ 0x44, %zmm31, %zmm27, %zmm11
	vshuff64x2	$ 0xee, %zmm31, %zmm27, %zmm15


	vunpcklpd	%zmm17, %zmm16, %zmm24
	vunpckhpd	%zmm17, %zmm16, %zmm25
	vunpcklpd	%zmm19, %zmm18, %zmm26
	vunpckhpd	%zmm19, %zmm18, %zmm27
	vunpcklpd	%zmm21, %zmm20, %zmm28
	vunpckhpd	%zmm21, %zmm20, %zmm29
	vunpcklpd	%zmm23, %zmm22, %zmm30
	vunpckhpd	%zmm23, %zmm22, %zmm31

	vmovapd		%zmm24, %zmm16
	vpermq		$ 0x40, %zmm26, %zmm24 {%k1}
	vpermq		$ 0x0e, %zmm16, %zmm26 {%k2}
	vmovapd		%zmm25, %zmm16
	vpermq		$ 0x40, %zmm27, %zmm25 {%k1}
	vpermq		$ 0x0e, %zmm16, %zmm27 {%k2}
	vmovapd		%zmm28, %zmm16
	vpermq		$ 0x40, %zmm30, %zmm28 {%k1}
	vpermq		$ 0x0e, %zmm16, %zmm30 {%k2}
	vmovapd		%zmm29, %zmm16
	vpermq		$ 0x40, %zmm31, %zmm29 {%k1}
	vpermq		$ 0x0e, %zmm16, %zmm31 {%k2}

	vshuff64x2	$ 0x44, %zmm28, %zmm24, %zmm16
	vshuff64x2	$ 0xee, %zmm28, %zmm24, %zmm20
	vshuff64x2	$ 0x44, %zmm29, %zmm25, %zmm17
	vshuff64x2	$ 0xee, %zmm29, %zmm25, %zmm21
	vshuff64x2	$ 0x44, %zmm30, %zmm26, %zmm18
	vshuff64x2	$ 0xee, %zmm30, %zmm26, %zmm22
	vshuff64x2	$ 0x44, %zmm31, %zmm27, %zmm19
	vshuff64x2	$ 0xee, %zmm31, %zmm27, %zmm23

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_tran_24x8_lib8)
#endif





//                                1      2              3          4        5          6             7          8        9          10
// void kernel_dgemm_nt_24x8_lib8(int k, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_24x8_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 6, %r12d // 8*sda*sizeof(double)
	movq	ARG5, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_24X8_LIB8
#else
	CALL(inner_kernel_dgemm_nt_24x8_lib8)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12 // C
	movq	ARG8, %r13 // sdc
	sall	$ 6, %r13d // 8*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_24X8_LIB8
#else
	CALL(inner_scale_ab_24x8_lib8)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // sdd
	sall	$ 6, %r11d // 8*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_24X8_LIB8
#else
	CALL(inner_store_24x8_lib8)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dgemm_nt_24x8_lib8)





//                                   1      2              3          4        5          6             7          8        9          10       11      12
// void kernel_dgemm_nt_24x8_vs_lib8(int k, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_24x8_vs_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 6, %r12d // 8*sda*sizeof(double)
	movq	ARG5, %r13 // B
	movq	ARG11, %r14 // m1
	movq	ARG12, %r15 // n1

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_24X8_VS_LIB8
#else
	CALL(inner_kernel_dgemm_nt_24x8_vs_lib8)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12 // C
	movq	ARG8, %r13 // sdc
	sall	$ 6, %r13d // 8*sdc*sizeof(double)
	movq	ARG11, %r14 // m1
	movq	ARG12, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_24X8_VS_LIB8
#else
	CALL(inner_scale_ab_24x8_vs_lib8)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // sdd
	sall	$ 6, %r11d // 8*sdd*sizeof(double)
	movq	ARG11, %r12 // m1
	movq	ARG12, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_24X8_VS_LIB8
#else
	CALL(inner_store_24x8_vs_lib8)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dgemm_nt_24x8_vs_lib8)





//                                       1      2          3        4          5             6          7        8          9        10         11
// void kernel_dtrsm_nt_rl_inv_24x8_lib8(int k, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd, double *E, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_inv_24x8_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 6, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_24X8_LIB8
#else
	CALL(inner_kernel_dgemm_nt_24x8_lib8)
#endif


	// call inner blender nn

	movq	ARG5, %r10 // beta
	movq	ARG6, %r11 // C
	movq	ARG7, %r12 // sdc
	sall	$ 6, %r12d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_24X8_LIB8
#else
	CALL(inner_scale_m1b_24x8_lib8)
#endif


	// solve

	movq	ARG10, %r10  // E 
	movq	ARG11, %r11  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_24X8_LIB8
#else
	CALL(inner_edge_dtrsm_rlt_inv_24x8_lib8)
#endif


	// store n

	movq	ARG8, %r10 // store address D
	movq	ARG9, %r11 // sdd
	sall	$ 6, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_24X8_LIB8
#else
	CALL(inner_store_24x8_lib8)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_24x8_lib8)





//                                          1      2          3        4          5             6          7        8          9        10         11                  12      13
// void kernel_dtrsm_nt_rl_inv_24x8_vs_lib8(int k, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd, double *E, double *inv_diag_E, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_inv_24x8_vs_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 6, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B
	movq	ARG12, %r14 // m1
	movq	ARG13, %r15 // n1

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_24X8_VS_LIB8
#else
	CALL(inner_kernel_dgemm_nt_24x8_vs_lib8)
#endif


	// call inner blender nn

	movq	ARG5, %r10 // beta
	movq	ARG6, %r11 // C
	movq	ARG7, %r12 // sdc
	sall	$ 6, %r12d // 4*sdc*sizeof(double)
	movq	ARG12, %r13 // m1
	movq	ARG13, %r14 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_24X8_VS_LIB8
#else
	CALL(inner_scale_m1b_24x8_vs_lib8)
#endif


	// solve

	movq	ARG10, %r10  // E 
	movq	ARG11, %r11  // inv_diag_E 
	movq	ARG13, %r12 // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_24X8_VS_LIB8
#else
	CALL(inner_edge_dtrsm_rlt_inv_24x8_vs_lib8)
#endif


	// store n

	movq	ARG8, %r10 // store address D
	movq	ARG9, %r11 // sdd
	sall	$ 6, %r11d // 4*sdd*sizeof(double)
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_24X8_VS_LIB8
#else
	CALL(inner_store_24x8_vs_lib8)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_24x8_vs_lib8)





//                                             1       2           3         4           5       6           7         8           9          10       11         12       13         14
// void kernel_dgemm_dtrsm_nt_rl_inv_24x8_lib8(int kp, double *Ap, int sdap, double *Bp, int km, double *Am, int sdam, double *Bm, double *C, int sdc, double *D, int sdd, double *E, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_dtrsm_nt_rl_inv_24x8_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG2, %r11  // Ap
	movq	ARG3, %r12 // sdap
	sall	$ 6, %r12d   // 4*sdap*sizeof(double)
	movq	ARG4, %r13  // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_24X8_LIB8
#else
	CALL(inner_kernel_dgemm_nt_24x8_lib8)
#endif


	// change sign
	NEG_ACC


	// call inner dgemm kernel nt sub

	movq	ARG5, %r10 // km
	movq	ARG6, %r11 // Am
	movq	ARG7, %r12 // sdam
	sall	$ 6, %r12d // 4*sda*sizeof(double)
	movq	ARG8, %r13  // Bm

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_24X8_LIB8
#else
	CALL(inner_kernel_dgemm_nt_24x8_lib8)
#endif


	// call inner blender nn

	movq	ARG9, %r10  // C
	movq	ARG10, %r11 // sdc
	sall	$ 6, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_24X8_LIB8
#else
	CALL(inner_scale_m11_24x8_lib8)
#endif


	// solve

	movq	ARG13, %r10  // E 
	movq	ARG14, %r11  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_24X8_LIB8
#else
	CALL(inner_edge_dtrsm_rlt_inv_24x8_lib8)
#endif


	// store n

	movq	ARG11, %r10 // store address D
	movq	ARG12, %r11 // sdd
	sall	$ 6, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_24X8_LIB8
#else
	CALL(inner_store_24x8_lib8)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dgemm_dtrsm_nt_rl_inv_24x8_lib8)





//                                                1       2           3         4           5       6           7         8           9          10       11         12       13         14                  15      16
// void kernel_dgemm_dtrsm_nt_rl_inv_24x8_vs_lib8(int kp, double *Ap, int sdap, double *Bp, int km, double *Am, int sdam, double *Bm, double *C, int sdc, double *D, int sdd, double *E, double *inv_diag_E, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_dtrsm_nt_rl_inv_24x8_vs_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG2, %r11  // Ap
	movq	ARG3, %r12 // sdap
	sall	$ 6, %r12d   // 4*sdap*sizeof(double)
	movq	ARG4, %r13  // Bp
	movq	ARG15, %r14 // m1
	movq	ARG16, %r15 // n1

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_24X8_VS_LIB8
#else
	CALL(inner_kernel_dgemm_nt_24x8_vs_lib8)
#endif


	// change sign
	NEG_ACC


	// call inner dgemm kernel nt sub

	movq	ARG5, %r10 // km
	movq	ARG6, %r11 // Am
	movq	ARG7, %r12 // sdam
	sall	$ 6, %r12d // 4*sda*sizeof(double)
	movq	ARG8, %r13  // Bm
	movq	ARG15, %r14 // m1
	movq	ARG16, %r15 // n1

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_24X8_VS_LIB8
#else
	CALL(inner_kernel_dgemm_nt_24x8_vs_lib8)
#endif


	// call inner blender nn

	movq	ARG9, %r10  // C
	movq	ARG10, %r11 // sdc
	sall	$ 6, %r11d // 4*sdc*sizeof(double)
	movq	ARG15, %r12 // m1
	movq	ARG16, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_24X8_VS_LIB8
#else
	CALL(inner_scale_m11_24x8_vs_lib8)
#endif


	// solve

	movq	ARG13, %r10  // E 
	movq	ARG14, %r11  // inv_diag_E 
	movq	ARG16, %r12 // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_24X8_VS_LIB8
#else
	CALL(inner_edge_dtrsm_rlt_inv_24x8_vs_lib8)
#endif


	// store n

	movq	ARG11, %r10 // store address D
	movq	ARG12, %r11 // sdd
	sall	$ 6, %r11d // 4*sdd*sizeof(double)
	movq	ARG15, %r12 // m1
	movq	ARG16, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_24X8_VS_LIB8
#else
	CALL(inner_store_24x8_vs_lib8)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dgemm_dtrsm_nt_rl_inv_24x8_vs_lib8)





//                                   1      2          3        4          5          6        7          8        9
// void kernel_dpotrf_nt_l_24x8_lib8(int k, double *A, int sda, double *B, double *C, int sdc, double *D, int sdd, double *inv_diag_D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpotrf_nt_l_24x8_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	sall	$ 6, %r12d // 8*sda*sizeof(double)
	movq	ARG4, %r13

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_24X8_LIB8
#else
	CALL(inner_kernel_dgemm_nt_24x8_lib8)
#endif


	// call inner blender nn

	movq	ARG5, %r10 // C
	movq	ARG6, %r11 // sdc
	sall	$ 6, %r11d // 8*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_24X8_LIB8
#else
	CALL(inner_scale_m11_24x8_lib8)
#endif


	// factorization

	movq	ARG9, %r10  // inv_diag_D 

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_24X8_LIB8
#else
	CALL(inner_edge_dpotrf_24x8_lib8)
#endif


	// store n

	movq	ARG7, %r10 // store address D
	movq	ARG8, %r11 // sdd
	sall	$ 6, %r11d // 8*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_L_24X8_LIB8
#else
	CALL(inner_store_l_24x8_lib8)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dpotrf_nt_l_24x8_lib8)





//                                      1      2          3        4          5          6        7          8        9                   10      11
// void kernel_dpotrf_nt_l_24x8_vs_lib8(int k, double *A, int sda, double *B, double *C, int sdc, double *D, int sdd, double *inv_diag_D, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpotrf_nt_l_24x8_vs_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	sall	$ 6, %r12d // 8*sda*sizeof(double)
	movq	ARG4, %r13
	movq	ARG10, %r14 // m1
	movq	ARG11, %r15 // n1

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_24X8_VS_LIB8
#else
	CALL(inner_kernel_dgemm_nt_24x8_vs_lib8)
#endif


	// call inner blender nn

	movq	ARG5, %r10 // C
	movq	ARG6, %r11 // sdc
	sall	$ 6, %r11d // 8*sdc*sizeof(double)
	movq	ARG10, %r12 // m1
	movq	ARG11, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_24X8_VS_LIB8
#else
	CALL(inner_scale_m11_24x8_vs_lib8)
#endif


	// factorization

	movq	ARG9, %r10  // inv_diag_D 
	movq	ARG10, %r11 // m1
	movq	ARG11, %r12 // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_24X8_VS_LIB8
#else
	CALL(inner_edge_dpotrf_24x8_vs_lib8)
#endif


	// store n

	movq	ARG7, %r10 // store address D
	movq	ARG8, %r11 // sdd
	sall	$ 6, %r11d // 8*sdd*sizeof(double)
	movq	ARG10, %r12 // m1
	movq	ARG11, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_L_24X8_VS_LIB8
#else
	CALL(inner_store_l_24x8_vs_lib8)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dpotrf_nt_l_24x8_vs_lib8)





//                                         1       2           3         4           5       6           7         8           9          10       11         12       13
// void kernel_dsyrk_dpotrf_nt_l_24x8_lib8(int kp, double *Ap, int sdap, double *Bp, int km, double *Am, int sdam, double *Bm, double *C, int sdc, double *D, int sdd, double *inv_diag_D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_dpotrf_nt_l_24x8_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG2, %r11  // Ap
	movq	ARG3, %r12 // sdap
	sall	$ 6, %r12d   // 4*sdap*sizeof(double)
	movq	ARG4, %r13  // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_24X8_LIB8
#else
	CALL(inner_kernel_dgemm_nt_24x8_lib8)
#endif


	// change sign
	NEG_ACC


	// call inner dgemm kernel nt sub

	movq	ARG5, %r10 // km
	movq	ARG6, %r11 // Am
	movq	ARG7, %r12 // sdam
	sall	$ 6, %r12d // 4*sdam*sizeof(double)
	movq	ARG8, %r13  // Bm

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_24X8_LIB8
#else
	CALL(inner_kernel_dgemm_nt_24x8_lib8)
#endif


	// call inner blender nn

	movq	ARG9, %r10 // C
	movq	ARG10, %r11 // sdc
	sall	$ 6, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_24X8_LIB8
#else
	CALL(inner_scale_m11_24x8_lib8)
#endif


	// factorization

	movq	ARG13, %r10  // inv_diag_D 

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_24X8_LIB8
#else
	CALL(inner_edge_dpotrf_24x8_lib8)
#endif


	// store n

	movq	ARG11, %r10 // store address D
	movq	ARG12, %r11 // sdd
	sall	$ 6, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_L_24X8_LIB8
#else
	CALL(inner_store_l_24x8_lib8)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dsyrk_dpotrf_nt_l_24x8_lib8)





//                                            1       2           3         4           5       6           7         8           9          10       11         12       13                  14      15
// void kernel_dsyrk_dpotrf_nt_l_24x8_vs_lib8(int kp, double *Ap, int sdap, double *Bp, int km, double *Am, int sdam, double *Bm, double *C, int sdc, double *D, int sdd, double *inv_diag_D, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_dpotrf_nt_l_24x8_vs_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG2, %r11  // Ap
	movq	ARG3, %r12 // sdap
	sall	$ 6, %r12d   // 4*sdap*sizeof(double)
	movq	ARG4, %r13  // Bp
	movq	ARG14, %r14 // m1
	movq	ARG15, %r15 // n1

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_24X8_VS_LIB8
#else
	CALL(inner_kernel_dgemm_nt_24x8_vs_lib8)
#endif


	// change sign
	NEG_ACC


	// call inner dgemm kernel nt sub

	movq	ARG5, %r10 // km
	movq	ARG6, %r11 // Am
	movq	ARG7, %r12 // sdam
	sall	$ 6, %r12d // 4*sdam*sizeof(double)
	movq	ARG8, %r13  // Bm
	movq	ARG14, %r14 // m1
	movq	ARG15, %r15 // n1

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_24X8_VS_LIB8
#else
	CALL(inner_kernel_dgemm_nt_24x8_vs_lib8)
#endif


	// call inner blender nn

	movq	ARG9, %r10 // C
	movq	ARG10, %r11 // sdc
	sall	$ 6, %r11d // 4*sdc*sizeof(double)
	movq	ARG14, %r12 // m1
	movq	ARG15, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_24X8_VS_LIB8
#else
	CALL(inner_scale_m11_24x8_vs_lib8)
#endif


	// factorization

	movq	ARG13, %r10  // inv_diag_D 
	movq	ARG14, %r11 // m1
	movq	ARG15, %r12 // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_24X8_VS_LIB8
#else
	CALL(inner_edge_dpotrf_24x8_vs_lib8)
#endif


	// store n

	movq	ARG11, %r10 // store address D
	movq	ARG12, %r11 // sdd
	sall	$ 6, %r11d // 4*sdd*sizeof(double)
	movq	ARG14, %r12 // m1
	movq	ARG15, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_L_24X8_VS_LIB8
#else
	CALL(inner_store_l_24x8_vs_lib8)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_dsyrk_dpotrf_nt_l_24x8_vs_lib8)





//                                1         2           3           4           5
// void kernel_dlarfb8_rn_24_lib8(int kmax, double *pV, double *pT, double *pD, int sdd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dlarfb8_rn_24_lib8)
	
	PROLOGUE

	// zero accumulation registers

//	ZERO_ACC

	movq	ARG1, %r10 // k
	movq	ARG4, %r11 // D
	movq	ARG5, %r12 // sdd
	sall	$ 6, %r12d
	movq	ARG2, %r13 // V

	//
	vmovapd			0*64(%r11), %zmm0
	vmovapd			0*64(%r11, %r12), %zmm8
	vmovapd			0*64(%r11, %r12, 2), %zmm16
	//
	vmovapd			1*64(%r11), %zmm1
	vmovapd			1*64(%r11, %r12), %zmm9
	vmovapd			1*64(%r11, %r12, 2), %zmm17
	vbroadcastsd	1*64(%r13), %zmm24
	vfmadd231pd		%zmm24, %zmm1, %zmm0
	vfmadd231pd		%zmm24, %zmm9, %zmm8
	vfmadd231pd		%zmm24, %zmm17, %zmm16
	//
	vmovapd			2*64(%r11), %zmm2
	vmovapd			2*64(%r11, %r12), %zmm10
	vmovapd			2*64(%r11, %r12, 2), %zmm18
	vbroadcastsd	0+2*64(%r13), %zmm24
	vfmadd231pd		%zmm24, %zmm2, %zmm0
	vfmadd231pd		%zmm24, %zmm10, %zmm8
	vfmadd231pd		%zmm24, %zmm18, %zmm16
	vbroadcastsd	8+2*64(%r13), %zmm24
	vfmadd231pd		%zmm24, %zmm2, %zmm1
	vfmadd231pd		%zmm24, %zmm10, %zmm9
	vfmadd231pd		%zmm24, %zmm18, %zmm17
	//
	vmovapd			3*64(%r11), %zmm3
	vmovapd			3*64(%r11, %r12), %zmm11
	vmovapd			3*64(%r11, %r12, 2), %zmm19
	vbroadcastsd	0+3*64(%r13), %zmm24
	vfmadd231pd		%zmm24, %zmm3, %zmm0
	vfmadd231pd		%zmm24, %zmm11, %zmm8
	vfmadd231pd		%zmm24, %zmm19, %zmm16
	vbroadcastsd	8+3*64(%r13), %zmm24
	vfmadd231pd		%zmm24, %zmm3, %zmm1
	vfmadd231pd		%zmm24, %zmm11, %zmm9
	vfmadd231pd		%zmm24, %zmm19, %zmm17
	vbroadcastsd	16+3*64(%r13), %zmm24
	vfmadd231pd		%zmm24, %zmm3, %zmm2
	vfmadd231pd		%zmm24, %zmm11, %zmm10
	vfmadd231pd		%zmm24, %zmm19, %zmm18
	//
	vmovapd			4*64(%r11), %zmm4
	vmovapd			4*64(%r11, %r12), %zmm12
	vmovapd			4*64(%r11, %r12, 2), %zmm20
	vbroadcastsd	0+4*64(%r13), %zmm24
	vfmadd231pd		%zmm24, %zmm4, %zmm0
	vfmadd231pd		%zmm24, %zmm12, %zmm8
	vfmadd231pd		%zmm24, %zmm20, %zmm16
	vbroadcastsd	8+4*64(%r13), %zmm24
	vfmadd231pd		%zmm24, %zmm4, %zmm1
	vfmadd231pd		%zmm24, %zmm12, %zmm9
	vfmadd231pd		%zmm24, %zmm20, %zmm17
	vbroadcastsd	16+4*64(%r13), %zmm24
	vfmadd231pd		%zmm24, %zmm4, %zmm2
	vfmadd231pd		%zmm24, %zmm12, %zmm10
	vfmadd231pd		%zmm24, %zmm20, %zmm18
	vbroadcastsd	24+4*64(%r13), %zmm24
	vfmadd231pd		%zmm24, %zmm4, %zmm3
	vfmadd231pd		%zmm24, %zmm12, %zmm11
	vfmadd231pd		%zmm24, %zmm20, %zmm19
	//
	vmovapd			5*64(%r11), %zmm5
	vmovapd			5*64(%r11, %r12), %zmm13
	vmovapd			5*64(%r11, %r12, 2), %zmm21
	vbroadcastsd	0+5*64(%r13), %zmm24
	vfmadd231pd		%zmm24, %zmm5, %zmm0
	vfmadd231pd		%zmm24, %zmm13, %zmm8
	vfmadd231pd		%zmm24, %zmm21, %zmm16
	vbroadcastsd	8+5*64(%r13), %zmm24
	vfmadd231pd		%zmm24, %zmm5, %zmm1
	vfmadd231pd		%zmm24, %zmm13, %zmm9
	vfmadd231pd		%zmm24, %zmm21, %zmm17
	vbroadcastsd	16+5*64(%r13), %zmm24
	vfmadd231pd		%zmm24, %zmm5, %zmm2
	vfmadd231pd		%zmm24, %zmm13, %zmm10
	vfmadd231pd		%zmm24, %zmm21, %zmm18
	vbroadcastsd	24+5*64(%r13), %zmm24
	vfmadd231pd		%zmm24, %zmm5, %zmm3
	vfmadd231pd		%zmm24, %zmm13, %zmm11
	vfmadd231pd		%zmm24, %zmm21, %zmm19
	vbroadcastsd	32+5*64(%r13), %zmm24
	vfmadd231pd		%zmm24, %zmm5, %zmm4
	vfmadd231pd		%zmm24, %zmm13, %zmm12
	vfmadd231pd		%zmm24, %zmm21, %zmm20
	//
	vmovapd			6*64(%r11), %zmm6
	vmovapd			6*64(%r11, %r12), %zmm14
	vmovapd			6*64(%r11, %r12, 2), %zmm22
	vbroadcastsd	0+6*64(%r13), %zmm24
	vfmadd231pd		%zmm24, %zmm6, %zmm0
	vfmadd231pd		%zmm24, %zmm14, %zmm8
	vfmadd231pd		%zmm24, %zmm22, %zmm16
	vbroadcastsd	8+6*64(%r13), %zmm24
	vfmadd231pd		%zmm24, %zmm6, %zmm1
	vfmadd231pd		%zmm24, %zmm14, %zmm9
	vfmadd231pd		%zmm24, %zmm22, %zmm17
	vbroadcastsd	16+6*64(%r13), %zmm24
	vfmadd231pd		%zmm24, %zmm6, %zmm2
	vfmadd231pd		%zmm24, %zmm14, %zmm10
	vfmadd231pd		%zmm24, %zmm22, %zmm18
	vbroadcastsd	24+6*64(%r13), %zmm24
	vfmadd231pd		%zmm24, %zmm6, %zmm3
	vfmadd231pd		%zmm24, %zmm14, %zmm11
	vfmadd231pd		%zmm24, %zmm22, %zmm19
	vbroadcastsd	32+6*64(%r13), %zmm24
	vfmadd231pd		%zmm24, %zmm6, %zmm4
	vfmadd231pd		%zmm24, %zmm14, %zmm12
	vfmadd231pd		%zmm24, %zmm22, %zmm20
	vbroadcastsd	40+6*64(%r13), %zmm24
	vfmadd231pd		%zmm24, %zmm6, %zmm5
	vfmadd231pd		%zmm24, %zmm14, %zmm13
	vfmadd231pd		%zmm24, %zmm22, %zmm21
	//
	vmovapd			7*64(%r11), %zmm7
	vmovapd			7*64(%r11, %r12), %zmm15
	vmovapd			7*64(%r11, %r12, 2), %zmm23
	vbroadcastsd	0+7*64(%r13), %zmm24
	vfmadd231pd		%zmm24, %zmm7, %zmm0
	vfmadd231pd		%zmm24, %zmm15, %zmm8
	vfmadd231pd		%zmm24, %zmm23, %zmm16
	vbroadcastsd	8+7*64(%r13), %zmm24
	vfmadd231pd		%zmm24, %zmm7, %zmm1
	vfmadd231pd		%zmm24, %zmm15, %zmm9
	vfmadd231pd		%zmm24, %zmm23, %zmm17
	vbroadcastsd	16+7*64(%r13), %zmm24
	vfmadd231pd		%zmm24, %zmm7, %zmm2
	vfmadd231pd		%zmm24, %zmm15, %zmm10
	vfmadd231pd		%zmm24, %zmm23, %zmm18
	vbroadcastsd	24+7*64(%r13), %zmm24
	vfmadd231pd		%zmm24, %zmm7, %zmm3
	vfmadd231pd		%zmm24, %zmm15, %zmm11
	vfmadd231pd		%zmm24, %zmm23, %zmm19
	vbroadcastsd	32+7*64(%r13), %zmm24
	vfmadd231pd		%zmm24, %zmm7, %zmm4
	vfmadd231pd		%zmm24, %zmm15, %zmm12
	vfmadd231pd		%zmm24, %zmm23, %zmm20
	vbroadcastsd	40+7*64(%r13), %zmm24
	vfmadd231pd		%zmm24, %zmm7, %zmm5
	vfmadd231pd		%zmm24, %zmm15, %zmm13
	vfmadd231pd		%zmm24, %zmm23, %zmm21
	vbroadcastsd	48+7*64(%r13), %zmm24
	vfmadd231pd		%zmm24, %zmm7, %zmm6
	vfmadd231pd		%zmm24, %zmm15, %zmm14
	vfmadd231pd		%zmm24, %zmm23, %zmm22

	subl	$ 8, %r10d
	addq	$ 512, %r11
	addq	$ 512, %r13

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_24X8_LIB8
#else
	CALL(inner_kernel_dgemm_nt_24x8_lib8)
#endif

	movq	ARG3, %r10 // T

	//
	vbroadcastsd	56+7*64(%r10), %zmm24
	vmulpd			%zmm7, %zmm24, %zmm7
	vmulpd			%zmm15, %zmm24, %zmm15
	vmulpd			%zmm23, %zmm24, %zmm23
	//
	vbroadcastsd	48+7*64(%r10), %zmm24
	vfmadd231pd		%zmm6, %zmm24, %zmm7
	vfmadd231pd		%zmm14, %zmm24, %zmm15
	vfmadd231pd		%zmm22, %zmm24, %zmm23
	vbroadcastsd	48+6*64(%r10), %zmm24
	vmulpd			%zmm6, %zmm24, %zmm6
	vmulpd			%zmm14, %zmm24, %zmm14
	vmulpd			%zmm22, %zmm24, %zmm22
	//
	vbroadcastsd	40+7*64(%r10), %zmm24
	vfmadd231pd		%zmm5, %zmm24, %zmm7
	vfmadd231pd		%zmm13, %zmm24, %zmm15
	vfmadd231pd		%zmm21, %zmm24, %zmm23
	vbroadcastsd	40+6*64(%r10), %zmm24
	vfmadd231pd		%zmm5, %zmm24, %zmm6
	vfmadd231pd		%zmm13, %zmm24, %zmm14
	vfmadd231pd		%zmm21, %zmm24, %zmm22
	vbroadcastsd	40+5*64(%r10), %zmm24
	vmulpd			%zmm5, %zmm24, %zmm5
	vmulpd			%zmm13, %zmm24, %zmm13
	vmulpd			%zmm21, %zmm24, %zmm21
	//
	vbroadcastsd	32+7*64(%r10), %zmm24
	vfmadd231pd		%zmm4, %zmm24, %zmm7
	vfmadd231pd		%zmm12, %zmm24, %zmm15
	vfmadd231pd		%zmm20, %zmm24, %zmm23
	vbroadcastsd	32+6*64(%r10), %zmm24
	vfmadd231pd		%zmm4, %zmm24, %zmm6
	vfmadd231pd		%zmm12, %zmm24, %zmm14
	vfmadd231pd		%zmm20, %zmm24, %zmm22
	vbroadcastsd	32+5*64(%r10), %zmm24
	vfmadd231pd		%zmm4, %zmm24, %zmm5
	vfmadd231pd		%zmm12, %zmm24, %zmm13
	vfmadd231pd		%zmm20, %zmm24, %zmm21
	vbroadcastsd	32+4*64(%r10), %zmm24
	vmulpd			%zmm4, %zmm24, %zmm4
	vmulpd			%zmm12, %zmm24, %zmm12
	vmulpd			%zmm20, %zmm24, %zmm20
	//
	vbroadcastsd	24+7*64(%r10), %zmm24
	vfmadd231pd		%zmm3, %zmm24, %zmm7
	vfmadd231pd		%zmm11, %zmm24, %zmm15
	vfmadd231pd		%zmm19, %zmm24, %zmm23
	vbroadcastsd	24+6*64(%r10), %zmm24
	vfmadd231pd		%zmm3, %zmm24, %zmm6
	vfmadd231pd		%zmm11, %zmm24, %zmm14
	vfmadd231pd		%zmm19, %zmm24, %zmm22
	vbroadcastsd	24+5*64(%r10), %zmm24
	vfmadd231pd		%zmm3, %zmm24, %zmm5
	vfmadd231pd		%zmm11, %zmm24, %zmm13
	vfmadd231pd		%zmm19, %zmm24, %zmm21
	vbroadcastsd	24+4*64(%r10), %zmm24
	vfmadd231pd		%zmm3, %zmm24, %zmm4
	vfmadd231pd		%zmm11, %zmm24, %zmm12
	vfmadd231pd		%zmm19, %zmm24, %zmm20
	vbroadcastsd	24+3*64(%r10), %zmm24
	vmulpd			%zmm3, %zmm24, %zmm3
	vmulpd			%zmm11, %zmm24, %zmm11
	vmulpd			%zmm19, %zmm24, %zmm19
	//
	vbroadcastsd	16+7*64(%r10), %zmm24
	vfmadd231pd		%zmm2, %zmm24, %zmm7
	vfmadd231pd		%zmm10, %zmm24, %zmm15
	vfmadd231pd		%zmm18, %zmm24, %zmm23
	vbroadcastsd	16+6*64(%r10), %zmm24
	vfmadd231pd		%zmm2, %zmm24, %zmm6
	vfmadd231pd		%zmm10, %zmm24, %zmm14
	vfmadd231pd		%zmm18, %zmm24, %zmm22
	vbroadcastsd	16+5*64(%r10), %zmm24
	vfmadd231pd		%zmm2, %zmm24, %zmm5
	vfmadd231pd		%zmm10, %zmm24, %zmm13
	vfmadd231pd		%zmm18, %zmm24, %zmm21
	vbroadcastsd	16+4*64(%r10), %zmm24
	vfmadd231pd		%zmm2, %zmm24, %zmm4
	vfmadd231pd		%zmm10, %zmm24, %zmm12
	vfmadd231pd		%zmm18, %zmm24, %zmm20
	vbroadcastsd	16+3*64(%r10), %zmm24
	vfmadd231pd		%zmm2, %zmm24, %zmm3
	vfmadd231pd		%zmm10, %zmm24, %zmm11
	vfmadd231pd		%zmm18, %zmm24, %zmm19
	vbroadcastsd	16+2*64(%r10), %zmm24
	vmulpd			%zmm2, %zmm24, %zmm2
	vmulpd			%zmm10, %zmm24, %zmm10
	vmulpd			%zmm18, %zmm24, %zmm18
	//
	vbroadcastsd	8+7*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm7
	vfmadd231pd		%zmm9, %zmm24, %zmm15
	vfmadd231pd		%zmm17, %zmm24, %zmm23
	vbroadcastsd	8+6*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm6
	vfmadd231pd		%zmm9, %zmm24, %zmm14
	vfmadd231pd		%zmm17, %zmm24, %zmm22
	vbroadcastsd	8+5*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm5
	vfmadd231pd		%zmm9, %zmm24, %zmm13
	vfmadd231pd		%zmm17, %zmm24, %zmm21
	vbroadcastsd	8+4*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm4
	vfmadd231pd		%zmm9, %zmm24, %zmm12
	vfmadd231pd		%zmm17, %zmm24, %zmm20
	vbroadcastsd	8+3*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm3
	vfmadd231pd		%zmm9, %zmm24, %zmm11
	vfmadd231pd		%zmm17, %zmm24, %zmm19
	vbroadcastsd	8+2*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm2
	vfmadd231pd		%zmm9, %zmm24, %zmm10
	vfmadd231pd		%zmm17, %zmm24, %zmm18
	vbroadcastsd	8+1*64(%r10), %zmm24
	vmulpd			%zmm1, %zmm24, %zmm1
	vmulpd			%zmm9, %zmm24, %zmm9
	vmulpd			%zmm17, %zmm24, %zmm17
	//
	vbroadcastsd	0+7*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm7
	vfmadd231pd		%zmm8, %zmm24, %zmm15
	vfmadd231pd		%zmm16, %zmm24, %zmm23
	vbroadcastsd	0+6*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm6
	vfmadd231pd		%zmm8, %zmm24, %zmm14
	vfmadd231pd		%zmm16, %zmm24, %zmm22
	vbroadcastsd	0+5*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm5
	vfmadd231pd		%zmm8, %zmm24, %zmm13
	vfmadd231pd		%zmm16, %zmm24, %zmm21
	vbroadcastsd	0+4*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm4
	vfmadd231pd		%zmm8, %zmm24, %zmm12
	vfmadd231pd		%zmm16, %zmm24, %zmm20
	vbroadcastsd	0+3*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm3
	vfmadd231pd		%zmm8, %zmm24, %zmm11
	vfmadd231pd		%zmm16, %zmm24, %zmm19
	vbroadcastsd	0+2*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm2
	vfmadd231pd		%zmm8, %zmm24, %zmm10
	vfmadd231pd		%zmm16, %zmm24, %zmm18
	vbroadcastsd	0+1*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm1
	vfmadd231pd		%zmm8, %zmm24, %zmm9
	vfmadd231pd		%zmm16, %zmm24, %zmm17
	vbroadcastsd	0+0*64(%r10), %zmm24
	vmulpd			%zmm0, %zmm24, %zmm0
	vmulpd			%zmm8, %zmm24, %zmm8
	vmulpd			%zmm16, %zmm24, %zmm16

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // V
	movq	ARG4, %r12 // D
	movq	ARG5, %r13 // sdd
	sall	$ 6, %r13d

	//
	vmovapd			0+0*64(%r12), %zmm24
	vmovapd			0+0*64(%r12, %r13), %zmm26
	vmovapd			0+0*64(%r12, %r13, 2), %zmm27
	vaddpd			%zmm24, %zmm0, %zmm24
	vaddpd			%zmm26, %zmm8, %zmm26
	vaddpd			%zmm27, %zmm16, %zmm27
	vmovapd			%zmm24, 0+0*64(%r12)
	vmovapd			%zmm26, 0+0*64(%r12, %r13)
	vmovapd			%zmm27, 0+0*64(%r12, %r13, 2)
	//
	vmovapd			0+1*64(%r12), %zmm24
	vmovapd			0+1*64(%r12, %r13), %zmm26
	vmovapd			0+1*64(%r12, %r13, 2), %zmm27
	vbroadcastsd	0+1*64(%r11), %zmm25
	vfmadd231pd		%zmm0, %zmm25, %zmm24
	vfmadd231pd		%zmm8, %zmm25, %zmm26
	vfmadd231pd		%zmm16, %zmm25, %zmm27
	vaddpd			%zmm24, %zmm1, %zmm24
	vaddpd			%zmm26, %zmm9, %zmm26
	vaddpd			%zmm27, %zmm17, %zmm27
	vmovapd			%zmm24, 0+1*64(%r12)
	vmovapd			%zmm26, 0+1*64(%r12, %r13)
	vmovapd			%zmm27, 0+1*64(%r12, %r13, 2)
	//
	vmovapd			0+2*64(%r12), %zmm24
	vmovapd			0+2*64(%r12, %r13), %zmm26
	vmovapd			0+2*64(%r12, %r13, 2), %zmm27
	vbroadcastsd	0+2*64(%r11), %zmm25
	vfmadd231pd		%zmm0, %zmm25, %zmm24
	vfmadd231pd		%zmm8, %zmm25, %zmm26
	vfmadd231pd		%zmm16, %zmm25, %zmm27
	vbroadcastsd	8+2*64(%r11), %zmm25
	vfmadd231pd		%zmm1, %zmm25, %zmm24
	vfmadd231pd		%zmm9, %zmm25, %zmm26
	vfmadd231pd		%zmm17, %zmm25, %zmm27
	vaddpd			%zmm24, %zmm2, %zmm24
	vaddpd			%zmm26, %zmm10, %zmm26
	vaddpd			%zmm27, %zmm18, %zmm27
	vmovapd			%zmm24, 0+2*64(%r12)
	vmovapd			%zmm26, 0+2*64(%r12, %r13)
	vmovapd			%zmm27, 0+2*64(%r12, %r13, 2)
	//
	vmovapd			0+3*64(%r12), %zmm24
	vmovapd			0+3*64(%r12, %r13), %zmm26
	vmovapd			0+3*64(%r12, %r13, 2), %zmm27
	vbroadcastsd	0+3*64(%r11), %zmm25
	vfmadd231pd		%zmm0, %zmm25, %zmm24
	vfmadd231pd		%zmm8, %zmm25, %zmm26
	vfmadd231pd		%zmm16, %zmm25, %zmm27
	vbroadcastsd	8+3*64(%r11), %zmm25
	vfmadd231pd		%zmm1, %zmm25, %zmm24
	vfmadd231pd		%zmm9, %zmm25, %zmm26
	vfmadd231pd		%zmm17, %zmm25, %zmm27
	vbroadcastsd	16+3*64(%r11), %zmm25
	vfmadd231pd		%zmm2, %zmm25, %zmm24
	vfmadd231pd		%zmm10, %zmm25, %zmm26
	vfmadd231pd		%zmm18, %zmm25, %zmm27
	vaddpd			%zmm24, %zmm3, %zmm24
	vaddpd			%zmm26, %zmm11, %zmm26
	vaddpd			%zmm27, %zmm19, %zmm27
	vmovapd			%zmm24, 0+3*64(%r12)
	vmovapd			%zmm26, 0+3*64(%r12, %r13)
	vmovapd			%zmm27, 0+3*64(%r12, %r13, 2)
	//
	vmovapd			0+4*64(%r12), %zmm24
	vmovapd			0+4*64(%r12, %r13), %zmm26
	vmovapd			0+4*64(%r12, %r13, 2), %zmm27
	vbroadcastsd	0+4*64(%r11), %zmm25
	vfmadd231pd		%zmm0, %zmm25, %zmm24
	vfmadd231pd		%zmm8, %zmm25, %zmm26
	vfmadd231pd		%zmm16, %zmm25, %zmm27
	vbroadcastsd	8+4*64(%r11), %zmm25
	vfmadd231pd		%zmm1, %zmm25, %zmm24
	vfmadd231pd		%zmm9, %zmm25, %zmm26
	vfmadd231pd		%zmm17, %zmm25, %zmm27
	vbroadcastsd	16+4*64(%r11), %zmm25
	vfmadd231pd		%zmm2, %zmm25, %zmm24
	vfmadd231pd		%zmm10, %zmm25, %zmm26
	vfmadd231pd		%zmm18, %zmm25, %zmm27
	vbroadcastsd	24+4*64(%r11), %zmm25
	vfmadd231pd		%zmm3, %zmm25, %zmm24
	vfmadd231pd		%zmm11, %zmm25, %zmm26
	vfmadd231pd		%zmm19, %zmm25, %zmm27
	vaddpd			%zmm24, %zmm4, %zmm24
	vaddpd			%zmm26, %zmm12, %zmm26
	vaddpd			%zmm27, %zmm20, %zmm27
	vmovapd			%zmm24, 0+4*64(%r12)
	vmovapd			%zmm26, 0+4*64(%r12, %r13)
	vmovapd			%zmm27, 0+4*64(%r12, %r13, 2)
	//
	vmovapd			0+5*64(%r12), %zmm24
	vmovapd			0+5*64(%r12, %r13), %zmm26
	vmovapd			0+5*64(%r12, %r13, 2), %zmm27
	vbroadcastsd	0+5*64(%r11), %zmm25
	vfmadd231pd		%zmm0, %zmm25, %zmm24
	vfmadd231pd		%zmm8, %zmm25, %zmm26
	vfmadd231pd		%zmm16, %zmm25, %zmm27
	vbroadcastsd	8+5*64(%r11), %zmm25
	vfmadd231pd		%zmm1, %zmm25, %zmm24
	vfmadd231pd		%zmm9, %zmm25, %zmm26
	vfmadd231pd		%zmm17, %zmm25, %zmm27
	vbroadcastsd	16+5*64(%r11), %zmm25
	vfmadd231pd		%zmm2, %zmm25, %zmm24
	vfmadd231pd		%zmm10, %zmm25, %zmm26
	vfmadd231pd		%zmm18, %zmm25, %zmm27
	vbroadcastsd	24+5*64(%r11), %zmm25
	vfmadd231pd		%zmm3, %zmm25, %zmm24
	vfmadd231pd		%zmm11, %zmm25, %zmm26
	vfmadd231pd		%zmm19, %zmm25, %zmm27
	vbroadcastsd	32+5*64(%r11), %zmm25
	vfmadd231pd		%zmm4, %zmm25, %zmm24
	vfmadd231pd		%zmm12, %zmm25, %zmm26
	vfmadd231pd		%zmm20, %zmm25, %zmm27
	vaddpd			%zmm24, %zmm5, %zmm24
	vaddpd			%zmm26, %zmm13, %zmm26
	vaddpd			%zmm27, %zmm21, %zmm27
	vmovapd			%zmm24, 0+5*64(%r12)
	vmovapd			%zmm26, 0+5*64(%r12, %r13)
	vmovapd			%zmm27, 0+5*64(%r12, %r13, 2)
	//
	vmovapd			0+6*64(%r12), %zmm24
	vmovapd			0+6*64(%r12, %r13), %zmm26
	vmovapd			0+6*64(%r12, %r13, 2), %zmm27
	vbroadcastsd	0+6*64(%r11), %zmm25
	vfmadd231pd		%zmm0, %zmm25, %zmm24
	vfmadd231pd		%zmm8, %zmm25, %zmm26
	vfmadd231pd		%zmm16, %zmm25, %zmm27
	vbroadcastsd	8+6*64(%r11), %zmm25
	vfmadd231pd		%zmm1, %zmm25, %zmm24
	vfmadd231pd		%zmm9, %zmm25, %zmm26
	vfmadd231pd		%zmm17, %zmm25, %zmm27
	vbroadcastsd	16+6*64(%r11), %zmm25
	vfmadd231pd		%zmm2, %zmm25, %zmm24
	vfmadd231pd		%zmm10, %zmm25, %zmm26
	vfmadd231pd		%zmm18, %zmm25, %zmm27
	vbroadcastsd	24+6*64(%r11), %zmm25
	vfmadd231pd		%zmm3, %zmm25, %zmm24
	vfmadd231pd		%zmm11, %zmm25, %zmm26
	vfmadd231pd		%zmm19, %zmm25, %zmm27
	vbroadcastsd	32+6*64(%r11), %zmm25
	vfmadd231pd		%zmm4, %zmm25, %zmm24
	vfmadd231pd		%zmm12, %zmm25, %zmm26
	vfmadd231pd		%zmm20, %zmm25, %zmm27
	vbroadcastsd	40+6*64(%r11), %zmm25
	vfmadd231pd		%zmm5, %zmm25, %zmm24
	vfmadd231pd		%zmm13, %zmm25, %zmm26
	vfmadd231pd		%zmm21, %zmm25, %zmm27
	vaddpd			%zmm24, %zmm6, %zmm24
	vaddpd			%zmm26, %zmm14, %zmm26
	vaddpd			%zmm27, %zmm22, %zmm27
	vmovapd			%zmm24, 0+6*64(%r12)
	vmovapd			%zmm26, 0+6*64(%r12, %r13)
	vmovapd			%zmm27, 0+6*64(%r12, %r13, 2)
	//
	vmovapd			0+7*64(%r12), %zmm24
	vmovapd			0+7*64(%r12, %r13), %zmm26
	vmovapd			0+7*64(%r12, %r13, 2), %zmm27
	vbroadcastsd	0+7*64(%r11), %zmm25
	vfmadd231pd		%zmm0, %zmm25, %zmm24
	vfmadd231pd		%zmm8, %zmm25, %zmm26
	vfmadd231pd		%zmm16, %zmm25, %zmm27
	vbroadcastsd	8+7*64(%r11), %zmm25
	vfmadd231pd		%zmm1, %zmm25, %zmm24
	vfmadd231pd		%zmm9, %zmm25, %zmm26
	vfmadd231pd		%zmm17, %zmm25, %zmm27
	vbroadcastsd	16+7*64(%r11), %zmm25
	vfmadd231pd		%zmm2, %zmm25, %zmm24
	vfmadd231pd		%zmm10, %zmm25, %zmm26
	vfmadd231pd		%zmm18, %zmm25, %zmm27
	vbroadcastsd	24+7*64(%r11), %zmm25
	vfmadd231pd		%zmm3, %zmm25, %zmm24
	vfmadd231pd		%zmm11, %zmm25, %zmm26
	vfmadd231pd		%zmm19, %zmm25, %zmm27
	vbroadcastsd	32+7*64(%r11), %zmm25
	vfmadd231pd		%zmm4, %zmm25, %zmm24
	vfmadd231pd		%zmm12, %zmm25, %zmm26
	vfmadd231pd		%zmm20, %zmm25, %zmm27
	vbroadcastsd	40+7*64(%r11), %zmm25
	vfmadd231pd		%zmm5, %zmm25, %zmm24
	vfmadd231pd		%zmm13, %zmm25, %zmm26
	vfmadd231pd		%zmm21, %zmm25, %zmm27
	vbroadcastsd	48+7*64(%r11), %zmm25
	vfmadd231pd		%zmm6, %zmm25, %zmm24
	vfmadd231pd		%zmm14, %zmm25, %zmm26
	vfmadd231pd		%zmm22, %zmm25, %zmm27
	vaddpd			%zmm24, %zmm7, %zmm24
	vaddpd			%zmm26, %zmm15, %zmm26
	vaddpd			%zmm27, %zmm23, %zmm27
	vmovapd			%zmm24, 0+7*64(%r12)
	vmovapd			%zmm26, 0+7*64(%r12, %r13)
	vmovapd			%zmm27, 0+7*64(%r12, %r13, 2)

	subl	$ 8, %r10d
	addq	$ 512, %r11
	addq	$ 512, %r12

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEBP_NN_24X8_LIB8
#else
	CALL(inner_kernel_dgebp_nn_24x8_lib8)
#endif

	EPILOGUE

	ret

	FUN_END(kernel_dlarfb8_rn_24_lib8)





//#if defined(BLAS_API)
#if ( defined(BLAS_API) | ( defined(LA_HIGH_PERFORMANCE) & defined(MF_COLMAJ) ) )

#include "kernel_dgemm_24x8_lib.S"

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
	.align 64
.LC00:
#elif defined(OS_MAC)
	.align 6
LC00:
#endif
	.double 0.5
	.double 1.5
	.double 2.5
	.double 3.5
	.double 4.5
	.double 5.5
	.double 6.5
	.double 7.5


#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 64
.LC01:
#elif defined(OS_MAC)
	.align 6
LC01:
#endif
	.double 8.5
	.double 9.5
	.double 10.5
	.double 11.5
	.double 12.5
	.double 13.5
	.double 14.5
	.double 15.5


#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 64
.LC02:
#elif defined(OS_MAC)
	.align 6
LC02:
#endif
	.double 16.5
	.double 17.5
	.double 18.5
	.double 19.5
	.double 20.5
	.double 21.5
	.double 22.5
	.double 23.5


#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 64
.LC03:
#elif defined(OS_MAC)
	.align 6
LC03:
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
	.align 64
.LC04:
#elif defined(OS_MAC)
	.align 6
LC04:
#endif
	.double 1.0
	.double 1.0
	.double 1.0
	.double 1.0
	.double 1.0
	.double 1.0
	.double 1.0
	.double 1.0


#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 64
.LC05:
#elif defined(OS_MAC)
	.align 6
LC05:
#endif
	.quad 0x0
	.quad 0x1
	.quad 0x2
	.quad 0x3
	.quad 0x4
	.quad 0x5
	.quad 0x6
	.quad 0x7


#if defined(OS_LINUX)
	.section	.note.GNU-stack,"",@progbits
#elif defined(OS_MAC)
	.subsections_via_symbols
#endif

