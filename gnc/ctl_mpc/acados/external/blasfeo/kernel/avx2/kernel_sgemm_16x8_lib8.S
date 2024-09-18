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
	vxorps	%ymm0, %ymm0, %ymm0; \
	vmovaps	%ymm0, %ymm1; \
	vmovaps	%ymm0, %ymm2; \
	vmovaps	%ymm0, %ymm3; \
	vmovaps	%ymm0, %ymm4; \
	vmovaps	%ymm0, %ymm5; \
	vmovaps	%ymm0, %ymm6; \
	vmovaps	%ymm0, %ymm7; \
	vmovaps	%ymm0, %ymm8; \
	vmovaps	%ymm0, %ymm9; \
	vmovaps	%ymm0, %ymm10; \
	vmovaps	%ymm0, %ymm11; \
	vmovaps	%ymm0, 0(%r10); \
	vmovaps	%ymm0, 32(%r10); \
	vmovaps	%ymm0, 64(%r10); \
	vmovaps	%ymm0, 96(%r10); \
//#define NEG_ACC \
//	vmovapd		.LC11(%rip), %ymm15; \
//	vxorpd		%ymm15, %ymm0, %ymm0; \
//	vxorpd		%ymm15, %ymm1, %ymm1; \
//	vxorpd		%ymm15, %ymm2, %ymm2; \
//	vxorpd		%ymm15, %ymm3, %ymm3

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
	vxorps	%ymm0, %ymm0, %ymm0; \
	vmovaps	%ymm0, %ymm1; \
	vmovaps	%ymm0, %ymm2; \
	vmovaps	%ymm0, %ymm3; \
	vmovaps	%ymm0, %ymm4; \
	vmovaps	%ymm0, %ymm5; \
	vmovaps	%ymm0, %ymm6; \
	vmovaps	%ymm0, %ymm7; \
	vmovaps	%ymm0, %ymm8; \
	vmovaps	%ymm0, %ymm9; \
	vmovaps	%ymm0, %ymm10; \
	vmovaps	%ymm0, %ymm11; \
	vmovaps	%ymm0, 0(%r10); \
	vmovaps	%ymm0, 32(%r10); \
	vmovaps	%ymm0, 64(%r10); \
	vmovaps	%ymm0, 96(%r10); \
//#define NEG_ACC \
//	vmovapd		LC11(%rip), %ymm15; \
//	vxorpd		%ymm15, %ymm0, %ymm0; \
//	vxorpd		%ymm15, %ymm1, %ymm1; \
//	vxorpd		%ymm15, %ymm2, %ymm2; \
//	vxorpd		%ymm15, %ymm3, %ymm3

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
	vxorps	%ymm0, %ymm0, %ymm0; \
	vmovaps	%ymm0, %ymm1; \
	vmovaps	%ymm0, %ymm2; \
	vmovaps	%ymm0, %ymm3; \
	vmovaps	%ymm0, %ymm4; \
	vmovaps	%ymm0, %ymm5; \
	vmovaps	%ymm0, %ymm6; \
	vmovaps	%ymm0, %ymm7; \
	vmovaps	%ymm0, %ymm8; \
	vmovaps	%ymm0, %ymm9; \
	vmovaps	%ymm0, %ymm10; \
	vmovaps	%ymm0, %ymm11; \
	vmovaps	%ymm0, 0(%r10); \
	vmovaps	%ymm0, 32(%r10); \
	vmovaps	%ymm0, 64(%r10); \
	vmovaps	%ymm0, 96(%r10); \
//#define NEG_ACC \
//	vmovapd		.LC11(%rip), %ymm15; \
//	vxorpd		%ymm15, %ymm0, %ymm0; \
//	vxorpd		%ymm15, %ymm1, %ymm1; \
//	vxorpd		%ymm15, %ymm2, %ymm2; \
//	vxorpd		%ymm15, %ymm3, %ymm3

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
// r12   <- 8*sda*sizeof(float)
// r13   <- B
// r14   <- spil
// ymm0  <- []
// ...
// ymm11 <- []

//
// output arguments:
// r10d  <-
// r11   <-
// r12   <-
// r13   <-
// r14   <- spil
// ymm0  <- []
// ...
// ymm11 <- []

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NT_16X8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemm_add_nt_16x8_lib8)
#endif
	
#if 1



	cmpl	$ 0, %r10d
	jle		2f // return

	// preload

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// 0
	vmovaps			(0*32)(%r11), %ymm12 // A0
	vmovaps			(0*32)(%r11, %r12), %ymm13 // A1

	vbroadcastss	(0+0*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm0
	vfmadd213ps		0(%r14), %ymm13, %ymm14
	vmovaps			%ymm14, 0(%r14)

	vbroadcastss	(4+0*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm1
	vfmadd231ps		%ymm13, %ymm14, %ymm8

	vbroadcastss	(8+0*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm2
	vfmadd213ps		32(%r14), %ymm13, %ymm14
	vmovaps			%ymm14, 32(%r14)

	vbroadcastss	(12+0*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm3
	vfmadd231ps		%ymm13, %ymm14, %ymm9

	vbroadcastss	(16+0*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm4
	vfmadd213ps		64(%r14), %ymm13, %ymm14
	vmovaps			%ymm14, 64(%r14)

	vbroadcastss	(20+0*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm5
	vfmadd231ps		%ymm13, %ymm14, %ymm10

	vbroadcastss	(24+0*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm6
	vfmadd213ps		96(%r14), %ymm13, %ymm14
	vmovaps			%ymm14, 96(%r14)

	vbroadcastss	(28+0*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm7
	vfmadd231ps		%ymm13, %ymm14, %ymm11


	// 1
	vmovaps			(1*32)(%r11), %ymm12 // A0
	vmovaps			(1*32)(%r11, %r12), %ymm13 // A1

	vbroadcastss	(0+1*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm0
	vfmadd213ps		0(%r14), %ymm13, %ymm14
	vmovaps			%ymm14, 0(%r14)

	vbroadcastss	(4+1*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm1
	vfmadd231ps		%ymm13, %ymm14, %ymm8

	vbroadcastss	(8+1*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm2
	vfmadd213ps		32(%r14), %ymm13, %ymm14
	vmovaps			%ymm14, 32(%r14)

	vbroadcastss	(12+1*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm3
	vfmadd231ps		%ymm13, %ymm14, %ymm9

	vbroadcastss	(16+1*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm4
	vfmadd213ps		64(%r14), %ymm13, %ymm14
	vmovaps			%ymm14, 64(%r14)

	vbroadcastss	(20+1*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm5
	vfmadd231ps		%ymm13, %ymm14, %ymm10

	vbroadcastss	(24+1*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm6
	vfmadd213ps		96(%r14), %ymm13, %ymm14
	vmovaps			%ymm14, 96(%r14)

	vbroadcastss	(28+1*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm7
	vfmadd231ps		%ymm13, %ymm14, %ymm11


	// 2
	vmovaps			(2*32)(%r11), %ymm12 // A0
	vmovaps			(2*32)(%r11, %r12), %ymm13 // A1

	vbroadcastss	(0+2*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm0
	vfmadd213ps		0(%r14), %ymm13, %ymm14
	vmovaps			%ymm14, 0(%r14)

	vbroadcastss	(4+2*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm1
	vfmadd231ps		%ymm13, %ymm14, %ymm8

	vbroadcastss	(8+2*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm2
	vfmadd213ps		32(%r14), %ymm13, %ymm14
	vmovaps			%ymm14, 32(%r14)

	vbroadcastss	(12+2*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm3
	vfmadd231ps		%ymm13, %ymm14, %ymm9

	vbroadcastss	(16+2*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm4
	vfmadd213ps		64(%r14), %ymm13, %ymm14
	vmovaps			%ymm14, 64(%r14)

	vbroadcastss	(20+2*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm5
	vfmadd231ps		%ymm13, %ymm14, %ymm10

	vbroadcastss	(24+2*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm6
	vfmadd213ps		96(%r14), %ymm13, %ymm14
	vmovaps			%ymm14, 96(%r14)

	vbroadcastss	(28+2*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm7
	vfmadd231ps		%ymm13, %ymm14, %ymm11


	// 3
	vmovaps			(3*32)(%r11), %ymm12 // A0
	vmovaps			(3*32)(%r11, %r12), %ymm13 // A1

	vbroadcastss	(0+3*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm0
	vfmadd213ps		0(%r14), %ymm13, %ymm14
	vmovaps			%ymm14, 0(%r14)

	vbroadcastss	(4+3*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm1
	subl	$ 4, %r10d
	vfmadd231ps		%ymm13, %ymm14, %ymm8

	vbroadcastss	(8+3*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm2
	vfmadd213ps		32(%r14), %ymm13, %ymm14
	vmovaps			%ymm14, 32(%r14)

	vbroadcastss	(12+3*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm3
	addq	$ 128, %r11
	vfmadd231ps		%ymm13, %ymm14, %ymm9

	vbroadcastss	(16+3*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm4
	vfmadd213ps		64(%r14), %ymm13, %ymm14
	vmovaps			%ymm14, 64(%r14)

	vbroadcastss	(20+3*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm5
	vfmadd231ps		%ymm13, %ymm14, %ymm10

	vbroadcastss	(24+3*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm6
	vfmadd213ps		96(%r14), %ymm13, %ymm14
	vmovaps			%ymm14, 96(%r14)

	vbroadcastss	(28+3*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm7
	addq	$ 128, %r13
	vfmadd231ps		%ymm13, %ymm14, %ymm11


	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1

	// 0
	vmovaps			(0*32)(%r11), %ymm12 // A0
	vmovaps			(0*32)(%r11, %r12), %ymm13 // A1

	vbroadcastss	(0+0*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm0
	vfmadd213ps		0(%r14), %ymm13, %ymm14
	vmovaps			%ymm14, 0(%r14)

	vbroadcastss	(4+0*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm1
	vfmadd231ps		%ymm13, %ymm14, %ymm8

	vbroadcastss	(8+0*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm2
	vfmadd213ps		32(%r14), %ymm13, %ymm14
	vmovaps			%ymm14, 32(%r14)

	vbroadcastss	(12+0*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm3
	vfmadd231ps		%ymm13, %ymm14, %ymm9

	vbroadcastss	(16+0*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm4
	vfmadd213ps		64(%r14), %ymm13, %ymm14
	vmovaps			%ymm14, 64(%r14)

	vbroadcastss	(20+0*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm5
	vfmadd231ps		%ymm13, %ymm14, %ymm10

	vbroadcastss	(24+0*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm6
	vfmadd213ps		96(%r14), %ymm13, %ymm14
	vmovaps			%ymm14, 96(%r14)

	vbroadcastss	(28+0*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm7
	vfmadd231ps		%ymm13, %ymm14, %ymm11


	// 1
	vmovaps			(1*32)(%r11), %ymm12 // A0
	vmovaps			(1*32)(%r11, %r12), %ymm13 // A1

	vbroadcastss	(0+1*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm0
	vfmadd213ps		0(%r14), %ymm13, %ymm14
	vmovaps			%ymm14, 0(%r14)

	vbroadcastss	(4+1*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm1
	vfmadd231ps		%ymm13, %ymm14, %ymm8

	vbroadcastss	(8+1*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm2
	vfmadd213ps		32(%r14), %ymm13, %ymm14
	vmovaps			%ymm14, 32(%r14)

	vbroadcastss	(12+1*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm3
	vfmadd231ps		%ymm13, %ymm14, %ymm9

	vbroadcastss	(16+1*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm4
	vfmadd213ps		64(%r14), %ymm13, %ymm14
	vmovaps			%ymm14, 64(%r14)

	vbroadcastss	(20+1*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm5
	vfmadd231ps		%ymm13, %ymm14, %ymm10

	vbroadcastss	(24+1*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm6
	vfmadd213ps		96(%r14), %ymm13, %ymm14
	vmovaps			%ymm14, 96(%r14)

	vbroadcastss	(28+1*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm7
	vfmadd231ps		%ymm13, %ymm14, %ymm11


	// 2
	vmovaps			(2*32)(%r11), %ymm12 // A0
	vmovaps			(2*32)(%r11, %r12), %ymm13 // A1

	vbroadcastss	(0+2*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm0
	vfmadd213ps		0(%r14), %ymm13, %ymm14
	vmovaps			%ymm14, 0(%r14)

	vbroadcastss	(4+2*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm1
	vfmadd231ps		%ymm13, %ymm14, %ymm8

	vbroadcastss	(8+2*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm2
	vfmadd213ps		32(%r14), %ymm13, %ymm14
	vmovaps			%ymm14, 32(%r14)

	vbroadcastss	(12+2*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm3
	vfmadd231ps		%ymm13, %ymm14, %ymm9

	vbroadcastss	(16+2*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm4
	vfmadd213ps		64(%r14), %ymm13, %ymm14
	vmovaps			%ymm14, 64(%r14)

	vbroadcastss	(20+2*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm5
	vfmadd231ps		%ymm13, %ymm14, %ymm10

	vbroadcastss	(24+2*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm6
	vfmadd213ps		96(%r14), %ymm13, %ymm14
	vmovaps			%ymm14, 96(%r14)

	vbroadcastss	(28+2*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm7
	vfmadd231ps		%ymm13, %ymm14, %ymm11


	// 3
	vmovaps			(3*32)(%r11), %ymm12 // A0
	vmovaps			(3*32)(%r11, %r12), %ymm13 // A1

	vbroadcastss	(0+3*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm0
	vfmadd213ps		0(%r14), %ymm13, %ymm14
	vmovaps			%ymm14, 0(%r14)

	vbroadcastss	(4+3*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm1
//	subl	$ 4, %r10d
	vfmadd231ps		%ymm13, %ymm14, %ymm8

	vbroadcastss	(8+3*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm2
	vfmadd213ps		32(%r14), %ymm13, %ymm14
	vmovaps			%ymm14, 32(%r14)

	vbroadcastss	(12+3*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm3
	addq	$ 128, %r11
	vfmadd231ps		%ymm13, %ymm14, %ymm9

	vbroadcastss	(16+3*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm4
	vfmadd213ps		64(%r14), %ymm13, %ymm14
	vmovaps			%ymm14, 64(%r14)

	vbroadcastss	(20+3*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm5
	vfmadd231ps		%ymm13, %ymm14, %ymm10

	vbroadcastss	(24+3*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm6
	vfmadd213ps		96(%r14), %ymm13, %ymm14
	vmovaps			%ymm14, 96(%r14)

	vbroadcastss	(28+3*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm7
	addq	$ 128, %r13
	vfmadd231ps		%ymm13, %ymm14, %ymm11


//	cmpl	$ 4, %r10d
	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	vmovaps			(0*32)(%r11), %ymm12 // A0
	vmovaps			(0*32)(%r11, %r12), %ymm13 // A1

	vbroadcastss	(0+0*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm0
	vfmadd213ps		0(%r14), %ymm13, %ymm14
	vmovaps			%ymm14, 0(%r14)

	vbroadcastss	(4+0*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm1
	subl	$ 1, %r10d
	vfmadd231ps		%ymm13, %ymm14, %ymm8

	vbroadcastss	(8+0*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm2
	vfmadd213ps		32(%r14), %ymm13, %ymm14
	vmovaps			%ymm14, 32(%r14)

	vbroadcastss	(12+0*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm3
	vfmadd231ps		%ymm13, %ymm14, %ymm9

	vbroadcastss	(16+0*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm4
	vfmadd213ps		64(%r14), %ymm13, %ymm14
	vmovaps			%ymm14, 64(%r14)

	vbroadcastss	(20+0*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm5
	addq	$ 32, %r11
	vfmadd231ps		%ymm13, %ymm14, %ymm10

	vbroadcastss	(24+0*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm6
	vfmadd213ps		96(%r14), %ymm13, %ymm14
	vmovaps			%ymm14, 96(%r14)

	vbroadcastss	(28+0*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm7
	addq	$ 32, %r13
	vfmadd231ps		%ymm13, %ymm14, %ymm11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return




#else



	cmpl	$ 0, %r10d
	jle		2f // return

	// preload

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// 0
	vmovaps			(0*32)(%r11), %ymm12 // A0
	vmovaps			(0*32)(%r11, %r12), %ymm13 // A1

//	vbroadcastss	(0+0*32)(%r13), %ymm14 // B
	vbroadcastf128	(0+0*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm0
	vmovaps			%ymm14, %ymm15
	vfmadd213ps		0(%r14), %ymm13, %ymm15
	vmovaps			%ymm15, 0(%r14)

//	vbroadcastss	(4+0*32)(%r13), %ymm14 // B
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm1
	vfmadd231ps		%ymm13, %ymm14, %ymm8

//	vbroadcastss	(8+0*32)(%r13), %ymm14 // B
	vshufps			$ 0x4e, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm2
	vmovaps			%ymm14, %ymm15
	vfmadd213ps		32(%r14), %ymm13, %ymm15
	vmovaps			%ymm15, 32(%r14)

//	vbroadcastss	(12+0*32)(%r13), %ymm14 // B
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm3
	vfmadd231ps		%ymm13, %ymm14, %ymm9

//	vbroadcastss	(16+0*32)(%r13), %ymm14 // B
	vbroadcastf128	(16+0*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm4
	vmovaps			%ymm14, %ymm15
	vfmadd213ps		64(%r14), %ymm13, %ymm15
	vmovaps			%ymm15, 64(%r14)

//	vbroadcastss	(20+0*32)(%r13), %ymm14 // B
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm5
	vfmadd231ps		%ymm13, %ymm14, %ymm10

//	vbroadcastss	(24+0*32)(%r13), %ymm14 // B
	vshufps			$ 0x4e, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm6
	vmovaps			%ymm14, %ymm15
	vfmadd213ps		96(%r14), %ymm13, %ymm15
	vmovaps			%ymm15, 96(%r14)

//	vbroadcastss	(28+0*32)(%r13), %ymm14 // B
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm7
	vfmadd231ps		%ymm13, %ymm14, %ymm11


	// 1
	vmovaps			(1*32)(%r11), %ymm12 // A0
	vmovaps			(1*32)(%r11, %r12), %ymm13 // A1

//	vbroadcastss	(0+1*32)(%r13), %ymm14 // B
	vbroadcastf128	(0+1*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm0
	vmovaps			%ymm14, %ymm15
	vfmadd213ps		0(%r14), %ymm13, %ymm15
	vmovaps			%ymm15, 0(%r14)

//	vbroadcastss	(4+1*32)(%r13), %ymm14 // B
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm1
	vfmadd231ps		%ymm13, %ymm14, %ymm8

//	vbroadcastss	(8+1*32)(%r13), %ymm14 // B
	vshufps			$ 0x4e, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm2
	vmovaps			%ymm14, %ymm15
	vfmadd213ps		32(%r14), %ymm13, %ymm15
	vmovaps			%ymm15, 32(%r14)

//	vbroadcastss	(12+1*32)(%r13), %ymm14 // B
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm3
	vfmadd231ps		%ymm13, %ymm14, %ymm9

//	vbroadcastss	(16+1*32)(%r13), %ymm14 // B
	vbroadcastf128	(16+1*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm4
	vmovaps			%ymm14, %ymm15
	vfmadd213ps		64(%r14), %ymm13, %ymm15
	vmovaps			%ymm15, 64(%r14)

//	vbroadcastss	(20+1*32)(%r13), %ymm14 // B
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm5
	vfmadd231ps		%ymm13, %ymm14, %ymm10

//	vbroadcastss	(24+1*32)(%r13), %ymm14 // B
	vshufps			$ 0x4e, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm6
	vmovaps			%ymm14, %ymm15
	vfmadd213ps		96(%r14), %ymm13, %ymm15
	vmovaps			%ymm15, 96(%r14)

//	vbroadcastss	(28+1*32)(%r13), %ymm14 // B
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm7
	vfmadd231ps		%ymm13, %ymm14, %ymm11


	// 2
	vmovaps			(2*32)(%r11), %ymm12 // A0
	vmovaps			(2*32)(%r11, %r12), %ymm13 // A1

//	vbroadcastss	(0+2*32)(%r13), %ymm14 // B
	vbroadcastf128	(0+2*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm0
	vmovaps			%ymm14, %ymm15
	vfmadd213ps		0(%r14), %ymm13, %ymm15
	vmovaps			%ymm15, 0(%r14)

//	vbroadcastss	(4+2*32)(%r13), %ymm14 // B
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm1
	vfmadd231ps		%ymm13, %ymm14, %ymm8

//	vbroadcastss	(8+2*32)(%r13), %ymm14 // B
	vshufps			$ 0x4e, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm2
	vmovaps			%ymm14, %ymm15
	vfmadd213ps		32(%r14), %ymm13, %ymm15
	vmovaps			%ymm15, 32(%r14)

//	vbroadcastss	(12+2*32)(%r13), %ymm14 // B
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm3
	vfmadd231ps		%ymm13, %ymm14, %ymm9

//	vbroadcastss	(16+2*32)(%r13), %ymm14 // B
	vbroadcastf128	(16+2*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm4
	vmovaps			%ymm14, %ymm15
	vfmadd213ps		64(%r14), %ymm13, %ymm15
	vmovaps			%ymm15, 64(%r14)

//	vbroadcastss	(20+2*32)(%r13), %ymm14 // B
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm5
	vfmadd231ps		%ymm13, %ymm14, %ymm10

//	vbroadcastss	(24+2*32)(%r13), %ymm14 // B
	vshufps			$ 0x4e, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm6
	vmovaps			%ymm14, %ymm15
	vfmadd213ps		96(%r14), %ymm13, %ymm15
	vmovaps			%ymm15, 96(%r14)

//	vbroadcastss	(28+2*32)(%r13), %ymm14 // B
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm7
	vfmadd231ps		%ymm13, %ymm14, %ymm11


	// 3
	vmovaps			(3*32)(%r11), %ymm12 // A0
	vmovaps			(3*32)(%r11, %r12), %ymm13 // A1

//	vbroadcastss	(0+3*32)(%r13), %ymm14 // B
	vbroadcastf128	(0+3*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm0
	vmovaps			%ymm14, %ymm15
	vfmadd213ps		0(%r14), %ymm13, %ymm15
	vmovaps			%ymm15, 0(%r14)

//	vbroadcastss	(4+3*32)(%r13), %ymm14 // B
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm1
	subl	$ 4, %r10d
	vfmadd231ps		%ymm13, %ymm14, %ymm8

//	vbroadcastss	(8+3*32)(%r13), %ymm14 // B
	vshufps			$ 0x4e, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm2
	vmovaps			%ymm14, %ymm15
	vfmadd213ps		32(%r14), %ymm13, %ymm15
	vmovaps			%ymm15, 32(%r14)

//	vbroadcastss	(12+3*32)(%r13), %ymm14 // B
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm3
	addq	$ 128, %r11
	vfmadd231ps		%ymm13, %ymm14, %ymm9

//	vbroadcastss	(16+3*32)(%r13), %ymm14 // B
	vbroadcastf128	(16+3*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm4
	vmovaps			%ymm14, %ymm15
	vfmadd213ps		64(%r14), %ymm13, %ymm15
	vmovaps			%ymm15, 64(%r14)

//	vbroadcastss	(20+3*32)(%r13), %ymm14 // B
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm5
	vfmadd231ps		%ymm13, %ymm14, %ymm10

//	vbroadcastss	(24+3*32)(%r13), %ymm14 // B
	vshufps			$ 0x4e, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm6
	vmovaps			%ymm14, %ymm15
	vfmadd213ps		96(%r14), %ymm13, %ymm15
	vmovaps			%ymm15, 96(%r14)

//	vbroadcastss	(28+3*32)(%r13), %ymm14 // B
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm7
	addq	$ 128, %r13
	vfmadd231ps		%ymm13, %ymm14, %ymm11


	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1

	// 0
	vmovaps			(0*32)(%r11), %ymm12 // A0
	vmovaps			(0*32)(%r11, %r12), %ymm13 // A1

//	vbroadcastss	(0+0*32)(%r13), %ymm14 // B
	vbroadcastf128	(0+0*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm0
	vmovaps			%ymm14, %ymm15
	vfmadd213ps		0(%r14), %ymm13, %ymm15
	vmovaps			%ymm15, 0(%r14)

//	vbroadcastss	(4+0*32)(%r13), %ymm14 // B
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm1
	vfmadd231ps		%ymm13, %ymm14, %ymm8

//	vbroadcastss	(8+0*32)(%r13), %ymm14 // B
	vshufps			$ 0x4e, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm2
	vmovaps			%ymm14, %ymm15
	vfmadd213ps		32(%r14), %ymm13, %ymm15
	vmovaps			%ymm15, 32(%r14)

//	vbroadcastss	(12+0*32)(%r13), %ymm14 // B
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm3
	vfmadd231ps		%ymm13, %ymm14, %ymm9

//	vbroadcastss	(16+0*32)(%r13), %ymm14 // B
	vbroadcastf128	(16+0*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm4
	vmovaps			%ymm14, %ymm15
	vfmadd213ps		64(%r14), %ymm13, %ymm15
	vmovaps			%ymm15, 64(%r14)

//	vbroadcastss	(20+0*32)(%r13), %ymm14 // B
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm5
	vfmadd231ps		%ymm13, %ymm14, %ymm10

//	vbroadcastss	(24+0*32)(%r13), %ymm14 // B
	vshufps			$ 0x4e, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm6
	vmovaps			%ymm14, %ymm15
	vfmadd213ps		96(%r14), %ymm13, %ymm15
	vmovaps			%ymm15, 96(%r14)

//	vbroadcastss	(28+0*32)(%r13), %ymm14 // B
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm7
	vfmadd231ps		%ymm13, %ymm14, %ymm11


	// 1
	vmovaps			(1*32)(%r11), %ymm12 // A0
	vmovaps			(1*32)(%r11, %r12), %ymm13 // A1

//	vbroadcastss	(0+1*32)(%r13), %ymm14 // B
	vbroadcastf128	(0+1*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm0
	vmovaps			%ymm14, %ymm15
	vfmadd213ps		0(%r14), %ymm13, %ymm15
	vmovaps			%ymm15, 0(%r14)

//	vbroadcastss	(4+1*32)(%r13), %ymm14 // B
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm1
	vfmadd231ps		%ymm13, %ymm14, %ymm8

//	vbroadcastss	(8+1*32)(%r13), %ymm14 // B
	vshufps			$ 0x4e, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm2
	vmovaps			%ymm14, %ymm15
	vfmadd213ps		32(%r14), %ymm13, %ymm15
	vmovaps			%ymm15, 32(%r14)

//	vbroadcastss	(12+1*32)(%r13), %ymm14 // B
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm3
	vfmadd231ps		%ymm13, %ymm14, %ymm9

//	vbroadcastss	(16+1*32)(%r13), %ymm14 // B
	vbroadcastf128	(16+1*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm4
	vmovaps			%ymm14, %ymm15
	vfmadd213ps		64(%r14), %ymm13, %ymm15
	vmovaps			%ymm15, 64(%r14)

//	vbroadcastss	(20+1*32)(%r13), %ymm14 // B
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm5
	vfmadd231ps		%ymm13, %ymm14, %ymm10

//	vbroadcastss	(24+1*32)(%r13), %ymm14 // B
	vshufps			$ 0x4e, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm6
	vmovaps			%ymm14, %ymm15
	vfmadd213ps		96(%r14), %ymm13, %ymm15
	vmovaps			%ymm15, 96(%r14)

//	vbroadcastss	(28+1*32)(%r13), %ymm14 // B
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm7
	vfmadd231ps		%ymm13, %ymm14, %ymm11


	// 2
	vmovaps			(2*32)(%r11), %ymm12 // A0
	vmovaps			(2*32)(%r11, %r12), %ymm13 // A1

//	vbroadcastss	(0+2*32)(%r13), %ymm14 // B
	vbroadcastf128	(0+2*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm0
	vmovaps			%ymm14, %ymm15
	vfmadd213ps		0(%r14), %ymm13, %ymm15
	vmovaps			%ymm15, 0(%r14)

//	vbroadcastss	(4+2*32)(%r13), %ymm14 // B
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm1
	vfmadd231ps		%ymm13, %ymm14, %ymm8

//	vbroadcastss	(8+2*32)(%r13), %ymm14 // B
	vshufps			$ 0x4e, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm2
	vmovaps			%ymm14, %ymm15
	vfmadd213ps		32(%r14), %ymm13, %ymm15
	vmovaps			%ymm15, 32(%r14)

//	vbroadcastss	(12+2*32)(%r13), %ymm14 // B
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm3
	vfmadd231ps		%ymm13, %ymm14, %ymm9

//	vbroadcastss	(16+2*32)(%r13), %ymm14 // B
	vbroadcastf128	(16+2*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm4
	vmovaps			%ymm14, %ymm15
	vfmadd213ps		64(%r14), %ymm13, %ymm15
	vmovaps			%ymm15, 64(%r14)

//	vbroadcastss	(20+2*32)(%r13), %ymm14 // B
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm5
	vfmadd231ps		%ymm13, %ymm14, %ymm10

//	vbroadcastss	(24+2*32)(%r13), %ymm14 // B
	vshufps			$ 0x4e, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm6
	vmovaps			%ymm14, %ymm15
	vfmadd213ps		96(%r14), %ymm13, %ymm15
	vmovaps			%ymm15, 96(%r14)

//	vbroadcastss	(28+2*32)(%r13), %ymm14 // B
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm7
	vfmadd231ps		%ymm13, %ymm14, %ymm11


	// 3
	vmovaps			(3*32)(%r11), %ymm12 // A0
	vmovaps			(3*32)(%r11, %r12), %ymm13 // A1

//	vbroadcastss	(0+3*32)(%r13), %ymm14 // B
	vbroadcastf128	(0+3*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm0
	vmovaps			%ymm14, %ymm15
	vfmadd213ps		0(%r14), %ymm13, %ymm15
	vmovaps			%ymm15, 0(%r14)

//	vbroadcastss	(4+3*32)(%r13), %ymm14 // B
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm1
//	subl	$ 4, %r10d
	vfmadd231ps		%ymm13, %ymm14, %ymm8

//	vbroadcastss	(8+3*32)(%r13), %ymm14 // B
	vshufps			$ 0x4e, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm2
	vmovaps			%ymm14, %ymm15
	vfmadd213ps		32(%r14), %ymm13, %ymm15
	vmovaps			%ymm15, 32(%r14)

//	vbroadcastss	(12+3*32)(%r13), %ymm14 // B
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm3
	addq	$ 128, %r11
	vfmadd231ps		%ymm13, %ymm14, %ymm9

//	vbroadcastss	(16+3*32)(%r13), %ymm14 // B
	vbroadcastf128	(16+3*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm4
	vmovaps			%ymm14, %ymm15
	vfmadd213ps		64(%r14), %ymm13, %ymm15
	vmovaps			%ymm15, 64(%r14)

//	vbroadcastss	(20+3*32)(%r13), %ymm14 // B
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm5
	vfmadd231ps		%ymm13, %ymm14, %ymm10

//	vbroadcastss	(24+3*32)(%r13), %ymm14 // B
	vshufps			$ 0x4e, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm6
	vmovaps			%ymm14, %ymm15
	vfmadd213ps		96(%r14), %ymm13, %ymm15
	vmovaps			%ymm15, 96(%r14)

//	vbroadcastss	(28+3*32)(%r13), %ymm14 // B
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm7
	addq	$ 128, %r13
	vfmadd231ps		%ymm13, %ymm14, %ymm11


//	cmpl	$ 4, %r10d
	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	vmovaps			(0*32)(%r11), %ymm12 // A0
	vmovaps			(0*32)(%r11, %r12), %ymm13 // A1

//	vbroadcastss	(0+0*32)(%r13), %ymm14 // B
	vbroadcastf128	(0+0*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm0
	vmovaps			%ymm14, %ymm15
	vfmadd213ps		0(%r14), %ymm13, %ymm15
	vmovaps			%ymm15, 0(%r14)

//	vbroadcastss	(4+0*32)(%r13), %ymm14 // B
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm1
	subl	$ 1, %r10d
	vfmadd231ps		%ymm13, %ymm14, %ymm8

//	vbroadcastss	(8+0*32)(%r13), %ymm14 // B
	vshufps			$ 0x4e, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm2
	vmovaps			%ymm14, %ymm15
	vfmadd213ps		32(%r14), %ymm13, %ymm15
	vmovaps			%ymm15, 32(%r14)

//	vbroadcastss	(12+0*32)(%r13), %ymm14 // B
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm3
	vfmadd231ps		%ymm13, %ymm14, %ymm9

//	vbroadcastss	(16+0*32)(%r13), %ymm14 // B
	vbroadcastf128	(16+0*32)(%r13), %ymm14 // B
	vfmadd231ps		%ymm12, %ymm14, %ymm4
	vmovaps			%ymm14, %ymm15
	vfmadd213ps		64(%r14), %ymm13, %ymm15
	vmovaps			%ymm15, 64(%r14)

//	vbroadcastss	(20+0*32)(%r13), %ymm14 // B
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm5
	addq	$ 32, %r11
	vfmadd231ps		%ymm13, %ymm14, %ymm10

//	vbroadcastss	(24+0*32)(%r13), %ymm14 // B
	vshufps			$ 0x4e, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm6
	vmovaps			%ymm14, %ymm15
	vfmadd213ps		96(%r14), %ymm13, %ymm15
	vmovaps			%ymm15, 96(%r14)

//	vbroadcastss	(28+0*32)(%r13), %ymm14 // B
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vfmadd231ps		%ymm12, %ymm14, %ymm7
	addq	$ 32, %r13
	vfmadd231ps		%ymm13, %ymm14, %ymm11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return


#endif



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_16x8_lib8)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- 4*sdc*sizeof(float)
// r14   <- spil
// ymm0  <- []
// ...
// ymm11 <- []
//
// output arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- 4*sdc*sizeof(float)
// r14   <- spil
// ymm0  <- []
// ...
// ymm11 <- []

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_16X8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_blend_16x8_lib8)
#endif
	
	vblendps	$ 0x55, %ymm0, %ymm1, %ymm14
	vblendps	$ 0x55, %ymm1, %ymm0, %ymm1
	vblendps	$ 0x55, %ymm2, %ymm3, %ymm15
	vblendps	$ 0x55, %ymm3, %ymm2, %ymm2
	vmovaps		%ymm14, %ymm0
	vmovaps		%ymm15, %ymm3
	vblendps	$ 0x33, %ymm0, %ymm2, %ymm14
	vblendps	$ 0x33, %ymm2, %ymm0, %ymm2
	vblendps	$ 0x33, %ymm1, %ymm3, %ymm15
	vblendps	$ 0x33, %ymm3, %ymm1, %ymm3
	vmovaps		%ymm14, %ymm0
	vmovaps		%ymm15, %ymm1

	vblendps	$ 0x55, %ymm4, %ymm5, %ymm14
	vblendps	$ 0x55, %ymm5, %ymm4, %ymm5
	vblendps	$ 0x55, %ymm6, %ymm7, %ymm15
	vblendps	$ 0x55, %ymm7, %ymm6, %ymm6
	vmovaps		%ymm14, %ymm4
	vmovaps		%ymm15, %ymm7
	vblendps	$ 0x33, %ymm4, %ymm6, %ymm14
	vblendps	$ 0x33, %ymm6, %ymm4, %ymm6
	vblendps	$ 0x33, %ymm5, %ymm7, %ymm15
	vblendps	$ 0x33, %ymm7, %ymm5, %ymm7
	vmovaps		%ymm14, %ymm4
	vmovaps		%ymm15, %ymm5

	vmovaps		0(%r10), %ymm12
	vmovaps		32(%r10), %ymm13
	vblendps	$ 0x55, %ymm12, %ymm8, %ymm14
	vblendps	$ 0x55, %ymm8, %ymm12, %ymm8
	vblendps	$ 0x55, %ymm13, %ymm9, %ymm15
	vblendps	$ 0x55, %ymm9, %ymm13, %ymm13
	vmovaps		%ymm14, %ymm12
	vmovaps		%ymm15, %ymm9
	vblendps	$ 0x33, %ymm12, %ymm13, %ymm14
	vblendps	$ 0x33, %ymm13, %ymm12, %ymm13
	vblendps	$ 0x33, %ymm8, %ymm9, %ymm15
	vblendps	$ 0x33, %ymm9, %ymm8, %ymm9
	vmovaps		%ymm14, 0(%r10)
	vmovaps		%ymm13, 32(%r10)
	vmovaps		%ymm15, %ymm8

	vmovaps		64(%r10), %ymm12
	vmovaps		96(%r10), %ymm13
	vblendps	$ 0x55, %ymm12, %ymm10, %ymm14
	vblendps	$ 0x55, %ymm10, %ymm12, %ymm10
	vblendps	$ 0x55, %ymm13, %ymm11, %ymm15
	vblendps	$ 0x55, %ymm11, %ymm13, %ymm13
	vmovaps		%ymm14, %ymm12
	vmovaps		%ymm15, %ymm11
	vblendps	$ 0x33, %ymm12, %ymm13, %ymm14
	vblendps	$ 0x33, %ymm13, %ymm12, %ymm13
	vblendps	$ 0x33, %ymm10, %ymm11, %ymm15
	vblendps	$ 0x33, %ymm11, %ymm10, %ymm11
	vmovaps		%ymm14, 64(%r10)
	vmovaps		%ymm13, 96(%r10)
	vmovaps		%ymm15, %ymm10

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_blend_16x8_lib8)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- 4*sdc*sizeof(float)
// r14   <- spil
// ymm0  <- []
// ...
// ymm11 <- []
//
// output arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- 4*sdc*sizeof(float)
// r14   <- spil
// ymm0  <- []
// ...
// ymm11 <- []

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_16X8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_16x8_lib8)
#endif

	// alpha
	vbroadcastss	0(%r10), %ymm12

	vmulps		%ymm0, %ymm12, %ymm0
	vmulps		%ymm1, %ymm12, %ymm1
	vmulps		%ymm2, %ymm12, %ymm2
	vmulps		%ymm3, %ymm12, %ymm3

	vmulps		%ymm4, %ymm12, %ymm4
	vmulps		%ymm5, %ymm12, %ymm5
	vmulps		%ymm6, %ymm12, %ymm6
	vmulps		%ymm7, %ymm12, %ymm7

	vmulps		0(%r14), %ymm12, %ymm15
	vmovaps		%ymm15, 0(%r14)
	vmulps		%ymm8, %ymm12, %ymm8
	vmulps		32(%r14), %ymm12, %ymm15
	vmovaps		%ymm15, 32(%r14)
	vmulps		%ymm9, %ymm12, %ymm9

	vmulps		64(%r14), %ymm12, %ymm15
	vmovaps		%ymm15, 64(%r14)
	vmulps		%ymm10, %ymm12, %ymm10
	vmulps		96(%r14), %ymm12, %ymm15
	vmovaps		%ymm15, 96(%r14)
	vmulps		%ymm11, %ymm12, %ymm11

	// beta
	vbroadcastss	0(%r11), %ymm14

	vxorps		%ymm15, %ymm15, %ymm15 // 0.0

	vucomiss	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	// TODO fix
	vmovaps		0(%r12), %ymm15
	vmulps		%ymm15, %ymm14, %ymm15
	vaddps		%ymm0, %ymm15, %ymm0
	vmovaps		32(%r12), %ymm15
	vmulps		%ymm15, %ymm14, %ymm15
	vaddps		%ymm1, %ymm15, %ymm1
	vmovaps		64(%r12), %ymm15
	vmulps		%ymm15, %ymm14, %ymm15
	vaddps		%ymm2, %ymm15, %ymm2
	vmovaps		96(%r12), %ymm15
	vmulps		%ymm15, %ymm14, %ymm15
	vaddps		%ymm3, %ymm15, %ymm3

	vmovaps		0(%r12, %r13), %ymm15
	vmulps		%ymm15, %ymm14, %ymm15
	vaddps		%ymm4, %ymm15, %ymm4
	vmovaps		32(%r12, %r13), %ymm15
	vmulps		%ymm15, %ymm14, %ymm15
	vaddps		%ymm5, %ymm15, %ymm5
	vmovaps		64(%r12, %r13), %ymm15
	vmulps		%ymm15, %ymm14, %ymm15
	vaddps		%ymm6, %ymm15, %ymm6
	vmovaps		96(%r12, %r13), %ymm15
	vmulps		%ymm15, %ymm14, %ymm15
	vaddps		%ymm7, %ymm15, %ymm7

	vmovaps		0(%r12, %r13, 2), %ymm15
	vmulps		%ymm15, %ymm14, %ymm15
	vaddps		0(%r14), %ymm15, %ymm15
	vmovaps		%ymm15, 0(%r14)
	vmovaps		32(%r12, %r13, 2), %ymm15
	vmulps		%ymm15, %ymm14, %ymm15
	vaddps		%ymm8, %ymm15, %ymm8
	vmovaps		64(%r12, %r13, 2), %ymm15
	vmulps		%ymm15, %ymm14, %ymm15
	vaddps		32(%r14), %ymm15, %ymm15
	vmovaps		%ymm15, 32(%r14)
	vmovaps		96(%r12, %r13, 2), %ymm15
	vmulps		%ymm15, %ymm14, %ymm15
	vaddps		%ymm9, %ymm15, %ymm9

	vmovaps		0(%r12, %r15), %ymm15
	vmulps		%ymm15, %ymm14, %ymm15
	vaddps		64(%r14), %ymm15, %ymm15
	vmovaps		%ymm15, 64(%r14)
	vmovaps		32(%r12, %r15), %ymm15
	vmulps		%ymm15, %ymm14, %ymm15
	vaddps		%ymm8, %ymm15, %ymm8
	vmovaps		64(%r12, %r15), %ymm15
	vmulps		%ymm15, %ymm14, %ymm15
	vaddps		96(%r14), %ymm15, %ymm15
	vmovaps		%ymm15, 96(%r14)
	vmovaps		96(%r12, %r15), %ymm15
	vmulps		%ymm15, %ymm14, %ymm15
	vaddps		%ymm9, %ymm15, %ymm9

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_16x8_lib8)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// r11  <- 4*sdd*sizeof(float)
// r12   <- spil
// ymm0  <- []
// ...
// ymm11 <- []
//
// output arguments:
// r10  <- D
// r11  <- 4*sdd*sizeof(float)
// r12   <- spil
// ymm0  <- []
// ...
// ymm11 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_16X8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_store_16x8_lib8)
#endif
	
	vmovaps 	%ymm0, (0*32)(%r10)
	vmovaps 	%ymm1, (1*32)(%r10)
	vmovaps 	%ymm2, (2*32)(%r10)
	vmovaps		%ymm3, (3*32)(%r10)

	vmovaps 	%ymm4, (4*32)(%r10)
	vmovaps 	%ymm5, (5*32)(%r10)
	vmovaps 	%ymm6, (6*32)(%r10)
	vmovaps 	%ymm7, (7*32)(%r10)

	vmovaps		0(%r12), %ymm15
	vmovaps 	%ymm15, (0*32)(%r10, %r11, 1)
	vmovaps 	%ymm8, (1*32)(%r10, %r11, 1)
	vmovaps		32(%r12), %ymm15
	vmovaps 	%ymm15, (2*32)(%r10, %r11, 1)
	vmovaps 	%ymm9, (3*32)(%r10, %r11, 1)

	vmovaps		64(%r12), %ymm15
	vmovaps 	%ymm15, (4*32)(%r10, %r11, 1)
	vmovaps 	%ymm10, (5*32)(%r10, %r11, 1)
	vmovaps		96(%r12), %ymm15
	vmovaps 	%ymm15, (6*32)(%r10, %r11, 1)
	vmovaps 	%ymm11, (7*32)(%r10, %r11, 1)


#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_16x8_lib8)
#endif





//                                1      2             3         4        5         6            7         8        9         10       11
// void kernel_sgemm_nt_16x4_lib8(int k, float *alpha, float *A, int sda, float *B, float *beta, float *C, int sdc, float *D, int sdd, float *spil);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_nt_16x8_lib8)
	
	PROLOGUE

	// zero accumulation registers
	movq	ARG11, %r10  // spil

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG5, %r13  // B
	movq	ARG11, %r14  // spil

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_16X8_LIB8
#else
	CALL(inner_kernel_gemm_add_nt_16x8_lib8)
#endif


	// call inner scale

//	movq	ARG11, %r10  // spil

#if MACRO_LEVEL>=1
//	INNER_BLEND_16X8_LIB8
#else
//	CALL(inner_blend_16x8_lib8)
#endif

	movq	%rsi, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movl	ARG8, %r13d // sdc
	sall	$ 5, %r13d // 8*sdc*sizeof(float)
	movq	ARG11, %r14  // spil

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_16X8_LIB8
#else
	CALL(inner_scale_ab_16x8_lib8)
#endif


	// store n

	movq	ARG9, %r10 // D
	movl	ARG10, %r11d // sdd
	sall	$ 5, %r11d // 8*sdd*sizeof(float)
	movq	ARG11, %r12  // spil

#if MACRO_LEVEL>=1
	INNER_STORE_16X8_LIB8
#else
	CALL(inner_store_16x8_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_nt_16x8_lib8)







