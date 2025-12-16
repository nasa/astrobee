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
//#define ZERO_ACC \
//	vxorpd	%ymm0, %ymm0, %ymm0; \
//	vmovapd	%ymm0, %ymm1; \
//	vmovapd	%ymm0, %ymm2; \
//	vmovapd	%ymm0, %ymm3
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
//#define ZERO_ACC \
//	vxorpd	%ymm0, %ymm0, %ymm0; \
//	vmovapd	%ymm0, %ymm1; \
//	vmovapd	%ymm0, %ymm2; \
//	vmovapd	%ymm0, %ymm3
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
//#define ZERO_ACC \
//	vxorpd	%ymm0, %ymm0, %ymm0; \
//	vmovapd	%ymm0, %ymm1; \
//	vmovapd	%ymm0, %ymm2; \
//	vmovapd	%ymm0, %ymm3
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
// r12   <- x
// ymm0  <- [z0 z1 z2 z3]_a
// ymm1  <- [z0 z1 z2 z3]_b
// ymm2  <- [z0 z1 z2 z3]_c
// ymm3  <- [z0 z1 z2 z3]_d
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
// r12   <- x+k*sizeof(double)
// ymm0  <- [z0 z1 z2 z3]_a
// ymm1  <- [z0 z1 z2 z3]_b
// ymm2  <- [z0 z1 z2 z3]_c
// ymm3  <- [z0 z1 z2 z3]_d
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMV_ADD_N_8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemv_add_n_8_lib8)
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	cmpl	$ 4, %r10d
	jl		0f // clean-up loop

	// main loop
	.p2align 3
1: // main loop
	
	vmovaps			0(%r11), %ymm8
	vbroadcastss	0(%r12), %ymm12
	vmulps			%ymm8, %ymm12, %ymm15
	vaddps			%ymm0, %ymm15, %ymm0
	
	subl	$ 4, %r10d

	vmovaps			32(%r11), %ymm8
	vbroadcastss	4(%r12), %ymm12
	vmulps			%ymm8, %ymm12, %ymm15
	vaddps			%ymm1, %ymm15, %ymm1
	
	vmovaps			64(%r11), %ymm8
	vbroadcastss	8(%r12), %ymm12
	vmulps			%ymm8, %ymm12, %ymm15
	vaddps			%ymm2, %ymm15, %ymm2

	vmovaps			96(%r11), %ymm8
	vbroadcastss	12(%r12), %ymm12
	vmulps			%ymm8, %ymm12, %ymm15
	vaddps			%ymm3, %ymm15, %ymm3
	
	addq	$ 128, %r11
	addq	$ 16, %r12
	
	cmpl	$ 3, %r10d

	jg		1b // main loop 


	// consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

0: // clean-up
	
	vmovaps			0(%r11), %ymm8
	vbroadcastss	0(%r12), %ymm12
	vmulps			%ymm8, %ymm12, %ymm15
	vaddps			%ymm0, %ymm15, %ymm0
	
	addq	$ 32, %r11
	addq	$ 4, %r12
	
	subl	$ 1, %r10d
	cmpl	$ 0, %r10d

	jg		0b // clean

2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemv_add_n_8_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- bs*sda*sizeof(double) = 32*sda
// r13   <- x
// ymm0  <- [z0a z0b z0c z0d]
// ymm1  <- [z1a z1b z1c z1d]
// ymm2  <- [z2a z2b z2c z2d]
// ymm3  <- [z3a z3b z3c z3d]
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
// r12   <- bs*sda*sizeof(double) = 32*sda
// r13   <- x+k*sizeof(double)
// ymm0  <- [z0a z0b z0c z0d]
// ymm1  <- [z1a z1b z1c z1d]
// ymm2  <- [z2a z2b z2c z2d]
// ymm3  <- [z3a z3b z3c z3d]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMV_ADD_T_8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemv_add_t_8_lib8)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	cmpl	$ 8, %r10d
	jl		0f // clean-up loop

	// main loop
	.p2align 3
1: // main loop
	
	vmovups		0(%r13), %ymm12

	vmovaps		0(%r11), %ymm8
	vmulps		%ymm8, %ymm12, %ymm15
	vaddps		%ymm0, %ymm15, %ymm0
	
	subl	$ 8, %r10d

	vmovaps		32(%r11), %ymm8
	vmulps		%ymm8, %ymm12, %ymm15
	vaddps		%ymm1, %ymm15, %ymm1
	
	vmovaps		64(%r11), %ymm8
	vmulps		%ymm8, %ymm12, %ymm15
	vaddps		%ymm2, %ymm15, %ymm2

	vmovaps		96(%r11), %ymm8
	vmulps		%ymm8, %ymm12, %ymm15
	vaddps		%ymm3, %ymm15, %ymm3
	
	vmovaps		128(%r11), %ymm8
	vmulps		%ymm8, %ymm12, %ymm15
	vaddps		%ymm4, %ymm15, %ymm4
	
	vmovaps		160(%r11), %ymm8
	vmulps		%ymm8, %ymm12, %ymm15
	vaddps		%ymm5, %ymm15, %ymm5
	
	vmovaps		192(%r11), %ymm8
	vmulps		%ymm8, %ymm12, %ymm15
	vaddps		%ymm6, %ymm15, %ymm6
	
	vmovaps		224(%r11), %ymm8
	vmulps		%ymm8, %ymm12, %ymm15
	vaddps		%ymm7, %ymm15, %ymm7
	
	addq	%r12, %r11
	addq	$ 32, %r13
	
	cmpl	$ 7, %r10d

	jg		1b // main loop 


	// consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

0: // clean-up
	
	vcvtsi2ss	%r10d, %xmm14, %xmm14
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %ymm13
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %ymm13
#endif
	vshufps		$ 0x00, %xmm14, %xmm14, %xmm14
	vinsertf128	$ 0x1, %xmm14, %ymm14, %ymm14
	vsubps		%ymm14, %ymm13, %ymm14

	vmaskmovps	0(%r13), %ymm14, %ymm12

	vmaskmovps	0(%r11), %ymm14, %ymm8
	vmulps		%ymm8, %ymm12, %ymm15
	vaddps		%ymm0, %ymm15, %ymm0
	
	vmaskmovps	32(%r11), %ymm14, %ymm8
	vmulps		%ymm8, %ymm12, %ymm15
	vaddps		%ymm1, %ymm15, %ymm1
	
	vmaskmovps	64(%r11), %ymm14, %ymm8
	vmulps		%ymm8, %ymm12, %ymm15
	vaddps		%ymm2, %ymm15, %ymm2

	vmaskmovps	96(%r11), %ymm14, %ymm8
	vmulps		%ymm8, %ymm12, %ymm15
	vaddps		%ymm3, %ymm15, %ymm3
		
	vmaskmovps	128(%r11), %ymm14, %ymm8
	vmulps		%ymm8, %ymm12, %ymm15
	vaddps		%ymm4, %ymm15, %ymm4
		
	vmaskmovps	160(%r11), %ymm14, %ymm8
	vmulps		%ymm8, %ymm12, %ymm15
	vaddps		%ymm5, %ymm15, %ymm5
		
	vmaskmovps	192(%r11), %ymm14, %ymm8
	vmulps		%ymm8, %ymm12, %ymm15
	vaddps		%ymm6, %ymm15, %ymm6
		
	vmaskmovps	224(%r11), %ymm14, %ymm8
	vmulps		%ymm8, %ymm12, %ymm15
	vaddps		%ymm7, %ymm15, %ymm7
		
	sall	$ 2, %r10d
	addq	%r10, %r11
	addq	%r10, %r13
	xorl	%r10d, %r10d
	
	
2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemv_add_t_8_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- bs*sda*sizeof(double) = 32*sda
// r13   <- x
// r14d  <- offA
// ymm0  <- [z0a z0b z0c z0d]
// ymm1  <- [z1a z1b z1c z1d]
// ymm2  <- [z2a z2b z2c z2d]
// ymm3  <- [z3a z3b z3c z3d]
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
// r10d  <- 
// r11   <- 
// r12   <- 
// r13   <- 
// r14d  <- offA
// ymm0  <- [z0a z0b z0c z0d]
// ymm1  <- [z1a z1b z1c z1d]
// ymm2  <- [z2a z2b z2c z2d]
// ymm3  <- [z3a z3b z3c z3d]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=2
	.macro INNER_EDGE_GEMV_ADD_T_8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_gemv_add_t_8_lib8)
#endif

	cmpl	$ 0, %r14d
	jle		0f // return

	movl	%r14d, %r15d
	sall	$ 2, %r15d // offA*sizeof(float)

	subq	%r15, %r11 // A - offA
	subq	%r15, %r13 // x - offA

	movl	%r10d, %r15d // kmax
	addl	%r14d, %r15d // kmax + offA

	vcvtsi2ss	%r14d, %xmm14, %xmm14 // offA
	vcvtsi2ss	%r15d, %xmm15, %xmm15 // offA + kmax
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %ymm13
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %ymm13
#endif
	vshufps		$ 0x00, %xmm14, %xmm14, %xmm14
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
	vinsertf128	$ 1, %xmm14, %ymm14, %ymm14
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubps		%ymm13, %ymm14, %ymm14
	vsubps		%ymm15, %ymm13, %ymm15
	vandps		%ymm15, %ymm14, %ymm14

	vmaskmovps	0(%r13), %ymm14, %ymm12

	vmovaps		0(%r11), %ymm8
	vmulps		%ymm8, %ymm12, %ymm15
	vaddps		%ymm0, %ymm15, %ymm0
	
	vmovaps		32(%r11), %ymm8
	vmulps		%ymm8, %ymm12, %ymm15
	vaddps		%ymm1, %ymm15, %ymm1
	
	vmovaps		64(%r11), %ymm8
	vmulps		%ymm8, %ymm12, %ymm15
	vaddps		%ymm2, %ymm15, %ymm2

	vmovaps		96(%r11), %ymm8
	vmulps		%ymm8, %ymm12, %ymm15
	vaddps		%ymm3, %ymm15, %ymm3

	vmovaps		128(%r11), %ymm8
	vmulps		%ymm8, %ymm12, %ymm15
	vaddps		%ymm4, %ymm15, %ymm4

	vmovaps		160(%r11), %ymm8
	vmulps		%ymm8, %ymm12, %ymm15
	vaddps		%ymm5, %ymm15, %ymm5

	vmovaps		192(%r11), %ymm8
	vmulps		%ymm8, %ymm12, %ymm15
	vaddps		%ymm6, %ymm15, %ymm6

	vmovaps		224(%r11), %ymm8
	vmulps		%ymm8, %ymm12, %ymm15
	vaddps		%ymm7, %ymm15, %ymm7

	addq	$ 32, %r13 // x + 4
	addq	%r12, %r11 // A + bs*sda
		
	addl	%r14d, %r10d
	subl	$ 8, %r10d // kmax - (8-offA)
	
0: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_edge_gemv_add_t_8_lib8)
#endif





// common inner routine with file scope
//
// triangular substitution with vector RHS
//
// input arguments:
// r10  <- E
// r11  <- inv_diag_E
// ymm0 <- [z0 z1 z2 z3]
// ymm12 <- dirty
// ymm13 <- dirty
//
// output arguments:
// r10  <- E
// r11  <- inv_diag_E
// ymm0 <- [z0 z1 z2 z3]
// ymm12 <- dirty
// ymm13 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRSV_LN_INV_8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_trsv_ln_inv_8_lib8)
#endif
	
	vxorps			%ymm14, %ymm14, %ymm14

	vbroadcastss	0(%r11), %ymm12
	vmulps			%ymm0, %ymm12, %ymm1
	vblendps		$ 0x01, %ymm1, %ymm0, %ymm0

	vmovaps			0(%r10), %ymm13
	vblendps		$ 0x01, %ymm14, %ymm13, %ymm13
	vpermilps		$ 0x00, %ymm0, %ymm12
	vperm2f128		$ 0x00, %ymm12, %ymm12, %ymm12
	vmulps			%ymm13, %ymm12, %ymm15
	vsubps			%ymm15, %ymm0, %ymm0
	vbroadcastss	4(%r11), %ymm12
	vmulps			%ymm0, %ymm12, %ymm1
	vblendps		$ 0x02, %ymm1, %ymm0, %ymm0

	vmovaps			32(%r10), %ymm13
	vblendps		$ 0x03, %ymm14, %ymm13, %ymm13
	vpermilps		$ 0x55, %ymm0, %ymm12
	vperm2f128		$ 0x00, %ymm12, %ymm12, %ymm12
	vmulps			%ymm13, %ymm12, %ymm15
	vsubps			%ymm15, %ymm0, %ymm0
	vbroadcastss	8(%r11), %ymm12
	vmulps			%ymm0, %ymm12, %ymm1
	vblendps		$ 0x04, %ymm1, %ymm0, %ymm0

	vmovaps			64(%r10), %ymm13
	vblendps		$ 0x07, %ymm14, %ymm13, %ymm13
	vpermilps		$ 0xaa, %ymm0, %ymm12
	vperm2f128		$ 0x00, %ymm12, %ymm12, %ymm12
	vmulps			%ymm13, %ymm12, %ymm15
	vsubps			%ymm15, %ymm0, %ymm0
	vbroadcastss	12(%r11), %ymm12
	vmulps			%ymm0, %ymm12, %ymm1
	vblendps		$ 0x08, %ymm1, %ymm0, %ymm0

	vmovaps			96(%r10), %ymm13
	vblendps		$ 0x0f, %ymm14, %ymm13, %ymm13
	vpermilps		$ 0xff, %ymm0, %ymm12
	vperm2f128		$ 0x00, %ymm12, %ymm12, %ymm12
	vmulps			%ymm13, %ymm12, %ymm15
	vsubps			%ymm15, %ymm0, %ymm0
	vbroadcastss	16(%r11), %ymm12
	vmulps			%ymm0, %ymm12, %ymm1
	vblendps		$ 0x10, %ymm1, %ymm0, %ymm0

	vmovaps			128(%r10), %ymm13
	vblendps		$ 0x1f, %ymm14, %ymm13, %ymm13
	vpermilps		$ 0x00, %ymm0, %ymm12
	vperm2f128		$ 0x11, %ymm12, %ymm12, %ymm12
	vmulps			%ymm13, %ymm12, %ymm15
	vsubps			%ymm15, %ymm0, %ymm0
	vbroadcastss	20(%r11), %ymm12
	vmulps			%ymm0, %ymm12, %ymm1
	vblendps		$ 0x20, %ymm1, %ymm0, %ymm0

	vmovaps			160(%r10), %ymm13
	vblendps		$ 0x3f, %ymm14, %ymm13, %ymm13
	vpermilps		$ 0x55, %ymm0, %ymm12
	vperm2f128		$ 0x11, %ymm12, %ymm12, %ymm12
	vmulps			%ymm13, %ymm12, %ymm15
	vsubps			%ymm15, %ymm0, %ymm0
	vbroadcastss	24(%r11), %ymm12
	vmulps			%ymm0, %ymm12, %ymm1
	vblendps		$ 0x40, %ymm1, %ymm0, %ymm0

	vmovaps			192(%r10), %ymm13
	vblendps		$ 0x7f, %ymm14, %ymm13, %ymm13
	vpermilps		$ 0xaa, %ymm0, %ymm12
	vperm2f128		$ 0x11, %ymm12, %ymm12, %ymm12
	vmulps			%ymm13, %ymm12, %ymm15
	vsubps			%ymm15, %ymm0, %ymm0
	vbroadcastss	28(%r11), %ymm12
	vmulps			%ymm0, %ymm12, %ymm1
	vblendps		$ 0x80, %ymm1, %ymm0, %ymm0

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trsv_ln_inv_8_lib8)
#endif





// common inner routine with file scope
//
// triangular substitution with vector RHS
//
// input arguments:
// r10  <- E
// r11  <- inv_diag_E
// r12d <- kn
// ymm0 <- [z0 z1 z2 z3]
// ymm12 <- dirty
// ymm13 <- dirty
//
// output arguments:
// r10  <- E
// r11  <- inv_diag_E
// r12d <- kn
// ymm0 <- [z0 z1 z2 z3]
// ymm12 <- dirty
// ymm13 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRSV_LN_INV_8_VS_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_trsv_ln_inv_8_vs_lib8)
#endif
	
	vxorps			%ymm14, %ymm14, %ymm14

	vbroadcastss	0(%r11), %ymm12
	vmulps			%ymm0, %ymm12, %ymm1
	vblendps		$ 0x01, %ymm1, %ymm0, %ymm0
	vmovaps			0(%r10), %ymm13
	vblendps		$ 0x01, %ymm14, %ymm13, %ymm13
	vpermilps		$ 0x00, %ymm0, %ymm12
	vperm2f128		$ 0x00, %ymm12, %ymm12, %ymm12
	vmulps			%ymm13, %ymm12, %ymm15
	vsubps			%ymm15, %ymm0, %ymm0

	cmpl			$ 2, %r12d
	jl				0f // ret

	vbroadcastss	4(%r11), %ymm12
	vmulps			%ymm0, %ymm12, %ymm1
	vblendps		$ 0x02, %ymm1, %ymm0, %ymm0
	vmovaps			32(%r10), %ymm13
	vblendps		$ 0x03, %ymm14, %ymm13, %ymm13
	vpermilps		$ 0x55, %ymm0, %ymm12
	vperm2f128		$ 0x00, %ymm12, %ymm12, %ymm12
	vmulps			%ymm13, %ymm12, %ymm15
	vsubps			%ymm15, %ymm0, %ymm0

	cmpl			$ 3, %r12d
	jl				0f // ret

	vbroadcastss	8(%r11), %ymm12
	vmulps			%ymm0, %ymm12, %ymm1
	vblendps		$ 0x04, %ymm1, %ymm0, %ymm0
	vmovaps			64(%r10), %ymm13
	vblendps		$ 0x07, %ymm14, %ymm13, %ymm13
	vpermilps		$ 0xaa, %ymm0, %ymm12
	vperm2f128		$ 0x00, %ymm12, %ymm12, %ymm12
	vmulps			%ymm13, %ymm12, %ymm15
	vsubps			%ymm15, %ymm0, %ymm0

	cmpl			$ 4, %r12d
	jl				0f // ret

	vbroadcastss	12(%r11), %ymm12
	vmulps			%ymm0, %ymm12, %ymm1
	vblendps		$ 0x08, %ymm1, %ymm0, %ymm0
	vmovaps			96(%r10), %ymm13
	vblendps		$ 0x0f, %ymm14, %ymm13, %ymm13
	vpermilps		$ 0xff, %ymm0, %ymm12
	vperm2f128		$ 0x00, %ymm12, %ymm12, %ymm12
	vmulps			%ymm13, %ymm12, %ymm15
	vsubps			%ymm15, %ymm0, %ymm0

	cmpl			$ 5, %r12d
	jl				0f // ret

	vbroadcastss	16(%r11), %ymm12
	vmulps			%ymm0, %ymm12, %ymm1
	vblendps		$ 0x10, %ymm1, %ymm0, %ymm0
	vmovaps			128(%r10), %ymm13
	vblendps		$ 0x1f, %ymm14, %ymm13, %ymm13
	vpermilps		$ 0x00, %ymm0, %ymm12
	vperm2f128		$ 0x11, %ymm12, %ymm12, %ymm12
	vmulps			%ymm13, %ymm12, %ymm15
	vsubps			%ymm15, %ymm0, %ymm0

	cmpl			$ 6, %r12d
	jl				0f // ret

	vbroadcastss	20(%r11), %ymm12
	vmulps			%ymm0, %ymm12, %ymm1
	vblendps		$ 0x20, %ymm1, %ymm0, %ymm0
	vmovaps			160(%r10), %ymm13
	vblendps		$ 0x3f, %ymm14, %ymm13, %ymm13
	vpermilps		$ 0x55, %ymm0, %ymm12
	vperm2f128		$ 0x11, %ymm12, %ymm12, %ymm12
	vmulps			%ymm13, %ymm12, %ymm15
	vsubps			%ymm15, %ymm0, %ymm0

	cmpl			$ 7, %r12d
	jl				0f // ret

	vbroadcastss	24(%r11), %ymm12
	vmulps			%ymm0, %ymm12, %ymm1
	vblendps		$ 0x40, %ymm1, %ymm0, %ymm0
	vmovaps			192(%r10), %ymm13
	vblendps		$ 0x7f, %ymm14, %ymm13, %ymm13
	vpermilps		$ 0xaa, %ymm0, %ymm12
	vperm2f128		$ 0x11, %ymm12, %ymm12, %ymm12
	vmulps			%ymm13, %ymm12, %ymm15
	vsubps			%ymm15, %ymm0, %ymm0

	cmpl			$ 8, %r12d
	jl				0f // ret

	vbroadcastss	28(%r11), %ymm12
	vmulps			%ymm0, %ymm12, %ymm1
	vblendps		$ 0x80, %ymm1, %ymm0, %ymm0

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trsv_ln_inv_8_vs_lib8)
#endif





// common inner routine with file scope
//
// triangular substitution with vector RHS
//
// input arguments:
// r10  <- E
// r11  <- inv_diag_E
// ymm0 <- [z0 z1 z2 z3]
// ymm12 <- dirty
// ymm13 <- dirty
//
// output arguments:
// r10  <- E
// r11  <- inv_diag_E
// ymm0 <- [z0 z1 z2 z3]
// ymm12 <- dirty
// ymm13 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRSV_LT_INV_8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_trsv_lt_inv_8_lib8)
#endif
	
	vxorps			%ymm14, %ymm14, %ymm14

	vmovaps			0(%r10), %ymm12
	vblendps		$ 0x01, %ymm14, %ymm12, %ymm12
	vmovaps			32(%r10), %ymm13
	vblendps		$ 0x03, %ymm14, %ymm13, %ymm13
	vunpcklps		%ymm13, %ymm12, %ymm8
	vunpckhps		%ymm13, %ymm12, %ymm9

	vmovaps			64(%r10), %ymm12
	vblendps		$ 0x07, %ymm14, %ymm12, %ymm12
	vmovaps			96(%r10), %ymm13
	vblendps		$ 0x0f, %ymm14, %ymm13, %ymm13
	vunpcklps		%ymm13, %ymm12, %ymm10
	vunpckhps		%ymm13, %ymm12, %ymm11

	vshufps			$ 0x44, %ymm10, %ymm8, %ymm7
	vshufps			$ 0xee, %ymm10, %ymm8, %ymm4
	vshufps			$ 0x44, %ymm11, %ymm9, %ymm5
	vshufps			$ 0xee, %ymm11, %ymm9, %ymm6
	vextractf128	$ 0x1, %ymm7, %xmm7
	vextractf128	$ 0x1, %ymm4, %xmm8
	vextractf128	$ 0x1, %ymm5, %xmm9
	vextractf128	$ 0x1, %ymm6, %xmm10

	vmovaps			144(%r10), %xmm12
	vblendps		$ 0x01, %xmm14, %xmm12, %xmm12
	vmovaps			176(%r10), %xmm13
	vblendps		$ 0x03, %xmm14, %xmm13, %xmm13
	vunpcklps		%xmm13, %xmm12, %xmm1
	vunpckhps		%xmm13, %xmm12, %xmm2

	vmovaps			208(%r10), %xmm12
	vblendps		$ 0x07, %xmm14, %xmm12, %xmm12
	vmovaps			240(%r10), %xmm13
	vblendps		$ 0x0f, %xmm14, %xmm13, %xmm13
	vunpcklps		%xmm13, %xmm12, %xmm3
	vunpckhps		%xmm13, %xmm12, %xmm15

	vshufps			$ 0xee, %xmm3, %xmm1, %xmm11
	vshufps			$ 0x44, %xmm15, %xmm2, %xmm12
	vshufps			$ 0xee, %xmm15, %xmm2, %xmm13


	vxorps			%ymm14, %ymm14, %ymm14

	vextractf128	$ 0x1, %ymm0, %xmm1

	vshufps			$ 0xff, %xmm1, %xmm1, %xmm2
	vbroadcastss	28(%r11), %xmm15
	vmulps			%xmm2, %xmm15, %xmm2
	vblendps		$ 0x08, %xmm2, %xmm1, %xmm1
	vmulps			%xmm10, %xmm2, %xmm15
	vsubps			%xmm15, %xmm0, %xmm0
	vmulps			%xmm13, %xmm2, %xmm15
	vsubps			%xmm15, %xmm1, %xmm1

	vshufps			$ 0xaa, %xmm1, %xmm1, %xmm2
	vbroadcastss	24(%r11), %xmm15
	vmulps			%xmm2, %xmm15, %xmm2
	vblendps		$ 0x04, %xmm2, %xmm1, %xmm1
	vmulps			%xmm9, %xmm2, %xmm15
	vsubps			%xmm15, %xmm0, %xmm0
	vmulps			%xmm12, %xmm2, %xmm15
	vsubps			%xmm15, %xmm1, %xmm1

	vshufps			$ 0x55, %xmm1, %xmm1, %xmm2
	vbroadcastss	20(%r11), %xmm15
	vmulps			%xmm2, %xmm15, %xmm2
	vblendps		$ 0x02, %xmm2, %xmm1, %xmm1
	vmulps			%xmm8, %xmm2, %xmm15
	vsubps			%xmm15, %xmm0, %xmm0
	vmulps			%xmm11, %xmm2, %xmm15
	vsubps			%xmm15, %xmm1, %xmm1

	vshufps			$ 0x00, %xmm1, %xmm1, %xmm2
	vbroadcastss	16(%r11), %xmm15
	vmulps			%xmm2, %xmm15, %xmm2
	vblendps		$ 0x01, %xmm2, %xmm1, %xmm1
	vmulps			%xmm7, %xmm2, %xmm15
	vsubps			%xmm15, %xmm0, %xmm0

	vshufps			$ 0xff, %xmm0, %xmm0, %xmm2
	vbroadcastss	12(%r11), %xmm15
	vmulps			%xmm2, %xmm15, %xmm2
	vblendps		$ 0x08, %xmm2, %xmm0, %xmm0
	vmulps			%xmm6, %xmm2, %xmm15
	vsubps			%xmm15, %xmm0, %xmm0

	vshufps			$ 0xaa, %xmm0, %xmm0, %xmm2
	vbroadcastss	8(%r11), %xmm15
	vmulps			%xmm2, %xmm15, %xmm2
	vblendps		$ 0x04, %xmm2, %xmm0, %xmm0
	vmulps			%xmm5, %xmm2, %xmm15
	vsubps			%xmm15, %xmm0, %xmm0

	vshufps			$ 0x55, %xmm0, %xmm0, %xmm2
	vbroadcastss	4(%r11), %xmm15
	vmulps			%xmm2, %xmm15, %xmm2
	vblendps		$ 0x02, %xmm2, %xmm0, %xmm0
	vmulps			%xmm4, %xmm2, %xmm15
	vsubps			%xmm15, %xmm0, %xmm0

	vshufps			$ 0x00, %xmm0, %xmm0, %xmm2
	vbroadcastss	0(%r11), %xmm15
	vmulps			%xmm2, %xmm15, %xmm2
	vblendps		$ 0x01, %xmm2, %xmm0, %xmm0

	vinsertf128		$ 0x1, %xmm1, %ymm0, %ymm0

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trsv_lt_inv_8_lib8)
#endif





// common inner routine with file scope
//
// triangular substitution with vector RHS
//
// input arguments:
// r10  <- E
// r11  <- inv_diag_E
// r12  <- km
// r13  <- kn
// r14  <- x
// ymm0 <- [z0 z1 z2 z3]
// ymm12 <- dirty
// ymm13 <- dirty
//
// output arguments:
// r10  <- E
// r11  <- inv_diag_E
// r12  <- km
// r13  <- kn
// r14  <- x
// ymm0 <- [z0 z1 z2 z3]
// ymm12 <- dirty
// ymm13 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRSV_LT_INV_8_VS_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_trsv_lt_inv_8_vs_lib8)
#endif
	
	vcvtsi2ss	%r13d, %xmm14, %xmm14
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %ymm13
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %ymm13
#endif
	vshufps		$ 0x00, %xmm14, %xmm14, %xmm14
	vinsertf128	$ 0x1, %xmm14, %ymm14, %ymm14
	vsubps		%ymm14, %ymm13, %ymm14

	vmovups		0(%r14), %ymm15
	vblendvps	%ymm14, %ymm0, %ymm15, %ymm0



	vxorps			%ymm14, %ymm14, %ymm14

	vmovaps			0(%r10), %ymm12
	vblendps		$ 0x01, %ymm14, %ymm12, %ymm12
	cmpl	$ 2, %r13d
	jl		1f
	vmovaps			32(%r10), %ymm13
	vblendps		$ 0x03, %ymm14, %ymm13, %ymm13
	vunpcklps		%ymm13, %ymm12, %ymm8
	vunpckhps		%ymm13, %ymm12, %ymm9

	cmpl	$ 3, %r13d
	jl		2f
	vmovaps			64(%r10), %ymm12
	vblendps		$ 0x07, %ymm14, %ymm12, %ymm12
	cmpl	$ 4, %r13d
	jl		3f
	vmovaps			96(%r10), %ymm13
	vblendps		$ 0x0f, %ymm14, %ymm13, %ymm13
	vunpcklps		%ymm13, %ymm12, %ymm10
	vunpckhps		%ymm13, %ymm12, %ymm11

	vshufps			$ 0x44, %ymm10, %ymm8, %ymm7
	vshufps			$ 0xee, %ymm10, %ymm8, %ymm4
	vshufps			$ 0x44, %ymm11, %ymm9, %ymm5
	vshufps			$ 0xee, %ymm11, %ymm9, %ymm6
	vextractf128	$ 0x1, %ymm7, %xmm7
	vextractf128	$ 0x1, %ymm4, %xmm8
	vextractf128	$ 0x1, %ymm5, %xmm9
	vextractf128	$ 0x1, %ymm6, %xmm10

	cmpl	$ 5, %r13d
	jl		4f
	vmovaps			144(%r10), %xmm12
	vblendps		$ 0x01, %xmm14, %xmm12, %xmm12
	cmpl	$ 6, %r13d
	jl		5f
	vmovaps			176(%r10), %xmm13
	vblendps		$ 0x03, %xmm14, %xmm13, %xmm13
	vunpcklps		%xmm13, %xmm12, %xmm1
	vunpckhps		%xmm13, %xmm12, %xmm2

	cmpl	$ 7, %r13d
	jl		6f
	vmovaps			208(%r10), %xmm12
	vblendps		$ 0x07, %xmm14, %xmm12, %xmm12
	cmpl	$ 8, %r13d
	jl		7f
	vmovaps			240(%r10), %xmm13
	vblendps		$ 0x0f, %xmm14, %xmm13, %xmm13
	vunpcklps		%xmm13, %xmm12, %xmm3
	vunpckhps		%xmm13, %xmm12, %xmm15

	vshufps			$ 0xee, %xmm3, %xmm1, %xmm11
	vshufps			$ 0x44, %xmm15, %xmm2, %xmm12
	vshufps			$ 0xee, %xmm15, %xmm2, %xmm13

	jmp		0f



	vmovaps			%ymm14, %ymm12
1:
	vmovaps			%ymm14, %ymm13
	vunpcklps		%ymm13, %ymm12, %ymm8
	vunpckhps		%ymm13, %ymm12, %ymm9

2:
	vmovaps			%ymm14, %ymm12
3:
	vmovaps			%ymm14, %ymm13
	vunpcklps		%ymm13, %ymm12, %ymm10
	vunpckhps		%ymm13, %ymm12, %ymm11

	vshufps			$ 0x44, %ymm10, %ymm8, %ymm7
	vshufps			$ 0xee, %ymm10, %ymm8, %ymm4
	vshufps			$ 0x44, %ymm11, %ymm9, %ymm5
	vshufps			$ 0xee, %ymm11, %ymm9, %ymm6
	vextractf128	$ 0x1, %ymm7, %xmm7
	vextractf128	$ 0x1, %ymm4, %xmm8
	vextractf128	$ 0x1, %ymm5, %xmm9
	vextractf128	$ 0x1, %ymm6, %xmm10

	jmp		8f

4:
	vmovaps			%xmm14, %xmm12
5:
	vmovaps			%xmm14, %xmm13
	vunpcklps		%xmm13, %xmm12, %xmm1
	vunpckhps		%xmm13, %xmm12, %xmm2

6:
	vmovaps			%xmm14, %xmm12
7:
	vmovaps			%xmm14, %xmm13
	vunpcklps		%xmm13, %xmm12, %xmm3
	vunpckhps		%xmm13, %xmm12, %xmm15

	vshufps			$ 0xee, %xmm3, %xmm1, %xmm11
	vshufps			$ 0x44, %xmm15, %xmm2, %xmm12
	vshufps			$ 0xee, %xmm15, %xmm2, %xmm13

8:
	
	vmovaps			%xmm14, %xmm11
	vmovaps			%xmm14, %xmm12
	vmovaps			%xmm14, %xmm13

0:
	vxorps			%ymm14, %ymm14, %ymm14

	vextractf128	$ 0x1, %ymm0, %xmm1

	cmpl	$ 8, %r12d
	jl		0f

	vshufps			$ 0xff, %xmm1, %xmm1, %xmm2
	cmpl	$ 8, %r13d
	jl		1f
	vbroadcastss	28(%r11), %xmm15
	vmulps			%xmm2, %xmm15, %xmm2
	vblendps		$ 0x08, %xmm2, %xmm1, %xmm1
1:
	vmulps			%xmm10, %xmm2, %xmm15
	vsubps			%xmm15, %xmm0, %xmm0
	vmulps			%xmm13, %xmm2, %xmm15
	vsubps			%xmm15, %xmm1, %xmm1

0:
	cmpl	$ 7, %r12d
	jl		0f

	vshufps			$ 0xaa, %xmm1, %xmm1, %xmm2
	cmpl	$ 7, %r13d
	jl		1f
	vbroadcastss	24(%r11), %xmm15
	vmulps			%xmm2, %xmm15, %xmm2
	vblendps		$ 0x04, %xmm2, %xmm1, %xmm1
1:
	vmulps			%xmm9, %xmm2, %xmm15
	vsubps			%xmm15, %xmm0, %xmm0
	vmulps			%xmm12, %xmm2, %xmm15
	vsubps			%xmm15, %xmm1, %xmm1

0:
	cmpl	$ 6, %r12d
	jl		0f

	vshufps			$ 0x55, %xmm1, %xmm1, %xmm2
	cmpl	$ 6, %r13d
	jl		1f
	vbroadcastss	20(%r11), %xmm15
	vmulps			%xmm2, %xmm15, %xmm2
	vblendps		$ 0x02, %xmm2, %xmm1, %xmm1
1:
	vmulps			%xmm8, %xmm2, %xmm15
	vsubps			%xmm15, %xmm0, %xmm0
	vmulps			%xmm11, %xmm2, %xmm15
	vsubps			%xmm15, %xmm1, %xmm1

0:
	cmpl	$ 5, %r12d
	jl		0f

	vshufps			$ 0x00, %xmm1, %xmm1, %xmm2
	cmpl	$ 5, %r13d
	jl		1f
	vbroadcastss	16(%r11), %xmm15
	vmulps			%xmm2, %xmm15, %xmm2
	vblendps		$ 0x01, %xmm2, %xmm1, %xmm1
1:
	vmulps			%xmm7, %xmm2, %xmm15
	vsubps			%xmm15, %xmm0, %xmm0

0:
	cmpl	$ 4, %r12d
	jl		0f

	vshufps			$ 0xff, %xmm0, %xmm0, %xmm2
	cmpl	$ 4, %r13d
	jl		1f
	vbroadcastss	12(%r11), %xmm15
	vmulps			%xmm2, %xmm15, %xmm2
	vblendps		$ 0x08, %xmm2, %xmm0, %xmm0
1:
	vmulps			%xmm6, %xmm2, %xmm15
	vsubps			%xmm15, %xmm0, %xmm0

0:
	cmpl	$ 3, %r12d
	jl		0f

	vshufps			$ 0xaa, %xmm0, %xmm0, %xmm2
	cmpl	$ 3, %r13d
	jl		1f
	vbroadcastss	8(%r11), %xmm15
	vmulps			%xmm2, %xmm15, %xmm2
	vblendps		$ 0x04, %xmm2, %xmm0, %xmm0
1:
	vmulps			%xmm5, %xmm2, %xmm15
	vsubps			%xmm15, %xmm0, %xmm0

0:
	cmpl	$ 2, %r12d
	jl		0f

	vshufps			$ 0x55, %xmm0, %xmm0, %xmm2
	cmpl	$ 2, %r13d
	jl		1f
	vbroadcastss	4(%r11), %xmm15
	vmulps			%xmm2, %xmm15, %xmm2
	vblendps		$ 0x02, %xmm2, %xmm0, %xmm0
1:
	vmulps			%xmm4, %xmm2, %xmm15
	vsubps			%xmm15, %xmm0, %xmm0

0:
	cmpl	$ 1, %r12d
	jl		0f

	vshufps			$ 0x00, %xmm0, %xmm0, %xmm2
	cmpl	$ 1, %r13d
	jl		1f
	vbroadcastss	0(%r11), %xmm15
	vmulps			%xmm2, %xmm15, %xmm2
	vblendps		$ 0x01, %xmm2, %xmm0, %xmm0
1:

0:

	vinsertf128		$ 0x1, %xmm1, %ymm0, %ymm0

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trsv_lt_inv_8_vs_lib8)
#endif





// common inner routine with file scope
//
// blend for ta==n, scale for generic alpha and beta
//
// input arguments:
// r10  <- alpha
// r11  <- beta
// r12  <- y
// ymm0 <- [z0 z1 z2 z3]_a
// ymm1 <- [z0 z1 z2 z3]_b
// ymm2 <- [z0 z1 z2 z3]_c
// ymm3 <- [z0 z1 z2 z3]_d
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10  <- alpha
// r11  <- beta
// r12  <- y
// ymm0 <- [z0 z1 z2 z3]
// ymm1 <- dirty
// ymm2 <- dirty
// ymm3 <- dirty
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_N_SCALE_AB_8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_blend_n_scale_ab_8_lib8)
#endif

	// reduction
	vaddps			%ymm0, %ymm1, %ymm0
	vaddps			%ymm2, %ymm3, %ymm2
	vaddps			%ymm0, %ymm2, %ymm0

	// alpha
	vbroadcastss	0(%r10), %ymm15
	vmulps			%ymm0, %ymm15, %ymm0

	// beta
	vbroadcastss	0(%r11), %ymm15

	vxorps		%ymm14, %ymm14, %ymm14 // 0.0
	vucomiss	%xmm14, %xmm15 // beta==0.0 ?
	je			0f // end

	vmovups			0(%r12), %ymm14
	vmulps			%ymm15, %ymm14, %ymm14
	vaddps			%ymm0, %ymm14, %ymm0

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret
	
	FUN_END(inner_blend_n_scale_ab_8_lib8)
#endif





// common inner routine with file scope
//
// blend for ta==n, scale for alpha=-1.0 and beta=1.0
//
// input arguments:
// r10  <- y
// ymm0 <- [z0 z1 z2 z3]_a
// ymm1 <- [z0 z1 z2 z3]_b
// ymm2 <- [z0 z1 z2 z3]_c
// ymm3 <- [z0 z1 z2 z3]_d
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10  <- y
// ymm0 <- [z0 z1 z2 z3]
// ymm1 <- dirty
// ymm2 <- dirty
// ymm3 <- dirty
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_N_SCALE_M11_8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_blend_n_scale_m11_8_lib8)
#endif

	// reduction
	vaddps	%ymm0, %ymm1, %ymm0
	vaddps	%ymm2, %ymm3, %ymm2
	vaddps	%ymm0, %ymm2, %ymm0

	// beta
	vmovups		0(%r10), %ymm14
	vsubps		%ymm0, %ymm14, %ymm0

#if MACRO_LEVEL>=1
	.endm
#else
	ret
	
	FUN_END(inner_blend_n_scale_m11_8_lib8)
#endif





// common inner routine with file scope
//
// blend for ta==t, scale for generic alpha and beta
//
// input arguments:
// r10  <- alpha
// r11  <- beta
// r12  <- y
// ymm0 <- [z0a z0b z0c z0d]
// ymm1 <- [z1a z1b z1c z1d]
// ymm2 <- [z2a z2b z2c z2d]
// ymm3 <- [z3a z3b z3c z3d]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10  <- alpha
// r11  <- beta
// r12  <- y
// ymm0 <- [z0 z1 z2 z3]
// ymm1 <- dirty
// ymm2 <- dirty
// ymm3 <- dirty
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_T_SCALE_AB_8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_blend_t_scale_ab_8_lib8)
#endif

	// reduction
	vhaddps			%ymm1, %ymm0, %ymm0
	vhaddps			%ymm3, %ymm2, %ymm2
	vhaddps			%ymm5, %ymm4, %ymm4
	vhaddps			%ymm7, %ymm6, %ymm6

	vhaddps			%ymm2, %ymm0, %ymm0
	vhaddps			%ymm6, %ymm4, %ymm4

	vperm2f128		$ 0x20, %ymm4, %ymm0, %ymm1
	vperm2f128		$ 0x13, %ymm0, %ymm4, %ymm0

	vaddps			%ymm0, %ymm1, %ymm0

	// alpha
	vbroadcastss	0(%r10), %ymm15
	vmulps			%ymm0, %ymm15, %ymm0

	// beta
	vbroadcastss	0(%r11), %ymm15

	vxorps		%ymm14, %ymm14, %ymm14 // 0.0
	vucomiss	%xmm14, %xmm15 // beta==0.0 ?
	je			0f // end

	vmovups			0(%r12), %ymm14
	vmulps			%ymm15, %ymm14, %ymm14
	vaddps			%ymm0, %ymm14, %ymm0

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret
	
	FUN_END(inner_blend_t_scale_ab_8_lib8)
#endif





// common inner routine with file scope
//
// blend for ta==t, scale for alpha=-1.0 and beta=1.0
//
// input arguments:
// r10  <- y
// ymm0 <- [z0a z0b z0c z0d]
// ymm1 <- [z1a z1b z1c z1d]
// ymm2 <- [z2a z2b z2c z2d]
// ymm3 <- [z3a z3b z3c z3d]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10  <- y
// ymm0 <- [z0 z1 z2 z3]
// ymm1 <- dirty
// ymm2 <- dirty
// ymm3 <- dirty
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_T_SCALE_M11_8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_blend_t_scale_m11_8_lib8)
#endif

	// reduction
	vhaddps			%ymm1, %ymm0, %ymm0
	vhaddps			%ymm3, %ymm2, %ymm2
	vhaddps			%ymm5, %ymm4, %ymm4
	vhaddps			%ymm7, %ymm6, %ymm6

	vhaddps			%ymm2, %ymm0, %ymm0
	vhaddps			%ymm6, %ymm4, %ymm4

	vperm2f128		$ 0x20, %ymm4, %ymm0, %ymm1
	vperm2f128		$ 0x13, %ymm0, %ymm4, %ymm0

	vaddps			%ymm0, %ymm1, %ymm0

	// beta
	vmovups			0(%r10), %ymm14
	vsubps			%ymm0, %ymm14, %ymm0
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret
	
	FUN_END(inner_blend_t_scale_m11_8_lib8)
#endif





// common inner routine with file scope
//
// store 
//
// input arguments:
// r10  <- z
// ymm0 <- [z0 z1 z2 z3]
//
// output arguments:
// r10  <- z
// ymm0 <- [z0 z1 z2 z3]

#if MACRO_LEVEL>=1
	.macro INNER_STORE_8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_store_8_lib8)
#endif
	
	vmovups %ymm0,  0(%r10)
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_8_lib8)
#endif





// common inner routine with file scope
//
// store vs
//
// input arguments:
// r10   <- D
// r11d   <- km
// ymm0  <- [z0 z1 z2 z3]
// ymm14 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10   <- D
// r11d   <- km
// ymm0  <- [z0 z1 z2 z3]
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_STORE_8_VS_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_store_8_vs_lib8)
#endif
	
	vcvtsi2ss	%r11d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %ymm14
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %ymm14
#endif
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%ymm15, %ymm14, %ymm15

	vmaskmovps	%ymm0, %ymm15,  0(%r10)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_8_vs_lib8)
#endif





// common inner routine with file scope
//
// store gen
//
// input arguments:
// r10   <- D
// r11d  <- k0 : start form (inc)
// r12d  <- k1 : up to (exc)
// ymm0  <- [z0 z1 z2 z3]
// ymm14 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10   <- D
// r11d  <- k0 : start form (inc)
// r12d  <- k1 : up to (exc)
// ymm0  <- [z0 z1 z2 z3]
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_STORE_8_GEN_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_store_8_gen_lib8)
#endif
	
	// compute mask for rows
	vcvtsi2ss	%r11d, %xmm14, %xmm14
	vcvtsi2ss	%r12d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %ymm12
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %ymm12
#endif
	vshufps		$ 0x00, %xmm14, %xmm14, %xmm14
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm14, %ymm14, %ymm14
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%ymm12, %ymm14, %ymm14
	vsubps		%ymm15, %ymm12, %ymm15
	vandps		%ymm14, %ymm15, %ymm15

	vmaskmovps	%ymm0, %ymm15,  0(%r10)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_8_gen_lib8)
#endif





//                            1      2              3          4          5             6          7
// void kernel_sgemv_n_8_lib8(int k, double *alpha, double *A, double *x, double *beta, double *y, double *z);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemv_n_8_lib8)
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3


	// call inner sgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // x

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_N_8_LIB8
#else
	CALL(inner_kernel_gemv_add_n_8_lib8)
#endif


	// call inner blend n scale ab

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11   // beta
	movq	ARG6, %r12   // y

#if MACRO_LEVEL>=1
	INNER_BLEND_N_SCALE_AB_8_LIB8
#else
	CALL(inner_blend_n_scale_ab_8_lib8)
#endif


	// store

	movq	ARG7, %r10 // z 

#if MACRO_LEVEL>=1
	INNER_STORE_8_LIB8
#else
	CALL(inner_store_8_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemv_n_8_lib8)





//                               1      2              3          4          5             6          7          8
// void kernel_sgemv_n_8_vs_lib8(int k, double *alpha, double *A, double *x, double *beta, double *y, double *z, int k1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemv_n_8_vs_lib8)
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3


	// call inner sgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // x

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_N_8_LIB8
#else
	CALL(inner_kernel_gemv_add_n_8_lib8)
#endif


	// call inner blend n scale ab

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11   // beta
	movq	ARG6, %r12   // y

#if MACRO_LEVEL>=1
	INNER_BLEND_N_SCALE_AB_8_LIB8
#else
	CALL(inner_blend_n_scale_ab_8_lib8)
#endif


	// store

	movq	ARG7, %r10 // z 
	movq	ARG8, %r11 // k1 

#if MACRO_LEVEL>=1
	INNER_STORE_8_VS_LIB8
#else
	CALL(inner_store_8_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemv_n_8_vs_lib8)





//                                1      2              3          4          5             6          7          8       9
// void kernel_sgemv_n_8_gen_lib8(int k, double *alpha, double *A, double *x, double *beta, double *y, double *z, int k0, int kq);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemv_n_8_gen_lib8)
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3


	// call inner sgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // x

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_N_8_LIB8
#else
	CALL(inner_kernel_gemv_add_n_8_lib8)
#endif


	// call inner blend n scale ab

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11   // beta
	movq	ARG6, %r12   // y

#if MACRO_LEVEL>=1
	INNER_BLEND_N_SCALE_AB_8_LIB8
#else
	CALL(inner_blend_n_scale_ab_8_lib8)
#endif


	// store

	movq	ARG7, %r10 // z 
	movq	ARG8, %r11 // k1 
	movq	ARG9, %r12 // k2 

#if MACRO_LEVEL>=1
	INNER_STORE_8_GEN_LIB8
#else
	CALL(inner_store_8_gen_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemv_n_8_gen_lib8)





//                            1      2              3          4        5          6             7         8      9
// void kernel_sgemv_t_8_lib8(int k, double *alpha, offA, double *A, int sda, double *x, double *beta, double *y, double *z);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemv_t_8_lib8)
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3
	vmovaps	%ymm0, %ymm4
	vmovaps	%ymm0, %ymm5
	vmovaps	%ymm0, %ymm6
	vmovaps	%ymm0, %ymm7


	// call inner sgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG4, %r11  // A
	movq	ARG5, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG6, %r13  // x
	movq	ARG3, %r14 // offA

#if MACRO_LEVEL>=2
	INNER_EDGE_GEMV_ADD_T_8_LIB8
#else
	CALL(inner_edge_gemv_add_t_8_lib8)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_T_8_LIB8
#else
	CALL(inner_kernel_gemv_add_t_8_lib8)
#endif


	// call inner blender t

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11   // beta
	movq	ARG8, %r12 // y 

#if MACRO_LEVEL>=1
	INNER_BLEND_T_SCALE_AB_8_LIB8
#else
	CALL(inner_blend_t_scale_ab_8_lib8)
#endif


	// store

	movq	ARG9, %r10 // z 

#if MACRO_LEVEL>=1
	INNER_STORE_8_LIB8
#else
	CALL(inner_store_8_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemv_t_8_lib8)





//                               1      2              3          4        5          6             7         8           9     10
// void kernel_sgemv_t_8_vs_lib8(int k, double *alpha, offA, double *A, int sda, double *x, double *beta, double *y, double *z, int k1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemv_t_8_vs_lib8)
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3
	vmovaps	%ymm0, %ymm4
	vmovaps	%ymm0, %ymm5
	vmovaps	%ymm0, %ymm6
	vmovaps	%ymm0, %ymm7


	// call inner sgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG4, %r11  // A
	movq	ARG5, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG6, %r13  // x
	movq	ARG3, %r14 // offA

#if MACRO_LEVEL>=2
	INNER_EDGE_GEMV_ADD_T_8_LIB8
#else
	CALL(inner_edge_gemv_add_t_8_lib8)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_T_8_LIB8
#else
	CALL(inner_kernel_gemv_add_t_8_lib8)
#endif


	// call inner blender t

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11   // beta
	movq	ARG8, %r12 // y 

#if MACRO_LEVEL>=1
	INNER_BLEND_T_SCALE_AB_8_LIB8
#else
	CALL(inner_blend_t_scale_ab_8_lib8)
#endif


	// store

	movq	ARG9, %r10 // z 
	movq	ARG10, %r11 // km 

#if MACRO_LEVEL>=1
	INNER_STORE_8_VS_LIB8
#else
	CALL(inner_store_8_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemv_t_8_vs_lib8)





//                                 1      2          3                   4          5          6
// void kernel_strsv_ln_inv_8_lib8(int k, double *A, double *inv_diag_A, double *x, double *y, double *z);

	.p2align 4,,15
	GLOB_FUN_START(kernel_strsv_ln_inv_8_lib8)
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3
	vmovaps	%ymm0, %ymm4
	vmovaps	%ymm0, %ymm5
	vmovaps	%ymm0, %ymm6
	vmovaps	%ymm0, %ymm7


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG4, %r12  // x

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_N_8_LIB8
#else
	CALL(inner_kernel_gemv_add_n_8_lib8)
#endif

	movq	%r11, %r13 // A+k*sizeof(double)


	// call inner blender n

	movq	ARG5, %r10   // y

#if MACRO_LEVEL>=1
	INNER_BLEND_N_SCALE_M11_8_LIB8
#else
	CALL(inner_blend_n_scale_m11_8_lib8)
#endif


	// solution

	movq	%r13, %r10 // A+k*sizeof(double)
	movq	ARG3, %r11 // inv_diag_A

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSV_LN_INV_8_LIB8
#else
	CALL(inner_edge_trsv_ln_inv_8_lib8)
#endif


	// store

	movq	ARG6, %r10 // z 

#if MACRO_LEVEL>=1
	INNER_STORE_8_LIB8
#else
	CALL(inner_store_8_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_strsv_ln_inv_8_lib8)





//                                    1      2          3                   4          5          6          7       8
// void kernel_strsv_ln_inv_8_vs_lib8(int k, double *A, double *inv_diag_A, double *x, double *y, double *z, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_strsv_ln_inv_8_vs_lib8)
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3
	vmovaps	%ymm0, %ymm4
	vmovaps	%ymm0, %ymm5
	vmovaps	%ymm0, %ymm6
	vmovaps	%ymm0, %ymm7


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG4, %r12  // x

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_N_8_LIB8
#else
	CALL(inner_kernel_gemv_add_n_8_lib8)
#endif

	movq	%r11, %r13 // A+k*sizeof(double)


	// call inner blender n

	movq	ARG5, %r10   // y

#if MACRO_LEVEL>=1
	INNER_BLEND_N_SCALE_M11_8_LIB8
#else
	CALL(inner_blend_n_scale_m11_8_lib8)
#endif


	// solution

	movq	%r13, %r10 // A+k*sizeof(double)
	movq	ARG3, %r11 // inv_diag_A
	movq	ARG8, %r12 // kn

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSV_LN_INV_8_VS_LIB8
#else
	CALL(inner_edge_trsv_ln_inv_8_vs_lib8)
#endif


	// store

	movq	ARG6, %r10 // z 
	movq	ARG7, %r11 // km

#if MACRO_LEVEL>=1
	INNER_STORE_8_VS_LIB8
#else
	CALL(inner_store_8_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_strsv_ln_inv_8_vs_lib8)





//                                 1      2          3        4                   5          6          7
// void kernel_strsv_lt_inv_8_lib8(int k, double *A, int sda, double *inv_diag_A, double *x, double *y, double *z);

	.p2align 4,,15
	GLOB_FUN_START(kernel_strsv_lt_inv_8_lib8)
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3
	vmovaps	%ymm0, %ymm4
	vmovaps	%ymm0, %ymm5
	vmovaps	%ymm0, %ymm6
	vmovaps	%ymm0, %ymm7


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	subl	$ 8, %r10d
	movq	ARG2, %r11 // A
	movq	ARG3, %r12
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	addq	%r12, %r11 // A+8*sda*sizeof(float)
	movq	ARG5, %r13 // x
	addq	$ 32, %r13 // x+8 

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_T_8_LIB8
#else
	CALL(inner_kernel_gemv_add_t_8_lib8)
#endif


	// call inner blender t

	movq	ARG6, %r10 // y 

#if MACRO_LEVEL>=1
	INNER_BLEND_T_SCALE_M11_8_LIB8
#else
	CALL(inner_blend_t_scale_m11_8_lib8)
#endif


	// solution

	movq	ARG2, %r10 // A
	movq	ARG4, %r11 // inv_diag_A

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSV_LT_INV_8_LIB8
#else
	CALL(inner_edge_trsv_lt_inv_8_lib8)
#endif


	// store

	movq	ARG7, %r10 // z 

#if MACRO_LEVEL>=1
	INNER_STORE_8_LIB8
#else
	CALL(inner_store_8_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_strsv_lt_inv_8_lib8)





//                                    1      2          3        4                   5          6          7          8      9
// void kernel_strsv_lt_inv_8_vs_lib8(int k, double *A, int sda, double *inv_diag_A, double *x, double *y, double *z, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_strsv_lt_inv_8_vs_lib8)
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3
	vmovaps	%ymm0, %ymm4
	vmovaps	%ymm0, %ymm5
	vmovaps	%ymm0, %ymm6
	vmovaps	%ymm0, %ymm7


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	subl	$ 8, %r10d
	movq	ARG2, %r11 // A
	movq	ARG3, %r12
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	addq	%r12, %r11 // A+8*sda*sizeof(float)
	movq	ARG5, %r13 // x
	addq	$ 32, %r13 // x+8 

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_T_8_LIB8
#else
	CALL(inner_kernel_gemv_add_t_8_lib8)
#endif


	// call inner blender t

	movq	ARG6, %r10 // y 

#if MACRO_LEVEL>=1
	INNER_BLEND_T_SCALE_M11_8_LIB8
#else
	CALL(inner_blend_t_scale_m11_8_lib8)
#endif


	// solution

	movq	ARG2, %r10 // A
	movq	ARG4, %r11 // inv_diag_A
	movq	ARG8, %r12 // km
	movq	ARG9, %r13 // kn
	movq	ARG5, %r14 // x

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSV_LT_INV_8_VS_LIB8
#else
	CALL(inner_edge_trsv_lt_inv_8_vs_lib8)
#endif


	// store

	movq	ARG7, %r10 // z 
	movq	ARG9, %r11 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_8_VS_LIB8
#else
	CALL(inner_store_8_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_strsv_lt_inv_8_vs_lib8)





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
	.float	0.5
	.float	1.5
	.float	2.5
	.float	3.5
	.float	4.5
	.float	5.5
	.float	6.5
	.float	7.5




#if defined(OS_LINUX)
	.section	.note.GNU-stack,"",@progbits
#elif defined(OS_MAC)
	.subsections_via_symbols
#endif

