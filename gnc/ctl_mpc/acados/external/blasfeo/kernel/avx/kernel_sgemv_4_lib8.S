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
	.macro INNER_KERNEL_GEMV_ADD_T_4_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemv_add_t_4_lib8)
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
		
	sall	$ 2, %r10d // *sizeof(float)
	addq	%r10, %r11
	addq	%r10, %r13
	xorl	%r10d, %r10d
	
	
2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemv_add_t_4_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- bs*sda*sizeof(double) = 32*sda
// r13   <- x_t
// r14   <- z_n
// ymm0  <- [z_t_0a z_t_0b z_t_0c z_t_0d]
// ymm1  <- [z_t_1a z_t_1b z_t_1c z_t_1d]
// ymm2  <- [z_t_2a z_t_2b z_t_2c z_t_2d]
// ymm3  <- [z_t_3a z_t_3b z_t_3c z_t_3d]
// ymm6  <- x_n_0
// ymm7  <- x_n_1
// ymm8  <- x_n_2
// ymm9  <- x_n_3
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
// r13   <- x_t+k*sizeof(double)
// r14   <- z_n+k*sizeof(double)
// ymm0  <- [z_t_0a z_t_0b z_t_0c z_t_0d]
// ymm1  <- [z_t_1a z_t_1b z_t_1c z_t_1d]
// ymm2  <- [z_t_2a z_t_2b z_t_2c z_t_2d]
// ymm3  <- [z_t_3a z_t_3b z_t_3c z_t_3d]
// ymm6  <- x_n_0
// ymm7  <- x_n_1
// ymm8  <- x_n_2
// ymm9  <- x_n_3
// ymm10 <- dirty
// ymm11 <- dirty
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMV_ADD_NT_4_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemv_add_nt_4_lib8)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	cmpl	$ 8, %r10d
	jl		0f // clean-up loop

	// main loop
	.p2align 3
1: // main loop
	
	vmovups	0(%r13), %ymm12
	vmovups	0(%r14), %ymm13

	vmovaps	0(%r11), %ymm14
	vmulps	%ymm14, %ymm12, %ymm15
	vaddps	%ymm0, %ymm15, %ymm0
	vmulps	%ymm14, %ymm6, %ymm15
	vaddps	%ymm13, %ymm15, %ymm13
	
	subl	$ 8, %r10d

	vmovaps	32(%r11), %ymm14
	vmulps	%ymm14, %ymm12, %ymm15
	vaddps	%ymm1, %ymm15, %ymm1
	vmulps	%ymm14, %ymm7, %ymm15
	vaddps	%ymm13, %ymm15, %ymm13
	
	vmovaps	64(%r11), %ymm14
	vmulps	%ymm14, %ymm12, %ymm15
	vaddps	%ymm2, %ymm15, %ymm2
	vmulps	%ymm14, %ymm8, %ymm15
	vaddps	%ymm13, %ymm15, %ymm13

	vmovaps	96(%r11), %ymm14
	vmulps	%ymm14, %ymm12, %ymm15
	vaddps	%ymm3, %ymm15, %ymm3
	vmulps	%ymm14, %ymm9, %ymm15
	vaddps	%ymm13, %ymm15, %ymm13
	
	vmovups	%ymm13, 0(%r14) 

	addq	%r12, %r11
	addq	$ 32, %r13
	addq	$ 32, %r14
	
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
	vshufps		$ 0x0, %xmm14, %xmm14, %xmm14
	vinsertf128	$ 0x1, %xmm14, %ymm14, %ymm14
	vsubps		%ymm14, %ymm13, %ymm11

	vmaskmovps	0(%r13), %ymm11, %ymm12
	vmaskmovps	0(%r14), %ymm11, %ymm13

//	vmovups	%ymm14, -32(%rsp) // spill mask to stack

//	vmovups	-32(%rsp), %ymm14 // load mask form stack
	vmaskmovps	0(%r11), %ymm11, %ymm14
	vmulps	%ymm14, %ymm12, %ymm15
	vaddps	%ymm0, %ymm15, %ymm0
	vmulps	%ymm14, %ymm6, %ymm15
	vaddps	%ymm13, %ymm15, %ymm13
	
//	vmovups	-32(%rsp), %ymm14 // load mask form stack
	vmaskmovps	32(%r11), %ymm11, %ymm14
	vmulps	%ymm14, %ymm12, %ymm15
	vaddps	%ymm1, %ymm15, %ymm1
	vmulps	%ymm14, %ymm7, %ymm15
	vaddps	%ymm13, %ymm15, %ymm13
	
//	vmovups	-32(%rsp), %ymm14 // load mask form stack
	vmaskmovps	64(%r11), %ymm11, %ymm14
	vmulps	%ymm14, %ymm12, %ymm15
	vaddps	%ymm2, %ymm15, %ymm2
	vmulps	%ymm14, %ymm8, %ymm15
	vaddps	%ymm13, %ymm15, %ymm13

//	vmovups	-32(%rsp), %ymm14 // load mask form stack
	vmaskmovps	96(%r11), %ymm11, %ymm14
	vmulps	%ymm14, %ymm12, %ymm15
	vaddps	%ymm3, %ymm15, %ymm3
	vmulps	%ymm14, %ymm9, %ymm15
	vaddps	%ymm13, %ymm15, %ymm13
		
//	vmovups	-32(%rsp), %ymm14 // load mask form stack
	vmaskmovps	%ymm13, %ymm11, 0(%r14)

	sall	$ 2, %r10d // *sizeof(float)
	addq	%r10, %r11
	addq	%r10, %r13
	addq	%r10, %r14
	xorl	%r10d, %r10d
	
	
2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemv_add_nt_4_lib8)
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
	.macro INNER_EDGE_GEMV_ADD_T_4_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_gemv_add_t_4_lib8)
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

	addq	$ 32, %r13 // x + 4
	addq	%r12, %r11 // A + bs*sda
		
	addl	%r14d, %r10d
	subl	$ 8, %r10d // kmax - (8-offA)
	
0: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_edge_gemv_add_t_4_lib8)
#endif





#if 0
// TODO

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
// ymm0 <- [z0 z1 z2 z3]
// ymm12 <- dirty
// ymm13 <- dirty
//
// output arguments:
// r10  <- E
// r11  <- inv_diag_E
// r12  <- km
// r13  <- kn
// ymm0 <- [z0 z1 z2 z3]
// ymm12 <- dirty
// ymm13 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRSV_LT_INV_8_VS_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_trsv_lt_inv_8_vs_lib8)
#endif
	
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
	vbroadcastss	28(%r11), %xmm15
	vmulps			%xmm2, %xmm15, %xmm2
	vblendps		$ 0x08, %xmm2, %xmm1, %xmm1
	vmulps			%xmm10, %xmm2, %xmm15
	vsubps			%xmm15, %xmm0, %xmm0
	vmulps			%xmm13, %xmm2, %xmm15
	vsubps			%xmm15, %xmm1, %xmm1

0:
	cmpl	$ 7, %r12d
	jl		0f

	vshufps			$ 0xaa, %xmm1, %xmm1, %xmm2
	vbroadcastss	24(%r11), %xmm15
	vmulps			%xmm2, %xmm15, %xmm2
	vblendps		$ 0x04, %xmm2, %xmm1, %xmm1
	vmulps			%xmm9, %xmm2, %xmm15
	vsubps			%xmm15, %xmm0, %xmm0
	vmulps			%xmm12, %xmm2, %xmm15
	vsubps			%xmm15, %xmm1, %xmm1

0:
	cmpl	$ 6, %r12d
	jl		0f

	vshufps			$ 0x55, %xmm1, %xmm1, %xmm2
	vbroadcastss	20(%r11), %xmm15
	vmulps			%xmm2, %xmm15, %xmm2
	vblendps		$ 0x02, %xmm2, %xmm1, %xmm1
	vmulps			%xmm8, %xmm2, %xmm15
	vsubps			%xmm15, %xmm0, %xmm0
	vmulps			%xmm11, %xmm2, %xmm15
	vsubps			%xmm15, %xmm1, %xmm1

0:
	cmpl	$ 5, %r12d
	jl		0f

	vshufps			$ 0x00, %xmm1, %xmm1, %xmm2
	vbroadcastss	16(%r11), %xmm15
	vmulps			%xmm2, %xmm15, %xmm2
	vblendps		$ 0x01, %xmm2, %xmm1, %xmm1
	vmulps			%xmm7, %xmm2, %xmm15
	vsubps			%xmm15, %xmm0, %xmm0

0:
	cmpl	$ 4, %r12d
	jl		0f

	vshufps			$ 0xff, %xmm0, %xmm0, %xmm2
	vbroadcastss	12(%r11), %xmm15
	vmulps			%xmm2, %xmm15, %xmm2
	vblendps		$ 0x08, %xmm2, %xmm0, %xmm0
	vmulps			%xmm6, %xmm2, %xmm15
	vsubps			%xmm15, %xmm0, %xmm0

0:
	cmpl	$ 3, %r12d
	jl		0f

	vshufps			$ 0xaa, %xmm0, %xmm0, %xmm2
	vbroadcastss	8(%r11), %xmm15
	vmulps			%xmm2, %xmm15, %xmm2
	vblendps		$ 0x04, %xmm2, %xmm0, %xmm0
	vmulps			%xmm5, %xmm2, %xmm15
	vsubps			%xmm15, %xmm0, %xmm0

0:
	cmpl	$ 2, %r12d
	jl		0f

	vshufps			$ 0x55, %xmm0, %xmm0, %xmm2
	vbroadcastss	4(%r11), %xmm15
	vmulps			%xmm2, %xmm15, %xmm2
	vblendps		$ 0x02, %xmm2, %xmm0, %xmm0
	vmulps			%xmm4, %xmm2, %xmm15
	vsubps			%xmm15, %xmm0, %xmm0

0:
	cmpl	$ 1, %r12d
	jl		0f

	vshufps			$ 0x00, %xmm0, %xmm0, %xmm2
	vbroadcastss	0(%r11), %xmm15
	vmulps			%xmm2, %xmm15, %xmm2
	vblendps		$ 0x01, %xmm2, %xmm0, %xmm0

0:

	vinsertf128		$ 0x1, %xmm1, %ymm0, %ymm0

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trsv_lt_inv_8_vs_lib8)
#endif

#endif





// common inner routine with file scope
//
// input arguments:
// r10   <- kmax
// r11   <- A
// r12   <- bs*sda*sizeof(double) = 32*sda
// r13   <- x_t
// r14   <- z_n
// r15   <- offA
// ymm0  <- [z_t_0a z_t_0b z_t_0c z_t_0d]
// ymm1  <- [z_t_1a z_t_1b z_t_1c z_t_1d]
// ymm2  <- [z_t_2a z_t_2b z_t_2c z_t_2d]
// ymm3  <- [z_t_3a z_t_3b z_t_3c z_t_3d]
// ymm6  <- x_n_0
// ymm7  <- x_n_1
// ymm8  <- x_n_2
// ymm9  <- x_n_3
// ymm10 <- dirty
// ymm11 <- dirty
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

//
// output arguments:
// r10   <- kmax-4
// r11   <- A+4*k*sizeof(double)
// r12   <- bs*sda*sizeof(double) = 32*sda
// r13   <- x_t+k*sizeof(double)
// r14   <- z_n+k*sizeof(double)
// r15   <- offA
// ymm0  <- [z_t_0a z_t_0b z_t_0c z_t_0d]
// ymm1  <- [z_t_1a z_t_1b z_t_1c z_t_1d]
// ymm2  <- [z_t_2a z_t_2b z_t_2c z_t_2d]
// ymm3  <- [z_t_3a z_t_3b z_t_3c z_t_3d]
// ymm6  <- x_n_0
// ymm7  <- x_n_1
// ymm8  <- x_n_2
// ymm9  <- x_n_3
// ymm10 <- dirty
// ymm11 <- dirty
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=2
	.macro INNER_EDGE_SYMV_ADD_NT_4L_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_symv_add_nt_4l_lib8)
#endif

	movl	$ 8, %eax
	cmpl	%eax, %r10d
	jge		0f
	movl	%r10d, %eax
0:
	subl	%r15d, %eax

	vcvtsi2ss	%eax, %xmm14, %xmm14
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %ymm13
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %ymm13
#endif
	vshufps		$ 0x0, %xmm14, %xmm14, %xmm14
	vinsertf128	$ 0x1, %xmm14, %ymm14, %ymm14
	vsubps		%ymm14, %ymm13, %ymm11

	vmaskmovps	0(%r13), %ymm11, %ymm12
	vmaskmovps	0(%r14), %ymm11, %ymm13

	vmaskmovps	0(%r11), %ymm11, %ymm14
	vmulps		%ymm14, %ymm12, %ymm15
	vaddps		%ymm0, %ymm15, %ymm0
	vxorps		%ymm15, %ymm15, %ymm15
	vblendps	$ 0x01, %ymm15, %ymm14, %ymm14
	vmulps		%ymm14, %ymm6, %ymm15
	vaddps		%ymm13, %ymm15, %ymm13
	
	vmaskmovps	32(%r11), %ymm11, %ymm14
	vxorps		%ymm15, %ymm15, %ymm15
	vblendps	$ 0x01, %ymm15, %ymm14, %ymm14
	vmulps		%ymm14, %ymm12, %ymm15
	vaddps		%ymm1, %ymm15, %ymm1
	vxorps		%ymm15, %ymm15, %ymm15
	vblendps	$ 0x03, %ymm15, %ymm14, %ymm14
	vmulps		%ymm14, %ymm7, %ymm15
	vaddps		%ymm13, %ymm15, %ymm13
	
	vmaskmovps	64(%r11), %ymm11, %ymm14
	vxorps		%ymm15, %ymm15, %ymm15
	vblendps	$ 0x03, %ymm15, %ymm14, %ymm14
	vmulps		%ymm14, %ymm12, %ymm15
	vaddps		%ymm2, %ymm15, %ymm2
	vxorps		%ymm15, %ymm15, %ymm15
	vblendps	$ 0x07, %ymm15, %ymm14, %ymm14
	vmulps		%ymm14, %ymm8, %ymm15
	vaddps		%ymm13, %ymm15, %ymm13

	vmaskmovps	96(%r11), %ymm11, %ymm14
	vxorps		%ymm15, %ymm15, %ymm15
	vblendps	$ 0x07, %ymm15, %ymm14, %ymm14
	vmulps		%ymm14, %ymm12, %ymm15
	vaddps		%ymm3, %ymm15, %ymm3
	vxorps		%ymm15, %ymm15, %ymm15
	vblendps	$ 0x0f, %ymm15, %ymm14, %ymm14
	vmulps		%ymm14, %ymm9, %ymm15
	vaddps		%ymm13, %ymm15, %ymm13
	
	vmaskmovps	%ymm13, %ymm11, 0(%r14)

	subl	%eax, %r10d

	sall	$ 2, %eax // *sizeof(float)
	addq	%rax, %r11
	subq	$ 32, %r11
	addq	%r12, %r11
	addq	%rax, %r13
	addq	%rax, %r14
	
#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_edge_symv_add_nt_4l_lib8)
#endif






#if MACRO_LEVEL>=2
	.macro INNER_EDGE_SYMV_ADD_NT_4R_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_symv_add_nt_4r_lib8)
#endif

	movl	$ 4, %eax
	cmpl	%eax, %r10d
	jge		0f
	movl	%r10d, %eax
0:
	subl	%r15d, %eax

	vcvtsi2ss	%eax, %xmm14, %xmm14
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %xmm13
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %xmm13
#endif
	vshufps		$ 0x0, %xmm14, %xmm14, %xmm14
//	vinsertf128	$ 0x1, %xmm14, %ymm14, %ymm14
	vsubps		%xmm14, %xmm13, %xmm11

	vmaskmovps	0(%r13), %xmm11, %xmm12
	vmaskmovps	0(%r14), %xmm11, %xmm13

	vmaskmovps	0(%r11), %xmm11, %xmm14
	vmulps		%xmm14, %xmm12, %xmm15
	vaddps		%xmm0, %xmm15, %xmm0
	vxorps		%xmm15, %xmm15, %xmm15
	vblendps	$ 0x01, %xmm15, %xmm14, %xmm14
	vmulps		%xmm14, %xmm6, %xmm15
	vaddps		%xmm13, %xmm15, %xmm13
	
	vmaskmovps	32(%r11), %xmm11, %xmm14
	vxorps		%xmm15, %xmm15, %xmm15
	vblendps	$ 0x01, %xmm15, %xmm14, %xmm14
	vmulps		%xmm14, %xmm12, %xmm15
	vaddps		%xmm1, %xmm15, %xmm1
	vxorps		%xmm15, %xmm15, %xmm15
	vblendps	$ 0x03, %xmm15, %xmm14, %xmm14
	vmulps		%xmm14, %xmm7, %xmm15
	vaddps		%xmm13, %xmm15, %xmm13
	
	vmaskmovps	64(%r11), %xmm11, %xmm14
	vxorps		%xmm15, %xmm15, %xmm15
	vblendps	$ 0x03, %xmm15, %xmm14, %xmm14
	vmulps		%xmm14, %xmm12, %xmm15
	vaddps		%xmm2, %xmm15, %xmm2
	vxorps		%xmm15, %xmm15, %xmm15
	vblendps	$ 0x07, %xmm15, %xmm14, %xmm14
	vmulps		%xmm14, %xmm8, %xmm15
	vaddps		%xmm13, %xmm15, %xmm13

	vmaskmovps	96(%r11), %xmm11, %xmm14
	vxorps		%xmm15, %xmm15, %xmm15
	vblendps	$ 0x07, %xmm15, %xmm14, %xmm14
	vmulps		%xmm14, %xmm12, %xmm15
	vaddps		%xmm3, %xmm15, %xmm3
//	vxorps		%xmm15, %xmm15, %xmm15
//	vblendps	$ 0x0f, %xmm15, %xmm14, %xmm14
//	vmulps		%xmm14, %xmm9, %xmm15
//	vaddps		%xmm13, %xmm15, %xmm13
	
	vmaskmovps	%xmm13, %xmm11, 0(%r14)

	subl	%eax, %r10d

	sall	$ 2, %eax // *sizeof(float)
	addq	%rax, %r11
	subq	$ 32, %r11
	addq	%r12, %r11
	addq	%rax, %r13
	addq	%rax, %r14
	
#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_edge_symv_add_nt_4r_lib8)
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
	.macro INNER_BLEND_T_SCALE_AB_4_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_blend_t_scale_ab_4_lib8)
#endif

	// reduction
	vhaddps			%ymm1, %ymm0, %ymm0
	vhaddps			%ymm3, %ymm2, %ymm2

	vhaddps			%ymm2, %ymm0, %ymm0

	vextractf128	$ 0x1, %ymm0, %xmm1

	vaddps			%xmm0, %xmm1, %xmm0

	// alpha
	vbroadcastss	0(%r10), %xmm15
	vmulps			%xmm0, %xmm15, %xmm0

	// beta
	vbroadcastss	0(%r11), %xmm15

	vxorps		%ymm14, %ymm14, %ymm14 // 0.0
	vucomiss	%xmm14, %xmm15 // beta==0.0 ?
	je			0f // end

	vmovups			0(%r12), %xmm14
	vmulps			%xmm15, %xmm14, %xmm14
	vaddps			%xmm0, %xmm14, %xmm0

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret
	
	FUN_END(inner_blend_t_scale_ab_4_lib8)
#endif





// common inner routine with file scope
//
// blend for ta==t, scale for generic alpha and beta=1.0
//
// input arguments:
// r10  <- alpha
// r11  <- y
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
// r11  <- y
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
	.macro INNER_BLEND_T_SCALE_A1_4_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_blend_t_scale_a1_4_lib8)
#endif

	// reduction
	vhaddps			%ymm1, %ymm0, %ymm0
	vhaddps			%ymm3, %ymm2, %ymm2

	vhaddps			%ymm2, %ymm0, %ymm0

	vextractf128	$ 0x1, %ymm0, %xmm1

	vaddps			%xmm0, %xmm1, %xmm0

	// alpha
	vbroadcastss	0(%r10), %xmm15
	vmulps			%xmm0, %xmm15, %xmm0

	// beta
	vmovups			0(%r11), %xmm14
	vaddps			%xmm0, %xmm14, %xmm0
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret
	
	FUN_END(inner_blend_t_scale_a1_4_lib8)
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
	.macro INNER_BLEND_T_SCALE_M11_4_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_blend_t_scale_m11_4_lib8)
#endif

	// reduction
	vhaddps			%ymm1, %ymm0, %ymm0
	vhaddps			%ymm3, %ymm2, %ymm2

	vhaddps			%ymm2, %ymm0, %ymm0

	vextractf128	$ 0x1, %ymm0, %xmm1

	vaddps			%xmm0, %xmm1, %xmm0

	// beta
	vmovups			0(%r10), %xmm14
	vsubps			%xmm0, %xmm14, %xmm0
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret
	
	FUN_END(inner_blend_t_scale_m11_4_lib8)
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
	.macro INNER_STORE_4_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_store_4_lib8)
#endif
	
	vmovups %xmm0,  0(%r10)
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4_lib8)
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
	.macro INNER_STORE_4_VS_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_store_4_vs_lib8)
#endif
	
	vcvtsi2ss	%r11d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %xmm14
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %xmm14
#endif
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
//	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%xmm15, %xmm14, %xmm15

	vmaskmovps	%xmm0, %xmm15,  0(%r10)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4_vs_lib8)
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
	.macro INNER_STORE_4_GEN_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_store_4_gen_lib8)
#endif
	
	// compute mask for rows
	vcvtsi2ss	%r11d, %xmm14, %xmm14
	vcvtsi2ss	%r12d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %xmm12
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %xmm12
#endif
	vshufps		$ 0x00, %xmm14, %xmm14, %xmm14
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
//	vinsertf128	$ 0x1, %xmm14, %ymm14, %ymm14
//	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%xmm12, %xmm14, %xmm14
	vsubps		%xmm15, %xmm12, %xmm15
	vandps		%xmm14, %xmm15, %xmm15

	vmaskmovps	%xmm0, %xmm15,  0(%r10)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4_gen_lib8)
#endif





//                            1      2              3          4        5          6             7         8          9
// void kernel_sgemv_t_4_lib8(int k, double *alpha, int offA, double *A, int sda, double *x, double *beta, double *y, double *z);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemv_t_4_lib8)
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3


	// call inner sgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG4, %r11  // A
	movq	ARG5, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG6, %r13  // x
	movq	ARG3, %r14 // offA

#if MACRO_LEVEL>=2
	INNER_EDGE_GEMV_ADD_T_4_LIB8
#else
	CALL(inner_edge_gemv_add_t_4_lib8)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_T_4_LIB8
#else
	CALL(inner_kernel_gemv_add_t_4_lib8)
#endif


	// call inner blender t

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11   // beta
	movq	ARG8, %r12 // y 

#if MACRO_LEVEL>=1
	INNER_BLEND_T_SCALE_AB_4_LIB8
#else
	CALL(inner_blend_t_scale_ab_4_lib8)
#endif


	// store

	movq	ARG9, %r10 // z 

#if MACRO_LEVEL>=1
	INNER_STORE_4_LIB8
#else
	CALL(inner_store_4_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemv_t_4_lib8)





//                               1      2              3          4        5          6             7         8           9         10
// void kernel_sgemv_t_4_vs_lib8(int k, double *alpha, int offA, double *A, int sda, double *x, double *beta, double *y, double *z, int k1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemv_t_4_vs_lib8)
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3


	// call inner sgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG4, %r11  // A
	movq	ARG5, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG6, %r13  // x
	movq	ARG3, %r14 // offA

#if MACRO_LEVEL>=2
	INNER_EDGE_GEMV_ADD_T_4_LIB8
#else
	CALL(inner_edge_gemv_add_t_4_lib8)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_T_4_LIB8
#else
	CALL(inner_kernel_gemv_add_t_4_lib8)
#endif


	// call inner blender t

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11   // beta
	movq	ARG8, %r12 // y 

#if MACRO_LEVEL>=1
	INNER_BLEND_T_SCALE_AB_4_LIB8
#else
	CALL(inner_blend_t_scale_ab_4_lib8)
#endif


	// store

	movq	ARG9, %r10 // z 
	movq	ARG10, %r11 // km 

#if MACRO_LEVEL>=1
	INNER_STORE_4_VS_LIB8
#else
	CALL(inner_store_4_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemv_t_4_vs_lib8)





#if 0
// TODO

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

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSV_LT_INV_8_VS_LIB8
#else
	CALL(inner_edge_trsv_lt_inv_8_vs_lib8)
#endif


	// store

	movq	ARG7, %r10 // z 
	movq	ARG8, %r11 // km 

#if MACRO_LEVEL>=1
	INNER_STORE_8_VS_LIB8
#else
	CALL(inner_store_8_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_strsv_lt_inv_8_vs_lib8)

#endif





//                             1      2                3                4          5        6            7            8               9            10           11
// void kernel_sgemv_nt_4_lib8(int k, double *alpha_n, double *alpha_t, double *A, int sda, double *x_n, double *x_t, double *beta_t, double *y_t, double *z_n, double *z_t);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemv_nt_4_lib8)
	
	PROLOGUE

	// zero accumulation registers y_t

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3

	// initialize x_n
	movq	ARG2, %r10 // alpha_n
	vbroadcastss 0(%r10), %ymm15

	movq	ARG6, %r10 // x_n

	vbroadcastss 0(%r10), %ymm6
	vmulps		%ymm15, %ymm6, %ymm6
	vbroadcastss 4(%r10), %ymm7
	vmulps		%ymm15, %ymm7, %ymm7
	vbroadcastss 8(%r10), %ymm8
	vmulps		%ymm15, %ymm8, %ymm8
	vbroadcastss 12(%r10), %ymm9
	vmulps		%ymm15, %ymm9, %ymm9


	// inner kernel dgemv nt

	movq	ARG1, %r10 // k
	movq	ARG4, %r11  // A
	movq	ARG5, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
//	movslq	%r12d, %r12
	movq	ARG7, %r13  // x_t
	movq	ARG10, %r14  // z_n

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_NT_4_LIB8
#else
	CALL(inner_kernel_gemv_add_nt_4_lib8)
#endif


	// inner blend n scale ab

	movq	ARG3, %r10 // alpha_t
	movq	ARG8, %r11   // beta_t
	movq	ARG9, %r12   // y_t

#if MACRO_LEVEL>=1
	INNER_BLEND_T_SCALE_AB_4_LIB8
#else
	CALL(inner_blend_t_scale_ab_4_lib8)
#endif


	// store

	movq	ARG11, %r10 // z_t 

#if MACRO_LEVEL>=1
	INNER_STORE_4_LIB8
#else
	CALL(inner_store_4_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemv_nt_4_lib8)





//                                1      2                3                4          5        6            7            8               9            10           11           12
// void kernel_sgemv_nt_4_vs_lib8(int k, double *alpha_n, double *alpha_t, double *A, int sda, double *x_n, double *x_t, double *beta_t, double *y_t, double *z_n, double *z_t, int km);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemv_nt_4_vs_lib8)
	
	PROLOGUE

	// zero accumulation registers y_t

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3

	vmovaps	%ymm0, %ymm6
	vmovaps	%ymm0, %ymm7
	vmovaps	%ymm0, %ymm8
	vmovaps	%ymm0, %ymm9

	// initialize x_n
	movq	ARG2, %r10 // alpha_n
	vbroadcastss 0(%r10), %ymm15

	movq	ARG6, %r10 // x_n
	movq	ARG12, %r11 // km

	vbroadcastss 0(%r10), %ymm6
	vmulps		%ymm15, %ymm6, %ymm6
	cmpl	$ 2, %r11d
	jl		0f
	vbroadcastss 4(%r10), %ymm7
	vmulps		%ymm15, %ymm7, %ymm7
	cmpl	$ 3, %r11d
	jl		0f
	vbroadcastss 8(%r10), %ymm8
	vmulps		%ymm15, %ymm8, %ymm8
	je		0f
	vbroadcastss 12(%r10), %ymm9
	vmulps		%ymm15, %ymm9, %ymm9
0:

	// inner kernel dgemv nt

	movq	ARG1, %r10 // k
	movq	ARG4, %r11  // A
	movq	ARG5, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
//	movslq	%r12d, %r12
	movq	ARG7, %r13  // x_t
	movq	ARG10, %r14  // z_n

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_NT_4_LIB8
#else
	CALL(inner_kernel_gemv_add_nt_4_lib8)
#endif


	// inner blend n scale ab

	movq	ARG3, %r10 // alpha_t
	movq	ARG8, %r11   // beta_t
	movq	ARG9, %r12   // y_t

#if MACRO_LEVEL>=1
	INNER_BLEND_T_SCALE_AB_4_LIB8
#else
	CALL(inner_blend_t_scale_ab_4_lib8)
#endif


	// store

	movq	ARG11, %r10 // z_t 
	movq	ARG12, %r11 // km 

#if MACRO_LEVEL>=1
	INNER_STORE_4_VS_LIB8
#else
	CALL(inner_store_4_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemv_nt_4_vs_lib8)





//                             1      2              3          4        5          6
// void kernel_dsymv_l_4l_lib8(int k, double *alpha, double *A, int sda, double *x, double *z);

	.p2align 4,,15
	GLOB_FUN_START(kernel_ssymv_l_4l_lib8)
	
	PROLOGUE

	// zero accumulation registers y_t

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3

	// initialize x_n
	movq	ARG2, %r10 // alpha
	vbroadcastss 0(%r10), %ymm15

	movq	ARG5, %r10 // x_n

	vbroadcastss 0(%r10), %ymm6
	vmulps		%ymm15, %ymm6, %ymm6
	vbroadcastss 4(%r10), %ymm7
	vmulps		%ymm15, %ymm7, %ymm7
	vbroadcastss 8(%r10), %ymm8
	vmulps		%ymm15, %ymm8, %ymm8
	vbroadcastss 12(%r10), %ymm9
	vmulps		%ymm15, %ymm9, %ymm9


	// inner edge dsyrk & kernel dgemv nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG5, %r13  // x_t
	movq	ARG6, %r14  // z_n
	movq	$ 0, %r15 // offA

#if MACRO_LEVEL>=2
	INNER_EDGE_SYMV_ADD_NT_4L_LIB8
#else
	CALL(inner_edge_symv_add_nt_4l_lib8)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_NT_4_LIB8
#else
	CALL(inner_kernel_gemv_add_nt_4_lib8)
#endif


	// call inner blend n scale ab

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11   // z_t

#if MACRO_LEVEL>=1
	INNER_BLEND_T_SCALE_A1_4_LIB8
#else
	CALL(inner_blend_t_scale_a1_4_lib8)
#endif


	// store

	movq	ARG6, %r10 // z_t 

#if MACRO_LEVEL>=1
	INNER_STORE_4_LIB8
#else
	CALL(inner_store_4_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_ssymv_l_4l_lib8)





//                             1      2              3          4        5          6
// void kernel_dsymv_l_4r_lib8(int k, double *alpha, double *A, int sda, double *x, double *z);

	.p2align 4,,15
	GLOB_FUN_START(kernel_ssymv_l_4r_lib8)
	
	PROLOGUE

	// zero accumulation registers y_t

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3

	// initialize x_n
	movq	ARG2, %r10 // alpha
	vbroadcastss 0(%r10), %ymm15

	movq	ARG5, %r10 // x_n

	vbroadcastss 0(%r10), %ymm6
	vmulps		%ymm15, %ymm6, %ymm6
	vbroadcastss 4(%r10), %ymm7
	vmulps		%ymm15, %ymm7, %ymm7
	vbroadcastss 8(%r10), %ymm8
	vmulps		%ymm15, %ymm8, %ymm8
	vbroadcastss 12(%r10), %ymm9
	vmulps		%ymm15, %ymm9, %ymm9


	// inner edge dsyrk & kernel dgemv nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG5, %r13  // x_t
	movq	ARG6, %r14  // z_n
	movq	$ 0, %r15 // offA

#if MACRO_LEVEL>=2
	INNER_EDGE_SYMV_ADD_NT_4R_LIB8
#else
	CALL(inner_edge_symv_add_nt_4r_lib8)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_NT_4_LIB8
#else
	CALL(inner_kernel_gemv_add_nt_4_lib8)
#endif


	// call inner blend n scale ab

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11   // z_t

#if MACRO_LEVEL>=1
	INNER_BLEND_T_SCALE_A1_4_LIB8
#else
	CALL(inner_blend_t_scale_a1_4_lib8)
#endif


	// store

	movq	ARG6, %r10 // z_t 

#if MACRO_LEVEL>=1
	INNER_STORE_4_LIB8
#else
	CALL(inner_store_4_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_ssymv_l_4r_lib8)





//                                1      2              3          4          5        6          7          8
// void kernel_dsymv_l_4l_gen_lib8(int k, double *alpha, int offA, double *A, int sda, double *x, double *z, int km);

	.p2align 4,,15
	GLOB_FUN_START(kernel_ssymv_l_4l_gen_lib8)
	
	PROLOGUE

	// zero accumulation registers y_t

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3

	vmovaps	%ymm0, %ymm6
	vmovaps	%ymm0, %ymm7
	vmovaps	%ymm0, %ymm8
	vmovaps	%ymm0, %ymm9

	// initialize x_n
	movq	ARG2, %r10 // alpha
	vbroadcastss 0(%r10), %ymm15

	movq	ARG6, %r10 // x_n
	movq	ARG8, %r11 // km

	vbroadcastss 0(%r10), %ymm6
	vmulps		%ymm15, %ymm6, %ymm6
	cmpl	$ 2, %r11d
	jl		0f
	vbroadcastss 4(%r10), %ymm7
	vmulps		%ymm15, %ymm7, %ymm7
	cmpl	$ 3, %r11d
	jl		0f
	vbroadcastss 8(%r10), %ymm8
	vmulps		%ymm15, %ymm8, %ymm8
	je		0f
	vbroadcastss 12(%r10), %ymm9
	vmulps		%ymm15, %ymm9, %ymm9
0:

	// inner edge dsyrk & kernel dgemv nt

	movq	ARG1, %r10 // k
	movq	ARG4, %r11  // A
	movq	ARG5, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG6, %r13  // x_t
	movq	ARG7, %r14  // z_n
	movq	ARG3, %r15 // offA

#if MACRO_LEVEL>=2
	INNER_EDGE_SYMV_ADD_NT_4L_LIB8
#else
	CALL(inner_edge_symv_add_nt_4l_lib8)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_NT_4_LIB8
#else
	CALL(inner_kernel_gemv_add_nt_4_lib8)
#endif


	// call inner blend n scale ab

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11   // z_t

#if MACRO_LEVEL>=1
	INNER_BLEND_T_SCALE_A1_4_LIB8
#else
	CALL(inner_blend_t_scale_a1_4_lib8)
#endif


	// store

	movq	ARG7, %r10 // z_t 
	movq	ARG8, %r11 // km

#if MACRO_LEVEL>=1
	INNER_STORE_4_VS_LIB8
#else
	CALL(inner_store_4_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_ssymv_l_4l_gen_lib8)





//                                1      2              3          4          5        6          7          8
// void kernel_dsymv_l_4r_gen_lib8(int k, double *alpha, int offA, double *A, int sda, double *x, double *z, int km);

	.p2align 4,,15
	GLOB_FUN_START(kernel_ssymv_l_4r_gen_lib8)
	
	PROLOGUE

	// zero accumulation registers y_t

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3

	vmovaps	%ymm0, %ymm6
	vmovaps	%ymm0, %ymm7
	vmovaps	%ymm0, %ymm8
	vmovaps	%ymm0, %ymm9

	// initialize x_n
	movq	ARG2, %r10 // alpha
	vbroadcastss 0(%r10), %ymm15

	movq	ARG6, %r10 // x_n
	movq	ARG8, %r11 // km

	vbroadcastss 0(%r10), %ymm6
	vmulps		%ymm15, %ymm6, %ymm6
	cmpl	$ 2, %r11d
	jl		0f
	vbroadcastss 4(%r10), %ymm7
	vmulps		%ymm15, %ymm7, %ymm7
	cmpl	$ 3, %r11d
	jl		0f
	vbroadcastss 8(%r10), %ymm8
	vmulps		%ymm15, %ymm8, %ymm8
	je		0f
	vbroadcastss 12(%r10), %ymm9
	vmulps		%ymm15, %ymm9, %ymm9
0:

	// inner edge dsyrk & kernel dgemv nt

	movq	ARG1, %r10 // k
	movq	ARG4, %r11  // A
	movq	ARG5, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG6, %r13  // x_t
	movq	ARG7, %r14  // z_n
	movq	ARG3, %r15 // offA

#if MACRO_LEVEL>=2
	INNER_EDGE_SYMV_ADD_NT_4R_LIB8
#else
	CALL(inner_edge_symv_add_nt_4r_lib8)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_NT_4_LIB8
#else
	CALL(inner_kernel_gemv_add_nt_4_lib8)
#endif


	// call inner blend n scale ab

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11   // z_t

#if MACRO_LEVEL>=1
	INNER_BLEND_T_SCALE_A1_4_LIB8
#else
	CALL(inner_blend_t_scale_a1_4_lib8)
#endif


	// store

	movq	ARG7, %r10 // z_t 
	movq	ARG8, %r11 // km

#if MACRO_LEVEL>=1
	INNER_STORE_4_VS_LIB8
#else
	CALL(inner_store_4_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_ssymv_l_4r_gen_lib8)





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


