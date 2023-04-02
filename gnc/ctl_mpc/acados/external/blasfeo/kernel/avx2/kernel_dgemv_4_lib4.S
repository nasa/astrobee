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
	.macro INNER_KERNEL_GEMV_ADD_N_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemv_add_n_4_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	cmpl	$ 4, %r10d
	jl		0f // clean-up loop

	// main loop
	.p2align 3
1: // main loop
	
	vmovapd	0(%r11), %ymm8
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	
	subl	$ 4, %r10d

	vmovapd	32(%r11), %ymm8
	vbroadcastsd	8(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1
	
	vmovapd	64(%r11), %ymm8
	vbroadcastsd	16(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm2

	vmovapd	96(%r11), %ymm8
	vbroadcastsd	24(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm3
	
	addq	$ 128, %r11
	addq	$ 32, %r12
	
	cmpl	$ 3, %r10d

	jg		1b // main loop 


	// consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

0: // clean-up
	
	vmovapd	0(%r11), %ymm8
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	
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

	FUN_END(inner_kernel_gemv_add_n_4_lib4)
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
	.macro INNER_KERNEL_GEMV_ADD_T_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemv_add_t_4_lib4)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	cmpl	$ 4, %r10d
	jl		0f // clean-up loop

	// main loop
	.p2align 3
1: // main loop
	
	vmovupd	0(%r13), %ymm12

	vmovapd	0(%r11), %ymm8
	vfmadd231pd	%ymm8, %ymm12, %ymm0
	
	subl	$ 4, %r10d

	vmovapd	32(%r11), %ymm8
	vfmadd231pd	%ymm8, %ymm12, %ymm1
	
	vmovapd	64(%r11), %ymm8
	vfmadd231pd	%ymm8, %ymm12, %ymm2

	vmovapd	96(%r11), %ymm8
	vfmadd231pd	%ymm8, %ymm12, %ymm3
	
	addq	%r12, %r11
	addq	$ 32, %r13
	
	cmpl	$ 3, %r10d

	jg		1b // main loop 


	// consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

0: // clean-up
	
	vcvtsi2sd	%r10d, %xmm14, %xmm14
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC02(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC02(%rip), %ymm13
#endif
	vmovddup	%xmm14, %xmm14
	vinsertf128	$ 1, %xmm14, %ymm14, %ymm14
	vsubpd		%ymm14, %ymm13, %ymm14

	vmaskmovpd	0(%r13), %ymm14, %ymm12

	vmaskmovpd	0(%r11), %ymm14, %ymm8
	vfmadd231pd	%ymm8, %ymm12, %ymm0
	
	vmaskmovpd	32(%r11), %ymm14, %ymm8
	vfmadd231pd	%ymm8, %ymm12, %ymm1
	
	vmaskmovpd	64(%r11), %ymm14, %ymm8
	vfmadd231pd	%ymm8, %ymm12, %ymm2

	vmaskmovpd	96(%r11), %ymm14, %ymm8
	vfmadd231pd	%ymm8, %ymm12, %ymm3
		
	sall	$ 3, %r10d
//	movslq	%r10d, %r10
	addq	%r10, %r11
	addq	%r10, %r13
	xorl	%r10d, %r10d
	
	
2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemv_add_t_4_lib4)
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
	.macro INNER_KERNEL_GEMV_ADD_NT_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemv_add_nt_4_lib4)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	cmpl	$ 4, %r10d
	jl		0f // clean-up loop

	// main loop
	.p2align 3
1: // main loop
	
	vmovupd	0(%r13), %ymm12
	vmovupd	0(%r14), %ymm13

	vmovapd	0(%r11), %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm0
	vfmadd231pd	%ymm14, %ymm6, %ymm13
	
	subl	$ 4, %r10d

	vmovapd	32(%r11), %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm1
	vfmadd231pd	%ymm14, %ymm7, %ymm13
	
	vmovapd	64(%r11), %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm2
	vfmadd231pd	%ymm14, %ymm8, %ymm13

	vmovapd	96(%r11), %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm3
	vfmadd231pd	%ymm14, %ymm9, %ymm13
	
	vmovupd	%ymm13, 0(%r14) 

	addq	%r12, %r11
	addq	$ 32, %r13
	addq	$ 32, %r14
	
	cmpl	$ 3, %r10d

	jg		1b // main loop 


	// consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

0: // clean-up
	
	vcvtsi2sd	%r10d, %xmm14, %xmm14
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC02(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC02(%rip), %ymm13
#endif
	vmovddup	%xmm14, %xmm14
	vinsertf128	$ 1, %xmm14, %ymm14, %ymm14
	vsubpd		%ymm14, %ymm13, %ymm11

	vmaskmovpd	0(%r13), %ymm11, %ymm12
	vmaskmovpd	0(%r14), %ymm11, %ymm13

//	vmovupd	%ymm14, -32(%rsp) // spill mask to stack

//	vmovupd	-32(%rsp), %ymm14 // load mask form stack
	vmaskmovpd	0(%r11), %ymm11, %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm0
	vfmadd231pd	%ymm14, %ymm6, %ymm13
	
//	vmovupd	-32(%rsp), %ymm14 // load mask form stack
	vmaskmovpd	32(%r11), %ymm11, %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm1
	vfmadd231pd	%ymm14, %ymm7, %ymm13
	
//	vmovupd	-32(%rsp), %ymm14 // load mask form stack
	vmaskmovpd	64(%r11), %ymm11, %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm2
	vfmadd231pd	%ymm14, %ymm8, %ymm13

//	vmovupd	-32(%rsp), %ymm14 // load mask form stack
	vmaskmovpd	96(%r11), %ymm11, %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm3
	vfmadd231pd	%ymm14, %ymm9, %ymm13
		
//	vmovupd	-32(%rsp), %ymm14 // load mask form stack
	vmaskmovpd	%ymm13, %ymm11, 0(%r14)

	sall	$ 3, %r10d // *sizeof(double)
	addq	%r10, %r11
	addq	%r10, %r13
	addq	%r10, %r14
	xorl	%r10d, %r10d
	
	
2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemv_add_nt_4_lib4)
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

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_GEMV_ADD_T_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_gemv_add_t_4_lib4)
#endif

	cmpl	$ 0, %r14d
	jle		0f // return

	movl	%r14d, %r15d
	sall	$ 3, %r15d // offA*sizeof(double)

	subq	%r15, %r11 // A - offA
	subq	%r15, %r13 // x - offA

	movl	%r10d, %r15d // kmax
	addl	%r14d, %r15d // kmax + offA

	vcvtsi2sd	%r14d, %xmm14, %xmm14 // offA
	vcvtsi2sd	%r15d, %xmm15, %xmm15 // offA + kmax
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC02(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC02(%rip), %ymm13
#endif
	vmovddup	%xmm14, %xmm14
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm14, %ymm14, %ymm14
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm13, %ymm14, %ymm14
	vsubpd		%ymm15, %ymm13, %ymm15
	vandpd		%ymm15, %ymm14, %ymm14

	vmaskmovpd	0(%r13), %ymm14, %ymm12

	vmovapd	0(%r11), %ymm8
	vfmadd231pd	%ymm8, %ymm12, %ymm0
	
	vmovapd	32(%r11), %ymm8
	vfmadd231pd	%ymm8, %ymm12, %ymm1
	
	vmovapd	64(%r11), %ymm8
	vfmadd231pd	%ymm8, %ymm12, %ymm2

	vmovapd	96(%r11), %ymm8
	vfmadd231pd	%ymm8, %ymm12, %ymm3

	addq	$ 32, %r13 // x + 4
	addq	%r12, %r11 // A + bs*sda
		
	addl	%r14d, %r10d
	subl	$ 4, %r10d // kmax - (4-offA)
	
0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_gemv_add_t_4_lib4)
#endif





// common inner routine with file scope
//
// input arguments:
// r10   <- kmax
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
// r10   <- kmax-4
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
	.macro INNER_EDGE_DSYMV_ADD_NT_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dsymv_add_nt_4_lib4)
#endif

	vmovupd		0(%r13), %ymm12
	vmovupd		0(%r14), %ymm13

	vmovupd		0(%r11), %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm0
	vxorpd		%ymm15, %ymm15, %ymm15
	vblendpd	$ 0x1, %ymm15, %ymm14, %ymm14
	vfmadd231pd	%ymm14, %ymm6, %ymm13
	
	vmovupd		32(%r11), %ymm14
	vxorpd		%ymm15, %ymm15, %ymm15
	vblendpd	$ 0x1, %ymm15, %ymm14, %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm1
	vxorpd		%ymm15, %ymm15, %ymm15
	vblendpd	$ 0x3, %ymm15, %ymm14, %ymm14
	vfmadd231pd	%ymm14, %ymm7, %ymm13
	
	vmovupd		64(%r11), %ymm14
	vxorpd		%ymm15, %ymm15, %ymm15
	vblendpd	$ 0x3, %ymm15, %ymm14, %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm2
	vxorpd		%ymm15, %ymm15, %ymm15
	vblendpd	$ 0x7, %ymm15, %ymm14, %ymm14
	vfmadd231pd	%ymm14, %ymm8, %ymm13

	vmovupd		96(%r11), %ymm14
	vxorpd		%ymm15, %ymm15, %ymm15
	vblendpd	$ 0x7, %ymm15, %ymm14, %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm3
//	vxorpd		%ymm15, %ymm15, %ymm15
//	vblendpd	$ 0x0, %ymm14, %ymm15, %ymm14
//	vfmadd231pd	%ymm14, %ymm9, %ymm13
	
	vmovupd		%ymm13, 0(%r14) 

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
	.macro INNER_EDGE_DSYMV_ADD_NT_4_GEN_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dsymv_add_nt_4_gen_lib4)
#endif

	movl	$ 4, %eax
	cmpl	%eax, %r10d
	jge		0f
	movl	%r10d, %eax
0:
	subl	%r15d, %eax

	vcvtsi2sd	%eax, %xmm14, %xmm14
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC02(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC02(%rip), %ymm13
#endif
	vmovddup	%xmm14, %xmm14
	vinsertf128	$ 0x1, %xmm14, %ymm14, %ymm14
	vsubpd		%ymm14, %ymm13, %ymm11

	cmp		$ 3, %eax
	jg		0f
	vxorpd		%ymm9, %ymm9, %ymm9
	cmp		$ 2, %eax
	jg		0f
	vxorpd		%ymm8, %ymm8, %ymm8
	cmp		$ 1, %eax
	jg		0f
	vxorpd		%ymm7, %ymm7, %ymm7
0:

	vmaskmovpd	0(%r13), %ymm11, %ymm12
	vmaskmovpd	0(%r14), %ymm11, %ymm13

	vmaskmovpd	0(%r11), %ymm11, %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm0
	vxorpd		%ymm15, %ymm15, %ymm15
	vblendpd	$ 0x1, %ymm15, %ymm14, %ymm14
	vfmadd231pd	%ymm14, %ymm6, %ymm13
	
	vmaskmovpd	32(%r11), %ymm11, %ymm14
	vxorpd		%ymm15, %ymm15, %ymm15
	vblendpd	$ 0x1, %ymm15, %ymm14, %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm1
	vxorpd		%ymm15, %ymm15, %ymm15
	vblendpd	$ 0x3, %ymm15, %ymm14, %ymm14
	vfmadd231pd	%ymm14, %ymm7, %ymm13
	
	vmaskmovpd	64(%r11), %ymm11, %ymm14
	vxorpd		%ymm15, %ymm15, %ymm15
	vblendpd	$ 0x3, %ymm15, %ymm14, %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm2
	vxorpd		%ymm15, %ymm15, %ymm15
	vblendpd	$ 0x7, %ymm15, %ymm14, %ymm14
	vfmadd231pd	%ymm14, %ymm8, %ymm13

	vmaskmovpd	96(%r11), %ymm11, %ymm14
	vxorpd		%ymm15, %ymm15, %ymm15
	vblendpd	$ 0x7, %ymm15, %ymm14, %ymm14
	vfmadd231pd	%ymm14, %ymm12, %ymm3
//	vxorpd		%ymm15, %ymm15, %ymm15
//	vblendpd	$ 0x0, %ymm14, %ymm15, %ymm14
//	vfmadd231pd	%ymm14, %ymm9, %ymm13
	
	vmaskmovpd	%ymm13, %ymm11, 0(%r14)

	subl	%eax, %r10d

	sall	$ 3, %eax // *sizeof(double)
	addq	%rax, %r11
	subq	$ 32, %r11
	addq	%r12, %r11
	addq	%rax, %r13
	addq	%rax, %r14
	
#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_edge_dsymv_add_nt_4_gen_lib4)
#endif






// common inner routine with file scope
//
// blend for ta==n
//
// input arguments:
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
	.macro INNER_BLEND_N_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_blend_n_lib4)
#endif

	// reduction
	vaddpd	%ymm0, %ymm1, %ymm0
	vaddpd	%ymm2, %ymm3, %ymm2
	vaddpd	%ymm0, %ymm2, %ymm0

#if MACRO_LEVEL>=1
	.endm
#else
	ret
	
	FUN_END(inner_blend_n_lib4)
#endif





// common inner routine with file scope
//
// blend for ta==t
//
// input arguments:
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
	.macro INNER_BLEND_T_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_blend_t_lib4)
#endif

	// reduction
	vhaddpd	%ymm1, %ymm0, %ymm0
	vhaddpd	%ymm3, %ymm2, %ymm2
	vperm2f128	$ 0x2, %ymm0, %ymm2, %ymm1
	vperm2f128	$ 0x13, %ymm0, %ymm2, %ymm0
	vaddpd	%ymm0, %ymm1, %ymm0

#if MACRO_LEVEL>=1
	.endm
#else
	ret
	
	FUN_END(inner_blend_t_lib4)
#endif





// common inner routine with file scope
//
// blend for ta==t, scale for generic alpha and beta
//
// input arguments:
// r10  <- alpha
// r11  <- beta
// r12  <- y
// ymm0 <- [z0 z1 z2 z3]
//
// output arguments:
// r10  <- alpha
// r11  <- beta
// r12  <- y
// ymm0 <- [z0 z1 z2 z3]

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_4_lib4)
#endif

	// alpha
	vbroadcastsd	0(%r10), %ymm15
	vmulpd	%ymm0, %ymm15, %ymm0

	// beta
	vbroadcastsd	0(%r11), %ymm15

	vxorpd		%ymm14, %ymm14, %ymm14 // 0.0
	vucomisd	%xmm14, %xmm15 // beta==0.0 ?
	je			0f // end

	vmovupd		0(%r12), %ymm14
	vmulpd		%ymm15, %ymm14, %ymm14
	vaddpd		%ymm0, %ymm14, %ymm0

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret
	
	FUN_END(inner_scale_ab_4_lib4)
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
	.macro INNER_BLEND_N_SCALE_AB_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_blend_n_scale_ab_4_lib4)
#endif

	// reduction
	vaddpd	%ymm0, %ymm1, %ymm0
	vaddpd	%ymm2, %ymm3, %ymm2
	vaddpd	%ymm0, %ymm2, %ymm0

	// alpha
	vbroadcastsd	0(%r10), %ymm15
	vmulpd	%ymm0, %ymm15, %ymm0

	// beta
	vbroadcastsd	0(%r11), %ymm15

	vxorpd		%ymm14, %ymm14, %ymm14 // 0.0
	vucomisd	%xmm14, %xmm15 // beta==0.0 ?
	je			0f // end

	vmovupd		0(%r12), %ymm14
	vmulpd		%ymm15, %ymm14, %ymm14
	vaddpd		%ymm0, %ymm14, %ymm0

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret
	
	FUN_END(inner_blend_n_scale_ab_4_lib4)
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
	.macro INNER_BLEND_N_SCALE_M11_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_blend_n_scale_m11_4_lib4)
#endif

	// reduction
	vaddpd	%ymm0, %ymm1, %ymm0
	vaddpd	%ymm2, %ymm3, %ymm2
	vaddpd	%ymm0, %ymm2, %ymm0

	// beta
	vmovupd		0(%r10), %ymm14
	vsubpd		%ymm0, %ymm14, %ymm0

#if MACRO_LEVEL>=1
	.endm
#else
	ret
	
	FUN_END(inner_blend_n_scale_m11_4_lib4)
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
	.macro INNER_BLEND_T_SCALE_AB_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_blend_t_scale_ab_4_lib4)
#endif

	// reduction
	vhaddpd	%ymm1, %ymm0, %ymm0
	vhaddpd	%ymm3, %ymm2, %ymm2
	vperm2f128	$ 0x2, %ymm0, %ymm2, %ymm1
	vperm2f128	$ 0x13, %ymm0, %ymm2, %ymm0
	vaddpd	%ymm0, %ymm1, %ymm0

	// alpha
	vbroadcastsd	0(%r10), %ymm15
	vmulpd	%ymm0, %ymm15, %ymm0

	// beta
	vbroadcastsd	0(%r11), %ymm15

	vxorpd		%ymm14, %ymm14, %ymm14 // 0.0
	vucomisd	%xmm14, %xmm15 // beta==0.0 ?
	je			0f // end

	vmovupd		0(%r12), %ymm14
	vmulpd		%ymm15, %ymm14, %ymm14
	vaddpd		%ymm0, %ymm14, %ymm0

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
	.macro INNER_BLEND_T_SCALE_A1_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_blend_t_scale_a1_4_lib4)
#endif

	// reduction
	vhaddpd	%ymm1, %ymm0, %ymm0
	vhaddpd	%ymm3, %ymm2, %ymm2
	vperm2f128	$ 0x2, %ymm0, %ymm2, %ymm1
	vperm2f128	$ 0x13, %ymm0, %ymm2, %ymm0
	vaddpd	%ymm0, %ymm1, %ymm0

	// alpha
	vbroadcastsd	0(%r10), %ymm15
	vmulpd	%ymm0, %ymm15, %ymm0

	// beta
	vmovupd		0(%r11), %ymm14
	vaddpd		%ymm0, %ymm14, %ymm0
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret
	
	FUN_END(inner_blend_t_scale_a1_4_lib4)
#endif





// common inner routine with file scope
//
// blend for ta==t, scale for alpha=-1.0 and beta=1.0
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
	.macro INNER_BLEND_T_SCALE_M11_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_blend_t_scale_m11_4_lib4)
#endif

	// reduction
	vhaddpd	%ymm1, %ymm0, %ymm0
	vhaddpd	%ymm3, %ymm2, %ymm2
	vperm2f128	$ 0x2, %ymm0, %ymm2, %ymm1
	vperm2f128	$ 0x13, %ymm0, %ymm2, %ymm0
	vaddpd	%ymm0, %ymm1, %ymm0

	vmovupd		0(%r10), %ymm14
	vsubpd		%ymm0, %ymm14, %ymm0
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret
	
	FUN_END(inner_blend_t_scale_m11_4_lib4)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta=1.0
//
// input arguments:
// r10  <- alpha
// r11  <- y
// ymm0 <- [z0 z1 z2 z3]
//
// output arguments:
// r10  <- alpha
// r11  <- y
// ymm0 <- [z0 z1 z2 z3]

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_A1_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_scale_a1_4_lib4)
#endif

	// alpha
	vbroadcastsd	0(%r10), %ymm15
	vmulpd			%ymm0, %ymm15, %ymm0

	// beta
	vmovupd			0(%r11), %ymm14
	vaddpd			%ymm0, %ymm14, %ymm0
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret
	
	FUN_END(inner_scale_a1_4_lib4)
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
	.macro INNER_EDGE_DTRSV_LN_INV_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsv_ln_inv_4_lib4)
#endif
	
	vxorpd			%ymm14, %ymm14, %ymm14

	vbroadcastsd	0(%r11), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm1
	vblendpd		$ 0x1, %ymm1, %ymm0, %ymm0

	vmovapd			0(%r10), %ymm13
	vblendpd		$ 0x1, %ymm14, %ymm13, %ymm13
	vpermilpd		$ 0x0, %ymm0, %ymm12
	vperm2f128		$ 0x00, %ymm12, %ymm12, %ymm12
	vmulpd			%ymm13, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0
	vbroadcastsd	8(%r11), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm1
	vblendpd		$ 0x2, %ymm1, %ymm0, %ymm0

	vmovapd			32(%r10), %ymm13
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





// common inner routine with file scope
//
// triangular substitution with vector RHS, variable size version
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
	.macro INNER_EDGE_DTRSV_LN_INV_4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsv_ln_inv_4_vs_lib4)
#endif
	
	vxorpd			%ymm14, %ymm14, %ymm14

	vbroadcastsd	0(%r11), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm1
	vblendpd		$ 0x1, %ymm1, %ymm0, %ymm0
	vmovapd			0(%r10), %ymm13
	vblendpd		$ 0x1, %ymm14, %ymm13, %ymm13
	vpermilpd		$ 0x0, %ymm0, %ymm12
	vperm2f128		$ 0x00, %ymm12, %ymm12, %ymm12
	vmulpd			%ymm13, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0

	cmpl			$ 2, %r12d
	jl				0f // ret

	vbroadcastsd	8(%r11), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm1
	vblendpd		$ 0x2, %ymm1, %ymm0, %ymm0
	vmovapd			32(%r10), %ymm13
	vblendpd		$ 0x3, %ymm14, %ymm13, %ymm13
	vpermilpd		$ 0x3, %ymm0, %ymm12
	vperm2f128		$ 0x00, %ymm12, %ymm12, %ymm12
	vmulpd			%ymm13, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0

	cmpl			$ 3, %r12d
	jl				0f // ret

	vbroadcastsd	16(%r11), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm1
	vblendpd		$ 0x4, %ymm1, %ymm0, %ymm0
	vmovapd			64(%r10), %ymm13
	vblendpd		$ 0x7, %ymm14, %ymm13, %ymm13
	vpermilpd		$ 0x0, %ymm0, %ymm12
	vperm2f128		$ 0x11, %ymm12, %ymm12, %ymm12
	vmulpd			%ymm13, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0

	cmpl			$ 4, %r12d
	jl				0f // ret

	vbroadcastsd	24(%r11), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm1
	vblendpd		$ 0x8, %ymm1, %ymm0, %ymm0

	// return
0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsv_ln_inv_4_vs_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution with vector RHS
//
// input arguments:
// r10  <- E
// ymm0 <- [z0 z1 z2 z3]
//
// output arguments:
// r10  <- E
// ymm0 <- [z0 z1 z2 z3]

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSV_LN_ONE_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsv_ln_one_4_lib4)
#endif
	
	vxorpd			%ymm14, %ymm14, %ymm14

	vmovapd			0(%r10), %ymm13
	vblendpd		$ 0x1, %ymm14, %ymm13, %ymm13
	vpermilpd		$ 0x0, %ymm0, %ymm12
	vperm2f128		$ 0x00, %ymm12, %ymm12, %ymm12
	vmulpd			%ymm13, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0

	vmovapd			32(%r10), %ymm13
	vblendpd		$ 0x3, %ymm14, %ymm13, %ymm13
	vpermilpd		$ 0x3, %ymm0, %ymm12
	vperm2f128		$ 0x00, %ymm12, %ymm12, %ymm12
	vmulpd			%ymm13, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0

	vmovapd			64(%r10), %ymm13
	vblendpd		$ 0x7, %ymm14, %ymm13, %ymm13
	vpermilpd		$ 0x0, %ymm0, %ymm12
	vperm2f128		$ 0x11, %ymm12, %ymm12, %ymm12
	vmulpd			%ymm13, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsv_ln_one_4_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution with vector RHS, variable size version
//
// input arguments:
// r10  <- E
// r11d <- kn
// ymm0 <- [z0 z1 z2 z3]
//
// output arguments:
// r10  <- E
// r11d <- kn
// ymm0 <- [z0 z1 z2 z3]

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSV_LN_ONE_4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsv_ln_one_4_vs_lib4)
#endif
	
	vxorpd			%ymm14, %ymm14, %ymm14

	vmovapd			0(%r10), %ymm13
	vblendpd		$ 0x1, %ymm14, %ymm13, %ymm13
	vpermilpd		$ 0x0, %ymm0, %ymm12
	vperm2f128		$ 0x00, %ymm12, %ymm12, %ymm12
	vmulpd			%ymm13, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0

	cmpl			$ 2, %r11d
	jl				0f // ret

	vmovapd			32(%r10), %ymm13
	vblendpd		$ 0x3, %ymm14, %ymm13, %ymm13
	vpermilpd		$ 0x3, %ymm0, %ymm12
	vperm2f128		$ 0x00, %ymm12, %ymm12, %ymm12
	vmulpd			%ymm13, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0

	cmpl			$ 3, %r11d
	jl				0f // ret

	vmovapd			64(%r10), %ymm13
	vblendpd		$ 0x7, %ymm14, %ymm13, %ymm13
	vpermilpd		$ 0x0, %ymm0, %ymm12
	vperm2f128		$ 0x11, %ymm12, %ymm12, %ymm12
	vmulpd			%ymm13, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0

	// return
0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsv_ln_one_4_vs_lib4)
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
	.macro INNER_EDGE_DTRSV_LT_INV_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsv_lt_inv_4_lib4)
#endif
	
	vxorpd			%ymm14, %ymm14, %ymm14

	vmovapd			16(%r10), %xmm12
	vmovapd			48(%r10), %xmm13
	vunpcklpd		%xmm13, %xmm12, %xmm9
	vblendpd		$ 0xc, %ymm14, %ymm9, %ymm9
	vunpckhpd		%xmm13, %xmm12, %xmm10
	vmovsd			8(%r10), %xmm8
	vblendpd		$ 0xe, %ymm14, %ymm8, %ymm8
	vmovsd			88(%r10), %xmm11
	vinsertf128		$ 0x1, %xmm11, %ymm10, %ymm10
	vblendpd		$ 0x8, %ymm14, %ymm10, %ymm10

	vbroadcastsd	24(%r11), %ymm12
	vmulpd			%ymm12, %ymm0, %ymm1
	vblendpd		$ 0x8, %ymm1, %ymm0, %ymm0

	vpermilpd		$ 0xf, %ymm0, %ymm12
	vperm2f128		$ 0x11, %ymm12, %ymm12, %ymm12
	vmulpd			%ymm10, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0
	vbroadcastsd	16(%r11), %ymm12
	vmulpd			%ymm12, %ymm0, %ymm1
	vblendpd		$ 0x4, %ymm1, %ymm0, %ymm0

	vpermilpd		$ 0x0, %ymm0, %ymm12
	vperm2f128		$ 0x11, %ymm12, %ymm12, %ymm12
	vmulpd			%ymm9, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0
	vbroadcastsd	8(%r11), %ymm12
	vmulpd			%ymm12, %ymm0, %ymm1
	vblendpd		$ 0x2, %ymm1, %ymm0, %ymm0

	vpermilpd		$ 0x3, %ymm0, %ymm12
//	vperm2f128		$ 0x00, %ymm12, %ymm12, %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0
//	vbroadcastsd	8(%r11), %ymm12
	vmovsd			0(%r11), %xmm12
	vmulpd			%ymm12, %ymm0, %ymm1
	vblendpd		$ 0x1, %ymm1, %ymm0, %ymm0

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsv_lt_inv_4_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution with vector RHS
//
// input arguments:
// r10  <- E
// r11  <- inv_diag_E
// r12  <- k
// r13  <- x
// ymm0 <- [z0 z1 z2 z3]
// ymm12 <- dirty
// ymm13 <- dirty
//
// output arguments:
// r10  <- E
// r11  <- inv_diag_E
// r12  <- k
// r13  <- x
// ymm0 <- [z0 z1 z2 z3]
// ymm12 <- dirty
// ymm13 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSV_LT_INV_3_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsv_lt_inv_3_lib4)
#endif
	
	vxorpd			%ymm14, %ymm14, %ymm14

	vmovapd			16(%r10), %xmm12
	vmovapd			48(%r10), %xmm13
	vunpcklpd		%xmm13, %xmm12, %xmm9
	vblendpd		$ 0xc, %ymm14, %ymm9, %ymm9
	vunpckhpd		%xmm13, %xmm12, %xmm10
	vmovsd			8(%r10), %xmm8
	vblendpd		$ 0xe, %ymm14, %ymm8, %ymm8
	vmovsd			88(%r10), %xmm11
	vinsertf128		$ 0x1, %xmm11, %ymm10, %ymm10
	vblendpd		$ 0x8, %ymm14, %ymm10, %ymm10

//	vbroadcastsd	24(%r11), %ymm12
//	vmulpd			%ymm12, %ymm0, %ymm1
//	vblendpd		$ 0x8, %ymm1, %ymm0, %ymm0

	vmovupd			0(%r13), %ymm12
	vblendpd		$ 0x8, %ymm12, %ymm0, %ymm0
	
	cmpl			$ 4, %r12d
	jl				0f

	vpermilpd		$ 0xf, %ymm0, %ymm12
	vperm2f128		$ 0x11, %ymm12, %ymm12, %ymm12
	vmulpd			%ymm10, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0
0:
	vbroadcastsd	16(%r11), %ymm12
	vmulpd			%ymm12, %ymm0, %ymm1
	vblendpd		$ 0x4, %ymm1, %ymm0, %ymm0

	vpermilpd		$ 0x0, %ymm0, %ymm12
	vperm2f128		$ 0x11, %ymm12, %ymm12, %ymm12
	vmulpd			%ymm9, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0
	vbroadcastsd	8(%r11), %ymm12
	vmulpd			%ymm12, %ymm0, %ymm1
	vblendpd		$ 0x2, %ymm1, %ymm0, %ymm0

	vpermilpd		$ 0x3, %ymm0, %ymm12
//	vperm2f128		$ 0x00, %ymm12, %ymm12, %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0
//	vbroadcastsd	8(%r11), %ymm12
	vmovsd			0(%r11), %xmm12
	vmulpd			%ymm12, %ymm0, %ymm1
	vblendpd		$ 0x1, %ymm1, %ymm0, %ymm0

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsv_lt_inv_3_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution with vector RHS
//
// input arguments:
// r10  <- E
// r11  <- inv_diag_E
// r12  <- k
// r13  <- x
// ymm0 <- [z0 z1 z2 z3]
// ymm12 <- dirty
// ymm13 <- dirty
//
// output arguments:
// r10  <- E
// r11  <- inv_diag_E
// r12  <- k
// r13  <- x
// ymm0 <- [z0 z1 z2 z3]
// ymm12 <- dirty
// ymm13 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSV_LT_INV_2_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsv_lt_inv_2_lib4)
#endif
	
	vxorpd			%ymm14, %ymm14, %ymm14

	cmpl			$ 3, %r12d

	vmovapd			16(%r10), %xmm12
	vmovapd			48(%r10), %xmm13
	vunpcklpd		%xmm13, %xmm12, %xmm9
	vblendpd		$ 0xc, %ymm14, %ymm9, %ymm9
	vunpckhpd		%xmm13, %xmm12, %xmm10
	vmovsd			8(%r10), %xmm8
	vblendpd		$ 0xe, %ymm14, %ymm8, %ymm8
//	vmovsd			88(%r10), %xmm11
//	vinsertf128		$ 0x1, %xmm11, %ymm10, %ymm10
//	vblendpd		$ 0x8, %ymm14, %ymm10, %ymm10
	vblendpd		$ 0xc, %ymm14, %ymm10, %ymm10

//	vbroadcastsd	24(%r11), %ymm12
//	vmulpd			%ymm12, %ymm0, %ymm1
//	vblendpd		$ 0x8, %ymm1, %ymm0, %ymm0

	vmovupd			0(%r13), %ymm12
	vblendpd		$ 0xc, %ymm12, %ymm0, %ymm0
	
	je				0f
	jl				1f

	vpermilpd		$ 0xf, %ymm0, %ymm12
	vperm2f128		$ 0x11, %ymm12, %ymm12, %ymm12
	vmulpd			%ymm10, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0

0:

//	vbroadcastsd	16(%r11), %ymm12
//	vmulpd			%ymm12, %ymm0, %ymm1
//	vblendpd		$ 0x4, %ymm1, %ymm0, %ymm0

	vpermilpd		$ 0x0, %ymm0, %ymm12
	vperm2f128		$ 0x11, %ymm12, %ymm12, %ymm12
	vmulpd			%ymm9, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0

1:

	vbroadcastsd	8(%r11), %ymm12
	vmulpd			%ymm12, %ymm0, %ymm1
	vblendpd		$ 0x2, %ymm1, %ymm0, %ymm0

	vpermilpd		$ 0x3, %ymm0, %ymm12
//	vperm2f128		$ 0x00, %ymm12, %ymm12, %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0
//	vbroadcastsd	8(%r11), %ymm12

	vmovsd			0(%r11), %xmm12
	vmulpd			%ymm12, %ymm0, %ymm1
	vblendpd		$ 0x1, %ymm1, %ymm0, %ymm0

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsv_lt_inv_2_lib4)
#endif





// common inner routine with file scope 
//
// triangular substitution with vector RHS
//
// input arguments:
// r10  <- E
// r11  <- inv_diag_E
// r12  <- k
// r13  <- x
// ymm0 <- [z0 z1 z2 z3]
// ymm12 <- dirty
// ymm13 <- dirty
//
// output arguments:
// r10  <- E
// r11  <- inv_diag_E
// r12  <- k
// r13  <- x
// ymm0 <- [z0 z1 z2 z3]
// ymm12 <- dirty
// ymm13 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSV_LT_INV_1_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsv_lt_inv_1_lib4)
#endif
	
	vxorpd			%ymm14, %ymm14, %ymm14

	vmovupd			0(%r13), %ymm12
	vblendpd		$ 0xe, %ymm12, %ymm0, %ymm0
	
	cmpl			$ 3, %r12d
	je				0f

	cmpl			$ 2, %r12d
	je				1f
	jl				2f

	vmovsd			24(%r10), %xmm10
	vblendpd		$ 0xe, %ymm14, %ymm10, %ymm10
	vpermilpd		$ 0xf, %ymm0, %ymm12
	vperm2f128		$ 0x11, %ymm12, %ymm12, %ymm12
	vmulpd			%ymm10, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0

0:

	vmovsd			16(%r10), %xmm9
	vblendpd		$ 0xe, %ymm14, %ymm9, %ymm9
	vpermilpd		$ 0x0, %ymm0, %ymm12
	vperm2f128		$ 0x11, %ymm12, %ymm12, %ymm12
	vmulpd			%ymm9, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0

1:

	vmovsd			8(%r10), %xmm8
	vblendpd		$ 0xe, %ymm14, %ymm8, %ymm8
	vpermilpd		$ 0x3, %ymm0, %ymm12
//	vperm2f128		$ 0x00, %ymm12, %ymm12, %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0
//	vbroadcastsd	8(%r11), %ymm12

2:

	vmovsd			0(%r11), %xmm12
	vmulpd			%ymm12, %ymm0, %ymm1
	vblendpd		$ 0x1, %ymm1, %ymm0, %ymm0

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsv_lt_inv_1_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution with vector RHS
//
// input arguments:
// r10  <- E
// r11  <- inv_diag_E
// ymm0 <- [z0 z1 z2 z3]
//
// output arguments:
// r10  <- E
// r11  <- inv_diag_E
// ymm0 <- [z0 z1 z2 z3]

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSV_UN_INV_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsv_un_inv_4_lib4)
#endif
	
	vxorpd			%ymm14, %ymm14, %ymm14

	vpermilpd		$ 0xf, %ymm0, %ymm12
	vperm2f128		$ 0x11, %ymm12, %ymm12, %ymm12
	vbroadcastsd	24(%r11), %ymm13
	vmulpd			%ymm13, %ymm12, %ymm12
	vblendpd		$ 0x8, %ymm12, %ymm0, %ymm0
	vmovapd			96(%r10), %ymm13
	vblendpd		$ 0x8, %ymm14, %ymm13, %ymm13
	vmulpd			%ymm12, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0

	vpermilpd		$ 0x0, %ymm0, %ymm12
	vperm2f128		$ 0x11, %ymm12, %ymm12, %ymm12
	vbroadcastsd	16(%r11), %ymm13
	vmulpd			%ymm13, %ymm12, %ymm12
	vblendpd		$ 0x4, %ymm12, %ymm0, %ymm0
	vmovapd			64(%r10), %ymm13
	vblendpd		$ 0xc, %ymm14, %ymm13, %ymm13
	vmulpd			%ymm12, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0

	vpermilpd		$ 0xf, %ymm0, %ymm12
//	vperm2f128		$ 0x00, %ymm12, %ymm12, %ymm12
	vbroadcastsd	8(%r11), %ymm13
	vmulpd			%ymm13, %ymm12, %ymm12
	vblendpd		$ 0x2, %ymm12, %ymm0, %ymm0
	vmovapd			32(%r10), %ymm13
	vblendpd		$ 0xe, %ymm14, %ymm13, %ymm13
	vmulpd			%ymm12, %ymm13, %ymm15
	vsubpd			%ymm15, %ymm0, %ymm0

	vmovsd			(%r11), %xmm13
	vmulsd			%xmm13, %xmm0, %xmm12
	vblendpd		$ 0x1, %ymm12, %ymm0, %ymm0

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsv_un_inv_4_lib4)
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
// r10d  <- k-4
// r11   <- A+4*4*sizeof(double)
// r12   <- x+4*sizeof(double)
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

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMV_UN_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmv_un_4_lib4)
#endif
	
	vxorpd			%ymm14, %ymm14, %ymm14

	vmovapd			0(%r11), %ymm8
	vblendpd		$ 0x1, %ymm8, %ymm14, %ymm8
	vbroadcastsd	0(%r12), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
	
	subl			$ 4, %r10d

	vmovapd			32(%r11), %ymm8
	vblendpd		$ 0x3, %ymm8, %ymm14, %ymm8
	vbroadcastsd	8(%r12), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	
	vmovapd			64(%r11), %ymm8
	vblendpd		$ 0x7, %ymm8, %ymm14, %ymm8
	vbroadcastsd	16(%r12), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2

	vmovapd			96(%r11), %ymm8
	vbroadcastsd	24(%r12), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm3, %ymm15, %ymm3
	
	addq			$ 128, %r11
	addq			$ 32, %r12
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmv_un_4_lib4)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- bs*sda*sizeof(double) = 32*sda
// r13   <- x
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
// r12   <- bs*sda*sizeof(double) = 32*sda
// r13   <- x+k*sizeof(double)
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

#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_DTRMV_UT_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dtrmv_ut_4_lib4)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	cmpl	$ 4, %r10d
	jle		0f // clean-up loop

	// main loop
	.p2align 3
1: // main loop
	
	vmovupd	0(%r13), %ymm12

	vmovapd	0(%r11), %ymm8
	vmulpd	%ymm8, %ymm12, %ymm15
	vaddpd	%ymm0, %ymm15, %ymm0
	
	subl	$ 4, %r10d

	vmovapd	32(%r11), %ymm8
	vmulpd	%ymm8, %ymm12, %ymm15
	vaddpd	%ymm1, %ymm15, %ymm1
	
	vmovapd	64(%r11), %ymm8
	vmulpd	%ymm8, %ymm12, %ymm15
	vaddpd	%ymm2, %ymm15, %ymm2

	vmovapd	96(%r11), %ymm8
	vmulpd	%ymm8, %ymm12, %ymm15
	vaddpd	%ymm3, %ymm15, %ymm3
	
	addq	%r12, %r11
	addq	$ 32, %r13
	
	cmpl	$ 4, %r10d

	jg		1b // main loop 


	// consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

0: // clean-up
	
//	vcvtsi2sd	%r10d, %xmm14, %xmm14
#if defined(OS_LINUX) | defined(OS_WINDOWS)
//	vmovupd		.LC02(%rip), %ymm13
#elif defined(OS_MAC)
//	vmovupd		LC02(%rip), %ymm13
#endif
//	vmovddup	%xmm14, %xmm14
//	vinsertf128	$ 1, %xmm14, %ymm14, %ymm14
//	vsubpd		%ymm14, %ymm13, %ymm14
//
//	vmaskmovpd	0(%r13), %ymm14, %ymm12

	vmovupd		0(%r13), %ymm12

	vxorpd		%ymm14, %ymm14, %ymm14

	vmovapd		0(%r11), %ymm8
	vblendpd	$ 0x1, %ymm8, %ymm14, %ymm8
	vmulpd		%ymm8, %ymm12, %ymm15
	vaddpd		%ymm0, %ymm15, %ymm0
	
	vmovapd	32(%r11), %ymm8
	vblendpd	$ 0x3, %ymm8, %ymm14, %ymm8
	vmulpd		%ymm8, %ymm12, %ymm15
	vaddpd		%ymm1, %ymm15, %ymm1
	
	vmovapd		64(%r11), %ymm8
	vblendpd	$ 0x7, %ymm8, %ymm14, %ymm8
	vmulpd		%ymm8, %ymm12, %ymm15
	vaddpd		%ymm2, %ymm15, %ymm2

	vmovapd		96(%r11), %ymm8
	vmulpd		%ymm8, %ymm12, %ymm15
	vaddpd		%ymm3, %ymm15, %ymm3
		
	sall		$ 3, %r10d
//	movslq		%r10d, %r10
	addq		%r10, %r11
	addq		%r10, %r13
	xorl		%r10d, %r10d
	
	
2: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_dtrmv_ut_4_lib4)
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
	.macro INNER_STORE_4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_4_lib4)
#endif
	
	vmovupd %ymm0,  0(%r10)
	
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
	.macro INNER_STORE_4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_4_vs_lib4)
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

	vmaskmovpd	%ymm0, %ymm15,  0(%r10)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4_vs_lib4)
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
	.macro INNER_STORE_4_GEN_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_4_gen_lib4)
#endif
	
	// compute mask for rows
	vcvtsi2sd	%r11d, %xmm14, %xmm14
	vcvtsi2sd	%r12d, %xmm15, %xmm15
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

	vmaskmovpd	%ymm0, %ymm15,  0(%r10)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4_gen_lib4)
#endif





//                            1      2              3          4          5             6          7
// void kernel_dgemv_n_4_lib4(int k, double *alpha, double *A, double *x, double *beta, double *y, double *z);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemv_n_4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // x

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_N_4_LIB4
#else
	CALL(inner_kernel_gemv_add_n_4_lib4)
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

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // x

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_N_4_LIB4
#else
	CALL(inner_kernel_gemv_add_n_4_lib4)
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





//                                1      2              3          4          5             6          7          8       9
// void kernel_dgemv_n_4_gen_lib4(int k, double *alpha, double *A, double *x, double *beta, double *y, double *z, int k0, int k1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemv_n_4_gen_lib4)
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // x

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_N_4_LIB4
#else
	CALL(inner_kernel_gemv_add_n_4_lib4)
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
	movq	ARG8, %r11 // k0 
	movq	ARG9, %r12 // k1 

#if MACRO_LEVEL>=1
	INNER_STORE_4_GEN_LIB4
#else
	CALL(inner_store_4_gen_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemv_n_4_gen_lib4)





//                            1      2              3          4        5          6             7         8          9
// void kernel_dgemv_t_4_lib4(int k, double *alpha, int offa, double *A, int sda, double *x, double *beta, double *y, double *z);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemv_t_4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG4, %r11  // A
	movq	ARG5, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
//	movslq	%r12d, %r12
	movq	ARG6, %r13  // x
	movq	ARG3, %r14 // offA

#if MACRO_LEVEL>=1
	INNER_EDGE_GEMV_ADD_T_4_LIB4
#else
	CALL(inner_edge_gemv_add_t_4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_T_4_LIB4
#else
	CALL(inner_kernel_gemv_add_t_4_lib4)
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

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG4, %r11  // A
	movq	ARG5, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
//	movslq	%r12d, %r12
	movq	ARG6, %r13  // x
	movq	ARG3, %r14 // offA

#if MACRO_LEVEL>=1
	INNER_EDGE_GEMV_ADD_T_4_LIB4
#else
	CALL(inner_edge_gemv_add_t_4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_T_4_LIB4
#else
	CALL(inner_kernel_gemv_add_t_4_lib4)
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





//                                 1      2          3                   4          5          6
// void kernel_dtrsv_ln_inv_4_lib4(int k, double *A, double *inv_diag_A, double *x, double *y, double *z);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsv_ln_inv_4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG4, %r12  // x

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_N_4_LIB4
#else
	CALL(inner_kernel_gemv_add_n_4_lib4)
#endif

	movq	%r11, %r13 // A+k*sizeof(double)


	// call inner blender n

	movq	ARG5, %r10   // y

#if MACRO_LEVEL>=1
	INNER_BLEND_N_SCALE_M11_4_LIB4
#else
	CALL(inner_blend_n_scale_m11_4_lib4)
#endif


	// solution

	movq	%r13, %r10 // A+k*sizeof(double)
	movq	ARG3, %r11 // inv_diag_A

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSV_LN_INV_4_LIB4
#else
	CALL(inner_edge_dtrsv_ln_inv_4_lib4)
#endif


	// store

	movq	ARG6, %r10 // z 

#if MACRO_LEVEL>=1
	INNER_STORE_4_LIB4
#else
	CALL(inner_store_4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsv_ln_inv_4_lib4)





//                                    1      2          3                   4          5          6          7       8
// void kernel_dtrsv_ln_inv_4_vs_lib4(int k, double *A, double *inv_diag_A, double *x, double *y, double *z, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsv_ln_inv_4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG4, %r12  // x

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_N_4_LIB4
#else
	CALL(inner_kernel_gemv_add_n_4_lib4)
#endif

	movq	%r11, %r13


	// call inner blender n

	movq	ARG5, %r10   // y

#if MACRO_LEVEL>=1
	INNER_BLEND_N_SCALE_M11_4_LIB4
#else
	CALL(inner_blend_n_scale_m11_4_lib4)
#endif


	// solution

	movq	%r13, %r10 // A+k*sizeof(double)
	movq	ARG3, %r11 // inv_diag_A
	movq	ARG8, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSV_LN_INV_4_VS_LIB4
#else
	CALL(inner_edge_dtrsv_ln_inv_4_vs_lib4)
#endif


	// store vs

	movq	ARG6, %r10 // z 
	movq	ARG7, %r11 // km 

#if MACRO_LEVEL>=1
	INNER_STORE_4_VS_LIB4
#else
	CALL(inner_store_4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsv_ln_inv_4_vs_lib4)





//                                 1      2          3          4          5
// void kernel_dtrsv_ln_one_4_lib4(int k, double *A, double *x, double *y, double *z);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsv_ln_one_4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12  // x

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_N_4_LIB4
#else
	CALL(inner_kernel_gemv_add_n_4_lib4)
#endif

	movq	%r11, %r13 // A+k*sizeof(double)


	// call inner blender n

	movq	ARG4, %r10   // y

#if MACRO_LEVEL>=1
	INNER_BLEND_N_SCALE_M11_4_LIB4
#else
	CALL(inner_blend_n_scale_m11_4_lib4)
#endif


	// solution

	movq	%r13, %r10 // A+k*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSV_LN_ONE_4_LIB4
#else
	CALL(inner_edge_dtrsv_ln_one_4_lib4)
#endif


	// store

	movq	ARG5, %r10 // z 

#if MACRO_LEVEL>=1
	INNER_STORE_4_LIB4
#else
	CALL(inner_store_4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsv_ln_one_4_lib4)





//                                    1      2          3          4          5          6       7
// void kernel_dtrsv_ln_one_4_vs_lib4(int k, double *A, double *x, double *y, double *z, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsv_ln_one_4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12  // x

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_N_4_LIB4
#else
	CALL(inner_kernel_gemv_add_n_4_lib4)
#endif

	movq	%r11, %r13


	// call inner blender n

	movq	ARG4, %r10   // y

#if MACRO_LEVEL>=1
	INNER_BLEND_N_SCALE_M11_4_LIB4
#else
	CALL(inner_blend_n_scale_m11_4_lib4)
#endif


	// solution

	movq	%r13, %r10 // A+k*sizeof(double)
	movq	ARG7, %r11 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSV_LN_ONE_4_VS_LIB4
#else
	CALL(inner_edge_dtrsv_ln_one_4_vs_lib4)
#endif


	// store vs

	movq	ARG5, %r10 // z 
	movq	ARG6, %r11 // km 

#if MACRO_LEVEL>=1
	INNER_STORE_4_VS_LIB4
#else
	CALL(inner_store_4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsv_ln_one_4_vs_lib4)





//                                 1      2          3        4                   5          6          7
// void kernel_dtrsv_lt_inv_4_lib4(int k, double *A, int sda, double *inv_diag_A, double *x, double *y, double *z);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsv_lt_inv_4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	subl	$ 4, %r10d
	movq	ARG2, %r11 // A
	movq	ARG3, %r12
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	addq	%r12, %r11 // A+4*sda*sizeof(double)
	movq	ARG5, %r13 // x
	addq	$ 32, %r13 // x+4 

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_T_4_LIB4
#else
	CALL(inner_kernel_gemv_add_t_4_lib4)
#endif


	// call inner blender t

	movq	ARG6, %r10 // y 

#if MACRO_LEVEL>=1
	INNER_BLEND_T_SCALE_M11_4_LIB4
#else
	CALL(inner_blend_t_scale_m11_4_lib4)
#endif


	// solution

	movq	ARG2, %r10 // A
	movq	ARG4, %r11 // inv_diag_A

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSV_LT_INV_4_LIB4
#else
	CALL(inner_edge_dtrsv_lt_inv_4_lib4)
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

	FUN_END(kernel_dtrsv_lt_inv_4_lib4)





//                                 rdi    rsi        rdx      rcx                 r8         r9         rsp+8   
// void kernel_dtrsv_lt_inv_3_lib4(int k, double *A, int sda, double *inv_diag_A, double *x, double *y, double *z);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsv_lt_inv_3_lib4)
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	subl	$ 4, %r10d
	movq	ARG2, %r11 // A
	movq	ARG3, %r12
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	addq	%r12, %r11 // A+4*sda*sizeof(double)
	movq	ARG5, %r13 // x
	addq	$ 32, %r13 // x+4 

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_T_4_LIB4
#else
	CALL(inner_kernel_gemv_add_t_4_lib4)
#endif


	// call inner blender t

	movq	ARG6, %r10 // y 

#if MACRO_LEVEL>=1
	INNER_BLEND_T_SCALE_M11_4_LIB4
#else
	CALL(inner_blend_t_scale_m11_4_lib4)
#endif


	// solution

	movq	ARG2, %r10 // A
	movq	ARG4, %r11 // inv_diag_A
	movq	ARG1, %r12 // k
	movq	ARG5, %r13 // x

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSV_LT_INV_3_LIB4
#else
	CALL(inner_edge_dtrsv_lt_inv_3_lib4)
#endif


	// store

	movq	ARG7, %r10 // z 
	movq	$ 3, %r11

#if MACRO_LEVEL>=1
	INNER_STORE_4_VS_LIB4
#else
	CALL(inner_store_4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsv_lt_inv_3_lib4)





//                                 rdi    rsi        rdx      rcx                 r8         r9         rsp+8 
// void kernel_dtrsv_lt_inv_2_lib4(int k, double *A, int sda, double *inv_diag_A, double *x, double *y, double *z);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsv_lt_inv_2_lib4)
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	subl	$ 4, %r10d
	movq	ARG2, %r11 // A
	movq	ARG3, %r12
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movslq	%r12d, %r12
	addq	%r12, %r11 // A+4*sda*sizeof(double)
	movq	ARG5, %r13 // x
	addq	$ 32, %r13 // x+4 

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_T_4_LIB4
#else
	CALL(inner_kernel_gemv_add_t_4_lib4)
#endif


	// call inner blender t

	movq	ARG6, %r10 // y 

#if MACRO_LEVEL>=1
	INNER_BLEND_T_SCALE_M11_4_LIB4
#else
	CALL(inner_blend_t_scale_m11_4_lib4)
#endif


	// solution

	movq	ARG2, %r10 // A
	movq	ARG4, %r11 // inv_diag_A
	movq	ARG1, %r12 // k
	movq	ARG5, %r13 // x

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSV_LT_INV_2_LIB4
#else
	CALL(inner_edge_dtrsv_lt_inv_2_lib4)
#endif


	// store

	movq	ARG7, %r10 // z 
	movq	$ 2, %r11

#if MACRO_LEVEL>=1
	INNER_STORE_4_VS_LIB4
#else
	CALL(inner_store_4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsv_lt_inv_2_lib4)





//                                 rdi    rsi        rdx      rcx                 r8         r9         rsp+8 
// void kernel_dtrsv_lt_inv_1_lib4(int k, double *A, int sda, double *inv_diag_A, double *x, double *y, double *z);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsv_lt_inv_1_lib4)
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	subl	$ 4, %r10d
	movq	ARG2, %r11 // A
	movq	ARG3, %r12
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movslq	%r12d, %r12
	addq	%r12, %r11 // A+4*sda*sizeof(double)
	movq	ARG5, %r13 // x
	addq	$ 32, %r13 // x+4 

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_T_4_LIB4
#else
	CALL(inner_kernel_gemv_add_t_4_lib4)
#endif


	// call inner blender t

	movq	ARG6, %r10 // y 

#if MACRO_LEVEL>=1
	INNER_BLEND_T_SCALE_M11_4_LIB4
#else
	CALL(inner_blend_t_scale_m11_4_lib4)
#endif


	// solution

	movq	ARG2, %r10 // A
	movq	ARG4, %r11 // inv_diag_A
	movq	ARG1, %r12 // k
	movq	ARG5, %r13 // x

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSV_LT_INV_1_LIB4
#else
	CALL(inner_edge_dtrsv_lt_inv_1_lib4)
#endif


	// store

	movq	ARG7, %r10 // z 
	movq	$ 1, %r11

#if MACRO_LEVEL>=1
	INNER_STORE_4_VS_LIB4
#else
	CALL(inner_store_4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsv_lt_inv_1_lib4)





//                                 1      2          3                   4          5          6
// void kernel_dtrsv_un_inv_4_lib4(int k, double *A, double *inv_diag_A, double *x, double *y, double *z);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsv_un_inv_4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	subl	$ 4, %r10d
	movq	ARG2, %r11 // A
	addq	$ 128, %r11 // A+4*ps*sizeof(double)
	movq	ARG4, %r12 // x
	addq	$ 32, %r12 // x+4 

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_N_4_LIB4
#else
	CALL(inner_kernel_gemv_add_n_4_lib4)
#endif


	// call inner blender t

	movq	ARG5, %r10 // y 

#if MACRO_LEVEL>=1
	INNER_BLEND_N_SCALE_M11_4_LIB4
#else
	CALL(inner_blend_n_scale_m11_4_lib4)
#endif


	// solution

	movq	ARG2, %r10 // A
	movq	ARG3, %r11 // inv_diag_A

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSV_UN_INV_4_LIB4
#else
	CALL(inner_edge_dtrsv_un_inv_4_lib4)
#endif


	// store

	movq	ARG6, %r10 // z 

#if MACRO_LEVEL>=1
	INNER_STORE_4_LIB4
#else
	CALL(inner_store_4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsv_un_inv_4_lib4)





//                            rdi    rsi        rdx        rcx
// void kernel_dtrmv_un_4_lib4(int k, double *A, double *x, double *z);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmv_un_4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3


	// call inner dtrmv edge & dgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12  // x

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMV_UN_4_LIB4
#else
	CALL(inner_edge_dtrmv_un_4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_N_4_LIB4
#else
	CALL(inner_kernel_gemv_add_n_4_lib4)
#endif


	// call inner blend n

#if MACRO_LEVEL>=1
	INNER_BLEND_N_LIB4
#else
	CALL(inner_blend_n_lib4)
#endif


	// store

	movq	ARG4, %r10 // z

#if MACRO_LEVEL>=1
	INNER_STORE_4_LIB4
#else
	CALL(inner_store_4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmv_un_4_lib4)





//                             rdi    rsi        rdx      rcx        r8
// void kernel_dtrmv_ut_4_lib4(int k, double *A, int sda, double *x, double *z);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmv_ut_4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movslq	%r12d, %r12
	movq	ARG4, %r13  // x

#if MACRO_LEVEL>=1
	INNER_KERNEL_DTRMV_UT_4_LIB4
#else
	CALL(inner_kernel_dtrmv_ut_4_lib4)
#endif


	// call inner blend t

#if MACRO_LEVEL>=1
	INNER_BLEND_T_LIB4
#else
	CALL(inner_blend_t_lib4)
#endif


	// store

	movq	ARG5, %r10 // z 

#if MACRO_LEVEL>=1
	INNER_STORE_4_LIB4
#else
	CALL(inner_store_4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmv_ut_4_lib4)





//                                rdi    rsi        rdx      rcx        r8         r9
// void kernel_dtrmv_ut_4_vs_lib4(int k, double *A, int sda, double *x, double *y, int km);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmv_ut_4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movslq	%r12d, %r12
	movq	ARG4, %r13  // x

#if MACRO_LEVEL>=1
	INNER_KERNEL_DTRMV_UT_4_LIB4
#else
	CALL(inner_kernel_dtrmv_ut_4_lib4)
#endif


	// call inner blend t

#if MACRO_LEVEL>=1
	INNER_BLEND_T_LIB4
#else
	CALL(inner_blend_t_lib4)
#endif


	// store

	movq	ARG5, %r10 // z 
	movq	ARG6, %r11 // km 

#if MACRO_LEVEL>=1
	INNER_STORE_4_VS_LIB4
#else
	CALL(inner_store_4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmv_ut_4_vs_lib4)





//                             1      2                3                4          5        6            7            8               9            10           11
// void kernel_dgemv_nt_4_lib4(int k, double *alpha_n, double *alpha_t, double *A, int sda, double *x_n, double *x_t, double *beta_t, double *y_t, double *z_n, double *z_t);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemv_nt_4_lib4)
	
	PROLOGUE

	// zero accumulation registers y_t

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3

	// initialize x_n
	movq	ARG2, %r10 // alpha_n
	vbroadcastsd 0(%r10), %ymm15

	movq	ARG6, %r10 // x_n

	vbroadcastsd 0(%r10), %ymm6
	vmulpd		%ymm15, %ymm6, %ymm6
	vbroadcastsd 8(%r10), %ymm7
	vmulpd		%ymm15, %ymm7, %ymm7
	vbroadcastsd 16(%r10), %ymm8
	vmulpd		%ymm15, %ymm8, %ymm8
	vbroadcastsd 24(%r10), %ymm9
	vmulpd		%ymm15, %ymm9, %ymm9


	// inner kernel dgemv nt

	movq	ARG1, %r10 // k
	movq	ARG4, %r11  // A
	movq	ARG5, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
//	movslq	%r12d, %r12
	movq	ARG7, %r13  // x_t
	movq	ARG10, %r14  // z_n

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_NT_4_LIB4
#else
	CALL(inner_kernel_gemv_add_nt_4_lib4)
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

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3

	vmovapd	%ymm0, %ymm6
	vmovapd	%ymm0, %ymm7
	vmovapd	%ymm0, %ymm8
	vmovapd	%ymm0, %ymm9

	// initialize x_n
	movq	ARG2, %r10 // alpha_n
	vbroadcastsd 0(%r10), %ymm15

	movq	ARG6, %r10 // x_n
	movq	ARG12, %r11 // km

	vbroadcastsd 0(%r10), %ymm6
	vmulpd		%ymm15, %ymm6, %ymm6
	cmpl	$ 2, %r11d
	jl		0f
	vbroadcastsd 8(%r10), %ymm7
	vmulpd		%ymm15, %ymm7, %ymm7
	cmpl	$ 3, %r11d
	jl		0f
	vbroadcastsd 16(%r10), %ymm8
	vmulpd		%ymm15, %ymm8, %ymm8
	je		0f
	vbroadcastsd 24(%r10), %ymm9
	vmulpd		%ymm15, %ymm9, %ymm9
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
	INNER_KERNEL_GEMV_ADD_NT_4_LIB4
#else
	CALL(inner_kernel_gemv_add_nt_4_lib4)
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

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3

	// initialize x_n
	movq	ARG2, %r10 // alpha
	vbroadcastsd 0(%r10), %ymm15

	movq	ARG5, %r10 // x_n

	vbroadcastsd 0(%r10), %ymm6
	vmulpd		%ymm15, %ymm6, %ymm6
	vbroadcastsd 8(%r10), %ymm7
	vmulpd		%ymm15, %ymm7, %ymm7
	vbroadcastsd 16(%r10), %ymm8
	vmulpd		%ymm15, %ymm8, %ymm8
	vbroadcastsd 24(%r10), %ymm9
	vmulpd		%ymm15, %ymm9, %ymm9


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
	INNER_KERNEL_GEMV_ADD_NT_4_LIB4
#else
	CALL(inner_kernel_gemv_add_nt_4_lib4)
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





//                                1      2              3         4          5        6           7
// void kernel_dsymv_l_4_gen_lib4(int k, double *alpha, int offA, double *A, int sda, double *x, double *z);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsymv_l_4_gen_lib4)
	
	PROLOGUE

	// zero accumulation registers y_t

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3

	// initialize x_n
	movq	ARG2, %r10 // alpha
	vbroadcastsd 0(%r10), %ymm15

	movq	ARG6, %r10 // x_n

	vbroadcastsd 0(%r10), %ymm6
	vmulpd		%ymm15, %ymm6, %ymm6
	vbroadcastsd 8(%r10), %ymm7
	vmulpd		%ymm15, %ymm7, %ymm7
	vbroadcastsd 16(%r10), %ymm8
	vmulpd		%ymm15, %ymm8, %ymm8
	vbroadcastsd 24(%r10), %ymm9
	vmulpd		%ymm15, %ymm9, %ymm9


	// inner edge dsyrk & kernel dgemv nt

	movq	ARG1, %r10 // k
	movq	ARG4, %r11  // A
	movq	ARG5, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG6, %r13  // x_t
	movq	ARG7, %r14  // z_n
	movq	ARG3, %r15 // offA

#if MACRO_LEVEL>=2
	INNER_EDGE_DSYMV_ADD_NT_4_GEN_LIB4
#else
	CALL(inner_edge_dsymv_add_nt_4_gen_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_ADD_NT_4_LIB4
#else
	CALL(inner_kernel_gemv_add_nt_4_lib4)
#endif


	// call inner blend n scale ab

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11   // z_t

#if MACRO_LEVEL>=1
	INNER_BLEND_T_SCALE_A1_4_LIB4
#else
	CALL(inner_blend_t_scale_a1_4_lib4)
#endif


	// store

	movq	ARG7, %r10 // z_t 
	movq	ARG8, %r11 // km

#if MACRO_LEVEL>=1
	INNER_STORE_4_VS_LIB4
#else
	CALL(inner_store_4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dsymv_l_4_gen_lib4)





//#if defined(BLAS_API)
#if ( defined(BLAS_API) | ( defined(LA_HIGH_PERFORMANCE) & defined(MF_COLMAJ) ) )

#include "kernel_dgemv_4_lib.S"

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



#if defined(OS_LINUX)
	.section	.note.GNU-stack,"",@progbits
#elif defined(OS_MAC)
	.subsections_via_symbols
#endif


