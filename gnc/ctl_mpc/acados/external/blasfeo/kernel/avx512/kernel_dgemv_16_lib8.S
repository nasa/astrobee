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
// r12   <- sda*...
// r13  <- x
// zmm0  <- []_a
// ...
// zmm7  <- []_d

//
// output arguments:
// r10d  <- 0
// r11   <- A+4*k*sizeof(double)
// r12   <- sda*...
// r13  <- x+k*sizeof(double)
// zmm0  <- []_a
// ...
// zmm7  <- []_d

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMV_N_16_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemv_n_16_lib8)
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	cmpl	$ 4, %r10d
	jl		0f // clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// TODO vbroadcastf64x2 + vshufpd

	vbroadcastsd	0*8(%r13), %zmm25
	vmovapd			0*64(%r11), %zmm24
	vfmadd231pd		%zmm24, %zmm25, %zmm0
	vmovapd			0*64(%r11, %r12), %zmm24
	vfmadd231pd		%zmm24, %zmm25, %zmm4

	vbroadcastsd	1*8(%r13), %zmm25
	vmovapd			1*64(%r11), %zmm24
	vfmadd231pd		%zmm24, %zmm25, %zmm1
	vmovapd			1*64(%r11, %r12), %zmm24
	vfmadd231pd		%zmm24, %zmm25, %zmm5

	vbroadcastsd	2*8(%r13), %zmm25
	vmovapd			2*64(%r11), %zmm24
	vfmadd231pd		%zmm24, %zmm25, %zmm2
	vmovapd			2*64(%r11, %r12), %zmm24
	vfmadd231pd		%zmm24, %zmm25, %zmm6

	vbroadcastsd	3*8(%r13), %zmm25
	vmovapd			3*64(%r11), %zmm24
	vfmadd231pd		%zmm24, %zmm25, %zmm3
	vmovapd			3*64(%r11, %r12), %zmm24
	vfmadd231pd		%zmm24, %zmm25, %zmm7

	subl	$ 4, %r10d
	addq	$ 256, %r11
	addq	$ 32, %r13
	
	cmpl	$ 3, %r10d
	jg		1b // main loop 


	// consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

0: // clean-up
	
	vbroadcastsd	0*8(%r13), %zmm25
	vmovapd			0*64(%r11), %zmm24
	vfmadd231pd		%zmm24, %zmm25, %zmm0
	vmovapd			0*64(%r11, %r12), %zmm24
	vfmadd231pd		%zmm24, %zmm25, %zmm4
	
	subl	$ 1, %r10d
	addq	$ 64, %r11
	addq	$ 8, %r13
	
	cmpl	$ 0, %r10d
	jg		0b // clean

2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemv_n_16_lib8)
#endif





// common inner routine with file scope
//
// blend for ta==n
//
// input arguments:
// zmm0 <- []_a
// ...
// zmm3 <- []_d
// zmm4 <- []_a
// ...
// zmm7 <- []_d
//
// output arguments:
// zmm0 <-
// zmm1 <-

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_N_16_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_blend_n_16_lib8)
#endif

	// reduction
	vaddpd			%zmm0, %zmm1, %zmm0
	vaddpd			%zmm2, %zmm3, %zmm2

	vaddpd			%zmm4, %zmm5, %zmm4
	vaddpd			%zmm6, %zmm7, %zmm6

	vaddpd			%zmm0, %zmm2, %zmm0
	vaddpd			%zmm4, %zmm6, %zmm1

#if MACRO_LEVEL>=1
	.endm
#else
	ret
	
	FUN_END(inner_blend_n_16_lib8)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta
//
// input arguments:
// r10  <- alpha
// r11  <- beta
// r12  <- y
// zmm0 <- []
// zmm1 <- []
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_16_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_16_lib8)
#endif

	// alpha
	vbroadcastsd	0(%r10), %zmm24
	vmulpd			%zmm0, %zmm24, %zmm0
	vmulpd			%zmm1, %zmm24, %zmm1

	// beta
	vbroadcastsd	0(%r11), %zmm25

	vxorpd		%zmm24, %zmm24, %zmm24 // 0.0
	vucomisd	%xmm24, %xmm25 // beta==0.0 ?
	je			0f // end

	vmovupd			0(%r12), %zmm24
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovupd			64(%r12), %zmm24
	vfmadd231pd		%zmm25, %zmm24, %zmm1

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret
	
	FUN_END(inner_scale_ab_16_lib8)
#endif





// common inner routine with file scope
//
// store 
//
// input arguments:
// r10  <- z
// zmm0 <- []
// zmm1 <- []
//
// output arguments:
// r10  <- z
// zmm0 <- []
// zmm1 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_16_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_store_16_lib8)
#endif
	
	vmovupd %zmm0, 0(%r10)
	vmovupd %zmm1, 64(%r10)
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_16_lib8)
#endif





//                             1      2              3          4        5          6             7          8
// void kernel_dgemv_n_16_lib8(int k, double *alpha, double *A, int sda, double *x, double *beta, double *y, double *z);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemv_n_16_lib8)
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%zmm0, %zmm0, %zmm0
	vmovapd	%zmm0, %zmm1
	vmovapd	%zmm0, %zmm2
	vmovapd	%zmm0, %zmm3
	vmovapd	%zmm0, %zmm4
	vmovapd	%zmm0, %zmm5
	vmovapd	%zmm0, %zmm6
	vmovapd	%zmm0, %zmm7


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 6, %r12d // 8*sda*sizeof(double)
	movq	ARG5, %r13  // x

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_N_16_LIB8
#else
	CALL(inner_kernel_gemv_n_16_lib8)
#endif


	// call inner blend n scale ab

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11   // beta
	movq	ARG7, %r12   // y

#if MACRO_LEVEL>=1
	INNER_BLEND_N_16_LIB8
#else
	CALL(inner_blend_n_16_lib8)
#endif

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_16_LIB8
#else
	CALL(inner_scale_ab_16_lib8)
#endif


	// store

	movq	ARG8, %r10 // z 

#if MACRO_LEVEL>=1
	INNER_STORE_16_LIB8
#else
	CALL(inner_store_16_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemv_n_16_lib8)






