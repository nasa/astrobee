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

#else

#error wrong OS

#endif



#if defined(OS_LINUX)
	.text
#elif defined(OS_MAC)
	.section	__TEXT,__text,regular,pure_instructions
#endif



// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- lda
// r13   <- B
// r14   <- ldb
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_ADD_NN_4X4_LIB
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_dgemm_add_nn_4x4_lib, @function
inner_kernel_dgemm_add_nn_4x4_lib:
#elif defined(OS_MAC)
_inner_kernel_dgemm_add_nn_4x4_lib:
#elif defined(OS_WINDOWS)
	.def inner_kernel_dgemm_add_nn_4x4_lib; .scl 2; .type 32; .endef
inner_kernel_dgemm_add_nn_4x4_lib:
#endif
#endif

	cmpl	$0, %r10d
	jle		2f // return

	movq	%r13, %r15
	addq	%r14, %r15
	addq	%r14, %r15 // B+2*ldb

	// preload

	cmpl	$4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0	0(%r12, %r13, 2) // software prefetch
//	prefetcht0	64(%r12, %r13, 2) // software prefetch

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vbroadcastsd	0(%r13, %r14, 1), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vbroadcastsd	0(%r15), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vbroadcastsd	0(%r15, %r14, 1), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	addq	%r12, %r11

	// unroll 1
	vmovupd			0(%r11), %ymm13 // A
	vbroadcastsd	8(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vbroadcastsd	8(%r13, %r14, 1), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vbroadcastsd	8(%r15), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vbroadcastsd	8(%r15, %r14, 1), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	addq	%r12, %r11

	// unroll 2
	vmovupd			0(%r11), %ymm13 // A
	vbroadcastsd	16(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vbroadcastsd	16(%r13, %r14, 1), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vbroadcastsd	16(%r15), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vbroadcastsd	16(%r15, %r14, 1), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	addq	%r12, %r11

	// unroll 3
	vmovupd			0(%r11), %ymm13 // A
	vbroadcastsd	24(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vbroadcastsd	24(%r13, %r14, 1), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vbroadcastsd	24(%r15), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vbroadcastsd	24(%r15, %r14, 1), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	addq	%r12, %r11

	subl	$4, %r10d
	addq	$32, %r13
	addq	$32, %r15

	cmpl	$4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vbroadcastsd	0(%r13, %r14, 1), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vbroadcastsd	0(%r15), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vbroadcastsd	0(%r15, %r14, 1), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	addq	%r12, %r11

	// unroll 1
	vmovupd			0(%r11), %ymm13 // A
	vbroadcastsd	8(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vbroadcastsd	8(%r13, %r14, 1), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vbroadcastsd	8(%r15), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vbroadcastsd	8(%r15, %r14, 1), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	addq	%r12, %r11

	// unroll 2
	vmovupd			0(%r11), %ymm13 // A
	vbroadcastsd	16(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vbroadcastsd	16(%r13, %r14, 1), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vbroadcastsd	16(%r15), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vbroadcastsd	16(%r15, %r14, 1), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	addq	%r12, %r11

	// unroll 3
	vmovupd			0(%r11), %ymm13 // A
	vbroadcastsd	24(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vbroadcastsd	24(%r13, %r14, 1), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vbroadcastsd	24(%r15), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vbroadcastsd	24(%r15, %r14, 1), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	addq	%r12, %r11

	subl	$4, %r10d
	addq	$32, %r13
	addq	$32, %r15

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovupd			0(%r11), %ymm13 // A
	vbroadcastsd	0(%r13), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vbroadcastsd	0(%r13, %r14, 1), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vbroadcastsd	0(%r15), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vbroadcastsd	0(%r15, %r14, 1), %ymm12 // B
	vmulpd			%ymm13, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	addq	%r12, %r11

	subl	$1, %r10d
	addq	$8, %r13
	addq	$8, %r15

	cmpl	$0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_dgemm_add_nn_4x4_lib, .-inner_kernel_dgemm_add_nn_4x4_lib
#endif
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- ldc
// ymm0 <- [d00 d11 d22 d33]
// ymm1 <- [d01 d10 d23 d32]
// ymm2 <- [d03 d12 d21 d30]
// ymm3 <- [d02 d13 d20 d31]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_4X4_LIB
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_scale_ab_4x4_lib, @function
inner_scale_ab_4x4_lib:
#elif defined(OS_MAC)
_inner_scale_ab_4x4_lib:
#elif defined(OS_WINDOWS)
	.def inner_scale_ab_4x4_lib; .scl 2; .type 32; .endef
inner_scale_ab_4x4_lib:
#endif
#endif

	// alpha
	vbroadcastsd	0(%r10), %ymm15

	vmulpd		%ymm0, %ymm15, %ymm0
	vmulpd		%ymm1, %ymm15, %ymm1
	vmulpd		%ymm2, %ymm15, %ymm2
	vmulpd		%ymm3, %ymm15, %ymm3

	// beta
	vbroadcastsd	0(%r11), %ymm14

	vxorpd		%ymm15, %ymm15, %ymm15 // 0.0

	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	vmovapd		0(%r12), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm0, %ymm15, %ymm0
	addq		%r13, %r12
	vmovapd		0(%r12), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm1, %ymm15, %ymm1
	addq		%r13, %r12
	vmovapd		0(%r12), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm2, %ymm15, %ymm2
	addq		%r13, %r12
	vmovapd		0(%r12), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm3, %ymm15, %ymm3
	addq		%r13, %r12

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_scale_ab_4x4_lib, .-inner_scale_ab_4x4_lib
#endif
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// r11  <- ldd
// ymm0 <- [d00 d11 d22 d33]
// ymm1 <- [d01 d10 d23 d32]
// ymm2 <- [d03 d12 d21 d30]
// ymm3 <- [d02 d13 d20 d31]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X4_LIB
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_store_4x4_lib, @function
inner_store_4x4_lib:
#elif defined(OS_MAC)
_inner_store_4x4_lib:
#elif defined(OS_WINDOWS)
	.def inner_store_4x4_lib; .scl 2; .type 32; .endef
inner_store_4x4_lib:
#endif
#endif

	vmovupd		%ymm0, 0(%r10)
	addq		%r11, %r10
	vmovupd		%ymm1, 0(%r10)
	addq		%r11, %r10
	vmovupd		%ymm2, 0(%r10)
	addq		%r11, %r10
	vmovupd		%ymm3, 0(%r10)
//	addq	%r11, %r10

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_store_4x4_lib, .-inner_store_4x4_lib
#endif
#endif





//                              1      2              3          4        5          6        7             8          9        10         11
// void kernel_dgemm_nn_4x4_lib(int k, double *alpha, double *A, int lda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dgemm_nn_4x4_lib
	.type kernel_dgemm_nn_4x4_lib, @function
kernel_dgemm_nn_4x4_lib:
#elif defined(OS_MAC)
	.globl _kernel_dgemm_nn_4x4_lib
_kernel_dgemm_nn_4x4_lib:
#elif defined(OS_WINDOWS)
	.globl kernel_dgemm_nn_4x4_lib
	.def kernel_dgemm_nn_4x4_lib; .scl 2; .type 32; .endef
kernel_dgemm_nn_4x4_lib:
#endif

	PROLOGUE

	// zero accumulation registers

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // lda
	sall	$3, %r12d
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_ADD_NN_4X4_LIB
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_dgemm_add_nn_4x4_lib
#elif defined(OS_MAC)
	callq _inner_kernel_dgemm_add_nn_4x4_lib
#endif
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_scale_ab_4x4_lib
#elif defined(OS_MAC)
	callq _inner_scale_ab_4x4_lib
#endif
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_4x4_lib
#elif defined(OS_MAC)
	callq _inner_store_4x4_lib
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dgemm_nn_4x4_lib, .-kernel_dgemm_nn_4x4_lib
#endif






