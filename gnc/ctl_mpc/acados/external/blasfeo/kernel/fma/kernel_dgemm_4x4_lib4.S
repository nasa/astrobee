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
	movq	%r15, 40(%rsp);
#define EPILOGUE \
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
	vmovapd	%ymm0, %ymm3
#define NEG_ACC \
	vmovapd		.LC11(%rip), %ymm15; \
	vxorpd		%ymm15, %ymm0, %ymm0; \
	vxorpd		%ymm15, %ymm1, %ymm1; \
	vxorpd		%ymm15, %ymm2, %ymm2; \
	vxorpd		%ymm15, %ymm3, %ymm3

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
	vmovapd	%ymm0, %ymm3
#define NEG_ACC \
	vmovapd		LC11(%rip), %ymm15; \
	vxorpd		%ymm15, %ymm0, %ymm0; \
	vxorpd		%ymm15, %ymm1, %ymm1; \
	vxorpd		%ymm15, %ymm2, %ymm2; \
	vxorpd		%ymm15, %ymm3, %ymm3

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
	vmovups	%xmm15, 208(%rsp);
#define EPILOGUE \
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
	vmovapd	%ymm0, %ymm3
#define NEG_ACC \
	vmovapd		.LC11(%rip), %ymm15; \
	vxorpd		%ymm15, %ymm0, %ymm0; \
	vxorpd		%ymm15, %ymm1, %ymm1; \
	vxorpd		%ymm15, %ymm2, %ymm2; \
	vxorpd		%ymm15, %ymm3, %ymm3

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
// r10d   <- k
// r11   <- A
// r12   <- B
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

//
// output arguments:
// r10d  <- 0
// r11   <- A+4*k*sizeof(double)
// r12   <- B+4*k*sizeof(double)
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_dgemm_nt_4x4_lib4, @function
inner_kernel_dgemm_nt_4x4_lib4:
#elif defined(OS_MAC)
_inner_kernel_dgemm_nt_4x4_lib4:
#elif defined(OS_WINDOWS)
	.def inner_kernel_dgemm_nt_4x4_lib4; .scl 2; .type 32; .endef
inner_kernel_dgemm_nt_4x4_lib4:
#endif
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	// prefetch

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop
	
	// unroll 0
	vmovapd 		0(%r11), %xmm8 // A[0]
	vmovapd 		16(%r11), %xmm9 // A[2]

	vmovddup		0(%r12), %xmm12 // B[0]
	vfmadd231pd		%xmm8, %xmm12, %xmm0
	vfmadd231pd		%xmm9, %xmm12, %xmm1

	vmovddup		8(%r12), %xmm12 // B[1]
	vfmadd231pd		%xmm8, %xmm12, %xmm2
	vfmadd231pd		%xmm9, %xmm12, %xmm3

	vmovddup		16(%r12), %xmm12 // B[2]
	vfmadd231pd		%xmm8, %xmm12, %xmm4
	vfmadd231pd		%xmm9, %xmm12, %xmm5

	vmovddup		24(%r12), %xmm12 // B[3]
	vfmadd231pd		%xmm8, %xmm12, %xmm6
	vfmadd231pd		%xmm9, %xmm12, %xmm7


	subl	$ 4, %r10d


	// unroll 1
	vmovapd 		32(%r11), %xmm8 // A[4]
	vmovapd 		48(%r11), %xmm9 // A[6]

	vmovddup		32(%r12), %xmm12 // B[4]
	vfmadd231pd		%xmm8, %xmm12, %xmm0
	vfmadd231pd		%xmm9, %xmm12, %xmm1

	vmovddup		40(%r12), %xmm12 // B[5]
	vfmadd231pd		%xmm8, %xmm12, %xmm2
	vfmadd231pd		%xmm9, %xmm12, %xmm3

	vmovddup		48(%r12), %xmm12 // B[6]
	vfmadd231pd		%xmm8, %xmm12, %xmm4
	vfmadd231pd		%xmm9, %xmm12, %xmm5

	vmovddup		56(%r12), %xmm12 // B[7]
	vfmadd231pd		%xmm8, %xmm12, %xmm6
	vfmadd231pd		%xmm9, %xmm12, %xmm7


	// unroll 2
	vmovapd 		64(%r11), %xmm8 // A[8]
	vmovapd 		80(%r11), %xmm9 // A[10]

	vmovddup		64(%r12), %xmm12 // B[8]
	vfmadd231pd		%xmm8, %xmm12, %xmm0
	vfmadd231pd		%xmm9, %xmm12, %xmm1

	vmovddup		72(%r12), %xmm12 // B[9]
	vfmadd231pd		%xmm8, %xmm12, %xmm2
	vfmadd231pd		%xmm9, %xmm12, %xmm3

	vmovddup		80(%r12), %xmm12 // B[10]
	vfmadd231pd		%xmm8, %xmm12, %xmm4
	vfmadd231pd		%xmm9, %xmm12, %xmm5

	vmovddup		88(%r12), %xmm12 // B[11]
	vfmadd231pd		%xmm8, %xmm12, %xmm6
	vfmadd231pd		%xmm9, %xmm12, %xmm7


	// unroll 3
	vmovapd 		96(%r11), %xmm8 // A[12]
	vmovapd 		112(%r11), %xmm9 // A[14]

	vmovddup		96(%r12), %xmm12 // B[12]
	vfmadd231pd		%xmm8, %xmm12, %xmm0
	vfmadd231pd		%xmm9, %xmm12, %xmm1

	vmovddup		104(%r12), %xmm12 // B[13]
	vfmadd231pd		%xmm8, %xmm12, %xmm2
	vfmadd231pd		%xmm9, %xmm12, %xmm3

	vmovddup		112(%r12), %xmm12 // B[14]
	vfmadd231pd		%xmm8, %xmm12, %xmm4
	vfmadd231pd		%xmm9, %xmm12, %xmm5

	vmovddup		120(%r12), %xmm12 // B[15]
	vfmadd231pd		%xmm8, %xmm12, %xmm6
	vfmadd231pd		%xmm9, %xmm12, %xmm7


	addq	$ 128, %r11
	addq	$ 128, %r12


	cmpl	$ 4, %r10d


	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1


	// unroll 0
	vmovapd 		0(%r11), %xmm8 // A[0]
	vmovapd 		16(%r11), %xmm9 // A[2]

	vmovddup		0(%r12), %xmm12 // B[0]
	vfmadd231pd		%xmm8, %xmm12, %xmm0
	vfmadd231pd		%xmm9, %xmm12, %xmm1

	vmovddup		8(%r12), %xmm12 // B[1]
	vfmadd231pd		%xmm8, %xmm12, %xmm2
	vfmadd231pd		%xmm9, %xmm12, %xmm3

	vmovddup		16(%r12), %xmm12 // B[2]
	vfmadd231pd		%xmm8, %xmm12, %xmm4
	vfmadd231pd		%xmm9, %xmm12, %xmm5

	vmovddup		24(%r12), %xmm12 // B[3]
	vfmadd231pd		%xmm8, %xmm12, %xmm6
	vfmadd231pd		%xmm9, %xmm12, %xmm7


	// unroll 1
	vmovapd 		32(%r11), %xmm8 // A[4]
	vmovapd 		48(%r11), %xmm9 // A[6]

	vmovddup		32(%r12), %xmm12 // B[4]
	vfmadd231pd		%xmm8, %xmm12, %xmm0
	vfmadd231pd		%xmm9, %xmm12, %xmm1

	vmovddup		40(%r12), %xmm12 // B[5]
	vfmadd231pd		%xmm8, %xmm12, %xmm2
	vfmadd231pd		%xmm9, %xmm12, %xmm3

	vmovddup		48(%r12), %xmm12 // B[6]
	vfmadd231pd		%xmm8, %xmm12, %xmm4
	vfmadd231pd		%xmm9, %xmm12, %xmm5

	vmovddup		56(%r12), %xmm12 // B[7]
	vfmadd231pd		%xmm8, %xmm12, %xmm6
	vfmadd231pd		%xmm9, %xmm12, %xmm7


	// unroll 2
	vmovapd 		64(%r11), %xmm8 // A[8]
	vmovapd 		80(%r11), %xmm9 // A[10]

	vmovddup		64(%r12), %xmm12 // B[8]
	vfmadd231pd		%xmm8, %xmm12, %xmm0
	vfmadd231pd		%xmm9, %xmm12, %xmm1

	vmovddup		72(%r12), %xmm12 // B[9]
	vfmadd231pd		%xmm8, %xmm12, %xmm2
	vfmadd231pd		%xmm9, %xmm12, %xmm3

	vmovddup		80(%r12), %xmm12 // B[10]
	vfmadd231pd		%xmm8, %xmm12, %xmm4
	vfmadd231pd		%xmm9, %xmm12, %xmm5

	vmovddup		88(%r12), %xmm12 // B[11]
	vfmadd231pd		%xmm8, %xmm12, %xmm6
	vfmadd231pd		%xmm9, %xmm12, %xmm7


	// unroll 3
	vmovapd 		96(%r11), %xmm8 // A[12]
	vmovapd 		112(%r11), %xmm9 // A[14]

	vmovddup		96(%r12), %xmm12 // B[12]
	vfmadd231pd		%xmm8, %xmm12, %xmm0
	vfmadd231pd		%xmm9, %xmm12, %xmm1

	vmovddup		104(%r12), %xmm12 // B[13]
	vfmadd231pd		%xmm8, %xmm12, %xmm2
	vfmadd231pd		%xmm9, %xmm12, %xmm3

	vmovddup		112(%r12), %xmm12 // B[14]
	vfmadd231pd		%xmm8, %xmm12, %xmm4
	vfmadd231pd		%xmm9, %xmm12, %xmm5

	vmovddup		120(%r12), %xmm12 // B[15]
	vfmadd231pd		%xmm8, %xmm12, %xmm6
	vfmadd231pd		%xmm9, %xmm12, %xmm7


	addq	$ 128, %r12
	addq	$ 128, %r11


	jmp		2f


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovapd 		0(%r11), %xmm8 // A[0]
	vmovapd 		16(%r11), %xmm9 // A[2]

	vmovddup		0(%r12), %xmm12 // B[0]
	vfmadd231pd		%xmm8, %xmm12, %xmm0
	vfmadd231pd		%xmm9, %xmm12, %xmm1

	subl	$ 1, %r10d

	vmovddup		8(%r12), %xmm12 // B[1]
	vfmadd231pd		%xmm8, %xmm12, %xmm2
	vfmadd231pd		%xmm9, %xmm12, %xmm3

	vmovddup		16(%r12), %xmm12 // B[2]
	vfmadd231pd		%xmm8, %xmm12, %xmm4
	vfmadd231pd		%xmm9, %xmm12, %xmm5

	vmovddup		24(%r12), %xmm12 // B[3]
	vfmadd231pd		%xmm8, %xmm12, %xmm6
	vfmadd231pd	%xmm9, %xmm12, %xmm7

	addq	$ 32, %r11
	addq	$ 32, %r12

	cmpl	$ 0, %r10d

	jg		3b // clean up loop 


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_dgemm_nt_4x4_lib4, .-inner_kernel_dgemm_nt_4x4_lib4
#endif
#endif





// common inner routine with file scope
//
// input arguments:
// r10d   <- k
// r11   <- A
// r12   <- B
// r13   <- sdb*32
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

//
// output arguments:
// r10d  <- 0
// r11   <- A+4*k*sizeof(double)
// r12   <- B+4*k*sizeof(double)
// r13   <- sdb*32
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_4X4_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_dgemm_nn_4x4_lib4, @function
inner_kernel_dgemm_nn_4x4_lib4:
#elif defined(OS_MAC)
_inner_kernel_dgemm_nn_4x4_lib4:
#elif defined(OS_WINDOWS)
	.def inner_kernel_dgemm_nn_4x4_lib4; .scl 2; .type 32; .endef
inner_kernel_dgemm_nn_4x4_lib4:
#endif
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	// prefetch

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop
	
	// unroll 0
	vmovapd 		0(%r11), %xmm8 // A[0]
	vmovapd 		16(%r11), %xmm9 // A[2]

	vmovddup		0(%r12), %xmm12 // B[0]
	vfmadd231pd		%xmm8, %xmm12, %xmm0
	vfmadd231pd		%xmm9, %xmm12, %xmm1

	vmovddup		32(%r12), %xmm12 // B[1]
	vfmadd231pd		%xmm8, %xmm12, %xmm2
	vfmadd231pd		%xmm9, %xmm12, %xmm3

	vmovddup		64(%r12), %xmm12 // B[2]
	vfmadd231pd		%xmm8, %xmm12, %xmm4
	vfmadd231pd		%xmm9, %xmm12, %xmm5

	vmovddup		96(%r12), %xmm12 // B[3]
	vfmadd231pd		%xmm8, %xmm12, %xmm6
	vfmadd231pd		%xmm9, %xmm12, %xmm7


	subl	$ 4, %r10d


	// unroll 1
	vmovapd 		32(%r11), %xmm8 // A[4]
	vmovapd 		48(%r11), %xmm9 // A[6]

	vmovddup		8(%r12), %xmm12 // B[4]
	vfmadd231pd		%xmm8, %xmm12, %xmm0
	vfmadd231pd		%xmm9, %xmm12, %xmm1

	vmovddup		40(%r12), %xmm12 // B[5]
	vfmadd231pd		%xmm8, %xmm12, %xmm2
	vfmadd231pd		%xmm9, %xmm12, %xmm3

	vmovddup		72(%r12), %xmm12 // B[6]
	vfmadd231pd		%xmm8, %xmm12, %xmm4
	vfmadd231pd		%xmm9, %xmm12, %xmm5

	vmovddup		104(%r12), %xmm12 // B[7]
	vfmadd231pd		%xmm8, %xmm12, %xmm6
	vfmadd231pd		%xmm9, %xmm12, %xmm7


	// unroll 2
	vmovapd 		64(%r11), %xmm8 // A[8]
	vmovapd 		80(%r11), %xmm9 // A[10]

	vmovddup		16(%r12), %xmm12 // B[8]
	vfmadd231pd		%xmm8, %xmm12, %xmm0
	vfmadd231pd		%xmm9, %xmm12, %xmm1

	vmovddup		48(%r12), %xmm12 // B[9]
	vfmadd231pd		%xmm8, %xmm12, %xmm2
	vfmadd231pd		%xmm9, %xmm12, %xmm3

	vmovddup		80(%r12), %xmm12 // B[10]
	vfmadd231pd		%xmm8, %xmm12, %xmm4
	vfmadd231pd		%xmm9, %xmm12, %xmm5

	vmovddup		112(%r12), %xmm12 // B[11]
	vfmadd231pd		%xmm8, %xmm12, %xmm6
	vfmadd231pd		%xmm9, %xmm12, %xmm7


	// unroll 3
	vmovapd 		96(%r11), %xmm8 // A[12]
	vmovapd 		112(%r11), %xmm9 // A[14]

	vmovddup		24(%r12), %xmm12 // B[12]
	vfmadd231pd		%xmm8, %xmm12, %xmm0
	vfmadd231pd		%xmm9, %xmm12, %xmm1

	vmovddup		56(%r12), %xmm12 // B[13]
	vfmadd231pd		%xmm8, %xmm12, %xmm2
	vfmadd231pd		%xmm9, %xmm12, %xmm3

	vmovddup		88(%r12), %xmm12 // B[14]
	vfmadd231pd		%xmm8, %xmm12, %xmm4
	vfmadd231pd		%xmm9, %xmm12, %xmm5

	vmovddup		120(%r12), %xmm12 // B[15]
	vfmadd231pd		%xmm8, %xmm12, %xmm6
	vfmadd231pd		%xmm9, %xmm12, %xmm7


	addq	$ 128, %r11
	addq	%r13, %r12


	cmpl	$ 4, %r10d


	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1


	// unroll 0
	vmovapd 		0(%r11), %xmm8 // A[0]
	vmovapd 		16(%r11), %xmm9 // A[2]

	vmovddup		0(%r12), %xmm12 // B[0]
	vfmadd231pd		%xmm8, %xmm12, %xmm0
	vfmadd231pd		%xmm9, %xmm12, %xmm1

	vmovddup		32(%r12), %xmm12 // B[1]
	vfmadd231pd		%xmm8, %xmm12, %xmm2
	vfmadd231pd		%xmm9, %xmm12, %xmm3

	vmovddup		64(%r12), %xmm12 // B[2]
	vfmadd231pd		%xmm8, %xmm12, %xmm4
	vfmadd231pd		%xmm9, %xmm12, %xmm5

	vmovddup		96(%r12), %xmm12 // B[3]
	vfmadd231pd		%xmm8, %xmm12, %xmm6
	vfmadd231pd		%xmm9, %xmm12, %xmm7


	// unroll 1
	vmovapd 		32(%r11), %xmm8 // A[4]
	vmovapd 		48(%r11), %xmm9 // A[6]

	vmovddup		8(%r12), %xmm12 // B[4]
	vfmadd231pd		%xmm8, %xmm12, %xmm0
	vfmadd231pd		%xmm9, %xmm12, %xmm1

	vmovddup		40(%r12), %xmm12 // B[5]
	vfmadd231pd		%xmm8, %xmm12, %xmm2
	vfmadd231pd		%xmm9, %xmm12, %xmm3

	vmovddup		72(%r12), %xmm12 // B[6]
	vfmadd231pd		%xmm8, %xmm12, %xmm4
	vfmadd231pd		%xmm9, %xmm12, %xmm5

	vmovddup		104(%r12), %xmm12 // B[7]
	vfmadd231pd		%xmm8, %xmm12, %xmm6
	vfmadd231pd		%xmm9, %xmm12, %xmm7


	// unroll 2
	vmovapd 		64(%r11), %xmm8 // A[8]
	vmovapd 		80(%r11), %xmm9 // A[10]

	vmovddup		16(%r12), %xmm12 // B[8]
	vfmadd231pd		%xmm8, %xmm12, %xmm0
	vfmadd231pd		%xmm9, %xmm12, %xmm1

	vmovddup		48(%r12), %xmm12 // B[9]
	vfmadd231pd		%xmm8, %xmm12, %xmm2
	vfmadd231pd		%xmm9, %xmm12, %xmm3

	vmovddup		80(%r12), %xmm12 // B[10]
	vfmadd231pd		%xmm8, %xmm12, %xmm4
	vfmadd231pd		%xmm9, %xmm12, %xmm5

	vmovddup		112(%r12), %xmm12 // B[11]
	vfmadd231pd		%xmm8, %xmm12, %xmm6
	vfmadd231pd		%xmm9, %xmm12, %xmm7


	// unroll 3
	vmovapd 		96(%r11), %xmm8 // A[12]
	vmovapd 		112(%r11), %xmm9 // A[14]

	vmovddup		24(%r12), %xmm12 // B[12]
	vfmadd231pd		%xmm8, %xmm12, %xmm0
	vfmadd231pd		%xmm9, %xmm12, %xmm1

	vmovddup		56(%r12), %xmm12 // B[13]
	vfmadd231pd		%xmm8, %xmm12, %xmm2
	vfmadd231pd		%xmm9, %xmm12, %xmm3

	vmovddup		88(%r12), %xmm12 // B[14]
	vfmadd231pd		%xmm8, %xmm12, %xmm4
	vfmadd231pd		%xmm9, %xmm12, %xmm5

	vmovddup		120(%r12), %xmm12 // B[15]
	vfmadd231pd		%xmm8, %xmm12, %xmm6
	vfmadd231pd		%xmm9, %xmm12, %xmm7


	addq	%r13, %r12
	addq	$ 128, %r11


	jmp		2f


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovapd 		0(%r11), %xmm8 // A[0]
	vmovapd 		16(%r11), %xmm9 // A[2]

	vmovddup		0(%r12), %xmm12 // B[0]
	vfmadd231pd		%xmm8, %xmm12, %xmm0
	vfmadd231pd		%xmm9, %xmm12, %xmm1

	subl	$ 1, %r10d

	vmovddup		32(%r12), %xmm12 // B[1]
	vfmadd231pd		%xmm8, %xmm12, %xmm2
	vfmadd231pd		%xmm9, %xmm12, %xmm3

	vmovddup		64(%r12), %xmm12 // B[2]
	vfmadd231pd		%xmm8, %xmm12, %xmm4
	vfmadd231pd		%xmm9, %xmm12, %xmm5

	vmovddup		96(%r12), %xmm12 // B[3]
	vfmadd231pd		%xmm8, %xmm12, %xmm6
	vfmadd231pd		%xmm9, %xmm12, %xmm7

	addq	$ 32, %r11
	addq	$ 8, %r12

	cmpl	$ 0, %r10d

	jg		3b // clean up loop 


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_dgemm_nn_4x4_lib4, .-inner_kernel_dgemm_nn_4x4_lib4
#endif
#endif





// common inner routine with file scope
//
// edge for B unaligned, A aligned
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
	.macro INNER_EDGE_DGEMM_NN_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dgemm_nn_4x4_lib4)
#endif

	cmpl			$ 0, %r14d				// offset==0
	jle				2f						// end

	cmpl			$ 0, %r10d				// k==0
	jle				2f						// end

	movl			$ 4, %r15d				// load 4
	subl			%r14d, %r15d			// 4-offsetB
	cmpl			%r10d, %r15d			// k > 4-offsetB
	cmovgl			%r10d, %r15d			// kend=min(k,4-offsetB)

	movl			%r14d, %eax				// load offsetB
	sall			$ 3, %eax				// offsetB*sizeof(double)
	addq			%rax, %r12				// B+offsetB*sizeof(double)

1:
	vmovapd 		0(%r11), %xmm8 // A[0]
	vmovapd 		16(%r11), %xmm9 // A[2]

	vmovddup		0(%r12), %xmm12 // B[0]
	vfmadd231pd		%xmm8, %xmm12, %xmm0
	vfmadd231pd		%xmm9, %xmm12, %xmm1

	vmovddup		32(%r12), %xmm12 // B[1]
	vfmadd231pd		%xmm8, %xmm12, %xmm2
	vfmadd231pd		%xmm9, %xmm12, %xmm3

	vmovddup		64(%r12), %xmm12 // B[2]
	vfmadd231pd		%xmm8, %xmm12, %xmm4
	vfmadd231pd		%xmm9, %xmm12, %xmm5

	vmovddup		96(%r12), %xmm12 // B[3]
	vfmadd231pd		%xmm8, %xmm12, %xmm6
	vfmadd231pd		%xmm9, %xmm12, %xmm7

	subl			$ 1, %r10d				// k=-1
	subl			$ 1, %r15d				// k_panel=-1
	addq			$ 32, %r11				// A=+bs
	addq			$ 8, %r12				// B=+1

	cmpl			$ 0, %r15d				// if k_panel=0
	jg				1b						// loop 1

	cmpl			$ 0, %r10d				// if k=0
	jle				2f						// end

	addq			%r13, %r12				// B=Boff+sdb*bs
	subq			$ 32, %r12				// B-=4*sizeof(double) (loop+offsetB)

2:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dgemm_nn_4x4_lib4)
#endif





// common inner routine with file scope
//
// edge for B upper triangular
//
// input arguments:
// r10   <- A
// r11   <- B
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

//
// output arguments:
// r10   <- A+4*4*sizeof(double)
// r11   <- B+4*4*sizeof(double)
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NT_RU_4X4_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_edge_dtrmm_nt_ru_4x4_lib4, @function
inner_edge_dtrmm_nt_ru_4x4_lib4:
#elif defined(OS_MAC)
_inner_edge_dtrmm_nt_ru_4x4_lib4:
#elif defined(OS_WINDOWS)
	.def inner_edge_dtrmm_nt_ru_4x4_lib4; .scl 2; .type 32; .endef
inner_edge_dtrmm_nt_ru_4x4_lib4:
#endif
#endif
	
	vmovapd			0(%r10), %xmm8
	vmovapd			16(%r10), %xmm9
	vmovddup		0(%r11), %xmm12
	vfmadd231pd		%xmm8, %xmm12, %xmm0
	vfmadd231pd		%xmm9, %xmm12, %xmm1

	vmovapd			32(%r10), %xmm8
	vmovapd			48(%r10), %xmm9
	vmovddup		32(%r11), %xmm12
	vfmadd231pd		%xmm8, %xmm12, %xmm0
	vfmadd231pd		%xmm9, %xmm12, %xmm1
	vmovddup		40(%r11), %xmm12
	vfmadd231pd		%xmm8, %xmm12, %xmm2
	vfmadd231pd		%xmm9, %xmm12, %xmm3

	vmovapd			64(%r10), %xmm8
	vmovapd			80(%r10), %xmm9
	vmovddup		64(%r11), %xmm12
	vfmadd231pd		%xmm8, %xmm12, %xmm0
	vfmadd231pd		%xmm9, %xmm12, %xmm1
	vmovddup		72(%r11), %xmm12
	vfmadd231pd		%xmm8, %xmm12, %xmm2
	vfmadd231pd		%xmm9, %xmm12, %xmm3
	vmovddup		80(%r11), %xmm12
	vfmadd231pd		%xmm8, %xmm12, %xmm4
	vfmadd231pd		%xmm9, %xmm12, %xmm5

	vmovapd			96(%r10), %xmm8
	vmovapd			112(%r10), %xmm9
	vmovddup		96(%r11), %xmm12
	vfmadd231pd		%xmm8, %xmm12, %xmm0
	vfmadd231pd		%xmm9, %xmm12, %xmm1
	vmovddup		104(%r11), %xmm12
	vfmadd231pd		%xmm8, %xmm12, %xmm2
	vfmadd231pd		%xmm9, %xmm12, %xmm3
	vmovddup		112(%r11), %xmm12
	vfmadd231pd		%xmm8, %xmm12, %xmm4
	vfmadd231pd		%xmm9, %xmm12, %xmm5
	vmovddup		120(%r11), %xmm12
	vfmadd231pd		%xmm8, %xmm12, %xmm6
	vfmadd231pd		%xmm9, %xmm12, %xmm7

	addq			$ 128, %r10
	addq			$ 128, %r11

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_edge_dtrmm_nt_ru_4x4_lib4, .-inner_edge_dtrmm_nt_ru_4x4_lib4
#endif
#endif





// common inner routine with file scope
//
// edge for B upper triangular
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

//
// output arguments:
// r10d  <- max(k-4,0)
// r11   <- A+4*4*sizeof(double)
// r12   <- B+4*4*sizeof(double)
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NT_RU_4X4_VS_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_edge_dtrmm_nt_ru_4x4_vs_lib4, @function
inner_edge_dtrmm_nt_ru_4x4_vs_lib4:
#elif defined(OS_MAC)
_inner_edge_dtrmm_nt_ru_4x4_vs_lib4:
#elif defined(OS_WINDOWS)
	.def inner_edge_dtrmm_nt_ru_4x4_vs_lib4; .scl 2; .type 32; .endef
inner_edge_dtrmm_nt_ru_4x4_vs_lib4:
#endif
#endif
	
	vmovapd			0(%r11), %xmm8
	vmovapd			16(%r11), %xmm9
	subl			$ 1, %r10d
	vmovddup		0(%r12), %xmm12
	vfmadd231pd		%xmm8, %xmm12, %xmm0
	vfmadd231pd		%xmm9, %xmm12, %xmm1
	addq			$ 32, %r11
	addq			$ 32, %r12

	cmpl	$ 0, %r10d
	jle		0f

	vmovapd			0(%r11), %xmm8
	vmovapd			16(%r11), %xmm9
	subl			$ 1, %r10d
	vmovddup		0(%r12), %xmm12
	vfmadd231pd		%xmm8, %xmm12, %xmm0
	vfmadd231pd		%xmm9, %xmm12, %xmm1
	addq			$ 32, %r11
	vmovddup		8(%r12), %xmm12
	vfmadd231pd		%xmm8, %xmm12, %xmm2
	vfmadd231pd		%xmm9, %xmm12, %xmm3
	addq			$ 32, %r12

	cmpl	$ 0, %r10d
	jle		0f

	vmovapd			0(%r11), %xmm8
	vmovapd			16(%r11), %xmm9
	subl			$ 1, %r10d
	vmovddup		0(%r12), %xmm12
	vfmadd231pd		%xmm8, %xmm12, %xmm0
	vfmadd231pd		%xmm9, %xmm12, %xmm1
	vmovddup		8(%r12), %xmm12
	vfmadd231pd		%xmm8, %xmm12, %xmm2
	vfmadd231pd		%xmm9, %xmm12, %xmm3
	addq			$ 32, %r11
	vmovddup		16(%r12), %xmm12
	vfmadd231pd		%xmm8, %xmm12, %xmm4
	vfmadd231pd		%xmm9, %xmm12, %xmm5
	addq			$ 32, %r12

	cmpl	$ 0, %r10d
	jle		0f

	vmovapd			0(%r11), %xmm8
	vmovapd			16(%r11), %xmm9
	subl			$ 1, %r10d
	vmovddup		0(%r12), %xmm12
	vfmadd231pd		%xmm8, %xmm12, %xmm0
	vfmadd231pd		%xmm9, %xmm12, %xmm1
	vmovddup		8(%r12), %xmm12
	vfmadd231pd		%xmm8, %xmm12, %xmm2
	vfmadd231pd		%xmm9, %xmm12, %xmm3
	vmovddup		16(%r12), %xmm12
	vfmadd231pd		%xmm8, %xmm12, %xmm4
	vfmadd231pd		%xmm9, %xmm12, %xmm5
	addq			$ 32, %r11
	vmovddup		24(%r12), %xmm12
	vfmadd231pd		%xmm8, %xmm12, %xmm6
	vfmadd231pd		%xmm9, %xmm12, %xmm7
	addq			$ 32, %r12

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_edge_dtrmm_nt_ru_4x4_vs_lib4, .-inner_edge_dtrmm_nt_ru_4x4_vs_lib4
#endif
#endif





// common inner routine with file scope
//
// blend scale for generic alpha and beta
//
// input arguments:
// r10   <- C
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

// output arguments:
// r10   <- C
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_4X4_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_scale_ab_4x4_lib4, @function
inner_scale_ab_4x4_lib4:
#elif defined(OS_MAC)
_inner_scale_ab_4x4_lib4:
#elif defined(OS_WINDOWS)
	.def inner_scale_ab_4x4_lib4; .scl 2; .type 32; .endef
inner_scale_ab_4x4_lib4:
#endif
#endif
	
	// alpha
	vmovddup	0(%r10), %xmm15

	vmulpd	%xmm15, %xmm0, %xmm0
	vmulpd	%xmm15, %xmm1, %xmm1
	vmulpd	%xmm15, %xmm2, %xmm2
	vmulpd	%xmm15, %xmm3, %xmm3
	vmulpd	%xmm15, %xmm4, %xmm4
	vmulpd	%xmm15, %xmm5, %xmm5
	vmulpd	%xmm15, %xmm6, %xmm6
	vmulpd	%xmm15, %xmm7, %xmm7


	// beta
	vmovddup	0(%r11), %xmm14

	vxorpd		%xmm15, %xmm15, %xmm15 // 0.0

	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	vmovapd		0(%r12), %xmm15
	vfmadd231pd	%xmm14, %xmm15, %xmm0
	vmovapd		16(%r12), %xmm15
	vfmadd231pd	%xmm14, %xmm15, %xmm1
	vmovapd		32(%r12), %xmm15
	vfmadd231pd	%xmm14, %xmm15, %xmm2
	vmovapd		48(%r12), %xmm15
	vfmadd231pd	%xmm14, %xmm15, %xmm3
	vmovapd		64(%r12), %xmm15
	vfmadd231pd	%xmm14, %xmm15, %xmm4
	vmovapd		80(%r12), %xmm15
	vfmadd231pd	%xmm14, %xmm15, %xmm5
	vmovapd		96(%r12), %xmm15
	vfmadd231pd	%xmm14, %xmm15, %xmm6
	vmovapd		112(%r12), %xmm15
	vfmadd231pd	%xmm14, %xmm15, %xmm7

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_scale_ab_4x4_lib4, .-inner_scale_ab_4x4_lib4
#endif
#endif





// common inner routine with file scope
//
// blender for alpha = 1.0 and beta = 1.0
//
// input arguments:
// r10   <- beta
// r11   <- C
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

// output arguments:
// r10   <- beta
// r11   <- C
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M1B_4X4_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_scale_m1b_4x4_lib4, @function
inner_scale_m1b_4x4_lib4:
#elif defined(OS_MAC)
_inner_scale_m1b_4x4_lib4:
#elif defined(OS_WINDOWS)
	.def inner_scale_m1b_4x4_lib4; .scl 2; .type 32; .endef
inner_scale_m1b_4x4_lib4:
#endif
#endif	
	
	// beta
	vmovddup	0(%r10), %xmm14

	vxorpd		%xmm15, %xmm15, %xmm15 // 0.0

	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	vmovapd		0(%r11), %xmm15
	vfmsub231pd	%xmm14, %xmm15, %xmm0
	vmovapd		16(%r11), %xmm15
	vfmsub231pd	%xmm14, %xmm15, %xmm1
	vmovapd		32(%r11), %xmm15
	vfmsub231pd	%xmm14, %xmm15, %xmm2
	vmovapd		48(%r11), %xmm15
	vfmsub231pd	%xmm14, %xmm15, %xmm3
	vmovapd		64(%r11), %xmm15
	vfmsub231pd	%xmm14, %xmm15, %xmm4
	vmovapd		80(%r11), %xmm15
	vfmsub231pd	%xmm14, %xmm15, %xmm5
	vmovapd		96(%r11), %xmm15
	vfmsub231pd	%xmm14, %xmm15, %xmm6
	vmovapd		112(%r11), %xmm15
	vfmsub231pd	%xmm14, %xmm15, %xmm7

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_scale_m1b_4x4_lib4, .-inner_scale_m1b_4x4_lib4
#endif
#endif





// common inner routine with file scope
//
// blender for alpha = 1.0 and beta = 1.0
//
// input arguments:
// r10   <- C
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

// output arguments:
// r10   <- C
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M11_4X4_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_scale_m11_4x4_lib4, @function
inner_scale_m11_4x4_lib4:
#elif defined(OS_MAC)
_inner_scale_m11_4x4_lib4:
#elif defined(OS_WINDOWS)
	.def inner_scale_m11_4x4_lib4; .scl 2; .type 32; .endef
inner_scale_m11_4x4_lib4:
#endif
#endif	
	
	vmovapd		0(%r10), %xmm15
	vsubpd		%xmm0, %xmm15, %xmm0
	vmovapd		16(%r10), %xmm15
	vsubpd		%xmm1, %xmm15, %xmm1
	vmovapd		32(%r10), %xmm15
	vsubpd		%xmm2, %xmm15, %xmm2
	vmovapd		48(%r10), %xmm15
	vsubpd		%xmm3, %xmm15, %xmm3
	vmovapd		64(%r10), %xmm15
	vsubpd		%xmm4, %xmm15, %xmm4
	vmovapd		80(%r10), %xmm15
	vsubpd		%xmm5, %xmm15, %xmm5
	vmovapd		96(%r10), %xmm15
	vsubpd		%xmm6, %xmm15, %xmm6
	vmovapd		112(%r10), %xmm15
	vsubpd		%xmm7, %xmm15, %xmm7

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_scale_m11_4x4_lib4, .-inner_scale_m11_4x4_lib4
#endif
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta
//
// input arguments:
// r10   <- alpha
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

// output arguments:
// r10   <- alpha
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_A0_4X4_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_scale_a0_4x4_lib4, @function
inner_scale_a0_4x4_lib4:
#elif defined(OS_MAC)
_inner_scale_a0_4x4_lib4:
#elif defined(OS_WINDOWS)
	.def inner_scale_a0_4x4_lib4; .scl 2; .type 32; .endef
inner_scale_a0_4x4_lib4:
#endif
#endif
	
	// alpha
	vmovddup	0(%r10), %xmm15

	vmulpd	%xmm15, %xmm0, %xmm0
	vmulpd	%xmm15, %xmm1, %xmm1
	vmulpd	%xmm15, %xmm2, %xmm2
	vmulpd	%xmm15, %xmm3, %xmm3
	vmulpd	%xmm15, %xmm4, %xmm4
	vmulpd	%xmm15, %xmm5, %xmm5
	vmulpd	%xmm15, %xmm6, %xmm6
	vmulpd	%xmm15, %xmm7, %xmm7

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_scale_a0_4x4_lib4, .-inner_scale_a0_4x4_lib4
#endif
#endif





// common inner routine with file scope
//
// cholesky factorization 
//
// input arguments:
// r10  <- inv_diag_E
// r11d <- kn
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty
//
// output arguments:
// r10  <- inv_diag_E
// r11d <- kn
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DPOTRF_4X4_VS_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_edge_dpotrf_4x4_vs_lib4, @function
inner_edge_dpotrf_4x4_vs_lib4:
#elif defined(OS_MAC)
_inner_edge_dpotrf_4x4_vs_lib4:
#elif defined(OS_WINDOWS)
	.def inner_edge_dpotrf_4x4_vs_lib4; .scl 2; .type 32; .endef
inner_edge_dpotrf_4x4_vs_lib4:
#endif
#endif
	
	vxorpd			%xmm15, %xmm15, %xmm15 // 0.0
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
	cmpl			$ 2, %r11d
	vmovsd			%xmm13, 0(%r10)
	vmovddup		%xmm13, %xmm13
	vmulpd			%xmm0, %xmm13, %xmm0
	vmulpd			%xmm1, %xmm13, %xmm1

	jl				0f // ret

	vpermilpd		$ 0x3, %xmm0, %xmm13
	vfnmadd231pd	%xmm0, %xmm13, %xmm2
	vfnmadd231pd	%xmm1, %xmm13, %xmm3
	vpermilpd		$ 0x3, %xmm2, %xmm13
	vucomisd		%xmm15, %xmm13 // d_11 > 0.0 ?
	jbe				3f
	vsqrtsd			%xmm13, %xmm13, %xmm13
	vdivsd			%xmm13, %xmm14, %xmm13
4:
	cmpl			$ 3, %r11d
	vmovsd			%xmm13, 8(%r10)
	vmovddup		%xmm13, %xmm13
	vmulpd			%xmm2, %xmm13, %xmm2
	vmulpd			%xmm3, %xmm13, %xmm3

	jl				0f // ret

	vpermilpd		$ 0x0, %xmm1, %xmm13
//	vfnmadd231pd	%xmm0, %xmm13, %xmm4
	vfnmadd231pd	%xmm1, %xmm13, %xmm5
	vpermilpd		$ 0x0, %xmm3, %xmm13
//	vfnmadd231pd	%xmm2, %xmm13, %xmm4
	vfnmadd231pd	%xmm3, %xmm13, %xmm5
	vmovaps			%xmm5, %xmm13
	vucomisd		%xmm15, %xmm13 // d_22 > 0.0 ?
	jbe				5f
	vsqrtsd			%xmm13, %xmm13, %xmm13
	vdivsd			%xmm13, %xmm14, %xmm13
6:
	cmpl			$ 4, %r11d
	vmovsd			%xmm13, 16(%r10)
	vmovddup		%xmm13, %xmm13
//	vmulpd			%xmm4, %xmm13, %xmm4
	vmulpd			%xmm5, %xmm13, %xmm5

	jl				0f // ret

	vpermilpd		$ 0x3, %xmm1, %xmm13
//	vfnmadd231pd	%xmm0, %xmm13, %xmm6
	vfnmadd231pd	%xmm1, %xmm13, %xmm7
	vpermilpd		$ 0x3, %xmm3, %xmm13
//	vfnmadd231pd	%xmm2, %xmm13, %xmm6
	vfnmadd231pd	%xmm3, %xmm13, %xmm7
	vpermilpd		$ 0x3, %xmm5, %xmm13
//	vfnmadd231pd	%xmm4, %xmm13, %xmm6
	vfnmadd231pd	%xmm5, %xmm13, %xmm7
	vpermilpd		$ 0x3, %xmm7, %xmm13
	vucomisd		%xmm15, %xmm13 // d_33 > 0.0 ?
	jbe				7f
	vsqrtsd			%xmm13, %xmm13, %xmm13
	vdivsd			%xmm13, %xmm14, %xmm13
8:
	vmovsd			%xmm13, 24(%r10)
	vmovddup		%xmm13, %xmm13
//	vmulpd			%xmm6, %xmm13, %xmm6
	vmulpd			%xmm7, %xmm13, %xmm7

	jmp		0f
	
1:
	vxorpd	%xmm13, %xmm13, %xmm13
	jmp		2b

3:
	vxorpd	%xmm13, %xmm13, %xmm13
	jmp		4b

5:
	vxorpd	%xmm13, %xmm13, %xmm13
	jmp		6b

7:
	vxorpd	%xmm13, %xmm13, %xmm13
	jmp		8b

0:
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_edge_dpotrf_4x4_vs_lib4, .-inner_edge_dpotrf_4x4_vs_lib4
#endif
#endif





// common inner routine with file scope
//
// triangular substitution for cholesky factorization 
//
// input arguments:
// r10  <- E
// r11  <- inv_diag_E
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty
//
// output arguments:
// r10  <- E
// r11  <- inv_diag_E
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RLT_INV_4X4_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_edge_dtrsm_rlt_inv_4x4_lib4, @function
inner_edge_dtrsm_rlt_inv_4x4_lib4:
#elif defined(OS_MAC)
_inner_edge_dtrsm_rlt_inv_4x4_lib4:
#elif defined(OS_WINDOWS)
	.def inner_edge_dtrsm_rlt_inv_4x4_lib4; .scl 2; .type 32; .endef
inner_edge_dtrsm_rlt_inv_4x4_lib4:
#endif
#endif
	
	vmovddup		0(%r11), %xmm13
	vmulpd			%xmm0, %xmm13, %xmm0
	vmulpd			%xmm1, %xmm13, %xmm1

	vmovddup		8(%r10), %xmm13
	vfnmadd231pd	%xmm0, %xmm13, %xmm2
	vfnmadd231pd	%xmm1, %xmm13, %xmm3
	vmovddup		8(%r11), %xmm13
	vmulpd			%xmm2, %xmm13, %xmm2
	vmulpd			%xmm3, %xmm13, %xmm3

	vmovddup		16(%r10), %xmm13
	vfnmadd231pd	%xmm0, %xmm13, %xmm4
	vfnmadd231pd	%xmm1, %xmm13, %xmm5
	vmovddup		48(%r10), %xmm13
	vfnmadd231pd	%xmm2, %xmm13, %xmm4
	vfnmadd231pd	%xmm3, %xmm13, %xmm5
	vmovddup		16(%r11), %xmm13
	vmulpd			%xmm4, %xmm13, %xmm4
	vmulpd			%xmm5, %xmm13, %xmm5

	vmovddup		24(%r10), %xmm13
	vfnmadd231pd	%xmm0, %xmm13, %xmm6
	vfnmadd231pd	%xmm1, %xmm13, %xmm7
	vmovddup		56(%r10), %xmm13
	vfnmadd231pd	%xmm2, %xmm13, %xmm6
	vfnmadd231pd	%xmm3, %xmm13, %xmm7
	vmovddup		88(%r10), %xmm13
	vfnmadd231pd	%xmm4, %xmm13, %xmm6
	vfnmadd231pd	%xmm5, %xmm13, %xmm7
	vmovddup		24(%r11), %xmm13
	vmulpd			%xmm6, %xmm13, %xmm6
	vmulpd			%xmm7, %xmm13, %xmm7

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_edge_dtrsm_rlt_inv_4x4_lib4, .-inner_edge_dtrsm_rlt_inv_4x4_lib4
#endif
#endif





// common inner routine with file scope
//
// triangular substitution for cholesky factorization 
//
// input arguments:
// r10  <- D
// r11  <- inv_diag_D
// r12d <- kn
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty
//
// output arguments:
// r10  <- D
// r11  <- inv_diag_D
// r12d <- kn
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RLT_INV_4X4_VS_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_edge_dtrsm_rlt_inv_4x4_vs_lib4, @function
inner_edge_dtrsm_rlt_inv_4x4_vs_lib4:
#elif defined(OS_MAC)
_inner_edge_dtrsm_rlt_inv_4x4_vs_lib4:
#elif defined(OS_WINDOWS)
	.def inner_edge_dtrsm_rlt_inv_4x4_vs_lib4; .scl 2; .type 32; .endef
inner_edge_dtrsm_rlt_inv_4x4_vs_lib4:
#endif
#endif
	
	vmovddup		0(%r11), %xmm13
	cmpl			$ 2, %r12d
	vmulpd			%xmm0, %xmm13, %xmm0
	vmulpd			%xmm1, %xmm13, %xmm1

	jl				0f // ret

	vmovddup		8(%r10), %xmm13
	cmpl			$ 3, %r12d
	vfnmadd231pd	%xmm0, %xmm13, %xmm2
	vfnmadd231pd	%xmm1, %xmm13, %xmm3
	vmovddup		8(%r11), %xmm13
	vmulpd			%xmm2, %xmm13, %xmm2
	vmulpd			%xmm3, %xmm13, %xmm3

	jl				0f // ret

	vmovddup		16(%r10), %xmm13
	cmpl			$ 4, %r12d
	vfnmadd231pd	%xmm0, %xmm13, %xmm4
	vfnmadd231pd	%xmm1, %xmm13, %xmm5
	vmovddup		48(%r10), %xmm13
	vfnmadd231pd	%xmm2, %xmm13, %xmm4
	vfnmadd231pd	%xmm3, %xmm13, %xmm5
	vmovddup		16(%r11), %xmm13
	vmulpd			%xmm4, %xmm13, %xmm4
	vmulpd			%xmm5, %xmm13, %xmm5

	jl				0f // ret

	vmovddup		24(%r10), %xmm13
	vfnmadd231pd	%xmm0, %xmm13, %xmm6
	vfnmadd231pd	%xmm1, %xmm13, %xmm7
	vmovddup		56(%r10), %xmm13
	vfnmadd231pd	%xmm2, %xmm13, %xmm6
	vfnmadd231pd	%xmm3, %xmm13, %xmm7
	vmovddup		88(%r10), %xmm13
	vfnmadd231pd	%xmm4, %xmm13, %xmm6
	vfnmadd231pd	%xmm5, %xmm13, %xmm7
	vmovddup		24(%r11), %xmm13
	vmulpd			%xmm6, %xmm13, %xmm6
	vmulpd			%xmm7, %xmm13, %xmm7

0:
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_edge_dtrsm_rlt_inv_4x4_vs_lib4, .-inner_edge_dtrsm_rlt_inv_4x4_vs_lib4
#endif
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty
//
// output arguments:
// r10  <- D
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X4_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_store_4x4_lib4, @function
inner_store_4x4_lib4:
#elif defined(OS_MAC)
_inner_store_4x4_lib4:
#elif defined(OS_WINDOWS)
	.def inner_store_4x4_lib4; .scl 2; .type 32; .endef
inner_store_4x4_lib4:
#endif
#endif
	
	vmovapd %xmm0,   0(%r10)
	vmovapd %xmm1,  16(%r10)
	vmovapd %xmm2,  32(%r10)
	vmovapd %xmm3,  48(%r10)
	vmovapd %xmm4,  64(%r10)
	vmovapd %xmm5,  80(%r10)
	vmovapd %xmm6,  96(%r10)
	vmovapd %xmm7, 112(%r10)
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_store_4x4_lib4, .-inner_store_4x4_lib4
#endif
#endif





// TODO use blendv instead
// common inner routine with file scope
//
// store n vs
//
// input arguments:
// r10   <- D
// r11d   <- km
// r12d   <- kn
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty
//
// output arguments:
// r10   <- D
// r11d   <- km
// r12d   <- kn
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X4_VS_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_store_4x4_vs_lib4, @function
inner_store_4x4_vs_lib4:
#elif defined(OS_MAC)
_inner_store_4x4_vs_lib4:
#elif defined(OS_WINDOWS)
	.def inner_store_4x4_vs_lib4; .scl 2; .type 32; .endef
inner_store_4x4_vs_lib4:
#endif
#endif
	
	cmpl		$ 2, %r11d
	jg			1f
	je			0f

	// km==1
	cmpl		$ 2, %r12d
	vmovsd		%xmm0,  0(%r10)
	jl			4f // end
	cmpl		$ 3, %r12d
	vmovsd		%xmm2, 32(%r10)
	jl			4f // end
	vmovsd		%xmm4, 64(%r10)
	je			4f // end
	vmovsd		%xmm6, 96(%r10)

	jmp		4f

0:
	// km==2
	cmpl		$ 2, %r12d
	vmovapd		%xmm0,  0(%r10)
	jl			4f // end
	cmpl		$ 3, %r12d
	vmovapd		%xmm2, 32(%r10)
	jl			4f // end
	vmovapd		%xmm4, 64(%r10)
	je			4f // end
	vmovapd		%xmm6, 96(%r10)

	jmp		4f

1:
	cmpl		$ 3, %r11d
	jg			2f

	// km==3
	cmpl		$ 2, %r12d
	vmovapd		%xmm0,   0(%r10)
	vmovsd		%xmm1,  16(%r10)
	jl			4f // end
	cmpl		$ 3, %r12d
	vmovapd		%xmm2,  32(%r10)
	vmovsd		%xmm3,  48(%r10)
	jl			4f // end
	vmovapd		%xmm4,  64(%r10)
	vmovsd		%xmm5,  80(%r10)
	je			4f // end
	vmovapd		%xmm6,  96(%r10)
	vmovsd		%xmm7, 112(%r10)

	jmp		4f

2:
	// km==3
	cmpl		$ 2, %r12d
	vmovapd		%xmm0,   0(%r10)
	vmovapd		%xmm1,  16(%r10)
	jl			4f // end
	cmpl		$ 3, %r12d
	vmovapd		%xmm2,  32(%r10)
	vmovapd		%xmm3,  48(%r10)
	jl			4f // end
	vmovapd		%xmm4,  64(%r10)
	vmovapd		%xmm5,  80(%r10)
	je			4f // end
	vmovapd		%xmm6,  96(%r10)
	vmovapd		%xmm7, 112(%r10)

4:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_store_4x4_vs_lib4, .-inner_store_4x4_vs_lib4
#endif
#endif





// common inner routine with file scope
//
// store n lower triangular
//
// input arguments:
// r10   <- D
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty
//
// output arguments:
// r10   <- D
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_4X4_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_store_l_4x4_lib4, @function
inner_store_l_4x4_lib4:
#elif defined(OS_MAC)
_inner_store_l_4x4_lib4:
#elif defined(OS_WINDOWS)
	.def inner_store_l_4x4_lib4; .scl 2; .type 32; .endef
inner_store_l_4x4_lib4:
#endif
#endif
	
	vmovapd		%xmm0,   0(%r10)
	vmovapd		%xmm1,  16(%r10)
	vmovsd		32(%r10), %xmm15
	vmovsd		%xmm15, %xmm2, %xmm2
	vmovapd		%xmm2,  32(%r10)
	vmovapd		%xmm3,  48(%r10)
//	vmovapd		%xmm4,  64(%r10)
	vmovapd		%xmm5,  80(%r10)
//	vmovapd		%xmm6,  96(%r10)
	vmovsd		112(%r10), %xmm15
	vmovsd		%xmm15, %xmm7, %xmm7
	vmovapd		%xmm7, 112(%r10)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_store_l_4x4_lib4, .-inner_store_l_4x4_lib4
#endif
#endif





// common inner routine with file scope
//
// store n vs lower triangular
//
// input arguments:
// r10   <- D
// r11d   <- km
// r12d   <- kn
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty
//
// output arguments:
// r10   <- D
// r11d   <- km
// r12d   <- kn
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
// xmm8  <- dirty
// xmm9  <- dirty
// xmm10 <- dirty
// xmm11 <- dirty
// xmm12 <- dirty
// xmm13 <- dirty
// xmm14 <- dirty
// xmm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_4X4_VS_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_store_l_4x4_vs_lib4, @function
inner_store_l_4x4_vs_lib4:
#elif defined(OS_MAC)
_inner_store_l_4x4_vs_lib4:
#elif defined(OS_WINDOWS)
	.def inner_store_l_4x4_vs_lib4; .scl 2; .type 32; .endef
inner_store_l_4x4_vs_lib4:
#endif
#endif
	
	cmpl		$ 2, %r11d
	jg			1f
	je			0f

	// km==1
	vmovsd		%xmm0,  0(%r10)

	jmp		3f

0:
	// km==2
	cmpl		$ 2, %r12d
	vmovapd		%xmm0,  0(%r10)
	jl			3f // end
	vmovsd		32(%r10), %xmm15
	vmovsd		%xmm15, %xmm2, %xmm2
	vmovapd		%xmm2, 32(%r10)

	jmp		3f

1:
	cmpl		$ 3, %r11d
	jg			2f

	// km==3
	cmpl		$ 2, %r12d
	vmovapd		%xmm0,   0(%r10)
	vmovsd		%xmm1,  16(%r10)
	jl			3f // end
	cmpl		$ 3, %r12d
	vmovsd		32(%r10), %xmm15
	vmovsd		%xmm15, %xmm2, %xmm2
	vmovapd		%xmm2,  32(%r10)
	vmovsd		%xmm3,  48(%r10)
	jl			3f // end
//	vmovapd		%xmm4,  64(%r10)
	vmovsd		%xmm5,  80(%r10)

	jmp		3f

2:
	// km==3
	cmpl		$ 2, %r12d
	vmovapd		%xmm0,   0(%r10)
	vmovapd		%xmm1,  16(%r10)
	jl			3f // end
	cmpl		$ 3, %r12d
	vmovsd		32(%r10), %xmm15
	vmovsd		%xmm15, %xmm2, %xmm2
	vmovapd		%xmm2,  32(%r10)
	vmovapd		%xmm3,  48(%r10)
	jl			3f // end
//	vmovapd		%xmm4,  64(%r10)
	vmovapd		%xmm5,  80(%r10)
	je			3f // end
//	vmovapd		%xmm6,  96(%r10)
	vmovsd		112(%r10), %xmm15
	vmovsd		%xmm15, %xmm7, %xmm7
	vmovapd		%xmm7, 112(%r10)

3:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_store_l_4x4_vs_lib4, .-inner_store_l_4x4_vs_lib4
#endif
#endif





//                               1      2              3          4          5             6          7
// void kernel_dgemm_nt_4x4_lib4(int k, double *alpha, double *A, double *B, double *beta, double *C, double *D);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dgemm_nt_4x4_lib4
	.type kernel_dgemm_nt_4x4_lib4, @function
kernel_dgemm_nt_4x4_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dgemm_nt_4x4_lib4
_kernel_dgemm_nt_4x4_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dgemm_nt_4x4_lib4
	.def kernel_dgemm_nt_4x4_lib4; .scl 2; .type 32; .endef
kernel_dgemm_nt_4x4_lib4:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%xmm0, %xmm0, %xmm0
	vmovapd	%xmm0, %xmm1
	vmovapd	%xmm0, %xmm2
	vmovapd	%xmm0, %xmm3
	vmovapd	%xmm0, %xmm4
	vmovapd	%xmm0, %xmm5
	vmovapd	%xmm0, %xmm6
	vmovapd	%xmm0, %xmm7


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_dgemm_nt_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_kernel_dgemm_nt_4x4_lib4
#endif
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_scale_ab_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_scale_ab_4x4_lib4
#endif
#endif


	// store n

	movq	ARG7, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_store_4x4_lib4
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dgemm_nt_4x4_lib4, .-kernel_dgemm_nt_4x4_lib4
#endif





//                                  rdi    rsi            rdx        rcx        r8            r9         rsp+8     rsp+16   rsp+24
// void kernel_dgemm_nt_4x4_vs_lib4(int k, double *alpha, double *A, double *B, double *beta, double *C, double *D, int km, int kn);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dgemm_nt_4x4_vs_lib4
	.type kernel_dgemm_nt_4x4_vs_lib4, @function
kernel_dgemm_nt_4x4_vs_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dgemm_nt_4x4_vs_lib4
_kernel_dgemm_nt_4x4_vs_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dgemm_nt_4x4_vs_lib4
	.def kernel_dgemm_nt_4x4_vs_lib4; .scl 2; .type 32; .endef
kernel_dgemm_nt_4x4_vs_lib4:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%xmm0, %xmm0, %xmm0
	vmovapd	%xmm0, %xmm1
	vmovapd	%xmm0, %xmm2
	vmovapd	%xmm0, %xmm3
	vmovapd	%xmm0, %xmm4
	vmovapd	%xmm0, %xmm5
	vmovapd	%xmm0, %xmm6
	vmovapd	%xmm0, %xmm7


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_dgemm_nt_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_kernel_dgemm_nt_4x4_lib4
#endif
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_scale_ab_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_scale_ab_4x4_lib4
#endif
#endif


	// store n

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // km 
	movq	ARG9, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_4x4_vs_lib4
#elif defined(OS_MAC)
	callq _inner_store_4x4_vs_lib4
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dgemm_nt_4x4_vs_lib4, .-kernel_dgemm_nt_4x4_vs_lib4
#endif





//                               1      2              3          4            5          6        7             8          9
// void kernel_dgemm_nn_4x4_lib4(int k, double *alpha, double *A, int offsetB, double *B, int sdb, double *beta, double *C, double *D);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dgemm_nn_4x4_lib4
	.type kernel_dgemm_nn_4x4_lib4, @function
kernel_dgemm_nn_4x4_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dgemm_nn_4x4_lib4
_kernel_dgemm_nn_4x4_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dgemm_nn_4x4_lib4
	.def kernel_dgemm_nn_4x4_lib4; .scl 2; .type 32; .endef
kernel_dgemm_nn_4x4_lib4:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%xmm0, %xmm0, %xmm0
	vmovapd	%xmm0, %xmm1
	vmovapd	%xmm0, %xmm2
	vmovapd	%xmm0, %xmm3
	vmovapd	%xmm0, %xmm4
	vmovapd	%xmm0, %xmm5
	vmovapd	%xmm0, %xmm6
	vmovapd	%xmm0, %xmm7


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG5, %r12  // B
	movq	ARG6, %r13 // sdb
	sall	$ 5, %r13d // 4*sdb*sizeof(double)
	movq	ARG4, %r14 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_edge_dgemm_nn_4x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_dgemm_nn_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_kernel_dgemm_nn_4x4_lib4
#endif
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_scale_ab_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_scale_ab_4x4_lib4
#endif
#endif


	// store n

	movq	ARG9, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_store_4x4_lib4
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dgemm_nn_4x4_lib4, .-kernel_dgemm_nn_4x4_lib4
#endif





//                                 rdi    rsi            rdx        rcx        r8            r9         rsp+8
// void kernel_dsyrk_nt_l_4x4_lib4(int k, double *alpha, double *A, double *B, double *beta, double *C, double *D);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dsyrk_nt_l_4x4_lib4
	.type kernel_dsyrk_nt_l_4x4_lib4, @function
kernel_dsyrk_nt_l_4x4_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dsyrk_nt_l_4x4_lib4
_kernel_dsyrk_nt_l_4x4_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dsyrk_nt_l_4x4_lib4
	.def kernel_dsyrk_nt_l_4x4_lib4; .scl 2; .type 32; .endef
kernel_dsyrk_nt_l_4x4_lib4:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%xmm0, %xmm0, %xmm0
	vmovapd	%xmm0, %xmm1
	vmovapd	%xmm0, %xmm2
	vmovapd	%xmm0, %xmm3
	vmovapd	%xmm0, %xmm4
	vmovapd	%xmm0, %xmm5
	vmovapd	%xmm0, %xmm6
	vmovapd	%xmm0, %xmm7


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_dgemm_nt_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_kernel_dgemm_nt_4x4_lib4
#endif
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_scale_ab_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_scale_ab_4x4_lib4
#endif
#endif


	// store n

	movq	ARG7, %r10 // D


#if MACRO_LEVEL>=1
	INNER_STORE_L_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call	inner_store_l_4x4_lib4
#elif defined(OS_MAC)
	callq	_inner_store_l_4x4_lib4
#endif
#endif



	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dsyrk_nt_l_4x4_lib4, .-kernel_dsyrk_nt_l_4x4_lib4
#endif





//                                    rdi    rsi            rdx        rcx        r8            r9         rsp+8     rsp+16   rsp+24
// void kernel_dsyrk_nt_l_4x4_vs_lib4(int k, double *alpha, double *A, double *B, double *beta, double *C, double *D, int km, int kn);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dsyrk_nt_l_4x4_vs_lib4
	.type kernel_dsyrk_nt_l_4x4_vs_lib4, @function
kernel_dsyrk_nt_l_4x4_vs_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dsyrk_nt_l_4x4_vs_lib4
_kernel_dsyrk_nt_l_4x4_vs_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dsyrk_nt_l_4x4_vs_lib4
	.def kernel_dsyrk_nt_l_4x4_vs_lib4; .scl 2; .type 32; .endef
kernel_dsyrk_nt_l_4x4_vs_lib4:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%xmm0, %xmm0, %xmm0
	vmovapd	%xmm0, %xmm1
	vmovapd	%xmm0, %xmm2
	vmovapd	%xmm0, %xmm3
	vmovapd	%xmm0, %xmm4
	vmovapd	%xmm0, %xmm5
	vmovapd	%xmm0, %xmm6
	vmovapd	%xmm0, %xmm7


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_dgemm_nt_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_kernel_dgemm_nt_4x4_lib4
#endif
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_scale_ab_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_scale_ab_4x4_lib4
#endif
#endif


	// store n

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // km 
	movq	ARG9, %r12 // kn 


#if MACRO_LEVEL>=1
	INNER_STORE_L_4X4_VS_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call	inner_store_l_4x4_vs_lib4
#elif defined(OS_MAC)
	callq	_inner_store_l_4x4_vs_lib4
#endif
#endif



	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dsyrk_nt_l_4x4_vs_lib4, .-kernel_dsyrk_nt_l_4x4_vs_lib4
#endif





//                                  1      2              3          4          5
// void kernel_dtrmm_nt_ru_4x4_vs_lib4(int k, double *alpha, double *A, double *B, double *D);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dtrmm_nt_ru_4x4_lib4
	.type kernel_dtrmm_nt_ru_4x4_lib4, @function
kernel_dtrmm_nt_ru_4x4_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dtrmm_nt_ru_4x4_lib4
_kernel_dtrmm_nt_ru_4x4_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dtrmm_nt_ru_4x4_lib4
	.def kernel_dtrmm_nt_ru_4x4_lib4; .scl 2; .type 32; .endef
kernel_dtrmm_nt_ru_4x4_lib4:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%xmm0, %xmm0, %xmm0
	vmovapd	%xmm0, %xmm1
	vmovapd	%xmm0, %xmm2
	vmovapd	%xmm0, %xmm3
	vmovapd	%xmm0, %xmm4
	vmovapd	%xmm0, %xmm5
	vmovapd	%xmm0, %xmm6
	vmovapd	%xmm0, %xmm7


	// call inner dgemm kernel nt after initial triangle

	movq	ARG1, %r10 // k
	subl	$ 4, %r10d // k-4
	movq	ARG3, %r11 // A
	addq	$ 128, %r11 // A+4*bs
	movq	ARG4, %r12 // B
	addq	$ 128, %r12 // B+4*bs

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_dgemm_nt_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_kernel_dgemm_nt_4x4_lib4
#endif
#endif


	// initial triangle

	movq	ARG3, %r10
	movq	ARG4, %r11

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RU_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_dtrmm_nt_ru_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_edge_dtrmm_nt_ru_4x4_lib4
#endif
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha

#if MACRO_LEVEL>=1
	INNER_SCALE_A0_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_scale_a0_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_scale_0b_4x4_lib4
#endif
#endif


	// store n

	movq	ARG5, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_store_4x4_lib4
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dtrmm_nt_ru_4x4_lib4, .-kernel_dtrmm_nt_ru_4x4_lib4
#endif





//                                     1      2              3          4          5          6       7
// void kernel_dtrmm_nt_ru_4x4_vs_lib4(int k, double *alpha, double *A, double *B, double *D, int km, int kn);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dtrmm_nt_ru_4x4_vs_lib4
	.type kernel_dtrmm_nt_ru_4x4_vs_lib4, @function
kernel_dtrmm_nt_ru_4x4_vs_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dtrmm_nt_ru_4x4_vs_lib4
_kernel_dtrmm_nt_ru_4x4_vs_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dtrmm_nt_ru_4x4_vs_lib4
	.def kernel_dtrmm_nt_ru_4x4_vs_lib4; .scl 2; .type 32; .endef
kernel_dtrmm_nt_ru_4x4_vs_lib4:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%xmm0, %xmm0, %xmm0
	vmovapd	%xmm0, %xmm1
	vmovapd	%xmm0, %xmm2
	vmovapd	%xmm0, %xmm3
	vmovapd	%xmm0, %xmm4
	vmovapd	%xmm0, %xmm5
	vmovapd	%xmm0, %xmm6
	vmovapd	%xmm0, %xmm7


	// call inner dgemm kernel nt after initial triangle

	movq	ARG1, %r10 // k
	subl	$ 4, %r10d // k-4
	movq	ARG3, %r11 // A
	addq	$ 128, %r11 // A+4*bs
	movq	ARG4, %r12 // B
	addq	$ 128, %r12 // B+4*bs

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_dgemm_nt_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_kernel_dgemm_nt_4x4_lib4
#endif
#endif


	// initial triangle

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // B

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RU_4X4_VS_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_dtrmm_nt_ru_4x4_vs_lib4
#elif defined(OS_MAC)
	callq _inner_edge_dtrmm_nt_ru_4x4_vs_lib4
#endif
#endif


	// call inner loader nn

	movq	ARG2, %r10 // alpha

#if MACRO_LEVEL>=1
	INNER_SCALE_A0_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_scale_a0_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_scale_a0_4x4_lib4
#endif
#endif


	// store n

	movq	ARG5, %r10 // D
	movq	ARG6, %r11 // km 
	movq	ARG6, %r12   // C
	movq	ARG7, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_4x4_vs_lib4
#elif defined(OS_MAC)
	callq _inner_store_4x4_vs_lib4
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dtrmm_nt_ru_4x4_vs_lib4, .-kernel_dtrmm_nt_ru_4x4_vs_lib4
#endif





//                                  edi    rsi        rdx        ecx        r8         r9
// void kernel_dpotrf_nt_l_4x4_lib4(int k, double *A, double *B, double *C, double *D, double *inv_diag_D);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dpotrf_nt_l_4x4_lib4
	.type kernel_dpotrf_nt_l_4x4_lib4, @function
kernel_dpotrf_nt_l_4x4_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dpotrf_nt_l_4x4_lib4
_kernel_dpotrf_nt_l_4x4_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dpotrf_nt_l_4x4_lib4
	.def kernel_dpotrf_nt_l_4x4_lib4; .scl 2; .type 32; .endef
kernel_dpotrf_nt_l_4x4_lib4:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%xmm0, %xmm0, %xmm0
	vmovapd	%xmm0, %xmm1
	vmovapd	%xmm0, %xmm2
	vmovapd	%xmm0, %xmm3
	vmovapd	%xmm0, %xmm4
	vmovapd	%xmm0, %xmm5
	vmovapd	%xmm0, %xmm6
	vmovapd	%xmm0, %xmm7


	// call inner dgemm kernel nt

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_dgemm_nt_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_kernel_dgemm_nt_4x4_lib4
#endif
#endif


	// call inner blender_loader nn

	movq	ARG4, %r10 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_scale_m11_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_scale_m11_4x4_lib4
#endif
#endif


	// factorization

	movq	ARG6, %r10  // inv_diag_D 
	movl	$ 4, %r11d // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_4X4_VS_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_dpotrf_4x4_vs_lib4
#elif defined(OS_MAC)
	callq _inner_edge_dpotrf_4x4_vs_lib4
#endif
#endif


	// store

	movq	ARG5, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_L_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_l_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_store_l_4x4_lib4
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dpotrf_nt_l_4x4_lib4, .-kernel_dpotrf_nt_l_4x4_lib4
#endif





//                                     edi    rsi        rdx        ecx        r8         r9                  rsp+8   rsp+16
// void kernel_dpotrf_nt_l_4x4_vs_lib4(int k, double *A, double *B, double *C, double *D, double *inv_diag_D, int km, int kn);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dpotrf_nt_l_4x4_vs_lib4
	.type kernel_dpotrf_nt_l_4x4_vs_lib4, @function
kernel_dpotrf_nt_l_4x4_vs_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dpotrf_nt_l_4x4_vs_lib4
_kernel_dpotrf_nt_l_4x4_vs_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dpotrf_nt_l_4x4_vs_lib4
	.def kernel_dpotrf_nt_l_4x4_vs_lib4; .scl 2; .type 32; .endef
kernel_dpotrf_nt_l_4x4_vs_lib4:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%xmm0, %xmm0, %xmm0
	vmovapd	%xmm0, %xmm1
	vmovapd	%xmm0, %xmm2
	vmovapd	%xmm0, %xmm3
	vmovapd	%xmm0, %xmm4
	vmovapd	%xmm0, %xmm5
	vmovapd	%xmm0, %xmm6
	vmovapd	%xmm0, %xmm7


	// call inner dgemm kernel nt

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_dgemm_nt_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_kernel_dgemm_nt_4x4_lib4
#endif
#endif


	// call inner blender_loader nn

	movq	ARG4, %r10 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_scale_m11_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_scale_m11_4x4_lib4
#endif
#endif


	// factorization

	movq	ARG6, %r10  // inv_diag_D 
	movq	ARG8, %r11 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_4X4_VS_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_dpotrf_4x4_vs_lib4
#elif defined(OS_MAC)
	callq _inner_edge_dpotrf_4x4_vs_lib4
#endif
#endif


	// store

	movq	ARG5, %r10 // D
	movq	ARG7, %r11 // km 
	movq	ARG8, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_L_4X4_VS_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_l_4x4_vs_lib4
#elif defined(OS_MAC)
	callq _inner_store_l_4x4_vs_lib4
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dpotrf_nt_l_4x4_vs_lib4, .-kernel_dpotrf_nt_l_4x4_vs_lib4
#endif





//                                        edi     rsi         rdx         ecx     r8          r9          rsp+8      rsp+16     rsp+24
// void kernel_dsyrk_dpotrf_nt_l_4x4_lib4(int kp, double *Ap, double *Bp, int km, double *Am, double *Bm, double *C, double *D, double *inv_diag_D);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dsyrk_dpotrf_nt_l_4x4_lib4
	.type kernel_dsyrk_dpotrf_nt_l_4x4_lib4, @function
kernel_dsyrk_dpotrf_nt_l_4x4_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dsyrk_dpotrf_nt_l_4x4_lib4
_kernel_dsyrk_dpotrf_nt_l_4x4_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dsyrk_dpotrf_nt_l_4x4_lib4
	.def kernel_dsyrk_dpotrf_nt_l_4x4_lib4; .scl 2; .type 32; .endef
kernel_dsyrk_dpotrf_nt_l_4x4_lib4:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%xmm0, %xmm0, %xmm0
	vmovapd	%xmm0, %xmm1
	vmovapd	%xmm0, %xmm2
	vmovapd	%xmm0, %xmm3
	vmovapd	%xmm0, %xmm4
	vmovapd	%xmm0, %xmm5
	vmovapd	%xmm0, %xmm6
	vmovapd	%xmm0, %xmm7


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG2, %r11  // Ap
	movq	ARG3, %r12  // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_dgemm_nt_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_kernel_dgemm_nt_4x4_lib4
#endif
#endif


	// change sing
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd		.LC11(%rip), %ymm15
#elif defined(OS_MAC)
	vmovapd		LC11(%rip), %ymm15
#endif
	vxorpd		%ymm15, %ymm0, %ymm0
	vxorpd		%ymm15, %ymm1, %ymm1
	vxorpd		%ymm15, %ymm2, %ymm2
	vxorpd		%ymm15, %ymm3, %ymm3


	// call inner dgemm kernel nt sub

	movq	ARG4, %r10 // km
	movq	ARG5, %r11   // Am
	movq	ARG6, %r12   // Bm

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_dgemm_nt_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_kernel_dgemm_nt_4x4_lib4
#endif
#endif


	// call inner blender_loader nn

	movq	ARG7, %r10   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_scale_m11_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_scale_m11_4x4_lib4
#endif
#endif


	// factorization

	movq	ARG9, %r10  // inv_diag_D 
	movl	$ 4, %r11d

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_4X4_VS_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_dpotrf_4x4_vs_lib4
#elif defined(OS_MAC)
	callq _inner_edge_dpotrf_4x4_vs_lib4
#endif
#endif


	// store

	movq	ARG8, %r10  // D 

#if MACRO_LEVEL>=1
	INNER_STORE_L_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_l_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_store_l_4x4_lib4
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dsyrk_dpotrf_nt_l_4x4_lib4, .-kernel_dsyrk_dpotrf_nt_l_4x4_lib4
#endif





//                                           edi     rsi         rdx         ecx     r8          r9          rsp+8      rsp+16     rsp+24              rsp+32  rsp+40
// void kernel_dsyrk_dpotrf_nt_l_4x4_vs_lib4(int kp, double *Ap, double *Bp, int km, double *Am, double *Bm, double *C, double *D, double *inv_diag_D, int km, int kn);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dsyrk_dpotrf_nt_l_4x4_vs_lib4
	.type kernel_dsyrk_dpotrf_nt_l_4x4_vs_lib4, @function
kernel_dsyrk_dpotrf_nt_l_4x4_vs_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dsyrk_dpotrf_nt_l_4x4_vs_lib4
_kernel_dsyrk_dpotrf_nt_l_4x4_vs_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dsyrk_dpotrf_nt_l_4x4_vs_lib4
	.def kernel_dsyrk_dpotrf_nt_l_4x4_vs_lib4; .scl 2; .type 32; .endef
kernel_dsyrk_dpotrf_nt_l_4x4_vs_lib4:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%xmm0, %xmm0, %xmm0
	vmovapd	%xmm0, %xmm1
	vmovapd	%xmm0, %xmm2
	vmovapd	%xmm0, %xmm3
	vmovapd	%xmm0, %xmm4
	vmovapd	%xmm0, %xmm5
	vmovapd	%xmm0, %xmm6
	vmovapd	%xmm0, %xmm7


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG2, %r11  // Ap
	movq	ARG3, %r12  // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_dgemm_nt_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_kernel_dgemm_nt_4x4_lib4
#endif
#endif


	// change sing
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd		.LC11(%rip), %ymm15
#elif defined(OS_MAC)
	vmovapd		LC11(%rip), %ymm15
#endif
	vxorpd		%ymm15, %ymm0, %ymm0
	vxorpd		%ymm15, %ymm1, %ymm1
	vxorpd		%ymm15, %ymm2, %ymm2
	vxorpd		%ymm15, %ymm3, %ymm3


	// call inner dgemm kernel nt sub

	movq	ARG4, %r10 // km
	movq	ARG5, %r11   // Am
	movq	ARG6, %r12   // Bm

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_dgemm_nt_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_kernel_dgemm_nt_4x4_lib4
#endif
#endif


	// call inner blender_loader nn

	movq	ARG7, %r10   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_scale_m11_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_scale_m11_4x4_lib4
#endif
#endif


	// factorization

	movq	ARG9, %r10  // inv_diag_D 
	movq	ARG11, %r11 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_4X4_VS_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_dpotrf_4x4_vs_lib4
#elif defined(OS_MAC)
	callq _inner_edge_dpotrf_4x4_vs_lib4
#endif
#endif


	// store

	movq	ARG8, %r10  // D 
	movq	ARG10, %r11 // km 
	movq	ARG11, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_L_4X4_VS_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_l_4x4_vs_lib4
#elif defined(OS_MAC)
	callq _inner_store_l_4x4_vs_lib4
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dsyrk_dpotrf_nt_l_4x4_vs_lib4, .-kernel_dsyrk_dpotrf_nt_l_4x4_vs_lib4
#endif





//                                      1      2          3          4             5          6          7          8
// void kernel_dtrsm_nt_rl_inv_4x4_lib4(int k, double *A, double *B, double *beta, double *C, double *D, double *E, double *inv_diag_E);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dtrsm_nt_rl_inv_4x4_lib4
	.type kernel_dtrsm_nt_rl_inv_4x4_lib4, @function
kernel_dtrsm_nt_rl_inv_4x4_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dtrsm_nt_rl_inv_4x4_lib4
_kernel_dtrsm_nt_rl_inv_4x4_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dtrsm_nt_rl_inv_4x4_lib4
	.def kernel_dtrsm_nt_rl_inv_4x4_lib4; .scl 2; .type 32; .endef
kernel_dtrsm_nt_rl_inv_4x4_lib4:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%xmm0, %xmm0, %xmm0
	vmovapd	%xmm0, %xmm1
	vmovapd	%xmm0, %xmm2
	vmovapd	%xmm0, %xmm3
	vmovapd	%xmm0, %xmm4
	vmovapd	%xmm0, %xmm5
	vmovapd	%xmm0, %xmm6
	vmovapd	%xmm0, %xmm7


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_dgemm_nt_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_kernel_dgemm_nt_4x4_lib4
#endif
#endif


	// call inner blender_loader nn

	movq	ARG4, %r10 // beta
	movq	ARG5, %r11 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_scale_m1b_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_scale_m1b_4x4_lib4
#endif
#endif


	// solve

	movq	ARG7, %r10  // E 
	movq	ARG8, %r11  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_dtrsm_rlt_inv_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_edge_dtrsm_rlt_inv_4x4_lib4
#endif
#endif


	// store

	movq	ARG6, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_store_4x4_lib4
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dtrsm_nt_rl_inv_4x4_lib4, .-kernel_dtrsm_nt_rl_inv_4x4_lib4
#endif





//                                            edi     rsi         rdx         ecx     r8          r9          rsp+8      rsp+16     rsp+24     rsp+32
// void kernel_dgemm_dtrsm_nt_rl_inv_4x4_lib4(int kp, double *Ap, double *Bp, int km, double *Am, double *Bm, double *C, double *D, double *E, double *inv_diag_E);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dgemm_dtrsm_nt_rl_inv_4x4_lib4
	.type kernel_dgemm_dtrsm_nt_rl_inv_4x4_lib4, @function
kernel_dgemm_dtrsm_nt_rl_inv_4x4_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dgemm_dtrsm_nt_rl_inv_4x4_lib4
_kernel_dgemm_dtrsm_nt_rl_inv_4x4_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dgemm_dtrsm_nt_rl_inv_4x4_lib4
	.def kernel_dgemm_dtrsm_nt_rl_inv_4x4_lib4; .scl 2; .type 32; .endef
kernel_dgemm_dtrsm_nt_rl_inv_4x4_lib4:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%xmm0, %xmm0, %xmm0
	vmovapd	%xmm0, %xmm1
	vmovapd	%xmm0, %xmm2
	vmovapd	%xmm0, %xmm3
	vmovapd	%xmm0, %xmm4
	vmovapd	%xmm0, %xmm5
	vmovapd	%xmm0, %xmm6
	vmovapd	%xmm0, %xmm7


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG2, %r11  // Ap
	movq	ARG3, %r12  // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_dgemm_nt_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_kernel_dgemm_nt_4x4_lib4
#endif
#endif


	// change sing
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd		.LC11(%rip), %ymm15
#elif defined(OS_MAC)
	vmovapd		LC11(%rip), %ymm15
#endif
	vxorpd		%ymm15, %ymm0, %ymm0
	vxorpd		%ymm15, %ymm1, %ymm1
	vxorpd		%ymm15, %ymm2, %ymm2
	vxorpd		%ymm15, %ymm3, %ymm3


	// call inner dgemm kernel nt sub

	movq	ARG4, %r10 // km
	movq	ARG5, %r11   // Am
	movq	ARG6, %r12   // Bm

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_dgemm_nt_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_kernel_dgemm_nt_4x4_lib4
#endif
#endif


	// call inner blender_loader nn

	movq	ARG7, %r10   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_scale_m11_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_scale_m11_4x4_lib4
#endif
#endif


	// solve

	movq	ARG9, %r10  // E 
	movq	ARG10, %r11  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_dtrsm_rlt_inv_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_edge_dtrsm_rlt_inv_4x4_lib4
#endif
#endif


	// store

	movq	ARG8, %r10   // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_store_4x4_lib4
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dgemm_dtrsm_nt_rl_inv_4x4_lib4, .-kernel_dgemm_dtrsm_nt_rl_inv_4x4_lib4
#endif





//                                         1      2          3          4             5          6          7          8     9       10
// void kernel_dtrsm_nt_rl_inv_4x4_vs_lib4(int k, double *A, double *B, double *C, double *D, double *E, double *inv_diag_E, int km, int kn);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dtrsm_nt_rl_inv_4x4_vs_lib4
	.type kernel_dtrsm_nt_rl_inv_4x4_vs_lib4, @function
kernel_dtrsm_nt_rl_inv_4x4_vs_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dtrsm_nt_rl_inv_4x4_vs_lib4
_kernel_dtrsm_nt_rl_inv_4x4_vs_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dtrsm_nt_rl_inv_4x4_vs_lib4
	.def kernel_dtrsm_nt_rl_inv_4x4_vs_lib4; .scl 2; .type 32; .endef
kernel_dtrsm_nt_rl_inv_4x4_vs_lib4:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%xmm0, %xmm0, %xmm0
	vmovapd	%xmm0, %xmm1
	vmovapd	%xmm0, %xmm2
	vmovapd	%xmm0, %xmm3
	vmovapd	%xmm0, %xmm4
	vmovapd	%xmm0, %xmm5
	vmovapd	%xmm0, %xmm6
	vmovapd	%xmm0, %xmm7


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_dgemm_nt_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_kernel_dgemm_nt_4x4_lib4
#endif
#endif


	// call inner blender_loader nn // TODO scale gen

	movq	ARG4, %r10 // beta
	movq	ARG5, %r11 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_scale_m1b_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_scale_m1b_4x4_lib4
#endif
#endif


	// solve

	movq	ARG7, %r10  // E 
	movq	ARG8, %r11  // inv_diag_E 
	movq	ARG10, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_4X4_VS_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_dtrsm_rlt_inv_4x4_vs_lib4
#elif defined(OS_MAC)
	callq _inner_edge_dtrsm_rlt_inv_4x4_vs_lib4
#endif
#endif


	// store

	movq	ARG6, %r10 // D
	movq	ARG9, %r11 // km 
	movq	ARG10, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_4x4_vs_lib4
#elif defined(OS_MAC)
	callq _inner_store_4x4_vs_lib4
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dtrsm_nt_rl_inv_4x4_vs_lib4, .-kernel_dtrsm_nt_rl_inv_4x4_vs_lib4
#endif





//                                               edi     rsi         rdx         ecx     r8          r9          rsp+8      rsp+16     rsp+24     rsp+32              rsp+40  rsp+48
// void kernel_dgemm_dtrsm_nt_rl_inv_4x4_vs_lib4(int kp, double *Ap, double *Bp, int km, double *Am, double *Bm, double *C, double *D, double *E, double *inv_diag_E, int km, int kn);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dgemm_dtrsm_nt_rl_inv_4x4_vs_lib4
	.type kernel_dgemm_dtrsm_nt_rl_inv_4x4_vs_lib4, @function
kernel_dgemm_dtrsm_nt_rl_inv_4x4_vs_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dgemm_dtrsm_nt_rl_inv_4x4_vs_lib4
_kernel_dgemm_dtrsm_nt_rl_inv_4x4_vs_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dgemm_dtrsm_nt_rl_inv_4x4_vs_lib4
	.def kernel_dgemm_dtrsm_nt_rl_inv_4x4_vs_lib4; .scl 2; .type 32; .endef
kernel_dgemm_dtrsm_nt_rl_inv_4x4_vs_lib4:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%xmm0, %xmm0, %xmm0
	vmovapd	%xmm0, %xmm1
	vmovapd	%xmm0, %xmm2
	vmovapd	%xmm0, %xmm3
	vmovapd	%xmm0, %xmm4
	vmovapd	%xmm0, %xmm5
	vmovapd	%xmm0, %xmm6
	vmovapd	%xmm0, %xmm7


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG2, %r11  // Ap
	movq	ARG3, %r12  // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_dgemm_nt_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_kernel_dgemm_nt_4x4_lib4
#endif
#endif


	// change sing
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd		.LC11(%rip), %ymm15
#elif defined(OS_MAC)
	vmovapd		LC11(%rip), %ymm15
#endif
	vxorpd		%ymm15, %ymm0, %ymm0
	vxorpd		%ymm15, %ymm1, %ymm1
	vxorpd		%ymm15, %ymm2, %ymm2
	vxorpd		%ymm15, %ymm3, %ymm3


	// call inner dgemm kernel nt sub

	movq	ARG4, %r10 // km
	movq	ARG5, %r11   // Am
	movq	ARG6, %r12   // Bm

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_dgemm_nt_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_kernel_dgemm_nt_4x4_lib4
#endif
#endif


	// call inner blender_loader nn

	movq	ARG7, %r10  // C 

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_4X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_scale_m11_4x4_lib4
#elif defined(OS_MAC)
	callq _inner_scale_m11_4x4_lib4
#endif
#endif


	// solve

	movq	ARG9, %r10  // E 
	movq	ARG10, %r11  // inv_diag_E 
	movq	ARG12, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_4X4_VS_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_dtrsm_rlt_inv_4x4_vs_lib4
#elif defined(OS_MAC)
	callq _inner_edge_dtrsm_rlt_inv_4x4_vs_lib4
#endif
#endif


	// store

	movq	ARG8, %r10 // D 
	movq	ARG11, %r11 // km 
	movq	ARG12, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_4x4_vs_lib4
#elif defined(OS_MAC)
	callq _inner_store_4x4_vs_lib4
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dgemm_dtrsm_nt_rl_inv_4x4_vs_lib4, .-kernel_dgemm_dtrsm_nt_rl_inv_4x4_vs_lib4
#endif





//#if defined(BLAS_API)
#if ( defined(BLAS_API) | ( defined(LA_HIGH_PERFORMANCE) & defined(MF_COLMAJ) ) )

#include "kernel_dgemm_4x4_lib.S"

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


#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC11:
#elif defined(OS_MAC)
	.align 5
LC11:
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

