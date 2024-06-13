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
// r14   <- 4*sdb*sizeof(double)
//
// output arguments:
// r10d  <- k
// r11   <- A+4*sda*sizeof(double)
// r12   <- 4*sda*sizeof(double)
// r13   <- B+(k/4)*sdb*sizeof(double)+(k%4)
// r14   <- 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_ADD_NN_10X4_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_dgemm_add_nn_10x4_lib4, @function
inner_kernel_dgemm_add_nn_10x4_lib4:
#elif defined(OS_MAC)
_inner_kernel_dgemm_add_nn_10x4_lib4:
#elif defined(OS_WINDOWS)
	.def inner_kernel_dgemm_add_nn_10x4_lib4; .scl 2; .type 32; .endef
inner_kernel_dgemm_add_nn_10x4_lib4:
#endif
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	// preload

	cmpl	$ 3, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	prefetcht0	0(%r13, %r14, 2) // software prefetch
	prefetcht0	64(%r13, %r14, 2) // software prefetch

	// unroll 0
	vmovapd			0(%r11), %ymm10 // A0[0]
	vmovapd 		0(%r11, %r12, 1), %ymm11 // A1[0]
	vbroadcastsd	0(%r13), %ymm13
	vmulpd			%ymm10, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	32(%r13), %ymm14
	vmulpd			%ymm10, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm11, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastf128	0(%r11, %r12, 2), %ymm12 // A2[0]
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm8, %ymm8

	vbroadcastsd	64(%r13), %ymm13
	vmulpd			%ymm10, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6
	vbroadcastsd	96(%r13), %ymm14
	vmulpd			%ymm10, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm11, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm9, %ymm9
	subl	$ 4, %r10d

	// unroll 1
	vmovapd			32(%r11), %ymm10 // A0[0]
	vmovapd 		32(%r11, %r12, 1), %ymm11 // A1[0]
	vbroadcastsd	8(%r13), %ymm13
	vmulpd			%ymm10, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	40(%r13), %ymm14
	vmulpd			%ymm10, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm11, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastf128	32(%r11, %r12, 2), %ymm12 // A2[0]
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm8, %ymm8

	vbroadcastsd	72(%r13), %ymm13
	vmulpd			%ymm10, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6
	vbroadcastsd	104(%r13), %ymm14
	vmulpd			%ymm10, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm11, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm9, %ymm9

	// unroll 2
	vmovapd			64(%r11), %ymm10 // A0[0]
	vmovapd 		64(%r11, %r12, 1), %ymm11 // A1[0]
	vbroadcastsd	16(%r13), %ymm13
	vmulpd			%ymm10, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	48(%r13), %ymm14
	vmulpd			%ymm10, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm11, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastf128	64(%r11, %r12, 2), %ymm12 // A2[0]
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm8, %ymm8

	vbroadcastsd	80(%r13), %ymm13
	vmulpd			%ymm10, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6
	vbroadcastsd	112(%r13), %ymm14
	vmulpd			%ymm10, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm11, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm9, %ymm9

	// unroll 3
	vmovapd			96(%r11), %ymm10 // A0[0]
	vmovapd 		96(%r11, %r12, 1), %ymm11 // A1[0]
	vbroadcastsd	24(%r13), %ymm13
	vmulpd			%ymm10, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	56(%r13), %ymm14
	vmulpd			%ymm10, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm11, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastf128	96(%r11, %r12, 2), %ymm12 // A2[0]
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm8, %ymm8

	vbroadcastsd	88(%r13), %ymm13
	vmulpd			%ymm10, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	addq	$ 128, %r11
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6
	vbroadcastsd	120(%r13), %ymm14
	vmulpd			%ymm10, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	addq	%r14, %r13
	vmulpd			%ymm11, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm9, %ymm9


	cmpl	$ 4, %r10d
	jg		1b // main loop 

0: // consider clean4-up
	
	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	vmovapd			0(%r11), %ymm10 // A0[0]
	vmovapd 		0(%r11, %r12, 1), %ymm11 // A1[0]
	vbroadcastsd	0(%r13), %ymm13
	vmulpd			%ymm10, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	32(%r13), %ymm14
	vmulpd			%ymm10, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm11, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastf128	0(%r11, %r12, 2), %ymm12 // A2[0]
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm8, %ymm8
	addq	$ 32, %r11

	vbroadcastsd	64(%r13), %ymm13
	vmulpd			%ymm10, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6
	subl	$ 1, %r10d
	vbroadcastsd	96(%r13), %ymm14
	vmulpd			%ymm10, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	addq	$ 8, %r13
	vmulpd			%ymm11, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm9, %ymm9

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 

2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_dgemm_add_nn_10x4_lib4, .-inner_kernel_dgemm_add_nn_10x4_lib4
#endif
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- 4*sda*sizeof(double)
// r13   <- B
// r14   <- 4*sdb*sizeof(double)
//
// output arguments:
// r10d  <- k
// r11   <- A+4*sda*sizeof(double)
// r12   <- 4*sda*sizeof(double)
// r13   <- B+(k/4)*sdb*sizeof(double)+(k%4)
// r14   <- 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_ADD_NN_10X2_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_dgemm_add_nn_10x2_lib4, @function
inner_kernel_dgemm_add_nn_10x2_lib4:
#elif defined(OS_MAC)
_inner_kernel_dgemm_add_nn_10x2_lib4:
#elif defined(OS_WINDOWS)
	.def inner_kernel_dgemm_add_nn_10x2_lib4; .scl 2; .type 32; .endef
inner_kernel_dgemm_add_nn_10x2_lib4:
#endif
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	// preload

	cmpl	$ 3, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	prefetcht0	0(%r13, %r14, 2) // software prefetch

	// unroll 0
	vmovapd			0(%r11), %ymm10 // A0[0]
	vmovapd 		0(%r11, %r12, 1), %ymm11 // A1[0]
	vbroadcastsd	0(%r13), %ymm13
	vmulpd			%ymm10, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	32(%r13), %ymm14
	vmulpd			%ymm10, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm11, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastf128	0(%r11, %r12, 2), %ymm12 // A2[0]
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm8, %ymm8
	subl	$ 4, %r10d

	// unroll 1
	vmovapd			32(%r11), %ymm10 // A0[0]
	vmovapd 		32(%r11, %r12, 1), %ymm11 // A1[0]
	vbroadcastsd	8(%r13), %ymm13
	vmulpd			%ymm10, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	40(%r13), %ymm14
	vmulpd			%ymm10, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm11, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastf128	32(%r11, %r12, 2), %ymm12 // A2[0]
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm8, %ymm8

	// unroll 2
	vmovapd			64(%r11), %ymm10 // A0[0]
	vmovapd 		64(%r11, %r12, 1), %ymm11 // A1[0]
	vbroadcastsd	16(%r13), %ymm13
	vmulpd			%ymm10, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	48(%r13), %ymm14
	vmulpd			%ymm10, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm11, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastf128	64(%r11, %r12, 2), %ymm12 // A2[0]
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm8, %ymm8

	// unroll 3
	vmovapd			96(%r11), %ymm10 // A0[0]
	vmovapd 		96(%r11, %r12, 1), %ymm11 // A1[0]
	vbroadcastsd	24(%r13), %ymm13
	vmulpd			%ymm10, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	56(%r13), %ymm14
	vmulpd			%ymm10, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	addq	%r14, %r13
	vmulpd			%ymm11, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastf128	96(%r11, %r12, 2), %ymm12 // A2[0]
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm8, %ymm8
	addq	$ 128, %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop 

0: // consider clean4-up
	
	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	vmovapd			0(%r11), %ymm10 // A0[0]
	vmovapd 		0(%r11, %r12, 1), %ymm11 // A1[0]
	vbroadcastsd	0(%r13), %ymm13
	vmulpd			%ymm10, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	subl	$ 1, %r10d
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	32(%r13), %ymm14
	vmulpd			%ymm10, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	addq	$ 8, %r13
	vmulpd			%ymm11, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastf128	0(%r11, %r12, 2), %ymm12 // A2[0]
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm8, %ymm8
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 

2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_dgemm_add_nn_10x2_lib4, .-inner_kernel_dgemm_add_nn_10x2_lib4
#endif
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
	.macro INNER_EDGE_DGEMM_ADD_NN_10X4_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_edge_dgemm_add_nn_10x4_lib4, @function
inner_edge_dgemm_add_nn_10x4_lib4:
#elif defined(OS_MAC)
_inner_edge_dgemm_add_nn_10x4_lib4:
#elif defined(OS_WINDOWS)
	.def inner_edge_dgemm_add_nn_10x4_lib4; .scl 2; .type 32; .endef
inner_edge_dgemm_add_nn_10x4_lib4:
#endif
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
	vmovapd			0(%r11), %ymm10 // A0[0]
	vmovapd 		0(%r11, %r12, 1), %ymm11 // A1[0]
	vbroadcastsd	0(%r13), %ymm13
	vmulpd			%ymm10, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	32(%r13), %ymm14
	vmulpd			%ymm10, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm11, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastf128	0(%r11, %r12, 2), %ymm12 // A2[0]
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm8, %ymm8

	vbroadcastsd	64(%r13), %ymm13
	vmulpd			%ymm10, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm6, %ymm6
	vbroadcastsd	96(%r13), %ymm14
	vmulpd			%ymm10, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmulpd			%ymm11, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm7, %ymm7
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm9, %ymm9

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

#if defined(OS_LINUX)
	.size	inner_edge_dgemm_add_nn_10x4_lib4, .-inner_edge_dgemm_add_nn_10x4_lib4
#endif
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
	.macro INNER_EDGE_DGEMM_ADD_NN_10X2_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_edge_dgemm_add_nn_10x2_lib4, @function
inner_edge_dgemm_add_nn_10x2_lib4:
#elif defined(OS_MAC)
_inner_edge_dgemm_add_nn_10x2_lib4:
#elif defined(OS_WINDOWS)
	.def inner_edge_dgemm_add_nn_10x2_lib4; .scl 2; .type 32; .endef
inner_edge_dgemm_add_nn_10x2_lib4:
#endif
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
	vmovapd			0(%r11), %ymm10 // A0[0]
	vmovapd 		0(%r11, %r12, 1), %ymm11 // A1[0]
	vbroadcastsd	0(%r13), %ymm13
	vmulpd			%ymm10, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm15, %ymm4, %ymm4
	vbroadcastsd	32(%r13), %ymm14
	vmulpd			%ymm10, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm11, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm5, %ymm5
	vbroadcastf128	0(%r11, %r12, 2), %ymm12 // A2[0]
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vmulpd			%ymm12, %ymm14, %ymm15
	vaddpd			%ymm15, %ymm8, %ymm8

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

#if defined(OS_LINUX)
	.size	inner_edge_dgemm_add_nn_10x2_lib4, .-inner_edge_dgemm_add_nn_10x2_lib4
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
// r13   <- 4*sdc*sizeof(double)
//
// output arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_10X4_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_scale_ab_10x4_lib4, @function
inner_scale_ab_10x4_lib4:
#elif defined(OS_MAC)
_inner_scale_ab_10x4_lib4:
#elif defined(OS_WINDOWS)
	.def inner_scale_ab_10x4_lib4; .scl 2; .type 32; .endef
inner_scale_ab_10x4_lib4:
#endif
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

	// beta
	vbroadcastsd	0(%r11), %ymm14

	vxorpd		%ymm15, %ymm15, %ymm15 // 0.0

	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	// alg==1
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

	vmovapd		0(%r12, %r13, 1), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm4, %ymm15, %ymm4
	vmovapd		32(%r12, %r13, 1), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm5, %ymm15, %ymm5
	vmovapd		64(%r12, %r13, 1), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm6, %ymm15, %ymm6
	vmovapd		96(%r12, %r13, 1), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm7, %ymm15, %ymm7

	vmovapd		0(%r12, %r13, 2), %xmm15
	vmovupd		16(%r12, %r13, 2), %ymm12
	vblendpd	$ 0x3, %ymm15, %ymm12, %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm8, %ymm15, %ymm8
	vmovapd		64(%r12, %r13, 2), %xmm15
	vmovupd		80(%r12, %r13, 2), %ymm12
	vblendpd	$ 0x3, %ymm15, %ymm12, %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm9, %ymm15, %ymm9

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_scale_ab_10x4_lib4, .-inner_scale_ab_10x4_lib4
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
// r13   <- 4*sdc*sizeof(double)
//
// output arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_10X2_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_scale_ab_10x2_lib4, @function
inner_scale_ab_10x2_lib4:
#elif defined(OS_MAC)
_inner_scale_ab_10x2_lib4:
#elif defined(OS_WINDOWS)
	.def inner_scale_ab_10x2_lib4; .scl 2; .type 32; .endef
inner_scale_ab_10x2_lib4:
#endif
#endif

	// alpha
	vbroadcastsd	0(%r10), %ymm15

	vmulpd		%ymm0, %ymm15, %ymm0
	vmulpd		%ymm1, %ymm15, %ymm1
	vmulpd		%ymm4, %ymm15, %ymm4
	vmulpd		%ymm5, %ymm15, %ymm5


	vmulpd		%ymm8, %ymm15, %ymm8

	// beta
	vbroadcastsd	0(%r11), %ymm14

	vxorpd		%ymm15, %ymm15, %ymm15 // 0.0

	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	// alg==1
	vmovapd		0(%r12), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm0, %ymm15, %ymm0
	vmovapd		32(%r12), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm1, %ymm15, %ymm1

	vmovapd		0(%r12, %r13, 1), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm4, %ymm15, %ymm4
	vmovapd		32(%r12, %r13, 1), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm5, %ymm15, %ymm5

	vmovapd		0(%r12, %r13, 2), %xmm15
	vmovupd		16(%r12, %r13, 2), %ymm12
	vblendpd	$ 0x3, %ymm15, %ymm12, %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm8, %ymm15, %ymm8

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_scale_ab_10x2_lib4, .-inner_scale_ab_10x2_lib4
#endif
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// r11  <- 4*sdd*sizeof(double)
//
// output arguments:
// r10  <- D
// r11  <- 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	.macro INNER_STORE_10X4_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_store_10x4_lib4, @function
inner_store_10x4_lib4:
#elif defined(OS_MAC)
_inner_store_10x4_lib4:
#elif defined(OS_WINDOWS)
	.def inner_store_10x4_lib4; .scl 2; .type 32; .endef
inner_store_10x4_lib4:
#endif
#endif
	
	vmovapd 		%ymm0, 0(%r10)
	vmovapd 		%ymm4, 0(%r10, %r11, 1)
	vmovapd 		%xmm8, 0(%r10, %r11, 2)

	vextractf128	$ 0x1, %ymm8, %xmm15
	vmovapd 		%ymm1, 32(%r10)
	vmovapd 		%ymm5, 32(%r10, %r11, 1)
	vmovapd 		%xmm15, 32(%r10, %r11, 2)

	vmovapd 		%ymm2, 64(%r10)
	vmovapd 		%ymm6, 64(%r10, %r11, 1)
	vmovapd 		%xmm9, 64(%r10, %r11, 2)

	vextractf128	$ 0x1, %ymm9, %xmm15
	vmovapd 		%ymm3, 96(%r10)
	vmovapd 		%ymm7, 96(%r10, %r11, 1)
	vmovapd 		%xmm15, 96(%r10, %r11, 2)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_store_10x4_lib4, .-inner_store_10x4_lib4
#endif
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// r11  <- 4*sdd*sizeof(double)
//
// output arguments:
// r10  <- D
// r11  <- 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	.macro INNER_STORE_10X2_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_store_10x2_lib4, @function
inner_store_10x2_lib4:
#elif defined(OS_MAC)
_inner_store_10x2_lib4:
#elif defined(OS_WINDOWS)
	.def inner_store_10x2_lib4; .scl 2; .type 32; .endef
inner_store_10x2_lib4:
#endif
#endif
	
	vmovapd 		%ymm0, 0(%r10)
	vmovapd 		%ymm4, 0(%r10, %r11, 1)
	vmovapd 		%xmm8, 0(%r10, %r11, 2)

	vextractf128	$ 0x1, %ymm8, %xmm15
	vmovapd 		%ymm1, 32(%r10)
	vmovapd 		%ymm5, 32(%r10, %r11, 1)
	vmovapd 		%xmm15, 32(%r10, %r11, 2)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_store_10x2_lib4, .-inner_store_10x2_lib4
#endif
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
//
// output arguments:
// r10  <- D
// r11  <- 4*sdd*sizeof(double)
// r12d  <- km
// r13d  <- kn

#if MACRO_LEVEL>=1
	.macro INNER_STORE_10X4_VS_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_store_10x4_vs_lib4, @function
inner_store_10x4_vs_lib4:
#elif defined(OS_MAC)
_inner_store_10x4_vs_lib4:
#elif defined(OS_WINDOWS)
	.def inner_store_10x4_vs_lib4; .scl 2; .type 32; .endef
inner_store_10x4_vs_lib4:
#endif
#endif
	
	vcvtsi2sd	%r12d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC04(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC04(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm14


//	vxorpd		%ymm15, %ymm15, %ymm15
//	vblendpd	$ 0x3, %ymm14, %ymm15, %ymm14



	vmovapd 		%ymm0, 0(%r10)
	vmovapd 		%ymm4, 0(%r10, %r11, 1)
	vmaskmovpd		%xmm8, %xmm14, 0(%r10, %r11, 2)

	cmpl		$ 2, %r13d
	jl			0f // end

	vextractf128	$ 0x1, %ymm8, %xmm15
	vmovapd 		%ymm1, 32(%r10)
	vmovapd 		%ymm5, 32(%r10, %r11, 1)
	vmaskmovpd		%xmm15, %xmm14, 32(%r10, %r11, 2)

	cmpl		$ 3, %r13d
	jl			0f // end

	vmovapd 		%ymm2, 64(%r10)
	vmovapd 		%ymm6, 64(%r10, %r11, 1)
	vmaskmovpd		%xmm9, %xmm14, 64(%r10, %r11, 2)

	je			0f // end

	vextractf128	$ 0x1, %ymm9, %xmm15
	vmovapd 		%ymm3, 96(%r10)
	vmovapd 		%ymm7, 96(%r10, %r11, 1)
	vmaskmovpd		%xmm15, %xmm14, 96(%r10, %r11, 2)

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_store_10x4_vs_lib4, .-inner_store_10x4_vs_lib4
#endif
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
//
// output arguments:
// r10  <- D
// r11  <- 4*sdd*sizeof(double)
// r12d  <- km
// r13d  <- kn

#if MACRO_LEVEL>=1
	.macro INNER_STORE_10X2_VS_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_store_10x2_vs_lib4, @function
inner_store_10x2_vs_lib4:
#elif defined(OS_MAC)
_inner_store_10x2_vs_lib4:
#elif defined(OS_WINDOWS)
	.def inner_store_10x2_vs_lib4; .scl 2; .type 32; .endef
inner_store_10x2_vs_lib4:
#endif
#endif
	
	vcvtsi2sd	%r12d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC04(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC04(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm14


	vmovapd 		%ymm0, 0(%r10)
	vmovapd 		%ymm4, 0(%r10, %r11, 1)
	vmaskmovpd		%xmm8, %xmm14, 0(%r10, %r11, 2)

	cmpl		$ 2, %r13d
	jl			0f // end

	vextractf128	$ 0x1, %ymm8, %xmm15
	vmovapd 		%ymm1, 32(%r10)
	vmovapd 		%ymm5, 32(%r10, %r11, 1)
	vmaskmovpd		%xmm15, %xmm14, 32(%r10, %r11, 2)

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_store_10x2_vs_lib4, .-inner_store_10x2_vs_lib4
#endif
#endif





//                               1      2              3          4        5            6          7        8             9          10       11         12
// void kernel_dgemm_nn_10x4_lib4(int k, double *alpha, double *A, int sda, int offsetB, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dgemm_nn_10x4_lib4
	.type kernel_dgemm_nn_10x4_lib4, @function
kernel_dgemm_nn_10x4_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dgemm_nn_10x4_lib4
_kernel_dgemm_nn_10x4_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dgemm_nn_10x4_lib4
	.def kernel_dgemm_nn_10x4_lib4; .scl 2; .type 32; .endef
kernel_dgemm_nn_10x4_lib4:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3
	vmovapd	%ymm0, %ymm4
	vmovapd	%ymm0, %ymm5
	vmovapd	%ymm0, %ymm6
	vmovapd	%ymm0, %ymm7
	vmovapd	%ymm0, %ymm8
	vmovapd	%ymm0, %ymm9


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
	INNER_EDGE_DGEMM_ADD_NN_10X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_dgemm_add_nn_10x4_lib4
#elif defined(OS_MAC)
	callq _inner_edge_dgemm_add_nn_10x4_lib4
#endif
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_ADD_NN_10X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_dgemm_add_nn_10x4_lib4
#elif defined(OS_MAC)
	callq _inner_kernel_dgemm_add_nn_10x4_lib4
#endif
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12 // C
	movq	ARG10, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_10X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_scale_ab_10x4_lib4
#elif defined(OS_MAC)
	callq _inner_scale_ab_10x4_lib4
#endif
#endif


	// store n

	movq	ARG11, %r10 // D
	movq	ARG12, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_10X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_10x4_lib4
#elif defined(OS_MAC)
	callq _inner_store_10x4_lib4
#endif
#endif


	EPILOGUE
	
	ret

#if defined(OS_LINUX)
	.size	kernel_dgemm_nn_10x4_lib4, .-kernel_dgemm_nn_10x4_lib4
#endif





//                               1      2              3          4        5            6          7        8             9          10       11         12
// void kernel_dgemm_nn_10x2_lib4(int k, double *alpha, double *A, int sda, int offsetB, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dgemm_nn_10x2_lib4
	.type kernel_dgemm_nn_10x2_lib4, @function
kernel_dgemm_nn_10x2_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dgemm_nn_10x2_lib4
_kernel_dgemm_nn_10x2_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dgemm_nn_10x2_lib4
	.def kernel_dgemm_nn_10x2_lib4; .scl 2; .type 32; .endef
kernel_dgemm_nn_10x2_lib4:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3
	vmovapd	%ymm0, %ymm4
	vmovapd	%ymm0, %ymm5
	vmovapd	%ymm0, %ymm6
	vmovapd	%ymm0, %ymm7
	vmovapd	%ymm0, %ymm8
	vmovapd	%ymm0, %ymm9


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
	INNER_EDGE_DGEMM_ADD_NN_10X2_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_dgemm_add_nn_10x2_lib4
#elif defined(OS_MAC)
	callq _inner_edge_dgemm_add_nn_10x2_lib4
#endif
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_ADD_NN_10X2_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_dgemm_add_nn_10x2_lib4
#elif defined(OS_MAC)
	callq _inner_kernel_dgemm_add_nn_10x2_lib4
#endif
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12 // C
	movq	ARG10, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_10X2_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_scale_ab_10x2_lib4
#elif defined(OS_MAC)
	callq _inner_scale_ab_10x2_lib4
#endif
#endif


	// store n

	movq	ARG11, %r10 // D
	movq	ARG12, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_10X2_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_10x2_lib4
#elif defined(OS_MAC)
	callq _inner_store_10x2_lib4
#endif
#endif


	EPILOGUE
	
	ret

#if defined(OS_LINUX)
	.size	kernel_dgemm_nn_10x2_lib4, .-kernel_dgemm_nn_10x2_lib4
#endif





//                                  1      2              3          4        5            6          7        8             9          10       11         12        13      14
// void kernel_dgemm_nn_10x4_vs_lib4(int k, double *alpha, double *A, int sda, int offsetB, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd, int m1, int n1);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dgemm_nn_10x4_vs_lib4
	.type kernel_dgemm_nn_10x4_vs_lib4, @function
kernel_dgemm_nn_10x4_vs_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dgemm_nn_10x4_vs_lib4
_kernel_dgemm_nn_10x4_vs_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dgemm_nn_10x4_vs_lib4
	.def kernel_dgemm_nn_10x4_vs_lib4; .scl 2; .type 32; .endef
kernel_dgemm_nn_10x4_vs_lib4:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3
	vmovapd	%ymm0, %ymm4
	vmovapd	%ymm0, %ymm5
	vmovapd	%ymm0, %ymm6
	vmovapd	%ymm0, %ymm7
	vmovapd	%ymm0, %ymm8
	vmovapd	%ymm0, %ymm9


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
	INNER_EDGE_DGEMM_ADD_NN_10X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_dgemm_add_nn_10x4_lib4
#elif defined(OS_MAC)
	callq _inner_edge_dgemm_add_nn_10x4_lib4
#endif
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_ADD_NN_10X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_dgemm_add_nn_10x4_lib4
#elif defined(OS_MAC)
	callq _inner_kernel_dgemm_add_nn_10x4_lib4
#endif
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12 // C
	movq	ARG10, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_10X4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_scale_ab_10x4_lib4
#elif defined(OS_MAC)
	callq _inner_scale_ab_10x4_lib4
#endif
#endif


	// store n

	movq	ARG11, %r10 // D
	movq	ARG12, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG13, %r12 // m1
	movq	ARG14, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_10X4_VS_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_10x4_vs_lib4
#elif defined(OS_MAC)
	callq _inner_store_10x4_vs_lib4
#endif
#endif


	EPILOGUE
	
	ret

#if defined(OS_LINUX)
	.size	kernel_dgemm_nn_10x4_vs_lib4, .-kernel_dgemm_nn_10x4_vs_lib4
#endif





//                                  1      2              3          4        5            6          7        8             9          10       11         12        13      14
// void kernel_dgemm_nn_10x2_vs_lib4(int k, double *alpha, double *A, int sda, int offsetB, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd, int m1, int n1);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dgemm_nn_10x2_vs_lib4
	.type kernel_dgemm_nn_10x2_vs_lib4, @function
kernel_dgemm_nn_10x2_vs_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dgemm_nn_10x2_vs_lib4
_kernel_dgemm_nn_10x2_vs_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dgemm_nn_10x2_vs_lib4
	.def kernel_dgemm_nn_10x2_vs_lib4; .scl 2; .type 32; .endef
kernel_dgemm_nn_10x2_vs_lib4:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3
	vmovapd	%ymm0, %ymm4
	vmovapd	%ymm0, %ymm5
	vmovapd	%ymm0, %ymm6
	vmovapd	%ymm0, %ymm7
	vmovapd	%ymm0, %ymm8
	vmovapd	%ymm0, %ymm9


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
	INNER_EDGE_DGEMM_ADD_NN_10X2_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_dgemm_add_nn_10x2_lib4
#elif defined(OS_MAC)
	callq _inner_edge_dgemm_add_nn_10x2_lib4
#endif
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_ADD_NN_10X2_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_dgemm_add_nn_10x2_lib4
#elif defined(OS_MAC)
	callq _inner_kernel_dgemm_add_nn_10x2_lib4
#endif
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12 // C
	movq	ARG10, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_10X2_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_scale_ab_10x2_lib4
#elif defined(OS_MAC)
	callq _inner_scale_ab_10x2_lib4
#endif
#endif


	// store n

	movq	ARG11, %r10 // D
	movq	ARG12, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG13, %r12 // m1
	movq	ARG14, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_10X2_VS_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_10x2_vs_lib4
#elif defined(OS_MAC)
	callq _inner_store_10x2_vs_lib4
#endif
#endif


	EPILOGUE
	
	ret

#if defined(OS_LINUX)
	.size	kernel_dgemm_nn_10x2_vs_lib4, .-kernel_dgemm_nn_10x2_vs_lib4
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
.LC04: // { 11.5 10.5 9.5 8.5 }
#elif defined(OS_MAC)
LC04: // { 11.5 10.5 9.5 8.5 }
	.align 5
#endif
	.double	8.5
	.double	9.5
	.double	10.5
	.double	11.5



#if defined(OS_LINUX)
	.section	.note.GNU-stack,"",@progbits
#elif defined(OS_MAC)
	.subsections_via_symbols
#endif

