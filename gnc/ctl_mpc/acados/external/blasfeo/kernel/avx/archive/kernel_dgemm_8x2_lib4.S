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
// r15   <- dirty
// rax   <- dirty
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d40 d50 d60 d70]
// ymm3  <- [d41 d51 d61 d71]

//
// output arguments:
// r10d  <- k
// r11   <- A+4*sda*sizeof(double)
// r12   <- 4*sda*sizeof(double)
// r13   <- B+(k/4)*sdb*sizeof(double)+(k%4)
// r14   <- 4*sdb*sizeof(double)
// r15   <- dirty
// rax   <- dirty
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d40 d50 d60 d70]
// ymm3  <- [d41 d51 d61 d71]

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_ADD_NN_8X2_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_dgemm_add_nn_8x2_lib4, @function
inner_kernel_dgemm_add_nn_8x2_lib4:
#elif defined(OS_MAC)
_inner_kernel_dgemm_add_nn_8x2_lib4:
#elif defined(OS_WINDOWS)
	.def inner_kernel_dgemm_add_nn_8x2_lib4; .scl 2; .type 32; .endef
inner_kernel_dgemm_add_nn_8x2_lib4:
#endif
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	// preload
	vmovapd 0(%r11), %ymm8 // A0[0]
	vmovapd 0(%r11, %r12, 1), %ymm9 // A1[0]

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	prefetcht0	0(%r13, %r14, 2) // software prefetch

	// unroll 0
	vbroadcastsd	0(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmovapd			32(%r11), %ymm10 // A0

	vbroadcastsd	32(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmovapd			32(%r11, %r12, 1), %ymm11 // A1
	subl	$ 4, %r10d

	// unroll 1
	vbroadcastsd	8(%r13), %ymm12
	vmulpd			%ymm10, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm11, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmovapd			64(%r11), %ymm8 // A0

	vbroadcastsd	40(%r13), %ymm12
	vmulpd			%ymm10, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm11, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmovapd			64(%r11, %r12, 1), %ymm9 // A1

	// unroll 2
	vbroadcastsd	16(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmovapd			96(%r11), %ymm10 // A0

	vbroadcastsd	48(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmovapd			96(%r11, %r12, 1), %ymm11 // A1
	addq	$ 128, %r11

	// unroll 3
	vbroadcastsd	24(%r13), %ymm12
	vmulpd			%ymm10, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm11, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmovapd			0(%r11), %ymm8 // A0

	vbroadcastsd	56(%r13), %ymm12
	vmulpd			%ymm10, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm11, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmovapd			0(%r11, %r12, 1), %ymm9 // A1
	addq	%r14, %r13

	cmpl	$ 4, %r10d
	jg		1b // main loop 

0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmovapd			32(%r11), %ymm10 // A0

	vbroadcastsd	32(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmovapd			32(%r11, %r12, 1), %ymm11 // A1
	subl	$ 4, %r10d

	// unroll 1
	vbroadcastsd	8(%r13), %ymm12
	vmulpd			%ymm10, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm11, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmovapd			64(%r11), %ymm8 // A0

	vbroadcastsd	40(%r13), %ymm12
	vmulpd			%ymm10, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm11, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmovapd			64(%r11, %r12, 1), %ymm9 // A1

	// unroll 2
	vbroadcastsd	16(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vmovapd			96(%r11), %ymm10 // A0

	vbroadcastsd	48(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	vmovapd			96(%r11, %r12, 1), %ymm11 // A1
	addq	$ 128, %r11

	// unroll 3
	vbroadcastsd	24(%r13), %ymm12
	vmulpd			%ymm10, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm11, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
//	vmovapd			0(%r11), %ymm8 // A0

	vbroadcastsd	56(%r13), %ymm12
	vmulpd			%ymm10, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm11, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
//	vmovapd			0(%r11, %r12, 1), %ymm9 // A1
	addq	%r14, %r13

	jmp		2f


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	vmovapd			0(%r11), %ymm8 // A0[0]
	vmovapd 		0(%r11, %r12, 1), %ymm9 // A1[0]
	vbroadcastsd	0(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2

	vbroadcastsd	32(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3
	addq	$ 32, %r11
	subl	$ 1, %r10d
	addq	$ 8, %r13

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 

2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_dgemm_add_nn_8x2_lib4, .-inner_kernel_dgemm_add_nn_8x2_lib4
#endif
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// r13   <- 4*sdb*sizeof(double)

//
// output arguments:
// r10d  <- 0
// r11   <- A+4*k*sizeof(double)
// r12   <- B+(k/4)*sdb*sizeof(double)+(k%4)
// r13   <- 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_ADD_NN_2X8_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_dgemm_add_nn_2x8_lib4, @function
inner_kernel_dgemm_add_nn_2x8_lib4:
#elif defined(OS_MAC)
_inner_kernel_dgemm_add_nn_2x8_lib4:
#elif defined(OS_WINDOWS)
	.def inner_kernel_dgemm_add_nn_2x8_lib4; .scl 2; .type 32; .endef
inner_kernel_dgemm_add_nn_2x8_lib4:
#endif
#endif
	
	cmpl	$ 0, %r10d
	jle		5f // return

	// preload
	vbroadcastf128	0(%r11), %ymm11 // A
	vbroadcastf128	32(%r11), %ymm12 // A

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	prefetcht0	0(%r12, %r13, 2) // software prefetch
	prefetcht0	64(%r12, %r13, 2) // software prefetch
	prefetcht0	128(%r12, %r13, 2) // software prefetch
	prefetcht0	192(%r12, %r13, 2) // software prefetch

	// unroll 0 1
	vmovapd			0(%r12), %ymm13
	vmovupd			16(%r12), %ymm14
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vshufpd			$ 0x0, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
	vshufpd			$ 0xf, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm12, %ymm13, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	vmovapd			64(%r12), %ymm13
	vmovupd			80(%r12), %ymm14
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vshufpd			$ 0x0, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	vbroadcastf128	64(%r11), %ymm9 // A
	vshufpd			$ 0xf, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm12, %ymm13, %ymm15
	vaddpd			%ymm3, %ymm15, %ymm3
	vmovapd			128(%r12), %ymm13
	vmovupd			144(%r12), %ymm14
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vshufpd			$ 0x0, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm4, %ymm15, %ymm4
	vbroadcastf128	96(%r11), %ymm10 // A
	vshufpd			$ 0xf, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm12, %ymm13, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5
	vmovapd			192(%r12), %ymm13
	vmovupd			208(%r12), %ymm14
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vshufpd			$ 0x0, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6
	vshufpd			$ 0xf, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm12, %ymm13, %ymm15
	vaddpd			%ymm7, %ymm15, %ymm7

	// unroll 2 3
	vmovupd			16(%r12), %ymm13
	vmovapd			32(%r12), %ymm14
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vshufpd			$ 0x0, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm9, %ymm13, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
	vshufpd			$ 0xf, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm10, %ymm13, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	vmovupd			80(%r12), %ymm13
	vmovapd			96(%r12), %ymm14
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vshufpd			$ 0x0, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm9, %ymm13, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	addq	$ 128, %r11
	vshufpd			$ 0xf, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm10, %ymm13, %ymm15
	vaddpd			%ymm3, %ymm15, %ymm3
	vmovupd			144(%r12), %ymm13
	vmovapd			160(%r12), %ymm14
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vshufpd			$ 0x0, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm9, %ymm13, %ymm15
	vaddpd			%ymm4, %ymm15, %ymm4
	vbroadcastf128	0(%r11), %ymm11 // A
	vshufpd			$ 0xf, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm10, %ymm13, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5
	vmovupd			208(%r12), %ymm13
	vmovapd			224(%r12), %ymm14
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vshufpd			$ 0x0, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm9, %ymm13, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6
	vbroadcastf128	32(%r11), %ymm12 // A
	vshufpd			$ 0xf, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm10, %ymm13, %ymm15
	vaddpd			%ymm7, %ymm15, %ymm7

	subl	$ 4, %r10d
	addq	%r13, %r12

	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0 1
	vbroadcastf128	0(%r11), %ymm11 // A
	vbroadcastf128	32(%r11), %ymm12 // A
	vmovapd			0(%r12), %ymm13
	vmovupd			16(%r12), %ymm14
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vshufpd			$ 0x0, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
	vshufpd			$ 0xf, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm12, %ymm13, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	vmovapd			64(%r12), %ymm13
	vmovupd			80(%r12), %ymm14
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vshufpd			$ 0x0, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	vshufpd			$ 0xf, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm12, %ymm13, %ymm15
	vaddpd			%ymm3, %ymm15, %ymm3
	vmovapd			128(%r12), %ymm13
	vmovupd			144(%r12), %ymm14
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vshufpd			$ 0x0, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm4, %ymm15, %ymm4
	vshufpd			$ 0xf, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm12, %ymm13, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5
	vmovapd			192(%r12), %ymm13
	vmovupd			208(%r12), %ymm14
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vshufpd			$ 0x0, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6
	vshufpd			$ 0xf, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm12, %ymm13, %ymm15
	vaddpd			%ymm7, %ymm15, %ymm7

	// unroll 2 3
	vbroadcastf128	64(%r11), %ymm11 // A
	vbroadcastf128	96(%r11), %ymm12 // A
	vmovupd			16(%r12), %ymm13
	vmovapd			32(%r12), %ymm14
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vshufpd			$ 0x0, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
	vshufpd			$ 0xf, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm12, %ymm13, %ymm15
	vaddpd			%ymm1, %ymm15, %ymm1
	vmovupd			80(%r12), %ymm13
	vmovapd			96(%r12), %ymm14
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vshufpd			$ 0x0, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	vshufpd			$ 0xf, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm12, %ymm13, %ymm15
	vaddpd			%ymm3, %ymm15, %ymm3
	vmovupd			144(%r12), %ymm13
	vmovapd			160(%r12), %ymm14
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vshufpd			$ 0x0, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm4, %ymm15, %ymm4
	vshufpd			$ 0xf, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm12, %ymm13, %ymm15
	vaddpd			%ymm5, %ymm15, %ymm5
	vmovupd			208(%r12), %ymm13
	vmovapd			224(%r12), %ymm14
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vshufpd			$ 0x0, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6
	vshufpd			$ 0xf, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm12, %ymm13, %ymm15
	vaddpd			%ymm7, %ymm15, %ymm7

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	%r13, %r12

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vbroadcastf128	0(%r11), %ymm11 // A
	vmovupd			0(%r12), %ymm13
	vmovupd			16(%r12), %ymm14
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vshufpd			$ 0x0, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
	vmovupd			64(%r12), %ymm13
	vmovupd			80(%r12), %ymm14
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vshufpd			$ 0x0, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	vmovupd			128(%r12), %ymm13
	vmovupd			144(%r12), %ymm14
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vshufpd			$ 0x0, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm4, %ymm15, %ymm4
	vmovupd			192(%r12), %ymm13
	vmovupd			208(%r12), %ymm14
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vshufpd			$ 0x0, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6

	addq	$ 32, %r11
	addq	$ 8, %r12
	subl	$ 1, %r10d

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // reduce
	vaddpd			%ymm0, %ymm1, %ymm0
	vextractf128	$ 0x1, %ymm0, %xmm1
	vaddpd			%ymm2, %ymm3, %ymm2
	vextractf128	$ 0x1, %ymm2, %xmm3
	vaddpd			%ymm4, %ymm5, %ymm4
	vextractf128	$ 0x1, %ymm4, %xmm5
	vaddpd			%ymm6, %ymm7, %ymm6
	vextractf128	$ 0x1, %ymm6, %xmm7

5: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_dgemm_add_nn_2x8_lib4, .-inner_kernel_dgemm_add_nn_2x8_lib4
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
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d40 d50 d60 d70]
// ymm3  <- [d41 d51 d61 d71]

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
// ymm2  <- [d40 d50 d60 d70]
// ymm3  <- [d41 d51 d61 d71]


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DGEMM_ADD_NN_8X2_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_edge_dgemm_add_nn_8x2_lib4, @function
inner_edge_dgemm_add_nn_8x2_lib4:
#elif defined(OS_MAC)
_inner_edge_dgemm_add_nn_8x2_lib4:
#elif defined(OS_WINDOWS)
	.def inner_edge_dgemm_add_nn_8x2_lib4; .scl 2; .type 32; .endef
inner_edge_dgemm_add_nn_8x2_lib4:
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
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r12, 1), %ymm9
	vbroadcastsd	0(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm2, %ymm2
	vbroadcastsd	32(%r13), %ymm12
	vmulpd			%ymm8, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm9, %ymm12, %ymm15
	vaddpd			%ymm15, %ymm3, %ymm3

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
	.size	inner_edge_dgemm_add_nn_8x2_lib4, .-inner_edge_dgemm_add_nn_8x2_lib4
#endif
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

//
// output arguments:
// r10   <- k-(4-offB)
// r11   <- A+(4-offB)*bs*sizeof(double)
// r12   <- B-offB+bs*sdb*sizeof(double)
// r13   <- bs*sdb*sizeof(double)
// r14   <- offB


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DGEMM_ADD_NN_2X8_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_edge_dgemm_add_nn_2x8_lib4, @function
inner_edge_dgemm_add_nn_2x8_lib4:
#elif defined(OS_MAC)
_inner_edge_dgemm_add_nn_2x8_lib4:
#elif defined(OS_WINDOWS)
	.def inner_edge_dgemm_add_nn_2x8_lib4; .scl 2; .type 32; .endef
inner_edge_dgemm_add_nn_2x8_lib4:
#endif
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
	vbroadcastf128	0(%r11), %ymm11 // A
	vmovupd			0(%r12), %ymm13
	vmovupd			16(%r12), %ymm14
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vshufpd			$ 0x0, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm0, %ymm15, %ymm0
	vmovupd			64(%r12), %ymm13
	vmovupd			80(%r12), %ymm14
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vshufpd			$ 0x0, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm2, %ymm15, %ymm2
	vmovupd			128(%r12), %ymm13
	vmovupd			144(%r12), %ymm14
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vshufpd			$ 0x0, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm4, %ymm15, %ymm4
	vmovupd			192(%r12), %ymm13
	vmovupd			208(%r12), %ymm14
	vblendpd		$ 0x3, %ymm13, %ymm14, %ymm14
	vshufpd			$ 0x0, %ymm14, %ymm14, %ymm13
	vmulpd			%ymm11, %ymm13, %ymm15
	vaddpd			%ymm6, %ymm15, %ymm6

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

#if defined(OS_LINUX)
	.size	inner_edge_dgemm_add_nn_2x8_lib4, .-inner_edge_dgemm_add_nn_2x8_lib4
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
// r15   <- dirty
// ymm0  <- [d00 d11 d22 d33]
// ymm1  <- [d01 d10 d23 d32]
// ymm2  <- [d40 d51 d62 d73]
// ymm3  <- [d41 d50 d63 d72]
//
// output arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- 4*sdc*sizeof(double)
// r15   <- dirty
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d40 d50 d60 d70]
// ymm3  <- [d41 d51 d61 d71]

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_8X2_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_scale_ab_8x2_lib4, @function
inner_scale_ab_8x2_lib4:
#elif defined(OS_MAC)
_inner_scale_ab_8x2_lib4:
#elif defined(OS_WINDOWS)
	.def inner_scale_ab_8x2_lib4; .scl 2; .type 32; .endef
inner_scale_ab_8x2_lib4:
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

	// alg==1
	vmovapd		0(%r12), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm0, %ymm15, %ymm0
	vmovapd		32(%r12), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm1, %ymm15, %ymm1

	vmovapd		0(%r12, %r13, 1), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm2, %ymm15, %ymm2
	vmovapd		32(%r12, %r13, 1), %ymm15
	vmulpd		%ymm15, %ymm14, %ymm15
	vaddpd		%ymm3, %ymm15, %ymm3

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_scale_ab_8x2_lib4, .-inner_scale_ab_8x2_lib4
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
	.macro INNER_SCALE_AB_2X8_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_scale_ab_2x8_lib4, @function
inner_scale_ab_2x8_lib4:
#elif defined(OS_MAC)
_inner_scale_ab_2x8_lib4:
#elif defined(OS_WINDOWS)
	.def inner_scale_ab_2x8_lib4; .scl 2; .type 32; .endef
inner_scale_ab_2x8_lib4:
#endif
#endif
	
	// alpha
	vmovddup	0(%r10), %xmm15

	vmulpd		%xmm0, %xmm15, %xmm0
	vmulpd		%xmm1, %xmm15, %xmm1
	vmulpd		%xmm2, %xmm15, %xmm2
	vmulpd		%xmm3, %xmm15, %xmm3
	vmulpd		%xmm4, %xmm15, %xmm4
	vmulpd		%xmm5, %xmm15, %xmm5
	vmulpd		%xmm6, %xmm15, %xmm6
	vmulpd		%xmm7, %xmm15, %xmm7

	// beta
	vmovddup	0(%r11), %xmm14

	vxorpd		%xmm15, %xmm15, %xmm15 // 0.0

	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	vmovapd		0(%r12), %xmm15
	vmulpd		%xmm14, %xmm15, %xmm15
	vaddpd		%xmm15, %xmm0, %xmm0
	vmovapd		32(%r12), %xmm15
	vmulpd		%xmm14, %xmm15, %xmm15
	vaddpd		%xmm15, %xmm1, %xmm1
	vmovapd		64(%r12), %xmm15
	vmulpd		%xmm14, %xmm15, %xmm15
	vaddpd		%xmm15, %xmm2, %xmm2
	vmovapd		96(%r12), %xmm15
	vmulpd		%xmm14, %xmm15, %xmm15
	vaddpd		%xmm15, %xmm3, %xmm3
	vmovapd		128(%r12), %xmm15
	vmulpd		%xmm14, %xmm15, %xmm15
	vaddpd		%xmm15, %xmm4, %xmm4
	vmovapd		160(%r12), %xmm15
	vmulpd		%xmm14, %xmm15, %xmm15
	vaddpd		%xmm15, %xmm5, %xmm5
	vmovapd		192(%r12), %xmm15
	vmulpd		%xmm14, %xmm15, %xmm15
	vaddpd		%xmm15, %xmm6, %xmm6
	vmovapd		224(%r12), %xmm15
	vmulpd		%xmm14, %xmm15, %xmm15
	vaddpd		%xmm15, %xmm7, %xmm7

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_scale_ab_2x8_lib4, .-inner_scale_ab_2x8_lib4
#endif
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// r11  <- 4*sdd*sizeof(double)
// r15  <- dirty
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d40 d50 d60 d70]
// ymm3 <- [d41 d51 d61 d71]
//
// output arguments:
// r10  <- D
// r11  <- 4*sdd*sizeof(double)
// r15  <- dirty
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d40 d50 d60 d70]
// ymm3 <- [d41 d51 d61 d71]

#if MACRO_LEVEL>=1
	.macro INNER_STORE_8X2_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_store_8x2_lib4, @function
inner_store_8x2_lib4:
#elif defined(OS_MAC)
_inner_store_8x2_lib4:
#elif defined(OS_WINDOWS)
	.def inner_store_8x2_lib4; .scl 2; .type 32; .endef
inner_store_8x2_lib4:
#endif
#endif
	
	vmovapd %ymm0,  0(%r10)
	vmovapd %ymm1, 32(%r10)

	vmovapd %ymm2,  0(%r10, %r11, 1)
	vmovapd %ymm3, 32(%r10, %r11, 1)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_store_8x2_lib4, .-inner_store_8x2_lib4
#endif
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10   <- D
// ymm0  <- [d00 d10]
// ymm1  <- [d01 d11]
// ymm2  <- [d02 d12]
// ymm3  <- [d03 d13]
// ymm4  <- [d40 d50]
// ymm5  <- [d41 d51]
// ymm6  <- [d42 d52]
// ymm7  <- [d43 d53]
//
// output arguments:
// r10   <- D
// ymm0  <- [d00 d10]
// ymm1  <- [d01 d11]
// ymm2  <- [d02 d12]
// ymm3  <- [d03 d13]
// ymm4  <- [d40 d50]
// ymm5  <- [d41 d51]
// ymm6  <- [d42 d52]
// ymm7  <- [d43 d53]

#if MACRO_LEVEL>=1
	.macro INNER_STORE_2X8_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_store_2x8_lib4, @function
inner_store_2x8_lib4:
#elif defined(OS_MAC)
_inner_store_2x8_lib4:
#elif defined(OS_WINDOWS)
	.def inner_store_2x8_lib4; .scl 2; .type 32; .endef
inner_store_2x8_lib4:
#endif
#endif
	
	vmovapd %xmm0,   0(%r10)
	vmovapd %xmm1,  32(%r10)
	vmovapd %xmm2,  64(%r10)
	vmovapd %xmm3,  96(%r10)

	vmovapd %xmm4, 128(%r10)
	vmovapd %xmm5, 160(%r10)
	vmovapd %xmm6, 192(%r10)
	vmovapd %xmm7, 224(%r10)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_store_2x8_lib4, .-inner_store_2x8_lib4
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
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d40 d50 d60 d70]
// ymm3 <- [d41 d50 d61 d71]
//
// output arguments:
// r10  <- D
// r11  <- 4*sdd*sizeof(double)
// r12d  <- km
// r13d  <- kn
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d40 d50 d60 d70]
// ymm3 <- [d41 d51 d61 d71]

#if MACRO_LEVEL>=1
	.macro INNER_STORE_8X2_VS_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_store_8x2_vs_lib4, @function
inner_store_8x2_vs_lib4:
#elif defined(OS_MAC)
_inner_store_8x2_vs_lib4:
#elif defined(OS_WINDOWS)
	.def inner_store_8x2_vs_lib4; .scl 2; .type 32; .endef
inner_store_8x2_vs_lib4:
#endif
#endif
	
	vcvtsi2sd	%r12d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC03(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC03(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm15

	vmovapd		%ymm0, 0(%r10)
	vmaskmovpd	%ymm2, %ymm15,  0(%r10, %r11, 1)
	cmpl		$ 2, %r13d
	jl			0f // end
	vmovapd		%ymm1, 32(%r10)
	vmaskmovpd	%ymm3, %ymm15, 32(%r10, %r11, 1)

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_store_8x2_vs_lib4, .-inner_store_8x2_vs_lib4
#endif
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10   <- D
// r11   <- m1
// r12   <- n1
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]
//
// output arguments:
// r10   <- D
// r11   <- m1
// r12   <- n1
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm4  <- [d40 d50 d60 d70]
// ymm5  <- [d41 d51 d61 d71]
// ymm6  <- [d42 d52 d62 d72]
// ymm7  <- [d43 d53 d63 d73]

#if MACRO_LEVEL>=1
	.macro INNER_STORE_2X8_VS_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_store_2x8_vs_lib4, @function
inner_store_2x8_vs_lib4:
#elif defined(OS_MAC)
_inner_store_2x8_vs_lib4:
#elif defined(OS_WINDOWS)
	.def inner_store_2x8_vs_lib4; .scl 2; .type 32; .endef
inner_store_2x8_vs_lib4:
#endif
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

	vmaskmovpd	%xmm0, %xmm15,   0(%r10)
	vmaskmovpd	%xmm1, %xmm15,  32(%r10)
	vmaskmovpd	%xmm2, %xmm15,  64(%r10)
	vmaskmovpd	%xmm3, %xmm15,  96(%r10)

	vmaskmovpd	%xmm4, %xmm15, 128(%r10)
	cmpl		$ 6, %r12d
	jl			0f // end
	vmaskmovpd	%xmm5, %xmm15, 160(%r10)
	cmpl		$ 7, %r12d
	jl			0f // end
	vmaskmovpd	%xmm6, %xmm15, 192(%r10)
	je			0f // end
	vmaskmovpd	%xmm7, %xmm15, 224(%r10)

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_store_2x8_vs_lib4, .-inner_store_2x8_vs_lib4
#endif
#endif





//                               1      2              3          4        5            6          7        8             9          10       11         12
// void kernel_dgemm_nn_8x2_lib4(int k, double *alpha, double *A, int sda, int offsetB, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dgemm_nn_8x2_lib4
	.type kernel_dgemm_nn_8x2_lib4, @function
kernel_dgemm_nn_8x2_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dgemm_nn_8x2_lib4
_kernel_dgemm_nn_8x2_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dgemm_nn_8x2_lib4
	.def kernel_dgemm_nn_8x2_lib4; .scl 2; .type 32; .endef
kernel_dgemm_nn_8x2_lib4:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3


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
	INNER_EDGE_DGEMM_ADD_NN_8X2_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_dgemm_add_nn_8x2_lib4
#elif defined(OS_MAC)
	callq _inner_edge_dgemm_add_nn_8x2_lib4
#endif
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_ADD_NN_8X2_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_dgemm_add_nn_8x2_lib4
#elif defined(OS_MAC)
	callq _inner_kernel_dgemm_add_nn_8x2_lib4
#endif
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12 // C
	movq	ARG10, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X2_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_scale_ab_8x2_lib4
#elif defined(OS_MAC)
	callq _inner_scale_ab_8x2_lib4
#endif
#endif


	// store n

	movq	ARG11, %r10 // D
	movq	ARG12, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_8X2_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_8x2_lib4
#elif defined(OS_MAC)
	callq _inner_store_8x2_lib4
#endif
#endif


	EPILOGUE
	
	ret

#if defined(OS_LINUX)
	.size	kernel_dgemm_nn_8x2_lib4, .-kernel_dgemm_nn_8x2_lib4
#endif





//                                  1      2              3          4        5            6          7        8             9          10       11         12       13      14
// void kernel_dgemm_nn_8x2_vs_lib4(int k, double *alpha, double *A, int sda, int offsetB, double *B, int sdb, double *beta, double *C, int sdc, double *D, int sdd, int m1, int n1);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dgemm_nn_8x2_vs_lib4
	.type kernel_dgemm_nn_8x2_vs_lib4, @function
kernel_dgemm_nn_8x2_vs_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dgemm_nn_8x2_vs_lib4
_kernel_dgemm_nn_8x2_vs_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dgemm_nn_8x2_vs_lib4
	.def kernel_dgemm_nn_8x2_vs_lib4; .scl 2; .type 32; .endef
kernel_dgemm_nn_8x2_vs_lib4:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3


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
	INNER_EDGE_DGEMM_ADD_NN_8X2_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_dgemm_add_nn_8x2_lib4
#elif defined(OS_MAC)
	callq _inner_edge_dgemm_add_nn_8x2_lib4
#endif
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_ADD_NN_8X2_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_dgemm_add_nn_8x2_lib4
#elif defined(OS_MAC)
	callq _inner_kernel_dgemm_add_nn_8x2_lib4
#endif
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12 // C
	movq	ARG10, %r13 // sdc
	sall	$ 5, %r13d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X2_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_scale_ab_8x2_lib4
#elif defined(OS_MAC)
	callq _inner_scale_ab_8x2_lib4
#endif
#endif


	// store n

	movq	ARG11, %r10 // D
	movq	ARG12, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG13, %r12 // m1
	movq	ARG14, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_8X2_VS_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_8x2_vs_lib4
#elif defined(OS_MAC)
	callq _inner_store_8x2_vs_lib4
#endif
#endif


	EPILOGUE
	
	ret

#if defined(OS_LINUX)
	.size	kernel_dgemm_nn_8x2_vs_lib4, .-kernel_dgemm_nn_8x2_vs_lib4
#endif





//                               1      2              3          4            5          6        7             8          9
// void kernel_dgemm_nn_2x8_lib4(int k, double *alpha, double *A, int offsetB, double *B, int sdb, double *beta, double *C, double *D);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dgemm_nn_2x8_lib4
	.type kernel_dgemm_nn_2x8_lib4, @function
kernel_dgemm_nn_2x8_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dgemm_nn_2x8_lib4
_kernel_dgemm_nn_2x8_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dgemm_nn_2x8_lib4
	.def kernel_dgemm_nn_2x8_lib4; .scl 2; .type 32; .endef
kernel_dgemm_nn_2x8_lib4:
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


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG5, %r12  // B
	movq	ARG6, %r13 // sdb
	sall	$ 5, %r13d // 4*sdb*sizeof(double)
	movq	ARG4, %r14 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_DGEMM_ADD_NN_2X8_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_dgemm_add_nn_2x8_lib4
#elif defined(OS_MAC)
	callq _inner_edge_dgemm_add_nn_2x8_lib4
#endif
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_ADD_NN_2X8_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_dgemm_add_nn_2x8_lib4
#elif defined(OS_MAC)
	callq _inner_kernel_dgemm_add_nn_2x8_lib4
#endif
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_2X8_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_scale_ab_2x8_lib4
#elif defined(OS_MAC)
	callq _inner_scale_ab_2x8_lib4
#endif
#endif


	// store n

	movq	ARG9, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_2X8_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_2x8_lib4
#elif defined(OS_MAC)
	callq _inner_store_2x8_lib4
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dgemm_nn_2x8_lib4, .-kernel_dgemm_nn_2x8_lib4
#endif





//                                  1      2              3          4            5          6        7             8          9          10      11
// void kernel_dgemm_nn_2x8_vs_lib4(int k, double *alpha, double *A, int offsetB, double *B, int sdb, double *beta, double *C, double *D, int m1, int n1);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dgemm_nn_2x8_vs_lib4
	.type kernel_dgemm_nn_2x8_vs_lib4, @function
kernel_dgemm_nn_2x8_vs_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dgemm_nn_2x8_vs_lib4
_kernel_dgemm_nn_2x8_vs_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dgemm_nn_2x8_vs_lib4
	.def kernel_dgemm_nn_2x8_vs_lib4; .scl 2; .type 32; .endef
kernel_dgemm_nn_2x8_vs_lib4:
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


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG5, %r12  // B
	movq	ARG6, %r13 // sdb
	sall	$ 5, %r13d // 4*sdb*sizeof(double)
	movq	ARG4, %r14 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_DGEMM_ADD_NN_2X8_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_dgemm_add_nn_2x8_lib4
#elif defined(OS_MAC)
	callq _inner_edge_dgemm_add_nn_2x8_lib4
#endif
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_ADD_NN_2X8_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_dgemm_add_nn_2x8_lib4
#elif defined(OS_MAC)
	callq _inner_kernel_dgemm_add_nn_2x8_lib4
#endif
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_2X8_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_scale_ab_2x8_lib4
#elif defined(OS_MAC)
	callq _inner_scale_ab_2x8_lib4
#endif
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // m1
	movq	ARG11, %r12 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_2X8_VS_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_2x8_vs_lib4
#elif defined(OS_MAC)
	callq _inner_store_2x8_vs_lib4
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dgemm_nn_2x8_vs_lib4, .-kernel_dgemm_nn_2x8_vs_lib4
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
.LC05: // { 1.0 1.0 1.0 -1.0 }
#elif defined(OS_MAC)
	.align 5
LC05: // { 1.0 1.0 1.0 -1.0 }
#endif
	.long	0
	.long	-1074790400
	.long	0
	.long	1072693248
	.long	0
	.long	1072693248
	.long	0
	.long	1072693248

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC06: // { 1.0 1.0 -1.0 -1.0 }
#elif defined(OS_MAC)
	.align 5
LC06: // { 1.0 1.0 -1.0 -1.0 }
#endif
	.long	0
	.long	-1074790400
	.long	0
	.long	-1074790400
	.long	0
	.long	1072693248
	.long	0
	.long	1072693248

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC07: // { 1.0 -1.0 -1.0 -1.0 }
#elif defined(OS_MAC)
	.align 5
LC07: // { 1.0 -1.0 -1.0 -1.0 }
#endif
	.long	0
	.long	-1074790400
	.long	0
	.long	-1074790400
	.long	0
	.long	-1074790400
	.long	0
	.long	1072693248

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC08: // { -1.0 -1.0 -1.0 1.0 }
#elif defined(OS_MAC)
	.align 5
LC08: // { -1.0 -1.0 -1.0 1.0 }
#endif
	.long	0
	.long	1072693248
	.long	0
	.long	-1074790400
	.long	0
	.long	-1074790400
	.long	0
	.long	-1074790400

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC09: // { -1.0 -1.0 1.0 1.0 }
#elif defined(OS_MAC)
	.align 5
LC09: // { -1.0 -1.0 1.0 1.0 }
#endif
	.long	0
	.long	1072693248
	.long	0
	.long	1072693248
	.long	0
	.long	-1074790400
	.long	0
	.long	-1074790400

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC10: // { -1.0 1.0 1.0 1.0 }
#elif defined(OS_MAC)
	.align 5
LC10: // { -1.0 1.0 1.0 1.0 }
#endif
	.long	0
	.long	1072693248
	.long	0
	.long	1072693248
	.long	0
	.long	1072693248
	.long	0
	.long	-1074790400




#if defined(OS_LINUX)
	.section	.note.GNU-stack,"",@progbits
#elif defined(OS_MAC)
	.subsections_via_symbols
#endif


