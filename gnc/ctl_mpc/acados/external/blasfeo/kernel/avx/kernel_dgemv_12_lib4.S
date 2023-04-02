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
// r12   <- 4*sda*sizeof(double)
// r13   <- x
// r14   <- dirty
// r15   <- dirty
// ymm0  <- [z0 z1 z2 z3]_a
// ymm1  <- [z4 z5 z6 z7]_a
// ymm2  <- [z8 z9 za zb]_a
// ymm3  <- [z0 z1 z2 z3]_b
// ymm4  <- [z4 z5 z6 z7]_b
// ymm5  <- [z8 z9 za zb]_b
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
// r12   <- 4*sda*sizeof(double)
// r13   <- x+k*sizeof(double)
// r14   <- dirty
// r15   <- dirty
// ymm0  <- [z0 z1 z2 z3]_a
// ymm1  <- [z4 z5 z6 z7]_a
// ymm2  <- [z8 z9 za zb]_a
// ymm3  <- [z0 z1 z2 z3]_b
// ymm4  <- [z4 z5 z6 z7]_b
// ymm5  <- [z8 z9 za zb]_b
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMV_ADD_N_12_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemv_add_n_12_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	movq	%r11, %r14 // A1 <- A0
	addq	%r12, %r14 // A1 <- A0 + 4*sda*sizeof(double)
	movq	%r14, %r15 // A2 <- A1
	addq	%r12, %r15 // A2 <- A1 + 4*sda*sizeof(double)

	cmpl	$ 4, %r10d

	prefetcht0	0(%r11) // software prefetch
	prefetcht0	0(%r14) // software prefetch
	prefetcht0	0(%r15) // software prefetch
	prefetcht0	64(%r11) // software prefetch
	prefetcht0	64(%r14) // software prefetch
	prefetcht0	64(%r15) // software prefetch

	jl		0f // clean-up loop

	// main loop
	.p2align 3
1: // main loop
	
	prefetcht0	128(%r11) // software prefetch
	prefetcht0	128(%r14) // software prefetch
	prefetcht0	128(%r15) // software prefetch

	vbroadcastsd	0(%r13), %ymm12
	vmovapd	0(%r11), %ymm8
	vmulpd	%ymm8, %ymm12, %ymm15
	vaddpd	%ymm0, %ymm15, %ymm0
	vmovapd	0(%r14), %ymm8
	vmulpd	%ymm8, %ymm12, %ymm15
	vaddpd	%ymm1, %ymm15, %ymm1
	vmovapd	0(%r15), %ymm8
	vmulpd	%ymm8, %ymm12, %ymm15
	vaddpd	%ymm2, %ymm15, %ymm2
	
	subl	$ 4, %r10d

	vbroadcastsd	8(%r13), %ymm12
	vmovapd	32(%r11), %ymm8
	vmulpd	%ymm8, %ymm12, %ymm15
	vaddpd	%ymm3, %ymm15, %ymm3
	vmovapd	32(%r14), %ymm8
	vmulpd	%ymm8, %ymm12, %ymm15
	vaddpd	%ymm4, %ymm15, %ymm4
	vmovapd	32(%r15), %ymm8
	vmulpd	%ymm8, %ymm12, %ymm15
	vaddpd	%ymm5, %ymm15, %ymm5
	
	prefetcht0	192(%r11) // software prefetch
	prefetcht0	192(%r14) // software prefetch
	prefetcht0	192(%r15) // software prefetch

	vbroadcastsd	16(%r13), %ymm12
	vmovapd	64(%r11), %ymm8
	vmulpd	%ymm8, %ymm12, %ymm15
	vaddpd	%ymm0, %ymm15, %ymm0
	vmovapd	64(%r14), %ymm8
	vmulpd	%ymm8, %ymm12, %ymm15
	vaddpd	%ymm1, %ymm15, %ymm1
	vmovapd	64(%r15), %ymm8
	vmulpd	%ymm8, %ymm12, %ymm15
	vaddpd	%ymm2, %ymm15, %ymm2

	vbroadcastsd	24(%r13), %ymm12
	addq	$ 32, %r13 // x+4
	vmovapd	96(%r11), %ymm8
	addq	$ 128, %r11 // A0+4*bs
	vmulpd	%ymm8, %ymm12, %ymm15
	vaddpd	%ymm3, %ymm15, %ymm3
	vmovapd	96(%r14), %ymm8
	addq	$ 128, %r14 // A1+4*bs
	vmulpd	%ymm8, %ymm12, %ymm15
	vaddpd	%ymm4, %ymm15, %ymm4
	vmovapd	96(%r15), %ymm8
	addq	$ 128, %r15 // A2+4*bs
	vmulpd	%ymm8, %ymm12, %ymm15
	vaddpd	%ymm5, %ymm15, %ymm5
	
	cmpl	$ 3, %r10d

	jg		1b // main loop 


	// consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

0: // clean-up
	
	vbroadcastsd	0(%r13), %ymm12
	vmovapd	0(%r11), %ymm8
	vmulpd	%ymm8, %ymm12, %ymm15
	vaddpd	%ymm0, %ymm15, %ymm0
	vmovapd	0(%r14), %ymm8
	vmulpd	%ymm8, %ymm12, %ymm15
	vaddpd	%ymm1, %ymm15, %ymm1
	vmovapd	0(%r15), %ymm8
	vmulpd	%ymm8, %ymm12, %ymm15
	vaddpd	%ymm2, %ymm15, %ymm2
	
	addq	$ 32, %r11
	addq	$ 32, %r14
	addq	$ 32, %r15
	addq	$ 8, %r13
	
	subl	$ 1, %r10d
	cmpl	$ 0, %r10d

	jg		0b // clean

2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemv_add_n_12_lib4)
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
// ymm4  <- [z4a z4b z4c z4d]
// ymm5  <- [z5a z5b z5c z5d]
// ymm6  <- [z6a z6b z6c z6d]
// ymm7  <- [z7a z7b z7c z7d]
// ymm8  <- [z8a z8b z8c z8d]
// ymm9  <- [z9a z9b z9c z9d]
// ymm10 <- [zaa zab zac zad]
// ymm11 <- [zba zbb zbc zbd]
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
// r14   <- dirty
// ymm0  <- [z0a z0b z0c z0d]
// ymm1  <- [z1a z1b z1c z1d]
// ymm2  <- [z2a z2b z2c z2d]
// ymm3  <- [z3a z3b z3c z3d]
// ymm4  <- [z4a z4b z4c z4d]
// ymm5  <- [z5a z5b z5c z5d]
// ymm6  <- [z6a z6b z6c z6d]
// ymm7  <- [z7a z7b z7c z7d]
// ymm8  <- [z8a z8b z8c z8d]
// ymm9  <- [z9a z9b z9c z9d]
// ymm10 <- [zaa zab zac zad]
// ymm11 <- [zba zbb zbc zbd]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMV_ADD_T_12_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemv_add_t_12_lib4)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	cmpl	$ 4, %r10d

	prefetcht0	0(%r11) // software prefetch
	prefetcht0	64(%r11) // software prefetch
	prefetcht0	128(%r11) // software prefetch
	prefetcht0	192(%r11) // software prefetch
	prefetcht0	256(%r11) // software prefetch
	prefetcht0	320(%r11) // software prefetch

	jl		0f // clean-up loop

	movq	%r11, %r14
	addq	%r12, %r14 // A+bs*sda

	// main loop
	.p2align 3
1: // main loop
	
	prefetcht0	0(%r14) // software prefetch

	vmovupd	0(%r13), %ymm12
	addq	$ 32, %r13 // x+4

	vmovapd	0(%r11), %ymm13
	vmulpd	%ymm13, %ymm12, %ymm15
	vaddpd	%ymm0, %ymm15, %ymm0
	
	subl	$ 4, %r10d

	vmovapd	32(%r11), %ymm13
	vmulpd	%ymm13, %ymm12, %ymm15
	vaddpd	%ymm1, %ymm15, %ymm1
	
	prefetcht0	64(%r14) // software prefetch

	vmovapd	64(%r11), %ymm13
	vmulpd	%ymm13, %ymm12, %ymm15
	vaddpd	%ymm2, %ymm15, %ymm2

	vmovapd	96(%r11), %ymm13
	vmulpd	%ymm13, %ymm12, %ymm15
	vaddpd	%ymm3, %ymm15, %ymm3

	prefetcht0	128(%r14) // software prefetch

	vmovapd	128(%r11), %ymm13
	vmulpd	%ymm13, %ymm12, %ymm15
	vaddpd	%ymm4, %ymm15, %ymm4
	
	vmovapd	160(%r11), %ymm13
	vmulpd	%ymm13, %ymm12, %ymm15
	vaddpd	%ymm5, %ymm15, %ymm5
	
	prefetcht0	192(%r14) // software prefetch

	vmovapd	192(%r11), %ymm13
	vmulpd	%ymm13, %ymm12, %ymm15
	vaddpd	%ymm6, %ymm15, %ymm6

	vmovapd	224(%r11), %ymm13
	vmulpd	%ymm13, %ymm12, %ymm15
	vaddpd	%ymm7, %ymm15, %ymm7

	prefetcht0	256(%r14) // software prefetch

	vmovapd	256(%r11), %ymm13
	vmulpd	%ymm13, %ymm12, %ymm15
	vaddpd	%ymm8, %ymm15, %ymm8
	
	vmovapd	288(%r11), %ymm13
	vmulpd	%ymm13, %ymm12, %ymm15
	vaddpd	%ymm9, %ymm15, %ymm9
	
	prefetcht0	320(%r14) // software prefetch

	vmovapd	320(%r11), %ymm13
	vmulpd	%ymm13, %ymm12, %ymm15
	vaddpd	%ymm10, %ymm15, %ymm10

	vmovapd	352(%r11), %ymm13
	vmulpd	%ymm13, %ymm12, %ymm15
	vaddpd	%ymm11, %ymm15, %ymm11
	
//	addq	%r12, %r11 // A+bs*sda
	movq	%r14, %r11 // A+bs*sda
	addq	%r12, %r14 // A+bs*sda+bs*sda
	
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

	vmovapd	0(%r11), %ymm13
	vmulpd	%ymm13, %ymm12, %ymm15
	vaddpd	%ymm0, %ymm15, %ymm0
	
	vmovapd	32(%r11), %ymm13
	vmulpd	%ymm13, %ymm12, %ymm15
	vaddpd	%ymm1, %ymm15, %ymm1
	
	vmovapd	64(%r11), %ymm13
	vmulpd	%ymm13, %ymm12, %ymm15
	vaddpd	%ymm2, %ymm15, %ymm2

	vmovapd	96(%r11), %ymm13
	vmulpd	%ymm13, %ymm12, %ymm15
	vaddpd	%ymm3, %ymm15, %ymm3
		
	vmovapd	128(%r11), %ymm13
	vmulpd	%ymm13, %ymm12, %ymm15
	vaddpd	%ymm4, %ymm15, %ymm4
	
	vmovapd	160(%r11), %ymm13
	vmulpd	%ymm13, %ymm12, %ymm15
	vaddpd	%ymm5, %ymm15, %ymm5
	
	vmovapd	192(%r11), %ymm13
	vmulpd	%ymm13, %ymm12, %ymm15
	vaddpd	%ymm6, %ymm15, %ymm6

	vmovapd	224(%r11), %ymm13
	vmulpd	%ymm13, %ymm12, %ymm15
	vaddpd	%ymm7, %ymm15, %ymm7

	vmovapd	256(%r11), %ymm13
	vmulpd	%ymm13, %ymm12, %ymm15
	vaddpd	%ymm8, %ymm15, %ymm8
	
	vmovapd	288(%r11), %ymm13
	vmulpd	%ymm13, %ymm12, %ymm15
	vaddpd	%ymm9, %ymm15, %ymm9
	
	vmovapd	320(%r11), %ymm13
	vmulpd	%ymm13, %ymm12, %ymm15
	vaddpd	%ymm10, %ymm15, %ymm10

	vmovapd	352(%r11), %ymm13
	vmulpd	%ymm13, %ymm12, %ymm15
	vaddpd	%ymm11, %ymm15, %ymm11

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

	FUN_END(inner_kernel_dgemv_add_t_12_lib4)
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
	.macro INNER_EDGE_GEMV_ADD_T_12_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_gemv_add_t_12_lib4)
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

	vmovapd	0(%r11), %ymm13
	vmulpd	%ymm13, %ymm12, %ymm15
	vaddpd	%ymm0, %ymm15, %ymm0
	
	vmovapd	32(%r11), %ymm13
	vmulpd	%ymm13, %ymm12, %ymm15
	vaddpd	%ymm1, %ymm15, %ymm1
	
	vmovapd	64(%r11), %ymm13
	vmulpd	%ymm13, %ymm12, %ymm15
	vaddpd	%ymm2, %ymm15, %ymm2

	vmovapd	96(%r11), %ymm13
	vmulpd	%ymm13, %ymm12, %ymm15
	vaddpd	%ymm3, %ymm15, %ymm3
		
	vmovapd	128(%r11), %ymm13
	vmulpd	%ymm13, %ymm12, %ymm15
	vaddpd	%ymm4, %ymm15, %ymm4
	
	vmovapd	160(%r11), %ymm13
	vmulpd	%ymm13, %ymm12, %ymm15
	vaddpd	%ymm5, %ymm15, %ymm5
	
	vmovapd	192(%r11), %ymm13
	vmulpd	%ymm13, %ymm12, %ymm15
	vaddpd	%ymm6, %ymm15, %ymm6

	vmovapd	224(%r11), %ymm13
	vmulpd	%ymm13, %ymm12, %ymm15
	vaddpd	%ymm7, %ymm15, %ymm7

	vmovapd	256(%r11), %ymm13
	vmulpd	%ymm13, %ymm12, %ymm15
	vaddpd	%ymm8, %ymm15, %ymm8
	
	vmovapd	288(%r11), %ymm13
	vmulpd	%ymm13, %ymm12, %ymm15
	vaddpd	%ymm9, %ymm15, %ymm9
	
	vmovapd	320(%r11), %ymm13
	vmulpd	%ymm13, %ymm12, %ymm15
	vaddpd	%ymm10, %ymm15, %ymm10

	vmovapd	352(%r11), %ymm13
	vmulpd	%ymm13, %ymm12, %ymm15
	vaddpd	%ymm11, %ymm15, %ymm11

	addq	$ 32, %r13 // x + 4
	addq	%r12, %r11 // A + bs*sda
		
	addl	%r14d, %r10d
	subl	$ 4, %r10d // kmax - (4-offA)
	
0: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_edge_gemv_add_t_12_lib4)
#endif





// common inner routine with file scope
//
// blend for ta==n, scale for generic alpha and beta
//
// input arguments:
// r10  <- alpha
// r11  <- beta
// r12  <- y
// ymm0  <- [z0 z1 z2 z3]_a
// ymm1  <- [z4 z5 z6 z7]_a
// ymm2  <- [z8 z9 za zb]_a
// ymm3  <- [z0 z1 z2 z3]_b
// ymm4  <- [z4 z5 z6 z7]_b
// ymm5  <- [z8 z9 za zb]_b
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
// ymm1 <- [z4 z5 z6 z7]
// ymm2  <- [z8 z9 za zb]
// ymm3 <- dirty
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_N_SCALE_AB_12_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_blend_n_scale_ab_12_lib4)
#endif

	// reduction
	vaddpd	%ymm0, %ymm3, %ymm0
	vaddpd	%ymm1, %ymm4, %ymm1
	vaddpd	%ymm2, %ymm5, %ymm2

	// alpha
	vbroadcastsd	0(%r10), %ymm15
	vmulpd	%ymm0, %ymm15, %ymm0
	vmulpd	%ymm1, %ymm15, %ymm1
	vmulpd	%ymm2, %ymm15, %ymm2

	// beta
	vbroadcastsd	0(%r11), %ymm15

	vxorpd		%ymm14, %ymm14, %ymm14 // 0.0
	vucomisd	%xmm14, %xmm15 // beta==0.0 ?
	je			0f // end

	vmovupd		0(%r12), %ymm14
	vmulpd		%ymm15, %ymm14, %ymm14
	vaddpd		%ymm0, %ymm14, %ymm0
	vmovupd		32(%r12), %ymm14
	vmulpd		%ymm15, %ymm14, %ymm14
	vaddpd		%ymm1, %ymm14, %ymm1
	vmovupd		64(%r12), %ymm14
	vmulpd		%ymm15, %ymm14, %ymm14
	vaddpd		%ymm2, %ymm14, %ymm2

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret
	
	FUN_END(inner_blend_n_scale_ab_12_lib4)
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
// ymm4 <- [z4a z4b z4c z4d]
// ymm5 <- [z5a z5b z5c z5d]
// ymm6 <- [z6a z6b z6c z6d]
// ymm7 <- [z7a z7b z7c z7d]
// ymm8  <- [z8a z8b z8c z8d]
// ymm9  <- [z9a z9b z9c z9d]
// ymm10 <- [zaa zab zac zad]
// ymm11 <- [zba zbb zbc zbd]
// ymm15 <- dirty
//
// output arguments:
// r10  <- alpha
// r11  <- beta
// r12  <- y
// ymm0 <- [z0 z1 z2 z3]
// ymm1 <- [z4 z5 z6 z7]
// ymm2  <- [z8 z9 za zb]
// ymm3 <- dirty
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_T_SCALE_AB_12_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_blend_t_scale_ab_12_lib4)
#endif

	// reduction
	vhaddpd	%ymm1, %ymm0, %ymm0
	vhaddpd	%ymm5, %ymm4, %ymm4
	vhaddpd	%ymm9, %ymm8, %ymm8
	vhaddpd	%ymm3, %ymm2, %ymm2
	vhaddpd	%ymm7, %ymm6, %ymm6
	vhaddpd	%ymm11, %ymm10, %ymm10
	vperm2f128	$ 0x2, %ymm0, %ymm2, %ymm3
	vperm2f128	$ 0x2, %ymm4, %ymm6, %ymm5
	vperm2f128	$ 0x2, %ymm8, %ymm10, %ymm9
	vperm2f128	$ 0x13, %ymm0, %ymm2, %ymm0
	vperm2f128	$ 0x13, %ymm4, %ymm6, %ymm4
	vperm2f128	$ 0x13, %ymm8, %ymm10, %ymm8
	vaddpd	%ymm0, %ymm3, %ymm0
	vaddpd	%ymm4, %ymm5, %ymm1
	vaddpd	%ymm8, %ymm9, %ymm2

	// alpha
	vbroadcastsd	0(%r10), %ymm15
	vmulpd	%ymm0, %ymm15, %ymm0
	vmulpd	%ymm1, %ymm15, %ymm1
	vmulpd	%ymm2, %ymm15, %ymm2

	// beta
	vbroadcastsd	0(%r11), %ymm15

	vxorpd		%ymm14, %ymm14, %ymm14 // 0.0
	vucomisd	%xmm14, %xmm15 // beta==0.0 ?
	je			0f // end

	vmovupd		0(%r12), %ymm14
	vmulpd		%ymm15, %ymm14, %ymm14
	vaddpd		%ymm0, %ymm14, %ymm0
	vmovupd		32(%r12), %ymm14
	vmulpd		%ymm15, %ymm14, %ymm14
	vaddpd		%ymm1, %ymm14, %ymm1
	vmovupd		64(%r12), %ymm14
	vmulpd		%ymm15, %ymm14, %ymm14
	vaddpd		%ymm2, %ymm14, %ymm2

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret
	
	FUN_END(inner_blend_t_scale_ab_12_lib4)
#endif





// common inner routine with file scope
//
// blender for ta==n
//
// input arguments:
// r10d <- alg
// r11   <- y
// ymm0  <- [z0 z1 z2 z3]_a
// ymm1  <- [z4 z5 z6 z7]_a
// ymm2  <- [z8 z9 za zb]_a
// ymm3  <- [z0 z1 z2 z3]_b
// ymm4  <- [z4 z5 z6 z7]_b
// ymm5  <- [z8 z9 za zb]_b
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10d <- alg
// r11   <- y
// ymm0  <- [z0 z1 z2 z3]
// ymm1  <- [z4 z5 z6 z7]
// ymm2  <- [z8 z9 za zb]
// ymm3  <- dirty
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_BLENDER_N_12_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_blender_n_12_lib4)
#endif

	// reduction
	vaddpd	%ymm0, %ymm3, %ymm0
	vaddpd	%ymm1, %ymm4, %ymm1
	vaddpd	%ymm2, %ymm5, %ymm2

	cmpl	$ 0, %r10d // alg
	je		0f // return

	cmpl	$ 1, %r10d // alg
	jne		1f // alg==-1

	// alg==1
	vmovupd		0(%r11), %ymm15
	vaddpd		%ymm0, %ymm15, %ymm0
	vmovupd		32(%r11), %ymm15
	vaddpd		%ymm1, %ymm15, %ymm1
	vmovupd		64(%r11), %ymm15
	vaddpd		%ymm2, %ymm15, %ymm2

	jmp		0f // return

1:

	// alg==-1
	vmovupd		0(%r11), %ymm15
	vsubpd		%ymm0, %ymm15, %ymm0
	vmovupd		32(%r11), %ymm15
	vsubpd		%ymm1, %ymm15, %ymm1
	vmovupd		64(%r11), %ymm15
	vsubpd		%ymm2, %ymm15, %ymm2

0: // return
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret
	
	FUN_END(inner_blender_n_12_lib4)
#endif





// common inner routine with file scope
//
// blender for ta==t
//
// input arguments:
// r10d <- alg
// r11   <- y
// ymm0  <- [z0a z0b z0c z0d]
// ymm1  <- [z1a z1b z1c z1d]
// ymm2  <- [z2a z2b z2c z2d]
// ymm3  <- [z3a z3b z3c z3d]
// ymm4  <- [z4a z4b z4c z4d]
// ymm5  <- [z5a z5b z5c z5d]
// ymm6  <- [z6a z6b z6c z6d]
// ymm7  <- [z7a z7b z7c z7d]
// ymm8  <- [z8a z8b z8c z8d]
// ymm9  <- [z9a z9b z9c z9d]
// ymm10 <- [zaa zab zac zad]
// ymm11 <- [zba zbb zbc zbd]
// ymm15 <- dirty
//
// output arguments:
// r10d <- alg
// r11   <- y
// ymm0  <- [z0 z1 z2 z3]
// ymm1  <- [z4 z5 z6 z7]
// ymm2  <- [z8 z9 za zb]
// ymm3  <- dirty
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_BLENDER_T_12_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_blender_t_12_lib4)
#endif

	// reduction
	vhaddpd	%ymm1, %ymm0, %ymm0
	vhaddpd	%ymm5, %ymm4, %ymm4
	vhaddpd	%ymm9, %ymm8, %ymm8
	vhaddpd	%ymm3, %ymm2, %ymm2
	vhaddpd	%ymm7, %ymm6, %ymm6
	vhaddpd	%ymm11, %ymm10, %ymm10
	vperm2f128	$ 0x2, %ymm0, %ymm2, %ymm3
	vperm2f128	$ 0x2, %ymm4, %ymm6, %ymm5
	vperm2f128	$ 0x2, %ymm8, %ymm10, %ymm9
	vperm2f128	$ 0x13, %ymm0, %ymm2, %ymm0
	vperm2f128	$ 0x13, %ymm4, %ymm6, %ymm4
	vperm2f128	$ 0x13, %ymm8, %ymm10, %ymm8
	vaddpd	%ymm0, %ymm3, %ymm0
	vaddpd	%ymm4, %ymm5, %ymm1
	vaddpd	%ymm8, %ymm9, %ymm2

	cmpl	$ 0, %r10d // alg
	je		0f // return

	cmpl	$ 1, %r10d // alg
	jne		1f // alg==-1

	// alg==1
	vmovupd		0(%r11), %ymm15
	vaddpd		%ymm0, %ymm15, %ymm0
	vmovupd		32(%r11), %ymm15
	vaddpd		%ymm1, %ymm15, %ymm1
	vmovupd		64(%r11), %ymm15
	vaddpd		%ymm2, %ymm15, %ymm2

	jmp		0f // return

1:

	// alg==-1
	vmovupd		0(%r11), %ymm15
	vsubpd		%ymm0, %ymm15, %ymm0
	vmovupd		32(%r11), %ymm15
	vsubpd		%ymm1, %ymm15, %ymm1
	vmovupd		64(%r11), %ymm15
	vsubpd		%ymm2, %ymm15, %ymm2

0: // return
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret
	
	FUN_END(inner_blender_t_12_lib4)
#endif





// common inner routine with file scope
//
// store 
//
// input arguments:
// r10  <- z
// ymm0 <- [z0 z1 z2 z3]
// ymm1 <- [z4 z5 z6 z7]
//
// output arguments:
// r10  <- z
// ymm0 <- [z0 z1 z2 z3]
// ymm1 <- [z4 z5 z6 z7]

#if MACRO_LEVEL>=1
	.macro INNER_STORE_12_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_12_lib4)
#endif
	
	vmovupd %ymm0, 0(%r10)
	vmovupd %ymm1, 32(%r10)
	vmovupd %ymm2, 64(%r10)
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_12_lib4)
#endif





//                             rdi    rsi            rdx        rcx      r8         r9            rsp+8      rsp+16
// void kernel_dgemv_n_12_lib4(int k, double *alpha, double *A, int sda, double *x, double *beta, double *y, double *z);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemv_n_12_lib4)
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%ymm0, %ymm0, %ymm0
	vmovapd	%ymm0, %ymm1
	vmovapd	%ymm0, %ymm2
	vmovapd	%ymm0, %ymm3
	vmovapd	%ymm0, %ymm4
	vmovapd	%ymm0, %ymm5


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
//	movslq	%r12d, %r12
	movq	ARG5, %r13  // x

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMV_ADD_N_12_LIB4
#else
	CALL(inner_kernel_dgemv_add_n_12_lib4)
#endif


	// call inner blender n

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11   // beta
	movq	ARG7, %r12 // y

#if MACRO_LEVEL>=1
	INNER_BLEND_N_SCALE_AB_12_LIB4
#else
	CALL(inner_blend_n_scale_ab_12_lib4)
#endif


	// store

	movq	ARG8, %r10 // z

#if MACRO_LEVEL>=1
	INNER_STORE_12_LIB4
#else
	CALL(inner_store_12_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemv_n_12_lib4)





//                            1      2              3          4        5          6             7         8          9
// void kernel_dgemv_t_12_lib4(int k, double *alpha, int offa, double *A, int sda, double *x, double *beta, double *y, double *z);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemv_t_12_lib4)
	
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
	vmovapd	%ymm0, %ymm10
	vmovapd	%ymm0, %ymm11


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG4, %r11  // A
	movq	ARG5, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
//	movslq	%r12d, %r12
	movq	ARG6, %r13  // x
	movq	ARG3, %r14 // offA

#if MACRO_LEVEL>=2
	INNER_EDGE_GEMV_ADD_T_12_LIB4
#else
	CALL(inner_edge_gemv_add_t_12_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMV_ADD_T_12_LIB4
#else
	CALL(inner_kernel_dgemv_add_t_12_lib4)
#endif


	// call inner blender t

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11   // beta
	movq	ARG8, %r12 // y 

#if MACRO_LEVEL>=1
	INNER_BLEND_T_SCALE_AB_12_LIB4
#else
	CALL(inner_blend_t_scale_ab_12_lib4)
#endif


	// store

	movq	ARG9, %r10 // z 

#if MACRO_LEVEL>=1
	INNER_STORE_12_LIB4
#else
	CALL(inner_store_12_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemv_t_12_lib4)





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

