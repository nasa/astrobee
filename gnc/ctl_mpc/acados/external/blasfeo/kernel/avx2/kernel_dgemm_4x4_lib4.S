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
// ymm0  <- [d00 d11 d22 d33]
// ymm1  <- [d01 d10 d23 d32]
// ymm2  <- [d03 d12 d21 d30]
// ymm3  <- [d02 d13 d20 d31]
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
// r12   <- B+4*k*sizeof(double)
// ymm0  <- [d00 d11 d22 d33]
// ymm1  <- [d01 d10 d23 d32]
// ymm2  <- [d03 d12 d21 d30]
// ymm3  <- [d02 d13 d20 d31]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_4x4_lib4)
#endif
	
// broadcast scheme
#if 1

	cmpl	$ 0, %r10d
	jle		5f // return

	// prefetch
	prefetcht0	0(%r12) // software prefetch
	prefetcht0	0+64(%r12) // software prefetch

	// preload
	vmovapd 		0(%r11), %ymm13 // A

	vxorpd			%ymm4, %ymm4, %ymm4
	vmovapd			%ymm4, %ymm5
	vmovapd			%ymm4, %ymm6
	vmovapd			%ymm4, %ymm7

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			32(%r11), %ymm14 // A
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	prefetcht0	128(%r12) // software prefetch
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	prefetcht0	128+64(%r12) // software prefetch
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			64(%r11), %ymm13 // A
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	addq	$ 128, %r11

	// unroll 0
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			-32(%r11), %ymm14 // A
	vbroadcastsd	72(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	80(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	88(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	addq	$ 128, %r12

	// unroll 0
	vbroadcastsd	-32(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			0(%r11), %ymm13 // A
	vbroadcastsd	-24(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vbroadcastsd	-16(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vbroadcastsd	-8(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm7

	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			32(%r11), %ymm14 // A
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			64(%r11), %ymm13 // A
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	addq	$ 128, %r11

	// unroll 0
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			-32(%r11), %ymm14 // A
	vbroadcastsd	72(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	80(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	88(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	addq	$ 128, %r12

	// unroll 0
	vbroadcastsd	-32(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm4
//	vmovapd			0(%r11), %ymm13 // A
	vbroadcastsd	-24(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vbroadcastsd	-16(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vbroadcastsd	-8(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm7

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3

	addq	$ 32, %r11
	addq	$ 32, %r12
	subl	$ 1, %r10d

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // reduce

	vaddpd			%ymm4, %ymm0, %ymm0
	vaddpd			%ymm5, %ymm1, %ymm1
	vaddpd			%ymm6, %ymm2, %ymm2
	vaddpd			%ymm7, %ymm3, %ymm3

5: // return

// shuffle scheme
#else

	cmpl	$ 0, %r10d
	jle		5f // return

	vxorpd		%ymm4, %ymm4, %ymm4
	vmovapd		%ymm4, %ymm5
	vmovapd		%ymm4, %ymm5
	vmovapd		%ymm4, %ymm6

	// preload
	vmovapd 0(%r11), %ymm8 // A0[0]
	vmovapd 0(%r12), %ymm12 // B[0]

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop
	
	// unroll 0
	vmovapd 	32(%r12), %ymm13 // B[4]
	vfmadd231pd	%ymm8, %ymm12, %ymm0
	vshufpd 	$ 0x5, %ymm12, %ymm12, %ymm14

	vperm2f128	$ 0x1, %ymm14, %ymm14, %ymm12
	vfmadd231pd	%ymm8, %ymm14, %ymm1
	vmovapd 	32(%r11), %ymm10 // A0[4]

	vfmadd231pd	%ymm8, %ymm12, %ymm3
	vshufpd 	$ 0x5, %ymm12, %ymm12, %ymm14

	subl	$ 4, %r10d
	vfmadd231pd	%ymm8, %ymm14, %ymm2

	// unroll 1
	vmovapd 	64(%r12), %ymm12 // B[8]
	vfmadd231pd	%ymm10, %ymm13, %ymm4
	vshufpd 	$ 0x5, %ymm13, %ymm13, %ymm14

	vperm2f128	$ 0x1, %ymm14, %ymm14, %ymm13
	vfmadd231pd	%ymm10, %ymm14, %ymm5
	vmovapd 	64(%r11), %ymm8 // A0[8]

	vfmadd231pd	%ymm10, %ymm13, %ymm7
	vshufpd 	$ 0x5, %ymm13, %ymm13, %ymm14

	vfmadd231pd	%ymm10, %ymm14, %ymm6

	// unroll 2
	vmovapd 	96(%r12), %ymm13 // B[12]
	vfmadd231pd	%ymm8, %ymm12, %ymm0
	vshufpd 	$ 0x5, %ymm12, %ymm12, %ymm14

	vperm2f128	$ 0x1, %ymm14, %ymm14, %ymm12
	vfmadd231pd	%ymm8, %ymm14, %ymm1
	vmovapd 	96(%r11), %ymm10 // A0[12]

	vfmadd231pd	%ymm8, %ymm12, %ymm3
	vshufpd 	$ 0x5, %ymm12, %ymm12, %ymm14
	addq	$ 128, %r12

	vfmadd231pd	%ymm8, %ymm14, %ymm2
	addq	$ 128, %r11


	// unroll 3
	vmovapd 	0(%r12), %ymm12 // B[0]
	vfmadd231pd	%ymm10, %ymm13, %ymm4
	vshufpd 	$ 0x5, %ymm13, %ymm13, %ymm14

	vperm2f128	$ 0x1, %ymm14, %ymm14, %ymm13
	vfmadd231pd	%ymm10, %ymm14, %ymm5
	vmovapd 	0(%r11), %ymm8 // A0[0]

	vfmadd231pd	%ymm10, %ymm13, %ymm7
	vshufpd 	$ 0x5, %ymm13, %ymm13, %ymm14

	vfmadd231pd	%ymm10, %ymm14, %ymm6

	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovapd 	32(%r12), %ymm13 // B[4]
	vfmadd231pd	%ymm8, %ymm12, %ymm0
	vshufpd 	$ 0x5, %ymm12, %ymm12, %ymm14

	vperm2f128	$ 0x1, %ymm14, %ymm14, %ymm12
	vfmadd231pd	%ymm8, %ymm14, %ymm1
	vmovapd 	32(%r11), %ymm10 // A0[4]

	vfmadd231pd	%ymm8, %ymm12, %ymm3
	vshufpd 	$ 0x5, %ymm12, %ymm12, %ymm14

	subl	$ 4, %r10d
	vfmadd231pd	%ymm8, %ymm14, %ymm2

	// unroll 1
	vmovapd 	64(%r12), %ymm12 // B[8]
	vfmadd231pd	%ymm10, %ymm13, %ymm4
	vshufpd 	$ 0x5, %ymm13, %ymm13, %ymm14

	vperm2f128	$ 0x1, %ymm14, %ymm14, %ymm13
	vfmadd231pd	%ymm10, %ymm14, %ymm5
	vmovapd 	64(%r11), %ymm8 // A0[8]

	vfmadd231pd	%ymm10, %ymm13, %ymm7
	vshufpd 	$ 0x5, %ymm13, %ymm13, %ymm14

	vfmadd231pd	%ymm10, %ymm14, %ymm6

	// unroll 2
	vmovapd 	96(%r12), %ymm13 // B[12]
	vfmadd231pd	%ymm8, %ymm12, %ymm0
	vshufpd 	$ 0x5, %ymm12, %ymm12, %ymm14

	vperm2f128	$ 0x1, %ymm14, %ymm14, %ymm12
	vfmadd231pd	%ymm8, %ymm14, %ymm1
	vmovapd 	96(%r11), %ymm10 // A0[12]

	vfmadd231pd	%ymm8, %ymm12, %ymm3
	vshufpd 	$ 0x5, %ymm12, %ymm12, %ymm14
	addq	$ 128, %r12

	vfmadd231pd	%ymm8, %ymm14, %ymm2
	addq	$ 128, %r11


	// unroll 3
//	vmovapd 	0(%r12), %ymm12 // B[0]
	vfmadd231pd	%ymm10, %ymm13, %ymm4
	vshufpd 	$ 0x5, %ymm13, %ymm13, %ymm14

	vperm2f128	$ 0x1, %ymm14, %ymm14, %ymm13
	vfmadd231pd	%ymm10, %ymm14, %ymm5
//	vmovapd 	0(%r11), %ymm8 // A0[0]

	vfmadd231pd	%ymm10, %ymm13, %ymm7
	vshufpd 	$ 0x5, %ymm13, %ymm13, %ymm14

	vfmadd231pd	%ymm10, %ymm14, %ymm6



	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	vmovapd 0(%r12), %ymm12 // B[0]
	vmovapd 0(%r11), %ymm8 // A0[0]
	vfmadd231pd	%ymm8, %ymm12, %ymm0
	addq	$ 32, %r11

	vshufpd $ 0x5, %ymm12, %ymm12, %ymm14
	vfmadd231pd	%ymm8, %ymm14, %ymm1
	addq	$ 32, %r12

	vperm2f128 $ 0x1, %ymm14, %ymm14, %ymm14
	vfmadd231pd	%ymm8, %ymm14, %ymm3

	vshufpd $ 0x5, %ymm14, %ymm14, %ymm14
	vfmadd231pd	%ymm8, %ymm14, %ymm2
	subl	$ 1, %r10d

	cmpl	$ 0, %r10d

	jg		3b // clean up loop 


2: // reduce

	vaddpd			%ymm4, %ymm0, %ymm0
	vaddpd			%ymm5, %ymm1, %ymm1
	vaddpd			%ymm6, %ymm2, %ymm2
	vaddpd			%ymm7, %ymm3, %ymm3

5: // return

#endif

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_4x4_lib4)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// r13   <- 4*sdb*sizeof(double)
// r14   <= dirty
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
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
// r12   <- B+(k/4)*sdb*sizeof(double)+(k%4)
// r13   <- 4*sdb*sizeof(double)
// r14   <= dirty
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_4x4_lib4)
#endif
	
	cmpl	$ 0, %r10d
	jle		5f // return

	vxorpd			%ymm4, %ymm4, %ymm4
	vmovapd			%ymm4, %ymm5
	vmovapd			%ymm4, %ymm6
	vmovapd			%ymm4, %ymm7

	// preload
	vmovapd 		0(%r11), %ymm13 // A

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	prefetcht0	0(%r12, %r13, 2) // software prefetch
	prefetcht0	64(%r12, %r13, 2) // software prefetch

	// unroll 0
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			32(%r11), %ymm14 // A
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	96(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			64(%r11), %ymm13 // A
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vbroadcastsd	72(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vbroadcastsd	104(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	addq	$ 128, %r11

	// unroll 0
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			-32(%r11), %ymm14 // A
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	80(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	112(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3

	// unroll 0
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			0(%r11), %ymm13 // A
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vbroadcastsd	88(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vbroadcastsd	120(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	addq	%r13, %r12

	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			32(%r11), %ymm14 // A
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	96(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm4
	vmovapd			64(%r11), %ymm13 // A
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vbroadcastsd	72(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vbroadcastsd	104(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	addq	$ 128, %r11

	// unroll 0
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vmovapd			-32(%r11), %ymm14 // A
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	80(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	112(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3

	// unroll 0
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm4
//	vmovapd			0(%r11), %ymm13 // A
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm5
	vbroadcastsd	88(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm6
	vbroadcastsd	120(%r12), %ymm12 // B
	vfmadd231pd		%ymm14, %ymm12, %ymm7
	addq	%r13, %r12

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	32(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	64(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	96(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3

	addq	$ 32, %r11
	addq	$ 8, %r12
	subl	$ 1, %r10d

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // reduce

	vaddpd			%ymm4, %ymm0, %ymm0
	vaddpd			%ymm5, %ymm1, %ymm1
	vaddpd			%ymm6, %ymm2, %ymm2
	vaddpd			%ymm7, %ymm3, %ymm3

5: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_4x4_lib4)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- B
// r12   <- C
// ymm0  <- [a00 a10 a20 a30]
// ymm1  <- [a01 a11 a21 a31]
// ymm2  <- [a02 a12 a22 a32]
// ymm3  <- [a03 a13 a23 a33]

//
// output arguments:
// r10d  <- 0
// r11   <- ?
// r12   <- ?
// ymm0  <- [a00 a10 a20 a30]
// ymm1  <- [a01 a11 a21 a31]
// ymm2  <- [a02 a12 a22 a32]
// ymm3  <- [a03 a13 a23 a33]

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEBP_ADD_NN_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgebp_add_nn_4x4_lib4)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return

	cmpl	$ 3, %r10d
	jle		2f // cleanup loop

	// main loop
	.p2align 3
1:
	vmovapd			0(%r12), %ymm12
	vbroadcastsd	0(%r11), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm12
	vbroadcastsd	8(%r11), %ymm13
	subl	$ 4, %r10d
	vfmadd231pd		%ymm1, %ymm13, %ymm12
	vbroadcastsd	16(%r11), %ymm13
	vfmadd231pd		%ymm2, %ymm13, %ymm12
	vbroadcastsd	24(%r11), %ymm13
	vfmadd231pd		%ymm3, %ymm13, %ymm12
	vmovapd			%ymm12, 0(%r12)

	vmovapd			32(%r12), %ymm12
	vbroadcastsd	32(%r11), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm12
	vbroadcastsd	40(%r11), %ymm13
	vfmadd231pd		%ymm1, %ymm13, %ymm12
	vbroadcastsd	48(%r11), %ymm13
	vfmadd231pd		%ymm2, %ymm13, %ymm12
	vbroadcastsd	56(%r11), %ymm13
	vfmadd231pd		%ymm3, %ymm13, %ymm12
	vmovapd			%ymm12, 32(%r12)

	vmovapd			64(%r12), %ymm12
	vbroadcastsd	64(%r11), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm12
	vbroadcastsd	72(%r11), %ymm13
	vfmadd231pd		%ymm1, %ymm13, %ymm12
	vbroadcastsd	80(%r11), %ymm13
	vfmadd231pd		%ymm2, %ymm13, %ymm12
	vbroadcastsd	88(%r11), %ymm13
	vfmadd231pd		%ymm3, %ymm13, %ymm12
	vmovapd			%ymm12, 64(%r12)

	vmovapd			96(%r12), %ymm12
	vbroadcastsd	96(%r11), %ymm13
	addq	$ 128, %r11
	vfmadd231pd		%ymm0, %ymm13, %ymm12
	vbroadcastsd	-24(%r11), %ymm13
	vfmadd231pd		%ymm1, %ymm13, %ymm12
	vbroadcastsd	-16(%r11), %ymm13
	vfmadd231pd		%ymm2, %ymm13, %ymm12
	vbroadcastsd	-8(%r11), %ymm13
	addq	$ 128, %r12
	vfmadd231pd		%ymm3, %ymm13, %ymm12
	vmovapd			%ymm12, -32(%r12)

	cmpl	$ 3, %r10d
	jg		1b // main loop

	cmpl	$ 0, %r10d
	jle		0f // return

	// cleanup loop
2:
	vmovapd			0(%r12), %ymm12
	vbroadcastsd	0(%r11), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm12
	vbroadcastsd	8(%r11), %ymm13
	vfmadd231pd		%ymm1, %ymm13, %ymm12
	vbroadcastsd	16(%r11), %ymm13
	vfmadd231pd		%ymm2, %ymm13, %ymm12
	vbroadcastsd	24(%r11), %ymm13
	vfmadd231pd		%ymm3, %ymm13, %ymm12
	vmovapd			%ymm12, 0(%r12)

	addq	$ 32, %r11
	addq	$ 32, %r12

	subl	$ 1, %r10d
	cmpl	$ 0, %r10d
	jg		2b // main loop

	// return
0:

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgebp_add_nn_4x4_lib4)
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
	vmovapd			0(%r11), %ymm12
	vbroadcastsd	0(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	32(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	64(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	96(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3

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

	FUN_END(inner_edge_dgemm_nn_4x4_lib4)
#endif





// common inner routine with file scope
//
// edge for B lower triangular
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NT_RL_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nt_rl_4x4_lib4)
#endif
	
	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3

	// unroll 1
	vmovapd			32(%r11), %ymm13 // A
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3

	// unroll 2
	vmovapd			64(%r11), %ymm13 // A
	vbroadcastsd	80(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	88(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3

	// unroll 3
	vmovapd			96(%r11), %ymm13 // A
	vbroadcastsd	120(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r12

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nt_rl_4x4_lib4)
#endif





// common inner routine with file scope
//
// edge for B upper triangular
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// r13   <- n1
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NT_RL_4X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nt_rl_4x4_vs_lib4)
#endif
	
	cmpl	$ 0, %r13d
	jle		0f // end

	cmpl	$ 4, %r13d
	jl		1f // end

	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3

	// unroll 1
	vmovapd			32(%r11), %ymm13 // A
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3

	// unroll 2
	vmovapd			64(%r11), %ymm13 // A
	vbroadcastsd	80(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	88(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3

	// unroll 3
	vmovapd			96(%r11), %ymm13 // A
	vbroadcastsd	120(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r12

	jmp		0f

1:

	cmpl	$ 3, %r13d
	jl		2f // end

	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2

	// unroll 1
	vmovapd			32(%r11), %ymm13 // A
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2

	// unroll 2
	vmovapd			64(%r11), %ymm13 // A
	vbroadcastsd	80(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2

	// unroll 3

	subl	$ 3, %r10d
	addq	$ 96, %r11
	addq	$ 96, %r12

	jmp		0f

2:

	cmpl	$ 2, %r13d
	jl		3f // end

	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1

	// unroll 1
	vmovapd			32(%r11), %ymm13 // A
	vbroadcastsd	40(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1

	// unroll 2

	// unroll 3

	subl	$ 2, %r10d
	addq	$ 64, %r11
	addq	$ 64, %r12

	jmp		0f

3:

//	cmpl	$ 1, %r13d
//	jl		0f // end

	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vbroadcastsd	0(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm0

	// unroll 1

	// unroll 2

	// unroll 3

	subl	$ 1, %r10d
	addq	$ 32, %r11
	addq	$ 32, %r12

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nt_rl_4x4_vs_lib4)
#endif





// common inner routine with file scope
//
// edge for B lower triangular
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NT_RL_ONE_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nt_rl_one_4x4_lib4)
#endif
	
	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vaddpd			%ymm0, %ymm13, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3

	// unroll 1
	vmovapd			32(%r11), %ymm13 // A
	vaddpd			%ymm1, %ymm13, %ymm1
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3

	// unroll 2
	vmovapd			64(%r11), %ymm13 // A
	vaddpd			%ymm2, %ymm13, %ymm2
	vbroadcastsd	88(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3

	// unroll 3
	vmovapd			96(%r11), %ymm13 // A
	vaddpd			%ymm3, %ymm13, %ymm3

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r12

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nt_rl_one_4x4_lib4)
#endif





// common inner routine with file scope
//
// edge for B upper triangular
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// r13   <- n1
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NT_RL_ONE_4X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nt_rl_one_4x4_vs_lib4)
#endif
	
	cmpl	$ 0, %r13d
	jle		0f // end

	cmpl	$ 4, %r13d
	jl		1f // end

	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vaddpd			%ymm0, %ymm13, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3

	// unroll 1
	vmovapd			32(%r11), %ymm13 // A
	vaddpd			%ymm1, %ymm13, %ymm1
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2
	vbroadcastsd	56(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3

	// unroll 2
	vmovapd			64(%r11), %ymm13 // A
	vaddpd			%ymm2, %ymm13, %ymm2
	vbroadcastsd	88(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm3

	// unroll 3
	vmovapd			96(%r11), %ymm13 // A
	vaddpd			%ymm3, %ymm13, %ymm3

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r12

	jmp		0f

1:

	cmpl	$ 3, %r13d
	jl		2f // end

	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vaddpd			%ymm0, %ymm13, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2

	// unroll 1
	vmovapd			32(%r11), %ymm13 // A
	vaddpd			%ymm1, %ymm13, %ymm1
	vbroadcastsd	48(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm2

	// unroll 2
	vmovapd			64(%r11), %ymm13 // A
	vaddpd			%ymm2, %ymm13, %ymm2

	// unroll 3

	subl	$ 3, %r10d
	addq	$ 96, %r11
	addq	$ 96, %r12

	jmp		0f

2:

	cmpl	$ 2, %r13d
	jl		3f // end

	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vaddpd			%ymm0, %ymm13, %ymm0
	vbroadcastsd	8(%r12), %ymm12 // B
	vfmadd231pd		%ymm13, %ymm12, %ymm1

	// unroll 1
	vmovapd			32(%r11), %ymm13 // A
	vaddpd			%ymm1, %ymm13, %ymm1

	// unroll 2

	// unroll 3

	subl	$ 2, %r10d
	addq	$ 64, %r11
	addq	$ 64, %r12

	jmp		0f

3:

//	cmpl	$ 1, %r13d
//	jl		0f // end

	// unroll 0
	vmovapd			0(%r11), %ymm13 // A
	vaddpd			%ymm0, %ymm13, %ymm0

	// unroll 1

	// unroll 2

	// unroll 3

	subl	$ 1, %r10d
	addq	$ 32, %r11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nt_rl_one_4x4_vs_lib4)
#endif





// common inner routine with file scope
//
// edge for B upper triangular
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm8  <- dirty
// ymm12 <- dirty
// ymm15 <- dirty

//
// output arguments:
// r10d  <- k
// r11   <- A+4*4*sizeof(double)
// r12   <- B+4*4*sizeof(double)
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm8  <- dirty
// ymm12 <- dirty
// ymm15 <- dirty


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NT_RU_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nt_ru_4x4_lib4)
#endif
	
	vmovapd			0(%r11), %ymm8
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0

	vmovapd			32(%r11), %ymm8
	vbroadcastsd	32(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	40(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1

	vmovapd			64(%r11), %ymm8
	vbroadcastsd	64(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	72(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1
	vbroadcastsd	80(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm2

	vmovapd			96(%r11), %ymm8
	vbroadcastsd	96(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	104(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1
	vbroadcastsd	112(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm2
	vbroadcastsd	120(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm3

	subl			$ 4, %r10d
	addq			$ 128, %r11
	addq			$ 128, %r12

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nt_ru_4x4_lib4)
#endif





// common inner routine with file scope
//
// edge for B upper triangular
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm8  <- dirty
// ymm12 <- dirty
// ymm15 <- dirty

//
// output arguments:
// r10d  <- max(k-4,0)
// r11   <- A+4*4*sizeof(double)
// r12   <- B+4*4*sizeof(double)
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm8  <- dirty
// ymm12 <- dirty
// ymm15 <- dirty


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NT_RU_4X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nt_ru_4x4_vs_lib4)
#endif
	
	vmovapd			0(%r11), %ymm8
	subl			$ 1, %r10d
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	addq			$ 32, %r11
	addq			$ 32, %r12

	cmpl	$ 0, %r10d
	jle		0f

	vmovapd			0(%r11), %ymm8
	subl			$ 1, %r10d
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	addq			$ 32, %r11
	vbroadcastsd	8(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1
	addq			$ 32, %r12

	cmpl	$ 0, %r10d
	jle		0f

	vmovapd			0(%r11), %ymm8
	subl			$ 1, %r10d
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1
	addq			$ 32, %r11
	vbroadcastsd	16(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm2
	addq			$ 32, %r12

	cmpl	$ 0, %r10d
	jle		0f

	vmovapd			0(%r11), %ymm8
	subl			$ 1, %r10d
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm2
	addq			$ 32, %r11
	vbroadcastsd	24(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm3
	addq			$ 32, %r12

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nt_ru_4x4_vs_lib4)
#endif





// common inner routine with file scope
//
// edge for B upper triangular
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm8  <- dirty
// ymm12 <- dirty
// ymm15 <- dirty

//
// output arguments:
// r10d  <- k
// r11   <- A+4*4*sizeof(double)
// r12   <- B+4*4*sizeof(double)
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm8  <- dirty
// ymm12 <- dirty
// ymm15 <- dirty


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NT_RU_ONE_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nt_ru_one_4x4_lib4)
#endif
	
	vmovapd			0(%r11), %ymm8
	vaddpd			%ymm0, %ymm8, %ymm0

	vmovapd			32(%r11), %ymm8
	vbroadcastsd	32(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vaddpd			%ymm1, %ymm8, %ymm1

	vmovapd			64(%r11), %ymm8
	vbroadcastsd	64(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	72(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1
	vaddpd			%ymm2, %ymm8, %ymm2

	vmovapd			96(%r11), %ymm8
	vbroadcastsd	96(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	104(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1
	vbroadcastsd	112(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm2
	vaddpd			%ymm3, %ymm8, %ymm3

	subl			$ 4, %r10d
	addq			$ 128, %r11
	addq			$ 128, %r12

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nt_ru_one_4x4_lib4)
#endif





// common inner routine with file scope
//
// edge for B upper triangular
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm8  <- dirty
// ymm12 <- dirty
// ymm15 <- dirty

//
// output arguments:
// r10d  <- max(k-4,0)
// r11   <- A+4*4*sizeof(double)
// r12   <- B+4*4*sizeof(double)
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm8  <- dirty
// ymm12 <- dirty
// ymm15 <- dirty


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NT_RU_ONE_4X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nt_ru_one_4x4_vs_lib4)
#endif
	
	vmovapd			0(%r11), %ymm8
	subl			$ 1, %r10d
	vaddpd			%ymm0, %ymm8, %ymm0
	addq			$ 32, %r11
	addq			$ 32, %r12

	cmpl	$ 0, %r10d
	jle		0f

	vmovapd			0(%r11), %ymm8
	subl			$ 1, %r10d
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	addq			$ 32, %r11
	vaddpd			%ymm1, %ymm8, %ymm1
	addq			$ 32, %r12

	cmpl	$ 0, %r10d
	jle		0f

	vmovapd			0(%r11), %ymm8
	subl			$ 1, %r10d
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1
	addq			$ 32, %r11
	vaddpd			%ymm2, %ymm8, %ymm2
	addq			$ 32, %r12

	cmpl	$ 0, %r10d
	jle		0f

	vmovapd			0(%r11), %ymm8
	subl			$ 1, %r10d
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm2
	addq			$ 32, %r11
	vaddpd			%ymm3, %ymm8, %ymm3
	addq			$ 32, %r12

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nt_ru_one_4x4_vs_lib4)
#endif





// common inner routine with file scope
//
// edge for B lower triangular
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
	.macro INNER_EDGE_DTRMM_NN_RL_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nn_rl_4x4_lib4)
#endif
	
	cmpl	$ 0, %r14d
	jg		0f

	// offB==0

	vmovapd			0(%r11), %ymm8
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0

	vmovapd			32(%r11), %ymm8
	vbroadcastsd	8(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	40(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1

	vmovapd			64(%r11), %ymm8
	vbroadcastsd	16(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	48(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1
	vbroadcastsd	80(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm2

	vmovapd			96(%r11), %ymm8
	vbroadcastsd	24(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	56(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1
	vbroadcastsd	88(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm2
	vbroadcastsd	120(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm3

	subl			$ 4, %r10d // k-4
	addq			$ 128, %r11 // A+4*bs*sizeof(double)
	addq			%r13, %r12 // B+bs*sdb*sizeof(double)

	jmp		3f

0:
	cmpl	$ 1, %r14d
	jg		1f

	// offB==1

	addq			$ 8, %r12 // B+1*sizeof(double)

	vmovapd			0(%r11), %ymm8
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0

	vmovapd			32(%r11), %ymm8
	vbroadcastsd	8(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	40(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1

	vmovapd			64(%r11), %ymm8
	vbroadcastsd	16(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	48(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1
	vbroadcastsd	80(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm2

	subl			$ 3, %r10d // k-3
	addq			$ 96, %r11 // A+3*bs*sizeof(double)
	addq			%r13, %r12
	subq			$ 8, %r12 // B+bs*sdb*sizeof(double)-1

	jmp		3f

1:
	cmpl	$ 2, %r14d
	jg		2f

	// offB==2

	addq			$ 16, %r12 // B+2*sizeof(double)

	vmovapd			0(%r11), %ymm8
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0

	vmovapd			32(%r11), %ymm8
	vbroadcastsd	8(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	40(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1

	subl			$ 2, %r10d // k-2
	addq			$ 64, %r11 // A+2*bs*sizeof(double)
	addq			%r13, %r12
	subq			$ 16, %r12 // B+bs*sdb*sizeof(double)-2

	vmovapd			0(%r11), %ymm8
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	32(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1
	vbroadcastsd	64(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm2

	vmovapd			32(%r11), %ymm8
	vbroadcastsd	8(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	40(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1
	vbroadcastsd	72(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm2
	vbroadcastsd	104(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm3

	vmovapd			64(%r11), %ymm8
	vbroadcastsd	16(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	48(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1
	vbroadcastsd	80(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm2
	vbroadcastsd	112(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm3

	vmovapd			96(%r11), %ymm8
	vbroadcastsd	24(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	56(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1
	vbroadcastsd	88(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm2
	vbroadcastsd	120(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm3

	subl			$ 4, %r10d // k-4
	addq			$ 128, %r11 // A+4*bs*sizeof(double)
	addq			%r13, %r12 // B+bs*sdb*sizeof(double)

	jmp		3f

2:
	// offB==3

	addq			$ 24, %r12 // B+3*sizeof(double)

	vmovapd			0(%r11), %ymm8
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A+1*bs*sizeof(double)
	addq			%r13, %r12
	subq			$ 24, %r12 // B+bs*sdb*sizeof(double)-3

	vmovapd			0(%r11), %ymm8
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	32(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1

	vmovapd			32(%r11), %ymm8
	vbroadcastsd	8(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	40(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1
	vbroadcastsd	72(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm2

	vmovapd			64(%r11), %ymm8
	vbroadcastsd	16(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	48(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1
	vbroadcastsd	80(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm2
	vbroadcastsd	112(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm3

	vmovapd			96(%r11), %ymm8
	vbroadcastsd	24(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	56(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1
	vbroadcastsd	88(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm2
	vbroadcastsd	120(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm3

	subl			$ 4, %r10d // k-4
	addq			$ 128, %r11 // A+4*bs*sizeof(double)
	addq			%r13, %r12 // B+bs*sdb*sizeof(double)

3:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nn_rl_4x4_lib4)
#endif





// common inner routine with file scope
//
// edge for B lower triangular
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
	.macro INNER_EDGE_DTRMM_NN_RL_4X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nn_rl_4x4_vs_lib4)
#endif
	
	cmpl			$ 0, %r10d
	jle				3f // end

	cmpl			$ 0, %r14d
	jg				0f // offB>0

	// offB==0

	vmovapd			0(%r11), %ymm8
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A+1*bs*sizeof(double)
	addq			$ 8, %r12 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm8
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	32(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A+1*bs*sizeof(double)
	addq			$ 8, %r12 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm8
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	32(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1
	vbroadcastsd	64(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm2

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A+1*bs*sizeof(double)
	addq			$ 8, %r12 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm8
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	32(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1
	vbroadcastsd	64(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm2
	vbroadcastsd	96(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm3

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A+1*bs*sizeof(double)
	addq			%r13, %r12
	subq			$ 24, %r12 // B+bs*sdb*sizeof(double)-(bs-1)*sizeof(double)

	jmp				3f // end

0:
	cmpl			$ 1, %r14d
	jg				1f // offB>1

	// offB==1

	addq			$ 8, %r12 // B+1*sizeof(double)

	vmovapd			0(%r11), %ymm8
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A+1*bs*sizeof(double)
	addq			$ 8, %r12 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm8
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	32(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A+1*bs*sizeof(double)
	addq			$ 8, %r12 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm8
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	32(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1
	vbroadcastsd	64(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm2

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A+1*bs*sizeof(double)
	addq			%r13, %r12
	subq			$ 24, %r12 // B+bs*sdb*sizeof(double)-(bs-1)*sizeof(double)

	jmp				3f // end

1:
	cmpl			$ 2, %r14d
	jg				2f // offB>2

	// offB==2

	addq			$ 16, %r12 // B+2*sizeof(double)

	vmovapd			0(%r11), %ymm8
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A+1*bs*sizeof(double)
	addq			$ 8, %r12 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm8
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	32(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1

	subl			$ 1, %r10d // k-2
	addq			$ 32, %r11 // A+2*bs*sizeof(double)
	addq			%r13, %r12
	subq			$ 24, %r12 // B+bs*sdb*sizeof(double)-(bs-1)*sizeof(double)

	vmovapd			0(%r11), %ymm8
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	32(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1
	vbroadcastsd	64(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm2

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A+1*bs*sizeof(double)
	addq			$ 8, %r12 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm8
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	32(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1
	vbroadcastsd	64(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm2
	vbroadcastsd	96(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm3

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A+1*bs*sizeof(double)
	addq			$ 8, %r12 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm8
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	32(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1
	vbroadcastsd	64(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm2
	vbroadcastsd	96(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm3

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A+1*bs*sizeof(double)
	addq			$ 8, %r12 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm8
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	32(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1
	vbroadcastsd	64(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm2
	vbroadcastsd	96(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm3

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A+1*bs*sizeof(double)
	addq			%r13, %r12
	subq			$ 24, %r12 // B+bs*sdb*sizeof(double)-(bs-1)*sizeof(double)

	jmp				3f

2:
	// offB==3

	addq			$ 24, %r12 // B+3*sizeof(double)

	vmovapd			0(%r11), %ymm8
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A+1*bs*sizeof(double)
	addq			%r13, %r12
	subq			$ 24, %r12 // B+bs*sdb*sizeof(double)-(bs-1)*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm8
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	32(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A+1*bs*sizeof(double)
	addq			$ 8, %r12 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm8
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	32(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1
	vbroadcastsd	64(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm2

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A+1*bs*sizeof(double)
	addq			$ 8, %r12 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm8
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	32(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1
	vbroadcastsd	64(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm2
	vbroadcastsd	96(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm3

	subl			$ 1, %r10d // k-1
	addq			$ 32, %r11 // A+1*bs*sizeof(double)
	addq			$ 8, %r12 // B+1*sizeof(double)

	cmpl			$ 0, %r10d
	jle				3f // end

	vmovapd			0(%r11), %ymm8
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	32(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1
	vbroadcastsd	64(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm2
	vbroadcastsd	96(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm3

	subl			$ 1, %r10d // k-4
	addq			$ 32, %r11 // A+4*bs*sizeof(double)
	addq			%r13, %r12
	subq			$ 24, %r12 // B+bs*sdb*sizeof(double)-(bs-1)*sizeof(double)

3:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nn_rl_4x4_vs_lib4)
#endif





// common inner routine with file scope
//
// edge for dlauum
//
// input arguments:
// r10   <- A
// r11   <- B
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm8  <- dirty
// ymm12 <- dirty
// ymm15 <- dirty

//
// output arguments:
// r10   <- A+4*4*sizeof(double)
// r11   <- B+4*4*sizeof(double)
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm8  <- dirty
// ymm12 <- dirty
// ymm15 <- dirty


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DLAUUM_NT_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dlauum_nt_4x4_lib4)
#endif
	
	vxorpd			%ymm14, %ymm14, %ymm14

	vmovapd			0(%r10), %ymm8
	vblendpd		$ 0x1, %ymm8, %ymm14, %ymm8
	vbroadcastsd	0(%r11), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0

	vmovapd			32(%r10), %ymm8
	vblendpd		$ 0x3, %ymm8, %ymm14, %ymm8
	vbroadcastsd	32(%r11), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	40(%r11), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1

	vmovapd			64(%r10), %ymm8
	vblendpd		$ 0x7, %ymm8, %ymm14, %ymm8
	vbroadcastsd	64(%r11), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	72(%r11), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1
	vbroadcastsd	80(%r11), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm2

	vmovapd			96(%r10), %ymm8
	vbroadcastsd	96(%r11), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	104(%r11), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1
	vbroadcastsd	112(%r11), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm2
	vbroadcastsd	120(%r11), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm3

	addq			$ 128, %r10
	addq			$ 128, %r11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dlauum_nt_4x4_lib4)
#endif





// common inner routine with file scope
//
// edge for dlauum
//
// input arguments:
// r10   <- k
// r11   <- A
// r12   <- B
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm8  <- dirty
// ymm12 <- dirty
// ymm15 <- dirty

//
// output arguments:
// r10   <- 0
// r11   <- A+4*k*sizeof(double)
// r12   <- B+4*k*sizeof(double)
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm8  <- dirty
// ymm12 <- dirty
// ymm15 <- dirty


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DLAUUM_NT_4X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dlauum_nt_4x4_vs_lib4)
#endif
	
	vxorpd			%ymm14, %ymm14, %ymm14

	vmovapd			0(%r11), %ymm8
	subl			$ 1, %r10d
	vblendpd		$ 0x1, %ymm8, %ymm14, %ymm8
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	addq			$ 32, %r11
	addq			$ 32, %r12

	cmpl			$ 0, %r10d
	jle				0f

	vmovapd			0(%r11), %ymm8
	subl			$ 1, %r10d
	vblendpd		$ 0x3, %ymm8, %ymm14, %ymm8
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1
	addq			$ 32, %r11
	addq			$ 32, %r12

	cmpl			$ 0, %r10d
	jle				0f

	vmovapd			0(%r11), %ymm8
	subl			$ 1, %r10d
	vblendpd		$ 0x7, %ymm8, %ymm14, %ymm8
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm2
	addq			$ 32, %r11
	addq			$ 32, %r12

	cmpl			$ 0, %r10d
	jle				0f

	vmovapd			0(%r11), %ymm8
	subl			$ 1, %r10d
	vbroadcastsd	0(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm0
	vbroadcastsd	8(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm1
	vbroadcastsd	16(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm2
	vbroadcastsd	24(%r12), %ymm12
	vfmadd231pd		%ymm8, %ymm12, %ymm3
	addq			$ 32, %r11
	addq			$ 32, %r12

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dlauum_nt_4x4_vs_lib4)
#endif





// common inner routine with file scope
//
// blend
//
// input arguments:
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
	.macro INNER_BLEND_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_blend_4x4_lib4)
#endif	
	

	// tc==n
	vblendpd	$ 0xa, %ymm1, %ymm0, %ymm8
	vblendpd	$ 0x5, %ymm1, %ymm0, %ymm9
	vblendpd	$ 0xa, %ymm3, %ymm2, %ymm10
	vblendpd	$ 0x5, %ymm3, %ymm2, %ymm11

	vblendpd	$ 0xc, %ymm10, %ymm8, %ymm0
	vblendpd	$ 0x3, %ymm10, %ymm8, %ymm2
	vblendpd	$ 0xc, %ymm11, %ymm9, %ymm1
	vblendpd	$ 0x3, %ymm11, %ymm9, %ymm3

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_blend_4x4_lib4)
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
	.macro INNER_SCALE_AB_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_4x4_lib4)
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
	vfmadd231pd	%ymm14, %ymm15, %ymm0
	vmovapd		32(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm1
	vmovapd		64(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm2
	vmovapd		96(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm3

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_4x4_lib4)
#endif





// common inner routine with file scope
//
// blend for generic alpha and beta
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12  <- offset
// r13   <- C
// r14  <- 4*sdc*sizeof(double)
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
// r12  <- offset
// r13   <- C
// r14  <- 4*sdc*sizeof(double)
// r15  <- dirty
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
	.macro INNER_SCALE_AB_4X4_GEN_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_4x4_gen_lib4)
#endif
	
	// alpha
	vbroadcastsd	0(%r10), %ymm15

	vmulpd		%ymm0, %ymm15, %ymm0
	vmulpd		%ymm1, %ymm15, %ymm1
	vmulpd		%ymm2, %ymm15, %ymm2
	vmulpd		%ymm3, %ymm15, %ymm3

	// beta
	vbroadcastsd	0(%r11), %ymm15

	vxorpd		%ymm14, %ymm14, %ymm14 // 0.0

	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			3f // end

	cmpl	$ 0, %r12d
	jg		0f

	// offset==0

	vmovapd		0(%r13), %ymm12
	vfmadd231pd	%ymm12, %ymm15, %ymm0
	vmovapd		32(%r13), %ymm12
	vfmadd231pd	%ymm12, %ymm15, %ymm1
	vmovapd		64(%r13), %ymm12
	vfmadd231pd	%ymm12, %ymm15, %ymm2
	vmovapd		96(%r13), %ymm12
	vfmadd231pd	%ymm12, %ymm15, %ymm3

	jmp		3f

0:

	cmpl	$ 1, %r12d
	jg		1f

	// offset==1

	vmovapd		0(%r13), %ymm12
	vmovapd		0(%r13, %r14), %ymm14
	vblendpd	$ 0x1, %ymm14, %ymm12, %ymm12
	vmovapd		32(%r13), %ymm13
	vmovapd		32(%r13, %r14), %ymm14
	vblendpd	$ 0x1, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm14
	vshufpd		$ 0x5, %ymm14, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm14
	vshufpd		$ 0x5, %ymm14, %ymm13, %ymm13
	vfmadd231pd	%ymm12, %ymm15, %ymm0
	vfmadd231pd	%ymm13, %ymm15, %ymm1
	vmovapd		64(%r13), %ymm12
	vmovapd		64(%r13, %r14), %ymm14
	vblendpd	$ 0x1, %ymm14, %ymm12, %ymm12
	vmovapd		96(%r13), %ymm13
	vmovapd		96(%r13, %r14), %ymm14
	vblendpd	$ 0x1, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm14
	vshufpd		$ 0x5, %ymm14, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm14
	vshufpd		$ 0x5, %ymm14, %ymm13, %ymm13
	vfmadd231pd	%ymm12, %ymm15, %ymm2
	vfmadd231pd	%ymm13, %ymm15, %ymm3

	jmp		3f

1:

	cmpl	$ 2, %r12d
	jg		2f

	// offset==2

	vmovapd		0(%r13), %ymm12
	vmovapd		0(%r13, %r14), %ymm14
	vblendpd	$ 0x3, %ymm14, %ymm12, %ymm12
	vmovapd		32(%r13), %ymm13
	vmovapd		32(%r13, %r14), %ymm14
	vblendpd	$ 0x3, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm13
	vfmadd231pd	%ymm12, %ymm15, %ymm0
	vfmadd231pd	%ymm13, %ymm15, %ymm1
	vmovapd		64(%r13), %ymm12
	vmovapd		64(%r13, %r14), %ymm14
	vblendpd	$ 0x3, %ymm14, %ymm12, %ymm12
	vmovapd		96(%r13), %ymm13
	vmovapd		96(%r13, %r14), %ymm14
	vblendpd	$ 0x3, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm13
	vfmadd231pd	%ymm12, %ymm15, %ymm2
	vfmadd231pd	%ymm13, %ymm15, %ymm3

	jmp		3f

2:

	// offset==3

	vmovapd		0(%r13), %ymm12
	vmovapd		0(%r13, %r14), %ymm14
	vblendpd	$ 0x7, %ymm14, %ymm12, %ymm12
	vmovapd		32(%r13), %ymm13
	vmovapd		32(%r13, %r14), %ymm14
	vblendpd	$ 0x7, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm14
	vshufpd		$ 0x5, %ymm12, %ymm14, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm14
	vshufpd		$ 0x5, %ymm13, %ymm14, %ymm13
	vfmadd231pd	%ymm12, %ymm15, %ymm0
	vfmadd231pd	%ymm13, %ymm15, %ymm1
	vmovapd		64(%r13), %ymm12
	vmovapd		64(%r13, %r14), %ymm14
	vblendpd	$ 0x7, %ymm14, %ymm12, %ymm12
	vmovapd		96(%r13), %ymm13
	vmovapd		96(%r13, %r14), %ymm14
	vblendpd	$ 0x7, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm14
	vshufpd		$ 0x5, %ymm12, %ymm14, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm14
	vshufpd		$ 0x5, %ymm13, %ymm14, %ymm13
	vfmadd231pd	%ymm12, %ymm15, %ymm2
	vfmadd231pd	%ymm13, %ymm15, %ymm3

3:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_4x4_gen_lib4)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta=0
//
// input arguments:
// r10   <- alpha
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
	.macro INNER_SCALE_A0_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_scale_a0_4x4_lib4)
#endif
	
	// alpha
	vbroadcastsd	0(%r10), %ymm15

	vmulpd		%ymm0, %ymm15, %ymm0
	vmulpd		%ymm1, %ymm15, %ymm1
	vmulpd		%ymm2, %ymm15, %ymm2
	vmulpd		%ymm3, %ymm15, %ymm3

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_a0_4x4_lib4)
#endif





// common inner routine with file scope
//
// scale for alpha = -1.0 and beta
//
// input arguments:
// r10   <- beta
// r11   <- C
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
// r10   <- beta
// r11   <- C
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
	.macro INNER_SCALE_M1B_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_scale_m1b_4x4_lib4)
#endif	
	
	// beta
	vbroadcastsd	0(%r10), %ymm14

	vxorpd		%ymm15, %ymm15, %ymm15 // 0.0

	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	vmovapd		0(%r11), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm0
	vmovapd		32(%r11), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm1
	vmovapd		64(%r11), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm2
	vmovapd		96(%r11), %ymm15
	vfmsub231pd	%ymm14, %ymm15, %ymm3

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m1b_4x4_lib4)
#endif





// common inner routine with file scope
//
// scale for alpha = 1.0 and beta = 1.0
//
// input arguments:
// r10   <- C
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
	.macro INNER_SCALE_M11_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_scale_m11_4x4_lib4)
#endif	
	
	vmovapd		0(%r10), %ymm15
	vsubpd		%ymm0, %ymm15, %ymm0
	vmovapd		32(%r10), %ymm15
	vsubpd		%ymm1, %ymm15, %ymm1
	vmovapd		64(%r10), %ymm15
	vsubpd		%ymm2, %ymm15, %ymm2
	vmovapd		96(%r10), %ymm15
	vsubpd		%ymm3, %ymm15, %ymm3

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m11_4x4_lib4)
#endif





// common inner routine with file scope
//
// blend for generic alpha and beta
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
	.macro INNER_BLEND_SCALE_AB_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_blend_scale_ab_4x4_lib4)
#endif
	
	// alpha
	vbroadcastsd	0(%r10), %ymm15

	vblendpd	$ 0xa, %ymm1, %ymm0, %ymm8
	vblendpd	$ 0x5, %ymm1, %ymm0, %ymm9
	vblendpd	$ 0xa, %ymm3, %ymm2, %ymm10
	vblendpd	$ 0x5, %ymm3, %ymm2, %ymm11

	vblendpd	$ 0xc, %ymm10, %ymm8, %ymm0
	vblendpd	$ 0x3, %ymm10, %ymm8, %ymm2
	vblendpd	$ 0xc, %ymm11, %ymm9, %ymm1
	vblendpd	$ 0x3, %ymm11, %ymm9, %ymm3

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
	vfmadd231pd	%ymm14, %ymm15, %ymm0
	vmovapd		32(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm1
	vmovapd		64(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm2
	vmovapd		96(%r12), %ymm15
	vfmadd231pd	%ymm14, %ymm15, %ymm3

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_blend_scale_ab_4x4_lib4)
#endif





// common inner routine with file scope
//
// blend for generic alpha and beta
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12  <- offset
// r13   <- C
// r14  <- 4*sdc*sizeof(double)
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
// r12  <- offset
// r13   <- C
// r14  <- 4*sdc*sizeof(double)
// r15  <- dirty
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
	.macro INNER_BLEND_SCALE_AB_4X4_GEN_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_blend_scale_ab_4x4_gen_lib4)
#endif
	
	// alpha
	vbroadcastsd	0(%r10), %ymm15

	vblendpd	$ 0xa, %ymm1, %ymm0, %ymm8
	vblendpd	$ 0x5, %ymm1, %ymm0, %ymm9
	vblendpd	$ 0xa, %ymm3, %ymm2, %ymm10
	vblendpd	$ 0x5, %ymm3, %ymm2, %ymm11

	vblendpd	$ 0xc, %ymm10, %ymm8, %ymm0
	vblendpd	$ 0x3, %ymm10, %ymm8, %ymm2
	vblendpd	$ 0xc, %ymm11, %ymm9, %ymm1
	vblendpd	$ 0x3, %ymm11, %ymm9, %ymm3

	vmulpd		%ymm0, %ymm15, %ymm0
	vmulpd		%ymm1, %ymm15, %ymm1
	vmulpd		%ymm2, %ymm15, %ymm2
	vmulpd		%ymm3, %ymm15, %ymm3

	// beta
	vbroadcastsd	0(%r11), %ymm15

	vxorpd		%ymm14, %ymm14, %ymm14 // 0.0

	vucomisd	%xmm15, %xmm14 // beta==0.0 ?
	je			3f // end

	cmpl	$ 0, %r12d
	jg		0f

	// offset==0

	vmovapd		0(%r13), %ymm12
	vfmadd231pd	%ymm12, %ymm15, %ymm0
	vmovapd		32(%r13), %ymm12
	vfmadd231pd	%ymm12, %ymm15, %ymm1
	vmovapd		64(%r13), %ymm12
	vfmadd231pd	%ymm12, %ymm15, %ymm2
	vmovapd		96(%r13), %ymm12
	vfmadd231pd	%ymm12, %ymm15, %ymm3

	jmp		3f

0:

	movq	%r13, %r15 // C0
	addq	%r14, %r15 // C1 <- C0 + 4*sdc*sizeof(double)

	cmpl	$ 1, %r12d
	jg		1f

	// offset==1

	vmovapd		0(%r13), %ymm12
	vmovapd		0(%r15), %ymm14
	vblendpd	$ 0x1, %ymm14, %ymm12, %ymm12
	vmovapd		32(%r13), %ymm13
	vmovapd		32(%r15), %ymm14
	vblendpd	$ 0x1, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm14
	vshufpd		$ 0x5, %ymm14, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm14
	vshufpd		$ 0x5, %ymm14, %ymm13, %ymm13
	vfmadd231pd	%ymm12, %ymm15, %ymm0
	vfmadd231pd	%ymm13, %ymm15, %ymm1
	vmovapd		64(%r13), %ymm12
	vmovapd		64(%r15), %ymm14
	vblendpd	$ 0x1, %ymm14, %ymm12, %ymm12
	vmovapd		96(%r13), %ymm13
	vmovapd		96(%r15), %ymm14
	vblendpd	$ 0x1, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm14
	vshufpd		$ 0x5, %ymm14, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm14
	vshufpd		$ 0x5, %ymm14, %ymm13, %ymm13
	vfmadd231pd	%ymm12, %ymm15, %ymm2
	vfmadd231pd	%ymm13, %ymm15, %ymm3

	jmp		3f

1:

	cmpl	$ 2, %r12d
	jg		2f

	// offset==2

	vmovapd		0(%r13), %ymm12
	vmovapd		0(%r15), %ymm14
	vblendpd	$ 0x3, %ymm14, %ymm12, %ymm12
	vmovapd		32(%r13), %ymm13
	vmovapd		32(%r15), %ymm14
	vblendpd	$ 0x3, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm13
	vfmadd231pd	%ymm12, %ymm15, %ymm0
	vfmadd231pd	%ymm13, %ymm15, %ymm1
	vmovapd		64(%r13), %ymm12
	vmovapd		64(%r15), %ymm14
	vblendpd	$ 0x3, %ymm14, %ymm12, %ymm12
	vmovapd		96(%r13), %ymm13
	vmovapd		96(%r15), %ymm14
	vblendpd	$ 0x3, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm13
	vfmadd231pd	%ymm12, %ymm15, %ymm2
	vfmadd231pd	%ymm13, %ymm15, %ymm3

	jmp		3f

2:

	// offset==3

	vmovapd		0(%r13), %ymm12
	vmovapd		0(%r15), %ymm14
	vblendpd	$ 0x7, %ymm14, %ymm12, %ymm12
	vmovapd		32(%r13), %ymm13
	vmovapd		32(%r15), %ymm14
	vblendpd	$ 0x7, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm14
	vshufpd		$ 0x5, %ymm12, %ymm14, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm14
	vshufpd		$ 0x5, %ymm13, %ymm14, %ymm13
	vfmadd231pd	%ymm12, %ymm15, %ymm0
	vfmadd231pd	%ymm13, %ymm15, %ymm1
	vmovapd		64(%r13), %ymm12
	vmovapd		64(%r15), %ymm14
	vblendpd	$ 0x7, %ymm14, %ymm12, %ymm12
	vmovapd		96(%r13), %ymm13
	vmovapd		96(%r15), %ymm14
	vblendpd	$ 0x7, %ymm14, %ymm13, %ymm13
	vperm2f128	$ 0x01, %ymm12, %ymm12, %ymm14
	vshufpd		$ 0x5, %ymm12, %ymm14, %ymm12
	vperm2f128	$ 0x01, %ymm13, %ymm13, %ymm14
	vshufpd		$ 0x5, %ymm13, %ymm14, %ymm13
	vfmadd231pd	%ymm12, %ymm15, %ymm2
	vfmadd231pd	%ymm13, %ymm15, %ymm3

3:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_blend_scale_ab_4x4_gen_lib4)
#endif





// common inner routine with file scope
//
// blender for alpha = 1.0 and beta = 1.0
//
// input arguments:
// r10   <- C
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
	.macro INNER_BLEND_SCALE_M11_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_blend_scale_m11_4x4_lib4)
#endif	

	vblendpd	$ 0xa, %ymm1, %ymm0, %ymm8
	vblendpd	$ 0x5, %ymm1, %ymm0, %ymm9
	vblendpd	$ 0xa, %ymm3, %ymm2, %ymm10
	vblendpd	$ 0x5, %ymm3, %ymm2, %ymm11

	vblendpd	$ 0xc, %ymm10, %ymm8, %ymm0
	vblendpd	$ 0x3, %ymm10, %ymm8, %ymm2
	vblendpd	$ 0xc, %ymm11, %ymm9, %ymm1
	vblendpd	$ 0x3, %ymm11, %ymm9, %ymm3

	vmovapd		0(%r10), %ymm15
	vsubpd		%ymm0, %ymm15, %ymm0
	vmovapd		32(%r10), %ymm15
	vsubpd		%ymm1, %ymm15, %ymm1
	vmovapd		64(%r10), %ymm15
	vsubpd		%ymm2, %ymm15, %ymm2
	vmovapd		96(%r10), %ymm15
	vsubpd		%ymm3, %ymm15, %ymm3

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_blend_scale_m11_4x4_lib4)
#endif





// common inner routine with file scope
//
// cholesky factorization 
//
// input arguments:
// r10  <- inv_diag_E
// r11d <- kn
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10  <- inv_diag_E
// r11d <- kn
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DPOTRF_4X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dpotrf_4x4_vs_lib4)
#endif

	vxorpd	%ymm15, %ymm15, %ymm15 // 0.0
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd	.LC04(%rip), %xmm14 // 1.0
#elif defined(OS_MAC)
	vmovsd	LC04(%rip), %xmm14 // 1.0
#endif

	vmovsd			%xmm0, %xmm0, %xmm13
	vucomisd		%xmm15, %xmm13 // d_00 > 0.0 ?
	jbe				1f
	vsqrtsd			%xmm13, %xmm13, %xmm13
	vdivsd			%xmm13, %xmm14, %xmm13
2:
	vmovsd			%xmm13, 0(%r10)
//	vmovddup		%xmm13, %xmm13
//	vperm2f128		$ 0x00, %ymm13, %ymm13, %ymm13
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmulpd			%ymm0, %ymm13, %ymm0
	cmpl			$ 2, %r11d
	jl				0f // ret
//	vperm2f128		$ 0x00, %ymm0, %ymm0, %ymm12
//	vpermilpd		$ 0xf, %ymm12, %ymm13
	vpermpd			$ 0x55, %ymm0, %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm1
	vperm2f128		$ 0x11, %ymm0, %ymm0, %ymm12
	vpermilpd		$ 0x0, %ymm12, %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm2
	vpermilpd		$ 0xf, %ymm12, %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm3

	vpermilpd		$ 0x3, %xmm1, %xmm13
	vucomisd		%xmm15, %xmm13 // d_11 > 0.0 ?
	jbe				3f
	vsqrtsd			%xmm13, %xmm13, %xmm13
	vdivsd			%xmm13, %xmm14, %xmm13
4:
	vmovsd			%xmm13, 8(%r10)
//	vmovddup		%xmm13, %xmm13
//	vperm2f128		$ 0x00, %ymm13, %ymm13, %ymm13
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmulpd			%ymm1, %ymm13, %ymm1
	cmpl			$ 3, %r11d
	jl				0f // ret
	vperm2f128		$ 0x11, %ymm1, %ymm1, %ymm12
	vpermilpd		$ 0x0, %ymm12, %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm2
	vpermilpd		$ 0xf, %ymm12, %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm3

	vextractf128	$ 0x1, %ymm2, %xmm13
	vucomisd		%xmm15, %xmm13 // d_22 > 0.0 ?
	jbe				5f
	vsqrtsd			%xmm13, %xmm13, %xmm13
	vdivsd			%xmm13, %xmm14, %xmm13
6:
	vmovsd			%xmm13, 16(%r10)
//	vmovddup		%xmm13, %xmm13
//	vperm2f128		$ 0x00, %ymm13, %ymm13, %ymm13
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmulpd			%ymm2, %ymm13, %ymm2
	cmpl			$ 4, %r11d
	jl				0f // ret
//	vperm2f128		$ 0x11, %ymm2, %ymm2, %ymm12
//	vpermilpd		$ 0xf, %ymm12, %ymm13
	vpermpd			$ 0xff, %ymm2, %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm3

//	vextractf128	$ 0x1, %ymm3, %xmm13
//	vpermilpd		$ 0x3, %xmm13, %xmm13
	vpermpd			$ 0xff, %ymm3, %ymm13
	vucomisd		%xmm15, %xmm13 // d_33 > 0.0 ?
	jbe				7f
	vsqrtsd			%xmm13, %xmm13, %xmm13
	vdivsd			%xmm13, %xmm14, %xmm13
8:
	vmovsd			%xmm13, 24(%r10)
//	vmovddup		%xmm13, %xmm13
//	vperm2f128		$ 0x00, %ymm13, %ymm13, %ymm13
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmulpd			%ymm3, %ymm13, %ymm3

	jmp				0f

1:
	vxorpd			%ymm13, %ymm13, %ymm13
	jmp				2b

3:
	vxorpd			%ymm13, %ymm13, %ymm13
	jmp				4b

5:
	vxorpd			%ymm13, %ymm13, %ymm13
	jmp				6b

7:
	vxorpd			%ymm13, %ymm13, %ymm13
	jmp				8b

0:
	#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dpotrf_4x4_vs_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = right
// uplo = lower
// tran = transposed
// requires explicit inverse of diagonal
//
// input arguments:
// r10  <- E
// r11  <- inv_diag_E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm12 <- dirty
// ymm13 <- dirty
//
// output arguments:
// r10  <- E
// r11  <- inv_diag_E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm12 <- dirty
// ymm13 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RLT_INV_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rlt_inv_4x4_lib4)
#endif
	
	vbroadcastsd	0(%r11), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm0
	vbroadcastsd	8(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm1
	vbroadcastsd	16(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm2
	vbroadcastsd	24(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm3

	vbroadcastsd	8(%r11), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm1
	vbroadcastsd	48(%r10), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm2
	vbroadcastsd	56(%r10), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm3

	vbroadcastsd	16(%r11), %ymm13
	vmulpd			%ymm2, %ymm13, %ymm2
	vbroadcastsd	88(%r10), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm3

	vbroadcastsd	24(%r11), %ymm13
	vmulpd			%ymm3, %ymm13, %ymm3

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rlt_inv_4x4_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = right
// uplo = lower
// tran = transposed
// requires explicit inverse of diagonal
//
// input arguments:
// r10  <- D
// r11  <- inv_diag_D
// r12d <- kn
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm12 <- dirty
// ymm13 <- dirty
//
// output arguments:
// r10  <- D
// r11  <- inv_diag_D
// r12d <- kn
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm12 <- dirty
// ymm13 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RLT_INV_4X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rlt_inv_4x4_vs_lib4)
#endif
	
	vbroadcastsd	0(%r11), %ymm13
	vmulpd			%ymm0, %ymm13, %ymm0

	cmpl			$ 2, %r12d
	jl				0f // ret

	vbroadcastsd	8(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm1
	vbroadcastsd	8(%r11), %ymm13
	vmulpd			%ymm1, %ymm13, %ymm1

	cmpl			$ 3, %r12d
	jl				0f // ret

	vbroadcastsd	16(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm2
	vbroadcastsd	48(%r10), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm2
	vbroadcastsd	16(%r11), %ymm13
	vmulpd			%ymm2, %ymm13, %ymm2

	cmpl			$ 4, %r12d
	jl				0f // ret

	vbroadcastsd	24(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm3
	vbroadcastsd	56(%r10), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm3
	vbroadcastsd	88(%r10), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm3
	vbroadcastsd	24(%r11), %ymm13
	vmulpd			%ymm3, %ymm13, %ymm3

0:
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rlt_inv_4x4_vs_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = right
// uplo = lower
// tran = transposed
// unit diagonal
//
// input arguments:
// r10  <- E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm12 <- dirty
// ymm13 <- dirty
//
// output arguments:
// r10  <- E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm12 <- dirty
// ymm13 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RLT_ONE_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rlt_one_4x4_lib4)
#endif
	
	vbroadcastsd	8(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm1

	vbroadcastsd	16(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm2
	vbroadcastsd	48(%r10), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm2

	vbroadcastsd	24(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm3
	vbroadcastsd	56(%r10), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm3
	vbroadcastsd	88(%r10), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm3

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rlt_one_4x4_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = right
// uplo = lower
// tran = transposed
// unit diagonal
//
// input arguments:
// r10  <- D
// r11d <- kn
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm12 <- dirty
// ymm13 <- dirty
//
// output arguments:
// r10  <- D
// r11d <- kn
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm12 <- dirty
// ymm13 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RLT_ONE_4X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rlt_one_4x4_vs_lib4)
#endif
	
	cmpl			$ 2, %r11d

	jl				0f // ret

	vbroadcastsd	8(%r10), %ymm13
	cmpl			$ 3, %r11d
	vfnmadd231pd	%ymm0, %ymm13, %ymm1

	jl				0f // ret

	vbroadcastsd	16(%r10), %ymm13
	cmpl			$ 4, %r11d
	vfnmadd231pd	%ymm0, %ymm13, %ymm2
	vbroadcastsd	48(%r10), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm2

	jl				0f // ret

	vbroadcastsd	24(%r10), %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm3
	vbroadcastsd	56(%r10), %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm3
	vbroadcastsd	88(%r10), %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm3

0:
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rlt_one_4x4_vs_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = right
// uplo = upper
// tran = transposed
// requires explicit inverse of diagonal
//
// input arguments:
// r10  <- E
// r11  <- inv_diag_E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm12 <- dirty
// ymm13 <- dirty
//
// output arguments:
// r10  <- E
// r11  <- inv_diag_E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm12 <- dirty
// ymm13 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RUT_INV_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rut_inv_4x4_lib4)
#endif
	
	vbroadcastsd	24(%r11), %ymm12
	vmulpd			%ymm3, %ymm12, %ymm3
	vbroadcastsd	112(%r10), %ymm12
	vfnmadd231pd	%ymm3, %ymm12, %ymm2
	vbroadcastsd	104(%r10), %ymm12
	vfnmadd231pd	%ymm3, %ymm12, %ymm1
	vbroadcastsd	96(%r10), %ymm12
	vfnmadd231pd	%ymm3, %ymm12, %ymm0

	vbroadcastsd	16(%r11), %ymm12
	vmulpd			%ymm2, %ymm12, %ymm2
	vbroadcastsd	72(%r10), %ymm12
	vfnmadd231pd	%ymm2, %ymm12, %ymm1
	vbroadcastsd	64(%r10), %ymm12
	vfnmadd231pd	%ymm2, %ymm12, %ymm0

	vbroadcastsd	8(%r11), %ymm12
	vmulpd			%ymm1, %ymm12, %ymm1
	vbroadcastsd	32(%r10), %ymm12
	vfnmadd231pd	%ymm1, %ymm12, %ymm0

	vbroadcastsd	0(%r11), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm0

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rut_inv_4x4_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = right
// uplo = lower
// tran = transposed
// requires explicit inverse of diagonal
//
// input arguments:
// r10  <- D
// r11  <- inv_diag_D
// r12d <- kn
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm12 <- dirty
// ymm13 <- dirty
//
// output arguments:
// r10  <- D
// r11  <- inv_diag_D
// r12d <- kn
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm12 <- dirty
// ymm13 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RUT_INV_4X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rut_inv_4x4_vs_lib4)
#endif
	
	cmpl			$ 3, %r12d
	jle				0f

	vbroadcastsd	24(%r11), %ymm12
	vmulpd			%ymm3, %ymm12, %ymm3
	vbroadcastsd	112(%r10), %ymm12
	vfnmadd231pd	%ymm3, %ymm12, %ymm2
	vbroadcastsd	104(%r10), %ymm12
	vfnmadd231pd	%ymm3, %ymm12, %ymm1
	vbroadcastsd	96(%r10), %ymm12
	vfnmadd231pd	%ymm3, %ymm12, %ymm0

0:
	cmpl			$ 2, %r12d
	jle				1f

	vbroadcastsd	16(%r11), %ymm12
	vmulpd			%ymm2, %ymm12, %ymm2
	vbroadcastsd	72(%r10), %ymm12
	vfnmadd231pd	%ymm2, %ymm12, %ymm1
	vbroadcastsd	64(%r10), %ymm12
	vfnmadd231pd	%ymm2, %ymm12, %ymm0

1:
	cmpl			$ 1, %r12d
	jle				2f

	vbroadcastsd	8(%r11), %ymm12
	vmulpd			%ymm1, %ymm12, %ymm1
	vbroadcastsd	32(%r10), %ymm12
	vfnmadd231pd	%ymm1, %ymm12, %ymm0

2:

	vbroadcastsd	0(%r11), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm0

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rut_inv_4x4_vs_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = right
// uplo = upper
// tran = transposed
// unit diagonal
//
// input arguments:
// r10  <- E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm12 <- dirty
// ymm13 <- dirty
//
// output arguments:
// r10  <- E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm12 <- dirty
// ymm13 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RUT_ONE_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rut_one_4x4_lib4)
#endif
	
	vbroadcastsd	112(%r10), %ymm12
	vfnmadd231pd	%ymm3, %ymm12, %ymm2
	vbroadcastsd	104(%r10), %ymm12
	vfnmadd231pd	%ymm3, %ymm12, %ymm1
	vbroadcastsd	96(%r10), %ymm12
	vfnmadd231pd	%ymm3, %ymm12, %ymm0

	vbroadcastsd	72(%r10), %ymm12
	vfnmadd231pd	%ymm2, %ymm12, %ymm1
	vbroadcastsd	64(%r10), %ymm12
	vfnmadd231pd	%ymm2, %ymm12, %ymm0

	vbroadcastsd	32(%r10), %ymm12
	vfnmadd231pd	%ymm1, %ymm12, %ymm0

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rut_one_4x4_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = right
// uplo = lower
// tran = transposed
// unit diagonal
//
// input arguments:
// r10  <- D
// r11d <- kn
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm12 <- dirty
// ymm13 <- dirty
//
// output arguments:
// r10  <- D
// r11d <- kn
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm12 <- dirty
// ymm13 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RUT_ONE_4X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rut_one_4x4_vs_lib4)
#endif
	
	cmpl			$ 3, %r11d
	jle				0f

	vbroadcastsd	112(%r10), %ymm12
	vfnmadd231pd	%ymm3, %ymm12, %ymm2
	vbroadcastsd	104(%r10), %ymm12
	vfnmadd231pd	%ymm3, %ymm12, %ymm1
	vbroadcastsd	96(%r10), %ymm12
	vfnmadd231pd	%ymm3, %ymm12, %ymm0

0:
	cmpl			$ 2, %r11d
	jle				1f

	vbroadcastsd	72(%r10), %ymm12
	vfnmadd231pd	%ymm2, %ymm12, %ymm1
	vbroadcastsd	64(%r10), %ymm12
	vfnmadd231pd	%ymm2, %ymm12, %ymm0

1:
	cmpl			$ 1, %r11d
	jle				2f

	vbroadcastsd	32(%r10), %ymm12
	vfnmadd231pd	%ymm1, %ymm12, %ymm0

2:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rut_one_4x4_vs_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = right
// uplo = up
// tran = normal
// requires explicit inverse of diagonal
//
// input arguments:
// r10  <- E
// r11  <- inv_diag_E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10  <- E
// r11  <- inv_diag_E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RUN_INV_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_run_inv_4x4_lib4)
#endif

	// first column
	vbroadcastsd	0(%r11), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm0

	// second column
	vbroadcastsd	32(%r10), %ymm12
	vfnmadd231pd	%ymm0, %ymm12, %ymm1
	vbroadcastsd	8(%r11), %ymm12
	vmulpd			%ymm1, %ymm12, %ymm1

	// third column
	vbroadcastsd	64(%r10), %ymm12
	vfnmadd231pd	%ymm0, %ymm12, %ymm2
	vbroadcastsd	72(%r10), %ymm12
	vfnmadd231pd	%ymm1, %ymm12, %ymm2
	vbroadcastsd	16(%r11), %ymm12
	vmulpd			%ymm2, %ymm12, %ymm2

	// fourth column
	vbroadcastsd	96(%r10), %ymm12
	vfnmadd231pd	%ymm0, %ymm12, %ymm3
	vbroadcastsd	104(%r10), %ymm12
	vfnmadd231pd	%ymm1, %ymm12, %ymm3
	vbroadcastsd	112(%r10), %ymm12
	vfnmadd231pd	%ymm2, %ymm12, %ymm3
	vbroadcastsd	24(%r11), %ymm12
	vmulpd			%ymm3, %ymm12, %ymm3

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_run_inv_4x4_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = left
// uplo = lower
// tran = normal
// not-unit diagonal
//
// input arguments:
// r10  <- E
// r11  <- inv_diag_E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10  <- E
// r11  <- inv_diag_E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_LLN_INV_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_lln_inv_4x4_lib4)
#endif

	vxorpd			%ymm14, %ymm14, %ymm14
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC04(%rip), %ymm15 // 1.0
#elif defined(OS_MAC)
	vmovapd			LC04(%rip), %ymm15 // 1.0
#endif

	vbroadcastsd	0(%r11), %ymm12
	vblendpd		$ 0x1, %ymm12, %ymm15, %ymm12
	vmulpd			%ymm12, %ymm0, %ymm0
	vmulpd			%ymm12, %ymm1, %ymm1
	vmulpd			%ymm12, %ymm2, %ymm2
	vmulpd			%ymm12, %ymm3, %ymm3
	vmovapd			0(%r10), %ymm12
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm12
	vpermpd			$ 0x00, %ymm0, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm0
	vpermpd			$ 0x00, %ymm1, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm1
	vpermpd			$ 0x00, %ymm2, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm2
	vpermpd			$ 0x00, %ymm3, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm3

	vbroadcastsd	8(%r11), %ymm12
	vblendpd		$ 0x2, %ymm12, %ymm15, %ymm12
	vmulpd			%ymm12, %ymm0, %ymm0
	vmulpd			%ymm12, %ymm1, %ymm1
	vmulpd			%ymm12, %ymm2, %ymm2
	vmulpd			%ymm12, %ymm3, %ymm3
	vmovapd			32(%r10), %ymm12
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm12
	vpermpd			$ 0x55, %ymm0, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm0
	vpermpd			$ 0x55, %ymm1, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm1
	vpermpd			$ 0x55, %ymm2, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm2
	vpermpd			$ 0x55, %ymm3, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm3

	vbroadcastsd	16(%r11), %ymm12
	vblendpd		$ 0x4, %ymm12, %ymm15, %ymm12
	vmulpd			%ymm12, %ymm0, %ymm0
	vmulpd			%ymm12, %ymm1, %ymm1
	vmulpd			%ymm12, %ymm2, %ymm2
	vmulpd			%ymm12, %ymm3, %ymm3
	vmovapd			64(%r10), %ymm12
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm12
	vpermpd			$ 0xaa, %ymm0, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm0
	vpermpd			$ 0xaa, %ymm1, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm1
	vpermpd			$ 0xaa, %ymm2, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm2
	vpermpd			$ 0xaa, %ymm3, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm3

	vbroadcastsd	24(%r11), %ymm12
	vblendpd		$ 0x8, %ymm12, %ymm15, %ymm12
	vmulpd			%ymm12, %ymm0, %ymm0
	vmulpd			%ymm12, %ymm1, %ymm1
	vmulpd			%ymm12, %ymm2, %ymm2
	vmulpd			%ymm12, %ymm3, %ymm3

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_lln_inv_4x4_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = left
// uplo = lower
// tran = normal
// unit diagonal
//
// input arguments:
// r10  <- E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10  <- E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_LLN_ONE_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_lln_one_4x4_lib4)
#endif

	vxorpd			%ymm14, %ymm14, %ymm14

	vmovapd			0(%r10), %ymm12
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm12
	vpermpd			$ 0x00, %ymm0, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm0
	vpermpd			$ 0x00, %ymm1, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm1
	vpermpd			$ 0x00, %ymm2, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm2
	vpermpd			$ 0x00, %ymm3, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm3

	vmovapd			32(%r10), %ymm12
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm12
	vpermpd			$ 0x55, %ymm0, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm0
	vpermpd			$ 0x55, %ymm1, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm1
	vpermpd			$ 0x55, %ymm2, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm2
	vpermpd			$ 0x55, %ymm3, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm3

	vmovapd			64(%r10), %ymm12
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm12
	vpermpd			$ 0xaa, %ymm0, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm0
	vpermpd			$ 0xaa, %ymm1, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm1
	vpermpd			$ 0xaa, %ymm2, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm2
	vpermpd			$ 0xaa, %ymm3, %ymm13
	vfnmadd231pd	%ymm12, %ymm13, %ymm3

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_lln_one_4x4_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = left
// uplo = upper
// tran = normal
// requires explicit inverse of diagonal
//
// input arguments:
// r10  <- E
// r11  <- inv_diag_E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10  <- E
// r11  <- inv_diag_E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_LUN_INV_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_lun_inv_4x4_lib4)
#endif
	
	vmovapd			96(%r10), %ymm13
	vxorpd			%ymm14, %ymm14, %ymm14 // 0.0
	vblendpd		$ 0x7, %ymm13, %ymm14, %ymm13
	vbroadcastsd	24(%r11), %ymm12

	vperm2f128		$ 0x11, %ymm0, %ymm0, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm0, %ymm0
	vfnmadd231pd	%ymm13, %ymm14, %ymm0

	vperm2f128		$ 0x11, %ymm1, %ymm1, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm1, %ymm1
	vfnmadd231pd	%ymm13, %ymm14, %ymm1

	vperm2f128		$ 0x11, %ymm2, %ymm2, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm2, %ymm2
	vfnmadd231pd	%ymm13, %ymm14, %ymm2

	vperm2f128		$ 0x11, %ymm3, %ymm3, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm3, %ymm3
	vfnmadd231pd	%ymm13, %ymm14, %ymm3


	vxorpd			%ymm13, %ymm13, %ymm13 // 0.0
	vmovapd			64(%r10), %xmm13
	vbroadcastsd	16(%r11), %ymm12

	vperm2f128		$ 0x11, %ymm0, %ymm0, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm0, %ymm0
	vfnmadd231pd	%ymm13, %ymm14, %ymm0

	vperm2f128		$ 0x11, %ymm1, %ymm1, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm1, %ymm1
	vfnmadd231pd	%ymm13, %ymm14, %ymm1

	vperm2f128		$ 0x11, %ymm2, %ymm2, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm2, %ymm2
	vfnmadd231pd	%ymm13, %ymm14, %ymm2

	vperm2f128		$ 0x11, %ymm3, %ymm3, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm3, %ymm3
	vfnmadd231pd	%ymm13, %ymm14, %ymm3


	vxorpd			%ymm13, %ymm13, %ymm13 // 0.0
	vmovsd			32(%r10), %xmm13
	vbroadcastsd	8(%r11), %ymm12

	vpermilpd		$ 0xf, %ymm0, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm0, %ymm0
	vfnmadd231pd	%ymm13, %ymm14, %ymm0

	vpermilpd		$ 0xf, %ymm1, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm1, %ymm1
	vfnmadd231pd	%ymm13, %ymm14, %ymm1

	vpermilpd		$ 0xf, %ymm2, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm2, %ymm2
	vfnmadd231pd	%ymm13, %ymm14, %ymm2

	vpermilpd		$ 0xf, %ymm3, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm3, %ymm3
	vfnmadd231pd	%ymm13, %ymm14, %ymm3


	vbroadcastsd	0(%r11), %ymm12

	vmulpd			%ymm0, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm0, %ymm0

	vmulpd			%ymm1, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm1, %ymm1

	vmulpd			%ymm2, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm2, %ymm2

	vmulpd			%ymm3, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm3, %ymm3

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_lun_inv_4x4_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = left
// uplo = upper
// tran = normal
// requires explicit inverse of diagonal
//
// input arguments:
// r10  <- E
// r11  <- inv_diag_E
// r12  <- km
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10  <- E
// r11  <- inv_diag_E
// r12  <- km
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_LUN_INV_4X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_lun_inv_4x4_vs_lib4)
#endif
	
	cmpl	$ 3, %r12d
	jle		0f

	vmovapd			96(%r10), %ymm13
	vxorpd			%ymm14, %ymm14, %ymm14 // 0.0
	vblendpd		$ 0x7, %ymm13, %ymm14, %ymm13
	vbroadcastsd	24(%r11), %ymm12

	vperm2f128		$ 0x11, %ymm0, %ymm0, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm0, %ymm0
	vfnmadd231pd	%ymm13, %ymm14, %ymm0

	vperm2f128		$ 0x11, %ymm1, %ymm1, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm1, %ymm1
	vfnmadd231pd	%ymm13, %ymm14, %ymm1

	vperm2f128		$ 0x11, %ymm2, %ymm2, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm2, %ymm2
	vfnmadd231pd	%ymm13, %ymm14, %ymm2

	vperm2f128		$ 0x11, %ymm3, %ymm3, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x8, %ymm14, %ymm3, %ymm3
	vfnmadd231pd	%ymm13, %ymm14, %ymm3

0:
	cmpl	$ 2, %r12d
	jle		1f

	vxorpd			%ymm13, %ymm13, %ymm13 // 0.0
	vmovapd			64(%r10), %xmm13
	vbroadcastsd	16(%r11), %ymm12

	vperm2f128		$ 0x11, %ymm0, %ymm0, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm0, %ymm0
	vfnmadd231pd	%ymm13, %ymm14, %ymm0

	vperm2f128		$ 0x11, %ymm1, %ymm1, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm1, %ymm1
	vfnmadd231pd	%ymm13, %ymm14, %ymm1

	vperm2f128		$ 0x11, %ymm2, %ymm2, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm2, %ymm2
	vfnmadd231pd	%ymm13, %ymm14, %ymm2

	vperm2f128		$ 0x11, %ymm3, %ymm3, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x4, %ymm14, %ymm3, %ymm3
	vfnmadd231pd	%ymm13, %ymm14, %ymm3

1:
	cmpl	$ 1, %r12d
	jle		2f

	vxorpd			%ymm13, %ymm13, %ymm13 // 0.0
	vmovsd			32(%r10), %xmm13
	vbroadcastsd	8(%r11), %ymm12

	vpermilpd		$ 0xf, %ymm0, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm0, %ymm0
	vfnmadd231pd	%ymm13, %ymm14, %ymm0

	vpermilpd		$ 0xf, %ymm1, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm1, %ymm1
	vfnmadd231pd	%ymm13, %ymm14, %ymm1

	vpermilpd		$ 0xf, %ymm2, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm2, %ymm2
	vfnmadd231pd	%ymm13, %ymm14, %ymm2

	vpermilpd		$ 0xf, %ymm3, %ymm14
	vmulpd			%ymm14, %ymm12, %ymm14
	vblendpd		$ 0x2, %ymm14, %ymm3, %ymm3
	vfnmadd231pd	%ymm13, %ymm14, %ymm3

2:

	vbroadcastsd	0(%r11), %ymm12

	vmulpd			%ymm0, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm0, %ymm0

	vmulpd			%ymm1, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm1, %ymm1

	vmulpd			%ymm2, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm2, %ymm2

	vmulpd			%ymm3, %ymm12, %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm3, %ymm3

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_lun_inv_4x4_vs_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = left
// uplo = upper
// tran = normal
// unit diagonal
//
// input arguments:
// r10  <- E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10  <- E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_LUN_ONE_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_lun_one_4x4_lib4)
#endif
	
	vmovapd			96(%r10), %ymm13
	vxorpd			%ymm14, %ymm14, %ymm14 // 0.0
	vblendpd		$ 0x7, %ymm13, %ymm14, %ymm13
//	vbroadcastsd	24(%r11), %ymm12

	vperm2f128		$ 0x11, %ymm0, %ymm0, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
//	vmulpd			%ymm14, %ymm12, %ymm14
//	vblendpd		$ 0x8, %ymm14, %ymm0, %ymm0
	vfnmadd231pd	%ymm13, %ymm14, %ymm0

	vperm2f128		$ 0x11, %ymm1, %ymm1, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
//	vmulpd			%ymm14, %ymm12, %ymm14
//	vblendpd		$ 0x8, %ymm14, %ymm1, %ymm1
	vfnmadd231pd	%ymm13, %ymm14, %ymm1

	vperm2f128		$ 0x11, %ymm2, %ymm2, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
//	vmulpd			%ymm14, %ymm12, %ymm14
//	vblendpd		$ 0x8, %ymm14, %ymm2, %ymm2
	vfnmadd231pd	%ymm13, %ymm14, %ymm2

	vperm2f128		$ 0x11, %ymm3, %ymm3, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
//	vmulpd			%ymm14, %ymm12, %ymm14
//	vblendpd		$ 0x8, %ymm14, %ymm3, %ymm3
	vfnmadd231pd	%ymm13, %ymm14, %ymm3


	vxorpd			%ymm13, %ymm13, %ymm13 // 0.0
	vmovapd			64(%r10), %xmm13
//	vbroadcastsd	16(%r11), %ymm12

	vperm2f128		$ 0x11, %ymm0, %ymm0, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
//	vmulpd			%ymm14, %ymm12, %ymm14
//	vblendpd		$ 0x4, %ymm14, %ymm0, %ymm0
	vfnmadd231pd	%ymm13, %ymm14, %ymm0

	vperm2f128		$ 0x11, %ymm1, %ymm1, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
//	vmulpd			%ymm14, %ymm12, %ymm14
//	vblendpd		$ 0x4, %ymm14, %ymm1, %ymm1
	vfnmadd231pd	%ymm13, %ymm14, %ymm1

	vperm2f128		$ 0x11, %ymm2, %ymm2, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
//	vmulpd			%ymm14, %ymm12, %ymm14
//	vblendpd		$ 0x4, %ymm14, %ymm2, %ymm2
	vfnmadd231pd	%ymm13, %ymm14, %ymm2

	vperm2f128		$ 0x11, %ymm3, %ymm3, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
//	vmulpd			%ymm14, %ymm12, %ymm14
//	vblendpd		$ 0x4, %ymm14, %ymm3, %ymm3
	vfnmadd231pd	%ymm13, %ymm14, %ymm3


	vxorpd			%ymm13, %ymm13, %ymm13 // 0.0
	vmovsd			32(%r10), %xmm13
//	vbroadcastsd	8(%r11), %ymm12

	vpermilpd		$ 0xf, %ymm0, %ymm14
//	vmulpd			%ymm14, %ymm12, %ymm14
//	vblendpd		$ 0x2, %ymm14, %ymm0, %ymm0
	vfnmadd231pd	%ymm13, %ymm14, %ymm0

	vpermilpd		$ 0xf, %ymm1, %ymm14
//	vmulpd			%ymm14, %ymm12, %ymm14
//	vblendpd		$ 0x2, %ymm14, %ymm1, %ymm1
	vfnmadd231pd	%ymm13, %ymm14, %ymm1

	vpermilpd		$ 0xf, %ymm2, %ymm14
//	vmulpd			%ymm14, %ymm12, %ymm14
//	vblendpd		$ 0x2, %ymm14, %ymm2, %ymm2
	vfnmadd231pd	%ymm13, %ymm14, %ymm2

	vpermilpd		$ 0xf, %ymm3, %ymm14
//	vmulpd			%ymm14, %ymm12, %ymm14
//	vblendpd		$ 0x2, %ymm14, %ymm3, %ymm3
	vfnmadd231pd	%ymm13, %ymm14, %ymm3


//	vbroadcastsd	0(%r11), %ymm12

//	vmulpd			%ymm0, %ymm12, %ymm14
//	vblendpd		$ 0x1, %ymm14, %ymm0, %ymm0

//	vmulpd			%ymm1, %ymm12, %ymm14
//	vblendpd		$ 0x1, %ymm14, %ymm1, %ymm1

//	vmulpd			%ymm2, %ymm12, %ymm14
//	vblendpd		$ 0x1, %ymm14, %ymm2, %ymm2

//	vmulpd			%ymm3, %ymm12, %ymm14
//	vblendpd		$ 0x1, %ymm14, %ymm3, %ymm3

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_lun_one_4x4_lib4)
#endif





// common inner routine with file scope
//
// triangular substitution:
// side = left
// uplo = upper
// tran = normal
// unit diagonal
//
// input arguments:
// r10  <- E
// r11  <- km
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10  <- E
// r11  <- km
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_LUN_ONE_4X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_lun_one_4x4_vs_lib4)
#endif
	
	cmpl	$ 3, %r11d
	jle		0f

	vmovapd			96(%r10), %ymm13
	vxorpd			%ymm14, %ymm14, %ymm14 // 0.0
	vblendpd		$ 0x7, %ymm13, %ymm14, %ymm13
//	vbroadcastsd	24(%r11), %ymm12

	vperm2f128		$ 0x11, %ymm0, %ymm0, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
//	vmulpd			%ymm14, %ymm12, %ymm14
//	vblendpd		$ 0x8, %ymm14, %ymm0, %ymm0
	vfnmadd231pd	%ymm13, %ymm14, %ymm0

	vperm2f128		$ 0x11, %ymm1, %ymm1, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
//	vmulpd			%ymm14, %ymm12, %ymm14
//	vblendpd		$ 0x8, %ymm14, %ymm1, %ymm1
	vfnmadd231pd	%ymm13, %ymm14, %ymm1

	vperm2f128		$ 0x11, %ymm2, %ymm2, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
//	vmulpd			%ymm14, %ymm12, %ymm14
//	vblendpd		$ 0x8, %ymm14, %ymm2, %ymm2
	vfnmadd231pd	%ymm13, %ymm14, %ymm2

	vperm2f128		$ 0x11, %ymm3, %ymm3, %ymm14
	vpermilpd		$ 0xf, %ymm14, %ymm14
//	vmulpd			%ymm14, %ymm12, %ymm14
//	vblendpd		$ 0x8, %ymm14, %ymm3, %ymm3
	vfnmadd231pd	%ymm13, %ymm14, %ymm3

0:
	cmpl	$ 2, %r11d
	jle		1f

	vxorpd			%ymm13, %ymm13, %ymm13 // 0.0
	vmovapd			64(%r10), %xmm13
//	vbroadcastsd	16(%r11), %ymm12

	vperm2f128		$ 0x11, %ymm0, %ymm0, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
//	vmulpd			%ymm14, %ymm12, %ymm14
//	vblendpd		$ 0x4, %ymm14, %ymm0, %ymm0
	vfnmadd231pd	%ymm13, %ymm14, %ymm0

	vperm2f128		$ 0x11, %ymm1, %ymm1, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
//	vmulpd			%ymm14, %ymm12, %ymm14
//	vblendpd		$ 0x4, %ymm14, %ymm1, %ymm1
	vfnmadd231pd	%ymm13, %ymm14, %ymm1

	vperm2f128		$ 0x11, %ymm2, %ymm2, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
//	vmulpd			%ymm14, %ymm12, %ymm14
//	vblendpd		$ 0x4, %ymm14, %ymm2, %ymm2
	vfnmadd231pd	%ymm13, %ymm14, %ymm2

	vperm2f128		$ 0x11, %ymm3, %ymm3, %ymm14
	vpermilpd		$ 0x0, %ymm14, %ymm14
//	vmulpd			%ymm14, %ymm12, %ymm14
//	vblendpd		$ 0x4, %ymm14, %ymm3, %ymm3
	vfnmadd231pd	%ymm13, %ymm14, %ymm3

1:
	cmpl	$ 1, %r11d
	jle		2f

	vxorpd			%ymm13, %ymm13, %ymm13 // 0.0
	vmovsd			32(%r10), %xmm13
//	vbroadcastsd	8(%r11), %ymm12

	vpermilpd		$ 0xf, %ymm0, %ymm14
//	vmulpd			%ymm14, %ymm12, %ymm14
//	vblendpd		$ 0x2, %ymm14, %ymm0, %ymm0
	vfnmadd231pd	%ymm13, %ymm14, %ymm0

	vpermilpd		$ 0xf, %ymm1, %ymm14
//	vmulpd			%ymm14, %ymm12, %ymm14
//	vblendpd		$ 0x2, %ymm14, %ymm1, %ymm1
	vfnmadd231pd	%ymm13, %ymm14, %ymm1

	vpermilpd		$ 0xf, %ymm2, %ymm14
//	vmulpd			%ymm14, %ymm12, %ymm14
//	vblendpd		$ 0x2, %ymm14, %ymm2, %ymm2
	vfnmadd231pd	%ymm13, %ymm14, %ymm2

	vpermilpd		$ 0xf, %ymm3, %ymm14
//	vmulpd			%ymm14, %ymm12, %ymm14
//	vblendpd		$ 0x2, %ymm14, %ymm3, %ymm3
	vfnmadd231pd	%ymm13, %ymm14, %ymm3

2:

//	vbroadcastsd	0(%r11), %ymm12

//	vmulpd			%ymm0, %ymm12, %ymm14
//	vblendpd		$ 0x1, %ymm14, %ymm0, %ymm0

//	vmulpd			%ymm1, %ymm12, %ymm14
//	vblendpd		$ 0x1, %ymm14, %ymm1, %ymm1

//	vmulpd			%ymm2, %ymm12, %ymm14
//	vblendpd		$ 0x1, %ymm14, %ymm2, %ymm2

//	vmulpd			%ymm3, %ymm12, %ymm14
//	vblendpd		$ 0x1, %ymm14, %ymm3, %ymm3

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_lun_one_4x4_vs_lib4)
#endif





// common inner routine with file scope
//
// LU factorization without pivoting
//
// input arguments:
// r10  <- inv_diag_E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10  <- inv_diag_E
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DGETRF_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_edge_dgetrf_4x4_lib4)
#endif
	
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC04(%rip), %xmm14 // 1.0
#elif defined(OS_MAC)
	vmovsd			LC04(%rip), %xmm14 // 1.0
#endif
//	vmovddup		%xmm14, %xmm14

	// first column
//	vblendpd		$ 0x1, %ymm0, %ymm12, %ymm12
	vmovapd			%ymm0, %ymm12
	vdivsd			%xmm0, %xmm14, %xmm13
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmovsd			%xmm13, 0(%r10)
	vmulpd			%ymm0, %ymm13, %ymm0
	vblendpd		$ 0x1, %ymm12, %ymm0, %ymm0

	// second column
	vpermpd			$ 0x00, %ymm1, %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm1
	vblendpd		$ 0x2, %ymm1, %ymm13, %ymm12

	vpermilpd		$ 0x3, %xmm1, %xmm13
	vdivsd			%xmm13, %xmm14, %xmm13
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmovsd			%xmm13, 8(%r10)
	vmulpd			%ymm1, %ymm13, %ymm1
	vblendpd		$ 0x3, %ymm12, %ymm1, %ymm1

	// third column
	vpermpd			$ 0x00, %ymm2, %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm2
	vblendpd		$ 0x2, %ymm2, %ymm13, %ymm12

	vpermpd			$ 0x55, %ymm2, %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm2
	vblendpd		$ 0x4, %ymm2, %ymm12, %ymm12

	vpermpd			$ 0xaa, %ymm2, %ymm13
	vdivsd			%xmm13, %xmm14, %xmm13
	vpermpd			$ 0x00, %ymm13, %ymm13
	vmovsd			%xmm13, 16(%r10)
	vmulpd			%ymm2, %ymm13, %ymm2
	vblendpd		$ 0x7, %ymm12, %ymm2, %ymm2

	// fourth column
	vpermpd			$ 0x00, %ymm3, %ymm13
	vfnmadd231pd	%ymm0, %ymm13, %ymm3
	vblendpd		$ 0x2, %ymm3, %ymm13, %ymm12

	vpermpd			$ 0x55, %ymm3, %ymm13
	vfnmadd231pd	%ymm1, %ymm13, %ymm3
	vblendpd		$ 0x4, %ymm3, %ymm12, %ymm12

	vpermpd			$ 0xaa, %ymm3, %ymm13
	vfnmadd231pd	%ymm2, %ymm13, %ymm3
	vblendpd		$ 0x8, %ymm3, %ymm12, %ymm12
	
	vpermpd			$ 0xff, %ymm3, %ymm13
	vdivsd			%xmm13, %xmm14, %xmm13
//	vperm2f128		$ 0x00, %ymm13, %ymm13, %ymm13
	vmovsd			%xmm13, 24(%r10)
//	vmulpd			%ymm3, %ymm13, %ymm3
	vblendpd		$ 0x7, %ymm12, %ymm3, %ymm3

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dgetrf_4x4_lib4)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// ymm0 <- [d00 d11 d22 d33]
// ymm1 <- [d01 d10 d23 d32]
// ymm2 <- [d03 d12 d21 d30]
// ymm3 <- [d02 d13 d20 d31]
//
// output arguments:
// r10  <- D
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_4x4_lib4)
#endif
	
	vmovapd %ymm0,  0(%r10)
	vmovapd %ymm1, 32(%r10)
	vmovapd %ymm2, 64(%r10)
	vmovapd %ymm3, 96(%r10)
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4x4_lib4)
#endif





// common inner routine with file scope
//
// store n lower triangular
//
// input arguments:
// r10   <- D
// ymm0  <- [d00 d11 d22 d33]
// ymm1  <- [d01 d10 d23 d32]
// ymm2  <- [d03 d12 d21 d30]
// ymm3  <- [d02 d13 d20 d31]
// ymm14 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10   <- D
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_l_4x4_lib4)
#endif
	
	vmovapd		%ymm0, 0(%r10)
	vmovapd		32(%r10), %ymm14
	vblendpd	$ 0x1, %ymm14, %ymm1, %ymm1	
	vmovapd		%ymm1, 32(%r10)
	vmovapd		64(%r10), %ymm14
	vblendpd	$ 0x3, %ymm14, %ymm2, %ymm2	
	vmovapd		%ymm2, 64(%r10)
	vmovapd		96(%r10), %ymm14
	vblendpd	$ 0x7, %ymm14, %ymm3, %ymm3	
	vmovapd		%ymm3, 96(%r10)

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_4x4_lib4)
#endif





// common inner routine with file scope
//
// store n upper triangular
//
// input arguments:
// r10   <- D
// ymm0  <- [d00 d11 d22 d33]
// ymm1  <- [d01 d10 d23 d32]
// ymm2  <- [d03 d12 d21 d30]
// ymm3  <- [d02 d13 d20 d31]
// ymm14 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10   <- D
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_STORE_U_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_u_4x4_lib4)
#endif
	
	//
	vmovapd		0(%r10), %ymm14
	vblendpd	$ 0x1, %ymm0, %ymm14, %ymm0
	vmovapd		%ymm0, 0(%r10)
	//
	vmovapd		32(%r10), %ymm14
	vblendpd	$ 0x3, %ymm1, %ymm14, %ymm1
	vmovapd		%ymm1, 32(%r10)
	//
	vmovapd		64(%r10), %ymm14
	vblendpd	$ 0x7, %ymm2, %ymm14, %ymm2
	vmovapd		%ymm2, 64(%r10)
	//
	vmovapd		%ymm3, 96(%r10)

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_u_4x4_lib4)
#endif





// common inner routine with file scope
//
// store n vs
//
// input arguments:
// r10   <- D
// r11d   <- km
// r12d   <- kn
// ymm0  <- [d00 d11 d22 d33]
// ymm1  <- [d01 d10 d23 d32]
// ymm2  <- [d03 d12 d21 d30]
// ymm3  <- [d02 d13 d20 d31]
// ymm14 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10   <- D
// r11d   <- km
// r12d   <- kn
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_4x4_vs_lib4)
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
	cmpl		$ 2, %r12d
	jl			0f // end
	vmaskmovpd	%ymm1, %ymm15, 32(%r10)
	cmpl		$ 3, %r12d
	jl			0f // end
	vmaskmovpd	%ymm2, %ymm15, 64(%r10)
	je			0f // end
	vmaskmovpd	%ymm3, %ymm15, 96(%r10)

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4x4_vs_lib4)
#endif





// common inner routine with file scope
//
// store n vs lower triangular
//
// input arguments:
// r10   <- D
// r11d   <- km
// r12d   <- kn
// ymm0  <- [d00 d11 d22 d33]
// ymm1  <- [d01 d10 d23 d32]
// ymm2  <- [d03 d12 d21 d30]
// ymm3  <- [d02 d13 d20 d31]
// ymm14 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10   <- D
// r11d   <- km
// r12d   <- kn
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_4X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_l_4x4_vs_lib4)
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
	cmpl		$ 2, %r12d
	jl			0f // end
	vmovapd		32(%r10), %ymm14
	vblendpd	$ 0x1, %ymm14, %ymm1, %ymm1	
	vmaskmovpd	%ymm1, %ymm15, 32(%r10)
	cmpl		$ 3, %r12d
	jl			0f // end
	vmovapd		64(%r10), %ymm14
	vblendpd	$ 0x3, %ymm14, %ymm2, %ymm2	
	vmaskmovpd	%ymm2, %ymm15, 64(%r10)
	je			0f // end
	vmovapd		96(%r10), %ymm14
	vblendpd	$ 0x7, %ymm14, %ymm3, %ymm3	
	vmaskmovpd	%ymm3, %ymm15, 96(%r10)

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_4x4_vs_lib4)
#endif





// common inner routine with file scope
//
// store n vs upper triangular
//
// input arguments:
// r10   <- D
// r11d   <- km
// r12d   <- kn
// ymm0  <- [d00 d11 d22 d33]
// ymm1  <- [d01 d10 d23 d32]
// ymm2  <- [d03 d12 d21 d30]
// ymm3  <- [d02 d13 d20 d31]
// ymm14 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10   <- D
// r11d   <- km
// r12d   <- kn
// ymm0  <- [d00 d10 d20 d30]
// ymm1  <- [d01 d11 d21 d31]
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_STORE_U_4X4_VS_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_u_4x4_vs_lib4)
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

	//
	vmovapd		0(%r10), %ymm14
	vblendpd	$ 0x1, %ymm0, %ymm14, %ymm0
	vmaskmovpd	%ymm0, %ymm15,  0(%r10)
	//
	cmpl		$ 2, %r12d
	jl			0f // end
	vmovapd		32(%r10), %ymm14
	vblendpd	$ 0x3, %ymm1, %ymm14, %ymm1
	vmaskmovpd	%ymm1, %ymm15, 32(%r10)
	//
	cmpl		$ 3, %r12d
	jl			0f // end
	vmovapd		64(%r10), %ymm14
	vblendpd	$ 0x7, %ymm2, %ymm14, %ymm2
	vmaskmovpd	%ymm2, %ymm15, 64(%r10)
	//
	je			0f // end
	vmaskmovpd	%ymm3, %ymm15, 96(%r10)

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_u_4x4_vs_lib4)
#endif





// common inner routine with file scope
//
// store n generalized
//
// input arguments:
// r10  <- offset
// r11  <- D
// r12  <- 4*sdd*sizeof(double)
// r13  <- m0 // row index: start from (inc)
// r14  <- m1 // row index: up to (exc)
// r15  <- n0 // col index: start from (inc)
// rax  <- n1 // col index: up to (exc)
// rbx  <- dirty
// ymm0 <- [d00 d11 d22 d33]
// ymm1 <- [d01 d10 d23 d32]
// ymm2 <- [d03 d12 d21 d30]
// ymm3 <- [d02 d13 d20 d31]
//
// output arguments:
// r10  <- offset
// r11  <- D
// r12  <- 4*sdd*sizeof(double)
// r13  <- m0 // row index: start from (inc)
// r14  <- m1 // row index: up to (exc)
// r15  <- n1-n0
// rax  <- n1-n0
// rbx  <- dirty
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X4_GEN_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_4x4_gen_lib4)
#endif

	// compute mask for rows
	vcvtsi2sd	%r13d, %xmm14, %xmm14
	vcvtsi2sd	%r14d, %xmm15, %xmm15
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

	// shift D and sol for cols
	cmpl	$ 0, %r15d
	jle		0f

	vmovapd		%ymm1, %ymm0
	vmovapd		%ymm2, %ymm1
	vmovapd		%ymm3, %ymm2
	addq		$ 32, %r11

	cmpl	$ 1, %r15d
	jle		0f

	vmovapd		%ymm1, %ymm0
	vmovapd		%ymm2, %ymm1
	addq		$ 32, %r11

	cmpl	$ 2, %r15d
	jle		0f

	vmovapd		%ymm1, %ymm0
	addq		$ 32, %r11

0:

	// compute number of cols
	cmpl	$ 4, %eax
	jle		0f
	movl	$ 4, %eax
0:
	subl	%r15d, %eax
	movl	%eax, %r15d

	cmpl	$ 0, %r15d
	jle		4f


	cmpl	$ 0, %r10d
	jg		0f

	// offset==0

	vmaskmovpd	%ymm0, %ymm15,  0(%r11)
	cmpl		$ 2, %r15d
	jl			4f // end
	vmaskmovpd	%ymm1, %ymm15, 32(%r11)
	cmpl		$ 3, %r15d
	jl			4f // end
	vmaskmovpd	%ymm2, %ymm15, 64(%r11)
	je			4f // end
	vmaskmovpd	%ymm3, %ymm15, 96(%r11)

	jmp		4f

0:
	
	cmpl	$ 1, %r10d
	jg		1f

	// offset==1

	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm12
	vshufpd		$ 0x5, %ymm0, %ymm12, %ymm0

	vperm2f128	$ 0x01, %ymm1, %ymm1, %ymm12
	vshufpd		$ 0x5, %ymm1, %ymm12, %ymm1

	vperm2f128	$ 0x01, %ymm2, %ymm2, %ymm12
	vshufpd		$ 0x5, %ymm2, %ymm12, %ymm2

	vperm2f128	$ 0x01, %ymm3, %ymm3, %ymm12
	vshufpd		$ 0x5, %ymm3, %ymm12, %ymm3

	vperm2f128	$ 0x01, %ymm15, %ymm15, %ymm12
	vshufpd		$ 0x5, %ymm15, %ymm12, %ymm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC08(%rip), %ymm12
	vmovupd		.LC05(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC08(%rip), %ymm12
	vmovupd		LC05(%rip), %ymm13
#endif
	vandpd		%ymm12, %ymm15, %ymm12
	vandpd		%ymm13, %ymm15, %ymm13

	jmp		3f

1:

	cmpl	$ 2, %r10d
	jg		2f

	// offset==2

	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm0

	vperm2f128	$ 0x01, %ymm1, %ymm1, %ymm1

	vperm2f128	$ 0x01, %ymm2, %ymm2, %ymm2

	vperm2f128	$ 0x01, %ymm3, %ymm3, %ymm3

	vperm2f128	$ 0x01, %ymm15, %ymm15, %ymm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC09(%rip), %ymm12
	vmovupd		.LC06(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC09(%rip), %ymm12
	vmovupd		LC06(%rip), %ymm13
#endif
	vandpd		%ymm12, %ymm15, %ymm12
	vandpd		%ymm13, %ymm15, %ymm13

	jmp		3f

2:

	// offset==3

	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm0, %ymm0

	vperm2f128	$ 0x01, %ymm1, %ymm1, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm1, %ymm1

	vperm2f128	$ 0x01, %ymm2, %ymm2, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm2, %ymm2

	vperm2f128	$ 0x01, %ymm3, %ymm3, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm3, %ymm3

	vperm2f128	$ 0x01, %ymm15, %ymm15, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm15, %ymm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC10(%rip), %ymm12
	vmovupd		.LC07(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC10(%rip), %ymm12
	vmovupd		LC07(%rip), %ymm13
#endif
	vandpd		%ymm12, %ymm15, %ymm12
	vandpd		%ymm13, %ymm15, %ymm13

3:

	vmaskmovpd	%ymm0, %ymm12, 0(%r11)
	vmaskmovpd	%ymm0, %ymm13, 0(%r11, %r12)
	cmpl		$ 2, %r15d
	jl			4f // end
	vmaskmovpd	%ymm1, %ymm12, 32(%r11)
	vmaskmovpd	%ymm1, %ymm13, 32(%r11, %r12)
	cmpl		$ 3, %r15d
	jl			4f // end
	vmaskmovpd	%ymm2, %ymm12, 64(%r11)
	vmaskmovpd	%ymm2, %ymm13, 64(%r11, %r12)
	je			4f // end
	vmaskmovpd	%ymm3, %ymm12, 96(%r11)
	vmaskmovpd	%ymm3, %ymm13, 96(%r11, %r12)

4:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4x4_gen_lib4)
#endif





// common inner routine with file scope
//
// store l generalized
//
// input arguments:
// r10  <- offset
// r11  <- D
// r12  <- 4*sdd*sizeof(double)
// r13  <- m0 // row index: start from (inc)
// r14  <- m1 // row index: up to (exc)
// r15  <- n0 // col index: start from (inc)
// rax  <- n1 // col index: up to (exc)
// rbx  <- dirty
// ymm0 <- [d00 d11 d22 d33]
// ymm1 <- [d01 d10 d23 d32]
// ymm2 <- [d03 d12 d21 d30]
// ymm3 <- [d02 d13 d20 d31]
//
// output arguments:
// r10  <- offset
// r11  <- D
// r12  <- 4*sdd*sizeof(double)
// r13  <- m0 // row index: start from (inc)
// r14  <- m1 // row index: up to (exc)
// r15  <- n1-n0
// rax  <- n1-n0
// rbx  <- dirty
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_4X4_GEN_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_l_4x4_gen_lib4)
#endif
	
	// compute mask for rows
	vcvtsi2sd	%r14d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC02(%rip), %ymm12
#elif defined(OS_MAC)
	vmovupd		LC02(%rip), %ymm12
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm12, %ymm15


	// shift D and sol for cols
	cmpl	$ 0, %r15d
	jle		0f

	vmovapd		%ymm1, %ymm0
	vmovapd		%ymm2, %ymm1
	vmovapd		%ymm3, %ymm2
	addq		$ 32, %r11

	cmpl	$ 1, %r15d
	jle		0f

	vmovapd		%ymm1, %ymm0
	vmovapd		%ymm2, %ymm1
	addq		$ 32, %r11

	cmpl	$ 2, %r15d
	jle		0f

	vmovapd		%ymm1, %ymm0
	addq		$ 32, %r11

0:

	// compute number of cols
	cmpl	$ 4, %eax
	jle		0f
	movl	$ 4, %eax
0:
	subl	%r15d, %eax
	movl	%eax, %r15d

	cmpl	$ 0, %r15d
	jle		3f


	// select offset
	cmpl	$ 0, %r10d
	je		10f
	cmpl	$ 1, %r10d
	je		11f
	cmpl	$ 2, %r10d
	je		12f
	cmpl	$ 3, %r10d
	je		13f

	jmp 3f


	// offset==0
10:
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd		.LC04(%rip), %ymm14
#elif defined(OS_MAC)
	vmovapd		LC04(%rip), %ymm14
#endif

	// select m0
	cmpl	$ 0, %r13d
	jle		20f
	cmpl	$ 1, %r13d
	je		21f
	cmpl	$ 2, %r13d
	je		22f
	cmpl	$ 3, %r13d
	je		23f

	jmp 3f

20:
	vmaskmovpd	%ymm0, %ymm15,  0(%r11)
	cmpl		$ 2, %r15d
	jl			3f // end
	vblendpd	$ 0x1, %ymm14, %ymm15, %ymm15
	vmaskmovpd	%ymm1, %ymm15, 32(%r11)
	cmpl		$ 3, %r15d
	jl			3f // end
	vblendpd	$ 0x2, %ymm14, %ymm15, %ymm15
	vmaskmovpd	%ymm2, %ymm15, 64(%r11)
	cmpl		$ 4, %r15d
	jl			3f // end
	vblendpd	$ 0x4, %ymm14, %ymm15, %ymm15
	vmaskmovpd	%ymm3, %ymm15, 96(%r11)

	jmp 3f

21:
	vblendpd	$ 0x1, %ymm14, %ymm15, %ymm15
	vmaskmovpd	%ymm0, %ymm15,  0(%r11)
	cmpl		$ 2, %r15d
	jl			3f // end
	vblendpd	$ 0x2, %ymm14, %ymm15, %ymm15
	vmaskmovpd	%ymm1, %ymm15, 32(%r11)
	cmpl		$ 3, %r15d
	jl			3f // end
	vblendpd	$ 0x4, %ymm14, %ymm15, %ymm15
	vmaskmovpd	%ymm2, %ymm15, 64(%r11)

	jmp 3f

22:
	vblendpd	$ 0x3, %ymm14, %ymm15, %ymm15
	vmaskmovpd	%ymm0, %ymm15,  0(%r11)
	cmpl		$ 2, %r15d
	jl			3f // end
	vblendpd	$ 0x4, %ymm14, %ymm15, %ymm15
	vmaskmovpd	%ymm1, %ymm15, 32(%r11)

	jmp 3f

23:
	vblendpd	$ 0x7, %ymm14, %ymm15, %ymm15
	vmaskmovpd	%ymm0, %ymm15,  0(%r11)

	jmp 3f


	// offset==1
11:
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm12
	vshufpd		$ 0x5, %ymm0, %ymm12, %ymm0

	vperm2f128	$ 0x01, %ymm1, %ymm1, %ymm12
	vshufpd		$ 0x5, %ymm1, %ymm12, %ymm1

	vperm2f128	$ 0x01, %ymm2, %ymm2, %ymm12
	vshufpd		$ 0x5, %ymm2, %ymm12, %ymm2

	vperm2f128	$ 0x01, %ymm3, %ymm3, %ymm12
	vshufpd		$ 0x5, %ymm3, %ymm12, %ymm3

	vperm2f128	$ 0x01, %ymm15, %ymm15, %ymm12
	vshufpd		$ 0x5, %ymm15, %ymm12, %ymm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC08(%rip), %ymm12
	vmovupd		.LC05(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC08(%rip), %ymm12
	vmovupd		LC05(%rip), %ymm13
#endif
	vandpd		%ymm12, %ymm15, %ymm12
	vandpd		%ymm13, %ymm15, %ymm13

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd		.LC04(%rip), %ymm14
#elif defined(OS_MAC)
	vmovapd		LC04(%rip), %ymm14
#endif

	// select m0
	cmpl	$ 0, %r13d
	jle		20f
	cmpl	$ 1, %r13d
	je		21f
	cmpl	$ 2, %r13d
	je		22f
	cmpl	$ 3, %r13d
	je		23f

	jmp 3f

20:
	vblendpd	$ 0x1, %ymm14, %ymm12, %ymm12
	vmaskmovpd	%ymm0, %ymm12, 0(%r11)
	vmaskmovpd	%ymm0, %ymm13, 0(%r11, %r12)
	cmpl		$ 2, %r15d
	jl			3f // end
	vblendpd	$ 0x2, %ymm14, %ymm12, %ymm12
	vmaskmovpd	%ymm1, %ymm12, 32(%r11)
	vmaskmovpd	%ymm1, %ymm13, 32(%r11, %r12)
	cmpl		$ 3, %r15d
	jl			3f // end
	vblendpd	$ 0x4, %ymm14, %ymm12, %ymm12
	vmaskmovpd	%ymm2, %ymm12, 64(%r11)
	vmaskmovpd	%ymm2, %ymm13, 64(%r11, %r12)
	cmpl		$ 4, %r15d
	jl			3f // end
	vmaskmovpd	%ymm3, %ymm13, 96(%r11, %r12)

	jmp 3f

21:
	vblendpd	$ 0x3, %ymm14, %ymm12, %ymm12
	vmaskmovpd	%ymm0, %ymm12, 0(%r11)
	vmaskmovpd	%ymm0, %ymm13, 0(%r11, %r12)
	cmpl		$ 2, %r15d
	jl			3f // end
	vblendpd	$ 0x4, %ymm14, %ymm12, %ymm12
	vmaskmovpd	%ymm1, %ymm12, 32(%r11)
	vmaskmovpd	%ymm1, %ymm13, 32(%r11, %r12)
	cmpl		$ 3, %r15d
	jl			3f // end
	vmaskmovpd	%ymm2, %ymm13, 64(%r11, %r12)

	jmp 3f

22:
	vblendpd	$ 0x7, %ymm14, %ymm12, %ymm12
	vmaskmovpd	%ymm0, %ymm12, 0(%r11)
	vmaskmovpd	%ymm0, %ymm13, 0(%r11, %r12)
	cmpl		$ 2, %r15d
	jl			3f // end
	vmaskmovpd	%ymm1, %ymm13, 32(%r11, %r12)

	jmp 3f

23:
	vmaskmovpd	%ymm0, %ymm13, 0(%r11, %r12)

	jmp 3f


	// offset==2
12:
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm0

	vperm2f128	$ 0x01, %ymm1, %ymm1, %ymm1

	vperm2f128	$ 0x01, %ymm2, %ymm2, %ymm2

	vperm2f128	$ 0x01, %ymm3, %ymm3, %ymm3

	vperm2f128	$ 0x01, %ymm15, %ymm15, %ymm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC09(%rip), %ymm12
	vmovupd		.LC06(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC09(%rip), %ymm12
	vmovupd		LC06(%rip), %ymm13
#endif
	vandpd		%ymm12, %ymm15, %ymm12
	vandpd		%ymm13, %ymm15, %ymm13

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd		.LC04(%rip), %ymm14
#elif defined(OS_MAC)
	vmovapd		LC04(%rip), %ymm14
#endif

	// select m0
	cmpl	$ 0, %r13d
	jle		20f
	cmpl	$ 1, %r13d
	je		21f
	cmpl	$ 2, %r13d
	je		22f
	cmpl	$ 3, %r13d
	je		23f

	jmp 3f

20:
	vblendpd	$ 0x3, %ymm14, %ymm12, %ymm12
	vmaskmovpd	%ymm0, %ymm12, 0(%r11)
	vmaskmovpd	%ymm0, %ymm13, 0(%r11, %r12)
	cmpl		$ 2, %r15d
	jl			3f // end
	vblendpd	$ 0x4, %ymm14, %ymm12, %ymm12
	vmaskmovpd	%ymm1, %ymm12, 32(%r11)
	vmaskmovpd	%ymm1, %ymm13, 32(%r11, %r12)
	cmpl		$ 3, %r15d
	jl			3f // end
	vmaskmovpd	%ymm2, %ymm13, 64(%r11, %r12)
	cmpl		$ 4, %r15d
	jl			3f // end
	vblendpd	$ 0x1, %ymm14, %ymm13, %ymm13
	vmaskmovpd	%ymm3, %ymm13, 96(%r11, %r12)

	jmp		3f

21:
	vblendpd	$ 0x7, %ymm14, %ymm12, %ymm12
	vmaskmovpd	%ymm0, %ymm12, 0(%r11)
	vmaskmovpd	%ymm0, %ymm13, 0(%r11, %r12)
	cmpl		$ 2, %r15d
	jl			3f // end
	vmaskmovpd	%ymm1, %ymm13, 32(%r11, %r12)
	cmpl		$ 3, %r15d
	jl			3f // end
	vblendpd	$ 0x1, %ymm14, %ymm13, %ymm13
	vmaskmovpd	%ymm2, %ymm13, 64(%r11, %r12)

	jmp		3f

22:
	vmaskmovpd	%ymm0, %ymm13, 0(%r11, %r12)
	cmpl		$ 2, %r15d
	jl			3f // end
	vblendpd	$ 0x1, %ymm14, %ymm13, %ymm13
	vmaskmovpd	%ymm1, %ymm13, 32(%r11, %r12)

	jmp		3f

23:
	vblendpd	$ 0x1, %ymm14, %ymm13, %ymm13
	vmaskmovpd	%ymm0, %ymm13, 0(%r11, %r12)

	jmp		3f


	// offset==3
13:
	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm0, %ymm0

	vperm2f128	$ 0x01, %ymm1, %ymm1, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm1, %ymm1

	vperm2f128	$ 0x01, %ymm2, %ymm2, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm2, %ymm2

	vperm2f128	$ 0x01, %ymm3, %ymm3, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm3, %ymm3

	vperm2f128	$ 0x01, %ymm15, %ymm15, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm15, %ymm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC10(%rip), %ymm12
	vmovupd		.LC07(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC10(%rip), %ymm12
	vmovupd		LC07(%rip), %ymm13
#endif
	vandpd		%ymm12, %ymm15, %ymm12
	vandpd		%ymm13, %ymm15, %ymm13

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd		.LC04(%rip), %ymm14
#elif defined(OS_MAC)
	vmovapd		LC04(%rip), %ymm14
#endif

	// select m0
	cmpl	$ 0, %r13d
	jle		20f
	cmpl	$ 1, %r13d
	je		21f
	cmpl	$ 2, %r13d
	je		22f
	cmpl	$ 3, %r13d
	je		23f

	jmp 3f

20:
	vblendpd	$ 0x7, %ymm14, %ymm12, %ymm12
	vmaskmovpd	%ymm0, %ymm12, 0(%r11)
	vmaskmovpd	%ymm0, %ymm13, 0(%r11, %r12)
	cmpl		$ 2, %r15d
	jl			3f // end
	vmaskmovpd	%ymm1, %ymm13, 32(%r11, %r12)
	cmpl		$ 3, %r15d
	jl			3f // end
	vblendpd	$ 0x1, %ymm14, %ymm13, %ymm13
	vmaskmovpd	%ymm2, %ymm13, 64(%r11, %r12)
	cmpl		$ 4, %r15d
	jl			3f // end
	vblendpd	$ 0x2, %ymm14, %ymm13, %ymm13
	vmaskmovpd	%ymm3, %ymm13, 96(%r11, %r12)

	jmp 3f

21:
	vmaskmovpd	%ymm0, %ymm13, 0(%r11, %r12)
	cmpl		$ 2, %r15d
	jl			3f // end
	vblendpd	$ 0x1, %ymm14, %ymm13, %ymm13
	vmaskmovpd	%ymm1, %ymm13, 32(%r11, %r12)
	cmpl		$ 3, %r15d
	jl			3f // end
	vblendpd	$ 0x2, %ymm14, %ymm13, %ymm13
	vmaskmovpd	%ymm2, %ymm13, 64(%r11, %r12)

	jmp 3f

22:
	vblendpd	$ 0x1, %ymm14, %ymm13, %ymm13
	vmaskmovpd	%ymm0, %ymm13, 0(%r11, %r12)
	cmpl		$ 2, %r15d
	jl			3f // end
	vblendpd	$ 0x2, %ymm14, %ymm13, %ymm13
	vmaskmovpd	%ymm1, %ymm13, 32(%r11, %r12)

	jmp 3f

23:
	vblendpd	$ 0x3, %ymm14, %ymm13, %ymm13
	vmaskmovpd	%ymm0, %ymm13, 0(%r11, %r12)

	jmp 3f

3:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_4x4_gen_lib4)
#endif





// common inner routine with file scope
//
// store u generalized
//
// input arguments:
// r10  <- offset
// r11  <- D
// r12  <- 4*sdd*sizeof(double)
// r13  <- m0 // row index: start from (inc)
// r14  <- m1 // row index: up to (exc)
// r15  <- n0 // col index: start from (inc)
// rax  <- n1 // col index: up to (exc)
// rbx  <- dirty
// ymm0 <- [d00 d11 d22 d33]
// ymm1 <- [d01 d10 d23 d32]
// ymm2 <- [d03 d12 d21 d30]
// ymm3 <- [d02 d13 d20 d31]
//
// output arguments:
// r10  <- offset
// r11  <- D
// r12  <- 4*sdd*sizeof(double)
// r13  <- m0 // row index: start from (inc)
// r14  <- m1 // row index: up to (exc)
// r15  <- n1-n0
// rax  <- n1-n0
// rbx  <- dirty
// ymm0 <- [d00 d10 d20 d30]
// ymm1 <- [d01 d11 d21 d31]
// ymm2 <- [d02 d12 d22 d32]
// ymm3 <- [d03 d13 d23 d33]

#if MACRO_LEVEL>=1
	.macro INNER_STORE_U_4X4_GEN_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_store_u_4x4_gen_lib4)
#endif
	
	// compute mask for rows
	vcvtsi2sd	%r13d, %xmm14, %xmm14
	vcvtsi2sd	%r14d, %xmm15, %xmm15
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


	cmpl	$ 0, %r10d
	jg		0f

	// offset==0
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd		.LC04(%rip), %ymm14
#elif defined(OS_MAC)
	vmovapd		LC04(%rip), %ymm14
#endif

	cmpl		$ 0, %r15d
	je			10f
	vblendpd	$ 0x1, %ymm15, %ymm14, %ymm14
	cmpl		$ 1, %r15d
	je			11f
	vblendpd	$ 0x2, %ymm15, %ymm14, %ymm14
	cmpl		$ 2, %r15d
	je			12f
	vblendpd	$ 0x4, %ymm15, %ymm14, %ymm14
	cmpl		$ 3, %r15d
	je			13f
	jmp			3f // end

10:
	vblendpd	$ 0x1, %ymm15, %ymm14, %ymm14
	vmaskmovpd	%ymm0, %ymm14,  0(%r11)
11:
	cmpl		$ 2, %eax
	jl			3f // end
	vblendpd	$ 0x2, %ymm15, %ymm14, %ymm14
	vmaskmovpd	%ymm1, %ymm14, 32(%r11)
12:
	cmpl		$ 3, %eax
	jl			3f // end
	vblendpd	$ 0x4, %ymm15, %ymm14, %ymm14
	vmaskmovpd	%ymm2, %ymm14, 64(%r11)
13:
	cmpl		$ 4, %eax
	jl			3f // end
	vblendpd	$ 0x8, %ymm15, %ymm14, %ymm14
	vmaskmovpd	%ymm3, %ymm14, 96(%r11)

	jmp		3f

0:
	
	cmpl	$ 1, %r10d
	jg		1f

	// offset==1

	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm12
	vshufpd		$ 0x5, %ymm0, %ymm12, %ymm0

	vperm2f128	$ 0x01, %ymm1, %ymm1, %ymm12
	vshufpd		$ 0x5, %ymm1, %ymm12, %ymm1

	vperm2f128	$ 0x01, %ymm2, %ymm2, %ymm12
	vshufpd		$ 0x5, %ymm2, %ymm12, %ymm2

	vperm2f128	$ 0x01, %ymm3, %ymm3, %ymm12
	vshufpd		$ 0x5, %ymm3, %ymm12, %ymm3

	vperm2f128	$ 0x01, %ymm15, %ymm15, %ymm12
	vshufpd		$ 0x5, %ymm15, %ymm12, %ymm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC08(%rip), %ymm12
	vmovupd		.LC05(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC08(%rip), %ymm12
	vmovupd		LC05(%rip), %ymm13
#endif
	vandpd		%ymm12, %ymm15, %ymm12
	vandpd		%ymm13, %ymm15, %ymm13

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd		.LC04(%rip), %ymm14
#elif defined(OS_MAC)
	vmovapd		LC04(%rip), %ymm14
#endif
	vmovapd		%ymm14, %ymm15

	cmpl		$ 0, %r15d
	je			10f
	vblendpd	$ 0x2, %ymm12, %ymm14, %ymm14
	cmpl		$ 1, %r15d
	je			11f
	vblendpd	$ 0x4, %ymm12, %ymm14, %ymm14
	cmpl		$ 2, %r15d
	je			12f
	vblendpd	$ 0x8, %ymm12, %ymm14, %ymm14
	cmpl		$ 3, %r15d
	je			13f
	jmp			3f // end

10:
	vblendpd	$ 0x2, %ymm12, %ymm14, %ymm14
	vmaskmovpd	%ymm0, %ymm14, 0(%r11)
//	vmaskmovpd	%ymm0, %ymm15, 0(%r11, %r12)
11:
	cmpl		$ 2, %eax
	jl			3f // end
	vblendpd	$ 0x4, %ymm12, %ymm14, %ymm14
	vmaskmovpd	%ymm1, %ymm14, 32(%r11)
//	vmaskmovpd	%ymm1, %ymm15, 32(%r11, %r12)
12:
	cmpl		$ 3, %eax
	jl			3f // end
	vblendpd	$ 0x8, %ymm12, %ymm14, %ymm14
	vmaskmovpd	%ymm2, %ymm14, 64(%r11)
//	vmaskmovpd	%ymm2, %ymm15, 64(%r11, %r12)
13:
	cmpl		$ 4, %eax
	jl			3f // end
	vblendpd	$ 0x1, %ymm13, %ymm15, %ymm15
	vmaskmovpd	%ymm3, %ymm14, 96(%r11)
	vmaskmovpd	%ymm3, %ymm15, 96(%r11, %r12)

	jmp		3f

1:

	cmpl	$ 2, %r10d
	jg		2f

	// offset==2

	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm0

	vperm2f128	$ 0x01, %ymm1, %ymm1, %ymm1

	vperm2f128	$ 0x01, %ymm2, %ymm2, %ymm2

	vperm2f128	$ 0x01, %ymm3, %ymm3, %ymm3

	vperm2f128	$ 0x01, %ymm15, %ymm15, %ymm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC09(%rip), %ymm12
	vmovupd		.LC06(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC09(%rip), %ymm12
	vmovupd		LC06(%rip), %ymm13
#endif
	vandpd		%ymm12, %ymm15, %ymm12
	vandpd		%ymm13, %ymm15, %ymm13

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd		.LC04(%rip), %ymm14
#elif defined(OS_MAC)
	vmovapd		LC04(%rip), %ymm14
#endif
	vmovapd		%ymm14, %ymm15

	cmpl		$ 0, %r15d
	je			10f
	vblendpd	$ 0x4, %ymm12, %ymm14, %ymm14
	cmpl		$ 1, %r15d
	je			11f
	vblendpd	$ 0x8, %ymm12, %ymm14, %ymm14
	cmpl		$ 2, %r15d
	je			12f
	vblendpd	$ 0x1, %ymm13, %ymm15, %ymm15
	cmpl		$ 3, %r15d
	je			13f
	jmp			3f // end

10:
	vblendpd	$ 0x4, %ymm12, %ymm14, %ymm14
	vmaskmovpd	%ymm0, %ymm14, 0(%r11)
//	vmaskmovpd	%ymm0, %ymm15, 0(%r11, %r12)
11:
	cmpl		$ 2, %eax
	jl			3f // end
	vblendpd	$ 0x8, %ymm12, %ymm14, %ymm14
	vmaskmovpd	%ymm1, %ymm14, 32(%r11)
//	vmaskmovpd	%ymm1, %ymm15, 32(%r11, %r12)
12:
	cmpl		$ 3, %eax
	jl			3f // end
	vblendpd	$ 0x1, %ymm13, %ymm15, %ymm15
	vmaskmovpd	%ymm2, %ymm14, 64(%r11)
	vmaskmovpd	%ymm2, %ymm15, 64(%r11, %r12)
13:
	cmpl		$ 4, %eax
	jl			3f // end
	vblendpd	$ 0x2, %ymm13, %ymm15, %ymm15
	vmaskmovpd	%ymm3, %ymm14, 96(%r11)
	vmaskmovpd	%ymm3, %ymm15, 96(%r11, %r12)

	jmp		3f

2:

	// offset==3

	vperm2f128	$ 0x01, %ymm0, %ymm0, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm0, %ymm0

	vperm2f128	$ 0x01, %ymm1, %ymm1, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm1, %ymm1

	vperm2f128	$ 0x01, %ymm2, %ymm2, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm2, %ymm2

	vperm2f128	$ 0x01, %ymm3, %ymm3, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm3, %ymm3

	vperm2f128	$ 0x01, %ymm15, %ymm15, %ymm12
	vshufpd		$ 0x5, %ymm12, %ymm15, %ymm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC10(%rip), %ymm12
	vmovupd		.LC07(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC10(%rip), %ymm12
	vmovupd		LC07(%rip), %ymm13
#endif
	vandpd		%ymm12, %ymm15, %ymm12
	vandpd		%ymm13, %ymm15, %ymm13

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd		.LC04(%rip), %ymm14
#elif defined(OS_MAC)
	vmovapd		LC04(%rip), %ymm14
#endif
	vmovapd		%ymm14, %ymm15

	cmpl		$ 0, %r15d
	je			10f
	vblendpd	$ 0x8, %ymm12, %ymm14, %ymm14
	cmpl		$ 1, %r15d
	je			11f
	vblendpd	$ 0x1, %ymm13, %ymm15, %ymm15
	cmpl		$ 2, %r15d
	je			12f
	vblendpd	$ 0x2, %ymm13, %ymm15, %ymm15
	cmpl		$ 3, %r15d
	je			13f
	jmp			3f // end

10:
	vblendpd	$ 0x8, %ymm12, %ymm14, %ymm14
	vmaskmovpd	%ymm0, %ymm14, 0(%r11)
//	vmaskmovpd	%ymm0, %ymm15, 0(%r11, %r12)
11:
	cmpl		$ 2, %eax
	jl			3f // end
	vblendpd	$ 0x1, %ymm13, %ymm15, %ymm15
	vmaskmovpd	%ymm1, %ymm14, 32(%r11)
	vmaskmovpd	%ymm1, %ymm15, 32(%r11, %r12)
12:
	cmpl		$ 3, %eax
	jl			3f // end
	vblendpd	$ 0x2, %ymm13, %ymm15, %ymm15
	vmaskmovpd	%ymm2, %ymm14, 64(%r11)
	vmaskmovpd	%ymm2, %ymm15, 64(%r11, %r12)
13:
	cmpl		$ 4, %eax
	jl			3f // end
	vblendpd	$ 0x4, %ymm13, %ymm15, %ymm15
	vmaskmovpd	%ymm3, %ymm14, 96(%r11)
	vmaskmovpd	%ymm3, %ymm15, 96(%r11, %r12)

3:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_u_4x4_gen_lib4)
#endif





// common inner routine with file scope
//
// transpose
//
// input arguments:
// ymm0 <- [d00 d11 d22 d33]
// ymm1 <- [d01 d10 d23 d32]
// ymm2 <- [d03 d12 d21 d30]
// ymm3 <- [d02 d13 d20 d31]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_TRAN_4X4_LIB4
#else
	.p2align 4,,15
	FUN_START(inner_tran_4x4_lib4)
#endif

	vunpcklpd	%ymm1, %ymm0, %ymm12
	vunpckhpd	%ymm1, %ymm0, %ymm13
	vunpcklpd	%ymm3, %ymm2, %ymm14
	vunpckhpd	%ymm3, %ymm2, %ymm15

	vperm2f128	$ 0x20, %ymm14, %ymm12, %ymm0
	vperm2f128	$ 0x31, %ymm14, %ymm12, %ymm2
	vperm2f128	$ 0x20, %ymm15, %ymm13, %ymm1
	vperm2f128	$ 0x31, %ymm15, %ymm13, %ymm3

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_tran_4x4_lib4)
#endif





//                               1      2              3          4          5             6          7
// void kernel_dgemm_nt_4x4_lib4(int k, double *alpha, double *A, double *B, double *beta, double *C, double *D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_4x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
	CALL(inner_scale_ab_4x4_lib4)
#endif


	// store n

	movq	ARG7, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
	CALL(inner_store_4x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_4x4_lib4)





//                                  1      2              3          4          5             6          7          8       9
// void kernel_dgemm_nt_4x4_vs_lib4(int k, double *alpha, double *A, double *B, double *beta, double *C, double *D, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_4x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
	CALL(inner_scale_ab_4x4_lib4)
#endif


	// store n

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // km 
	movq	ARG9, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB4
#else
	CALL(inner_store_4x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_4x4_vs_lib4)





//                                   1      2              3          4          5             6            7          8        9            10         11       12      13      14      15
// void kernel_dgemm_nt_4x4_gen_lib4(int k, double *alpha, double *A, double *B, double *beta, int offsetC, double *C, int sdc, int offsetD, double *D, int sdd, int m0, int m1, int n0, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_4x4_gen_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12 // offsetC
	movq	ARG7, %r13 // C
	movq	ARG8, %r14 // sdc
	sall	$ 5, %r14d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_GEN_LIB4
#else
	CALL(inner_scale_ab_4x4_gen_lib4)
#endif


	// store n gen

	movq	ARG9, %r10 // offsetD
	movq	ARG10, %r11 // D
	movq	ARG11, %r12 // sdd
	sall	$ 5, %r12d // 4*sdb*sizeof(double)
	movq	ARG12, %r13 // m0
	movq	ARG13, %r14 // m1
	movq	ARG14, %r15 // n0
	movq	ARG15, %rax // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_GEN_LIB4
#else
	CALL(inner_store_4x4_gen_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_4x4_gen_lib4)





//                               1      2              3          4            5          6        7             8          9
// void kernel_dgemm_nn_4x4_lib4(int k, double *alpha, double *A, int offsetB, double *B, int sdb, double *beta, double *C, double *D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nn_4x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

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
	CALL(inner_kernel_dgemm_nn_4x4_lib4)
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
	CALL(inner_scale_ab_4x4_lib4)
#endif


	// store n

	movq	ARG9, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
	CALL(inner_store_4x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nn_4x4_lib4)





//                                  1      2              3          4            5          6        7             8          9          10      11
// void kernel_dgemm_nn_4x4_vs_lib4(int k, double *alpha, double *A, int offsetB, double *B, int sdb, double *beta, double *C, double *D, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nn_4x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

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
	CALL(inner_kernel_dgemm_nn_4x4_lib4)
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
	CALL(inner_scale_ab_4x4_lib4)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // m1
	movq	ARG11, %r12 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB4
#else
	CALL(inner_store_4x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nn_4x4_vs_lib4)





//                                   1      2              3          4         5          6        7             8         9          10       11        12         13       14      15      16      17
// void kernel_dgemm_nn_4x4_gen_lib4(int k, double *alpha, double *A, int offB, double *B, int sdb, double *beta, int offC, double *C, int sdc, int offD, double *D, int sdd, int m0, int m1, int n0, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nn_4x4_gen_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

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
	CALL(inner_kernel_dgemm_nn_4x4_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12 // offsetC
	movq	ARG9, %r13 // C
	movq	ARG10, %r14 // sdc
	sall	$ 5, %r14d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_GEN_LIB4
#else
	CALL(inner_scale_ab_4x4_gen_lib4)
#endif


	// store n gen

	movq	ARG11, %r10 // offsetD
	movq	ARG12, %r11 // D
	movq	ARG13, %r12 // sdd
	sall	$ 5, %r12d // 4*sdb*sizeof(double)
	movq	ARG14, %r13 // m0
	movq	ARG15, %r14 // m1
	movq	ARG16, %r15 // n0
	movq	ARG17, %rax // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_GEN_LIB4
#else
	CALL(inner_store_4x4_gen_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nn_4x4_gen_lib4)





//                               1      2              3          4            5          6        7             8          9
// void kernel_dgemm_tt_4x4_lib4(int k, double *alpha, int offsetA, double *A, int sda, double *B, double *beta, double *C, double *D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_tt_4x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG6, %r11  // B
	movq	ARG4, %r12  // A
	movq	ARG5, %r13 // sda
	sall	$ 5, %r13d // 4*sda*sizeof(double)
	movq	ARG3, %r14 // offsetA

#if MACRO_LEVEL>=1
	INNER_EDGE_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_edge_dgemm_nn_4x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_4x4_lib4)
#endif

#if MACRO_LEVEL>=1
	INNER_TRAN_4X4_LIB4
#else
	CALL(inner_tran_4x4_lib4)
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
	CALL(inner_scale_ab_4x4_lib4)
#endif


	// store n

	movq	ARG9, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
	CALL(inner_store_4x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_tt_4x4_lib4)





//                                  1      2              3          4            5          6        7             8          9          10      11
// void kernel_dgemm_tt_4x4_vs_lib4(int k, double *alpha, int offsetA, double *A, int sda, double *B, double *beta, double *C, double *D, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_tt_4x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG6, %r11  // B
	movq	ARG4, %r12  // A
	movq	ARG5, %r13 // sda
	sall	$ 5, %r13d // 4*sda*sizeof(double)
	movq	ARG3, %r14 // offsetA

#if MACRO_LEVEL>=1
	INNER_EDGE_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_edge_dgemm_nn_4x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_4x4_lib4)
#endif

#if MACRO_LEVEL>=1
	INNER_TRAN_4X4_LIB4
#else
	CALL(inner_tran_4x4_lib4)
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
	CALL(inner_scale_ab_4x4_lib4)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // m1
	movq	ARG11, %r12 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB4
#else
	CALL(inner_store_4x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_tt_4x4_vs_lib4)





//                                   1      2              3          4         5          6        7             8         9          10       11        12         13       14      15      16      17
// void kernel_dgemm_tt_4x4_gen_lib4(int k, double *alpha, int offsetA, double *A, int sda, double *B, double *beta, int offC, double *C, int sdc, int offD, double *D, int sdd, int m0, int m1, int n0, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_tt_4x4_gen_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG6, %r11  // B
	movq	ARG4, %r12  // A
	movq	ARG5, %r13 // sda
	sall	$ 5, %r13d // 4*sda*sizeof(double)
	movq	ARG3, %r14 // offsetA

#if MACRO_LEVEL>=1
	INNER_EDGE_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_edge_dgemm_nn_4x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_4x4_lib4)
#endif

#if MACRO_LEVEL>=1
	INNER_TRAN_4X4_LIB4
#else
	CALL(inner_tran_4x4_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12 // offsetC
	movq	ARG9, %r13 // C
	movq	ARG10, %r14 // sdc
	sall	$ 5, %r14d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_GEN_LIB4
#else
	CALL(inner_scale_ab_4x4_gen_lib4)
#endif


	// store n gen

	movq	ARG11, %r10 // offsetD
	movq	ARG12, %r11 // D
	movq	ARG13, %r12 // sdd
	sall	$ 5, %r12d // 4*sdb*sizeof(double)
	movq	ARG14, %r13 // m0
	movq	ARG15, %r14 // m1
	movq	ARG16, %r15 // n0
	movq	ARG17, %rax // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_GEN_LIB4
#else
	CALL(inner_store_4x4_gen_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_tt_4x4_gen_lib4)





//                                 1      2              3          4            5          6        7             8          9
// void kernel_dsyrk_nn_u_4x4_lib4(int k, double *alpha, double *A, int offsetB, double *B, int sdb, double *beta, double *C, double *D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nn_u_4x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


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
	CALL(inner_kernel_dgemm_nn_4x4_lib4)
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
	CALL(inner_scale_ab_4x4_lib4)
#endif


	// store n

	movq	ARG9, %r10 // D


#if MACRO_LEVEL>=1
	INNER_STORE_U_4X4_LIB4
#else
	CALL(inner_store_u_4x4_lib4)
#endif



	EPILOGUE

	ret

	FUN_END(kernel_dsyrk_nn_u_4x4_lib4)





//                                    1      2              3          4            5          6        7             8          9          10      11
// void kernel_dsyrk_nn_u_4x4_vs_lib4(int k, double *alpha, double *A, int offsetB, double *B, int sdb, double *beta, double *C, double *D, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nn_u_4x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


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
	CALL(inner_kernel_dgemm_nn_4x4_lib4)
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
	CALL(inner_scale_ab_4x4_lib4)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // m1
	movq	ARG11, %r12 // n1


#if MACRO_LEVEL>=1
	INNER_STORE_U_4X4_VS_LIB4
#else
	CALL(inner_store_u_4x4_vs_lib4)
#endif



	EPILOGUE

	ret

	FUN_END(kernel_dsyrk_nn_u_4x4_vs_lib4)





//                                 1      2              3          4          5             6          7
// void kernel_dsyrk_nt_l_4x4_lib4(int k, double *alpha, double *A, double *B, double *beta, double *C, double *D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nt_l_4x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
	CALL(inner_scale_ab_4x4_lib4)
#endif


	// store n

	movq	ARG7, %r10 // D


#if MACRO_LEVEL>=1
	INNER_STORE_L_4X4_LIB4
#else
	CALL(inner_store_l_4x4_lib4)
#endif



	EPILOGUE

	ret

	FUN_END(kernel_dsyrk_nt_l_4x4_lib4)





//                                    1      2              3          4          5             6          7          8       9
// void kernel_dsyrk_nt_l_4x4_vs_lib4(int k, double *alpha, double *A, double *B, double *beta, double *C, double *D, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nt_l_4x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
	CALL(inner_scale_ab_4x4_lib4)
#endif


	// store n

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // km 
	movq	ARG9, %r12 // kn 


#if MACRO_LEVEL>=1
	INNER_STORE_L_4X4_VS_LIB4
#else
	CALL(inner_store_l_4x4_vs_lib4)
#endif



	EPILOGUE

	ret

	FUN_END(kernel_dsyrk_nt_l_4x4_vs_lib4)





//                                     1      2              3          4          5             6            7          8        9            10         11       12      13      14      15
// void kernel_dsyrk_nt_l_4x4_gen_lib4(int k, double *alpha, double *A, double *B, double *beta, int offsetC, double *C, int sdc, int offsetD, double *D, int sdd, int m0, int m1, int n0, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nt_l_4x4_gen_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12 // offsetC
	movq	ARG7, %r13 // C
	movq	ARG8, %r14 // sdc
	sall	$ 5, %r14d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_GEN_LIB4
#else
	CALL(inner_scale_ab_4x4_gen_lib4)
#endif


	// store n gen

	movq	ARG9, %r10 // offsetD
	movq	ARG10, %r11 // D
	movq	ARG11, %r12 // sdd
	sall	$ 5, %r12d // 4*sdb*sizeof(double)
	movq	ARG12, %r13 // m0
	movq	ARG13, %r14 // m1
	movq	ARG14, %r15 // n0
	movq	ARG15, %rax // n1

#if MACRO_LEVEL>=1
	INNER_STORE_L_4X4_GEN_LIB4
#else
	CALL(inner_store_l_4x4_gen_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dsyrk_nt_l_4x4_gen_lib4)





//                                 1      2              3          4          5             6          7
// void kernel_dsyrk_nt_u_4x4_lib4(int k, double *alpha, double *A, double *B, double *beta, double *C, double *D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nt_u_4x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
	CALL(inner_scale_ab_4x4_lib4)
#endif


	// store n

	movq	ARG7, %r10 // D


#if MACRO_LEVEL>=1
	INNER_STORE_U_4X4_LIB4
#else
	CALL(inner_store_u_4x4_lib4)
#endif



	EPILOGUE

	ret

	FUN_END(kernel_dsyrk_nt_u_4x4_lib4)





//                                    1      2              3          4          5             6          7          8       9
// void kernel_dsyrk_nt_u_4x4_vs_lib4(int k, double *alpha, double *A, double *B, double *beta, double *C, double *D, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nt_u_4x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
	CALL(inner_scale_ab_4x4_lib4)
#endif


	// store n

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // km 
	movq	ARG9, %r12 // kn 


#if MACRO_LEVEL>=1
	INNER_STORE_U_4X4_VS_LIB4
#else
	CALL(inner_store_u_4x4_vs_lib4)
#endif



	EPILOGUE

	ret

	FUN_END(kernel_dsyrk_nt_u_4x4_vs_lib4)





//                                     1      2              3          4          5             6            7          8        9            10         11       12      13      14      15
// void kernel_dsyrk_nt_u_4x4_gen_lib4(int k, double *alpha, double *A, double *B, double *beta, int offsetC, double *C, int sdc, int offsetD, double *D, int sdd, int m0, int m1, int n0, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nt_u_4x4_gen_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12 // offsetC
	movq	ARG7, %r13 // C
	movq	ARG8, %r14 // sdc
	sall	$ 5, %r14d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_GEN_LIB4
#else
	CALL(inner_scale_ab_4x4_gen_lib4)
#endif


	// store n gen

	movq	ARG9, %r10 // offsetD
	movq	ARG10, %r11 // D
	movq	ARG11, %r12 // sdd
	sall	$ 5, %r12d // 4*sdb*sizeof(double)
	movq	ARG12, %r13 // m0
	movq	ARG13, %r14 // m1
	movq	ARG14, %r15 // n0
	movq	ARG15, %rax // n1

#if MACRO_LEVEL>=1
	INNER_STORE_U_4X4_GEN_LIB4
#else
	CALL(inner_store_u_4x4_gen_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dsyrk_nt_u_4x4_gen_lib4)





//                                  1      2               3         4            5          6        7
// void kernel_dtrmm_nn_rl_4x4_lib4(int k, double *alpha, double *A, int offsetB, double *B, int sdb, double *D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nn_rl_4x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// initial triangle

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG5, %r12 // B
	movq	ARG6, %r13 // sdb
	sall	$ 5, %r13d // 4*sdb*sizeof(double)
	movq	ARG4, %r14 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NN_RL_4X4_LIB4
#else
	CALL(inner_edge_dtrmm_nn_rl_4x4_lib4)
#endif

	// call inner dgemm kernel nt after initial triangle

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_4x4_lib4)
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha

#if MACRO_LEVEL>=1
	INNER_SCALE_A0_4X4_LIB4
#else
	CALL(inner_scale_a0_4x4_lib4)
#endif


	// store n

	movq	ARG7, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
	CALL(inner_store_4x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nn_rl_4x4_lib4)





//                                     1      2              3          4            5          6        7          8       9
// void kernel_dtrmm_nn_rl_4x4_vs_lib4(int k, double *alpha, double *A, int offsetB, double *B, int sdb, double *D, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nn_rl_4x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// initial triangle

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG5, %r12 // B
	movq	ARG6, %r13 // sdb
	sall	$ 5, %r13d // 4*sdb*sizeof(double)
	movq	ARG4, %r14 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NN_RL_4X4_VS_LIB4
#else
	CALL(inner_edge_dtrmm_nn_rl_4x4_vs_lib4)
#endif

	// call inner dgemm kernel nt after initial triangle

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_4x4_lib4)
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha

#if MACRO_LEVEL>=1
	INNER_SCALE_A0_4X4_LIB4
#else
	CALL(inner_scale_a0_4x4_lib4)
#endif


	// store n

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // m1
	movq	ARG9, %r12 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB4
#else
	CALL(inner_store_4x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nn_rl_4x4_vs_lib4)





//                                      1      2              3          4            5          6        7            8          9       10       11      12      13
// void kernel_dtrmm_nn_rl_4x4_gen_lib4(int k, double *alpha, double *A, int offsetB, double *B, int sdb, int offsetD, double *D, int sdd, int m0, int m1, int n0, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nn_rl_4x4_gen_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// initial triangle

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG5, %r12 // B
	movq	ARG6, %r13 // sdb
	sall	$ 5, %r13d // 4*sdb*sizeof(double)
	movq	ARG4, %r14 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NN_RL_4X4_VS_LIB4
#else
	CALL(inner_edge_dtrmm_nn_rl_4x4_vs_lib4)
#endif

	// call inner dgemm kernel nt after initial triangle

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_4x4_lib4)
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha

#if MACRO_LEVEL>=1
	INNER_SCALE_A0_4X4_LIB4
#else
	CALL(inner_scale_a0_4x4_lib4)
#endif


	// store n

	movq	ARG7, %r10 // offsetD
	movq	ARG8, %r11 // D
	movq	ARG9, %r12 // sdd
	sall	$ 5, %r12d // 4*sdb*sizeof(double)
	movq	ARG10, %r13 // m0
	movq	ARG11, %r14 // m1
	movq	ARG12, %r15 // n0
	movq	ARG13, %rax // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_GEN_LIB4
#else
	CALL(inner_store_4x4_gen_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nn_rl_4x4_gen_lib4)





//                                  1      2              3          4          5
// void kernel_dtrmm_nt_ru_4x4_lib4(int k, double *alpha, double *A, double *B, double *D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_ru_4x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt after initial triangle

	movq	ARG1, %r10 // k
//	subl	$ 4, %r10d //k-4
	movq	ARG3, %r11 // A
//	addq	$ 128, %r11 // A+4*bs
	movq	ARG4, %r12 // B
//	addq	$ 128, %r12 // B+4*bs

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RU_4X4_LIB4
#else
	CALL(inner_edge_dtrmm_nt_ru_4x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blend

#if MACRO_LEVEL>=1
//	INNER_BLEND_4X4_LIB4
#else
//	CALL(inner_blend_4x4_lib4)
#endif


	// initial triangle

//	movq	ARG1, %r10
//	movq	ARG3, %r12
//	movq	ARG4, %r13

#if MACRO_LEVEL>=1
//	INNER_EDGE_DTRMM_NT_RU_4X4_LIB4
#else
//	CALL(inner_edge_dtrmm_nt_ru_4x4_lib4)
#endif

	// call inner scale

	movq	ARG2, %r10 // alpha

#if MACRO_LEVEL>=1
	INNER_SCALE_A0_4X4_LIB4
#else
	CALL(inner_scale_a0_4x4_lib4)
#endif


	// store n

	movq	ARG5, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
	CALL(inner_store_4x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_ru_4x4_lib4)





//                                     1      2              3          4          5          6       7
// void kernel_dtrmm_nt_ru_4x4_vs_lib4(int k, double *alpha, double *A, double *B, double *D, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nt_ru_4x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt after initial triangle

	movq	ARG1, %r10 // k
//	subl	$ 4, %r10d // k-4
	movq	ARG3, %r11 // A
//	addq	$ 128, %r11 // A+4*bs
	movq	ARG4, %r12 // B
//	addq	$ 128, %r12 // B+4*bs

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NT_RU_4X4_VS_LIB4
#else
	CALL(inner_edge_dtrmm_nt_ru_4x4_vs_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blender nn

#if MACRO_LEVEL>=1
//	INNER_BLEND_4X4_LIB4
#else
//	CALL(inner_blend_4x4_lib4)
#endif


	// initial triangle

//	movq	ARG1, %r10 // k
//	movq	ARG3, %r11 // A
//	movq	ARG4, %r12 // B

#if MACRO_LEVEL>=1
//	INNER_EDGE_DTRMM_NT_RU_4X4_VS_LIB4
#else
//	CALL(inner_edge_dtrmm_nt_ru_4x4_vs_lib4)
#endif


	// call inner loader nn

	movq	ARG2, %r10 // alpha

#if MACRO_LEVEL>=1
	INNER_SCALE_A0_4X4_LIB4
#else
	CALL(inner_scale_a0_4x4_lib4)
#endif


	// store n

	movq	ARG5, %r10 // D
	movq	ARG6, %r11 // km 
	movq	ARG7, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB4
#else
	CALL(inner_store_4x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nt_ru_4x4_vs_lib4)





//                                  1      2          3          4          5          6
// void kernel_dpotrf_nt_l_4x4_lib4(int k, double *A, double *B, double *C, double *D, double *inv_diag_D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpotrf_nt_l_4x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG4, %r10 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_4X4_LIB4
#else
	CALL(inner_scale_m11_4x4_lib4)
#endif


	// factorization

	movq	ARG6, %r10  // inv_diag_D 
	movl	$ 4, %r11d

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_4X4_VS_LIB4
#else
	CALL(inner_edge_dpotrf_4x4_vs_lib4)
#endif


	// store

	movq	ARG5, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_L_4X4_LIB4
#else
	CALL(inner_store_l_4x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dpotrf_nt_l_4x4_lib4)





//                                     1      2          3          4          5          6                   7       8
// void kernel_dpotrf_nt_l_4x4_vs_lib4(int k, double *A, double *B, double *C, double *D, double *inv_diag_D, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpotrf_nt_l_4x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG4, %r10 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_4X4_LIB4
#else
	CALL(inner_scale_m11_4x4_lib4)
#endif


	// factorization

	movq	ARG6, %r10  // inv_diag_D 
	movq	ARG8, %r11 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_4X4_VS_LIB4
#else
	CALL(inner_edge_dpotrf_4x4_vs_lib4)
#endif


	// store

	movq	ARG5, %r10 // D
	movq	ARG7, %r11 // km 
	movq	ARG8, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_L_4X4_VS_LIB4
#else
	CALL(inner_store_l_4x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dpotrf_nt_l_4x4_vs_lib4)





//                                        1       2           3           4       5           6           7          8          9
// void kernel_dsyrk_dpotrf_nt_l_4x4_lib4(int kp, double *Ap, double *Bp, int km, double *Am, double *Bm, double *C, double *D, double *inv_diag_D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_dpotrf_nt_l_4x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG2, %r11  // Ap
	movq	ARG3, %r12  // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
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
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG7, %r10   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_4X4_LIB4
#else
	CALL(inner_scale_m11_4x4_lib4)
#endif


	// factorization

	movq	ARG9, %r10  // inv_diag_D 
	movl	$ 4, %r11d

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_4X4_VS_LIB4
#else
	CALL(inner_edge_dpotrf_4x4_vs_lib4)
#endif


	// store

	movq	ARG8, %r10  // D 

#if MACRO_LEVEL>=1
	INNER_STORE_L_4X4_LIB4
#else
	CALL(inner_store_l_4x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dsyrk_dpotrf_nt_l_4x4_lib4)





//                                           edi     rsi         rdx         ecx     r8          r9          rsp+8      rsp+16     rsp+24             rsp+32   rsp+40
// void kernel_dsyrk_dpotrf_nt_l_4x4_vs_lib4(int kp, double *Ap, double *Bp, int km, double *Am, double *Bm, double *C, double *D, double *inv_diag_D, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_dpotrf_nt_l_4x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG2, %r11  // Ap
	movq	ARG3, %r12  // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// change sign
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
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG7, %r10   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_4X4_LIB4
#else
	CALL(inner_scale_m11_4x4_lib4)
#endif


	// factorization

	movq	ARG9, %r10  // inv_diag_D 
	movq	ARG11, %r11 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_4X4_VS_LIB4
#else
	CALL(inner_edge_dpotrf_4x4_vs_lib4)
#endif


	// store

	movq	ARG8, %r10  // D 
	movq	ARG10, %r11 // km 
	movq	ARG11, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_L_4X4_VS_LIB4
#else
	CALL(inner_store_l_4x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dsyrk_dpotrf_nt_l_4x4_vs_lib4)





//                                      1      2          3          4             5          6          7          8
// void kernel_dtrsm_nt_rl_inv_4x4_lib4(int k, double *A, double *B, double *beta, double *C, double *D, double *E, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_inv_4x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG4, %r10 // C
	movq	ARG5, %r11 // beta

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_LIB4
#else
	CALL(inner_scale_m1b_4x4_lib4)
#endif


	// solve

	movq	ARG7, %r10  // E 
	movq	ARG8, %r11  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_4X4_LIB4
#else
	CALL(inner_edge_dtrsm_rlt_inv_4x4_lib4)
#endif


	// store

	movq	ARG6, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
	CALL(inner_store_4x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_4x4_lib4)





//                                            1       2           3           4       5           6           7          8          9          10
// void kernel_dgemm_dtrsm_nt_rl_inv_4x4_lib4(int kp, double *Ap, double *Bp, int km, double *Am, double *Bm, double *C, double *D, double *E, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_dtrsm_nt_rl_inv_4x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG2, %r11  // Ap
	movq	ARG3, %r12  // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// change sign
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
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG7, %r10   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_4X4_LIB4
#else
	CALL(inner_scale_m11_4x4_lib4)
#endif


	// solve

	movq	ARG9, %r10  // E 
	movq	ARG10, %r11  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_4X4_LIB4
#else
	CALL(inner_edge_dtrsm_rlt_inv_4x4_lib4)
#endif


	// store

	movq	ARG8, %r10   // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
	CALL(inner_store_4x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_dtrsm_nt_rl_inv_4x4_lib4)





//                                         1      2          3          4             5          6          7          8                   9       10
// void kernel_dtrsm_nt_rl_inv_4x4_vs_lib4(int k, double *A, double *B, double *beta, double *C, double *D, double *E, double *inv_diag_E, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_inv_4x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blender_loader nn // TODO scale gen

	movq	ARG4, %r10 // C
	movq	ARG5, %r11 // beta

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_LIB4
#else
	CALL(inner_scale_m1b_4x4_lib4)
#endif


	// solve

	movq	ARG7, %r10  // E 
	movq	ARG8, %r11  // inv_diag_E 
	movq	ARG10, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_4X4_VS_LIB4
#else
	CALL(inner_edge_dtrsm_rlt_inv_4x4_vs_lib4)
#endif


	// store

	movq	ARG6, %r10 // D
	movq	ARG9, %r11 // km 
	movq	ARG10, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB4
#else
	CALL(inner_store_4x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_4x4_vs_lib4)





//                                               edi     rsi         rdx         ecx     r8          r9          rsp+8    rsp+16     rsp+24     rsp+32                rsp+40 rsp+48
// void kernel_dgemm_dtrsm_nt_rl_inv_4x4_vs_lib4(int kp, double *Ap, double *Bp, int km, double *Am, double *Bm, double *C, double *D, double *E, double *inv_diag_E, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_dtrsm_nt_rl_inv_4x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG2, %r11  // Ap
	movq	ARG3, %r12  // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// change sign
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
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG7, %r10  // C 

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_4X4_LIB4
#else
	CALL(inner_scale_m11_4x4_lib4)
#endif


	// solve

	movq	ARG9, %r10  // E 
	movq	ARG10, %r11  // inv_diag_E 
	movq	ARG12, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_4X4_VS_LIB4
#else
	CALL(inner_edge_dtrsm_rlt_inv_4x4_vs_lib4)
#endif


	// store

	movq	ARG8, %r10 // D 
	movq	ARG11, %r11 // km 
	movq	ARG12, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB4
#else
	CALL(inner_store_4x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_dtrsm_nt_rl_inv_4x4_vs_lib4)





//                                      1      2          3          4             5          6          7
// void kernel_dtrsm_nt_rl_one_4x4_lib4(int k, double *A, double *B, double *beta, double *C, double *D, double *E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_one_4x4_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG4, %r10 // beta
	movq	ARG5, %r11 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_LIB4
#else
	CALL(inner_scale_m1b_4x4_lib4)
#endif


	// solve

	movq	ARG7, %r10  // E

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_ONE_4X4_LIB4
#else
	CALL(inner_edge_dtrsm_rlt_one_4x4_lib4)
#endif


	// store

	movq	ARG6, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
	CALL(inner_store_4x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_rl_one_4x4_lib4)





//                                         1      2          3          4             5          6          7          8       9
// void kernel_dtrsm_nt_rl_one_4x4_vs_lib4(int k, double *A, double *B, double *beta, double *C, double *D, double *E, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_one_4x4_vs_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG4, %r10 // beta
	movq	ARG5, %r11 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_LIB4
#else
	CALL(inner_scale_m1b_4x4_lib4)
#endif


	// solve

	movq	ARG7, %r10  // E
	movq	ARG9, %r11 // kn

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_ONE_4X4_VS_LIB4
#else
	CALL(inner_edge_dtrsm_rlt_one_4x4_vs_lib4)
#endif


	// store

	movq	ARG6, %r10 // D
	movq	ARG8, %r11 // km
	movq	ARG9, %r12 // kn

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB4
#else
	CALL(inner_store_4x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_rl_one_4x4_vs_lib4)





//                                      1      2          3          4             5          6          7          8
// void kernel_dtrsm_nt_ru_inv_4x4_lib4(int k, double *A, double *B, double *beta, double *C, double *D, double *E, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_ru_inv_4x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG4, %r10 // beta
	movq	ARG5, %r11 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_LIB4
#else
	CALL(inner_scale_m1b_4x4_lib4)
#endif


	// solve

	movq	ARG7, %r10  // E 
	movq	ARG8, %r11 // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUT_INV_4X4_LIB4
#else
	CALL(inner_edge_dtrsm_rut_inv_4x4_lib4)
#endif


	// store

	movq	ARG6, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
	CALL(inner_store_4x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_ru_inv_4x4_lib4)





//                                         1      2          3          4             5          6          7          8                    9       10
// void kernel_dtrsm_nt_ru_inv_4x4_vs_lib4(int k, double *A, double *B, double *beta, double *C, double *D, double *E, double  *inv_diag_E, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_ru_inv_4x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG4, %r10 // beta
	movq	ARG5, %r11 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_LIB4
#else
	CALL(inner_scale_m1b_4x4_lib4)
#endif


	// solve

	movq	ARG7, %r10  // E 
	movq	ARG8, %r11 // inv_diag_E 
	movq	ARG10, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUT_INV_4X4_VS_LIB4
#else
	CALL(inner_edge_dtrsm_rut_inv_4x4_vs_lib4)
#endif


	// store

	movq	ARG6, %r10 // D
	movq	ARG9, %r11 // km 
	movq	ARG10, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB4
#else
	CALL(inner_store_4x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_ru_inv_4x4_vs_lib4)





//                                      1      2          3          4             5          6          7
// void kernel_dtrsm_nt_ru_one_4x4_lib4(int k, double *A, double *B, double *beta, double *C, double *D, double *E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_ru_one_4x4_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG4, %r10 // beta
	movq	ARG5, %r11 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_LIB4
#else
	CALL(inner_scale_m1b_4x4_lib4)
#endif


	// solve

	movq	ARG7, %r10  // E

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUT_ONE_4X4_LIB4
#else
	CALL(inner_edge_dtrsm_rut_one_4x4_lib4)
#endif


	// store

	movq	ARG6, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
	CALL(inner_store_4x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_ru_one_4x4_lib4)





//                                         1      2          3          4             5          6          7          8       9
// void kernel_dtrsm_nt_ru_one_4x4_vs_lib4(int k, double *A, double *B, double *beta, double *C, double *D, double *E, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_ru_one_4x4_vs_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG4, %r10 // C
	movq	ARG5, %r11 // beta

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_LIB4
#else
	CALL(inner_scale_m1b_4x4_lib4)
#endif


	// solve

	movq	ARG7, %r10  // E
	movq	ARG9, %r11 // kn

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUT_ONE_4X4_VS_LIB4
#else
	CALL(inner_edge_dtrsm_rut_one_4x4_vs_lib4)
#endif


	// store

	movq	ARG6, %r10 // D
	movq	ARG8, %r11 // km
	movq	ARG9, %r12 // kn

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB4
#else
	CALL(inner_store_4x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_ru_one_4x4_vs_lib4)





//                                      1      2          3          4        5             6          7          8          9
// void kernel_dtrsm_nn_ru_inv_4x4_lib4(int k, double *A, double *B, int sdb, double *beta, double *C, double *D, double *E, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_ru_inv_4x4_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12  // B
	movq	ARG4, %r13 // sdb
	sall	$ 5, %r13d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_4x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // beta
	movq	ARG6, %r11 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_LIB4
#else
	CALL(inner_scale_m1b_4x4_lib4)
#endif


	// solve

	movq	ARG8, %r10  // E
	movq	ARG9, %r11  // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUN_INV_4X4_LIB4
#else
	CALL(inner_edge_dtrsm_run_inv_4x4_lib4)
#endif


	// store

	movq	ARG7, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
	CALL(inner_store_4x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nn_ru_inv_4x4_lib4)





//                                         1      2          3          4        5             6          7          8          9                  10      11
// void kernel_dtrsm_nn_ru_inv_4x4_vs_lib4(int k, double *A, double *B, int sdb, double *beta double *C, double *D, double *E, double *inv_diag_E, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_ru_inv_4x4_vs_lib4)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12  // B
	movq	ARG4, %r13 // sdb
	sall	$ 5, %r13d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_4x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // beta
	movq	ARG6, %r11 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_LIB4
#else
	CALL(inner_scale_m1b_4x4_lib4)
#endif


	// solve

	movq	ARG8, %r10  // E
	movq	ARG9, %r11  // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RUN_INV_4X4_LIB4
#else
	CALL(inner_edge_dtrsm_run_inv_4x4_lib4)
#endif


	// store

	movq	ARG7, %r10 // D

	movq	ARG10, %r11  // km
	movq	ARG11, %r12  // kn

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB4
#else
	CALL(inner_store_4x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nn_ru_inv_4x4_vs_lib4)





//                                      1      2          3          4        5             6          7          8          9
// void kernel_dtrsm_nn_ll_inv_4x4_lib4(int k, double *A, double *B, int sdb, double *beta, double *C, double *D, double *E, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_ll_inv_4x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // B
	movq	ARG4, %r13 // sdb
	sall	$ 5, %r13d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_4x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // beta
	movq	ARG6, %r11 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_LIB4
#else
	CALL(inner_scale_m1b_4x4_lib4)
#endif


	// solve

	movq	ARG8, %r10  // E 
	movq	ARG9, %r11  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_LLN_INV_4X4_LIB4
#else
	CALL(inner_edge_dtrsm_lln_inv_4x4_lib4)
#endif


	// store

	movq	ARG7, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
	CALL(inner_store_4x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nn_ll_inv_4x4_lib4)





//                                         1      2          3          4        5             6          7          8          9                   10      11
// void kernel_dtrsm_nn_ll_inv_4x4_vs_lib4(int k, double *A, double *B, int sdb, double *beta, double *C, double *D, double *E, double *inv_diag_E, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_ll_inv_4x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // B
	movq	ARG4, %r13 // sdb
	sall	$ 5, %r13d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_4x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // C
	movq	ARG6, %r11 // beta

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_LIB4
#else
	CALL(inner_scale_m1b_4x4_lib4)
#endif


	// solve

	movq	ARG8, %r10  // E 
	movq	ARG9, %r11  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_LLN_INV_4X4_LIB4
#else
	CALL(inner_edge_dtrsm_lln_inv_4x4_lib4)
#endif


	// store

	movq	ARG7, %r10 // D
	movq	ARG10, %r11  // km 
	movq	ARG11, %r12  // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB4
#else
	CALL(inner_store_4x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nn_ll_inv_4x4_vs_lib4)





//                                      1      2          3          4        5             6          7          8
// void kernel_dtrsm_nn_ll_one_4x4_lib4(int k, double *A, double *B, int sdb, double *beta, double *C, double *D, double *E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_ll_one_4x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // B
	movq	ARG4, %r13 // sdb
	sall	$ 5, %r13d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_4x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // beta
	movq	ARG6, %r11 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_LIB4
#else
	CALL(inner_scale_m1b_4x4_lib4)
#endif


	// solve

	movq	ARG8, %r10  // E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_LLN_ONE_4X4_LIB4
#else
	CALL(inner_edge_dtrsm_lln_one_4x4_lib4)
#endif


	// store

	movq	ARG7, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
	CALL(inner_store_4x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nn_ll_one_4x4_lib4)





//                                         1      2          3          4        5             6          7          8          9       10
// void kernel_dtrsm_nn_ll_one_4x4_vs_lib4(int k, double *A, double *B, int sdb, double *beta, double *C, double *D, double *E, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_ll_one_4x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // B
	movq	ARG4, %r13 // sdb
	sall	$ 5, %r13d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_4x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // C
	movq	ARG6, %r11 // beta

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_LIB4
#else
	CALL(inner_scale_m1b_4x4_lib4)
#endif


	// solve

	movq	ARG8, %r10  // E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_LLN_ONE_4X4_LIB4
#else
	CALL(inner_edge_dtrsm_lln_one_4x4_lib4)
#endif


	// store

	movq	ARG7, %r10 // D
	movq	ARG9, %r11  // km 
	movq	ARG10, %r12  // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB4
#else
	CALL(inner_store_4x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nn_ll_one_4x4_vs_lib4)





//                                      edi    rsi        rdx        ecx      r8         r9         rsp+8      rsp+16
// void kernel_dtrsm_nn_lu_inv_4x4_lib4(int k, double *A, double *B, int sdb, double *C, double *D, double *E, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_lu_inv_4x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12  // B
	movq	ARG4, %r13 // sdb
	sall	$ 5, %r13d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_4x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_4X4_LIB4
#else
	CALL(inner_scale_m11_4x4_lib4)
#endif


	// solve

	movq	ARG7, %r10  // E 
	movq	ARG8, %r11  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_LUN_INV_4X4_LIB4
#else
	CALL(inner_edge_dtrsm_lun_inv_4x4_lib4)
#endif


	// store

	movq	ARG6, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
	CALL(inner_store_4x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nn_lu_inv_4x4_lib4)





//                                         edi    rsi        rdx        ecx      r8         r9         rsp+8      rsp+16              rsp+24  rsp+32
// void kernel_dtrsm_nn_lu_inv_4x4_vs_lib4(int k, double *A, double *B, int sdb, double *C, double *D, double *E, double *inv_diag_E, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_lu_inv_4x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12  // B
	movq	ARG4, %r13 // sdb
	sall	$ 5, %r13d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_4x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_4X4_LIB4
#else
	CALL(inner_scale_m11_4x4_lib4)
#endif


	// solve

	movq	ARG7, %r10  // E 
	movq	ARG8, %r11  // inv_diag_E 
	movq	ARG9, %r12  // km 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_LUN_INV_4X4_VS_LIB4
#else
	CALL(inner_edge_dtrsm_lun_inv_4x4_vs_lib4)
#endif


	// store

	movq	ARG6, %r10 // D

	movq	ARG9, %r11  // km 
	movq	ARG10, %r12  // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB4
#else
	CALL(inner_store_4x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nn_lu_inv_4x4_vs_lib4)





//                                      edi    rsi        rdx        ecx      r8         r9         rsp+8
// void kernel_dtrsm_nn_lu_one_4x4_lib4(int k, double *A, double *B, int sdb, double *C, double *D, double *E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_lu_one_4x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12  // B
	movq	ARG4, %r13 // sdb
	sall	$ 5, %r13d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_4x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_4X4_LIB4
#else
	CALL(inner_scale_m11_4x4_lib4)
#endif


	// solve

	movq	ARG7, %r10  // E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_LUN_ONE_4X4_LIB4
#else
	CALL(inner_edge_dtrsm_lun_one_4x4_lib4)
#endif


	// store

	movq	ARG6, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
	CALL(inner_store_4x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nn_lu_one_4x4_lib4)





//                                         edi    rsi        rdx        ecx      r8         r9         rsp+8      rsp+16  rsp+24
// void kernel_dtrsm_nn_lu_one_4x4_vs_lib4(int k, double *A, double *B, int sdb, double *C, double *D, double *E, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nn_lu_one_4x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12  // B
	movq	ARG4, %r13 // sdb
	sall	$ 5, %r13d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_4x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_4X4_LIB4
#else
	CALL(inner_scale_m11_4x4_lib4)
#endif


	// solve

	movq	ARG7, %r10  // E 
	movq	ARG8, %r11  // km 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_LUN_ONE_4X4_VS_LIB4
#else
	CALL(inner_edge_dtrsm_lun_one_4x4_vs_lib4)
#endif


	// store

	movq	ARG6, %r10 // D

	movq	ARG8, %r11  // km 
	movq	ARG9, %r12  // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB4
#else
	CALL(inner_store_4x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nn_lu_one_4x4_vs_lib4)





//                                1      2          3          4        5          6          7
// void kernel_dgetrf_nn_4x4_lib4(int k, double *A, double *B, int sdb, double *C, double *D, double *inv_diag_D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgetrf_nn_4x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12  // B
	movq	ARG4, %r13 // sdb
	sall	$ 5, %r13d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_4x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_4X4_LIB4
#else
	CALL(inner_scale_m11_4x4_lib4)
#endif


	// factorization

	movq	ARG7, %r10  // inv_diag_D 

#if MACRO_LEVEL>=1
	INNER_EDGE_DGETRF_4X4_LIB4
#else
	CALL(inner_edge_dgetrf_4x4_lib4)
#endif


	// store

	movq	ARG6, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
	CALL(inner_store_4x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgetrf_nn_4x4_lib4)





//                                   1      2          3          4        5          6          7                   8       9
// void kernel_dgetrf_nn_4x4_vs_lib4(int k, double *A, double *B, int sdb, double *C, double *D, double *inv_diag_D, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgetrf_nn_4x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12  // B
	movq	ARG4, %r13 // sdb
	sall	$ 5, %r13d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nn_4x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_4X4_LIB4
#else
	CALL(inner_scale_m11_4x4_lib4)
#endif


	// factorization

	movq	ARG7, %r10  // inv_diag_D 

#if MACRO_LEVEL>=1
	INNER_EDGE_DGETRF_4X4_LIB4
#else
	CALL(inner_edge_dgetrf_4x4_lib4)
#endif


	// store

	movq	ARG6, %r10 // D
	movq	ARG8, %r11  // km 
	movq	ARG9, %r12  // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB4
#else
	CALL(inner_store_4x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgetrf_nn_4x4_vs_lib4)





//                                1      2          3          4          5          6
// void kernel_dgetrf_nt_4x4_lib4(int k, double *A, double *B, double *C, double *D, double *inv_diag_D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgetrf_nt_4x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG4, %r10 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_4X4_LIB4
#else
	CALL(inner_scale_m11_4x4_lib4)
#endif


	// factorization

	movq	ARG6, %r10  // inv_diag_D 

#if MACRO_LEVEL>=1
	INNER_EDGE_DGETRF_4X4_LIB4
#else
	CALL(inner_edge_dgetrf_4x4_lib4)
#endif


	// store

	movq	ARG5, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
	CALL(inner_store_4x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgetrf_nt_4x4_lib4)





//                                   1      2          3          4          5          6                   7       8
// void kernel_dgetrf_nt_4x4_vs_lib4(int k, double *A, double *B, double *C, double *D, double *inv_diag_D, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgetrf_nt_4x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG4, %r10 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_4X4_LIB4
#else
	CALL(inner_scale_m11_4x4_lib4)
#endif


	// factorization

	movq	ARG6, %r10  // inv_diag_D 

#if MACRO_LEVEL>=1
	INNER_EDGE_DGETRF_4X4_LIB4
#else
	CALL(inner_edge_dgetrf_4x4_lib4)
#endif


	// store

	movq	ARG5, %r10 // D
	movq	ARG7, %r11  // km 
	movq	ARG8, %r12  // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB4
#else
	CALL(inner_store_4x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgetrf_nt_4x4_vs_lib4)





#if 0
//                                   rdi    rsi            rdx        rcx        r8            r9         rsp+8
// void kernel_dlauum_nt_4x4_lib4(int k, double *alpha, double *A, double *B, double *beta, double *C, double *D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dlauum_nt_4x4_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


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
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blender nn

#if MACRO_LEVEL>=1
//	INNER_BLEND_4X4_LIB4
#else
//	CALL(inner_blend_4x4_lib4)
#endif


	// initial triangle

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // B

#if MACRO_LEVEL>=1
	INNER_EDGE_DLAUUM_NT_4X4_LIB4
#else
	CALL(inner_edge_dlauum_nt_4x4_lib4)
#endif


	// call inner loader nn

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
	CALL(inner_scale_ab_4x4_lib4)
#endif


	// store n

	movq	ARG7, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_L_4X4_LIB4
#else
	CALL(inner_store_l_4x4_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dlauum_nt_4x4_vs_lib4)





//                                   rdi    rsi            rdx        rcx        r8            r9         rsp+8      rsp+16  rsp+24
// void kernel_dlauum_nt_4x4_vs_lib4(int k, double *alpha, double *A, double *B, double *beta, double *C, double *D, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dlauum_nt_4x4_vs_lib4)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


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
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blender nn

#if MACRO_LEVEL>=1
//	INNER_BLEND_4X4_LIB4
#else
//	CALL(inner_blend_4x4_lib4)
#endif


	// initial triangle

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // B

#if MACRO_LEVEL>=1
	INNER_EDGE_DLAUUM_NT_4X4_VS_LIB4
#else
	CALL(inner_edge_dlauum_nt_4x4_vs_lib4)
#endif


	// call inner loader nn

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
	CALL(inner_scale_ab_4x4_lib4)
#endif


	// store n

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // km 
	movq	ARG9, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB4
#else
	CALL(inner_store_4x4_vs_lib4)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dlauum_nt_4x4_vs_lib4)
#endif





//                             1         2           3           4
// void kernel_dlarfb4_rn_4_lib4(int kmax, double *pV, double *pT, double *pD);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dlarfb4_rn_4_lib4)
	
	PROLOGUE

	// zero accumulation registers

//	ZERO_ACC

	movq	ARG1, %r10 // k
	movq	ARG4, %r11 // D
	movq	ARG2, %r12 // V

	//
	vmovapd			0(%r11), %ymm0
	//
	vmovapd			32(%r11), %ymm1
	vbroadcastsd	32(%r12), %ymm13
	vfmadd231pd		%ymm13, %ymm1, %ymm0
	//
	vmovapd			64(%r11), %ymm2
	vbroadcastsd	64(%r12), %ymm13
	vfmadd231pd		%ymm13, %ymm2, %ymm0
	vbroadcastsd	72(%r12), %ymm13
	vfmadd231pd		%ymm13, %ymm2, %ymm1
	//
	vmovapd			96(%r11), %ymm3
	vbroadcastsd	96(%r12), %ymm13
	vfmadd231pd		%ymm13, %ymm3, %ymm0
	vbroadcastsd	104(%r12), %ymm13
	vfmadd231pd		%ymm13, %ymm3, %ymm1
	vbroadcastsd	112(%r12), %ymm13
	vfmadd231pd		%ymm13, %ymm3, %ymm2

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r12

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif

	movq	ARG3, %r10 // T

	//
	vbroadcastsd	120(%r10), %ymm12
	vmulpd			%ymm3, %ymm12, %ymm3
	//
	vbroadcastsd	112(%r10), %ymm12
	vfmadd231pd		%ymm2, %ymm12, %ymm3
	vbroadcastsd	80(%r10), %ymm12
	vmulpd			%ymm2, %ymm12, %ymm2
	//
	vbroadcastsd	104(%r10), %ymm12
	vfmadd231pd		%ymm1, %ymm12, %ymm3
	vbroadcastsd	72(%r10), %ymm12
	vfmadd231pd		%ymm1, %ymm12, %ymm2
	vbroadcastsd	40(%r10), %ymm12
	vmulpd			%ymm1, %ymm12, %ymm1
	//
	vbroadcastsd	96(%r10), %ymm12
	vfmadd231pd		%ymm0, %ymm12, %ymm3
	vbroadcastsd	64(%r10), %ymm12
	vfmadd231pd		%ymm0, %ymm12, %ymm2
	vbroadcastsd	32(%r10), %ymm12
	vfmadd231pd		%ymm0, %ymm12, %ymm1
	vbroadcastsd	0(%r10), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm0

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // V
	movq	ARG4, %r12 // D

	//
	vmovapd			0(%r12), %ymm12
	vaddpd			%ymm12, %ymm0, %ymm12
	vmovapd			%ymm12, 0(%r12)
	//
	vmovapd			32(%r12), %ymm12
	vbroadcastsd	32(%r11), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm12
	vaddpd			%ymm12, %ymm1, %ymm12
	vmovapd			%ymm12, 32(%r12)
	//
	vmovapd			64(%r12), %ymm12
	vbroadcastsd	64(%r11), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm12
	vbroadcastsd	72(%r11), %ymm13
	vfmadd231pd		%ymm1, %ymm13, %ymm12
	vaddpd			%ymm12, %ymm2, %ymm12
	vmovapd			%ymm12, 64(%r12)
	//
	vmovapd			96(%r12), %ymm12
	vbroadcastsd	96(%r11), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm12
	vbroadcastsd	104(%r11), %ymm13
	vfmadd231pd		%ymm1, %ymm13, %ymm12
	vbroadcastsd	112(%r11), %ymm13
	vfmadd231pd		%ymm2, %ymm13, %ymm12
	vaddpd			%ymm12, %ymm3, %ymm12
	vmovapd			%ymm12, 96(%r12)

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r12

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEBP_ADD_NN_4X4_LIB4
#else
	CALL(inner_kernel_dgebp_add_nn_4x4_lib4)
#endif

	EPILOGUE

	ret

	FUN_END(kernel_dlarfb4_rn_4_lib4)





//                                 1       2            3           4           5
// void kernel_dlarfb4_rn_4_la_lib4(int n1, double *pVA, double *pT, double *pD, double *pA);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dlarfb4_rn_4_la_lib4)
	
	PROLOGUE

	// zero accumulation registers

//	ZERO_ACC

	movq	ARG4, %r10 // D

	//
	vmovapd			0(%r10), %ymm0
	//
	vmovapd			32(%r10), %ymm1
	//
	vmovapd			64(%r10), %ymm2
	//
	vmovapd			96(%r10), %ymm3

	movq	ARG1, %r10 // k
	movq	ARG5, %r11 // A
	movq	ARG2, %r12 // VA

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif

	movq	ARG3, %r10 // T

	//
	vbroadcastsd	120(%r10), %ymm12
	vmulpd			%ymm3, %ymm12, %ymm3
	//
	vbroadcastsd	112(%r10), %ymm12
	vfmadd231pd		%ymm2, %ymm12, %ymm3
	vbroadcastsd	80(%r10), %ymm12
	vmulpd			%ymm2, %ymm12, %ymm2
	//
	vbroadcastsd	104(%r10), %ymm12
	vfmadd231pd		%ymm1, %ymm12, %ymm3
	vbroadcastsd	72(%r10), %ymm12
	vfmadd231pd		%ymm1, %ymm12, %ymm2
	vbroadcastsd	40(%r10), %ymm12
	vmulpd			%ymm1, %ymm12, %ymm1
	//
	vbroadcastsd	96(%r10), %ymm12
	vfmadd231pd		%ymm0, %ymm12, %ymm3
	vbroadcastsd	64(%r10), %ymm12
	vfmadd231pd		%ymm0, %ymm12, %ymm2
	vbroadcastsd	32(%r10), %ymm12
	vfmadd231pd		%ymm0, %ymm12, %ymm1
	vbroadcastsd	0(%r10), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm0

	movq	ARG4, %r10 // D

	//
	vmovapd			0(%r10), %ymm12
	vaddpd			%ymm12, %ymm0, %ymm12
	vmovapd			%ymm12, 0(%r10)
	//
	vmovapd			32(%r10), %ymm12
	vaddpd			%ymm12, %ymm1, %ymm12
	vmovapd			%ymm12, 32(%r10)
	//
	vmovapd			64(%r10), %ymm12
	vaddpd			%ymm12, %ymm2, %ymm12
	vmovapd			%ymm12, 64(%r10)
	//
	vmovapd			96(%r10), %ymm12
	vaddpd			%ymm12, %ymm3, %ymm12
	vmovapd			%ymm12, 96(%r10)

	movq	ARG1, %r10 // n1
	movq	ARG2, %r11 // VA
	movq	ARG5, %r12 // A

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEBP_ADD_NN_4X4_LIB4
#else
	CALL(inner_kernel_dgebp_add_nn_4x4_lib4)
#endif

	EPILOGUE

	ret

	FUN_END(kernel_dlarfb4_rn_4_la_lib4)





//                                  1       2       3            4            5           6           7           8
// void kernel_dlarfb4_rn_4_lla_lib4(int n0, int n1, double *pVL, double *pVA, double *pT, double *pD, double *pL, double *pA);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dlarfb4_rn_4_lla_lib4)


	PROLOGUE

	// zero accumulation registers

//	ZERO_ACC

	movq	ARG6, %r10 // D

	//
	vmovapd			0(%r10), %ymm0
	//
	vmovapd			32(%r10), %ymm1
	//
	vmovapd			64(%r10), %ymm2
	//
	vmovapd			96(%r10), %ymm3

	// L

	movq	ARG1, %r10 // n0
	movq	ARG7, %r11 // L
	movq	ARG3, %r12 // VL

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif

	// L final 4x4 lower triangular

	movq	ARG1, %r10 // n0
	sall	$ 5, %r10d // n0*ps*sizeof(double)
	movq	ARG7, %r11 // L
	addq	%r10, %r11 // L+n0*ps
	movq	ARG3, %r12 // VL
	addq	%r10, %r12 // VL+n0*ps

	// 4
	vmovapd			0(%r11), %ymm12
	vbroadcastsd	0(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	8(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	16(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	24(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	// 3
	vmovapd			32(%r11), %ymm12
	vbroadcastsd	40(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	48(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	56(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	// 2
	vmovapd			64(%r11), %ymm12
	vbroadcastsd	80(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	88(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	// 1
	vmovapd			96(%r11), %ymm12
	vbroadcastsd	120(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3

	// A

	movq	ARG2, %r10 // n1
	movq	ARG8, %r11 // A
	movq	ARG4, %r12 // VA

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif

	// T

	movq	ARG5, %r10 // T

	//
	vbroadcastsd	120(%r10), %ymm12
	vmulpd			%ymm3, %ymm12, %ymm3
	//
	vbroadcastsd	112(%r10), %ymm12
	vfmadd231pd		%ymm2, %ymm12, %ymm3
	vbroadcastsd	80(%r10), %ymm12
	vmulpd			%ymm2, %ymm12, %ymm2
	//
	vbroadcastsd	104(%r10), %ymm12
	vfmadd231pd		%ymm1, %ymm12, %ymm3
	vbroadcastsd	72(%r10), %ymm12
	vfmadd231pd		%ymm1, %ymm12, %ymm2
	vbroadcastsd	40(%r10), %ymm12
	vmulpd			%ymm1, %ymm12, %ymm1
	//
	vbroadcastsd	96(%r10), %ymm12
	vfmadd231pd		%ymm0, %ymm12, %ymm3
	vbroadcastsd	64(%r10), %ymm12
	vfmadd231pd		%ymm0, %ymm12, %ymm2
	vbroadcastsd	32(%r10), %ymm12
	vfmadd231pd		%ymm0, %ymm12, %ymm1
	vbroadcastsd	0(%r10), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm0

	// D

	movq	ARG6, %r10 // D

	//
	vmovapd			0(%r10), %ymm12
	vaddpd			%ymm12, %ymm0, %ymm12
	vmovapd			%ymm12, 0(%r10)
	//
	vmovapd			32(%r10), %ymm12
	vaddpd			%ymm12, %ymm1, %ymm12
	vmovapd			%ymm12, 32(%r10)
	//
	vmovapd			64(%r10), %ymm12
	vaddpd			%ymm12, %ymm2, %ymm12
	vmovapd			%ymm12, 64(%r10)
	//
	vmovapd			96(%r10), %ymm12
	vaddpd			%ymm12, %ymm3, %ymm12
	vmovapd			%ymm12, 96(%r10)

	// L

	movq	ARG1, %r10 // n0
	movq	ARG3, %r11 // VL
	movq	ARG7, %r12 // L

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEBP_ADD_NN_4X4_LIB4
#else
	CALL(inner_kernel_dgebp_add_nn_4x4_lib4)
#endif

	// L final 4x4 lower triangular

	movq	ARG1, %r10 // n0
	sall	$ 5, %r10d // n0*ps*sizeof(double)
	movq	ARG3, %r11 // VL
	addq	%r10, %r11 // VL+n0*ps
	movq	ARG7, %r12 // L
	addq	%r10, %r12 // L+n0*ps

	// 4
	vmovapd			0(%r12), %ymm12
	vbroadcastsd	0(%r11), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm12
	vbroadcastsd	8(%r11), %ymm13
	vfmadd231pd		%ymm1, %ymm13, %ymm12
	vbroadcastsd	16(%r11), %ymm13
	vfmadd231pd		%ymm2, %ymm13, %ymm12
	vbroadcastsd	24(%r11), %ymm13
	vfmadd231pd		%ymm3, %ymm13, %ymm12
	vmovapd			%ymm12, 0(%r12)
	// 3
	vmovapd			32(%r12), %ymm12
	vbroadcastsd	40(%r11), %ymm13
	vfmadd231pd		%ymm1, %ymm13, %ymm12
	vbroadcastsd	48(%r11), %ymm13
	vfmadd231pd		%ymm2, %ymm13, %ymm12
	vbroadcastsd	56(%r11), %ymm13
	vfmadd231pd		%ymm3, %ymm13, %ymm12
	vmovapd			%ymm12, 32(%r12)
	// 2
	vmovapd			64(%r12), %ymm12
	vbroadcastsd	80(%r11), %ymm13
	vfmadd231pd		%ymm2, %ymm13, %ymm12
	vbroadcastsd	88(%r11), %ymm13
	vfmadd231pd		%ymm3, %ymm13, %ymm12
	vmovapd			%ymm12, 64(%r12)
	// 1
	vmovapd			96(%r12), %ymm12
	vbroadcastsd	120(%r11), %ymm13
	vfmadd231pd		%ymm3, %ymm13, %ymm12
	vmovapd			%ymm12, 96(%r12)

	// A

	movq	ARG2, %r10 // n1
	movq	ARG4, %r11 // VA
	movq	ARG8, %r12 // A

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEBP_ADD_NN_4X4_LIB4
#else
	CALL(inner_kernel_dgebp_add_nn_4x4_lib4)
#endif

	EPILOGUE

	ret

	FUN_END(kernel_dlarfb4_rn_4_lla_lib4)





//                             1         2           3           4
// void kernel_dlarfb4_rt_4_lib4(int kmax, double *pV, double *pT, double *pD);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dlarfb4_rt_4_lib4)
	
	PROLOGUE

	// zero accumulation registers

//	ZERO_ACC

	movq	ARG1, %r10 // k
	movq	ARG4, %r11 // D
	movq	ARG2, %r12 // V

	//
	vmovapd			0(%r11), %ymm0
	//
	vmovapd			32(%r11), %ymm1
	vbroadcastsd	32(%r12), %ymm13
	vfmadd231pd		%ymm13, %ymm1, %ymm0
	//
	vmovapd			64(%r11), %ymm2
	vbroadcastsd	64(%r12), %ymm13
	vfmadd231pd		%ymm13, %ymm2, %ymm0
	vbroadcastsd	72(%r12), %ymm13
	vfmadd231pd		%ymm13, %ymm2, %ymm1
	//
	vmovapd			96(%r11), %ymm3
	vbroadcastsd	96(%r12), %ymm13
	vfmadd231pd		%ymm13, %ymm3, %ymm0
	vbroadcastsd	104(%r12), %ymm13
	vfmadd231pd		%ymm13, %ymm3, %ymm1
	vbroadcastsd	112(%r12), %ymm13
	vfmadd231pd		%ymm13, %ymm3, %ymm2

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r12

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif

	movq	ARG3, %r10 // T

	//
	vbroadcastsd	0(%r10), %ymm12
	vmulpd			%ymm0, %ymm12, %ymm0
	//
	vbroadcastsd	32(%r10), %ymm12
	vfmadd231pd		%ymm1, %ymm12, %ymm0
	vbroadcastsd	40(%r10), %ymm12
	vmulpd			%ymm1, %ymm12, %ymm1
	//
	vbroadcastsd	64(%r10), %ymm12
	vfmadd231pd		%ymm2, %ymm12, %ymm0
	vbroadcastsd	72(%r10), %ymm12
	vfmadd231pd		%ymm2, %ymm12, %ymm1
	vbroadcastsd	80(%r10), %ymm12
	vmulpd			%ymm2, %ymm12, %ymm2
	//
	vbroadcastsd	96(%r10), %ymm12
	vfmadd231pd		%ymm3, %ymm12, %ymm0
	vbroadcastsd	104(%r10), %ymm12
	vfmadd231pd		%ymm3, %ymm12, %ymm1
	vbroadcastsd	112(%r10), %ymm12
	vfmadd231pd		%ymm3, %ymm12, %ymm2
	vbroadcastsd	120(%r10), %ymm12
	vmulpd			%ymm3, %ymm12, %ymm3

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // V
	movq	ARG4, %r12 // D

	//
	vmovapd			0(%r12), %ymm12
	vaddpd			%ymm12, %ymm0, %ymm12
	vmovapd			%ymm12, 0(%r12)
	//
	vmovapd			32(%r12), %ymm12
	vbroadcastsd	32(%r11), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm12
	vaddpd			%ymm12, %ymm1, %ymm12
	vmovapd			%ymm12, 32(%r12)
	//
	vmovapd			64(%r12), %ymm12
	vbroadcastsd	64(%r11), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm12
	vbroadcastsd	72(%r11), %ymm13
	vfmadd231pd		%ymm1, %ymm13, %ymm12
	vaddpd			%ymm12, %ymm2, %ymm12
	vmovapd			%ymm12, 64(%r12)
	//
	vmovapd			96(%r12), %ymm12
	vbroadcastsd	96(%r11), %ymm13
	vfmadd231pd		%ymm0, %ymm13, %ymm12
	vbroadcastsd	104(%r11), %ymm13
	vfmadd231pd		%ymm1, %ymm13, %ymm12
	vbroadcastsd	112(%r11), %ymm13
	vfmadd231pd		%ymm2, %ymm13, %ymm12
	vaddpd			%ymm12, %ymm3, %ymm12
	vmovapd			%ymm12, 96(%r12)

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r12

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEBP_ADD_NN_4X4_LIB4
#else
	CALL(inner_kernel_dgebp_add_nn_4x4_lib4)
#endif

	EPILOGUE

	ret

	FUN_END(kernel_dlarfb4_rt_4_lib4)





//#if defined(BLAS_API)
#if ( defined(BLAS_API) | ( defined(LA_HIGH_PERFORMANCE) & defined(MF_COLMAJ) ) )

#include "kernel_dgemm_4x4_lib.S"
//#include "archive/kernel_dgemm_4x4_lib.S"

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
	.align 5
LC00: // { -1 -1 -1 1 }
#endif
	.quad	-1
	.quad	-1
	.quad	-1
	.quad	1

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC01: // { -1 -1 -1 -1 }
#elif defined(OS_MAC)
	.align 5
LC01: // { -1 -1 -1 -1 }
#endif
	.quad	-1
	.quad	-1
	.quad	-1
	.quad	-1

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC02: // { 3.5 2.5 1.5 0.5 }
#elif defined(OS_MAC)
	.align 5
LC02: // { 3.5 2.5 1.5 0.5 }
#endif
	.double 0.5
	.double 1.5
	.double 2.5
	.double 3.5

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC03: // { 7.5 6.5 5.5 4.5 }
#elif defined(OS_MAC)
	.align 5
LC03: // { 7.5 6.5 5.5 4.5 }
#endif
	.double 4.5
	.double 5.5
	.double 6.5
	.double 7.5

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC04: // { 1.0 1.0 1.0 1.0 }
#elif defined(OS_MAC)
	.align 5
LC04: // { 1.0 1.0 1.0 1.0 }
#endif
	.double 1.0
	.double 1.0
	.double 1.0
	.double 1.0

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC05: // { 1.0 1.0 1.0 -1.0 }
#elif defined(OS_MAC)
	.align 5
LC05: // { 1.0 1.0 1.0 -1.0 }
#endif
	.double -1.0
	.double 1.0
	.double 1.0
	.double 1.0

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC06: // { 1.0 1.0 -1.0 -1.0 }
#elif defined(OS_MAC)
	.align 5
LC06: // { 1.0 1.0 -1.0 -1.0 }
#endif
	.double -1.0
	.double -1.0
	.double 1.0
	.double 1.0

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC07: // { 1.0 -1.0 -1.0 -1.0 }
#elif defined(OS_MAC)
	.align 5
LC07: // { 1.0 -1.0 -1.0 -1.0 }
#endif
	.double -1.0
	.double -1.0
	.double -1.0
	.double 1.0

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC08: // { -1.0 -1.0 -1.0 1.0 }
#elif defined(OS_MAC)
	.align 5
LC08: // { -1.0 -1.0 -1.0 1.0 }
#endif
	.double 1.0
	.double -1.0
	.double -1.0
	.double -1.0

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC09: // { -1.0 -1.0 1.0 1.0 }
#elif defined(OS_MAC)
	.align 5
LC09: // { -1.0 -1.0 1.0 1.0 }
#endif
	.double 1.0
	.double 1.0
	.double -1.0
	.double -1.0

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC10: // { -1.0 1.0 1.0 1.0 }
#elif defined(OS_MAC)
	.align 5
LC10: // { -1.0 1.0 1.0 1.0 }
#endif
	.double 1.0
	.double 1.0
	.double 1.0
	.double -1.0

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

