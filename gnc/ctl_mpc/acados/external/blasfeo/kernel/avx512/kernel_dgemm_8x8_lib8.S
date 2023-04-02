/**************************************************************************************************
*                                                                                                 *
* This file is part of BLASFEO.                                                                   *
*                                                                                                 *
* BLASFEO -- BLAS For Embedded Optimization.                                                      *
* Copyright (C) 2021 by Gianluca Frison.                                                          *
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
#define ARG19 STACKSIZE + 104(%rsp)
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
	vpxord	%zmm0, %zmm0, %zmm0; \
	vmovapd	%zmm0, %zmm1; \
	vmovapd	%zmm0, %zmm2; \
	vmovapd	%zmm0, %zmm3; \
	vmovapd	%zmm0, %zmm4; \
	vmovapd	%zmm0, %zmm5; \
	vmovapd	%zmm0, %zmm6; \
	vmovapd	%zmm0, %zmm7;
#define NEG_ACC \
	vmovapd		.LC03(%rip), %zmm24; \
	vxorpd		%zmm24, %zmm0, %zmm0; \
	vxorpd		%zmm24, %zmm1, %zmm1; \
	vxorpd		%zmm24, %zmm2, %zmm2; \
	vxorpd		%zmm24, %zmm3, %zmm3; \
	vxorpd		%zmm24, %zmm4, %zmm4; \
	vxorpd		%zmm24, %zmm5, %zmm5; \
	vxorpd		%zmm24, %zmm6, %zmm6; \
	vxorpd		%zmm24, %zmm7, %zmm7;

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
	vpxord	%zmm0, %zmm0, %zmm0; \
	vmovapd	%zmm0, %zmm1; \
	vmovapd	%zmm0, %zmm2; \
	vmovapd	%zmm0, %zmm3; \
	vmovapd	%zmm0, %zmm4; \
	vmovapd	%zmm0, %zmm5; \
	vmovapd	%zmm0, %zmm6; \
	vmovapd	%zmm0, %zmm7;
#define NEG_ACC \
	vmovapd		LC03(%rip), %zmm24; \
	vxorpd		%zmm24, %zmm0, %zmm0; \
	vxorpd		%zmm24, %zmm1, %zmm1; \
	vxorpd		%zmm24, %zmm2, %zmm2; \
	vxorpd		%zmm24, %zmm3, %zmm3; \
	vxorpd		%zmm24, %zmm4, %zmm4; \
	vxorpd		%zmm24, %zmm5, %zmm5; \
	vxorpd		%zmm24, %zmm6, %zmm6; \
	vxorpd		%zmm24, %zmm7, %zmm7;

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
#define ARG19 STACKSIZE + 152(%rsp)
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
	vpxord	%zmm0, %zmm0, %zmm0; \
	vmovapd	%zmm0, %zmm1; \
	vmovapd	%zmm0, %zmm2; \
	vmovapd	%zmm0, %zmm3; \
	vmovapd	%zmm0, %zmm4; \
	vmovapd	%zmm0, %zmm5; \
	vmovapd	%zmm0, %zmm6; \
	vmovapd	%zmm0, %zmm7;
#define NEG_ACC \
	vmovapd		.LC03(%rip), %zmm24; \
	vxorpd		%zmm24, %zmm0, %zmm0; \
	vxorpd		%zmm24, %zmm1, %zmm1; \
	vxorpd		%zmm24, %zmm2, %zmm2; \
	vxorpd		%zmm24, %zmm3, %zmm3; \
	vxorpd		%zmm24, %zmm4, %zmm4; \
	vxorpd		%zmm24, %zmm5, %zmm5; \
	vxorpd		%zmm24, %zmm6, %zmm6; \
	vxorpd		%zmm24, %zmm7, %zmm7;

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
// r12   <- B
// zmm0  <- []
// ...
// zmm7  <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+8*k*sizeof(double)
// r12   <- B+8*k*sizeof(double)
// zmm0  <- []
// ...
// zmm7  <- []

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_8X8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_8x8_lib8)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	// prefetch
	prefetcht0	0+0*64(%r12) // software prefetch
	prefetcht0	0+1*64(%r12) // software prefetch
	prefetcht0	0+2*64(%r12) // software prefetch
	prefetcht0	0+3*64(%r12) // software prefetch

	// preload
	vmovapd 		0(%r11), %zmm25 // A

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	vbroadcastsd	0(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			64(%r11), %zmm26 // A
	vbroadcastsd	8(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	prefetcht0	256+0*64(%r12) // software prefetch
	vbroadcastsd	16(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	prefetcht0	256+1*64(%r12) // software prefetch
	vbroadcastsd	24(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	56(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	0+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			128(%r11), %zmm25 // A
	vbroadcastsd	8+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	16+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	24+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	32+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	40+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	48+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vbroadcastsd	56+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 0
	vbroadcastsd	0+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			192(%r11), %zmm26 // A
	vbroadcastsd	8+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	prefetcht0	256+2*64(%r12) // software prefetch
	vbroadcastsd	16+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	prefetcht0	256+3*64(%r12) // software prefetch
	vbroadcastsd	24+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	56+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	addq	$ 256, %r11

	// unroll 0
	vbroadcastsd	0+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			0(%r11), %zmm25 // A
	vbroadcastsd	8+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	16+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	24+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	32+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	40+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	48+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vbroadcastsd	56+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7
	addq	$ 256, %r12

	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			64(%r11), %zmm26 // A
	vbroadcastsd	8(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	prefetcht0	128(%r12) // software prefetch
	vbroadcastsd	16(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	prefetcht0	128+64(%r12) // software prefetch
	vbroadcastsd	24(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	56(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	0+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			128(%r11), %zmm25 // A
	vbroadcastsd	8+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	16+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	24+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	32+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	40+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	48+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vbroadcastsd	56+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 0
	vbroadcastsd	0+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			192(%r11), %zmm26 // A
	vbroadcastsd	8+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	prefetcht0	128(%r12) // software prefetch
	vbroadcastsd	16+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	prefetcht0	128+64(%r12) // software prefetch
	vbroadcastsd	24+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	56+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	addq	$ 256, %r11

	// unroll 0
	vbroadcastsd	0+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
//	vmovapd			0(%r11), %zmm25 // A
	vbroadcastsd	8+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	16+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	24+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	32+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	40+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	48+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vbroadcastsd	56+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7
	addq	$ 256, %r12

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovapd			0(%r11), %zmm25 // A
	vbroadcastsd	0(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	8(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	56(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	addq	$ 64, %r11
	addq	$ 64, %r12
	subl	$ 1, %r10d

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_8x8_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// k1    <- m-maks
// zmm0  <- []
// ...
// zmm7  <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+8*k*sizeof(double)
// r12   <- B+8*k*sizeof(double)
// k1    <- m-maks
// zmm0  <- []
// ...
// zmm7  <- []

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_VX8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_vx8_lib8)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	// prefetch

	// preload
	vmovapd 		0(%r11), %zmm25 {%k1}{z} // A

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	vbroadcastsd	0(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	8(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	56(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	0+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			128(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	16+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	24+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	32+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	40+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	48+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vbroadcastsd	56+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 0
	vbroadcastsd	0+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			192(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	8+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	56+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	addq	$ 256, %r11

	// unroll 0
	vbroadcastsd	0+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			0(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	16+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	24+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	32+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	40+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	48+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vbroadcastsd	56+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7
	addq	$ 256, %r12

	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	8(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	56(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	0+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			128(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	16+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	24+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	32+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	40+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	48+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vbroadcastsd	56+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 0
	vbroadcastsd	0+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			192(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	8+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	56+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	addq	$ 256, %r11

	// unroll 0
	vbroadcastsd	0+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
//	vmovapd			0(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	16+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	24+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	32+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	40+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	48+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vbroadcastsd	56+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7
	addq	$ 256, %r12

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovapd			0(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	0(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	8(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	56(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	addq	$ 64, %r11
	addq	$ 64, %r12
	subl	$ 1, %r10d

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_vx8_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// k1    <- m-maks
// zmm0  <- []
// ...
// zmm7  <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+8*k*sizeof(double)
// r12   <- B+8*k*sizeof(double)
// k1    <- m-maks
// zmm0  <- []
// ...
// zmm7  <- []

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_VX7_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_vx7_lib8)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	// prefetch

	// preload
	vmovapd 		0(%r11), %zmm25 {%k1}{z} // A

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	vbroadcastsd	0(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	8(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	56(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	0+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			128(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	16+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	24+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	32+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	40+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	48+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	56+64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 0
	vbroadcastsd	0+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			192(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	8+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	56+128(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
	addq	$ 256, %r11

	// unroll 0
	vbroadcastsd	0+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			0(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	16+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	24+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	32+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	40+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	48+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	56+192(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7
	addq	$ 256, %r12

	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	8(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	56(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	0+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			128(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	16+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	24+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	32+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	40+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	48+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	56+64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 0
	vbroadcastsd	0+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			192(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	8+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	56+128(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
	addq	$ 256, %r11

	// unroll 0
	vbroadcastsd	0+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
//	vmovapd			0(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	16+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	24+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	32+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	40+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	48+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	56+192(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7
	addq	$ 256, %r12

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovapd			0(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	0(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	8(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	56(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7

	addq	$ 64, %r11
	addq	$ 64, %r12
	subl	$ 1, %r10d

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_vx7_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// k1    <- m-maks
// zmm0  <- []
// ...
// zmm7  <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+8*k*sizeof(double)
// r12   <- B+8*k*sizeof(double)
// k1    <- m-maks
// zmm0  <- []
// ...
// zmm7  <- []

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_VX6_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_vx6_lib8)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	// prefetch

	// preload
	vmovapd 		0(%r11), %zmm25 {%k1}{z} // A

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	vbroadcastsd	0(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	8(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vbroadcastsd	48(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	56(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	0+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			128(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	16+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	24+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	32+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	40+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	48+64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	56+64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 0
	vbroadcastsd	0+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			192(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	8+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vbroadcastsd	48+128(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	56+128(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
	addq	$ 256, %r11

	// unroll 0
	vbroadcastsd	0+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			0(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	16+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	24+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	32+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	40+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	48+192(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	56+192(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7
	addq	$ 256, %r12

	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	8(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vbroadcastsd	48(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	56(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	0+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			128(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	16+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	24+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	32+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	40+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	48+64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	56+64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 0
	vbroadcastsd	0+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			192(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	8+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vbroadcastsd	48+128(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	56+128(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
	addq	$ 256, %r11

	// unroll 0
	vbroadcastsd	0+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
//	vmovapd			0(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	16+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	24+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	32+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	40+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	48+192(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	56+192(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7
	addq	$ 256, %r12

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovapd			0(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	0(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	8(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vbroadcastsd	48(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	56(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7

	addq	$ 64, %r11
	addq	$ 64, %r12
	subl	$ 1, %r10d

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_vx6_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// k1    <- m-maks
// zmm0  <- []
// ...
// zmm7  <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+8*k*sizeof(double)
// r12   <- B+8*k*sizeof(double)
// k1    <- m-maks
// zmm0  <- []
// ...
// zmm7  <- []

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_VX5_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_vx5_lib8)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	// prefetch

	// preload
	vmovapd 		0(%r11), %zmm25 {%k1}{z} // A

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	vbroadcastsd	0(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	8(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
//	vbroadcastsd	40(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vbroadcastsd	48(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	56(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	0+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			128(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	16+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	24+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	32+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
//	vbroadcastsd	40+64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	48+64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	56+64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 0
	vbroadcastsd	0+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			192(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	8+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
//	vbroadcastsd	40+128(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vbroadcastsd	48+128(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	56+128(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
	addq	$ 256, %r11

	// unroll 0
	vbroadcastsd	0+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			0(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	16+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	24+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	32+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
//	vbroadcastsd	40+192(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	48+192(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	56+192(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7
	addq	$ 256, %r12

	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	8(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
//	vbroadcastsd	40(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vbroadcastsd	48(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	56(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	0+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			128(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	16+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	24+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	32+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
//	vbroadcastsd	40+64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	48+64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	56+64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 0
	vbroadcastsd	0+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			192(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	8+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
//	vbroadcastsd	40+128(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vbroadcastsd	48+128(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	56+128(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
	addq	$ 256, %r11

	// unroll 0
	vbroadcastsd	0+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
//	vmovapd			0(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	16+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	24+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	32+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
//	vbroadcastsd	40+192(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	48+192(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	56+192(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7
	addq	$ 256, %r12

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovapd			0(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	0(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	8(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
//	vbroadcastsd	40(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vbroadcastsd	48(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	56(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7

	addq	$ 64, %r11
	addq	$ 64, %r12
	subl	$ 1, %r10d

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_vx5_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// k1    <- m-maks
// zmm0  <- []
// ...
// zmm7  <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+8*k*sizeof(double)
// r12   <- B+8*k*sizeof(double)
// k1    <- m-maks
// zmm0  <- []
// ...
// zmm7  <- []

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_VX4_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_vx4_lib8)
#endif

	cmpl	$ 0, %r10d
	jle		5f // return

	// prefetch

	// preload
	vmovapd 		0(%r11), %zmm25 {%k1}{z} // A

	vxorpd			%zmm4, %zmm4, %zmm4
	vmovapd			%zmm4, %zmm5
	vmovapd			%zmm4, %zmm6
	vmovapd			%zmm4, %zmm7

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	vbroadcastsd	0(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	8(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	0+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			128(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	16+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vbroadcastsd	24+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 0
	vbroadcastsd	0+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			192(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	8+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	addq	$ 256, %r11

	// unroll 0
	vbroadcastsd	0+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			0(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	16+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vbroadcastsd	24+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7
	addq	$ 256, %r12

	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	8(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	0+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			128(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	16+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vbroadcastsd	24+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 0
	vbroadcastsd	0+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			192(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	8+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	addq	$ 256, %r11

	// unroll 0
	vbroadcastsd	0+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
//	vmovapd			0(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	16+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vbroadcastsd	24+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7
	addq	$ 256, %r12

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovapd			0(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	0(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	8(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3

	addq	$ 64, %r11
	addq	$ 64, %r12
	subl	$ 1, %r10d

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

	vaddpd			%zmm4, %zmm0, %zmm0
	vaddpd			%zmm5, %zmm1, %zmm1
	vaddpd			%zmm6, %zmm2, %zmm2
	vaddpd			%zmm7, %zmm3, %zmm3

5: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_vx4_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// k1    <- m-maks
// zmm0  <- []
// ...
// zmm7  <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+8*k*sizeof(double)
// r12   <- B+8*k*sizeof(double)
// k1    <- m-maks
// zmm0  <- []
// ...
// zmm7  <- []

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_VX3_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_vx3_lib8)
#endif

	cmpl	$ 0, %r10d
	jle		5f // return

	// prefetch

	// preload
	vmovapd 		0(%r11), %zmm25 {%k1}{z} // A

	vxorpd			%zmm4, %zmm4, %zmm4
	vmovapd			%zmm4, %zmm5
	vmovapd			%zmm4, %zmm6
//	vmovapd			%zmm4, %zmm7

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	vbroadcastsd	0(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	8(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	24(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	0+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			128(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	16+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	24+64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 0
	vbroadcastsd	0+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			192(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	8+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	24+128(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
	addq	$ 256, %r11

	// unroll 0
	vbroadcastsd	0+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			0(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	16+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	24+192(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7
	addq	$ 256, %r12

	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	8(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	24(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	0+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			128(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	16+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	24+64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 0
	vbroadcastsd	0+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			192(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	8+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	24+128(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
	addq	$ 256, %r11

	// unroll 0
	vbroadcastsd	0+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
//	vmovapd			0(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	16+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	24+192(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7
	addq	$ 256, %r12

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovapd			0(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	0(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	8(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	24(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3

	addq	$ 64, %r11
	addq	$ 64, %r12
	subl	$ 1, %r10d

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

	vaddpd			%zmm4, %zmm0, %zmm0
	vaddpd			%zmm5, %zmm1, %zmm1
	vaddpd			%zmm6, %zmm2, %zmm2
//	vaddpd			%zmm7, %zmm3, %zmm3

5: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_vx3_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// k1    <- m-maks
// zmm0  <- []
// ...
// zmm7  <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+8*k*sizeof(double)
// r12   <- B+8*k*sizeof(double)
// k1    <- m-maks
// zmm0  <- []
// ...
// zmm7  <- []

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_VX2_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_vx2_lib8)
#endif

	cmpl	$ 0, %r10d
	jle		5f // return

	// prefetch

	// preload
	vmovapd 		0(%r11), %zmm25 {%k1}{z} // A

	vxorpd			%zmm4, %zmm4, %zmm4
	vmovapd			%zmm4, %zmm5
//	vmovapd			%zmm4, %zmm6
//	vmovapd			%zmm4, %zmm7

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	vbroadcastsd	0(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	8(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vbroadcastsd	16(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	24(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	0+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			128(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	16+64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	24+64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 0
	vbroadcastsd	0+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			192(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	8+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vbroadcastsd	16+128(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	24+128(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
	addq	$ 256, %r11

	// unroll 0
	vbroadcastsd	0+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			0(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	16+192(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	24+192(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7
	addq	$ 256, %r12

	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	8(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vbroadcastsd	16(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	24(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	0+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			128(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	16+64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	24+64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 0
	vbroadcastsd	0+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			192(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	8+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vbroadcastsd	16+128(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	24+128(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
	addq	$ 256, %r11

	// unroll 0
	vbroadcastsd	0+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
//	vmovapd			0(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	16+192(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	24+192(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7
	addq	$ 256, %r12

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovapd			0(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	0(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	8(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vbroadcastsd	16(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	24(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3

	addq	$ 64, %r11
	addq	$ 64, %r12
	subl	$ 1, %r10d

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

	vaddpd			%zmm4, %zmm0, %zmm0
	vaddpd			%zmm5, %zmm1, %zmm1
//	vaddpd			%zmm6, %zmm2, %zmm2
//	vaddpd			%zmm7, %zmm3, %zmm3

5: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_vx2_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// k1    <- m-maks
// zmm0  <- []
// ...
// zmm7  <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+8*k*sizeof(double)
// r12   <- B+8*k*sizeof(double)
// k1    <- m-maks
// zmm0  <- []
// ...
// zmm7  <- []

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_VX1_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_vx1_lib8)
#endif

	cmpl	$ 0, %r10d
	jle		5f // return

	// prefetch

	// preload
	vmovapd 		0(%r11), %zmm25 {%k1}{z} // A

	vxorpd			%zmm4, %zmm4, %zmm4
//	vmovapd			%zmm4, %zmm5
//	vmovapd			%zmm4, %zmm6
//	vmovapd			%zmm4, %zmm7

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	vbroadcastsd	0(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			64(%r11), %zmm26 {%k1}{z} // A
//	vbroadcastsd	8(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vbroadcastsd	16(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	24(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	0+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			128(%r11), %zmm25 {%k1}{z} // A
//	vbroadcastsd	8+64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	16+64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	24+64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 0
	vbroadcastsd	0+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			192(%r11), %zmm26 {%k1}{z} // A
//	vbroadcastsd	8+128(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vbroadcastsd	16+128(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	24+128(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
	addq	$ 256, %r11

	// unroll 0
	vbroadcastsd	0+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			0(%r11), %zmm25 {%k1}{z} // A
//	vbroadcastsd	8+192(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	16+192(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	24+192(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7
	addq	$ 256, %r12

	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			64(%r11), %zmm26 {%k1}{z} // A
//	vbroadcastsd	8(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vbroadcastsd	16(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	24(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
	subl	$ 4, %r10d

	// unroll 0
	vbroadcastsd	0+64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			128(%r11), %zmm25 {%k1}{z} // A
//	vbroadcastsd	8+64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	16+64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	24+64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 0
	vbroadcastsd	0+128(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			192(%r11), %zmm26 {%k1}{z} // A
//	vbroadcastsd	8+128(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vbroadcastsd	16+128(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	24+128(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
	addq	$ 256, %r11

	// unroll 0
	vbroadcastsd	0+192(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
//	vmovapd			0(%r11), %zmm25 {%k1}{z} // A
//	vbroadcastsd	8+192(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	16+192(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	24+192(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7
	addq	$ 256, %r12

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovapd			0(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	0(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
//	vbroadcastsd	8(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vbroadcastsd	16(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	24(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3

	addq	$ 64, %r11
	addq	$ 64, %r12
	subl	$ 1, %r10d

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

	vaddpd			%zmm4, %zmm0, %zmm0
//	vaddpd			%zmm5, %zmm1, %zmm1
//	vaddpd			%zmm6, %zmm2, %zmm2
//	vaddpd			%zmm7, %zmm3, %zmm3

5: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_vx1_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// r13   <- m1
// r14   <- n1
// zmm0  <- []
// ...
// zmm7  <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+8*k*sizeof(double)
// r12   <- B+8*k*sizeof(double)
// r13   <- m1
// r14   <- n1
// zmm0  <- []
// ...
// zmm7  <- []

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_8X8_VS_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_8x8_vs_lib8)
#endif

	// compute mask for rows
	vcvtsi2sd	%r13d, %xmm25, %xmm25
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC00(%rip), %zmm24
#elif defined(OS_MAC)
	vmovupd		LC00(%rip), %zmm24
#endif
	vbroadcastsd	%xmm25, %zmm25
	vsubpd		%zmm25, %zmm24, %zmm25
	vpmovq2m	%zmm25, %k1

	cmpl	$ 1, %r14d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_VX1_LIB8
#else
	CALL(inner_kernel_dgemm_nt_vx1_lib8)
#endif
	
	jmp		107f

100:

	cmpl	$ 2, %r14d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_VX2_LIB8
#else
	CALL(inner_kernel_dgemm_nt_vx2_lib8)
#endif
	
	jmp		107f

101:

	cmpl	$ 3, %r14d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_VX3_LIB8
#else
	CALL(inner_kernel_dgemm_nt_vx3_lib8)
#endif
	
	jmp		107f

102:

	cmpl	$ 4, %r14d
	jg		103f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_VX4_LIB8
#else
	CALL(inner_kernel_dgemm_nt_vx4_lib8)
#endif
	
	jmp		107f

103:

	cmpl	$ 5, %r14d
	jg		104f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_VX5_LIB8
#else
	CALL(inner_kernel_dgemm_nt_vx5_lib8)
#endif
	
	jmp		107f

104:

	cmpl	$ 6, %r14d
	jg		105f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_VX6_LIB8
#else
	CALL(inner_kernel_dgemm_nt_vx6_lib8)
#endif
	
	jmp		107f

105:

	cmpl	$ 7, %r14d
	jg		106f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_VX7_LIB8
#else
	CALL(inner_kernel_dgemm_nt_vx7_lib8)
#endif
	
	jmp		107f

106:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_VX8_LIB8
#else
	CALL(inner_kernel_dgemm_nt_vx8_lib8)
#endif

107:

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_8x8_vs_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// r13   <- 4*sdb*sizeof(double)
// zmm0  <- []
// ...
// zmm7  <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+8*k*sizeof(double)
// r12   <- B+8*k*sizeof(double)
// r13   <- 4*sdb*sizeof(double)
// zmm0  <- []
// ...
// zmm7  <- []

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_8X8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_8x8_lib8)
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	// prefetch
	prefetcht0	0*64(%r12) // software prefetch
	prefetcht0	1*64(%r12) // software prefetch
	prefetcht0	2*64(%r12) // software prefetch
	prefetcht0	3*64(%r12) // software prefetch
	prefetcht0	4*64(%r12) // software prefetch
	prefetcht0	5*64(%r12) // software prefetch
	prefetcht0	6*64(%r12) // software prefetch
	prefetcht0	7*64(%r12) // software prefetch

	// preload
	vmovapd 		0(%r11), %zmm25 // A

	cmpl	$ 8, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	vbroadcastsd	0*64(%r12), %zmm24 // B
	prefetcht0	0*64(%r12, %r13) // software prefetch
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			1*64(%r11), %zmm26 // A
	vbroadcastsd	1*64(%r12), %zmm24 // B
	prefetcht0	1*64(%r12, %r13) // software prefetch
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	2*64(%r12), %zmm24 // B
	prefetcht0	2*64(%r12, %r13) // software prefetch
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	3*64(%r12), %zmm24 // B
	prefetcht0	3*64(%r12, %r13) // software prefetch
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	4*64(%r12), %zmm24 // B
	prefetcht0	4*64(%r12, %r13) // software prefetch
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	5*64(%r12), %zmm24 // B
	prefetcht0	5*64(%r12, %r13) // software prefetch
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	6*64(%r12), %zmm24 // B
	prefetcht0	6*64(%r12, %r13) // software prefetch
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	7*64(%r12), %zmm24 // B
	prefetcht0	7*64(%r12, %r13) // software prefetch
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 1
	vbroadcastsd	8+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			2*64(%r11), %zmm25 // A
	vbroadcastsd	8+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	8+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	8+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	8+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	8+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	8+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vbroadcastsd	8+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7
	subl	$ 8, %r10d

	// unroll 2
	vbroadcastsd	16+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			3*64(%r11), %zmm26 // A
	vbroadcastsd	16+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	16+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	16+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	16+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	16+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	16+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 3
	vbroadcastsd	24+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			4*64(%r11), %zmm25 // A
	vbroadcastsd	24+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	24+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	24+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	24+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	24+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	24+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vbroadcastsd	24+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 4
	vbroadcastsd	32+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			5*64(%r11), %zmm26 // A
	vbroadcastsd	32+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	32+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	32+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	32+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	32+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	32+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 5
	vbroadcastsd	40+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			6*64(%r11), %zmm25 // A
	vbroadcastsd	40+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	40+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	40+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	40+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	40+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	40+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vbroadcastsd	40+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 6
	vbroadcastsd	48+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			7*64(%r11), %zmm26 // A
	vbroadcastsd	48+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	48+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	48+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	48+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	48+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	48+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	addq	$ 512, %r11

	// unroll 7
	vbroadcastsd	56+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			0*64(%r11), %zmm25 // A
	vbroadcastsd	56+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	56+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	56+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	56+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	56+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	56+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vbroadcastsd	56+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7
	addq	%r13, %r12

	cmpl	$ 8, %r10d
	jg		1b // main loop 


0: // consider clean8-up
	
	cmpl	$ 7, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			1*64(%r11), %zmm26 // A
	vbroadcastsd	1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	subl	$ 8, %r10d

	// unroll 1
	vbroadcastsd	8+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			2*64(%r11), %zmm25 // A
	vbroadcastsd	8+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	8+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	8+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	8+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	8+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	8+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vbroadcastsd	8+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 2
	vbroadcastsd	16+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			3*64(%r11), %zmm26 // A
	vbroadcastsd	16+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	16+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	16+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	16+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	16+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	16+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 3
	vbroadcastsd	24+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			4*64(%r11), %zmm25 // A
	vbroadcastsd	24+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	24+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	24+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	24+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	24+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	24+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vbroadcastsd	24+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 4
	vbroadcastsd	32+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			5*64(%r11), %zmm26 // A
	vbroadcastsd	32+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	32+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	32+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	32+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	32+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	32+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 5
	vbroadcastsd	40+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			6*64(%r11), %zmm25 // A
	vbroadcastsd	40+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	40+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	40+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	40+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	40+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	40+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vbroadcastsd	40+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 6
	vbroadcastsd	48+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			7*64(%r11), %zmm26 // A
	vbroadcastsd	48+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	48+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	48+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	48+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	48+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	48+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	addq	$ 512, %r11

	// unroll 7
	vbroadcastsd	56+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
//	vmovapd			0*64(%r11), %zmm25 // A
	vbroadcastsd	56+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	56+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	56+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	56+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	56+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	56+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vbroadcastsd	56+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7
	addq	%r13, %r12

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovapd			0(%r11), %zmm25 // A
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	addq	$ 64, %r11
	addq	$ 8, %r12
	subl	$ 1, %r10d

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_8x8_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// r13   <- 4*sdb*sizeof(double)
// k1    <- m-maks
// zmm0  <- []
// ...
// zmm7  <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+8*k*sizeof(double)
// r12   <- B+8*k*sizeof(double)
// r13   <- 4*sdb*sizeof(double)
// k1    <- m-maks
// zmm0  <- []
// ...
// zmm7  <- []

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_VX8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_vx8_lib8)
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	// prefetch
//	prefetcht0	0(%r12) // software prefetch
//	prefetcht0	0+64(%r12) // software prefetch

	// preload
	vmovapd 		0(%r11), %zmm25 {%k1}{z} // A

	cmpl	$ 8, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0	128(%r12) // software prefetch
//	prefetcht0	128+64(%r12) // software prefetch

	// unroll 0
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			1*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	subl	$ 8, %r10d

	// unroll 1
	vbroadcastsd	8+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			2*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	8+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	8+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	8+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	8+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	8+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vbroadcastsd	8+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 2
	vbroadcastsd	16+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			3*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	16+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	16+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	16+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	16+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	16+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	16+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 3
	vbroadcastsd	24+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			4*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	24+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	24+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	24+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	24+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	24+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	24+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vbroadcastsd	24+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 4
	vbroadcastsd	32+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			5*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	32+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	32+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	32+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	32+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	32+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	32+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 5
	vbroadcastsd	40+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			6*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	40+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	40+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	40+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	40+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	40+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	40+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vbroadcastsd	40+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 6
	vbroadcastsd	48+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			7*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	48+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	48+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	48+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	48+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	48+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	48+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	addq	$ 512, %r11

	// unroll 7
	vbroadcastsd	56+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			0*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	56+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	56+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	56+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	56+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	56+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	56+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vbroadcastsd	56+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7
	addq	%r13, %r12

	cmpl	$ 8, %r10d
	jg		1b // main loop 


0: // consider clean8-up
	
	cmpl	$ 7, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			1*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	subl	$ 8, %r10d

	// unroll 1
	vbroadcastsd	8+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			2*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	8+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	8+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	8+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	8+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	8+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vbroadcastsd	8+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 2
	vbroadcastsd	16+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			3*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	16+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	16+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	16+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	16+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	16+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	16+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 3
	vbroadcastsd	24+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			4*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	24+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	24+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	24+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	24+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	24+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	24+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vbroadcastsd	24+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 4
	vbroadcastsd	32+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			5*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	32+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	32+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	32+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	32+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	32+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	32+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 5
	vbroadcastsd	40+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			6*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	40+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	40+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	40+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	40+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	40+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	40+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vbroadcastsd	40+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 6
	vbroadcastsd	48+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			7*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	48+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	48+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	48+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	48+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	48+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	48+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	addq	$ 512, %r11

	// unroll 7
	vbroadcastsd	56+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
//	vmovapd			0*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	56+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	56+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	56+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	56+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	56+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	56+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vbroadcastsd	56+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7
	addq	%r13, %r12

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovapd			0(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	addq	$ 64, %r11
	addq	$ 8, %r12
	subl	$ 1, %r10d

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_vx8_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// r13   <- 4*sdb*sizeof(double)
// k1    <- m-maks
// zmm0  <- []
// ...
// zmm7  <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+8*k*sizeof(double)
// r12   <- B+8*k*sizeof(double)
// r13   <- 4*sdb*sizeof(double)
// k1    <- m-maks
// zmm0  <- []
// ...
// zmm7  <- []

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_VX7_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_vx7_lib8)
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	// prefetch
//	prefetcht0	0(%r12) // software prefetch
//	prefetcht0	0+64(%r12) // software prefetch

	// preload
	vmovapd 		0(%r11), %zmm25 {%k1}{z} // A

	cmpl	$ 8, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0	128(%r12) // software prefetch
//	prefetcht0	128+64(%r12) // software prefetch

	// unroll 0
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			1*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
	subl	$ 8, %r10d

	// unroll 1
	vbroadcastsd	8+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			2*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	8+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	8+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	8+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	8+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	8+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	8+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 2
	vbroadcastsd	16+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			3*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	16+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	16+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	16+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	16+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	16+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	16+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 3
	vbroadcastsd	24+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			4*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	24+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	24+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	24+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	24+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	24+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	24+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	24+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 4
	vbroadcastsd	32+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			5*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	32+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	32+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	32+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	32+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	32+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	32+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 5
	vbroadcastsd	40+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			6*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	40+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	40+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	40+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	40+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	40+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	40+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	40+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 6
	vbroadcastsd	48+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			7*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	48+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	48+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	48+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	48+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	48+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	48+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
	addq	$ 512, %r11

	// unroll 7
	vbroadcastsd	56+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			0*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	56+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	56+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	56+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	56+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	56+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	56+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	56+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7
	addq	%r13, %r12

	cmpl	$ 8, %r10d
	jg		1b // main loop 


0: // consider clean8-up
	
	cmpl	$ 7, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			1*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
	subl	$ 8, %r10d

	// unroll 1
	vbroadcastsd	8+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			2*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	8+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	8+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	8+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	8+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	8+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	8+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 2
	vbroadcastsd	16+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			3*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	16+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	16+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	16+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	16+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	16+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	16+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 3
	vbroadcastsd	24+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			4*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	24+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	24+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	24+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	24+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	24+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	24+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	24+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 4
	vbroadcastsd	32+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			5*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	32+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	32+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	32+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	32+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	32+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	32+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 5
	vbroadcastsd	40+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			6*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	40+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	40+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	40+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	40+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	40+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	40+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	40+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 6
	vbroadcastsd	48+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			7*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	48+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	48+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	48+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	48+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	48+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	48+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
	addq	$ 512, %r11

	// unroll 7
	vbroadcastsd	56+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
//	vmovapd			0*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	56+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	56+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	56+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	56+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	56+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	56+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	56+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7
	addq	%r13, %r12

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovapd			0(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7

	addq	$ 64, %r11
	addq	$ 8, %r12
	subl	$ 1, %r10d

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_vx7_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// r13   <- 4*sdb*sizeof(double)
// k1    <- m-maks
// zmm0  <- []
// ...
// zmm7  <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+8*k*sizeof(double)
// r12   <- B+8*k*sizeof(double)
// r13   <- 4*sdb*sizeof(double)
// k1    <- m-maks
// zmm0  <- []
// ...
// zmm7  <- []

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_VX6_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_vx6_lib8)
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	// prefetch
//	prefetcht0	0(%r12) // software prefetch
//	prefetcht0	0+64(%r12) // software prefetch

	// preload
	vmovapd 		0(%r11), %zmm25 {%k1}{z} // A

	cmpl	$ 8, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0	128(%r12) // software prefetch
//	prefetcht0	128+64(%r12) // software prefetch

	// unroll 0
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			1*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vbroadcastsd	6*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
	subl	$ 8, %r10d

	// unroll 1
	vbroadcastsd	8+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			2*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	8+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	8+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	8+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	8+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	8+6*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	8+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 2
	vbroadcastsd	16+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			3*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	16+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	16+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	16+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	16+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vbroadcastsd	16+6*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	16+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 3
	vbroadcastsd	24+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			4*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	24+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	24+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	24+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	24+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	24+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	24+6*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	24+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 4
	vbroadcastsd	32+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			5*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	32+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	32+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	32+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	32+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vbroadcastsd	32+6*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	32+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 5
	vbroadcastsd	40+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			6*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	40+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	40+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	40+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	40+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	40+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	40+6*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	40+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 6
	vbroadcastsd	48+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			7*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	48+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	48+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	48+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	48+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	48+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vbroadcastsd	48+6*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	48+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
	addq	$ 512, %r11

	// unroll 7
	vbroadcastsd	56+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			0*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	56+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	56+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	56+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	56+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	56+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	56+6*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	56+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7
	addq	%r13, %r12

	cmpl	$ 8, %r10d
	jg		1b // main loop 


0: // consider clean8-up
	
	cmpl	$ 7, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			1*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vbroadcastsd	6*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
	subl	$ 8, %r10d

	// unroll 1
	vbroadcastsd	8+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			2*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	8+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	8+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	8+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	8+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	8+6*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	8+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 2
	vbroadcastsd	16+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			3*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	16+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	16+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	16+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	16+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vbroadcastsd	16+6*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	16+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 3
	vbroadcastsd	24+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			4*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	24+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	24+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	24+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	24+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	24+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	24+6*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	24+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 4
	vbroadcastsd	32+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			5*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	32+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	32+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	32+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	32+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vbroadcastsd	32+6*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	32+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 5
	vbroadcastsd	40+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			6*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	40+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	40+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	40+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	40+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	40+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	40+6*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	40+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 6
	vbroadcastsd	48+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			7*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	48+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	48+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	48+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	48+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	48+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vbroadcastsd	48+6*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	48+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
	addq	$ 512, %r11

	// unroll 7
	vbroadcastsd	56+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
//	vmovapd			0*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	56+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	56+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	56+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	56+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vbroadcastsd	56+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	56+6*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	56+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7
	addq	%r13, %r12

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovapd			0(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vbroadcastsd	6*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7

	addq	$ 64, %r11
	addq	$ 8, %r12
	subl	$ 1, %r10d

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_vx6_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// r13   <- 4*sdb*sizeof(double)
// k1    <- m-maks
// zmm0  <- []
// ...
// zmm7  <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+8*k*sizeof(double)
// r12   <- B+8*k*sizeof(double)
// r13   <- 4*sdb*sizeof(double)
// k1    <- m-maks
// zmm0  <- []
// ...
// zmm7  <- []

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_VX5_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_vx5_lib8)
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	// prefetch
//	prefetcht0	0(%r12) // software prefetch
//	prefetcht0	0+64(%r12) // software prefetch

	// preload
	vmovapd 		0(%r11), %zmm25 {%k1}{z} // A

	cmpl	$ 8, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0	128(%r12) // software prefetch
//	prefetcht0	128+64(%r12) // software prefetch

	// unroll 0
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			1*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
//	vbroadcastsd	5*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vbroadcastsd	6*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
	subl	$ 8, %r10d

	// unroll 1
	vbroadcastsd	8+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			2*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	8+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	8+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	8+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
//	vbroadcastsd	8+5*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	8+6*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	8+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 2
	vbroadcastsd	16+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			3*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	16+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	16+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	16+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
//	vbroadcastsd	16+5*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vbroadcastsd	16+6*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	16+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 3
	vbroadcastsd	24+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			4*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	24+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	24+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	24+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	24+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
//	vbroadcastsd	24+5*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	24+6*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	24+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 4
	vbroadcastsd	32+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			5*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	32+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	32+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	32+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
//	vbroadcastsd	32+5*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vbroadcastsd	32+6*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	32+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 5
	vbroadcastsd	40+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			6*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	40+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	40+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	40+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	40+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
//	vbroadcastsd	40+5*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	40+6*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	40+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 6
	vbroadcastsd	48+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			7*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	48+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	48+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	48+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	48+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
//	vbroadcastsd	48+5*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vbroadcastsd	48+6*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	48+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
	addq	$ 512, %r11

	// unroll 7
	vbroadcastsd	56+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			0*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	56+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	56+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	56+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	56+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
//	vbroadcastsd	56+5*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	56+6*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	56+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7
	addq	%r13, %r12

	cmpl	$ 8, %r10d
	jg		1b // main loop 


0: // consider clean8-up
	
	cmpl	$ 7, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			1*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
//	vbroadcastsd	5*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vbroadcastsd	6*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
	subl	$ 8, %r10d

	// unroll 1
	vbroadcastsd	8+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			2*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	8+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	8+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	8+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
//	vbroadcastsd	8+5*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	8+6*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	8+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 2
	vbroadcastsd	16+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			3*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	16+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	16+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	16+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
//	vbroadcastsd	16+5*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vbroadcastsd	16+6*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	16+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 3
	vbroadcastsd	24+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			4*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	24+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	24+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	24+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	24+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
//	vbroadcastsd	24+5*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	24+6*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	24+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 4
	vbroadcastsd	32+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			5*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	32+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	32+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	32+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
//	vbroadcastsd	32+5*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vbroadcastsd	32+6*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	32+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 5
	vbroadcastsd	40+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
	vmovapd			6*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	40+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	40+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	40+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	40+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
//	vbroadcastsd	40+5*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	40+6*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	40+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 6
	vbroadcastsd	48+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			7*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	48+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	48+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	48+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	48+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
//	vbroadcastsd	48+5*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vbroadcastsd	48+6*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	48+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
	addq	$ 512, %r11

	// unroll 7
	vbroadcastsd	56+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm0
//	vmovapd			0*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	56+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm1
	vbroadcastsd	56+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm2
	vbroadcastsd	56+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm3
	vbroadcastsd	56+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
//	vbroadcastsd	56+5*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	56+6*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	56+7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7
	addq	%r13, %r12

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovapd			0(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
//	vbroadcastsd	5*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vbroadcastsd	6*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vbroadcastsd	7*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7

	addq	$ 64, %r11
	addq	$ 8, %r12
	subl	$ 1, %r10d

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_vx5_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// r13   <- 4*sdb*sizeof(double)
// k1    <- m-maks
// zmm0  <- []
// ...
// zmm7  <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+8*k*sizeof(double)
// r12   <- B+8*k*sizeof(double)
// r13   <- 4*sdb*sizeof(double)
// k1    <- m-maks
// zmm0  <- []
// ...
// zmm7  <- []

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_VX4_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_vx4_lib8)
#endif
	
	cmpl	$ 0, %r10d
	jle		5f // return

	// prefetch
//	prefetcht0	0(%r12) // software prefetch
//	prefetcht0	0+64(%r12) // software prefetch

	// preload
	vmovapd 		0(%r11), %zmm25 {%k1}{z} // A

	vxorpd			%zmm4, %zmm4, %zmm4
	vmovapd			%zmm4, %zmm5
	vmovapd			%zmm4, %zmm6
	vmovapd			%zmm4, %zmm7

	cmpl	$ 8, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0	128(%r12) // software prefetch
//	prefetcht0	128+64(%r12) // software prefetch

	// unroll 0
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			1*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	subl	$ 8, %r10d

	// unroll 1
	vbroadcastsd	8+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			2*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	8+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vbroadcastsd	8+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 2
	vbroadcastsd	16+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			3*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	16+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	16+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3

	// unroll 3
	vbroadcastsd	24+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			4*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	24+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	24+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vbroadcastsd	24+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 4
	vbroadcastsd	32+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			5*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	32+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	32+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	32+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3

	// unroll 5
	vbroadcastsd	40+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			6*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	40+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	40+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vbroadcastsd	40+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 6
	vbroadcastsd	48+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			7*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	48+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	48+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	48+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	addq	$ 512, %r11

	// unroll 7
	vbroadcastsd	56+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			0*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	56+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	56+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vbroadcastsd	56+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7
	addq	%r13, %r12

	cmpl	$ 8, %r10d
	jg		1b // main loop 


0: // consider clean8-up
	
	cmpl	$ 7, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			1*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	subl	$ 8, %r10d

	// unroll 1
	vbroadcastsd	8+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			2*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	8+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vbroadcastsd	8+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 2
	vbroadcastsd	16+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			3*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	16+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	16+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3

	// unroll 3
	vbroadcastsd	24+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			4*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	24+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	24+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vbroadcastsd	24+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 4
	vbroadcastsd	32+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			5*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	32+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	32+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	32+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3

	// unroll 5
	vbroadcastsd	40+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			6*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	40+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	40+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vbroadcastsd	40+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 6
	vbroadcastsd	48+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			7*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	48+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	48+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	48+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	addq	$ 512, %r11

	// unroll 7
	vbroadcastsd	56+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
//	vmovapd			0*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	56+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	56+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
	vbroadcastsd	56+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm7
	addq	%r13, %r12

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovapd			0(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3

	addq	$ 64, %r11
	addq	$ 8, %r12
	subl	$ 1, %r10d

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

	vaddpd			%zmm4, %zmm0, %zmm0
	vaddpd			%zmm5, %zmm1, %zmm1
	vaddpd			%zmm6, %zmm2, %zmm2
	vaddpd			%zmm7, %zmm3, %zmm3

5: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_vx4_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// r13   <- 4*sdb*sizeof(double)
// k1    <- m-maks
// zmm0  <- []
// ...
// zmm7  <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+8*k*sizeof(double)
// r12   <- B+8*k*sizeof(double)
// r13   <- 4*sdb*sizeof(double)
// k1    <- m-maks
// zmm0  <- []
// ...
// zmm7  <- []

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_VX3_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_vx3_lib8)
#endif
	
	cmpl	$ 0, %r10d
	jle		5f // return

	// prefetch
//	prefetcht0	0(%r12) // software prefetch
//	prefetcht0	0+64(%r12) // software prefetch

	// preload
	vmovapd 		0(%r11), %zmm25 {%k1}{z} // A

	vxorpd			%zmm4, %zmm4, %zmm4
	vmovapd			%zmm4, %zmm5
	vmovapd			%zmm4, %zmm6
//	vmovapd			%zmm4, %zmm7

	cmpl	$ 8, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0	128(%r12) // software prefetch
//	prefetcht0	128+64(%r12) // software prefetch

	// unroll 0
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			1*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
	subl	$ 8, %r10d

	// unroll 1
	vbroadcastsd	8+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			2*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	8+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	8+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 2
	vbroadcastsd	16+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			3*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	16+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	16+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3

	// unroll 3
	vbroadcastsd	24+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			4*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	24+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	24+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	24+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 4
	vbroadcastsd	32+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			5*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	32+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	32+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	32+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3

	// unroll 5
	vbroadcastsd	40+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			6*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	40+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	40+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	40+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 6
	vbroadcastsd	48+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			7*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	48+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	48+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	48+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
	addq	$ 512, %r11

	// unroll 7
	vbroadcastsd	56+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			0*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	56+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	56+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	56+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7
	addq	%r13, %r12

	cmpl	$ 8, %r10d
	jg		1b // main loop 


0: // consider clean8-up
	
	cmpl	$ 7, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			1*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
	subl	$ 8, %r10d

	// unroll 1
	vbroadcastsd	8+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			2*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	8+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	8+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 2
	vbroadcastsd	16+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			3*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	16+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	16+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3

	// unroll 3
	vbroadcastsd	24+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			4*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	24+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	24+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	24+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 4
	vbroadcastsd	32+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			5*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	32+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	32+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	32+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3

	// unroll 5
	vbroadcastsd	40+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			6*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	40+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	40+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	40+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 6
	vbroadcastsd	48+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			7*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	48+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	48+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	48+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
	addq	$ 512, %r11

	// unroll 7
	vbroadcastsd	56+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
//	vmovapd			0*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	56+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
	vbroadcastsd	56+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	56+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7
	addq	%r13, %r12

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovapd			0(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3

	addq	$ 64, %r11
	addq	$ 8, %r12
	subl	$ 1, %r10d

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

	vaddpd			%zmm4, %zmm0, %zmm0
	vaddpd			%zmm5, %zmm1, %zmm1
	vaddpd			%zmm6, %zmm2, %zmm2
//	vaddpd			%zmm7, %zmm3, %zmm3

5: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_vx3_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// r13   <- 4*sdb*sizeof(double)
// k1    <- m-maks
// zmm0  <- []
// ...
// zmm7  <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+8*k*sizeof(double)
// r12   <- B+8*k*sizeof(double)
// r13   <- 4*sdb*sizeof(double)
// k1    <- m-maks
// zmm0  <- []
// ...
// zmm7  <- []

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_VX2_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_vx2_lib8)
#endif
	
	cmpl	$ 0, %r10d
	jle		5f // return

	// prefetch
//	prefetcht0	0(%r12) // software prefetch
//	prefetcht0	0+64(%r12) // software prefetch

	// preload
	vmovapd 		0(%r11), %zmm25 {%k1}{z} // A

	vxorpd			%zmm4, %zmm4, %zmm4
	vmovapd			%zmm4, %zmm5
//	vmovapd			%zmm4, %zmm6
//	vmovapd			%zmm4, %zmm7

	cmpl	$ 8, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0	128(%r12) // software prefetch
//	prefetcht0	128+64(%r12) // software prefetch

	// unroll 0
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			1*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vbroadcastsd	2*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
	subl	$ 8, %r10d

	// unroll 1
	vbroadcastsd	8+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			2*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	8+2*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	8+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 2
	vbroadcastsd	16+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			3*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	16+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vbroadcastsd	16+2*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	16+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3

	// unroll 3
	vbroadcastsd	24+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			4*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	24+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	24+2*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	24+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 4
	vbroadcastsd	32+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			5*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	32+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vbroadcastsd	32+2*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	32+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3

	// unroll 5
	vbroadcastsd	40+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			6*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	40+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	40+2*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	40+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 6
	vbroadcastsd	48+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			7*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	48+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vbroadcastsd	48+2*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	48+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
	addq	$ 512, %r11

	// unroll 7
	vbroadcastsd	56+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			0*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	56+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	56+2*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	56+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7
	addq	%r13, %r12

	cmpl	$ 8, %r10d
	jg		1b // main loop 


0: // consider clean8-up
	
	cmpl	$ 7, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			1*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vbroadcastsd	2*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
	subl	$ 8, %r10d

	// unroll 1
	vbroadcastsd	8+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			2*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	8+2*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	8+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 2
	vbroadcastsd	16+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			3*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	16+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vbroadcastsd	16+2*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	16+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3

	// unroll 3
	vbroadcastsd	24+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			4*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	24+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	24+2*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	24+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 4
	vbroadcastsd	32+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			5*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	32+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vbroadcastsd	32+2*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	32+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3

	// unroll 5
	vbroadcastsd	40+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			6*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	40+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	40+2*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	40+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 6
	vbroadcastsd	48+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			7*64(%r11), %zmm26 {%k1}{z} // A
	vbroadcastsd	48+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vbroadcastsd	48+2*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	48+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
	addq	$ 512, %r11

	// unroll 7
	vbroadcastsd	56+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
//	vmovapd			0*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	56+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	56+2*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	56+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7
	addq	%r13, %r12

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovapd			0(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vbroadcastsd	2*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3

	addq	$ 64, %r11
	addq	$ 8, %r12
	subl	$ 1, %r10d

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

	vaddpd			%zmm4, %zmm0, %zmm0
	vaddpd			%zmm5, %zmm1, %zmm1
//	vaddpd			%zmm6, %zmm2, %zmm2
//	vaddpd			%zmm7, %zmm3, %zmm3

5: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_vx2_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// r13   <- 4*sdb*sizeof(double)
// k1    <- m-maks
// zmm0  <- []
// ...
// zmm7  <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+8*k*sizeof(double)
// r12   <- B+8*k*sizeof(double)
// r13   <- 4*sdb*sizeof(double)
// k1    <- m-maks
// zmm0  <- []
// ...
// zmm7  <- []

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_VX1_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_vx1_lib8)
#endif
	
	cmpl	$ 0, %r10d
	jle		5f // return

	// prefetch
//	prefetcht0	0(%r12) // software prefetch
//	prefetcht0	0+64(%r12) // software prefetch

	// preload
	vmovapd 		0(%r11), %zmm25 {%k1}{z} // A

	vxorpd			%zmm4, %zmm4, %zmm4
//	vmovapd			%zmm4, %zmm5
//	vmovapd			%zmm4, %zmm6
//	vmovapd			%zmm4, %zmm7

	cmpl	$ 8, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0	128(%r12) // software prefetch
//	prefetcht0	128+64(%r12) // software prefetch

	// unroll 0
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			1*64(%r11), %zmm26 {%k1}{z} // A
//	vbroadcastsd	1*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vbroadcastsd	2*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
	subl	$ 8, %r10d

	// unroll 1
	vbroadcastsd	8+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			2*64(%r11), %zmm25 {%k1}{z} // A
//	vbroadcastsd	8+1*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	8+2*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	8+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 2
	vbroadcastsd	16+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			3*64(%r11), %zmm26 {%k1}{z} // A
//	vbroadcastsd	16+1*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vbroadcastsd	16+2*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	16+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3

	// unroll 3
	vbroadcastsd	24+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			4*64(%r11), %zmm25 {%k1}{z} // A
//	vbroadcastsd	24+1*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	24+2*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	24+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 4
	vbroadcastsd	32+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			5*64(%r11), %zmm26 {%k1}{z} // A
//	vbroadcastsd	32+1*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vbroadcastsd	32+2*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	32+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3

	// unroll 5
	vbroadcastsd	40+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			6*64(%r11), %zmm25 {%k1}{z} // A
//	vbroadcastsd	40+1*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	40+2*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	40+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 6
	vbroadcastsd	48+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			7*64(%r11), %zmm26 {%k1}{z} // A
//	vbroadcastsd	48+1*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vbroadcastsd	48+2*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	48+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
	addq	$ 512, %r11

	// unroll 7
	vbroadcastsd	56+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			0*64(%r11), %zmm25 {%k1}{z} // A
//	vbroadcastsd	56+1*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	56+2*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	56+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7
	addq	%r13, %r12

	cmpl	$ 8, %r10d
	jg		1b // main loop 


0: // consider clean8-up
	
	cmpl	$ 7, %r10d
	jle		4f // clean1

	// unroll 0
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			1*64(%r11), %zmm26 {%k1}{z} // A
//	vbroadcastsd	1*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vbroadcastsd	2*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
	subl	$ 8, %r10d

	// unroll 1
	vbroadcastsd	8+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			2*64(%r11), %zmm25 {%k1}{z} // A
//	vbroadcastsd	8+1*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	8+2*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	8+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 2
	vbroadcastsd	16+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			3*64(%r11), %zmm26 {%k1}{z} // A
//	vbroadcastsd	16+1*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vbroadcastsd	16+2*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	16+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3

	// unroll 3
	vbroadcastsd	24+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			4*64(%r11), %zmm25 {%k1}{z} // A
//	vbroadcastsd	24+1*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	24+2*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	24+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 4
	vbroadcastsd	32+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			5*64(%r11), %zmm26 {%k1}{z} // A
//	vbroadcastsd	32+1*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vbroadcastsd	32+2*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	32+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3

	// unroll 5
	vbroadcastsd	40+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
	vmovapd			6*64(%r11), %zmm25 {%k1}{z} // A
//	vbroadcastsd	40+1*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	40+2*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	40+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7

	// unroll 6
	vbroadcastsd	48+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vmovapd			7*64(%r11), %zmm26 {%k1}{z} // A
//	vbroadcastsd	48+1*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vbroadcastsd	48+2*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	48+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
	addq	$ 512, %r11

	// unroll 7
	vbroadcastsd	56+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm26, %zmm24, %zmm4
//	vmovapd			0*64(%r11), %zmm25 {%k1}{z} // A
//	vbroadcastsd	56+1*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm5
//	vbroadcastsd	56+2*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm6
//	vbroadcastsd	56+3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm26, %zmm24, %zmm7
	addq	%r13, %r12

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovapd			0(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
//	vbroadcastsd	1*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vbroadcastsd	2*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vbroadcastsd	3*64(%r12), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3

	addq	$ 64, %r11
	addq	$ 8, %r12
	subl	$ 1, %r10d

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

	vaddpd			%zmm4, %zmm0, %zmm0
//	vaddpd			%zmm5, %zmm1, %zmm1
//	vaddpd			%zmm6, %zmm2, %zmm2
//	vaddpd			%zmm7, %zmm3, %zmm3

5: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_vx1_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// r13   <- 4*sdb*sizeof(double)
// r14   <- m1
// r15   <- n1
// zmm0  <- []
// ...
// zmm7  <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+8*k*sizeof(double)
// r12   <- B+8*k*sizeof(double)
// r13   <- 4*sdb*sizeof(double)
// r14   <- m1
// r15   <- n1
// zmm0  <- []
// ...
// zmm7  <- []

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_8X8_VS_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_8x8_vs_lib8)
#endif

	// compute mask for rows
	vcvtsi2sd	%r14d, %xmm25, %xmm25
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC00(%rip), %zmm24
#elif defined(OS_MAC)
	vmovupd		LC00(%rip), %zmm24
#endif
	vbroadcastsd	%xmm25, %zmm25
	vsubpd		%zmm25, %zmm24, %zmm25
	vpmovq2m	%zmm25, %k1

	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_VX1_LIB8
#else
	CALL(inner_kernel_dgemm_nn_vx1_lib8)
#endif
	
	jmp		107f

100:

	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_VX2_LIB8
#else
	CALL(inner_kernel_dgemm_nn_vx2_lib8)
#endif
	
	jmp		107f

101:

	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_VX3_LIB8
#else
	CALL(inner_kernel_dgemm_nn_vx3_lib8)
#endif
	
	jmp		107f

102:

	cmpl	$ 4, %r15d
	jg		103f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_VX4_LIB8
#else
	CALL(inner_kernel_dgemm_nn_vx4_lib8)
#endif
	
	jmp		107f

103:

	cmpl	$ 5, %r15d
	jg		104f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_VX5_LIB8
#else
	CALL(inner_kernel_dgemm_nn_vx5_lib8)
#endif
	
	jmp		107f

104:

	cmpl	$ 6, %r15d
	jg		105f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_VX6_LIB8
#else
	CALL(inner_kernel_dgemm_nn_vx6_lib8)
#endif
	
	jmp		107f

105:

	cmpl	$ 7, %r15d
	jg		106f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_VX7_LIB8
#else
	CALL(inner_kernel_dgemm_nn_vx7_lib8)
#endif
	
	jmp		107f

106:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_VX8_LIB8
#else
	CALL(inner_kernel_dgemm_nn_vx8_lib8)
#endif

107:

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_8x8_vs_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- B
// r12   <- C
// zmm0  <- [a00 ... a70]
// ...
// zmm7  <- [a07 ... a77]

//
// output arguments:
// r10d  <- 0
// r11   <- ?
// r12   <- ?
// zmm0  <- [a00 ... a70]
// ...
// zmm7  <- [a07 ... a77]

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEBP_NN_8X8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgebp_nn_8x8_lib8)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return

	cmpl	$ 3, %r10d
	jle		2f // cleanup loop

	// main loop
	.p2align 
1:
	// merged k-iter 0-3 to have more independent acc
	vmovapd			0*64(%r12), %zmm28
	vbroadcastsd	0+0*64(%r11), %zmm25
	vfmadd231pd		%zmm0, %zmm25, %zmm28
	vmovapd			1*64(%r12), %zmm29
	vbroadcastsd	0+1*64(%r11), %zmm25
	vfmadd231pd		%zmm0, %zmm25, %zmm29
	vmovapd			2*64(%r12), %zmm30
	vbroadcastsd	0+2*64(%r11), %zmm25
	vfmadd231pd		%zmm0, %zmm25, %zmm30
	vmovapd			3*64(%r12), %zmm31
	vbroadcastsd	0+3*64(%r11), %zmm25
	vfmadd231pd		%zmm0, %zmm25, %zmm31

	vbroadcastsd	8+0*64(%r11), %zmm25
	vfmadd231pd		%zmm1, %zmm25, %zmm28
	vbroadcastsd	8+1*64(%r11), %zmm25
	vfmadd231pd		%zmm1, %zmm25, %zmm29
	vbroadcastsd	8+2*64(%r11), %zmm25
	vfmadd231pd		%zmm1, %zmm25, %zmm30
	vbroadcastsd	8+3*64(%r11), %zmm25
	vfmadd231pd		%zmm1, %zmm25, %zmm31

	vbroadcastsd	16+0*64(%r11), %zmm25
	vfmadd231pd		%zmm2, %zmm25, %zmm28
	vbroadcastsd	16+1*64(%r11), %zmm25
	vfmadd231pd		%zmm2, %zmm25, %zmm29
	vbroadcastsd	16+2*64(%r11), %zmm25
	vfmadd231pd		%zmm2, %zmm25, %zmm30
	vbroadcastsd	16+3*64(%r11), %zmm25
	vfmadd231pd		%zmm2, %zmm25, %zmm31

	vbroadcastsd	24+0*64(%r11), %zmm25
	vfmadd231pd		%zmm3, %zmm25, %zmm28
	vbroadcastsd	24+1*64(%r11), %zmm25
	vfmadd231pd		%zmm3, %zmm25, %zmm29
	vbroadcastsd	24+2*64(%r11), %zmm25
	vfmadd231pd		%zmm3, %zmm25, %zmm30
	vbroadcastsd	24+3*64(%r11), %zmm25
	vfmadd231pd		%zmm3, %zmm25, %zmm31

	vbroadcastsd	32+0*64(%r11), %zmm25
	vfmadd231pd		%zmm4, %zmm25, %zmm28
	vbroadcastsd	32+1*64(%r11), %zmm25
	vfmadd231pd		%zmm4, %zmm25, %zmm29
	vbroadcastsd	32+2*64(%r11), %zmm25
	vfmadd231pd		%zmm4, %zmm25, %zmm30
	vbroadcastsd	32+3*64(%r11), %zmm25
	vfmadd231pd		%zmm4, %zmm25, %zmm31

	vbroadcastsd	40+0*64(%r11), %zmm25
	vfmadd231pd		%zmm5, %zmm25, %zmm28
	vbroadcastsd	40+1*64(%r11), %zmm25
	vfmadd231pd		%zmm5, %zmm25, %zmm29
	vbroadcastsd	40+2*64(%r11), %zmm25
	vfmadd231pd		%zmm5, %zmm25, %zmm30
	vbroadcastsd	40+3*64(%r11), %zmm25
	vfmadd231pd		%zmm5, %zmm25, %zmm31

	vbroadcastsd	48+0*64(%r11), %zmm25
	vfmadd231pd		%zmm6, %zmm25, %zmm28
	vbroadcastsd	48+1*64(%r11), %zmm25
	vfmadd231pd		%zmm6, %zmm25, %zmm29
	vbroadcastsd	48+2*64(%r11), %zmm25
	vfmadd231pd		%zmm6, %zmm25, %zmm30
	vbroadcastsd	48+3*64(%r11), %zmm25
	vfmadd231pd		%zmm6, %zmm25, %zmm31

	vbroadcastsd	56+0*64(%r11), %zmm25
	vfmadd231pd		%zmm7, %zmm25, %zmm28
	vmovapd			%zmm28, 0*64(%r12)
	vbroadcastsd	56+1*64(%r11), %zmm25
	vfmadd231pd		%zmm7, %zmm25, %zmm29
	vmovapd			%zmm29, 1*64(%r12)
	vbroadcastsd	56+2*64(%r11), %zmm25
	vfmadd231pd		%zmm7, %zmm25, %zmm30
	vmovapd			%zmm30, 2*64(%r12)
	vbroadcastsd	56+3*64(%r11), %zmm25
	vfmadd231pd		%zmm7, %zmm25, %zmm31
	vmovapd			%zmm31, 3*64(%r12)

	addq	$ 256, %r11
	addq	$ 256, %r12
	subl	$ 4, %r10d

	cmpl	$ 3, %r10d
	jg		1b // main loop

	cmpl	$ 0, %r10d
	jle		0f // return

	// cleanup loop
2:
	vmovapd			0*64(%r12), %zmm28
	vbroadcastsd	0+0*64(%r11), %zmm25
	vfmadd231pd		%zmm0, %zmm25, %zmm28
	vbroadcastsd	8+0*64(%r11), %zmm25
	vfmadd231pd		%zmm1, %zmm25, %zmm28
	vbroadcastsd	16+0*64(%r11), %zmm25
	vfmadd231pd		%zmm2, %zmm25, %zmm28
	vbroadcastsd	24+0*64(%r11), %zmm25
	vfmadd231pd		%zmm3, %zmm25, %zmm28
	vbroadcastsd	32+0*64(%r11), %zmm25
	vfmadd231pd		%zmm4, %zmm25, %zmm28
	vbroadcastsd	40+0*64(%r11), %zmm25
	vfmadd231pd		%zmm5, %zmm25, %zmm28
	vbroadcastsd	48+0*64(%r11), %zmm25
	vfmadd231pd		%zmm6, %zmm25, %zmm28
	vbroadcastsd	56+0*64(%r11), %zmm25
	vfmadd231pd		%zmm7, %zmm25, %zmm28
	vmovapd			%zmm28, 0*64(%r12)

	subl	$ 1, %r10d
	addq	$ 64, %r11
	addq	$ 64, %r12

	cmpl	$ 0, %r10d
	jg		2b // main loop

	// return
0:

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgebp_nn_8x8_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- B
// r12   <- C
// k1    <- m-mask
// zmm0  <- [a00 ... a70]
// ...
// zmm7  <- [a07 ... a77]

//
// output arguments:
// r10d  <- 0
// r11   <- ?
// r12   <- ?
// k1    <- m-mask
// zmm0  <- [a00 ... a70]
// ...
// zmm7  <- [a07 ... a77]

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEBP_NN_VX8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgebp_nn_vx8_lib8)
#endif

	cmpl	$ 0, %r10d
	jle		0f // return

	cmpl	$ 3, %r10d
	jle		2f // cleanup loop

	// main loop
	.p2align 
1:
	// merged k-iter 0-3 to have more independent acc
	vmovapd			0*64(%r12), %zmm28 {%k1}{z}
	vbroadcastsd	0+0*64(%r11), %zmm25
	vfmadd231pd		%zmm0, %zmm25, %zmm28
	vmovapd			1*64(%r12), %zmm29 {%k1}{z}
	vbroadcastsd	0+1*64(%r11), %zmm25
	vfmadd231pd		%zmm0, %zmm25, %zmm29
	vmovapd			2*64(%r12), %zmm30 {%k1}{z}
	vbroadcastsd	0+2*64(%r11), %zmm25
	vfmadd231pd		%zmm0, %zmm25, %zmm30
	vmovapd			3*64(%r12), %zmm31 {%k1}{z}
	vbroadcastsd	0+3*64(%r11), %zmm25
	vfmadd231pd		%zmm0, %zmm25, %zmm31

	vbroadcastsd	8+0*64(%r11), %zmm25
	vfmadd231pd		%zmm1, %zmm25, %zmm28
	vbroadcastsd	8+1*64(%r11), %zmm25
	vfmadd231pd		%zmm1, %zmm25, %zmm29
	vbroadcastsd	8+2*64(%r11), %zmm25
	vfmadd231pd		%zmm1, %zmm25, %zmm30
	vbroadcastsd	8+3*64(%r11), %zmm25
	vfmadd231pd		%zmm1, %zmm25, %zmm31

	vbroadcastsd	16+0*64(%r11), %zmm25
	vfmadd231pd		%zmm2, %zmm25, %zmm28
	vbroadcastsd	16+1*64(%r11), %zmm25
	vfmadd231pd		%zmm2, %zmm25, %zmm29
	vbroadcastsd	16+2*64(%r11), %zmm25
	vfmadd231pd		%zmm2, %zmm25, %zmm30
	vbroadcastsd	16+3*64(%r11), %zmm25
	vfmadd231pd		%zmm2, %zmm25, %zmm31

	vbroadcastsd	24+0*64(%r11), %zmm25
	vfmadd231pd		%zmm3, %zmm25, %zmm28
	vbroadcastsd	24+1*64(%r11), %zmm25
	vfmadd231pd		%zmm3, %zmm25, %zmm29
	vbroadcastsd	24+2*64(%r11), %zmm25
	vfmadd231pd		%zmm3, %zmm25, %zmm30
	vbroadcastsd	24+3*64(%r11), %zmm25
	vfmadd231pd		%zmm3, %zmm25, %zmm31

	vbroadcastsd	32+0*64(%r11), %zmm25
	vfmadd231pd		%zmm4, %zmm25, %zmm28
	vbroadcastsd	32+1*64(%r11), %zmm25
	vfmadd231pd		%zmm4, %zmm25, %zmm29
	vbroadcastsd	32+2*64(%r11), %zmm25
	vfmadd231pd		%zmm4, %zmm25, %zmm30
	vbroadcastsd	32+3*64(%r11), %zmm25
	vfmadd231pd		%zmm4, %zmm25, %zmm31

	vbroadcastsd	40+0*64(%r11), %zmm25
	vfmadd231pd		%zmm5, %zmm25, %zmm28
	vbroadcastsd	40+1*64(%r11), %zmm25
	vfmadd231pd		%zmm5, %zmm25, %zmm29
	vbroadcastsd	40+2*64(%r11), %zmm25
	vfmadd231pd		%zmm5, %zmm25, %zmm30
	vbroadcastsd	40+3*64(%r11), %zmm25
	vfmadd231pd		%zmm5, %zmm25, %zmm31

	vbroadcastsd	48+0*64(%r11), %zmm25
	vfmadd231pd		%zmm6, %zmm25, %zmm28
	vbroadcastsd	48+1*64(%r11), %zmm25
	vfmadd231pd		%zmm6, %zmm25, %zmm29
	vbroadcastsd	48+2*64(%r11), %zmm25
	vfmadd231pd		%zmm6, %zmm25, %zmm30
	vbroadcastsd	48+3*64(%r11), %zmm25
	vfmadd231pd		%zmm6, %zmm25, %zmm31

	vbroadcastsd	56+0*64(%r11), %zmm25
	vfmadd231pd		%zmm7, %zmm25, %zmm28
	vmovapd			%zmm28, 0*64(%r12) {%k1}
	vbroadcastsd	56+1*64(%r11), %zmm25
	vfmadd231pd		%zmm7, %zmm25, %zmm29
	vmovapd			%zmm29, 1*64(%r12) {%k1}
	vbroadcastsd	56+2*64(%r11), %zmm25
	vfmadd231pd		%zmm7, %zmm25, %zmm30
	vmovapd			%zmm30, 2*64(%r12) {%k1}
	vbroadcastsd	56+3*64(%r11), %zmm25
	vfmadd231pd		%zmm7, %zmm25, %zmm31
	vmovapd			%zmm31, 3*64(%r12) {%k1}

	addq	$ 256, %r11
	addq	$ 256, %r12
	subl	$ 4, %r10d

	cmpl	$ 3, %r10d
	jg		1b // main loop

	cmpl	$ 0, %r10d
	jle		0f // return

	// cleanup loop
2:
	vmovapd			0*64(%r12), %zmm28 {%k1}{z}
	vbroadcastsd	0+0*64(%r11), %zmm25
	vfmadd231pd		%zmm0, %zmm25, %zmm28
	vbroadcastsd	8+0*64(%r11), %zmm25
	vfmadd231pd		%zmm1, %zmm25, %zmm28
	vbroadcastsd	16+0*64(%r11), %zmm25
	vfmadd231pd		%zmm2, %zmm25, %zmm28
	vbroadcastsd	24+0*64(%r11), %zmm25
	vfmadd231pd		%zmm3, %zmm25, %zmm28
	vbroadcastsd	32+0*64(%r11), %zmm25
	vfmadd231pd		%zmm4, %zmm25, %zmm28
	vbroadcastsd	40+0*64(%r11), %zmm25
	vfmadd231pd		%zmm5, %zmm25, %zmm28
	vbroadcastsd	48+0*64(%r11), %zmm25
	vfmadd231pd		%zmm6, %zmm25, %zmm28
	vbroadcastsd	56+0*64(%r11), %zmm25
	vfmadd231pd		%zmm7, %zmm25, %zmm28
	vmovapd			%zmm28, 0*64(%r12) {%k1}

	subl	$ 1, %r10d
	addq	$ 64, %r11
	addq	$ 64, %r12

	cmpl	$ 0, %r10d
	jg		2b // main loop

	// return
0:

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgebp_nn_vx8_lib8)
#endif





// common inner routine with file scope
//
// edge for B unaligned
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// r13   <- 4*sdb*sizeof(double)
// r14   <- offB
// zmm0  <- []
// ...
// zmm7  <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+8*k*sizeof(double)
// r12   <- B+8*k*sizeof(double)
// r13   <- 4*sdb*sizeof(double)
// r14   <- offB
// zmm0  <- []
// ...
// zmm7  <- []


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DGEMM_NN_8X8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_dgemm_nn_8x8_lib8)
#endif
	
	cmpl			$ 0, %r14d // offset==0
	jle				2f // end

	cmpl			$ 0, %r10d // k==0
	jle				2f // end

	movl			$ 8, %r15d
	subl			%r14d, %r15d // 8-offsetB
	cmpl			%r10d, %r15d
//	jle				0f
//	movl			%r10d, %r15d // kend=min(k,4-offsetB)
//0:
	cmovgl			%r10d, %r15d // kend=min(k,4-offsetB)

	movl			%r14d, %eax
	sall			$ 3, %eax // offsetB*sizeof(double)
	addq			%rax, %r12 // B+offsetB*sizeof(double)

1:
	vmovapd			0(%r11), %zmm25 // A
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	subl			$ 1, %r10d // k-1
	subl			$ 1, %r15d // kend-1
	addq			$ 64, %r11 // A+1*bs*sizeof(double)
	addq			$ 8, %r12 // B+1*sizeof(double)

	cmpl			$ 0, %r15d
	jg				1b

	cmpl			$ 0, %r10d
	jle				2f // end

	addq			%r13, %r12
	subq			$ 64, %r12 // B+bs*(sdb-1)*sizeof(double)

2:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dgemm_nn_8x8_lib8)
#endif





#if 0 // TODO keep as reference !!!!!!!!!!!!!!!!!!!!!!
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
// zmm0  <- []
// ...
// zmm7  <- []

//
// output arguments:
// r10   <- k-(4-offB)
// r11   <- A+(4-offB)*bs*sizeof(double)
// r12   <- B-offB+bs*sdb*sizeof(double)
// r13   <- bs*sdb*sizeof(double)
// r14   <- offB
// zmm0  <- []
// ...
// zmm7  <- []


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NN_RL_8X8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nn_rl_8x8_lib8)
#endif
	
	cmpl	$ 0, %r14d
	jg		0f

	// offB==0

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vbroadcastsd	8+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	8+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vbroadcastsd	16+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	16+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vbroadcastsd	24+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	24+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	24+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3

	// unroll 4
	vmovapd			4*64(%r11), %zmm25 // A
	vbroadcastsd	32+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	32+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	32+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	32+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4

	// unroll 5
	vmovapd			5*64(%r11), %zmm25 // A
	vbroadcastsd	40+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	40+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	40+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	40+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	40+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5

	// unroll 6
	vmovapd			6*64(%r11), %zmm25 // A
	vbroadcastsd	48+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	48+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	48+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	48+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	48+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	48+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6

	// unroll 7
	vmovapd			7*64(%r11), %zmm25 // A
	vbroadcastsd	56+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	56+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	56+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	56+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	56+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	56+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	56+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	56+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	subl	$ 8, %r10d
	addq	$ 8*64, %r11
	addq	%r13, %r12

	jmp		7f

0:
	cmpl	$ 1, %r14d
	jg		1f

	// offB==1

	addq			$ 8, %r12 // B+1*sizeof(double)

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vbroadcastsd	8+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	8+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vbroadcastsd	16+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	16+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vbroadcastsd	24+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	24+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	24+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3

	// unroll 4
	vmovapd			4*64(%r11), %zmm25 // A
	vbroadcastsd	32+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	32+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	32+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	32+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4

	// unroll 5
	vmovapd			5*64(%r11), %zmm25 // A
	vbroadcastsd	40+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	40+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	40+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	40+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	40+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5

	// unroll 6
	vmovapd			6*64(%r11), %zmm25 // A
	vbroadcastsd	48+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	48+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	48+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	48+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	48+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	48+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6

	subl			$ 7, %r10d // k-7
	addq			$ 7*64, %r11 // A+7*bs*sizeof(double)
	addq			%r13, %r12
	subq			$ 8, %r12 // B+bs*sdb*sizeof(double)-1

	jmp		7f

1:
	cmpl	$ 2, %r14d
	jg		2f

	// offB==2

	addq			$ 16, %r12 // B+2*sizeof(double)

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vbroadcastsd	8+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	8+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vbroadcastsd	16+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	16+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vbroadcastsd	24+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	24+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	24+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3

	// unroll 4
	vmovapd			4*64(%r11), %zmm25 // A
	vbroadcastsd	32+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	32+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	32+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	32+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4

	// unroll 5
	vmovapd			5*64(%r11), %zmm25 // A
	vbroadcastsd	40+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	40+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	40+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	40+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	40+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5

	subl			$ 6, %r10d // k-6
	addq			$ 6*64, %r11 // A+2*bs*sizeof(double)
	addq			%r13, %r12
	subq			$ 16, %r12 // B+bs*sdb*sizeof(double)-2

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vbroadcastsd	8+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	8+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	8+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	8+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	8+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	8+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	8+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	8+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vbroadcastsd	16+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	16+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	16+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	16+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	16+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	16+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	16+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vbroadcastsd	24+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	24+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	24+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	24+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	24+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	24+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	24+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 4
	vmovapd			4*64(%r11), %zmm25 // A
	vbroadcastsd	32+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	32+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	32+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	32+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	32+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	32+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	32+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 5
	vmovapd			5*64(%r11), %zmm25 // A
	vbroadcastsd	40+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	40+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	40+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	40+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	40+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	40+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	40+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 6
	vmovapd			6*64(%r11), %zmm25 // A
	vbroadcastsd	48+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	48+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	48+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	48+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	48+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	48+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	48+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 7
	vmovapd			7*64(%r11), %zmm25 // A
	vbroadcastsd	56+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	56+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	56+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	56+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	56+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	56+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	56+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	56+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	subl			$ 8, %r10d // k-8
	addq			$ 8*64, %r11 // A+4*bs*sizeof(double)
	addq			%r13, %r12 // B+bs*sdb*sizeof(double)

	jmp		7f

2:
	cmpl	$ 3, %r14d
	jg		3f

	// offB==3

	addq			$ 24, %r12 // B+3*sizeof(double)

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vbroadcastsd	8+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	8+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vbroadcastsd	16+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	16+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vbroadcastsd	24+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	24+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	24+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3

	// unroll 4
	vmovapd			4*64(%r11), %zmm25 // A
	vbroadcastsd	32+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	32+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	32+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	32+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4

	subl			$ 5, %r10d // k-1
	addq			$ 5*64, %r11 // A+1*bs*sizeof(double)
	addq			%r13, %r12
	subq			$ 24, %r12 // B+bs*sdb*sizeof(double)-3

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vbroadcastsd	8+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	8+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	8+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	8+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	8+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	8+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	8+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vbroadcastsd	16+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	16+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	16+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	16+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	16+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	16+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	16+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vbroadcastsd	24+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	24+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	24+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	24+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	24+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	24+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	24+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 4
	vmovapd			4*64(%r11), %zmm25 // A
	vbroadcastsd	32+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	32+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	32+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	32+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	32+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	32+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	32+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 5
	vmovapd			5*64(%r11), %zmm25 // A
	vbroadcastsd	40+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	40+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	40+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	40+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	40+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	40+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	40+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 6
	vmovapd			6*64(%r11), %zmm25 // A
	vbroadcastsd	48+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	48+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	48+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	48+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	48+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	48+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	48+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 7
	vmovapd			7*64(%r11), %zmm25 // A
	vbroadcastsd	56+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	56+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	56+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	56+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	56+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	56+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	56+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	56+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	subl			$ 8, %r10d // k-8
	addq			$ 8*64, %r11 // A+4*bs*sizeof(double)
	addq			%r13, %r12 // B+bs*sdb*sizeof(double)

3:
	cmpl	$ 4, %r14d
	jg		4f

	// offB==4

	addq			$ 32, %r12 // B+3*sizeof(double)

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vbroadcastsd	8+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	8+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vbroadcastsd	16+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	16+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vbroadcastsd	24+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	24+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	24+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3

	subl			$ 4, %r10d // k-1
	addq			$ 4*64, %r11 // A+1*bs*sizeof(double)
	addq			%r13, %r12
	subq			$ 32, %r12 // B+bs*sdb*sizeof(double)-3

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vbroadcastsd	8+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	8+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	8+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	8+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	8+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	8+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vbroadcastsd	16+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	16+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	16+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	16+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	16+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	16+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vbroadcastsd	24+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	24+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	24+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	24+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	24+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	24+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	24+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 4
	vmovapd			4*64(%r11), %zmm25 // A
	vbroadcastsd	32+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	32+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	32+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	32+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	32+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	32+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	32+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 5
	vmovapd			5*64(%r11), %zmm25 // A
	vbroadcastsd	40+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	40+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	40+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	40+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	40+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	40+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	40+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 6
	vmovapd			6*64(%r11), %zmm25 // A
	vbroadcastsd	48+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	48+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	48+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	48+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	48+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	48+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	48+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 7
	vmovapd			7*64(%r11), %zmm25 // A
	vbroadcastsd	56+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	56+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	56+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	56+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	56+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	56+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	56+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	56+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	subl			$ 8, %r10d // k-8
	addq			$ 8*64, %r11 // A+4*bs*sizeof(double)
	addq			%r13, %r12 // B+bs*sdb*sizeof(double)

4:
	cmpl	$ 5, %r14d
	jg		5f

	// offB==5

	addq			$ 40, %r12 // B+3*sizeof(double)

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vbroadcastsd	8+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	8+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vbroadcastsd	16+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	16+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2

	subl			$ 3, %r10d // k-1
	addq			$ 3*64, %r11 // A+1*bs*sizeof(double)
	addq			%r13, %r12
	subq			$ 40, %r12 // B+bs*sdb*sizeof(double)-3

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vbroadcastsd	8+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	8+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	8+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	8+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	8+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vbroadcastsd	16+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	16+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	16+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	16+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	16+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vbroadcastsd	24+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	24+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	24+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	24+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	24+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	24+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6

	// unroll 4
	vmovapd			4*64(%r11), %zmm25 // A
	vbroadcastsd	32+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	32+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	32+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	32+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	32+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	32+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	32+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 5
	vmovapd			5*64(%r11), %zmm25 // A
	vbroadcastsd	40+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	40+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	40+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	40+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	40+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	40+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	40+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 6
	vmovapd			6*64(%r11), %zmm25 // A
	vbroadcastsd	48+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	48+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	48+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	48+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	48+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	48+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	48+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 7
	vmovapd			7*64(%r11), %zmm25 // A
	vbroadcastsd	56+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	56+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	56+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	56+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	56+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	56+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	56+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	56+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	subl			$ 8, %r10d // k-8
	addq			$ 8*64, %r11 // A+4*bs*sizeof(double)
	addq			%r13, %r12 // B+bs*sdb*sizeof(double)

5:
	cmpl	$ 6, %r14d
	jg		6f

	// offB==6

	addq			$ 48, %r12 // B+3*sizeof(double)

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vbroadcastsd	8+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	8+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1

	subl			$ 2, %r10d // k-1
	addq			$ 2*64, %r11 // A+1*bs*sizeof(double)
	addq			%r13, %r12
	subq			$ 48, %r12 // B+bs*sdb*sizeof(double)-3

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vbroadcastsd	8+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	8+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	8+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	8+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vbroadcastsd	16+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	16+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	16+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	16+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vbroadcastsd	24+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	24+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	24+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	24+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	24+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5

	// unroll 4
	vmovapd			4*64(%r11), %zmm25 // A
	vbroadcastsd	32+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	32+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	32+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	32+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	32+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	32+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6

	// unroll 5
	vmovapd			5*64(%r11), %zmm25 // A
	vbroadcastsd	40+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	40+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	40+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	40+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	40+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	40+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	40+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 6
	vmovapd			6*64(%r11), %zmm25 // A
	vbroadcastsd	48+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	48+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	48+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	48+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	48+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	48+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	48+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 7
	vmovapd			7*64(%r11), %zmm25 // A
	vbroadcastsd	56+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	56+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	56+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	56+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	56+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	56+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	56+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	56+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	subl			$ 8, %r10d // k-8
	addq			$ 8*64, %r11 // A+4*bs*sizeof(double)
	addq			%r13, %r12 // B+bs*sdb*sizeof(double)

6:
//	cmpl	$ 7, %r14d
//	jg		7f

	// offB==7

	addq			$ 56, %r12 // B+3*sizeof(double)

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0

	subl			$ 1, %r10d // k-1
	addq			$ 1*64, %r11 // A+1*bs*sizeof(double)
	addq			%r13, %r12
	subq			$ 56, %r12 // B+bs*sdb*sizeof(double)-3

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vbroadcastsd	0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vbroadcastsd	8+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	8+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	8+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vbroadcastsd	16+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	16+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	16+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vbroadcastsd	24+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	24+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	24+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	24+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4

	// unroll 4
	vmovapd			4*64(%r11), %zmm25 // A
	vbroadcastsd	32+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	32+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	32+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	32+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	32+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5

	// unroll 5
	vmovapd			5*64(%r11), %zmm25 // A
	vbroadcastsd	40+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	40+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	40+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	40+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	40+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	40+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6

	// unroll 6
	vmovapd			6*64(%r11), %zmm25 // A
	vbroadcastsd	48+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	48+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	48+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	48+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	48+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	48+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	48+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// unroll 7
	vmovapd			7*64(%r11), %zmm25 // A
	vbroadcastsd	56+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	56+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	56+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	56+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	56+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	56+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	56+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	56+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	subl			$ 8, %r10d // k-8
	addq			$ 8*64, %r11 // A+4*bs*sizeof(double)
	addq			%r13, %r12 // B+bs*sdb*sizeof(double)

7:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nn_rl_8x8_lib8)
#endif
#endif // TODO keep as reference !!!!!!!!!!!!!!





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
// zmm0  <- []
// ...
// zmm7  <- []

//
// output arguments:
// r10   <- k-(4-offB)
// r11   <- A+(4-offB)*bs*sizeof(double)
// r12   <- B-offB+bs*sdb*sizeof(double)
// r13   <- bs*sdb*sizeof(double)
// r14   <- offB
// zmm0  <- []
// ...
// zmm7  <- []


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRMM_NN_RL_8X8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrmm_nn_rl_8x8_lib8)
#endif
	
	cmpl	$ 0, %r10d
	jle		0f // end

	// offB==xxx

	// TODO saturate offB to 8 ???

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vbroadcastsd	0*64(%r12, %r14, 8), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0

	subl	$ 1, %r10d // k-1
	cmpl	$ 0, %r10d
	jle		0f // end
	addq	$ 64, %r11 // A+1*bs*sizeof(double)
	addl	$ 1, %r14d
	cmpl	$ 7, %r14d
	jle		1f
	addq	%r13, %r12
	movl	$ 0, %r14d
1:

	// unroll 1
	vmovapd			0*64(%r11), %zmm25 // A
	vbroadcastsd	0+0*64(%r12, %r14, 8), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	0+1*64(%r12, %r14, 8), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1

	subl	$ 1, %r10d // k-1
	cmpl	$ 0, %r10d
	jle		0f // end
	addq	$ 64, %r11 // A+1*bs*sizeof(double)
	addl	$ 1, %r14d
	cmpl	$ 7, %r14d
	jle		1f
	addq	%r13, %r12
	movl	$ 0, %r14d
1:

	// unroll 2
	vmovapd			0*64(%r11), %zmm25 // A
	vbroadcastsd	0+0*64(%r12, %r14, 8), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	0+1*64(%r12, %r14, 8), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	0+2*64(%r12, %r14, 8), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2

	subl	$ 1, %r10d // k-1
	cmpl	$ 0, %r10d
	jle		0f // end
	addq	$ 64, %r11 // A+1*bs*sizeof(double)
	addl	$ 1, %r14d
	cmpl	$ 7, %r14d
	jle		1f
	addq	%r13, %r12
	movl	$ 0, %r14d
1:

	// unroll 3
	vmovapd			0*64(%r11), %zmm25 // A
	vbroadcastsd	0+0*64(%r12, %r14, 8), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	0+1*64(%r12, %r14, 8), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	0+2*64(%r12, %r14, 8), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	0+3*64(%r12, %r14, 8), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3

	subl	$ 1, %r10d // k-1
	cmpl	$ 0, %r10d
	jle		0f // end
	addq	$ 64, %r11 // A+1*bs*sizeof(double)
	addl	$ 1, %r14d
	cmpl	$ 7, %r14d
	jle		1f
	addq	%r13, %r12
	movl	$ 0, %r14d
1:

	// unroll 4
	vmovapd			0*64(%r11), %zmm25 // A
	vbroadcastsd	0+0*64(%r12, %r14, 8), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	0+1*64(%r12, %r14, 8), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	0+2*64(%r12, %r14, 8), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	0+3*64(%r12, %r14, 8), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	0+4*64(%r12, %r14, 8), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4

	subl	$ 1, %r10d // k-1
	cmpl	$ 0, %r10d
	jle		0f // end
	addq	$ 64, %r11 // A+1*bs*sizeof(double)
	addl	$ 1, %r14d
	cmpl	$ 7, %r14d
	jle		1f
	addq	%r13, %r12
	movl	$ 0, %r14d
1:

	// unroll 5
	vmovapd			0*64(%r11), %zmm25 // A
	vbroadcastsd	0+0*64(%r12, %r14, 8), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	0+1*64(%r12, %r14, 8), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	0+2*64(%r12, %r14, 8), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	0+3*64(%r12, %r14, 8), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	0+4*64(%r12, %r14, 8), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	0+5*64(%r12, %r14, 8), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5

	subl	$ 1, %r10d // k-1
	cmpl	$ 0, %r10d
	jle		0f // end
	addq	$ 64, %r11 // A+1*bs*sizeof(double)
	addl	$ 1, %r14d
	cmpl	$ 7, %r14d
	jle		1f
	addq	%r13, %r12
	movl	$ 0, %r14d
1:

	// unroll 6
	vmovapd			0*64(%r11), %zmm25 // A
	vbroadcastsd	0+0*64(%r12, %r14, 8), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	0+1*64(%r12, %r14, 8), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	0+2*64(%r12, %r14, 8), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	0+3*64(%r12, %r14, 8), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	0+4*64(%r12, %r14, 8), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	0+5*64(%r12, %r14, 8), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	0+6*64(%r12, %r14, 8), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6

	subl	$ 1, %r10d // k-1
	cmpl	$ 0, %r10d
	jle		0f // end
	addq	$ 64, %r11 // A+1*bs*sizeof(double)
	addl	$ 1, %r14d
	cmpl	$ 7, %r14d
	jle		1f
	addq	%r13, %r12
	movl	$ 0, %r14d
1:

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrmm_nn_rl_8x8_lib8)
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
// zmm0 <- []
// ...
// zmm7 <- []
//
// output arguments:
// r10  <- E
// r11  <- inv_diag_E
// zmm0 <- []
// ...
// zmm7 <- []

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RLT_INV_8X8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rlt_inv_8x8_lib8)
#endif
	
	vbroadcastsd	0(%r11), %zmm24
	vmulpd			%zmm0, %zmm24, %zmm0
	vbroadcastsd	8+0*64(%r10), %zmm24
	vfnmadd231pd	%zmm0, %zmm24, %zmm1
	vbroadcastsd	16+0*64(%r10), %zmm24
	vfnmadd231pd	%zmm0, %zmm24, %zmm2
	vbroadcastsd	24+0*64(%r10), %zmm24
	vfnmadd231pd	%zmm0, %zmm24, %zmm3
	vbroadcastsd	32+0*64(%r10), %zmm24
	vfnmadd231pd	%zmm0, %zmm24, %zmm4
	vbroadcastsd	40+0*64(%r10), %zmm24
	vfnmadd231pd	%zmm0, %zmm24, %zmm5
	vbroadcastsd	48+0*64(%r10), %zmm24
	vfnmadd231pd	%zmm0, %zmm24, %zmm6
	vbroadcastsd	56+0*64(%r10), %zmm24
	vfnmadd231pd	%zmm0, %zmm24, %zmm7

	vbroadcastsd	8(%r11), %zmm24
	vmulpd			%zmm1, %zmm24, %zmm1
	vbroadcastsd	16+1*64(%r10), %zmm24
	vfnmadd231pd	%zmm1, %zmm24, %zmm2
	vbroadcastsd	24+1*64(%r10), %zmm24
	vfnmadd231pd	%zmm1, %zmm24, %zmm3
	vbroadcastsd	32+1*64(%r10), %zmm24
	vfnmadd231pd	%zmm1, %zmm24, %zmm4
	vbroadcastsd	40+1*64(%r10), %zmm24
	vfnmadd231pd	%zmm1, %zmm24, %zmm5
	vbroadcastsd	48+1*64(%r10), %zmm24
	vfnmadd231pd	%zmm1, %zmm24, %zmm6
	vbroadcastsd	56+1*64(%r10), %zmm24
	vfnmadd231pd	%zmm1, %zmm24, %zmm7

	vbroadcastsd	16(%r11), %zmm24
	vmulpd			%zmm2, %zmm24, %zmm2
	vbroadcastsd	24+2*64(%r10), %zmm24
	vfnmadd231pd	%zmm2, %zmm24, %zmm3
	vbroadcastsd	32+2*64(%r10), %zmm24
	vfnmadd231pd	%zmm2, %zmm24, %zmm4
	vbroadcastsd	40+2*64(%r10), %zmm24
	vfnmadd231pd	%zmm2, %zmm24, %zmm5
	vbroadcastsd	48+2*64(%r10), %zmm24
	vfnmadd231pd	%zmm2, %zmm24, %zmm6
	vbroadcastsd	56+2*64(%r10), %zmm24
	vfnmadd231pd	%zmm2, %zmm24, %zmm7

	vbroadcastsd	24(%r11), %zmm24
	vmulpd			%zmm3, %zmm24, %zmm3
	vbroadcastsd	32+3*64(%r10), %zmm24
	vfnmadd231pd	%zmm3, %zmm24, %zmm4
	vbroadcastsd	40+3*64(%r10), %zmm24
	vfnmadd231pd	%zmm3, %zmm24, %zmm5
	vbroadcastsd	48+3*64(%r10), %zmm24
	vfnmadd231pd	%zmm3, %zmm24, %zmm6
	vbroadcastsd	56+3*64(%r10), %zmm24
	vfnmadd231pd	%zmm3, %zmm24, %zmm7

	vbroadcastsd	32(%r11), %zmm24
	vmulpd			%zmm4, %zmm24, %zmm4
	vbroadcastsd	40+4*64(%r10), %zmm24
	vfnmadd231pd	%zmm4, %zmm24, %zmm5
	vbroadcastsd	48+4*64(%r10), %zmm24
	vfnmadd231pd	%zmm4, %zmm24, %zmm6
	vbroadcastsd	56+4*64(%r10), %zmm24
	vfnmadd231pd	%zmm4, %zmm24, %zmm7

	vbroadcastsd	40(%r11), %zmm24
	vmulpd			%zmm5, %zmm24, %zmm5
	vbroadcastsd	48+5*64(%r10), %zmm24
	vfnmadd231pd	%zmm5, %zmm24, %zmm6
	vbroadcastsd	56+5*64(%r10), %zmm24
	vfnmadd231pd	%zmm5, %zmm24, %zmm7

	vbroadcastsd	48(%r11), %zmm24
	vmulpd			%zmm6, %zmm24, %zmm6
	vbroadcastsd	56+6*64(%r10), %zmm24
	vfnmadd231pd	%zmm6, %zmm24, %zmm7

	vbroadcastsd	56(%r11), %zmm24
	vmulpd			%zmm7, %zmm24, %zmm7

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rlt_inv_8x8_lib8)
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
// r12d <- n1
// zmm0 <- []
// ...
// zmm7 <- []
//
// output arguments:
// r10  <- D
// r11  <- inv_diag_D
// r12d <- n1
// zmm0 <- []
// ...
// zmm7 <- []

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RLT_INV_8X8_VS_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rlt_inv_8x8_vs_lib8)
#endif
	
	vbroadcastsd	0(%r11), %zmm24
	vmulpd			%zmm0, %zmm24, %zmm0

	cmpl			$ 2, %r12d
	jl				0f // ret

	vbroadcastsd	8+0*64(%r10), %zmm24
	vfnmadd231pd	%zmm0, %zmm24, %zmm1
	vbroadcastsd	8(%r11), %zmm24
	vmulpd			%zmm1, %zmm24, %zmm1

	cmpl			$ 3, %r12d
	jl				0f // ret

	vbroadcastsd	16+0*64(%r10), %zmm24
	vfnmadd231pd	%zmm0, %zmm24, %zmm2
	vbroadcastsd	16+1*64(%r10), %zmm24
	vfnmadd231pd	%zmm1, %zmm24, %zmm2
	vbroadcastsd	16(%r11), %zmm24
	vmulpd			%zmm2, %zmm24, %zmm2

	cmpl			$ 4, %r12d
	jl				0f // ret

	vbroadcastsd	24+0*64(%r10), %zmm24
	vfnmadd231pd	%zmm0, %zmm24, %zmm3
	vbroadcastsd	24+1*64(%r10), %zmm24
	vfnmadd231pd	%zmm1, %zmm24, %zmm3
	vbroadcastsd	24+2*64(%r10), %zmm24
	vfnmadd231pd	%zmm2, %zmm24, %zmm3
	vbroadcastsd	24(%r11), %zmm24
	vmulpd			%zmm3, %zmm24, %zmm3

	cmpl			$ 5, %r12d
	jl				0f // ret

	vbroadcastsd	32+0*64(%r10), %zmm24
	vfnmadd231pd	%zmm0, %zmm24, %zmm4
	vbroadcastsd	32+1*64(%r10), %zmm24
	vfnmadd231pd	%zmm1, %zmm24, %zmm4
	vbroadcastsd	32+2*64(%r10), %zmm24
	vfnmadd231pd	%zmm2, %zmm24, %zmm4
	vbroadcastsd	32+3*64(%r10), %zmm24
	vfnmadd231pd	%zmm3, %zmm24, %zmm4
	vbroadcastsd	32(%r11), %zmm24
	vmulpd			%zmm4, %zmm24, %zmm4

	cmpl			$ 6, %r12d
	jl				0f // ret

	vbroadcastsd	40+0*64(%r10), %zmm24
	vfnmadd231pd	%zmm0, %zmm24, %zmm5
	vbroadcastsd	40+1*64(%r10), %zmm24
	vfnmadd231pd	%zmm1, %zmm24, %zmm5
	vbroadcastsd	40+2*64(%r10), %zmm24
	vfnmadd231pd	%zmm2, %zmm24, %zmm5
	vbroadcastsd	40+3*64(%r10), %zmm24
	vfnmadd231pd	%zmm3, %zmm24, %zmm5
	vbroadcastsd	40+4*64(%r10), %zmm24
	vfnmadd231pd	%zmm4, %zmm24, %zmm5
	vbroadcastsd	40(%r11), %zmm24
	vmulpd			%zmm5, %zmm24, %zmm5

	cmpl			$ 7, %r12d
	jl				0f // ret

	vbroadcastsd	48+0*64(%r10), %zmm24
	vfnmadd231pd	%zmm0, %zmm24, %zmm6
	vbroadcastsd	48+1*64(%r10), %zmm24
	vfnmadd231pd	%zmm1, %zmm24, %zmm6
	vbroadcastsd	48+2*64(%r10), %zmm24
	vfnmadd231pd	%zmm2, %zmm24, %zmm6
	vbroadcastsd	48+3*64(%r10), %zmm24
	vfnmadd231pd	%zmm3, %zmm24, %zmm6
	vbroadcastsd	48+4*64(%r10), %zmm24
	vfnmadd231pd	%zmm4, %zmm24, %zmm6
	vbroadcastsd	48+5*64(%r10), %zmm24
	vfnmadd231pd	%zmm5, %zmm24, %zmm6
	vbroadcastsd	48(%r11), %zmm24
	vmulpd			%zmm6, %zmm24, %zmm6

	cmpl			$ 8, %r12d
	jl				0f // ret

	vbroadcastsd	56+0*64(%r10), %zmm24
	vfnmadd231pd	%zmm0, %zmm24, %zmm7
	vbroadcastsd	56+1*64(%r10), %zmm24
	vfnmadd231pd	%zmm1, %zmm24, %zmm7
	vbroadcastsd	56+2*64(%r10), %zmm24
	vfnmadd231pd	%zmm2, %zmm24, %zmm7
	vbroadcastsd	56+3*64(%r10), %zmm24
	vfnmadd231pd	%zmm3, %zmm24, %zmm7
	vbroadcastsd	56+4*64(%r10), %zmm24
	vfnmadd231pd	%zmm4, %zmm24, %zmm7
	vbroadcastsd	56+5*64(%r10), %zmm24
	vfnmadd231pd	%zmm5, %zmm24, %zmm7
	vbroadcastsd	56+6*64(%r10), %zmm24
	vfnmadd231pd	%zmm6, %zmm24, %zmm7
	vbroadcastsd	56(%r11), %zmm24
	vmulpd			%zmm7, %zmm24, %zmm7

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rlt_inv_8x8_vs_lib8)
#endif





// common inner routine with file scope
//
// cholesky factorization 
//
// input arguments:
// r10  <- inv_diag_E
// zmm0 <- []
// ...
// ymm7 <- []
//
// output arguments:
// r10  <- inv_diag_E
// zmm0 <- []
// ...
// ymm7 <- []

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DPOTRF_8X8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_dpotrf_8x8_lib8)
#endif

	vxorpd	%ymm25, %ymm25, %ymm25 // 0.0
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd	.LC04(%rip), %xmm24 // 1.0
#elif defined(OS_MAC)
	vmovsd	LC04(%rip), %xmm24 // 1.0
#endif

	// 0
	vmovsd			%xmm0, %xmm0, %xmm26
	vucomisd		%xmm25, %xmm26 // d_00 > 0.0 ?
	jbe				1f
	vsqrtsd			%xmm26, %xmm26, %xmm26
	vdivsd			%xmm26, %xmm24, %xmm26
2:
	vmovsd			%xmm26, 0(%r10)
	vbroadcastsd	%xmm26, %zmm26
	vmulpd			%zmm0, %zmm26, %zmm0
	vmovapd			%zmm1, %zmm26
	vfnmadd231pd	%zmm0, %zmm0, %zmm26

	// 1
//	vpermilpd		$ 0x3, %xmm1, %xmm26
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd			%zmm26, %zmm27, %zmm26
	vucomisd		%xmm25, %xmm26 // d_11 > 0.0 ?
	jbe				3f
	vsqrtsd			%xmm26, %xmm26, %xmm26
	vdivsd			%xmm26, %xmm24, %xmm26
4:
#if defined(OS_LINUX) | defined(OS_WINDOWS)
//	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
//	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm27
	vfnmadd231pd	%zmm0, %zmm27, %zmm1
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm27
	vfnmadd231pd	%zmm0, %zmm27, %zmm2
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm27
	vfnmadd231pd	%zmm0, %zmm27, %zmm3
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm27
	vfnmadd231pd	%zmm0, %zmm27, %zmm4
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm27
	vfnmadd231pd	%zmm0, %zmm27, %zmm5
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm27
	vfnmadd231pd	%zmm0, %zmm27, %zmm6
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm27
	vfnmadd231pd	%zmm0, %zmm27, %zmm7
	vmovsd			%xmm26, 8(%r10)
	vbroadcastsd	%xmm26, %zmm26
	vmulpd			%zmm1, %zmm26, %zmm1
	vmovapd			%zmm2, %zmm26
	vfnmadd231pd	%zmm1, %zmm1, %zmm26

	// 2
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd			%zmm26, %zmm27, %zmm26
	vucomisd		%xmm25, %xmm26 // d_22 > 0.0 ?
	jbe				5f
	vsqrtsd			%xmm26, %xmm26, %xmm26
	vdivsd			%xmm26, %xmm24, %xmm26
6:
#if defined(OS_LINUX) | defined(OS_WINDOWS)
//	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
//	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd			%zmm1, %zmm27, %zmm27
	vfnmadd231pd	%zmm1, %zmm27, %zmm2
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd			%zmm1, %zmm27, %zmm27
	vfnmadd231pd	%zmm1, %zmm27, %zmm3
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd			%zmm1, %zmm27, %zmm27
	vfnmadd231pd	%zmm1, %zmm27, %zmm4
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd			%zmm1, %zmm27, %zmm27
	vfnmadd231pd	%zmm1, %zmm27, %zmm5
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd			%zmm1, %zmm27, %zmm27
	vfnmadd231pd	%zmm1, %zmm27, %zmm6
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd			%zmm1, %zmm27, %zmm27
	vfnmadd231pd	%zmm1, %zmm27, %zmm7
	vmovsd			%xmm26, 16(%r10)
	vbroadcastsd	%xmm26, %zmm26
	vmulpd			%zmm2, %zmm26, %zmm2
	vmovapd			%zmm3, %zmm26
	vfnmadd231pd	%zmm2, %zmm2, %zmm26

	// 3
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd			%zmm26, %zmm27, %zmm26
	vucomisd		%xmm25, %xmm26 // d_22 > 0.0 ?
	jbe				7f
	vsqrtsd			%xmm26, %xmm26, %xmm26
	vdivsd			%xmm26, %xmm24, %xmm26
8:
#if defined(OS_LINUX) | defined(OS_WINDOWS)
//	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
//	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd			%zmm2, %zmm27, %zmm27
	vfnmadd231pd	%zmm2, %zmm27, %zmm3
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd			%zmm2, %zmm27, %zmm27
	vfnmadd231pd	%zmm2, %zmm27, %zmm4
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd			%zmm2, %zmm27, %zmm27
	vfnmadd231pd	%zmm2, %zmm27, %zmm5
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd			%zmm2, %zmm27, %zmm27
	vfnmadd231pd	%zmm2, %zmm27, %zmm6
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd			%zmm2, %zmm27, %zmm27
	vfnmadd231pd	%zmm2, %zmm27, %zmm7
	vmovsd			%xmm26, 24(%r10)
	vbroadcastsd	%xmm26, %zmm26
	vmulpd			%zmm3, %zmm26, %zmm3
	vmovapd			%zmm4, %zmm26
	vfnmadd231pd	%zmm3, %zmm3, %zmm26

	// 4
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd			%zmm26, %zmm27, %zmm26
	vucomisd		%xmm25, %xmm26 // d_22 > 0.0 ?
	jbe				9f
	vsqrtsd			%xmm26, %xmm26, %xmm26
	vdivsd			%xmm26, %xmm24, %xmm26
10:
#if defined(OS_LINUX) | defined(OS_WINDOWS)
//	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
//	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd			%zmm3, %zmm27, %zmm27
	vfnmadd231pd	%zmm3, %zmm27, %zmm4
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd			%zmm3, %zmm27, %zmm27
	vfnmadd231pd	%zmm3, %zmm27, %zmm5
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd			%zmm3, %zmm27, %zmm27
	vfnmadd231pd	%zmm3, %zmm27, %zmm6
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd			%zmm3, %zmm27, %zmm27
	vfnmadd231pd	%zmm3, %zmm27, %zmm7
	vmovsd			%xmm26, 32(%r10)
	vbroadcastsd	%xmm26, %zmm26
	vmulpd			%zmm4, %zmm26, %zmm4
	vmovapd			%zmm5, %zmm26
	vfnmadd231pd	%zmm4, %zmm4, %zmm26

	// 5
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd			%zmm26, %zmm27, %zmm26
	vucomisd		%xmm25, %xmm26 // d_22 > 0.0 ?
	jbe				11f
	vsqrtsd			%xmm26, %xmm26, %xmm26
	vdivsd			%xmm26, %xmm24, %xmm26
12:
#if defined(OS_LINUX) | defined(OS_WINDOWS)
//	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
//	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd			%zmm4, %zmm27, %zmm27
	vfnmadd231pd	%zmm4, %zmm27, %zmm5
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd			%zmm4, %zmm27, %zmm27
	vfnmadd231pd	%zmm4, %zmm27, %zmm6
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd			%zmm4, %zmm27, %zmm27
	vfnmadd231pd	%zmm4, %zmm27, %zmm7
	vmovsd			%xmm26, 40(%r10)
	vbroadcastsd	%xmm26, %zmm26
	vmulpd			%zmm5, %zmm26, %zmm5
	vmovapd			%zmm6, %zmm26
	vfnmadd231pd	%zmm5, %zmm5, %zmm26

	// 6
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd			%zmm26, %zmm27, %zmm26
	vucomisd		%xmm25, %xmm26 // d_22 > 0.0 ?
	jbe				13f
	vsqrtsd			%xmm26, %xmm26, %xmm26
	vdivsd			%xmm26, %xmm24, %xmm26
14:
#if defined(OS_LINUX) | defined(OS_WINDOWS)
//	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
//	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd			%zmm5, %zmm27, %zmm27
	vfnmadd231pd	%zmm5, %zmm27, %zmm6
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd			%zmm5, %zmm27, %zmm27
	vfnmadd231pd	%zmm5, %zmm27, %zmm7
	vmovsd			%xmm26, 48(%r10)
	vbroadcastsd	%xmm26, %zmm26
	vmulpd			%zmm6, %zmm26, %zmm6
	vmovapd			%zmm7, %zmm26
	vfnmadd231pd	%zmm6, %zmm6, %zmm26

	// 7
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd			%zmm26, %zmm27, %zmm26
	vucomisd		%xmm25, %xmm26 // d_22 > 0.0 ?
	jbe				15f
	vsqrtsd			%xmm26, %xmm26, %xmm26
	vdivsd			%xmm26, %xmm24, %xmm26
16:
#if defined(OS_LINUX) | defined(OS_WINDOWS)
//	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
//	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd			%zmm6, %zmm27, %zmm27
	vfnmadd231pd	%zmm6, %zmm27, %zmm7
	vmovsd			%xmm26, 56(%r10)
	vbroadcastsd	%xmm26, %zmm26
	vmulpd			%zmm7, %zmm26, %zmm7

	jmp				0f

1:
	vxorpd			%zmm26, %zmm26, %zmm26
	jmp				2b

3:
	vxorpd			%zmm26, %zmm26, %zmm26
	jmp				4b

5:
	vxorpd			%zmm26, %zmm26, %zmm26
	jmp				6b

7:
	vxorpd			%zmm26, %zmm26, %zmm26
	jmp				8b

9:
	vxorpd			%zmm26, %zmm26, %zmm26
	jmp				10b

11:
	vxorpd			%zmm26, %zmm26, %zmm26
	jmp				12b

13:
	vxorpd			%zmm26, %zmm26, %zmm26
	jmp				14b

15:
	vxorpd			%zmm26, %zmm26, %zmm26
	jmp				16b

0:
	#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dpotrf_8x8_lib8)
#endif





// common inner routine with file scope
//
// cholesky factorization 
//
// input arguments:
// r10  <- inv_diag_E
// r11d <- m1
// r12d <- n1
// zmm0 <- []
// ...
// ymm7 <- []
//
// output arguments:
// r10  <- inv_diag_E
// r11d <- m1
// r12d <- n1
// zmm0 <- []
// ...
// ymm7 <- []

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DPOTRF_8X8_VS_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_dpotrf_8x8_vs_lib8)
#endif

	// compute mask for rows
	vcvtsi2sd	%r11d, %xmm25, %xmm25
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC00(%rip), %zmm24
#elif defined(OS_MAC)
	vmovupd		LC00(%rip), %zmm24
#endif
	vbroadcastsd	%xmm25, %zmm25
	vsubpd		%zmm25, %zmm24, %zmm25
	vpmovq2m	%zmm25, %k1

	vmovapd		%zmm0, %zmm0 {%k1}{z}
	vmovapd		%zmm1, %zmm1 {%k1}{z}
	vmovapd		%zmm2, %zmm2 {%k1}{z}
	vmovapd		%zmm3, %zmm3 {%k1}{z}
	vmovapd		%zmm4, %zmm4 {%k1}{z}
	vmovapd		%zmm5, %zmm5 {%k1}{z}
	vmovapd		%zmm6, %zmm6 {%k1}{z}
	vmovapd		%zmm7, %zmm7 {%k1}{z}

	vxorpd	%ymm25, %ymm25, %ymm25 // 0.0
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd	.LC04(%rip), %xmm24 // 1.0
#elif defined(OS_MAC)
	vmovsd	LC04(%rip), %xmm24 // 1.0
#endif

	// 0
	vmovsd			%xmm0, %xmm0, %xmm26
	vucomisd		%xmm25, %xmm26 // d_00 > 0.0 ?
	jbe				1f
	vsqrtsd			%xmm26, %xmm26, %xmm26
	vdivsd			%xmm26, %xmm24, %xmm26
2:
	vmovsd			%xmm26, 0(%r10)
	vbroadcastsd	%xmm26, %zmm26
	vmulpd			%zmm0, %zmm26, %zmm0
	cmpl			$ 2, %r12d
	jl				0f // ret
	vmovapd			%zmm1, %zmm26
	vfnmadd231pd	%zmm0, %zmm0, %zmm26

	// 1
//	vpermilpd		$ 0x3, %xmm1, %xmm26
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd			%zmm26, %zmm27, %zmm26
	vucomisd		%xmm25, %xmm26 // d_11 > 0.0 ?
	jbe				3f
	vsqrtsd			%xmm26, %xmm26, %xmm26
	vdivsd			%xmm26, %xmm24, %xmm26
4:
#if defined(OS_LINUX) | defined(OS_WINDOWS)
//	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
//	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm27
	vfnmadd231pd	%zmm0, %zmm27, %zmm1
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm27
	vfnmadd231pd	%zmm0, %zmm27, %zmm2
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm27
	vfnmadd231pd	%zmm0, %zmm27, %zmm3
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm27
	vfnmadd231pd	%zmm0, %zmm27, %zmm4
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm27
	vfnmadd231pd	%zmm0, %zmm27, %zmm5
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm27
	vfnmadd231pd	%zmm0, %zmm27, %zmm6
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm27
	vfnmadd231pd	%zmm0, %zmm27, %zmm7
	vmovsd			%xmm26, 8(%r10)
	vbroadcastsd	%xmm26, %zmm26
	vmulpd			%zmm1, %zmm26, %zmm1
	cmpl			$ 3, %r12d
	jl				0f // ret
	vmovapd			%zmm2, %zmm26
	vfnmadd231pd	%zmm1, %zmm1, %zmm26

	// 2
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd			%zmm26, %zmm27, %zmm26
	vucomisd		%xmm25, %xmm26 // d_22 > 0.0 ?
	jbe				5f
	vsqrtsd			%xmm26, %xmm26, %xmm26
	vdivsd			%xmm26, %xmm24, %xmm26
6:
#if defined(OS_LINUX) | defined(OS_WINDOWS)
//	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
//	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd			%zmm1, %zmm27, %zmm27
	vfnmadd231pd	%zmm1, %zmm27, %zmm2
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd			%zmm1, %zmm27, %zmm27
	vfnmadd231pd	%zmm1, %zmm27, %zmm3
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd			%zmm1, %zmm27, %zmm27
	vfnmadd231pd	%zmm1, %zmm27, %zmm4
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd			%zmm1, %zmm27, %zmm27
	vfnmadd231pd	%zmm1, %zmm27, %zmm5
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd			%zmm1, %zmm27, %zmm27
	vfnmadd231pd	%zmm1, %zmm27, %zmm6
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd			%zmm1, %zmm27, %zmm27
	vfnmadd231pd	%zmm1, %zmm27, %zmm7
	vmovsd			%xmm26, 16(%r10)
	vbroadcastsd	%xmm26, %zmm26
	vmulpd			%zmm2, %zmm26, %zmm2
	cmpl			$ 4, %r12d
	jl				0f // ret
	vmovapd			%zmm3, %zmm26
	vfnmadd231pd	%zmm2, %zmm2, %zmm26

	// 3
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd			%zmm26, %zmm27, %zmm26
	vucomisd		%xmm25, %xmm26 // d_22 > 0.0 ?
	jbe				7f
	vsqrtsd			%xmm26, %xmm26, %xmm26
	vdivsd			%xmm26, %xmm24, %xmm26
8:
#if defined(OS_LINUX) | defined(OS_WINDOWS)
//	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
//	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd			%zmm2, %zmm27, %zmm27
	vfnmadd231pd	%zmm2, %zmm27, %zmm3
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd			%zmm2, %zmm27, %zmm27
	vfnmadd231pd	%zmm2, %zmm27, %zmm4
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd			%zmm2, %zmm27, %zmm27
	vfnmadd231pd	%zmm2, %zmm27, %zmm5
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd			%zmm2, %zmm27, %zmm27
	vfnmadd231pd	%zmm2, %zmm27, %zmm6
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd			%zmm2, %zmm27, %zmm27
	vfnmadd231pd	%zmm2, %zmm27, %zmm7
	vmovsd			%xmm26, 24(%r10)
	vbroadcastsd	%xmm26, %zmm26
	vmulpd			%zmm3, %zmm26, %zmm3
	cmpl			$ 5, %r12d
	jl				0f // ret
	vmovapd			%zmm4, %zmm26
	vfnmadd231pd	%zmm3, %zmm3, %zmm26

	// 4
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd			%zmm26, %zmm27, %zmm26
	vucomisd		%xmm25, %xmm26 // d_22 > 0.0 ?
	jbe				9f
	vsqrtsd			%xmm26, %xmm26, %xmm26
	vdivsd			%xmm26, %xmm24, %xmm26
10:
#if defined(OS_LINUX) | defined(OS_WINDOWS)
//	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
//	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd			%zmm3, %zmm27, %zmm27
	vfnmadd231pd	%zmm3, %zmm27, %zmm4
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd			%zmm3, %zmm27, %zmm27
	vfnmadd231pd	%zmm3, %zmm27, %zmm5
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd			%zmm3, %zmm27, %zmm27
	vfnmadd231pd	%zmm3, %zmm27, %zmm6
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd			%zmm3, %zmm27, %zmm27
	vfnmadd231pd	%zmm3, %zmm27, %zmm7
	vmovsd			%xmm26, 32(%r10)
	vbroadcastsd	%xmm26, %zmm26
	vmulpd			%zmm4, %zmm26, %zmm4
	cmpl			$ 6, %r12d
	jl				0f // ret
	vmovapd			%zmm5, %zmm26
	vfnmadd231pd	%zmm4, %zmm4, %zmm26

	// 5
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd			%zmm26, %zmm27, %zmm26
	vucomisd		%xmm25, %xmm26 // d_22 > 0.0 ?
	jbe				11f
	vsqrtsd			%xmm26, %xmm26, %xmm26
	vdivsd			%xmm26, %xmm24, %xmm26
12:
#if defined(OS_LINUX) | defined(OS_WINDOWS)
//	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
//	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd			%zmm4, %zmm27, %zmm27
	vfnmadd231pd	%zmm4, %zmm27, %zmm5
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd			%zmm4, %zmm27, %zmm27
	vfnmadd231pd	%zmm4, %zmm27, %zmm6
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd			%zmm4, %zmm27, %zmm27
	vfnmadd231pd	%zmm4, %zmm27, %zmm7
	vmovsd			%xmm26, 40(%r10)
	vbroadcastsd	%xmm26, %zmm26
	vmulpd			%zmm5, %zmm26, %zmm5
	cmpl			$ 7, %r12d
	jl				0f // ret
	vmovapd			%zmm6, %zmm26
	vfnmadd231pd	%zmm5, %zmm5, %zmm26

	// 6
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd			%zmm26, %zmm27, %zmm26
	vucomisd		%xmm25, %xmm26 // d_22 > 0.0 ?
	jbe				13f
	vsqrtsd			%xmm26, %xmm26, %xmm26
	vdivsd			%xmm26, %xmm24, %xmm26
14:
#if defined(OS_LINUX) | defined(OS_WINDOWS)
//	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
//	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd			%zmm5, %zmm27, %zmm27
	vfnmadd231pd	%zmm5, %zmm27, %zmm6
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd			%zmm5, %zmm27, %zmm27
	vfnmadd231pd	%zmm5, %zmm27, %zmm7
	vmovsd			%xmm26, 48(%r10)
	vbroadcastsd	%xmm26, %zmm26
	vmulpd			%zmm6, %zmm26, %zmm6
	cmpl			$ 8, %r12d
	jl				0f // ret
	vmovapd			%zmm7, %zmm26
	vfnmadd231pd	%zmm6, %zmm6, %zmm26

	// 7
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd			%zmm26, %zmm27, %zmm26
	vucomisd		%xmm25, %xmm26 // d_22 > 0.0 ?
	jbe				15f
	vsqrtsd			%xmm26, %xmm26, %xmm26
	vdivsd			%xmm26, %xmm24, %xmm26
16:
#if defined(OS_LINUX) | defined(OS_WINDOWS)
//	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
//	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd			%zmm6, %zmm27, %zmm27
	vfnmadd231pd	%zmm6, %zmm27, %zmm7
	vmovsd			%xmm26, 56(%r10)
	vbroadcastsd	%xmm26, %zmm26
	vmulpd			%zmm7, %zmm26, %zmm7

	jmp				0f

1:
	vxorpd			%zmm26, %zmm26, %zmm26
	jmp				2b

3:
	vxorpd			%zmm26, %zmm26, %zmm26
	jmp				4b

5:
	vxorpd			%zmm26, %zmm26, %zmm26
	jmp				6b

7:
	vxorpd			%zmm26, %zmm26, %zmm26
	jmp				8b

9:
	vxorpd			%zmm26, %zmm26, %zmm26
	jmp				10b

11:
	vxorpd			%zmm26, %zmm26, %zmm26
	jmp				12b

13:
	vxorpd			%zmm26, %zmm26, %zmm26
	jmp				14b

15:
	vxorpd			%zmm26, %zmm26, %zmm26
	jmp				16b

0:
	#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dpotrf_8x8_vs_lib8)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// zmm0 <- []
// ...
// zmm7 <- []
//
// output arguments:
// r10   <- alpha
// r11   <- beta
// r10   <- C
// zmm0 <- []
// ...
// zmm7 <- []

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_8X8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_8x8_lib8)
#endif
	
	// alpha
	vbroadcastsd	0(%r10), %zmm25

	vmulpd		%zmm0, %zmm25, %zmm0
	vmulpd		%zmm1, %zmm25, %zmm1
	vmulpd		%zmm2, %zmm25, %zmm2
	vmulpd		%zmm3, %zmm25, %zmm3
	vmulpd		%zmm4, %zmm25, %zmm4
	vmulpd		%zmm5, %zmm25, %zmm5
	vmulpd		%zmm6, %zmm25, %zmm6
	vmulpd		%zmm7, %zmm25, %zmm7

	// beta
	vbroadcastsd	0(%r11), %zmm24

	vxorpd		%zmm25, %zmm25, %zmm25 // 0.0

	vucomisd	%xmm25, %xmm24 // beta==0.0 ?
	je			0f // end

	vmovapd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm0
	vmovapd		64(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm1
	vmovapd		128(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm2
	vmovapd		192(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm3
	vmovapd		0+256(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm4
	vmovapd		64+256(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm5
	vmovapd		128+256(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm6
	vmovapd		192+256(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm7

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_8x8_lib8)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13d  <- m1
// r14d  <- n1
// zmm0 <- []
// ...
// zmm7 <- []
//
// output arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13d  <- m1
// r14d  <- n1
// zmm0 <- []
// ...
// zmm7 <- []

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_8X8_VS_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_8x8_vs_lib8)
#endif
	
	// alpha
	vbroadcastsd	0(%r10), %zmm25

	vmulpd		%zmm0, %zmm25, %zmm0
	vmulpd		%zmm1, %zmm25, %zmm1
	vmulpd		%zmm2, %zmm25, %zmm2
	vmulpd		%zmm3, %zmm25, %zmm3
	vmulpd		%zmm4, %zmm25, %zmm4
	vmulpd		%zmm5, %zmm25, %zmm5
	vmulpd		%zmm6, %zmm25, %zmm6
	vmulpd		%zmm7, %zmm25, %zmm7

	// beta
	vbroadcastsd	0(%r11), %zmm24

	vxorpd		%zmm25, %zmm25, %zmm25 // 0.0

	vucomisd	%xmm25, %xmm24 // beta==0.0 ?
	je			0f // end

	// compute mask for rows
	vcvtsi2sd	%r13d, %xmm25, %xmm25
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC00(%rip), %zmm26
#elif defined(OS_MAC)
	vmovupd		LC00(%rip), %zmm26
#endif
	vbroadcastsd	%xmm25, %zmm25
	vsubpd		%zmm25, %zmm26, %zmm25
	vpmovq2m	%zmm25, %k1

	vmovapd		0(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm0 {%k1}
	cmpl	$ 2, %r14d
	jl		0f // end
	vmovapd		64(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm1 {%k1}
	cmpl	$ 3, %r14d
	jl		0f // end
	vmovapd		128(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm2 {%k1}
	cmpl	$ 4, %r14d
	jl		0f // end
	vmovapd		192(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm3 {%k1}
	cmpl	$ 5, %r14d
	jl		0f // end
	vmovapd		0+256(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm4 {%k1}
	cmpl	$ 6, %r14d
	jl		0f // end
	vmovapd		64+256(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm5 {%k1}
	cmpl	$ 7, %r14d
	jl		0f // end
	vmovapd		128+256(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm6 {%k1}
	je		0f // end {%k1}
	vmovapd		192+256(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm7 {%k1}

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_8x8_vs_lib8)
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
// r14  <- 8*sdc*sizeof(double)
// r15   <- n0
// rax   <- n1
// zmm0 <- []
// ...
// zmm7 <- []
//
// output arguments:
// r10   <- alpha
// r11   <- beta
// r12  <- offset
// r13   <- C
// r14  <- 8*sdc*sizeof(double)
// r15   <- n0
// rax   <- n1
// zmm0 <- []
// ...
// zmm7 <- []

// TODO m-mask !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_8X8_GEN_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_8x8_gen_lib8)
#endif
	
	// alpha
	vbroadcastsd	0(%r10), %zmm24

	vmulpd		%zmm0, %zmm24, %zmm0
	vmulpd		%zmm1, %zmm24, %zmm1
	vmulpd		%zmm2, %zmm24, %zmm2
	vmulpd		%zmm3, %zmm24, %zmm3
	vmulpd		%zmm4, %zmm24, %zmm4
	vmulpd		%zmm5, %zmm24, %zmm5
	vmulpd		%zmm6, %zmm24, %zmm6
	vmulpd		%zmm7, %zmm24, %zmm7

	// beta
	vbroadcastsd	0(%r11), %zmm24

	vxorpd		%zmm25, %zmm25, %zmm25 // 0.0

	vucomisd	%xmm24, %xmm25 // beta==0.0 ?
	je			8f // end

	cmpl	$ 0, %r12d
	jg		0f

	// offset==0

	cmpl	$ 0, %r15d
	jg		1f // next
	vmovapd		0(%r13), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm0
	jmp		2f
1:
	cmpl	$ 1, %r15d
	jg		1f // next
2:
	cmpl	$ 2, %eax
	jl		8f // end
	vmovapd		64(%r13), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm1
	jmp		2f
1:
	cmpl	$ 2, %r15d
	jg		1f // next
2:
	cmpl	$ 3, %eax
	jl		8f // end
	vmovapd		128(%r13), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm2
	jmp		2f
1:
	cmpl	$ 3, %r15d
	jg		1f // next
2:
	cmpl	$ 4, %eax
	jl		8f // end
	vmovapd		192(%r13), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm3
	jmp		2f
1:
	cmpl	$ 4, %r15d
	jg		1f // next
2:
	cmpl	$ 5, %eax
	jl		8f // end
	vmovapd		0+256(%r13), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm4
	jmp		2f
1:
	cmpl	$ 5, %r15d
	jg		1f // next
2:
	cmpl	$ 6, %eax
	jl		8f // end
	vmovapd		64+256(%r13), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm5
	jmp		2f
1:
	cmpl	$ 6, %r15d
	jg		1f // next
2:
	cmpl	$ 7, %eax
	jl		8f // end
	vmovapd		128+256(%r13), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm6
	jmp		2f
1:
	cmpl	$ 7, %r15d
	jg		8f // end 1f // next
2:
	cmpl	$ 7, %eax
	je		8f // end
	vmovapd		192+256(%r13), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm7
//1:

	jmp		8f

0:

	cmpl	$ 3, %r12d
	jg		14f

	cmpl	$ 1, %r12d
	jg		11f

	jmp		0f

11: // 2 3

	cmpl	$ 2, %r12d
	jg		2f

	jmp		1f

14: // 4 5 6 7

	cmpl	$ 5, %r12d
	jg		16f

	cmpl	$ 4, %r12d
	jg		4f

	jmp		3f

16: // 6 7

	cmpl	$ 6, %r12d
	jg		6f

	jmp		5f

0: 	// offset==1

	movl	$ 0x7f, %ebx
	kmovd	%ebx, %k2
//	kandd	%k2, %k1, %k2
	movl	$ 0x80, %ebx
	kmovd	%ebx, %k3
//	kandd	%k3, %k1, %k3

	leaq	8(%r13), %rbx

	jmp		7f

1:  // offset==2

	movl	$ 0x3f, %ebx
	kmovd	%ebx, %k2
//	kandd	%k2, %k1, %k2
	movl	$ 0xc0, %ebx
	kmovd	%ebx, %k3
//	kandd	%k3, %k1, %k3

	leaq	16(%r13), %rbx

	jmp		7f

2:  // offset==3

	movl	$ 0x1f, %ebx
	kmovd	%ebx, %k2
//	kandd	%k2, %k1, %k2
	movl	$ 0xe0, %ebx
	kmovd	%ebx, %k3
//	kandd	%k3, %k1, %k3

	leaq	24(%r13), %rbx

	jmp		7f

3:  // offset==4

	movl	$ 0x0f, %ebx
	kmovd	%ebx, %k2
//	kandd	%k2, %k1, %k2
	movl	$ 0xf0, %ebx
	kmovd	%ebx, %k3
//	kandd	%k3, %k1, %k3

	leaq	32(%r13), %rbx

	jmp		7f

4:  // offset==5

	movl	$ 0x07, %ebx
	kmovd	%ebx, %k2
//	kandd	%k2, %k1, %k2
	movl	$ 0xf8, %ebx
	kmovd	%ebx, %k3
//	kandd	%k3, %k1, %k3

	leaq	40(%r13), %rbx

	jmp		7f

5:  // offset==6

	movl	$ 0x03, %ebx
	kmovd	%ebx, %k2
//	kandd	%k2, %k1, %k2
	movl	$ 0xfc, %ebx
	kmovd	%ebx, %k3
//	kandd	%k3, %k1, %k3

	leaq	48(%r13), %rbx

	jmp		7f

6:  // offset==7

	movl	$ 0x01, %ebx
	kmovd	%ebx, %k2
//	kandd	%k2, %k1, %k2
	movl	$ 0xfe, %ebx
	kmovd	%ebx, %k3
//	kandd	%k3, %k1, %k3

	leaq	56(%r13), %rbx

//	jmp		7f

7:

	cmpl	$ 0, %r15d
	jg		1f // next
	vmovupd 0(%rbx), %zmm25 {%k2}
	vmovupd 0-64(%rbx, %r14), %zmm25 {%k3}
	vfmadd231pd	%zmm24, %zmm25, %zmm0
	jmp		2f
1:
	cmpl	$ 1, %r15d
	jg		1f // next
2:
	cmpl	$ 2, %eax
	jl		8f // end
	vmovupd 64(%rbx), %zmm25 {%k2}
	vmovupd 64-64(%rbx, %r14), %zmm25 {%k3}
	vfmadd231pd	%zmm24, %zmm25, %zmm1
	jmp		2f
1:
	cmpl	$ 2, %r15d
	jg		1f // next
2:
	cmpl	$ 3, %eax
	jl		8f // end
	vmovupd 128(%rbx), %zmm25 {%k2}
	vmovupd 128-64(%rbx, %r14), %zmm25 {%k3}
	vfmadd231pd	%zmm24, %zmm25, %zmm2
	jmp		2f
1:
	cmpl	$ 3, %r15d
	jg		1f // next
2:
	cmpl	$ 4, %eax
	jl		8f // end
	vmovupd 192(%rbx), %zmm25 {%k2}
	vmovupd 192-64(%rbx, %r14), %zmm25 {%k3}
	vfmadd231pd	%zmm24, %zmm25, %zmm3
	jmp		2f
1:
	cmpl	$ 4, %r15d
	jg		1f // next
2:
	cmpl	$ 5, %eax
	jl		8f // end
	vmovupd 0+256(%rbx), %zmm25 {%k2}
	vmovupd 0+256-64(%rbx, %r14), %zmm25 {%k3}
	vfmadd231pd	%zmm24, %zmm25, %zmm4
	jmp		2f
1:
	cmpl	$ 5, %r15d
	jg		1f // next
2:
	cmpl	$ 6, %eax
	jl		8f // end
	vmovupd 64+256(%rbx), %zmm25 {%k2}
	vmovupd 64+256-64(%rbx, %r14), %zmm25 {%k3}
	vfmadd231pd	%zmm24, %zmm25, %zmm5
	jmp		2f
1:
	cmpl	$ 6, %r15d
	jg		1f // next
2:
	cmpl	$ 7, %eax
	jl		8f // end
	vmovupd 128+256(%rbx), %zmm25 {%k2}
	vmovupd 128+256-64(%rbx, %r14), %zmm25 {%k3}
	vfmadd231pd	%zmm24, %zmm25, %zmm6
	jmp		2f
1:
	cmpl	$ 7, %r15d
	jg		8f // end 1f // next
2:
	cmpl	$ 7, %eax
	je		8f // end
	vmovupd 192+256(%rbx), %zmm25 {%k2}
	vmovupd 192+256-64(%rbx, %r14), %zmm25 {%k3}
	vfmadd231pd	%zmm24, %zmm25, %zmm7
//1:

8:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_8x8_gen_lib8)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta=0
//
// input arguments:
// r10   <- alpha
// zmm0 <- []
// ...
// zmm7 <- []
//
// output arguments:
// r10   <- alpha
// zmm0 <- []
// ...
// zmm7 <- []

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_A0_8X8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_scale_a0_8x8_lib8)
#endif
	
	// alpha
	vbroadcastsd	0(%r10), %zmm25

	vmulpd		%zmm0, %zmm25, %zmm0
	vmulpd		%zmm1, %zmm25, %zmm1
	vmulpd		%zmm2, %zmm25, %zmm2
	vmulpd		%zmm3, %zmm25, %zmm3
	vmulpd		%zmm4, %zmm25, %zmm4
	vmulpd		%zmm5, %zmm25, %zmm5
	vmulpd		%zmm6, %zmm25, %zmm6
	vmulpd		%zmm7, %zmm25, %zmm7

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_a0_8x8_lib8)
#endif





// common inner routine with file scope
//
// scale for alpha = -1.0 and beta
//
// input arguments:
// r10   <- beta
// r11   <- C
// zmm0 <- []
// ...
// zmm7 <- []
//
// output arguments:
// r10   <- beta
// r11   <- C
// zmm0 <- []
// ...
// zmm7 <- []

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M1B_8X8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_scale_m1b_8x8_lib8)
#endif	
	
	// beta
	vbroadcastsd	0(%r10), %zmm24

	vxorpd		%zmm25, %zmm25, %zmm25 // 0.0

	vucomisd	%xmm25, %xmm24 // beta==0.0 ?
	je			0f // end

	vmovapd		0(%r11), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm0
	vmovapd		1*64(%r11), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm1
	vmovapd		2*64(%r11), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm2
	vmovapd		3*64(%r11), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm3
	vmovapd		4*64(%r11), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm4
	vmovapd		5*64(%r11), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm5
	vmovapd		6*64(%r11), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm6
	vmovapd		7*64(%r11), %zmm25
	vfmsub231pd	%zmm24, %zmm25, %zmm7

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m1b_8x8_lib8)
#endif





// common inner routine with file scope
//
// scale for alpha = -1.0 and beta
//
// input arguments:
// r10   <- beta
// r11   <- C
// r12d  <- m1
// r13d  <- n1
// zmm0 <- []
// ...
// zmm7 <- []
//
// output arguments:
// r10   <- beta
// r11   <- C
// r12d  <- m1
// r13d  <- n1
// zmm0 <- []
// ...
// zmm7 <- []

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M1B_8X8_VS_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_scale_m1b_8x8_vs_lib8)
#endif	
	
	// beta
	vbroadcastsd	0(%r10), %zmm24

	vxorpd		%zmm25, %zmm25, %zmm25 // 0.0

	vucomisd	%xmm25, %xmm24 // beta==0.0 ?
	je			0f // end

	// compute mask for rows
	vcvtsi2sd	%r12d, %xmm25, %xmm25
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC00(%rip), %zmm26
#elif defined(OS_MAC)
	vmovupd		LC00(%rip), %zmm26
#endif
	vbroadcastsd	%xmm25, %zmm25
	vsubpd		%zmm25, %zmm26, %zmm25
	vpmovq2m	%zmm25, %k1

	vmovapd		0(%r11), %zmm25 {%k1}{z}
	vfmsub231pd	%zmm24, %zmm25, %zmm0 {%k1}
	cmpl	$ 2, %r13d
	jl		0f // end
	vmovapd		1*64(%r11), %zmm25 {%k1}{z}
	vfmsub231pd	%zmm24, %zmm25, %zmm1 {%k1}
	cmpl	$ 3, %r13d
	jl		0f // end
	vmovapd		2*64(%r11), %zmm25 {%k1}{z}
	vfmsub231pd	%zmm24, %zmm25, %zmm2 {%k1}
	cmpl	$ 4, %r13d
	jl		0f // end
	vmovapd		3*64(%r11), %zmm25 {%k1}{z}
	vfmsub231pd	%zmm24, %zmm25, %zmm3 {%k1}
	cmpl	$ 5, %r13d
	jl		0f // end
	vmovapd		4*64(%r11), %zmm25 {%k1}{z}
	vfmsub231pd	%zmm24, %zmm25, %zmm4 {%k1}
	cmpl	$ 6, %r13d
	jl		0f // end
	vmovapd		5*64(%r11), %zmm25 {%k1}{z}
	vfmsub231pd	%zmm24, %zmm25, %zmm5 {%k1}
	cmpl	$ 7, %r13d
	jl		0f // end
	vmovapd		6*64(%r11), %zmm25 {%k1}{z}
	vfmsub231pd	%zmm24, %zmm25, %zmm6 {%k1}
	je		0f // end
	vmovapd		7*64(%r11), %zmm25 {%k1}{z}
	vfmsub231pd	%zmm24, %zmm25, %zmm7 {%k1}

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m1b_8x8_vs_lib8)
#endif





// common inner routine with file scope
//
// scale for alpha = -1.0 and beta=1.0
//
// input arguments:
// r10   <- C
// zmm0 <- []
// ...
// zmm7 <- []
//
// output arguments:
// r10   <- C
// zmm0 <- []
// ...
// zmm7 <- []

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M11_8X8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_scale_m11_8x8_lib8)
#endif	
	
	vmovapd		0(%r10), %zmm24
	vsubpd		%zmm0, %zmm24, %zmm0
	vmovapd		1*64(%r10), %zmm24
	vsubpd		%zmm1, %zmm24, %zmm1
	vmovapd		2*64(%r10), %zmm24
	vsubpd		%zmm2, %zmm24, %zmm2
	vmovapd		3*64(%r10), %zmm24
	vsubpd		%zmm3, %zmm24, %zmm3
	vmovapd		4*64(%r10), %zmm24
	vsubpd		%zmm4, %zmm24, %zmm4
	vmovapd		5*64(%r10), %zmm24
	vsubpd		%zmm5, %zmm24, %zmm5
	vmovapd		6*64(%r10), %zmm24
	vsubpd		%zmm6, %zmm24, %zmm6
	vmovapd		7*64(%r10), %zmm24
	vsubpd		%zmm7, %zmm24, %zmm7

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m11_8x8_lib8)
#endif





// common inner routine with file scope
//
// scale for alpha = -1.0 and beta=1.0
//
// input arguments:
// r10   <- C
// r11d  <- m1
// r12d  <- n1
// zmm0 <- []
// ...
// zmm7 <- []
//
// output arguments:
// r10   <- C
// r11d  <- m1
// r12d  <- n1
// zmm0 <- []
// ...
// zmm7 <- []

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M11_8X8_VS_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_scale_m11_8x8_vs_lib8)
#endif	
	
	// compute mask for rows
	vcvtsi2sd	%r11d, %xmm25, %xmm25
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC00(%rip), %zmm26
#elif defined(OS_MAC)
	vmovupd		LC00(%rip), %zmm26
#endif
	vbroadcastsd	%xmm25, %zmm25
	vsubpd		%zmm25, %zmm26, %zmm25
	vpmovq2m	%zmm25, %k1

	vmovapd		0(%r10), %zmm24 {%k1}{z}
	vsubpd		%zmm0, %zmm24, %zmm0 {%k1}
	cmpl	$ 2, %r12d
	jl		0f // end
	vmovapd		1*64(%r10), %zmm24 {%k1}{z}
	vsubpd		%zmm1, %zmm24, %zmm1 {%k1}
	cmpl	$ 3, %r12d
	jl		0f // end
	vmovapd		2*64(%r10), %zmm24 {%k1}{z}
	vsubpd		%zmm2, %zmm24, %zmm2 {%k1}
	cmpl	$ 4, %r12d
	jl		0f // end
	vmovapd		3*64(%r10), %zmm24 {%k1}{z}
	vsubpd		%zmm3, %zmm24, %zmm3 {%k1}
	cmpl	$ 5, %r12d
	jl		0f // end
	vmovapd		4*64(%r10), %zmm24 {%k1}{z}
	vsubpd		%zmm4, %zmm24, %zmm4 {%k1}
	cmpl	$ 6, %r12d
	jl		0f // end
	vmovapd		5*64(%r10), %zmm24 {%k1}{z}
	vsubpd		%zmm5, %zmm24, %zmm5 {%k1}
	cmpl	$ 7, %r12d
	jl		0f // end
	vmovapd		6*64(%r10), %zmm24 {%k1}{z}
	vsubpd		%zmm6, %zmm24, %zmm6 {%k1}
	je		0f // end
	vmovapd		7*64(%r10), %zmm24 {%k1}{z}
	vsubpd		%zmm7, %zmm24, %zmm7 {%k1}

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m11_8x8_vs_lib8)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// zmm0 <- []
// ...
// zmm7 <- []
//
// output arguments:
// r10  <- D
// zmm0 <- []
// ...
// zmm7 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_8X8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_store_8x8_lib8)
#endif
	
	vmovapd %zmm0,   0(%r10)
	vmovapd %zmm1,  64(%r10)
	vmovapd %zmm2, 128(%r10)
	vmovapd %zmm3, 192(%r10)
	vmovapd %zmm4,   0+256(%r10)
	vmovapd %zmm5,  64+256(%r10)
	vmovapd %zmm6, 128+256(%r10)
	vmovapd %zmm7, 192+256(%r10)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_8x8_lib8)
#endif





// common inner routine with file scope
//
// store n vs
//
// input arguments:
// r10   <- D
// r11d  <- m1
// r12d  <- n1
// zmm0 <- []
// ...
// zmm7 <- []
//
// output arguments:
// r10   <- D
// r11d  <- m1
// r12d  <- n1
// zmm0 <- []
// ...
// zmm7 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_8X8_VS_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_store_8x8_vs_lib8)
#endif
	
#if 1

	// compute mask for rows
	vcvtsi2sd	%r11d, %xmm25, %xmm25
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC00(%rip), %zmm24
#elif defined(OS_MAC)
	vmovupd		LC00(%rip), %zmm24
#endif
	vbroadcastsd	%xmm25, %zmm25
	vsubpd		%zmm25, %zmm24, %zmm25
	vpmovq2m	%zmm25, %k1

#else

	movq	%rcx, %r14 // backup
	movl	$ 8, %ecx // saturate to ps=8
	cmpl	%r11d, %ecx
	cmovgl	%r11d, %ecx
	movl	$ 1, %r13d // shift base
	sall	%cl, %r13d // shift by cl
	subl	$ 1, %r13d
	kmovd	%r13d, %k1
	movq	%r14, %rcx // restore

#endif

	vmovapd %zmm0,   0(%r10) {%k1}
	cmpl	$ 2, %r12d
	jl		0f // end
	vmovapd %zmm1,  64(%r10) {%k1}
	cmpl	$ 3, %r12d
	jl		0f // end
	vmovapd %zmm2, 128(%r10) {%k1}
	cmpl	$ 4, %r12d
	jl		0f // end
	vmovapd %zmm3, 192(%r10) {%k1}
	cmpl	$ 5, %r12d
	jl		0f // end
	vmovapd %zmm4,   0+256(%r10) {%k1}
	cmpl	$ 6, %r12d
	jl		0f // end
	vmovapd %zmm5,  64+256(%r10) {%k1}
	cmpl	$ 7, %r12d
	jl		0f // end
	vmovapd %zmm6, 128+256(%r10) {%k1}
	je		0f // end
	vmovapd %zmm7, 192+256(%r10) {%k1}

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_8x8_vs_lib8)
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
// zmm0 <- []
// ...
// zmm7 <- []
//
// output arguments:
// r10  <- offset
// r11  <- D
// r12  <- 4*sdd*sizeof(double)
// r13  <- m0 // row index: start from (inc)
// r14  <- m1 // row index: up to (exc)
// r15  <- n1-n0
// rax  <- n1-n0
// zmm0 <- []
// ...
// zmm7 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_8X8_GEN_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_store_8x8_gen_lib8)
#endif

#if 1

	// compute mask for rows
	vcvtsi2sd	%r13d, %xmm25, %xmm25
	vcvtsi2sd	%r14d, %xmm26, %xmm26
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC00(%rip), %zmm24
#elif defined(OS_MAC)
	vmovupd		LC00(%rip), %zmm24
#endif
	vbroadcastsd	%xmm25, %zmm25
	vbroadcastsd	%xmm26, %zmm26
	vsubpd		%zmm24, %zmm25, %zmm25
	vsubpd		%zmm26, %zmm24, %zmm26
	vandpd		%zmm25, %zmm26, %zmm26

	vpmovq2m	%zmm26, %k1

#else

	movq	%rcx, %rbp // backup

	movl	$ 0, %ecx // saturate to 0
	cmpl	%r13d, %ecx
	cmovgl	%ecx, %r13d

	movl	$ 8, %ecx // saturate to ps=8
	cmpl	%r13d, %ecx
	cmovgl	%r13d, %ecx

	movl	$ 1, %ebx // shift base
	sall	%cl, %ebx // shift by cl
	subl	$ 1, %ebx
	notl	%ebx
	kmovd	%ebx, %k1

	movl	$ 8, %ecx // saturate to ps=8
	cmpl	%r14d, %ecx
	cmovgl	%r14d, %ecx
	movl	$ 1, %ebx // shift base
	sall	%cl, %ebx // shift by cl
	subl	$ 1, %ebx
	kmovd	%ebx, %k2

	kandd	%k2, %k1, %k1

	movq	%rbp, %rcx // restore

#endif


	// return if no cols
	cmpl	$ 0, %eax
	jle		8f

	cmpl	$ 0, %r10d
	jg		0f

	// offset==0

	cmpl	$ 0, %r15d
	jg		1f // next
	vmovapd %zmm0,   0(%r11) {%k1}
	jmp		2f
1:
	cmpl	$ 1, %r15d
	jg		1f // next
2:
	cmpl	$ 2, %eax
	jl		8f // end
	vmovapd %zmm1,  64(%r11) {%k1}
	jmp		2f
1:
	cmpl	$ 2, %r15d
	jg		1f // next
2:
	cmpl	$ 3, %eax
	jl		8f // end
	vmovapd %zmm2, 128(%r11) {%k1}
	jmp		2f
1:
	cmpl	$ 3, %r15d
	jg		1f // next
2:
	cmpl	$ 4, %eax
	jl		8f // end
	vmovapd %zmm3, 192(%r11) {%k1}
	jmp		2f
1:
	cmpl	$ 4, %r15d
	jg		1f // next
2:
	cmpl	$ 5, %eax
	jl		8f // end
	vmovapd %zmm4,   0+256(%r11) {%k1}
	jmp		2f
1:
	cmpl	$ 5, %r15d
	jg		1f // next
2:
	cmpl	$ 6, %eax
	jl		8f // end
	vmovapd %zmm5,  64+256(%r11) {%k1}
	jmp		2f
1:
	cmpl	$ 6, %r15d
	jg		1f // next
2:
	cmpl	$ 7, %eax
	jl		8f // end
	vmovapd %zmm6, 128+256(%r11) {%k1}
	jmp		2f
1:
	cmpl	$ 7, %r15d
	jg		8f // end 1f // next
2:
	cmpl	$ 7, %eax
	je		8f // end
	vmovapd %zmm7, 192+256(%r11) {%k1}
//1:

	jmp		8f

0:
	
	cmpl	$ 3, %r10d
	jg		14f

	cmpl	$ 1, %r10d
	jg		11f

	jmp		0f

11: // 2 3

	cmpl	$ 2, %r10d
	jg		2f

	jmp		1f

14: // 4 5 6 7

	cmpl	$ 5, %r10d
	jg		16f

	cmpl	$ 4, %r10d
	jg		4f

	jmp		3f

16: // 6 7

	cmpl	$ 6, %r10d
	jg		6f

	jmp		5f

0: 	// offset==1

	movl	$ 0x7f, %ebx
	kmovd	%ebx, %k2
	kandd	%k2, %k1, %k2
	movl	$ 0x80, %ebx
	kmovd	%ebx, %k3
	kandd	%k3, %k1, %k3

	leaq	8(%r11), %rbx

	jmp		7f

1:  // offset==2

	movl	$ 0x3f, %ebx
	kmovd	%ebx, %k2
	kandd	%k2, %k1, %k2
	movl	$ 0xc0, %ebx
	kmovd	%ebx, %k3
	kandd	%k3, %k1, %k3

	leaq	16(%r11), %rbx

	jmp		7f

2:  // offset==3

	movl	$ 0x1f, %ebx
	kmovd	%ebx, %k2
	kandd	%k2, %k1, %k2
	movl	$ 0xe0, %ebx
	kmovd	%ebx, %k3
	kandd	%k3, %k1, %k3

	leaq	24(%r11), %rbx

	jmp		7f

3:  // offset==4

	movl	$ 0x0f, %ebx
	kmovd	%ebx, %k2
	kandd	%k2, %k1, %k2
	movl	$ 0xf0, %ebx
	kmovd	%ebx, %k3
	kandd	%k3, %k1, %k3

	leaq	32(%r11), %rbx

	jmp		7f

4:  // offset==5

	movl	$ 0x07, %ebx
	kmovd	%ebx, %k2
	kandd	%k2, %k1, %k2
	movl	$ 0xf8, %ebx
	kmovd	%ebx, %k3
	kandd	%k3, %k1, %k3

	leaq	40(%r11), %rbx

	jmp		7f

5:  // offset==6

	movl	$ 0x03, %ebx
	kmovd	%ebx, %k2
	kandd	%k2, %k1, %k2
	movl	$ 0xfc, %ebx
	kmovd	%ebx, %k3
	kandd	%k3, %k1, %k3

	leaq	48(%r11), %rbx

	jmp		7f

6:  // offset==7

	movl	$ 0x01, %ebx
	kmovd	%ebx, %k2
	kandd	%k2, %k1, %k2
	movl	$ 0xfe, %ebx
	kmovd	%ebx, %k3
	kandd	%k3, %k1, %k3

	leaq	56(%r11), %rbx

//	jmp		7f

7:

	cmpl	$ 0, %r15d
	jg		1f // next
	vmovupd %zmm0,   0(%rbx) {%k2}
	vmovupd %zmm0,   0-64(%rbx, %r12) {%k3}
	jmp		2f
1:
	cmpl	$ 1, %r15d
	jg		1f // next
2:
	cmpl	$ 2, %eax
	jl		8f // end
	vmovupd %zmm1,  64(%rbx) {%k2}
	vmovupd %zmm1,  64-64(%rbx, %r12) {%k3}
	jmp		2f
1:
	cmpl	$ 2, %r15d
	jg		1f // next
2:
	cmpl	$ 3, %eax
	jl		8f // end
	vmovupd %zmm2, 128(%rbx) {%k2}
	vmovupd %zmm2, 128-64(%rbx, %r12) {%k3}
	jmp		2f
1:
	cmpl	$ 3, %r15d
	jg		1f // next
2:
	cmpl	$ 4, %eax
	jl		8f // end
	vmovupd %zmm3, 192(%rbx) {%k2}
	vmovupd %zmm3, 192-64(%rbx, %r12) {%k3}
	jmp		2f
1:
	cmpl	$ 4, %r15d
	jg		1f // next
2:
	cmpl	$ 5, %eax
	jl		8f // end
	vmovupd %zmm4,   0+256(%rbx) {%k2}
	vmovupd %zmm4,   0+256-64(%rbx, %r12) {%k3}
	jmp		2f
1:
	cmpl	$ 5, %r15d
	jg		1f // next
2:
	cmpl	$ 6, %eax
	jl		8f // end
	vmovupd %zmm5,  64+256(%rbx) {%k2}
	vmovupd %zmm5,  64+256-64(%rbx, %r12) {%k3}
	jmp		2f
1:
	cmpl	$ 6, %r15d
	jg		1f // next
2:
	cmpl	$ 7, %eax
	jl		8f // end
	vmovupd %zmm6, 128+256(%rbx) {%k2}
	vmovupd %zmm6, 128+256-64(%rbx, %r12) {%k3}
	jmp		2f
1:
	cmpl	$ 7, %r15d
	jg		8f // end 1f // next
2:
	cmpl	$ 7, %eax
	je		8f // end
	vmovupd %zmm7, 192+256(%rbx) {%k2}
	vmovupd %zmm7, 192+256-64(%rbx, %r12) {%k3}
//1:

8:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_8x8_gen_lib8)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// zmm0 <- []
// ...
// zmm7 <- []
//
// output arguments:
// r10  <- D
// zmm0 <- []
// ...
// zmm7 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_8X8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_store_l_8x8_lib8)
#endif
	
	//
	vmovapd %zmm0,   0(%r10)
	//
	movl	$ 0xfe, %r11d
	kmovd	%r11d, %k1
	vmovapd %zmm1,  64(%r10) {%k1}
	//
	movl	$ 0xfc, %r11d
	kmovd	%r11d, %k1
	vmovapd %zmm2, 128(%r10) {%k1}
	//
	movl	$ 0xf8, %r11d
	kmovd	%r11d, %k1
	vmovapd %zmm3, 192(%r10) {%k1}
	//
	movl	$ 0xf0, %r11d
	kmovd	%r11d, %k1
	vmovapd %zmm4,   0+256(%r10) {%k1}
	//
	movl	$ 0xe0, %r11d
	kmovd	%r11d, %k1
	vmovapd %zmm5,  64+256(%r10) {%k1}
	//
	movl	$ 0xc0, %r11d
	kmovd	%r11d, %k1
	vmovapd %zmm6, 128+256(%r10) {%k1}
	//
	movl	$ 0x80, %r11d
	kmovd	%r11d, %k1
	vmovapd %zmm7, 192+256(%r10) {%k1}

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_8x8_lib8)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// r11d  <- m1
// r12d  <- n1
// zmm0 <- []
// ...
// zmm7 <- []
//
// output arguments:
// r10  <- D
// r11d  <- m1
// r12d  <- n1
// zmm0 <- []
// ...
// zmm7 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_8X8_VS_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_store_l_8x8_vs_lib8)
#endif
	
	// compute mask for rows
	vcvtsi2sd	%r11d, %xmm25, %xmm25
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC00(%rip), %zmm24
#elif defined(OS_MAC)
	vmovupd		LC00(%rip), %zmm24
#endif
	vbroadcastsd	%xmm25, %zmm25
	vsubpd		%zmm25, %zmm24, %zmm25

	vpmovq2m	%zmm25, %k2

	//
	vmovapd %zmm0,   0(%r10) {%k2}
	//
	cmpl	$ 2, %r12d
	jl		0f // end
	movl	$ 0xfe, %r11d
	kmovd	%r11d, %k1
	kandd	%k1, %k2, %k1
	vmovapd %zmm1,  64(%r10) {%k1}
	//
	cmpl	$ 3, %r12d
	jl		0f // end
	movl	$ 0xfc, %r11d
	kmovd	%r11d, %k1
	kandd	%k1, %k2, %k1
	vmovapd %zmm2, 128(%r10) {%k1}
	//
	cmpl	$ 4, %r12d
	jl		0f // end
	movl	$ 0xf8, %r11d
	kmovd	%r11d, %k1
	kandd	%k1, %k2, %k1
	vmovapd %zmm3, 192(%r10) {%k1}
	//
	cmpl	$ 5, %r12d
	jl		0f // end
	movl	$ 0xf0, %r11d
	kmovd	%r11d, %k1
	kandd	%k1, %k2, %k1
	vmovapd %zmm4,   0+256(%r10) {%k1}
	//
	cmpl	$ 6, %r12d
	jl		0f // end
	movl	$ 0xe0, %r11d
	kmovd	%r11d, %k1
	kandd	%k1, %k2, %k1
	vmovapd %zmm5,  64+256(%r10) {%k1}
	//
	cmpl	$ 7, %r12d
	jl		0f // end
	movl	$ 0xc0, %r11d
	kmovd	%r11d, %k1
	kandd	%k1, %k2, %k1
	vmovapd %zmm6, 128+256(%r10) {%k1}
	//
	cmpl	$ 7, %r12d
	je		0f // end
	movl	$ 0x80, %r11d
	kmovd	%r11d, %k1
	kandd	%k1, %k2, %k1
	vmovapd %zmm7, 192+256(%r10) {%k1}

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_8x8_vs_lib8)
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
// zmm0 <- []
// ...
// zmm7 <- []
//
// output arguments:
// r10  <- offset
// r11  <- D
// r12  <- 4*sdd*sizeof(double)
// r13  <- m0 // row index: start from (inc)
// r14  <- m1 // row index: up to (exc)
// r15  <- n1-n0
// rax  <- n1-n0
// zmm0 <- []
// ...
// zmm7 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_8X8_GEN_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_store_l_8x8_gen_lib8)
#endif

	// XXX this kernel uses a more "natural" definition of gen for lower triangular matrices, and not compatible with the one implemented in lib4 routines

	// compute mask for rows
	vcvtsi2sd	%r13d, %xmm25, %xmm25
	vcvtsi2sd	%r14d, %xmm26, %xmm26
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC00(%rip), %zmm24
#elif defined(OS_MAC)
	vmovupd		LC00(%rip), %zmm24
#endif
	vbroadcastsd	%xmm25, %zmm25
	vbroadcastsd	%xmm26, %zmm26
	vsubpd		%zmm24, %zmm25, %zmm25
	vsubpd		%zmm26, %zmm24, %zmm26
	vandpd		%zmm25, %zmm26, %zmm26

	vpmovq2m	%zmm26, %k1


	// return if no cols
	cmpl	$ 0, %eax
	jle		8f

	cmpl	$ 0, %r10d
	jg		0f

	// offset==0

	cmpl	$ 0, %r15d
	jg		1f // next
	vmovapd %zmm0,   0(%r11) {%k1}
	jmp		2f
1:
	cmpl	$ 1, %r15d
	jg		1f // next
2:
	cmpl	$ 2, %eax
	jl		8f // end
	movl	$ 0xfe, %ebp
	kmovd	%ebp, %k4
	kandd	%k1, %k4, %k4
	vmovapd %zmm1,  64(%r11) {%k4}
	jmp		2f
1:
	cmpl	$ 2, %r15d
	jg		1f // next
2:
	cmpl	$ 3, %eax
	jl		8f // end
	movl	$ 0xfc, %ebp
	kmovd	%ebp, %k4
	kandd	%k1, %k4, %k4
	vmovapd %zmm2, 128(%r11) {%k4}
	jmp		2f
1:
	cmpl	$ 3, %r15d
	jg		1f // next
2:
	cmpl	$ 4, %eax
	jl		8f // end
	movl	$ 0xf8, %ebp
	kmovd	%ebp, %k4
	kandd	%k1, %k4, %k4
	vmovapd %zmm3, 192(%r11) {%k4}
	jmp		2f
1:
	cmpl	$ 4, %r15d
	jg		1f // next
2:
	cmpl	$ 5, %eax
	jl		8f // end
	movl	$ 0xf0, %ebp
	kmovd	%ebp, %k4
	kandd	%k1, %k4, %k4
	vmovapd %zmm4,   0+256(%r11) {%k4}
	jmp		2f
1:
	cmpl	$ 5, %r15d
	jg		1f // next
2:
	cmpl	$ 6, %eax
	jl		8f // end
	movl	$ 0xe0, %ebp
	kmovd	%ebp, %k4
	kandd	%k1, %k4, %k4
	vmovapd %zmm5,  64+256(%r11) {%k4}
	jmp		2f
1:
	cmpl	$ 6, %r15d
	jg		1f // next
2:
	cmpl	$ 7, %eax
	jl		8f // end
	movl	$ 0xc0, %ebp
	kmovd	%ebp, %k4
	kandd	%k1, %k4, %k4
	vmovapd %zmm6, 128+256(%r11) {%k4}
	jmp		2f
1:
	cmpl	$ 7, %r15d
	jg		8f // end 1f // next
2:
	cmpl	$ 7, %eax
	je		8f // end
	movl	$ 0x80, %ebp
	kmovd	%ebp, %k4
	kandd	%k1, %k4, %k4
	vmovapd %zmm7, 192+256(%r11) {%k4}
//1:

	jmp		8f

0:
	
	cmpl	$ 3, %r10d
	jg		14f

	cmpl	$ 1, %r10d
	jg		11f

	jmp		0f

11: // 2 3

	cmpl	$ 2, %r10d
	jg		2f

	jmp		1f

14: // 4 5 6 7

	cmpl	$ 5, %r10d
	jg		16f

	cmpl	$ 4, %r10d
	jg		4f

	jmp		3f

16: // 6 7

	cmpl	$ 6, %r10d
	jg		6f

	jmp		5f

0: 	// offset==1

	movl	$ 0x7f, %ebx
	kmovd	%ebx, %k2
	kandd	%k2, %k1, %k2
	movl	$ 0x80, %ebx
	kmovd	%ebx, %k3
	kandd	%k3, %k1, %k3

	leaq	8(%r11), %rbx

	jmp		7f

1:  // offset==2

	movl	$ 0x3f, %ebx
	kmovd	%ebx, %k2
	kandd	%k2, %k1, %k2
	movl	$ 0xc0, %ebx
	kmovd	%ebx, %k3
	kandd	%k3, %k1, %k3

	leaq	16(%r11), %rbx

	jmp		7f

2:  // offset==3

	movl	$ 0x1f, %ebx
	kmovd	%ebx, %k2
	kandd	%k2, %k1, %k2
	movl	$ 0xe0, %ebx
	kmovd	%ebx, %k3
	kandd	%k3, %k1, %k3

	leaq	24(%r11), %rbx

	jmp		7f

3:  // offset==4

	movl	$ 0x0f, %ebx
	kmovd	%ebx, %k2
	kandd	%k2, %k1, %k2
	movl	$ 0xf0, %ebx
	kmovd	%ebx, %k3
	kandd	%k3, %k1, %k3

	leaq	32(%r11), %rbx

	jmp		7f

4:  // offset==5

	movl	$ 0x07, %ebx
	kmovd	%ebx, %k2
	kandd	%k2, %k1, %k2
	movl	$ 0xf8, %ebx
	kmovd	%ebx, %k3
	kandd	%k3, %k1, %k3

	leaq	40(%r11), %rbx

	jmp		7f

5:  // offset==6

	movl	$ 0x03, %ebx
	kmovd	%ebx, %k2
	kandd	%k2, %k1, %k2
	movl	$ 0xfc, %ebx
	kmovd	%ebx, %k3
	kandd	%k3, %k1, %k3

	leaq	48(%r11), %rbx

	jmp		7f

6:  // offset==7

	movl	$ 0x01, %ebx
	kmovd	%ebx, %k2
	kandd	%k2, %k1, %k2
	movl	$ 0xfe, %ebx
	kmovd	%ebx, %k3
	kandd	%k3, %k1, %k3

	leaq	56(%r11), %rbx

//	jmp		7f

7:

	cmpl	$ 0, %r15d
	jg		1f // next
	vmovupd %zmm0,   0(%rbx) {%k2}
	vmovupd %zmm0,   0-64(%rbx, %r12) {%k3}
	jmp		2f
1:
	cmpl	$ 1, %r15d
	jg		1f // next
2:
	cmpl	$ 2, %eax
	jl		8f // end
	movl	$ 0xfe, %ebp
	kmovd	%ebp, %k4
	kandd	%k3, %k4, %k5
	kandd	%k2, %k4, %k4
	vmovupd %zmm1,  64(%rbx) {%k4}
	vmovupd %zmm1,  64-64(%rbx, %r12) {%k5}
	jmp		2f
1:
	cmpl	$ 2, %r15d
	jg		1f // next
2:
	cmpl	$ 3, %eax
	jl		8f // end
	movl	$ 0xfc, %ebp
	kmovd	%ebp, %k4
	kandd	%k3, %k4, %k5
	kandd	%k2, %k4, %k4
	vmovupd %zmm2, 128(%rbx) {%k4}
	vmovupd %zmm2, 128-64(%rbx, %r12) {%k5}
	jmp		2f
1:
	cmpl	$ 3, %r15d
	jg		1f // next
2:
	cmpl	$ 4, %eax
	jl		8f // end
	movl	$ 0xf8, %ebp
	kmovd	%ebp, %k4
	kandd	%k3, %k4, %k5
	kandd	%k2, %k4, %k4
	vmovupd %zmm3, 192(%rbx) {%k4}
	vmovupd %zmm3, 192-64(%rbx, %r12) {%k5}
	jmp		2f
1:
	cmpl	$ 4, %r15d
	jg		1f // next
2:
	cmpl	$ 5, %eax
	jl		8f // end
	movl	$ 0xf0, %ebp
	kmovd	%ebp, %k4
	kandd	%k3, %k4, %k5
	kandd	%k2, %k4, %k4
	vmovupd %zmm4,   0+256(%rbx) {%k4}
	vmovupd %zmm4,   0+256-64(%rbx, %r12) {%k5}
	jmp		2f
1:
	cmpl	$ 5, %r15d
	jg		1f // next
2:
	cmpl	$ 6, %eax
	jl		8f // end
	movl	$ 0xe0, %ebp
	kmovd	%ebp, %k4
	kandd	%k3, %k4, %k5
	kandd	%k2, %k4, %k4
	vmovupd %zmm5,  64+256(%rbx) {%k4}
	vmovupd %zmm5,  64+256-64(%rbx, %r12) {%k5}
	jmp		2f
1:
	cmpl	$ 6, %r15d
	jg		1f // next
2:
	cmpl	$ 7, %eax
	jl		8f // end
	movl	$ 0xc0, %ebp
	kmovd	%ebp, %k4
	kandd	%k3, %k4, %k5
	kandd	%k2, %k4, %k4
	vmovupd %zmm6, 128+256(%rbx) {%k4}
	vmovupd %zmm6, 128+256-64(%rbx, %r12) {%k5}
	jmp		2f
1:
	cmpl	$ 7, %r15d
	jg		8f // end 1f // next
2:
	cmpl	$ 7, %eax
	je		8f // end
	movl	$ 0x80, %ebp
	kmovd	%ebp, %k4
	kandd	%k3, %k4, %k5
	kandd	%k2, %k4, %k4
	vmovupd %zmm7, 192+256(%rbx) {%k4}
	vmovupd %zmm7, 192+256-64(%rbx, %r12) {%k5}
//1:

8:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_8x8_gen_lib8)
#endif





// common inner routine with file scope
//
// transpose
//
// input arguments:
// zmm0  <- []
// ...
// zmm7  <- []
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_TRAN_8X8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_tran_8x8_lib8)
#endif

	vunpcklpd	%zmm1, %zmm0, %zmm24
	vunpckhpd	%zmm1, %zmm0, %zmm25
	vunpcklpd	%zmm3, %zmm2, %zmm26
	vunpckhpd	%zmm3, %zmm2, %zmm27
	vunpcklpd	%zmm5, %zmm4, %zmm28
	vunpckhpd	%zmm5, %zmm4, %zmm29
	vunpcklpd	%zmm7, %zmm6, %zmm30
	vunpckhpd	%zmm7, %zmm6, %zmm31

	movl	$ 0xcc, %r10d
	kmovd	%r10d, %k1
	movl	$ 0x33, %r10d
	kmovd	%r10d, %k2

	vmovapd		%zmm24, %zmm0
	vpermq		$ 0x40, %zmm26, %zmm24 {%k1}
	vpermq		$ 0x0e, %zmm0, %zmm26 {%k2}
	vmovapd		%zmm25, %zmm0
	vpermq		$ 0x40, %zmm27, %zmm25 {%k1}
	vpermq		$ 0x0e, %zmm0, %zmm27 {%k2}
	vmovapd		%zmm28, %zmm0
	vpermq		$ 0x40, %zmm30, %zmm28 {%k1}
	vpermq		$ 0x0e, %zmm0, %zmm30 {%k2}
	vmovapd		%zmm29, %zmm0
	vpermq		$ 0x40, %zmm31, %zmm29 {%k1}
	vpermq		$ 0x0e, %zmm0, %zmm31 {%k2}

	vshuff64x2	$ 0x44, %zmm28, %zmm24, %zmm0
	vshuff64x2	$ 0xee, %zmm28, %zmm24, %zmm4
	vshuff64x2	$ 0x44, %zmm29, %zmm25, %zmm1
	vshuff64x2	$ 0xee, %zmm29, %zmm25, %zmm5
	vshuff64x2	$ 0x44, %zmm30, %zmm26, %zmm2
	vshuff64x2	$ 0xee, %zmm30, %zmm26, %zmm6
	vshuff64x2	$ 0x44, %zmm31, %zmm27, %zmm3
	vshuff64x2	$ 0xee, %zmm31, %zmm27, %zmm7

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_tran_8x8_lib8)
#endif





//                               1      2              3          4          5             6          7
// void kernel_dgemm_nt_8x8_lib8(int k, double *alpha, double *A, double *B, double *beta, double *C, double *D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_8x8_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB8
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib8)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X8_LIB8
#else
	CALL(inner_scale_ab_8x8_lib8)
#endif


	// store n

	movq	ARG7, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_8X8_LIB8
#else
	CALL(inner_store_8x8_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_8x8_lib8)





//                                  1      2              3          4          5             6          7          8       9
// void kernel_dgemm_nt_8x8_vs_lib8(int k, double *alpha, double *A, double *B, double *beta, double *C, double *D, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_8x8_vs_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B
	movq	ARG8, %r13 // m1 
	movq	ARG9, %r14 // n1

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_VS_LIB8
#else
	CALL(inner_kernel_dgemm_nt_8x8_vs_lib8)
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12   // C
	movq	ARG8, %r13 // m1 
	movq	ARG9, %r14 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X8_VS_LIB8
#else
	CALL(inner_scale_ab_8x8_vs_lib8)
#endif


	// store n

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // m1 
	movq	ARG9, %r12 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_8X8_VS_LIB8
#else
	CALL(inner_store_8x8_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_8x8_vs_lib8)





//                                   1      2              3          4          5             6            7          8        9            10         11       12      13      14      15
// void kernel_dgemm_nt_8x8_gen_lib8(int k, double *alpha, double *A, double *B, double *beta, int offsetC, double *C, int sdc, int offsetD, double *D, int sdd, int m0, int m1, int n0, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_8x8_gen_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B
	movq	ARG13, %r13 // m1
	movq	ARG15, %r14 // n1

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_VS_LIB8
#else
	CALL(inner_kernel_dgemm_nt_8x8_vs_lib8)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12 // offsetC
	movq	ARG7, %r13 // C
	movq	ARG8, %r14 // sdc
	sall	$ 6, %r14d // 4*sdc*sizeof(double)
	movq	ARG14, %r15 // n0
	movq	ARG15, %rax // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X8_GEN_LIB8
#else
	CALL(inner_scale_ab_8x8_gen_lib8)
#endif


	// store n gen

	movq	ARG9, %r10 // offsetD
	movq	ARG10, %r11 // D
	movq	ARG11, %r12 // sdd
	sall	$ 6, %r12d // 4*sdb*sizeof(double)
	movq	ARG12, %r13 // m0
	movq	ARG13, %r14 // m1
	movq	ARG14, %r15 // n0
	movq	ARG15, %rax // n1

#if MACRO_LEVEL>=1
	INNER_STORE_8X8_GEN_LIB8
#else
	CALL(inner_store_8x8_gen_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_8x8_gen_lib8)





//                               1      2              3          4            5          6        7             8          9
// void kernel_dgemm_nn_8x8_lib8(int k, double *alpha, double *A, int offsetB, double *B, int sdb, double *beta, double *C, double *D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nn_8x8_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG5, %r12  // B
	movq	ARG6, %r13 // sdb
	sall	$ 6, %r13d // 4*sdb*sizeof(double)
	movq	ARG4, %r14 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_DGEMM_NN_8X8_LIB8
#else
	CALL(inner_edge_dgemm_nn_8x8_lib8)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_8X8_LIB8
#else
	CALL(inner_kernel_dgemm_nn_8x8_lib8)
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X8_LIB8
#else
	CALL(inner_scale_ab_8x8_lib8)
#endif


	// store n

	movq	ARG9, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_8X8_LIB8
#else
	CALL(inner_store_8x8_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nn_8x8_lib8)





//                                  1      2              3          4            5          6        7             8          9          10      11
// void kernel_dgemm_nn_8x8_vs_lib8(int k, double *alpha, double *A, int offsetB, double *B, int sdb, double *beta, double *C, double *D, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nn_8x8_vs_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG5, %r12  // B
	movq	ARG6, %r13 // sdb
	sall	$ 6, %r13d // 4*sdb*sizeof(double)

	movq	ARG4, %r14 // offsetB

	// TODO vs
#if MACRO_LEVEL>=1
	INNER_EDGE_DGEMM_NN_8X8_LIB8
#else
	CALL(inner_edge_dgemm_nn_8x8_lib8)
#endif

	movq	ARG10, %r14 // km 
	movq	ARG11, %r15 // kn 

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_8X8_VS_LIB8
#else
	CALL(inner_kernel_dgemm_nn_8x8_vs_lib8)
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG10, %r13 // km 
	movq	ARG11, %r14 // kn 

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X8_VS_LIB8
#else
	CALL(inner_scale_ab_8x8_vs_lib8)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // km 
	movq	ARG11, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_8X8_VS_LIB8
#else
	CALL(inner_store_8x8_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nn_8x8_vs_lib8)





//                                   1      2              3          4            5          6        7             8         9          10       11        12         13       14      15      16      17
// void kernel_dgemm_nn_8x8_gen_lib8(int k, double *alpha, double *A, int offsetB, double *B, int sdb, double *beta, int offC, double *C, int sdc, int offD, double *D, int sdd, int m0, int m1, int n0, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nn_8x8_gen_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG5, %r12  // B
	movq	ARG6, %r13 // sdb
	sall	$ 6, %r13d // 4*sdb*sizeof(double)

	movq	ARG4, %r14 // offsetB

	// TODO vs
#if MACRO_LEVEL>=1
	INNER_EDGE_DGEMM_NN_8X8_LIB8
#else
	CALL(inner_edge_dgemm_nn_8x8_lib8)
#endif

	movq	ARG15, %r14 // km 
	movq	ARG17, %r15 // kn 

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_8X8_VS_LIB8
#else
	CALL(inner_kernel_dgemm_nn_8x8_vs_lib8)
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12 // offsetC
	movq	ARG9, %r13 // C
	movq	ARG10, %r14 // sdc
	sall	$ 6, %r14d // 4*sdc*sizeof(double)
	movq	ARG16, %r15 // n0
	movq	ARG17, %rax // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X8_GEN_LIB8
#else
	CALL(inner_scale_ab_8x8_gen_lib8)
#endif


	// store n

	movq	ARG11, %r10 // offsetD
	movq	ARG12, %r11 // D
	movq	ARG13, %r12 // sdd
	sall	$ 6, %r12d // 4*sdb*sizeof(double)
	movq	ARG14, %r13 // m0
	movq	ARG15, %r14 // m1
	movq	ARG16, %r15 // n0
	movq	ARG17, %rax // n1

#if MACRO_LEVEL>=1
	INNER_STORE_8X8_GEN_LIB8
#else
	CALL(inner_store_8x8_gen_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nn_8x8_gen_lib8)





//                               1      2              3          4            5          6        7             8          9
// void kernel_dgemm_tt_8x8_lib8(int k, double *alpha, int offsetA, double *A, int sda, double *B, double *beta, double *C, double *D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_tt_8x8_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG6, %r11  // B
	movq	ARG4, %r12  // A
	movq	ARG5, %r13 // sda
	sall	$ 6, %r13d // 8*sda*sizeof(double)
	movq	ARG3, %r14 // offsetA

#if MACRO_LEVEL>=1
	INNER_EDGE_DGEMM_NN_8X8_LIB8
#else
	CALL(inner_edge_dgemm_nn_8x8_lib8)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_8X8_LIB8
#else
	CALL(inner_kernel_dgemm_nn_8x8_lib8)
#endif

#if MACRO_LEVEL>=1
	INNER_TRAN_8X8_LIB8
#else
	CALL(inner_tran_8x8_lib8)
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X8_LIB8
#else
	CALL(inner_scale_ab_8x8_lib8)
#endif


	// store n

	movq	ARG9, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_8X8_LIB8
#else
	CALL(inner_store_8x8_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_tt_8x8_lib8)





//                                  1      2              3          4            5          6        7             8          9          10      11
// void kernel_dgemm_tt_8x8_vs_lib8(int k, double *alpha, int offsetA, double *A, int sda, double *B, double *beta, double *C, double *D, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_tt_8x8_vs_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG6, %r11  // B
	movq	ARG4, %r12  // A
	movq	ARG5, %r13 // sda
	sall	$ 6, %r13d // 8*sda*sizeof(double)

	movq	ARG3, %r14 // offsetA

	// TODO vs
#if MACRO_LEVEL>=1
	INNER_EDGE_DGEMM_NN_8X8_LIB8
#else
	CALL(inner_edge_dgemm_nn_8x8_lib8)
#endif

	movq	ARG11, %r14 // n1
	movq	ARG10, %r15 // m1

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_8X8_VS_LIB8
#else
	CALL(inner_kernel_dgemm_nn_8x8_vs_lib8)
#endif

#if MACRO_LEVEL>=1
	INNER_TRAN_8X8_LIB8
#else
	CALL(inner_tran_8x8_lib8)
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12 // C
	movq	ARG10, %r13 // km 
	movq	ARG11, %r14 // kn 

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X8_VS_LIB8
#else
	CALL(inner_scale_ab_8x8_vs_lib8)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // km 
	movq	ARG11, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_8X8_VS_LIB8
#else
	CALL(inner_store_8x8_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_tt_8x8_vs_lib8)





//                                   1      2              3            4          5        6          7             8         9          10       11        12         13       14      15      16      17
// void kernel_dgemm_tt_8x8_gen_lib8(int k, double *alpha, int offsetA, double *A, int sda, double *B, double *beta, int offC, double *C, int sdc, int offD, double *D, int sdd, int m0, int m1, int n0, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_tt_8x8_gen_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG6, %r11  // B
	movq	ARG4, %r12  // A
	movq	ARG5, %r13 // sda
	sall	$ 6, %r13d // 8*sda*sizeof(double)

	movq	ARG3, %r14 // offsetA

	// TODO vs
#if MACRO_LEVEL>=1
	INNER_EDGE_DGEMM_NN_8X8_LIB8
#else
	CALL(inner_edge_dgemm_nn_8x8_lib8)
#endif

	movq	ARG17, %r14 // n1
	movq	ARG15, %r15 // m1

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_8X8_VS_LIB8
#else
	CALL(inner_kernel_dgemm_nn_8x8_vs_lib8)
#endif

#if MACRO_LEVEL>=1
	INNER_TRAN_8X8_LIB8
#else
	CALL(inner_tran_8x8_lib8)
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12 // offsetC
	movq	ARG9, %r13 // C
	movq	ARG10, %r14 // sdc
	sall	$ 6, %r14d // 4*sdc*sizeof(double)
	movq	ARG16, %r15 // n0
	movq	ARG17, %rax // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X8_GEN_LIB8
#else
	CALL(inner_scale_ab_8x8_gen_lib8)
#endif


	// store n

	movq	ARG11, %r10 // offsetD
	movq	ARG12, %r11 // D
	movq	ARG13, %r12 // sdd
	sall	$ 6, %r12d // 4*sdb*sizeof(double)
	movq	ARG14, %r13 // m0
	movq	ARG15, %r14 // m1
	movq	ARG16, %r15 // n0
	movq	ARG17, %rax // n1

#if MACRO_LEVEL>=1
	INNER_STORE_8X8_GEN_LIB8
#else
	CALL(inner_store_8x8_gen_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_tt_8x8_gen_lib8)





//                                 1      2              3          4          5             6          7
// void kernel_dsyrk_nt_l_8x8_lib8(int k, double *alpha, double *A, double *B, double *beta, double *C, double *D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nt_l_8x8_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB8
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib8)
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X8_LIB8
#else
	CALL(inner_scale_ab_8x8_lib8)
#endif


	// store n

	movq	ARG7, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X8_LIB8
#else
	CALL(inner_store_l_8x8_lib8)
#endif



	EPILOGUE

	ret

	FUN_END(kernel_dsyrk_nt_l_8x8_lib8)





//                                    1      2              3          4          5             6          7          8       9
// void kernel_dsyrk_nt_l_8x8_vs_lib8(int k, double *alpha, double *A, double *B, double *beta, double *C, double *D, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nt_l_8x8_vs_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B
	movq	ARG8, %r13 // m1 
	movq	ARG9, %r14 // n1

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_VS_LIB8
#else
	CALL(inner_kernel_dgemm_nt_8x8_vs_lib8)
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12   // C
	movq	ARG8, %r13 // m1 
	movq	ARG9, %r14 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X8_VS_LIB8
#else
	CALL(inner_scale_ab_8x8_vs_lib8)
#endif


	// store n

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // m1 
	movq	ARG9, %r12 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X8_VS_LIB8
#else
	CALL(inner_store_l_8x8_vs_lib8)
#endif



	EPILOGUE

	ret

	FUN_END(kernel_dsyrk_nt_l_8x8_vs_lib8)





//                                     1      2              3          4          5             6            7          8        9            10         11       12      13      14      15
// void kernel_dsyrk_nt_l_8x8_gen_lib8(int k, double *alpha, double *A, double *B, double *beta, int offsetC, double *C, int sdc, int offsetD, double *D, int sdd, int m0, int m1, int n0, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_nt_l_8x8_gen_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B
	movq	ARG13, %r13 // m1
	movq	ARG15, %r14 // n1

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_VS_LIB8
#else
	CALL(inner_kernel_dgemm_nt_8x8_vs_lib8)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12 // offsetC
	movq	ARG7, %r13 // C
	movq	ARG8, %r14 // sdc
	sall	$ 6, %r14d // 4*sdc*sizeof(double)
	movq	ARG14, %r15 // n0
	movq	ARG15, %rax // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X8_GEN_LIB8
#else
	CALL(inner_scale_ab_8x8_gen_lib8)
#endif


	// store n gen

	movq	ARG9, %r10 // offsetD
	movq	ARG10, %r11 // D
	movq	ARG11, %r12 // sdd
	sall	$ 6, %r12d // 4*sdb*sizeof(double)
	movq	ARG12, %r13 // m0
	movq	ARG13, %r14 // m1
	movq	ARG14, %r15 // n0
	movq	ARG15, %rax // n1

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X8_GEN_LIB8
#else
	CALL(inner_store_l_8x8_gen_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dsyrk_nt_l_8x8_gen_lib8)





//                                  1      2               3         4            5          6        7
// void kernel_dtrmm_nn_rl_8x8_lib8(int k, double *alpha, double *A, int offsetB, double *B, int sdb, double *D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nn_rl_8x8_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// initial triangle

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG5, %r12 // B
	movq	ARG6, %r13 // sdb
	sall	$ 6, %r13d // 4*sdb*sizeof(double)
	movq	ARG4, %r14 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NN_RL_8X8_LIB8
#else
	CALL(inner_edge_dtrmm_nn_rl_8x8_lib8)
#endif

#if MACRO_LEVEL>=1
	INNER_EDGE_DGEMM_NN_8X8_LIB8
#else
	CALL(inner_edge_dgemm_nn_8x8_lib8)
#endif

	// call inner dgemm kernel nt after initial triangle

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_8X8_LIB8
#else
	CALL(inner_kernel_dgemm_nn_8x8_lib8)
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha

#if MACRO_LEVEL>=1
	INNER_SCALE_A0_8X8_LIB8
#else
	CALL(inner_scale_a0_8x8_lib8)
#endif


	// store n

	movq	ARG7, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_8X8_LIB8
#else
	CALL(inner_store_8x8_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nn_rl_8x8_lib8)





//                                     1      2               3         4            5          6        7          8       9
// void kernel_dtrmm_nn_rl_8x8_vs_lib8(int k, double *alpha, double *A, int offsetB, double *B, int sdb, double *D, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nn_rl_8x8_vs_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// initial triangle

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG5, %r12 // B
	movq	ARG6, %r13 // sdb
	sall	$ 6, %r13d // 4*sdb*sizeof(double)

	movq	ARG4, %r14 // offsetB

	// TODO vs
#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NN_RL_8X8_LIB8
#else
	CALL(inner_edge_dtrmm_nn_rl_8x8_lib8)
#endif

	// TODO vs
#if MACRO_LEVEL>=1
	INNER_EDGE_DGEMM_NN_8X8_LIB8
#else
	CALL(inner_edge_dgemm_nn_8x8_lib8)
#endif

	// call inner dgemm kernel nn after initial triangle

	movq	ARG8, %r14 // m1
	movq	ARG9, %r15 // n1

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_8X8_VS_LIB8
#else
	CALL(inner_kernel_dgemm_nn_8x8_vs_lib8)
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha

#if MACRO_LEVEL>=1
	INNER_SCALE_A0_8X8_LIB8
#else
	CALL(inner_scale_a0_8x8_lib8)
#endif


	// store n

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // m1
	movq	ARG9, %r12 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_8X8_VS_LIB8
#else
	CALL(inner_store_8x8_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nn_rl_8x8_vs_lib8)





//                                      1      2              3          4            5          6        7            8          9       10       11      12      13
// void kernel_dtrmm_nn_rl_8x8_gen_lib8(int k, double *alpha, double *A, int offsetB, double *B, int sdb, int offsetD, double *D, int sdd, int m0, int m1, int n0, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmm_nn_rl_8x8_gen_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// initial triangle

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG5, %r12 // B
	movq	ARG6, %r13 // sdb
	sall	$ 6, %r13d // 4*sdb*sizeof(double)

	movq	ARG4, %r14 // offsetB

	// TODO vs
#if MACRO_LEVEL>=1
	INNER_EDGE_DTRMM_NN_RL_8X8_LIB8
#else
	CALL(inner_edge_dtrmm_nn_rl_8x8_lib8)
#endif

	// TODO vs
#if MACRO_LEVEL>=1
	INNER_EDGE_DGEMM_NN_8X8_LIB8
#else
	CALL(inner_edge_dgemm_nn_8x8_lib8)
#endif

	// call inner dgemm kernel nn after initial triangle

	movq	ARG11, %r14 // m1
	movq	ARG13, %r15 // n1

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_8X8_VS_LIB8
#else
	CALL(inner_kernel_dgemm_nn_8x8_vs_lib8)
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha

#if MACRO_LEVEL>=1
	INNER_SCALE_A0_8X8_LIB8
#else
	CALL(inner_scale_a0_8x8_lib8)
#endif


	// store n

	movq	ARG7, %r10 // offsetD
	movq	ARG8, %r11 // D
	movq	ARG9, %r12 // sdd
	sall	$ 6, %r12d // 4*sdb*sizeof(double)
	movq	ARG10, %r13 // m0
	movq	ARG11, %r14 // m1
	movq	ARG12, %r15 // n0
	movq	ARG13, %rax // n1

#if MACRO_LEVEL>=1
	INNER_STORE_8X8_GEN_LIB8
#else
	CALL(inner_store_8x8_gen_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmm_nn_rl_8x8_gen_lib8)





//                                      1      2          3          4             5          6          7          8
// void kernel_dtrsm_nt_rl_inv_8x8_lib8(int k, double *A, double *B, double *beta, double *C, double *D, double *E, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_inv_8x8_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB8
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib8)
#endif


	// call inner blender_loader nn

	movq	ARG4, %r10 // C
	movq	ARG5, %r11 // beta

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_8X8_LIB8
#else
	CALL(inner_scale_m1b_8x8_lib8)
#endif


	// solve

	movq	ARG7, %r10  // E 
	movq	ARG8, %r11  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_8X8_LIB8
#else
	CALL(inner_edge_dtrsm_rlt_inv_8x8_lib8)
#endif


	// store

	movq	ARG6, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_8X8_LIB8
#else
	CALL(inner_store_8x8_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_8x8_lib8)





//                                         1      2          3          4             5          6          7          8                   9       10
// void kernel_dtrsm_nt_rl_inv_8x8_vs_lib8(int k, double *A, double *B, double *beta, double *C, double *D, double *E, double *inv_diag_E, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_inv_8x8_vs_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // B
	movq	ARG9, %r13 // m1
	movq	ARG10, %r14 // n1

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_VS_LIB8
#else
	CALL(inner_kernel_dgemm_nt_8x8_vs_lib8)
#endif


	// call inner blender_loader nn

	movq	ARG4, %r10 // C
	movq	ARG5, %r11 // beta
	movq	ARG9, %r12 // m1
	movq	ARG10, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_8X8_VS_LIB8
#else
	CALL(inner_scale_m1b_8x8_vs_lib8)
#endif


	// solve

	movq	ARG7, %r10  // E 
	movq	ARG8, %r11  // inv_diag_E 
	movq	ARG10, %r12 // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_8X8_VS_LIB8
#else
	CALL(inner_edge_dtrsm_rlt_inv_8x8_vs_lib8)
#endif


	// store

	movq	ARG6, %r10 // D
	movq	ARG9, %r11 // m1
	movq	ARG10, %r12 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_8X8_VS_LIB8
#else
	CALL(inner_store_8x8_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_8x8_vs_lib8)





//                                            1       2           3           4       5           6           7          8          9          10
// void kernel_dgemm_dtrsm_nt_rl_inv_8x8_lib8(int kp, double *Ap, double *Bp, int km, double *Am, double *Bm, double *C, double *D, double *E, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_dtrsm_nt_rl_inv_8x8_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG2, %r11  // Ap
	movq	ARG3, %r12  // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB8
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib8)
#endif


	// change sign
	NEG_ACC


	// call inner dgemm kernel nt sub

	movq	ARG4, %r10 // km
	movq	ARG5, %r11   // Am
	movq	ARG6, %r12   // Bm

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB8
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib8)
#endif


	// call inner blender_loader nn

	movq	ARG7, %r10   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_8X8_LIB8
#else
	CALL(inner_scale_m11_8x8_lib8)
#endif


	// solve

	movq	ARG9, %r10  // E 
	movq	ARG10, %r11  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_8X8_LIB8
#else
	CALL(inner_edge_dtrsm_rlt_inv_8x8_lib8)
#endif


	// store

	movq	ARG8, %r10   // D

#if MACRO_LEVEL>=1
	INNER_STORE_8X8_LIB8
#else
	CALL(inner_store_8x8_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_dtrsm_nt_rl_inv_8x8_lib8)





//                                               1       2           3           4       5           6           7          8          9          10                  11      12
// void kernel_dgemm_dtrsm_nt_rl_inv_8x8_vs_lib8(int kp, double *Ap, double *Bp, int km, double *Am, double *Bm, double *C, double *D, double *E, double *inv_diag_E, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_dtrsm_nt_rl_inv_8x8_vs_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG2, %r11  // Ap
	movq	ARG3, %r12  // Bp
	movq	ARG11, %r13   // m1
	movq	ARG12, %r14   // n1

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_VS_LIB8
#else
	CALL(inner_kernel_dgemm_nt_8x8_vs_lib8)
#endif


	// change sign
	NEG_ACC


	// call inner dgemm kernel nt sub

	movq	ARG4, %r10 // km
	movq	ARG5, %r11   // Am
	movq	ARG6, %r12   // Bm
	movq	ARG11, %r13   // m1
	movq	ARG12, %r14   // n1

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_VS_LIB8
#else
	CALL(inner_kernel_dgemm_nt_8x8_vs_lib8)
#endif


	// call inner blender_loader nn

	movq	ARG7, %r10   // C
	movq	ARG11, %r11   // m1
	movq	ARG12, %r12   // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_8X8_VS_LIB8
#else
	CALL(inner_scale_m11_8x8_vs_lib8)
#endif


	// solve

	movq	ARG9, %r10  // E 
	movq	ARG10, %r11  // inv_diag_E 
	movq	ARG12, %r12   // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_8X8_VS_LIB8
#else
	CALL(inner_edge_dtrsm_rlt_inv_8x8_vs_lib8)
#endif


	// store

	movq	ARG8, %r10   // D
	movq	ARG11, %r11   // m1
	movq	ARG12, %r12   // n1

#if MACRO_LEVEL>=1
	INNER_STORE_8X8_VS_LIB8
#else
	CALL(inner_store_8x8_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_dtrsm_nt_rl_inv_8x8_vs_lib8)





//                                  1      2          3          4          5          6
// void kernel_dpotrf_nt_l_8x8_lib8(int k, double *A, double *B, double *C, double *D, double *inv_diag_D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpotrf_nt_l_8x8_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB8
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib8)
#endif


	// call inner blender_loader nn

	movq	ARG4, %r10 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_8X8_LIB8
#else
	CALL(inner_scale_m11_8x8_lib8)
#endif


	// factorization

	movq	ARG6, %r10  // inv_diag_D 

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_8X8_LIB8
#else
	CALL(inner_edge_dpotrf_8x8_lib8)
#endif


	// store

	movq	ARG5, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X8_LIB8
#else
	CALL(inner_store_l_8x8_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dpotrf_nt_l_8x8_lib8)





//                                     1      2          3          4          5          6                   7       8
// void kernel_dpotrf_nt_l_8x8_vs_lib8(int k, double *A, double *B, double *C, double *D, double *inv_diag_D, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dpotrf_nt_l_8x8_vs_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	movq	ARG7, %r13 // m1
	movq	ARG8, %r14 // n1

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_VS_LIB8
#else
	CALL(inner_kernel_dgemm_nt_8x8_vs_lib8)
#endif


	// call inner blender_loader nn

	movq	ARG4, %r10 // C
	movq	ARG7, %r11 // m1
	movq	ARG8, %r12 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_8X8_VS_LIB8
#else
	CALL(inner_scale_m11_8x8_vs_lib8)
#endif


	// factorization

	movq	ARG6, %r10  // inv_diag_D 
	movq	ARG7, %r11 // m1
	movq	ARG8, %r12 // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_8X8_VS_LIB8
#else
	CALL(inner_edge_dpotrf_8x8_vs_lib8)
#endif


	// store

	movq	ARG5, %r10 // D
	movq	ARG7, %r11 // m1
	movq	ARG8, %r12 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X8_VS_LIB8
#else
	CALL(inner_store_l_8x8_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dpotrf_nt_l_8x8_vs_lib8)





//                                        1       2           3           4       5           6           7          8          9
// void kernel_dsyrk_dpotrf_nt_l_8x8_lib8(int kp, double *Ap, double *Bp, int km, double *Am, double *Bm, double *C, double *D, double *inv_diag_D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_dpotrf_nt_l_8x8_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG2, %r11  // Ap
	movq	ARG3, %r12  // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB8
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib8)
#endif


	// change sing
	NEG_ACC


	// call inner dgemm kernel nt sub

	movq	ARG4, %r10 // km
	movq	ARG5, %r11   // Am
	movq	ARG6, %r12   // Bm

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB8
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib8)
#endif


	// call inner blender_loader nn

	movq	ARG7, %r10   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_8X8_LIB8
#else
	CALL(inner_scale_m11_8x8_lib8)
#endif


	// factorization

	movq	ARG9, %r10  // inv_diag_D 

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_8X8_LIB8
#else
	CALL(inner_edge_dpotrf_8x8_lib8)
#endif


	// store

	movq	ARG8, %r10  // D 

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X8_LIB8
#else
	CALL(inner_store_l_8x8_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dsyrk_dpotrf_nt_l_8x8_lib8)





//                                           1       2           3           4       5           6           7          8          9                   10      11
// void kernel_dsyrk_dpotrf_nt_l_8x8_vs_lib8(int kp, double *Ap, double *Bp, int km, double *Am, double *Bm, double *C, double *D, double *inv_diag_D, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsyrk_dpotrf_nt_l_8x8_vs_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG2, %r11  // Ap
	movq	ARG3, %r12  // Bp
	movq	ARG10, %r13  // m1
	movq	ARG11, %r14  // n1

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_VS_LIB8
#else
	CALL(inner_kernel_dgemm_nt_8x8_vs_lib8)
#endif


	// change sing
	NEG_ACC


	// call inner dgemm kernel nt sub

	movq	ARG4, %r10 // km
	movq	ARG5, %r11   // Am
	movq	ARG6, %r12   // Bm
	movq	ARG10, %r13  // m1
	movq	ARG11, %r14  // n1

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_VS_LIB8
#else
	CALL(inner_kernel_dgemm_nt_8x8_vs_lib8)
#endif


	// call inner blender_loader nn

	movq	ARG7, %r10   // C
	movq	ARG10, %r11  // m1
	movq	ARG11, %r12  // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_8X8_VS_LIB8
#else
	CALL(inner_scale_m11_8x8_vs_lib8)
#endif


	// factorization

	movq	ARG9, %r10  // inv_diag_D 
	movq	ARG10, %r11  // m1
	movq	ARG11, %r12  // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_DPOTRF_8X8_VS_LIB8
#else
	CALL(inner_edge_dpotrf_8x8_vs_lib8)
#endif


	// store

	movq	ARG8, %r10  // D
	movq	ARG10, %r11  // m1
	movq	ARG11, %r12  // n1

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X8_VS_LIB8
#else
	CALL(inner_store_l_8x8_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dsyrk_dpotrf_nt_l_8x8_vs_lib8)





//                               1         2           3           4
// void kernel_dlarfb8_rn_8_lib8(int kmax, double *pV, double *pT, double *pD);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dlarfb8_rn_8_lib8)
	
	PROLOGUE

	// zero accumulation registers

//	ZERO_ACC

	movq	ARG1, %r10 // k
	movq	ARG4, %r11 // D
	movq	ARG2, %r12 // V

	//
	vmovapd			0*64(%r11), %zmm0
	//
	vmovapd			1*64(%r11), %zmm1
	vbroadcastsd	1*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm1, %zmm0
	//
	vmovapd			2*64(%r11), %zmm2
	vbroadcastsd	0+2*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm2, %zmm0
	vbroadcastsd	8+2*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm2, %zmm1
	//
	vmovapd			3*64(%r11), %zmm3
	vbroadcastsd	0+3*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm3, %zmm0
	vbroadcastsd	8+3*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm3, %zmm1
	vbroadcastsd	16+3*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm3, %zmm2
	//
	vmovapd			4*64(%r11), %zmm4
	vbroadcastsd	0+4*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm4, %zmm0
	vbroadcastsd	8+4*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm4, %zmm1
	vbroadcastsd	16+4*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm4, %zmm2
	vbroadcastsd	24+4*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm4, %zmm3
	//
	vmovapd			5*64(%r11), %zmm5
	vbroadcastsd	0+5*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm5, %zmm0
	vbroadcastsd	8+5*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm5, %zmm1
	vbroadcastsd	16+5*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm5, %zmm2
	vbroadcastsd	24+5*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm5, %zmm3
	vbroadcastsd	32+5*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm5, %zmm4
	//
	vmovapd			6*64(%r11), %zmm6
	vbroadcastsd	0+6*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm6, %zmm0
	vbroadcastsd	8+6*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm6, %zmm1
	vbroadcastsd	16+6*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm6, %zmm2
	vbroadcastsd	24+6*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm6, %zmm3
	vbroadcastsd	32+6*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm6, %zmm4
	vbroadcastsd	40+6*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm6, %zmm5
	//
	vmovapd			7*64(%r11), %zmm7
	vbroadcastsd	0+7*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm7, %zmm0
	vbroadcastsd	8+7*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm7, %zmm1
	vbroadcastsd	16+7*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm7, %zmm2
	vbroadcastsd	24+7*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm7, %zmm3
	vbroadcastsd	32+7*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm7, %zmm4
	vbroadcastsd	40+7*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm7, %zmm5
	vbroadcastsd	48+7*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm7, %zmm6

	subl	$ 8, %r10d
	addq	$ 512, %r11
	addq	$ 512, %r12

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB8
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib8)
#endif

	movq	ARG3, %r10 // T

	//
	vbroadcastsd	56+7*64(%r10), %zmm24
	vmulpd			%zmm7, %zmm24, %zmm7
	//
	vbroadcastsd	48+7*64(%r10), %zmm24
	vfmadd231pd		%zmm6, %zmm24, %zmm7
	vbroadcastsd	48+6*64(%r10), %zmm24
	vmulpd			%zmm6, %zmm24, %zmm6
	//
	vbroadcastsd	40+7*64(%r10), %zmm24
	vfmadd231pd		%zmm5, %zmm24, %zmm7
	vbroadcastsd	40+6*64(%r10), %zmm24
	vfmadd231pd		%zmm5, %zmm24, %zmm6
	vbroadcastsd	40+5*64(%r10), %zmm24
	vmulpd			%zmm5, %zmm24, %zmm5
	//
	vbroadcastsd	32+7*64(%r10), %zmm24
	vfmadd231pd		%zmm4, %zmm24, %zmm7
	vbroadcastsd	32+6*64(%r10), %zmm24
	vfmadd231pd		%zmm4, %zmm24, %zmm6
	vbroadcastsd	32+5*64(%r10), %zmm24
	vfmadd231pd		%zmm4, %zmm24, %zmm5
	vbroadcastsd	32+4*64(%r10), %zmm24
	vmulpd			%zmm4, %zmm24, %zmm4
	//
	vbroadcastsd	24+7*64(%r10), %zmm24
	vfmadd231pd		%zmm3, %zmm24, %zmm7
	vbroadcastsd	24+6*64(%r10), %zmm24
	vfmadd231pd		%zmm3, %zmm24, %zmm6
	vbroadcastsd	24+5*64(%r10), %zmm24
	vfmadd231pd		%zmm3, %zmm24, %zmm5
	vbroadcastsd	24+4*64(%r10), %zmm24
	vfmadd231pd		%zmm3, %zmm24, %zmm4
	vbroadcastsd	24+3*64(%r10), %zmm24
	vmulpd			%zmm3, %zmm24, %zmm3
	//
	vbroadcastsd	16+7*64(%r10), %zmm24
	vfmadd231pd		%zmm2, %zmm24, %zmm7
	vbroadcastsd	16+6*64(%r10), %zmm24
	vfmadd231pd		%zmm2, %zmm24, %zmm6
	vbroadcastsd	16+5*64(%r10), %zmm24
	vfmadd231pd		%zmm2, %zmm24, %zmm5
	vbroadcastsd	16+4*64(%r10), %zmm24
	vfmadd231pd		%zmm2, %zmm24, %zmm4
	vbroadcastsd	16+3*64(%r10), %zmm24
	vfmadd231pd		%zmm2, %zmm24, %zmm3
	vbroadcastsd	16+2*64(%r10), %zmm24
	vmulpd			%zmm2, %zmm24, %zmm2
	//
	vbroadcastsd	8+7*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm7
	vbroadcastsd	8+6*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm6
	vbroadcastsd	8+5*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm5
	vbroadcastsd	8+4*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm4
	vbroadcastsd	8+3*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm3
	vbroadcastsd	8+2*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm2
	vbroadcastsd	8+1*64(%r10), %zmm24
	vmulpd			%zmm1, %zmm24, %zmm1
	//
	vbroadcastsd	0+7*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm7
	vbroadcastsd	0+6*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm6
	vbroadcastsd	0+5*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm5
	vbroadcastsd	0+4*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm4
	vbroadcastsd	0+3*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm3
	vbroadcastsd	0+2*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm2
	vbroadcastsd	0+1*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm1
	vbroadcastsd	0+0*64(%r10), %zmm24
	vmulpd			%zmm0, %zmm24, %zmm0

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // V
	movq	ARG4, %r12 // D

	//
	vmovapd			0+0*64(%r12), %zmm24
	vaddpd			%zmm24, %zmm0, %zmm24
	vmovapd			%zmm24, 0+0*64(%r12)
	//
	vmovapd			0+1*64(%r12), %zmm24
	vbroadcastsd	0+1*64(%r11), %zmm25
	vfmadd231pd		%zmm0, %zmm25, %zmm24
	vaddpd			%zmm24, %zmm1, %zmm24
	vmovapd			%zmm24, 0+1*64(%r12)
	//
	vmovapd			0+2*64(%r12), %zmm24
	vbroadcastsd	0+2*64(%r11), %zmm25
	vfmadd231pd		%zmm0, %zmm25, %zmm24
	vbroadcastsd	8+2*64(%r11), %zmm25
	vfmadd231pd		%zmm1, %zmm25, %zmm24
	vaddpd			%zmm24, %zmm2, %zmm24
	vmovapd			%zmm24, 0+2*64(%r12)
	//
	vmovapd			0+3*64(%r12), %zmm24
	vbroadcastsd	0+3*64(%r11), %zmm25
	vfmadd231pd		%zmm0, %zmm25, %zmm24
	vbroadcastsd	8+3*64(%r11), %zmm25
	vfmadd231pd		%zmm1, %zmm25, %zmm24
	vbroadcastsd	16+3*64(%r11), %zmm25
	vfmadd231pd		%zmm2, %zmm25, %zmm24
	vaddpd			%zmm24, %zmm3, %zmm24
	vmovapd			%zmm24, 0+3*64(%r12)
	//
	vmovapd			0+4*64(%r12), %zmm24
	vbroadcastsd	0+4*64(%r11), %zmm25
	vfmadd231pd		%zmm0, %zmm25, %zmm24
	vbroadcastsd	8+4*64(%r11), %zmm25
	vfmadd231pd		%zmm1, %zmm25, %zmm24
	vbroadcastsd	16+4*64(%r11), %zmm25
	vfmadd231pd		%zmm2, %zmm25, %zmm24
	vbroadcastsd	24+4*64(%r11), %zmm25
	vfmadd231pd		%zmm3, %zmm25, %zmm24
	vaddpd			%zmm24, %zmm4, %zmm24
	vmovapd			%zmm24, 0+4*64(%r12)
	//
	vmovapd			0+5*64(%r12), %zmm24
	vbroadcastsd	0+5*64(%r11), %zmm25
	vfmadd231pd		%zmm0, %zmm25, %zmm24
	vbroadcastsd	8+5*64(%r11), %zmm25
	vfmadd231pd		%zmm1, %zmm25, %zmm24
	vbroadcastsd	16+5*64(%r11), %zmm25
	vfmadd231pd		%zmm2, %zmm25, %zmm24
	vbroadcastsd	24+5*64(%r11), %zmm25
	vfmadd231pd		%zmm3, %zmm25, %zmm24
	vbroadcastsd	32+5*64(%r11), %zmm25
	vfmadd231pd		%zmm4, %zmm25, %zmm24
	vaddpd			%zmm24, %zmm5, %zmm24
	vmovapd			%zmm24, 0+5*64(%r12)
	//
	vmovapd			0+6*64(%r12), %zmm24
	vbroadcastsd	0+6*64(%r11), %zmm25
	vfmadd231pd		%zmm0, %zmm25, %zmm24
	vbroadcastsd	8+6*64(%r11), %zmm25
	vfmadd231pd		%zmm1, %zmm25, %zmm24
	vbroadcastsd	16+6*64(%r11), %zmm25
	vfmadd231pd		%zmm2, %zmm25, %zmm24
	vbroadcastsd	24+6*64(%r11), %zmm25
	vfmadd231pd		%zmm3, %zmm25, %zmm24
	vbroadcastsd	32+6*64(%r11), %zmm25
	vfmadd231pd		%zmm4, %zmm25, %zmm24
	vbroadcastsd	40+6*64(%r11), %zmm25
	vfmadd231pd		%zmm5, %zmm25, %zmm24
	vaddpd			%zmm24, %zmm6, %zmm24
	vmovapd			%zmm24, 0+6*64(%r12)
	//
	vmovapd			0+7*64(%r12), %zmm24
	vbroadcastsd	0+7*64(%r11), %zmm25
	vfmadd231pd		%zmm0, %zmm25, %zmm24
	vbroadcastsd	8+7*64(%r11), %zmm25
	vfmadd231pd		%zmm1, %zmm25, %zmm24
	vbroadcastsd	16+7*64(%r11), %zmm25
	vfmadd231pd		%zmm2, %zmm25, %zmm24
	vbroadcastsd	24+7*64(%r11), %zmm25
	vfmadd231pd		%zmm3, %zmm25, %zmm24
	vbroadcastsd	32+7*64(%r11), %zmm25
	vfmadd231pd		%zmm4, %zmm25, %zmm24
	vbroadcastsd	40+7*64(%r11), %zmm25
	vfmadd231pd		%zmm5, %zmm25, %zmm24
	vbroadcastsd	48+7*64(%r11), %zmm25
	vfmadd231pd		%zmm6, %zmm25, %zmm24
	vaddpd			%zmm24, %zmm7, %zmm24
	vmovapd			%zmm24, 0+7*64(%r12)

	subl	$ 8, %r10d
	addq	$ 512, %r11
	addq	$ 512, %r12

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEBP_NN_8X8_LIB8
#else
	CALL(inner_kernel_dgebp_nn_8x8_lib8)
#endif

	EPILOGUE

	ret

	FUN_END(kernel_dlarfb8_rn_8_lib8)





//                                  1         2           3           4           5
// void kernel_dlarfb8_rn_8_vs_lib8(int kmax, double *pV, double *pT, double *pD, int m1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dlarfb8_rn_8_vs_lib8)
	
	PROLOGUE


	movq	ARG5, %r10 // k

	// compute mask for rows
	vcvtsi2sd	%r10d, %xmm25, %xmm25
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC00(%rip), %zmm24
#elif defined(OS_MAC)
	vmovupd		LC00(%rip), %zmm24
#endif
	vbroadcastsd	%xmm25, %zmm25
	vsubpd		%zmm25, %zmm24, %zmm25
	vpmovq2m	%zmm25, %k1


	// zero accumulation registers

//	ZERO_ACC

	movq	ARG1, %r10 // k
	movq	ARG4, %r11 // D
	movq	ARG2, %r12 // V

	//
	vmovapd			0*64(%r11), %zmm0 {%k1}{z}
	//
	vmovapd			1*64(%r11), %zmm1 {%k1}{z}
	vbroadcastsd	1*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm1, %zmm0
	//
	vmovapd			2*64(%r11), %zmm2 {%k1}{z}
	vbroadcastsd	0+2*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm2, %zmm0
	vbroadcastsd	8+2*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm2, %zmm1
	//
	vmovapd			3*64(%r11), %zmm3 {%k1}{z}
	vbroadcastsd	0+3*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm3, %zmm0
	vbroadcastsd	8+3*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm3, %zmm1
	vbroadcastsd	16+3*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm3, %zmm2
	//
	vmovapd			4*64(%r11), %zmm4 {%k1}{z}
	vbroadcastsd	0+4*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm4, %zmm0
	vbroadcastsd	8+4*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm4, %zmm1
	vbroadcastsd	16+4*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm4, %zmm2
	vbroadcastsd	24+4*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm4, %zmm3
	//
	vmovapd			5*64(%r11), %zmm5 {%k1}{z}
	vbroadcastsd	0+5*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm5, %zmm0
	vbroadcastsd	8+5*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm5, %zmm1
	vbroadcastsd	16+5*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm5, %zmm2
	vbroadcastsd	24+5*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm5, %zmm3
	vbroadcastsd	32+5*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm5, %zmm4
	//
	vmovapd			6*64(%r11), %zmm6 {%k1}{z}
	vbroadcastsd	0+6*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm6, %zmm0
	vbroadcastsd	8+6*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm6, %zmm1
	vbroadcastsd	16+6*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm6, %zmm2
	vbroadcastsd	24+6*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm6, %zmm3
	vbroadcastsd	32+6*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm6, %zmm4
	vbroadcastsd	40+6*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm6, %zmm5
	//
	vmovapd			7*64(%r11), %zmm7 {%k1}{z}
	vbroadcastsd	0+7*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm7, %zmm0
	vbroadcastsd	8+7*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm7, %zmm1
	vbroadcastsd	16+7*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm7, %zmm2
	vbroadcastsd	24+7*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm7, %zmm3
	vbroadcastsd	32+7*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm7, %zmm4
	vbroadcastsd	40+7*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm7, %zmm5
	vbroadcastsd	48+7*64(%r12), %zmm24
	vfmadd231pd		%zmm24, %zmm7, %zmm6

	subl	$ 8, %r10d
	addq	$ 512, %r11
	addq	$ 512, %r12

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_VX8_LIB8
#else
	CALL(inner_kernel_dgemm_nt_vx8_lib8)
#endif

	movq	ARG3, %r10 // T

	//
	vbroadcastsd	56+7*64(%r10), %zmm24
	vmulpd			%zmm7, %zmm24, %zmm7
	//
	vbroadcastsd	48+7*64(%r10), %zmm24
	vfmadd231pd		%zmm6, %zmm24, %zmm7
	vbroadcastsd	48+6*64(%r10), %zmm24
	vmulpd			%zmm6, %zmm24, %zmm6
	//
	vbroadcastsd	40+7*64(%r10), %zmm24
	vfmadd231pd		%zmm5, %zmm24, %zmm7
	vbroadcastsd	40+6*64(%r10), %zmm24
	vfmadd231pd		%zmm5, %zmm24, %zmm6
	vbroadcastsd	40+5*64(%r10), %zmm24
	vmulpd			%zmm5, %zmm24, %zmm5
	//
	vbroadcastsd	32+7*64(%r10), %zmm24
	vfmadd231pd		%zmm4, %zmm24, %zmm7
	vbroadcastsd	32+6*64(%r10), %zmm24
	vfmadd231pd		%zmm4, %zmm24, %zmm6
	vbroadcastsd	32+5*64(%r10), %zmm24
	vfmadd231pd		%zmm4, %zmm24, %zmm5
	vbroadcastsd	32+4*64(%r10), %zmm24
	vmulpd			%zmm4, %zmm24, %zmm4
	//
	vbroadcastsd	24+7*64(%r10), %zmm24
	vfmadd231pd		%zmm3, %zmm24, %zmm7
	vbroadcastsd	24+6*64(%r10), %zmm24
	vfmadd231pd		%zmm3, %zmm24, %zmm6
	vbroadcastsd	24+5*64(%r10), %zmm24
	vfmadd231pd		%zmm3, %zmm24, %zmm5
	vbroadcastsd	24+4*64(%r10), %zmm24
	vfmadd231pd		%zmm3, %zmm24, %zmm4
	vbroadcastsd	24+3*64(%r10), %zmm24
	vmulpd			%zmm3, %zmm24, %zmm3
	//
	vbroadcastsd	16+7*64(%r10), %zmm24
	vfmadd231pd		%zmm2, %zmm24, %zmm7
	vbroadcastsd	16+6*64(%r10), %zmm24
	vfmadd231pd		%zmm2, %zmm24, %zmm6
	vbroadcastsd	16+5*64(%r10), %zmm24
	vfmadd231pd		%zmm2, %zmm24, %zmm5
	vbroadcastsd	16+4*64(%r10), %zmm24
	vfmadd231pd		%zmm2, %zmm24, %zmm4
	vbroadcastsd	16+3*64(%r10), %zmm24
	vfmadd231pd		%zmm2, %zmm24, %zmm3
	vbroadcastsd	16+2*64(%r10), %zmm24
	vmulpd			%zmm2, %zmm24, %zmm2
	//
	vbroadcastsd	8+7*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm7
	vbroadcastsd	8+6*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm6
	vbroadcastsd	8+5*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm5
	vbroadcastsd	8+4*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm4
	vbroadcastsd	8+3*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm3
	vbroadcastsd	8+2*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm2
	vbroadcastsd	8+1*64(%r10), %zmm24
	vmulpd			%zmm1, %zmm24, %zmm1
	//
	vbroadcastsd	0+7*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm7
	vbroadcastsd	0+6*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm6
	vbroadcastsd	0+5*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm5
	vbroadcastsd	0+4*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm4
	vbroadcastsd	0+3*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm3
	vbroadcastsd	0+2*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm2
	vbroadcastsd	0+1*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm1
	vbroadcastsd	0+0*64(%r10), %zmm24
	vmulpd			%zmm0, %zmm24, %zmm0

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // V
	movq	ARG4, %r12 // D

	//
	vmovapd			0+0*64(%r12), %zmm24 {%k1}{z}
	vaddpd			%zmm24, %zmm0, %zmm24
	vmovapd			%zmm24, 0+0*64(%r12) {%k1}
	//
	vmovapd			0+1*64(%r12), %zmm24 {%k1}{z}
	vbroadcastsd	0+1*64(%r11), %zmm25
	vfmadd231pd		%zmm0, %zmm25, %zmm24
	vaddpd			%zmm24, %zmm1, %zmm24
	vmovapd			%zmm24, 0+1*64(%r12) {%k1}
	//
	vmovapd			0+2*64(%r12), %zmm24 {%k1}{z}
	vbroadcastsd	0+2*64(%r11), %zmm25
	vfmadd231pd		%zmm0, %zmm25, %zmm24
	vbroadcastsd	8+2*64(%r11), %zmm25
	vfmadd231pd		%zmm1, %zmm25, %zmm24
	vaddpd			%zmm24, %zmm2, %zmm24
	vmovapd			%zmm24, 0+2*64(%r12) {%k1}
	//
	vmovapd			0+3*64(%r12), %zmm24 {%k1}{z}
	vbroadcastsd	0+3*64(%r11), %zmm25
	vfmadd231pd		%zmm0, %zmm25, %zmm24
	vbroadcastsd	8+3*64(%r11), %zmm25
	vfmadd231pd		%zmm1, %zmm25, %zmm24
	vbroadcastsd	16+3*64(%r11), %zmm25
	vfmadd231pd		%zmm2, %zmm25, %zmm24
	vaddpd			%zmm24, %zmm3, %zmm24
	vmovapd			%zmm24, 0+3*64(%r12) {%k1}
	//
	vmovapd			0+4*64(%r12), %zmm24 {%k1}{z}
	vbroadcastsd	0+4*64(%r11), %zmm25
	vfmadd231pd		%zmm0, %zmm25, %zmm24
	vbroadcastsd	8+4*64(%r11), %zmm25
	vfmadd231pd		%zmm1, %zmm25, %zmm24
	vbroadcastsd	16+4*64(%r11), %zmm25
	vfmadd231pd		%zmm2, %zmm25, %zmm24
	vbroadcastsd	24+4*64(%r11), %zmm25
	vfmadd231pd		%zmm3, %zmm25, %zmm24
	vaddpd			%zmm24, %zmm4, %zmm24
	vmovapd			%zmm24, 0+4*64(%r12) {%k1}
	//
	vmovapd			0+5*64(%r12), %zmm24 {%k1}{z}
	vbroadcastsd	0+5*64(%r11), %zmm25
	vfmadd231pd		%zmm0, %zmm25, %zmm24
	vbroadcastsd	8+5*64(%r11), %zmm25
	vfmadd231pd		%zmm1, %zmm25, %zmm24
	vbroadcastsd	16+5*64(%r11), %zmm25
	vfmadd231pd		%zmm2, %zmm25, %zmm24
	vbroadcastsd	24+5*64(%r11), %zmm25
	vfmadd231pd		%zmm3, %zmm25, %zmm24
	vbroadcastsd	32+5*64(%r11), %zmm25
	vfmadd231pd		%zmm4, %zmm25, %zmm24
	vaddpd			%zmm24, %zmm5, %zmm24
	vmovapd			%zmm24, 0+5*64(%r12) {%k1}
	//
	vmovapd			0+6*64(%r12), %zmm24 {%k1}{z}
	vbroadcastsd	0+6*64(%r11), %zmm25
	vfmadd231pd		%zmm0, %zmm25, %zmm24
	vbroadcastsd	8+6*64(%r11), %zmm25
	vfmadd231pd		%zmm1, %zmm25, %zmm24
	vbroadcastsd	16+6*64(%r11), %zmm25
	vfmadd231pd		%zmm2, %zmm25, %zmm24
	vbroadcastsd	24+6*64(%r11), %zmm25
	vfmadd231pd		%zmm3, %zmm25, %zmm24
	vbroadcastsd	32+6*64(%r11), %zmm25
	vfmadd231pd		%zmm4, %zmm25, %zmm24
	vbroadcastsd	40+6*64(%r11), %zmm25
	vfmadd231pd		%zmm5, %zmm25, %zmm24
	vaddpd			%zmm24, %zmm6, %zmm24
	vmovapd			%zmm24, 0+6*64(%r12) {%k1}
	//
	vmovapd			0+7*64(%r12), %zmm24 {%k1}{z}
	vbroadcastsd	0+7*64(%r11), %zmm25
	vfmadd231pd		%zmm0, %zmm25, %zmm24
	vbroadcastsd	8+7*64(%r11), %zmm25
	vfmadd231pd		%zmm1, %zmm25, %zmm24
	vbroadcastsd	16+7*64(%r11), %zmm25
	vfmadd231pd		%zmm2, %zmm25, %zmm24
	vbroadcastsd	24+7*64(%r11), %zmm25
	vfmadd231pd		%zmm3, %zmm25, %zmm24
	vbroadcastsd	32+7*64(%r11), %zmm25
	vfmadd231pd		%zmm4, %zmm25, %zmm24
	vbroadcastsd	40+7*64(%r11), %zmm25
	vfmadd231pd		%zmm5, %zmm25, %zmm24
	vbroadcastsd	48+7*64(%r11), %zmm25
	vfmadd231pd		%zmm6, %zmm25, %zmm24
	vaddpd			%zmm24, %zmm7, %zmm24
	vmovapd			%zmm24, 0+7*64(%r12) {%k1}

	subl	$ 8, %r10d
	addq	$ 512, %r11
	addq	$ 512, %r12

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEBP_NN_VX8_LIB8
#else
	CALL(inner_kernel_dgebp_nn_vx8_lib8)
#endif

1000:

	EPILOGUE

	ret

	FUN_END(kernel_dlarfb8_rn_8_vs_lib8)





//                                  1         2            3           4           5
// void kernel_dlarfb8_rn_la_8_lib8(int kmax, double *pVA, double *pT, double *pD, double *pA);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dlarfb8_rn_la_8_lib8)
	
	PROLOGUE

	// zero accumulation registers

//	ZERO_ACC

	movq	ARG4, %r10 // D

	//
	vmovapd			0*64(%r10), %zmm0
	//
	vmovapd			1*64(%r10), %zmm1
	//
	vmovapd			2*64(%r10), %zmm2
	//
	vmovapd			3*64(%r10), %zmm3
	//
	vmovapd			4*64(%r10), %zmm4
	//
	vmovapd			5*64(%r10), %zmm5
	//
	vmovapd			6*64(%r10), %zmm6
	//
	vmovapd			7*64(%r10), %zmm7

	movq	ARG1, %r10 // k
	movq	ARG5, %r11 // A
	movq	ARG2, %r12 // VA

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB8
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib8)
#endif

	movq	ARG3, %r10 // T

	//
	vbroadcastsd	56+7*64(%r10), %zmm24
	vmulpd			%zmm7, %zmm24, %zmm7
	//
	vbroadcastsd	48+7*64(%r10), %zmm24
	vfmadd231pd		%zmm6, %zmm24, %zmm7
	vbroadcastsd	48+6*64(%r10), %zmm24
	vmulpd			%zmm6, %zmm24, %zmm6
	//
	vbroadcastsd	40+7*64(%r10), %zmm24
	vfmadd231pd		%zmm5, %zmm24, %zmm7
	vbroadcastsd	40+6*64(%r10), %zmm24
	vfmadd231pd		%zmm5, %zmm24, %zmm6
	vbroadcastsd	40+5*64(%r10), %zmm24
	vmulpd			%zmm5, %zmm24, %zmm5
	//
	vbroadcastsd	32+7*64(%r10), %zmm24
	vfmadd231pd		%zmm4, %zmm24, %zmm7
	vbroadcastsd	32+6*64(%r10), %zmm24
	vfmadd231pd		%zmm4, %zmm24, %zmm6
	vbroadcastsd	32+5*64(%r10), %zmm24
	vfmadd231pd		%zmm4, %zmm24, %zmm5
	vbroadcastsd	32+4*64(%r10), %zmm24
	vmulpd			%zmm4, %zmm24, %zmm4
	//
	vbroadcastsd	24+7*64(%r10), %zmm24
	vfmadd231pd		%zmm3, %zmm24, %zmm7
	vbroadcastsd	24+6*64(%r10), %zmm24
	vfmadd231pd		%zmm3, %zmm24, %zmm6
	vbroadcastsd	24+5*64(%r10), %zmm24
	vfmadd231pd		%zmm3, %zmm24, %zmm5
	vbroadcastsd	24+4*64(%r10), %zmm24
	vfmadd231pd		%zmm3, %zmm24, %zmm4
	vbroadcastsd	24+3*64(%r10), %zmm24
	vmulpd			%zmm3, %zmm24, %zmm3
	//
	vbroadcastsd	16+7*64(%r10), %zmm24
	vfmadd231pd		%zmm2, %zmm24, %zmm7
	vbroadcastsd	16+6*64(%r10), %zmm24
	vfmadd231pd		%zmm2, %zmm24, %zmm6
	vbroadcastsd	16+5*64(%r10), %zmm24
	vfmadd231pd		%zmm2, %zmm24, %zmm5
	vbroadcastsd	16+4*64(%r10), %zmm24
	vfmadd231pd		%zmm2, %zmm24, %zmm4
	vbroadcastsd	16+3*64(%r10), %zmm24
	vfmadd231pd		%zmm2, %zmm24, %zmm3
	vbroadcastsd	16+2*64(%r10), %zmm24
	vmulpd			%zmm2, %zmm24, %zmm2
	//
	vbroadcastsd	8+7*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm7
	vbroadcastsd	8+6*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm6
	vbroadcastsd	8+5*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm5
	vbroadcastsd	8+4*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm4
	vbroadcastsd	8+3*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm3
	vbroadcastsd	8+2*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm2
	vbroadcastsd	8+1*64(%r10), %zmm24
	vmulpd			%zmm1, %zmm24, %zmm1
	//
	vbroadcastsd	0+7*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm7
	vbroadcastsd	0+6*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm6
	vbroadcastsd	0+5*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm5
	vbroadcastsd	0+4*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm4
	vbroadcastsd	0+3*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm3
	vbroadcastsd	0+2*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm2
	vbroadcastsd	0+1*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm1
	vbroadcastsd	0+0*64(%r10), %zmm24
	vmulpd			%zmm0, %zmm24, %zmm0

	movq	ARG4, %r10 // D

	//
	vmovapd			0+0*64(%r10), %zmm24
	vaddpd			%zmm24, %zmm0, %zmm24
	vmovapd			%zmm24, 0+0*64(%r10)
	//
	vmovapd			0+1*64(%r10), %zmm24
	vaddpd			%zmm24, %zmm1, %zmm24
	vmovapd			%zmm24, 0+1*64(%r10)
	//
	vmovapd			0+2*64(%r10), %zmm24
	vaddpd			%zmm24, %zmm2, %zmm24
	vmovapd			%zmm24, 0+2*64(%r10)
	//
	vmovapd			0+3*64(%r10), %zmm24
	vaddpd			%zmm24, %zmm3, %zmm24
	vmovapd			%zmm24, 0+3*64(%r10)
	//
	vmovapd			0+4*64(%r10), %zmm24
	vaddpd			%zmm24, %zmm4, %zmm24
	vmovapd			%zmm24, 0+4*64(%r10)
	//
	vmovapd			0+5*64(%r10), %zmm24
	vaddpd			%zmm24, %zmm5, %zmm24
	vmovapd			%zmm24, 0+5*64(%r10)
	//
	vmovapd			0+6*64(%r10), %zmm24
	vaddpd			%zmm24, %zmm6, %zmm24
	vmovapd			%zmm24, 0+6*64(%r10)
	//
	vmovapd			0+7*64(%r10), %zmm24
	vaddpd			%zmm24, %zmm7, %zmm24
	vmovapd			%zmm24, 0+7*64(%r10)

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // VA
	movq	ARG5, %r12 // A

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEBP_NN_8X8_LIB8
#else
	CALL(inner_kernel_dgebp_nn_8x8_lib8)
#endif

	EPILOGUE

	ret

	FUN_END(kernel_dlarfb8_rn_la_8_lib8)





//                                     1         2            3           4           5           6
// void kernel_dlarfb8_rn_la_8_vs_lib8(int kmax, double *pVA, double *pT, double *pD, double *pA, int m1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dlarfb8_rn_la_8_vs_lib8)
	
	PROLOGUE


	movq	ARG6, %r10 // k

	// compute mask for rows
	vcvtsi2sd	%r10d, %xmm25, %xmm25
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC00(%rip), %zmm24
#elif defined(OS_MAC)
	vmovupd		LC00(%rip), %zmm24
#endif
	vbroadcastsd	%xmm25, %zmm25
	vsubpd		%zmm25, %zmm24, %zmm25
	vpmovq2m	%zmm25, %k1


	// zero accumulation registers

//	ZERO_ACC

	movq	ARG4, %r10 // D

	//
	vmovapd			0*64(%r10), %zmm0 {%k1}{z}
	//
	vmovapd			1*64(%r10), %zmm1 {%k1}{z}
	//
	vmovapd			2*64(%r10), %zmm2 {%k1}{z}
	//
	vmovapd			3*64(%r10), %zmm3 {%k1}{z}
	//
	vmovapd			4*64(%r10), %zmm4 {%k1}{z}
	//
	vmovapd			5*64(%r10), %zmm5 {%k1}{z}
	//
	vmovapd			6*64(%r10), %zmm6 {%k1}{z}
	//
	vmovapd			7*64(%r10), %zmm7 {%k1}{z}

	movq	ARG1, %r10 // k
	movq	ARG5, %r11 // A
	movq	ARG2, %r12 // VA

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_VX8_LIB8
#else
	CALL(inner_kernel_dgemm_nt_vx8_lib8)
#endif

	movq	ARG3, %r10 // T

	//
	vbroadcastsd	56+7*64(%r10), %zmm24
	vmulpd			%zmm7, %zmm24, %zmm7
	//
	vbroadcastsd	48+7*64(%r10), %zmm24
	vfmadd231pd		%zmm6, %zmm24, %zmm7
	vbroadcastsd	48+6*64(%r10), %zmm24
	vmulpd			%zmm6, %zmm24, %zmm6
	//
	vbroadcastsd	40+7*64(%r10), %zmm24
	vfmadd231pd		%zmm5, %zmm24, %zmm7
	vbroadcastsd	40+6*64(%r10), %zmm24
	vfmadd231pd		%zmm5, %zmm24, %zmm6
	vbroadcastsd	40+5*64(%r10), %zmm24
	vmulpd			%zmm5, %zmm24, %zmm5
	//
	vbroadcastsd	32+7*64(%r10), %zmm24
	vfmadd231pd		%zmm4, %zmm24, %zmm7
	vbroadcastsd	32+6*64(%r10), %zmm24
	vfmadd231pd		%zmm4, %zmm24, %zmm6
	vbroadcastsd	32+5*64(%r10), %zmm24
	vfmadd231pd		%zmm4, %zmm24, %zmm5
	vbroadcastsd	32+4*64(%r10), %zmm24
	vmulpd			%zmm4, %zmm24, %zmm4
	//
	vbroadcastsd	24+7*64(%r10), %zmm24
	vfmadd231pd		%zmm3, %zmm24, %zmm7
	vbroadcastsd	24+6*64(%r10), %zmm24
	vfmadd231pd		%zmm3, %zmm24, %zmm6
	vbroadcastsd	24+5*64(%r10), %zmm24
	vfmadd231pd		%zmm3, %zmm24, %zmm5
	vbroadcastsd	24+4*64(%r10), %zmm24
	vfmadd231pd		%zmm3, %zmm24, %zmm4
	vbroadcastsd	24+3*64(%r10), %zmm24
	vmulpd			%zmm3, %zmm24, %zmm3
	//
	vbroadcastsd	16+7*64(%r10), %zmm24
	vfmadd231pd		%zmm2, %zmm24, %zmm7
	vbroadcastsd	16+6*64(%r10), %zmm24
	vfmadd231pd		%zmm2, %zmm24, %zmm6
	vbroadcastsd	16+5*64(%r10), %zmm24
	vfmadd231pd		%zmm2, %zmm24, %zmm5
	vbroadcastsd	16+4*64(%r10), %zmm24
	vfmadd231pd		%zmm2, %zmm24, %zmm4
	vbroadcastsd	16+3*64(%r10), %zmm24
	vfmadd231pd		%zmm2, %zmm24, %zmm3
	vbroadcastsd	16+2*64(%r10), %zmm24
	vmulpd			%zmm2, %zmm24, %zmm2
	//
	vbroadcastsd	8+7*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm7
	vbroadcastsd	8+6*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm6
	vbroadcastsd	8+5*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm5
	vbroadcastsd	8+4*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm4
	vbroadcastsd	8+3*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm3
	vbroadcastsd	8+2*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm2
	vbroadcastsd	8+1*64(%r10), %zmm24
	vmulpd			%zmm1, %zmm24, %zmm1
	//
	vbroadcastsd	0+7*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm7
	vbroadcastsd	0+6*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm6
	vbroadcastsd	0+5*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm5
	vbroadcastsd	0+4*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm4
	vbroadcastsd	0+3*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm3
	vbroadcastsd	0+2*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm2
	vbroadcastsd	0+1*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm1
	vbroadcastsd	0+0*64(%r10), %zmm24
	vmulpd			%zmm0, %zmm24, %zmm0

	movq	ARG4, %r10 // D

	//
	vmovapd			0+0*64(%r10), %zmm24 {%k1}{z}
	vaddpd			%zmm24, %zmm0, %zmm24
	vmovapd			%zmm24, 0+0*64(%r10) {%k1}
	//
	vmovapd			0+1*64(%r10), %zmm24 {%k1}{z}
	vaddpd			%zmm24, %zmm1, %zmm24
	vmovapd			%zmm24, 0+1*64(%r10) {%k1}
	//
	vmovapd			0+2*64(%r10), %zmm24 {%k1}{z}
	vaddpd			%zmm24, %zmm2, %zmm24
	vmovapd			%zmm24, 0+2*64(%r10) {%k1}
	//
	vmovapd			0+3*64(%r10), %zmm24 {%k1}{z}
	vaddpd			%zmm24, %zmm3, %zmm24
	vmovapd			%zmm24, 0+3*64(%r10) {%k1}
	//
	vmovapd			0+4*64(%r10), %zmm24 {%k1}{z}
	vaddpd			%zmm24, %zmm4, %zmm24
	vmovapd			%zmm24, 0+4*64(%r10) {%k1}
	//
	vmovapd			0+5*64(%r10), %zmm24 {%k1}{z}
	vaddpd			%zmm24, %zmm5, %zmm24
	vmovapd			%zmm24, 0+5*64(%r10) {%k1}
	//
	vmovapd			0+6*64(%r10), %zmm24 {%k1}{z}
	vaddpd			%zmm24, %zmm6, %zmm24
	vmovapd			%zmm24, 0+6*64(%r10) {%k1}
	//
	vmovapd			0+7*64(%r10), %zmm24 {%k1}{z}
	vaddpd			%zmm24, %zmm7, %zmm24
	vmovapd			%zmm24, 0+7*64(%r10) {%k1}

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // VA
	movq	ARG5, %r12 // A

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEBP_NN_VX8_LIB8
#else
	CALL(inner_kernel_dgebp_nn_vx8_lib8)
#endif

	EPILOGUE

	ret

	FUN_END(kernel_dlarfb8_rn_la_8_vs_lib8)





//                                   1       2       3            4            5           6           7           8
// void kernel_dlarfb8_rn_lla_8_lib8(int n0, int n1, double *pVL, double *pVA, double *pT, double *pD, double *pL, double *pA);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dlarfb8_rn_lla_8_lib8)
	
	PROLOGUE

	// zero accumulation registers

//	ZERO_ACC

	movq	ARG6, %r10 // D

	// D
	//
	vmovapd			0*64(%r10), %zmm0
	//
	vmovapd			1*64(%r10), %zmm1
	//
	vmovapd			2*64(%r10), %zmm2
	//
	vmovapd			3*64(%r10), %zmm3
	//
	vmovapd			4*64(%r10), %zmm4
	//
	vmovapd			5*64(%r10), %zmm5
	//
	vmovapd			6*64(%r10), %zmm6
	//
	vmovapd			7*64(%r10), %zmm7

	// L
	movq	ARG1, %r10 // n0
	movq	ARG7, %r11 // L
	movq	ARG3, %r12 // VL

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB8
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib8)
#endif
	
	// L final 8x8 lower triangle
	movq	ARG1, %r10 // n0
	sall	$ 6, %r10d // n0*ps*sizeof(double)
	movq	ARG7, %r11 // L
	addq	%r10, %r11
	movq	ARG3, %r12 // VL
	addq	%r10, %r12

	// L 7
	vmovapd			0+0*64(%r11), %zmm25 // A
	vbroadcastsd	0+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	8+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	56+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	// L 6
	vmovapd			0+1*64(%r11), %zmm25 // A
	vbroadcastsd	8+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	56+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	// L 5
	vmovapd			0+2*64(%r11), %zmm25 // A
	vbroadcastsd	16+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	56+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	// L 4
	vmovapd			0+3*64(%r11), %zmm25 // A
	vbroadcastsd	24+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	56+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	// L 3
	vmovapd			0+4*64(%r11), %zmm25 // A
	vbroadcastsd	32+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	56+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	// L 2
	vmovapd			0+5*64(%r11), %zmm25 // A
	vbroadcastsd	40+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	56+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	// L 1
	vmovapd			0+6*64(%r11), %zmm25 // A
	vbroadcastsd	48+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	56+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	// L 0
	vmovapd			0+7*64(%r11), %zmm25 // A
	vbroadcastsd	56+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// A
	movq	ARG2, %r10 // n1
	movq	ARG8, %r11 // A
	movq	ARG4, %r12 // VA

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_8X8_LIB8
#else
	CALL(inner_kernel_dgemm_nt_8x8_lib8)
#endif

	// T
	movq	ARG5, %r10 // T

	//
	vbroadcastsd	56+7*64(%r10), %zmm24
	vmulpd			%zmm7, %zmm24, %zmm7
	//
	vbroadcastsd	48+7*64(%r10), %zmm24
	vfmadd231pd		%zmm6, %zmm24, %zmm7
	vbroadcastsd	48+6*64(%r10), %zmm24
	vmulpd			%zmm6, %zmm24, %zmm6
	//
	vbroadcastsd	40+7*64(%r10), %zmm24
	vfmadd231pd		%zmm5, %zmm24, %zmm7
	vbroadcastsd	40+6*64(%r10), %zmm24
	vfmadd231pd		%zmm5, %zmm24, %zmm6
	vbroadcastsd	40+5*64(%r10), %zmm24
	vmulpd			%zmm5, %zmm24, %zmm5
	//
	vbroadcastsd	32+7*64(%r10), %zmm24
	vfmadd231pd		%zmm4, %zmm24, %zmm7
	vbroadcastsd	32+6*64(%r10), %zmm24
	vfmadd231pd		%zmm4, %zmm24, %zmm6
	vbroadcastsd	32+5*64(%r10), %zmm24
	vfmadd231pd		%zmm4, %zmm24, %zmm5
	vbroadcastsd	32+4*64(%r10), %zmm24
	vmulpd			%zmm4, %zmm24, %zmm4
	//
	vbroadcastsd	24+7*64(%r10), %zmm24
	vfmadd231pd		%zmm3, %zmm24, %zmm7
	vbroadcastsd	24+6*64(%r10), %zmm24
	vfmadd231pd		%zmm3, %zmm24, %zmm6
	vbroadcastsd	24+5*64(%r10), %zmm24
	vfmadd231pd		%zmm3, %zmm24, %zmm5
	vbroadcastsd	24+4*64(%r10), %zmm24
	vfmadd231pd		%zmm3, %zmm24, %zmm4
	vbroadcastsd	24+3*64(%r10), %zmm24
	vmulpd			%zmm3, %zmm24, %zmm3
	//
	vbroadcastsd	16+7*64(%r10), %zmm24
	vfmadd231pd		%zmm2, %zmm24, %zmm7
	vbroadcastsd	16+6*64(%r10), %zmm24
	vfmadd231pd		%zmm2, %zmm24, %zmm6
	vbroadcastsd	16+5*64(%r10), %zmm24
	vfmadd231pd		%zmm2, %zmm24, %zmm5
	vbroadcastsd	16+4*64(%r10), %zmm24
	vfmadd231pd		%zmm2, %zmm24, %zmm4
	vbroadcastsd	16+3*64(%r10), %zmm24
	vfmadd231pd		%zmm2, %zmm24, %zmm3
	vbroadcastsd	16+2*64(%r10), %zmm24
	vmulpd			%zmm2, %zmm24, %zmm2
	//
	vbroadcastsd	8+7*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm7
	vbroadcastsd	8+6*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm6
	vbroadcastsd	8+5*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm5
	vbroadcastsd	8+4*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm4
	vbroadcastsd	8+3*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm3
	vbroadcastsd	8+2*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm2
	vbroadcastsd	8+1*64(%r10), %zmm24
	vmulpd			%zmm1, %zmm24, %zmm1
	//
	vbroadcastsd	0+7*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm7
	vbroadcastsd	0+6*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm6
	vbroadcastsd	0+5*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm5
	vbroadcastsd	0+4*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm4
	vbroadcastsd	0+3*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm3
	vbroadcastsd	0+2*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm2
	vbroadcastsd	0+1*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm1
	vbroadcastsd	0+0*64(%r10), %zmm24
	vmulpd			%zmm0, %zmm24, %zmm0

	// D
	movq	ARG6, %r10 // D

	//
	vmovapd			0+0*64(%r10), %zmm24
	vaddpd			%zmm24, %zmm0, %zmm24
	vmovapd			%zmm24, 0+0*64(%r10)
	//
	vmovapd			0+1*64(%r10), %zmm24
	vaddpd			%zmm24, %zmm1, %zmm24
	vmovapd			%zmm24, 0+1*64(%r10)
	//
	vmovapd			0+2*64(%r10), %zmm24
	vaddpd			%zmm24, %zmm2, %zmm24
	vmovapd			%zmm24, 0+2*64(%r10)
	//
	vmovapd			0+3*64(%r10), %zmm24
	vaddpd			%zmm24, %zmm3, %zmm24
	vmovapd			%zmm24, 0+3*64(%r10)
	//
	vmovapd			0+4*64(%r10), %zmm24
	vaddpd			%zmm24, %zmm4, %zmm24
	vmovapd			%zmm24, 0+4*64(%r10)
	//
	vmovapd			0+5*64(%r10), %zmm24
	vaddpd			%zmm24, %zmm5, %zmm24
	vmovapd			%zmm24, 0+5*64(%r10)
	//
	vmovapd			0+6*64(%r10), %zmm24
	vaddpd			%zmm24, %zmm6, %zmm24
	vmovapd			%zmm24, 0+6*64(%r10)
	//
	vmovapd			0+7*64(%r10), %zmm24
	vaddpd			%zmm24, %zmm7, %zmm24
	vmovapd			%zmm24, 0+7*64(%r10)

	// L
	movq	ARG1, %r10 // n0
	movq	ARG3, %r11 // VL
	movq	ARG7, %r12 // L

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEBP_NN_8X8_LIB8
#else
	CALL(inner_kernel_dgebp_nn_8x8_lib8)
#endif

	// L final 8x8 lower triangle
	movq	ARG1, %r10 // n0
	sall	$ 6, %r10d // n0*ps*sizeof(double)
	movq	ARG3, %r11 // VL
	addq	%r10, %r11
	movq	ARG7, %r12 // L
	addq	%r10, %r12

	// L 7
	vmovapd			0*64(%r12), %zmm28
	vbroadcastsd	0+0*64(%r11), %zmm25
	vfmadd231pd		%zmm0, %zmm25, %zmm28
	vbroadcastsd	8+0*64(%r11), %zmm25
	vfmadd231pd		%zmm1, %zmm25, %zmm28
	vbroadcastsd	16+0*64(%r11), %zmm25
	vfmadd231pd		%zmm2, %zmm25, %zmm28
	vbroadcastsd	24+0*64(%r11), %zmm25
	vfmadd231pd		%zmm3, %zmm25, %zmm28
	vbroadcastsd	32+0*64(%r11), %zmm25
	vfmadd231pd		%zmm4, %zmm25, %zmm28
	vbroadcastsd	40+0*64(%r11), %zmm25
	vfmadd231pd		%zmm5, %zmm25, %zmm28
	vbroadcastsd	48+0*64(%r11), %zmm25
	vfmadd231pd		%zmm6, %zmm25, %zmm28
	vbroadcastsd	56+0*64(%r11), %zmm25
	vfmadd231pd		%zmm7, %zmm25, %zmm28
	vmovapd			%zmm28, 0*64(%r12)
	// L 6
	vmovapd			1*64(%r12), %zmm28
	vbroadcastsd	8+1*64(%r11), %zmm25
	vfmadd231pd		%zmm1, %zmm25, %zmm28
	vbroadcastsd	16+1*64(%r11), %zmm25
	vfmadd231pd		%zmm2, %zmm25, %zmm28
	vbroadcastsd	24+1*64(%r11), %zmm25
	vfmadd231pd		%zmm3, %zmm25, %zmm28
	vbroadcastsd	32+1*64(%r11), %zmm25
	vfmadd231pd		%zmm4, %zmm25, %zmm28
	vbroadcastsd	40+1*64(%r11), %zmm25
	vfmadd231pd		%zmm5, %zmm25, %zmm28
	vbroadcastsd	48+1*64(%r11), %zmm25
	vfmadd231pd		%zmm6, %zmm25, %zmm28
	vbroadcastsd	56+1*64(%r11), %zmm25
	vfmadd231pd		%zmm7, %zmm25, %zmm28
	vmovapd			%zmm28, 1*64(%r12)
	// L 5
	vmovapd			2*64(%r12), %zmm28
	vbroadcastsd	16+2*64(%r11), %zmm25
	vfmadd231pd		%zmm2, %zmm25, %zmm28
	vbroadcastsd	24+2*64(%r11), %zmm25
	vfmadd231pd		%zmm3, %zmm25, %zmm28
	vbroadcastsd	32+2*64(%r11), %zmm25
	vfmadd231pd		%zmm4, %zmm25, %zmm28
	vbroadcastsd	40+2*64(%r11), %zmm25
	vfmadd231pd		%zmm5, %zmm25, %zmm28
	vbroadcastsd	48+2*64(%r11), %zmm25
	vfmadd231pd		%zmm6, %zmm25, %zmm28
	vbroadcastsd	56+2*64(%r11), %zmm25
	vfmadd231pd		%zmm7, %zmm25, %zmm28
	vmovapd			%zmm28, 2*64(%r12)
	// L 4
	vmovapd			3*64(%r12), %zmm28
	vbroadcastsd	24+3*64(%r11), %zmm25
	vfmadd231pd		%zmm3, %zmm25, %zmm28
	vbroadcastsd	32+3*64(%r11), %zmm25
	vfmadd231pd		%zmm4, %zmm25, %zmm28
	vbroadcastsd	40+3*64(%r11), %zmm25
	vfmadd231pd		%zmm5, %zmm25, %zmm28
	vbroadcastsd	48+3*64(%r11), %zmm25
	vfmadd231pd		%zmm6, %zmm25, %zmm28
	vbroadcastsd	56+3*64(%r11), %zmm25
	vfmadd231pd		%zmm7, %zmm25, %zmm28
	vmovapd			%zmm28, 3*64(%r12)
	// L 3
	vmovapd			4*64(%r12), %zmm28
	vbroadcastsd	32+4*64(%r11), %zmm25
	vfmadd231pd		%zmm4, %zmm25, %zmm28
	vbroadcastsd	40+4*64(%r11), %zmm25
	vfmadd231pd		%zmm5, %zmm25, %zmm28
	vbroadcastsd	48+4*64(%r11), %zmm25
	vfmadd231pd		%zmm6, %zmm25, %zmm28
	vbroadcastsd	56+4*64(%r11), %zmm25
	vfmadd231pd		%zmm7, %zmm25, %zmm28
	vmovapd			%zmm28, 4*64(%r12)
	// L 2
	vmovapd			5*64(%r12), %zmm28
	vbroadcastsd	40+5*64(%r11), %zmm25
	vfmadd231pd		%zmm5, %zmm25, %zmm28
	vbroadcastsd	48+5*64(%r11), %zmm25
	vfmadd231pd		%zmm6, %zmm25, %zmm28
	vbroadcastsd	56+5*64(%r11), %zmm25
	vfmadd231pd		%zmm7, %zmm25, %zmm28
	vmovapd			%zmm28, 5*64(%r12)
	// L 1
	vmovapd			6*64(%r12), %zmm28
	vbroadcastsd	48+6*64(%r11), %zmm25
	vfmadd231pd		%zmm6, %zmm25, %zmm28
	vbroadcastsd	56+6*64(%r11), %zmm25
	vfmadd231pd		%zmm7, %zmm25, %zmm28
	vmovapd			%zmm28, 6*64(%r12)
	// L 0
	vmovapd			7*64(%r12), %zmm28
	vbroadcastsd	56+7*64(%r11), %zmm25
	vfmadd231pd		%zmm7, %zmm25, %zmm28
	vmovapd			%zmm28, 7*64(%r12)

	// A
	movq	ARG2, %r10 // n1
	movq	ARG4, %r11 // VA
	movq	ARG8, %r12 // A

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEBP_NN_8X8_LIB8
#else
	CALL(inner_kernel_dgebp_nn_8x8_lib8)
#endif

	EPILOGUE

	ret

	FUN_END(kernel_dlarfb8_rn_lla_8_lib8)





//                                      1       2       3            4            5           6           7           8          9
// void kernel_dlarfb8_rn_lla_8_vs_lib8(int n0, int n1, double *pVL, double *pVA, double *pT, double *pD, double *pL, double *pA, int m1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dlarfb8_rn_lla_8_vs_lib8)
	
	PROLOGUE


	movq	ARG9, %r10 // k

	// compute mask for rows
	vcvtsi2sd	%r10d, %xmm25, %xmm25
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC00(%rip), %zmm24
#elif defined(OS_MAC)
	vmovupd		LC00(%rip), %zmm24
#endif
	vbroadcastsd	%xmm25, %zmm25
	vsubpd		%zmm25, %zmm24, %zmm25
	vpmovq2m	%zmm25, %k1


	// zero accumulation registers

//	ZERO_ACC

	movq	ARG6, %r10 // D

	// D
	//
	vmovapd			0*64(%r10), %zmm0 {%k1}{z}
	//
	vmovapd			1*64(%r10), %zmm1 {%k1}{z}
	//
	vmovapd			2*64(%r10), %zmm2 {%k1}{z}
	//
	vmovapd			3*64(%r10), %zmm3 {%k1}{z}
	//
	vmovapd			4*64(%r10), %zmm4 {%k1}{z}
	//
	vmovapd			5*64(%r10), %zmm5 {%k1}{z}
	//
	vmovapd			6*64(%r10), %zmm6 {%k1}{z}
	//
	vmovapd			7*64(%r10), %zmm7 {%k1}{z}

	// L
	movq	ARG1, %r10 // n0
	movq	ARG7, %r11 // L
	movq	ARG3, %r12 // VL

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_VX8_LIB8
#else
	CALL(inner_kernel_dgemm_nt_vx8_lib8)
#endif
	
	// L final 8x8 lower triangle
	movq	ARG1, %r10 // n0
	sall	$ 6, %r10d // n0*ps*sizeof(double)
	movq	ARG7, %r11 // L
	addq	%r10, %r11
	movq	ARG3, %r12 // VL
	addq	%r10, %r12

	// L 7
	vmovapd			0+0*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	0+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vbroadcastsd	8+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	56+0*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	// L 6
	vmovapd			0+1*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	8+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vbroadcastsd	16+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	56+1*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	// L 5
	vmovapd			0+2*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	16+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vbroadcastsd	24+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	56+2*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	// L 4
	vmovapd			0+3*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	24+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vbroadcastsd	32+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	56+3*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	// L 3
	vmovapd			0+4*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	32+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vbroadcastsd	40+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	56+4*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	// L 2
	vmovapd			0+5*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	40+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vbroadcastsd	48+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	56+5*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	// L 1
	vmovapd			0+6*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	48+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vbroadcastsd	56+6*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	// L 0
	vmovapd			0+7*64(%r11), %zmm25 {%k1}{z} // A
	vbroadcastsd	56+7*64(%r12), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7

	// A
	movq	ARG2, %r10 // n1
	movq	ARG8, %r11 // A
	movq	ARG4, %r12 // VA

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_VX8_LIB8
#else
	CALL(inner_kernel_dgemm_nt_vx8_lib8)
#endif

	// T
	movq	ARG5, %r10 // T

	//
	vbroadcastsd	56+7*64(%r10), %zmm24
	vmulpd			%zmm7, %zmm24, %zmm7
	//
	vbroadcastsd	48+7*64(%r10), %zmm24
	vfmadd231pd		%zmm6, %zmm24, %zmm7
	vbroadcastsd	48+6*64(%r10), %zmm24
	vmulpd			%zmm6, %zmm24, %zmm6
	//
	vbroadcastsd	40+7*64(%r10), %zmm24
	vfmadd231pd		%zmm5, %zmm24, %zmm7
	vbroadcastsd	40+6*64(%r10), %zmm24
	vfmadd231pd		%zmm5, %zmm24, %zmm6
	vbroadcastsd	40+5*64(%r10), %zmm24
	vmulpd			%zmm5, %zmm24, %zmm5
	//
	vbroadcastsd	32+7*64(%r10), %zmm24
	vfmadd231pd		%zmm4, %zmm24, %zmm7
	vbroadcastsd	32+6*64(%r10), %zmm24
	vfmadd231pd		%zmm4, %zmm24, %zmm6
	vbroadcastsd	32+5*64(%r10), %zmm24
	vfmadd231pd		%zmm4, %zmm24, %zmm5
	vbroadcastsd	32+4*64(%r10), %zmm24
	vmulpd			%zmm4, %zmm24, %zmm4
	//
	vbroadcastsd	24+7*64(%r10), %zmm24
	vfmadd231pd		%zmm3, %zmm24, %zmm7
	vbroadcastsd	24+6*64(%r10), %zmm24
	vfmadd231pd		%zmm3, %zmm24, %zmm6
	vbroadcastsd	24+5*64(%r10), %zmm24
	vfmadd231pd		%zmm3, %zmm24, %zmm5
	vbroadcastsd	24+4*64(%r10), %zmm24
	vfmadd231pd		%zmm3, %zmm24, %zmm4
	vbroadcastsd	24+3*64(%r10), %zmm24
	vmulpd			%zmm3, %zmm24, %zmm3
	//
	vbroadcastsd	16+7*64(%r10), %zmm24
	vfmadd231pd		%zmm2, %zmm24, %zmm7
	vbroadcastsd	16+6*64(%r10), %zmm24
	vfmadd231pd		%zmm2, %zmm24, %zmm6
	vbroadcastsd	16+5*64(%r10), %zmm24
	vfmadd231pd		%zmm2, %zmm24, %zmm5
	vbroadcastsd	16+4*64(%r10), %zmm24
	vfmadd231pd		%zmm2, %zmm24, %zmm4
	vbroadcastsd	16+3*64(%r10), %zmm24
	vfmadd231pd		%zmm2, %zmm24, %zmm3
	vbroadcastsd	16+2*64(%r10), %zmm24
	vmulpd			%zmm2, %zmm24, %zmm2
	//
	vbroadcastsd	8+7*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm7
	vbroadcastsd	8+6*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm6
	vbroadcastsd	8+5*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm5
	vbroadcastsd	8+4*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm4
	vbroadcastsd	8+3*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm3
	vbroadcastsd	8+2*64(%r10), %zmm24
	vfmadd231pd		%zmm1, %zmm24, %zmm2
	vbroadcastsd	8+1*64(%r10), %zmm24
	vmulpd			%zmm1, %zmm24, %zmm1
	//
	vbroadcastsd	0+7*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm7
	vbroadcastsd	0+6*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm6
	vbroadcastsd	0+5*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm5
	vbroadcastsd	0+4*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm4
	vbroadcastsd	0+3*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm3
	vbroadcastsd	0+2*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm2
	vbroadcastsd	0+1*64(%r10), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm1
	vbroadcastsd	0+0*64(%r10), %zmm24
	vmulpd			%zmm0, %zmm24, %zmm0

	// D
	movq	ARG6, %r10 // D

	//
	vmovapd			0+0*64(%r10), %zmm24 {%k1}{z}
	vaddpd			%zmm24, %zmm0, %zmm24
	vmovapd			%zmm24, 0+0*64(%r10) {%k1}
	//
	vmovapd			0+1*64(%r10), %zmm24 {%k1}{z}
	vaddpd			%zmm24, %zmm1, %zmm24
	vmovapd			%zmm24, 0+1*64(%r10) {%k1}
	//
	vmovapd			0+2*64(%r10), %zmm24 {%k1}{z}
	vaddpd			%zmm24, %zmm2, %zmm24
	vmovapd			%zmm24, 0+2*64(%r10) {%k1}
	//
	vmovapd			0+3*64(%r10), %zmm24 {%k1}{z}
	vaddpd			%zmm24, %zmm3, %zmm24
	vmovapd			%zmm24, 0+3*64(%r10) {%k1}
	//
	vmovapd			0+4*64(%r10), %zmm24 {%k1}{z}
	vaddpd			%zmm24, %zmm4, %zmm24
	vmovapd			%zmm24, 0+4*64(%r10) {%k1}
	//
	vmovapd			0+5*64(%r10), %zmm24 {%k1}{z}
	vaddpd			%zmm24, %zmm5, %zmm24
	vmovapd			%zmm24, 0+5*64(%r10) {%k1}
	//
	vmovapd			0+6*64(%r10), %zmm24 {%k1}{z}
	vaddpd			%zmm24, %zmm6, %zmm24
	vmovapd			%zmm24, 0+6*64(%r10) {%k1}
	//
	vmovapd			0+7*64(%r10), %zmm24 {%k1}{z}
	vaddpd			%zmm24, %zmm7, %zmm24
	vmovapd			%zmm24, 0+7*64(%r10) {%k1}

	// L
	movq	ARG1, %r10 // n0
	movq	ARG3, %r11 // VL
	movq	ARG7, %r12 // L

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEBP_NN_VX8_LIB8
#else
	CALL(inner_kernel_dgebp_nn_vx8_lib8)
#endif

	// L final 8x8 lower triangle
	movq	ARG1, %r10 // n0
	sall	$ 6, %r10d // n0*ps*sizeof(double)
	movq	ARG3, %r11 // VL
	addq	%r10, %r11
	movq	ARG7, %r12 // L
	addq	%r10, %r12

	// L 7
	vmovapd			0*64(%r12), %zmm28 {%k1}{z}
	vbroadcastsd	0+0*64(%r11), %zmm25
	vfmadd231pd		%zmm0, %zmm25, %zmm28
	vbroadcastsd	8+0*64(%r11), %zmm25
	vfmadd231pd		%zmm1, %zmm25, %zmm28
	vbroadcastsd	16+0*64(%r11), %zmm25
	vfmadd231pd		%zmm2, %zmm25, %zmm28
	vbroadcastsd	24+0*64(%r11), %zmm25
	vfmadd231pd		%zmm3, %zmm25, %zmm28
	vbroadcastsd	32+0*64(%r11), %zmm25
	vfmadd231pd		%zmm4, %zmm25, %zmm28
	vbroadcastsd	40+0*64(%r11), %zmm25
	vfmadd231pd		%zmm5, %zmm25, %zmm28
	vbroadcastsd	48+0*64(%r11), %zmm25
	vfmadd231pd		%zmm6, %zmm25, %zmm28
	vbroadcastsd	56+0*64(%r11), %zmm25
	vfmadd231pd		%zmm7, %zmm25, %zmm28
	vmovapd			%zmm28, 0*64(%r12) {%k1}
	// L 6
	vmovapd			1*64(%r12), %zmm28 {%k1}{z}
	vbroadcastsd	8+1*64(%r11), %zmm25
	vfmadd231pd		%zmm1, %zmm25, %zmm28
	vbroadcastsd	16+1*64(%r11), %zmm25
	vfmadd231pd		%zmm2, %zmm25, %zmm28
	vbroadcastsd	24+1*64(%r11), %zmm25
	vfmadd231pd		%zmm3, %zmm25, %zmm28
	vbroadcastsd	32+1*64(%r11), %zmm25
	vfmadd231pd		%zmm4, %zmm25, %zmm28
	vbroadcastsd	40+1*64(%r11), %zmm25
	vfmadd231pd		%zmm5, %zmm25, %zmm28
	vbroadcastsd	48+1*64(%r11), %zmm25
	vfmadd231pd		%zmm6, %zmm25, %zmm28
	vbroadcastsd	56+1*64(%r11), %zmm25
	vfmadd231pd		%zmm7, %zmm25, %zmm28
	vmovapd			%zmm28, 1*64(%r12) {%k1}
	// L 5
	vmovapd			2*64(%r12), %zmm28 {%k1}{z}
	vbroadcastsd	16+2*64(%r11), %zmm25
	vfmadd231pd		%zmm2, %zmm25, %zmm28
	vbroadcastsd	24+2*64(%r11), %zmm25
	vfmadd231pd		%zmm3, %zmm25, %zmm28
	vbroadcastsd	32+2*64(%r11), %zmm25
	vfmadd231pd		%zmm4, %zmm25, %zmm28
	vbroadcastsd	40+2*64(%r11), %zmm25
	vfmadd231pd		%zmm5, %zmm25, %zmm28
	vbroadcastsd	48+2*64(%r11), %zmm25
	vfmadd231pd		%zmm6, %zmm25, %zmm28
	vbroadcastsd	56+2*64(%r11), %zmm25
	vfmadd231pd		%zmm7, %zmm25, %zmm28
	vmovapd			%zmm28, 2*64(%r12) {%k1}
	// L 4
	vmovapd			3*64(%r12), %zmm28 {%k1}{z}
	vbroadcastsd	24+3*64(%r11), %zmm25
	vfmadd231pd		%zmm3, %zmm25, %zmm28
	vbroadcastsd	32+3*64(%r11), %zmm25
	vfmadd231pd		%zmm4, %zmm25, %zmm28
	vbroadcastsd	40+3*64(%r11), %zmm25
	vfmadd231pd		%zmm5, %zmm25, %zmm28
	vbroadcastsd	48+3*64(%r11), %zmm25
	vfmadd231pd		%zmm6, %zmm25, %zmm28
	vbroadcastsd	56+3*64(%r11), %zmm25
	vfmadd231pd		%zmm7, %zmm25, %zmm28
	vmovapd			%zmm28, 3*64(%r12) {%k1}
	// L 3
	vmovapd			4*64(%r12), %zmm28 {%k1}{z}
	vbroadcastsd	32+4*64(%r11), %zmm25
	vfmadd231pd		%zmm4, %zmm25, %zmm28
	vbroadcastsd	40+4*64(%r11), %zmm25
	vfmadd231pd		%zmm5, %zmm25, %zmm28
	vbroadcastsd	48+4*64(%r11), %zmm25
	vfmadd231pd		%zmm6, %zmm25, %zmm28
	vbroadcastsd	56+4*64(%r11), %zmm25
	vfmadd231pd		%zmm7, %zmm25, %zmm28
	vmovapd			%zmm28, 4*64(%r12) {%k1}
	// L 2
	vmovapd			5*64(%r12), %zmm28 {%k1}{z}
	vbroadcastsd	40+5*64(%r11), %zmm25
	vfmadd231pd		%zmm5, %zmm25, %zmm28
	vbroadcastsd	48+5*64(%r11), %zmm25
	vfmadd231pd		%zmm6, %zmm25, %zmm28
	vbroadcastsd	56+5*64(%r11), %zmm25
	vfmadd231pd		%zmm7, %zmm25, %zmm28
	vmovapd			%zmm28, 5*64(%r12) {%k1}
	// L 1
	vmovapd			6*64(%r12), %zmm28 {%k1}{z}
	vbroadcastsd	48+6*64(%r11), %zmm25
	vfmadd231pd		%zmm6, %zmm25, %zmm28
	vbroadcastsd	56+6*64(%r11), %zmm25
	vfmadd231pd		%zmm7, %zmm25, %zmm28
	vmovapd			%zmm28, 6*64(%r12) {%k1}
	// L 0
	vmovapd			7*64(%r12), %zmm28 {%k1}{z}
	vbroadcastsd	56+7*64(%r11), %zmm25
	vfmadd231pd		%zmm7, %zmm25, %zmm28
	vmovapd			%zmm28, 7*64(%r12) {%k1}

	// A
	movq	ARG2, %r10 // n1
	movq	ARG4, %r11 // VA
	movq	ARG8, %r12 // A

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEBP_NN_VX8_LIB8
#else
	CALL(inner_kernel_dgebp_nn_vx8_lib8)
#endif

	EPILOGUE

	ret

	FUN_END(kernel_dlarfb8_rn_lla_8_lib8)





//#if defined(BLAS_API)
#if ( defined(BLAS_API) | ( defined(LA_HIGH_PERFORMANCE) & defined(MF_COLMAJ) ) )

#include "kernel_dgemm_8x8_lib.S"

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
	.align 64
.LC00:
#elif defined(OS_MAC)
	.align 6
LC00:
#endif
	.double 0.5
	.double 1.5
	.double 2.5
	.double 3.5
	.double 4.5
	.double 5.5
	.double 6.5
	.double 7.5


#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 64
.LC03:
#elif defined(OS_MAC)
	.align 6
LC03:
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


#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 64
.LC04:
#elif defined(OS_MAC)
	.align 6
LC04:
#endif
	.double 1.0
	.double 1.0
	.double 1.0
	.double 1.0
	.double 1.0
	.double 1.0
	.double 1.0
	.double 1.0


#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 64
.LC05:
#elif defined(OS_MAC)
	.align 6
LC05:
#endif
	.quad 0x0
	.quad 0x1
	.quad 0x2
	.quad 0x3
	.quad 0x4
	.quad 0x5
	.quad 0x6
	.quad 0x7



#if defined(OS_LINUX)
	.section	.note.GNU-stack,"",@progbits
#elif defined(OS_MAC)
	.subsections_via_symbols
#endif

