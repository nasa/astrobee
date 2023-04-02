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
	vxorps	%ymm0, %ymm0, %ymm0; \
	vmovaps	%ymm0, %ymm1; \
	vmovaps	%ymm0, %ymm2; \
	vmovaps	%ymm0, %ymm3; \
	vmovaps	%ymm0, %ymm4; \
	vmovaps	%ymm0, %ymm5; \
	vmovaps	%ymm0, %ymm6; \
	vmovaps	%ymm0, %ymm7; \
	vmovaps	%ymm0, %ymm8; \
	vmovaps	%ymm0, %ymm9; \
	vmovaps	%ymm0, %ymm10; \
	vmovaps	%ymm0, %ymm11
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
#define ZERO_ACC \
	vxorps	%ymm0, %ymm0, %ymm0; \
	vmovaps	%ymm0, %ymm1; \
	vmovaps	%ymm0, %ymm2; \
	vmovaps	%ymm0, %ymm3; \
	vmovaps	%ymm0, %ymm4; \
	vmovaps	%ymm0, %ymm5; \
	vmovaps	%ymm0, %ymm6; \
	vmovaps	%ymm0, %ymm7; \
	vmovaps	%ymm0, %ymm8; \
	vmovaps	%ymm0, %ymm9; \
	vmovaps	%ymm0, %ymm10; \
	vmovaps	%ymm0, %ymm11
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
#define ZERO_ACC \
	vxorps	%ymm0, %ymm0, %ymm0; \
	vmovaps	%ymm0, %ymm1; \
	vmovaps	%ymm0, %ymm2; \
	vmovaps	%ymm0, %ymm3; \
	vmovaps	%ymm0, %ymm4; \
	vmovaps	%ymm0, %ymm5; \
	vmovaps	%ymm0, %ymm6; \
	vmovaps	%ymm0, %ymm7; \
	vmovaps	%ymm0, %ymm8; \
	vmovaps	%ymm0, %ymm9; \
	vmovaps	%ymm0, %ymm10; \
	vmovaps	%ymm0, %ymm11
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
// r10d   <- k
// r11   <- A
// r12   <- 4*sda*sizeof(double)
// r13   <- B
// ymm0  <- [d00 d10 d20 d30 d40 d50 d60 d70]
// ymm1  <- [d01 d11 d21 d31 d41 d51 d61 d71]
// ymm2  <- [d02 d12 d22 d32 d42 d52 d62 d72]
// ymm3  <- [d03 d13 d23 d33 d43 d53 d63 d73]
// ymm4  <- []
// ymm5  <- []
// ymm6  <- []
// ymm7  <- []
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
// r13   <- B+4*k*sizeof(double)
// ymm0  <- [d00 d10 d20 d30 d40 d50 d60 d70]
// ymm1  <- [d01 d11 d21 d31 d41 d51 d61 d71]
// ymm2  <- [d02 d12 d22 d32 d42 d52 d62 d72]
// ymm3  <- [d03 d13 d23 d33 d43 d53 d63 d73]
// ymm4  <- []
// ymm5  <- []
// ymm6  <- []
// ymm7  <- []
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NT_24X4_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemm_add_nt_24x4_lib8)
#endif
	
// broadcast scheme
#if 1

	cmpl	$ 0, %r10d
	jle		2f // return

	// prefetch
	prefetcht0		0(%r13) // software prefetch
	prefetcht0		0+64(%r13) // software prefetch

	// preload
	vmovaps 		0(%r11), %ymm13 // A
	vmovaps 		0(%r11, %r12, 1), %ymm14 // A
	vmovaps 		0(%r11, %r12, 2), %ymm15 // A

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop
	
	// unroll 0
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	prefetcht0	128(%r13) // software prefetch
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	prefetcht0	128+64(%r13) // software prefetch
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	subl	$ 4, %r10d
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vmovapd			32(%r11), %ymm13 // A
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vmovapd			32(%r11, %r12, 1), %ymm14 // A
	vfmadd231ps		%ymm15, %ymm12, %ymm11
	vmovapd			32(%r11, %r12, 2), %ymm15 // A

	// unroll 0
	vbroadcastss	32(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	36(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	40(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	44(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vmovapd			64(%r11), %ymm13 // A
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vmovapd			64(%r11, %r12, 1), %ymm14 // A
	vfmadd231ps		%ymm15, %ymm12, %ymm11
	vmovapd			64(%r11, %r12, 2), %ymm15 // A

	// unroll 0
	vbroadcastss	64(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	addq	$ 128, %r11
	vbroadcastss	68(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	72(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	76(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vmovapd			-32(%r11), %ymm13 // A
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vmovapd			-32(%r11, %r12, 1), %ymm14 // A
	vfmadd231ps		%ymm15, %ymm12, %ymm11
	vmovapd			-32(%r11, %r12, 2), %ymm15 // A

	// unroll 0
	vbroadcastss	96(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	addq	$ 128, %r13
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	-28(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	-24(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	-20(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vmovapd			0(%r11), %ymm13 // A
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vmovapd			0(%r11, %r12, 1), %ymm14 // A
	vfmadd231ps		%ymm15, %ymm12, %ymm11
	vmovapd			0(%r11, %r12, 2), %ymm15 // A

	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1


	// unroll 0
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	subl	$ 4, %r10d
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vmovapd			32(%r11), %ymm13 // A
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vmovapd			32(%r11, %r12, 1), %ymm14 // A
	vfmadd231ps		%ymm15, %ymm12, %ymm11
	vmovapd			32(%r11, %r12, 2), %ymm15 // A

	// unroll 0
	vbroadcastss	32(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	36(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	40(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	44(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vmovapd			64(%r11), %ymm13 // A
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vmovapd			64(%r11, %r12, 1), %ymm14 // A
	vfmadd231ps		%ymm15, %ymm12, %ymm11
	vmovapd			64(%r11, %r12, 2), %ymm15 // A

	// unroll 0
	vbroadcastss	64(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	addq	$ 128, %r11
	vbroadcastss	68(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	72(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	76(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vmovapd			-32(%r11), %ymm13 // A
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vmovapd			-32(%r11, %r12, 1), %ymm14 // A
	vfmadd231ps		%ymm15, %ymm12, %ymm11
	vmovapd			-32(%r11, %r12, 2), %ymm15 // A

	// unroll 0
	vbroadcastss	96(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	addq	$ 128, %r13
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	-28(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	-24(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	-20(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vmovapd			0(%r11), %ymm13 // A
	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vmovapd			0(%r11, %r12, 1), %ymm14 // A
	vfmadd231ps		%ymm15, %ymm12, %ymm11
//	vmovapd			0(%r11, %r12, 2), %ymm15 // A

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovapd			0(%r11), %ymm13 // a
	vmovapd			0(%r11, %r12, 1), %ymm14 // A
	vmovapd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // b
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13), %ymm12 // b
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	subl	$ 1, %r10d
	vbroadcastss	8(%r13), %ymm12 // b
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	addq	$ 32, %r11
	vbroadcastss	12(%r13), %ymm12 // b
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vfmadd231ps		%ymm15, %ymm12, %ymm11
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

// shuffle scheme
#else

	cmpl	$ 0, %r10d
	jle		2f // return

	movq	%r11, %r15 // A1 <- A0
	addq	%r12, %r15 // A1 <- A0 + 4*sda*sizeof(float)
	movq	%r15, %rax // A2 <- A1
	addq	%r12, %rax // A2 <- A1 + 4*sda*sizeof(float)

	// preload
	vbroadcastf128	0(%r13), %ymm12 // B
	vmovaps			0(%r11), %ymm13 // A0
	vmovaps			0(%r15), %ymm14 // A1
	vmovaps			0(%rax), %ymm15 // A2

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vpermilps		$ 0xb1, %ymm12, %ymm12

	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vpermilps		$ 0x4e, %ymm12, %ymm12

	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vpermilps		$ 0xb1, %ymm12, %ymm12

	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vmovaps			32(%r11), %ymm13 // A0
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vmovaps			32(%r15), %ymm14 // A1
	vfmadd231ps		%ymm15, %ymm12, %ymm11
	vbroadcastf128	32(%r13), %ymm12 // B
	vmovaps			32(%rax), %ymm15 // A2


	// unroll 1
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vpermilps		$ 0xb1, %ymm12, %ymm12

	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vpermilps		$ 0x4e, %ymm12, %ymm12

	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vpermilps		$ 0xb1, %ymm12, %ymm12

	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vmovaps			64(%r11), %ymm13 // A0
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vmovaps			64(%r15), %ymm14 // A1
	vfmadd231ps		%ymm15, %ymm12, %ymm11
	vbroadcastf128	64(%r13), %ymm12 // B
	vmovaps			64(%rax), %ymm15 // A2


	// unroll 2
	subl	$ 4, %r10d
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vpermilps		$ 0xb1, %ymm12, %ymm12

	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vpermilps		$ 0x4e, %ymm12, %ymm12

	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vpermilps		$ 0xb1, %ymm12, %ymm12

	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vmovaps			96(%r11), %ymm13 // A0
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vmovaps			96(%r15), %ymm14 // A1
	vfmadd231ps		%ymm15, %ymm12, %ymm11
	vbroadcastf128	96(%r13), %ymm12 // B
	vmovaps			96(%rax), %ymm15 // A2


	// unroll 3
	addq	$ 128, %r13
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vpermilps		$ 0xb1, %ymm12, %ymm12

	addq	$ 128, %r11
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vpermilps		$ 0x4e, %ymm12, %ymm12

	addq	$ 128, %r15
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vpermilps		$ 0xb1, %ymm12, %ymm12

	addq	$ 128, %rax
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vmovaps			0(%r11), %ymm13 // A0
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vmovaps			0(%r15), %ymm14 // A1
	vfmadd231ps		%ymm15, %ymm12, %ymm11
	vbroadcastf128	0(%r13), %ymm12 // B
	vmovaps			0(%rax), %ymm15 // A2


	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1


	// unroll 0
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vpermilps		$ 0xb1, %ymm12, %ymm12

	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vpermilps		$ 0x4e, %ymm12, %ymm12

	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vpermilps		$ 0xb1, %ymm12, %ymm12

	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vmovaps			32(%r11), %ymm13 // A0
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vmovaps			32(%r15), %ymm14 // A1
	vfmadd231ps		%ymm15, %ymm12, %ymm11
	vbroadcastf128	32(%r13), %ymm12 // B
	vmovaps			32(%rax), %ymm15 // A2


	// unroll 1
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vpermilps		$ 0xb1, %ymm12, %ymm12

	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vpermilps		$ 0x4e, %ymm12, %ymm12

	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vpermilps		$ 0xb1, %ymm12, %ymm12

	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vmovaps			64(%r11), %ymm13 // A0
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vmovaps			64(%r15), %ymm14 // A1
	vfmadd231ps		%ymm15, %ymm12, %ymm11
	vbroadcastf128	64(%r13), %ymm12 // B
	vmovaps			64(%rax), %ymm15 // A2


	// unroll 2
	subl	$ 4, %r10d
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vpermilps		$ 0xb1, %ymm12, %ymm12

	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vpermilps		$ 0x4e, %ymm12, %ymm12

	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vpermilps		$ 0xb1, %ymm12, %ymm12

	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vmovaps			96(%r11), %ymm13 // A0
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vmovaps			96(%r15), %ymm14 // A1
	vfmadd231ps		%ymm15, %ymm12, %ymm11
	vbroadcastf128	96(%r13), %ymm12 // B
	vmovaps			96(%rax), %ymm15 // A2


	// unroll 3
	addq	$ 128, %r13
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vpermilps		$ 0xb1, %ymm12, %ymm12

	addq	$ 128, %r11
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vpermilps		$ 0x4e, %ymm12, %ymm12

	addq	$ 128, %r15
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vpermilps		$ 0xb1, %ymm12, %ymm12

	addq	$ 128, %rax
	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vmovaps			0(%r11), %ymm13 // A0
	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vmovaps			0(%r15), %ymm14 // A1
	vfmadd231ps		%ymm15, %ymm12, %ymm11
//	vbroadcastf128	0(%r13), %ymm12 // B
//	vmovaps			0(%rax), %ymm15 // A2


//	cmpl	$ 4, %r10d
	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vbroadcastf128	32(%r13), %ymm12 // B
	vmovaps			32(%r11), %ymm13 // A0
	vmovaps			32(%r15), %ymm14 // A1
	vmovaps			32(%rax), %ymm15 // A2
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	subl	$ 1, %r10d

	vpermilps		$ 0xb1, %ymm12, %ymm12
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	addq	$ 32, %r11

	vpermilps		$ 0x4e, %ymm12, %ymm12
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	addq	$ 32, %r13

	vpermilps		$ 0xb1, %ymm12, %ymm12
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vfmadd231ps		%ymm15, %ymm12, %ymm11
	addq	$ 32, %r15


	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

#endif

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_24x4_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d   <- k
// r11   <- A
// r12   <- 4*sda*sizeof(double)
// r13   <- B
// ymm0  <- [d00 d10 d20 d30 d40 d50 d60 d70]
// ymm1  <- [d01 d11 d21 d31 d41 d51 d61 d71]
// ymm2  <- [d02 d12 d22 d32 d42 d52 d62 d72]
// ymm3  <- [d03 d13 d23 d33 d43 d53 d63 d73]
// ymm4  <- []
// ymm5  <- []
// ymm6  <- []
// ymm7  <- []
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
// r13   <- B+4*k*sizeof(double)
// ymm0  <- [d00 d10 d20 d30 d40 d50 d60 d70]
// ymm1  <- [d01 d11 d21 d31 d41 d51 d61 d71]
// ymm2  <- [d02 d12 d22 d32 d42 d52 d62 d72]
// ymm3  <- [d03 d13 d23 d33 d43 d53 d63 d73]
// ymm4  <- []
// ymm5  <- []
// ymm6  <- []
// ymm7  <- []
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_SUB_NT_24X4_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemm_sub_nt_24x4_lib8)
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	// preload
	vmovaps 		0(%r11), %ymm13 // A
	vmovaps 		0(%r11, %r12, 1), %ymm14 // A
	vmovaps 		0(%r11, %r12, 2), %ymm15 // A

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop
	
	// unroll 0
	vbroadcastss	0(%r13), %ymm12 // B
	vfnmadd231ps	%ymm13, %ymm12, %ymm0
	vfnmadd231ps	%ymm14, %ymm12, %ymm4
	vfnmadd231ps	%ymm15, %ymm12, %ymm8
	subl	$ 4, %r10d
	vbroadcastss	4(%r13), %ymm12 // B
	vfnmadd231ps	%ymm13, %ymm12, %ymm1
	vfnmadd231ps	%ymm14, %ymm12, %ymm5
	vfnmadd231ps	%ymm15, %ymm12, %ymm9
	vbroadcastss	8(%r13), %ymm12 // B
	vfnmadd231ps	%ymm13, %ymm12, %ymm2
	vfnmadd231ps	%ymm14, %ymm12, %ymm6
	vfnmadd231ps	%ymm15, %ymm12, %ymm10
	vbroadcastss	12(%r13), %ymm12 // B
	vfnmadd231ps	%ymm13, %ymm12, %ymm3
	vmovapd			32(%r11), %ymm13 // A
	vfnmadd231ps	%ymm14, %ymm12, %ymm7
	vmovapd			32(%r11, %r12, 1), %ymm14 // A
	vfnmadd231ps	%ymm15, %ymm12, %ymm11
	vmovapd			32(%r11, %r12, 2), %ymm15 // A

	// unroll 0
	vbroadcastss	32(%r13), %ymm12 // B
	vfnmadd231ps	%ymm13, %ymm12, %ymm0
	vfnmadd231ps	%ymm14, %ymm12, %ymm4
	vfnmadd231ps	%ymm15, %ymm12, %ymm8
	vbroadcastss	36(%r13), %ymm12 // B
	vfnmadd231ps	%ymm13, %ymm12, %ymm1
	vfnmadd231ps	%ymm14, %ymm12, %ymm5
	vfnmadd231ps	%ymm15, %ymm12, %ymm9
	vbroadcastss	40(%r13), %ymm12 // B
	vfnmadd231ps	%ymm13, %ymm12, %ymm2
	vfnmadd231ps	%ymm14, %ymm12, %ymm6
	vfnmadd231ps	%ymm15, %ymm12, %ymm10
	vbroadcastss	44(%r13), %ymm12 // B
	vfnmadd231ps	%ymm13, %ymm12, %ymm3
	vmovapd			64(%r11), %ymm13 // A
	vfnmadd231ps	%ymm14, %ymm12, %ymm7
	vmovapd			64(%r11, %r12, 1), %ymm14 // A
	vfnmadd231ps	%ymm15, %ymm12, %ymm11
	vmovapd			64(%r11, %r12, 2), %ymm15 // A

	// unroll 0
	vbroadcastss	64(%r13), %ymm12 // B
	vfnmadd231ps	%ymm13, %ymm12, %ymm0
	vfnmadd231ps	%ymm14, %ymm12, %ymm4
	vfnmadd231ps	%ymm15, %ymm12, %ymm8
	addq	$ 128, %r11
	vbroadcastss	68(%r13), %ymm12 // B
	vfnmadd231ps	%ymm13, %ymm12, %ymm1
	vfnmadd231ps	%ymm14, %ymm12, %ymm5
	vfnmadd231ps	%ymm15, %ymm12, %ymm9
	vbroadcastss	72(%r13), %ymm12 // B
	vfnmadd231ps	%ymm13, %ymm12, %ymm2
	vfnmadd231ps	%ymm14, %ymm12, %ymm6
	vfnmadd231ps	%ymm15, %ymm12, %ymm10
	vbroadcastss	76(%r13), %ymm12 // B
	vfnmadd231ps	%ymm13, %ymm12, %ymm3
	vmovapd			-32(%r11), %ymm13 // A
	vfnmadd231ps	%ymm14, %ymm12, %ymm7
	vmovapd			-32(%r11, %r12, 1), %ymm14 // A
	vfnmadd231ps	%ymm15, %ymm12, %ymm11
	vmovapd			-32(%r11, %r12, 2), %ymm15 // A

	// unroll 0
	vbroadcastss	96(%r13), %ymm12 // B
	vfnmadd231ps	%ymm13, %ymm12, %ymm0
	vfnmadd231ps	%ymm14, %ymm12, %ymm4
	addq	$ 128, %r13
	vfnmadd231ps	%ymm15, %ymm12, %ymm8
	vbroadcastss	-28(%r13), %ymm12 // B
	vfnmadd231ps	%ymm13, %ymm12, %ymm1
	vfnmadd231ps	%ymm14, %ymm12, %ymm5
	vfnmadd231ps	%ymm15, %ymm12, %ymm9
	vbroadcastss	-24(%r13), %ymm12 // B
	vfnmadd231ps	%ymm13, %ymm12, %ymm2
	vfnmadd231ps	%ymm14, %ymm12, %ymm6
	vfnmadd231ps	%ymm15, %ymm12, %ymm10
	vbroadcastss	-20(%r13), %ymm12 // B
	vfnmadd231ps	%ymm13, %ymm12, %ymm3
	vmovapd			0(%r11), %ymm13 // A
	vfnmadd231ps	%ymm14, %ymm12, %ymm7
	vmovapd			0(%r11, %r12, 1), %ymm14 // A
	vfnmadd231ps	%ymm15, %ymm12, %ymm11
	vmovapd			0(%r11, %r12, 2), %ymm15 // A

	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1


	// unroll 0
	vbroadcastss	0(%r13), %ymm12 // B
	vfnmadd231ps	%ymm13, %ymm12, %ymm0
	vfnmadd231ps	%ymm14, %ymm12, %ymm4
	vfnmadd231ps	%ymm15, %ymm12, %ymm8
	subl	$ 4, %r10d
	vbroadcastss	4(%r13), %ymm12 // B
	vfnmadd231ps	%ymm13, %ymm12, %ymm1
	vfnmadd231ps	%ymm14, %ymm12, %ymm5
	vfnmadd231ps	%ymm15, %ymm12, %ymm9
	vbroadcastss	8(%r13), %ymm12 // B
	vfnmadd231ps	%ymm13, %ymm12, %ymm2
	vfnmadd231ps	%ymm14, %ymm12, %ymm6
	vfnmadd231ps	%ymm15, %ymm12, %ymm10
	vbroadcastss	12(%r13), %ymm12 // B
	vfnmadd231ps	%ymm13, %ymm12, %ymm3
	vmovapd			32(%r11), %ymm13 // A
	vfnmadd231ps	%ymm14, %ymm12, %ymm7
	vmovapd			32(%r11, %r12, 1), %ymm14 // A
	vfnmadd231ps	%ymm15, %ymm12, %ymm11
	vmovapd			32(%r11, %r12, 2), %ymm15 // A

	// unroll 0
	vbroadcastss	32(%r13), %ymm12 // B
	vfnmadd231ps	%ymm13, %ymm12, %ymm0
	vfnmadd231ps	%ymm14, %ymm12, %ymm4
	vfnmadd231ps	%ymm15, %ymm12, %ymm8
	vbroadcastss	36(%r13), %ymm12 // B
	vfnmadd231ps	%ymm13, %ymm12, %ymm1
	vfnmadd231ps	%ymm14, %ymm12, %ymm5
	vfnmadd231ps	%ymm15, %ymm12, %ymm9
	vbroadcastss	40(%r13), %ymm12 // B
	vfnmadd231ps	%ymm13, %ymm12, %ymm2
	vfnmadd231ps	%ymm14, %ymm12, %ymm6
	vfnmadd231ps	%ymm15, %ymm12, %ymm10
	vbroadcastss	44(%r13), %ymm12 // B
	vfnmadd231ps	%ymm13, %ymm12, %ymm3
	vmovapd			64(%r11), %ymm13 // A
	vfnmadd231ps	%ymm14, %ymm12, %ymm7
	vmovapd			64(%r11, %r12, 1), %ymm14 // A
	vfnmadd231ps	%ymm15, %ymm12, %ymm11
	vmovapd			64(%r11, %r12, 2), %ymm15 // A

	// unroll 0
	vbroadcastss	64(%r13), %ymm12 // B
	vfnmadd231ps	%ymm13, %ymm12, %ymm0
	vfnmadd231ps	%ymm14, %ymm12, %ymm4
	vfnmadd231ps	%ymm15, %ymm12, %ymm8
	addq	$ 128, %r11
	vbroadcastss	68(%r13), %ymm12 // B
	vfnmadd231ps	%ymm13, %ymm12, %ymm1
	vfnmadd231ps	%ymm14, %ymm12, %ymm5
	vfnmadd231ps	%ymm15, %ymm12, %ymm9
	vbroadcastss	72(%r13), %ymm12 // B
	vfnmadd231ps	%ymm13, %ymm12, %ymm2
	vfnmadd231ps	%ymm14, %ymm12, %ymm6
	vfnmadd231ps	%ymm15, %ymm12, %ymm10
	vbroadcastss	76(%r13), %ymm12 // B
	vfnmadd231ps	%ymm13, %ymm12, %ymm3
	vmovapd			-32(%r11), %ymm13 // A
	vfnmadd231ps	%ymm14, %ymm12, %ymm7
	vmovapd			-32(%r11, %r12, 1), %ymm14 // A
	vfnmadd231ps	%ymm15, %ymm12, %ymm11
	vmovapd			-32(%r11, %r12, 2), %ymm15 // A

	// unroll 0
	vbroadcastss	96(%r13), %ymm12 // B
	vfnmadd231ps	%ymm13, %ymm12, %ymm0
	vfnmadd231ps	%ymm14, %ymm12, %ymm4
	addq	$ 128, %r13
	vfnmadd231ps	%ymm15, %ymm12, %ymm8
	vbroadcastss	-28(%r13), %ymm12 // B
	vfnmadd231ps	%ymm13, %ymm12, %ymm1
	vfnmadd231ps	%ymm14, %ymm12, %ymm5
	vfnmadd231ps	%ymm15, %ymm12, %ymm9
	vbroadcastss	-24(%r13), %ymm12 // B
	vfnmadd231ps	%ymm13, %ymm12, %ymm2
	vfnmadd231ps	%ymm14, %ymm12, %ymm6
	vfnmadd231ps	%ymm15, %ymm12, %ymm10
	vbroadcastss	-20(%r13), %ymm12 // B
	vfnmadd231ps	%ymm13, %ymm12, %ymm3
//	vmovapd			0(%r11), %ymm13 // A
	vfnmadd231ps	%ymm14, %ymm12, %ymm7
//	vmovapd			0(%r11, %r12, 1), %ymm14 // A
	vfnmadd231ps	%ymm15, %ymm12, %ymm11
//	vmovapd			0(%r11, %r12, 2), %ymm15 // A

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovapd			0(%r11), %ymm13 // a
	vmovapd			0(%r11, %r12, 1), %ymm14 // A
	vmovapd			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // b
	vfnmadd231ps	%ymm13, %ymm12, %ymm0
	vfnmadd231ps	%ymm14, %ymm12, %ymm4
	vfnmadd231ps	%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13), %ymm12 // b
	vfnmadd231ps	%ymm13, %ymm12, %ymm1
	vfnmadd231ps	%ymm14, %ymm12, %ymm5
	vfnmadd231ps	%ymm15, %ymm12, %ymm9
	subl	$ 1, %r10d
	vbroadcastss	8(%r13), %ymm12 // b
	vfnmadd231ps	%ymm13, %ymm12, %ymm2
	vfnmadd231ps	%ymm14, %ymm12, %ymm6
	vfnmadd231ps	%ymm15, %ymm12, %ymm10
	addq	$ 32, %r11
	vbroadcastss	12(%r13), %ymm12 // b
	vfnmadd231ps	%ymm13, %ymm12, %ymm3
	vfnmadd231ps	%ymm14, %ymm12, %ymm7
	vfnmadd231ps	%ymm15, %ymm12, %ymm11
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_sub_nt_24x4_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- 4*sda*sizeof(double)
// r13   <- B
// r14   <- 4*sdb*sizeof(double)
// r15   <= dirty
// ymm0  <- []
// ymm1  <- []
// ymm2  <- []
// ymm3  <- []
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
// r13   <- B+(k/4)*sdb*sizeof(double)+(k%4)
// r14   <- 4*sdb*sizeof(double)
// r15   <= dirty
// ymm0  <- []
// ymm1  <- []
// ymm2  <- []
// ymm3  <- []
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NN_24X4_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemm_add_nn_24x4_lib8)
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	cmpl	$ 8, %r10d
	jl		0f // consider clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	prefetcht0	0(%r13, %r14, 1) // software prefetch
	prefetcht0	64(%r13, %r14, 1) // software prefetch

	// unroll 0
	vmovaps			0(%r11), %ymm12 // A0
	vmovaps			0(%r11, %r12, 1), %ymm13 // A1
	vmovaps			0(%r11, %r12, 2), %ymm14 // A2
	vbroadcastss	0(%r13), %ymm15 // B[0]
	vfmadd231ps		%ymm12, %ymm15, %ymm0
	vfmadd231ps		%ymm13, %ymm15, %ymm4
	vfmadd231ps		%ymm14, %ymm15, %ymm8
	vbroadcastss	32(%r13), %ymm15 // B[1]
	vfmadd231ps		%ymm12, %ymm15, %ymm1
	vfmadd231ps		%ymm13, %ymm15, %ymm5
	vfmadd231ps		%ymm14, %ymm15, %ymm9
	vbroadcastss	64(%r13), %ymm15 // B[2]
	vfmadd231ps		%ymm12, %ymm15, %ymm2
	vfmadd231ps		%ymm13, %ymm15, %ymm6
	vfmadd231ps		%ymm14, %ymm15, %ymm10
	vbroadcastss	96(%r13), %ymm15 // B[3]
	vfmadd231ps		%ymm12, %ymm15, %ymm3
	vfmadd231ps		%ymm13, %ymm15, %ymm7
	vfmadd231ps		%ymm14, %ymm15, %ymm11


	// unroll 1
	vmovaps			32(%r11), %ymm12 // A0
	vmovaps			32(%r11, %r12, 1), %ymm13 // A1
	vmovaps			32(%r11, %r12, 2), %ymm14 // A2
	vbroadcastss	4(%r13), %ymm15 // B[0]
	vfmadd231ps		%ymm12, %ymm15, %ymm0
	vfmadd231ps		%ymm13, %ymm15, %ymm4
	vfmadd231ps		%ymm14, %ymm15, %ymm8
	vbroadcastss	36(%r13), %ymm15 // B[1]
	vfmadd231ps		%ymm12, %ymm15, %ymm1
	vfmadd231ps		%ymm13, %ymm15, %ymm5
	vfmadd231ps		%ymm14, %ymm15, %ymm9
	vbroadcastss	68(%r13), %ymm15 // B[2]
	vfmadd231ps		%ymm12, %ymm15, %ymm2
	vfmadd231ps		%ymm13, %ymm15, %ymm6
	vfmadd231ps		%ymm14, %ymm15, %ymm10
	vbroadcastss	100(%r13), %ymm15 // B[3]
	vfmadd231ps		%ymm12, %ymm15, %ymm3
	vfmadd231ps		%ymm13, %ymm15, %ymm7
	vfmadd231ps		%ymm14, %ymm15, %ymm11


	// unroll 2
	vmovaps			64(%r11), %ymm12 // A0
	vmovaps			64(%r11, %r12, 1), %ymm13 // A1
	vmovaps			64(%r11, %r12, 2), %ymm14 // A2
	vbroadcastss	8(%r13), %ymm15 // B[0]
	vfmadd231ps		%ymm12, %ymm15, %ymm0
	vfmadd231ps		%ymm13, %ymm15, %ymm4
	vfmadd231ps		%ymm14, %ymm15, %ymm8
	vbroadcastss	40(%r13), %ymm15 // B[1]
	vfmadd231ps		%ymm12, %ymm15, %ymm1
	vfmadd231ps		%ymm13, %ymm15, %ymm5
	vfmadd231ps		%ymm14, %ymm15, %ymm9
	vbroadcastss	72(%r13), %ymm15 // B[2]
	vfmadd231ps		%ymm12, %ymm15, %ymm2
	vfmadd231ps		%ymm13, %ymm15, %ymm6
	vfmadd231ps		%ymm14, %ymm15, %ymm10
	vbroadcastss	104(%r13), %ymm15 // B[3]
	vfmadd231ps		%ymm12, %ymm15, %ymm3
	vfmadd231ps		%ymm13, %ymm15, %ymm7
	vfmadd231ps		%ymm14, %ymm15, %ymm11


	// unroll 3
	vmovaps			96(%r11), %ymm12 // A0
	vmovaps			96(%r11, %r12, 1), %ymm13 // A1
	vmovaps			96(%r11, %r12, 2), %ymm14 // A2
	vbroadcastss	12(%r13), %ymm15 // B[0]
	vfmadd231ps		%ymm12, %ymm15, %ymm0
	vfmadd231ps		%ymm13, %ymm15, %ymm4
	vfmadd231ps		%ymm14, %ymm15, %ymm8
	vbroadcastss	44(%r13), %ymm15 // B[1]
	vfmadd231ps		%ymm12, %ymm15, %ymm1
	vfmadd231ps		%ymm13, %ymm15, %ymm5
	vfmadd231ps		%ymm14, %ymm15, %ymm9
	vbroadcastss	76(%r13), %ymm15 // B[2]
	vfmadd231ps		%ymm12, %ymm15, %ymm2
	vfmadd231ps		%ymm13, %ymm15, %ymm6
	vfmadd231ps		%ymm14, %ymm15, %ymm10
	vbroadcastss	108(%r13), %ymm15 // B[3]
	vfmadd231ps		%ymm12, %ymm15, %ymm3
	vfmadd231ps		%ymm13, %ymm15, %ymm7
	vfmadd231ps		%ymm14, %ymm15, %ymm11


	// unroll 4
	vmovaps			128(%r11), %ymm12 // A0
	vmovaps			128(%r11, %r12, 1), %ymm13 // A1
	vmovaps			128(%r11, %r12, 2), %ymm14 // A2
	vbroadcastss	16(%r13), %ymm15 // B[0]
	vfmadd231ps		%ymm12, %ymm15, %ymm0
	vfmadd231ps		%ymm13, %ymm15, %ymm4
	vfmadd231ps		%ymm14, %ymm15, %ymm8
	vbroadcastss	48(%r13), %ymm15 // B[1]
	vfmadd231ps		%ymm12, %ymm15, %ymm1
	vfmadd231ps		%ymm13, %ymm15, %ymm5
	vfmadd231ps		%ymm14, %ymm15, %ymm9
	vbroadcastss	80(%r13), %ymm15 // B[2]
	vfmadd231ps		%ymm12, %ymm15, %ymm2
	vfmadd231ps		%ymm13, %ymm15, %ymm6
	vfmadd231ps		%ymm14, %ymm15, %ymm10
	vbroadcastss	112(%r13), %ymm15 // B[3]
	vfmadd231ps		%ymm12, %ymm15, %ymm3
	vfmadd231ps		%ymm13, %ymm15, %ymm7
	vfmadd231ps		%ymm14, %ymm15, %ymm11


	// unroll 5
	vmovaps			160(%r11), %ymm12 // A0
	vmovaps			160(%r11, %r12, 1), %ymm13 // A1
	vmovaps			160(%r11, %r12, 2), %ymm14 // A2
	vbroadcastss	20(%r13), %ymm15 // B[0]
	vfmadd231ps		%ymm12, %ymm15, %ymm0
	vfmadd231ps		%ymm13, %ymm15, %ymm4
	vfmadd231ps		%ymm14, %ymm15, %ymm8
	vbroadcastss	52(%r13), %ymm15 // B[1]
	vfmadd231ps		%ymm12, %ymm15, %ymm1
	vfmadd231ps		%ymm13, %ymm15, %ymm5
	vfmadd231ps		%ymm14, %ymm15, %ymm9
	vbroadcastss	84(%r13), %ymm15 // B[2]
	vfmadd231ps		%ymm12, %ymm15, %ymm2
	vfmadd231ps		%ymm13, %ymm15, %ymm6
	vfmadd231ps		%ymm14, %ymm15, %ymm10
	vbroadcastss	116(%r13), %ymm15 // B[3]
	vfmadd231ps		%ymm12, %ymm15, %ymm3
	vfmadd231ps		%ymm13, %ymm15, %ymm7
	vfmadd231ps		%ymm14, %ymm15, %ymm11


	// unroll 6
	vmovaps			192(%r11), %ymm12 // A0
	vmovaps			192(%r11, %r12, 1), %ymm13 // A1
	vmovaps			192(%r11, %r12, 2), %ymm14 // A2
	vbroadcastss	24(%r13), %ymm15 // B[0]
	vfmadd231ps		%ymm12, %ymm15, %ymm0
	vfmadd231ps		%ymm13, %ymm15, %ymm4
	vfmadd231ps		%ymm14, %ymm15, %ymm8
	vbroadcastss	56(%r13), %ymm15 // B[1]
	vfmadd231ps		%ymm12, %ymm15, %ymm1
	subl	$ 8, %r10d
	vfmadd231ps		%ymm13, %ymm15, %ymm5
	vfmadd231ps		%ymm14, %ymm15, %ymm9
	vbroadcastss	88(%r13), %ymm15 // B[2]
	vfmadd231ps		%ymm12, %ymm15, %ymm2
	vfmadd231ps		%ymm13, %ymm15, %ymm6
	vbroadcastss	120(%r13), %ymm15 // B[3]
	vfmadd231ps		%ymm14, %ymm15, %ymm10
	vfmadd231ps		%ymm12, %ymm15, %ymm3
	vfmadd231ps		%ymm13, %ymm15, %ymm7
	vfmadd231ps		%ymm14, %ymm15, %ymm11


	// unroll 7
	vmovaps			224(%r11), %ymm12 // A0
	vmovaps			224(%r11, %r12, 1), %ymm13 // A1
	vmovaps			224(%r11, %r12, 2), %ymm14 // A2
	vbroadcastss	28(%r13), %ymm15 // B[0]
	vfmadd231ps		%ymm12, %ymm15, %ymm0
	vfmadd231ps		%ymm13, %ymm15, %ymm4
	vfmadd231ps		%ymm14, %ymm15, %ymm8
	vbroadcastss	60(%r13), %ymm15 // B[1]
	vfmadd231ps		%ymm12, %ymm15, %ymm1
	addq	$ 256, %r11
	vfmadd231ps		%ymm13, %ymm15, %ymm5
	vfmadd231ps		%ymm14, %ymm15, %ymm9
	vbroadcastss	92(%r13), %ymm15 // B[2]
	vfmadd231ps		%ymm12, %ymm15, %ymm2
	vfmadd231ps		%ymm13, %ymm15, %ymm6
	vfmadd231ps		%ymm14, %ymm15, %ymm10
	vbroadcastss	124(%r13), %ymm15 // B[3]
	vfmadd231ps		%ymm12, %ymm15, %ymm3
	vfmadd231ps		%ymm13, %ymm15, %ymm7
	vfmadd231ps		%ymm14, %ymm15, %ymm11

	addq	%r14, %r13

	cmpl	$ 7, %r10d
	jg		1b // main loop 


0: // consider clean1-up loop
	
	cmpl	$ 0, %r10d
	jle		2f // return

3: // clean1-up loop
	
	// unroll 0
	vmovaps			0(%r11), %ymm12 // A0
	vmovaps			0(%r11, %r12, 1), %ymm13 // A1
	vmovaps			0(%r11, %r12, 2), %ymm14 // A2
	vbroadcastss	0(%r13), %ymm15 // B[0]
	vfmadd231ps		%ymm12, %ymm15, %ymm0
	vfmadd231ps		%ymm13, %ymm15, %ymm4
	vfmadd231ps		%ymm14, %ymm15, %ymm8
	vbroadcastss	32(%r13), %ymm15 // B[1]
	vfmadd231ps		%ymm12, %ymm15, %ymm1
	vfmadd231ps		%ymm13, %ymm15, %ymm5
	vfmadd231ps		%ymm14, %ymm15, %ymm9
	vbroadcastss	64(%r13), %ymm15 // B[2]
	vfmadd231ps		%ymm12, %ymm15, %ymm2
	vfmadd231ps		%ymm13, %ymm15, %ymm6
	vfmadd231ps		%ymm14, %ymm15, %ymm10
	vbroadcastss	96(%r13), %ymm15 // B[3]
	vfmadd231ps		%ymm12, %ymm15, %ymm3
	vfmadd231ps		%ymm13, %ymm15, %ymm7
	vfmadd231ps		%ymm14, %ymm15, %ymm11

	subl	$ 1, %r10d
	addq	$ 32, %r11
	addq	$ 4, %r13

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_24x4_lib8)
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
// ymm0  <- []
// ymm1  <- []
// ymm2  <- []
// ymm3  <- []
// ymm8  <- dirty
// ymm12 <- dirty
// ymm15 <- dirty

//
// output arguments:
// r10   <- k-(4-offB)
// r11   <- A+(4-offB)*bs*sizeof(double)
// r12   <- bs*sda*sizeof(double)
// r13   <- B-offB+bs*sdb*sizeof(double)
// r14   <- bs*sdb*sizeof(double)
// r15   <- offB
// ymm0  <- []
// ymm1  <- []
// ymm2  <- []
// ymm3  <- []
// ymm8  <- dirty
// ymm12 <- dirty
// ymm15 <- dirty


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_GEMM_ADD_NN_24X4_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_gemm_add_nn_24x4_lib8)
#endif
	
	cmpl			$ 0, %r15d // offset==0
	jle				2f // end

	cmpl			$ 0, %r10d // k==0
	jle				2f // end

	movl			$ 8, %ebx
	subl			%r15d, %ebx // 8-offsetB
	cmpl			%r10d, %ebx
//	jle				0f
//	movl			%r10d, %ebx // kend=min(k,8-offsetB)
//0:
	cmovgl			%r10d, %ebx // kend=min(k,8-offsetB)

	movl			%r15d, %eax
	sall			$ 2, %eax // offsetB*sizeof(float)
	addq			%rax, %r13 // B+offsetB*sizeof(float)

1:
	// unroll 0
	vmovaps			0(%r11), %ymm12 // A0
	vmovaps			0(%r11, %r12, 1), %ymm13 // A1
	vmovaps			0(%r11, %r12, 2), %ymm14 // A2
	vbroadcastss	0(%r13), %ymm15 // B[0]
	vfmadd231ps		%ymm12, %ymm15, %ymm0
	vfmadd231ps		%ymm13, %ymm15, %ymm4
	vfmadd231ps		%ymm14, %ymm15, %ymm8
	vbroadcastss	32(%r13), %ymm15 // B[1]
	vfmadd231ps		%ymm12, %ymm15, %ymm1
	vfmadd231ps		%ymm13, %ymm15, %ymm5
	vfmadd231ps		%ymm14, %ymm15, %ymm9
	vbroadcastss	64(%r13), %ymm15 // B[2]
	vfmadd231ps		%ymm12, %ymm15, %ymm2
	vfmadd231ps		%ymm13, %ymm15, %ymm6
	vfmadd231ps		%ymm14, %ymm15, %ymm10
	vbroadcastss	96(%r13), %ymm15 // B[3]
	vfmadd231ps		%ymm12, %ymm15, %ymm3
	vfmadd231ps		%ymm13, %ymm15, %ymm7
	vfmadd231ps		%ymm14, %ymm15, %ymm11

	subl			$ 1, %r10d // k-1
	subl			$ 1, %ebx // kend-1
	addq			$ 32, %r11 // A+1*bs*sizeof(float)
	addq			$ 4, %r13 // B+1*sizeof(float)

	cmpl			$ 0, %ebx
	jg				1b

	cmpl			$ 0, %r10d
	jle				2f // end

	addq			%r14, %r13
	subq			$ 32, %r13 // B+bs*(sdb-1)*sizeof(float)

2:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_gemm_add_nn_24x4_lib8)
#endif





// common inner routine with file scope
//
// edge for B lower triangular
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
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm8  <- dirty
// ymm12 <- dirty
// ymm15 <- dirty

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
// ymm2  <- [d02 d12 d22 d32]
// ymm3  <- [d03 d13 d23 d33]
// ymm8  <- dirty
// ymm12 <- dirty
// ymm15 <- dirty


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRMM_NN_RL_24X4_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_trmm_nn_rl_24x4_lib8)
#endif
	
	cmpl		$ 0, %r10d
	jle			0f // end

	movl		%r15d, %eax
	sall		$ 2, %eax // offsetB*sizeof(float)
	movq		%r13, %rbx // B
	addq		%rax, %rbx // B+offsetB*sizeof(float)


	cmpl	$ 4, %r15d
	jg		1f

	// offB==0, 1, 2, 3, 4

	vmovaps			0(%r11), %ymm13
	vmovaps			0(%r11, %r12, 1), %ymm14
	vmovaps			0(%r11, %r12, 2), %ymm15
	vbroadcastss	0(%rbx), %ymm12
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8

	subl		$ 1, %r10d // k-1
	addq		$ 32, %r11 // A+1*bs*sizeof(float)
	addl		$ 1, %r15d // offsetB+1

	cmpl		$ 0, %r10d
	jle			0f // end

	vmovaps			0(%r11), %ymm13
	vmovaps			0(%r11, %r12, 1), %ymm14
	vmovaps			0(%r11, %r12, 2), %ymm15
	vbroadcastss	4(%rbx), %ymm12
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	36(%rbx), %ymm12
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9

	subl		$ 1, %r10d // k-1
	addq		$ 32, %r11 // A+1*bs*sizeof(float)
	addl		$ 1, %r15d // offsetB+1

	cmpl		$ 0, %r10d
	jle			0f // end

	vmovaps			0(%r11), %ymm13
	vmovaps			0(%r11, %r12, 1), %ymm14
	vmovaps			0(%r11, %r12, 2), %ymm15
	vbroadcastss	8(%rbx), %ymm12
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	40(%rbx), %ymm12
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	72(%rbx), %ymm12
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10

	subl		$ 1, %r10d // k-1
	addq		$ 32, %r11 // A+1*bs*sizeof(float)
	addl		$ 1, %r15d // offsetB+1

	jmp			0f // end


1:
	cmpl	$ 5, %r15d
	jg		1f

	// offB==5

	vmovaps			0(%r11), %ymm13
	vmovaps			0(%r11, %r12, 1), %ymm14
	vmovaps			0(%r11, %r12, 2), %ymm15
	vbroadcastss	0(%rbx), %ymm12
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8

	subl		$ 1, %r10d // k-1
	addq		$ 32, %r11 // A+1*bs*sizeof(float)
	addl		$ 1, %r15d // offsetB+1

	cmpl		$ 0, %r10d
	jle			0f // end

	vmovaps			0(%r11), %ymm13
	vmovaps			0(%r11, %r12, 1), %ymm14
	vmovaps			0(%r11, %r12, 2), %ymm15
	vbroadcastss	4(%rbx), %ymm12
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	36(%rbx), %ymm12
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9

	subl		$ 1, %r10d // k-1
	addq		$ 32, %r11 // A+1*bs*sizeof(float)
	addl		$ 1, %r15d // offsetB+1

	cmpl		$ 0, %r10d
	jle			0f // end

	vmovaps			0(%r11), %ymm13
	vmovaps			0(%r11, %r12, 1), %ymm14
	vmovaps			0(%r11, %r12, 2), %ymm15
	vbroadcastss	8(%rbx), %ymm12
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	40(%rbx), %ymm12
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	72(%rbx), %ymm12
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10

	subl		$ 1, %r10d // k-1
	addq		$ 32, %r11 // A+1*bs*sizeof(float)
	addq		%r14, %r13 // B+8*sdb*sizeof(float)
	movl		$ 0, %r15d // offsetB=0

	jmp			0f // end


1:
	cmpl	$ 6, %r15d
	jg		1f

	// offB==6

	vmovaps			0(%r11), %ymm13
	vmovaps			0(%r11, %r12, 1), %ymm14
	vmovaps			0(%r11, %r12, 2), %ymm15
	vbroadcastss	0(%rbx), %ymm12
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8

	subl		$ 1, %r10d // k-1
	addq		$ 32, %r11 // A+1*bs*sizeof(float)
	addl		$ 1, %r15d // offsetB+1

	cmpl		$ 0, %r10d
	jle			0f // end

	vmovaps			0(%r11), %ymm13
	vmovaps			0(%r11, %r12, 1), %ymm14
	vmovaps			0(%r11, %r12, 2), %ymm15
	vbroadcastss	4(%rbx), %ymm12
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	36(%rbx), %ymm12
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9

	subl		$ 1, %r10d // k-1
	addq		$ 32, %r11 // A+1*bs*sizeof(float)
	addq		%r14, %r13 // B+8*sdb*sizeof(float)
	movq		%r13, %rbx // B
	movl		$ 0, %r15d // offsetB=0

	cmpl		$ 0, %r10d
	jle			0f // end

	vmovaps			0(%r11), %ymm13
	vmovaps			0(%r11, %r12, 1), %ymm14
	vmovaps			0(%r11, %r12, 2), %ymm15
	vbroadcastss	0(%rbx), %ymm12
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	32(%rbx), %ymm12
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	64(%rbx), %ymm12
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10

	subl		$ 1, %r10d // k-1
	addq		$ 32, %r11 // A+1*bs*sizeof(float)
	addl		$ 1, %r15d // offsetB+1

	jmp			0f // end


1:
//	cmpl	$ 7, %r15d
//	jg		0f

	// offB==6

	vmovaps			0(%r11), %ymm13
	vmovaps			0(%r11, %r12, 1), %ymm14
	vmovaps			0(%r11, %r12, 2), %ymm15
	vbroadcastss	0(%rbx), %ymm12
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8

	subl		$ 1, %r10d // k-1
	addq		$ 32, %r11 // A+1*bs*sizeof(float)
	addq		%r14, %r13 // B+8*sdb*sizeof(float)
	movq		%r13, %rbx // B
	movl		$ 0, %r15d // offsetB=0

	cmpl		$ 0, %r10d
	jle			0f // end

	vmovaps			0(%r11), %ymm13
	vmovaps			0(%r11, %r12, 1), %ymm14
	vmovaps			0(%r11, %r12, 2), %ymm15
	vbroadcastss	0(%rbx), %ymm12
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	32(%rbx), %ymm12
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9

	subl		$ 1, %r10d // k-1
	addq		$ 32, %r11 // A+1*bs*sizeof(float)
	addl		$ 1, %r15d // offsetB+1

	cmpl		$ 0, %r10d
	jle			0f // end

	vmovaps			0(%r11), %ymm13
	vmovaps			0(%r11, %r12, 1), %ymm14
	vmovaps			0(%r11, %r12, 2), %ymm15
	vbroadcastss	4(%rbx), %ymm12
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	36(%rbx), %ymm12
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	68(%rbx), %ymm12
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10

	subl		$ 1, %r10d // k-1
	addq		$ 32, %r11 // A+1*bs*sizeof(float)
	addl		$ 1, %r15d // offsetB+1

//	jmp			0f // end


	// end
0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trmm_nn_rl_24x4_lib8)
#endif





// common inner routine with file scope
//
// strsm
// right
// lower
// transposed
// not-unit
//
// input arguments:
// r10  <- D
// r11  <- inv_diag_D
// r12d <- kn
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10  <- D
// r11  <- inv_diag_D
// r12d <- kn
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRSM_RLT_INV_24X4_VS_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_trsm_rlt_inv_24x4_vs_lib8)
#endif

	vbroadcastss	0(%r11), %ymm13
	vmulps			%ymm0, %ymm13, %ymm0
	vmulps			%ymm4, %ymm13, %ymm4
	vmulps			%ymm8, %ymm13, %ymm8
	cmpl			$ 2, %r12d
	jl				0f // ret
	vbroadcastss	4(%r10), %ymm13
	vfnmadd231ps	%ymm0, %ymm13, %ymm1
	vfnmadd231ps	%ymm4, %ymm13, %ymm5
	vfnmadd231ps	%ymm8, %ymm13, %ymm9
	vbroadcastss	8(%r10), %ymm13
	vfnmadd231ps	%ymm0, %ymm13, %ymm2
	vfnmadd231ps	%ymm4, %ymm13, %ymm6
	vfnmadd231ps	%ymm8, %ymm13, %ymm10
	vbroadcastss	12(%r10), %ymm13
	vfnmadd231ps	%ymm0, %ymm13, %ymm3
	vfnmadd231ps	%ymm4, %ymm13, %ymm7
	vfnmadd231ps	%ymm8, %ymm13, %ymm11

	vbroadcastss	4(%r11), %ymm13
	vmulps			%ymm1, %ymm13, %ymm1
	vmulps			%ymm5, %ymm13, %ymm5
	vmulps			%ymm9, %ymm13, %ymm9
	cmpl			$ 3, %r12d
	jl				0f // ret
	vbroadcastss	40(%r10), %ymm13
	vfnmadd231ps	%ymm1, %ymm13, %ymm2
	vfnmadd231ps	%ymm5, %ymm13, %ymm6
	vfnmadd231ps	%ymm9, %ymm13, %ymm10
	vbroadcastss	44(%r10), %ymm13
	vfnmadd231ps	%ymm1, %ymm13, %ymm3
	vfnmadd231ps	%ymm5, %ymm13, %ymm7
	vfnmadd231ps	%ymm9, %ymm13, %ymm11

	vbroadcastss	8(%r11), %ymm13
	vmulps			%ymm2, %ymm13, %ymm2
	vmulps			%ymm6, %ymm13, %ymm6
	vmulps			%ymm10, %ymm13, %ymm10
	cmpl			$ 4, %r12d
	jl				0f // ret
	vbroadcastss	76(%r10), %ymm13
	vfnmadd231ps	%ymm2, %ymm13, %ymm3
	vfnmadd231ps	%ymm6, %ymm13, %ymm7
	vfnmadd231ps	%ymm10, %ymm13, %ymm11

	vbroadcastss	12(%r11), %ymm13
	vmulps			%ymm3, %ymm13, %ymm3
	vmulps			%ymm7, %ymm13, %ymm7
	vmulps			%ymm11, %ymm13, %ymm11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trsm_rlt_inv_24x4_vs_lib8)
#endif





// common inner routine with file scope
//
// cholesky factorization gen
//
// input arguments:
// r10  <- inv_diag_E
// r11d <- kn
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []
// ymm4 <- []
// ymm5 <- []
// ymm6 <- []
// ymm7 <- []
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10  <- inv_diag_E
// r11d <- kn
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []
// ymm4 <- []
// ymm5 <- []
// ymm6 <- []
// ymm7 <- []
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_POTRF_24X4_VS_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_potrf_24x4_vs_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovss	.LC03(%rip), %xmm14 // 1.0
#elif defined(OS_MAC)
	vmovss	LC03(%rip), %xmm14 // 1.0
#endif

	vmovss			%xmm0, %xmm0, %xmm13
	vxorps	%ymm15, %ymm15, %ymm15 // 0.0
	vucomiss		%xmm15, %xmm13 // d_00 > 0.0 ?
	jbe			1f
	vsqrtss			%xmm13, %xmm13, %xmm13
	vdivss			%xmm13, %xmm14, %xmm13
2:
	vmovss			%xmm13, 0(%r10)
	vpermilps		$ 0x00, %xmm13, %xmm13
	vinsertf128		$ 0x1, %xmm13, %ymm13, %ymm13
	vmulps			%ymm0, %ymm13, %ymm0
	vmulps			%ymm4, %ymm13, %ymm4
	vmulps			%ymm8, %ymm13, %ymm8
	cmpl		$ 2, %r11d
	jl			0f // ret
	vperm2f128		$ 0x00, %ymm0, %ymm0, %ymm15
	vpermilps		$ 0x55, %ymm15, %ymm13
	vfnmadd231ps	%ymm0, %ymm13, %ymm1
	vfnmadd231ps	%ymm4, %ymm13, %ymm5
	vfnmadd231ps	%ymm8, %ymm13, %ymm9
	vpermilps		$ 0xaa, %ymm15, %ymm13
	vfnmadd231ps	%ymm0, %ymm13, %ymm2
	vfnmadd231ps	%ymm4, %ymm13, %ymm6
	vfnmadd231ps	%ymm8, %ymm13, %ymm10
	vpermilps		$ 0xff, %ymm15, %ymm13
	vfnmadd231ps	%ymm0, %ymm13, %ymm3
	vfnmadd231ps	%ymm4, %ymm13, %ymm7
	vfnmadd231ps	%ymm8, %ymm13, %ymm11


	vpermilps		$ 0x55, %xmm1, %xmm13
	vxorps	%ymm15, %ymm15, %ymm15 // 0.0
	vucomiss		%xmm15, %xmm13 // d_11 > 0.0 ?
	jbe			3f
	vsqrtss			%xmm13, %xmm13, %xmm13
	vdivss			%xmm13, %xmm14, %xmm13
4:
	vmovss			%xmm13, 4(%r10)
	vpermilps		$ 0x00, %xmm13, %xmm13
	vinsertf128		$ 0x1, %xmm13, %ymm13, %ymm13
	vmulps			%ymm1, %ymm13, %ymm1
	vmulps			%ymm5, %ymm13, %ymm5
	vmulps			%ymm9, %ymm13, %ymm9
	cmpl		$ 3, %r11d
	jl			0f // ret
	vperm2f128		$ 0x00, %ymm1, %ymm1, %ymm15
	vpermilps		$ 0xaa, %ymm15, %ymm13
	vfnmadd231ps	%ymm1, %ymm13, %ymm2
	vfnmadd231ps	%ymm5, %ymm13, %ymm6
	vfnmadd231ps	%ymm9, %ymm13, %ymm10
	vpermilps		$ 0xff, %ymm15, %ymm13
	vfnmadd231ps	%ymm1, %ymm13, %ymm3
	vfnmadd231ps	%ymm5, %ymm13, %ymm7
	vfnmadd231ps	%ymm9, %ymm13, %ymm11


	vpermilps		$ 0xaa, %xmm2, %xmm13
	vxorps	%ymm15, %ymm15, %ymm15 // 0.0
	vucomiss		%xmm15, %xmm13 // d_22 > 0.0 ?
	jbe			5f
	vsqrtss			%xmm13, %xmm13, %xmm13
	vdivss			%xmm13, %xmm14, %xmm13
6:
	vmovss			%xmm13, 8(%r10)
	vpermilps		$ 0x00, %xmm13, %xmm13
	vinsertf128		$ 0x1, %xmm13, %ymm13, %ymm13
	vmulps			%ymm2, %ymm13, %ymm2
	vmulps			%ymm6, %ymm13, %ymm6
	vmulps			%ymm10, %ymm13, %ymm10
	cmpl		$ 4, %r11d
	jl			0f // ret
	vperm2f128		$ 0x00, %ymm2, %ymm2, %ymm15
	vpermilps		$ 0xff, %ymm15, %ymm13
	vfnmadd231ps	%ymm2, %ymm13, %ymm3
	vfnmadd231ps	%ymm6, %ymm13, %ymm7
	vfnmadd231ps	%ymm10, %ymm13, %ymm11


	vpermilps		$ 0xff, %xmm3, %xmm13
	vxorps	%ymm15, %ymm15, %ymm15 // 0.0
	vucomiss		%xmm15, %xmm13 // d_33 > 0.0 ?
	jbe			7f
	vsqrtss			%xmm13, %xmm13, %xmm13
	vdivss			%xmm13, %xmm14, %xmm13
8:
	vmovsd			%xmm13, 12(%r10)
	vpermilps		$ 0x00, %xmm13, %xmm13
	vinsertf128		$ 0x1, %xmm13, %ymm13, %ymm13
	vmulps			%ymm3, %ymm13, %ymm3
	vmulps			%ymm7, %ymm13, %ymm7
	vmulps			%ymm11, %ymm13, %ymm11

	jmp		0f


1:
	vxorps			%ymm13, %ymm13, %ymm13
	jmp		2b

3:
	vxorpd			%ymm13, %ymm13, %ymm13
	jmp		4b

5:
	vxorpd			%ymm13, %ymm13, %ymm13
	jmp		6b

7:
	vxorpd			%ymm13, %ymm13, %ymm13
	jmp		8b

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_potrf_24x4_vs_lib8)
#endif





// common inner routine with file scope
//
// cholesky factorization gen
//
// input arguments:
// r10  <- inv_diag_E
// r11d <- kn
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []
// ymm4 <- []
// ymm5 <- []
// ymm6 <- []
// ymm7 <- []
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10  <- inv_diag_E
// r11d <- kn
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []
// ymm4 <- []
// ymm5 <- []
// ymm6 <- []
// ymm7 <- []
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_POTRF_20X4_VS_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_potrf_20x4_vs_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovss	.LC03(%rip), %xmm14 // 1.0
#elif defined(OS_MAC)
	vmovss	LC03(%rip), %xmm14 // 1.0
#endif

	vextractf128	$ 0x1, %ymm0, %xmm13
//	vpermilps		$ 0x00, %xmm13, %xmm13
	vxorps	%ymm15, %ymm15, %ymm15 // 0.0
	vucomiss		%xmm15, %xmm13 // d_00 > 0.0 ?
	jbe			1f
	vsqrtss			%xmm13, %xmm13, %xmm13
	vdivss			%xmm13, %xmm14, %xmm13
2:
	vmovss			%xmm13, 0(%r10)
	vpermilps		$ 0x00, %xmm13, %xmm13
	vinsertf128		$ 0x1, %xmm13, %ymm13, %ymm13
	vmulps			%ymm0, %ymm13, %ymm0
	vmulps			%ymm4, %ymm13, %ymm4
	vmulps			%ymm8, %ymm13, %ymm8
	cmpl		$ 2, %r11d
	jl			0f // ret
	vperm2f128		$ 0x11, %ymm0, %ymm0, %ymm15
	vpermilps		$ 0x55, %ymm15, %ymm13
	vfnmadd231ps	%ymm0, %ymm13, %ymm1
	vfnmadd231ps	%ymm4, %ymm13, %ymm5
	vfnmadd231ps	%ymm8, %ymm13, %ymm9
	vpermilps		$ 0xaa, %ymm15, %ymm13
	vfnmadd231ps	%ymm0, %ymm13, %ymm2
	vfnmadd231ps	%ymm4, %ymm13, %ymm6
	vfnmadd231ps	%ymm8, %ymm13, %ymm10
	vpermilps		$ 0xff, %ymm15, %ymm13
	vfnmadd231ps	%ymm0, %ymm13, %ymm3
	vfnmadd231ps	%ymm4, %ymm13, %ymm7
	vfnmadd231ps	%ymm8, %ymm13, %ymm11


	vextractf128	$ 0x1, %ymm1, %xmm13
	vpermilps		$ 0x55, %xmm13, %xmm13
	vxorps	%ymm15, %ymm15, %ymm15 // 0.0
	vucomiss		%xmm15, %xmm13 // d_11 > 0.0 ?
	jbe			3f
	vsqrtss			%xmm13, %xmm13, %xmm13
	vdivss			%xmm13, %xmm14, %xmm13
4:
	vmovss			%xmm13, 4(%r10)
	vpermilps		$ 0x00, %xmm13, %xmm13
	vinsertf128		$ 0x1, %xmm13, %ymm13, %ymm13
	vmulps			%ymm1, %ymm13, %ymm1
	vmulps			%ymm5, %ymm13, %ymm5
	vmulps			%ymm9, %ymm13, %ymm9
	cmpl		$ 3, %r11d
	jl			0f // ret
	vperm2f128		$ 0x11, %ymm1, %ymm1, %ymm15
	vpermilps		$ 0xaa, %ymm15, %ymm13
	vfnmadd231ps	%ymm1, %ymm13, %ymm2
	vfnmadd231ps	%ymm5, %ymm13, %ymm6
	vfnmadd231ps	%ymm9, %ymm13, %ymm10
	vpermilps		$ 0xff, %ymm15, %ymm13
	vfnmadd231ps	%ymm1, %ymm13, %ymm3
	vfnmadd231ps	%ymm5, %ymm13, %ymm7
	vfnmadd231ps	%ymm9, %ymm13, %ymm11


	vextractf128	$ 0x1, %ymm2, %xmm13
	vpermilps		$ 0xaa, %xmm13, %xmm13
	vxorps	%ymm15, %ymm15, %ymm15 // 0.0
	vucomiss		%xmm15, %xmm13 // d_22 > 0.0 ?
	jbe			5f
	vsqrtss			%xmm13, %xmm13, %xmm13
	vdivss			%xmm13, %xmm14, %xmm13
6:
	vmovss			%xmm13, 8(%r10)
	vpermilps		$ 0x00, %xmm13, %xmm13
	vinsertf128		$ 0x1, %xmm13, %ymm13, %ymm13
	vmulps			%ymm2, %ymm13, %ymm2
	vmulps			%ymm6, %ymm13, %ymm6
	vmulps			%ymm10, %ymm13, %ymm10
	cmpl		$ 4, %r11d
	jl			0f // ret
	vperm2f128		$ 0x11, %ymm2, %ymm2, %ymm15
	vpermilps		$ 0xff, %ymm15, %ymm13
	vfnmadd231ps	%ymm2, %ymm13, %ymm3
	vfnmadd231ps	%ymm6, %ymm13, %ymm7
	vfnmadd231ps	%ymm10, %ymm13, %ymm11


	vextractf128	$ 0x1, %ymm3, %xmm13
	vpermilps		$ 0xff, %xmm13, %xmm13
	vxorps	%ymm15, %ymm15, %ymm15 // 0.0
	vucomiss		%xmm15, %xmm13 // d_33 > 0.0 ?
	jbe			7f
	vsqrtss			%xmm13, %xmm13, %xmm13
	vdivss			%xmm13, %xmm14, %xmm13
8:
	vmovsd			%xmm13, 12(%r10)
	vpermilps		$ 0x00, %xmm13, %xmm13
	vinsertf128		$ 0x1, %xmm13, %ymm13, %ymm13
	vmulps			%ymm3, %ymm13, %ymm3
	vmulps			%ymm7, %ymm13, %ymm7
	vmulps			%ymm11, %ymm13, %ymm11

	jmp		0f


1:
	vxorps			%ymm13, %ymm13, %ymm13
	jmp		2b

3:
	vxorpd			%ymm13, %ymm13, %ymm13
	jmp		4b

5:
	vxorpd			%ymm13, %ymm13, %ymm13
	jmp		6b

7:
	vxorpd			%ymm13, %ymm13, %ymm13
	jmp		8b

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_potrf_20x4_vs_lib8)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- 4*sdc*sizeof(float)
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- 4*sdc*sizeof(float)
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_24X4_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_24x4_lib8)
#endif
	
	// alpha
	vbroadcastss	0(%r10), %ymm15

	vmulps		%ymm0, %ymm15, %ymm0
	vmulps		%ymm1, %ymm15, %ymm1
	vmulps		%ymm2, %ymm15, %ymm2
	vmulps		%ymm3, %ymm15, %ymm3

	vmulps		%ymm4, %ymm15, %ymm4
	vmulps		%ymm5, %ymm15, %ymm5
	vmulps		%ymm6, %ymm15, %ymm6
	vmulps		%ymm7, %ymm15, %ymm7

	vmulps		%ymm8, %ymm15, %ymm8
	vmulps		%ymm9, %ymm15, %ymm9
	vmulps		%ymm10, %ymm15, %ymm10
	vmulps		%ymm11, %ymm15, %ymm11

	// beta
	vbroadcastss	0(%r11), %ymm14

	vxorps		%ymm15, %ymm15, %ymm15 // 0.0

	vucomiss	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	vmovaps		0(%r12), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm0
	vmovaps		32(%r12), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm1
	vmovaps		64(%r12), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm2
	vmovaps		96(%r12), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm3

	vmovaps		0(%r12, %r13, 1), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm4
	vmovaps		32(%r12, %r13, 1), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm5
	vmovaps		64(%r12, %r13, 1), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm6
	vmovaps		96(%r12, %r13, 1), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm7

	vmovaps		0(%r12, %r13, 2), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm8
	vmovaps		32(%r12, %r13, 2), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm9
	vmovaps		64(%r12, %r13, 2), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm10
	vmovaps		96(%r12, %r13, 2), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_24x4_lib8)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta
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
	.macro INNER_SCALE_AB_24X4_GEN_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_24x4_gen_lib8)
#endif
	
	// alpha
	vbroadcastss	0(%r10), %ymm15

	vmulps		%ymm0, %ymm15, %ymm0
	vmulps		%ymm1, %ymm15, %ymm1
	vmulps		%ymm2, %ymm15, %ymm2
	vmulps		%ymm3, %ymm15, %ymm3

	vmulps		%ymm4, %ymm15, %ymm4
	vmulps		%ymm5, %ymm15, %ymm5
	vmulps		%ymm6, %ymm15, %ymm6
	vmulps		%ymm7, %ymm15, %ymm7

	vmulps		%ymm8, %ymm15, %ymm8
	vmulps		%ymm9, %ymm15, %ymm9
	vmulps		%ymm10, %ymm15, %ymm10
	vmulps		%ymm11, %ymm15, %ymm11

	// beta
	vbroadcastss	0(%r11), %ymm14

	vxorps		%ymm15, %ymm15, %ymm15 // 0.0

	vucomiss	%xmm15, %xmm14 // beta==0.0 ?
	je			3f // end

	movq	%r13, %r15 // C1 <- C0
	addq	%r14, %r15 // C1 <- C0 + 4*sdc*sizeof(double)
	movq	%r15, %rax // C2 <- C1
	addq	%r14, %rax // C2 <- C1 + 4*sdc*sizeof(double)

	cmpl	$ 0, %r12d
	jg		0f

	// offset==0

	vmovaps		0(%r13), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm0
	vmovaps		32(%r13), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm1
	vmovaps		64(%r13), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm2
	vmovaps		96(%r13), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm3

	vmovaps		0(%r15), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm4
	vmovaps		32(%r15), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm5
	vmovaps		64(%r15), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm6
	vmovaps		96(%r15), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm7

	vmovaps		0(%rax), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm8
	vmovaps		32(%rax), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm9
	vmovaps		64(%rax), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm10
	vmovaps		96(%rax), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm11

	jmp		7f

0:

	// offset > 0
	// 1 2 3 4 5 6 7
	
	movq	%rax, %rbx // C1
	addq	%r14, %rbx // C2 <- C1 + 4*sdc*sizeof(double)

	cmpl	$ 4, %r12d
	jl		1f
	jg		2f

	// offset==4
	// TODO
	jmp		7f

1:
	// 1 2 3

	cmpl	$ 2, %r12d
	jl		3f
	jg		4f

	// offset==2
	// TODO
	jmp		7f

3:
	// offset==1
	// TODO
	jmp		7f

4:
	// offset==3
	// TODO
	jmp		7f

2:
	// 5 6 7

	cmpl	$ 6, %r12d
	jl		5f
	jg		6f

	// offset==6
	// TODO
	jmp		7f

5:
	// offset==5
	// TODO
	jmp		7f

6:
	// offset==7
	// TODO
	jmp		7f

	// end
7:


#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_24x4_gen_lib8)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta=0
//
// input arguments:
// r10   <- alpha
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10   <- alpha
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_A0_24X4_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_scale_a0_24x4_lib8)
#endif
	
	// alpha
	vbroadcastss	0(%r10), %ymm12

	vmulps		%ymm0, %ymm12, %ymm0
	vmulps		%ymm1, %ymm12, %ymm1
	vmulps		%ymm2, %ymm12, %ymm2
	vmulps		%ymm3, %ymm12, %ymm3

	vmulps		%ymm4, %ymm12, %ymm4
	vmulps		%ymm5, %ymm12, %ymm5
	vmulps		%ymm6, %ymm12, %ymm6
	vmulps		%ymm7, %ymm12, %ymm7

	vmulps		%ymm8, %ymm12, %ymm8
	vmulps		%ymm9, %ymm12, %ymm9
	vmulps		%ymm10, %ymm12, %ymm10
	vmulps		%ymm11, %ymm12, %ymm11

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_a0_24x4_lib8)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta
//
// input arguments:
// r10   <- C
// r11   <- 4*sdc*sizeof(float)
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10   <- C
// r11   <- 4*sdc*sizeof(float)
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_11_24X4_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_scale_11_24x4_lib8)
#endif
	
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovaps		.LC03(%rip), %ymm14
#elif defined(OS_MAC)
	vmovaps		LC03(%rip), %ymm14
#endif

	vmovaps		0(%r10), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm0
	vmovaps		32(%r10), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm1
	vmovaps		64(%r10), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm2
	vmovaps		96(%r10), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm3

	vmovaps		0(%r10, %r11, 1), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm4
	vmovaps		32(%r10, %r11, 1), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm5
	vmovaps		64(%r10, %r11, 1), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm6
	vmovaps		96(%r10, %r11, 1), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm7

	vmovaps		0(%r10, %r11, 2), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm8
	vmovaps		32(%r10, %r11, 2), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm9
	vmovaps		64(%r10, %r11, 2), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm10
	vmovaps		96(%r10, %r11, 2), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_11_24x4_lib8)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta
//
// input arguments:
// r10  <- offset
// r11   <- C
// r12  <- 4*sdc*sizeof(double)
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
// r10  <- offset
// r11   <- C
// r12  <- 4*sdc*sizeof(double)
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
	.macro INNER_SCALE_11_24X4_GEN_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_scale_11_24x4_gen_lib8)
#endif
	
	cmpl	$ 0, %r10d
	jg		0f

	// offset==0

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovaps		.LC03(%rip), %ymm14
#elif defined(OS_MAC)
	vmovaps		LC03(%rip), %ymm14
#endif

	vmovaps		0(%r11), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm0
	vmovaps		32(%r11), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm1
	vmovaps		64(%r11), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm2
	vmovaps		96(%r11), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm3

	vmovaps		0(%r11, %r12, 1), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm4
	vmovaps		32(%r11, %r12, 1), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm5
	vmovaps		64(%r11, %r12, 1), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm6
	vmovaps		96(%r11, %r12, 1), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm7

	vmovaps		0(%r11, %r12, 2), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm8
	vmovaps		32(%r11, %r12, 2), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm9
	vmovaps		64(%r11, %r12, 2), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm10
	vmovaps		96(%r11, %r12, 2), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm11

	jmp		7f

0:

	// offset > 0
	// 1 2 3 4 5 6 7
	
	cmpl	$ 4, %r10d
	jl		1f
	jg		2f

	// offset==4
	// TODO
	jmp		7f

1:
	// 1 2 3

	cmpl	$ 2, %r10d
	jl		3f
	jg		4f

	// offset==2
	// TODO
	jmp		7f

3:
	// offset==1
	// TODO
	jmp		7f

4:
	// offset==3
	// TODO
	jmp		7f

2:
	// 5 6 7

	cmpl	$ 6, %r10d
	jl		5f
	jg		6f

	// offset==6
	// TODO
	jmp		7f

5:
	// offset==5
	// TODO
	jmp		7f

6:
	// offset==7
	// TODO
	jmp		7f

	// end
7:


#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_11_24x4_gen_lib8)
#endif





// common inner routine with file scope
//
// blend for generic alpha and beta
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- 4*sdc*sizeof(float)
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- 4*sdc*sizeof(float)
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_SCALE_AB_24X4_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_blend_scale_ab_24x4_lib8)
#endif
	
	vblendps	$ 0xaa, %ymm1, %ymm0, %ymm12 // 1010 1010
	vblendps	$ 0x55, %ymm1, %ymm0, %ymm13 // 0101 0101
	vblendps	$ 0xaa, %ymm3, %ymm2, %ymm14
	vblendps	$ 0x55, %ymm3, %ymm2, %ymm15

	vblendps	$ 0xcc, %ymm15, %ymm12, %ymm0 // 1100 1100
	vblendps	$ 0x33, %ymm15, %ymm12, %ymm2 // 0011 0011
	vblendps	$ 0xcc, %ymm14, %ymm13, %ymm1
	vblendps	$ 0x33, %ymm14, %ymm13, %ymm3

	// alpha
	vbroadcastss	0(%r10), %ymm15

	vmulps		%ymm0, %ymm15, %ymm0
	vmulps		%ymm1, %ymm15, %ymm1
	vmulps		%ymm2, %ymm15, %ymm2
	vmulps		%ymm3, %ymm15, %ymm3

	vblendps	$ 0xaa, %ymm5, %ymm4, %ymm12
	vblendps	$ 0x55, %ymm5, %ymm4, %ymm13
	vblendps	$ 0xaa, %ymm7, %ymm6, %ymm14
	vblendps	$ 0x55, %ymm7, %ymm6, %ymm15

	vblendps	$ 0xcc, %ymm15, %ymm12, %ymm4
	vblendps	$ 0x33, %ymm15, %ymm12, %ymm6
	vblendps	$ 0xcc, %ymm14, %ymm13, %ymm5
	vblendps	$ 0x33, %ymm14, %ymm13, %ymm7

	// alpha
	vbroadcastss	0(%r10), %ymm15

	vmulps		%ymm4, %ymm15, %ymm4
	vmulps		%ymm5, %ymm15, %ymm5
	vmulps		%ymm6, %ymm15, %ymm6
	vmulps		%ymm7, %ymm15, %ymm7

	vblendps	$ 0xaa, %ymm9, %ymm8, %ymm12
	vblendps	$ 0x55, %ymm9, %ymm8, %ymm13
	vblendps	$ 0xaa, %ymm11, %ymm10, %ymm14
	vblendps	$ 0x55, %ymm11, %ymm10, %ymm15

	vblendps	$ 0xcc, %ymm15, %ymm12, %ymm8
	vblendps	$ 0x33, %ymm15, %ymm12, %ymm10
	vblendps	$ 0xcc, %ymm14, %ymm13, %ymm9
	vblendps	$ 0x33, %ymm14, %ymm13, %ymm11

	// alpha
	vbroadcastss	0(%r10), %ymm15

	vmulps		%ymm8, %ymm15, %ymm8
	vmulps		%ymm9, %ymm15, %ymm9
	vmulps		%ymm10, %ymm15, %ymm10
	vmulps		%ymm11, %ymm15, %ymm11

	// beta
	vbroadcastss	0(%r11), %ymm14

	vxorps		%ymm15, %ymm15, %ymm15 // 0.0

	vucomiss	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	movq	%r12, %r15 // C1 <- C0
	addq	%r13, %r15 // C1 <- C0 + 4*sdc*sizeof(double)
	movq	%r15, %rax // C2 <- C1
	addq	%r13, %rax // C2 <- C1 + 4*sdc*sizeof(double)

	vmovaps		0(%r12), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm0
	vmovaps		32(%r12), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm1
	vmovaps		64(%r12), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm2
	vmovaps		96(%r12), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm3

	vmovaps		0(%r15), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm4
	vmovaps		32(%r15), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm5
	vmovaps		64(%r15), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm6
	vmovaps		96(%r15), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm7

	vmovaps		0(%rax), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm8
	vmovaps		32(%rax), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm9
	vmovaps		64(%rax), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm10
	vmovaps		96(%rax), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_blend_scale_ab_24x4_lib8)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta
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
	.macro INNER_BLEND_SCALE_AB_24X4_GEN_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_blend_scale_ab_24x4_gen_lib8)
#endif
	
	vblendps	$ 0xaa, %ymm1, %ymm0, %ymm12 // 1010 1010
	vblendps	$ 0x55, %ymm1, %ymm0, %ymm13 // 0101 0101
	vblendps	$ 0xaa, %ymm3, %ymm2, %ymm14
	vblendps	$ 0x55, %ymm3, %ymm2, %ymm15

	vblendps	$ 0xcc, %ymm15, %ymm12, %ymm0 // 1100 1100
	vblendps	$ 0x33, %ymm15, %ymm12, %ymm2 // 0011 0011
	vblendps	$ 0xcc, %ymm14, %ymm13, %ymm1
	vblendps	$ 0x33, %ymm14, %ymm13, %ymm3

	// alpha
	vbroadcastss	0(%r10), %ymm15

	vmulps		%ymm0, %ymm15, %ymm0
	vmulps		%ymm1, %ymm15, %ymm1
	vmulps		%ymm2, %ymm15, %ymm2
	vmulps		%ymm3, %ymm15, %ymm3

	vblendps	$ 0xaa, %ymm5, %ymm4, %ymm12
	vblendps	$ 0x55, %ymm5, %ymm4, %ymm13
	vblendps	$ 0xaa, %ymm7, %ymm6, %ymm14
	vblendps	$ 0x55, %ymm7, %ymm6, %ymm15

	vblendps	$ 0xcc, %ymm15, %ymm12, %ymm4
	vblendps	$ 0x33, %ymm15, %ymm12, %ymm6
	vblendps	$ 0xcc, %ymm14, %ymm13, %ymm5
	vblendps	$ 0x33, %ymm14, %ymm13, %ymm7

	// alpha
	vbroadcastss	0(%r10), %ymm15

	vmulps		%ymm4, %ymm15, %ymm4
	vmulps		%ymm5, %ymm15, %ymm5
	vmulps		%ymm6, %ymm15, %ymm6
	vmulps		%ymm7, %ymm15, %ymm7

	vblendps	$ 0xaa, %ymm9, %ymm8, %ymm12
	vblendps	$ 0x55, %ymm9, %ymm8, %ymm13
	vblendps	$ 0xaa, %ymm11, %ymm10, %ymm14
	vblendps	$ 0x55, %ymm11, %ymm10, %ymm15

	vblendps	$ 0xcc, %ymm15, %ymm12, %ymm8
	vblendps	$ 0x33, %ymm15, %ymm12, %ymm10
	vblendps	$ 0xcc, %ymm14, %ymm13, %ymm9
	vblendps	$ 0x33, %ymm14, %ymm13, %ymm11

	// alpha
	vbroadcastss	0(%r10), %ymm15

	vmulps		%ymm8, %ymm15, %ymm8
	vmulps		%ymm9, %ymm15, %ymm9
	vmulps		%ymm10, %ymm15, %ymm10
	vmulps		%ymm11, %ymm15, %ymm11

	// beta
	vbroadcastss	0(%r11), %ymm14

	vxorps		%ymm15, %ymm15, %ymm15 // 0.0

	vucomiss	%xmm15, %xmm14 // beta==0.0 ?
	je			3f // end

	movq	%r13, %r15 // C1 <- C0
	addq	%r14, %r15 // C1 <- C0 + 4*sdc*sizeof(double)
	movq	%r15, %rax // C2 <- C1
	addq	%r14, %rax // C2 <- C1 + 4*sdc*sizeof(double)

	cmpl	$ 0, %r12d
	jg		0f

	// offset==0

	vmovaps		0(%r13), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm0
	vmovaps		32(%r13), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm1
	vmovaps		64(%r13), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm2
	vmovaps		96(%r13), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm3

	vmovaps		0(%r15), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm4
	vmovaps		32(%r15), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm5
	vmovaps		64(%r15), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm6
	vmovaps		96(%r15), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm7

	vmovaps		0(%rax), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm8
	vmovaps		32(%rax), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm9
	vmovaps		64(%rax), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm10
	vmovaps		96(%rax), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm11

	jmp		7f

0:

	// offset > 0
	// 1 2 3 4 5 6 7
	
	movq	%rax, %rbx // C1
	addq	%r14, %rbx // C2 <- C1 + 4*sdc*sizeof(double)

	cmpl	$ 4, %r12d
	jl		1f
	jg		2f

	// offset==4
	// TODO
	jmp		7f

1:
	// 1 2 3

	cmpl	$ 2, %r12d
	jl		3f
	jg		4f

	// offset==2
	// TODO
	jmp		7f

3:
	// offset==1
	// TODO
	jmp		7f

4:
	// offset==3
	// TODO
	jmp		7f

2:
	// 5 6 7

	cmpl	$ 6, %r12d
	jl		5f
	jg		6f

	// offset==6
	// TODO
	jmp		7f

5:
	// offset==5
	// TODO
	jmp		7f

6:
	// offset==7
	// TODO
	jmp		7f

	// end
7:


#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_blend_scale_ab_24x4_gen_lib8)
#endif





// common inner routine with file scope
//
// blend for generic alpha and beta
//
// input arguments:
// r10   <- C
// r11   <- 4*sdc*sizeof(float)
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty
//
// output arguments:
// r10   <- C
// r11   <- 4*sdc*sizeof(float)
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_SCALE_11_24X4_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_blend_scale_11_24x4_lib8)
#endif
	
	vblendps	$ 0xaa, %ymm1, %ymm0, %ymm12 // 1010 1010
	vblendps	$ 0x55, %ymm1, %ymm0, %ymm13 // 0101 0101
	vblendps	$ 0xaa, %ymm3, %ymm2, %ymm14
	vblendps	$ 0x55, %ymm3, %ymm2, %ymm15

	vblendps	$ 0xcc, %ymm15, %ymm12, %ymm0 // 1100 1100
	vblendps	$ 0x33, %ymm15, %ymm12, %ymm2 // 0011 0011
	vblendps	$ 0xcc, %ymm14, %ymm13, %ymm1
	vblendps	$ 0x33, %ymm14, %ymm13, %ymm3

	vblendps	$ 0xaa, %ymm5, %ymm4, %ymm12
	vblendps	$ 0x55, %ymm5, %ymm4, %ymm13
	vblendps	$ 0xaa, %ymm7, %ymm6, %ymm14
	vblendps	$ 0x55, %ymm7, %ymm6, %ymm15

	vblendps	$ 0xcc, %ymm15, %ymm12, %ymm4
	vblendps	$ 0x33, %ymm15, %ymm12, %ymm6
	vblendps	$ 0xcc, %ymm14, %ymm13, %ymm5
	vblendps	$ 0x33, %ymm14, %ymm13, %ymm7

	vblendps	$ 0xaa, %ymm9, %ymm8, %ymm12
	vblendps	$ 0x55, %ymm9, %ymm8, %ymm13
	vblendps	$ 0xaa, %ymm11, %ymm10, %ymm14
	vblendps	$ 0x55, %ymm11, %ymm10, %ymm15

	vblendps	$ 0xcc, %ymm15, %ymm12, %ymm8
	vblendps	$ 0x33, %ymm15, %ymm12, %ymm10
	vblendps	$ 0xcc, %ymm14, %ymm13, %ymm9
	vblendps	$ 0x33, %ymm14, %ymm13, %ymm11

	movq	%r10, %r15 // C1 <- C0
	addq	%r11, %r15 // C1 <- C0 + 4*sdc*sizeof(double)
	movq	%r15, %rax // C2 <- C1
	addq	%r11, %rax // C2 <- C1 + 4*sdc*sizeof(double)

	vmovaps		0(%r10), %ymm15
	vaddps		%ymm15, %ymm0, %ymm0
	vmovaps		32(%r10), %ymm15
	vaddps		%ymm15, %ymm1, %ymm1
	vmovaps		64(%r10), %ymm15
	vaddps		%ymm15, %ymm2, %ymm2
	vmovaps		96(%r10), %ymm15
	vaddps		%ymm15, %ymm3, %ymm3

	vmovaps		0(%r15), %ymm15
	vaddps		%ymm15, %ymm4, %ymm4
	vmovaps		32(%r15), %ymm15
	vaddps		%ymm15, %ymm5, %ymm5
	vmovaps		64(%r15), %ymm15
	vaddps		%ymm15, %ymm6, %ymm6
	vmovaps		96(%r15), %ymm15
	vaddps		%ymm15, %ymm7, %ymm7

	vmovaps		0(%rax), %ymm15
	vaddps		%ymm15, %ymm8, %ymm8
	vmovaps		32(%rax), %ymm15
	vaddps		%ymm15, %ymm9, %ymm9
	vmovaps		64(%rax), %ymm15
	vaddps		%ymm15, %ymm10, %ymm10
	vmovaps		96(%rax), %ymm15
	vaddps		%ymm15, %ymm11, %ymm11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_blend_scale_11_24x4_lib8)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta
//
// input arguments:
// r10  <- offset
// r11   <- C
// r12  <- 4*sdc*sizeof(double)
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
// r10  <- offset
// r11   <- C
// r12  <- 4*sdc*sizeof(double)
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
	.macro INNER_BLEND_SCALE_11_24X4_GEN_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_blend_scale_11_24x4_gen_lib8)
#endif
	
	vblendps	$ 0xaa, %ymm1, %ymm0, %ymm12 // 1010 1010
	vblendps	$ 0x55, %ymm1, %ymm0, %ymm13 // 0101 0101
	vblendps	$ 0xaa, %ymm3, %ymm2, %ymm14
	vblendps	$ 0x55, %ymm3, %ymm2, %ymm15

	vblendps	$ 0xcc, %ymm15, %ymm12, %ymm0 // 1100 1100
	vblendps	$ 0x33, %ymm15, %ymm12, %ymm2 // 0011 0011
	vblendps	$ 0xcc, %ymm14, %ymm13, %ymm1
	vblendps	$ 0x33, %ymm14, %ymm13, %ymm3

	vblendps	$ 0xaa, %ymm5, %ymm4, %ymm12
	vblendps	$ 0x55, %ymm5, %ymm4, %ymm13
	vblendps	$ 0xaa, %ymm7, %ymm6, %ymm14
	vblendps	$ 0x55, %ymm7, %ymm6, %ymm15

	vblendps	$ 0xcc, %ymm15, %ymm12, %ymm4
	vblendps	$ 0x33, %ymm15, %ymm12, %ymm6
	vblendps	$ 0xcc, %ymm14, %ymm13, %ymm5
	vblendps	$ 0x33, %ymm14, %ymm13, %ymm7

	vblendps	$ 0xaa, %ymm9, %ymm8, %ymm12
	vblendps	$ 0x55, %ymm9, %ymm8, %ymm13
	vblendps	$ 0xaa, %ymm11, %ymm10, %ymm14
	vblendps	$ 0x55, %ymm11, %ymm10, %ymm15

	vblendps	$ 0xcc, %ymm15, %ymm12, %ymm8
	vblendps	$ 0x33, %ymm15, %ymm12, %ymm10
	vblendps	$ 0xcc, %ymm14, %ymm13, %ymm9
	vblendps	$ 0x33, %ymm14, %ymm13, %ymm11

	movq	%r11, %r15 // C1 <- C0
	addq	%r12, %r15 // C1 <- C0 + 4*sdc*sizeof(double)
	movq	%r15, %rax // C2 <- C1
	addq	%r12, %rax // C2 <- C1 + 4*sdc*sizeof(double)

	cmpl	$ 0, %r10d
	jg		0f

	// offset==0

	vmovaps		0(%r11), %ymm15
	vaddps		%ymm15, %ymm0, %ymm0
	vmovaps		32(%r11), %ymm15
	vaddps		%ymm15, %ymm1, %ymm1
	vmovaps		64(%r11), %ymm15
	vaddps		%ymm15, %ymm2, %ymm2
	vmovaps		96(%r11), %ymm15
	vaddps		%ymm15, %ymm3, %ymm3

	vmovaps		0(%r15), %ymm15
	vaddps		%ymm15, %ymm4, %ymm4
	vmovaps		32(%r15), %ymm15
	vaddps		%ymm15, %ymm5, %ymm5
	vmovaps		64(%r15), %ymm15
	vaddps		%ymm15, %ymm6, %ymm6
	vmovaps		96(%r15), %ymm15
	vaddps		%ymm15, %ymm7, %ymm7

	vmovaps		0(%rax), %ymm15
	vaddps		%ymm15, %ymm8, %ymm8
	vmovaps		32(%rax), %ymm15
	vaddps		%ymm15, %ymm9, %ymm9
	vmovaps		64(%rax), %ymm15
	vaddps		%ymm15, %ymm10, %ymm10
	vmovaps		96(%rax), %ymm15
	vaddps		%ymm15, %ymm11, %ymm11

	jmp		7f

0:

	// offset > 0
	// 1 2 3 4 5 6 7
	
	movq	%rax, %rbx // C1
	addq	%r12, %rbx // C2 <- C1 + 4*sdc*sizeof(double)

	cmpl	$ 4, %r10d
	jl		1f
	jg		2f

	// offset==4
	// TODO
	jmp		7f

1:
	// 1 2 3

	cmpl	$ 2, %r10d
	jl		3f
	jg		4f

	// offset==2
	// TODO
	jmp		7f

3:
	// offset==1
	// TODO
	jmp		7f

4:
	// offset==3
	// TODO
	jmp		7f

2:
	// 5 6 7

	cmpl	$ 6, %r10d
	jl		5f
	jg		6f

	// offset==6
	// TODO
	jmp		7f

5:
	// offset==5
	// TODO
	jmp		7f

6:
	// offset==7
	// TODO
	jmp		7f

	// end
7:


#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_blend_scale_11_24x4_gen_lib8)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// r11  <- 4*sdd*sizeof(float)
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []
//
// output arguments:
// r10  <- D
// r11  <- 4*sdd*sizeof(float)
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_24X4_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_store_24x4_lib8)
#endif
	
	movq	%r10, %r15 // D1 <- D0
	addq	%r11, %r15 // D1 <- D0 + 4*sdd*sizeof(double)
	movq	%r15, %rax // D2 <- D1
	addq	%r11, %rax // D2 <- D1 + 4*sdd*sizeof(double)

	vmovaps 	%ymm0,  0(%r10)
	vmovaps 	%ymm1, 32(%r10)
	vmovaps 	%ymm2, 64(%r10)
	vmovaps		%ymm3, 96(%r10)

	vmovaps 	%ymm4,  0(%r15)
	vmovaps 	%ymm5, 32(%r15)
	vmovaps 	%ymm6, 64(%r15)
	vmovaps 	%ymm7, 96(%r15)

	vmovaps 	%ymm8,  0(%rax)
	vmovaps 	%ymm9, 32(%rax)
	vmovaps 	%ymm10, 64(%rax)
	vmovaps 	%ymm11, 96(%rax)


#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_24x4_lib8)
#endif





// common inner routine with file scope
//
// store n vs
//
// input arguments:
// r10  <- D
// r11  <- 4*sdd*sizeof(double)
// r12  <- km
// r13  <- kn
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []
//
// output arguments:
// r10  <- D
// r11  <- 4*sdd*sizeof(double)
// r12  <- km
// r13  <- kn
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_24X4_VS_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_store_24x4_vs_lib8)
#endif
	
	// compute mask for rows
	vcvtsi2ss	%r12d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC02(%rip), %ymm13
#elif defined(OS_MAC)
	vmovups		LC02(%rip), %ymm13
#endif
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%ymm15, %ymm13, %ymm15

	vmovaps		%ymm0,  0(%r10)
	vmovaps		%ymm4, 0(%r10, %r11, 1)
	vmaskmovps	%ymm8, %ymm15,  0(%r10, %r11, 2)
	cmpl		$ 2, %r13d
	jl			0f // end
	vmovaps		%ymm1, 32(%r10)
	vmovaps		%ymm5, 32(%r10, %r11, 1)
	vmaskmovps	%ymm9, %ymm15, 32(%r10, %r11, 2)
	cmpl		$ 3, %r13d
	jl			0f // end
	vmovaps		%ymm2, 64(%r10)
	vmovaps		%ymm6, 64(%r10, %r11, 1)
	vmaskmovps	%ymm10, %ymm15, 64(%r10, %r11, 2)
	je			0f // end
	vmovaps		%ymm3, 96(%r10)
	vmovaps		%ymm7, 96(%r10, %r11, 1)
	vmaskmovps	%ymm11, %ymm15, 96(%r10, %r11, 2)

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_24x4_vs_lib8)
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
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []
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
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_24X4_GEN_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_store_24x4_gen_lib8)
#endif
	
	// compute mask for rows
	vcvtsi2ss	%r13d, %xmm14, %xmm14
	vcvtsi2ss	%r14d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %ymm12
	vmovups		.LC02(%rip), %ymm13
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %ymm12
	vmovups		LC02(%rip), %ymm13
#endif
	vshufps		$ 0x00, %xmm14, %xmm14, %xmm14
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm14, %ymm14, %ymm14
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%ymm12, %ymm14, %ymm14
	vsubps		%ymm15, %ymm13, %ymm15

	// shift D and sol for cols
	cmpl	$ 0, %r15d
	jle		0f

	vmovaps		%ymm1, %ymm0
	vmovaps		%ymm5, %ymm4
	vmovaps		%ymm9, %ymm8
	vmovaps		%ymm2, %ymm1
	vmovaps		%ymm6, %ymm5
	vmovaps		%ymm10, %ymm9
	vmovaps		%ymm3, %ymm2
	vmovaps		%ymm7, %ymm6
	vmovaps		%ymm11, %ymm10
	addq		$ 32, %r11

	cmpl	$ 1, %r15d
	jle		0f

	vmovaps		%ymm1, %ymm0
	vmovaps		%ymm5, %ymm4
	vmovaps		%ymm9, %ymm8
	vmovaps		%ymm2, %ymm1
	vmovaps		%ymm6, %ymm5
	vmovaps		%ymm10, %ymm9
	addq		$ 32, %r11

	cmpl	$ 2, %r15d
	jle		0f

	vmovaps		%ymm1, %ymm0
	vmovaps		%ymm5, %ymm4
	vmovaps		%ymm9, %ymm8
	addq		$ 32, %r11

0:

	// compute D1
	movq	%r11, %rbx // D1
	addq	%r12, %rbx // D1 <- D0 + 4*sdd*sizeof(float)
	movq	%rbx, %rbp // D2
	addq	%r12, %rbp // D2 <- D1 + 4*sdd*sizeof(float)

	// compute number of cols
	cmpl	$ 4, %eax
	jle		0f
	movl	$ 4, %eax
0:
	subl	%r15d, %eax
	movl	%eax, %r15d

	cmpl	$ 0, %r10d
	jg		0f

	// offset==0
	cmpl		$ 2, %r15d
	vmaskmovps	%ymm0, %ymm14,  0(%r11)
	vmovaps		%ymm4, 0(%rbx)
	vmaskmovps	%ymm8, %ymm15,  0(%rbp)
	jl			7f // end
	cmpl		$ 3, %r15d
	vmaskmovps	%ymm1, %ymm14, 32(%r11)
	vmovaps		%ymm5, 32(%rbx)
	vmaskmovps	%ymm9, %ymm15, 32(%rbp)
	jl			7f // end
	vmaskmovps	%ymm2, %ymm14, 64(%r11)
	vmovaps		%ymm6, 64(%rbx)
	vmaskmovps	%ymm10, %ymm15, 64(%rbp)
	je			7f // end
	vmaskmovps	%ymm3, %ymm14, 96(%r11)
	vmovaps		%ymm7, 96(%rbx)
	vmaskmovps	%ymm11, %ymm15, 96(%rbp)
	//
	jmp		7f

0:
	// offset > 0
	// 1 2 3 4 5 6 7
	
//	movq	%r11, %rbp // D1
//	addq	%r12, %rbp // D2 <- D1 + 4*sdd*sizeof(float)
	addq	%rbp, %r12 // D3 <- D2 + 4*sdd*sizeof(float)

	cmpl	$ 4, %r10d
	jl		1f
	jg		2f

	// offset==4
	// TODO
	jmp		7f

1:
	// 1 2 3

	cmpl	$ 2, %r10d
	jl		3f
	jg		4f

	// offset==2
	// TODO
	jmp		7f

3:
	// offset==1
	// TODO
	jmp		7f

4:
	// offset==3
	// TODO
	jmp		7f

2:
	// 5 6 7

	cmpl	$ 6, %r10d
	jl		5f
	jg		6f

	// offset==6
	// TODO
	jmp		7f

5:
	// offset==5
	// TODO
	jmp		7f

6:
	// offset==7
	// TODO
	jmp		7f

	// end
7:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_24x4_gen_lib8)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// r11  <- 4*sdd*sizeof(float)
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []
//
// output arguments:
// r10  <- D
// r11  <- 4*sdd*sizeof(float)
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_20X4_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_store_l_20x4_lib8)
#endif
	
	vmovaps		0(%r10), %ymm12
	vmovaps		32(%r10), %ymm13
	vmovaps		64(%r10), %ymm14
	vmovaps		96(%r10), %ymm15

	vblendps	$ 0x0f, %ymm12, %ymm0, %ymm0
	vblendps	$ 0x1f, %ymm13, %ymm1, %ymm1
	vblendps	$ 0x3f, %ymm14, %ymm2, %ymm2
	vblendps	$ 0x7f, %ymm15, %ymm3, %ymm3

	vmovaps 	%ymm0,  0(%r10)
	vmovaps 	%ymm1, 32(%r10)
	vmovaps 	%ymm2, 64(%r10)
	vmovaps		%ymm3, 96(%r10)

	vmovaps 	%ymm4,  0(%r10, %r11, 1)
	vmovaps 	%ymm5, 32(%r10, %r11, 1)
	vmovaps 	%ymm6, 64(%r10, %r11, 1)
	vmovaps 	%ymm7, 96(%r10, %r11, 1)

	vmovaps 	%ymm8,  0(%r10, %r11, 2)
	vmovaps 	%ymm9, 32(%r10, %r11, 2)
	vmovaps 	%ymm10, 64(%r10, %r11, 2)
	vmovaps 	%ymm11, 96(%r10, %r11, 2)


#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_20x4_lib8)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// r11  <- 4*sdd*sizeof(float)
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []
//
// output arguments:
// r10  <- D
// r11  <- 4*sdd*sizeof(float)
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_24X4_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_store_l_24x4_lib8)
#endif
	
	vmovaps		32(%r10), %ymm12
	vmovaps		64(%r10), %ymm13
	vmovaps		96(%r10), %ymm14

	vblendps	$ 0x01, %ymm12, %ymm1, %ymm1
	vblendps	$ 0x03, %ymm13, %ymm2, %ymm2
	vblendps	$ 0x07, %ymm14, %ymm3, %ymm3

	vmovaps 	%ymm0,  0(%r10)
	vmovaps 	%ymm1, 32(%r10)
	vmovaps 	%ymm2, 64(%r10)
	vmovaps		%ymm3, 96(%r10)

	vmovaps 	%ymm4,  0(%r10, %r11, 1)
	vmovaps 	%ymm5, 32(%r10, %r11, 1)
	vmovaps 	%ymm6, 64(%r10, %r11, 1)
	vmovaps 	%ymm7, 96(%r10, %r11, 1)

	vmovaps 	%ymm8,  0(%r10, %r11, 2)
	vmovaps 	%ymm9, 32(%r10, %r11, 2)
	vmovaps 	%ymm10, 64(%r10, %r11, 2)
	vmovaps 	%ymm11, 96(%r10, %r11, 2)


#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_24x4_lib8)
#endif





// common inner routine with file scope
//
// store n generalized
//
// input arguments:
// r10  <- D
// r11  <- 4*sdd*sizeof(double)
// r12  <- km
// r13  <- kn
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []
//
// output arguments:
// r10  <- D
// r11  <- 4*sdd*sizeof(double)
// r12  <- km
// r13  <- kn
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_20X4_VS_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_store_l_20x4_vs_lib8)
#endif
	
	// compute mask for rows
	vcvtsi2ss	%r12d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC02(%rip), %ymm13
#elif defined(OS_MAC)
	vmovups		LC02(%rip), %ymm13
#endif
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%ymm15, %ymm13, %ymm15

	vmovaps		0(%r10), %ymm12
	vblendps	$ 0x0f, %ymm12, %ymm0, %ymm0
	vmovaps		%ymm0, 0(%r10)
	vmovaps		%ymm4, 0(%r10, %r11, 1)
	vmaskmovps	%ymm8, %ymm15, 0(%r10, %r11, 2)
	cmpl		$ 2, %r13d
	jl			0f // end
	vmovaps		32(%r10), %ymm12
	vblendps	$ 0x1f, %ymm12, %ymm1, %ymm1
	vmovaps		%ymm1, 32(%r10)
	vmovaps		%ymm5, 32(%r10, %r11, 1)
	vmaskmovps	%ymm9, %ymm15, 32(%r10, %r11, 2)
	cmpl		$ 3, %r13d
	jl			0f // end
	vmovaps		64(%r10), %ymm12
	vblendps	$ 0x3f, %ymm12, %ymm2, %ymm2
	vmovaps		%ymm2, 64(%r10)
	vmovaps		%ymm6, 64(%r10, %r11, 1)
	vmaskmovps	%ymm10, %ymm15, 64(%r10, %r11, 2)
	je			0f // end
	vmovaps		96(%r10), %ymm12
	vblendps	$ 0x7f, %ymm12, %ymm3, %ymm3
	vmovaps		%ymm3, 96(%r10)
	vmovaps		%ymm7, 96(%r10, %r11, 1)
	vmaskmovps	%ymm11, %ymm15, 96(%r10, %r11, 2)

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_20x4_vs_lib8)
#endif





// common inner routine with file scope
//
// store n vs
//
// input arguments:
// r10  <- D
// r11  <- 4*sdd*sizeof(double)
// r12  <- km
// r13  <- kn
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []
//
// output arguments:
// r10  <- D
// r11  <- 4*sdd*sizeof(double)
// r12  <- km
// r13  <- kn
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_24X4_VS_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_store_l_24x4_vs_lib8)
#endif
	
	// compute mask for rows
	vcvtsi2ss	%r12d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC02(%rip), %ymm13
#elif defined(OS_MAC)
	vmovups		LC02(%rip), %ymm13
#endif
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%ymm15, %ymm13, %ymm15

	vmovaps		%ymm0, 0(%r10)
	vmovaps		%ymm4, 0(%r10, %r11, 1)
	vmaskmovps	%ymm8, %ymm15, 0(%r10, %r11, 2)
	cmpl		$ 2, %r13d
	jl			0f // end
	vmovaps		32(%r10), %ymm12
	vblendps	$ 0x01, %ymm12, %ymm1, %ymm1
	vmovaps		%ymm1, 32(%r10)
	vmovaps		%ymm5, 32(%r10, %r11, 1)
	vmaskmovps	%ymm9, %ymm15, 32(%r10, %r11, 2)
	cmpl		$ 3, %r13d
	jl			0f // end
	vmovaps		64(%r10), %ymm12
	vblendps	$ 0x03, %ymm12, %ymm2, %ymm2
	vmovaps		%ymm2, 64(%r10)
	vmovaps		%ymm6, 64(%r10, %r11, 1)
	vmaskmovps	%ymm10, %ymm15, 64(%r10, %r11, 2)
	je			0f // end
	vmovaps		96(%r10), %ymm12
	vblendps	$ 0x07, %ymm12, %ymm3, %ymm3
	vmovaps		%ymm3, 96(%r10)
	vmovaps		%ymm7, 96(%r10, %r11, 1)
	vmaskmovps	%ymm11, %ymm15, 96(%r10, %r11, 2)

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_24x4_vs_lib8)
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
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []
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
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_20X4_GEN_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_store_l_20x4_gen_lib8)
#endif
	
	// compute mask for rows
	vcvtsi2ss	%r13d, %xmm14, %xmm14
	vcvtsi2ss	%r14d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %ymm12
	vmovups		.LC02(%rip), %ymm13
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %ymm12
	vmovups		LC02(%rip), %ymm13
#endif
	vshufps		$ 0x00, %xmm14, %xmm14, %xmm14
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm14, %ymm14, %ymm14
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%ymm12, %ymm14, %ymm14
	vsubps		%ymm15, %ymm13, %ymm15

	// shift D and sol for cols
	cmpl	$ 0, %r15d
	jle		0f

	vmovaps		%ymm1, %ymm0
	vmovaps		%ymm5, %ymm4
	vmovaps		%ymm9, %ymm8
	vmovaps		%ymm2, %ymm1
	vmovaps		%ymm6, %ymm5
	vmovaps		%ymm10, %ymm9
	vmovaps		%ymm3, %ymm2
	vmovaps		%ymm7, %ymm6
	vmovaps		%ymm11, %ymm10
	addq		$ 32, %r11

	cmpl	$ 1, %r15d
	jle		0f

	vmovaps		%ymm1, %ymm0
	vmovaps		%ymm5, %ymm4
	vmovaps		%ymm9, %ymm8
	vmovaps		%ymm2, %ymm1
	vmovaps		%ymm6, %ymm5
	vmovaps		%ymm10, %ymm9
	addq		$ 32, %r11

	cmpl	$ 2, %r15d
	jle		0f

	vmovaps		%ymm1, %ymm0
	vmovaps		%ymm5, %ymm4
	vmovaps		%ymm9, %ymm8
	addq		$ 32, %r11

0:

	// compute number of cols
	cmpl	$ 4, %eax
	jle		0f
	movl	$ 4, %eax
0:
	subl	%r15d, %eax
	movl	%eax, %r15d

	cmpl	$ 0, %r10d
	jg		0f

	// offset==0
	vmovaps		0(%r11), %ymm12
	vblendps	$ 0x0f, %ymm12, %ymm0, %ymm0
	vmaskmovps	%ymm0, %ymm14,  0(%r11)
	vmovaps		%ymm4, 0(%r11, %r12, 1)
	vmaskmovps	%ymm8, %ymm15,  0(%r11, %r12, 2)
	cmpl		$ 2, %r15d
	jl			7f // end
	vmovaps		32(%r11), %ymm12
	vblendps	$ 0x1f, %ymm12, %ymm1, %ymm1
	vmaskmovps	%ymm1, %ymm14, 32(%r11)
	vmovaps		%ymm5, 32(%r11, %r12, 1)
	vmaskmovps	%ymm9, %ymm15, 32(%r11, %r12, 2)
	cmpl		$ 3, %r15d
	jl			7f // end
	vmovaps		64(%r11), %ymm12
	vblendps	$ 0x3f, %ymm12, %ymm2, %ymm2
	vmaskmovps	%ymm2, %ymm14, 64(%r11)
	vmovaps		%ymm6, 64(%r11, %r12, 1)
	vmaskmovps	%ymm10, %ymm15, 64(%r11, %r12, 2)
	je			7f // end
	vmovaps		96(%r11), %ymm12
	vblendps	$ 0x7f, %ymm12, %ymm3, %ymm3
	vmaskmovps	%ymm3, %ymm14, 96(%r11)
	vmovaps		%ymm7, 96(%r11, %r12, 1)
	vmaskmovps	%ymm11, %ymm15, 96(%r11, %r12, 2)
	//
	jmp		7f

0:
	// offset > 0
	// 1 2 3 4 5 6 7
	
	cmpl	$ 4, %r10d
	jl		1f
	jg		2f

	// offset==4
	// TODO
	jmp		7f

1:
	// 1 2 3

	cmpl	$ 2, %r10d
	jl		3f
	jg		4f

	// offset==2
	// TODO
	jmp		7f

3:
	// offset==1
	// TODO
	jmp		7f

4:
	// offset==3
	// TODO
	jmp		7f

2:
	// 5 6 7

	cmpl	$ 6, %r10d
	jl		5f
	jg		6f

	// offset==6
	// TODO
	jmp		7f

5:
	// offset==5
	// TODO
	jmp		7f

6:
	// offset==7
	// TODO
	jmp		7f

	// end
7:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_20x4_gen_lib8)
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
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []
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
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_24X4_GEN_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_store_l_24x4_gen_lib8)
#endif
	
	// compute mask for rows
	vcvtsi2ss	%r13d, %xmm14, %xmm14
	vcvtsi2ss	%r14d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %ymm12
	vmovups		.LC02(%rip), %ymm13
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %ymm12
	vmovups		LC02(%rip), %ymm13
#endif
	vshufps		$ 0x00, %xmm14, %xmm14, %xmm14
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm14, %ymm14, %ymm14
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%ymm12, %ymm14, %ymm14
	vsubps		%ymm15, %ymm13, %ymm15

	// shift D and sol for cols
	cmpl	$ 0, %r15d
	jle		0f

	vmovaps		%ymm1, %ymm0
	vmovaps		%ymm5, %ymm4
	vmovaps		%ymm9, %ymm8
	vmovaps		%ymm2, %ymm1
	vmovaps		%ymm6, %ymm5
	vmovaps		%ymm10, %ymm9
	vmovaps		%ymm3, %ymm2
	vmovaps		%ymm7, %ymm6
	vmovaps		%ymm11, %ymm10
	addq		$ 32, %r11

	cmpl	$ 1, %r15d
	jle		0f

	vmovaps		%ymm1, %ymm0
	vmovaps		%ymm5, %ymm4
	vmovaps		%ymm9, %ymm8
	vmovaps		%ymm2, %ymm1
	vmovaps		%ymm6, %ymm5
	vmovaps		%ymm10, %ymm9
	addq		$ 32, %r11

	cmpl	$ 2, %r15d
	jle		0f

	vmovaps		%ymm1, %ymm0
	vmovaps		%ymm5, %ymm4
	vmovaps		%ymm9, %ymm8
	addq		$ 32, %r11

0:

	// compute number of cols
	cmpl	$ 4, %eax
	jle		0f
	movl	$ 4, %eax
0:
	subl	%r15d, %eax
	movl	%eax, %r15d

	cmpl	$ 0, %r10d
	jg		0f

	// offset==0
	vmaskmovps	%ymm0, %ymm14,  0(%r11)
	vmovaps		%ymm4, 0(%r11, %r12, 1)
	vmaskmovps	%ymm8, %ymm15,  0(%r11, %r12, 2)
	cmpl		$ 2, %r15d
	jl			7f // end
	vmovaps		32(%r11), %ymm12
	vblendps	$ 0x01, %ymm12, %ymm1, %ymm1
	vmaskmovps	%ymm1, %ymm14, 32(%r11)
	vmovaps		%ymm5, 32(%r11, %r12, 1)
	vmaskmovps	%ymm9, %ymm15, 32(%r11, %r12, 2)
	cmpl		$ 3, %r15d
	jl			7f // end
	vmovaps		64(%r11), %ymm12
	vblendps	$ 0x03, %ymm12, %ymm2, %ymm2
	vmaskmovps	%ymm2, %ymm14, 64(%r11)
	vmovaps		%ymm6, 64(%r11, %r12, 1)
	vmaskmovps	%ymm10, %ymm15, 64(%r11, %r12, 2)
	je			7f // end
	vmovaps		96(%r11), %ymm12
	vblendps	$ 0x07, %ymm12, %ymm3, %ymm3
	vmaskmovps	%ymm3, %ymm14, 96(%r11)
	vmovaps		%ymm7, 96(%r11, %r12, 1)
	vmaskmovps	%ymm11, %ymm15, 96(%r11, %r12, 2)
	//
	jmp		7f

0:
	// offset > 0
	// 1 2 3 4 5 6 7
	
	cmpl	$ 4, %r10d
	jl		1f
	jg		2f

	// offset==4
	// TODO
	jmp		7f

1:
	// 1 2 3

	cmpl	$ 2, %r10d
	jl		3f
	jg		4f

	// offset==2
	// TODO
	jmp		7f

3:
	// offset==1
	// TODO
	jmp		7f

4:
	// offset==3
	// TODO
	jmp		7f

2:
	// 5 6 7

	cmpl	$ 6, %r10d
	jl		5f
	jg		6f

	// offset==6
	// TODO
	jmp		7f

5:
	// offset==5
	// TODO
	jmp		7f

6:
	// offset==7
	// TODO
	jmp		7f

	// end
7:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_24x4_gen_lib8)
#endif





//                                1      2             3         4        5         6            7         8        9         10
// void kernel_sgemm_nt_24x4_lib8(int k, float *alpha, float *A, int sda, float *B, float *beta, float *C, int sdc, float *D, int sdd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_nt_24x4_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG5, %r13  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_24X4_LIB8
#else
	CALL(inner_kernel_gemm_add_nt_24x4_lib8)
#endif


	// call inner scale

	movq	%rsi, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movl	ARG8, %r13d // sdc
	sall	$ 5, %r13d // 8*sdc*sizeof(float)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_24X4_LIB8
#else
	CALL(inner_scale_ab_24x4_lib8)
#endif


	// store n

	movq	ARG9, %r10 // D
	movl	ARG10, %r11d // sdd
	sall	$ 5, %r11d // 8*sdd*sizeof(float)

#if MACRO_LEVEL>=1
	INNER_STORE_24X4_LIB8
#else
	CALL(inner_store_24x4_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_nt_24x4_lib8)





//                                   1      2             3         4        5         6            7         8        9         10       11      12
// void kernel_sgemm_nt_24x4_vs_lib8(int k, float *alpha, float *A, int sda, float *B, float *beta, float *C, int sdc, float *D, int sdd, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_nt_24x4_vs_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG5, %r13  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_24X4_LIB8
#else
	CALL(inner_kernel_gemm_add_nt_24x4_lib8)
#endif


	// call inner scale

	movq	%rsi, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movl	ARG8, %r13d // sdc
	sall	$ 5, %r13d // 8*sdc*sizeof(float)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_24X4_LIB8
#else
	CALL(inner_scale_ab_24x4_lib8)
#endif


	// store n

	movq	ARG9, %r10 // D
	movl	ARG10, %r11d // sdd
	sall	$ 5, %r11d // 8*sdd*sizeof(float)
	movq	ARG11, %r12 // km
	movq	ARG12, %r13 // kn

#if MACRO_LEVEL>=1
	INNER_STORE_24X4_VS_LIB8
#else
	CALL(inner_store_24x4_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_nt_24x4_vs_lib8)





//                                    rdi    rsi           rdx       rcx      r8        r9           rsp+8        rsp+16    rsp+24   rsp+32       rsp+40    rsp+48   rsp+56  rsp+64  rsp+72  rsp+80
// void kernel_sgemm_nt_24x4_gen_lib8(int k, float *alpha, float *A, int sda, float *B, float *beta, int offsetC, float *C, int sdc, int offsetD, float *D, int sdd, int m0, int m1, int n0, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_nt_24x4_gen_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG5, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_24X4_LIB8
#else
	CALL(inner_kernel_gemm_add_nt_24x4_lib8)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12 // offsetC
	movq	ARG8, %r13 // C
	movq	ARG9, %r14 // sdc
	sall	$ 5, %r14d // 8*sdc*sizeof(float)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_24X4_GEN_LIB8
#else
	CALL(inner_scale_ab_24x4_gen_lib8)
#endif


	// store n gen

	movq	ARG10, %r10 // offsetD
	movq	ARG11, %r11 // D
	movq	ARG12, %r12 // sdd
	sall	$ 5, %r12d // 8*sdb*sizeof(float)
	movq	ARG13, %r13 // m0
	movq	ARG14, %r14 // m1
	movq	ARG15, %r15 // n0
	movq	ARG16, %rax // n1

#if MACRO_LEVEL>=1
	INNER_STORE_24X4_GEN_LIB8
#else
	CALL(inner_store_24x4_gen_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_nt_24x4_gen_lib8)





//                                   1      2             3         4        5            6         7        8            9         10       11        12
// void kernel_sgemm_nn_24x4_lib8(int k, float *alpha, float *A, int sda, int offsetB, float *B, int sdb, float *beta, float *C, int sdc, float *D, int sdd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_nn_24x4_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sdb
	sall	$ 5, %r12d // 4*sdb*sizeof(double)
	movq	ARG6, %r13  // B
	movq	ARG7, %r14 // sdb
	sall	$ 5, %r14d // 4*sdb*sizeof(double)
	movq	ARG5, %r15 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_GEMM_ADD_NN_24X4_LIB8
#else
	CALL(inner_edge_gemm_add_nn_24x4_lib8)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_24X4_LIB8
#else
	CALL(inner_kernel_gemm_add_nn_24x4_lib8)
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12   // C
	movq	ARG10, %r13   // sdc
	sall	$ 5, %r13d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_24X4_LIB8
#else
	CALL(inner_scale_ab_24x4_lib8)
#endif


	// store n

	movq	ARG11, %r10 // D
	movq	ARG12, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_24X4_LIB8
#else
	CALL(inner_store_24x4_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_nn_24x4_lib8)





//                                   1      2             3         4        5            6         7        8            9         10       11        12       13      14
// void kernel_sgemm_nn_24x4_vs_lib8(int k, float *alpha, float *A, int sda, int offsetB, float *B, int sdb, float *beta, float *C, int sdc, float *D, int sdd, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_nn_24x4_vs_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sdb
	sall	$ 5, %r12d // 4*sdb*sizeof(double)
	movq	ARG6, %r13  // B
	movq	ARG7, %r14 // sdb
	sall	$ 5, %r14d // 4*sdb*sizeof(double)
	movq	ARG5, %r15 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_GEMM_ADD_NN_24X4_LIB8
#else
	CALL(inner_edge_gemm_add_nn_24x4_lib8)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_24X4_LIB8
#else
	CALL(inner_kernel_gemm_add_nn_24x4_lib8)
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12   // C
	movq	ARG10, %r13   // sdc
	sall	$ 5, %r13d // 4*sdb*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_24X4_LIB8
#else
	CALL(inner_scale_ab_24x4_lib8)
#endif


	// store n

	movq	ARG11, %r10 // D
	movq	ARG12, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG13, %r12 // km
	movq	ARG14, %r13 // kn

#if MACRO_LEVEL>=1
	INNER_STORE_24X4_VS_LIB8
#else
	CALL(inner_store_24x4_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_nn_24x4_vs_lib8)





//                                    rdi    rsi           rdx       rcx      r8        r9        rsp+8    rsp+16       rsp+24    rsp+32    rsp+40   rsp+48    rsp+56    rsp+64   rsp+72  rsp+80  rsp+88  rsp+96
// void kernel_sgemm_nn_24x4_gen_lib4(int k, float *alpha, float *A, int sda, int offB, float *B, int sdb, float *beta, int offC, float *C, int sdc, int offD, float *D, int sdd, int m0, int m1, int n0, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_nn_24x4_gen_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG6, %r13  // B
	movq	ARG7, %r14 // sdb
	sall	$ 5, %r14d // 4*sdb*sizeof(double)
	movq	ARG5, %r15 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_GEMM_ADD_NN_24X4_LIB8
#else
	CALL(inner_edge_gemm_add_nn_24x4_lib8)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_24X4_LIB8
#else
	CALL(inner_kernel_gemm_add_nn_24x4_lib8)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12 // offsetC
	movq	ARG10, %r13 // C
	movq	ARG11, %r14 // sdc
	sall	$ 5, %r14d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_24X4_GEN_LIB8
#else
	CALL(inner_scale_ab_24x4_gen_lib8)
#endif


	// store n gen

	movq	ARG12, %r10 // offsetD
	movq	ARG13, %r11 // D
	movq	ARG14, %r12 // sdd
	sall	$ 5, %r12d // 4*sdb*sizeof(double)
	movq	ARG15, %r13 // m0
	movq	ARG16, %r14 // m1
	movq	ARG17, %r15 // n0
	movq	ARG18, %rax // n1

#if MACRO_LEVEL>=1
	INNER_STORE_24X4_GEN_LIB8
#else
	CALL(inner_store_24x4_gen_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_nn_24x4_gen_lib8)





//                                       rdi    rsi       rdx      rcx       r8        r9       rsp+8     rsp+16   rsp+24    rsp+32 
// void kernel_strsm_nt_rl_inv_24x4_lib8(int k, float *A, int sda, float *B, float *C, int sdc, float *D, int sdd, float *E, float *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_strsm_nt_rl_inv_24x4_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG4, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_SUB_NT_24X4_LIB8
#else
	CALL(inner_kernel_gemm_sub_nt_24x4_lib8)
#endif


	// call inner blender nn

	movq	ARG5, %r10 // C
	movq	ARG6, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_11_24X4_LIB8
#else
	CALL(inner_scale_11_24x4_lib8)
#endif


	// solve

	movq	ARG9, %r10  // E 
	movq	ARG10, %r11  // inv_diag_E 
	movl	$ 4, %r12d // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_INV_24X4_VS_LIB8
#else
	CALL(inner_edge_trsm_rlt_inv_24x4_vs_lib8)
#endif


	// store n

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_24X4_LIB8
#else
	CALL(inner_store_24x4_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_strsm_nt_rl_inv_24x4_lib8)





//                                          rdi    rsi       rdx      rcx       r8        r9       rsp+8     rsp+16   rsp+24    rsp+32             rsp+40  rsp+48
// void kernel_strsm_nt_rl_inv_24x4_vs_lib8(int k, float *A, int sda, float *B, float *C, int sdc, float *D, int sdd, float *E, float *inv_diag_E, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_strsm_nt_rl_inv_24x4_vs_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_SUB_NT_24X4_LIB8
#else
	CALL(inner_kernel_gemm_sub_nt_24x4_lib8)
#endif


	// call inner blender nn

	movq	ARG5, %r10 // C
	movq	ARG6, %r11 // sdc
	sall	$ 5, %r11d // 8*sdc*sizeof(float)

#if MACRO_LEVEL>=1
	INNER_SCALE_11_24X4_LIB8
#else
	CALL(inner_scale_11_24x4_lib8)
#endif


	// solve

	movq	ARG9, %r10  // E 
	movq	ARG10, %r11  // inv_diag_E 
	movq	ARG12, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_INV_24X4_VS_LIB8
#else
	CALL(inner_edge_trsm_rlt_inv_24x4_vs_lib8)
#endif


	// store n

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG11, %r12 // m1 
	movq	ARG12, %r13 // n1 

#if MACRO_LEVEL>=1
	INNER_STORE_24X4_VS_LIB8
#else
	CALL(inner_store_24x4_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_strsm_nt_rl_inv_24x4_vs_lib8)





//                                             1       2          3         4          5       6          7         8          9         10       11        12       13        14
// void kernel_sgemm_strsm_nt_rl_inv_24x4_lib8(int kp, float *Ap, int sdap, float *Bp, int km, float *Am, int sdam, float *Bm, float *C, int sdc, float *D, int sdd, float *E, float *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_strsm_nt_rl_inv_24x4_lib8)
	
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
	vmovaps	%ymm0, %ymm8
	vmovaps	%ymm0, %ymm9
	vmovaps	%ymm0, %ymm10
	vmovaps	%ymm0, %ymm11


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG2, %r11  // Ap
	movq	ARG3, %r12 // sdap
	sall	$ 5, %r12d   // 4*sdap*sizeof(double)
	movq	ARG4, %r13  // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_24X4_LIB8
#else
	CALL(inner_kernel_gemm_add_nt_24x4_lib8)
#endif


	// call inner dgemm kernel nt sub

	movq	ARG5, %r10                 // km
	movq	ARG6, %r11                   // Am
	movq	ARG7, %r12 // sdam
	sall	$ 5, %r12d                   // 4*sda*sizeof(double)
	movq	ARG8, %r13  // Bm

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_SUB_NT_24X4_LIB8
#else
	CALL(inner_kernel_gemm_sub_nt_24x4_lib8)
#endif


	// call inner blender nn

	movq	ARG9, %r10  // C
	movq	ARG10, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_11_24X4_LIB8
#else
	CALL(inner_scale_11_24x4_lib8)
#endif


	// solve

	movq	ARG13, %r10  // E 
	movq	ARG14, %r11  // inv_diag_E 
	movl	$ 4, %r12d // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_INV_24X4_VS_LIB8
#else
	CALL(inner_edge_trsm_rlt_inv_24x4_vs_lib8)
#endif


	// store n

	movq	ARG11, %r10 // store address D
	movq	ARG12, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_24X4_LIB8
#else
	CALL(inner_store_24x4_lib8)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_sgemm_strsm_nt_rl_inv_24x4_lib8)





//                                                1       2          3         4          5       6          7         8          9         10       11        12       13        14                 15      16
// void kernel_sgemm_strsm_nt_rl_inv_24x4_vs_lib8(int kp, float *Ap, int sdap, float *Bp, int km, float *Am, int sdam, float *Bm, float *C, int sdc, float *D, int sdd, float *E, float *inv_diag_E, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_strsm_nt_rl_inv_24x4_vs_lib8)
	
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
	vmovaps	%ymm0, %ymm8
	vmovaps	%ymm0, %ymm9
	vmovaps	%ymm0, %ymm10
	vmovaps	%ymm0, %ymm11


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG2, %r11  // Ap
	movq	ARG3, %r12 // sdap
	sall	$ 5, %r12d   // 4*sdap*sizeof(double)
	movq	ARG4, %r13  // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_24X4_LIB8
#else
	CALL(inner_kernel_gemm_add_nt_24x4_lib8)
#endif


	// call inner dgemm kernel nt sub

	movq	ARG5, %r10                 // km
	movq	ARG6, %r11                   // Am
	movq	ARG7, %r12 // sdam
	sall	$ 5, %r12d                   // 4*sda*sizeof(double)
	movq	ARG8, %r13  // Bm

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_SUB_NT_24X4_LIB8
#else
	CALL(inner_kernel_gemm_sub_nt_24x4_lib8)
#endif


	// call inner blender nn

	movq	ARG9, %r10  // C
	movq	ARG10, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_11_24X4_LIB8
#else
	CALL(inner_scale_11_24x4_lib8)
#endif


	// solve

	movq	ARG13, %r10  // E 
	movq	ARG14, %r11  // inv_diag_E 
	movq	ARG16, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_INV_24X4_VS_LIB8
#else
	CALL(inner_edge_trsm_rlt_inv_24x4_vs_lib8)
#endif


	// store n

	movq	ARG11, %r10 // store address D
	movq	ARG12, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG15, %r12 // km 
	movq	ARG16, %r13 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_24X4_VS_LIB8
#else
	CALL(inner_store_24x4_vs_lib8)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_sgemm_strsm_nt_rl_inv_24x4_vs_lib8)





//                                   1      2         3        4         5         6        7         8        9
// void kernel_spotrf_nt_l_20x4_lib8(int k, float *A, int sda, float *B, float *C, int sdc, float *D, int sdd, float *inv_diag_D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_spotrf_nt_l_20x4_lib8)
	
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
	vmovaps	%ymm0, %ymm8
	vmovaps	%ymm0, %ymm9
	vmovaps	%ymm0, %ymm10
	vmovaps	%ymm0, %ymm11


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG4, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_SUB_NT_24X4_LIB8
#else
	CALL(inner_kernel_gemm_sub_nt_24x4_lib8)
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // C
	movq	ARG6, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_11_24X4_LIB8
#else
	CALL(inner_scale_11_24x4_lib8)
#endif


	// factorization

	movq	ARG9, %r10  // inv_diag_D 
	movl	$ 4, %r11d // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_POTRF_20X4_VS_LIB8
#else
	CALL(inner_edge_potrf_20x4_vs_lib8)
#endif


	// store

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_L_20X4_LIB8
#else
	CALL(inner_store_l_20x4_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_spotrf_nt_l_20x4_lib8)





//                                      1      2         3        4         5         6        7         8        9                  10      11
// void kernel_spotrf_nt_l_20x4_vs_lib8(int k, float *A, int sda, float *B, float *C, int sdc, float *D, int sdd, float *inv_diag_D, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_spotrf_nt_l_20x4_vs_lib8)
	
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
	vmovaps	%ymm0, %ymm8
	vmovaps	%ymm0, %ymm9
	vmovaps	%ymm0, %ymm10
	vmovaps	%ymm0, %ymm11


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG4, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_SUB_NT_24X4_LIB8
#else
	CALL(inner_kernel_gemm_sub_nt_24x4_lib8)
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // C
	movq	ARG6, %r11 // sdc
	sall	$ 5, %r11d // 8*sdc*sizeof(float)

#if MACRO_LEVEL>=1
	INNER_SCALE_11_24X4_LIB8
#else
	CALL(inner_scale_11_24x4_lib8)
#endif


	// factorization

	movq	ARG9, %r10  // inv_diag_D 
	movq	ARG11, %r11 // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_POTRF_20X4_VS_LIB8
#else
	CALL(inner_edge_potrf_20x4_vs_lib8)
#endif


	// store

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG10, %r12 // m1 
	movq	ARG11, %r13 // n1 

#if MACRO_LEVEL>=1
	INNER_STORE_L_20X4_VS_LIB8
#else
	CALL(inner_store_l_20x4_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_spotrf_nt_l_20x4_lib8)





//                                   1      2         3        4         5         6        7         8        9
// void kernel_spotrf_nt_l_24x4_lib8(int k, float *A, int sda, float *B, float *C, int sdc, float *D, int sdd, float *inv_diag_D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_spotrf_nt_l_24x4_lib8)
	
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
	vmovaps	%ymm0, %ymm8
	vmovaps	%ymm0, %ymm9
	vmovaps	%ymm0, %ymm10
	vmovaps	%ymm0, %ymm11


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG4, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_SUB_NT_24X4_LIB8
#else
	CALL(inner_kernel_gemm_sub_nt_24x4_lib8)
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // C
	movq	ARG6, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_11_24X4_LIB8
#else
	CALL(inner_scale_11_24x4_lib8)
#endif


	// factorization

	movq	ARG9, %r10  // inv_diag_D 
	movl	$ 4, %r11d // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_POTRF_24X4_VS_LIB8
#else
	CALL(inner_edge_potrf_24x4_vs_lib8)
#endif


	// store

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_L_24X4_LIB8
#else
	CALL(inner_store_l_24x4_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_spotrf_nt_l_24x4_lib8)





//                                      1      2         3        4         5         6        7         8        9                  10      11
// void kernel_spotrf_nt_l_24x4_vs_lib8(int k, float *A, int sda, float *B, float *C, int sdc, float *D, int sdd, float *inv_diag_D, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_spotrf_nt_l_24x4_vs_lib8)
	
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
	vmovaps	%ymm0, %ymm8
	vmovaps	%ymm0, %ymm9
	vmovaps	%ymm0, %ymm10
	vmovaps	%ymm0, %ymm11


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG4, %r13 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_SUB_NT_24X4_LIB8
#else
	CALL(inner_kernel_gemm_sub_nt_24x4_lib8)
#endif


	// call inner blender_loader nn

	movq	ARG5, %r10 // C
	movq	ARG6, %r11 // sdc
	sall	$ 5, %r11d // 8*sdc*sizeof(float)

#if MACRO_LEVEL>=1
	INNER_SCALE_11_24X4_LIB8
#else
	CALL(inner_scale_11_24x4_lib8)
#endif


	// factorization

	movq	ARG9, %r10  // inv_diag_D 
	movq	ARG11, %r11 // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_POTRF_24X4_VS_LIB8
#else
	CALL(inner_edge_potrf_24x4_vs_lib8)
#endif


	// store

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG10, %r12 // m1 
	movq	ARG11, %r13 // n1 

#if MACRO_LEVEL>=1
	INNER_STORE_L_24X4_VS_LIB8
#else
	CALL(inner_store_l_24x4_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_spotrf_nt_l_24x4_lib8)





//                                        1        2          3         4          5       6          7         8          9         10       11        12       13
// void kernel_ssyrk_spotrf_nt_l_20x4_lib8(int kp, float *Ap, int sdap, float *Bp, int km, float *Am, int sdam, float *Bm, float *C, int sdc, float *D, int sdd, float *inv_diag_D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_ssyrk_spotrf_nt_l_20x4_lib8)
	
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
	vmovaps	%ymm0, %ymm8
	vmovaps	%ymm0, %ymm9
	vmovaps	%ymm0, %ymm10
	vmovaps	%ymm0, %ymm11


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG2, %r11  // Ap
	movq	ARG3, %r12 // sdap
	sall	$ 5, %r12d   // 4*sdap*sizeof(double)
	movq	ARG4, %r13  // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_24X4_LIB8
#else
	CALL(inner_kernel_gemm_add_nt_24x4_lib8)
#endif


	// call inner dgemm kernel nt sub

	movq	ARG5, %r10                 // km
	movq	ARG6, %r11                   // Am
	movq	ARG7, %r12 // sdam
	sall	$ 5, %r12d                   // 4*sdam*sizeof(double)
	movq	ARG8, %r13  // Bm

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_SUB_NT_24X4_LIB8
#else
	CALL(inner_kernel_gemm_sub_nt_24x4_lib8)
#endif


	// call inner blender nn

	movq	ARG9, %r10 // C
	movq	ARG10, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_11_24X4_LIB8
#else
	CALL(inner_scale_11_24x4_lib8)
#endif


	// factorization

	movq	ARG13, %r10  // inv_diag_D 
	movl	$ 4, %r11d

#if MACRO_LEVEL>=1
	INNER_EDGE_POTRF_20X4_VS_LIB8
#else
	CALL(inner_edge_potrf_20x4_vs_lib8)
#endif


	// store n

	movq	ARG11, %r10 // store address D
	movq	ARG12, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_L_20X4_LIB8
#else
	CALL(inner_store_l_20x4_lib8)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_ssyrk_spotrf_nt_l_20x4_lib8)





//                                            1        2          3         4          5       6          7         8          9         10       11        12       13                14      15
// void kernel_ssyrk_spotrf_nt_l_20x4_vs_lib8(int kp, float *Ap, int sdap, float *Bp, int km, float *Am, int sdam, float *Bm, float *C, int sdc, float *D, int sdd, float *inv_diag_D, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_ssyrk_spotrf_nt_l_20x4_vs_lib8)
	
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
	vmovaps	%ymm0, %ymm8
	vmovaps	%ymm0, %ymm9
	vmovaps	%ymm0, %ymm10
	vmovaps	%ymm0, %ymm11


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG2, %r11  // Ap
	movq	ARG3, %r12 // sdap
	sall	$ 5, %r12d   // 4*sdap*sizeof(double)
	movq	ARG4, %r13  // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_24X4_LIB8
#else
	CALL(inner_kernel_gemm_add_nt_24x4_lib8)
#endif


	// call inner dgemm kernel nt sub

	movq	ARG5, %r10                 // km
	movq	ARG6, %r11                   // Am
	movq	ARG7, %r12 // sdam
	sall	$ 5, %r12d                   // 4*sdam*sizeof(double)
	movq	ARG8, %r13  // Bm

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_SUB_NT_24X4_LIB8
#else
	CALL(inner_kernel_gemm_sub_nt_24x4_lib8)
#endif


	// call inner blender nn

	movq	ARG9, %r10 // C
	movq	ARG10, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_11_24X4_LIB8
#else
	CALL(inner_scale_11_24x4_lib8)
#endif


	// factorization

	movq	ARG13, %r10  // inv_diag_D 
	movq	ARG15, %r11 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_POTRF_20X4_VS_LIB8
#else
	CALL(inner_edge_potrf_20x4_vs_lib8)
#endif


	// store n

	movq	ARG11, %r10 // store address D
	movq	ARG12, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

	movq	ARG14, %r12 // km 
	movq	ARG15, %r13 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_L_20X4_VS_LIB8
#else
	CALL(inner_store_l_20x4_vs_lib8)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_ssyrk_spotrf_nt_l_20x4_vs_lib8)





//                                        1        2          3         4          5       6          7         8          9         10       11        12       13
// void kernel_ssyrk_spotrf_nt_l_24x4_lib8(int kp, float *Ap, int sdap, float *Bp, int km, float *Am, int sdam, float *Bm, float *C, int sdc, float *D, int sdd, float *inv_diag_D);

	.p2align 4,,15
	GLOB_FUN_START(kernel_ssyrk_spotrf_nt_l_24x4_lib8)
	
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
	vmovaps	%ymm0, %ymm8
	vmovaps	%ymm0, %ymm9
	vmovaps	%ymm0, %ymm10
	vmovaps	%ymm0, %ymm11


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG2, %r11  // Ap
	movq	ARG3, %r12 // sdap
	sall	$ 5, %r12d   // 4*sdap*sizeof(double)
	movq	ARG4, %r13  // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_24X4_LIB8
#else
	CALL(inner_kernel_gemm_add_nt_24x4_lib8)
#endif


	// call inner dgemm kernel nt sub

	movq	ARG5, %r10                 // km
	movq	ARG6, %r11                   // Am
	movq	ARG7, %r12 // sdam
	sall	$ 5, %r12d                   // 4*sdam*sizeof(double)
	movq	ARG8, %r13  // Bm

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_SUB_NT_24X4_LIB8
#else
	CALL(inner_kernel_gemm_sub_nt_24x4_lib8)
#endif


	// call inner blender nn

	movq	ARG9, %r10 // C
	movq	ARG10, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_11_24X4_LIB8
#else
	CALL(inner_scale_11_24x4_lib8)
#endif


	// factorization

	movq	ARG13, %r10  // inv_diag_D 
	movl	$ 4, %r11d

#if MACRO_LEVEL>=1
	INNER_EDGE_POTRF_24X4_VS_LIB8
#else
	CALL(inner_edge_potrf_24x4_vs_lib8)
#endif


	// store n

	movq	ARG11, %r10 // store address D
	movq	ARG12, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_L_24X4_LIB8
#else
	CALL(inner_store_l_24x4_lib8)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_ssyrk_spotrf_nt_l_24x4_lib8)





//                                            1        2          3         4          5       6          7         8          9         10       11        12       13                14      15
// void kernel_ssyrk_spotrf_nt_l_24x4_vs_lib8(int kp, float *Ap, int sdap, float *Bp, int km, float *Am, int sdam, float *Bm, float *C, int sdc, float *D, int sdd, float *inv_diag_D, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_ssyrk_spotrf_nt_l_24x4_vs_lib8)
	
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
	vmovaps	%ymm0, %ymm8
	vmovaps	%ymm0, %ymm9
	vmovaps	%ymm0, %ymm10
	vmovaps	%ymm0, %ymm11


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG2, %r11  // Ap
	movq	ARG3, %r12 // sdap
	sall	$ 5, %r12d   // 4*sdap*sizeof(double)
	movq	ARG4, %r13  // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_24X4_LIB8
#else
	CALL(inner_kernel_gemm_add_nt_24x4_lib8)
#endif


	// call inner dgemm kernel nt sub

	movq	ARG5, %r10                 // km
	movq	ARG6, %r11                   // Am
	movq	ARG7, %r12 // sdam
	sall	$ 5, %r12d                   // 4*sdam*sizeof(double)
	movq	ARG8, %r13  // Bm

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_SUB_NT_24X4_LIB8
#else
	CALL(inner_kernel_gemm_sub_nt_24x4_lib8)
#endif


	// call inner blender nn

	movq	ARG9, %r10 // C
	movq	ARG10, %r11 // sdc
	sall	$ 5, %r11d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_11_24X4_LIB8
#else
	CALL(inner_scale_11_24x4_lib8)
#endif


	// factorization

	movq	ARG13, %r10  // inv_diag_D 
	movq	ARG15, %r11 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_POTRF_24X4_VS_LIB8
#else
	CALL(inner_edge_potrf_24x4_vs_lib8)
#endif


	// store n

	movq	ARG11, %r10 // store address D
	movq	ARG12, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

	movq	ARG14, %r12 // km 
	movq	ARG15, %r13 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_L_24X4_VS_LIB8
#else
	CALL(inner_store_l_24x4_vs_lib8)
#endif


	EPILOGUE
	
	ret

	FUN_END(kernel_ssyrk_spotrf_nt_l_24x4_vs_lib8)





//                                1      2             3         4        5         6            7         8        9         10
// void kernel_ssyrk_nt_l_24x4_lib8(int k, float *alpha, float *A, int sda, float *B, float *beta, float *C, int sdc, float *D, int sdd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_ssyrk_nt_l_24x4_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG5, %r13  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_24X4_LIB8
#else
	CALL(inner_kernel_gemm_add_nt_24x4_lib8)
#endif


	// call inner scale

	movq	%rsi, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movl	ARG8, %r13d // sdc
	sall	$ 5, %r13d // 8*sdc*sizeof(float)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_24X4_LIB8
#else
	CALL(inner_scale_ab_24x4_lib8)
#endif


	// store n

	movq	ARG9, %r10 // D
	movl	ARG10, %r11d // sdd
	sall	$ 5, %r11d // 8*sdd*sizeof(float)
	movq	ARG11, %r12 // km
	movq	ARG12, %r13 // kn

#if MACRO_LEVEL>=1
	INNER_STORE_L_24X4_LIB8
#else
	CALL(inner_store_l_24x4_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_ssyrk_nt_l_24x4_lib8)





//                                   1      2             3         4        5         6            7         8        9         10       11      12
// void kernel_ssyrk_nt_l_24x4_vs_lib8(int k, float *alpha, float *A, int sda, float *B, float *beta, float *C, int sdc, float *D, int sdd, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_ssyrk_nt_l_24x4_vs_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG5, %r13  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_24X4_LIB8
#else
	CALL(inner_kernel_gemm_add_nt_24x4_lib8)
#endif


	// call inner scale

	movq	%rsi, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movl	ARG8, %r13d // sdc
	sall	$ 5, %r13d // 8*sdc*sizeof(float)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_24X4_LIB8
#else
	CALL(inner_scale_ab_24x4_lib8)
#endif


	// store n

	movq	ARG9, %r10 // D
	movl	ARG10, %r11d // sdd
	sall	$ 5, %r11d // 8*sdd*sizeof(float)
	movq	ARG11, %r12 // km
	movq	ARG12, %r13 // kn

#if MACRO_LEVEL>=1
	INNER_STORE_L_24X4_VS_LIB8
#else
	CALL(inner_store_l_24x4_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_ssyrk_nt_l_24x4_vs_lib8)





//                                1      2             3         4        5         6            7         8        9         10
// void kernel_ssyrk_nt_l_20x4_lib8(int k, float *alpha, float *A, int sda, float *B, float *beta, float *C, int sdc, float *D, int sdd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_ssyrk_nt_l_20x4_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG5, %r13  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_24X4_LIB8
#else
	CALL(inner_kernel_gemm_add_nt_24x4_lib8)
#endif


	// call inner scale

	movq	%rsi, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movl	ARG8, %r13d // sdc
	sall	$ 5, %r13d // 8*sdc*sizeof(float)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_24X4_LIB8
#else
	CALL(inner_scale_ab_24x4_lib8)
#endif


	// store n

	movq	ARG9, %r10 // D
	movl	ARG10, %r11d // sdd
	sall	$ 5, %r11d // 8*sdd*sizeof(float)
	movq	ARG11, %r12 // km
	movq	ARG12, %r13 // kn

#if MACRO_LEVEL>=1
	INNER_STORE_L_20X4_LIB8
#else
	CALL(inner_store_l_20x4_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_ssyrk_nt_l_20x4_lib8)





//                                   1      2             3         4        5         6            7         8        9         10       11      12
// void kernel_ssyrk_nt_l_20x4_vs_lib8(int k, float *alpha, float *A, int sda, float *B, float *beta, float *C, int sdc, float *D, int sdd, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_ssyrk_nt_l_20x4_vs_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG5, %r13  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_24X4_LIB8
#else
	CALL(inner_kernel_gemm_add_nt_24x4_lib8)
#endif


	// call inner scale

	movq	%rsi, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movl	ARG8, %r13d // sdc
	sall	$ 5, %r13d // 8*sdc*sizeof(float)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_24X4_LIB8
#else
	CALL(inner_scale_ab_24x4_lib8)
#endif


	// store n

	movq	ARG9, %r10 // D
	movl	ARG10, %r11d // sdd
	sall	$ 5, %r11d // 8*sdd*sizeof(float)
	movq	ARG11, %r12 // km
	movq	ARG12, %r13 // kn

#if MACRO_LEVEL>=1
	INNER_STORE_L_20X4_VS_LIB8
#else
	CALL(inner_store_l_20x4_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_ssyrk_nt_l_20x4_vs_lib8)





//                                   1      2             3         4        5            6         7        8         9
// void kernel_strmm_nn_rl_24x4_lib8(int k, float *alpha, float *A, int sda, int offsetB, float *B, int sdb, float *D, int sdd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_strmm_nn_rl_24x4_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// initial triangle

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sdb
	sall	$ 5, %r12d // 4*sdb*sizeof(double)
	movq	ARG6, %r13 // B
	movq	ARG7, %r14 // sdb
	sall	$ 5, %r14d // 4*sdb*sizeof(double)
	movq	ARG5, %r15 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMM_NN_RL_24X4_LIB8
#else
	CALL(inner_edge_trmm_nn_rl_24x4_lib8)
#endif

#if MACRO_LEVEL>=1
	INNER_EDGE_GEMM_ADD_NN_24X4_LIB8
#else
	CALL(inner_edge_gemm_add_nn_24x4_lib8)
#endif

	// call inner dgemm kernel nt after initial triangle

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_24X4_LIB8
#else
	CALL(inner_kernel_gemm_add_nn_24x4_lib8)
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha

#if MACRO_LEVEL>=1
	INNER_SCALE_A0_24X4_LIB8
#else
	CALL(inner_scale_a0_24x4_lib8)
#endif


	// store n

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_STORE_24X4_LIB8
#else
	CALL(inner_store_24x4_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_strmm_nn_rl_24x4_lib8)





//                                      1      2             3         4        5            6         7        8         9        10      11
// void kernel_strmm_nn_rl_24x4_vs_lib8(int k, float *alpha, float *A, int sda, int offsetB, float *B, int sdb, float *D, int sdd, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_strmm_nn_rl_24x4_vs_lib8)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// initial triangle

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG4, %r12 // sdb
	sall	$ 5, %r12d // 4*sdb*sizeof(double)
	movq	ARG6, %r13 // B
	movq	ARG7, %r14 // sdb
	sall	$ 5, %r14d // 4*sdb*sizeof(double)
	movq	ARG5, %r15 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMM_NN_RL_24X4_LIB8
#else
	CALL(inner_edge_trmm_nn_rl_24x4_lib8)
#endif

#if MACRO_LEVEL>=1
	INNER_EDGE_GEMM_ADD_NN_24X4_LIB8
#else
	CALL(inner_edge_gemm_add_nn_24x4_lib8)
#endif

	// call inner dgemm kernel nt after initial triangle

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_24X4_LIB8
#else
	CALL(inner_kernel_gemm_add_nn_24x4_lib8)
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha

#if MACRO_LEVEL>=1
	INNER_SCALE_A0_24X4_LIB8
#else
	CALL(inner_scale_a0_24x4_lib8)
#endif


	// store n

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // sdd
	sall	$ 5, %r11d // 4*sdd*sizeof(double)
	movq	ARG10, %r12 // km
	movq	ARG11, %r13 // kn

#if MACRO_LEVEL>=1
	INNER_STORE_24X4_VS_LIB8
#else
	CALL(inner_store_24x4_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_strmm_nn_rl_24x4_vs_lib8)





//#if defined(BLAS_API)
#if ( defined(BLAS_API) | ( defined(LA_HIGH_PERFORMANCE) & defined(MF_COLMAJ) ) )

#include "kernel_sgemm_24x4_lib.S"

#endif





	// read-only data
#if defined(OS_LINUX)
	.section	.rodata.cst32,"aM",@progbits,32
#elif defined(OS_MAC)
	.section	__TEXT,__const
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC00: // { 7.5 6.5 5.5 4.5 3.5 2.5 1.5 0.5 }
#elif defined(OS_MAC)
	.align 5
LC00: // { 7.5 6.5 5.5 4.5 3.5 2.5 1.5 0.5 }
#endif
	.long	1056964608
	.long	1069547520
	.long	1075838976
	.long	1080033280
	.long	1083179008
	.long	1085276160
	.long	1087373312
	.long	1089470464

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC01: // { 15.5 14.5 13.5 12.5 11.5 10.5 9.5 8.5 }
#elif defined(OS_MAC)
	.align 5
LC01: // { 15.5 14.5 13.5 12.5 11.5 10.5 9.5 8.5 }
#endif
	.long	1091043328
	.long	1092091904
	.long	1093140480
	.long	1094189056
	.long	1095237632
	.long	1096286208
	.long	1097334784
	.long	1098383360

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC02: // { 23.5 22.5 21.5 20.5 19.5 18.5 17.5 16.5 }
#elif defined(OS_MAC)
	.align 5
LC02: // { 23.5 22.5 21.5 20.5 19.5 18.5 17.5 16.5 }
#endif
	.long	1099169792
	.long	1099694080
	.long	1100218368
	.long	1100742656
	.long	1101266944
	.long	1101791232
	.long	1102315520
	.long	1102839808

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC03: // { 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 }
#elif defined(OS_MAC)
	.align 5
LC03: // { 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 }
#endif
	.long	1065353216
	.long	1065353216
	.long	1065353216
	.long	1065353216
	.long	1065353216
	.long	1065353216
	.long	1065353216
	.long	1065353216

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC04: // { -1.0 -1.0 1.0 1.0 1.0 1.0 1.0 1.0 }
#elif defined(OS_MAC)
	.align 5
LC04: // { -1.0 -1.0 1.0 1.0 1.0 1.0 1.0 1.0 }
#endif
	.long	1065353216
	.long	1065353216
	.long	1065353216
	.long	1065353216
	.long	1065353216
	.long	1065353216
	.long	3212836864
	.long	3212836864



#if defined(OS_LINUX)
	.section	.note.GNU-stack,"",@progbits
#elif defined(OS_MAC)
	.subsections_via_symbols
#endif

