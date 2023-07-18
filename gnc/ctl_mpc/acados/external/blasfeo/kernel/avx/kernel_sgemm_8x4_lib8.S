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
	.p2align 4,,15; \
	.globl NAME; \
	.type NAME, @function; \
NAME:
#define FUN_START(NAME) \
	.p2align 4,,15; \
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
	vmovaps	%ymm0, %ymm3
//#define NEG_ACC \
//	vmovapd		.LC11(%rip), %ymm15; \
//	vxorpd		%ymm15, %ymm0, %ymm0; \
//	vxorpd		%ymm15, %ymm1, %ymm1; \
//	vxorpd		%ymm15, %ymm2, %ymm2; \
//	vxorpd		%ymm15, %ymm3, %ymm3

#else // defined(OS_MAC)

#define GLOB_FUN_START(NAME) \
	.p2align 4,,15; \
	.globl _ ## NAME; \
_ ## NAME:
#define FUN_START(NAME) \
	.p2align 4,,15; \
_ ## NAME:
#define FUN_END(NAME)
#define CALL(NAME) \
	callq _ ## NAME
#define ZERO_ACC \
	vxorps	%ymm0, %ymm0, %ymm0; \
	vmovaps	%ymm0, %ymm1; \
	vmovaps	%ymm0, %ymm2; \
	vmovaps	%ymm0, %ymm3
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
	.p2align 4,,15; \
	.globl NAME; \
	.def NAME; .scl 2; .type 32; .endef; \
NAME:
#define FUN_START(NAME) \
	.p2align 4,,15; \
	.def NAME; .scl 2; .type 32; .endef; \
NAME:
#define FUN_END(NAME)
#define CALL(NAME) \
	call NAME
#define ZERO_ACC \
	vxorps	%ymm0, %ymm0, %ymm0; \
	vmovaps	%ymm0, %ymm1; \
	vmovaps	%ymm0, %ymm2; \
	vmovaps	%ymm0, %ymm3
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
// r12   <- B
// ymm0  <- [d00 d10 d20 d30 d40 d50 d60 d70]
// ymm1  <- [d01 d11 d21 d31 d41 d51 d61 d71]
// ymm2  <- [d02 d12 d22 d32 d42 d52 d62 d72]
// ymm3  <- [d03 d13 d23 d33 d43 d53 d63 d73]
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
// ymm0  <- [d00 d10 d20 d30 d40 d50 d60 d70]
// ymm1  <- [d01 d11 d21 d31 d41 d51 d61 d71]
// ymm2  <- [d02 d12 d22 d32 d42 d52 d62 d72]
// ymm3  <- [d03 d13 d23 d33 d43 d53 d63 d73]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NT_8X4_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_gemm_add_nt_8x4_lib8, @function
inner_kernel_gemm_add_nt_8x4_lib8:
#elif defined(OS_MAC)
_inner_kernel_gemm_add_nt_8x4_lib8:
#elif defined(OS_WINDOWS)
	.def inner_kernel_gemm_add_nt_8x4_lib8; .scl 2; .type 32; .endef
inner_kernel_gemm_add_nt_8x4_lib8:
#endif
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	// prefetch
	prefetcht0		0(%r12) // software prefetch
	prefetcht0		0+64(%r12) // software prefetch

	// prefetch
	vbroadcastf128	0(%r12), %ymm14 // B
	vmovaps			0(%r11), %ymm12 // A
	vbroadcastf128	32(%r12), %ymm15 // B
	vmovaps			32(%r11), %ymm13 // A

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop
	
	// unroll 0
	vmulps			%ymm12, %ymm14, %ymm11
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14 // 10 11 00 01
	vaddps			%ymm11, %ymm0, %ymm0
	prefetcht0	128(%r12) // software prefetch

	vmulps			%ymm12, %ymm14, %ymm11
	vshufps			$ 0x4e, %ymm14, %ymm14, %ymm14 // 01 00 11 10
	vaddps			%ymm11, %ymm1, %ymm1
	prefetcht0	128+64(%r12) // software prefetch

	vmulps			%ymm12, %ymm14, %ymm11
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14 // 10 11 00 01
	vaddps			%ymm11, %ymm2, %ymm2

	vmulps			%ymm12, %ymm14, %ymm11
	vbroadcastf128	64(%r12), %ymm14 // B
	vaddps			%ymm11, %ymm3, %ymm3
	vmovaps			64(%r11), %ymm12 // A


	// unroll 1
	vmulps			%ymm13, %ymm15, %ymm11
	vshufps			$ 0xb1, %ymm15, %ymm15, %ymm15
	vaddps			%ymm11, %ymm0, %ymm0

	vmulps			%ymm13, %ymm15, %ymm11
	vshufps			$ 0x4e, %ymm15, %ymm15, %ymm15
	vaddps			%ymm11, %ymm1, %ymm1

	vmulps			%ymm13, %ymm15, %ymm11
	vshufps			$ 0xb1, %ymm15, %ymm15, %ymm15
	vaddps			%ymm11, %ymm2, %ymm2

	vmulps			%ymm13, %ymm15, %ymm11
	vbroadcastf128	96(%r12), %ymm15 // B
	vaddps			%ymm11, %ymm3, %ymm3
	vmovaps			96(%r11), %ymm13 // A


	// unroll 2
	vmulps			%ymm12, %ymm14, %ymm11
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vaddps			%ymm11, %ymm0, %ymm0

	vmulps			%ymm12, %ymm14, %ymm11
	vshufps			$ 0x4e, %ymm14, %ymm14, %ymm14
	vaddps			%ymm11, %ymm1, %ymm1

	vmulps			%ymm12, %ymm14, %ymm11
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vaddps			%ymm11, %ymm2, %ymm2

	vmulps			%ymm12, %ymm14, %ymm11
	vbroadcastf128	128(%r12), %ymm14 // B
	vaddps			%ymm11, %ymm3, %ymm3
	vmovaps			128(%r11), %ymm12 // A

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r12

	// unroll 3
	vmulps			%ymm13, %ymm15, %ymm11
	vshufps			$ 0xb1, %ymm15, %ymm15, %ymm15
	vaddps			%ymm11, %ymm0, %ymm0

	vmulps			%ymm13, %ymm15, %ymm11
	vshufps			$ 0x4e, %ymm15, %ymm15, %ymm15
	vaddps			%ymm11, %ymm1, %ymm1

	vmulps			%ymm13, %ymm15, %ymm11
	vshufps			$ 0xb1, %ymm15, %ymm15, %ymm15
	vaddps			%ymm11, %ymm2, %ymm2

	vmulps			%ymm13, %ymm15, %ymm11
	vbroadcastf128	32(%r12), %ymm15 // B
	vaddps			%ymm11, %ymm3, %ymm3
	vmovaps			32(%r11), %ymm13 // A


	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1


	// unroll 0
	vmulps			%ymm12, %ymm14, %ymm11
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vaddps			%ymm11, %ymm0, %ymm0

	vmulps			%ymm12, %ymm14, %ymm11
	vshufps			$ 0x4e, %ymm14, %ymm14, %ymm14
	vaddps			%ymm11, %ymm1, %ymm1

	vmulps			%ymm12, %ymm14, %ymm11
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vaddps			%ymm11, %ymm2, %ymm2

	vmulps			%ymm12, %ymm14, %ymm11
	vbroadcastf128	64(%r12), %ymm14 // B
	vaddps			%ymm11, %ymm3, %ymm3
	vmovaps			64(%r11), %ymm12 // A


	// unroll 1
	vmulps			%ymm13, %ymm15, %ymm11
	vshufps			$ 0xb1, %ymm15, %ymm15, %ymm15
	vaddps			%ymm11, %ymm0, %ymm0

	vmulps			%ymm13, %ymm15, %ymm11
	vshufps			$ 0x4e, %ymm15, %ymm15, %ymm15
	vaddps			%ymm11, %ymm1, %ymm1

	vmulps			%ymm13, %ymm15, %ymm11
	vshufps			$ 0xb1, %ymm15, %ymm15, %ymm15
	vaddps			%ymm11, %ymm2, %ymm2

	vmulps			%ymm13, %ymm15, %ymm11
	vbroadcastf128	96(%r12), %ymm15 // B
	vaddps			%ymm11, %ymm3, %ymm3
	vmovaps			96(%r11), %ymm13 // A


	// unroll 2
	vmulps			%ymm12, %ymm14, %ymm11
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vaddps			%ymm11, %ymm0, %ymm0

	vmulps			%ymm12, %ymm14, %ymm11
	vshufps			$ 0x4e, %ymm14, %ymm14, %ymm14
	vaddps			%ymm11, %ymm1, %ymm1

	vmulps			%ymm12, %ymm14, %ymm11
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vaddps			%ymm11, %ymm2, %ymm2

	vmulps			%ymm12, %ymm14, %ymm11
//	vbroadcastf128	128(%r12), %ymm14 // B
	vaddps			%ymm11, %ymm3, %ymm3
//	vmovaps			128(%r11), %ymm12 // A

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r12

	// unroll 3
	vmulps			%ymm13, %ymm15, %ymm11
	vshufps			$ 0xb1, %ymm15, %ymm15, %ymm15
	vaddps			%ymm11, %ymm0, %ymm0

	vmulps			%ymm13, %ymm15, %ymm11
	vshufps			$ 0x4e, %ymm15, %ymm15, %ymm15
	vaddps			%ymm11, %ymm1, %ymm1

	vmulps			%ymm13, %ymm15, %ymm11
	vshufps			$ 0xb1, %ymm15, %ymm15, %ymm15
	vaddps			%ymm11, %ymm2, %ymm2

	vmulps			%ymm13, %ymm15, %ymm11
//	vbroadcastf128	32(%r12), %ymm15 // B
	vaddps			%ymm11, %ymm3, %ymm3
//	vmovaps			32(%r11), %ymm13 // A


//	cmpl	$ 4, %r10d
	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vbroadcastf128	0(%r12), %ymm14 // B
	vmovaps			0(%r11), %ymm12 // A
	vmulps			%ymm12, %ymm14, %ymm11
	vaddps			%ymm11, %ymm0, %ymm0

	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vmulps			%ymm12, %ymm14, %ymm11
	vaddps			%ymm11, %ymm1, %ymm1

	vshufps			$ 0x4e, %ymm14, %ymm14, %ymm14
	vmulps			%ymm12, %ymm14, %ymm11
	vaddps			%ymm11, %ymm2, %ymm2

	subl	$ 1, %r10d
	addq	$ 32, %r11
	addq	$ 32, %r12

	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vmulps			%ymm12, %ymm14, %ymm11
	vaddps			%ymm11, %ymm3, %ymm3

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_gemm_add_nt_8x4_lib8, .-inner_kernel_gemm_add_nt_8x4_lib8
#endif
#endif





// common inner routine with file scope
//
// input arguments:
// r10d   <- k
// r11   <- A
// r12   <- B
// ymm0  <- [d00 d10 d20 d30 d40 d50 d60 d70]
// ymm1  <- [d01 d11 d21 d31 d41 d51 d61 d71]
// ymm2  <- [d02 d12 d22 d32 d42 d52 d62 d72]
// ymm3  <- [d03 d13 d23 d33 d43 d53 d63 d73]
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
// ymm0  <- [d00 d10 d20 d30 d40 d50 d60 d70]
// ymm1  <- [d01 d11 d21 d31 d41 d51 d61 d71]
// ymm2  <- [d02 d12 d22 d32 d42 d52 d62 d72]
// ymm3  <- [d03 d13 d23 d33 d43 d53 d63 d73]
// ymm8  <- dirty
// ymm9  <- dirty
// ymm10 <- dirty
// ymm11 <- dirty
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_SUB_NT_8X4_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_gemm_sub_nt_8x4_lib8, @function
inner_kernel_gemm_sub_nt_8x4_lib8:
#elif defined(OS_MAC)
_inner_kernel_gemm_sub_nt_8x4_lib8:
#elif defined(OS_WINDOWS)
	.def inner_kernel_gemm_sub_nt_8x4_lib8; .scl 2; .type 32; .endef
inner_kernel_gemm_sub_nt_8x4_lib8:
#endif
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	// prefetch
	vbroadcastf128	0(%r12), %ymm14 // B
	vmovaps			0(%r11), %ymm12 // A
	vbroadcastf128	32(%r12), %ymm15 // B
	vmovaps			32(%r11), %ymm13 // A

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop
	
	// unroll 0
	vmulps			%ymm12, %ymm14, %ymm11
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14 // 10 11 00 01
	vsubps			%ymm11, %ymm0, %ymm0

	vmulps			%ymm12, %ymm14, %ymm11
	vshufps			$ 0x4e, %ymm14, %ymm14, %ymm14 // 01 00 11 10
	vsubps			%ymm11, %ymm1, %ymm1

	vmulps			%ymm12, %ymm14, %ymm11
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14 // 10 11 00 01
	vsubps			%ymm11, %ymm2, %ymm2

	vmulps			%ymm12, %ymm14, %ymm11
	vbroadcastf128	64(%r12), %ymm14 // B
	vsubps			%ymm11, %ymm3, %ymm3
	vmovaps			64(%r11), %ymm12 // A


	// unroll 1
	vmulps			%ymm13, %ymm15, %ymm11
	vshufps			$ 0xb1, %ymm15, %ymm15, %ymm15
	vsubps			%ymm11, %ymm0, %ymm0

	vmulps			%ymm13, %ymm15, %ymm11
	vshufps			$ 0x4e, %ymm15, %ymm15, %ymm15
	vsubps			%ymm11, %ymm1, %ymm1

	vmulps			%ymm13, %ymm15, %ymm11
	vshufps			$ 0xb1, %ymm15, %ymm15, %ymm15
	vsubps			%ymm11, %ymm2, %ymm2

	vmulps			%ymm13, %ymm15, %ymm11
	vbroadcastf128	96(%r12), %ymm15 // B
	vsubps			%ymm11, %ymm3, %ymm3
	vmovaps			96(%r11), %ymm13 // A


	// unroll 2
	vmulps			%ymm12, %ymm14, %ymm11
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vsubps			%ymm11, %ymm0, %ymm0

	vmulps			%ymm12, %ymm14, %ymm11
	vshufps			$ 0x4e, %ymm14, %ymm14, %ymm14
	vsubps			%ymm11, %ymm1, %ymm1

	vmulps			%ymm12, %ymm14, %ymm11
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vsubps			%ymm11, %ymm2, %ymm2

	vmulps			%ymm12, %ymm14, %ymm11
	vbroadcastf128	128(%r12), %ymm14 // B
	vsubps			%ymm11, %ymm3, %ymm3
	vmovaps			128(%r11), %ymm12 // A

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r12

	// unroll 3
	vmulps			%ymm13, %ymm15, %ymm11
	vshufps			$ 0xb1, %ymm15, %ymm15, %ymm15
	vsubps			%ymm11, %ymm0, %ymm0

	vmulps			%ymm13, %ymm15, %ymm11
	vshufps			$ 0x4e, %ymm15, %ymm15, %ymm15
	vsubps			%ymm11, %ymm1, %ymm1

	vmulps			%ymm13, %ymm15, %ymm11
	vshufps			$ 0xb1, %ymm15, %ymm15, %ymm15
	vsubps			%ymm11, %ymm2, %ymm2

	vmulps			%ymm13, %ymm15, %ymm11
	vbroadcastf128	32(%r12), %ymm15 // B
	vsubps			%ymm11, %ymm3, %ymm3
	vmovaps			32(%r11), %ymm13 // A


	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1


	// unroll 0
	vmulps			%ymm12, %ymm14, %ymm11
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vsubps			%ymm11, %ymm0, %ymm0

	vmulps			%ymm12, %ymm14, %ymm11
	vshufps			$ 0x4e, %ymm14, %ymm14, %ymm14
	vsubps			%ymm11, %ymm1, %ymm1

	vmulps			%ymm12, %ymm14, %ymm11
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vsubps			%ymm11, %ymm2, %ymm2

	vmulps			%ymm12, %ymm14, %ymm11
	vbroadcastf128	64(%r12), %ymm14 // B
	vsubps			%ymm11, %ymm3, %ymm3
	vmovaps			64(%r11), %ymm12 // A


	// unroll 1
	vmulps			%ymm13, %ymm15, %ymm11
	vshufps			$ 0xb1, %ymm15, %ymm15, %ymm15
	vsubps			%ymm11, %ymm0, %ymm0

	vmulps			%ymm13, %ymm15, %ymm11
	vshufps			$ 0x4e, %ymm15, %ymm15, %ymm15
	vsubps			%ymm11, %ymm1, %ymm1

	vmulps			%ymm13, %ymm15, %ymm11
	vshufps			$ 0xb1, %ymm15, %ymm15, %ymm15
	vsubps			%ymm11, %ymm2, %ymm2

	vmulps			%ymm13, %ymm15, %ymm11
	vbroadcastf128	96(%r12), %ymm15 // B
	vsubps			%ymm11, %ymm3, %ymm3
	vmovaps			96(%r11), %ymm13 // A


	// unroll 2
	vmulps			%ymm12, %ymm14, %ymm11
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vsubps			%ymm11, %ymm0, %ymm0

	vmulps			%ymm12, %ymm14, %ymm11
	vshufps			$ 0x4e, %ymm14, %ymm14, %ymm14
	vsubps			%ymm11, %ymm1, %ymm1

	vmulps			%ymm12, %ymm14, %ymm11
	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vsubps			%ymm11, %ymm2, %ymm2

	vmulps			%ymm12, %ymm14, %ymm11
//	vbroadcastf128	128(%r12), %ymm14 // B
	vsubps			%ymm11, %ymm3, %ymm3
//	vmovaps			128(%r11), %ymm12 // A

	subl	$ 4, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r12

	// unroll 3
	vmulps			%ymm13, %ymm15, %ymm11
	vshufps			$ 0xb1, %ymm15, %ymm15, %ymm15
	vsubps			%ymm11, %ymm0, %ymm0

	vmulps			%ymm13, %ymm15, %ymm11
	vshufps			$ 0x4e, %ymm15, %ymm15, %ymm15
	vsubps			%ymm11, %ymm1, %ymm1

	vmulps			%ymm13, %ymm15, %ymm11
	vshufps			$ 0xb1, %ymm15, %ymm15, %ymm15
	vsubps			%ymm11, %ymm2, %ymm2

	vmulps			%ymm13, %ymm15, %ymm11
//	vbroadcastf128	32(%r12), %ymm15 // B
	vsubps			%ymm11, %ymm3, %ymm3
//	vmovaps			32(%r11), %ymm13 // A


//	cmpl	$ 4, %r10d
	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vbroadcastf128	0(%r12), %ymm14 // B
	vmovaps			0(%r11), %ymm12 // A
	vmulps			%ymm12, %ymm14, %ymm11
	vsubps			%ymm11, %ymm0, %ymm0

	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vmulps			%ymm12, %ymm14, %ymm11
	vsubps			%ymm11, %ymm1, %ymm1

	vshufps			$ 0x4e, %ymm14, %ymm14, %ymm14
	vmulps			%ymm12, %ymm14, %ymm11
	vsubps			%ymm11, %ymm2, %ymm2

	subl	$ 1, %r10d
	addq	$ 32, %r11
	addq	$ 32, %r12

	vshufps			$ 0xb1, %ymm14, %ymm14, %ymm14
	vmulps			%ymm12, %ymm14, %ymm11
	vsubps			%ymm11, %ymm3, %ymm3

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_gemm_sub_nt_8x4_lib8, .-inner_kernel_gemm_sub_nt_8x4_lib8
#endif
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// r13   <- 4*sdb*sizeof(double)
// r14   <= dirty
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
// r12   <- B+(k/4)*sdb*sizeof(double)+(k%4)
// r13   <- 4*sdb*sizeof(double)
// r14   <= dirty
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
	.macro INNER_KERNEL_GEMM_ADD_NN_8X4_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_gemm_add_nn_8x4_lib8, @function
inner_kernel_gemm_add_nn_8x4_lib8:
#elif defined(OS_MAC)
_inner_kernel_gemm_add_nn_8x4_lib8:
#elif defined(OS_WINDOWS)
	.def inner_kernel_gemm_add_nn_8x4_lib8; .scl 2; .type 32; .endef
inner_kernel_gemm_add_nn_8x4_lib8:
#endif
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	movq	%r12, %r14 // B_next <- B
	addq	%r13, %r14 // B_next <- B + 4*sda*sizeof(double)

	cmpl	$ 8, %r10d
	jl		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop
	
	prefetcht0	0(%r14) // software prefetch
	prefetcht0	64(%r14) // software prefetch

	// unroll 0
	vmovaps			0(%r11), %ymm12 // A[0]
	vbroadcastss	0(%r12), %ymm13 // B[0]
	vmulps			%ymm12, %ymm13, %ymm15
	vaddps			%ymm15, %ymm0, %ymm0
	vbroadcastss	32(%r12), %ymm13 // B[1]
	vmulps			%ymm12, %ymm13, %ymm15
	vaddps			%ymm15, %ymm1, %ymm1
	vbroadcastss	64(%r12), %ymm13 // B[2]
	vmulps			%ymm12, %ymm13, %ymm15
	vaddps			%ymm15, %ymm2, %ymm2
	vbroadcastss	96(%r12), %ymm13 // B[3]
	vmulps			%ymm12, %ymm13, %ymm15
	vaddps			%ymm15, %ymm3, %ymm3


	// unroll 1
	vmovaps			32(%r11), %ymm12 // A[0]
	vbroadcastss	4(%r12), %ymm13 // B[0]
	vmulps			%ymm12, %ymm13, %ymm15
	vaddps			%ymm15, %ymm0, %ymm0
	vbroadcastss	36(%r12), %ymm13 // B[1]
	vmulps			%ymm12, %ymm13, %ymm15
	vaddps			%ymm15, %ymm1, %ymm1
	vbroadcastss	68(%r12), %ymm13 // B[2]
	vmulps			%ymm12, %ymm13, %ymm15
	vaddps			%ymm15, %ymm2, %ymm2
	vbroadcastss	100(%r12), %ymm13 // B[3]
	vmulps			%ymm12, %ymm13, %ymm15
	vaddps			%ymm15, %ymm3, %ymm3


	// unroll 2
	vmovaps			64(%r11), %ymm12 // A[0]
	vbroadcastss	8(%r12), %ymm13 // B[0]
	vmulps			%ymm12, %ymm13, %ymm15
	vaddps			%ymm15, %ymm0, %ymm0
	vbroadcastss	40(%r12), %ymm13 // B[1]
	vmulps			%ymm12, %ymm13, %ymm15
	vaddps			%ymm15, %ymm1, %ymm1
	vbroadcastss	72(%r12), %ymm13 // B[2]
	vmulps			%ymm12, %ymm13, %ymm15
	vaddps			%ymm15, %ymm2, %ymm2
	vbroadcastss	104(%r12), %ymm13 // B[3]
	vmulps			%ymm12, %ymm13, %ymm15
	vaddps			%ymm15, %ymm3, %ymm3


	// unroll 3
	vmovaps			96(%r11), %ymm12 // A[0]
	vbroadcastss	12(%r12), %ymm13 // B[0]
	vmulps			%ymm12, %ymm13, %ymm15
	vaddps			%ymm15, %ymm0, %ymm0
	vbroadcastss	44(%r12), %ymm13 // B[1]
	vmulps			%ymm12, %ymm13, %ymm15
	vaddps			%ymm15, %ymm1, %ymm1
	vbroadcastss	76(%r12), %ymm13 // B[2]
	vmulps			%ymm12, %ymm13, %ymm15
	vaddps			%ymm15, %ymm2, %ymm2
	vbroadcastss	108(%r12), %ymm13 // B[3]
	vmulps			%ymm12, %ymm13, %ymm15
	vaddps			%ymm15, %ymm3, %ymm3


	// unroll 4
	vmovaps			128(%r11), %ymm12 // A[0]
	vbroadcastss	16(%r12), %ymm13 // B[0]
	vmulps			%ymm12, %ymm13, %ymm15
	vaddps			%ymm15, %ymm0, %ymm0
	vbroadcastss	48(%r12), %ymm13 // B[1]
	vmulps			%ymm12, %ymm13, %ymm15
	vaddps			%ymm15, %ymm1, %ymm1
	vbroadcastss	80(%r12), %ymm13 // B[2]
	vmulps			%ymm12, %ymm13, %ymm15
	vaddps			%ymm15, %ymm2, %ymm2
	vbroadcastss	112(%r12), %ymm13 // B[3]
	vmulps			%ymm12, %ymm13, %ymm15
	vaddps			%ymm15, %ymm3, %ymm3


	// unroll 5
	vmovaps			160(%r11), %ymm12 // A[0]
	vbroadcastss	20(%r12), %ymm13 // B[0]
	vmulps			%ymm12, %ymm13, %ymm15
	vaddps			%ymm15, %ymm0, %ymm0
	vbroadcastss	52(%r12), %ymm13 // B[1]
	vmulps			%ymm12, %ymm13, %ymm15
	vaddps			%ymm15, %ymm1, %ymm1
	vbroadcastss	84(%r12), %ymm13 // B[2]
	vmulps			%ymm12, %ymm13, %ymm15
	vaddps			%ymm15, %ymm2, %ymm2
	vbroadcastss	116(%r12), %ymm13 // B[3]
	vmulps			%ymm12, %ymm13, %ymm15
	vaddps			%ymm15, %ymm3, %ymm3


	// unroll 6
	vmovaps			192(%r11), %ymm12 // A[0]
	vbroadcastss	24(%r12), %ymm13 // B[0]
	vmulps			%ymm12, %ymm13, %ymm15
	vaddps			%ymm15, %ymm0, %ymm0
	vbroadcastss	56(%r12), %ymm13 // B[1]
	vmulps			%ymm12, %ymm13, %ymm15
	vaddps			%ymm15, %ymm1, %ymm1
	vbroadcastss	88(%r12), %ymm13 // B[2]
	vmulps			%ymm12, %ymm13, %ymm15
	vaddps			%ymm15, %ymm2, %ymm2
	vbroadcastss	120(%r12), %ymm13 // B[3]
	vmulps			%ymm12, %ymm13, %ymm15
	vaddps			%ymm15, %ymm3, %ymm3


	// unroll 7
	vmovaps			224(%r11), %ymm12 // A[0]
	vbroadcastss	28(%r12), %ymm13 // B[0]
	vmulps			%ymm12, %ymm13, %ymm15
	vaddps			%ymm15, %ymm0, %ymm0
	vbroadcastss	60(%r12), %ymm13 // B[1]
	vmulps			%ymm12, %ymm13, %ymm15
	vaddps			%ymm15, %ymm1, %ymm1
	vbroadcastss	92(%r12), %ymm13 // B[2]
	vmulps			%ymm12, %ymm13, %ymm15
	vaddps			%ymm15, %ymm2, %ymm2
	vbroadcastss	124(%r12), %ymm13 // B[3]
	vmulps			%ymm12, %ymm13, %ymm15
	vaddps			%ymm15, %ymm3, %ymm3

	subl	$ 8, %r10d
	addq	$ 256, %r11

	mov		%r14, %r12
	addq	%r13, %r14

	cmpl	$ 7, %r10d
	jg		1b // main loop 


0: // consider clean1-up loop
	
	cmpl	$ 0, %r10d
	jle		2f // return

3: // clean1-up loop
	
	vmovaps			0(%r11), %ymm12 // A[0]
	vbroadcastss	0(%r12), %ymm13 // B[0]
	vmulps			%ymm12, %ymm13, %ymm15
	vaddps			%ymm15, %ymm0, %ymm0
	vbroadcastss	32(%r12), %ymm13 // B[1]
	vmulps			%ymm12, %ymm13, %ymm15
	vaddps			%ymm15, %ymm1, %ymm1
	vbroadcastss	64(%r12), %ymm13 // B[2]
	vmulps			%ymm12, %ymm13, %ymm15
	vaddps			%ymm15, %ymm2, %ymm2
	vbroadcastss	96(%r12), %ymm13 // B[3]
	vmulps			%ymm12, %ymm13, %ymm15
	vaddps			%ymm15, %ymm3, %ymm3

	subl	$ 1, %r10d
	addq	$ 32, %r11
	addq	$ 4, %r12

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_gemm_add_nn_8x4_lib8, .-inner_kernel_gemm_add_nn_8x4_lib8
#endif
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// r13   <- 4*sdb*sizeof(double)
// r14   <= dirty
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
// r12   <- B+(k/4)*sdb*sizeof(double)+(k%4)
// r13   <- 4*sdb*sizeof(double)
// r14   <= dirty
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
	.macro INNER_KERNEL_GEMM_SUB_NN_8X4_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_gemm_sub_nn_8x4_lib8, @function
inner_kernel_gemm_sub_nn_8x4_lib8:
#elif defined(OS_MAC)
_inner_kernel_gemm_sub_nn_8x4_lib8:
#elif defined(OS_WINDOWS)
	.def inner_kernel_gemm_sub_nn_8x4_lib8; .scl 2; .type 32; .endef
inner_kernel_gemm_sub_nn_8x4_lib8:
#endif
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	movq	%r12, %r14 // B_next <- B
	addq	%r13, %r14 // B_next <- B + 4*sda*sizeof(double)

	cmpl	$ 8, %r10d
	jl		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop
	
	prefetcht0	0(%r14) // software prefetch
	prefetcht0	64(%r14) // software prefetch

	// unroll 0
	vmovaps			0(%r11), %ymm12 // A[0]
	vbroadcastss	0(%r12), %ymm13 // B[0]
	vmulps			%ymm12, %ymm13, %ymm15
	vsubps			%ymm15, %ymm0, %ymm0
	vbroadcastss	32(%r12), %ymm13 // B[1]
	vmulps			%ymm12, %ymm13, %ymm15
	vsubps			%ymm15, %ymm1, %ymm1
	vbroadcastss	64(%r12), %ymm13 // B[2]
	vmulps			%ymm12, %ymm13, %ymm15
	vsubps			%ymm15, %ymm2, %ymm2
	vbroadcastss	96(%r12), %ymm13 // B[3]
	vmulps			%ymm12, %ymm13, %ymm15
	vsubps			%ymm15, %ymm3, %ymm3


	// unroll 1
	vmovaps			32(%r11), %ymm12 // A[0]
	vbroadcastss	4(%r12), %ymm13 // B[0]
	vmulps			%ymm12, %ymm13, %ymm15
	vsubps			%ymm15, %ymm0, %ymm0
	vbroadcastss	36(%r12), %ymm13 // B[1]
	vmulps			%ymm12, %ymm13, %ymm15
	vsubps			%ymm15, %ymm1, %ymm1
	vbroadcastss	68(%r12), %ymm13 // B[2]
	vmulps			%ymm12, %ymm13, %ymm15
	vsubps			%ymm15, %ymm2, %ymm2
	vbroadcastss	100(%r12), %ymm13 // B[3]
	vmulps			%ymm12, %ymm13, %ymm15
	vsubps			%ymm15, %ymm3, %ymm3


	// unroll 2
	vmovaps			64(%r11), %ymm12 // A[0]
	vbroadcastss	8(%r12), %ymm13 // B[0]
	vmulps			%ymm12, %ymm13, %ymm15
	vsubps			%ymm15, %ymm0, %ymm0
	vbroadcastss	40(%r12), %ymm13 // B[1]
	vmulps			%ymm12, %ymm13, %ymm15
	vsubps			%ymm15, %ymm1, %ymm1
	vbroadcastss	72(%r12), %ymm13 // B[2]
	vmulps			%ymm12, %ymm13, %ymm15
	vsubps			%ymm15, %ymm2, %ymm2
	vbroadcastss	104(%r12), %ymm13 // B[3]
	vmulps			%ymm12, %ymm13, %ymm15
	vsubps			%ymm15, %ymm3, %ymm3


	// unroll 3
	vmovaps			96(%r11), %ymm12 // A[0]
	vbroadcastss	12(%r12), %ymm13 // B[0]
	vmulps			%ymm12, %ymm13, %ymm15
	vsubps			%ymm15, %ymm0, %ymm0
	vbroadcastss	44(%r12), %ymm13 // B[1]
	vmulps			%ymm12, %ymm13, %ymm15
	vsubps			%ymm15, %ymm1, %ymm1
	vbroadcastss	76(%r12), %ymm13 // B[2]
	vmulps			%ymm12, %ymm13, %ymm15
	vsubps			%ymm15, %ymm2, %ymm2
	vbroadcastss	108(%r12), %ymm13 // B[3]
	vmulps			%ymm12, %ymm13, %ymm15
	vsubps			%ymm15, %ymm3, %ymm3


	// unroll 4
	vmovaps			128(%r11), %ymm12 // A[0]
	vbroadcastss	16(%r12), %ymm13 // B[0]
	vmulps			%ymm12, %ymm13, %ymm15
	vsubps			%ymm15, %ymm0, %ymm0
	vbroadcastss	48(%r12), %ymm13 // B[1]
	vmulps			%ymm12, %ymm13, %ymm15
	vsubps			%ymm15, %ymm1, %ymm1
	vbroadcastss	80(%r12), %ymm13 // B[2]
	vmulps			%ymm12, %ymm13, %ymm15
	vsubps			%ymm15, %ymm2, %ymm2
	vbroadcastss	112(%r12), %ymm13 // B[3]
	vmulps			%ymm12, %ymm13, %ymm15
	vsubps			%ymm15, %ymm3, %ymm3


	// unroll 5
	vmovaps			160(%r11), %ymm12 // A[0]
	vbroadcastss	20(%r12), %ymm13 // B[0]
	vmulps			%ymm12, %ymm13, %ymm15
	vsubps			%ymm15, %ymm0, %ymm0
	vbroadcastss	52(%r12), %ymm13 // B[1]
	vmulps			%ymm12, %ymm13, %ymm15
	vsubps			%ymm15, %ymm1, %ymm1
	vbroadcastss	84(%r12), %ymm13 // B[2]
	vmulps			%ymm12, %ymm13, %ymm15
	vsubps			%ymm15, %ymm2, %ymm2
	vbroadcastss	116(%r12), %ymm13 // B[3]
	vmulps			%ymm12, %ymm13, %ymm15
	vsubps			%ymm15, %ymm3, %ymm3


	// unroll 6
	vmovaps			192(%r11), %ymm12 // A[0]
	vbroadcastss	24(%r12), %ymm13 // B[0]
	vmulps			%ymm12, %ymm13, %ymm15
	vsubps			%ymm15, %ymm0, %ymm0
	vbroadcastss	56(%r12), %ymm13 // B[1]
	vmulps			%ymm12, %ymm13, %ymm15
	vsubps			%ymm15, %ymm1, %ymm1
	vbroadcastss	88(%r12), %ymm13 // B[2]
	vmulps			%ymm12, %ymm13, %ymm15
	vsubps			%ymm15, %ymm2, %ymm2
	vbroadcastss	120(%r12), %ymm13 // B[3]
	vmulps			%ymm12, %ymm13, %ymm15
	vsubps			%ymm15, %ymm3, %ymm3


	// unroll 7
	vmovaps			224(%r11), %ymm12 // A[0]
	vbroadcastss	28(%r12), %ymm13 // B[0]
	vmulps			%ymm12, %ymm13, %ymm15
	vsubps			%ymm15, %ymm0, %ymm0
	vbroadcastss	60(%r12), %ymm13 // B[1]
	vmulps			%ymm12, %ymm13, %ymm15
	vsubps			%ymm15, %ymm1, %ymm1
	vbroadcastss	92(%r12), %ymm13 // B[2]
	vmulps			%ymm12, %ymm13, %ymm15
	vsubps			%ymm15, %ymm2, %ymm2
	vbroadcastss	124(%r12), %ymm13 // B[3]
	vmulps			%ymm12, %ymm13, %ymm15
	vsubps			%ymm15, %ymm3, %ymm3

	subl	$ 8, %r10d
	addq	$ 256, %r11

	mov		%r14, %r12
	addq	%r13, %r14

	cmpl	$ 7, %r10d
	jg		1b // main loop 


0: // consider clean1-up loop
	
	cmpl	$ 0, %r10d
	jle		2f // return

3: // clean1-up loop
	
	vmovaps			0(%r11), %ymm12 // A[0]
	vbroadcastss	0(%r12), %ymm13 // B[0]
	vmulps			%ymm12, %ymm13, %ymm15
	vsubps			%ymm15, %ymm0, %ymm0
	vbroadcastss	32(%r12), %ymm13 // B[1]
	vmulps			%ymm12, %ymm13, %ymm15
	vsubps			%ymm15, %ymm1, %ymm1
	vbroadcastss	64(%r12), %ymm13 // B[2]
	vmulps			%ymm12, %ymm13, %ymm15
	vsubps			%ymm15, %ymm2, %ymm2
	vbroadcastss	96(%r12), %ymm13 // B[3]
	vmulps			%ymm12, %ymm13, %ymm15
	vsubps			%ymm15, %ymm3, %ymm3

	subl	$ 1, %r10d
	addq	$ 32, %r11
	addq	$ 4, %r12

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_gemm_sub_nn_8x4_lib8, .-inner_kernel_gemm_sub_nn_8x4_lib8
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
// r12   <- B-offB+bs*sdb*sizeof(double)
// r13   <- bs*sdb*sizeof(double)
// r14   <- offB
// ymm0  <- []
// ymm1  <- []
// ymm2  <- []
// ymm3  <- []
// ymm8  <- dirty
// ymm12 <- dirty
// ymm15 <- dirty


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_GEMM_ADD_NN_8X4_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_edge_gemm_add_nn_8x4_lib8, @function
inner_edge_gemm_add_nn_8x4_lib8:
#elif defined(OS_MAC)
_inner_edge_gemm_add_nn_8x4_lib8:
#elif defined(OS_WINDOWS)
	.def inner_edge_gemm_add_nn_8x4_lib8; .scl 2; .type 32; .endef
inner_edge_gemm_add_nn_8x4_lib8:
#endif
#endif
	
	cmpl			$ 0, %r14d // offset==0
	jle				2f // end

	cmpl			$ 0, %r10d // k==0
	jle				2f // end

	movl			$ 8, %r15d
	subl			%r14d, %r15d // 8-offsetB
	cmpl			%r10d, %r15d
//	jle				0f
//	movl			%r10d, %r15d // kend=min(k,8-offsetB)
//0:
	cmovgl			%r10d, %r15d // kend=min(k,8-offsetB)

	movl			%r14d, %eax
	sall			$ 2, %eax // offsetB*sizeof(float)
	addq			%rax, %r12 // B+offsetB*sizeof(float)

1:
	vmovaps			0(%r11), %ymm8
	vbroadcastss	0(%r12), %ymm12
	vmulps			%ymm8, %ymm12, %ymm15
	vaddps			%ymm15, %ymm0, %ymm0
	vbroadcastss	32(%r12), %ymm12
	vmulps			%ymm8, %ymm12, %ymm15
	vaddps			%ymm15, %ymm1, %ymm1
	vbroadcastss	64(%r12), %ymm12
	vmulps			%ymm8, %ymm12, %ymm15
	vaddps			%ymm15, %ymm2, %ymm2
	vbroadcastss	96(%r12), %ymm12
	vmulps			%ymm8, %ymm12, %ymm15
	vaddps			%ymm15, %ymm3, %ymm3

	subl			$ 1, %r10d // k-1
	subl			$ 1, %r15d // kend-1
	addq			$ 32, %r11 // A+1*bs*sizeof(float)
	addq			$ 4, %r12 // B+1*sizeof(float)

	cmpl			$ 0, %r15d
	jg				1b

	cmpl			$ 0, %r10d
	jle				2f // end

	addq			%r13, %r12
	subq			$ 32, %r12 // B+bs*(sdb-1)*sizeof(float)

2:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_edge_gemm_add_nn_8x4_lib8, .-inner_edge_gemm_add_nn_8x4_lib8
#endif
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
	.macro INNER_EDGE_TRMM_NN_RL_8X4_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_edge_trmm_nn_rl_8x4_lib8, @function
inner_edge_trmm_nn_rl_8x4_lib8:
#elif defined(OS_MAC)
_inner_edge_trmm_nn_rl_8x4_lib8:
#elif defined(OS_WINDOWS)
	.def inner_edge_trmm_nn_rl_8x4_lib8; .scl 2; .type 32; .endef
inner_edge_trmm_nn_rl_8x4_lib8:
#endif
#endif
	
	cmpl		$ 0, %r10d
	jle			0f // end

	movl		%r14d, %eax
	sall		$ 2, %eax // offsetB*sizeof(float)
	movq		%r12, %rbx // B
	addq		%rax, %rbx // B+offsetB*sizeof(float)


	cmpl	$ 4, %r14d
	jg		1f

	// offB==0, 1, 2, 3, 4

	vmovaps			0(%r11), %ymm8
	vbroadcastss	0(%rbx), %ymm12
	vmulps			%ymm8, %ymm12, %ymm15
	vaddps			%ymm15, %ymm0, %ymm0

	subl		$ 1, %r10d // k-1
	addq		$ 32, %r11 // A+1*bs*sizeof(float)
	addl		$ 1, %r14d // offsetB+1

	cmpl		$ 0, %r10d
	jle			0f // end

	vmovaps			0(%r11), %ymm8
	vbroadcastss	4(%rbx), %ymm12
	vmulps			%ymm8, %ymm12, %ymm15
	vaddps			%ymm15, %ymm0, %ymm0
	vbroadcastss	36(%rbx), %ymm12
	vmulps			%ymm8, %ymm12, %ymm15
	vaddps			%ymm15, %ymm1, %ymm1

	subl		$ 1, %r10d // k-1
	addq		$ 32, %r11 // A+1*bs*sizeof(float)
	addl		$ 1, %r14d // offsetB+1

	cmpl		$ 0, %r10d
	jle			0f // end

	vmovaps			0(%r11), %ymm8
	vbroadcastss	8(%rbx), %ymm12
	vmulps			%ymm8, %ymm12, %ymm15
	vaddps			%ymm15, %ymm0, %ymm0
	vbroadcastss	40(%rbx), %ymm12
	vmulps			%ymm8, %ymm12, %ymm15
	vaddps			%ymm15, %ymm1, %ymm1
	vbroadcastss	72(%rbx), %ymm12
	vmulps			%ymm8, %ymm12, %ymm15
	vaddps			%ymm15, %ymm2, %ymm2

	subl		$ 1, %r10d // k-1
	addq		$ 32, %r11 // A+1*bs*sizeof(float)
	addl		$ 1, %r14d // offsetB+1

	jmp			0f // end


1:
	cmpl	$ 5, %r14d
	jg		1f

	// offB==5

	vmovaps			0(%r11), %ymm8
	vbroadcastss	0(%rbx), %ymm12
	vmulps			%ymm8, %ymm12, %ymm15
	vaddps			%ymm15, %ymm0, %ymm0

	subl		$ 1, %r10d // k-1
	addq		$ 32, %r11 // A+1*bs*sizeof(float)
	addl		$ 1, %r14d // offsetB+1

	cmpl		$ 0, %r10d
	jle			0f // end

	vmovaps			0(%r11), %ymm8
	vbroadcastss	4(%rbx), %ymm12
	vmulps			%ymm8, %ymm12, %ymm15
	vaddps			%ymm15, %ymm0, %ymm0
	vbroadcastss	36(%rbx), %ymm12
	vmulps			%ymm8, %ymm12, %ymm15
	vaddps			%ymm15, %ymm1, %ymm1

	subl		$ 1, %r10d // k-1
	addq		$ 32, %r11 // A+1*bs*sizeof(float)
	addl		$ 1, %r14d // offsetB+1

	cmpl		$ 0, %r10d
	jle			0f // end

	vmovaps			0(%r11), %ymm8
	vbroadcastss	8(%rbx), %ymm12
	vmulps			%ymm8, %ymm12, %ymm15
	vaddps			%ymm15, %ymm0, %ymm0
	vbroadcastss	40(%rbx), %ymm12
	vmulps			%ymm8, %ymm12, %ymm15
	vaddps			%ymm15, %ymm1, %ymm1
	vbroadcastss	72(%rbx), %ymm12
	vmulps			%ymm8, %ymm12, %ymm15
	vaddps			%ymm15, %ymm2, %ymm2

	subl		$ 1, %r10d // k-1
	addq		$ 32, %r11 // A+1*bs*sizeof(float)
	addq		%r13, %r12 // B+8*sdb*sizeof(float)
	movl		$ 0, %r14d // offsetB=0

	jmp			0f // end


1:
	cmpl	$ 6, %r14d
	jg		1f

	// offB==6

	vmovaps			0(%r11), %ymm8
	vbroadcastss	0(%rbx), %ymm12
	vmulps			%ymm8, %ymm12, %ymm15
	vaddps			%ymm15, %ymm0, %ymm0

	subl		$ 1, %r10d // k-1
	addq		$ 32, %r11 // A+1*bs*sizeof(float)
	addl		$ 1, %r14d // offsetB+1

	cmpl		$ 0, %r10d
	jle			0f // end

	vmovaps			0(%r11), %ymm8
	vbroadcastss	4(%rbx), %ymm12
	vmulps			%ymm8, %ymm12, %ymm15
	vaddps			%ymm15, %ymm0, %ymm0
	vbroadcastss	36(%rbx), %ymm12
	vmulps			%ymm8, %ymm12, %ymm15
	vaddps			%ymm15, %ymm1, %ymm1

	subl		$ 1, %r10d // k-1
	addq		$ 32, %r11 // A+1*bs*sizeof(float)
	addq		%r13, %r12 // B+8*sdb*sizeof(float)
	movq		%r12, %rbx // B
	movl		$ 0, %r14d // offsetB=0

	cmpl		$ 0, %r10d
	jle			0f // end

	vmovaps			0(%r11), %ymm8
	vbroadcastss	0(%rbx), %ymm12
	vmulps			%ymm8, %ymm12, %ymm15
	vaddps			%ymm15, %ymm0, %ymm0
	vbroadcastss	32(%rbx), %ymm12
	vmulps			%ymm8, %ymm12, %ymm15
	vaddps			%ymm15, %ymm1, %ymm1
	vbroadcastss	64(%rbx), %ymm12
	vmulps			%ymm8, %ymm12, %ymm15
	vaddps			%ymm15, %ymm2, %ymm2

	subl		$ 1, %r10d // k-1
	addq		$ 32, %r11 // A+1*bs*sizeof(float)
	addl		$ 1, %r14d // offsetB+1

	jmp			0f // end


1:
//	cmpl	$ 7, %r14d
//	jg		0f

	// offB==6

	vmovaps			0(%r11), %ymm8
	vbroadcastss	0(%rbx), %ymm12
	vmulps			%ymm8, %ymm12, %ymm15
	vaddps			%ymm15, %ymm0, %ymm0

	subl		$ 1, %r10d // k-1
	addq		$ 32, %r11 // A+1*bs*sizeof(float)
	addq		%r13, %r12 // B+8*sdb*sizeof(float)
	movq		%r12, %rbx // B
	movl		$ 0, %r14d // offsetB=0

	cmpl		$ 0, %r10d
	jle			0f // end

	vmovaps			0(%r11), %ymm8
	vbroadcastss	0(%rbx), %ymm12
	vmulps			%ymm8, %ymm12, %ymm15
	vaddps			%ymm15, %ymm0, %ymm0
	vbroadcastss	32(%rbx), %ymm12
	vmulps			%ymm8, %ymm12, %ymm15
	vaddps			%ymm15, %ymm1, %ymm1

	subl		$ 1, %r10d // k-1
	addq		$ 32, %r11 // A+1*bs*sizeof(float)
	addl		$ 1, %r14d // offsetB+1

	cmpl		$ 0, %r10d
	jle			0f // end

	vmovaps			0(%r11), %ymm8
	vbroadcastss	4(%rbx), %ymm12
	vmulps			%ymm8, %ymm12, %ymm15
	vaddps			%ymm15, %ymm0, %ymm0
	vbroadcastss	36(%rbx), %ymm12
	vmulps			%ymm8, %ymm12, %ymm15
	vaddps			%ymm15, %ymm1, %ymm1
	vbroadcastss	68(%rbx), %ymm12
	vmulps			%ymm8, %ymm12, %ymm15
	vaddps			%ymm15, %ymm2, %ymm2

	subl		$ 1, %r10d // k-1
	addq		$ 32, %r11 // A+1*bs*sizeof(float)
	addl		$ 1, %r14d // offsetB+1

//	jmp			0f // end


	// end
0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_edge_trmm_nn_rl_8x4_lib8, .-inner_edge_trmm_nn_rl_8x4_lib8
#endif
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
// r10  <- inv_diag_E
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
// r10  <- inv_diag_E
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []
// ymm12 <- dirty
// ymm13 <- dirty
// ymm14 <- dirty
// ymm15 <- dirty

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRSM_RLT_INV_8X4_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_edge_trsm_rlt_inv_8x4_lib8, @function
inner_edge_trsm_rlt_inv_8x4_lib8:
#elif defined(OS_MAC)
_inner_edge_trsm_rlt_inv_8x4_lib8:
#elif defined(OS_WINDOWS)
	.def inner_edge_trsm_rlt_inv_8x4_lib8; .scl 2; .type 32; .endef
inner_edge_trsm_rlt_inv_8x4_lib8:
#endif
#endif

	vbroadcastss	0(%r11), %ymm13
	vmulps			%ymm0, %ymm13, %ymm0
	vbroadcastss	4(%r10), %ymm13
	vmulps			%ymm0, %ymm13, %ymm12
	vsubps			%ymm12, %ymm1, %ymm1
	vbroadcastss	8(%r10), %ymm13
	vmulps			%ymm0, %ymm13, %ymm12
	vsubps			%ymm12, %ymm2, %ymm2
	vbroadcastss	12(%r10), %ymm13
	vmulps			%ymm0, %ymm13, %ymm12
	vsubps			%ymm12, %ymm3, %ymm3

	vbroadcastss	4(%r11), %ymm13
	vmulps			%ymm1, %ymm13, %ymm1
	vbroadcastss	40(%r10), %ymm13
	vmulps			%ymm1, %ymm13, %ymm12
	vsubps			%ymm12, %ymm2, %ymm2
	vbroadcastss	44(%r10), %ymm13
	vmulps			%ymm1, %ymm13, %ymm12
	vsubps			%ymm12, %ymm3, %ymm3

	vbroadcastss	8(%r11), %ymm13
	vmulps			%ymm2, %ymm13, %ymm2
	vbroadcastss	76(%r10), %ymm13
	vmulps			%ymm2, %ymm13, %ymm12
	vsubps			%ymm12, %ymm3, %ymm3

	vbroadcastss	12(%r11), %ymm13
	vmulps			%ymm3, %ymm13, %ymm3

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_edge_trsm_rlt_inv_8x4_lib8, .-inner_edge_trsm_rlt_inv_8x4_lib8
#endif
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
	.macro INNER_EDGE_TRSM_RLT_INV_8X4_VS_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_edge_trsm_rlt_inv_8x4_vs_lib8, @function
inner_edge_trsm_rlt_inv_8x4_vs_lib8:
#elif defined(OS_MAC)
_inner_edge_trsm_rlt_inv_8x4_vs_lib8:
#elif defined(OS_WINDOWS)
	.def inner_edge_trsm_rlt_inv_8x4_vs_lib8; .scl 2; .type 32; .endef
inner_edge_trsm_rlt_inv_8x4_vs_lib8:
#endif
#endif

	vbroadcastss	0(%r11), %ymm13
	vmulps			%ymm0, %ymm13, %ymm0
	cmpl			$ 2, %r12d
	jl				0f // ret
	vbroadcastss	4(%r10), %ymm13
	vmulps			%ymm0, %ymm13, %ymm12
	vsubps			%ymm12, %ymm1, %ymm1
	vbroadcastss	8(%r10), %ymm13
	vmulps			%ymm0, %ymm13, %ymm12
	vsubps			%ymm12, %ymm2, %ymm2
	vbroadcastss	12(%r10), %ymm13
	vmulps			%ymm0, %ymm13, %ymm12
	vsubps			%ymm12, %ymm3, %ymm3

	vbroadcastss	4(%r11), %ymm13
	vmulps			%ymm1, %ymm13, %ymm1
	cmpl			$ 3, %r12d
	jl				0f // ret
	vbroadcastss	40(%r10), %ymm13
	vmulps			%ymm1, %ymm13, %ymm12
	vsubps			%ymm12, %ymm2, %ymm2
	vbroadcastss	44(%r10), %ymm13
	vmulps			%ymm1, %ymm13, %ymm12
	vsubps			%ymm12, %ymm3, %ymm3

	vbroadcastss	8(%r11), %ymm13
	vmulps			%ymm2, %ymm13, %ymm2
	cmpl			$ 4, %r12d
	jl				0f // ret
	vbroadcastss	76(%r10), %ymm13
	vmulps			%ymm2, %ymm13, %ymm12
	vsubps			%ymm12, %ymm3, %ymm3

	vbroadcastss	12(%r11), %ymm13
	vmulps			%ymm3, %ymm13, %ymm3

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_edge_trsm_rlt_inv_8x4_vs_lib8, .-inner_edge_trsm_rlt_inv_8x4_vs_lib8
#endif
#endif





// common inner routine with file scope
//
// cholesky factorization 
//
// input arguments:
// r10  <- inv_diag_E
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
	.macro INNER_EDGE_POTRF_8X4_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_edge_potrf_8x4_lib8, @function
inner_edge_potrf_8x4_lib8:
#elif defined(OS_MAC)
_inner_edge_potrf_8x4_lib8:
#elif defined(OS_WINDOWS)
	.def inner_edge_potrf_8x4_lib8; .scl 2; .type 32; .endef
inner_edge_potrf_8x4_lib8:
#endif
#endif

	vxorps	%ymm15, %ymm15, %ymm15 // 0.0
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovss	.LC03(%rip), %xmm14 // 1.0
#elif defined(OS_MAC)
	vmovss	LC03(%rip), %xmm14 // 1.0
#endif

	vmovss		%xmm0, %xmm0, %xmm13
	vucomiss	%xmm15, %xmm13 // d_00 > 0.0 ?
	jbe			1f
	vsqrtss		%xmm13, %xmm13, %xmm13
	vdivss		%xmm13, %xmm14, %xmm13
2:
	vmovss		%xmm13, 0(%r10)
	vpermilps	$ 0x00, %xmm13, %xmm13
	vinsertf128	$ 0x1, %xmm13, %ymm13, %ymm13
	vmulps		%ymm0, %ymm13, %ymm0
	vperm2f128	$ 0x00, %ymm0, %ymm0, %ymm11
	vpermilps	$ 0x55, %ymm11, %ymm13
	vmulps		%ymm0, %ymm13, %ymm12
	vsubps		%ymm12, %ymm1, %ymm1
	vpermilps	$ 0xaa, %ymm11, %ymm13
	vmulps		%ymm0, %ymm13, %ymm12
	vsubps		%ymm12, %ymm2, %ymm2
	vpermilps	$ 0xff, %ymm11, %ymm13
	vmulps		%ymm0, %ymm13, %ymm12
	vsubps		%ymm12, %ymm3, %ymm3


	vpermilps	$ 0x55, %xmm1, %xmm13
	vucomiss	%xmm15, %xmm13 // d_11 > 0.0 ?
	jbe			3f
	vsqrtss		%xmm13, %xmm13, %xmm13
	vdivss		%xmm13, %xmm14, %xmm13
4:
	vmovss		%xmm13, 4(%r10)
	vpermilps	$ 0x00, %xmm13, %xmm13
	vinsertf128	$ 0x1, %xmm13, %ymm13, %ymm13
	vmulps		%ymm1, %ymm13, %ymm1
	vperm2f128	$ 0x00, %ymm1, %ymm1, %ymm11
	vpermilps	$ 0xaa, %ymm11, %ymm13
	vmulps		%ymm1, %ymm13, %ymm12
	vsubps		%ymm12, %ymm2, %ymm2
	vpermilps	$ 0xff, %ymm11, %ymm13
	vmulps		%ymm1, %ymm13, %ymm12
	vsubps		%ymm12, %ymm3, %ymm3


	vpermilps	$ 0xaa, %xmm2, %xmm13
	vucomiss	%xmm15, %xmm13 // d_22 > 0.0 ?
	jbe			5f
	vsqrtss		%xmm13, %xmm13, %xmm13
	vdivss		%xmm13, %xmm14, %xmm13
6:
	vmovss		%xmm13, 8(%r10)
	vpermilps	$ 0x00, %xmm13, %xmm13
	vinsertf128	$ 0x1, %xmm13, %ymm13, %ymm13
	vmulps		%ymm2, %ymm13, %ymm2
	vperm2f128	$ 0x00, %ymm2, %ymm2, %ymm11
	vpermilps	$ 0xff, %ymm11, %ymm13
	vmulps		%ymm2, %ymm13, %ymm12
	vsubps		%ymm12, %ymm3, %ymm3


	vpermilps	$ 0xff, %xmm3, %xmm13
	vucomiss	%xmm15, %xmm13 // d_33 > 0.0 ?
	jbe			7f
	vsqrtss		%xmm13, %xmm13, %xmm13
	vdivss		%xmm13, %xmm14, %xmm13
8:
	vmovsd		%xmm13, 12(%r10)
	vpermilps	$ 0x00, %xmm13, %xmm13
	vinsertf128	$ 0x1, %xmm13, %ymm13, %ymm13
	vmulps		%ymm3, %ymm13, %ymm3

	jmp		0f


1:
	vxorps	%ymm13, %ymm13, %ymm13
	jmp		2b

3:
	vxorps	%ymm13, %ymm13, %ymm13
	jmp		4b

5:
	vxorps	%ymm13, %ymm13, %ymm13
	jmp		6b

7:
	vxorps	%ymm13, %ymm13, %ymm13
	jmp		8b

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_edge_potrf_8x4_lib8, .-inner_edge_potrf_8x4_lib8
#endif
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
	.macro INNER_EDGE_POTRF_8X4_VS_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_edge_potrf_8x4_vs_lib8, @function
inner_edge_potrf_8x4_vs_lib8:
#elif defined(OS_MAC)
_inner_edge_potrf_8x4_vs_lib8:
#elif defined(OS_WINDOWS)
	.def inner_edge_potrf_8x4_vs_lib8; .scl 2; .type 32; .endef
inner_edge_potrf_8x4_vs_lib8:
#endif
#endif

	vxorps	%ymm15, %ymm15, %ymm15 // 0.0
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovss	.LC03(%rip), %xmm14 // 1.0
#elif defined(OS_MAC)
	vmovss	LC03(%rip), %xmm14 // 1.0
#endif

	vmovss		%xmm0, %xmm0, %xmm13
	vucomiss	%xmm15, %xmm13 // d_00 > 0.0 ?
	jbe			1f
	vsqrtss		%xmm13, %xmm13, %xmm13
	vdivss		%xmm13, %xmm14, %xmm13
2:
	vmovss		%xmm13, 0(%r10)
	vpermilps	$ 0x00, %xmm13, %xmm13
	vinsertf128	$ 0x1, %xmm13, %ymm13, %ymm13
	vmulps		%ymm0, %ymm13, %ymm0
	cmpl		$ 2, %r11d
	jl			0f // ret
	vperm2f128	$ 0x00, %ymm0, %ymm0, %ymm11
	vpermilps	$ 0x55, %ymm11, %ymm13
	vmulps		%ymm0, %ymm13, %ymm12
	vsubps		%ymm12, %ymm1, %ymm1
	vpermilps	$ 0xaa, %ymm11, %ymm13
	vmulps		%ymm0, %ymm13, %ymm12
	vsubps		%ymm12, %ymm2, %ymm2
	vpermilps	$ 0xff, %ymm11, %ymm13
	vmulps		%ymm0, %ymm13, %ymm12
	vsubps		%ymm12, %ymm3, %ymm3


	vpermilps	$ 0x55, %xmm1, %xmm13
	vucomiss	%xmm15, %xmm13 // d_11 > 0.0 ?
	jbe			3f
	vsqrtss		%xmm13, %xmm13, %xmm13
	vdivss		%xmm13, %xmm14, %xmm13
4:
	vmovss		%xmm13, 4(%r10)
	vpermilps	$ 0x00, %xmm13, %xmm13
	vinsertf128	$ 0x1, %xmm13, %ymm13, %ymm13
	vmulps		%ymm1, %ymm13, %ymm1
	cmpl		$ 3, %r11d
	jl			0f // ret
	vperm2f128	$ 0x00, %ymm1, %ymm1, %ymm11
	vpermilps	$ 0xaa, %ymm11, %ymm13
	vmulps		%ymm1, %ymm13, %ymm12
	vsubps		%ymm12, %ymm2, %ymm2
	vpermilps	$ 0xff, %ymm11, %ymm13
	vmulps		%ymm1, %ymm13, %ymm12
	vsubps		%ymm12, %ymm3, %ymm3


	vpermilps	$ 0xaa, %xmm2, %xmm13
	vucomiss	%xmm15, %xmm13 // d_22 > 0.0 ?
	jbe			5f
	vsqrtss		%xmm13, %xmm13, %xmm13
	vdivss		%xmm13, %xmm14, %xmm13
6:
	vmovss		%xmm13, 8(%r10)
	vpermilps	$ 0x00, %xmm13, %xmm13
	vinsertf128	$ 0x1, %xmm13, %ymm13, %ymm13
	vmulps		%ymm2, %ymm13, %ymm2
	cmpl		$ 4, %r11d
	jl			0f // ret
	vperm2f128	$ 0x00, %ymm2, %ymm2, %ymm11
	vpermilps	$ 0xff, %ymm11, %ymm13
	vmulps		%ymm2, %ymm13, %ymm12
	vsubps		%ymm12, %ymm3, %ymm3


	vpermilps	$ 0xff, %xmm3, %xmm13
	vucomiss	%xmm15, %xmm13 // d_33 > 0.0 ?
	jbe			7f
	vsqrtss		%xmm13, %xmm13, %xmm13
	vdivss		%xmm13, %xmm14, %xmm13
8:
	vmovsd		%xmm13, 12(%r10)
	vpermilps	$ 0x00, %xmm13, %xmm13
	vinsertf128	$ 0x1, %xmm13, %ymm13, %ymm13
	vmulps		%ymm3, %ymm13, %ymm3

	jmp		0f


1:
	vxorps	%ymm13, %ymm13, %ymm13
	jmp		2b

3:
	vxorps	%ymm13, %ymm13, %ymm13
	jmp		4b

5:
	vxorps	%ymm13, %ymm13, %ymm13
	jmp		6b

7:
	vxorps	%ymm13, %ymm13, %ymm13
	jmp		8b

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_edge_potrf_8x4_vs_lib8, .-inner_edge_potrf_8x4_vs_lib8
#endif
#endif





// common inner routine with file scope
//
// transpose
//
// input arguments:
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_TRAN_8X4_LIB8
#else
	FUN_START(inner_tran_8x4_lib8)
#endif
	
	// transpose
	vunpcklps	%ymm1, %ymm0, %ymm5
	vunpckhps	%ymm1, %ymm0, %ymm4
	vunpcklps	%ymm3, %ymm2, %ymm7
	vunpckhps	%ymm3, %ymm2, %ymm6

	vunpcklpd	%ymm7, %ymm5, %ymm0
	vunpckhpd	%ymm7, %ymm5, %ymm1
	vunpcklpd	%ymm6, %ymm4, %ymm2
	vunpckhpd	%ymm6, %ymm4, %ymm3

	vextractf128 $ 0x1, %ymm0, %xmm4
	vextractf128 $ 0x1, %ymm1, %xmm5
	vextractf128 $ 0x1, %ymm2, %xmm6
	vextractf128 $ 0x1, %ymm3, %xmm7

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_tran_8x4_lib8)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
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
	.macro INNER_SCALE_AB_8X4_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_scale_ab_8x4_lib8, @function
inner_scale_ab_8x4_lib8:
#elif defined(OS_MAC)
_inner_scale_ab_8x4_lib8:
#elif defined(OS_WINDOWS)
	.def inner_scale_ab_8x4_lib8; .scl 2; .type 32; .endef
inner_scale_ab_8x4_lib8:
#endif
#endif
	
	// alpha
	vbroadcastss	0(%r10), %ymm15

	vmulps		%ymm0, %ymm15, %ymm0
	vmulps		%ymm1, %ymm15, %ymm1
	vmulps		%ymm2, %ymm15, %ymm2
	vmulps		%ymm3, %ymm15, %ymm3

	// beta
	vbroadcastss	0(%r11), %ymm14

	vxorps		%ymm15, %ymm15, %ymm15 // 0.0

	vucomiss	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	vmovaps		0(%r12), %ymm15
	vmulps		%ymm15, %ymm14, %ymm15
	vaddps		%ymm0, %ymm15, %ymm0
	vmovaps		32(%r12), %ymm15
	vmulps		%ymm15, %ymm14, %ymm15
	vaddps		%ymm1, %ymm15, %ymm1
	vmovaps		64(%r12), %ymm15
	vmulps		%ymm15, %ymm14, %ymm15
	vaddps		%ymm2, %ymm15, %ymm2
	vmovaps		96(%r12), %ymm15
	vmulps		%ymm15, %ymm14, %ymm15
	vaddps		%ymm3, %ymm15, %ymm3

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_scale_ab_8x4_lib8, .-inner_scale_ab_8x4_lib8
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
	.macro INNER_TRAN_SCALE_AB_4X8_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_tran_scale_ab_4x8_lib8, @function
inner_tran_scale_ab_4x8_lib8:
#elif defined(OS_MAC)
_inner_tran_scale_ab_4x8_lib8:
#elif defined(OS_WINDOWS)
	.def inner_tran_scale_ab_4x8_lib8; .scl 2; .type 32; .endef
inner_tran_scale_ab_4x8_lib8:
#endif
#endif
	
	// alpha
	vbroadcastss	0(%r10), %ymm15

	vmulps		%ymm0, %ymm15, %ymm4
	vmulps		%ymm1, %ymm15, %ymm5
	vmulps		%ymm2, %ymm15, %ymm6
	vmulps		%ymm3, %ymm15, %ymm7

	// transpose
	vblendps	$ 0xaa, %ymm5, %ymm4, %ymm0
	vblendps	$ 0xaa, %ymm5, %ymm5, %ymm1
	vblendps	$ 0xaa, %ymm6, %ymm7, %ymm2
	vblendps	$ 0xaa, %ymm7, %ymm6, %ymm3

	vunpcklps	%ymm1, %ymm0, %ymm4
	vunpckhps	%ymm1, %ymm0, %ymm5
	vunpcklps	%ymm3, %ymm2, %ymm6
	vunpckhps	%ymm3, %ymm2, %ymm7

	vunpcklpd	%ymm5, %ymm7, %ymm2
	vunpckhpd	%ymm5, %ymm7, %ymm3
	vunpcklpd	%ymm6, %ymm4, %ymm0
	vunpckhpd	%ymm6, %ymm4, %ymm1

	vextractf128 $ 0x1, %ymm0, %xmm4
	vextractf128 $ 0x1, %ymm1, %xmm5
	vextractf128 $ 0x1, %ymm2, %xmm6
	vextractf128 $ 0x1, %ymm3, %xmm7

	// beta
	vbroadcastss	0(%r11), %ymm14

	vxorps		%ymm15, %ymm15, %ymm15 // 0.0

	vucomiss	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	vmovaps		0(%r12), %xmm15
	vmulps		%xmm15, %xmm14, %xmm15
	vaddps		%xmm15, %xmm0, %xmm0
	vmovaps		32(%r12), %xmm15
	vmulps		%xmm15, %xmm14, %xmm15
	vaddps		%xmm15, %xmm1, %xmm1
	vmovaps		64(%r12), %xmm15
	vmulps		%xmm15, %xmm14, %xmm15
	vaddps		%xmm15, %xmm2, %xmm2
	vmovaps		96(%r12), %xmm15
	vmulps		%xmm15, %xmm14, %xmm15
	vaddps		%xmm15, %xmm3, %xmm3
	vmovaps		128(%r12), %xmm15
	vmulps		%xmm15, %xmm14, %xmm15
	vaddps		%xmm15, %xmm4, %xmm4
	vmovaps		160(%r12), %xmm15
	vmulps		%xmm15, %xmm14, %xmm15
	vaddps		%xmm15, %xmm5, %xmm5
	vmovaps		192(%r12), %xmm15
	vmulps		%xmm15, %xmm14, %xmm15
	vaddps		%xmm15, %xmm6, %xmm6
	vmovaps		224(%r12), %xmm15
	vmulps		%xmm15, %xmm14, %xmm15
	vaddps		%xmm15, %xmm7, %xmm7

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_tran_scale_ab_4x8_lib8, .-inner_tran_scale_ab_4x8_lib8
#endif
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
// r15  <- n0 // col index: start from (inc)
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
	.macro INNER_SCALE_AB_8X4_GEN_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_scale_ab_8x4_gen_lib8, @function
inner_scale_ab_8x4_gen_lib8:
#elif defined(OS_MAC)
_inner_scale_ab_8x4_gen_lib8:
#elif defined(OS_WINDOWS)
	.def inner_scale_ab_8x4_gen_lib8; .scl 2; .type 32; .endef
inner_scale_ab_8x4_gen_lib8:
#endif
#endif
	
	// alpha
	vbroadcastss	0(%r10), %ymm15

	vmulps		%ymm0, %ymm15, %ymm0
	vmulps		%ymm1, %ymm15, %ymm1
	vmulps		%ymm2, %ymm15, %ymm2
	vmulps		%ymm3, %ymm15, %ymm3

	// beta
	vbroadcastss	0(%r11), %ymm15

	vxorps		%ymm14, %ymm14, %ymm14 // 0.0

	vucomiss	%xmm15, %xmm14 // beta==0.0 ?
	je			3f // end

	cmpl	$ 0, %r12d
	jg		0f

	// offset==0

	vmovaps		0(%r13), %ymm12
	vmulps		%ymm12, %ymm15, %ymm12
	vaddps		%ymm0, %ymm12, %ymm0
	vmovaps		32(%r13), %ymm12
	vmulps		%ymm12, %ymm15, %ymm12
	vaddps		%ymm1, %ymm12, %ymm1
	vmovaps		64(%r13), %ymm12
	vmulps		%ymm12, %ymm15, %ymm12
	vaddps		%ymm2, %ymm12, %ymm2
	vmovaps		96(%r13), %ymm12
	vmulps		%ymm12, %ymm15, %ymm12
	vaddps		%ymm3, %ymm12, %ymm3

	jmp		7f

0:

	// offset > 0
	// 1 2 3 4 5 6 7
	
	movq	%r13, %r15 // C0
	addq	%r14, %r15 // C1 <- C0 + 4*sdc*sizeof(double)

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

#if defined(OS_LINUX)
	.size	inner_scale_ab_8x4_gen_lib8, .-inner_scale_ab_8x4_gen_lib8
#endif
#endif





// common inner routine with file scope
//
// blend scale for generic alpha and beta
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
// r15  <- n0 // col index: start from (inc)
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
	.macro INNER_TRAN_SCALE_AB_4X8_GEN_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_tran_scale_ab_4x8_gen_lib8, @function
inner_tran_scale_ab_4x8_gen_lib8:
#elif defined(OS_MAC)
_inner_tran_scale_ab_4x8_gen_lib8:
#elif defined(OS_WINDOWS)
	.def inner_tran_scale_ab_4x8_gen_lib8; .scl 2; .type 32; .endef
inner_tran_scale_ab_4x8_gen_lib8:
#endif
#endif
	
	// alpha
	vbroadcastss	0(%r10), %ymm15

	vmulps		%ymm0, %ymm15, %ymm4
	vmulps		%ymm1, %ymm15, %ymm5
	vmulps		%ymm2, %ymm15, %ymm6
	vmulps		%ymm3, %ymm15, %ymm7

	// transpose
	vblendps	$ 0xaa, %ymm5, %ymm4, %ymm0
	vblendps	$ 0xaa, %ymm5, %ymm5, %ymm1
	vblendps	$ 0xaa, %ymm6, %ymm7, %ymm2
	vblendps	$ 0xaa, %ymm7, %ymm6, %ymm3

	vunpcklps	%ymm1, %ymm0, %ymm4
	vunpckhps	%ymm1, %ymm0, %ymm5
	vunpcklps	%ymm3, %ymm2, %ymm6
	vunpckhps	%ymm3, %ymm2, %ymm7

	vunpcklpd	%ymm5, %ymm7, %ymm2
	vunpckhpd	%ymm5, %ymm7, %ymm3
	vunpcklpd	%ymm6, %ymm4, %ymm0
	vunpckhpd	%ymm6, %ymm4, %ymm1

	vextractf128 $ 0x1, %ymm0, %xmm4
	vextractf128 $ 0x1, %ymm1, %xmm5
	vextractf128 $ 0x1, %ymm2, %xmm6
	vextractf128 $ 0x1, %ymm3, %xmm7

	// beta
	vbroadcastss	0(%r11), %ymm15

	vxorps		%ymm14, %ymm14, %ymm14 // 0.0

	vucomiss	%xmm15, %xmm14 // beta==0.0 ?
	je			3f // end

	cmpl	$ 0, %r12d
	jg		0f

	// offset==0

	vmovaps		0(%r12), %xmm15
	vmulps		%xmm15, %xmm14, %xmm15
	vaddps		%xmm15, %xmm0, %xmm0
	vmovaps		32(%r12), %xmm15
	vmulps		%xmm15, %xmm14, %xmm15
	vaddps		%xmm15, %xmm1, %xmm1
	vmovaps		64(%r12), %xmm15
	vmulps		%xmm15, %xmm14, %xmm15
	vaddps		%xmm15, %xmm2, %xmm2
	vmovaps		96(%r12), %xmm15
	vmulps		%xmm15, %xmm14, %xmm15
	vaddps		%xmm15, %xmm3, %xmm3
	vmovaps		128(%r12), %xmm15
	vmulps		%xmm15, %xmm14, %xmm15
	vaddps		%xmm15, %xmm4, %xmm4
	vmovaps		160(%r12), %xmm15
	vmulps		%xmm15, %xmm14, %xmm15
	vaddps		%xmm15, %xmm5, %xmm5
	vmovaps		192(%r12), %xmm15
	vmulps		%xmm15, %xmm14, %xmm15
	vaddps		%xmm15, %xmm6, %xmm6
	vmovaps		224(%r12), %xmm15
	vmulps		%xmm15, %xmm14, %xmm15
	vaddps		%xmm15, %xmm7, %xmm7

	jmp		7f

0:

	// offset > 0
	// 1 2 3 4 5 6 7
	
	movq	%r13, %r15 // C0
	addq	%r14, %r15 // C1 <- C0 + 4*sdc*sizeof(double)

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

#if defined(OS_LINUX)
	.size	inner_tran_scale_ab_4x8_gen_lib8, .-inner_tran_scale_ab_4x8_gen_lib8
#endif
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
	.macro INNER_SCALE_A0_8X4_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_scale_a0_8x4_lib8, @function
inner_scale_a0_8x4_lib8:
#elif defined(OS_MAC)
_inner_scale_a0_8x4_lib8:
#elif defined(OS_WINDOWS)
	.def inner_scale_a0_8x4_lib8; .scl 2; .type 32; .endef
inner_scale_a0_8x4_lib8:
#endif
#endif
	
	// alpha
	vbroadcastss	0(%r10), %ymm15

	vmulps		%ymm0, %ymm15, %ymm0
	vmulps		%ymm1, %ymm15, %ymm1
	vmulps		%ymm2, %ymm15, %ymm2
	vmulps		%ymm3, %ymm15, %ymm3

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_scale_a0_8x4_lib8, .-inner_scale_a0_8x4_lib8
#endif
#endif





// common inner routine with file scope
//
// blend scale for generic alpha and beta
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
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
	.macro INNER_BLEND_SCALE_AB_8X4_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_blend_scale_ab_8x4_lib8, @function
inner_blend_scale_ab_8x4_lib8:
#elif defined(OS_MAC)
_inner_blend_scale_ab_8x4_lib8:
#elif defined(OS_WINDOWS)
	.def inner_blend_scale_ab_8x4_lib8; .scl 2; .type 32; .endef
inner_blend_scale_ab_8x4_lib8:
#endif
#endif
	
	vblendps	$ 0xaa, %ymm1, %ymm0, %ymm8 // 1010 1010
	vblendps	$ 0x55, %ymm1, %ymm0, %ymm9 // 0101 0101
	vblendps	$ 0xaa, %ymm3, %ymm2, %ymm10
	vblendps	$ 0x55, %ymm3, %ymm2, %ymm11

	vblendps	$ 0xcc, %ymm11, %ymm8, %ymm0 // 1100 1100
	vblendps	$ 0x33, %ymm11, %ymm8, %ymm2 // 0011 0011
	vblendps	$ 0xcc, %ymm10, %ymm9, %ymm1
	vblendps	$ 0x33, %ymm10, %ymm9, %ymm3

	// alpha
	vbroadcastss	0(%r10), %ymm15

	vmulps		%ymm0, %ymm15, %ymm0
	vmulps		%ymm1, %ymm15, %ymm1
	vmulps		%ymm2, %ymm15, %ymm2
	vmulps		%ymm3, %ymm15, %ymm3

	// beta
	vbroadcastss	0(%r11), %ymm14

	vxorps		%ymm15, %ymm15, %ymm15 // 0.0

	vucomiss	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	vmovaps		0(%r12), %ymm15
	vmulps		%ymm15, %ymm14, %ymm15
	vaddps		%ymm0, %ymm15, %ymm0
	vmovaps		32(%r12), %ymm15
	vmulps		%ymm15, %ymm14, %ymm15
	vaddps		%ymm1, %ymm15, %ymm1
	vmovaps		64(%r12), %ymm15
	vmulps		%ymm15, %ymm14, %ymm15
	vaddps		%ymm2, %ymm15, %ymm2
	vmovaps		96(%r12), %ymm15
	vmulps		%ymm15, %ymm14, %ymm15
	vaddps		%ymm3, %ymm15, %ymm3

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_blend_scale_ab_8x4_lib8, .-inner_blend_scale_ab_8x4_lib8
#endif
#endif





// common inner routine with file scope
//
// blend scale for generic alpha and beta
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
// r15  <- n0 // col index: start from (inc)
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
	.macro INNER_BLEND_SCALE_AB_8X4_GEN_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_blend_scale_ab_8x4_gen_lib8, @function
inner_blend_scale_ab_8x4_gen_lib8:
#elif defined(OS_MAC)
_inner_blend_scale_ab_8x4_gen_lib8:
#elif defined(OS_WINDOWS)
	.def inner_blend_scale_ab_8x4_gen_lib8; .scl 2; .type 32; .endef
inner_blend_scale_ab_8x4_gen_lib8:
#endif
#endif
	
	vblendps	$ 0xaa, %ymm1, %ymm0, %ymm8 // 1010 1010
	vblendps	$ 0x55, %ymm1, %ymm0, %ymm9 // 0101 0101
	vblendps	$ 0xaa, %ymm3, %ymm2, %ymm10
	vblendps	$ 0x55, %ymm3, %ymm2, %ymm11

	vblendps	$ 0xcc, %ymm11, %ymm8, %ymm0 // 1100 1100
	vblendps	$ 0x33, %ymm11, %ymm8, %ymm2 // 0011 0011
	vblendps	$ 0xcc, %ymm10, %ymm9, %ymm1
	vblendps	$ 0x33, %ymm10, %ymm9, %ymm3

	// alpha
	vbroadcastss	0(%r10), %ymm15

	vmulps		%ymm0, %ymm15, %ymm0
	vmulps		%ymm1, %ymm15, %ymm1
	vmulps		%ymm2, %ymm15, %ymm2
	vmulps		%ymm3, %ymm15, %ymm3

	// beta
	vbroadcastss	0(%r11), %ymm15

	vxorps		%ymm14, %ymm14, %ymm14 // 0.0

	vucomiss	%xmm15, %xmm14 // beta==0.0 ?
	je			3f // end

	cmpl	$ 0, %r12d
	jg		0f

	// offset==0

	vmovaps		0(%r13), %ymm12
	vmulps		%ymm12, %ymm15, %ymm12
	vaddps		%ymm0, %ymm12, %ymm0
	vmovaps		32(%r13), %ymm12
	vmulps		%ymm12, %ymm15, %ymm12
	vaddps		%ymm1, %ymm12, %ymm1
	vmovaps		64(%r13), %ymm12
	vmulps		%ymm12, %ymm15, %ymm12
	vaddps		%ymm2, %ymm12, %ymm2
	vmovaps		96(%r13), %ymm12
	vmulps		%ymm12, %ymm15, %ymm12
	vaddps		%ymm3, %ymm12, %ymm3

	jmp		7f

0:

	// offset > 0
	// 1 2 3 4 5 6 7
	
	movq	%r13, %r15 // C0
	addq	%r14, %r15 // C1 <- C0 + 4*sdc*sizeof(double)

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

#if defined(OS_LINUX)
	.size	inner_blend_scale_ab_8x4_gen_lib8, .-inner_blend_scale_ab_8x4_gen_lib8
#endif
#endif





// common inner routine with file scope
//
// blend scale for generic alpha=1.0 and beta=1.0
//
// input arguments:
// r10   <- C
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
	.macro INNER_BLEND_SCALE_11_8X4_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_blend_scale_11_8x4_lib8, @function
inner_blend_scale_11_8x4_lib8:
#elif defined(OS_MAC)
_inner_blend_scale_11_8x4_lib8:
#elif defined(OS_WINDOWS)
	.def inner_blend_scale_11_8x4_lib8; .scl 2; .type 32; .endef
inner_blend_scale_11_8x4_lib8:
#endif
#endif
	
	vblendps	$ 0xaa, %ymm1, %ymm0, %ymm8 // 1010 1010
	vblendps	$ 0x55, %ymm1, %ymm0, %ymm9 // 0101 0101
	vblendps	$ 0xaa, %ymm3, %ymm2, %ymm10
	vblendps	$ 0x55, %ymm3, %ymm2, %ymm11

	vblendps	$ 0xcc, %ymm11, %ymm8, %ymm0 // 1100 1100
	vblendps	$ 0x33, %ymm11, %ymm8, %ymm2 // 0011 0011
	vblendps	$ 0xcc, %ymm10, %ymm9, %ymm1
	vblendps	$ 0x33, %ymm10, %ymm9, %ymm3

	vmovaps		0(%r10), %ymm15
	vaddps		%ymm0, %ymm15, %ymm0
	vmovaps		32(%r10), %ymm15
	vaddps		%ymm1, %ymm15, %ymm1
	vmovaps		64(%r10), %ymm15
	vaddps		%ymm2, %ymm15, %ymm2
	vmovaps		96(%r10), %ymm15
	vaddps		%ymm3, %ymm15, %ymm3

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_blend_scale_11_8x4_lib8, .-inner_blend_scale_11_8x4_lib8
#endif
#endif





// common inner routine with file scope
//
// blend scale for generic alpha=1.0 and beta=1.0
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
	.macro INNER_BLEND_SCALE_11_8X4_GEN_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_blend_scale_11_8x4_gen_lib8, @function
inner_blend_scale_11_8x4_gen_lib8:
#elif defined(OS_MAC)
_inner_blend_scale_11_8x4_gen_lib8:
#elif defined(OS_WINDOWS)
	.def inner_blend_scale_11_8x4_gen_lib8; .scl 2; .type 32; .endef
inner_blend_scale_11_8x4_gen_lib8:
#endif
#endif
	
	vblendps	$ 0xaa, %ymm1, %ymm0, %ymm8 // 1010 1010
	vblendps	$ 0x55, %ymm1, %ymm0, %ymm9 // 0101 0101
	vblendps	$ 0xaa, %ymm3, %ymm2, %ymm10
	vblendps	$ 0x55, %ymm3, %ymm2, %ymm11

	vblendps	$ 0xcc, %ymm11, %ymm8, %ymm0 // 1100 1100
	vblendps	$ 0x33, %ymm11, %ymm8, %ymm2 // 0011 0011
	vblendps	$ 0xcc, %ymm10, %ymm9, %ymm1
	vblendps	$ 0x33, %ymm10, %ymm9, %ymm3

	// offset==0

	vmovaps		0(%r11), %ymm12
	vaddps		%ymm0, %ymm12, %ymm0
	vmovaps		32(%r11), %ymm12
	vaddps		%ymm1, %ymm12, %ymm1
	vmovaps		64(%r11), %ymm12
	vaddps		%ymm2, %ymm12, %ymm2
	vmovaps		96(%r11), %ymm12
	vaddps		%ymm3, %ymm12, %ymm3

	jmp		7f

0:

	// offset > 0
	// 1 2 3 4 5 6 7
	
	movq	%r11, %r15 // C0
	addq	%r12, %r15 // C1 <- C0 + 4*sdc*sizeof(double)

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

#if defined(OS_LINUX)
	.size	inner_blend_scale_11_8x4_gen_lib8, .-inner_blend_scale_11_8x4_gen_lib8
#endif
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []
//
// output arguments:
// r10  <- D
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_8X4_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_store_8x4_lib8, @function
inner_store_8x4_lib8:
#elif defined(OS_MAC)
_inner_store_8x4_lib8:
#elif defined(OS_WINDOWS)
	.def inner_store_8x4_lib8; .scl 2; .type 32; .endef
inner_store_8x4_lib8:
#endif
#endif
	
	vmovaps 	%ymm0,  0(%r10)
	vmovaps 	%ymm1, 32(%r10)
	vmovaps 	%ymm2, 64(%r10)
	vmovaps 	%ymm3, 96(%r10)
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_store_8x4_lib8, .-inner_store_8x4_lib8
#endif
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []
//
// output arguments:
// r10  <- D
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X8_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_store_4x8_lib8, @function
inner_store_4x8_lib8:
#elif defined(OS_MAC)
_inner_store_4x8_lib8:
#elif defined(OS_WINDOWS)
	.def inner_store_4x8_lib8; .scl 2; .type 32; .endef
inner_store_4x8_lib8:
#endif
#endif
	
	vmovaps 	%xmm0,  0(%r10)
	vmovaps 	%xmm1, 32(%r10)
	vmovaps 	%xmm2, 64(%r10)
	vmovaps 	%xmm3, 96(%r10)
	vmovaps 	%xmm4, 128(%r10)
	vmovaps 	%xmm5, 160(%r10)
	vmovaps 	%xmm6, 192(%r10)
	vmovaps 	%xmm7, 224(%r10)
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_store_4x8_lib8, .-inner_store_4x8_lib8
#endif
#endif





// common inner routine with file scope
//
// store n vs
//
// input arguments:
// r10  <- D
// r11  <- km
// r12  <- kn
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []
//
// output arguments:
// r10  <- D
// r11  <- km
// r12  <- kn
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_8X4_VS_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_store_8x4_vs_lib8, @function
inner_store_8x4_vs_lib8:
#elif defined(OS_MAC)
_inner_store_8x4_vs_lib8:
#elif defined(OS_WINDOWS)
	.def inner_store_8x4_vs_lib8; .scl 2; .type 32; .endef
inner_store_8x4_vs_lib8:
#endif
#endif
	
	// compute mask for rows
	vcvtsi2ss	%r11d, %xmm14, %xmm14
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %ymm12
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %ymm12
#endif
	vshufps		$ 0x00, %xmm14, %xmm14, %xmm14
	vinsertf128	$ 0x1, %xmm14, %ymm14, %ymm14
	vsubps		%ymm14, %ymm12, %ymm14

	// offset==0
	vmaskmovps	%ymm0, %ymm14,  0(%r10)
	cmpl		$ 2, %r12d
	jl			0f // end
	vmaskmovps	%ymm1, %ymm14, 32(%r10)
	cmpl		$ 3, %r12d
	jl			0f // end
	vmaskmovps	%ymm2, %ymm14, 64(%r10)
	je			0f // end
	vmaskmovps	%ymm3, %ymm14, 96(%r10)

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_store_8x4_vs_lib8, .-inner_store_8x4_vs_lib8
#endif
#endif





// common inner routine with file scope
//
// store n generalized
//
// input arguments:
// r10  <- D
// r11  <- km
// r12  <- kn
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []
//
// output arguments:
// r10  <- D
// r11  <- km
// r12  <- kn
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X8_VS_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_store_4x8_vs_lib8, @function
inner_store_4x8_vs_lib8:
#elif defined(OS_MAC)
_inner_store_4x8_vs_lib8:
#elif defined(OS_WINDOWS)
	.def inner_store_4x8_vs_lib8; .scl 2; .type 32; .endef
inner_store_4x8_vs_lib8:
#endif
#endif
	
	// compute mask for rows
	vcvtsi2ss	%r11d, %xmm14, %xmm14
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %xmm12
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %xmm12
#endif
	vshufps		$ 0x00, %xmm14, %xmm14, %xmm14
	vinsertf128	$ 0x1, %xmm14, %ymm14, %ymm14
	vsubps		%xmm14, %xmm12, %xmm14

	// offset==0
	vmaskmovps	%xmm0, %xmm14,  0(%r10)
	cmpl		$ 2, %r12d
	jl			0f // end
	vmaskmovps	%xmm1, %xmm14, 32(%r10)
	cmpl		$ 3, %r12d
	jl			0f // end
	vmaskmovps	%xmm2, %xmm14, 64(%r10)
	cmpl		$ 4, %r12d
	jl			0f // end
	vmaskmovps	%xmm3, %xmm14, 96(%r10)
	cmpl		$ 5, %r12d
	jl			0f // end
	vmaskmovps	%xmm4, %xmm14, 128(%r10)
	cmpl		$ 6, %r12d
	jl			0f // end
	vmaskmovps	%xmm5, %xmm14, 160(%r10)
	cmpl		$ 7, %r12d
	jl			0f // end
	vmaskmovps	%xmm6, %xmm14, 192(%r10)
	je			0f // end
	vmaskmovps	%xmm7, %xmm14, 224(%r10)
	//

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_store_4x8_vs_lib8, .-inner_store_4x8_vs_lib8
#endif
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
	.macro INNER_STORE_8X4_GEN_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_store_8x4_gen_lib8, @function
inner_store_8x4_gen_lib8:
#elif defined(OS_MAC)
_inner_store_8x4_gen_lib8:
#elif defined(OS_WINDOWS)
	.def inner_store_8x4_gen_lib8; .scl 2; .type 32; .endef
inner_store_8x4_gen_lib8:
#endif
#endif
	
	// compute mask for rows
	vcvtsi2ss	%r13d, %xmm14, %xmm14
	vcvtsi2ss	%r14d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %ymm12
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %ymm12
#endif
	vshufps		$ 0x00, %xmm14, %xmm14, %xmm14
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm14, %ymm14, %ymm14
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%ymm12, %ymm14, %ymm14
	vsubps		%ymm15, %ymm12, %ymm15
	vandps		%ymm14, %ymm15, %ymm15

	// shift D and sol for cols
	cmpl	$ 0, %r15d
	jle		0f

	vmovaps		%ymm1, %ymm0
	vmovaps		%ymm2, %ymm1
	vmovaps		%ymm3, %ymm2
	addq		$ 32, %r11

	cmpl	$ 1, %r15d
	jle		0f

	vmovaps		%ymm1, %ymm0
	vmovaps		%ymm2, %ymm1
	addq		$ 32, %r11

	cmpl	$ 2, %r15d
	jle		0f

	vmovaps		%ymm1, %ymm0
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
	vmaskmovps	%ymm0, %ymm15,  0(%r11)
	cmpl		$ 2, %r15d
	jl			7f // end
	vmaskmovps	%ymm1, %ymm15, 32(%r11)
	cmpl		$ 3, %r15d
	jl			7f // end
	vmaskmovps	%ymm2, %ymm15, 64(%r11)
	je			7f // end
	vmaskmovps	%ymm3, %ymm15, 96(%r11)
	//
	jmp		7f

0:
	// offset > 0
	// 1 2 3 4 5 6 7
	
	movq	%r11, %rbx // D0
	addq	%r12, %rbx // D1 <- D0 + 4*sdd*sizeof(double)

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

#if defined(OS_LINUX)
	.size	inner_store_8x4_gen_lib8, .-inner_store_8x4_gen_lib8
#endif
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
	.macro INNER_STORE_4X8_GEN_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_store_4x8_gen_lib8, @function
inner_store_4x8_gen_lib8:
#elif defined(OS_MAC)
_inner_store_4x8_gen_lib8:
#elif defined(OS_WINDOWS)
	.def inner_store_4x8_gen_lib8; .scl 2; .type 32; .endef
inner_store_4x8_gen_lib8:
#endif
#endif
	
	// compute mask for rows
	vcvtsi2ss	%r13d, %xmm14, %xmm14
	vcvtsi2ss	%r14d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %xmm12
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %xmm12
#endif
	vshufps		$ 0x00, %xmm14, %xmm14, %xmm14
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm14, %ymm14, %ymm14
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%xmm12, %xmm14, %xmm14
	vsubps		%xmm15, %xmm12, %xmm15
	vandps		%xmm14, %xmm15, %xmm15

	// shift D and sol for cols
	cmpl	$ 0, %r15d
	jle		0f

	vmovaps		%xmm1, %xmm0
	vmovaps		%xmm2, %xmm1
	vmovaps		%xmm3, %xmm2
	vmovaps		%xmm4, %xmm3
	vmovaps		%xmm5, %xmm4
	vmovaps		%xmm6, %xmm5
	vmovaps		%xmm7, %xmm6
	addq		$ 32, %r11

	cmpl	$ 1, %r15d
	jle		0f

	vmovaps		%xmm1, %xmm0
	vmovaps		%xmm2, %xmm1
	vmovaps		%xmm3, %xmm2
	vmovaps		%xmm4, %xmm3
	vmovaps		%xmm5, %xmm4
	vmovaps		%xmm6, %xmm5
	addq		$ 32, %r11

	cmpl	$ 2, %r15d
	jle		0f

	vmovaps		%xmm1, %xmm0
	vmovaps		%xmm2, %xmm1
	vmovaps		%xmm3, %xmm2
	vmovaps		%xmm4, %xmm3
	vmovaps		%xmm5, %xmm4
	addq		$ 32, %r11

	cmpl	$ 3, %r15d
	jle		0f

	vmovaps		%xmm1, %xmm0
	vmovaps		%xmm2, %xmm1
	vmovaps		%xmm3, %xmm2
	vmovaps		%xmm4, %xmm3
	addq		$ 32, %r11

	cmpl	$ 4, %r15d
	jle		0f

	vmovaps		%xmm1, %xmm0
	vmovaps		%xmm2, %xmm1
	vmovaps		%xmm3, %xmm2
	addq		$ 32, %r11

	cmpl	$ 5, %r15d
	jle		0f

	vmovaps		%xmm1, %xmm0
	vmovaps		%xmm2, %xmm1
	addq		$ 32, %r11

	cmpl	$ 6, %r15d
	jle		0f

	vmovaps		%xmm1, %xmm0
	addq		$ 32, %r11

0:

	// compute number of cols
	cmpl	$ 8, %eax
	jle		0f
	movl	$ 8, %eax
0:
	subl	%r15d, %eax
	movl	%eax, %r15d

	cmpl	$ 0, %r10d
	jg		0f

	// offset==0
	vmaskmovps	%xmm0, %xmm15,  0(%r11)
	cmpl		$ 2, %r15d
	jl			7f // end
	vmaskmovps	%xmm1, %xmm15, 32(%r11)
	cmpl		$ 3, %r15d
	jl			7f // end
	vmaskmovps	%xmm2, %xmm15, 64(%r11)
	cmpl		$ 4, %r15d
	jl			7f // end
	vmaskmovps	%xmm3, %xmm15, 96(%r11)
	cmpl		$ 5, %r15d
	jl			7f // end
	vmaskmovps	%xmm4, %xmm15, 128(%r11)
	cmpl		$ 6, %r15d
	jl			7f // end
	vmaskmovps	%xmm5, %xmm15, 160(%r11)
	cmpl		$ 7, %r15d
	jl			7f // end
	vmaskmovps	%xmm6, %xmm15, 192(%r11)
	je			7f // end
	vmaskmovps	%xmm7, %xmm15, 224(%r11)
	//
	jmp		7f

0:
	// offset > 0
	// 1 2 3 4 5 6 7
	
	movq	%r11, %rbx // D0
	addq	%r12, %rbx // D1 <- D0 + 4*sdd*sizeof(double)

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

#if defined(OS_LINUX)
	.size	inner_store_8x4_gen_lib8, .-inner_store_8x4_gen_lib8
#endif
#endif





// common inner routine with file scope
//
// store lower
//
// input arguments:
// r10  <- D
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []
//
// output arguments:
// r10  <- D
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_8X4_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_store_l_8x4_lib8, @function
inner_store_l_8x4_lib8:
#elif defined(OS_MAC)
_inner_store_l_8x4_lib8:
#elif defined(OS_WINDOWS)
	.def inner_store_l_8x4_lib8; .scl 2; .type 32; .endef
inner_store_l_8x4_lib8:
#endif
#endif
	
	vmovaps 	32(%r10), %ymm12
	vmovaps 	64(%r10), %ymm13
	vmovaps 	96(%r10), %ymm14

	vblendps	$ 0x1, %ymm12, %ymm1, %ymm1
	vblendps	$ 0x3, %ymm13, %ymm2, %ymm2
	vblendps	$ 0x7, %ymm14, %ymm3, %ymm3

	vmovaps 	%ymm0,  0(%r10)
	vmovaps 	%ymm1, 32(%r10)
	vmovaps 	%ymm2, 64(%r10)
	vmovaps 	%ymm3, 96(%r10)
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_store_l_8x4_lib8, .-inner_store_l_8x4_lib8
#endif
#endif





// common inner routine with file scope
//
// store n vs
//
// input arguments:
// r10  <- D
// r11  <- km
// r12  <- kn
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []
//
// output arguments:
// r10  <- D
// r11  <- km
// r12  <- kn
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_8X4_VS_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_store_l_8x4_vs_lib8, @function
inner_store_l_8x4_vs_lib8:
#elif defined(OS_MAC)
_inner_store_l_8x4_vs_lib8:
#elif defined(OS_WINDOWS)
	.def inner_store_l_8x4_vs_lib8; .scl 2; .type 32; .endef
inner_store_l_8x4_vs_lib8:
#endif
#endif
	
	// compute mask for rows
	vcvtsi2ss	%r11d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %ymm12
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %ymm12
#endif
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%ymm15, %ymm12, %ymm15

	vmaskmovps	%ymm0, %ymm15,  0(%r10)
	cmpl		$ 2, %r12d
	jl			0f // end
	vmovaps 	32(%r10), %ymm12
	vblendps	$ 0x1, %ymm12, %ymm1, %ymm1
	vmaskmovps	%ymm1, %ymm15, 32(%r10)
	cmpl		$ 3, %r12d
	jl			0f // end
	vmovaps 	64(%r10), %ymm12
	vblendps	$ 0x3, %ymm12, %ymm2, %ymm2
	vmaskmovps	%ymm2, %ymm15, 64(%r10)
	je			0f // end
	vmovaps 	96(%r10), %ymm12
	vblendps	$ 0x7, %ymm12, %ymm3, %ymm3
	vmaskmovps	%ymm3, %ymm15, 96(%r10)
	//
0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_store_l_8x4_vs_lib8, .-inner_store_l_8x4_vs_lib8
#endif
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
	.macro INNER_STORE_L_8X4_GEN_LIB8
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_store_l_8x4_gen_lib8, @function
inner_store_l_8x4_gen_lib8:
#elif defined(OS_MAC)
_inner_store_l_8x4_gen_lib8:
#elif defined(OS_WINDOWS)
	.def inner_store_l_8x4_gen_lib8; .scl 2; .type 32; .endef
inner_store_l_8x4_gen_lib8:
#endif
#endif
	
	// compute mask for rows
	vcvtsi2ss	%r13d, %xmm14, %xmm14
	vcvtsi2ss	%r14d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %ymm12
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %ymm12
#endif
	vshufps		$ 0x00, %xmm14, %xmm14, %xmm14
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm14, %ymm14, %ymm14
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%ymm12, %ymm14, %ymm14
	vsubps		%ymm15, %ymm12, %ymm15
	vandps		%ymm14, %ymm15, %ymm15

	// shift D and sol for cols
	cmpl	$ 0, %r15d
	jle		0f

	vmovaps		%ymm1, %ymm0
	vmovaps		%ymm2, %ymm1
	vmovaps		%ymm3, %ymm2
	addq		$ 32, %r11

	cmpl	$ 1, %r15d
	jle		0f

	vmovaps		%ymm1, %ymm0
	vmovaps		%ymm2, %ymm1
	addq		$ 32, %r11

	cmpl	$ 2, %r15d
	jle		0f

	vmovaps		%ymm1, %ymm0
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
	vmaskmovps	%ymm0, %ymm15,  0(%r11)
	cmpl		$ 2, %r15d
	jl			7f // end
	vmovaps 	32(%r11), %ymm12
	vblendps	$ 0x1, %ymm12, %ymm1, %ymm1
	vmaskmovps	%ymm1, %ymm15, 32(%r11)
	cmpl		$ 3, %r15d
	jl			7f // end
	vmovaps 	64(%r11), %ymm12
	vblendps	$ 0x3, %ymm12, %ymm2, %ymm2
	vmaskmovps	%ymm2, %ymm15, 64(%r11)
	je			7f // end
	vmovaps 	96(%r11), %ymm12
	vblendps	$ 0x7, %ymm12, %ymm3, %ymm3
	vmaskmovps	%ymm3, %ymm15, 96(%r11)
	//
	jmp		7f

0:
	// offset > 0
	// 1 2 3 4 5 6 7
	
	movq	%r11, %rbx // D0
	addq	%r12, %rbx // D1 <- D0 + 4*sdd*sizeof(double)

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

#if defined(OS_LINUX)
	.size	inner_store_l_8x4_gen_lib8, .-inner_store_l_8x4_gen_lib8
#endif
#endif





//                               rdi    rsi           rdx       rcx       r8           r9        rsp+8
// void kernel_sgemm_nt_8x4_lib8(int k, float *alpha, float *A, float *B, float *beta, float *C, float *D);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgemm_nt_8x4_lib8
	.type kernel_sgemm_nt_8x4_lib8, @function
kernel_sgemm_nt_8x4_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgemm_nt_8x4_lib8
_kernel_sgemm_nt_8x4_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgemm_nt_8x4_lib8
	.def kernel_sgemm_nt_8x4_lib8; .scl 2; .type 32; .endef
kernel_sgemm_nt_8x4_lib8:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_gemm_add_nt_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_gemm_add_nt_8x4_lib8
#endif
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12   // C

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_AB_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_blend_scale_ab_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_blend_scale_ab_8x4_lib8
#endif
#endif


	// store n

	movq	ARG7, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_store_8x4_lib8
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgemm_nt_8x4_lib8, .-kernel_sgemm_nt_8x4_lib8
#endif





//                               rdi    rsi           rdx       rcx       r8           r9        rsp+8
// void kernel_sgemm_nt_4x8_lib8(int k, float *alpha, float *A, float *B, float *beta, float *C, float *D);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgemm_nt_4x8_lib8
	.type kernel_sgemm_nt_4x8_lib8, @function
kernel_sgemm_nt_4x8_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgemm_nt_4x8_lib8
_kernel_sgemm_nt_4x8_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgemm_nt_4x8_lib8
	.def kernel_sgemm_nt_4x8_lib8; .scl 2; .type 32; .endef
kernel_sgemm_nt_4x8_lib8:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG4, %r11  // B
	movq	ARG3, %r12  // A

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_gemm_add_nt_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_gemm_add_nt_8x4_lib8
#endif
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12   // C

#if MACRO_LEVEL>=1
	INNER_TRAN_SCALE_AB_4X8_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_tran_scale_ab_4x8_lib8
#elif defined(OS_MAC)
	callq _inner_tran_scale_ab_4x8_lib8
#endif
#endif


	// store n

	movq	ARG7, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_4x8_lib8
#elif defined(OS_MAC)
	callq _inner_store_4x8_lib8
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgemm_nt_4x8_lib8, .-kernel_sgemm_nt_4x8_lib8
#endif





//                               rdi    rsi           rdx       rcx       r8           r9        rsp+8
// void kernel_sgemm_nt_8x4_vs_lib8(int k, float *alpha, float *A, float *B, float *beta, float *C, float *D, int km, int kn);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgemm_nt_8x4_vs_lib8
	.type kernel_sgemm_nt_8x4_vs_lib8, @function
kernel_sgemm_nt_8x4_vs_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgemm_nt_8x4_vs_lib8
_kernel_sgemm_nt_8x4_vs_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgemm_nt_8x4_vs_lib8
	.def kernel_sgemm_nt_8x4_vs_lib8; .scl 2; .type 32; .endef
kernel_sgemm_nt_8x4_vs_lib8:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_gemm_add_nt_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_gemm_add_nt_8x4_lib8
#endif
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12   // C

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_AB_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_blend_scale_ab_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_blend_scale_ab_8x4_lib8
#endif
#endif


	// store n

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // km
	movq	ARG9, %r12 // kn

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_VS_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_8x4_vs_lib8
#elif defined(OS_MAC)
	callq _inner_store_8x4_vs_lib8
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgemm_nt_8x4_vs_lib8, .-kernel_sgemm_nt_8x4_vs_lib8
#endif





//                                  rdi    rsi           rdx       rcx       r8           r9        rsp+8
// void kernel_sgemm_nt_4x8_vs_lib8(int k, float *alpha, float *A, float *B, float *beta, float *C, float *D, int km, int kn);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgemm_nt_4x8_vs_lib8
	.type kernel_sgemm_nt_4x8_vs_lib8, @function
kernel_sgemm_nt_4x8_vs_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgemm_nt_4x8_vs_lib8
_kernel_sgemm_nt_4x8_vs_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgemm_nt_4x8_vs_lib8
	.def kernel_sgemm_nt_4x8_vs_lib8; .scl 2; .type 32; .endef
kernel_sgemm_nt_4x8_vs_lib8:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG4, %r11  // B
	movq	ARG3, %r12  // A

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_gemm_add_nt_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_gemm_add_nt_8x4_lib8
#endif
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12   // C

#if MACRO_LEVEL>=1
	INNER_TRAN_SCALE_AB_4X8_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_tran_scale_ab_4x8_lib8
#elif defined(OS_MAC)
	callq _inner_tran_scale_ab_4x8_lib8
#endif
#endif


	// store n

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // km
	movq	ARG9, %r12 // kn

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_VS_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_4x8_vs_lib8
#elif defined(OS_MAC)
	callq _inner_store_4x8_vs_lib8
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgemm_nt_4x8_vs_lib8, .-kernel_sgemm_nt_4x8_vs_lib8
#endif





//                                   rdi    rsi           rdx       rcx       r8           r9           rsp+8     rsp+16   rsp+24       rsp+32    rsp+40   rsp+48  rsp+56  rsp+64  rsp+72
// void kernel_sgemm_nt_8x4_gen_lib8(int k, float *alpha, float *A, float *B, float *beta, int offsetC, float *C, int sdc, int offsetD, float *D, int sdd, int m0, int m1, int n0, int n1);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgemm_nt_8x4_gen_lib8
	.type kernel_sgemm_nt_8x4_gen_lib8, @function
kernel_sgemm_nt_8x4_gen_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgemm_nt_8x4_gen_lib8
_kernel_sgemm_nt_8x4_gen_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgemm_nt_8x4_gen_lib8
	.def kernel_sgemm_nt_8x4_gen_lib8; .scl 2; .type 32; .endef
kernel_sgemm_nt_8x4_gen_lib8:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_gemm_add_nt_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_gemm_add_nt_8x4_lib8
#endif
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12 // offsetC
	movq	ARG7, %r13 // C
	movq	ARG8, %r14 // sdc
	sall	$ 5, %r14d // 8*sdc*sizeof(float)

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_AB_8X4_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_blend_scale_ab_8x4_gen_lib8
#elif defined(OS_MAC)
	callq _inner_blend_scale_ab_8x4_gen_lib8
#endif
#endif


	// store n gen

	movq	ARG9, %r10 // offsetD
	movq	ARG10, %r11 // D
	movq	ARG11, %r12 // sdd
	sall	$ 5, %r12d // 8*sdb*sizeof(float)
	movq	ARG12, %r13 // m0
	movq	ARG13, %r14 // m1
	movq	ARG14, %r15 // n0
	movq	ARG15, %rax // n1

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_8x4_gen_lib8
#elif defined(OS_MAC)
	callq _inner_store_8x4_gen_lib8
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgemm_nt_8x4_gen_lib8, .-kernel_sgemm_nt_8x4_gen_lib8
#endif





//                                   rdi    rsi           rdx       rcx       r8           r9           rsp+8     rsp+16   rsp+24       rsp+32    rsp+40   rsp+48  rsp+56  rsp+64  rsp+72
// void kernel_sgemm_nt_4x8_gen_lib8(int k, float *alpha, float *A, float *B, float *beta, int offsetC, float *C, int sdc, int offsetD, float *D, int sdd, int m0, int m1, int n0, int n1);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgemm_nt_4x8_gen_lib8
	.type kernel_sgemm_nt_4x8_gen_lib8, @function
kernel_sgemm_nt_4x8_gen_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgemm_nt_4x8_gen_lib8
_kernel_sgemm_nt_4x8_gen_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgemm_nt_4x8_gen_lib8
	.def kernel_sgemm_nt_4x8_gen_lib8; .scl 2; .type 32; .endef
kernel_sgemm_nt_4x8_gen_lib8:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG4, %r11  // A
	movq	ARG3, %r12  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_gemm_add_nt_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_gemm_add_nt_8x4_lib8
#endif
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12 // offsetC
	movq	ARG7, %r13 // C
	movq	ARG8, %r14 // sdc
	sall	$ 5, %r14d // 8*sdc*sizeof(float)

#if MACRO_LEVEL>=1
	INNER_TRAN_SCALE_AB_4X8_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_tran_scale_ab_4x8_gen_lib8
#elif defined(OS_MAC)
	callq _inner_tran_scale_ab_4x8_gen_lib8
#endif
#endif


	// store n gen

	movq	ARG9, %r10 // offsetD
	movq	ARG10, %r11 // D
	movq	ARG11, %r12 // sdd
	sall	$ 5, %r12d // 8*sdb*sizeof(float)
	movq	ARG12, %r13 // m0
	movq	ARG13, %r14 // m1
	movq	ARG14, %r15 // n0
	movq	ARG15, %rax // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_4x8_gen_lib8
#elif defined(OS_MAC)
	callq _inner_store_4x8_gen_lib8
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgemm_nt_4x8_gen_lib8, .-kernel_sgemm_nt_4x8_gen_lib8
#endif





//                               rdi    rsi           rdx        rcx         r8         r9      rsp+8        rsp+16    rsp+24
// void kernel_sgemm_nn_8x4_lib8(int k, float *alpha, float *A, int offsetB, float *B, int sdb, float *beta, float *C, float *D);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgemm_nn_8x4_lib8
	.type kernel_sgemm_nn_8x4_lib8, @function
kernel_sgemm_nn_8x4_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgemm_nn_8x4_lib8
_kernel_sgemm_nn_8x4_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgemm_nn_8x4_lib8
	.def kernel_sgemm_nn_8x4_lib8; .scl 2; .type 32; .endef
kernel_sgemm_nn_8x4_lib8:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG5, %r12  // B
	movq	ARG6, %r13 // sdb
	sall	$ 5, %r13d // 4*sdb*sizeof(double)
	movq	ARG4, %r14 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_GEMM_ADD_NN_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_gemm_add_nn_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_edge_gemm_add_nn_8x4_lib8
#endif
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_gemm_add_nn_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_gemm_add_nn_8x4_lib8
#endif
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_scale_ab_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_scale_ab_8x4_lib8
#endif
#endif


	// store n

	movq	ARG9, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_store_8x4_lib8
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgemm_nn_8x4_lib8, .-kernel_sgemm_nn_8x4_lib8
#endif





//                               1      2             3         4            5         6        7            8         9         10      11
// void kernel_sgemm_nn_8x4_lib8(int k, float *alpha, float *A, int offsetB, float *B, int sdb, float *beta, float *C, float *D, int km, int kn);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgemm_nn_8x4_vs_lib8
	.type kernel_sgemm_nn_8x4_vs_lib8, @function
kernel_sgemm_nn_8x4_vs_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgemm_nn_8x4_vs_lib8
_kernel_sgemm_nn_8x4_vs_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgemm_nn_8x4_vs_lib8
	.def kernel_sgemm_nn_8x4_vs_lib8; .scl 2; .type 32; .endef
kernel_sgemm_nn_8x4_vs_lib8:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG5, %r12  // B
	movq	ARG6, %r13 // sdb
	sall	$ 5, %r13d // 4*sdb*sizeof(double)
	movq	ARG4, %r14 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_GEMM_ADD_NN_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_gemm_add_nn_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_edge_gemm_add_nn_8x4_lib8
#endif
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_gemm_add_nn_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_gemm_add_nn_8x4_lib8
#endif
#endif


	// call inner blend 

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_scale_ab_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_scale_ab_8x4_lib8
#endif
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // km
	movq	ARG11, %r12 // kn

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_VS_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_8x4_vs_lib8
#elif defined(OS_MAC)
	callq _inner_store_8x4_vs_lib8
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgemm_nn_8x4_vs_lib8, .-kernel_sgemm_nn_8x4_vs_lib8
#endif





//                                   rdi    rsi           rdx       rcx       r8        r9       rsp+8        rsp+16    rsp+24    rsp+32    rsp+40   rsp+48     rsp+56   rsp+64  rsp+72  rsp+80  rsp+88
// void kernel_sgemm_nn_8x4_gen_lib4(int k, float *alpha, float *A, int offB, float *B, int sdb, float *beta, int offC, float *C, int sdc, int offD, float *D, int sdd, int m0, int m1, int n0, int n1);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgemm_nn_8x4_gen_lib8
	.type kernel_sgemm_nn_8x4_gen_lib8, @function
kernel_sgemm_nn_8x4_gen_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgemm_nn_8x4_gen_lib8
_kernel_sgemm_nn_8x4_gen_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgemm_nn_8x4_gen_lib8
	.def kernel_sgemm_nn_8x4_gen_lib8; .scl 2; .type 32; .endef
kernel_sgemm_nn_8x4_gen_lib8:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG5, %r12  // B
	movq	ARG6, %r13 // sdb
	sall	$ 5, %r13d // 4*sdb*sizeof(double)
	movq	ARG4, %r14 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_GEMM_ADD_NN_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_gemm_add_nn_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_edge_gemm_add_nn_8x4_lib8
#endif
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_gemm_add_nn_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_gemm_add_nn_8x4_lib8
#endif
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12 // offsetC
	movq	ARG9, %r13 // C
	movq	ARG10, %r14 // sdc
	sall	$ 5, %r14d // 4*sdc*sizeof(double)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_scale_ab_8x4_gen_lib8
#elif defined(OS_MAC)
	callq _inner_scale_ab_8x4_gen_lib8
#endif
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
	INNER_STORE_8X4_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_8x4_gen_lib8
#elif defined(OS_MAC)
	callq _inner_store_8x4_gen_lib8
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgemm_nn_8x4_gen_lib8, .-kernel_sgemm_nn_8x4_gen_lib8
#endif





//                                 rdi    rsi           rdx       rcx       r8           r9        rsp+8
// void kernel_ssyrk_nt_l_8x4_lib8(int k, float *alpha, float *A, float *B, float *beta, float *C, float *D);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_ssyrk_nt_l_8x4_lib8
	.type kernel_ssyrk_nt_l_8x4_lib8, @function
kernel_ssyrk_nt_l_8x4_lib8:
#elif defined(OS_MAC)
	.globl _kernel_ssyrk_nt_l_8x4_lib8
_kernel_ssyrk_nt_l_8x4_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_ssyrk_nt_l_8x4_lib8
	.def kernel_ssyrk_nt_l_8x4_lib8; .scl 2; .type 32; .endef
kernel_ssyrk_nt_l_8x4_lib8:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_gemm_add_nt_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_gemm_add_nt_8x4_lib8
#endif
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12   // C

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_AB_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_blend_scale_ab_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_blend_scale_ab_8x4_lib8
#endif
#endif


	// store n

	movq	ARG7, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_l_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_store_l_8x4_lib8
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_ssyrk_nt_l_8x4_lib8, .-kernel_ssyrk_nt_l_8x4_lib8
#endif





//                                    1      2             3         4         5            6         7         8       9
// void kernel_ssyrk_nt_l_8x4_vs_lib8(int k, float *alpha, float *A, float *B, float *beta, float *C, float *D, int km, int kn);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_ssyrk_nt_l_8x4_vs_lib8
	.type kernel_ssyrk_nt_l_8x4_vs_lib8, @function
kernel_ssyrk_nt_l_8x4_vs_lib8:
#elif defined(OS_MAC)
	.globl _kernel_ssyrk_nt_l_8x4_vs_lib8
_kernel_ssyrk_nt_l_8x4_vs_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_ssyrk_nt_l_8x4_vs_lib8
	.def kernel_ssyrk_nt_l_8x4_vs_lib8; .scl 2; .type 32; .endef
kernel_ssyrk_nt_l_8x4_vs_lib8:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_gemm_add_nt_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_gemm_add_nt_8x4_lib8
#endif
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12   // C

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_AB_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_blend_scale_ab_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_blend_scale_ab_8x4_lib8
#endif
#endif


	// store n

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // km
	movq	ARG9, %r12 // kn

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X4_VS_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_l_8x4_vs_lib8
#elif defined(OS_MAC)
	callq _inner_store_l_8x4_vs_lib8
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_ssyrk_nt_l_8x4_vs_lib8, .-kernel_ssyrk_nt_l_8x4_vs_lib8
#endif





//                                      edi    rsi       rdx       ecx       r8        r9        rsp+8     
// void kernel_strsm_nt_rl_inv_8x4_lib8(int k, float *A, float *B, float *C, float *D, float *E, float *inv_diag_E);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_strsm_nt_rl_inv_8x4_lib8
	.type kernel_strsm_nt_rl_inv_8x4_lib8, @function
kernel_strsm_nt_rl_inv_8x4_lib8:
#elif defined(OS_MAC)
	.globl _kernel_strsm_nt_rl_inv_8x4_lib8
_kernel_strsm_nt_rl_inv_8x4_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_strsm_nt_rl_inv_8x4_lib8
	.def kernel_strsm_nt_rl_inv_8x4_lib8; .scl 2; .type 32; .endef
kernel_strsm_nt_rl_inv_8x4_lib8:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3

	// call inner dgemm kernel nt 

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_SUB_NT_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_gemm_sub_nt_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_gemm_sub_nt_8x4_lib8
#endif
#endif


	// call inner blender_loader nn

	movq	ARG4, %r10

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_11_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_blend_scale_11_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_blend_scale_11_8x4_lib8
#endif
#endif


	// solve

	movq	ARG6, %r10  // E 
	movq	ARG7, %r11  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_INV_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_trsm_rlt_inv_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_edge_trsm_rlt_inv_8x4_lib8
#endif
#endif


	// store

	movq	ARG5, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_store_8x4_lib8
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_strsm_nt_rl_inv_8x4_lib8, .-kernel_strsm_nt_rl_inv_8x4_lib8
#endif





//                                         edi    rsi       rdx       ecx       r8        r9        rsp+8               rsp+16  rsp+24  
// void kernel_strsm_nt_rl_inv_8x4_vs_lib8(int k, float *A, float *B, float *C, float *D, float *E, float *inv_diag_E, int km, int kn);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_strsm_nt_rl_inv_8x4_vs_lib8
	.type kernel_strsm_nt_rl_inv_8x4_vs_lib8, @function
kernel_strsm_nt_rl_inv_8x4_vs_lib8:
#elif defined(OS_MAC)
	.globl _kernel_strsm_nt_rl_inv_8x4_vs_lib8
_kernel_strsm_nt_rl_inv_8x4_vs_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_strsm_nt_rl_inv_8x4_vs_lib8
	.def kernel_strsm_nt_rl_inv_8x4_vs_lib8; .scl 2; .type 32; .endef
kernel_strsm_nt_rl_inv_8x4_vs_lib8:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3


	// call inner dgemm kernel nt 

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_SUB_NT_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_gemm_sub_nt_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_gemm_sub_nt_8x4_lib8
#endif
#endif


	// call inner blender_loader nn // TODO scale gen

	movq	ARG4, %r10 // C

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_11_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_blend_scale_11_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_blend_scale_11_8x4_lib8
#endif
#endif


	// solve

	movq	ARG6, %r10  // E 
	movq	ARG7, %r11  // inv_diag_E 
	movq	ARG9, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_INV_8X4_VS_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_trsm_rlt_inv_8x4_vs_lib8
#elif defined(OS_MAC)
	callq _inner_edge_trsm_rlt_inv_8x4_vs_lib8
#endif
#endif


	// store

	movq	ARG5, %r10 // D
	movq	ARG8, %r11 // km 
	movq	ARG9, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_VS_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_8x4_vs_lib8
#elif defined(OS_MAC)
	callq _inner_store_8x4_vs_lib8
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_strsm_nt_rl_inv_8x4_vs_lib8, .-kernel_strsm_nt_rl_inv_8x4_vs_lib8
#endif





//                                            1       2          3          4       5          6          7         8         9         10
// void kernel_sgemm_strsm_nt_rl_inv_8x4_lib8(int kp, float *Ap, float *Bp, int km, float *Am, float *Bm, float *C, float *D, float *E, float *inv_diag_E);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgemm_strsm_nt_rl_inv_8x4_lib8
	.type kernel_sgemm_strsm_nt_rl_inv_8x4_lib8, @function
kernel_sgemm_strsm_nt_rl_inv_8x4_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgemm_strsm_nt_rl_inv_8x4_lib8
_kernel_sgemm_strsm_nt_rl_inv_8x4_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgemm_strsm_nt_rl_inv_8x4_lib8
	.def kernel_sgemm_strsm_nt_rl_inv_8x4_lib8; .scl 2; .type 32; .endef
kernel_sgemm_strsm_nt_rl_inv_8x4_lib8:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG2, %r11  // Ap
	movq	ARG3, %r12  // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_gemm_add_nt_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_gemm_add_nt_8x4_lib8
#endif
#endif


	// call inner dgemm kernel nt sub

	movq	ARG4, %r10 // km
	movq	ARG5, %r11   // Am
	movq	ARG6, %r12   // Bm

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_SUB_NT_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_gemm_sub_nt_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_gemm_sub_nt_8x4_lib8
#endif
#endif


	// call inner blender_loader nn

	movq	ARG7, %r10   // C

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_11_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_blend_scale_11_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_blend_scale_11_8x4_lib8
#endif
#endif


	// solve

	movq	ARG9, %r10  // E 
	movq	ARG10, %r11  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_INV_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_trsm_rlt_inv_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_edge_trsm_rlt_inv_8x4_lib8
#endif
#endif


	// store

	movq	ARG8, %r10   // D

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_store_8x4_lib8
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgemm_strsm_nt_rl_inv_8x4_lib8, .-kernel_sgemm_strsm_nt_rl_inv_8x4_lib8
#endif





//                                               1       2          3          4       5          6          7         8         9         10                 11      12
// void kernel_sgemm_strsm_nt_rl_inv_8x4_vs_lib8(int kp, float *Ap, float *Bp, int km, float *Am, float *Bm, float *C, float *D, float *E, float *inv_diag_E, int km, int kn);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_sgemm_strsm_nt_rl_inv_8x4_vs_lib8
	.type kernel_sgemm_strsm_nt_rl_inv_8x4_vs_lib8, @function
kernel_sgemm_strsm_nt_rl_inv_8x4_vs_lib8:
#elif defined(OS_MAC)
	.globl _kernel_sgemm_strsm_nt_rl_inv_8x4_vs_lib8
_kernel_sgemm_strsm_nt_rl_inv_8x4_vs_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_sgemm_strsm_nt_rl_inv_8x4_vs_lib8
	.def kernel_sgemm_strsm_nt_rl_inv_8x4_vs_lib8; .scl 2; .type 32; .endef
kernel_sgemm_strsm_nt_rl_inv_8x4_vs_lib8:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG2, %r11  // Ap
	movq	ARG3, %r12  // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_gemm_add_nt_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_gemm_add_nt_8x4_lib8
#endif
#endif


	// call inner dgemm kernel nt sub

	movq	ARG4, %r10 // km
	movq	ARG5, %r11   // Am
	movq	ARG6, %r12   // Bm

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_SUB_NT_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_gemm_sub_nt_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_gemm_sub_nt_8x4_lib8
#endif
#endif


	// call inner blender_loader nn

	movq	ARG7, %r10  // C 

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_11_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_blend_scale_11_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_blend_scale_11_8x4_lib8
#endif
#endif


	// solve

	movq	ARG9, %r10  // E 
	movq	ARG10, %r11  // inv_diag_E 
	movq	ARG12, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_INV_8X4_VS_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_trsm_rlt_inv_8x4_vs_lib8
#elif defined(OS_MAC)
	callq _inner_edge_trsm_rlt_inv_8x4_vs_lib8
#endif
#endif


	// store

	movq	ARG8, %r10 // D 
	movq	ARG11, %r11 // km 
	movq	ARG12, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_VS_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_8x4_vs_lib8
#elif defined(OS_MAC)
	callq _inner_store_8x4_vs_lib8
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_sgemm_strsm_nt_rl_inv_8x4_vs_lib8, .-kernel_sgemm_strsm_nt_rl_inv_8x4_vs_lib8
#endif





//                                  edi    rsi       rdx       rcx       r8        r9
// void kernel_spotrf_nt_l_8x4_lib8(int k, float *A, float *B, float *C, float *D, float *inv_diag_D);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_spotrf_nt_l_8x4_lib8
	.type kernel_spotrf_nt_l_8x4_lib8, @function
kernel_spotrf_nt_l_8x4_lib8:
#elif defined(OS_MAC)
	.globl _kernel_spotrf_nt_l_8x4_lib8
_kernel_spotrf_nt_l_8x4_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_spotrf_nt_l_8x4_lib8
	.def kernel_spotrf_nt_l_8x4_lib8; .scl 2; .type 32; .endef
kernel_spotrf_nt_l_8x4_lib8:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3


	// call inner dgemm kernel nt

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_SUB_NT_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_gemm_sub_nt_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_gemm_sub_nt_8x4_lib8
#endif
#endif


	// call inner blender_loader nn

	movq	ARG4, %r10 // C

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_11_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_blend_scale_11_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_blend_scale_11_8x4_lib8
#endif
#endif


	// factorization

	movq	ARG6, %r10  // inv_diag_D 

#if MACRO_LEVEL>=1
	INNER_EDGE_POTRF_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_potrf_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_edge_potrf_8x4_lib8
#endif
#endif


	// store

	movq	ARG5, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_l_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_store_l_8x4_lib8
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_spotrf_nt_l_8x4_lib8, .-kernel_spotrf_nt_l_8x4_lib8
#endif





//                                     edi    rsi       rdx       rcx       r8        r9                  rsp+8   rsp+16
// void kernel_spotrf_nt_l_8x4_vs_lib8(int k, float *A, float *B, float *C, float *D, float *inv_diag_D, int km, int kn);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_spotrf_nt_l_8x4_vs_lib8
	.type kernel_spotrf_nt_l_8x4_vs_lib8, @function
kernel_spotrf_nt_l_8x4_vs_lib8:
#elif defined(OS_MAC)
	.globl _kernel_spotrf_nt_l_8x4_vs_lib8
_kernel_spotrf_nt_l_8x4_vs_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_spotrf_nt_l_8x4_vs_lib8
	.def kernel_spotrf_nt_l_8x4_vs_lib8; .scl 2; .type 32; .endef
kernel_spotrf_nt_l_8x4_vs_lib8:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3


	// call inner dgemm kernel nt

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_SUB_NT_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_gemm_sub_nt_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_gemm_sub_nt_8x4_lib8
#endif
#endif


	// call inner blender_loader nn

	movq	ARG4, %r10 // C

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_11_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_blend_scale_11_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_blend_scale_11_8x4_lib8
#endif
#endif


	// factorization

	movq	ARG6, %r10  // inv_diag_D 
	movq	ARG8, %r11 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_POTRF_8X4_VS_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_potrf_8x4_vs_lib8
#elif defined(OS_MAC)
	callq _inner_edge_potrf_8x4_vs_lib8
#endif
#endif


	// store

	movq	ARG5, %r10 // D
	movq	ARG7, %r11 // m1 
	movq	ARG8, %r12 // n1 

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X4_VS_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_l_8x4_vs_lib8
#elif defined(OS_MAC)
	callq _inner_store_l_8x4_vs_lib8
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_spotrf_nt_l_8x4_vs_lib8, .-kernel_spotrf_nt_l_8x4_vs_lib8
#endif





//                                        1       2          3          4       5          6          7         8         9
// void kernel_ssyrk_spotrf_nt_l_8x4_lib8(int kp, float *Ap, float *Bp, int km, float *Am, float *Bm, float *C, float *D, float *inv_diag_D);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_ssyrk_spotrf_nt_l_8x4_lib8
	.type kernel_ssyrk_spotrf_nt_l_8x4_lib8, @function
kernel_ssyrk_spotrf_nt_l_8x4_lib8:
#elif defined(OS_MAC)
	.globl _kernel_ssyrk_spotrf_nt_l_8x4_lib8
_kernel_ssyrk_spotrf_nt_l_8x4_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_ssyrk_spotrf_nt_l_8x4_lib8
	.def kernel_ssyrk_spotrf_nt_l_8x4_lib8; .scl 2; .type 32; .endef
kernel_ssyrk_spotrf_nt_l_8x4_lib8:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG2, %r11  // Ap
	movq	ARG3, %r12  // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_gemm_add_nt_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_gemm_add_nt_8x4_lib8
#endif
#endif


	// call inner dgemm kernel nt sub

	movq	ARG4, %r10 // km
	movq	ARG5, %r11   // Am
	movq	ARG6, %r12   // Bm

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_SUB_NT_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_gemm_sub_nt_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_gemm_sub_nt_8x4_lib8
#endif
#endif


	// call inner blender_loader nn

	movq	ARG7, %r10   // C

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_11_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_blend_scale_11_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_blend_scale_11_8x4_lib8
#endif
#endif


	// factorization

	movq	ARG9, %r10  // inv_diag_D 
	movl	$ 4, %r11d

#if MACRO_LEVEL>=1
	INNER_EDGE_POTRF_8X4_VS_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_potrf_8x4_vs_lib8
#elif defined(OS_MAC)
	callq _inner_edge_potrf_8x4_vs_lib8
#endif
#endif


	// store

	movq	ARG8, %r10  // D 

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_l_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_store_l_8x4_lib8
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_ssyrk_spotrf_nt_l_8x4_lib8, .-kernel_ssyrk_spotrf_nt_l_8x4_lib8
#endif





//                                           1       2          3          4       5          6          7         8         9                  10      11
// void kernel_ssyrk_spotrf_nt_l_8x4_vs_lib8(int kp, float *Ap, float *Bp, int km, float *Am, float *Bm, float *C, float *D, float *inv_diag_D, int km, int kn);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_ssyrk_spotrf_nt_l_8x4_vs_lib8
	.type kernel_ssyrk_spotrf_nt_l_8x4_vs_lib8, @function
kernel_ssyrk_spotrf_nt_l_8x4_vs_lib8:
#elif defined(OS_MAC)
	.globl _kernel_ssyrk_spotrf_nt_l_8x4_vs_lib8
_kernel_ssyrk_spotrf_nt_l_8x4_vs_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_ssyrk_spotrf_nt_l_8x4_vs_lib8
	.def kernel_ssyrk_spotrf_nt_l_8x4_vs_lib8; .scl 2; .type 32; .endef
kernel_ssyrk_spotrf_nt_l_8x4_vs_lib8:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3


	// call inner dgemm kernel nt add

	movq	ARG1, %r10 // kp
	movq	ARG2, %r11  // Ap
	movq	ARG3, %r12  // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_gemm_add_nt_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_gemm_add_nt_8x4_lib8
#endif
#endif


	// call inner dgemm kernel nt sub

	movq	ARG4, %r10 // km
	movq	ARG5, %r11   // Am
	movq	ARG6, %r12   // Bm

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_SUB_NT_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_gemm_sub_nt_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_gemm_sub_nt_8x4_lib8
#endif
#endif


	// call inner blender_loader nn

	movq	ARG7, %r10   // C

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_11_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_blend_scale_11_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_blend_scale_11_8x4_lib8
#endif
#endif


	// factorization

	movq	ARG9, %r10  // inv_diag_D 
	movq	ARG11, %r11 // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_POTRF_8X4_VS_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_potrf_8x4_vs_lib8
#elif defined(OS_MAC)
	callq _inner_edge_potrf_8x4_vs_lib8
#endif
#endif


	// store

	movq	ARG8, %r10  // D 
	movq	ARG10, %r11 // km 
	movq	ARG11, %r12 // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X4_VS_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_l_8x4_vs_lib8
#elif defined(OS_MAC)
	callq _inner_store_l_8x4_vs_lib8
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_ssyrk_spotrf_nt_l_8x4_vs_lib8, .-kernel_ssyrk_spotrf_nt_l_8x4_vs_lib8
#endif





//                                  1      2             3         4            5         6        7
// void kernel_strmm_nn_rl_8x4_lib8(int k, float *alpha, float *A, int offsetB, float *B, int sdb, float *D);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_strmm_nn_rl_8x4_lib8
	.type kernel_strmm_nn_rl_8x4_lib8, @function
kernel_strmm_nn_rl_8x4_lib8:
#elif defined(OS_MAC)
	.globl _kernel_strmm_nn_rl_8x4_lib8
_kernel_strmm_nn_rl_8x4_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_strmm_nn_rl_8x4_lib8
	.def kernel_strmm_nn_rl_8x4_lib8; .scl 2; .type 32; .endef
kernel_strmm_nn_rl_8x4_lib8:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3


	// initial triangle

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG5, %r12 // B
	movq	ARG6, %r13 // sdb
	sall	$ 5, %r13d // 4*sdb*sizeof(double)
	movq	ARG4, %r14 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMM_NN_RL_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_trmm_nn_rl_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_edge_trmm_nn_rl_8x4_lib8
#endif
#endif

#if MACRO_LEVEL>=1
	INNER_EDGE_GEMM_ADD_NN_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_gemm_add_nn_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_edge_gemm_add_nn_8x4_lib8
#endif
#endif

	// call inner dgemm kernel nt after initial triangle

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_gemm_add_nn_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_gemm_add_nn_8x4_lib8
#endif
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha

#if MACRO_LEVEL>=1
	INNER_SCALE_A0_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_scale_a0_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_scale_a0_8x4_lib8
#endif
#endif


	// store n

	movq	ARG7, %r10 // D

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_store_8x4_lib8
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_strmm_nn_rl_8x4_lib8, .-kernel_strmm_nn_rl_8x4_lib8
#endif





//                                     1      2             3         4            5         6        7         8       9
// void kernel_strmm_nn_rl_8x4_vs_lib8(int k, float *alpha, float *A, int offsetB, float *B, int sdb, float *D, int km, int kn);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_strmm_nn_rl_8x4_vs_lib8
	.type kernel_strmm_nn_rl_8x4_vs_lib8, @function
kernel_strmm_nn_rl_8x4_vs_lib8:
#elif defined(OS_MAC)
	.globl _kernel_strmm_nn_rl_8x4_vs_lib8
_kernel_strmm_nn_rl_8x4_vs_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_strmm_nn_rl_8x4_vs_lib8
	.def kernel_strmm_nn_rl_8x4_vs_lib8; .scl 2; .type 32; .endef
kernel_strmm_nn_rl_8x4_vs_lib8:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3


	// initial triangle

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG5, %r12 // B
	movq	ARG6, %r13 // sdb
	sall	$ 5, %r13d // 4*sdb*sizeof(double)
	movq	ARG4, %r14 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMM_NN_RL_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_trmm_nn_rl_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_edge_trmm_nn_rl_8x4_lib8
#endif
#endif

#if MACRO_LEVEL>=1
	INNER_EDGE_GEMM_ADD_NN_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_gemm_add_nn_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_edge_gemm_add_nn_8x4_lib8
#endif
#endif

	// call inner dgemm kernel nt after initial triangle

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_gemm_add_nn_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_gemm_add_nn_8x4_lib8
#endif
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha

#if MACRO_LEVEL>=1
	INNER_SCALE_A0_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_scale_a0_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_scale_a0_8x4_lib8
#endif
#endif


	// store n

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // km
	movq	ARG9, %r12 // kn

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_VS_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_8x4_vs_lib8
#elif defined(OS_MAC)
	callq _inner_store_8x4_vs_lib8
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_strmm_nn_rl_8x4_vs_lib8, .-kernel_strmm_nn_rl_8x4_vs_lib8
#endif





//                                      1      2             3         4            5         6        7            8         9        10      11      12      13
// void kernel_strmm_nn_rl_8x4_gen_lib8(int k, float *alpha, float *A, int offsetB, float *B, int sdb, int offsetD, float *D, int sdd, int m0, int m1, int n0, int n1);

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_strmm_nn_rl_8x4_gen_lib8
	.type kernel_strmm_nn_rl_8x4_gen_lib8, @function
kernel_strmm_nn_rl_8x4_gen_lib8:
#elif defined(OS_MAC)
	.globl _kernel_strmm_nn_rl_8x4_gen_lib8
_kernel_strmm_nn_rl_8x4_gen_lib8:
#elif defined(OS_WINDOWS)
	.globl kernel_strmm_nn_rl_8x4_gen_lib8
	.def kernel_strmm_nn_rl_8x4_gen_lib8; .scl 2; .type 32; .endef
kernel_strmm_nn_rl_8x4_gen_lib8:
#endif
	
	PROLOGUE

	// zero accumulation registers

	vxorps	%ymm0, %ymm0, %ymm0
	vmovaps	%ymm0, %ymm1
	vmovaps	%ymm0, %ymm2
	vmovaps	%ymm0, %ymm3


	// initial triangle

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // A
	movq	ARG5, %r12 // B
	movq	ARG6, %r13 // sdb
	sall	$ 5, %r13d // 4*sdb*sizeof(double)
	movq	ARG4, %r14 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMM_NN_RL_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_trmm_nn_rl_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_edge_trmm_nn_rl_8x4_lib8
#endif
#endif

#if MACRO_LEVEL>=1
	INNER_EDGE_GEMM_ADD_NN_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_edge_gemm_add_nn_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_edge_gemm_add_nn_8x4_lib8
#endif
#endif

	// call inner dgemm kernel nt after initial triangle

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_gemm_add_nn_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_kernel_gemm_add_nn_8x4_lib8
#endif
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha

#if MACRO_LEVEL>=1
	INNER_SCALE_A0_8X4_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_scale_a0_8x4_lib8
#elif defined(OS_MAC)
	callq _inner_scale_a0_8x4_lib8
#endif
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
	INNER_STORE_8X4_GEN_LIB8
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_store_8x4_gen_lib8
#elif defined(OS_MAC)
	callq _inner_store_8x4_gen_lib8
#endif
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_strmm_nn_rl_8x4_gen_lib8, .-kernel_strmm_nn_rl_8x4_gen_lib8
#endif





//#if defined(BLAS_API)
#if ( defined(BLAS_API) | ( defined(LA_HIGH_PERFORMANCE) & defined(MF_COLMAJ) ) )

#include "kernel_sgemm_8x4_lib.S"

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
.LC03: // { -1.0 -1.0 1.0 1.0 1.0 1.0 1.0 1.0 }
#elif defined(OS_MAC)
	.align 5
LC03: // { -1.0 -1.0 1.0 1.0 1.0 1.0 1.0 1.0 }
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

