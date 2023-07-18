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
// r12   <- x
// zmm0  <- []_a
// ...
// zmm3  <- []_d

//
// output arguments:
// r10d  <- 0
// r11   <- A+4*k*sizeof(double)
// r12   <- x+k*sizeof(double)
// zmm0  <- []_a
// ...
// zmm3  <- []_d

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMV_N_8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemv_n_8_lib8)
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	cmpl	$ 4, %r10d
	jl		0f // clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// TODO vbroadcastf64x2 + vshufpd

	vmovapd			0*64(%r11), %zmm24
	vbroadcastsd	0*8(%r12), %zmm25
	vfmadd231pd		%zmm24, %zmm25, %zmm0

	vmovapd			1*64(%r11), %zmm24
	vbroadcastsd	1*8(%r12), %zmm25
	vfmadd231pd		%zmm24, %zmm25, %zmm1

	vmovapd			2*64(%r11), %zmm24
	vbroadcastsd	2*8(%r12), %zmm25
	vfmadd231pd		%zmm24, %zmm25, %zmm2

	vmovapd			3*64(%r11), %zmm24
	vbroadcastsd	3*8(%r12), %zmm25
	vfmadd231pd		%zmm24, %zmm25, %zmm3

	subl	$ 4, %r10d
	addq	$ 256, %r11
	addq	$ 32, %r12
	
	cmpl	$ 3, %r10d
	jg		1b // main loop 


	// consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

0: // clean-up
	
	vmovapd			0*64(%r11), %zmm24
	vbroadcastsd	0*8(%r12), %zmm25
	vfmadd231pd		%zmm24, %zmm25, %zmm0
	
	subl	$ 1, %r10d
	addq	$ 64, %r11
	addq	$ 8, %r12
	
	cmpl	$ 0, %r10d
	jg		0b // clean

2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemv_n_8_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- x
// r13   <- offsetA
// zmm0  <- []_a
// ...
// zmm3  <- []_d

//
// output arguments:
// r10d  <- 0
// r11   <- A+4*k*sizeof(double)
// r12   <- x+k*sizeof(double)
// r13   <- offsetA
// zmm0  <- []_a
// ...
// zmm3  <- []_d

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMV_N_8_GEN_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemv_n_8_gen_lib8)
#endif
	
	cmpl	$ 0, %r10d
	jle		2f // return

	// ail = ps-offsetA
	movl	$ 8, %ebx
	subl	%eax, %ebx

	vcvtsi2sd		%ebx, %xmm24, %xmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd			.LC00(%rip), %zmm26
#elif defined(OS_MAC)
	vmovupd			LC00(%rip), %zmm26
#endif
	vbroadcastsd	%xmm24, %zmm24
	vsubpd			%zmm24, %zmm26, %zmm24
	vpmovq2m		%zmm24, %k1

	cmpl	$ 4, %r10d
	jl		0f // clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// TODO vbroadcastf64x2 + vshufpd

	vmovupd			0*64(%r11, %r13, 8), %zmm24 {%k1}{z}
	vbroadcastsd	0*8(%r12), %zmm25
	vfmadd231pd		%zmm24, %zmm25, %zmm0

	vmovupd			1*64(%r11, %r13, 8), %zmm24 {%k1}{z}
	vbroadcastsd	1*8(%r12), %zmm25
	vfmadd231pd		%zmm24, %zmm25, %zmm1

	vmovupd			2*64(%r11, %r13, 8), %zmm24 {%k1}{z}
	vbroadcastsd	2*8(%r12), %zmm25
	vfmadd231pd		%zmm24, %zmm25, %zmm2

	vmovupd			3*64(%r11, %r13, 8), %zmm24 {%k1}{z}
	vbroadcastsd	3*8(%r12), %zmm25
	vfmadd231pd		%zmm24, %zmm25, %zmm3

	subl	$ 4, %r10d
	addq	$ 256, %r11
	addq	$ 32, %r12
	
	cmpl	$ 3, %r10d
	jg		1b // main loop 


	// consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

0: // clean-up
	
	vmovupd			0*64(%r11, %r13, 8), %zmm24 {%k1}{z}
	vbroadcastsd	0*8(%r12), %zmm25
	vfmadd231pd		%zmm24, %zmm25, %zmm0
	
	subl	$ 1, %r10d
	addq	$ 64, %r11
	addq	$ 8, %r12
	
	cmpl	$ 0, %r10d
	jg		0b // clean

2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemv_n_8_gen_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- bs*sda*sizeof(double)
// r13   <- x
// ymm0  <- []
// ...
// ymm7  <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+4*k*sizeof(double)
// r12   <- bs*sda*sizeof(double)
// r13   <- x+k*sizeof(double)
// ymm0  <- []
// ...
// ymm7  <- []

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMV_T_8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemv_t_8_lib8)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	cmpl	$ 8, %r10d
	jl		0f // clean-up loop

	// main loop
	.p2align 3
1: // main loop

	vmovupd		0(%r13), %zmm25

	vmovapd		0*64(%r11), %zmm24
	vfmadd231pd	%zmm24, %zmm25, %zmm0

	vmovapd		1*64(%r11), %zmm24
	vfmadd231pd	%zmm24, %zmm25, %zmm1

	vmovapd		2*64(%r11), %zmm24
	vfmadd231pd	%zmm24, %zmm25, %zmm2

	vmovapd		3*64(%r11), %zmm24
	vfmadd231pd	%zmm24, %zmm25, %zmm3

	vmovapd		4*64(%r11), %zmm24
	vfmadd231pd	%zmm24, %zmm25, %zmm4

	vmovapd		5*64(%r11), %zmm24
	vfmadd231pd	%zmm24, %zmm25, %zmm5

	vmovapd		6*64(%r11), %zmm24
	vfmadd231pd	%zmm24, %zmm25, %zmm6

	vmovapd		7*64(%r11), %zmm24
	vfmadd231pd	%zmm24, %zmm25, %zmm7

	subl	$ 8, %r10d
	addq	%r12, %r11
	addq	$ 64, %r13
	
	cmpl	$ 7, %r10d

	jg		1b // main loop 


	// consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

0: // clean-up
	
	vcvtsi2sd		%r10d, %xmm25, %xmm25
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd			.LC00(%rip), %zmm24
#elif defined(OS_MAC)
	vmovupd			LC00(%rip), %zmm24
#endif
	vbroadcastsd	%xmm25, %zmm25
	vsubpd			%zmm25, %zmm24, %zmm25
	vpmovq2m		%zmm25, %k1

	vmovupd		0(%r13), %zmm25 {%k1}{z}

	vmovapd		0*64(%r11), %zmm24 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm0

	vmovapd		1*64(%r11), %zmm24 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm1

	vmovapd		2*64(%r11), %zmm24 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm2

	vmovapd		3*64(%r11), %zmm24 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm3

	vmovapd		4*64(%r11), %zmm24 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm4

	vmovapd		5*64(%r11), %zmm24 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm5

	vmovapd		6*64(%r11), %zmm24 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm6

	vmovapd		7*64(%r11), %zmm24 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm7

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

	FUN_END(inner_kernel_gemv_t_8_lib8)
#endif




// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- bs*sda*sizeof(double) = 32*sda
// r13   <- x_t
// r14   <- z_n
// ymm0  <- []
// ...
// ymm7  <- []
// ymm8  <- x_n_0
// ...
// ymm15 <- x_n_7

//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMV_NT_8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemv_nt_8_lib8)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	cmpl	$ 8, %r10d
	jl		0f // clean-up loop

	// main loop
	.p2align 3
1: // main loop

	vmovupd		0(%r13), %zmm24
	vmovupd		0(%r14), %zmm25

	vmovapd		0*64(%r11), %zmm26
	vfmadd231pd	%zmm26, %zmm24, %zmm0
	vfmadd231pd	%zmm26, %zmm8, %zmm25

	vmovapd		1*64(%r11), %zmm26
	vfmadd231pd	%zmm26, %zmm24, %zmm1
//	vfmadd231pd	%zmm26, %zmm9, %zmm25
	vmulpd		%zmm26, %zmm9, %zmm27

	vmovapd		2*64(%r11), %zmm26
	vfmadd231pd	%zmm26, %zmm24, %zmm2
//	vfmadd231pd	%zmm26, %zmm10, %zmm25
	vmulpd		%zmm26, %zmm10, %zmm28

	vmovapd		3*64(%r11), %zmm26
	vfmadd231pd	%zmm26, %zmm24, %zmm3
//	vfmadd231pd	%zmm26, %zmm11, %zmm25
	vmulpd		%zmm26, %zmm11, %zmm29

	vmovapd		4*64(%r11), %zmm26
	vfmadd231pd	%zmm26, %zmm24, %zmm4
	vfmadd231pd	%zmm26, %zmm12, %zmm25

	vmovapd		5*64(%r11), %zmm26
	vfmadd231pd	%zmm26, %zmm24, %zmm5
//	vfmadd231pd	%zmm26, %zmm13, %zmm25
	vfmadd231pd	%zmm26, %zmm13, %zmm27

	vmovapd		6*64(%r11), %zmm26
	vfmadd231pd	%zmm26, %zmm24, %zmm6
//	vfmadd231pd	%zmm26, %zmm14, %zmm25
	vfmadd231pd	%zmm26, %zmm14, %zmm28

	vmovapd		7*64(%r11), %zmm26
	vfmadd231pd	%zmm26, %zmm24, %zmm7
//	vfmadd231pd	%zmm26, %zmm15, %zmm25
	vfmadd231pd	%zmm26, %zmm15, %zmm29

	vaddpd		%zmm25, %zmm27, %zmm25
	vaddpd		%zmm28, %zmm29, %zmm28
	vaddpd		%zmm25, %zmm28, %zmm25

	vmovupd	%zmm25, 0(%r14) 

	subl	$ 8, %r10d
	addq	%r12, %r11
	addq	$ 64, %r13
	addq	$ 64, %r14
	
	cmpl	$ 7, %r10d

	jg		1b // main loop 


	// consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

0: // clean-up
	
	vcvtsi2sd		%r10d, %xmm25, %xmm25
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd			.LC00(%rip), %zmm24
#elif defined(OS_MAC)
	vmovupd			LC00(%rip), %zmm24
#endif
	vbroadcastsd	%xmm25, %zmm25
	vsubpd			%zmm25, %zmm24, %zmm25
	vpmovq2m		%zmm25, %k1

	vmovupd		0(%r13), %zmm24 {%k1}{z}
	vmovupd		0(%r14), %zmm25 {%k1}{z}

	vmovapd		0*64(%r11), %zmm26 {%k1}{z}
	vfmadd231pd	%zmm26, %zmm24, %zmm0
	vfmadd231pd	%zmm26, %zmm8, %zmm25

	vmovapd		1*64(%r11), %zmm26 {%k1}{z}
	vfmadd231pd	%zmm26, %zmm24, %zmm1
//	vfmadd231pd	%zmm26, %zmm9, %zmm25
	vmulpd		%zmm26, %zmm9, %zmm27

	vmovapd		2*64(%r11), %zmm26 {%k1}{z}
	vfmadd231pd	%zmm26, %zmm24, %zmm2
//	vfmadd231pd	%zmm26, %zmm10, %zmm25
	vmulpd		%zmm26, %zmm10, %zmm28

	vmovapd		3*64(%r11), %zmm26 {%k1}{z}
	vfmadd231pd	%zmm26, %zmm24, %zmm3
//	vfmadd231pd	%zmm26, %zmm11, %zmm25
	vmulpd		%zmm26, %zmm11, %zmm29

	vmovapd		4*64(%r11), %zmm26 {%k1}{z}
	vfmadd231pd	%zmm26, %zmm24, %zmm4
	vfmadd231pd	%zmm26, %zmm12, %zmm25

	vmovapd		5*64(%r11), %zmm26 {%k1}{z}
	vfmadd231pd	%zmm26, %zmm24, %zmm5
//	vfmadd231pd	%zmm26, %zmm13, %zmm25
	vfmadd231pd	%zmm26, %zmm13, %zmm27

	vmovapd		6*64(%r11), %zmm26 {%k1}{z}
	vfmadd231pd	%zmm26, %zmm24, %zmm6
//	vfmadd231pd	%zmm26, %zmm14, %zmm25
	vfmadd231pd	%zmm26, %zmm14, %zmm28

	vmovapd		7*64(%r11), %zmm26 {%k1}{z}
	vfmadd231pd	%zmm26, %zmm24, %zmm7
//	vfmadd231pd	%zmm26, %zmm15, %zmm25
	vfmadd231pd	%zmm26, %zmm15, %zmm29

	vaddpd		%zmm25, %zmm27, %zmm25
	vaddpd		%zmm28, %zmm29, %zmm28
	vaddpd		%zmm25, %zmm28, %zmm25

	vmovupd	%zmm25, 0(%r14)  {%k1}

	sall	$ 3, %r10d // *sizeof(double)
	addq	%r10, %r11
	addq	%r10, %r13
	addq	%r10, %r14
	xorl	%r10d, %r10d
	
	
2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemv_nt_8_lib8)
#endif






// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- bs*sda*sizeof(double)
// r13   <- x
// r14d  <- offA
// ymm0  <- []
// ...
// ymm7  <- []

//
// output arguments:
// r10d  <- 
// r11   <- 
// r12   <- 
// r13   <- 
// r14d  <- offA
// ymm0  <- []
// ...
// ymm7  <- []

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_GEMV_T_8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_gemv_t_8_lib8)
#endif

	cmpl	$ 0, %r14d
	jle		0f // return

	movl	%r14d, %r15d
	sall	$ 3, %r15d // offA*sizeof(double)

//	subq	%r15, %r11 // A - offA
	subq	%r15, %r13 // x - offA

//	movl	%r10d, %r15d // kmax
//	addl	%r14d, %r15d // kmax + offA
	leal	0(%r10d, %r14d), %r15d

	vcvtsi2sd		%r14d, %xmm24, %xmm24 // offA
	vcvtsi2sd		%r15d, %xmm25, %xmm25 // offA + kmax
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd			.LC00(%rip), %zmm26
#elif defined(OS_MAC)
	vmovupd			LC00(%rip), %zmm26
#endif
	vbroadcastsd	%xmm24, %zmm24
	vbroadcastsd	%xmm25, %zmm25
	vsubpd			%zmm26, %zmm24, %zmm24
	vsubpd			%zmm25, %zmm26, %zmm25
	vandpd			%zmm25, %zmm24, %zmm24
	vpmovq2m		%zmm24, %k1

	vmovupd		0(%r13), %zmm25 {%k1}{z}

	vmovapd		0*64(%r11), %zmm24 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm0

	vmovapd		1*64(%r11), %zmm24 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm1

	vmovapd		2*64(%r11), %zmm24 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm2

	vmovapd		3*64(%r11), %zmm24 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm3

	vmovapd		4*64(%r11), %zmm24 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm4

	vmovapd		5*64(%r11), %zmm24 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm5

	vmovapd		6*64(%r11), %zmm24 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm6

	vmovapd		7*64(%r11), %zmm24 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm7

	addq	$ 64, %r13 // x + 8
	addq	%r12, %r11 // A + bs*sda
		
	addl	%r14d, %r10d
	subl	$ 8, %r10d // kmax - (8-offA)
	
0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_gemv_t_8_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- bs*sda*sizeof(double) = 32*sda
// r13   <- x_t
// r14   <- z_n
// r15   <- offsetA
// ymm0  <- []
// ...
// ymm7  <- []
// ymm8  <- x_n_0
// ...
// ymm15 <- x_n_7

//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_GEMV_NT_8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_gemv_nt_8_lib8)
#endif

	cmpl	$ 0, %r15d
	jle		0f // return

	movl	%r15d, %eax
	sall	$ 3, %eax // offA*sizeof(double)

//	subq	%rax, %r11 // A - offA
	subq	%rax, %r13 // x_t - offA
	subq	%rax, %r14 // z_n - offA

//	movl	%r10d, %eax // kmax
//	addl	%r15d, %eax // kmax + offA
	leal	0(%r10d, %r15d), %eax

	vcvtsi2sd		%r15d, %xmm24, %xmm24 // offA
	vcvtsi2sd		%eax, %xmm25, %xmm25 // offA + kmax
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd			.LC00(%rip), %zmm26
#elif defined(OS_MAC)
	vmovupd			LC00(%rip), %zmm26
#endif
	vbroadcastsd	%xmm24, %zmm24
	vbroadcastsd	%xmm25, %zmm25
	vsubpd			%zmm26, %zmm24, %zmm24
	vsubpd			%zmm25, %zmm26, %zmm25
	vandpd			%zmm25, %zmm24, %zmm24
	vpmovq2m		%zmm24, %k1

	vmovupd		0(%r13), %zmm24 {%k1}{z}
	vmovupd		0(%r14), %zmm25 {%k1}{z}

	vmovapd		0*64(%r11), %zmm26 {%k1}{z}
	vfmadd231pd	%zmm26, %zmm24, %zmm0
	vfmadd231pd	%zmm26, %zmm8, %zmm25

	vmovapd		1*64(%r11), %zmm26 {%k1}{z}
	vfmadd231pd	%zmm26, %zmm24, %zmm1
//	vfmadd231pd	%zmm26, %zmm9, %zmm25
	vmulpd		%zmm26, %zmm9, %zmm27

	vmovapd		2*64(%r11), %zmm26 {%k1}{z}
	vfmadd231pd	%zmm26, %zmm24, %zmm2
//	vfmadd231pd	%zmm26, %zmm10, %zmm25
	vmulpd		%zmm26, %zmm10, %zmm28

	vmovapd		3*64(%r11), %zmm26 {%k1}{z}
	vfmadd231pd	%zmm26, %zmm24, %zmm3
//	vfmadd231pd	%zmm26, %zmm11, %zmm25
	vmulpd		%zmm26, %zmm11, %zmm29

	vmovapd		4*64(%r11), %zmm26 {%k1}{z}
	vfmadd231pd	%zmm26, %zmm24, %zmm4
	vfmadd231pd	%zmm26, %zmm12, %zmm25

	vmovapd		5*64(%r11), %zmm26 {%k1}{z}
	vfmadd231pd	%zmm26, %zmm24, %zmm5
//	vfmadd231pd	%zmm26, %zmm13, %zmm25
	vfmadd231pd	%zmm26, %zmm13, %zmm27

	vmovapd		6*64(%r11), %zmm26 {%k1}{z}
	vfmadd231pd	%zmm26, %zmm24, %zmm6
//	vfmadd231pd	%zmm26, %zmm14, %zmm25
	vfmadd231pd	%zmm26, %zmm14, %zmm28

	vmovapd		7*64(%r11), %zmm26 {%k1}{z}
	vfmadd231pd	%zmm26, %zmm24, %zmm7
//	vfmadd231pd	%zmm26, %zmm15, %zmm25
	vfmadd231pd	%zmm26, %zmm15, %zmm29

	vaddpd		%zmm25, %zmm27, %zmm25
	vaddpd		%zmm28, %zmm29, %zmm28
	vaddpd		%zmm25, %zmm28, %zmm25

	vmovupd	%zmm25, 0(%r14) {%k1}

	addq	$ 64, %r13 // x_t + 8
	addq	$ 64, %r14 // y_n + 8
	addq	%r12, %r11 // A + bs*sda
		
	addl	%r15d, %r10d
	subl	$ 8, %r10d // kmax - (8-offA)
	
0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_gemv_nt_8_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- bs*sda*sizeof(double) = 64*sda
// r13   <- x_t
// r14   <- z_n
// ymm0  <- []
// ...
// ymm7  <- []
// ymm8  <- x_n_0
// ...
// ymm15 <- x_n_7

//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_SYMV_NT_L_8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_symv_l_nt_8_lib8)
#endif

	vmovupd		0(%r13), %zmm24
	vmovupd		0(%r14), %zmm25

	vmovapd		0*64(%r11), %zmm26
	vfmadd231pd	%zmm26, %zmm24, %zmm0
	movl		$ 0xfe, %ebx
	kmovd		%ebx, %k2
	vfmadd231pd	%zmm26, %zmm8, %zmm25 {%k2}

	vmovapd		1*64(%r11), %zmm26
	vfmadd231pd	%zmm26, %zmm24, %zmm1 {%k2}
	movl		$ 0xfc, %ebx
	kmovd		%ebx, %k2
//	vfmadd231pd	%zmm26, %zmm9, %zmm25 {%k2}
	vmulpd		%zmm26, %zmm9, %zmm27 {%k2}{z}

	vmovapd		2*64(%r11), %zmm26
	vfmadd231pd	%zmm26, %zmm24, %zmm2 {%k2}
	movl		$ 0xf8, %ebx
	kmovd		%ebx, %k2
//	vfmadd231pd	%zmm26, %zmm10, %zmm25 {%k2}
	vmulpd		%zmm26, %zmm10, %zmm28 {%k2}{z}

	vmovapd		3*64(%r11), %zmm26
	vfmadd231pd	%zmm26, %zmm24, %zmm3 {%k2}
	movl		$ 0xf0, %ebx
	kmovd		%ebx, %k2
//	vfmadd231pd	%zmm26, %zmm11, %zmm25 {%k2}
	vmulpd		%zmm26, %zmm11, %zmm29 {%k2}{z}

	vmovapd		4*64(%r11), %zmm26
	vfmadd231pd	%zmm26, %zmm24, %zmm4 {%k2}
	movl		$ 0xe0, %ebx
	kmovd		%ebx, %k2
	vfmadd231pd	%zmm26, %zmm12, %zmm25 {%k2}

	vmovapd		5*64(%r11), %zmm26
	vfmadd231pd	%zmm26, %zmm24, %zmm5 {%k2}
	movl		$ 0xc0, %ebx
	kmovd		%ebx, %k2
//	vfmadd231pd	%zmm26, %zmm13, %zmm25 {%k2}
	vfmadd231pd	%zmm26, %zmm13, %zmm27 {%k2}

	vmovapd		6*64(%r11), %zmm26
	vfmadd231pd	%zmm26, %zmm24, %zmm6 {%k2}
	movl		$ 0x80, %ebx
	kmovd		%ebx, %k2
//	vfmadd231pd	%zmm26, %zmm14, %zmm25 {%k2}
	vfmadd231pd	%zmm26, %zmm14, %zmm28 {%k2}

	vmovapd		7*64(%r11), %zmm26
	vfmadd231pd	%zmm26, %zmm24, %zmm7 {%k2}
//	movl		$ 0x00, %ebx
//	kmovd		%ebx, %k2
//	vfmadd231pd	%zmm26, %zmm15, %zmm25 {%k2}
//	vfmadd231pd	%zmm26, %zmm15, %zmm29 {%k2}

	vaddpd		%zmm25, %zmm27, %zmm25
	vaddpd		%zmm28, %zmm29, %zmm28
	vaddpd		%zmm25, %zmm28, %zmm25

	vmovupd	%zmm25, 0(%r14)

	addq	$ 64, %r13 // x_t + 8
	addq	$ 64, %r14 // y_n + 8
	addq	%r12, %r11 // A + bs*sda
		
//	subl	$ 8, %r10d // kmax - (8-offA)
	
0: // return

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_symv_nt_l_8_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- bs*sda*sizeof(double) = 64*sda
// r13   <- x_t
// r14   <- z_n
// r15   <- m1
// ymm0  <- []
// ...
// ymm7  <- []
// ymm8  <- x_n_0
// ...
// ymm15 <- x_n_7

//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_SYMV_NT_L_8_VS_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_symv_nt_l_8_vs_lib8)
#endif

	// saturate m1 to 8
	cmpl	$ 8, %r15d
	jle		0f
	movl	$ 8, %r15d
0:

	// k + m1
	leal	0(%r10d, %r15d), %ebp
	cmpl	$ 8, %ebp
	jle		0f
	movl	$ 8, %ebp
0:

	vcvtsi2sd		%ebp, %xmm24, %xmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd			.LC00(%rip), %zmm26
#elif defined(OS_MAC)
	vmovupd			LC00(%rip), %zmm26
#endif
	vbroadcastsd	%xmm24, %zmm24
	vsubpd			%zmm24, %zmm26, %zmm24
	vpmovq2m		%zmm24, %k1

	vmovupd		0(%r13), %zmm24 {%k1}{z}
	vmovupd		0(%r14), %zmm25 {%k1}{z}

	vmovapd		0*64(%r11), %zmm26 {%k1}{z}
	vfmadd231pd	%zmm26, %zmm24, %zmm0
	movl		$ 0xfe, %ebx
	kmovd		%ebx, %k2
	vfmadd231pd	%zmm26, %zmm8, %zmm25 {%k2}
	cmpl	$ 1, %r15d
	jle		0f

	vmovapd		1*64(%r11), %zmm26 {%k1}{z}
	vfmadd231pd	%zmm26, %zmm24, %zmm1 {%k2}
	movl		$ 0xfc, %ebx
	kmovd		%ebx, %k2
	vfmadd231pd	%zmm26, %zmm9, %zmm25 {%k2}
	cmpl	$ 2, %r15d
	jle		0f

	vmovapd		2*64(%r11), %zmm26 {%k1}{z}
	vfmadd231pd	%zmm26, %zmm24, %zmm2 {%k2}
	movl		$ 0xf8, %ebx
	kmovd		%ebx, %k2
	vfmadd231pd	%zmm26, %zmm10, %zmm25 {%k2}
	cmpl	$ 3, %r15d
	jle		0f

	vmovapd		3*64(%r11), %zmm26 {%k1}{z}
	vfmadd231pd	%zmm26, %zmm24, %zmm3 {%k2}
	movl		$ 0xf0, %ebx
	kmovd		%ebx, %k2
	vfmadd231pd	%zmm26, %zmm11, %zmm25 {%k2}
	cmpl	$ 4, %r15d
	jle		0f

	vmovapd		4*64(%r11), %zmm26 {%k1}{z}
	vfmadd231pd	%zmm26, %zmm24, %zmm4 {%k2}
	movl		$ 0xe0, %ebx
	kmovd		%ebx, %k2
	vfmadd231pd	%zmm26, %zmm12, %zmm25 {%k2}
	cmpl	$ 5, %r15d
	jle		0f

	vmovapd		5*64(%r11), %zmm26 {%k1}{z}
	vfmadd231pd	%zmm26, %zmm24, %zmm5 {%k2}
	movl		$ 0xc0, %ebx
	kmovd		%ebx, %k2
	vfmadd231pd	%zmm26, %zmm13, %zmm25 {%k2}
	cmpl	$ 6, %r15d
	jle		0f

	vmovapd		6*64(%r11), %zmm26 {%k1}{z}
	vfmadd231pd	%zmm26, %zmm24, %zmm6 {%k2}
	movl		$ 0x80, %ebx
	kmovd		%ebx, %k2
	vfmadd231pd	%zmm26, %zmm14, %zmm25 {%k2}
	cmpl	$ 7, %r15d
	jle		0f

	vmovapd		7*64(%r11), %zmm26 {%k1}{z}
	vfmadd231pd	%zmm26, %zmm24, %zmm7 {%k2}
//	movl		$ 0x00, %ebx
//	kmovd		%ebx, %k2
//	vfmadd231pd	%zmm26, %zmm15, %zmm25 {%k2}

0:

	vmovupd	%zmm25, 0(%r14) {%k1}

	// k - (ebp - m1)
	subl	%ebp, %r10d
	addl	%r15d, %r10d

	addq	%r12, %r11
	leaq	0(%r13, %rbp, 8), %r13
	leaq	0(%r14, %rbp, 8), %r14

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_symv_nt_l_8_vs_lib8)
#endif






// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- bs*sda*sizeof(double) = 64*sda
// r13   <- x_t
// r14   <- z_n
// r15   <- m1
// rax   <- offsetA
// ymm0  <- []
// ...
// ymm7  <- []
// ymm8  <- x_n_0
// ...
// ymm15 <- x_n_7

//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_SYMV_NT_L_8_GEN_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_symv_nt_l_8_gen_lib8)
#endif

	// saturate m1 to 8
	cmpl	$ 8, %r15d
	jle		0f
	movl	$ 8, %r15d
0:
	
	// k + m1
	leal	0(%r10d, %r15d), %ebp
	cmpl	$ 8, %ebp
	jle		0f
	movl	$ 8, %ebp
0:

	// ail = ps-offsetA
	movl	$ 8, %ebx
	subl	%eax, %ebx

	// min of m1 and ail
	cmpl	%ebx, %ebp
	jle		0f
	movl	%ebx, %ebp
0:

	vcvtsi2sd		%ebp, %xmm24, %xmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd			.LC00(%rip), %zmm26
#elif defined(OS_MAC)
	vmovupd			LC00(%rip), %zmm26
#endif
	vbroadcastsd	%xmm24, %zmm24
	vsubpd			%zmm24, %zmm26, %zmm24
	vpmovq2m		%zmm24, %k1

	vmovupd		0(%r13), %zmm24 {%k1}{z}
	vmovupd		0(%r14), %zmm25 {%k1}{z}

	vmovupd		0*64(%r11, %rax, 8), %zmm26 {%k1}{z}
	vfmadd231pd	%zmm26, %zmm24, %zmm0
	movl		$ 0xfe, %ebx
	kmovd		%ebx, %k2
	vfmadd231pd	%zmm26, %zmm8, %zmm25 {%k2}
	cmpl	$ 1, %r15d
	jle		0f

	vmovupd		1*64(%r11, %rax, 8), %zmm26 {%k1}{z}
	vfmadd231pd	%zmm26, %zmm24, %zmm1 {%k2}
	movl		$ 0xfc, %ebx
	kmovd		%ebx, %k2
	vfmadd231pd	%zmm26, %zmm9, %zmm25 {%k2}
	cmpl	$ 2, %r15d
	jle		0f

	vmovupd		2*64(%r11, %rax, 8), %zmm26 {%k1}{z}
	vfmadd231pd	%zmm26, %zmm24, %zmm2 {%k2}
	movl		$ 0xf8, %ebx
	kmovd		%ebx, %k2
	vfmadd231pd	%zmm26, %zmm10, %zmm25 {%k2}
	cmpl	$ 3, %r15d
	jle		0f

	vmovupd		3*64(%r11, %rax, 8), %zmm26 {%k1}{z}
	vfmadd231pd	%zmm26, %zmm24, %zmm3 {%k2}
	movl		$ 0xf0, %ebx
	kmovd		%ebx, %k2
	vfmadd231pd	%zmm26, %zmm11, %zmm25 {%k2}
	cmpl	$ 4, %r15d
	jle		0f

	vmovupd		4*64(%r11, %rax, 8), %zmm26 {%k1}{z}
	vfmadd231pd	%zmm26, %zmm24, %zmm4 {%k2}
	movl		$ 0xe0, %ebx
	kmovd		%ebx, %k2
	vfmadd231pd	%zmm26, %zmm12, %zmm25 {%k2}
	cmpl	$ 5, %r15d
	jle		0f

	vmovupd		5*64(%r11, %rax, 8), %zmm26 {%k1}{z}
	vfmadd231pd	%zmm26, %zmm24, %zmm5 {%k2}
	movl		$ 0xc0, %ebx
	kmovd		%ebx, %k2
	vfmadd231pd	%zmm26, %zmm13, %zmm25 {%k2}
	cmpl	$ 6, %r15d
	jle		0f

	vmovupd		6*64(%r11, %rax, 8), %zmm26 {%k1}{z}
	vfmadd231pd	%zmm26, %zmm24, %zmm6 {%k2}
	movl		$ 0x80, %ebx
	kmovd		%ebx, %k2
	vfmadd231pd	%zmm26, %zmm14, %zmm25 {%k2}
	cmpl	$ 7, %r15d
	jle		0f

	vmovupd		7*64(%r11, %rax, 8), %zmm26 {%k1}{z}
	vfmadd231pd	%zmm26, %zmm24, %zmm7 {%k2}
//	movl		$ 0x00, %ebx
//	kmovd		%ebx, %k2
//	vfmadd231pd	%zmm26, %zmm15, %zmm25 {%k2}

0:

	vmovupd	%zmm25, 0(%r14) {%k1}

	// k - (ebp - m1)
	subl	%ebp, %r10d
	addl	%r15d, %r10d

	addq	%r12, %r11
	leaq	0(%r13, %rbp, 8), %r13
	leaq	0(%r14, %rbp, 8), %r14

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_symv_nt_l_8_gen_lib8)
#endif






// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- x
// zmm0  <- []_a
// ...
// zmm3  <- []_d

//
// output arguments:
// r10d  <- 0
// r11   <- A+4*k*sizeof(double)
// r12   <- x+k*sizeof(double)
// zmm0  <- []_a
// ...
// zmm3  <- []_d

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRMV_N_LN_8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_trmv_n_ln_8_lib8)
#endif
	
	vmovapd			0*64(%r11), %zmm24
	vbroadcastsd	0*8(%r12), %zmm25
	vfmadd231pd		%zmm24, %zmm25, %zmm0

	vmovapd			1*64(%r11), %zmm24
	vbroadcastsd	1*8(%r12), %zmm25
	movl			$ 0xfe, %r13d
	kmovd			%r13d, %k1
	vfmadd231pd		%zmm24, %zmm25, %zmm1 {%k1}

	vmovapd			2*64(%r11), %zmm24
	vbroadcastsd	2*8(%r12), %zmm25
	movl			$ 0xfc, %r13d
	kmovd			%r13d, %k1
	vfmadd231pd		%zmm24, %zmm25, %zmm2 {%k1}

	vmovapd			3*64(%r11), %zmm24
	vbroadcastsd	3*8(%r12), %zmm25
	movl			$ 0xf8, %r13d
	kmovd			%r13d, %k1
	vfmadd231pd		%zmm24, %zmm25, %zmm3 {%k1}

	vmovapd			4*64(%r11), %zmm24
	vbroadcastsd	4*8(%r12), %zmm25
	movl			$ 0xf0, %r13d
	kmovd			%r13d, %k1
	vfmadd231pd		%zmm24, %zmm25, %zmm0 {%k1}

	vmovapd			5*64(%r11), %zmm24
	vbroadcastsd	5*8(%r12), %zmm25
	movl			$ 0xe0, %r13d
	kmovd			%r13d, %k1
	vfmadd231pd		%zmm24, %zmm25, %zmm1 {%k1}

	vmovapd			6*64(%r11), %zmm24
	vbroadcastsd	6*8(%r12), %zmm25
	movl			$ 0xc0, %r13d
	kmovd			%r13d, %k1
	vfmadd231pd		%zmm24, %zmm25, %zmm2 {%k1}

	vmovapd			7*64(%r11), %zmm24
	vbroadcastsd	7*8(%r12), %zmm25
	movl			$ 0x80, %r13d
	kmovd			%r13d, %k1
	vfmadd231pd		%zmm24, %zmm25, %zmm3 {%k1}

//	subl	$ 8, %r10d
	addq	$ 8*64, %r11
	addq	$ 8*8, %r12
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trmv_n_ln_8_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- x
// r13   <- m1
// zmm0  <- []_a
// ...
// zmm3  <- []_d

//
// output arguments:
// r10d  <- 0
// r11   <- A+4*k*sizeof(double)
// r12   <- x+k*sizeof(double)
// r13   <- m1
// zmm0  <- []_a
// ...
// zmm3  <- []_d

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRMV_N_LN_8_VS_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_trmv_n_ln_8_vs_lib8)
#endif
	
	// saturate m1 to 8
	cmpl	$ 8, %r13d
	jle		0f
	movl	$ 8, %r13d
0:

	vmovapd			0*64(%r11), %zmm24
	vbroadcastsd	0*8(%r12), %zmm25
	vfmadd231pd		%zmm24, %zmm25, %zmm0
	cmpl	$ 1, %r13d
	jle		0f

	vmovapd			1*64(%r11), %zmm24
	vbroadcastsd	1*8(%r12), %zmm25
	movl			$ 0xfe, %ebx
	kmovd			%ebx, %k1
	vfmadd231pd		%zmm24, %zmm25, %zmm1 {%k1}
	cmpl	$ 1, %r13d
	jle		0f

	vmovapd			2*64(%r11), %zmm24
	vbroadcastsd	2*8(%r12), %zmm25
	movl			$ 0xfc, %ebx
	kmovd			%ebx, %k1
	vfmadd231pd		%zmm24, %zmm25, %zmm2 {%k1}
	cmpl	$ 1, %r13d
	jle		0f

	vmovapd			3*64(%r11), %zmm24
	vbroadcastsd	3*8(%r12), %zmm25
	movl			$ 0xf8, %ebx
	kmovd			%ebx, %k1
	vfmadd231pd		%zmm24, %zmm25, %zmm3 {%k1}
	cmpl	$ 1, %r13d
	jle		0f

	vmovapd			4*64(%r11), %zmm24
	vbroadcastsd	4*8(%r12), %zmm25
	movl			$ 0xf0, %ebx
	kmovd			%ebx, %k1
	vfmadd231pd		%zmm24, %zmm25, %zmm0 {%k1}
	cmpl	$ 1, %r13d
	jle		0f

	vmovapd			5*64(%r11), %zmm24
	vbroadcastsd	5*8(%r12), %zmm25
	movl			$ 0xe0, %ebx
	kmovd			%ebx, %k1
	vfmadd231pd		%zmm24, %zmm25, %zmm1 {%k1}
	cmpl	$ 1, %r13d
	jle		0f

	vmovapd			6*64(%r11), %zmm24
	vbroadcastsd	6*8(%r12), %zmm25
	movl			$ 0xc0, %ebx
	kmovd			%ebx, %k1
	vfmadd231pd		%zmm24, %zmm25, %zmm2 {%k1}
	cmpl	$ 1, %r13d
	jle		0f

	vmovapd			7*64(%r11), %zmm24
	vbroadcastsd	7*8(%r12), %zmm25
	movl			$ 0x80, %ebx
	kmovd			%ebx, %k1
	vfmadd231pd		%zmm24, %zmm25, %zmm3 {%k1}
	cmpl	$ 1, %r13d
	jle		0f

0:

	// no meaning, that's the end
//	subl	$ 8, %r10d
	addq	$ 8*64, %r11
	addq	$ 8*8, %r12
//	leaq	0(%r12, %rbp, 8), %r12
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trmv_n_ln_8_vs_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- x
// r13   <- m1
// r14   <- offsetA
// zmm0  <- []_a
// ...
// zmm3  <- []_d

//
// output arguments:
// r10d  <- 0
// r11   <- A+4*k*sizeof(double)
// r12   <- x+k*sizeof(double)
// r13   <- m1
// r14   <- offsetA
// zmm0  <- []_a
// ...
// zmm3  <- []_d

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRMV_N_LN_8_GEN_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_trmv_n_ln_8_gen_lib8)
#endif
	
	// saturate m1 to 8
//	cmpl	$ 8, %r13d
//	jle		0f
//	movl	$ 8, %r13d
//0:

	// ail = ps-offsetA
	movl	$ 8, %ebx
	subl	%r14d, %ebx

	// min of m1 and ail
	movl	%r13d, %ebp
	cmpl	%ebx, %ebp
	jle		0f
	movl	%ebx, %ebp
0:

	vcvtsi2sd		%ebp, %xmm24, %xmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd			.LC00(%rip), %zmm26
#elif defined(OS_MAC)
	vmovupd			LC00(%rip), %zmm26
#endif
	vbroadcastsd	%xmm24, %zmm24
	vsubpd			%zmm24, %zmm26, %zmm24
	vpmovq2m		%zmm24, %k1

	vmovupd			0*64(%r11, %r14, 8), %zmm24 {%k1}{z}
	vbroadcastsd	0*8(%r12), %zmm25
	vfmadd231pd		%zmm24, %zmm25, %zmm0
	cmpl	$ 1, %ebp
	jle		0f

	vmovupd			1*64(%r11, %r14, 8), %zmm24 {%k1}{z}
	vbroadcastsd	1*8(%r12), %zmm25
	movl			$ 0xfe, %ebx
	kmovd			%ebx, %k1
	vfmadd231pd		%zmm24, %zmm25, %zmm1 {%k1}
	cmpl	$ 2, %ebp
	jle		0f

	vmovupd			2*64(%r11, %r14, 8), %zmm24 {%k1}{z}
	vbroadcastsd	2*8(%r12), %zmm25
	movl			$ 0xfc, %ebx
	kmovd			%ebx, %k1
	vfmadd231pd		%zmm24, %zmm25, %zmm2 {%k1}
	cmpl	$ 3, %ebp
	jle		0f

	vmovupd			3*64(%r11, %r14, 8), %zmm24 {%k1}{z}
	vbroadcastsd	3*8(%r12), %zmm25
	movl			$ 0xf8, %ebx
	kmovd			%ebx, %k1
	vfmadd231pd		%zmm24, %zmm25, %zmm3 {%k1}
	cmpl	$ 4, %ebp
	jle		0f

	vmovupd			4*64(%r11, %r14, 8), %zmm24 {%k1}{z}
	vbroadcastsd	4*8(%r12), %zmm25
	movl			$ 0xf0, %ebx
	kmovd			%ebx, %k1
	vfmadd231pd		%zmm24, %zmm25, %zmm0 {%k1}
	cmpl	$ 5, %ebp
	jle		0f

	vmovupd			5*64(%r11, %r14, 8), %zmm24 {%k1}{z}
	vbroadcastsd	5*8(%r12), %zmm25
	movl			$ 0xe0, %ebx
	kmovd			%ebx, %k1
	vfmadd231pd		%zmm24, %zmm25, %zmm1 {%k1}
	cmpl	$ 6, %ebp
	jle		0f

	vmovupd			6*64(%r11, %r14, 8), %zmm24 {%k1}{z}
	vbroadcastsd	6*8(%r12), %zmm25
	movl			$ 0xc0, %ebx
	kmovd			%ebx, %k1
	vfmadd231pd		%zmm24, %zmm25, %zmm2 {%k1}
	cmpl	$ 7, %ebp
	jle		0f

	vmovupd			7*64(%r11, %r14, 8), %zmm24 {%k1}{z}
	vbroadcastsd	7*8(%r12), %zmm25
	movl			$ 0x80, %ebx
	kmovd			%ebx, %k1
	vfmadd231pd		%zmm24, %zmm25, %zmm3 {%k1}

0:

	// no meaning, that's the end
//	subl	$ 8, %r10d
	addq	$ 8*64, %r11
	addq	$ 8*8, %r12
//	leaq	0(%r12, %rbp, 8), %r12
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trmv_n_ln_8_gen_lib8)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- bs*sda*sizeof(double)
// r13   <- x
// ymm0  <- []
// ...
// ymm7  <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+4*k*sizeof(double)
// r12   <- bs*sda*sizeof(double)
// r13   <- x+k*sizeof(double)
// ymm0  <- []
// ...
// ymm7  <- []

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRMV_T_LN_8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_trmv_t_ln_8_lib8)
#endif

	vmovupd		0(%r13), %zmm25

	vmovapd		0*64(%r11), %zmm24
	vfmadd231pd	%zmm24, %zmm25, %zmm0

	vmovapd		1*64(%r11), %zmm24
	movl		$ 0xfe, %r14d
	kmovd		%r14d, %k1
	vfmadd231pd	%zmm24, %zmm25, %zmm1 {%k1}

	vmovapd		2*64(%r11), %zmm24
	movl		$ 0xfc, %r14d
	kmovd		%r14d, %k1
	vfmadd231pd	%zmm24, %zmm25, %zmm2 {%k1}

	vmovapd		3*64(%r11), %zmm24
	movl		$ 0xf8, %r14d
	kmovd		%r14d, %k1
	vfmadd231pd	%zmm24, %zmm25, %zmm3 {%k1}

	vmovapd		4*64(%r11), %zmm24
	movl		$ 0xf0, %r14d
	kmovd		%r14d, %k1
	vfmadd231pd	%zmm24, %zmm25, %zmm4 {%k1}

	vmovapd		5*64(%r11), %zmm24
	movl		$ 0xe0, %r14d
	kmovd		%r14d, %k1
	vfmadd231pd	%zmm24, %zmm25, %zmm5 {%k1}

	vmovapd		6*64(%r11), %zmm24
	movl		$ 0xc0, %r14d
	kmovd		%r14d, %k1
	vfmadd231pd	%zmm24, %zmm25, %zmm6 {%k1}

	vmovapd		7*64(%r11), %zmm24
	movl		$ 0x80, %r14d
	kmovd		%r14d, %k1
	vfmadd231pd	%zmm24, %zmm25, %zmm7 {%k1}

//	subl	$ 8, %r10d
	addq	%r12, %r11
	addq	$ 64, %r13
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trmv_t_ln_8_lib8)
#endif




// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- bs*sda*sizeof(double)
// r13   <- x
// r14   <- m1
// ymm0  <- []
// ...
// ymm7  <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+4*k*sizeof(double)
// r12   <- bs*sda*sizeof(double)
// r13   <- x+k*sizeof(double)
// r14   <- m1
// ymm0  <- []
// ...
// ymm7  <- []

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRMV_T_LN_8_VS_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_trmv_t_ln_8_vs_lib8)
#endif

	// saturate m1 to 8
	cmpl	$ 8, %r14d
	jle		0f
	movl	$ 8, %r14d
0:

	// k + m1
	leal	0(%r10d, %r14d), %ebp
	cmpl	$ 8, %ebp
	jle		0f
	movl	$ 8, %ebp
0:

	vcvtsi2sd		%ebp, %xmm25, %xmm25
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd			.LC00(%rip), %zmm24
#elif defined(OS_MAC)
	vmovupd			LC00(%rip), %zmm24
#endif
	vbroadcastsd	%xmm25, %zmm25
	vsubpd			%zmm25, %zmm24, %zmm25
	vpmovq2m		%zmm25, %k2

	vmovupd		0(%r13), %zmm25 {%k2}{z}

	vmovapd		0*64(%r11), %zmm24 {%k2}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm0
	cmpl	$ 1, %r14d
	jle		0f

	vmovapd		1*64(%r11), %zmm24 {%k2}{z}
	movl		$ 0xfe, %r15d
	kmovd		%r15d, %k1
	vfmadd231pd	%zmm24, %zmm25, %zmm1 {%k1}
	cmpl	$ 2, %r14d
	jle		0f

	vmovapd		2*64(%r11), %zmm24 {%k2}{z}
	movl		$ 0xfc, %r15d
	kmovd		%r15d, %k1
	vfmadd231pd	%zmm24, %zmm25, %zmm2 {%k1}
	cmpl	$ 3, %r14d
	jle		0f

	vmovapd		3*64(%r11), %zmm24 {%k2}{z}
	movl		$ 0xf8, %r15d
	kmovd		%r15d, %k1
	vfmadd231pd	%zmm24, %zmm25, %zmm3 {%k1}
	cmpl	$ 4, %r14d
	jle		0f

	vmovapd		4*64(%r11), %zmm24 {%k2}{z}
	movl		$ 0xf0, %r15d
	kmovd		%r15d, %k1
	vfmadd231pd	%zmm24, %zmm25, %zmm4 {%k1}
	cmpl	$ 5, %r14d
	jle		0f

	vmovapd		5*64(%r11), %zmm24 {%k2}{z}
	movl		$ 0xe0, %r15d
	kmovd		%r15d, %k1
	vfmadd231pd	%zmm24, %zmm25, %zmm5 {%k1}
	cmpl	$ 6, %r14d
	jle		0f

	vmovapd		6*64(%r11), %zmm24 {%k2}{z}
	movl		$ 0xc0, %r15d
	kmovd		%r15d, %k1
	vfmadd231pd	%zmm24, %zmm25, %zmm6 {%k1}
	cmpl	$ 7, %r14d
	jle		0f

	vmovapd		7*64(%r11), %zmm24 {%k2}{z}
	movl		$ 0x80, %r15d
	kmovd		%r15d, %k1
	vfmadd231pd	%zmm24, %zmm25, %zmm7 {%k1}

0:

	// k - (ebp - m1)
	subl	%ebp, %r10d
	addl	%r14d, %r10d

	addq	%r12, %r11
	leaq	0(%r13, %rbp, 8), %r13

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trmv_t_ln_8_vs_lib8)
#endif




// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- bs*sda*sizeof(double)
// r13   <- x
// r14   <- m1
// r15   <- offsetA
// ymm0  <- []
// ...
// ymm7  <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+4*k*sizeof(double)
// r12   <- bs*sda*sizeof(double)
// r13   <- x+k*sizeof(double)
// r14   <- m1
// r15   <- offsetA
// ymm0  <- []
// ...
// ymm7  <- []

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRMV_T_LN_8_GEN_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_trmv_t_ln_8_gen_lib8)
#endif

	// saturate m1 to 8
	cmpl	$ 8, %r14d
	jle		0f
	movl	$ 8, %r14d
0:

	// k + m1
	leal	0(%r10d, %r14d), %ebp
	cmpl	$ 8, %ebp
	jle		0f
	movl	$ 8, %ebp
0:

	// ail = ps-offsetA
	movl	$ 8, %ebx
	subl	%r15d, %ebx

	// min of m1 and ail
	cmpl	%ebx, %ebp
	jle		0f
	movl	%ebx, %ebp
0:

	vcvtsi2sd		%ebp, %xmm25, %xmm25
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd			.LC00(%rip), %zmm24
#elif defined(OS_MAC)
	vmovupd			LC00(%rip), %zmm24
#endif
	vbroadcastsd	%xmm25, %zmm25
	vsubpd			%zmm25, %zmm24, %zmm25
	vpmovq2m		%zmm25, %k2

	vmovupd		0(%r13), %zmm25 {%k2}{z}

	vmovupd		0*64(%r11, %r15, 8), %zmm24 {%k2}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm0
	cmpl	$ 1, %r14d
	jle		0f

	vmovupd		1*64(%r11, %r15, 8), %zmm24 {%k2}{z}
	movl		$ 0xfe, %ebx
	kmovd		%ebx, %k1
	vfmadd231pd	%zmm24, %zmm25, %zmm1 {%k1}
	cmpl	$ 2, %r14d
	jle		0f

	vmovupd		2*64(%r11, %r15, 8), %zmm24 {%k2}{z}
	movl		$ 0xfc, %ebx
	kmovd		%ebx, %k1
	vfmadd231pd	%zmm24, %zmm25, %zmm2 {%k1}
	cmpl	$ 3, %r14d
	jle		0f

	vmovupd		3*64(%r11, %r15, 8), %zmm24 {%k2}{z}
	movl		$ 0xf8, %ebx
	kmovd		%ebx, %k1
	vfmadd231pd	%zmm24, %zmm25, %zmm3 {%k1}
	cmpl	$ 4, %r14d
	jle		0f

	vmovupd		4*64(%r11, %r15, 8), %zmm24 {%k2}{z}
	movl		$ 0xf0, %ebx
	kmovd		%ebx, %k1
	vfmadd231pd	%zmm24, %zmm25, %zmm4 {%k1}
	cmpl	$ 5, %r14d
	jle		0f

	vmovupd		5*64(%r11, %r15, 8), %zmm24 {%k2}{z}
	movl		$ 0xe0, %ebx
	kmovd		%ebx, %k1
	vfmadd231pd	%zmm24, %zmm25, %zmm5 {%k1}
	cmpl	$ 6, %r14d
	jle		0f

	vmovupd		6*64(%r11, %r15, 8), %zmm24 {%k2}{z}
	movl		$ 0xc0, %ebx
	kmovd		%ebx, %k1
	vfmadd231pd	%zmm24, %zmm25, %zmm6 {%k1}
	cmpl	$ 7, %r14d
	jle		0f

	vmovupd		7*64(%r11, %r15, 8), %zmm24 {%k2}{z}
	movl		$ 0x80, %ebx
	kmovd		%ebx, %k1
	vfmadd231pd	%zmm24, %zmm25, %zmm7 {%k1}

0:

	// k - (ebp - m1)
	subl	%ebp, %r10d
	addl	%r14d, %r10d

	addq	%r12, %r11
	leaq	0(%r13, %rbp, 8), %r13

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trmv_t_ln_8_gen_lib8)
#endif




// common inner routine with file scope
//
// triangular substitution with vector RHS
//
// input arguments:
// r10  <- E
// r11  <- inv_diag_E
// zmm0 <- []
//
// output arguments:
// r10  <- E
// r11  <- inv_diag_E
// zmm0 <- []

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSV_N_L_INV_8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsv_n_l_inv_8_lib8)
#endif
	
	vbroadcastsd	0(%r11), %zmm24
	movl			$ 0x01, %r12d
	kmovd			%r12d, %k1
	vmulpd			%zmm0, %zmm24, %zmm0 {%k1}

	vbroadcastsd	%xmm0, %zmm25
	movl			$ 0xfe, %r12d
	kmovd			%r12d, %k1
	vmovapd			0(%r10), %zmm24 {%k1}{z}
	vfnmadd231pd	%zmm24, %zmm25, %zmm0 {%k1}
	vbroadcastsd	1*8(%r11), %zmm24
	movl			$ 0x02, %r12d
	kmovd			%r12d, %k1
	vmulpd			%zmm0, %zmm24, %zmm0 {%k1}

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm25
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm25
#endif
	vpermpd			%zmm0, %zmm25, %zmm25
	movl			$ 0xfc, %r12d
	kmovd			%r12d, %k1
	vmovapd			1*64(%r10), %zmm24 {%k1}{z}
	vfnmadd231pd	%zmm24, %zmm25, %zmm0 {%k1}
	vbroadcastsd	2*8(%r11), %zmm24
	movl			$ 0x04, %r12d
	kmovd			%r12d, %k1
	vmulpd			%zmm0, %zmm24, %zmm0 {%k1}

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm25
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm25
#endif
	vpermpd			%zmm0, %zmm25, %zmm25
	movl			$ 0xf8, %r12d
	kmovd			%r12d, %k1
	vmovapd			2*64(%r10), %zmm24 {%k1}{z}
	vfnmadd231pd	%zmm24, %zmm25, %zmm0 {%k1}
	vbroadcastsd	3*8(%r11), %zmm24
	movl			$ 0x08, %r12d
	kmovd			%r12d, %k1
	vmulpd			%zmm0, %zmm24, %zmm0 {%k1}

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm25
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm25
#endif
	vpermpd			%zmm0, %zmm25, %zmm25
	movl			$ 0xf0, %r12d
	kmovd			%r12d, %k1
	vmovapd			3*64(%r10), %zmm24 {%k1}{z}
	vfnmadd231pd	%zmm24, %zmm25, %zmm0 {%k1}
	vbroadcastsd	4*8(%r11), %zmm24
	movl			$ 0x10, %r12d
	kmovd			%r12d, %k1
	vmulpd			%zmm0, %zmm24, %zmm0 {%k1}

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm25
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm25
#endif
	vpermpd			%zmm0, %zmm25, %zmm25
	movl			$ 0xe0, %r12d
	kmovd			%r12d, %k1
	vmovapd			4*64(%r10), %zmm24 {%k1}{z}
	vfnmadd231pd	%zmm24, %zmm25, %zmm0 {%k1}
	vbroadcastsd	5*8(%r11), %zmm24
	movl			$ 0x20, %r12d
	kmovd			%r12d, %k1
	vmulpd			%zmm0, %zmm24, %zmm0 {%k1}

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm25
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm25
#endif
	vpermpd			%zmm0, %zmm25, %zmm25
	movl			$ 0xc0, %r12d
	kmovd			%r12d, %k1
	vmovapd			5*64(%r10), %zmm24 {%k1}{z}
	vfnmadd231pd	%zmm24, %zmm25, %zmm0 {%k1}
	vbroadcastsd	6*8(%r11), %zmm24
	movl			$ 0x40, %r12d
	kmovd			%r12d, %k1
	vmulpd			%zmm0, %zmm24, %zmm0 {%k1}

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm25
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm25
#endif
	vpermpd			%zmm0, %zmm25, %zmm25
	movl			$ 0x80, %r12d
	kmovd			%r12d, %k1
	vmovapd			6*64(%r10), %zmm24 {%k1}{z}
	vfnmadd231pd	%zmm24, %zmm25, %zmm0 {%k1}
	vbroadcastsd	7*8(%r11), %zmm24
	movl			$ 0x80, %r12d
	kmovd			%r12d, %k1
	vmulpd			%zmm0, %zmm24, %zmm0 {%k1}

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsv_n_l_inv_8_lib8)
#endif





// common inner routine with file scope
//
// triangular substitution with vector RHS
//
// input arguments:
// r10  <- E
// r11  <- inv_diag_E
// r12  <- n1
// zmm0 <- []
//
// output arguments:
// r10  <- E
// r11  <- inv_diag_E
// r12  <- n1
// zmm0 <- []

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSV_N_L_INV_8_VS_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsv_n_l_inv_8_vs_lib8)
#endif
	
	vbroadcastsd	0(%r11), %zmm24
	movl			$ 0x01, %r13d
	kmovd			%r13d, %k1
	vmulpd			%zmm0, %zmm24, %zmm0 {%k1}
	vbroadcastsd	%xmm0, %zmm25
	movl			$ 0xfe, %r13d
	kmovd			%r13d, %k1
	vmovapd			0(%r10), %zmm24 {%k1}{z}
	vfnmadd231pd	%zmm24, %zmm25, %zmm0 {%k1}

	cmpl			$ 2, %r12d
	jl				0f // ret

	vbroadcastsd	1*8(%r11), %zmm24
	movl			$ 0x02, %r13d
	kmovd			%r13d, %k1
	vmulpd			%zmm0, %zmm24, %zmm0 {%k1}
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm25
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm25
#endif
	vpermpd			%zmm0, %zmm25, %zmm25
	movl			$ 0xfc, %r13d
	kmovd			%r13d, %k1
	vmovapd			1*64(%r10), %zmm24 {%k1}{z}
	vfnmadd231pd	%zmm24, %zmm25, %zmm0 {%k1}

	cmpl			$ 3, %r12d
	jl				0f // ret

	vbroadcastsd	2*8(%r11), %zmm24
	movl			$ 0x04, %r13d
	kmovd			%r13d, %k1
	vmulpd			%zmm0, %zmm24, %zmm0 {%k1}
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm25
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm25
#endif
	vpermpd			%zmm0, %zmm25, %zmm25
	movl			$ 0xf8, %r13d
	kmovd			%r13d, %k1
	vmovapd			2*64(%r10), %zmm24 {%k1}{z}
	vfnmadd231pd	%zmm24, %zmm25, %zmm0 {%k1}

	cmpl			$ 4, %r12d
	jl				0f // ret

	vbroadcastsd	3*8(%r11), %zmm24
	movl			$ 0x08, %r13d
	kmovd			%r13d, %k1
	vmulpd			%zmm0, %zmm24, %zmm0 {%k1}
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm25
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm25
#endif
	vpermpd			%zmm0, %zmm25, %zmm25
	movl			$ 0xf0, %r13d
	kmovd			%r13d, %k1
	vmovapd			3*64(%r10), %zmm24 {%k1}{z}
	vfnmadd231pd	%zmm24, %zmm25, %zmm0 {%k1}

	cmpl			$ 5, %r12d
	jl				0f // ret

	vbroadcastsd	4*8(%r11), %zmm24
	movl			$ 0x10, %r13d
	kmovd			%r13d, %k1
	vmulpd			%zmm0, %zmm24, %zmm0 {%k1}
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm25
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm25
#endif
	vpermpd			%zmm0, %zmm25, %zmm25
	movl			$ 0xe0, %r13d
	kmovd			%r13d, %k1
	vmovapd			4*64(%r10), %zmm24 {%k1}{z}
	vfnmadd231pd	%zmm24, %zmm25, %zmm0 {%k1}

	cmpl			$ 6, %r12d
	jl				0f // ret

	vbroadcastsd	5*8(%r11), %zmm24
	movl			$ 0x20, %r13d
	kmovd			%r13d, %k1
	vmulpd			%zmm0, %zmm24, %zmm0 {%k1}
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm25
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm25
#endif
	vpermpd			%zmm0, %zmm25, %zmm25
	movl			$ 0xc0, %r13d
	kmovd			%r13d, %k1
	vmovapd			5*64(%r10), %zmm24 {%k1}{z}
	vfnmadd231pd	%zmm24, %zmm25, %zmm0 {%k1}

	cmpl			$ 7, %r12d
	jl				0f // ret

	vbroadcastsd	6*8(%r11), %zmm24
	movl			$ 0x40, %r13d
	kmovd			%r13d, %k1
	vmulpd			%zmm0, %zmm24, %zmm0 {%k1}
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm25
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm25
#endif
	vpermpd			%zmm0, %zmm25, %zmm25
	movl			$ 0x80, %r13d
	kmovd			%r13d, %k1
	vmovapd			6*64(%r10), %zmm24 {%k1}{z}
	vfnmadd231pd	%zmm24, %zmm25, %zmm0 {%k1}

	cmpl			$ 8, %r12d
	jl				0f // ret

	vbroadcastsd	7*8(%r11), %zmm24
	movl			$ 0x80, %r13d
	kmovd			%r13d, %k1
	vmulpd			%zmm0, %zmm24, %zmm0 {%k1}

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsv_n_l_inv_8_vs_lib8)
#endif





// common inner routine with file scope
//
// triangular substitution with vector RHS
//
// input arguments:
// r10  <- E
// r11  <- inv_diag_E
// zmm0 <- []
//
// output arguments:
// r10  <- E
// r11  <- inv_diag_E
// zmm0 <- []

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSV_T_L_INV_8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsv_t_l_inv_8_lib8)
#endif
	
	// triangle transpose
	movl	$ 0xcc, %r15d
	kmovd	%r15d, %k1
	movl	$ 0x33, %r15d
	kmovd	%r15d, %k2

	// TODO mask already here the (strict) lower triangle ???
	vmovapd			0+0*64(%r10), %ymm16
//	vmovapd			0+4*64(%r10), %ymm24 //
//	vinsertf64x4	$ 0x1, %ymm24, %zmm16, %zmm16 //
	vmovapd			0+1*64(%r10), %ymm17
//	vmovapd			0+5*64(%r10), %ymm24 //
//	vinsertf64x4	$ 0x1, %ymm24, %zmm17, %zmm17 //
	vmovapd			0+2*64(%r10), %ymm18
//	vmovapd			0+6*64(%r10), %ymm24 //
//	vinsertf64x4	$ 0x1, %ymm24, %zmm18, %zmm18 //
//	vmovapd			0+3*64(%r10), %ymm19 //
	vxorpd			%zmm19, %zmm19, %zmm19
//	vmovapd			0+7*64(%r10), %ymm24 //
//	vinsertf64x4	$ 0x1, %ymm24, %zmm19, %zmm19 //
	vmovapd			32+0*64(%r10), %ymm20
	vmovapd			32+4*64(%r10), %ymm24
	vinsertf64x4	$ 0x1, %ymm24, %zmm20, %zmm20
	vmovapd			32+1*64(%r10), %ymm21
	vmovapd			32+5*64(%r10), %ymm24
	vinsertf64x4	$ 0x1, %ymm24, %zmm21, %zmm21
	vmovapd			32+2*64(%r10), %ymm22
	vmovapd			32+6*64(%r10), %ymm24
	vinsertf64x4	$ 0x1, %ymm24, %zmm22, %zmm22
	vmovapd			32+3*64(%r10), %ymm23
//	vmovapd			32+7*64(%r10), %ymm24 //
//	vinsertf64x4	$ 0x1, %ymm24, %zmm23, %zmm23 //

	vmovapd		%zmm16, %zmm24
	vpermq		$ 0x40, %zmm18, %zmm16 {%k1}
	vpermq		$ 0x0e, %zmm24, %zmm18 {%k2}
	vmovapd		%zmm17, %zmm24
	vpermq		$ 0x40, %zmm19, %zmm17 {%k1}
	vpermq		$ 0x0e, %zmm24, %zmm19 {%k2}
	vmovapd		%zmm20, %zmm24
	vpermq		$ 0x40, %zmm22, %zmm20 {%k1}
	vpermq		$ 0x0e, %zmm24, %zmm22 {%k2}
	vmovapd		%zmm21, %zmm24
	vpermq		$ 0x40, %zmm23, %zmm21 {%k1}
	vpermq		$ 0x0e, %zmm24, %zmm23 {%k2}

//	vunpcklpd	%zmm17, %zmm16, %zmm24 //
	vunpckhpd	%zmm17, %zmm16, %zmm17
//	vmovapd		%zmm24, %zmm16 //
	vunpcklpd	%zmm19, %zmm18, %zmm24
	vunpckhpd	%zmm19, %zmm18, %zmm19
	vmovapd		%zmm24, %zmm18
	vunpcklpd	%zmm21, %zmm20, %zmm24
	vunpckhpd	%zmm21, %zmm20, %zmm21
	vmovapd		%zmm24, %zmm20
	vunpcklpd	%zmm23, %zmm22, %zmm24
	vunpckhpd	%zmm23, %zmm22, %zmm23
	vmovapd		%zmm24, %zmm22

//vmovapd		%zmm16, %zmm0
//jmp 0f

	// triangle solve

	vbroadcastsd	7*8(%r11), %zmm24
	movl			$ 0x80, %r12d
	kmovd			%r12d, %k1
	vmulpd			%zmm0, %zmm24, %zmm0 {%k1}

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+7*8(%rip), %zmm25
#elif defined(OS_MAC)
	vbroadcastsd	LC05+7*8(%rip), %zmm25
#endif
	vpermpd			%zmm0, %zmm25, %zmm25
	movl			$ 0x7f, %r12d
	kmovd			%r12d, %k1
	vfnmadd231pd	%zmm23, %zmm25, %zmm0 {%k1}
	vbroadcastsd	6*8(%r11), %zmm24
	movl			$ 0x40, %r12d
	kmovd			%r12d, %k1
	vmulpd			%zmm0, %zmm24, %zmm0 {%k1}

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm25
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm25
#endif
	vpermpd			%zmm0, %zmm25, %zmm25
	movl			$ 0x3f, %r12d
	kmovd			%r12d, %k1
	vfnmadd231pd	%zmm22, %zmm25, %zmm0 {%k1}
	vbroadcastsd	5*8(%r11), %zmm24
	movl			$ 0x20, %r12d
	kmovd			%r12d, %k1
	vmulpd			%zmm0, %zmm24, %zmm0 {%k1}

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm25
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm25
#endif
	vpermpd			%zmm0, %zmm25, %zmm25
	movl			$ 0x1f, %r12d
	kmovd			%r12d, %k1
	vfnmadd231pd	%zmm21, %zmm25, %zmm0 {%k1}
	vbroadcastsd	4*8(%r11), %zmm24
	movl			$ 0x10, %r12d
	kmovd			%r12d, %k1
	vmulpd			%zmm0, %zmm24, %zmm0 {%k1}

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm25
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm25
#endif
	vpermpd			%zmm0, %zmm25, %zmm25
	movl			$ 0x0f, %r12d
	kmovd			%r12d, %k1
	vfnmadd231pd	%zmm20, %zmm25, %zmm0 {%k1}
	vbroadcastsd	3*8(%r11), %zmm24
	movl			$ 0x08, %r12d
	kmovd			%r12d, %k1
	vmulpd			%zmm0, %zmm24, %zmm0 {%k1}

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm25
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm25
#endif
	vpermpd			%zmm0, %zmm25, %zmm25
	movl			$ 0x07, %r12d
	kmovd			%r12d, %k1
	vfnmadd231pd	%zmm19, %zmm25, %zmm0 {%k1}
	vbroadcastsd	2*8(%r11), %zmm24
	movl			$ 0x04, %r12d
	kmovd			%r12d, %k1
	vmulpd			%zmm0, %zmm24, %zmm0 {%k1}

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm25
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm25
#endif
	vpermpd			%zmm0, %zmm25, %zmm25
	movl			$ 0x03, %r12d
	kmovd			%r12d, %k1
	vfnmadd231pd	%zmm18, %zmm25, %zmm0 {%k1}
	vbroadcastsd	1*8(%r11), %zmm24
	movl			$ 0x02, %r12d
	kmovd			%r12d, %k1
	vmulpd			%zmm0, %zmm24, %zmm0 {%k1}

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm25
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm25
#endif
	vpermpd			%zmm0, %zmm25, %zmm25
	movl			$ 0x01, %r12d
	kmovd			%r12d, %k1
	vfnmadd231pd	%zmm17, %zmm25, %zmm0 {%k1}
	vbroadcastsd	0*8(%r11), %zmm24
	movl			$ 0x01, %r12d
	kmovd			%r12d, %k1
	vmulpd			%zmm0, %zmm24, %zmm0 {%k1}

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsv_t_l_inv_8_lib8)
#endif





// common inner routine with file scope
//
// triangular substitution with vector RHS
//
// input arguments:
// r10  <- E
// r11  <- inv_diag_E
// r12  <- m1
// r13  <- n1
// zmm0 <- []
//
// output arguments:
// r10  <- E
// r11  <- inv_diag_E
// r12  <- m1
// r13  <- n1
// zmm0 <- []

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSV_T_L_INV_8_VS_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsv_t_l_inv_8_vs_lib8)
#endif
	
	// compute mask
	vcvtsi2sd		%r13d, %xmm25, %xmm25
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd			.LC00(%rip), %zmm24
#elif defined(OS_MAC)
	vmovupd			LC00(%rip), %zmm24
#endif
	vbroadcastsd	%xmm25, %zmm25
	vsubpd			%zmm25, %zmm24, %zmm25
	vpmovq2m		%zmm25, %k3

	// triangle transpose
	movl	$ 0xcc, %r15d
	kmovd	%r15d, %k1
	movl	$ 0x33, %r15d
	kmovd	%r15d, %k2

	// TODO mask already here the (strict) lower triangle ???
	// TODO only load needed lanes ???
	//
	vmovapd			0+0*64(%r10), %ymm16
	vmovapd			32+0*64(%r10), %ymm20
	//
	vmovapd			0+1*64(%r10), %ymm17
	vmovapd			32+1*64(%r10), %ymm21
	//
	vmovapd			0+2*64(%r10), %ymm18
	vmovapd			32+2*64(%r10), %ymm22
	//
//	vmovapd			0+3*64(%r10), %ymm19 //
	vxorpd			%zmm19, %zmm19, %zmm19
	vmovapd			32+3*64(%r10), %ymm23
	//
	cmpl			$ 5, %r13d
	jl				1f
//	vmovapd			0+4*64(%r10), %ymm24 //
//	vinsertf64x4	$ 0x1, %ymm24, %zmm16, %zmm16 //
	vmovapd			32+4*64(%r10), %ymm24
	vinsertf64x4	$ 0x1, %ymm24, %zmm20, %zmm20
	//
//	vmovapd			0+5*64(%r10), %ymm24 //
//	vinsertf64x4	$ 0x1, %ymm24, %zmm17, %zmm17 //
	vmovapd			32+5*64(%r10), %ymm24
	vinsertf64x4	$ 0x1, %ymm24, %zmm21, %zmm21
	//
//	vmovapd			0+6*64(%r10), %ymm24 //
//	vinsertf64x4	$ 0x1, %ymm24, %zmm18, %zmm18 //
	vmovapd			32+6*64(%r10), %ymm24
	vinsertf64x4	$ 0x1, %ymm24, %zmm22, %zmm22
	//
//	vmovapd			0+7*64(%r10), %ymm24 //
//	vinsertf64x4	$ 0x1, %ymm24, %zmm19, %zmm19 //
//	vmovapd			32+7*64(%r10), %ymm24 //
//	vinsertf64x4	$ 0x1, %ymm24, %zmm23, %zmm23 //
1:

	vmovapd		%zmm16, %zmm24
	vpermq		$ 0x40, %zmm18, %zmm16 {%k1}
	vpermq		$ 0x0e, %zmm24, %zmm18 {%k2}
	vmovapd		%zmm17, %zmm24
	vpermq		$ 0x40, %zmm19, %zmm17 {%k1}
	vpermq		$ 0x0e, %zmm24, %zmm19 {%k2}

//	vunpcklpd	%zmm17, %zmm16, %zmm24 //
	vunpckhpd	%zmm17, %zmm16, %zmm17
//	vmovapd		%zmm24, %zmm16 //
	vunpcklpd	%zmm19, %zmm18, %zmm24
	vunpckhpd	%zmm19, %zmm18, %zmm19
	vmovapd		%zmm24, %zmm18

	cmpl		$ 5, %r12d
//	jl			1f
	jl			2f

	vmovapd		%zmm20, %zmm24
	vpermq		$ 0x40, %zmm22, %zmm20 {%k1}
	vpermq		$ 0x0e, %zmm24, %zmm22 {%k2}
	vmovapd		%zmm21, %zmm24
	vpermq		$ 0x40, %zmm23, %zmm21 {%k1}
	vpermq		$ 0x0e, %zmm24, %zmm23 {%k2}

	vunpcklpd	%zmm21, %zmm20, %zmm24
	vunpckhpd	%zmm21, %zmm20, %zmm21
	vmovapd		%zmm24, %zmm20
	vunpcklpd	%zmm23, %zmm22, %zmm24
	vunpckhpd	%zmm23, %zmm22, %zmm23
	vmovapd		%zmm24, %zmm22
1:


	// triangle solve

	cmpl			$ 8, %r12d
	jl				1f

	vbroadcastsd	7*8(%r11), %zmm24
	movl			$ 0x80, %r15d
	kmovd			%r15d, %k1
	kandd			%k3, %k1, %k1
	vmulpd			%zmm0, %zmm24, %zmm0 {%k1}
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+7*8(%rip), %zmm25
#elif defined(OS_MAC)
	vbroadcastsd	LC05+7*8(%rip), %zmm25
#endif
	vpermpd			%zmm0, %zmm25, %zmm25
	movl			$ 0x7f, %r15d
	kmovd			%r15d, %k1
	kandd			%k3, %k1, %k1
	vfnmadd231pd	%zmm23, %zmm25, %zmm0 {%k1}

1:
	cmpl			$ 7, %r12d
	jl				1f

	vbroadcastsd	6*8(%r11), %zmm24
	movl			$ 0x40, %r15d
	kmovd			%r15d, %k1
	kandd			%k3, %k1, %k1
	vmulpd			%zmm0, %zmm24, %zmm0 {%k1}
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm25
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm25
#endif
	vpermpd			%zmm0, %zmm25, %zmm25
	movl			$ 0x3f, %r15d
	kmovd			%r15d, %k1
	kandd			%k3, %k1, %k1
	vfnmadd231pd	%zmm22, %zmm25, %zmm0 {%k1}

1:
	cmpl			$ 6, %r12d
	jl				1f

	vbroadcastsd	5*8(%r11), %zmm24
	movl			$ 0x20, %r15d
	kmovd			%r15d, %k1
	kandd			%k3, %k1, %k1
	vmulpd			%zmm0, %zmm24, %zmm0 {%k1}
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm25
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm25
#endif
	vpermpd			%zmm0, %zmm25, %zmm25
	movl			$ 0x1f, %r15d
	kmovd			%r15d, %k1
	kandd			%k3, %k1, %k1
	vfnmadd231pd	%zmm21, %zmm25, %zmm0 {%k1}

1:
	cmpl			$ 5, %r12d
	jl				1f

	vbroadcastsd	4*8(%r11), %zmm24
	movl			$ 0x10, %r15d
	kmovd			%r15d, %k1
	kandd			%k3, %k1, %k1
	vmulpd			%zmm0, %zmm24, %zmm0 {%k1}
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm25
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm25
#endif
	vpermpd			%zmm0, %zmm25, %zmm25
	movl			$ 0x0f, %r15d
	kmovd			%r15d, %k1
	kandd			%k3, %k1, %k1
	vfnmadd231pd	%zmm20, %zmm25, %zmm0 {%k1}

1:
2:
	cmpl			$ 4, %r12d
	jl				1f

	vbroadcastsd	3*8(%r11), %zmm24
	movl			$ 0x08, %r15d
	kmovd			%r15d, %k1
	kandd			%k3, %k1, %k1
	vmulpd			%zmm0, %zmm24, %zmm0 {%k1}
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm25
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm25
#endif
	vpermpd			%zmm0, %zmm25, %zmm25
	movl			$ 0x07, %r15d
	kmovd			%r15d, %k1
	kandd			%k3, %k1, %k1
	vfnmadd231pd	%zmm19, %zmm25, %zmm0 {%k1}

1:
	cmpl			$ 3, %r12d
	jl				1f

	vbroadcastsd	2*8(%r11), %zmm24
	movl			$ 0x04, %r15d
	kmovd			%r15d, %k1
	kandd			%k3, %k1, %k1
	vmulpd			%zmm0, %zmm24, %zmm0 {%k1}
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm25
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm25
#endif
	vpermpd			%zmm0, %zmm25, %zmm25
	movl			$ 0x03, %r15d
	kmovd			%r15d, %k1
	kandd			%k3, %k1, %k1
	vfnmadd231pd	%zmm18, %zmm25, %zmm0 {%k1}

1:
	cmpl			$ 2, %r12d
	jl				1f

	vbroadcastsd	1*8(%r11), %zmm24
	movl			$ 0x02, %r15d
	kmovd			%r15d, %k1
	kandd			%k3, %k1, %k1
	vmulpd			%zmm0, %zmm24, %zmm0 {%k1}
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm25
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm25
#endif
	vpermpd			%zmm0, %zmm25, %zmm25
	movl			$ 0x01, %r15d
	kmovd			%r15d, %k1
	kandd			%k3, %k1, %k1
	vfnmadd231pd	%zmm17, %zmm25, %zmm0 {%k1}

1:
//	cmpl			$ 1, %r12d
//	jl				1f

	vbroadcastsd	0*8(%r11), %zmm24
	movl			$ 0x01, %r15d
	kmovd			%r15d, %k1
	kandd			%k3, %k1, %k1
	vmulpd			%zmm0, %zmm24, %zmm0 {%k1}
//1:

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsv_t_l_inv_8_vs_lib8)
#endif





// common inner routine with file scope
//
// blend for ta==n
//
// input arguments:
// zmm0 <- []_a
// ...
// zmm3 <- []_d
//
// output arguments:
// zmm0 <-

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_N_8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_blend_n_8_lib8)
#endif

	// reduction
	vaddpd			%zmm0, %zmm1, %zmm0
	vaddpd			%zmm2, %zmm3, %zmm2
	vaddpd			%zmm0, %zmm2, %zmm0

#if MACRO_LEVEL>=1
	.endm
#else
	ret
	
	FUN_END(inner_blend_n_8_lib8)
#endif






// common inner routine with file scope
//
// blend for ta==t
//
// input arguments:
// zmm0 <- []_a
// ...
// zmm3 <- []_d
//
// output arguments:
// zmm0 <-

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_T_8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_blend_t_8_lib8)
#endif

	// reduction
	vextractf64x4	$ 0x1, %zmm0, %ymm24
	vaddpd			%ymm24, %ymm0, %ymm0
	vextractf64x4	$ 0x1, %zmm1, %ymm24
	vaddpd			%ymm24, %ymm1, %ymm1
	vextractf64x4	$ 0x1, %zmm2, %ymm24
	vaddpd			%ymm24, %ymm2, %ymm2
	vextractf64x4	$ 0x1, %zmm3, %ymm24
	vaddpd			%ymm24, %ymm3, %ymm3
	vextractf64x4	$ 0x1, %zmm4, %ymm24
	vaddpd			%ymm24, %ymm4, %ymm4
	vextractf64x4	$ 0x1, %zmm5, %ymm24
	vaddpd			%ymm24, %ymm5, %ymm5
	vextractf64x4	$ 0x1, %zmm6, %ymm24
	vaddpd			%ymm24, %ymm6, %ymm6
	vextractf64x4	$ 0x1, %zmm7, %ymm24
	vaddpd			%ymm24, %ymm7, %ymm7

	vhaddpd			%ymm1, %ymm0, %ymm12
	vhaddpd			%ymm3, %ymm2, %ymm13
	vhaddpd			%ymm5, %ymm4, %ymm14
	vhaddpd			%ymm7, %ymm6, %ymm15

	vperm2f128		$ 0x02, %ymm12, %ymm13, %ymm1
	vperm2f128		$ 0x13, %ymm12, %ymm13, %ymm0
	vaddpd			%ymm0, %ymm1, %ymm0
	vperm2f128		$ 0x02, %ymm14, %ymm15, %ymm3
	vperm2f128		$ 0x13, %ymm14, %ymm15, %ymm2
	vaddpd			%ymm2, %ymm3, %ymm2

	vinsertf64x4	$ 0x1, %ymm2, %zmm0, %zmm0

//	vshuff64x2		$ 0x

#if MACRO_LEVEL>=1
	.endm
#else
	ret
	
	FUN_END(inner_blend_t_8_lib8)
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
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_8_lib8)
#endif

	// alpha
	vbroadcastsd	0(%r10), %zmm24
	vmulpd			%zmm0, %zmm24, %zmm0

	// beta
	vbroadcastsd	0(%r11), %zmm25

	vxorpd		%zmm24, %zmm24, %zmm24 // 0.0
	vucomisd	%xmm24, %xmm25 // beta==0.0 ?
	je			0f // end

	vmovupd			0(%r12), %zmm24
	vfmadd231pd		%zmm25, %zmm24, %zmm0

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret
	
	FUN_END(inner_scale_ab_8_lib8)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta=1.0
//
// input arguments:
// r10  <- alpha
// r11  <- y
// zmm0 <- []
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_A1_8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_scale_a1_8_lib8)
#endif

	// alpha
	vbroadcastsd	0(%r10), %zmm24
	vmulpd			%zmm0, %zmm24, %zmm0

	// beta
	vmovupd			0(%r11), %zmm24
	vaddpd			%zmm0, %zmm24, %zmm0

#if MACRO_LEVEL>=1
	.endm
#else
	ret
	
	FUN_END(inner_scale_a1_8_lib8)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta=1.0
//
// input arguments:
// r10  <- y
// zmm0 <- []
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M11_8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_scale_m11_8_lib8)
#endif

	vmovupd			0(%r10), %zmm24
	vsubpd			%zmm0, %zmm24, %zmm0

#if MACRO_LEVEL>=1
	.endm
#else
	ret
	
	FUN_END(inner_scale_m11_8_lib8)
#endif





// common inner routine with file scope
//
// store 
//
// input arguments:
// r10  <- z
// zmm0 <- []
//
// output arguments:
// r10  <- z
// zmm0 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_store_8_lib8)
#endif
	
	vmovupd %zmm0, 0(%r10)
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_8_lib8)
#endif





// common inner routine with file scope
//
// store vs
//
// input arguments:
// r10   <- D
// r11d   <- m1
// zmm0 <- []
//
// output arguments:
// r10   <- D
// r11d   <- m1
// zmm0 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_8_VS_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_store_8_vs_lib8)
#endif
	
	vcvtsi2sd		%r11d, %xmm25, %xmm25
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd			.LC00(%rip), %zmm24
#elif defined(OS_MAC)
	vmovupd			LC00(%rip), %zmm24
#endif
	vbroadcastsd	%xmm25, %zmm25
	vsubpd			%zmm25, %zmm24, %zmm25
	vpmovq2m		%zmm25, %k1

	vmovupd 		%zmm0, 0(%r10) {%k1}

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_8_vs_lib8)
#endif





// common inner routine with file scope
//
// store gen
//
// input arguments:
// r10   <- D
// r11d  <- m0 : start form (inc)
// r12d  <- m1 : up to (exc)
// zmm0 <- []
//
// output arguments:
// r10   <- D
// r11d  <- m0 : start form (inc)
// r12d  <- m1 : up to (exc)
// zmm0 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_8_GEN_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_store_8_gen_lib8)
#endif
	
	// compute mask for rows
	vcvtsi2sd		%r11d, %xmm24, %xmm24
	vcvtsi2sd		%r12d, %xmm25, %xmm25
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd			.LC00(%rip), %zmm26
#elif defined(OS_MAC)
	vmovupd			LC00(%rip), %zmm26
#endif
	vbroadcastsd	%xmm24, %zmm24
	vbroadcastsd	%xmm25, %zmm25
	vsubpd			%zmm26, %zmm24, %zmm24
	vsubpd			%zmm25, %zmm26, %zmm25
	vandpd			%zmm24, %zmm25, %zmm25
	vpmovq2m		%zmm25, %k1

	vmovupd			%zmm0, 0(%r10) {%k1}

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_8_gen_lib8)
#endif





// common inner routine with file scope
//
// clip
//
// input arguments:
// r10   <- m1
// zmm0 <- []
//
// output arguments:
// r10   <- m1
// zmm0 <- []

#if MACRO_LEVEL>=1
	.macro INNER_ZERO_8_VS_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_zero_8_vs_lib8)
#endif
	
	vcvtsi2sd		%r10d, %xmm25, %xmm25
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd			.LC00(%rip), %zmm24
#elif defined(OS_MAC)
	vmovupd			LC00(%rip), %zmm24
#endif
	vbroadcastsd	%xmm25, %zmm25
	vsubpd			%zmm25, %zmm24, %zmm25
	vpmovq2m		%zmm25, %k1

	vmovapd			%zmm0, %zmm0 {%k1}{z}

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_zero_8_vs_lib8)
#endif





//                            1      2              3          4          5             6          7
// void kernel_dgemv_n_8_lib8(int k, double *alpha, double *A, double *x, double *beta, double *y, double *z);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemv_n_8_lib8)
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%zmm0, %zmm0, %zmm0
	vmovapd	%zmm0, %zmm1
	vmovapd	%zmm0, %zmm2
	vmovapd	%zmm0, %zmm3


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // x

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_N_8_LIB8
#else
	CALL(inner_kernel_gemv_n_8_lib8)
#endif


	// call inner blend n scale ab

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11   // beta
	movq	ARG6, %r12   // y

#if MACRO_LEVEL>=1
	INNER_BLEND_N_8_LIB8
#else
	CALL(inner_blend_n_8_lib8)
#endif

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8_LIB8
#else
	CALL(inner_scale_ab_8_lib8)
#endif


	// store

	movq	ARG7, %r10 // z 

#if MACRO_LEVEL>=1
	INNER_STORE_8_LIB8
#else
	CALL(inner_store_8_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemv_n_8_lib8)





//                               1      2              3          4          5             6          7          8
// void kernel_dgemv_n_8_vs_lib8(int k, double *alpha, double *A, double *x, double *beta, double *y, double *z, int m1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemv_n_8_vs_lib8)
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%zmm0, %zmm0, %zmm0
	vmovapd	%zmm0, %zmm1
	vmovapd	%zmm0, %zmm2
	vmovapd	%zmm0, %zmm3


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // x

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_N_8_LIB8
#else
	CALL(inner_kernel_gemv_n_8_lib8)
#endif


	// call inner blend n scale ab

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11   // beta
	movq	ARG6, %r12   // y

#if MACRO_LEVEL>=1
	INNER_BLEND_N_8_LIB8
#else
	CALL(inner_blend_n_8_lib8)
#endif

	// TODO vs_ ???
#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8_LIB8
#else
	CALL(inner_scale_ab_8_lib8)
#endif


	// store

	movq	ARG7, %r10 // z 
	movq	ARG8, %r11 // m1 

#if MACRO_LEVEL>=1
	INNER_STORE_8_VS_LIB8
#else
	CALL(inner_store_8_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemv_n_8_vs_lib8)





#if 1
//                                1      2              3            4          5          6             7          8          9
// void kernel_dgemv_n_8_gen_lib8(int k, double *alpha, int offsetA, double *A, double *x, double *beta, double *y, double *z, int m1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemv_n_8_gen_lib8)
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%zmm0, %zmm0, %zmm0
	vmovapd	%zmm0, %zmm1
	vmovapd	%zmm0, %zmm2
	vmovapd	%zmm0, %zmm3


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG4, %r11  // A
	movq	ARG5, %r12  // x
	movq	ARG3, %r13  // offsetA

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_N_8_GEN_LIB8
#else
	CALL(inner_kernel_gemv_n_8_gen_lib8)
#endif


	// call inner blend n scale ab

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11   // beta
	movq	ARG7, %r12   // y

#if MACRO_LEVEL>=1
	INNER_BLEND_N_8_LIB8
#else
	CALL(inner_blend_n_8_lib8)
#endif

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8_LIB8
#else
	CALL(inner_scale_ab_8_lib8)
#endif


	// store

	movq	ARG8, %r10 // z
	movq	ARG9, %r11 // m1 

#if MACRO_LEVEL>=1
	INNER_STORE_8_VS_LIB8
#else
	CALL(inner_store_8_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemv_n_8_gen_lib8)
#else
//                                1      2              3          4          5             6          7          8       9
// void kernel_dgemv_n_8_gen_lib8(int k, double *alpha, double *A, double *x, double *beta, double *y, double *z, int m0, int m1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemv_n_8_gen_lib8)
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%zmm0, %zmm0, %zmm0
	vmovapd	%zmm0, %zmm1
	vmovapd	%zmm0, %zmm2
	vmovapd	%zmm0, %zmm3


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // x

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_N_8_LIB8
#else
	CALL(inner_kernel_gemv_n_8_lib8)
#endif


	// call inner blend n scale ab

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11   // beta
	movq	ARG6, %r12   // y

#if MACRO_LEVEL>=1
	INNER_BLEND_N_8_LIB8
#else
	CALL(inner_blend_n_8_lib8)
#endif

	// TODO gen_ ???
#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8_LIB8
#else
	CALL(inner_scale_ab_8_lib8)
#endif


	// store

	movq	ARG7, %r10 // z
	movq	ARG8, %r11 // m0
	movq	ARG9, %r12 // m1 

#if MACRO_LEVEL>=1
	INNER_STORE_8_GEN_LIB8
#else
	CALL(inner_store_8_gen_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemv_n_8_gen_lib8)
#endif





//                            1      2              3          4        5          6             7         8          9
// void kernel_dgemv_t_8_lib8(int k, double *alpha, int offa, double *A, int sda, double *x, double *beta, double *y, double *z);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemv_t_8_lib8)
	
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
	movq	ARG4, %r11  // A
	movq	ARG5, %r12 // sda
	sall	$ 6, %r12d // 4*sda*sizeof(double)
//	movslq	%r12d, %r12
	movq	ARG6, %r13  // x
	movq	ARG3, %r14 // offA

#if MACRO_LEVEL>=1
	INNER_EDGE_GEMV_T_8_LIB8
#else
	CALL(inner_edge_gemv_t_8_lib8)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_T_8_LIB8
#else
	CALL(inner_kernel_gemv_t_8_lib8)
#endif


	// call inner blender t

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11   // beta
	movq	ARG8, %r12 // y 

#if MACRO_LEVEL>=1
	INNER_BLEND_T_8_LIB8
#else
	CALL(inner_blend_t_8_lib8)
#endif

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8_LIB8
#else
	CALL(inner_scale_ab_8_lib8)
#endif


	// store

	movq	ARG9, %r10 // z 

#if MACRO_LEVEL>=1
	INNER_STORE_8_LIB8
#else
	CALL(inner_store_8_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemv_t_8_lib8)





//                               1      2              3          4        5          6             7         8          9          10
// void kernel_dgemv_t_8_vs_lib8(int k, double *alpha, int offa, double *A, int sda, double *x, double *beta, double *y, double *z, int m1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemv_t_8_vs_lib8)
	
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
	movq	ARG4, %r11  // A
	movq	ARG5, %r12 // sda
	sall	$ 6, %r12d // 4*sda*sizeof(double)
//	movslq	%r12d, %r12
	movq	ARG6, %r13  // x
	movq	ARG3, %r14 // offA

#if MACRO_LEVEL>=1
	INNER_EDGE_GEMV_T_8_LIB8
#else
	CALL(inner_edge_gemv_t_8_lib8)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_T_8_LIB8
#else
	CALL(inner_kernel_gemv_t_8_lib8)
#endif


	// call inner blender t

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11   // beta
	movq	ARG8, %r12 // y 

#if MACRO_LEVEL>=1
	INNER_BLEND_T_8_LIB8
#else
	CALL(inner_blend_t_8_lib8)
#endif

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8_LIB8
#else
	CALL(inner_scale_ab_8_lib8)
#endif


	// store

	movq	ARG9, %r10 // z
	movq	ARG10, %r11 // m1

#if MACRO_LEVEL>=1
	INNER_STORE_8_VS_LIB8
#else
	CALL(inner_store_8_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemv_t_8_vs_lib8)





//                             1      2                3                4            5          6        7            8            9               10           11           12
// void kernel_dgemv_nt_8_lib8(int k, double *alpha_n, double *alpha_t, int offsetA, double *A, int sda, double *x_n, double *x_t, double *beta_t, double *y_t, double *z_n, double *z_t);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemv_nt_8_lib8)
	
	PROLOGUE

	// zero accumulation registers y_t

	vxorpd	%zmm0, %zmm0, %zmm0
	vmovapd	%zmm0, %zmm1
	vmovapd	%zmm0, %zmm2
	vmovapd	%zmm0, %zmm3
	vmovapd	%zmm0, %zmm4
	vmovapd	%zmm0, %zmm5
	vmovapd	%zmm0, %zmm6
	vmovapd	%zmm0, %zmm7

	// initialize x_n
	movq	ARG2, %r10 // alpha_n
	vbroadcastsd 0(%r10), %zmm24

	movq	ARG7, %r10 // x_n

	vbroadcastsd 0(%r10), %zmm8
	vmulpd		%zmm24, %zmm8, %zmm8
	vbroadcastsd 8(%r10), %zmm9
	vmulpd		%zmm24, %zmm9, %zmm9
	vbroadcastsd 16(%r10), %zmm10
	vmulpd		%zmm24, %zmm10, %zmm10
	vbroadcastsd 24(%r10), %zmm11
	vmulpd		%zmm24, %zmm11, %zmm11
	vbroadcastsd 32(%r10), %zmm12
	vmulpd		%zmm24, %zmm12, %zmm12
	vbroadcastsd 40(%r10), %zmm13
	vmulpd		%zmm24, %zmm13, %zmm13
	vbroadcastsd 48(%r10), %zmm14
	vmulpd		%zmm24, %zmm14, %zmm14
	vbroadcastsd 56(%r10), %zmm15
	vmulpd		%zmm24, %zmm15, %zmm15


	// inner kernel dgemv nt

	movq	ARG1, %r10 // k
	movq	ARG5, %r11  // A
	movq	ARG6, %r12 // sda
	sall	$ 6, %r12d // 4*sda*sizeof(double)
	movq	ARG8, %r13  // x_t
	movq	ARG11, %r14  // z_n
	movq	ARG4, %r15  // offsetA

#if MACRO_LEVEL>=1
	INNER_EDGE_GEMV_NT_8_LIB8
#else
	CALL(inner_edge_gemv_nt_8_lib8)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_NT_8_LIB8
#else
	CALL(inner_kernel_gemv_nt_8_lib8)
#endif


	// inner blend n scale ab

	movq	ARG3, %r10 // alpha_t
	movq	ARG9, %r11   // beta_t
	movq	ARG10, %r12   // y_t

#if MACRO_LEVEL>=1
	INNER_BLEND_T_8_LIB8
#else
	CALL(inner_blend_t_8_lib8)
#endif

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8_LIB8
#else
	CALL(inner_scale_ab_8_lib8)
#endif


	// store

	movq	ARG12, %r10 // z_t 

#if MACRO_LEVEL>=1
	INNER_STORE_8_LIB8
#else
	CALL(inner_store_8_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemv_nt_8_lib8)





//                                1      2                3                4            5          6        7            8            9               10           11           12           13
// void kernel_dgemv_nt_8_vs_lib8(int k, double *alpha_n, double *alpha_t, int offsetA, double *A, int sda, double *x_n, double *x_t, double *beta_t, double *y_t, double *z_n, double *z_t, int m1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemv_nt_8_vs_lib8)
	
	PROLOGUE

	// zero accumulation registers y_t

	vxorpd	%zmm0, %zmm0, %zmm0
	vmovapd	%zmm0, %zmm1
	vmovapd	%zmm0, %zmm2
	vmovapd	%zmm0, %zmm3
	vmovapd	%zmm0, %zmm4
	vmovapd	%zmm0, %zmm5
	vmovapd	%zmm0, %zmm6
	vmovapd	%zmm0, %zmm7

	vmovapd	%zmm0, %zmm8
	vmovapd	%zmm0, %zmm9
	vmovapd	%zmm0, %zmm10
	vmovapd	%zmm0, %zmm11
	vmovapd	%zmm0, %zmm12
	vmovapd	%zmm0, %zmm13
	vmovapd	%zmm0, %zmm14
	vmovapd	%zmm0, %zmm15

	// initialize x_n
	movq	ARG2, %r10 // alpha_n
	vbroadcastsd 0(%r10), %zmm24

	movq	ARG7, %r10 // x_n
	movq	ARG13, %r11 // m1

	vbroadcastsd 0(%r10), %zmm8
	vmulpd		%zmm24, %zmm8, %zmm8
	cmpl	$ 2, %r11d
	jl		0f
	vbroadcastsd 8(%r10), %zmm9
	vmulpd		%zmm24, %zmm9, %zmm9
	cmpl	$ 3, %r11d
	jl		0f
	vbroadcastsd 16(%r10), %zmm10
	vmulpd		%zmm24, %zmm10, %zmm10
	cmpl	$ 4, %r11d
	jl		0f
	vbroadcastsd 24(%r10), %zmm11
	vmulpd		%zmm24, %zmm11, %zmm11
	cmpl	$ 5, %r11d
	jl		0f
	vbroadcastsd 32(%r10), %zmm12
	vmulpd		%zmm24, %zmm12, %zmm12
	cmpl	$ 6, %r11d
	jl		0f
	vbroadcastsd 40(%r10), %zmm13
	vmulpd		%zmm24, %zmm13, %zmm13
	cmpl	$ 7, %r11d
	jl		0f
	vbroadcastsd 48(%r10), %zmm14
	vmulpd		%zmm24, %zmm14, %zmm14
	je		0f
	vbroadcastsd 56(%r10), %zmm15
	vmulpd		%zmm24, %zmm15, %zmm15
0:


	// inner kernel dgemv nt

	movq	ARG1, %r10 // k
	movq	ARG5, %r11  // A
	movq	ARG6, %r12 // sda
	sall	$ 6, %r12d // 4*sda*sizeof(double)
	movq	ARG8, %r13  // x_t
	movq	ARG11, %r14  // z_n
	movq	ARG4, %r15  // offsetA

#if MACRO_LEVEL>=1
	INNER_EDGE_GEMV_NT_8_LIB8
#else
	CALL(inner_edge_gemv_nt_8_lib8)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_NT_8_LIB8
#else
	CALL(inner_kernel_gemv_nt_8_lib8)
#endif


	// inner blend n scale ab

	movq	ARG3, %r10 // alpha_t
	movq	ARG9, %r11   // beta_t
	movq	ARG10, %r12   // y_t

#if MACRO_LEVEL>=1
	INNER_BLEND_T_8_LIB8
#else
	CALL(inner_blend_t_8_lib8)
#endif

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8_LIB8
#else
	CALL(inner_scale_ab_8_lib8)
#endif


	// store

	movq	ARG12, %r10 // z_t 
	movq	ARG13, %r11 // m1

#if MACRO_LEVEL>=1
	INNER_STORE_8_VS_LIB8
#else
	CALL(inner_store_8_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemv_nt_8_vs_lib8)





//                            1      2              3          4        5           6
// void kernel_dsymv_l_8_lib8(int k, double *alpha, double *A, int sda, double *x, double *z);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsymv_l_8_lib8)
	
	PROLOGUE

	// zero accumulation registers y_t

	vxorpd	%zmm0, %zmm0, %zmm0
	vmovapd	%zmm0, %zmm1
	vmovapd	%zmm0, %zmm2
	vmovapd	%zmm0, %zmm3
	vmovapd	%zmm0, %zmm4
	vmovapd	%zmm0, %zmm5
	vmovapd	%zmm0, %zmm6
	vmovapd	%zmm0, %zmm7

	// initialize x_n
	movq	ARG2, %r10 // alpha_n
	vbroadcastsd 0(%r10), %zmm24

	movq	ARG5, %r10 // x_n

	vbroadcastsd 0(%r10), %zmm8
	vmulpd		%zmm24, %zmm8, %zmm8
	vbroadcastsd 8(%r10), %zmm9
	vmulpd		%zmm24, %zmm9, %zmm9
	vbroadcastsd 16(%r10), %zmm10
	vmulpd		%zmm24, %zmm10, %zmm10
	vbroadcastsd 24(%r10), %zmm11
	vmulpd		%zmm24, %zmm11, %zmm11
	vbroadcastsd 32(%r10), %zmm12
	vmulpd		%zmm24, %zmm12, %zmm12
	vbroadcastsd 40(%r10), %zmm13
	vmulpd		%zmm24, %zmm13, %zmm13
	vbroadcastsd 48(%r10), %zmm14
	vmulpd		%zmm24, %zmm14, %zmm14
	vbroadcastsd 56(%r10), %zmm15
	vmulpd		%zmm24, %zmm15, %zmm15


	// inner edge dsyrk & kernel dgemv nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 6, %r12d // 8*sda*sizeof(double)
	movq	ARG5, %r13  // x_t
	movq	ARG6, %r14  // z_n

#if MACRO_LEVEL>=1
	INNER_EDGE_SYMV_NT_L_8_LIB8
#else
	CALL(inner_edge_symv_nt_l_8_lib8)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_NT_8_LIB8
#else
	CALL(inner_kernel_gemv_nt_8_lib8)
#endif


	// call inner blend n scale ab

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11   // z_t

#if MACRO_LEVEL>=1
	INNER_BLEND_T_8_LIB8
#else
	CALL(inner_blend_t_8_lib8)
#endif

#if MACRO_LEVEL>=1
	INNER_SCALE_A1_8_LIB8
#else
	CALL(inner_scale_a1_8_lib8)
#endif


	// store

	movq	ARG6, %r10 // z_t 

#if MACRO_LEVEL>=1
	INNER_STORE_8_LIB8
#else
	CALL(inner_store_8_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dsymv_l_8_lib8)





//                               1      2              3          4        5           6         7
// void kernel_dsymv_l_8_vs_lib8(int k, double *alpha, double *A, int sda, double *x, double *z, int m1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsymv_l_8_vs_lib8)
	
	PROLOGUE

	// zero accumulation registers y_t

	vxorpd	%zmm0, %zmm0, %zmm0
	vmovapd	%zmm0, %zmm1
	vmovapd	%zmm0, %zmm2
	vmovapd	%zmm0, %zmm3
	vmovapd	%zmm0, %zmm4
	vmovapd	%zmm0, %zmm5
	vmovapd	%zmm0, %zmm6
	vmovapd	%zmm0, %zmm7

	vmovapd	%zmm0, %zmm8
	vmovapd	%zmm0, %zmm9
	vmovapd	%zmm0, %zmm10
	vmovapd	%zmm0, %zmm11
	vmovapd	%zmm0, %zmm12
	vmovapd	%zmm0, %zmm13
	vmovapd	%zmm0, %zmm14
	vmovapd	%zmm0, %zmm15

	// initialize x_n
	movq	ARG2, %r10 // alpha_n
	vbroadcastsd 0(%r10), %zmm24

	movq	ARG5, %r10 // x_n
	movq	ARG7, %r11 // m1

	vbroadcastsd 0(%r10), %zmm8
	vmulpd		%zmm24, %zmm8, %zmm8
	cmpl	$ 2, %r11d
	jl		0f
	vbroadcastsd 8(%r10), %zmm9
	vmulpd		%zmm24, %zmm9, %zmm9
	cmpl	$ 3, %r11d
	jl		0f
	vbroadcastsd 16(%r10), %zmm10
	vmulpd		%zmm24, %zmm10, %zmm10
	cmpl	$ 4, %r11d
	jl		0f
	vbroadcastsd 24(%r10), %zmm11
	vmulpd		%zmm24, %zmm11, %zmm11
	cmpl	$ 5, %r11d
	jl		0f
	vbroadcastsd 32(%r10), %zmm12
	vmulpd		%zmm24, %zmm12, %zmm12
	cmpl	$ 6, %r11d
	jl		0f
	vbroadcastsd 40(%r10), %zmm13
	vmulpd		%zmm24, %zmm13, %zmm13
	cmpl	$ 7, %r11d
	jl		0f
	vbroadcastsd 48(%r10), %zmm14
	vmulpd		%zmm24, %zmm14, %zmm14
	je		0f
	vbroadcastsd 56(%r10), %zmm15
	vmulpd		%zmm24, %zmm15, %zmm15
0:


	// inner edge dsyrk & kernel dgemv nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 6, %r12d // 8*sda*sizeof(double)
	movq	ARG5, %r13  // x_t
	movq	ARG6, %r14  // z_n
	movq	ARG7, %r15  // m1

#if MACRO_LEVEL>=1
	INNER_EDGE_SYMV_NT_L_8_VS_LIB8
#else
	CALL(inner_edge_symv_nt_l_8_vs_lib8)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_NT_8_LIB8
#else
	CALL(inner_kernel_gemv_nt_8_lib8)
#endif


	// call inner blend n scale ab

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11   // z_t

#if MACRO_LEVEL>=1
	INNER_BLEND_T_8_LIB8
#else
	CALL(inner_blend_t_8_lib8)
#endif

#if MACRO_LEVEL>=1
	INNER_SCALE_A1_8_LIB8
#else
	CALL(inner_scale_a1_8_lib8)
#endif


	// store

	movq	ARG6, %r10 // z_t 
	movq	ARG7, %r11  // m1

#if MACRO_LEVEL>=1
	INNER_STORE_8_VS_LIB8
#else
	CALL(inner_store_8_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dsymv_l_8_vs_lib8)





//                                1      2              3            4          5        6          7          8
// void kernel_dsymv_l_8_gen_lib8(int k, double *alpha, int offsetA, double *A, int sda, double *x, double *z, int m1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dsymv_l_8_gen_lib8)
	
	PROLOGUE

	// zero accumulation registers y_t

	vxorpd	%zmm0, %zmm0, %zmm0
	vmovapd	%zmm0, %zmm1
	vmovapd	%zmm0, %zmm2
	vmovapd	%zmm0, %zmm3
	vmovapd	%zmm0, %zmm4
	vmovapd	%zmm0, %zmm5
	vmovapd	%zmm0, %zmm6
	vmovapd	%zmm0, %zmm7

	vmovapd	%zmm0, %zmm8
	vmovapd	%zmm0, %zmm9
	vmovapd	%zmm0, %zmm10
	vmovapd	%zmm0, %zmm11
	vmovapd	%zmm0, %zmm12
	vmovapd	%zmm0, %zmm13
	vmovapd	%zmm0, %zmm14
	vmovapd	%zmm0, %zmm15

	// initialize x_n
	movq	ARG2, %r10 // alpha_n
	vbroadcastsd 0(%r10), %zmm24

	movq	ARG6, %r10 // x_n
	movq	ARG8, %r11 // m1

	vbroadcastsd 0(%r10), %zmm8
	vmulpd		%zmm24, %zmm8, %zmm8
	cmpl	$ 2, %r11d
	jl		0f
	vbroadcastsd 8(%r10), %zmm9
	vmulpd		%zmm24, %zmm9, %zmm9
	cmpl	$ 3, %r11d
	jl		0f
	vbroadcastsd 16(%r10), %zmm10
	vmulpd		%zmm24, %zmm10, %zmm10
	cmpl	$ 4, %r11d
	jl		0f
	vbroadcastsd 24(%r10), %zmm11
	vmulpd		%zmm24, %zmm11, %zmm11
	cmpl	$ 5, %r11d
	jl		0f
	vbroadcastsd 32(%r10), %zmm12
	vmulpd		%zmm24, %zmm12, %zmm12
	cmpl	$ 6, %r11d
	jl		0f
	vbroadcastsd 40(%r10), %zmm13
	vmulpd		%zmm24, %zmm13, %zmm13
	cmpl	$ 7, %r11d
	jl		0f
	vbroadcastsd 48(%r10), %zmm14
	vmulpd		%zmm24, %zmm14, %zmm14
	je		0f
	vbroadcastsd 56(%r10), %zmm15
	vmulpd		%zmm24, %zmm15, %zmm15
0:


	// inner edge dsyrk & kernel dgemv nt

	movq	ARG1, %r10 // k
	movq	ARG4, %r11  // A
	movq	ARG5, %r12 // sda
	sall	$ 6, %r12d // 8*sda*sizeof(double)
	movq	ARG6, %r13  // x_t
	movq	ARG7, %r14  // z_n
	movq	ARG8, %r15  // m1
	movq	ARG3, %rax  // offsetA

#if MACRO_LEVEL>=1
	INNER_EDGE_SYMV_NT_L_8_GEN_LIB8
#else
	CALL(inner_edge_symv_nt_l_8_gen_lib8)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_NT_8_LIB8
#else
	CALL(inner_kernel_gemv_nt_8_lib8)
#endif


	// call inner blend n scale ab

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11   // z_t

#if MACRO_LEVEL>=1
	INNER_BLEND_T_8_LIB8
#else
	CALL(inner_blend_t_8_lib8)
#endif

#if MACRO_LEVEL>=1
	INNER_SCALE_A1_8_LIB8
#else
	CALL(inner_scale_a1_8_lib8)
#endif


	// store

	movq	ARG7, %r10 // z_t 
	movq	ARG8, %r11  // m1

#if MACRO_LEVEL>=1
	INNER_STORE_8_VS_LIB8
#else
	CALL(inner_store_8_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dsymv_l_8_gen_lib8)





//                               1      2          3          4
// void kernel_dtrmv_n_ln_8_lib8(int k, double *A, double *x, double *z);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmv_n_ln_8_lib8)
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%zmm0, %zmm0, %zmm0
	vmovapd	%zmm0, %zmm1
	vmovapd	%zmm0, %zmm2
	vmovapd	%zmm0, %zmm3


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12  // x

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_N_8_LIB8
#else
	CALL(inner_kernel_gemv_n_8_lib8)
#endif

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMV_N_LN_8_LIB8
#else
	CALL(inner_edge_trmv_n_ln_8_lib8)
#endif


	// call inner blend n

#if MACRO_LEVEL>=1
	INNER_BLEND_N_8_LIB8
#else
	CALL(inner_blend_n_8_lib8)
#endif


	// store

	movq	ARG4, %r10 // z 

#if MACRO_LEVEL>=1
	INNER_STORE_8_LIB8
#else
	CALL(inner_store_8_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmv_n_ln_8_lib8)





//                                  1      2          3          4          5
// void kernel_dtrmv_n_ln_8_vs_lib8(int k, double *A, double *x, double *z, int m1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmv_n_ln_8_vs_lib8)
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%zmm0, %zmm0, %zmm0
	vmovapd	%zmm0, %zmm1
	vmovapd	%zmm0, %zmm2
	vmovapd	%zmm0, %zmm3


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG3, %r12  // x

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_N_8_LIB8
#else
	CALL(inner_kernel_gemv_n_8_lib8)
#endif

	movq	ARG5, %r13 // m1 

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMV_N_LN_8_VS_LIB8
#else
	CALL(inner_edge_trmv_n_ln_8_vs_lib8)
#endif


	// call inner blend n

#if MACRO_LEVEL>=1
	INNER_BLEND_N_8_LIB8
#else
	CALL(inner_blend_n_8_lib8)
#endif


	// store

	movq	ARG4, %r10 // z 
	movq	ARG5, %r11 // m1 

#if MACRO_LEVEL>=1
	INNER_STORE_8_VS_LIB8
#else
	CALL(inner_store_8_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmv_n_ln_8_vs_lib8)





//                                   1      2            3          4          5          6
// void kernel_dtrmv_n_ln_8_gen_lib8(int k, int offsetA, double *A, double *x, double *z, int m1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmv_n_ln_8_gen_lib8)
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%zmm0, %zmm0, %zmm0
	vmovapd	%zmm0, %zmm1
	vmovapd	%zmm0, %zmm2
	vmovapd	%zmm0, %zmm3


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // x
	movq	ARG2, %r13  // offsetA

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_N_8_GEN_LIB8
#else
	CALL(inner_kernel_gemv_n_8_gen_lib8)
#endif

	movq	ARG6, %r13 // m1 
	movq	ARG2, %r14  // offsetA

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMV_N_LN_8_GEN_LIB8
#else
	CALL(inner_edge_trmv_n_ln_8_gen_lib8)
#endif


	// call inner blend n

#if MACRO_LEVEL>=1
	INNER_BLEND_N_8_LIB8
#else
	CALL(inner_blend_n_8_lib8)
#endif


	// store

	movq	ARG5, %r10 // z 
	movq	ARG6, %r11 // m1 

#if MACRO_LEVEL>=1
	INNER_STORE_8_VS_LIB8
#else
	CALL(inner_store_8_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmv_n_ln_8_gen_lib8)





//                               1      2          3        4          5
// void kernel_dtrmv_t_ln_8_lib8(int k, double *A, int sda, double *x, double *z);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmv_t_ln_8_lib8)
	
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
	movq	ARG2, %r11  // A
	movq	ARG3, %r12 // sda
	sall	$ 6, %r12d // 4*sda*sizeof(double)
//	movslq	%r12d, %r12
	movq	ARG4, %r13  // x

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMV_T_LN_8_LIB8
#else
	CALL(inner_edge_trmv_t_ln_8_lib8)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_T_8_LIB8
#else
	CALL(inner_kernel_gemv_t_8_lib8)
#endif


	// call inner blender t

#if MACRO_LEVEL>=1
	INNER_BLEND_T_8_LIB8
#else
	CALL(inner_blend_t_8_lib8)
#endif


	// store

	movq	ARG5, %r10 // z 

#if MACRO_LEVEL>=1
	INNER_STORE_8_LIB8
#else
	CALL(inner_store_8_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmv_t_ln_8_lib8)





//                                  1      2          3        4          5          6
// void kernel_dtrmv_t_ln_8_vs_lib8(int k, double *A, int sda, double *x, double *z, int m1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmv_t_ln_8_vs_lib8)
	
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
	movq	ARG2, %r11  // A
	movq	ARG3, %r12 // sda
	sall	$ 6, %r12d // 4*sda*sizeof(double)
//	movslq	%r12d, %r12
	movq	ARG4, %r13  // x
	movq	ARG6, %r14  // m1

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMV_T_LN_8_VS_LIB8
#else
	CALL(inner_edge_trmv_t_ln_8_vs_lib8)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_T_8_LIB8
#else
	CALL(inner_kernel_gemv_t_8_lib8)
#endif


	// call inner blender t

#if MACRO_LEVEL>=1
	INNER_BLEND_T_8_LIB8
#else
	CALL(inner_blend_t_8_lib8)
#endif


	// store

	movq	ARG5, %r10 // z
	movq	ARG6, %r11 // m1

#if MACRO_LEVEL>=1
	INNER_STORE_8_VS_LIB8
#else
	CALL(inner_store_8_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmv_t_ln_8_vs_lib8)





//                                   1      2            3          4        5          6          7
// void kernel_dtrmv_t_ln_8_gen_lib8(int k, int offsetA, double *A, int sda, double *x, double *z, int m1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrmv_t_ln_8_gen_lib8)
	
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
	sall	$ 6, %r12d // 4*sda*sizeof(double)
//	movslq	%r12d, %r12
	movq	ARG5, %r13  // x
	movq	ARG7, %r14  // m1
	movq	ARG2, %r15  // offsetA

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMV_T_LN_8_GEN_LIB8
#else
	CALL(inner_edge_trmv_t_ln_8_gen_lib8)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_T_8_LIB8
#else
	CALL(inner_kernel_gemv_t_8_lib8)
#endif


	// call inner blender t

#if MACRO_LEVEL>=1
	INNER_BLEND_T_8_LIB8
#else
	CALL(inner_blend_t_8_lib8)
#endif


	// store

	movq	ARG6, %r10 // z
	movq	ARG7, %r11 // m1

#if MACRO_LEVEL>=1
	INNER_STORE_8_VS_LIB8
#else
	CALL(inner_store_8_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrmv_t_ln_8_gen_lib8)





//                                  1      2          3                   4          5
// void kernel_dtrsv_n_l_inv_8_lib8(int k, double *A, double *inv_diag_A, double *x, double *z);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsv_n_l_inv_8_lib8)
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%zmm0, %zmm0, %zmm0
	vmovapd	%zmm0, %zmm1
	vmovapd	%zmm0, %zmm2
	vmovapd	%zmm0, %zmm3


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG4, %r12  // x

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_N_8_LIB8
#else
	CALL(inner_kernel_gemv_n_8_lib8)
#endif


	// call inner blender n

	movq	ARG5, %r10   // z

#if MACRO_LEVEL>=1
	INNER_BLEND_N_8_LIB8
#else
	CALL(inner_blend_n_8_lib8)
#endif

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_8_LIB8
#else
	CALL(inner_scale_m11_8_lib8)
#endif


	// solution

	movq	ARG1, %r10
	sall	$ 6, %r10d
	leaq	0(ARG2, %r10), %r10
	movq	ARG3, %r11 // inv_diag_A

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSV_N_L_INV_8_LIB8
#else
	CALL(inner_edge_dtrsv_n_l_inv_8_lib8)
#endif


	// store

	movq	ARG5, %r10 // z 

#if MACRO_LEVEL>=1
	INNER_STORE_8_LIB8
#else
	CALL(inner_store_8_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsv_n_l_inv_8_lib8)





//                                     1      2          3                   4          5          6       7
// void kernel_dtrsv_n_l_inv_8_vs_lib8(int k, double *A, double *inv_diag_A, double *x, double *z, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsv_n_l_inv_8_vs_lib8)
	
	PROLOGUE

	// zero accumulation registers

	vxorpd	%zmm0, %zmm0, %zmm0
	vmovapd	%zmm0, %zmm1
	vmovapd	%zmm0, %zmm2
	vmovapd	%zmm0, %zmm3


	// call inner dgemv kernel n

	movq	ARG1, %r10 // k
	movq	ARG2, %r11  // A
	movq	ARG4, %r12  // x

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_N_8_LIB8
#else
	CALL(inner_kernel_gemv_n_8_lib8)
#endif


	// call inner blender n

	movq	ARG5, %r10   // z

#if MACRO_LEVEL>=1
	INNER_BLEND_N_8_LIB8
#else
	CALL(inner_blend_n_8_lib8)
#endif

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_8_LIB8
#else
	CALL(inner_scale_m11_8_lib8)
#endif


	// solution

	movq	ARG1, %r10
	sall	$ 6, %r10d
	leaq	0(ARG2, %r10), %r10
	movq	ARG3, %r11 // inv_diag_A
	movq	ARG7, %r12 // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSV_N_L_INV_8_VS_LIB8
#else
	CALL(inner_edge_dtrsv_n_l_inv_8_vs_lib8)
#endif


	// store

	movq	ARG5, %r10 // z 
	movq	ARG6, %r11 // m1

#if MACRO_LEVEL>=1
	INNER_STORE_8_VS_LIB8
#else
	CALL(inner_store_8_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsv_n_l_inv_8_vs_lib8)





//                                  1      2          3        4                   5          6
// void kernel_dtrsv_t_l_inv_8_lib8(int k, double *A, int sda, double *inv_diag_A, double *x, double *z);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsv_t_l_inv_8_lib8)
	
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
//	subl	$ 8, %r10d
	movq	ARG2, %r11 // A
	movq	ARG3, %r12
	sall	$ 6, %r12d // 4*sda*sizeof(double)
	addq	%r12, %r11 // A+4*sda*sizeof(double)
	movq	ARG5, %r13 // x
	addq	$ 64, %r13 // x+8 

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_T_8_LIB8
#else
	CALL(inner_kernel_gemv_t_8_lib8)
#endif


	// call inner blender t

	movq	ARG6, %r10 // z

#if MACRO_LEVEL>=1
	INNER_BLEND_T_8_LIB8
#else
	CALL(inner_blend_t_8_lib8)
#endif

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_8_LIB8
#else
	CALL(inner_scale_m11_8_lib8)
#endif


	// solution

	movq	ARG2, %r10 // A
	movq	ARG4, %r11 // inv_diag_A

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSV_T_L_INV_8_LIB8
#else
	CALL(inner_edge_dtrsv_t_l_inv_8_lib8)
#endif


	// store

	movq	ARG6, %r10 // z 

#if MACRO_LEVEL>=1
	INNER_STORE_8_LIB8
#else
	CALL(inner_store_8_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsv_t_l_inv_8_lib8)





//                                     1      2          3        4                   5          6          7       8
// void kernel_dtrsv_t_l_inv_8_vs_lib8(int k, double *A, int sda, double *inv_diag_A, double *x, double *z, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsv_t_l_inv_8_vs_lib8)
	
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
//	subl	$ 8, %r10d
	movq	ARG2, %r11 // A
	movq	ARG3, %r12
	sall	$ 6, %r12d // 4*sda*sizeof(double)
	addq	%r12, %r11 // A+4*sda*sizeof(double)
	movq	ARG5, %r13 // x
	addq	$ 64, %r13 // x+8 

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMV_T_8_LIB8
#else
	CALL(inner_kernel_gemv_t_8_lib8)
#endif


	// call inner blender t

#if MACRO_LEVEL>=1
	INNER_BLEND_T_8_LIB8
#else
	CALL(inner_blend_t_8_lib8)
#endif

	// zero out dummy components
	movq	ARG8, %r10 // n1

#if MACRO_LEVEL>=1
	INNER_ZERO_8_VS_LIB8
#else
	CALL(inner_zero_8_vs_lib8)
#endif

	// call scale
	movq	ARG6, %r10 // z

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_8_LIB8
#else
	CALL(inner_scale_m11_8_lib8)
#endif


	// solution

	movq	ARG2, %r10 // A
	movq	ARG4, %r11 // inv_diag_A
	movq	ARG7, %r12 // m1
	movq	ARG8, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSV_T_L_INV_8_VS_LIB8
#else
	CALL(inner_edge_dtrsv_t_l_inv_8_vs_lib8)
#endif


	// store

	movq	ARG6, %r10 // z
	movq	ARG8, %r11 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_8_VS_LIB8
#else
	CALL(inner_store_8_vs_lib8)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsv_t_l_inv_8_vs_lib8)





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
LC00:
	.align 6
#endif
	.double	0.5
	.double	1.5
	.double	2.5
	.double	3.5
	.double	4.5
	.double	5.5
	.double	6.5
	.double	7.5


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

