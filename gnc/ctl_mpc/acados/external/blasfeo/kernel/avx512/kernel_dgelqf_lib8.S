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

#else // defined(OS_MAC)

#define GLOB_FUN_START(NAME) \
	.globl _ ## NAME; \
_ ## NAME:
#define FUN_START(NAME) \
_ ## NAME:
#define FUN_END(NAME)
#define CALL(NAME) \
	callq _ ## NAME

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

#else

#error wrong OS

#endif



#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.text
#elif defined(OS_MAC)
	.section	__TEXT,__text,regular,pure_instructions
#endif






#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_1_DGELQF_8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_1_dgelqf_8_lib8)
#endif

	cmpl	$ 7, %r10d
	jle		109f
103:
	vbroadcastsd	0*64(%r11, %r14), %zmm16
	vmulpd			%zmm25, %zmm16, %zmm16
	vbroadcastsd	1*64(%r11, %r14), %zmm17
	vmulpd			%zmm25, %zmm17, %zmm17
	vbroadcastsd	2*64(%r11, %r14), %zmm18
	vmulpd			%zmm25, %zmm18, %zmm18
	vbroadcastsd	3*64(%r11, %r14), %zmm19
	vmulpd			%zmm25, %zmm19, %zmm19
	vbroadcastsd	4*64(%r11, %r14), %zmm20
	vmulpd			%zmm25, %zmm20, %zmm20
	vbroadcastsd	5*64(%r11, %r14), %zmm21
	vmulpd			%zmm25, %zmm21, %zmm21
	vbroadcastsd	6*64(%r11, %r14), %zmm22
	vmulpd			%zmm25, %zmm22, %zmm22
	vbroadcastsd	7*64(%r11, %r14), %zmm23
	vmulpd			%zmm25, %zmm23, %zmm23
	vfmadd231pd		0*64(%r11), %zmm16, %zmm0
	vmovsd			%xmm16, 0*64(%r11, %r14)
	vfmadd231pd		1*64(%r11), %zmm17, %zmm1
	vmovsd			%xmm17, 1*64(%r11, %r14)
	vfmadd231pd		2*64(%r11), %zmm18, %zmm0
	vmovsd			%xmm18, 2*64(%r11, %r14)
	vfmadd231pd		3*64(%r11), %zmm19, %zmm1
	vmovsd			%xmm19, 3*64(%r11, %r14)
	vfmadd231pd		4*64(%r11), %zmm20, %zmm0
	vmovsd			%xmm20, 4*64(%r11, %r14)
	vfmadd231pd		5*64(%r11), %zmm21, %zmm1
	vmovsd			%xmm21, 5*64(%r11, %r14)
	vfmadd231pd		6*64(%r11), %zmm22, %zmm0
	vmovsd			%xmm22, 6*64(%r11, %r14)
	vfmadd231pd		7*64(%r11), %zmm23, %zmm1
	vmovsd			%xmm23, 7*64(%r11, %r14)
	subl	$ 8, %r10d
	addq	$ 512, %r11
	cmpl	$ 7, %r10d
	jg		103b
109:
	vaddpd			%zmm1, %zmm0, %zmm0 // reduce acc
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	0(%r11, %r14), %zmm16
	vmulpd			%zmm25, %zmm16, %zmm16
	vfmadd231pd		0(%r11), %zmm16, %zmm0
	vmovsd			%xmm16, 0(%r11, %r14)
	subl	$ 1, %r10d
	addq	$ 64, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_1_dgelqf_8_lib8)
#endif





#if MACRO_LEVEL>=1
	.macro INNER_KERNEL_2_DGELQF_8_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_2_dgelqf_8_lib8)
#endif

	cmpl	$ 7, %r10d
	jle		110f
106:
	vmovapd			0*64(%r11), %zmm16
	vmovapd			1*64(%r11), %zmm17
	vmovapd			2*64(%r11), %zmm18
	vmovapd			3*64(%r11), %zmm19
	vmovapd			4*64(%r11), %zmm20
	vmovapd			5*64(%r11), %zmm21
	vmovapd			6*64(%r11), %zmm22
	vmovapd			7*64(%r11), %zmm23
	vbroadcastsd	0*64(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm16
	vbroadcastsd	1*64(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm17
	vbroadcastsd	2*64(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm18
	vbroadcastsd	3*64(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm19
	vbroadcastsd	4*64(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm20
	vbroadcastsd	5*64(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm21
	vbroadcastsd	6*64(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm22
	vbroadcastsd	7*64(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vfmadd231pd		%zmm16, %zmm16, %zmm25
	vmovapd			%zmm16, 0*64(%r11)
	vfmadd231pd		%zmm17, %zmm17, %zmm26
	vmovapd			%zmm17, 1*64(%r11)
	vfmadd231pd		%zmm18, %zmm18, %zmm25
	vmovapd			%zmm18, 2*64(%r11)
	vfmadd231pd		%zmm19, %zmm19, %zmm26
	vmovapd			%zmm19, 3*64(%r11)
	vfmadd231pd		%zmm20, %zmm20, %zmm25
	vmovapd			%zmm20, 4*64(%r11)
	vfmadd231pd		%zmm21, %zmm21, %zmm26
	vmovapd			%zmm21, 5*64(%r11)
	vfmadd231pd		%zmm22, %zmm22, %zmm25
	vmovapd			%zmm22, 6*64(%r11)
	vfmadd231pd		%zmm23, %zmm23, %zmm26
	vmovapd			%zmm23, 7*64(%r11)
	subl	$ 8, %r10d
	addq	$ 512, %r11
	cmpl	$ 7, %r10d
	jg		106b
110:
	vaddpd			%zmm26, %zmm25, %zmm25 // reduce acc
	cmpl	$ 0, %r10d
	jle		107f
108:
	vmovapd			0(%r11), %zmm16
	vbroadcastsd	0(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm16
	vfmadd231pd		%zmm16, %zmm16, %zmm25
	vmovapd			%zmm16, 0(%r11)
	subl	$ 1, %r10d
	addq	$ 64, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_kernel_2_dgelqf_8_lib8)
#endif





// input
// r14      <- pD
// r15      <- dD
// zmm25    <- sigma
//
// output
// zmm24[0] <- beta
// zmm25    <- tmp
// zmm31    <- sign mask
//
#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DGELQF_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_dgelqf_lib8)
#endif

	vmovsd			0(%r14), %xmm24 // alpha
	vfmadd231sd		%xmm24, %xmm24, %xmm25 // beta
	vsqrtsd			%xmm25, %xmm25, %xmm25 // beta
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC00(%rip), %zmm31 // mask
#else
	vbroadcastsd	LC00(%rip), %zmm31 // mask
#endif
	vandpd			%xmm31, %xmm24, %xmm27
	vxorpd			%xmm31, %xmm27, %xmm27
	vxorpd			%xmm27, %xmm25, %xmm25 // beta
	vmovsd			%xmm25, 0(%r14) // pD[0+ps*0]
	vsubsd			%xmm24, %xmm25, %xmm24 // beta-alpha
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovddup		.LC00+1*8(%rip), %xmm27 // -1.0
#else
	vmovddup		LC00+1*8(%rip), %xmm27 // -1.0
#endif
	vmovsd			%xmm24, %xmm27, %xmm27
	vmovddup		%xmm24, %xmm24
	vmovsd			%xmm25, %xmm24, %xmm24
	vdivpd			%xmm24, %xmm27, %xmm24
	vmovsd			%xmm24, 0(%r15) // dD[0]

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd			%zmm24, %zmm27, %zmm25 // tmp

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dgelqf_lib8)
#endif





// input
// r14   <- pD
// r15   <- dD
// zmm25 <- sigma
//
// output
// zmm24[0] <- beta
// zmm25    <- tmp
// zmm31    <- sign mask
//
#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DGELQF_PD_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_edge_dgelqf_pd_lib8)
#endif

#if 0

	vmovsd			0(%r14), %xmm24 // alpha
	vmovapd			%xmm25, %xmm26 // beta
	vfmadd231sd		%xmm24, %xmm24, %xmm26 // beta
	vsqrtsd			%xmm26, %xmm26, %xmm26 // beta
	vmovsd			%xmm26, 0(%r14) // pD[0+ps*0]

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC00(%rip), %zmm31 // mask
#else
	vbroadcastsd	LC00(%rip), %zmm31 // mask
#endif

	vxorpd			%xmm27, %xmm27, %xmm27
	vucomisd		%xmm27, %xmm24
	jbe		111f
	// alpha>0
	vaddsd			%xmm26, %xmm24, %xmm24 // tmp
	vdivsd			%xmm24, %xmm25, %xmm24 // tmp
	vxorpd			%xmm31, %xmm24, %xmm24
	jmp		112f
111: // alpha<=0
	vsubsd			%xmm26, %xmm24, %xmm24 // tmp
112:
	vmulsd			%xmm24, %xmm24, %xmm27 // tmp*tmp
	vaddsd			%xmm27, %xmm25, %xmm25 // sigma+tmp*tmp
	vaddsd			%xmm27, %xmm27, %xmm27 // 2*tmp*tmp
	vdivsd			%xmm25, %xmm27, %xmm26
	vmovsd			%xmm26, 0(%r15) // dD[0]

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00+16(%rip), %xmm27
#else
	vmovsd			LC00+16(%rip), %xmm27
#endif
	vdivsd			%xmm24, %xmm27, %xmm24 // tmp
	vbroadcastsd	%xmm24, %zmm25

	vmovapd			%xmm26, %xmm24

#else

	vmovsd			0(%r14), %xmm24 // alpha
	vmovapd			%xmm25, %xmm26 // beta
	vfmadd231sd		%xmm24, %xmm24, %xmm26 // beta
	vsqrtsd			%xmm26, %xmm26, %xmm26 // beta
	vmovsd			%xmm26, 0(%r14) // pD[0+ps*0]

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC00(%rip), %zmm31 // mask
#else
	vbroadcastsd	LC00(%rip), %zmm31 // mask
#endif

	vxorpd			%xmm27, %xmm27, %xmm27
	vucomisd		%xmm27, %xmm24
	jbe		111f
	// alpha>0
	vaddsd			%xmm26, %xmm24, %xmm24 // tmp
	vdivsd			%xmm24, %xmm25, %xmm24 // tmp
	vxorpd			%xmm31, %xmm24, %xmm24
	jmp		112f
111: // alpha<=0
	vsubsd			%xmm26, %xmm24, %xmm24 // tmp
112:
	vmulsd			%xmm24, %xmm24, %xmm27 // tmp*tmp
	vaddsd			%xmm27, %xmm25, %xmm25 // sigma+tmp*tmp
	vaddsd			%xmm27, %xmm27, %xmm27 // 2*tmp*tmp
	vmovlhps		%xmm24, %xmm25, %xmm25
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00+16(%rip), %xmm26 // 1.0
#else
	vmovsd			LC00+16(%rip), %xmm26 // 1.0
#endif
	vmovlhps		%xmm26, %xmm27, %xmm27
	vdivpd			%xmm25, %xmm27, %xmm26
	vmovhlps		%xmm26, %xmm24, %xmm24 // tmp
	vmovsd			%xmm26, 0(%r15) // dD[0]
	vbroadcastsd	%xmm24, %zmm25
	vmovapd			%xmm26, %xmm24

#endif

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dgelqf_pd_lib8)
#endif





//                           1      2           3
// void kernel_dgelqf_8_lib8(int n, double *pD, double *dD)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgelqf_8_lib8)
	
	PROLOGUE


	movq	ARG2, %r11 // D
	movq	ARG3, %r12 // dD

	vxorpd			%xmm25, %xmm25, %xmm25
	movq	ARG1, %r10 // n
	subl	$ 1, %r10d
	addq	$ 64, %r11
100:
	vmovsd			0(%r11), %xmm24
	vfmadd231sd		%xmm24, %xmm24, %xmm25
	subl	$ 1, %r10d
	addq	$ 64, %r11
	cmpl	$ 0, %r10d
	jg		100b

	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		101f
	vmovsd			%xmm24, 0(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D

	leaq	0+64*0(%r11), %r14 // XXX chech for reg overwrite
	leaq	0(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_LIB8
#else
	CALL(inner_edge_dgelqf_lib8)
#endif

	vmovapd			0*64(%r11), %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
	vbroadcastsd	0+1*64(%r11), %zmm17
	vmulpd			%zmm25, %zmm17, %zmm17
	vbroadcastsd	0+2*64(%r11), %zmm18
	vmulpd			%zmm25, %zmm18, %zmm18
	vbroadcastsd	0+3*64(%r11), %zmm19
	vmulpd			%zmm25, %zmm19, %zmm19
	vbroadcastsd	0+4*64(%r11), %zmm20
	vmulpd			%zmm25, %zmm20, %zmm20
	vbroadcastsd	0+5*64(%r11), %zmm21
	vmulpd			%zmm25, %zmm21, %zmm21
	vbroadcastsd	0+6*64(%r11), %zmm22
	vmulpd			%zmm25, %zmm22, %zmm22
	vbroadcastsd	0+7*64(%r11), %zmm23
	vmulpd			%zmm25, %zmm23, %zmm23
	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
	vmovsd			%xmm17, 0+1*64(%r11)
	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
	vmovsd			%xmm18, 0+2*64(%r11)
	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
	vmovsd			%xmm19, 0+3*64(%r11)
	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
	vmovsd			%xmm20, 0+4*64(%r11)
	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
	vmovsd			%xmm21, 0+5*64(%r11)
	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
	vmovsd			%xmm22, 0+6*64(%r11)
	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
	vmovsd			%xmm23, 0+7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 0, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0xfe, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vbroadcastsd	0(%r12), %zmm25
	vxorpd			%zmm31, %zmm25, %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0 {%k1}{z}

	movq	ARG2, %r11 // D
	vmovapd			0*64(%r11), %zmm16
	vmovapd			1*64(%r11), %zmm17
	vmovapd			2*64(%r11), %zmm18
	vmovapd			3*64(%r11), %zmm19
	vmovapd			4*64(%r11), %zmm20
	vmovapd			5*64(%r11), %zmm21
	vmovapd			6*64(%r11), %zmm22
	vmovapd			7*64(%r11), %zmm23
	vaddpd			%zmm0, %zmm16, %zmm16
	vbroadcastsd	0+1*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm17
	vbroadcastsd	0+2*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm18
	vbroadcastsd	0+3*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm19
	vbroadcastsd	0+4*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm20
	vbroadcastsd	0+5*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm21
	vbroadcastsd	0+6*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm22
	vbroadcastsd	0+7*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc
	vmovapd			%zmm16, 0*64(%r11)
	vmovapd			%zmm17, 1*64(%r11)
	vfmadd231pd		%zmm18, %zmm18, %zmm25
	vmovapd			%zmm18, 2*64(%r11)
	vfmadd231pd		%zmm19, %zmm19, %zmm26
	vmovapd			%zmm19, 3*64(%r11)
	vfmadd231pd		%zmm20, %zmm20, %zmm25
	vmovapd			%zmm20, 4*64(%r11)
	vfmadd231pd		%zmm21, %zmm21, %zmm26
	vmovapd			%zmm21, 5*64(%r11)
	vfmadd231pd		%zmm22, %zmm22, %zmm25
	vmovapd			%zmm22, 6*64(%r11)
	vfmadd231pd		%zmm23, %zmm23, %zmm26
	vmovapd			%zmm23, 7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 0, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// second column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 8(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	leaq	8+64*1(%r11), %r14 // XXX chech for reg overwrite
	leaq	8(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_LIB8
#else
	CALL(inner_edge_dgelqf_lib8)
#endif

	vmovapd			1*64(%r11), %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
//	vbroadcastsd	8+1*64(%r11), %zmm17
//	vmulpd			%zmm25, %zmm17, %zmm17
	vbroadcastsd	8+2*64(%r11), %zmm18
	vmulpd			%zmm25, %zmm18, %zmm18
	vbroadcastsd	8+3*64(%r11), %zmm19
	vmulpd			%zmm25, %zmm19, %zmm19
	vbroadcastsd	8+4*64(%r11), %zmm20
	vmulpd			%zmm25, %zmm20, %zmm20
	vbroadcastsd	8+5*64(%r11), %zmm21
	vmulpd			%zmm25, %zmm21, %zmm21
	vbroadcastsd	8+6*64(%r11), %zmm22
	vmulpd			%zmm25, %zmm22, %zmm22
	vbroadcastsd	8+7*64(%r11), %zmm23
	vmulpd			%zmm25, %zmm23, %zmm23
//	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
//	vmovsd			%xmm17, 8+1*64(%r11)
	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
	vmovsd			%xmm18, 8+2*64(%r11)
	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
	vmovsd			%xmm19, 8+3*64(%r11)
	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
	vmovsd			%xmm20, 8+4*64(%r11)
	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
	vmovsd			%xmm21, 8+5*64(%r11)
	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
	vmovsd			%xmm22, 8+6*64(%r11)
	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
	vmovsd			%xmm23, 8+7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 8, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0xfc, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vbroadcastsd	8(%r12), %zmm25
	vxorpd			%zmm31, %zmm25, %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0 {%k1}{z}

	movq	ARG2, %r11 // D
//	vmovapd			0*64(%r11), %zmm16
	vmovapd			1*64(%r11), %zmm17
	vmovapd			2*64(%r11), %zmm18
	vmovapd			3*64(%r11), %zmm19
	vmovapd			4*64(%r11), %zmm20
	vmovapd			5*64(%r11), %zmm21
	vmovapd			6*64(%r11), %zmm22
	vmovapd			7*64(%r11), %zmm23
//	vaddpd			%zmm0, %zmm16, %zmm16
//	vbroadcastsd	8+1*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm17
	vaddpd			%zmm0, %zmm17, %zmm17
	vbroadcastsd	8+2*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm18
	vbroadcastsd	8+3*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm19
	vbroadcastsd	8+4*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm20
	vbroadcastsd	8+5*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm21
	vbroadcastsd	8+6*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm22
	vbroadcastsd	8+7*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc
//	vmovapd			%zmm16, 0*64(%r11)
	vmovapd			%zmm17, 1*64(%r11)
//	vfmadd231pd		%zmm18, %zmm18, %zmm25
	vmovapd			%zmm18, 2*64(%r11)
	vfmadd231pd		%zmm19, %zmm19, %zmm26
	vmovapd			%zmm19, 3*64(%r11)
	vfmadd231pd		%zmm20, %zmm20, %zmm25
	vmovapd			%zmm20, 4*64(%r11)
	vfmadd231pd		%zmm21, %zmm21, %zmm26
	vmovapd			%zmm21, 5*64(%r11)
	vfmadd231pd		%zmm22, %zmm22, %zmm25
	vmovapd			%zmm22, 6*64(%r11)
	vfmadd231pd		%zmm23, %zmm23, %zmm26
	vmovapd			%zmm23, 7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 8, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// third column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 16(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	leaq	16+64*2(%r11), %r14 // XXX chech for reg overwrite
	leaq	16(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_LIB8
#else
	CALL(inner_edge_dgelqf_lib8)
#endif

	vmovapd			2*64(%r11), %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
//	vbroadcastsd	16+1*64(%r11), %zmm17
//	vmulpd			%zmm25, %zmm17, %zmm17
//	vbroadcastsd	16+2*64(%r11), %zmm18
//	vmulpd			%zmm25, %zmm18, %zmm18
	vbroadcastsd	16+3*64(%r11), %zmm19
	vmulpd			%zmm25, %zmm19, %zmm19
	vbroadcastsd	16+4*64(%r11), %zmm20
	vmulpd			%zmm25, %zmm20, %zmm20
	vbroadcastsd	16+5*64(%r11), %zmm21
	vmulpd			%zmm25, %zmm21, %zmm21
	vbroadcastsd	16+6*64(%r11), %zmm22
	vmulpd			%zmm25, %zmm22, %zmm22
	vbroadcastsd	16+7*64(%r11), %zmm23
	vmulpd			%zmm25, %zmm23, %zmm23
//	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
//	vmovsd			%xmm17, 16+1*64(%r11)
//	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
//	vmovsd			%xmm18, 16+2*64(%r11)
	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
	vmovsd			%xmm19, 16+3*64(%r11)
	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
	vmovsd			%xmm20, 16+4*64(%r11)
	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
	vmovsd			%xmm21, 16+5*64(%r11)
	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
	vmovsd			%xmm22, 16+6*64(%r11)
	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
	vmovsd			%xmm23, 16+7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 16, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0xf8, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vbroadcastsd	16(%r12), %zmm25
	vxorpd			%zmm31, %zmm25, %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0 {%k1}{z}

	movq	ARG2, %r11 // D
//	vmovapd			0*64(%r11), %zmm16
//	vmovapd			1*64(%r11), %zmm17
	vmovapd			2*64(%r11), %zmm18
	vmovapd			3*64(%r11), %zmm19
	vmovapd			4*64(%r11), %zmm20
	vmovapd			5*64(%r11), %zmm21
	vmovapd			6*64(%r11), %zmm22
	vmovapd			7*64(%r11), %zmm23
//	vaddpd			%zmm0, %zmm16, %zmm16
//	vbroadcastsd	16+1*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm17
//	vaddpd			%zmm0, %zmm17, %zmm17
//	vbroadcastsd	16+2*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm18
	vaddpd			%zmm0, %zmm18, %zmm18
	vbroadcastsd	16+3*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm19
	vbroadcastsd	16+4*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm20
	vbroadcastsd	16+5*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm21
	vbroadcastsd	16+6*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm22
	vbroadcastsd	16+7*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc
//	vmovapd			%zmm16, 0*64(%r11)
//	vmovapd			%zmm17, 1*64(%r11)
//	vfmadd231pd		%zmm18, %zmm18, %zmm25
	vmovapd			%zmm18, 2*64(%r11)
//	vfmadd231pd		%zmm19, %zmm19, %zmm26
	vmovapd			%zmm19, 3*64(%r11)
	vfmadd231pd		%zmm20, %zmm20, %zmm25
	vmovapd			%zmm20, 4*64(%r11)
	vfmadd231pd		%zmm21, %zmm21, %zmm26
	vmovapd			%zmm21, 5*64(%r11)
	vfmadd231pd		%zmm22, %zmm22, %zmm25
	vmovapd			%zmm22, 6*64(%r11)
	vfmadd231pd		%zmm23, %zmm23, %zmm26
	vmovapd			%zmm23, 7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 16, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// fourth column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 24(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D

	leaq	24+64*3(%r11), %r14 // XXX chech for reg overwrite
	leaq	24(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_LIB8
#else
	CALL(inner_edge_dgelqf_lib8)
#endif

	vmovapd			3*64(%r11), %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
//	vbroadcastsd	24+1*64(%r11), %zmm17
//	vmulpd			%zmm25, %zmm17, %zmm17
//	vbroadcastsd	24+2*64(%r11), %zmm18
//	vmulpd			%zmm25, %zmm18, %zmm18
//	vbroadcastsd	24+3*64(%r11), %zmm19
//	vmulpd			%zmm25, %zmm19, %zmm19
	vbroadcastsd	24+4*64(%r11), %zmm20
	vmulpd			%zmm25, %zmm20, %zmm20
	vbroadcastsd	24+5*64(%r11), %zmm21
	vmulpd			%zmm25, %zmm21, %zmm21
	vbroadcastsd	24+6*64(%r11), %zmm22
	vmulpd			%zmm25, %zmm22, %zmm22
	vbroadcastsd	24+7*64(%r11), %zmm23
	vmulpd			%zmm25, %zmm23, %zmm23
//	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
//	vmovsd			%xmm17, 24+1*64(%r11)
//	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
//	vmovsd			%xmm18, 24+2*64(%r11)
//	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
//	vmovsd			%xmm19, 24+3*64(%r11)
	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
	vmovsd			%xmm20, 24+4*64(%r11)
	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
	vmovsd			%xmm21, 24+5*64(%r11)
	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
	vmovsd			%xmm22, 24+6*64(%r11)
	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
	vmovsd			%xmm23, 24+7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 24, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0xf0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vbroadcastsd	24(%r12), %zmm25
	vxorpd			%zmm31, %zmm25, %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0 {%k1}{z}

	movq	ARG2, %r11 // D
//	vmovapd			0*64(%r11), %zmm16
//	vmovapd			1*64(%r11), %zmm17
//	vmovapd			2*64(%r11), %zmm18
	vmovapd			3*64(%r11), %zmm19
	vmovapd			4*64(%r11), %zmm20
	vmovapd			5*64(%r11), %zmm21
	vmovapd			6*64(%r11), %zmm22
	vmovapd			7*64(%r11), %zmm23
//	vaddpd			%zmm0, %zmm16, %zmm16
//	vbroadcastsd	24+1*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm17
//	vaddpd			%zmm0, %zmm17, %zmm17
//	vbroadcastsd	24+2*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm18
//	vaddpd			%zmm0, %zmm18, %zmm18
//	vbroadcastsd	24+3*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm19
	vaddpd			%zmm0, %zmm19, %zmm19
	vbroadcastsd	24+4*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm20
	vbroadcastsd	24+5*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm21
	vbroadcastsd	24+6*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm22
	vbroadcastsd	24+7*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc
//	vmovapd			%zmm16, 0*64(%r11)
//	vmovapd			%zmm17, 1*64(%r11)
//	vfmadd231pd		%zmm18, %zmm18, %zmm25
//	vmovapd			%zmm18, 2*64(%r11)
//	vfmadd231pd		%zmm19, %zmm19, %zmm26
	vmovapd			%zmm19, 3*64(%r11)
//	vfmadd231pd		%zmm20, %zmm20, %zmm25
	vmovapd			%zmm20, 4*64(%r11)
	vfmadd231pd		%zmm21, %zmm21, %zmm26
	vmovapd			%zmm21, 5*64(%r11)
	vfmadd231pd		%zmm22, %zmm22, %zmm25
	vmovapd			%zmm22, 6*64(%r11)
	vfmadd231pd		%zmm23, %zmm23, %zmm26
	vmovapd			%zmm23, 7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 24, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// fifth column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 32(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D

	leaq	32+64*4(%r11), %r14 // XXX chech for reg overwrite
	leaq	32(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_LIB8
#else
	CALL(inner_edge_dgelqf_lib8)
#endif

	vmovapd			4*64(%r11), %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
//	vbroadcastsd	32+1*64(%r11), %zmm17
//	vmulpd			%zmm25, %zmm17, %zmm17
//	vbroadcastsd	32+2*64(%r11), %zmm18
//	vmulpd			%zmm25, %zmm18, %zmm18
//	vbroadcastsd	32+3*64(%r11), %zmm19
//	vmulpd			%zmm25, %zmm19, %zmm19
//	vbroadcastsd	32+4*64(%r11), %zmm20
//	vmulpd			%zmm25, %zmm20, %zmm20
	vbroadcastsd	32+5*64(%r11), %zmm21
	vmulpd			%zmm25, %zmm21, %zmm21
	vbroadcastsd	32+6*64(%r11), %zmm22
	vmulpd			%zmm25, %zmm22, %zmm22
	vbroadcastsd	32+7*64(%r11), %zmm23
	vmulpd			%zmm25, %zmm23, %zmm23
//	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
//	vmovsd			%xmm17, 32+1*64(%r11)
//	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
//	vmovsd			%xmm18, 32+2*64(%r11)
//	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
//	vmovsd			%xmm19, 32+3*64(%r11)
//	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
//	vmovsd			%xmm20, 32+4*64(%r11)
	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
	vmovsd			%xmm21, 32+5*64(%r11)
	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
	vmovsd			%xmm22, 32+6*64(%r11)
	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
	vmovsd			%xmm23, 32+7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 32, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0xe0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vbroadcastsd	32(%r12), %zmm25
	vxorpd			%zmm31, %zmm25, %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0 {%k1}{z}

	movq	ARG2, %r11 // D
//	vmovapd			0*64(%r11), %zmm16
//	vmovapd			1*64(%r11), %zmm17
//	vmovapd			2*64(%r11), %zmm18
//	vmovapd			3*64(%r11), %zmm19
	vmovapd			4*64(%r11), %zmm20
	vmovapd			5*64(%r11), %zmm21
	vmovapd			6*64(%r11), %zmm22
	vmovapd			7*64(%r11), %zmm23
//	vaddpd			%zmm0, %zmm16, %zmm16
//	vbroadcastsd	32+1*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm17
//	vaddpd			%zmm0, %zmm17, %zmm17
//	vbroadcastsd	32+2*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm18
//	vaddpd			%zmm0, %zmm18, %zmm18
//	vbroadcastsd	32+3*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm19
//	vaddpd			%zmm0, %zmm19, %zmm19
//	vbroadcastsd	32+4*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm20
	vaddpd			%zmm0, %zmm20, %zmm20
	vbroadcastsd	32+5*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm21
	vbroadcastsd	32+6*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm22
	vbroadcastsd	32+7*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc
//	vmovapd			%zmm16, 0*64(%r11)
//	vmovapd			%zmm17, 1*64(%r11)
//	vfmadd231pd		%zmm18, %zmm18, %zmm25
//	vmovapd			%zmm18, 2*64(%r11)
//	vfmadd231pd		%zmm19, %zmm19, %zmm26
//	vmovapd			%zmm19, 3*64(%r11)
//	vfmadd231pd		%zmm20, %zmm20, %zmm25
	vmovapd			%zmm20, 4*64(%r11)
//	vfmadd231pd		%zmm21, %zmm21, %zmm26
	vmovapd			%zmm21, 5*64(%r11)
	vfmadd231pd		%zmm22, %zmm22, %zmm25
	vmovapd			%zmm22, 6*64(%r11)
	vfmadd231pd		%zmm23, %zmm23, %zmm26
	vmovapd			%zmm23, 7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 32, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// sixth column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 40(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D

	leaq	40+64*5(%r11), %r14 // XXX chech for reg overwrite
	leaq	40(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_LIB8
#else
	CALL(inner_edge_dgelqf_lib8)
#endif

	vmovapd			5*64(%r11), %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
//	vbroadcastsd	40+1*64(%r11), %zmm17
//	vmulpd			%zmm25, %zmm17, %zmm17
//	vbroadcastsd	40+2*64(%r11), %zmm18
//	vmulpd			%zmm25, %zmm18, %zmm18
//	vbroadcastsd	40+3*64(%r11), %zmm19
//	vmulpd			%zmm25, %zmm19, %zmm19
//	vbroadcastsd	40+4*64(%r11), %zmm20
//	vmulpd			%zmm25, %zmm20, %zmm20
//	vbroadcastsd	40+5*64(%r11), %zmm21
//	vmulpd			%zmm25, %zmm21, %zmm21
	vbroadcastsd	40+6*64(%r11), %zmm22
	vmulpd			%zmm25, %zmm22, %zmm22
	vbroadcastsd	40+7*64(%r11), %zmm23
	vmulpd			%zmm25, %zmm23, %zmm23
//	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
//	vmovsd			%xmm17, 40+1*64(%r11)
//	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
//	vmovsd			%xmm18, 40+2*64(%r11)
//	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
//	vmovsd			%xmm19, 40+3*64(%r11)
//	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
//	vmovsd			%xmm20, 40+4*64(%r11)
//	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
//	vmovsd			%xmm21, 40+5*64(%r11)
	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
	vmovsd			%xmm22, 40+6*64(%r11)
	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
	vmovsd			%xmm23, 40+7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 40, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0xc0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vbroadcastsd	40(%r12), %zmm25
	vxorpd			%zmm31, %zmm25, %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0 {%k1}{z}

	movq	ARG2, %r11 // D
//	vmovapd			0*64(%r11), %zmm16
//	vmovapd			1*64(%r11), %zmm17
//	vmovapd			2*64(%r11), %zmm18
//	vmovapd			3*64(%r11), %zmm19
//	vmovapd			4*64(%r11), %zmm20
	vmovapd			5*64(%r11), %zmm21
	vmovapd			6*64(%r11), %zmm22
	vmovapd			7*64(%r11), %zmm23
//	vaddpd			%zmm0, %zmm16, %zmm16
//	vbroadcastsd	40+1*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm17
//	vaddpd			%zmm0, %zmm17, %zmm17
//	vbroadcastsd	40+2*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm18
//	vaddpd			%zmm0, %zmm18, %zmm18
//	vbroadcastsd	40+3*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm19
//	vaddpd			%zmm0, %zmm19, %zmm19
//	vbroadcastsd	40+4*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm20
//	vaddpd			%zmm0, %zmm20, %zmm20
//	vbroadcastsd	40+5*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm21
	vaddpd			%zmm0, %zmm21, %zmm21
	vbroadcastsd	40+6*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm22
	vbroadcastsd	40+7*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc
//	vmovapd			%zmm16, 0*64(%r11)
//	vmovapd			%zmm17, 1*64(%r11)
//	vfmadd231pd		%zmm18, %zmm18, %zmm25
//	vmovapd			%zmm18, 2*64(%r11)
//	vfmadd231pd		%zmm19, %zmm19, %zmm26
//	vmovapd			%zmm19, 3*64(%r11)
//	vfmadd231pd		%zmm20, %zmm20, %zmm25
//	vmovapd			%zmm20, 4*64(%r11)
//	vfmadd231pd		%zmm21, %zmm21, %zmm26
	vmovapd			%zmm21, 5*64(%r11)
//	vfmadd231pd		%zmm22, %zmm22, %zmm25
	vmovapd			%zmm22, 6*64(%r11)
	vfmadd231pd		%zmm23, %zmm23, %zmm26
	vmovapd			%zmm23, 7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 40, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// seventh column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 48(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D

	leaq	48+64*6(%r11), %r14 // XXX chech for reg overwrite
	leaq	48(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_LIB8
#else
	CALL(inner_edge_dgelqf_lib8)
#endif

	vmovapd			6*64(%r11), %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
//	vbroadcastsd	48+1*64(%r11), %zmm17
//	vmulpd			%zmm25, %zmm17, %zmm17
//	vbroadcastsd	48+2*64(%r11), %zmm18
//	vmulpd			%zmm25, %zmm18, %zmm18
//	vbroadcastsd	48+3*64(%r11), %zmm19
//	vmulpd			%zmm25, %zmm19, %zmm19
//	vbroadcastsd	48+4*64(%r11), %zmm20
//	vmulpd			%zmm25, %zmm20, %zmm20
//	vbroadcastsd	48+5*64(%r11), %zmm21
//	vmulpd			%zmm25, %zmm21, %zmm21
//	vbroadcastsd	48+6*64(%r11), %zmm22
//	vmulpd			%zmm25, %zmm22, %zmm22
	vbroadcastsd	48+7*64(%r11), %zmm23
	vmulpd			%zmm25, %zmm23, %zmm23
//	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
//	vmovsd			%xmm17, 48+1*64(%r11)
//	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
//	vmovsd			%xmm18, 48+2*64(%r11)
//	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
//	vmovsd			%xmm19, 48+3*64(%r11)
//	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
//	vmovsd			%xmm20, 48+4*64(%r11)
//	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
//	vmovsd			%xmm21, 48+5*64(%r11)
//	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
//	vmovsd			%xmm22, 48+6*64(%r11)
	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
	vmovsd			%xmm23, 48+7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 48, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0x80, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vbroadcastsd	48(%r12), %zmm25
	vxorpd			%zmm31, %zmm25, %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0 {%k1}{z}

	movq	ARG2, %r11 // D
//	vmovapd			0*64(%r11), %zmm16
//	vmovapd			1*64(%r11), %zmm17
//	vmovapd			2*64(%r11), %zmm18
//	vmovapd			3*64(%r11), %zmm19
//	vmovapd			4*64(%r11), %zmm20
//	vmovapd			5*64(%r11), %zmm21
	vmovapd			6*64(%r11), %zmm22
	vmovapd			7*64(%r11), %zmm23
//	vaddpd			%zmm0, %zmm16, %zmm16
//	vbroadcastsd	48+1*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm17
//	vaddpd			%zmm0, %zmm17, %zmm17
//	vbroadcastsd	48+2*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm18
//	vaddpd			%zmm0, %zmm18, %zmm18
//	vbroadcastsd	48+3*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm19
//	vaddpd			%zmm0, %zmm19, %zmm19
//	vbroadcastsd	48+4*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm20
//	vaddpd			%zmm0, %zmm20, %zmm20
//	vbroadcastsd	48+5*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm21
//	vaddpd			%zmm0, %zmm21, %zmm21
//	vbroadcastsd	48+6*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm22
	vaddpd			%zmm0, %zmm22, %zmm22
	vbroadcastsd	48+7*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc
//	vmovapd			%zmm16, 0*64(%r11)
//	vmovapd			%zmm17, 1*64(%r11)
//	vfmadd231pd		%zmm18, %zmm18, %zmm25
//	vmovapd			%zmm18, 2*64(%r11)
//	vfmadd231pd		%zmm19, %zmm19, %zmm26
//	vmovapd			%zmm19, 3*64(%r11)
//	vfmadd231pd		%zmm20, %zmm20, %zmm25
//	vmovapd			%zmm20, 4*64(%r11)
//	vfmadd231pd		%zmm21, %zmm21, %zmm26
//	vmovapd			%zmm21, 5*64(%r11)
//	vfmadd231pd		%zmm22, %zmm22, %zmm25
	vmovapd			%zmm22, 6*64(%r11)
//	vfmadd231pd		%zmm23, %zmm23, %zmm26
	vmovapd			%zmm23, 7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 48, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// eigth column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 56(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D

	leaq	56+64*7(%r11), %r14 // XXX chech for reg overwrite
	leaq	56(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_LIB8
#else
	CALL(inner_edge_dgelqf_lib8)
#endif

	vmovapd			7*64(%r11), %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
//	vbroadcastsd	56+1*64(%r11), %zmm17
//	vmulpd			%zmm25, %zmm17, %zmm17
//	vbroadcastsd	56+2*64(%r11), %zmm18
//	vmulpd			%zmm25, %zmm18, %zmm18
//	vbroadcastsd	56+3*64(%r11), %zmm19
//	vmulpd			%zmm25, %zmm19, %zmm19
//	vbroadcastsd	56+4*64(%r11), %zmm20
//	vmulpd			%zmm25, %zmm20, %zmm20
//	vbroadcastsd	56+5*64(%r11), %zmm21
//	vmulpd			%zmm25, %zmm21, %zmm21
//	vbroadcastsd	56+6*64(%r11), %zmm22
//	vmulpd			%zmm25, %zmm22, %zmm22
//	vbroadcastsd	56+7*64(%r11), %zmm23
//	vmulpd			%zmm25, %zmm23, %zmm23
//	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
//	vmovsd			%xmm17, 56+1*64(%r11)
//	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
//	vmovsd			%xmm18, 56+2*64(%r11)
//	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
//	vmovsd			%xmm19, 56+3*64(%r11)
//	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
//	vmovsd			%xmm20, 56+4*64(%r11)
//	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
//	vmovsd			%xmm21, 56+5*64(%r11)
//	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
//	vmovsd			%xmm22, 56+6*64(%r11)
//	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
//	vmovsd			%xmm23, 56+7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 56, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

102:

	EPILOGUE
	
	ret

	FUN_END(kernel_dgelqf_8_lib8)





//                                  1      2           3           4
// void kernel_dgelqf_dlarft8_8_lib8(int n, double *pD, double *dD, double *pT)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgelqf_dlarft8_8_lib8)
	
	PROLOGUE

	// zero T

	movq	ARG4, %r10 // T

	vxorpd			%zmm24, %zmm24, %zmm24
	vmovapd			%zmm24, 0*64(%r10)
	vmovapd			%zmm24, 1*64(%r10)
	vmovapd			%zmm24, 2*64(%r10)
	vmovapd			%zmm24, 3*64(%r10)
	vmovapd			%zmm24, 4*64(%r10)
	vmovapd			%zmm24, 5*64(%r10)
	vmovapd			%zmm24, 6*64(%r10)
	vmovapd			%zmm24, 7*64(%r10)

	// first column

	movq	ARG2, %r11 // D
	movq	ARG3, %r12 // dD
	movq	ARG4, %r13 // T

	vxorpd			%xmm25, %xmm25, %xmm25
	movq	ARG1, %r10 // n
	subl	$ 1, %r10d
	addq	$ 64, %r11
100:
	vmovsd			0(%r11), %xmm24
	vfmadd231sd		%xmm24, %xmm24, %xmm25
	subl	$ 1, %r10d
	addq	$ 64, %r11
	cmpl	$ 0, %r10d
	jg		100b

	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		101f
	vmovsd			%xmm24, 0(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D

	leaq	0+64*0(%r11), %r14 // XXX chech for reg overwrite
	leaq	0(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_LIB8
#else
	CALL(inner_edge_dgelqf_lib8)
#endif

	vxorpd			%xmm31, %xmm24, %xmm27
	vmovsd			%xmm27, 0+64*0(%r13) // pT[0+ps*0]

	vmovapd			0*64(%r11), %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
	vbroadcastsd	0+1*64(%r11), %zmm17
	vmulpd			%zmm25, %zmm17, %zmm17
	vbroadcastsd	0+2*64(%r11), %zmm18
	vmulpd			%zmm25, %zmm18, %zmm18
	vbroadcastsd	0+3*64(%r11), %zmm19
	vmulpd			%zmm25, %zmm19, %zmm19
	vbroadcastsd	0+4*64(%r11), %zmm20
	vmulpd			%zmm25, %zmm20, %zmm20
	vbroadcastsd	0+5*64(%r11), %zmm21
	vmulpd			%zmm25, %zmm21, %zmm21
	vbroadcastsd	0+6*64(%r11), %zmm22
	vmulpd			%zmm25, %zmm22, %zmm22
	vbroadcastsd	0+7*64(%r11), %zmm23
	vmulpd			%zmm25, %zmm23, %zmm23
	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
	vmovsd			%xmm17, 0+1*64(%r11)
	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
	vmovsd			%xmm18, 0+2*64(%r11)
	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
	vmovsd			%xmm19, 0+3*64(%r11)
	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
	vmovsd			%xmm20, 0+4*64(%r11)
	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
	vmovsd			%xmm21, 0+5*64(%r11)
	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
	vmovsd			%xmm22, 0+6*64(%r11)
	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
	vmovsd			%xmm23, 0+7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 0, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0xfe, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vbroadcastsd	0(%r13), %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0 {%k1}{z}

	movq	ARG2, %r11 // D
	vmovapd			0*64(%r11), %zmm16
	vmovapd			1*64(%r11), %zmm17
	vmovapd			2*64(%r11), %zmm18
	vmovapd			3*64(%r11), %zmm19
	vmovapd			4*64(%r11), %zmm20
	vmovapd			5*64(%r11), %zmm21
	vmovapd			6*64(%r11), %zmm22
	vmovapd			7*64(%r11), %zmm23
	vaddpd			%zmm0, %zmm16, %zmm16
	vbroadcastsd	0+1*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm17
	vbroadcastsd	0+2*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm18
	vbroadcastsd	0+3*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm19
	vbroadcastsd	0+4*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm20
	vbroadcastsd	0+5*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm21
	vbroadcastsd	0+6*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm22
	vbroadcastsd	0+7*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc
	vmovapd			%zmm16, 0*64(%r11)
	vmovapd			%zmm17, 1*64(%r11)
	vfmadd231pd		%zmm18, %zmm18, %zmm25
	vmovapd			%zmm18, 2*64(%r11)
	vfmadd231pd		%zmm19, %zmm19, %zmm26
	vmovapd			%zmm19, 3*64(%r11)
	vfmadd231pd		%zmm20, %zmm20, %zmm25
	vmovapd			%zmm20, 4*64(%r11)
	vfmadd231pd		%zmm21, %zmm21, %zmm26
	vmovapd			%zmm21, 5*64(%r11)
	vfmadd231pd		%zmm22, %zmm22, %zmm25
	vmovapd			%zmm22, 6*64(%r11)
	vfmadd231pd		%zmm23, %zmm23, %zmm26
	vmovapd			%zmm23, 7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 0, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// second column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 8(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D

	leaq	8+64*1(%r11), %r14 // XXX chech for reg overwrite
	leaq	8(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_LIB8
#else
	CALL(inner_edge_dgelqf_lib8)
#endif

	vxorpd			%xmm31, %xmm24, %xmm27
	vmovsd			%xmm27, 8+64*1(%r13) // pT[1+ps*1]

	vmovapd			1*64(%r11), %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
//	vbroadcastsd	8+1*64(%r11), %zmm17
//	vmulpd			%zmm25, %zmm17, %zmm17
	vbroadcastsd	8+2*64(%r11), %zmm18
	vmulpd			%zmm25, %zmm18, %zmm18
	vbroadcastsd	8+3*64(%r11), %zmm19
	vmulpd			%zmm25, %zmm19, %zmm19
	vbroadcastsd	8+4*64(%r11), %zmm20
	vmulpd			%zmm25, %zmm20, %zmm20
	vbroadcastsd	8+5*64(%r11), %zmm21
	vmulpd			%zmm25, %zmm21, %zmm21
	vbroadcastsd	8+6*64(%r11), %zmm22
	vmulpd			%zmm25, %zmm22, %zmm22
	vbroadcastsd	8+7*64(%r11), %zmm23
	vmulpd			%zmm25, %zmm23, %zmm23
//	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
//	vmovsd			%xmm17, 8+1*64(%r11)
	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
	vmovsd			%xmm18, 8+2*64(%r11)
	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
	vmovsd			%xmm19, 8+3*64(%r11)
	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
	vmovsd			%xmm20, 8+4*64(%r11)
	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
	vmovsd			%xmm21, 8+5*64(%r11)
	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
	vmovsd			%xmm22, 8+6*64(%r11)
	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
	vmovsd			%xmm23, 8+7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 8, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC00+16(%rip), %zmm27 // 1.0
#else
	vbroadcastsd	LC00+16(%rip), %zmm27 // 1.0
#endif
	movl	$ 0x01, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			0+64*0(%r13), %zmm27 {%k1}
	vmulpd			%zmm27, %zmm0, %zmm0
	vbroadcastsd	8+64*1(%r13), %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0
	vmovapd			%zmm0, 1*64(%r13) {%k1}

	movl	$ 0xfc, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm0, %zmm0 {%k1}{z}

	movq	ARG2, %r11 // D
//	vmovapd			0*64(%r11), %zmm16
	vmovapd			1*64(%r11), %zmm17
	vmovapd			2*64(%r11), %zmm18
	vmovapd			3*64(%r11), %zmm19
	vmovapd			4*64(%r11), %zmm20
	vmovapd			5*64(%r11), %zmm21
	vmovapd			6*64(%r11), %zmm22
	vmovapd			7*64(%r11), %zmm23
//	vaddpd			%zmm0, %zmm16, %zmm16
//	vbroadcastsd	8+1*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm17
	vaddpd			%zmm0, %zmm17, %zmm17
	vbroadcastsd	8+2*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm18
	vbroadcastsd	8+3*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm19
	vbroadcastsd	8+4*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm20
	vbroadcastsd	8+5*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm21
	vbroadcastsd	8+6*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm22
	vbroadcastsd	8+7*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc
//	vmovapd			%zmm16, 0*64(%r11)
	vmovapd			%zmm17, 1*64(%r11)
//	vfmadd231pd		%zmm18, %zmm18, %zmm25
	vmovapd			%zmm18, 2*64(%r11)
	vfmadd231pd		%zmm19, %zmm19, %zmm26
	vmovapd			%zmm19, 3*64(%r11)
	vfmadd231pd		%zmm20, %zmm20, %zmm25
	vmovapd			%zmm20, 4*64(%r11)
	vfmadd231pd		%zmm21, %zmm21, %zmm26
	vmovapd			%zmm21, 5*64(%r11)
	vfmadd231pd		%zmm22, %zmm22, %zmm25
	vmovapd			%zmm22, 6*64(%r11)
	vfmadd231pd		%zmm23, %zmm23, %zmm26
	vmovapd			%zmm23, 7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 8, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// third column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 16(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D

	leaq	16+64*2(%r11), %r14 // XXX chech for reg overwrite
	leaq	16(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_LIB8
#else
	CALL(inner_edge_dgelqf_lib8)
#endif

	vxorpd			%xmm31, %xmm24, %xmm27
	vmovsd			%xmm27, 16+64*2(%r13) // pT[1+ps*1]

	vmovapd			2*64(%r11), %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
//	vbroadcastsd	16+1*64(%r11), %zmm17
//	vmulpd			%zmm25, %zmm17, %zmm17
//	vbroadcastsd	16+2*64(%r11), %zmm18
//	vmulpd			%zmm25, %zmm18, %zmm18
	vbroadcastsd	16+3*64(%r11), %zmm19
	vmulpd			%zmm25, %zmm19, %zmm19
	vbroadcastsd	16+4*64(%r11), %zmm20
	vmulpd			%zmm25, %zmm20, %zmm20
	vbroadcastsd	16+5*64(%r11), %zmm21
	vmulpd			%zmm25, %zmm21, %zmm21
	vbroadcastsd	16+6*64(%r11), %zmm22
	vmulpd			%zmm25, %zmm22, %zmm22
	vbroadcastsd	16+7*64(%r11), %zmm23
	vmulpd			%zmm25, %zmm23, %zmm23
//	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
//	vmovsd			%xmm17, 16+1*64(%r11)
//	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
//	vmovsd			%xmm18, 16+2*64(%r11)
	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
	vmovsd			%xmm19, 16+3*64(%r11)
	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
	vmovsd			%xmm20, 16+4*64(%r11)
	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
	vmovsd			%xmm21, 16+5*64(%r11)
	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
	vmovsd			%xmm22, 16+6*64(%r11)
	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
	vmovsd			%xmm23, 16+7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 16, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0x01, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			0*64(%r13), %zmm24
	vmulpd			%zmm24, %zmm0, %zmm1 {%k1}{z}
	vmovapd			1*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x03, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	vmovapd			%zmm1, %zmm0 {%k1}
	vbroadcastsd	16+2*64(%r13), %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0
	vmovapd			%zmm0, 2*64(%r13) {%k1}

	movl	$ 0xf8, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm0, %zmm0 {%k1}{z}

	movq	ARG2, %r11 // D
//	vmovapd			0*64(%r11), %zmm16
//	vmovapd			1*64(%r11), %zmm17
	vmovapd			2*64(%r11), %zmm18
	vmovapd			3*64(%r11), %zmm19
	vmovapd			4*64(%r11), %zmm20
	vmovapd			5*64(%r11), %zmm21
	vmovapd			6*64(%r11), %zmm22
	vmovapd			7*64(%r11), %zmm23
//	vaddpd			%zmm0, %zmm16, %zmm16
//	vbroadcastsd	16+1*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm17
//	vaddpd			%zmm0, %zmm17, %zmm17
//	vbroadcastsd	16+2*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm18
	vaddpd			%zmm0, %zmm18, %zmm18
	vbroadcastsd	16+3*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm19
	vbroadcastsd	16+4*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm20
	vbroadcastsd	16+5*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm21
	vbroadcastsd	16+6*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm22
	vbroadcastsd	16+7*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc
//	vmovapd			%zmm16, 0*64(%r11)
//	vmovapd			%zmm17, 1*64(%r11)
//	vfmadd231pd		%zmm18, %zmm18, %zmm25
	vmovapd			%zmm18, 2*64(%r11)
//	vfmadd231pd		%zmm19, %zmm19, %zmm26
	vmovapd			%zmm19, 3*64(%r11)
	vfmadd231pd		%zmm20, %zmm20, %zmm25
	vmovapd			%zmm20, 4*64(%r11)
	vfmadd231pd		%zmm21, %zmm21, %zmm26
	vmovapd			%zmm21, 5*64(%r11)
	vfmadd231pd		%zmm22, %zmm22, %zmm25
	vmovapd			%zmm22, 6*64(%r11)
	vfmadd231pd		%zmm23, %zmm23, %zmm26
	vmovapd			%zmm23, 7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 16, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// fourth column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 24(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D

	leaq	24+64*3(%r11), %r14 // XXX chech for reg overwrite
	leaq	24(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_LIB8
#else
	CALL(inner_edge_dgelqf_lib8)
#endif

	vxorpd			%xmm31, %xmm24, %xmm27
	vmovsd			%xmm27, 24+64*3(%r13) // pT[1+ps*1]

	vmovapd			3*64(%r11), %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
//	vbroadcastsd	24+1*64(%r11), %zmm17
//	vmulpd			%zmm25, %zmm17, %zmm17
//	vbroadcastsd	24+2*64(%r11), %zmm18
//	vmulpd			%zmm25, %zmm18, %zmm18
//	vbroadcastsd	24+3*64(%r11), %zmm19
//	vmulpd			%zmm25, %zmm19, %zmm19
	vbroadcastsd	24+4*64(%r11), %zmm20
	vmulpd			%zmm25, %zmm20, %zmm20
	vbroadcastsd	24+5*64(%r11), %zmm21
	vmulpd			%zmm25, %zmm21, %zmm21
	vbroadcastsd	24+6*64(%r11), %zmm22
	vmulpd			%zmm25, %zmm22, %zmm22
	vbroadcastsd	24+7*64(%r11), %zmm23
	vmulpd			%zmm25, %zmm23, %zmm23
//	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
//	vmovsd			%xmm17, 24+1*64(%r11)
//	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
//	vmovsd			%xmm18, 24+2*64(%r11)
//	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
//	vmovsd			%xmm19, 24+3*64(%r11)
	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
	vmovsd			%xmm20, 24+4*64(%r11)
	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
	vmovsd			%xmm21, 24+5*64(%r11)
	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
	vmovsd			%xmm22, 24+6*64(%r11)
	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
	vmovsd			%xmm23, 24+7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 24, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0x01, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			0*64(%r13), %zmm24
	vmulpd			%zmm24, %zmm0, %zmm1 {%k1}{z}
	vmovapd			1*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x03, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	vmovapd			2*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x07, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	vmovapd			%zmm1, %zmm0 {%k1}
	vbroadcastsd	24+3*64(%r13), %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0
	vmovapd			%zmm0, 3*64(%r13) {%k1}

	movl	$ 0xf0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm0, %zmm0 {%k1}{z}

	movq	ARG2, %r11 // D
//	vmovapd			0*64(%r11), %zmm16
//	vmovapd			1*64(%r11), %zmm17
//	vmovapd			2*64(%r11), %zmm18
	vmovapd			3*64(%r11), %zmm19
	vmovapd			4*64(%r11), %zmm20
	vmovapd			5*64(%r11), %zmm21
	vmovapd			6*64(%r11), %zmm22
	vmovapd			7*64(%r11), %zmm23
//	vaddpd			%zmm0, %zmm16, %zmm16
//	vbroadcastsd	24+1*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm17
//	vaddpd			%zmm0, %zmm17, %zmm17
//	vbroadcastsd	24+2*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm18
//	vaddpd			%zmm0, %zmm18, %zmm18
//	vbroadcastsd	24+3*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm19
	vaddpd			%zmm0, %zmm19, %zmm19
	vbroadcastsd	24+4*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm20
	vbroadcastsd	24+5*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm21
	vbroadcastsd	24+6*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm22
	vbroadcastsd	24+7*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc
//	vmovapd			%zmm16, 0*64(%r11)
//	vmovapd			%zmm17, 1*64(%r11)
//	vfmadd231pd		%zmm18, %zmm18, %zmm25
//	vmovapd			%zmm18, 2*64(%r11)
//	vfmadd231pd		%zmm19, %zmm19, %zmm26
	vmovapd			%zmm19, 3*64(%r11)
//	vfmadd231pd		%zmm20, %zmm20, %zmm25
	vmovapd			%zmm20, 4*64(%r11)
	vfmadd231pd		%zmm21, %zmm21, %zmm26
	vmovapd			%zmm21, 5*64(%r11)
	vfmadd231pd		%zmm22, %zmm22, %zmm25
	vmovapd			%zmm22, 6*64(%r11)
	vfmadd231pd		%zmm23, %zmm23, %zmm26
	vmovapd			%zmm23, 7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 24, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// fifth column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 32(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D

	leaq	32+64*4(%r11), %r14 // XXX chech for reg overwrite
	leaq	32(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_LIB8
#else
	CALL(inner_edge_dgelqf_lib8)
#endif

	vxorpd			%xmm31, %xmm24, %xmm27
	vmovsd			%xmm27, 32+64*4(%r13) // pT[1+ps*1]

	vmovapd			4*64(%r11), %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
//	vbroadcastsd	32+1*64(%r11), %zmm17
//	vmulpd			%zmm25, %zmm17, %zmm17
//	vbroadcastsd	32+2*64(%r11), %zmm18
//	vmulpd			%zmm25, %zmm18, %zmm18
//	vbroadcastsd	32+3*64(%r11), %zmm19
//	vmulpd			%zmm25, %zmm19, %zmm19
//	vbroadcastsd	32+4*64(%r11), %zmm20
//	vmulpd			%zmm25, %zmm20, %zmm20
	vbroadcastsd	32+5*64(%r11), %zmm21
	vmulpd			%zmm25, %zmm21, %zmm21
	vbroadcastsd	32+6*64(%r11), %zmm22
	vmulpd			%zmm25, %zmm22, %zmm22
	vbroadcastsd	32+7*64(%r11), %zmm23
	vmulpd			%zmm25, %zmm23, %zmm23
//	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
//	vmovsd			%xmm17, 32+1*64(%r11)
//	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
//	vmovsd			%xmm18, 32+2*64(%r11)
//	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
//	vmovsd			%xmm19, 32+3*64(%r11)
//	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
//	vmovsd			%xmm20, 32+4*64(%r11)
	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
	vmovsd			%xmm21, 32+5*64(%r11)
	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
	vmovsd			%xmm22, 32+6*64(%r11)
	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
	vmovsd			%xmm23, 32+7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 32, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0x01, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			0*64(%r13), %zmm24
	vmulpd			%zmm24, %zmm0, %zmm1 {%k1}{z}
	//
	vmovapd			1*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x03, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			2*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x07, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			3*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x0f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			%zmm1, %zmm0 {%k1}
	vbroadcastsd	32+4*64(%r13), %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0
	vmovapd			%zmm0, 4*64(%r13) {%k1}

	movl	$ 0xe0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm0, %zmm0 {%k1}{z}

	movq	ARG2, %r11 // D
//	vmovapd			0*64(%r11), %zmm16
//	vmovapd			1*64(%r11), %zmm17
//	vmovapd			2*64(%r11), %zmm18
//	vmovapd			3*64(%r11), %zmm19
	vmovapd			4*64(%r11), %zmm20
	vmovapd			5*64(%r11), %zmm21
	vmovapd			6*64(%r11), %zmm22
	vmovapd			7*64(%r11), %zmm23
//	vaddpd			%zmm0, %zmm16, %zmm16
//	vbroadcastsd	32+1*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm17
//	vaddpd			%zmm0, %zmm17, %zmm17
//	vbroadcastsd	32+2*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm18
//	vaddpd			%zmm0, %zmm18, %zmm18
//	vbroadcastsd	32+3*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm19
//	vaddpd			%zmm0, %zmm19, %zmm19
//	vbroadcastsd	32+4*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm20
	vaddpd			%zmm0, %zmm20, %zmm20
	vbroadcastsd	32+5*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm21
	vbroadcastsd	32+6*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm22
	vbroadcastsd	32+7*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc
//	vmovapd			%zmm16, 0*64(%r11)
//	vmovapd			%zmm17, 1*64(%r11)
//	vfmadd231pd		%zmm18, %zmm18, %zmm25
//	vmovapd			%zmm18, 2*64(%r11)
//	vfmadd231pd		%zmm19, %zmm19, %zmm26
//	vmovapd			%zmm19, 3*64(%r11)
//	vfmadd231pd		%zmm20, %zmm20, %zmm25
	vmovapd			%zmm20, 4*64(%r11)
//	vfmadd231pd		%zmm21, %zmm21, %zmm26
	vmovapd			%zmm21, 5*64(%r11)
	vfmadd231pd		%zmm22, %zmm22, %zmm25
	vmovapd			%zmm22, 6*64(%r11)
	vfmadd231pd		%zmm23, %zmm23, %zmm26
	vmovapd			%zmm23, 7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 32, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// sixth column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 40(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D

	leaq	40+64*5(%r11), %r14 // XXX chech for reg overwrite
	leaq	40(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_LIB8
#else
	CALL(inner_edge_dgelqf_lib8)
#endif

	vxorpd			%xmm31, %xmm24, %xmm27
	vmovsd			%xmm27, 40+64*5(%r13) // pT[1+ps*1]

	vmovapd			5*64(%r11), %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
//	vbroadcastsd	40+1*64(%r11), %zmm17
//	vmulpd			%zmm25, %zmm17, %zmm17
//	vbroadcastsd	40+2*64(%r11), %zmm18
//	vmulpd			%zmm25, %zmm18, %zmm18
//	vbroadcastsd	40+3*64(%r11), %zmm19
//	vmulpd			%zmm25, %zmm19, %zmm19
//	vbroadcastsd	40+4*64(%r11), %zmm20
//	vmulpd			%zmm25, %zmm20, %zmm20
//	vbroadcastsd	40+5*64(%r11), %zmm21
//	vmulpd			%zmm25, %zmm21, %zmm21
	vbroadcastsd	40+6*64(%r11), %zmm22
	vmulpd			%zmm25, %zmm22, %zmm22
	vbroadcastsd	40+7*64(%r11), %zmm23
	vmulpd			%zmm25, %zmm23, %zmm23
//	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
//	vmovsd			%xmm17, 40+1*64(%r11)
//	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
//	vmovsd			%xmm18, 40+2*64(%r11)
//	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
//	vmovsd			%xmm19, 40+3*64(%r11)
//	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
//	vmovsd			%xmm20, 40+4*64(%r11)
//	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
//	vmovsd			%xmm21, 40+5*64(%r11)
	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
	vmovsd			%xmm22, 40+6*64(%r11)
	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
	vmovsd			%xmm23, 40+7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 40, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0x01, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			0*64(%r13), %zmm24
	vmulpd			%zmm24, %zmm0, %zmm1 {%k1}{z}
	//
	vmovapd			1*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x03, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			2*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x07, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			3*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x0f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			4*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x1f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			%zmm1, %zmm0 {%k1}
	vbroadcastsd	40+5*64(%r13), %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0
	vmovapd			%zmm0, 5*64(%r13) {%k1}

	movl	$ 0xc0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm0, %zmm0 {%k1}{z}

	movq	ARG2, %r11 // D
//	vmovapd			0*64(%r11), %zmm16
//	vmovapd			1*64(%r11), %zmm17
//	vmovapd			2*64(%r11), %zmm18
//	vmovapd			3*64(%r11), %zmm19
//	vmovapd			4*64(%r11), %zmm20
	vmovapd			5*64(%r11), %zmm21
	vmovapd			6*64(%r11), %zmm22
	vmovapd			7*64(%r11), %zmm23
//	vaddpd			%zmm0, %zmm16, %zmm16
//	vbroadcastsd	40+1*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm17
//	vaddpd			%zmm0, %zmm17, %zmm17
//	vbroadcastsd	40+2*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm18
//	vaddpd			%zmm0, %zmm18, %zmm18
//	vbroadcastsd	40+3*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm19
//	vaddpd			%zmm0, %zmm19, %zmm19
//	vbroadcastsd	40+4*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm20
//	vaddpd			%zmm0, %zmm20, %zmm20
//	vbroadcastsd	40+5*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm21
	vaddpd			%zmm0, %zmm21, %zmm21
	vbroadcastsd	40+6*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm22
	vbroadcastsd	40+7*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc
//	vmovapd			%zmm16, 0*64(%r11)
//	vmovapd			%zmm17, 1*64(%r11)
//	vfmadd231pd		%zmm18, %zmm18, %zmm25
//	vmovapd			%zmm18, 2*64(%r11)
//	vfmadd231pd		%zmm19, %zmm19, %zmm26
//	vmovapd			%zmm19, 3*64(%r11)
//	vfmadd231pd		%zmm20, %zmm20, %zmm25
//	vmovapd			%zmm20, 4*64(%r11)
//	vfmadd231pd		%zmm21, %zmm21, %zmm26
	vmovapd			%zmm21, 5*64(%r11)
//	vfmadd231pd		%zmm22, %zmm22, %zmm25
	vmovapd			%zmm22, 6*64(%r11)
	vfmadd231pd		%zmm23, %zmm23, %zmm26
	vmovapd			%zmm23, 7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 40, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// seventh column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 48(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D

	leaq	48+64*6(%r11), %r14 // XXX chech for reg overwrite
	leaq	48(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_LIB8
#else
	CALL(inner_edge_dgelqf_lib8)
#endif

	vxorpd			%xmm31, %xmm24, %xmm27
	vmovsd			%xmm27, 48+64*6(%r13) // pT[1+ps*1]

	vmovapd			6*64(%r11), %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
//	vbroadcastsd	48+1*64(%r11), %zmm17
//	vmulpd			%zmm25, %zmm17, %zmm17
//	vbroadcastsd	48+2*64(%r11), %zmm18
//	vmulpd			%zmm25, %zmm18, %zmm18
//	vbroadcastsd	48+3*64(%r11), %zmm19
//	vmulpd			%zmm25, %zmm19, %zmm19
//	vbroadcastsd	48+4*64(%r11), %zmm20
//	vmulpd			%zmm25, %zmm20, %zmm20
//	vbroadcastsd	48+5*64(%r11), %zmm21
//	vmulpd			%zmm25, %zmm21, %zmm21
//	vbroadcastsd	48+6*64(%r11), %zmm22
//	vmulpd			%zmm25, %zmm22, %zmm22
	vbroadcastsd	48+7*64(%r11), %zmm23
	vmulpd			%zmm25, %zmm23, %zmm23
//	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
//	vmovsd			%xmm17, 48+1*64(%r11)
//	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
//	vmovsd			%xmm18, 48+2*64(%r11)
//	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
//	vmovsd			%xmm19, 48+3*64(%r11)
//	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
//	vmovsd			%xmm20, 48+4*64(%r11)
//	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
//	vmovsd			%xmm21, 48+5*64(%r11)
//	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
//	vmovsd			%xmm22, 48+6*64(%r11)
	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
	vmovsd			%xmm23, 48+7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 48, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0x01, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			0*64(%r13), %zmm24
	vmulpd			%zmm24, %zmm0, %zmm1 {%k1}{z}
	//
	vmovapd			1*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x03, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			2*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x07, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			3*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x0f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			4*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x1f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			5*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x3f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			%zmm1, %zmm0 {%k1}
	vbroadcastsd	48+6*64(%r13), %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0
	vmovapd			%zmm0, 6*64(%r13) {%k1}

	movl	$ 0x80, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm0, %zmm0 {%k1}{z}

	movq	ARG2, %r11 // D
//	vmovapd			0*64(%r11), %zmm16
//	vmovapd			1*64(%r11), %zmm17
//	vmovapd			2*64(%r11), %zmm18
//	vmovapd			3*64(%r11), %zmm19
//	vmovapd			4*64(%r11), %zmm20
//	vmovapd			5*64(%r11), %zmm21
	vmovapd			6*64(%r11), %zmm22
	vmovapd			7*64(%r11), %zmm23
//	vaddpd			%zmm0, %zmm16, %zmm16
//	vbroadcastsd	48+1*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm17
//	vaddpd			%zmm0, %zmm17, %zmm17
//	vbroadcastsd	48+2*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm18
//	vaddpd			%zmm0, %zmm18, %zmm18
//	vbroadcastsd	48+3*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm19
//	vaddpd			%zmm0, %zmm19, %zmm19
//	vbroadcastsd	48+4*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm20
//	vaddpd			%zmm0, %zmm20, %zmm20
//	vbroadcastsd	48+5*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm21
//	vaddpd			%zmm0, %zmm21, %zmm21
//	vbroadcastsd	48+6*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm22
	vaddpd			%zmm0, %zmm22, %zmm22
	vbroadcastsd	48+7*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc
//	vmovapd			%zmm16, 0*64(%r11)
//	vmovapd			%zmm17, 1*64(%r11)
//	vfmadd231pd		%zmm18, %zmm18, %zmm25
//	vmovapd			%zmm18, 2*64(%r11)
//	vfmadd231pd		%zmm19, %zmm19, %zmm26
//	vmovapd			%zmm19, 3*64(%r11)
//	vfmadd231pd		%zmm20, %zmm20, %zmm25
//	vmovapd			%zmm20, 4*64(%r11)
//	vfmadd231pd		%zmm21, %zmm21, %zmm26
//	vmovapd			%zmm21, 5*64(%r11)
//	vfmadd231pd		%zmm22, %zmm22, %zmm25
	vmovapd			%zmm22, 6*64(%r11)
//	vfmadd231pd		%zmm23, %zmm23, %zmm26
	vmovapd			%zmm23, 7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 48, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// eigth column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 56(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D

	leaq	56+64*7(%r11), %r14 // XXX chech for reg overwrite
	leaq	56(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_LIB8
#else
	CALL(inner_edge_dgelqf_lib8)
#endif

	vxorpd			%xmm31, %xmm24, %xmm27
	vmovsd			%xmm27, 56+64*7(%r13) // pT[1+ps*1]

	vmovapd			7*64(%r11), %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
//	vbroadcastsd	56+1*64(%r11), %zmm17
//	vmulpd			%zmm25, %zmm17, %zmm17
//	vbroadcastsd	56+2*64(%r11), %zmm18
//	vmulpd			%zmm25, %zmm18, %zmm18
//	vbroadcastsd	56+3*64(%r11), %zmm19
//	vmulpd			%zmm25, %zmm19, %zmm19
//	vbroadcastsd	56+4*64(%r11), %zmm20
//	vmulpd			%zmm25, %zmm20, %zmm20
//	vbroadcastsd	56+5*64(%r11), %zmm21
//	vmulpd			%zmm25, %zmm21, %zmm21
//	vbroadcastsd	56+6*64(%r11), %zmm22
//	vmulpd			%zmm25, %zmm22, %zmm22
//	vbroadcastsd	56+7*64(%r11), %zmm23
//	vmulpd			%zmm25, %zmm23, %zmm23
//	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
//	vmovsd			%xmm17, 56+1*64(%r11)
//	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
//	vmovsd			%xmm18, 56+2*64(%r11)
//	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
//	vmovsd			%xmm19, 56+3*64(%r11)
//	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
//	vmovsd			%xmm20, 56+4*64(%r11)
//	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
//	vmovsd			%xmm21, 56+5*64(%r11)
//	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
//	vmovsd			%xmm22, 56+6*64(%r11)
//	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
//	vmovsd			%xmm23, 56+7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 56, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0x01, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			0*64(%r13), %zmm24
	vmulpd			%zmm24, %zmm0, %zmm1 {%k1}{z}
	//
	vmovapd			1*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x03, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			2*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x07, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			3*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x0f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			4*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x1f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			5*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x3f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			6*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x7f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			%zmm1, %zmm0 {%k1}
	vbroadcastsd	56+7*64(%r13), %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0
	vmovapd			%zmm0, 7*64(%r13) {%k1}

102:

	EPILOGUE
	
	ret

	FUN_END(kernel_dgelqf_dlarft8_8_lib8)





//                           1      2           3
// void kernel_dgelqf_pd_8_lib8(int n, double *pD, double *dD)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgelqf_pd_8_lib8)
	
	PROLOGUE


	movq	ARG2, %r11 // D
	movq	ARG3, %r12 // dD

	vxorpd			%xmm25, %xmm25, %xmm25
	movq	ARG1, %r10 // n
	subl	$ 1, %r10d
	addq	$ 64, %r11
100:
	vmovsd			0(%r11), %xmm24
	vfmadd231sd		%xmm24, %xmm24, %xmm25
	subl	$ 1, %r10d
	addq	$ 64, %r11
	cmpl	$ 0, %r10d
	jg		100b

	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		101f
	vmovsd			%xmm24, 0(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D

	leaq	0+64*0(%r11), %r14 // XXX chech for reg overwrite
	leaq	0(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_PD_LIB8
#else
	CALL(inner_edge_dgelqf_pd_lib8)
#endif

	vmovapd			0*64(%r11), %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
	vbroadcastsd	0+1*64(%r11), %zmm17
	vmulpd			%zmm25, %zmm17, %zmm17
	vbroadcastsd	0+2*64(%r11), %zmm18
	vmulpd			%zmm25, %zmm18, %zmm18
	vbroadcastsd	0+3*64(%r11), %zmm19
	vmulpd			%zmm25, %zmm19, %zmm19
	vbroadcastsd	0+4*64(%r11), %zmm20
	vmulpd			%zmm25, %zmm20, %zmm20
	vbroadcastsd	0+5*64(%r11), %zmm21
	vmulpd			%zmm25, %zmm21, %zmm21
	vbroadcastsd	0+6*64(%r11), %zmm22
	vmulpd			%zmm25, %zmm22, %zmm22
	vbroadcastsd	0+7*64(%r11), %zmm23
	vmulpd			%zmm25, %zmm23, %zmm23
	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
	vmovsd			%xmm17, 0+1*64(%r11)
	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
	vmovsd			%xmm18, 0+2*64(%r11)
	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
	vmovsd			%xmm19, 0+3*64(%r11)
	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
	vmovsd			%xmm20, 0+4*64(%r11)
	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
	vmovsd			%xmm21, 0+5*64(%r11)
	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
	vmovsd			%xmm22, 0+6*64(%r11)
	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
	vmovsd			%xmm23, 0+7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 0, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0xfe, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vbroadcastsd	0(%r12), %zmm25
	vxorpd			%zmm31, %zmm25, %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0 {%k1}{z}

	movq	ARG2, %r11 // D
	vmovapd			0*64(%r11), %zmm16
	vmovapd			1*64(%r11), %zmm17
	vmovapd			2*64(%r11), %zmm18
	vmovapd			3*64(%r11), %zmm19
	vmovapd			4*64(%r11), %zmm20
	vmovapd			5*64(%r11), %zmm21
	vmovapd			6*64(%r11), %zmm22
	vmovapd			7*64(%r11), %zmm23
	vaddpd			%zmm0, %zmm16, %zmm16
	vbroadcastsd	0+1*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm17
	vbroadcastsd	0+2*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm18
	vbroadcastsd	0+3*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm19
	vbroadcastsd	0+4*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm20
	vbroadcastsd	0+5*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm21
	vbroadcastsd	0+6*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm22
	vbroadcastsd	0+7*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc
	vmovapd			%zmm16, 0*64(%r11)
	vmovapd			%zmm17, 1*64(%r11)
	vfmadd231pd		%zmm18, %zmm18, %zmm25
	vmovapd			%zmm18, 2*64(%r11)
	vfmadd231pd		%zmm19, %zmm19, %zmm26
	vmovapd			%zmm19, 3*64(%r11)
	vfmadd231pd		%zmm20, %zmm20, %zmm25
	vmovapd			%zmm20, 4*64(%r11)
	vfmadd231pd		%zmm21, %zmm21, %zmm26
	vmovapd			%zmm21, 5*64(%r11)
	vfmadd231pd		%zmm22, %zmm22, %zmm25
	vmovapd			%zmm22, 6*64(%r11)
	vfmadd231pd		%zmm23, %zmm23, %zmm26
	vmovapd			%zmm23, 7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 0, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// second column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 8(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	leaq	8+64*1(%r11), %r14 // XXX chech for reg overwrite
	leaq	8(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_PD_LIB8
#else
	CALL(inner_edge_dgelqf_pd_lib8)
#endif

	vmovapd			1*64(%r11), %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
//	vbroadcastsd	8+1*64(%r11), %zmm17
//	vmulpd			%zmm25, %zmm17, %zmm17
	vbroadcastsd	8+2*64(%r11), %zmm18
	vmulpd			%zmm25, %zmm18, %zmm18
	vbroadcastsd	8+3*64(%r11), %zmm19
	vmulpd			%zmm25, %zmm19, %zmm19
	vbroadcastsd	8+4*64(%r11), %zmm20
	vmulpd			%zmm25, %zmm20, %zmm20
	vbroadcastsd	8+5*64(%r11), %zmm21
	vmulpd			%zmm25, %zmm21, %zmm21
	vbroadcastsd	8+6*64(%r11), %zmm22
	vmulpd			%zmm25, %zmm22, %zmm22
	vbroadcastsd	8+7*64(%r11), %zmm23
	vmulpd			%zmm25, %zmm23, %zmm23
//	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
//	vmovsd			%xmm17, 8+1*64(%r11)
	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
	vmovsd			%xmm18, 8+2*64(%r11)
	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
	vmovsd			%xmm19, 8+3*64(%r11)
	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
	vmovsd			%xmm20, 8+4*64(%r11)
	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
	vmovsd			%xmm21, 8+5*64(%r11)
	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
	vmovsd			%xmm22, 8+6*64(%r11)
	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
	vmovsd			%xmm23, 8+7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 8, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0xfc, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vbroadcastsd	8(%r12), %zmm25
	vxorpd			%zmm31, %zmm25, %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0 {%k1}{z}

	movq	ARG2, %r11 // D
//	vmovapd			0*64(%r11), %zmm16
	vmovapd			1*64(%r11), %zmm17
	vmovapd			2*64(%r11), %zmm18
	vmovapd			3*64(%r11), %zmm19
	vmovapd			4*64(%r11), %zmm20
	vmovapd			5*64(%r11), %zmm21
	vmovapd			6*64(%r11), %zmm22
	vmovapd			7*64(%r11), %zmm23
//	vaddpd			%zmm0, %zmm16, %zmm16
//	vbroadcastsd	8+1*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm17
	vaddpd			%zmm0, %zmm17, %zmm17
	vbroadcastsd	8+2*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm18
	vbroadcastsd	8+3*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm19
	vbroadcastsd	8+4*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm20
	vbroadcastsd	8+5*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm21
	vbroadcastsd	8+6*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm22
	vbroadcastsd	8+7*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc
//	vmovapd			%zmm16, 0*64(%r11)
	vmovapd			%zmm17, 1*64(%r11)
//	vfmadd231pd		%zmm18, %zmm18, %zmm25
	vmovapd			%zmm18, 2*64(%r11)
	vfmadd231pd		%zmm19, %zmm19, %zmm26
	vmovapd			%zmm19, 3*64(%r11)
	vfmadd231pd		%zmm20, %zmm20, %zmm25
	vmovapd			%zmm20, 4*64(%r11)
	vfmadd231pd		%zmm21, %zmm21, %zmm26
	vmovapd			%zmm21, 5*64(%r11)
	vfmadd231pd		%zmm22, %zmm22, %zmm25
	vmovapd			%zmm22, 6*64(%r11)
	vfmadd231pd		%zmm23, %zmm23, %zmm26
	vmovapd			%zmm23, 7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 8, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// third column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 16(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	leaq	16+64*2(%r11), %r14 // XXX chech for reg overwrite
	leaq	16(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_PD_LIB8
#else
	CALL(inner_edge_dgelqf_pd_lib8)
#endif

	vmovapd			2*64(%r11), %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
//	vbroadcastsd	16+1*64(%r11), %zmm17
//	vmulpd			%zmm25, %zmm17, %zmm17
//	vbroadcastsd	16+2*64(%r11), %zmm18
//	vmulpd			%zmm25, %zmm18, %zmm18
	vbroadcastsd	16+3*64(%r11), %zmm19
	vmulpd			%zmm25, %zmm19, %zmm19
	vbroadcastsd	16+4*64(%r11), %zmm20
	vmulpd			%zmm25, %zmm20, %zmm20
	vbroadcastsd	16+5*64(%r11), %zmm21
	vmulpd			%zmm25, %zmm21, %zmm21
	vbroadcastsd	16+6*64(%r11), %zmm22
	vmulpd			%zmm25, %zmm22, %zmm22
	vbroadcastsd	16+7*64(%r11), %zmm23
	vmulpd			%zmm25, %zmm23, %zmm23
//	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
//	vmovsd			%xmm17, 16+1*64(%r11)
//	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
//	vmovsd			%xmm18, 16+2*64(%r11)
	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
	vmovsd			%xmm19, 16+3*64(%r11)
	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
	vmovsd			%xmm20, 16+4*64(%r11)
	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
	vmovsd			%xmm21, 16+5*64(%r11)
	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
	vmovsd			%xmm22, 16+6*64(%r11)
	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
	vmovsd			%xmm23, 16+7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 16, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0xf8, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vbroadcastsd	16(%r12), %zmm25
	vxorpd			%zmm31, %zmm25, %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0 {%k1}{z}

	movq	ARG2, %r11 // D
//	vmovapd			0*64(%r11), %zmm16
//	vmovapd			1*64(%r11), %zmm17
	vmovapd			2*64(%r11), %zmm18
	vmovapd			3*64(%r11), %zmm19
	vmovapd			4*64(%r11), %zmm20
	vmovapd			5*64(%r11), %zmm21
	vmovapd			6*64(%r11), %zmm22
	vmovapd			7*64(%r11), %zmm23
//	vaddpd			%zmm0, %zmm16, %zmm16
//	vbroadcastsd	16+1*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm17
//	vaddpd			%zmm0, %zmm17, %zmm17
//	vbroadcastsd	16+2*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm18
	vaddpd			%zmm0, %zmm18, %zmm18
	vbroadcastsd	16+3*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm19
	vbroadcastsd	16+4*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm20
	vbroadcastsd	16+5*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm21
	vbroadcastsd	16+6*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm22
	vbroadcastsd	16+7*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc
//	vmovapd			%zmm16, 0*64(%r11)
//	vmovapd			%zmm17, 1*64(%r11)
//	vfmadd231pd		%zmm18, %zmm18, %zmm25
	vmovapd			%zmm18, 2*64(%r11)
//	vfmadd231pd		%zmm19, %zmm19, %zmm26
	vmovapd			%zmm19, 3*64(%r11)
	vfmadd231pd		%zmm20, %zmm20, %zmm25
	vmovapd			%zmm20, 4*64(%r11)
	vfmadd231pd		%zmm21, %zmm21, %zmm26
	vmovapd			%zmm21, 5*64(%r11)
	vfmadd231pd		%zmm22, %zmm22, %zmm25
	vmovapd			%zmm22, 6*64(%r11)
	vfmadd231pd		%zmm23, %zmm23, %zmm26
	vmovapd			%zmm23, 7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 16, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// fourth column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 24(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D

	leaq	24+64*3(%r11), %r14 // XXX chech for reg overwrite
	leaq	24(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_PD_LIB8
#else
	CALL(inner_edge_dgelqf_pd_lib8)
#endif

	vmovapd			3*64(%r11), %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
//	vbroadcastsd	24+1*64(%r11), %zmm17
//	vmulpd			%zmm25, %zmm17, %zmm17
//	vbroadcastsd	24+2*64(%r11), %zmm18
//	vmulpd			%zmm25, %zmm18, %zmm18
//	vbroadcastsd	24+3*64(%r11), %zmm19
//	vmulpd			%zmm25, %zmm19, %zmm19
	vbroadcastsd	24+4*64(%r11), %zmm20
	vmulpd			%zmm25, %zmm20, %zmm20
	vbroadcastsd	24+5*64(%r11), %zmm21
	vmulpd			%zmm25, %zmm21, %zmm21
	vbroadcastsd	24+6*64(%r11), %zmm22
	vmulpd			%zmm25, %zmm22, %zmm22
	vbroadcastsd	24+7*64(%r11), %zmm23
	vmulpd			%zmm25, %zmm23, %zmm23
//	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
//	vmovsd			%xmm17, 24+1*64(%r11)
//	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
//	vmovsd			%xmm18, 24+2*64(%r11)
//	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
//	vmovsd			%xmm19, 24+3*64(%r11)
	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
	vmovsd			%xmm20, 24+4*64(%r11)
	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
	vmovsd			%xmm21, 24+5*64(%r11)
	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
	vmovsd			%xmm22, 24+6*64(%r11)
	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
	vmovsd			%xmm23, 24+7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 24, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0xf0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vbroadcastsd	24(%r12), %zmm25
	vxorpd			%zmm31, %zmm25, %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0 {%k1}{z}

	movq	ARG2, %r11 // D
//	vmovapd			0*64(%r11), %zmm16
//	vmovapd			1*64(%r11), %zmm17
//	vmovapd			2*64(%r11), %zmm18
	vmovapd			3*64(%r11), %zmm19
	vmovapd			4*64(%r11), %zmm20
	vmovapd			5*64(%r11), %zmm21
	vmovapd			6*64(%r11), %zmm22
	vmovapd			7*64(%r11), %zmm23
//	vaddpd			%zmm0, %zmm16, %zmm16
//	vbroadcastsd	24+1*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm17
//	vaddpd			%zmm0, %zmm17, %zmm17
//	vbroadcastsd	24+2*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm18
//	vaddpd			%zmm0, %zmm18, %zmm18
//	vbroadcastsd	24+3*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm19
	vaddpd			%zmm0, %zmm19, %zmm19
	vbroadcastsd	24+4*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm20
	vbroadcastsd	24+5*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm21
	vbroadcastsd	24+6*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm22
	vbroadcastsd	24+7*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc
//	vmovapd			%zmm16, 0*64(%r11)
//	vmovapd			%zmm17, 1*64(%r11)
//	vfmadd231pd		%zmm18, %zmm18, %zmm25
//	vmovapd			%zmm18, 2*64(%r11)
//	vfmadd231pd		%zmm19, %zmm19, %zmm26
	vmovapd			%zmm19, 3*64(%r11)
//	vfmadd231pd		%zmm20, %zmm20, %zmm25
	vmovapd			%zmm20, 4*64(%r11)
	vfmadd231pd		%zmm21, %zmm21, %zmm26
	vmovapd			%zmm21, 5*64(%r11)
	vfmadd231pd		%zmm22, %zmm22, %zmm25
	vmovapd			%zmm22, 6*64(%r11)
	vfmadd231pd		%zmm23, %zmm23, %zmm26
	vmovapd			%zmm23, 7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 24, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// fifth column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 32(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D

	leaq	32+64*4(%r11), %r14 // XXX chech for reg overwrite
	leaq	32(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_PD_LIB8
#else
	CALL(inner_edge_dgelqf_pd_lib8)
#endif

	vmovapd			4*64(%r11), %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
//	vbroadcastsd	32+1*64(%r11), %zmm17
//	vmulpd			%zmm25, %zmm17, %zmm17
//	vbroadcastsd	32+2*64(%r11), %zmm18
//	vmulpd			%zmm25, %zmm18, %zmm18
//	vbroadcastsd	32+3*64(%r11), %zmm19
//	vmulpd			%zmm25, %zmm19, %zmm19
//	vbroadcastsd	32+4*64(%r11), %zmm20
//	vmulpd			%zmm25, %zmm20, %zmm20
	vbroadcastsd	32+5*64(%r11), %zmm21
	vmulpd			%zmm25, %zmm21, %zmm21
	vbroadcastsd	32+6*64(%r11), %zmm22
	vmulpd			%zmm25, %zmm22, %zmm22
	vbroadcastsd	32+7*64(%r11), %zmm23
	vmulpd			%zmm25, %zmm23, %zmm23
//	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
//	vmovsd			%xmm17, 32+1*64(%r11)
//	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
//	vmovsd			%xmm18, 32+2*64(%r11)
//	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
//	vmovsd			%xmm19, 32+3*64(%r11)
//	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
//	vmovsd			%xmm20, 32+4*64(%r11)
	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
	vmovsd			%xmm21, 32+5*64(%r11)
	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
	vmovsd			%xmm22, 32+6*64(%r11)
	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
	vmovsd			%xmm23, 32+7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 32, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0xe0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vbroadcastsd	32(%r12), %zmm25
	vxorpd			%zmm31, %zmm25, %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0 {%k1}{z}

	movq	ARG2, %r11 // D
//	vmovapd			0*64(%r11), %zmm16
//	vmovapd			1*64(%r11), %zmm17
//	vmovapd			2*64(%r11), %zmm18
//	vmovapd			3*64(%r11), %zmm19
	vmovapd			4*64(%r11), %zmm20
	vmovapd			5*64(%r11), %zmm21
	vmovapd			6*64(%r11), %zmm22
	vmovapd			7*64(%r11), %zmm23
//	vaddpd			%zmm0, %zmm16, %zmm16
//	vbroadcastsd	32+1*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm17
//	vaddpd			%zmm0, %zmm17, %zmm17
//	vbroadcastsd	32+2*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm18
//	vaddpd			%zmm0, %zmm18, %zmm18
//	vbroadcastsd	32+3*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm19
//	vaddpd			%zmm0, %zmm19, %zmm19
//	vbroadcastsd	32+4*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm20
	vaddpd			%zmm0, %zmm20, %zmm20
	vbroadcastsd	32+5*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm21
	vbroadcastsd	32+6*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm22
	vbroadcastsd	32+7*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc
//	vmovapd			%zmm16, 0*64(%r11)
//	vmovapd			%zmm17, 1*64(%r11)
//	vfmadd231pd		%zmm18, %zmm18, %zmm25
//	vmovapd			%zmm18, 2*64(%r11)
//	vfmadd231pd		%zmm19, %zmm19, %zmm26
//	vmovapd			%zmm19, 3*64(%r11)
//	vfmadd231pd		%zmm20, %zmm20, %zmm25
	vmovapd			%zmm20, 4*64(%r11)
//	vfmadd231pd		%zmm21, %zmm21, %zmm26
	vmovapd			%zmm21, 5*64(%r11)
	vfmadd231pd		%zmm22, %zmm22, %zmm25
	vmovapd			%zmm22, 6*64(%r11)
	vfmadd231pd		%zmm23, %zmm23, %zmm26
	vmovapd			%zmm23, 7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 32, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// sixth column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 40(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D

	leaq	40+64*5(%r11), %r14 // XXX chech for reg overwrite
	leaq	40(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_PD_LIB8
#else
	CALL(inner_edge_dgelqf_pd_lib8)
#endif

	vmovapd			5*64(%r11), %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
//	vbroadcastsd	40+1*64(%r11), %zmm17
//	vmulpd			%zmm25, %zmm17, %zmm17
//	vbroadcastsd	40+2*64(%r11), %zmm18
//	vmulpd			%zmm25, %zmm18, %zmm18
//	vbroadcastsd	40+3*64(%r11), %zmm19
//	vmulpd			%zmm25, %zmm19, %zmm19
//	vbroadcastsd	40+4*64(%r11), %zmm20
//	vmulpd			%zmm25, %zmm20, %zmm20
//	vbroadcastsd	40+5*64(%r11), %zmm21
//	vmulpd			%zmm25, %zmm21, %zmm21
	vbroadcastsd	40+6*64(%r11), %zmm22
	vmulpd			%zmm25, %zmm22, %zmm22
	vbroadcastsd	40+7*64(%r11), %zmm23
	vmulpd			%zmm25, %zmm23, %zmm23
//	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
//	vmovsd			%xmm17, 40+1*64(%r11)
//	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
//	vmovsd			%xmm18, 40+2*64(%r11)
//	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
//	vmovsd			%xmm19, 40+3*64(%r11)
//	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
//	vmovsd			%xmm20, 40+4*64(%r11)
//	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
//	vmovsd			%xmm21, 40+5*64(%r11)
	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
	vmovsd			%xmm22, 40+6*64(%r11)
	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
	vmovsd			%xmm23, 40+7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 40, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0xc0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vbroadcastsd	40(%r12), %zmm25
	vxorpd			%zmm31, %zmm25, %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0 {%k1}{z}

	movq	ARG2, %r11 // D
//	vmovapd			0*64(%r11), %zmm16
//	vmovapd			1*64(%r11), %zmm17
//	vmovapd			2*64(%r11), %zmm18
//	vmovapd			3*64(%r11), %zmm19
//	vmovapd			4*64(%r11), %zmm20
	vmovapd			5*64(%r11), %zmm21
	vmovapd			6*64(%r11), %zmm22
	vmovapd			7*64(%r11), %zmm23
//	vaddpd			%zmm0, %zmm16, %zmm16
//	vbroadcastsd	40+1*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm17
//	vaddpd			%zmm0, %zmm17, %zmm17
//	vbroadcastsd	40+2*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm18
//	vaddpd			%zmm0, %zmm18, %zmm18
//	vbroadcastsd	40+3*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm19
//	vaddpd			%zmm0, %zmm19, %zmm19
//	vbroadcastsd	40+4*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm20
//	vaddpd			%zmm0, %zmm20, %zmm20
//	vbroadcastsd	40+5*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm21
	vaddpd			%zmm0, %zmm21, %zmm21
	vbroadcastsd	40+6*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm22
	vbroadcastsd	40+7*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc
//	vmovapd			%zmm16, 0*64(%r11)
//	vmovapd			%zmm17, 1*64(%r11)
//	vfmadd231pd		%zmm18, %zmm18, %zmm25
//	vmovapd			%zmm18, 2*64(%r11)
//	vfmadd231pd		%zmm19, %zmm19, %zmm26
//	vmovapd			%zmm19, 3*64(%r11)
//	vfmadd231pd		%zmm20, %zmm20, %zmm25
//	vmovapd			%zmm20, 4*64(%r11)
//	vfmadd231pd		%zmm21, %zmm21, %zmm26
	vmovapd			%zmm21, 5*64(%r11)
//	vfmadd231pd		%zmm22, %zmm22, %zmm25
	vmovapd			%zmm22, 6*64(%r11)
	vfmadd231pd		%zmm23, %zmm23, %zmm26
	vmovapd			%zmm23, 7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 40, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// seventh column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 48(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D

	leaq	48+64*6(%r11), %r14 // XXX chech for reg overwrite
	leaq	48(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_PD_LIB8
#else
	CALL(inner_edge_dgelqf_pd_lib8)
#endif

	vmovapd			6*64(%r11), %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
//	vbroadcastsd	48+1*64(%r11), %zmm17
//	vmulpd			%zmm25, %zmm17, %zmm17
//	vbroadcastsd	48+2*64(%r11), %zmm18
//	vmulpd			%zmm25, %zmm18, %zmm18
//	vbroadcastsd	48+3*64(%r11), %zmm19
//	vmulpd			%zmm25, %zmm19, %zmm19
//	vbroadcastsd	48+4*64(%r11), %zmm20
//	vmulpd			%zmm25, %zmm20, %zmm20
//	vbroadcastsd	48+5*64(%r11), %zmm21
//	vmulpd			%zmm25, %zmm21, %zmm21
//	vbroadcastsd	48+6*64(%r11), %zmm22
//	vmulpd			%zmm25, %zmm22, %zmm22
	vbroadcastsd	48+7*64(%r11), %zmm23
	vmulpd			%zmm25, %zmm23, %zmm23
//	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
//	vmovsd			%xmm17, 48+1*64(%r11)
//	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
//	vmovsd			%xmm18, 48+2*64(%r11)
//	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
//	vmovsd			%xmm19, 48+3*64(%r11)
//	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
//	vmovsd			%xmm20, 48+4*64(%r11)
//	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
//	vmovsd			%xmm21, 48+5*64(%r11)
//	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
//	vmovsd			%xmm22, 48+6*64(%r11)
	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
	vmovsd			%xmm23, 48+7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 48, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0x80, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vbroadcastsd	48(%r12), %zmm25
	vxorpd			%zmm31, %zmm25, %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0 {%k1}{z}

	movq	ARG2, %r11 // D
//	vmovapd			0*64(%r11), %zmm16
//	vmovapd			1*64(%r11), %zmm17
//	vmovapd			2*64(%r11), %zmm18
//	vmovapd			3*64(%r11), %zmm19
//	vmovapd			4*64(%r11), %zmm20
//	vmovapd			5*64(%r11), %zmm21
	vmovapd			6*64(%r11), %zmm22
	vmovapd			7*64(%r11), %zmm23
//	vaddpd			%zmm0, %zmm16, %zmm16
//	vbroadcastsd	48+1*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm17
//	vaddpd			%zmm0, %zmm17, %zmm17
//	vbroadcastsd	48+2*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm18
//	vaddpd			%zmm0, %zmm18, %zmm18
//	vbroadcastsd	48+3*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm19
//	vaddpd			%zmm0, %zmm19, %zmm19
//	vbroadcastsd	48+4*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm20
//	vaddpd			%zmm0, %zmm20, %zmm20
//	vbroadcastsd	48+5*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm21
//	vaddpd			%zmm0, %zmm21, %zmm21
//	vbroadcastsd	48+6*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm22
	vaddpd			%zmm0, %zmm22, %zmm22
	vbroadcastsd	48+7*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc
//	vmovapd			%zmm16, 0*64(%r11)
//	vmovapd			%zmm17, 1*64(%r11)
//	vfmadd231pd		%zmm18, %zmm18, %zmm25
//	vmovapd			%zmm18, 2*64(%r11)
//	vfmadd231pd		%zmm19, %zmm19, %zmm26
//	vmovapd			%zmm19, 3*64(%r11)
//	vfmadd231pd		%zmm20, %zmm20, %zmm25
//	vmovapd			%zmm20, 4*64(%r11)
//	vfmadd231pd		%zmm21, %zmm21, %zmm26
//	vmovapd			%zmm21, 5*64(%r11)
//	vfmadd231pd		%zmm22, %zmm22, %zmm25
	vmovapd			%zmm22, 6*64(%r11)
//	vfmadd231pd		%zmm23, %zmm23, %zmm26
	vmovapd			%zmm23, 7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 48, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// eigth column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 56(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D

	leaq	56+64*7(%r11), %r14 // XXX chech for reg overwrite
	leaq	56(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_PD_LIB8
#else
	CALL(inner_edge_dgelqf_pd_lib8)
#endif

	vmovapd			7*64(%r11), %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
//	vbroadcastsd	56+1*64(%r11), %zmm17
//	vmulpd			%zmm25, %zmm17, %zmm17
//	vbroadcastsd	56+2*64(%r11), %zmm18
//	vmulpd			%zmm25, %zmm18, %zmm18
//	vbroadcastsd	56+3*64(%r11), %zmm19
//	vmulpd			%zmm25, %zmm19, %zmm19
//	vbroadcastsd	56+4*64(%r11), %zmm20
//	vmulpd			%zmm25, %zmm20, %zmm20
//	vbroadcastsd	56+5*64(%r11), %zmm21
//	vmulpd			%zmm25, %zmm21, %zmm21
//	vbroadcastsd	56+6*64(%r11), %zmm22
//	vmulpd			%zmm25, %zmm22, %zmm22
//	vbroadcastsd	56+7*64(%r11), %zmm23
//	vmulpd			%zmm25, %zmm23, %zmm23
//	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
//	vmovsd			%xmm17, 56+1*64(%r11)
//	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
//	vmovsd			%xmm18, 56+2*64(%r11)
//	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
//	vmovsd			%xmm19, 56+3*64(%r11)
//	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
//	vmovsd			%xmm20, 56+4*64(%r11)
//	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
//	vmovsd			%xmm21, 56+5*64(%r11)
//	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
//	vmovsd			%xmm22, 56+6*64(%r11)
//	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
//	vmovsd			%xmm23, 56+7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 56, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

102:

	EPILOGUE
	
	ret

	FUN_END(kernel_dgelqf_pd_8_lib8)





//                                  1      2           3           4
// void kernel_dgelqf_pd_dlarft8_8_lib8(int n, double *pD, double *dD, double *pT)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgelqf_pd_dlarft8_8_lib8)
	
	PROLOGUE

	// zero T

	movq	ARG4, %r10 // T

	vxorpd			%zmm24, %zmm24, %zmm24
	vmovapd			%zmm24, 0*64(%r10)
	vmovapd			%zmm24, 1*64(%r10)
	vmovapd			%zmm24, 2*64(%r10)
	vmovapd			%zmm24, 3*64(%r10)
	vmovapd			%zmm24, 4*64(%r10)
	vmovapd			%zmm24, 5*64(%r10)
	vmovapd			%zmm24, 6*64(%r10)
	vmovapd			%zmm24, 7*64(%r10)

	// first column

	movq	ARG2, %r11 // D
	movq	ARG3, %r12 // dD
	movq	ARG4, %r13 // T

	vxorpd			%xmm25, %xmm25, %xmm25
	movq	ARG1, %r10 // n
	subl	$ 1, %r10d
	addq	$ 64, %r11
100:
	vmovsd			0(%r11), %xmm24
	vfmadd231sd		%xmm24, %xmm24, %xmm25
	subl	$ 1, %r10d
	addq	$ 64, %r11
	cmpl	$ 0, %r10d
	jg		100b

	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		101f
	vmovsd			%xmm24, 0(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D

	leaq	0+64*0(%r11), %r14 // XXX chech for reg overwrite
	leaq	0(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_PD_LIB8
#else
	CALL(inner_edge_dgelqf_pd_lib8)
#endif

	vxorpd			%xmm31, %xmm24, %xmm27
	vmovsd			%xmm27, 0+64*0(%r13) // pT[0+ps*0]

	vmovapd			0*64(%r11), %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
	vbroadcastsd	0+1*64(%r11), %zmm17
	vmulpd			%zmm25, %zmm17, %zmm17
	vbroadcastsd	0+2*64(%r11), %zmm18
	vmulpd			%zmm25, %zmm18, %zmm18
	vbroadcastsd	0+3*64(%r11), %zmm19
	vmulpd			%zmm25, %zmm19, %zmm19
	vbroadcastsd	0+4*64(%r11), %zmm20
	vmulpd			%zmm25, %zmm20, %zmm20
	vbroadcastsd	0+5*64(%r11), %zmm21
	vmulpd			%zmm25, %zmm21, %zmm21
	vbroadcastsd	0+6*64(%r11), %zmm22
	vmulpd			%zmm25, %zmm22, %zmm22
	vbroadcastsd	0+7*64(%r11), %zmm23
	vmulpd			%zmm25, %zmm23, %zmm23
	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
	vmovsd			%xmm17, 0+1*64(%r11)
	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
	vmovsd			%xmm18, 0+2*64(%r11)
	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
	vmovsd			%xmm19, 0+3*64(%r11)
	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
	vmovsd			%xmm20, 0+4*64(%r11)
	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
	vmovsd			%xmm21, 0+5*64(%r11)
	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
	vmovsd			%xmm22, 0+6*64(%r11)
	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
	vmovsd			%xmm23, 0+7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 0, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0xfe, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vbroadcastsd	0(%r13), %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0 {%k1}{z}

	movq	ARG2, %r11 // D
	vmovapd			0*64(%r11), %zmm16
	vmovapd			1*64(%r11), %zmm17
	vmovapd			2*64(%r11), %zmm18
	vmovapd			3*64(%r11), %zmm19
	vmovapd			4*64(%r11), %zmm20
	vmovapd			5*64(%r11), %zmm21
	vmovapd			6*64(%r11), %zmm22
	vmovapd			7*64(%r11), %zmm23
	vaddpd			%zmm0, %zmm16, %zmm16
	vbroadcastsd	0+1*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm17
	vbroadcastsd	0+2*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm18
	vbroadcastsd	0+3*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm19
	vbroadcastsd	0+4*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm20
	vbroadcastsd	0+5*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm21
	vbroadcastsd	0+6*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm22
	vbroadcastsd	0+7*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc
	vmovapd			%zmm16, 0*64(%r11)
	vmovapd			%zmm17, 1*64(%r11)
	vfmadd231pd		%zmm18, %zmm18, %zmm25
	vmovapd			%zmm18, 2*64(%r11)
	vfmadd231pd		%zmm19, %zmm19, %zmm26
	vmovapd			%zmm19, 3*64(%r11)
	vfmadd231pd		%zmm20, %zmm20, %zmm25
	vmovapd			%zmm20, 4*64(%r11)
	vfmadd231pd		%zmm21, %zmm21, %zmm26
	vmovapd			%zmm21, 5*64(%r11)
	vfmadd231pd		%zmm22, %zmm22, %zmm25
	vmovapd			%zmm22, 6*64(%r11)
	vfmadd231pd		%zmm23, %zmm23, %zmm26
	vmovapd			%zmm23, 7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 0, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// second column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 8(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D

	leaq	8+64*1(%r11), %r14 // XXX chech for reg overwrite
	leaq	8(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_PD_LIB8
#else
	CALL(inner_edge_dgelqf_pd_lib8)
#endif

	vxorpd			%xmm31, %xmm24, %xmm27
	vmovsd			%xmm27, 8+64*1(%r13) // pT[1+ps*1]

	vmovapd			1*64(%r11), %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
//	vbroadcastsd	8+1*64(%r11), %zmm17
//	vmulpd			%zmm25, %zmm17, %zmm17
	vbroadcastsd	8+2*64(%r11), %zmm18
	vmulpd			%zmm25, %zmm18, %zmm18
	vbroadcastsd	8+3*64(%r11), %zmm19
	vmulpd			%zmm25, %zmm19, %zmm19
	vbroadcastsd	8+4*64(%r11), %zmm20
	vmulpd			%zmm25, %zmm20, %zmm20
	vbroadcastsd	8+5*64(%r11), %zmm21
	vmulpd			%zmm25, %zmm21, %zmm21
	vbroadcastsd	8+6*64(%r11), %zmm22
	vmulpd			%zmm25, %zmm22, %zmm22
	vbroadcastsd	8+7*64(%r11), %zmm23
	vmulpd			%zmm25, %zmm23, %zmm23
//	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
//	vmovsd			%xmm17, 8+1*64(%r11)
	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
	vmovsd			%xmm18, 8+2*64(%r11)
	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
	vmovsd			%xmm19, 8+3*64(%r11)
	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
	vmovsd			%xmm20, 8+4*64(%r11)
	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
	vmovsd			%xmm21, 8+5*64(%r11)
	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
	vmovsd			%xmm22, 8+6*64(%r11)
	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
	vmovsd			%xmm23, 8+7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 8, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC00+16(%rip), %zmm27 // 1.0
#else
	vbroadcastsd	LC00+16(%rip), %zmm27 // 1.0
#endif
	movl	$ 0x01, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			0+64*0(%r13), %zmm27 {%k1}
	vmulpd			%zmm27, %zmm0, %zmm0
	vbroadcastsd	8+64*1(%r13), %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0
	vmovapd			%zmm0, 1*64(%r13) {%k1}

	movl	$ 0xfc, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm0, %zmm0 {%k1}{z}

	movq	ARG2, %r11 // D
//	vmovapd			0*64(%r11), %zmm16
	vmovapd			1*64(%r11), %zmm17
	vmovapd			2*64(%r11), %zmm18
	vmovapd			3*64(%r11), %zmm19
	vmovapd			4*64(%r11), %zmm20
	vmovapd			5*64(%r11), %zmm21
	vmovapd			6*64(%r11), %zmm22
	vmovapd			7*64(%r11), %zmm23
//	vaddpd			%zmm0, %zmm16, %zmm16
//	vbroadcastsd	8+1*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm17
	vaddpd			%zmm0, %zmm17, %zmm17
	vbroadcastsd	8+2*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm18
	vbroadcastsd	8+3*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm19
	vbroadcastsd	8+4*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm20
	vbroadcastsd	8+5*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm21
	vbroadcastsd	8+6*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm22
	vbroadcastsd	8+7*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc
//	vmovapd			%zmm16, 0*64(%r11)
	vmovapd			%zmm17, 1*64(%r11)
//	vfmadd231pd		%zmm18, %zmm18, %zmm25
	vmovapd			%zmm18, 2*64(%r11)
	vfmadd231pd		%zmm19, %zmm19, %zmm26
	vmovapd			%zmm19, 3*64(%r11)
	vfmadd231pd		%zmm20, %zmm20, %zmm25
	vmovapd			%zmm20, 4*64(%r11)
	vfmadd231pd		%zmm21, %zmm21, %zmm26
	vmovapd			%zmm21, 5*64(%r11)
	vfmadd231pd		%zmm22, %zmm22, %zmm25
	vmovapd			%zmm22, 6*64(%r11)
	vfmadd231pd		%zmm23, %zmm23, %zmm26
	vmovapd			%zmm23, 7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 8, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// third column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 16(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D

	leaq	16+64*2(%r11), %r14 // XXX chech for reg overwrite
	leaq	16(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_PD_LIB8
#else
	CALL(inner_edge_dgelqf_pd_lib8)
#endif

	vxorpd			%xmm31, %xmm24, %xmm27
	vmovsd			%xmm27, 16+64*2(%r13) // pT[1+ps*1]

	vmovapd			2*64(%r11), %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
//	vbroadcastsd	16+1*64(%r11), %zmm17
//	vmulpd			%zmm25, %zmm17, %zmm17
//	vbroadcastsd	16+2*64(%r11), %zmm18
//	vmulpd			%zmm25, %zmm18, %zmm18
	vbroadcastsd	16+3*64(%r11), %zmm19
	vmulpd			%zmm25, %zmm19, %zmm19
	vbroadcastsd	16+4*64(%r11), %zmm20
	vmulpd			%zmm25, %zmm20, %zmm20
	vbroadcastsd	16+5*64(%r11), %zmm21
	vmulpd			%zmm25, %zmm21, %zmm21
	vbroadcastsd	16+6*64(%r11), %zmm22
	vmulpd			%zmm25, %zmm22, %zmm22
	vbroadcastsd	16+7*64(%r11), %zmm23
	vmulpd			%zmm25, %zmm23, %zmm23
//	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
//	vmovsd			%xmm17, 16+1*64(%r11)
//	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
//	vmovsd			%xmm18, 16+2*64(%r11)
	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
	vmovsd			%xmm19, 16+3*64(%r11)
	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
	vmovsd			%xmm20, 16+4*64(%r11)
	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
	vmovsd			%xmm21, 16+5*64(%r11)
	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
	vmovsd			%xmm22, 16+6*64(%r11)
	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
	vmovsd			%xmm23, 16+7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 16, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0x01, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			0*64(%r13), %zmm24
	vmulpd			%zmm24, %zmm0, %zmm1 {%k1}{z}
	vmovapd			1*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x03, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	vmovapd			%zmm1, %zmm0 {%k1}
	vbroadcastsd	16+2*64(%r13), %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0
	vmovapd			%zmm0, 2*64(%r13) {%k1}

	movl	$ 0xf8, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm0, %zmm0 {%k1}{z}

	movq	ARG2, %r11 // D
//	vmovapd			0*64(%r11), %zmm16
//	vmovapd			1*64(%r11), %zmm17
	vmovapd			2*64(%r11), %zmm18
	vmovapd			3*64(%r11), %zmm19
	vmovapd			4*64(%r11), %zmm20
	vmovapd			5*64(%r11), %zmm21
	vmovapd			6*64(%r11), %zmm22
	vmovapd			7*64(%r11), %zmm23
//	vaddpd			%zmm0, %zmm16, %zmm16
//	vbroadcastsd	16+1*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm17
//	vaddpd			%zmm0, %zmm17, %zmm17
//	vbroadcastsd	16+2*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm18
	vaddpd			%zmm0, %zmm18, %zmm18
	vbroadcastsd	16+3*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm19
	vbroadcastsd	16+4*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm20
	vbroadcastsd	16+5*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm21
	vbroadcastsd	16+6*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm22
	vbroadcastsd	16+7*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc
//	vmovapd			%zmm16, 0*64(%r11)
//	vmovapd			%zmm17, 1*64(%r11)
//	vfmadd231pd		%zmm18, %zmm18, %zmm25
	vmovapd			%zmm18, 2*64(%r11)
//	vfmadd231pd		%zmm19, %zmm19, %zmm26
	vmovapd			%zmm19, 3*64(%r11)
	vfmadd231pd		%zmm20, %zmm20, %zmm25
	vmovapd			%zmm20, 4*64(%r11)
	vfmadd231pd		%zmm21, %zmm21, %zmm26
	vmovapd			%zmm21, 5*64(%r11)
	vfmadd231pd		%zmm22, %zmm22, %zmm25
	vmovapd			%zmm22, 6*64(%r11)
	vfmadd231pd		%zmm23, %zmm23, %zmm26
	vmovapd			%zmm23, 7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 16, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// fourth column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 24(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D

	leaq	24+64*3(%r11), %r14 // XXX chech for reg overwrite
	leaq	24(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_PD_LIB8
#else
	CALL(inner_edge_dgelqf_pd_lib8)
#endif

	vxorpd			%xmm31, %xmm24, %xmm27
	vmovsd			%xmm27, 24+64*3(%r13) // pT[1+ps*1]

	vmovapd			3*64(%r11), %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
//	vbroadcastsd	24+1*64(%r11), %zmm17
//	vmulpd			%zmm25, %zmm17, %zmm17
//	vbroadcastsd	24+2*64(%r11), %zmm18
//	vmulpd			%zmm25, %zmm18, %zmm18
//	vbroadcastsd	24+3*64(%r11), %zmm19
//	vmulpd			%zmm25, %zmm19, %zmm19
	vbroadcastsd	24+4*64(%r11), %zmm20
	vmulpd			%zmm25, %zmm20, %zmm20
	vbroadcastsd	24+5*64(%r11), %zmm21
	vmulpd			%zmm25, %zmm21, %zmm21
	vbroadcastsd	24+6*64(%r11), %zmm22
	vmulpd			%zmm25, %zmm22, %zmm22
	vbroadcastsd	24+7*64(%r11), %zmm23
	vmulpd			%zmm25, %zmm23, %zmm23
//	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
//	vmovsd			%xmm17, 24+1*64(%r11)
//	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
//	vmovsd			%xmm18, 24+2*64(%r11)
//	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
//	vmovsd			%xmm19, 24+3*64(%r11)
	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
	vmovsd			%xmm20, 24+4*64(%r11)
	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
	vmovsd			%xmm21, 24+5*64(%r11)
	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
	vmovsd			%xmm22, 24+6*64(%r11)
	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
	vmovsd			%xmm23, 24+7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 24, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0x01, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			0*64(%r13), %zmm24
	vmulpd			%zmm24, %zmm0, %zmm1 {%k1}{z}
	vmovapd			1*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x03, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	vmovapd			2*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x07, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	vmovapd			%zmm1, %zmm0 {%k1}
	vbroadcastsd	24+3*64(%r13), %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0
	vmovapd			%zmm0, 3*64(%r13) {%k1}

	movl	$ 0xf0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm0, %zmm0 {%k1}{z}

	movq	ARG2, %r11 // D
//	vmovapd			0*64(%r11), %zmm16
//	vmovapd			1*64(%r11), %zmm17
//	vmovapd			2*64(%r11), %zmm18
	vmovapd			3*64(%r11), %zmm19
	vmovapd			4*64(%r11), %zmm20
	vmovapd			5*64(%r11), %zmm21
	vmovapd			6*64(%r11), %zmm22
	vmovapd			7*64(%r11), %zmm23
//	vaddpd			%zmm0, %zmm16, %zmm16
//	vbroadcastsd	24+1*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm17
//	vaddpd			%zmm0, %zmm17, %zmm17
//	vbroadcastsd	24+2*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm18
//	vaddpd			%zmm0, %zmm18, %zmm18
//	vbroadcastsd	24+3*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm19
	vaddpd			%zmm0, %zmm19, %zmm19
	vbroadcastsd	24+4*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm20
	vbroadcastsd	24+5*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm21
	vbroadcastsd	24+6*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm22
	vbroadcastsd	24+7*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc
//	vmovapd			%zmm16, 0*64(%r11)
//	vmovapd			%zmm17, 1*64(%r11)
//	vfmadd231pd		%zmm18, %zmm18, %zmm25
//	vmovapd			%zmm18, 2*64(%r11)
//	vfmadd231pd		%zmm19, %zmm19, %zmm26
	vmovapd			%zmm19, 3*64(%r11)
//	vfmadd231pd		%zmm20, %zmm20, %zmm25
	vmovapd			%zmm20, 4*64(%r11)
	vfmadd231pd		%zmm21, %zmm21, %zmm26
	vmovapd			%zmm21, 5*64(%r11)
	vfmadd231pd		%zmm22, %zmm22, %zmm25
	vmovapd			%zmm22, 6*64(%r11)
	vfmadd231pd		%zmm23, %zmm23, %zmm26
	vmovapd			%zmm23, 7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 24, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// fifth column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 32(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D

	leaq	32+64*4(%r11), %r14 // XXX chech for reg overwrite
	leaq	32(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_PD_LIB8
#else
	CALL(inner_edge_dgelqf_pd_lib8)
#endif

	vxorpd			%xmm31, %xmm24, %xmm27
	vmovsd			%xmm27, 32+64*4(%r13) // pT[1+ps*1]

	vmovapd			4*64(%r11), %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
//	vbroadcastsd	32+1*64(%r11), %zmm17
//	vmulpd			%zmm25, %zmm17, %zmm17
//	vbroadcastsd	32+2*64(%r11), %zmm18
//	vmulpd			%zmm25, %zmm18, %zmm18
//	vbroadcastsd	32+3*64(%r11), %zmm19
//	vmulpd			%zmm25, %zmm19, %zmm19
//	vbroadcastsd	32+4*64(%r11), %zmm20
//	vmulpd			%zmm25, %zmm20, %zmm20
	vbroadcastsd	32+5*64(%r11), %zmm21
	vmulpd			%zmm25, %zmm21, %zmm21
	vbroadcastsd	32+6*64(%r11), %zmm22
	vmulpd			%zmm25, %zmm22, %zmm22
	vbroadcastsd	32+7*64(%r11), %zmm23
	vmulpd			%zmm25, %zmm23, %zmm23
//	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
//	vmovsd			%xmm17, 32+1*64(%r11)
//	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
//	vmovsd			%xmm18, 32+2*64(%r11)
//	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
//	vmovsd			%xmm19, 32+3*64(%r11)
//	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
//	vmovsd			%xmm20, 32+4*64(%r11)
	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
	vmovsd			%xmm21, 32+5*64(%r11)
	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
	vmovsd			%xmm22, 32+6*64(%r11)
	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
	vmovsd			%xmm23, 32+7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 32, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0x01, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			0*64(%r13), %zmm24
	vmulpd			%zmm24, %zmm0, %zmm1 {%k1}{z}
	//
	vmovapd			1*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x03, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			2*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x07, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			3*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x0f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			%zmm1, %zmm0 {%k1}
	vbroadcastsd	32+4*64(%r13), %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0
	vmovapd			%zmm0, 4*64(%r13) {%k1}

	movl	$ 0xe0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm0, %zmm0 {%k1}{z}

	movq	ARG2, %r11 // D
//	vmovapd			0*64(%r11), %zmm16
//	vmovapd			1*64(%r11), %zmm17
//	vmovapd			2*64(%r11), %zmm18
//	vmovapd			3*64(%r11), %zmm19
	vmovapd			4*64(%r11), %zmm20
	vmovapd			5*64(%r11), %zmm21
	vmovapd			6*64(%r11), %zmm22
	vmovapd			7*64(%r11), %zmm23
//	vaddpd			%zmm0, %zmm16, %zmm16
//	vbroadcastsd	32+1*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm17
//	vaddpd			%zmm0, %zmm17, %zmm17
//	vbroadcastsd	32+2*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm18
//	vaddpd			%zmm0, %zmm18, %zmm18
//	vbroadcastsd	32+3*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm19
//	vaddpd			%zmm0, %zmm19, %zmm19
//	vbroadcastsd	32+4*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm20
	vaddpd			%zmm0, %zmm20, %zmm20
	vbroadcastsd	32+5*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm21
	vbroadcastsd	32+6*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm22
	vbroadcastsd	32+7*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc
//	vmovapd			%zmm16, 0*64(%r11)
//	vmovapd			%zmm17, 1*64(%r11)
//	vfmadd231pd		%zmm18, %zmm18, %zmm25
//	vmovapd			%zmm18, 2*64(%r11)
//	vfmadd231pd		%zmm19, %zmm19, %zmm26
//	vmovapd			%zmm19, 3*64(%r11)
//	vfmadd231pd		%zmm20, %zmm20, %zmm25
	vmovapd			%zmm20, 4*64(%r11)
//	vfmadd231pd		%zmm21, %zmm21, %zmm26
	vmovapd			%zmm21, 5*64(%r11)
	vfmadd231pd		%zmm22, %zmm22, %zmm25
	vmovapd			%zmm22, 6*64(%r11)
	vfmadd231pd		%zmm23, %zmm23, %zmm26
	vmovapd			%zmm23, 7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 32, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// sixth column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 40(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D

	leaq	40+64*5(%r11), %r14 // XXX chech for reg overwrite
	leaq	40(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_PD_LIB8
#else
	CALL(inner_edge_dgelqf_pd_lib8)
#endif

	vxorpd			%xmm31, %xmm24, %xmm27
	vmovsd			%xmm27, 40+64*5(%r13) // pT[1+ps*1]

	vmovapd			5*64(%r11), %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
//	vbroadcastsd	40+1*64(%r11), %zmm17
//	vmulpd			%zmm25, %zmm17, %zmm17
//	vbroadcastsd	40+2*64(%r11), %zmm18
//	vmulpd			%zmm25, %zmm18, %zmm18
//	vbroadcastsd	40+3*64(%r11), %zmm19
//	vmulpd			%zmm25, %zmm19, %zmm19
//	vbroadcastsd	40+4*64(%r11), %zmm20
//	vmulpd			%zmm25, %zmm20, %zmm20
//	vbroadcastsd	40+5*64(%r11), %zmm21
//	vmulpd			%zmm25, %zmm21, %zmm21
	vbroadcastsd	40+6*64(%r11), %zmm22
	vmulpd			%zmm25, %zmm22, %zmm22
	vbroadcastsd	40+7*64(%r11), %zmm23
	vmulpd			%zmm25, %zmm23, %zmm23
//	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
//	vmovsd			%xmm17, 40+1*64(%r11)
//	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
//	vmovsd			%xmm18, 40+2*64(%r11)
//	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
//	vmovsd			%xmm19, 40+3*64(%r11)
//	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
//	vmovsd			%xmm20, 40+4*64(%r11)
//	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
//	vmovsd			%xmm21, 40+5*64(%r11)
	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
	vmovsd			%xmm22, 40+6*64(%r11)
	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
	vmovsd			%xmm23, 40+7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 40, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0x01, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			0*64(%r13), %zmm24
	vmulpd			%zmm24, %zmm0, %zmm1 {%k1}{z}
	//
	vmovapd			1*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x03, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			2*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x07, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			3*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x0f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			4*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x1f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			%zmm1, %zmm0 {%k1}
	vbroadcastsd	40+5*64(%r13), %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0
	vmovapd			%zmm0, 5*64(%r13) {%k1}

	movl	$ 0xc0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm0, %zmm0 {%k1}{z}

	movq	ARG2, %r11 // D
//	vmovapd			0*64(%r11), %zmm16
//	vmovapd			1*64(%r11), %zmm17
//	vmovapd			2*64(%r11), %zmm18
//	vmovapd			3*64(%r11), %zmm19
//	vmovapd			4*64(%r11), %zmm20
	vmovapd			5*64(%r11), %zmm21
	vmovapd			6*64(%r11), %zmm22
	vmovapd			7*64(%r11), %zmm23
//	vaddpd			%zmm0, %zmm16, %zmm16
//	vbroadcastsd	40+1*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm17
//	vaddpd			%zmm0, %zmm17, %zmm17
//	vbroadcastsd	40+2*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm18
//	vaddpd			%zmm0, %zmm18, %zmm18
//	vbroadcastsd	40+3*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm19
//	vaddpd			%zmm0, %zmm19, %zmm19
//	vbroadcastsd	40+4*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm20
//	vaddpd			%zmm0, %zmm20, %zmm20
//	vbroadcastsd	40+5*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm21
	vaddpd			%zmm0, %zmm21, %zmm21
	vbroadcastsd	40+6*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm22
	vbroadcastsd	40+7*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc
//	vmovapd			%zmm16, 0*64(%r11)
//	vmovapd			%zmm17, 1*64(%r11)
//	vfmadd231pd		%zmm18, %zmm18, %zmm25
//	vmovapd			%zmm18, 2*64(%r11)
//	vfmadd231pd		%zmm19, %zmm19, %zmm26
//	vmovapd			%zmm19, 3*64(%r11)
//	vfmadd231pd		%zmm20, %zmm20, %zmm25
//	vmovapd			%zmm20, 4*64(%r11)
//	vfmadd231pd		%zmm21, %zmm21, %zmm26
	vmovapd			%zmm21, 5*64(%r11)
//	vfmadd231pd		%zmm22, %zmm22, %zmm25
	vmovapd			%zmm22, 6*64(%r11)
	vfmadd231pd		%zmm23, %zmm23, %zmm26
	vmovapd			%zmm23, 7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 40, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// seventh column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 48(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D

	leaq	48+64*6(%r11), %r14 // XXX chech for reg overwrite
	leaq	48(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_PD_LIB8
#else
	CALL(inner_edge_dgelqf_pd_lib8)
#endif

	vxorpd			%xmm31, %xmm24, %xmm27
	vmovsd			%xmm27, 48+64*6(%r13) // pT[1+ps*1]

	vmovapd			6*64(%r11), %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
//	vbroadcastsd	48+1*64(%r11), %zmm17
//	vmulpd			%zmm25, %zmm17, %zmm17
//	vbroadcastsd	48+2*64(%r11), %zmm18
//	vmulpd			%zmm25, %zmm18, %zmm18
//	vbroadcastsd	48+3*64(%r11), %zmm19
//	vmulpd			%zmm25, %zmm19, %zmm19
//	vbroadcastsd	48+4*64(%r11), %zmm20
//	vmulpd			%zmm25, %zmm20, %zmm20
//	vbroadcastsd	48+5*64(%r11), %zmm21
//	vmulpd			%zmm25, %zmm21, %zmm21
//	vbroadcastsd	48+6*64(%r11), %zmm22
//	vmulpd			%zmm25, %zmm22, %zmm22
	vbroadcastsd	48+7*64(%r11), %zmm23
	vmulpd			%zmm25, %zmm23, %zmm23
//	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
//	vmovsd			%xmm17, 48+1*64(%r11)
//	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
//	vmovsd			%xmm18, 48+2*64(%r11)
//	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
//	vmovsd			%xmm19, 48+3*64(%r11)
//	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
//	vmovsd			%xmm20, 48+4*64(%r11)
//	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
//	vmovsd			%xmm21, 48+5*64(%r11)
//	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
//	vmovsd			%xmm22, 48+6*64(%r11)
	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
	vmovsd			%xmm23, 48+7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 48, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0x01, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			0*64(%r13), %zmm24
	vmulpd			%zmm24, %zmm0, %zmm1 {%k1}{z}
	//
	vmovapd			1*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x03, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			2*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x07, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			3*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x0f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			4*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x1f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			5*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x3f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			%zmm1, %zmm0 {%k1}
	vbroadcastsd	48+6*64(%r13), %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0
	vmovapd			%zmm0, 6*64(%r13) {%k1}

	movl	$ 0x80, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm0, %zmm0 {%k1}{z}

	movq	ARG2, %r11 // D
//	vmovapd			0*64(%r11), %zmm16
//	vmovapd			1*64(%r11), %zmm17
//	vmovapd			2*64(%r11), %zmm18
//	vmovapd			3*64(%r11), %zmm19
//	vmovapd			4*64(%r11), %zmm20
//	vmovapd			5*64(%r11), %zmm21
	vmovapd			6*64(%r11), %zmm22
	vmovapd			7*64(%r11), %zmm23
//	vaddpd			%zmm0, %zmm16, %zmm16
//	vbroadcastsd	48+1*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm17
//	vaddpd			%zmm0, %zmm17, %zmm17
//	vbroadcastsd	48+2*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm18
//	vaddpd			%zmm0, %zmm18, %zmm18
//	vbroadcastsd	48+3*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm19
//	vaddpd			%zmm0, %zmm19, %zmm19
//	vbroadcastsd	48+4*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm20
//	vaddpd			%zmm0, %zmm20, %zmm20
//	vbroadcastsd	48+5*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm21
//	vaddpd			%zmm0, %zmm21, %zmm21
//	vbroadcastsd	48+6*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm22
	vaddpd			%zmm0, %zmm22, %zmm22
	vbroadcastsd	48+7*64(%r11), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc
//	vmovapd			%zmm16, 0*64(%r11)
//	vmovapd			%zmm17, 1*64(%r11)
//	vfmadd231pd		%zmm18, %zmm18, %zmm25
//	vmovapd			%zmm18, 2*64(%r11)
//	vfmadd231pd		%zmm19, %zmm19, %zmm26
//	vmovapd			%zmm19, 3*64(%r11)
//	vfmadd231pd		%zmm20, %zmm20, %zmm25
//	vmovapd			%zmm20, 4*64(%r11)
//	vfmadd231pd		%zmm21, %zmm21, %zmm26
//	vmovapd			%zmm21, 5*64(%r11)
//	vfmadd231pd		%zmm22, %zmm22, %zmm25
	vmovapd			%zmm22, 6*64(%r11)
//	vfmadd231pd		%zmm23, %zmm23, %zmm26
	vmovapd			%zmm23, 7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 48, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// eigth column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 56(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D

	leaq	56+64*7(%r11), %r14 // XXX chech for reg overwrite
	leaq	56(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_PD_LIB8
#else
	CALL(inner_edge_dgelqf_pd_lib8)
#endif

	vxorpd			%xmm31, %xmm24, %xmm27
	vmovsd			%xmm27, 56+64*7(%r13) // pT[1+ps*1]

	vmovapd			7*64(%r11), %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
//	vbroadcastsd	56+1*64(%r11), %zmm17
//	vmulpd			%zmm25, %zmm17, %zmm17
//	vbroadcastsd	56+2*64(%r11), %zmm18
//	vmulpd			%zmm25, %zmm18, %zmm18
//	vbroadcastsd	56+3*64(%r11), %zmm19
//	vmulpd			%zmm25, %zmm19, %zmm19
//	vbroadcastsd	56+4*64(%r11), %zmm20
//	vmulpd			%zmm25, %zmm20, %zmm20
//	vbroadcastsd	56+5*64(%r11), %zmm21
//	vmulpd			%zmm25, %zmm21, %zmm21
//	vbroadcastsd	56+6*64(%r11), %zmm22
//	vmulpd			%zmm25, %zmm22, %zmm22
//	vbroadcastsd	56+7*64(%r11), %zmm23
//	vmulpd			%zmm25, %zmm23, %zmm23
//	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
//	vmovsd			%xmm17, 56+1*64(%r11)
//	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
//	vmovsd			%xmm18, 56+2*64(%r11)
//	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
//	vmovsd			%xmm19, 56+3*64(%r11)
//	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
//	vmovsd			%xmm20, 56+4*64(%r11)
//	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
//	vmovsd			%xmm21, 56+5*64(%r11)
//	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
//	vmovsd			%xmm22, 56+6*64(%r11)
//	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
//	vmovsd			%xmm23, 56+7*64(%r11)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 512, %r11

	movq	$ 56, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0x01, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			0*64(%r13), %zmm24
	vmulpd			%zmm24, %zmm0, %zmm1 {%k1}{z}
	//
	vmovapd			1*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x03, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			2*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x07, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			3*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x0f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			4*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x1f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			5*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x3f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			6*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x7f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			%zmm1, %zmm0 {%k1}
	vbroadcastsd	56+7*64(%r13), %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0
	vmovapd			%zmm0, 7*64(%r13) {%k1}

102:

1000:
	EPILOGUE
	
	ret

	FUN_END(kernel_dgelqf_pd_dlarft8_8_lib8)





//                                         1      2           3           4           5
// void kernel_dgelqf_pd_la_dlarft8_8_lib8(int n, double *pD, double *dD, double *pA, double *pT)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgelqf_pd_la_dlarft8_8_lib8)
	
	PROLOGUE

	// zero T

	movq	ARG5, %r10 // T

	vxorpd			%zmm24, %zmm24, %zmm24
	vmovapd			%zmm24, 0*64(%r10)
	vmovapd			%zmm24, 1*64(%r10)
	vmovapd			%zmm24, 2*64(%r10)
	vmovapd			%zmm24, 3*64(%r10)
	vmovapd			%zmm24, 4*64(%r10)
	vmovapd			%zmm24, 5*64(%r10)
	vmovapd			%zmm24, 6*64(%r10)
	vmovapd			%zmm24, 7*64(%r10)

	// first column

	movq	ARG1, %r10 // n
	movq	ARG4, %r11 // A

	vxorpd			%xmm25, %xmm25, %xmm25
//	subl	$ 1, %r10d
//	addq	$ 64, %r11
	cmpl	$ 0, %r10d
	jle		101f
100:
	vmovsd			0(%r11), %xmm24
	vfmadd231sd		%xmm24, %xmm24, %xmm25
	subl	$ 1, %r10d
	addq	$ 64, %r11
	cmpl	$ 0, %r10d
	jg		100b
101:

	movq	ARG3, %r12 // dD
	movq	ARG5, %r13 // T

	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		101f
	vmovsd			%xmm24, 0(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D

	leaq	0+64*0(%r11), %r14 // XXX chech for reg overwrite
	leaq	0(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_PD_LIB8
#else
	CALL(inner_edge_dgelqf_pd_lib8)
#endif

	vxorpd			%xmm31, %xmm24, %xmm27
	vmovsd			%xmm27, 0+64*0(%r13) // pT[0+ps*0]

	vmovapd			0*64(%r11), %zmm0
//	vxorpd			%zmm0, %zmm0, %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
//	vbroadcastsd	0+1*64(%r11), %zmm17
//	vmulpd			%zmm25, %zmm17, %zmm17
//	vbroadcastsd	0+2*64(%r11), %zmm18
//	vmulpd			%zmm25, %zmm18, %zmm18
//	vbroadcastsd	0+3*64(%r11), %zmm19
//	vmulpd			%zmm25, %zmm19, %zmm19
//	vbroadcastsd	0+4*64(%r11), %zmm20
//	vmulpd			%zmm25, %zmm20, %zmm20
//	vbroadcastsd	0+5*64(%r11), %zmm21
//	vmulpd			%zmm25, %zmm21, %zmm21
//	vbroadcastsd	0+6*64(%r11), %zmm22
//	vmulpd			%zmm25, %zmm22, %zmm22
//	vbroadcastsd	0+7*64(%r11), %zmm23
//	vmulpd			%zmm25, %zmm23, %zmm23
//	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
//	vmovsd			%xmm17, 0+1*64(%r11)
//	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
//	vmovsd			%xmm18, 0+2*64(%r11)
//	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
//	vmovsd			%xmm19, 0+3*64(%r11)
//	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
//	vmovsd			%xmm20, 0+4*64(%r11)
//	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
//	vmovsd			%xmm21, 0+5*64(%r11)
//	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
//	vmovsd			%xmm22, 0+6*64(%r11)
//	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
//	vmovsd			%xmm23, 0+7*64(%r11)

	movq	ARG1, %r10 // n
	movq	ARG4, %r11

	movq	$ 0, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0xfe, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vbroadcastsd	0(%r13), %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0 {%k1}{z}

	movq	ARG2, %r11 // D
	vmovapd			0*64(%r11), %zmm16
//	vmovapd			1*64(%r11), %zmm17
//	vmovapd			2*64(%r11), %zmm18
//	vmovapd			3*64(%r11), %zmm19
//	vmovapd			4*64(%r11), %zmm20
//	vmovapd			5*64(%r11), %zmm21
//	vmovapd			6*64(%r11), %zmm22
//	vmovapd			7*64(%r11), %zmm23
	vaddpd			%zmm0, %zmm16, %zmm16
//	vbroadcastsd	0+1*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm17
//	vbroadcastsd	0+2*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm18
//	vbroadcastsd	0+3*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm19
//	vbroadcastsd	0+4*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm20
//	vbroadcastsd	0+5*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm21
//	vbroadcastsd	0+6*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm22
//	vbroadcastsd	0+7*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc
	vmovapd			%zmm16, 0*64(%r11)
//	vmovapd			%zmm17, 1*64(%r11)
//	vfmadd231pd		%zmm18, %zmm18, %zmm25
//	vmovapd			%zmm18, 2*64(%r11)
//	vfmadd231pd		%zmm19, %zmm19, %zmm26
//	vmovapd			%zmm19, 3*64(%r11)
//	vfmadd231pd		%zmm20, %zmm20, %zmm25
//	vmovapd			%zmm20, 4*64(%r11)
//	vfmadd231pd		%zmm21, %zmm21, %zmm26
//	vmovapd			%zmm21, 5*64(%r11)
//	vfmadd231pd		%zmm22, %zmm22, %zmm25
//	vmovapd			%zmm22, 6*64(%r11)
//	vfmadd231pd		%zmm23, %zmm23, %zmm26
//	vmovapd			%zmm23, 7*64(%r11)

	movq	ARG1, %r10 // n
	movq	ARG4, %r11 // A

	movq	$ 0, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// second column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 8(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D

	leaq	8+64*1(%r11), %r14 // XXX chech for reg overwrite
	leaq	8(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_PD_LIB8
#else
	CALL(inner_edge_dgelqf_pd_lib8)
#endif

	vxorpd			%xmm31, %xmm24, %xmm27
	vmovsd			%xmm27, 8+64*1(%r13) // pT[1+ps*1]

	movl	$ 0xfe, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			1*64(%r11), %zmm0 {%k1}{z}
//	vxorpd			%zmm0, %zmm0, %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
////	vbroadcastsd	8+1*64(%r11), %zmm17
////	vmulpd			%zmm25, %zmm17, %zmm17
//	vbroadcastsd	8+2*64(%r11), %zmm18
//	vmulpd			%zmm25, %zmm18, %zmm18
//	vbroadcastsd	8+3*64(%r11), %zmm19
//	vmulpd			%zmm25, %zmm19, %zmm19
//	vbroadcastsd	8+4*64(%r11), %zmm20
//	vmulpd			%zmm25, %zmm20, %zmm20
//	vbroadcastsd	8+5*64(%r11), %zmm21
//	vmulpd			%zmm25, %zmm21, %zmm21
//	vbroadcastsd	8+6*64(%r11), %zmm22
//	vmulpd			%zmm25, %zmm22, %zmm22
//	vbroadcastsd	8+7*64(%r11), %zmm23
//	vmulpd			%zmm25, %zmm23, %zmm23
////	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
////	vmovsd			%xmm17, 8+1*64(%r11)
//	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
//	vmovsd			%xmm18, 8+2*64(%r11)
//	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
//	vmovsd			%xmm19, 8+3*64(%r11)
//	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
//	vmovsd			%xmm20, 8+4*64(%r11)
//	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
//	vmovsd			%xmm21, 8+5*64(%r11)
//	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
//	vmovsd			%xmm22, 8+6*64(%r11)
//	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
//	vmovsd			%xmm23, 8+7*64(%r11)

	movq	ARG1, %r10 // n
	movq	ARG4, %r11

	movq	$ 8, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC00+16(%rip), %zmm27 // 1.0
#else
	vbroadcastsd	LC00+16(%rip), %zmm27 // 1.0
#endif
	movl	$ 0x01, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			0+64*0(%r13), %zmm27 {%k1}
	vmulpd			%zmm27, %zmm0, %zmm0
	vbroadcastsd	8+64*1(%r13), %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0
	vmovapd			%zmm0, 1*64(%r13) {%k1}

	movl	$ 0xfc, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm0, %zmm0 {%k1}{z}

	movq	ARG2, %r11 // D
////	vmovapd			0*64(%r11), %zmm16
	vmovapd			1*64(%r11), %zmm17
//	vmovapd			2*64(%r11), %zmm18
//	vmovapd			3*64(%r11), %zmm19
//	vmovapd			4*64(%r11), %zmm20
//	vmovapd			5*64(%r11), %zmm21
//	vmovapd			6*64(%r11), %zmm22
//	vmovapd			7*64(%r11), %zmm23
////	vaddpd			%zmm0, %zmm16, %zmm16
////	vbroadcastsd	8+1*64(%r11), %zmm24
////	vfmadd231pd		%zmm0, %zmm24, %zmm17
	vaddpd			%zmm0, %zmm17, %zmm17
//	vbroadcastsd	8+2*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm18
//	vbroadcastsd	8+3*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm19
//	vbroadcastsd	8+4*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm20
//	vbroadcastsd	8+5*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm21
//	vbroadcastsd	8+6*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm22
//	vbroadcastsd	8+7*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc
////	vmovapd			%zmm16, 0*64(%r11)
	vmovapd			%zmm17, 1*64(%r11)
////	vfmadd231pd		%zmm18, %zmm18, %zmm25
//	vmovapd			%zmm18, 2*64(%r11)
//	vfmadd231pd		%zmm19, %zmm19, %zmm26
//	vmovapd			%zmm19, 3*64(%r11)
//	vfmadd231pd		%zmm20, %zmm20, %zmm25
//	vmovapd			%zmm20, 4*64(%r11)
//	vfmadd231pd		%zmm21, %zmm21, %zmm26
//	vmovapd			%zmm21, 5*64(%r11)
//	vfmadd231pd		%zmm22, %zmm22, %zmm25
//	vmovapd			%zmm22, 6*64(%r11)
//	vfmadd231pd		%zmm23, %zmm23, %zmm26
//	vmovapd			%zmm23, 7*64(%r11)

	movq	ARG1, %r10 // n
	movq	ARG4, %r11

	movq	$ 8, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// third column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 16(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D

	leaq	16+64*2(%r11), %r14 // XXX chech for reg overwrite
	leaq	16(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_PD_LIB8
#else
	CALL(inner_edge_dgelqf_pd_lib8)
#endif

	vxorpd			%xmm31, %xmm24, %xmm27
	vmovsd			%xmm27, 16+64*2(%r13) // pT[1+ps*1]

	movl	$ 0xfc, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			2*64(%r11), %zmm0 {%k1}{z}
//	vxorpd			%zmm0, %zmm0, %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
////	vbroadcastsd	16+1*64(%r11), %zmm17
////	vmulpd			%zmm25, %zmm17, %zmm17
////	vbroadcastsd	16+2*64(%r11), %zmm18
////	vmulpd			%zmm25, %zmm18, %zmm18
//	vbroadcastsd	16+3*64(%r11), %zmm19
//	vmulpd			%zmm25, %zmm19, %zmm19
//	vbroadcastsd	16+4*64(%r11), %zmm20
//	vmulpd			%zmm25, %zmm20, %zmm20
//	vbroadcastsd	16+5*64(%r11), %zmm21
//	vmulpd			%zmm25, %zmm21, %zmm21
//	vbroadcastsd	16+6*64(%r11), %zmm22
//	vmulpd			%zmm25, %zmm22, %zmm22
//	vbroadcastsd	16+7*64(%r11), %zmm23
//	vmulpd			%zmm25, %zmm23, %zmm23
////	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
////	vmovsd			%xmm17, 16+1*64(%r11)
////	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
////	vmovsd			%xmm18, 16+2*64(%r11)
//	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
//	vmovsd			%xmm19, 16+3*64(%r11)
//	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
//	vmovsd			%xmm20, 16+4*64(%r11)
//	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
//	vmovsd			%xmm21, 16+5*64(%r11)
//	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
//	vmovsd			%xmm22, 16+6*64(%r11)
//	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
//	vmovsd			%xmm23, 16+7*64(%r11)

	movq	ARG1, %r10 // n
	movq	ARG4, %r11

	movq	$ 16, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0x01, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			0*64(%r13), %zmm24
	vmulpd			%zmm24, %zmm0, %zmm1 {%k1}{z}
	vmovapd			1*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x03, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	vmovapd			%zmm1, %zmm0 {%k1}
	vbroadcastsd	16+2*64(%r13), %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0
	vmovapd			%zmm0, 2*64(%r13) {%k1}

	movl	$ 0xf8, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm0, %zmm0 {%k1}{z}

	movq	ARG2, %r11 // D
////	vmovapd			0*64(%r11), %zmm16
////	vmovapd			1*64(%r11), %zmm17
	vmovapd			2*64(%r11), %zmm18
//	vmovapd			3*64(%r11), %zmm19
//	vmovapd			4*64(%r11), %zmm20
//	vmovapd			5*64(%r11), %zmm21
//	vmovapd			6*64(%r11), %zmm22
//	vmovapd			7*64(%r11), %zmm23
////	vaddpd			%zmm0, %zmm16, %zmm16
////	vbroadcastsd	16+1*64(%r11), %zmm24
////	vfmadd231pd		%zmm0, %zmm24, %zmm17
////	vaddpd			%zmm0, %zmm17, %zmm17
////	vbroadcastsd	16+2*64(%r11), %zmm24
////	vfmadd231pd		%zmm0, %zmm24, %zmm18
	vaddpd			%zmm0, %zmm18, %zmm18
//	vbroadcastsd	16+3*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm19
//	vbroadcastsd	16+4*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm20
//	vbroadcastsd	16+5*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm21
//	vbroadcastsd	16+6*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm22
//	vbroadcastsd	16+7*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc
////	vmovapd			%zmm16, 0*64(%r11)
////	vmovapd			%zmm17, 1*64(%r11)
////	vfmadd231pd		%zmm18, %zmm18, %zmm25
	vmovapd			%zmm18, 2*64(%r11)
////	vfmadd231pd		%zmm19, %zmm19, %zmm26
//	vmovapd			%zmm19, 3*64(%r11)
//	vfmadd231pd		%zmm20, %zmm20, %zmm25
//	vmovapd			%zmm20, 4*64(%r11)
//	vfmadd231pd		%zmm21, %zmm21, %zmm26
//	vmovapd			%zmm21, 5*64(%r11)
//	vfmadd231pd		%zmm22, %zmm22, %zmm25
//	vmovapd			%zmm22, 6*64(%r11)
//	vfmadd231pd		%zmm23, %zmm23, %zmm26
//	vmovapd			%zmm23, 7*64(%r11)

	movq	ARG1, %r10 // n
	movq	ARG4, %r11

	movq	$ 16, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// fourth column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 24(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D

	leaq	24+64*3(%r11), %r14 // XXX chech for reg overwrite
	leaq	24(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_PD_LIB8
#else
	CALL(inner_edge_dgelqf_pd_lib8)
#endif

	vxorpd			%xmm31, %xmm24, %xmm27
	vmovsd			%xmm27, 24+64*3(%r13) // pT[1+ps*1]

	movl	$ 0xf8, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			3*64(%r11), %zmm0 {%k1}{z}
//	vxorpd			%zmm0, %zmm0, %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
////	vbroadcastsd	24+1*64(%r11), %zmm17
////	vmulpd			%zmm25, %zmm17, %zmm17
////	vbroadcastsd	24+2*64(%r11), %zmm18
////	vmulpd			%zmm25, %zmm18, %zmm18
////	vbroadcastsd	24+3*64(%r11), %zmm19
////	vmulpd			%zmm25, %zmm19, %zmm19
//	vbroadcastsd	24+4*64(%r11), %zmm20
//	vmulpd			%zmm25, %zmm20, %zmm20
//	vbroadcastsd	24+5*64(%r11), %zmm21
//	vmulpd			%zmm25, %zmm21, %zmm21
//	vbroadcastsd	24+6*64(%r11), %zmm22
//	vmulpd			%zmm25, %zmm22, %zmm22
//	vbroadcastsd	24+7*64(%r11), %zmm23
//	vmulpd			%zmm25, %zmm23, %zmm23
////	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
////	vmovsd			%xmm17, 24+1*64(%r11)
////	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
////	vmovsd			%xmm18, 24+2*64(%r11)
////	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
////	vmovsd			%xmm19, 24+3*64(%r11)
//	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
//	vmovsd			%xmm20, 24+4*64(%r11)
//	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
//	vmovsd			%xmm21, 24+5*64(%r11)
//	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
//	vmovsd			%xmm22, 24+6*64(%r11)
//	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
//	vmovsd			%xmm23, 24+7*64(%r11)

	movq	ARG1, %r10 // n
	movq	ARG4, %r11

	movq	$ 24, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0x01, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			0*64(%r13), %zmm24
	vmulpd			%zmm24, %zmm0, %zmm1 {%k1}{z}
	vmovapd			1*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x03, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	vmovapd			2*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x07, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	vmovapd			%zmm1, %zmm0 {%k1}
	vbroadcastsd	24+3*64(%r13), %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0
	vmovapd			%zmm0, 3*64(%r13) {%k1}

	movl	$ 0xf0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm0, %zmm0 {%k1}{z}

	movq	ARG2, %r11 // D
////	vmovapd			0*64(%r11), %zmm16
////	vmovapd			1*64(%r11), %zmm17
////	vmovapd			2*64(%r11), %zmm18
	vmovapd			3*64(%r11), %zmm19
//	vmovapd			4*64(%r11), %zmm20
//	vmovapd			5*64(%r11), %zmm21
//	vmovapd			6*64(%r11), %zmm22
//	vmovapd			7*64(%r11), %zmm23
////	vaddpd			%zmm0, %zmm16, %zmm16
////	vbroadcastsd	24+1*64(%r11), %zmm24
////	vfmadd231pd		%zmm0, %zmm24, %zmm17
////	vaddpd			%zmm0, %zmm17, %zmm17
////	vbroadcastsd	24+2*64(%r11), %zmm24
////	vfmadd231pd		%zmm0, %zmm24, %zmm18
////	vaddpd			%zmm0, %zmm18, %zmm18
////	vbroadcastsd	24+3*64(%r11), %zmm24
////	vfmadd231pd		%zmm0, %zmm24, %zmm19
	vaddpd			%zmm0, %zmm19, %zmm19
//	vbroadcastsd	24+4*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm20
//	vbroadcastsd	24+5*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm21
//	vbroadcastsd	24+6*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm22
//	vbroadcastsd	24+7*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc
////	vmovapd			%zmm16, 0*64(%r11)
////	vmovapd			%zmm17, 1*64(%r11)
////	vfmadd231pd		%zmm18, %zmm18, %zmm25
////	vmovapd			%zmm18, 2*64(%r11)
////	vfmadd231pd		%zmm19, %zmm19, %zmm26
	vmovapd			%zmm19, 3*64(%r11)
////	vfmadd231pd		%zmm20, %zmm20, %zmm25
//	vmovapd			%zmm20, 4*64(%r11)
//	vfmadd231pd		%zmm21, %zmm21, %zmm26
//	vmovapd			%zmm21, 5*64(%r11)
//	vfmadd231pd		%zmm22, %zmm22, %zmm25
//	vmovapd			%zmm22, 6*64(%r11)
//	vfmadd231pd		%zmm23, %zmm23, %zmm26
//	vmovapd			%zmm23, 7*64(%r11)

	movq	ARG1, %r10 // n
	movq	ARG4, %r11

	movq	$ 24, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// fifth column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 32(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D

	leaq	32+64*4(%r11), %r14 // XXX chech for reg overwrite
	leaq	32(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_PD_LIB8
#else
	CALL(inner_edge_dgelqf_pd_lib8)
#endif

	vxorpd			%xmm31, %xmm24, %xmm27
	vmovsd			%xmm27, 32+64*4(%r13) // pT[1+ps*1]

	movl	$ 0xf0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			4*64(%r11), %zmm0 {%k1}{z}
//	vxorpd			%zmm0, %zmm0, %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
////	vbroadcastsd	32+1*64(%r11), %zmm17
////	vmulpd			%zmm25, %zmm17, %zmm17
////	vbroadcastsd	32+2*64(%r11), %zmm18
////	vmulpd			%zmm25, %zmm18, %zmm18
////	vbroadcastsd	32+3*64(%r11), %zmm19
////	vmulpd			%zmm25, %zmm19, %zmm19
////	vbroadcastsd	32+4*64(%r11), %zmm20
////	vmulpd			%zmm25, %zmm20, %zmm20
//	vbroadcastsd	32+5*64(%r11), %zmm21
//	vmulpd			%zmm25, %zmm21, %zmm21
//	vbroadcastsd	32+6*64(%r11), %zmm22
//	vmulpd			%zmm25, %zmm22, %zmm22
//	vbroadcastsd	32+7*64(%r11), %zmm23
//	vmulpd			%zmm25, %zmm23, %zmm23
////	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
////	vmovsd			%xmm17, 32+1*64(%r11)
////	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
////	vmovsd			%xmm18, 32+2*64(%r11)
////	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
////	vmovsd			%xmm19, 32+3*64(%r11)
////	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
////	vmovsd			%xmm20, 32+4*64(%r11)
//	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
//	vmovsd			%xmm21, 32+5*64(%r11)
//	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
//	vmovsd			%xmm22, 32+6*64(%r11)
//	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
//	vmovsd			%xmm23, 32+7*64(%r11)

	movq	ARG1, %r10 // n
	movq	ARG4, %r11

	movq	$ 32, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0x01, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			0*64(%r13), %zmm24
	vmulpd			%zmm24, %zmm0, %zmm1 {%k1}{z}
	//
	vmovapd			1*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x03, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			2*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x07, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			3*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x0f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			%zmm1, %zmm0 {%k1}
	vbroadcastsd	32+4*64(%r13), %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0
	vmovapd			%zmm0, 4*64(%r13) {%k1}

	movl	$ 0xe0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm0, %zmm0 {%k1}{z}

	movq	ARG2, %r11 // D
////	vmovapd			0*64(%r11), %zmm16
////	vmovapd			1*64(%r11), %zmm17
////	vmovapd			2*64(%r11), %zmm18
////	vmovapd			3*64(%r11), %zmm19
	vmovapd			4*64(%r11), %zmm20
//	vmovapd			5*64(%r11), %zmm21
//	vmovapd			6*64(%r11), %zmm22
//	vmovapd			7*64(%r11), %zmm23
////	vaddpd			%zmm0, %zmm16, %zmm16
////	vbroadcastsd	32+1*64(%r11), %zmm24
////	vfmadd231pd		%zmm0, %zmm24, %zmm17
////	vaddpd			%zmm0, %zmm17, %zmm17
////	vbroadcastsd	32+2*64(%r11), %zmm24
////	vfmadd231pd		%zmm0, %zmm24, %zmm18
////	vaddpd			%zmm0, %zmm18, %zmm18
////	vbroadcastsd	32+3*64(%r11), %zmm24
////	vfmadd231pd		%zmm0, %zmm24, %zmm19
////	vaddpd			%zmm0, %zmm19, %zmm19
////	vbroadcastsd	32+4*64(%r11), %zmm24
////	vfmadd231pd		%zmm0, %zmm24, %zmm20
	vaddpd			%zmm0, %zmm20, %zmm20
//	vbroadcastsd	32+5*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm21
//	vbroadcastsd	32+6*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm22
//	vbroadcastsd	32+7*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc
////	vmovapd			%zmm16, 0*64(%r11)
////	vmovapd			%zmm17, 1*64(%r11)
////	vfmadd231pd		%zmm18, %zmm18, %zmm25
////	vmovapd			%zmm18, 2*64(%r11)
////	vfmadd231pd		%zmm19, %zmm19, %zmm26
////	vmovapd			%zmm19, 3*64(%r11)
////	vfmadd231pd		%zmm20, %zmm20, %zmm25
	vmovapd			%zmm20, 4*64(%r11)
////	vfmadd231pd		%zmm21, %zmm21, %zmm26
//	vmovapd			%zmm21, 5*64(%r11)
//	vfmadd231pd		%zmm22, %zmm22, %zmm25
//	vmovapd			%zmm22, 6*64(%r11)
//	vfmadd231pd		%zmm23, %zmm23, %zmm26
//	vmovapd			%zmm23, 7*64(%r11)

	movq	ARG1, %r10 // n
	movq	ARG4, %r11

	movq	$ 32, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// sixth column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 40(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D

	leaq	40+64*5(%r11), %r14 // XXX chech for reg overwrite
	leaq	40(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_PD_LIB8
#else
	CALL(inner_edge_dgelqf_pd_lib8)
#endif

	vxorpd			%xmm31, %xmm24, %xmm27
	vmovsd			%xmm27, 40+64*5(%r13) // pT[1+ps*1]

	movl	$ 0xe0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			5*64(%r11), %zmm0 {%k1}{z}
//	vxorpd			%zmm0, %zmm0, %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
////	vbroadcastsd	40+1*64(%r11), %zmm17
////	vmulpd			%zmm25, %zmm17, %zmm17
////	vbroadcastsd	40+2*64(%r11), %zmm18
////	vmulpd			%zmm25, %zmm18, %zmm18
////	vbroadcastsd	40+3*64(%r11), %zmm19
////	vmulpd			%zmm25, %zmm19, %zmm19
////	vbroadcastsd	40+4*64(%r11), %zmm20
////	vmulpd			%zmm25, %zmm20, %zmm20
////	vbroadcastsd	40+5*64(%r11), %zmm21
////	vmulpd			%zmm25, %zmm21, %zmm21
//	vbroadcastsd	40+6*64(%r11), %zmm22
//	vmulpd			%zmm25, %zmm22, %zmm22
//	vbroadcastsd	40+7*64(%r11), %zmm23
//	vmulpd			%zmm25, %zmm23, %zmm23
////	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
////	vmovsd			%xmm17, 40+1*64(%r11)
////	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
////	vmovsd			%xmm18, 40+2*64(%r11)
////	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
////	vmovsd			%xmm19, 40+3*64(%r11)
////	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
////	vmovsd			%xmm20, 40+4*64(%r11)
////	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
////	vmovsd			%xmm21, 40+5*64(%r11)
//	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
//	vmovsd			%xmm22, 40+6*64(%r11)
//	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
//	vmovsd			%xmm23, 40+7*64(%r11)

	movq	ARG1, %r10 // n
	movq	ARG4, %r11

	movq	$ 40, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0x01, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			0*64(%r13), %zmm24
	vmulpd			%zmm24, %zmm0, %zmm1 {%k1}{z}
	//
	vmovapd			1*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x03, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			2*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x07, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			3*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x0f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			4*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x1f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			%zmm1, %zmm0 {%k1}
	vbroadcastsd	40+5*64(%r13), %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0
	vmovapd			%zmm0, 5*64(%r13) {%k1}

	movl	$ 0xc0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm0, %zmm0 {%k1}{z}

	movq	ARG2, %r11 // D
////	vmovapd			0*64(%r11), %zmm16
////	vmovapd			1*64(%r11), %zmm17
////	vmovapd			2*64(%r11), %zmm18
////	vmovapd			3*64(%r11), %zmm19
////	vmovapd			4*64(%r11), %zmm20
	vmovapd			5*64(%r11), %zmm21
//	vmovapd			6*64(%r11), %zmm22
//	vmovapd			7*64(%r11), %zmm23
////	vaddpd			%zmm0, %zmm16, %zmm16
////	vbroadcastsd	40+1*64(%r11), %zmm24
////	vfmadd231pd		%zmm0, %zmm24, %zmm17
////	vaddpd			%zmm0, %zmm17, %zmm17
////	vbroadcastsd	40+2*64(%r11), %zmm24
////	vfmadd231pd		%zmm0, %zmm24, %zmm18
////	vaddpd			%zmm0, %zmm18, %zmm18
////	vbroadcastsd	40+3*64(%r11), %zmm24
////	vfmadd231pd		%zmm0, %zmm24, %zmm19
////	vaddpd			%zmm0, %zmm19, %zmm19
////	vbroadcastsd	40+4*64(%r11), %zmm24
////	vfmadd231pd		%zmm0, %zmm24, %zmm20
////	vaddpd			%zmm0, %zmm20, %zmm20
////	vbroadcastsd	40+5*64(%r11), %zmm24
////	vfmadd231pd		%zmm0, %zmm24, %zmm21
	vaddpd			%zmm0, %zmm21, %zmm21
//	vbroadcastsd	40+6*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm22
//	vbroadcastsd	40+7*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc
////	vmovapd			%zmm16, 0*64(%r11)
////	vmovapd			%zmm17, 1*64(%r11)
////	vfmadd231pd		%zmm18, %zmm18, %zmm25
////	vmovapd			%zmm18, 2*64(%r11)
////	vfmadd231pd		%zmm19, %zmm19, %zmm26
////	vmovapd			%zmm19, 3*64(%r11)
////	vfmadd231pd		%zmm20, %zmm20, %zmm25
////	vmovapd			%zmm20, 4*64(%r11)
////	vfmadd231pd		%zmm21, %zmm21, %zmm26
	vmovapd			%zmm21, 5*64(%r11)
////	vfmadd231pd		%zmm22, %zmm22, %zmm25
//	vmovapd			%zmm22, 6*64(%r11)
//	vfmadd231pd		%zmm23, %zmm23, %zmm26
//	vmovapd			%zmm23, 7*64(%r11)

	movq	ARG1, %r10 // n
	movq	ARG4, %r11

	movq	$ 40, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// seventh column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 48(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D

	leaq	48+64*6(%r11), %r14 // XXX chech for reg overwrite
	leaq	48(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_PD_LIB8
#else
	CALL(inner_edge_dgelqf_pd_lib8)
#endif

	vxorpd			%xmm31, %xmm24, %xmm27
	vmovsd			%xmm27, 48+64*6(%r13) // pT[1+ps*1]

	movl	$ 0xc0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			6*64(%r11), %zmm0 {%k1}{z}
//	vxorpd			%zmm0, %zmm0, %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
////	vbroadcastsd	48+1*64(%r11), %zmm17
////	vmulpd			%zmm25, %zmm17, %zmm17
////	vbroadcastsd	48+2*64(%r11), %zmm18
////	vmulpd			%zmm25, %zmm18, %zmm18
////	vbroadcastsd	48+3*64(%r11), %zmm19
////	vmulpd			%zmm25, %zmm19, %zmm19
////	vbroadcastsd	48+4*64(%r11), %zmm20
////	vmulpd			%zmm25, %zmm20, %zmm20
////	vbroadcastsd	48+5*64(%r11), %zmm21
////	vmulpd			%zmm25, %zmm21, %zmm21
////	vbroadcastsd	48+6*64(%r11), %zmm22
////	vmulpd			%zmm25, %zmm22, %zmm22
//	vbroadcastsd	48+7*64(%r11), %zmm23
//	vmulpd			%zmm25, %zmm23, %zmm23
////	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
////	vmovsd			%xmm17, 48+1*64(%r11)
////	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
////	vmovsd			%xmm18, 48+2*64(%r11)
////	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
////	vmovsd			%xmm19, 48+3*64(%r11)
////	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
////	vmovsd			%xmm20, 48+4*64(%r11)
////	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
////	vmovsd			%xmm21, 48+5*64(%r11)
////	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
////	vmovsd			%xmm22, 48+6*64(%r11)
//	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
//	vmovsd			%xmm23, 48+7*64(%r11)

	movq	ARG1, %r10 // n
	movq	ARG4, %r11

	movq	$ 48, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0x01, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			0*64(%r13), %zmm24
	vmulpd			%zmm24, %zmm0, %zmm1 {%k1}{z}
	//
	vmovapd			1*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x03, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			2*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x07, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			3*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x0f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			4*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x1f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			5*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x3f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			%zmm1, %zmm0 {%k1}
	vbroadcastsd	48+6*64(%r13), %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0
	vmovapd			%zmm0, 6*64(%r13) {%k1}

	movl	$ 0x80, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm0, %zmm0 {%k1}{z}

	movq	ARG2, %r11 // D
////	vmovapd			0*64(%r11), %zmm16
////	vmovapd			1*64(%r11), %zmm17
////	vmovapd			2*64(%r11), %zmm18
////	vmovapd			3*64(%r11), %zmm19
////	vmovapd			4*64(%r11), %zmm20
////	vmovapd			5*64(%r11), %zmm21
	vmovapd			6*64(%r11), %zmm22
//	vmovapd			7*64(%r11), %zmm23
////	vaddpd			%zmm0, %zmm16, %zmm16
////	vbroadcastsd	48+1*64(%r11), %zmm24
////	vfmadd231pd		%zmm0, %zmm24, %zmm17
////	vaddpd			%zmm0, %zmm17, %zmm17
////	vbroadcastsd	48+2*64(%r11), %zmm24
////	vfmadd231pd		%zmm0, %zmm24, %zmm18
////	vaddpd			%zmm0, %zmm18, %zmm18
////	vbroadcastsd	48+3*64(%r11), %zmm24
////	vfmadd231pd		%zmm0, %zmm24, %zmm19
////	vaddpd			%zmm0, %zmm19, %zmm19
////	vbroadcastsd	48+4*64(%r11), %zmm24
////	vfmadd231pd		%zmm0, %zmm24, %zmm20
////	vaddpd			%zmm0, %zmm20, %zmm20
////	vbroadcastsd	48+5*64(%r11), %zmm24
////	vfmadd231pd		%zmm0, %zmm24, %zmm21
////	vaddpd			%zmm0, %zmm21, %zmm21
////	vbroadcastsd	48+6*64(%r11), %zmm24
////	vfmadd231pd		%zmm0, %zmm24, %zmm22
	vaddpd			%zmm0, %zmm22, %zmm22
//	vbroadcastsd	48+7*64(%r11), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc
////	vmovapd			%zmm16, 0*64(%r11)
////	vmovapd			%zmm17, 1*64(%r11)
////	vfmadd231pd		%zmm18, %zmm18, %zmm25
////	vmovapd			%zmm18, 2*64(%r11)
////	vfmadd231pd		%zmm19, %zmm19, %zmm26
////	vmovapd			%zmm19, 3*64(%r11)
////	vfmadd231pd		%zmm20, %zmm20, %zmm25
////	vmovapd			%zmm20, 4*64(%r11)
////	vfmadd231pd		%zmm21, %zmm21, %zmm26
////	vmovapd			%zmm21, 5*64(%r11)
////	vfmadd231pd		%zmm22, %zmm22, %zmm25
	vmovapd			%zmm22, 6*64(%r11)
////	vfmadd231pd		%zmm23, %zmm23, %zmm26
//	vmovapd			%zmm23, 7*64(%r11)

	movq	ARG1, %r10 // n
	movq	ARG4, %r11

	movq	$ 48, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// eigth column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 56(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D

	leaq	56+64*7(%r11), %r14 // XXX chech for reg overwrite
	leaq	56(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_PD_LIB8
#else
	CALL(inner_edge_dgelqf_pd_lib8)
#endif

	vxorpd			%xmm31, %xmm24, %xmm27
	vmovsd			%xmm27, 56+64*7(%r13) // pT[1+ps*1]

	movl	$ 0x80, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			7*64(%r11), %zmm0 {%k1}{z}
//	vxorpd			%zmm0, %zmm0, %zmm0
	vxorpd			%zmm1, %zmm1, %zmm1 // part acc
////	vbroadcastsd	56+1*64(%r11), %zmm17
////	vmulpd			%zmm25, %zmm17, %zmm17
////	vbroadcastsd	56+2*64(%r11), %zmm18
////	vmulpd			%zmm25, %zmm18, %zmm18
////	vbroadcastsd	56+3*64(%r11), %zmm19
////	vmulpd			%zmm25, %zmm19, %zmm19
////	vbroadcastsd	56+4*64(%r11), %zmm20
////	vmulpd			%zmm25, %zmm20, %zmm20
////	vbroadcastsd	56+5*64(%r11), %zmm21
////	vmulpd			%zmm25, %zmm21, %zmm21
////	vbroadcastsd	56+6*64(%r11), %zmm22
////	vmulpd			%zmm25, %zmm22, %zmm22
////	vbroadcastsd	56+7*64(%r11), %zmm23
////	vmulpd			%zmm25, %zmm23, %zmm23
////	vfmadd231pd		1*64(%r11), %zmm17, %zmm0
////	vmovsd			%xmm17, 56+1*64(%r11)
////	vfmadd231pd		2*64(%r11), %zmm18, %zmm1
////	vmovsd			%xmm18, 56+2*64(%r11)
////	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
////	vmovsd			%xmm19, 56+3*64(%r11)
////	vfmadd231pd		4*64(%r11), %zmm20, %zmm1
////	vmovsd			%xmm20, 56+4*64(%r11)
////	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
////	vmovsd			%xmm21, 56+5*64(%r11)
////	vfmadd231pd		6*64(%r11), %zmm22, %zmm1
////	vmovsd			%xmm22, 56+6*64(%r11)
////	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
////	vmovsd			%xmm23, 56+7*64(%r11)

	movq	ARG1, %r10 // n
	movq	ARG4, %r11

	movq	$ 56, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0x01, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			0*64(%r13), %zmm24
	vmulpd			%zmm24, %zmm0, %zmm1 {%k1}{z}
	//
	vmovapd			1*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x03, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			2*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x07, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			3*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x0f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			4*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x1f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			5*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x3f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			6*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x7f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			%zmm1, %zmm0 {%k1}
	vbroadcastsd	56+7*64(%r13), %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0
	vmovapd			%zmm0, 7*64(%r13) {%k1}

102:

1000:
	EPILOGUE
	
	ret

	FUN_END(kernel_dgelqf_pd_la_dlarft8_8_lib8)





//                                          1       2       3           4           5           6           7
// void kernel_dgelqf_pd_lla_dlarft8_8_lib8(int n0, int n1, double *pD, double *dD, double *pL, double *pA, double *pT)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgelqf_pd_lla_dlarft8_8_lib8)
	
	PROLOGUE

	// zero T

	movq	ARG7, %r10 // T

	vxorpd			%zmm24, %zmm24, %zmm24
	vmovapd			%zmm24, 0*64(%r10)
	vmovapd			%zmm24, 1*64(%r10)
	vmovapd			%zmm24, 2*64(%r10)
	vmovapd			%zmm24, 3*64(%r10)
	vmovapd			%zmm24, 4*64(%r10)
	vmovapd			%zmm24, 5*64(%r10)
	vmovapd			%zmm24, 6*64(%r10)
	vmovapd			%zmm24, 7*64(%r10)

	// first column

	movq	ARG1, %r10 // n0
	movq	ARG5, %r11 // L

	vxorpd			%xmm25, %xmm25, %xmm25
	vmovsd			0(%r11), %xmm24
	vfmadd231sd		%xmm24, %xmm24, %xmm25
//	subl	$ 1, %r10d
	addq	$ 64, %r11
	cmpl	$ 0, %r10d
	jle		101f
100:
	vmovsd			0(%r11), %xmm24
	vfmadd231sd		%xmm24, %xmm24, %xmm25
	subl	$ 1, %r10d
	addq	$ 64, %r11
	cmpl	$ 0, %r10d
	jg		100b
101:

	movq	ARG2, %r10 // n
	movq	ARG6, %r11 // A

//	subl	$ 1, %r10d
//	addq	$ 64, %r11
	cmpl	$ 0, %r10d
	jle		101f
100:
	vmovsd			0(%r11), %xmm24
	vfmadd231sd		%xmm24, %xmm24, %xmm25
	subl	$ 1, %r10d
	addq	$ 64, %r11
	cmpl	$ 0, %r10d
	jg		100b
101:

	movq	ARG4, %r12 // dD
	movq	ARG7, %r13 // T

	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		101f
	vmovsd			%xmm24, 0(%r12)
	jmp		102f

101:
	movq	ARG3, %r11 // D

	leaq	0+64*0(%r11), %r14 // XXX chech for reg overwrite
	leaq	0(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_PD_LIB8
#else
	CALL(inner_edge_dgelqf_pd_lib8)
#endif

	vxorpd			%xmm31, %xmm24, %xmm27
	vmovsd			%xmm27, 0+64*0(%r13) // pT[0+ps*0]

	// D

	vmovapd			0*64(%r11), %zmm0 // TODO k1 for next cols

	// L
	movq	ARG1, %r10 // n0
	movq	ARG5, %r11 // L

	movq	$ 0, %r14 // XXX chech for reg overwrite

	vxorpd			%zmm1, %zmm1, %zmm1 // part acc

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif
	// final triangle
	vbroadcastsd	0*64(%r11, %r14), %zmm16
	vmulpd			%zmm25, %zmm16, %zmm16
//	vbroadcastsd	1*64(%r11, %r14), %zmm17
//	vmulpd			%zmm25, %zmm17, %zmm17
//	vbroadcastsd	2*64(%r11, %r14), %zmm18
//	vmulpd			%zmm25, %zmm18, %zmm18
//	vbroadcastsd	3*64(%r11, %r14), %zmm19
//	vmulpd			%zmm25, %zmm19, %zmm19
//	vbroadcastsd	4*64(%r11, %r14), %zmm20
//	vmulpd			%zmm25, %zmm20, %zmm20
//	vbroadcastsd	5*64(%r11, %r14), %zmm21
//	vmulpd			%zmm25, %zmm21, %zmm21
//	vbroadcastsd	6*64(%r11, %r14), %zmm22
//	vmulpd			%zmm25, %zmm22, %zmm22
//	vbroadcastsd	7*64(%r11, %r14), %zmm23
//	vmulpd			%zmm25, %zmm23, %zmm23
	vfmadd231pd		0*64(%r11), %zmm16, %zmm0 // TODO k1 for next cols
	vmovsd			%xmm16, 0*64(%r11, %r14)
//	vfmadd231pd		1*64(%r11), %zmm17, %zmm1
//	vmovsd			%xmm17, 1*64(%r11, %r14)
//	vfmadd231pd		2*64(%r11), %zmm18, %zmm0
//	vmovsd			%xmm18, 2*64(%r11, %r14)
//	vfmadd231pd		3*64(%r11), %zmm19, %zmm1
//	vmovsd			%xmm19, 3*64(%r11, %r14)
//	vfmadd231pd		4*64(%r11), %zmm20, %zmm0
//	vmovsd			%xmm20, 4*64(%r11, %r14)
//	vfmadd231pd		5*64(%r11), %zmm21, %zmm1
//	vmovsd			%xmm21, 5*64(%r11, %r14)
//	vfmadd231pd		6*64(%r11), %zmm22, %zmm0
//	vmovsd			%xmm22, 6*64(%r11, %r14)
//	vfmadd231pd		7*64(%r11), %zmm23, %zmm1
//	vmovsd			%xmm23, 7*64(%r11, %r14)

	// A
	movq	ARG2, %r10 // n1
	movq	ARG6, %r11 // A

	movq	$ 0, %r14 // XXX chech for reg overwrite

	vxorpd			%zmm1, %zmm1, %zmm1 // part acc

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0xfe, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vbroadcastsd	0(%r13), %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0 {%k1}{z}

	// D
	movq	ARG3, %r11 // D
	vmovapd			0*64(%r11), %zmm16
	vaddpd			%zmm0, %zmm16, %zmm16
	vmovapd			%zmm16, 0*64(%r11)

	// L
	movq	ARG1, %r10 // n0
	movq	ARG5, %r11 // L

	movq	$ 0, %r14 // XXX chech for reg overwrite

	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif
	// final triangle
	vmovapd			0*64(%r11), %zmm16
	movl	$ 0xfe, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			1*64(%r11), %zmm17 {%k1}{z}
//	vmovapd			2*64(%r11), %zmm18
//	vmovapd			3*64(%r11), %zmm19
//	vmovapd			4*64(%r11), %zmm20
//	vmovapd			5*64(%r11), %zmm21
//	vmovapd			6*64(%r11), %zmm22
//	vmovapd			7*64(%r11), %zmm23
	vbroadcastsd	0*64(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm16
//	vbroadcastsd	1*64(%r11, %r14), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm17
//	vbroadcastsd	2*64(%r11, %r14), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm18
//	vbroadcastsd	3*64(%r11, %r14), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm19
//	vbroadcastsd	4*64(%r11, %r14), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm20
//	vbroadcastsd	5*64(%r11, %r14), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm21
//	vbroadcastsd	6*64(%r11, %r14), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm22
//	vbroadcastsd	7*64(%r11, %r14), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vfmadd231pd		%zmm16, %zmm16, %zmm25
	vmovapd			%zmm16, 0*64(%r11)
	vfmadd231pd		%zmm17, %zmm17, %zmm25
//	vmovapd			%zmm17, 1*64(%r11)
//	vfmadd231pd		%zmm18, %zmm18, %zmm25
//	vmovapd			%zmm18, 2*64(%r11)
//	vfmadd231pd		%zmm19, %zmm19, %zmm25
//	vmovapd			%zmm19, 3*64(%r11)
//	vfmadd231pd		%zmm20, %zmm20, %zmm25
//	vmovapd			%zmm20, 4*64(%r11)
//	vfmadd231pd		%zmm21, %zmm21, %zmm25
//	vmovapd			%zmm21, 5*64(%r11)
//	vfmadd231pd		%zmm22, %zmm22, %zmm25
//	vmovapd			%zmm22, 6*64(%r11)
//	vfmadd231pd		%zmm23, %zmm23, %zmm25
//	vmovapd			%zmm23, 7*64(%r11)

	// A
	movq	ARG2, %r10 // n1
	movq	ARG6, %r11 // A

	movq	$ 0, %r14 // XXX chech for reg overwrite

	vxorpd			%zmm26, %zmm26, %zmm26 // part acc

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// second column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 8(%r12)
	jmp		102f

101:
	movq	ARG3, %r11 // D

	leaq	8+64*1(%r11), %r14 // XXX chech for reg overwrite
	leaq	8(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_PD_LIB8
#else
	CALL(inner_edge_dgelqf_pd_lib8)
#endif

	vxorpd			%xmm31, %xmm24, %xmm27
	vmovsd			%xmm27, 8+64*1(%r13) // pT[1+ps*1]

	// D
	movq	ARG3, %r11 // D

	movl	$ 0xfe, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			1*64(%r11), %zmm0 {%k1}{z}

	// L
	movq	ARG1, %r10 // n0
	movq	ARG5, %r11 // L

	movq	$ 8, %r14 // XXX chech for reg overwrite

	vxorpd			%zmm1, %zmm1, %zmm1 // part acc

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif
	// final triangle
	vbroadcastsd	0*64(%r11, %r14), %zmm16
	vmulpd			%zmm25, %zmm16, %zmm16
	vbroadcastsd	1*64(%r11, %r14), %zmm17
	vmulpd			%zmm25, %zmm17, %zmm17
//	vbroadcastsd	2*64(%r11, %r14), %zmm18
//	vmulpd			%zmm25, %zmm18, %zmm18
//	vbroadcastsd	3*64(%r11, %r14), %zmm19
//	vmulpd			%zmm25, %zmm19, %zmm19
//	vbroadcastsd	4*64(%r11, %r14), %zmm20
//	vmulpd			%zmm25, %zmm20, %zmm20
//	vbroadcastsd	5*64(%r11, %r14), %zmm21
//	vmulpd			%zmm25, %zmm21, %zmm21
//	vbroadcastsd	6*64(%r11, %r14), %zmm22
//	vmulpd			%zmm25, %zmm22, %zmm22
//	vbroadcastsd	7*64(%r11, %r14), %zmm23
//	vmulpd			%zmm25, %zmm23, %zmm23
	vfmadd231pd		0*64(%r11), %zmm16, %zmm0 // TODO k1 for next cols
	vmovsd			%xmm16, 0*64(%r11, %r14)
	movl	$ 0xfe, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		1*64(%r11), %zmm17, %zmm0 {%k1}
	vmovsd			%xmm17, 1*64(%r11, %r14)
//	vfmadd231pd		2*64(%r11), %zmm18, %zmm0
//	vmovsd			%xmm18, 2*64(%r11, %r14)
//	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
//	vmovsd			%xmm19, 3*64(%r11, %r14)
//	vfmadd231pd		4*64(%r11), %zmm20, %zmm0
//	vmovsd			%xmm20, 4*64(%r11, %r14)
//	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
//	vmovsd			%xmm21, 5*64(%r11, %r14)
//	vfmadd231pd		6*64(%r11), %zmm22, %zmm0
//	vmovsd			%xmm22, 6*64(%r11, %r14)
//	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
//	vmovsd			%xmm23, 7*64(%r11, %r14)

	// A
	movq	ARG2, %r10 // n
	movq	ARG6, %r11

	movq	$ 8, %r14 // XXX chech for reg overwrite

	vxorpd			%zmm1, %zmm1, %zmm1 // part acc

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC00+16(%rip), %zmm27 // 1.0
#else
	vbroadcastsd	LC00+16(%rip), %zmm27 // 1.0
#endif
	movl	$ 0x01, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			0+64*0(%r13), %zmm27 {%k1}
	vmulpd			%zmm27, %zmm0, %zmm0
	vbroadcastsd	8+64*1(%r13), %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0
	vmovapd			%zmm0, 1*64(%r13) {%k1}

	movl	$ 0xfc, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm0, %zmm0 {%k1}{z}

	// D
	movq	ARG3, %r11 // D
	vmovapd			1*64(%r11), %zmm17
	vaddpd			%zmm0, %zmm17, %zmm17
	vmovapd			%zmm17, 1*64(%r11)

	// L
	movq	ARG1, %r10 // n
	movq	ARG5, %r11

	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc

	movq	$ 8, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif
	// final triangle
	vmovapd			0*64(%r11), %zmm16
	movl	$ 0xfe, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			1*64(%r11), %zmm17 {%k1}{z}
	movl	$ 0xfc, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			2*64(%r11), %zmm18 {%k1}{z}
//	vmovapd			3*64(%r11), %zmm19
//	vmovapd			4*64(%r11), %zmm20
//	vmovapd			5*64(%r11), %zmm21
//	vmovapd			6*64(%r11), %zmm22
//	vmovapd			7*64(%r11), %zmm23
	vbroadcastsd	0*64(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm16
	vbroadcastsd	1*64(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm17
//	vbroadcastsd	2*64(%r11, %r14), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm18
//	vbroadcastsd	3*64(%r11, %r14), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm19
//	vbroadcastsd	4*64(%r11, %r14), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm20
//	vbroadcastsd	5*64(%r11, %r14), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm21
//	vbroadcastsd	6*64(%r11, %r14), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm22
//	vbroadcastsd	7*64(%r11, %r14), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vfmadd231pd		%zmm16, %zmm16, %zmm25
	vmovapd			%zmm16, 0*64(%r11)
	vfmadd231pd		%zmm17, %zmm17, %zmm25
	movl	$ 0xfe, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm17, 1*64(%r11) {%k1}
	vfmadd231pd		%zmm18, %zmm18, %zmm25
//	vmovapd			%zmm18, 2*64(%r11)
//	vfmadd231pd		%zmm19, %zmm19, %zmm25
//	vmovapd			%zmm19, 3*64(%r11)
//	vfmadd231pd		%zmm20, %zmm20, %zmm25
//	vmovapd			%zmm20, 4*64(%r11)
//	vfmadd231pd		%zmm21, %zmm21, %zmm25
//	vmovapd			%zmm21, 5*64(%r11)
//	vfmadd231pd		%zmm22, %zmm22, %zmm25
//	vmovapd			%zmm22, 6*64(%r11)
//	vfmadd231pd		%zmm23, %zmm23, %zmm25
//	vmovapd			%zmm23, 7*64(%r11)

	// A
	movq	ARG2, %r10 // n
	movq	ARG6, %r11

	vxorpd			%zmm26, %zmm26, %zmm26 // part acc

	movq	$ 8, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// third column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 16(%r12)
	jmp		102f

101:
	movq	ARG3, %r11 // D

	leaq	16+64*2(%r11), %r14 // XXX chech for reg overwrite
	leaq	16(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_PD_LIB8
#else
	CALL(inner_edge_dgelqf_pd_lib8)
#endif

	vxorpd			%xmm31, %xmm24, %xmm27
	vmovsd			%xmm27, 16+64*2(%r13) // pT[1+ps*1]

	// D
	movq	ARG3, %r11 // D

	movl	$ 0xfc, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			2*64(%r11), %zmm0 {%k1}{z}

	// L
	movq	ARG1, %r10 // n0
	movq	ARG5, %r11 // L

	movq	$ 16, %r14 // XXX chech for reg overwrite

	vxorpd			%zmm1, %zmm1, %zmm1 // part acc

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif
	// final triangle
	vbroadcastsd	0*64(%r11, %r14), %zmm16
	vmulpd			%zmm25, %zmm16, %zmm16
	vbroadcastsd	1*64(%r11, %r14), %zmm17
	vmulpd			%zmm25, %zmm17, %zmm17
	vbroadcastsd	2*64(%r11, %r14), %zmm18
	vmulpd			%zmm25, %zmm18, %zmm18
//	vbroadcastsd	3*64(%r11, %r14), %zmm19
//	vmulpd			%zmm25, %zmm19, %zmm19
//	vbroadcastsd	4*64(%r11, %r14), %zmm20
//	vmulpd			%zmm25, %zmm20, %zmm20
//	vbroadcastsd	5*64(%r11, %r14), %zmm21
//	vmulpd			%zmm25, %zmm21, %zmm21
//	vbroadcastsd	6*64(%r11, %r14), %zmm22
//	vmulpd			%zmm25, %zmm22, %zmm22
//	vbroadcastsd	7*64(%r11, %r14), %zmm23
//	vmulpd			%zmm25, %zmm23, %zmm23
	vfmadd231pd		0*64(%r11), %zmm16, %zmm0 // TODO k1 for next cols
	vmovsd			%xmm16, 0*64(%r11, %r14)
	movl	$ 0xfe, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		1*64(%r11), %zmm17, %zmm0 {%k1}
	vmovsd			%xmm17, 1*64(%r11, %r14)
	movl	$ 0xfc, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		2*64(%r11), %zmm18, %zmm0 {%k1}
	vmovsd			%xmm18, 2*64(%r11, %r14)
//	vfmadd231pd		3*64(%r11), %zmm19, %zmm0
//	vmovsd			%xmm19, 3*64(%r11, %r14)
//	vfmadd231pd		4*64(%r11), %zmm20, %zmm0
//	vmovsd			%xmm20, 4*64(%r11, %r14)
//	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
//	vmovsd			%xmm21, 5*64(%r11, %r14)
//	vfmadd231pd		6*64(%r11), %zmm22, %zmm0
//	vmovsd			%xmm22, 6*64(%r11, %r14)
//	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
//	vmovsd			%xmm23, 7*64(%r11, %r14)

	// A
	movq	ARG2, %r10 // n
	movq	ARG6, %r11

	movq	$ 16, %r14 // XXX chech for reg overwrite

	vxorpd			%zmm1, %zmm1, %zmm1 // part acc

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0x01, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			0*64(%r13), %zmm24
	vmulpd			%zmm24, %zmm0, %zmm1 {%k1}{z}
	vmovapd			1*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x03, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	vmovapd			%zmm1, %zmm0 {%k1}
	vbroadcastsd	16+2*64(%r13), %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0
	vmovapd			%zmm0, 2*64(%r13) {%k1}

	movl	$ 0xf8, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm0, %zmm0 {%k1}{z}

	// D
	movq	ARG3, %r11 // D
	vmovapd			2*64(%r11), %zmm18
	vaddpd			%zmm0, %zmm18, %zmm18
	vmovapd			%zmm18, 2*64(%r11)

	// L
	movq	ARG1, %r10 // n
	movq	ARG5, %r11

	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc

	movq	$ 16, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif
	// final triangle
	vmovapd			0*64(%r11), %zmm16
	movl	$ 0xfe, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			1*64(%r11), %zmm17 {%k1}{z}
	movl	$ 0xfc, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			2*64(%r11), %zmm18 {%k1}{z}
	movl	$ 0xf8, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			3*64(%r11), %zmm19 {%k1}{z}
//	vmovapd			4*64(%r11), %zmm20
//	vmovapd			5*64(%r11), %zmm21
//	vmovapd			6*64(%r11), %zmm22
//	vmovapd			7*64(%r11), %zmm23
	vbroadcastsd	0*64(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm16
	vbroadcastsd	1*64(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm17
	vbroadcastsd	2*64(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm18
//	vbroadcastsd	3*64(%r11, %r14), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm19
//	vbroadcastsd	4*64(%r11, %r14), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm20
//	vbroadcastsd	5*64(%r11, %r14), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm21
//	vbroadcastsd	6*64(%r11, %r14), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm22
//	vbroadcastsd	7*64(%r11, %r14), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vfmadd231pd		%zmm16, %zmm16, %zmm25
	vmovapd			%zmm16, 0*64(%r11)
	vfmadd231pd		%zmm17, %zmm17, %zmm25
	movl	$ 0xfe, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm17, 1*64(%r11) {%k1}
	vfmadd231pd		%zmm18, %zmm18, %zmm25
	movl	$ 0xfc, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm18, 2*64(%r11) {%k1}
	vfmadd231pd		%zmm19, %zmm19, %zmm25
//	vmovapd			%zmm19, 3*64(%r11)
//	vfmadd231pd		%zmm20, %zmm20, %zmm25
//	vmovapd			%zmm20, 4*64(%r11)
//	vfmadd231pd		%zmm21, %zmm21, %zmm25
//	vmovapd			%zmm21, 5*64(%r11)
//	vfmadd231pd		%zmm22, %zmm22, %zmm25
//	vmovapd			%zmm22, 6*64(%r11)
//	vfmadd231pd		%zmm23, %zmm23, %zmm25
//	vmovapd			%zmm23, 7*64(%r11)

	// A
	movq	ARG2, %r10 // n
	movq	ARG6, %r11

	vxorpd			%zmm26, %zmm26, %zmm26 // part acc

	movq	$ 16, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// fourth column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 24(%r12)
	jmp		102f

101:
	movq	ARG3, %r11 // D

	leaq	24+64*3(%r11), %r14 // XXX chech for reg overwrite
	leaq	24(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_PD_LIB8
#else
	CALL(inner_edge_dgelqf_pd_lib8)
#endif

	vxorpd			%xmm31, %xmm24, %xmm27
	vmovsd			%xmm27, 24+64*3(%r13) // pT[1+ps*1]

	// D
	movl	$ 0xf8, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			3*64(%r11), %zmm0 {%k1}{z}

	// L
	movq	ARG1, %r10 // n0
	movq	ARG5, %r11 // L

	movq	$ 24, %r14 // XXX chech for reg overwrite

	vxorpd			%zmm1, %zmm1, %zmm1 // part acc

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif
	// final triangle
	vbroadcastsd	0*64(%r11, %r14), %zmm16
	vmulpd			%zmm25, %zmm16, %zmm16
	vbroadcastsd	1*64(%r11, %r14), %zmm17
	vmulpd			%zmm25, %zmm17, %zmm17
	vbroadcastsd	2*64(%r11, %r14), %zmm18
	vmulpd			%zmm25, %zmm18, %zmm18
	vbroadcastsd	3*64(%r11, %r14), %zmm19
	vmulpd			%zmm25, %zmm19, %zmm19
//	vbroadcastsd	4*64(%r11, %r14), %zmm20
//	vmulpd			%zmm25, %zmm20, %zmm20
//	vbroadcastsd	5*64(%r11, %r14), %zmm21
//	vmulpd			%zmm25, %zmm21, %zmm21
//	vbroadcastsd	6*64(%r11, %r14), %zmm22
//	vmulpd			%zmm25, %zmm22, %zmm22
//	vbroadcastsd	7*64(%r11, %r14), %zmm23
//	vmulpd			%zmm25, %zmm23, %zmm23
	vfmadd231pd		0*64(%r11), %zmm16, %zmm0
	vmovsd			%xmm16, 0*64(%r11, %r14)
	movl	$ 0xfe, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		1*64(%r11), %zmm17, %zmm0 {%k1}
	vmovsd			%xmm17, 1*64(%r11, %r14)
	movl	$ 0xfc, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		2*64(%r11), %zmm18, %zmm0 {%k1}
	vmovsd			%xmm18, 2*64(%r11, %r14)
	movl	$ 0xf8, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		3*64(%r11), %zmm19, %zmm0 {%k1}
	vmovsd			%xmm19, 3*64(%r11, %r14)
//	vfmadd231pd		4*64(%r11), %zmm20, %zmm0
//	vmovsd			%xmm20, 4*64(%r11, %r14)
//	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
//	vmovsd			%xmm21, 5*64(%r11, %r14)
//	vfmadd231pd		6*64(%r11), %zmm22, %zmm0
//	vmovsd			%xmm22, 6*64(%r11, %r14)
//	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
//	vmovsd			%xmm23, 7*64(%r11, %r14)

	// A
	movq	ARG2, %r10 // n
	movq	ARG6, %r11

	movq	$ 24, %r14 // XXX chech for reg overwrite

	vxorpd			%zmm1, %zmm1, %zmm1 // part acc

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0x01, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			0*64(%r13), %zmm24
	vmulpd			%zmm24, %zmm0, %zmm1 {%k1}{z}
	vmovapd			1*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x03, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	vmovapd			2*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x07, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	vmovapd			%zmm1, %zmm0 {%k1}
	vbroadcastsd	24+3*64(%r13), %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0
	vmovapd			%zmm0, 3*64(%r13) {%k1}

	movl	$ 0xf0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm0, %zmm0 {%k1}{z}

	// D
	movq	ARG3, %r11 // D
	vmovapd			3*64(%r11), %zmm19
	vaddpd			%zmm0, %zmm19, %zmm19
	vmovapd			%zmm19, 3*64(%r11)

	// L
	movq	ARG1, %r10 // n
	movq	ARG5, %r11

	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc

	movq	$ 24, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif
	// final triangle
	vmovapd			0*64(%r11), %zmm16
	movl	$ 0xfe, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			1*64(%r11), %zmm17 {%k1}{z}
	movl	$ 0xfc, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			2*64(%r11), %zmm18 {%k1}{z}
	movl	$ 0xf8, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			3*64(%r11), %zmm19 {%k1}{z}
	movl	$ 0xf0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			4*64(%r11), %zmm20 {%k1}{z}
//	vmovapd			5*64(%r11), %zmm21
//	vmovapd			6*64(%r11), %zmm22
//	vmovapd			7*64(%r11), %zmm23
	vbroadcastsd	0*64(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm16
	vbroadcastsd	1*64(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm17
	vbroadcastsd	2*64(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm18
	vbroadcastsd	3*64(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm19
//	vbroadcastsd	4*64(%r11, %r14), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm20
//	vbroadcastsd	5*64(%r11, %r14), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm21
//	vbroadcastsd	6*64(%r11, %r14), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm22
//	vbroadcastsd	7*64(%r11, %r14), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vfmadd231pd		%zmm16, %zmm16, %zmm25
	vmovapd			%zmm16, 0*64(%r11)
	vfmadd231pd		%zmm17, %zmm17, %zmm25
	movl	$ 0xfe, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm17, 1*64(%r11) {%k1}
	vfmadd231pd		%zmm18, %zmm18, %zmm25
	movl	$ 0xfc, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm18, 2*64(%r11) {%k1}
	vfmadd231pd		%zmm19, %zmm19, %zmm25
	movl	$ 0xf8, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm19, 3*64(%r11) {%k1}
	vfmadd231pd		%zmm20, %zmm20, %zmm25
//	vmovapd			%zmm20, 4*64(%r11)
//	vfmadd231pd		%zmm21, %zmm21, %zmm25
//	vmovapd			%zmm21, 5*64(%r11)
//	vfmadd231pd		%zmm22, %zmm22, %zmm25
//	vmovapd			%zmm22, 6*64(%r11)
//	vfmadd231pd		%zmm23, %zmm23, %zmm25
//	vmovapd			%zmm23, 7*64(%r11)

	// A
	movq	ARG2, %r10 // n
	movq	ARG6, %r11

	movq	$ 24, %r14 // XXX chech for reg overwrite

	vxorpd			%zmm26, %zmm26, %zmm26 // part acc

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// fifth column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 32(%r12)
	jmp		102f

101:
	movq	ARG3, %r11 // D

	leaq	32+64*4(%r11), %r14 // XXX chech for reg overwrite
	leaq	32(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_PD_LIB8
#else
	CALL(inner_edge_dgelqf_pd_lib8)
#endif

	vxorpd			%xmm31, %xmm24, %xmm27
	vmovsd			%xmm27, 32+64*4(%r13) // pT[1+ps*1]

	// D
	movl	$ 0xf0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			4*64(%r11), %zmm0 {%k1}{z}

	// L
	movq	ARG1, %r10 // n0
	movq	ARG5, %r11 // L

	movq	$ 32, %r14 // XXX chech for reg overwrite

	vxorpd			%zmm1, %zmm1, %zmm1 // part acc

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif
	// final triangle
	vbroadcastsd	0*64(%r11, %r14), %zmm16
	vmulpd			%zmm25, %zmm16, %zmm16
	vbroadcastsd	1*64(%r11, %r14), %zmm17
	vmulpd			%zmm25, %zmm17, %zmm17
	vbroadcastsd	2*64(%r11, %r14), %zmm18
	vmulpd			%zmm25, %zmm18, %zmm18
	vbroadcastsd	3*64(%r11, %r14), %zmm19
	vmulpd			%zmm25, %zmm19, %zmm19
	vbroadcastsd	4*64(%r11, %r14), %zmm20
	vmulpd			%zmm25, %zmm20, %zmm20
//	vbroadcastsd	5*64(%r11, %r14), %zmm21
//	vmulpd			%zmm25, %zmm21, %zmm21
//	vbroadcastsd	6*64(%r11, %r14), %zmm22
//	vmulpd			%zmm25, %zmm22, %zmm22
//	vbroadcastsd	7*64(%r11, %r14), %zmm23
//	vmulpd			%zmm25, %zmm23, %zmm23
	vfmadd231pd		0*64(%r11), %zmm16, %zmm0
	vmovsd			%xmm16, 0*64(%r11, %r14)
	movl	$ 0xfe, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		1*64(%r11), %zmm17, %zmm0 {%k1}
	vmovsd			%xmm17, 1*64(%r11, %r14)
	movl	$ 0xfc, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		2*64(%r11), %zmm18, %zmm0 {%k1}
	vmovsd			%xmm18, 2*64(%r11, %r14)
	movl	$ 0xf8, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		3*64(%r11), %zmm19, %zmm0 {%k1}
	vmovsd			%xmm19, 3*64(%r11, %r14)
	movl	$ 0xf0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		4*64(%r11), %zmm20, %zmm0 {%k1}
	vmovsd			%xmm20, 4*64(%r11, %r14)
//	vfmadd231pd		5*64(%r11), %zmm21, %zmm0
//	vmovsd			%xmm21, 5*64(%r11, %r14)
//	vfmadd231pd		6*64(%r11), %zmm22, %zmm0
//	vmovsd			%xmm22, 6*64(%r11, %r14)
//	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
//	vmovsd			%xmm23, 7*64(%r11, %r14)

	// A
	movq	ARG2, %r10 // n
	movq	ARG6, %r11

	movq	$ 32, %r14 // XXX chech for reg overwrite

	vxorpd			%zmm1, %zmm1, %zmm1 // part acc

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0x01, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			0*64(%r13), %zmm24
	vmulpd			%zmm24, %zmm0, %zmm1 {%k1}{z}
	//
	vmovapd			1*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x03, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			2*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x07, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			3*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x0f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			%zmm1, %zmm0 {%k1}
	vbroadcastsd	32+4*64(%r13), %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0
	vmovapd			%zmm0, 4*64(%r13) {%k1}

	movl	$ 0xe0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm0, %zmm0 {%k1}{z}

	// D
	movq	ARG3, %r11 // D
	vmovapd			4*64(%r11), %zmm20
	vaddpd			%zmm0, %zmm20, %zmm20
	vmovapd			%zmm20, 4*64(%r11)

	// L
	movq	ARG1, %r10 // n
	movq	ARG5, %r11

	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc

	movq	$ 32, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif
	// final triangle
	vmovapd			0*64(%r11), %zmm16
	movl	$ 0xfe, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			1*64(%r11), %zmm17 {%k1}{z}
	movl	$ 0xfc, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			2*64(%r11), %zmm18 {%k1}{z}
	movl	$ 0xf8, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			3*64(%r11), %zmm19 {%k1}{z}
	movl	$ 0xf0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			4*64(%r11), %zmm20 {%k1}{z}
	movl	$ 0xe0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			5*64(%r11), %zmm21 {%k1}{z}
//	vmovapd			6*64(%r11), %zmm22
//	vmovapd			7*64(%r11), %zmm23
	vbroadcastsd	0*64(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm16
	vbroadcastsd	1*64(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm17
	vbroadcastsd	2*64(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm18
	vbroadcastsd	3*64(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm19
	vbroadcastsd	4*64(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm20
//	vbroadcastsd	5*64(%r11, %r14), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm21
//	vbroadcastsd	6*64(%r11, %r14), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm22
//	vbroadcastsd	7*64(%r11, %r14), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vfmadd231pd		%zmm16, %zmm16, %zmm25
	vmovapd			%zmm16, 0*64(%r11)
	vfmadd231pd		%zmm17, %zmm17, %zmm25
	movl	$ 0xfe, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm17, 1*64(%r11) {%k1}
	vfmadd231pd		%zmm18, %zmm18, %zmm25
	movl	$ 0xfc, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm18, 2*64(%r11) {%k1}
	vfmadd231pd		%zmm19, %zmm19, %zmm25
	movl	$ 0xf8, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm19, 3*64(%r11) {%k1}
	vfmadd231pd		%zmm20, %zmm20, %zmm25
	movl	$ 0xf0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm20, 4*64(%r11) {%k1}
	vfmadd231pd		%zmm21, %zmm21, %zmm25
//	vmovapd			%zmm21, 5*64(%r11)
//	vfmadd231pd		%zmm22, %zmm22, %zmm25
//	vmovapd			%zmm22, 6*64(%r11)
//	vfmadd231pd		%zmm23, %zmm23, %zmm25
//	vmovapd			%zmm23, 7*64(%r11)

	// A
	movq	ARG2, %r10 // n
	movq	ARG6, %r11

	movq	$ 32, %r14 // XXX chech for reg overwrite

	vxorpd			%zmm26, %zmm26, %zmm26 // part acc

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// sixth column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 40(%r12)
	jmp		102f

101:
	movq	ARG3, %r11 // D

	leaq	40+64*5(%r11), %r14 // XXX chech for reg overwrite
	leaq	40(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_PD_LIB8
#else
	CALL(inner_edge_dgelqf_pd_lib8)
#endif

	vxorpd			%xmm31, %xmm24, %xmm27
	vmovsd			%xmm27, 40+64*5(%r13) // pT[1+ps*1]

	// D
	movl	$ 0xc0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			5*64(%r11), %zmm0 {%k1}{z}

	// L
	movq	ARG1, %r10 // n0
	movq	ARG5, %r11 // L

	movq	$ 40, %r14 // XXX chech for reg overwrite

	vxorpd			%zmm1, %zmm1, %zmm1 // part acc

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif
	// final triangle
	vbroadcastsd	0*64(%r11, %r14), %zmm16
	vmulpd			%zmm25, %zmm16, %zmm16
	vbroadcastsd	1*64(%r11, %r14), %zmm17
	vmulpd			%zmm25, %zmm17, %zmm17
	vbroadcastsd	2*64(%r11, %r14), %zmm18
	vmulpd			%zmm25, %zmm18, %zmm18
	vbroadcastsd	3*64(%r11, %r14), %zmm19
	vmulpd			%zmm25, %zmm19, %zmm19
	vbroadcastsd	4*64(%r11, %r14), %zmm20
	vmulpd			%zmm25, %zmm20, %zmm20
	vbroadcastsd	5*64(%r11, %r14), %zmm21
	vmulpd			%zmm25, %zmm21, %zmm21
//	vbroadcastsd	6*64(%r11, %r14), %zmm22
//	vmulpd			%zmm25, %zmm22, %zmm22
//	vbroadcastsd	7*64(%r11, %r14), %zmm23
//	vmulpd			%zmm25, %zmm23, %zmm23
	vfmadd231pd		0*64(%r11), %zmm16, %zmm0
	vmovsd			%xmm16, 0*64(%r11, %r14)
	movl	$ 0xfe, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		1*64(%r11), %zmm17, %zmm0 {%k1}
	vmovsd			%xmm17, 1*64(%r11, %r14)
	movl	$ 0xfc, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		2*64(%r11), %zmm18, %zmm0 {%k1}
	vmovsd			%xmm18, 2*64(%r11, %r14)
	movl	$ 0xf8, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		3*64(%r11), %zmm19, %zmm0 {%k1}
	vmovsd			%xmm19, 3*64(%r11, %r14)
	movl	$ 0xf0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		4*64(%r11), %zmm20, %zmm0 {%k1}
	vmovsd			%xmm20, 4*64(%r11, %r14)
	movl	$ 0xe0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		5*64(%r11), %zmm21, %zmm0 {%k1}
	vmovsd			%xmm21, 5*64(%r11, %r14)
//	vfmadd231pd		6*64(%r11), %zmm22, %zmm0
//	vmovsd			%xmm22, 6*64(%r11, %r14)
//	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
//	vmovsd			%xmm23, 7*64(%r11, %r14)

	// A
	movq	ARG2, %r10 // n
	movq	ARG6, %r11

	movq	$ 40, %r14 // XXX chech for reg overwrite

	vxorpd			%zmm1, %zmm1, %zmm1 // part acc

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0x01, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			0*64(%r13), %zmm24
	vmulpd			%zmm24, %zmm0, %zmm1 {%k1}{z}
	//
	vmovapd			1*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x03, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			2*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x07, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			3*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x0f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			4*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x1f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			%zmm1, %zmm0 {%k1}
	vbroadcastsd	40+5*64(%r13), %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0
	vmovapd			%zmm0, 5*64(%r13) {%k1}

	movl	$ 0xc0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm0, %zmm0 {%k1}{z}

	// D
	movq	ARG3, %r11 // D
	vmovapd			5*64(%r11), %zmm21
	vaddpd			%zmm0, %zmm21, %zmm21
	vmovapd			%zmm21, 5*64(%r11)

	// L
	movq	ARG1, %r10 // n
	movq	ARG5, %r11

	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc

	movq	$ 40, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif
	// final triangle
	vmovapd			0*64(%r11), %zmm16
	movl	$ 0xfe, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			1*64(%r11), %zmm17 {%k1}{z}
	movl	$ 0xfc, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			2*64(%r11), %zmm18 {%k1}{z}
	movl	$ 0xf8, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			3*64(%r11), %zmm19 {%k1}{z}
	movl	$ 0xf0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			4*64(%r11), %zmm20 {%k1}{z}
	movl	$ 0xe0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			5*64(%r11), %zmm21 {%k1}{z}
	movl	$ 0xc0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			6*64(%r11), %zmm22 {%k1}{z}
//	vmovapd			7*64(%r11), %zmm23
	vbroadcastsd	0*64(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm16
	vbroadcastsd	1*64(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm17
	vbroadcastsd	2*64(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm18
	vbroadcastsd	3*64(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm19
	vbroadcastsd	4*64(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm20
	vbroadcastsd	5*64(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm21
//	vbroadcastsd	6*64(%r11, %r14), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm22
//	vbroadcastsd	7*64(%r11, %r14), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vfmadd231pd		%zmm16, %zmm16, %zmm25
	vmovapd			%zmm16, 0*64(%r11)
	vfmadd231pd		%zmm17, %zmm17, %zmm25
	movl	$ 0xfe, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm17, 1*64(%r11) {%k1}
	vfmadd231pd		%zmm18, %zmm18, %zmm25
	movl	$ 0xfc, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm18, 2*64(%r11) {%k1}
	vfmadd231pd		%zmm19, %zmm19, %zmm25
	movl	$ 0xf8, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm19, 3*64(%r11) {%k1}
	vfmadd231pd		%zmm20, %zmm20, %zmm25
	movl	$ 0xf0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm20, 4*64(%r11) {%k1}
	vfmadd231pd		%zmm21, %zmm21, %zmm25
	movl	$ 0xe0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm21, 5*64(%r11) {%k1}
	vfmadd231pd		%zmm22, %zmm22, %zmm25
//	vmovapd			%zmm22, 6*64(%r11)
//	vfmadd231pd		%zmm23, %zmm23, %zmm25
//	vmovapd			%zmm23, 7*64(%r11)

	// A
	movq	ARG2, %r10 // n
	movq	ARG6, %r11

	movq	$ 40, %r14 // XXX chech for reg overwrite

	vxorpd			%zmm26, %zmm26, %zmm26 // part acc

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// seventh column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 48(%r12)
	jmp		102f

101:
	movq	ARG3, %r11 // D

	leaq	48+64*6(%r11), %r14 // XXX chech for reg overwrite
	leaq	48(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_PD_LIB8
#else
	CALL(inner_edge_dgelqf_pd_lib8)
#endif

	vxorpd			%xmm31, %xmm24, %xmm27
	vmovsd			%xmm27, 48+64*6(%r13) // pT[1+ps*1]

	// D
	movl	$ 0xc0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			6*64(%r11), %zmm0 {%k1}{z}

	// L
	movq	ARG1, %r10 // n0
	movq	ARG5, %r11 // L

	movq	$ 48, %r14 // XXX chech for reg overwrite

	vxorpd			%zmm1, %zmm1, %zmm1 // part acc

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif
	// final triangle
	vbroadcastsd	0*64(%r11, %r14), %zmm16
	vmulpd			%zmm25, %zmm16, %zmm16
	vbroadcastsd	1*64(%r11, %r14), %zmm17
	vmulpd			%zmm25, %zmm17, %zmm17
	vbroadcastsd	2*64(%r11, %r14), %zmm18
	vmulpd			%zmm25, %zmm18, %zmm18
	vbroadcastsd	3*64(%r11, %r14), %zmm19
	vmulpd			%zmm25, %zmm19, %zmm19
	vbroadcastsd	4*64(%r11, %r14), %zmm20
	vmulpd			%zmm25, %zmm20, %zmm20
	vbroadcastsd	5*64(%r11, %r14), %zmm21
	vmulpd			%zmm25, %zmm21, %zmm21
	vbroadcastsd	6*64(%r11, %r14), %zmm22
	vmulpd			%zmm25, %zmm22, %zmm22
//	vbroadcastsd	7*64(%r11, %r14), %zmm23
//	vmulpd			%zmm25, %zmm23, %zmm23
	vfmadd231pd		0*64(%r11), %zmm16, %zmm0
	vmovsd			%xmm16, 0*64(%r11, %r14)
	movl	$ 0xfe, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		1*64(%r11), %zmm17, %zmm0 {%k1}
	vmovsd			%xmm17, 1*64(%r11, %r14)
	movl	$ 0xfc, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		2*64(%r11), %zmm18, %zmm0 {%k1}
	vmovsd			%xmm18, 2*64(%r11, %r14)
	movl	$ 0xf8, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		3*64(%r11), %zmm19, %zmm0 {%k1}
	vmovsd			%xmm19, 3*64(%r11, %r14)
	movl	$ 0xf0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		4*64(%r11), %zmm20, %zmm0 {%k1}
	vmovsd			%xmm20, 4*64(%r11, %r14)
	movl	$ 0xe0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		5*64(%r11), %zmm21, %zmm0 {%k1}
	vmovsd			%xmm21, 5*64(%r11, %r14)
	movl	$ 0xc0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		6*64(%r11), %zmm22, %zmm0 {%k1}
	vmovsd			%xmm22, 6*64(%r11, %r14)
//	vfmadd231pd		7*64(%r11), %zmm23, %zmm0
//	vmovsd			%xmm23, 7*64(%r11, %r14)

	// A
	movq	ARG2, %r10 // n
	movq	ARG6, %r11

	movq	$ 48, %r14 // XXX chech for reg overwrite

	vxorpd			%zmm1, %zmm1, %zmm1 // part acc

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0x01, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			0*64(%r13), %zmm24
	vmulpd			%zmm24, %zmm0, %zmm1 {%k1}{z}
	//
	vmovapd			1*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x03, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			2*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x07, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			3*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x0f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			4*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x1f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			5*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x3f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			%zmm1, %zmm0 {%k1}
	vbroadcastsd	48+6*64(%r13), %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0
	vmovapd			%zmm0, 6*64(%r13) {%k1}

	movl	$ 0x80, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm0, %zmm0 {%k1}{z}

	// D
	movq	ARG3, %r11 // D
	vmovapd			6*64(%r11), %zmm22
	vaddpd			%zmm0, %zmm22, %zmm22
	vmovapd			%zmm22, 6*64(%r11)

	// L
	movq	ARG1, %r10 // n
	movq	ARG5, %r11

	vxorpd			%zmm25, %zmm25, %zmm25
	vxorpd			%zmm26, %zmm26, %zmm26 // part acc

	movq	$ 48, %r14 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif
	// final triangle
	vmovapd			0*64(%r11), %zmm16
	movl	$ 0xfe, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			1*64(%r11), %zmm17 {%k1}{z}
	movl	$ 0xfc, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			2*64(%r11), %zmm18 {%k1}{z}
	movl	$ 0xf8, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			3*64(%r11), %zmm19 {%k1}{z}
	movl	$ 0xf0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			4*64(%r11), %zmm20 {%k1}{z}
	movl	$ 0xe0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			5*64(%r11), %zmm21 {%k1}{z}
	movl	$ 0xc0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			6*64(%r11), %zmm22 {%k1}{z}
	movl	$ 0x80, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			7*64(%r11), %zmm23 {%k1}{z}
	vbroadcastsd	0*64(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm16
	vbroadcastsd	1*64(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm17
	vbroadcastsd	2*64(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm18
	vbroadcastsd	3*64(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm19
	vbroadcastsd	4*64(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm20
	vbroadcastsd	5*64(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm21
	vbroadcastsd	6*64(%r11, %r14), %zmm24
	vfmadd231pd		%zmm0, %zmm24, %zmm22
//	vbroadcastsd	7*64(%r11, %r14), %zmm24
//	vfmadd231pd		%zmm0, %zmm24, %zmm23
	vfmadd231pd		%zmm16, %zmm16, %zmm25
	vmovapd			%zmm16, 0*64(%r11)
	vfmadd231pd		%zmm17, %zmm17, %zmm25
	movl	$ 0xfe, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm17, 1*64(%r11) {%k1}
	vfmadd231pd		%zmm18, %zmm18, %zmm25
	movl	$ 0xfc, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm18, 2*64(%r11) {%k1}
	vfmadd231pd		%zmm19, %zmm19, %zmm25
	movl	$ 0xf8, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm19, 3*64(%r11) {%k1}
	vfmadd231pd		%zmm20, %zmm20, %zmm25
	movl	$ 0xf0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm20, 4*64(%r11) {%k1}
	vfmadd231pd		%zmm21, %zmm21, %zmm25
	movl	$ 0xe0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm21, 5*64(%r11) {%k1}
	vfmadd231pd		%zmm22, %zmm22, %zmm25
	movl	$ 0xc0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			%zmm22, 6*64(%r11) {%k1}
	vfmadd231pd		%zmm23, %zmm23, %zmm25
//	vmovapd			%zmm23, 7*64(%r11)

	// A
	movq	ARG2, %r10 // n
	movq	ARG6, %r11

	movq	$ 48, %r14 // XXX chech for reg overwrite

	vxorpd			%zmm26, %zmm26, %zmm26 // part acc

#if MACRO_LEVEL>=1
	INNER_KERNEL_2_DGELQF_8_LIB8
#else
	CALL(inner_kernel_2_dgelqf_8_lib8)
#endif

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+7*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+7*8(%rip), %zmm27
#endif
	vpermpd	%zmm25, %zmm27, %zmm25  // beta


	// eigth column
102:
	vxorpd			%xmm24, %xmm24, %xmm24
	vucomisd		%xmm24, %xmm25
	jne		101f
//	jp		111f
	vmovsd			%xmm24, 56(%r12)
	jmp		102f

101:
	movq	ARG3, %r11 // D

	leaq	56+64*7(%r11), %r14 // XXX chech for reg overwrite
	leaq	56(%r12), %r15 // XXX chech for reg overwrite

#if MACRO_LEVEL>=1
	INNER_EDGE_DGELQF_PD_LIB8
#else
	CALL(inner_edge_dgelqf_pd_lib8)
#endif

	vxorpd			%xmm31, %xmm24, %xmm27
	vmovsd			%xmm27, 56+64*7(%r13) // pT[1+ps*1]

	// D
	movl	$ 0x80, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			7*64(%r11), %zmm0 {%k1}{z}

	// L
	movq	ARG1, %r10 // n0
	movq	ARG5, %r11 // L

	movq	$ 56, %r14 // XXX chech for reg overwrite

	vxorpd			%zmm1, %zmm1, %zmm1 // part acc

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif
	// final triangle
	vbroadcastsd	0*64(%r11, %r14), %zmm16
	vmulpd			%zmm25, %zmm16, %zmm16
	vbroadcastsd	1*64(%r11, %r14), %zmm17
	vmulpd			%zmm25, %zmm17, %zmm17
	vbroadcastsd	2*64(%r11, %r14), %zmm18
	vmulpd			%zmm25, %zmm18, %zmm18
	vbroadcastsd	3*64(%r11, %r14), %zmm19
	vmulpd			%zmm25, %zmm19, %zmm19
	vbroadcastsd	4*64(%r11, %r14), %zmm20
	vmulpd			%zmm25, %zmm20, %zmm20
	vbroadcastsd	5*64(%r11, %r14), %zmm21
	vmulpd			%zmm25, %zmm21, %zmm21
	vbroadcastsd	6*64(%r11, %r14), %zmm22
	vmulpd			%zmm25, %zmm22, %zmm22
	vbroadcastsd	7*64(%r11, %r14), %zmm23
	vmulpd			%zmm25, %zmm23, %zmm23
	vfmadd231pd		0*64(%r11), %zmm16, %zmm0
	vmovsd			%xmm16, 0*64(%r11, %r14)
	movl	$ 0xfe, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		1*64(%r11), %zmm17, %zmm0 {%k1}
	vmovsd			%xmm17, 1*64(%r11, %r14)
	movl	$ 0xfc, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		2*64(%r11), %zmm18, %zmm0 {%k1}
	vmovsd			%xmm18, 2*64(%r11, %r14)
	movl	$ 0xf8, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		3*64(%r11), %zmm19, %zmm0 {%k1}
	vmovsd			%xmm19, 3*64(%r11, %r14)
	movl	$ 0xf0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		4*64(%r11), %zmm20, %zmm0 {%k1}
	vmovsd			%xmm20, 4*64(%r11, %r14)
	movl	$ 0xe0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		5*64(%r11), %zmm21, %zmm0 {%k1}
	vmovsd			%xmm21, 5*64(%r11, %r14)
	movl	$ 0xc0, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		6*64(%r11), %zmm22, %zmm0 {%k1}
	vmovsd			%xmm22, 6*64(%r11, %r14)
	movl	$ 0x80, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		7*64(%r11), %zmm23, %zmm0 {%k1}
	vmovsd			%xmm23, 7*64(%r11, %r14)

	// A
	movq	ARG2, %r10 // n
	movq	ARG6, %r11

	movq	$ 56, %r14 // XXX chech for reg overwrite

	vxorpd			%zmm1, %zmm1, %zmm1 // part acc

#if MACRO_LEVEL>=1
	INNER_KERNEL_1_DGELQF_8_LIB8
#else
	CALL(inner_kernel_1_dgelqf_8_lib8)
#endif

	movl	$ 0x01, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vmovapd			0*64(%r13), %zmm24
	vmulpd			%zmm24, %zmm0, %zmm1 {%k1}{z}
	//
	vmovapd			1*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+1*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+1*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x03, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			2*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+2*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+2*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x07, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			3*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+3*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+3*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x0f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			4*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+4*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+4*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x1f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			5*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+5*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+5*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x3f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			6*64(%r13), %zmm24
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vbroadcastsd	.LC05+6*8(%rip), %zmm27
#elif defined(OS_MAC)
	vbroadcastsd	LC05+6*8(%rip), %zmm27
#endif
	vpermpd			%zmm0, %zmm27, %zmm26
	movl	$ 0x7f, %ebx // XXX check for not reg overwrite
	kmovd	%ebx, %k1 // XXX check for not reg overwrite
	vfmadd231pd		%zmm24, %zmm26, %zmm1 {%k1}
	//
	vmovapd			%zmm1, %zmm0 {%k1}
	vbroadcastsd	56+7*64(%r13), %zmm25
	vmulpd			%zmm25, %zmm0, %zmm0
	vmovapd			%zmm0, 7*64(%r13) {%k1}

102:

1000:
	EPILOGUE
	
	ret

	FUN_END(kernel_dgelqf_pd_lla_dlarft8_8_lib8)





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
.LC00: // { 100...0 -1.0 1.0 }
#elif defined(OS_MAC)
LC00: // { 100...0 -1.0 1.0 }
	.align 6
#endif
	.long	0x00000000
	.long	0x80000000
	.double	-1.0
	.double	1.0

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

