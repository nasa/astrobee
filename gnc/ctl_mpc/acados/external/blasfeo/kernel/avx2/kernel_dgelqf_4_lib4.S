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





//                                   1      2           3        4           5
// void kernel_dgelqf_dlarft12_12_lib4(int n, double *pD, int sdd, double *dD, double *pT)

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dgelqf_dlarft12_12_lib4
	.type kernel_dgelqf_dlarft12_12_lib4, @function
kernel_dgelqf_dlarft12_12_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dgelqf_dlarft12_12_lib4
_kernel_dgelqf_dlarft12_12_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dgelqf_dlarft12_12_lib4
	.def kernel_dgelqf_dlarft12_12_lib4; .scl 2; .type 32; .endef
kernel_dgelqf_dlarft12_12_lib4:
#endif
	
	PROLOGUE

	// zero T

	movq	ARG5, %r10 // T

	vxorpd			%ymm15, %ymm15, %ymm15
	vmovapd			%ymm15, 0(%r10)
	vmovapd			%ymm15, 32(%r10)
	vmovapd			%ymm15, 64(%r10)
	vmovapd			%ymm15, 96(%r10)

	// first column

	movq	ARG2, %r11 // D
	movq	ARG3, %r14 // sdd
	sall	$ 5, %r14d
	movq	ARG4, %r12 // dD
	movq	ARG5, %r13 // T
	movq	$ 384, %r15 // sdt !!!!!!!!!!!!!!!!!!!!!!!!!

	vxorpd			%xmm15, %xmm15, %xmm15
	movq	ARG1, %r10 // n
	subl	$ 1, %r10d
	addq	$ 32, %r11
100:
	vmovsd			0(%r11), %xmm14
	vfmadd231sd		%xmm14, %xmm14, %xmm15
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		100b

	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		101f
	vmovsd			%xmm14, 0(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	vmovsd			0(%r11), %xmm14 // alpha
	vfmadd231sd		%xmm14, %xmm14, %xmm15 // beta
	vsqrtsd			%xmm15, %xmm15, %xmm15 // beta
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm13 // mask
#else
	vmovsd			LC00(%rip), %xmm13 // mask
#endif
	vandpd			%xmm13, %xmm14, %xmm12
	vxorpd			%xmm13, %xmm12, %xmm12
	vxorpd			%xmm12, %xmm15, %xmm15 // beta
	vmovsd			%xmm15, 0(%r11) // pD[0+ps*0]
	vsubsd			%xmm14, %xmm15, %xmm14 // beta-alpha
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC01(%rip), %xmm12
#else
	vmovapd			LC01(%rip), %xmm12
#endif
	vmovsd			%xmm14, %xmm12, %xmm12
	vmovddup		%xmm14, %xmm14
	vmovsd			%xmm15, %xmm14, %xmm14
	vdivpd			%xmm14, %xmm12, %xmm14
	vmovsd			%xmm14, 0(%r12) // dD[0]
	vxorpd			%xmm13, %xmm14, %xmm12
	vmovsd			%xmm12, 0(%r13) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			0(%r11), %ymm0
	vmovapd			0(%r11, %r14, 1), %ymm1
	vmovapd			0(%r11, %r14, 2), %ymm2
	vbroadcastsd	32(%r11), %ymm8
	vbroadcastsd	64(%r11), %ymm9
	vbroadcastsd	96(%r11), %ymm10
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vfmadd231pd		32(%r11), %ymm8, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		32(%r11, %r14, 2), %ymm8, %ymm2
	vfmadd231pd		64(%r11), %ymm9, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		96(%r11), %ymm10, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm10, %ymm2
	vmovsd			%xmm8, 32(%r11)
	vmovsd			%xmm9, 64(%r11)
	vmovsd			%xmm10, 96(%r11)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	0(%r11), %ymm8
	vbroadcastsd	32(%r11), %ymm9
	vbroadcastsd	64(%r11), %ymm10
	vbroadcastsd	96(%r11), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		32(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm10, %ymm2
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm11, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm11, %ymm2
	vmovsd			%xmm8, 0(%r11)
	vmovsd			%xmm9, 32(%r11)
	vmovsd			%xmm10, 64(%r11)
	vmovsd			%xmm11, 96(%r11)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	0(%r11), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vmovsd			%xmm8, 0(%r11)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vbroadcastsd	0(%r13), %ymm15
	vmulpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm15, %ymm2, %ymm2

	vxorpd			%ymm15, %ymm15, %ymm15
	vblendpd		$ 0x1, %ymm15, %ymm0, %ymm0

	movq	ARG2, %r11 // D
	//
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vaddpd			%ymm0, %ymm8, %ymm8
	vaddpd			%ymm1, %ymm9, %ymm9
	vaddpd			%ymm2, %ymm10, %ymm10
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11), %ymm8
	vmovapd			32(%r11, %r14, 1), %ymm9
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	32(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmovapd			%ymm8, 32(%r11)
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11), %ymm8
	vmovapd			64(%r11, %r14, 1), %ymm9
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	64(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmulpd			%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 64(%r11)
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11), %ymm8
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	96(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 96(%r11)
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	//
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	0(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11), %ymm8
	vmovapd			32(%r11, %r14, 1), %ymm9
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	32(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 32(%r11)
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11), %ymm8
	vmovapd			64(%r11, %r14, 1), %ymm9
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	64(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 64(%r11)
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11), %ymm8
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	96(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 96(%r11)
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	//
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	0(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
	vpermpd	$ 0x55, %ymm15, %ymm15  // beta

	// second column
102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 8(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	vmovsd			40(%r11), %xmm14 // alpha
	vfmadd231sd		%xmm14, %xmm14, %xmm15 // beta
	vsqrtsd			%xmm15, %xmm15, %xmm15 // beta
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm13 // mask
#else
	vmovsd			LC00(%rip), %xmm13 // mask
#endif
	vandpd			%xmm13, %xmm14, %xmm12
	vxorpd			%xmm13, %xmm12, %xmm12
	vxorpd			%xmm12, %xmm15, %xmm15 // beta
	vmovsd			%xmm15, 40(%r11) // pD[0+ps*0]
	vsubsd			%xmm14, %xmm15, %xmm14 // beta-alpha
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC01(%rip), %xmm12
#else
	vmovapd			LC01(%rip), %xmm12
#endif
	vmovsd			%xmm14, %xmm12, %xmm12
	vmovddup		%xmm14, %xmm14
	vmovsd			%xmm15, %xmm14, %xmm14
	vdivpd			%xmm14, %xmm12, %xmm14
	vmovsd			%xmm14, 8(%r12) // dD[0]
	vxorpd			%xmm13, %xmm14, %xmm12
	vmovsd			%xmm12, 40(%r13) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			32(%r11), %ymm0
	vmovapd			32(%r11, %r14, 1), %ymm1
	vmovapd			32(%r11, %r14, 2), %ymm2
	vbroadcastsd	72(%r11), %ymm9
	vbroadcastsd	104(%r11), %ymm10
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vfmadd231pd		64(%r11), %ymm9, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		96(%r11), %ymm10, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm10, %ymm2
	vmovsd			%xmm9, 72(%r11)
	vmovsd			%xmm10, 104(%r11)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	8(%r11), %ymm8
	vbroadcastsd	40(%r11), %ymm9
	vbroadcastsd	72(%r11), %ymm10
	vbroadcastsd	104(%r11), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		32(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm10, %ymm2
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm11, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm11, %ymm2
	vmovsd			%xmm8, 8(%r11)
	vmovsd			%xmm9, 40(%r11)
	vmovsd			%xmm10, 72(%r11)
	vmovsd			%xmm11, 104(%r11)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	8(%r11), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vmovsd			%xmm8, 8(%r11)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC02(%rip), %ymm12
#else
	vmovapd			LC02(%rip), %ymm12
#endif
	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm0
	vbroadcastsd	40(%r13), %ymm15
	vmulpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm15, %ymm2, %ymm2
	vmovsd			%xmm0, 32(%r13)

	vxorpd			%ymm12, %ymm12, %ymm12
	vblendpd		$ 0x3, %ymm12, %ymm0, %ymm0

	movq	ARG2, %r11 // D
	//
	vmovapd			32(%r11), %ymm8
	vmovapd			32(%r11, %r14, 1), %ymm9
	vmovapd			32(%r11, %r14, 2), %ymm10
	vaddpd			%ymm0, %ymm8, %ymm8
	vaddpd			%ymm1, %ymm9, %ymm9
	vaddpd			%ymm2, %ymm10, %ymm10
	vmovapd			%ymm8, 32(%r11)
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11), %ymm8
	vmovapd			64(%r11, %r14, 1), %ymm9
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	72(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmovapd			%ymm8, 64(%r11)
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11), %ymm8
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	104(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmulpd			%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 96(%r11)
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	//
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	8(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11), %ymm8
	vmovapd			32(%r11, %r14, 1), %ymm9
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	40(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 32(%r11)
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11), %ymm8
	vmovapd			64(%r11, %r14, 1), %ymm9
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	72(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 64(%r11)
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11), %ymm8
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	104(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 96(%r11)
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	8(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
	vpermpd	$ 0xaa, %ymm15, %ymm15  // beta

	// third column
102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 16(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	vmovsd			80(%r11), %xmm14 // alpha
	vfmadd231sd		%xmm14, %xmm14, %xmm15 // beta
	vsqrtsd			%xmm15, %xmm15, %xmm15 // beta
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm13 // mask
#else
	vmovsd			LC00(%rip), %xmm13 // mask
#endif
	vandpd			%xmm13, %xmm14, %xmm12
	vxorpd			%xmm13, %xmm12, %xmm12
	vxorpd			%xmm12, %xmm15, %xmm15 // beta
	vmovsd			%xmm15, 80(%r11) // pD[0+ps*0]
	vsubsd			%xmm14, %xmm15, %xmm14 // beta-alpha
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC01(%rip), %xmm12
#else
	vmovapd			LC01(%rip), %xmm12
#endif
	vmovsd			%xmm14, %xmm12, %xmm12
	vmovddup		%xmm14, %xmm14
	vmovsd			%xmm15, %xmm14, %xmm14
	vdivpd			%xmm14, %xmm12, %xmm14
	vmovsd			%xmm14, 16(%r12) // dD[0]
	vxorpd			%xmm13, %xmm14, %xmm12
	vmovsd			%xmm12, 80(%r13) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			64(%r11), %ymm0
	vmovapd			64(%r11, %r14, 1), %ymm1
	vmovapd			64(%r11, %r14, 2), %ymm2
	vbroadcastsd	112(%r11), %ymm10
	vmulpd			%ymm15, %ymm10, %ymm10
	vfmadd231pd		96(%r11), %ymm10, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm10, %ymm2
	vmovsd			%xmm10, 112(%r11)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	16(%r11), %ymm8
	vbroadcastsd	48(%r11), %ymm9
	vbroadcastsd	80(%r11), %ymm10
	vbroadcastsd	112(%r11), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		32(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm10, %ymm2
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm11, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm11, %ymm2
	vmovsd			%xmm8, 16(%r11)
	vmovsd			%xmm9, 48(%r11)
	vmovsd			%xmm10, 80(%r11)
	vmovsd			%xmm11, 112(%r11)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	16(%r11), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vmovsd			%xmm8, 16(%r11)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vxorpd			%xmm12, %xmm12, %xmm12
	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm15
	vmovapd			32(%r13), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vpermpd			$ 0x55, %ymm0, %ymm13
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vblendpd		$ 0x7, %ymm15, %ymm0, %ymm0
	vbroadcastsd	80(%r13), %ymm14
	vmulpd			%ymm14, %ymm0, %ymm0
	vmulpd			%ymm14, %ymm1, %ymm1
	vmulpd			%ymm14, %ymm2, %ymm2
	vmovapd			%xmm0, 64(%r13)

	vxorpd			%ymm12, %ymm12, %ymm12
	vblendpd		$ 0x7, %ymm12, %ymm0, %ymm0

	movq	ARG2, %r11 // D
	//
	vmovapd			64(%r11), %ymm8
	vmovapd			64(%r11, %r14, 1), %ymm9
	vmovapd			64(%r11, %r14, 2), %ymm10
	vaddpd			%ymm0, %ymm8, %ymm8
	vaddpd			%ymm1, %ymm9, %ymm9
	vaddpd			%ymm2, %ymm10, %ymm10
	vmovapd			%ymm8, 64(%r11)
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11), %ymm8
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	112(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vxorpd			%ymm15, %ymm15, %ymm15
	vmovapd			%ymm8, 96(%r11)
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	//
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	16(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11), %ymm8
	vmovapd			32(%r11, %r14, 1), %ymm9
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	48(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 32(%r11)
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11), %ymm8
	vmovapd			64(%r11, %r14, 1), %ymm9
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	80(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 64(%r11)
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11), %ymm8
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	112(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 96(%r11)
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	16(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
	vpermpd	$ 0xff, %ymm15, %ymm15  // beta

	// fourth column
102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 24(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	vmovsd			120(%r11), %xmm14 // alpha
	vfmadd231sd		%xmm14, %xmm14, %xmm15 // beta
	vsqrtsd			%xmm15, %xmm15, %xmm15 // beta
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm13 // mask
#else
	vmovsd			LC00(%rip), %xmm13 // mask
#endif
	vandpd			%xmm13, %xmm14, %xmm12
	vxorpd			%xmm13, %xmm12, %xmm12
	vxorpd			%xmm12, %xmm15, %xmm15 // beta
	vmovsd			%xmm15, 120(%r11) // pD[0+ps*0]
	vsubsd			%xmm14, %xmm15, %xmm14 // beta-alpha
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC01(%rip), %xmm12
#else
	vmovapd			LC01(%rip), %xmm12
#endif
	vmovsd			%xmm14, %xmm12, %xmm12
	vmovddup		%xmm14, %xmm14
	vmovsd			%xmm15, %xmm14, %xmm14
	vdivpd			%xmm14, %xmm12, %xmm14
	vmovsd			%xmm14, 24(%r12) // dD[0]
	vxorpd			%xmm13, %xmm14, %xmm12
	vmovsd			%xmm12, 120(%r13) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			96(%r11), %ymm0
	vmovapd			96(%r11, %r14, 1), %ymm1
	vmovapd			96(%r11, %r14, 2), %ymm2
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	24(%r11), %ymm8
	vbroadcastsd	56(%r11), %ymm9
	vbroadcastsd	88(%r11), %ymm10
	vbroadcastsd	120(%r11), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		32(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm10, %ymm2
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm11, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm11, %ymm2
	vmovsd			%xmm8, 24(%r11)
	vmovsd			%xmm9, 56(%r11)
	vmovsd			%xmm10, 88(%r11)
	vmovsd			%xmm11, 120(%r11)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	24(%r11), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vmovsd			%xmm8, 24(%r11)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vxorpd			%xmm12, %xmm12, %xmm12
	//
	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm15
	//
	vmovapd			32(%r13), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vpermpd			$ 0x55, %ymm0, %ymm13
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vmovapd			64(%r13), %ymm14
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm14
	vpermpd			$ 0xaa, %ymm0, %ymm13
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vbroadcastsd	120(%r13), %ymm14
	vmulpd			%ymm14, %ymm15, %ymm15
	vmulpd			%ymm14, %ymm1, %ymm1
	vmulpd			%ymm14, %ymm2, %ymm2
	vmovapd			96(%r13), %ymm0
	vblendpd		$ 0x7, %ymm15, %ymm0, %ymm0
	vmovapd			%ymm0, 96(%r13)

	movq	ARG2, %r11 // D
	//
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vaddpd			%ymm1, %ymm9, %ymm9
	vaddpd			%ymm2, %ymm10, %ymm10
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
	//
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	24(%r11), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11, %r14, 1), %ymm9
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	56(%r11), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmulpd			%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11, %r14, 1), %ymm9
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	88(%r11), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	120(%r11), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	//
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	24(%r11), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11, %r14, 1), %ymm9
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	56(%r11), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11, %r14, 1), %ymm9
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	88(%r11), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	120(%r11), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	24(%r11), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
//	vpermpd	$ 0x00, %ymm15, %ymm15  // beta

	// fifth column
102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 32(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	addq	$ 128, %r11
	vmovsd			0(%r11, %r14, 1), %xmm14 // alpha
	vfmadd231sd		%xmm14, %xmm14, %xmm15 // beta
	vsqrtsd			%xmm15, %xmm15, %xmm15 // beta
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm13 // mask
#else
	vmovsd			LC00(%rip), %xmm13 // mask
#endif
	vandpd			%xmm13, %xmm14, %xmm12
	vxorpd			%xmm13, %xmm12, %xmm12
	vxorpd			%xmm12, %xmm15, %xmm15 // beta
	vmovsd			%xmm15, 0(%r11, %r14, 1) // pD[0+ps*0]
	vsubsd			%xmm14, %xmm15, %xmm14 // beta-alpha
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC01(%rip), %xmm12
#else
	vmovapd			LC01(%rip), %xmm12
#endif
	vmovsd			%xmm14, %xmm12, %xmm12
	vmovddup		%xmm14, %xmm14
	vmovsd			%xmm15, %xmm14, %xmm14
	vdivpd			%xmm14, %xmm12, %xmm14
	vmovsd			%xmm14, 32(%r12) // dD[0]
	vxorpd			%xmm13, %xmm14, %xmm12
	vmovsd			%xmm12, 128(%r13, %r15, 1) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			0(%r11), %ymm0
	vmovapd			0(%r11, %r14, 1), %ymm1
	vmovapd			0(%r11, %r14, 2), %ymm2
	vbroadcastsd	32(%r11, %r14, 1), %ymm8
	vbroadcastsd	64(%r11, %r14, 1), %ymm9
	vbroadcastsd	96(%r11, %r14, 1), %ymm10
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vfmadd231pd		32(%r11), %ymm8, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		32(%r11, %r14, 2), %ymm8, %ymm2
	vfmadd231pd		64(%r11), %ymm9, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		96(%r11), %ymm10, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm10, %ymm2
	vmovsd			%xmm8, 32(%r11, %r14, 1)
	vmovsd			%xmm9, 64(%r11, %r14, 1)
	vmovsd			%xmm10, 96(%r11, %r14, 1)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	0(%r11, %r14, 1), %ymm8
	vbroadcastsd	32(%r11, %r14, 1), %ymm9
	vbroadcastsd	64(%r11, %r14, 1), %ymm10
	vbroadcastsd	96(%r11, %r14, 1), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		32(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm10, %ymm2
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm11, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm11, %ymm2
	vmovsd			%xmm8, 0(%r11, %r14, 1)
	vmovsd			%xmm9, 32(%r11, %r14, 1)
	vmovsd			%xmm10, 64(%r11, %r14, 1)
	vmovsd			%xmm11, 96(%r11, %r14, 1)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	0(%r11, %r14, 1), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vmovsd			%xmm8, 0(%r11, %r14, 1)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vxorpd			%xmm12, %xmm12, %xmm12
	//
	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm15
	//
	vmovapd			32(%r13), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vpermpd			$ 0x55, %ymm0, %ymm13
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vmovapd			64(%r13), %ymm14
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm14
	vpermpd			$ 0xaa, %ymm0, %ymm13
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vmovapd			96(%r13), %ymm14
	vpermpd			$ 0xff, %ymm0, %ymm13
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vbroadcastsd	128(%r13, %r15, 1), %ymm14
	vmulpd			%ymm14, %ymm15, %ymm15
	vmulpd			%ymm14, %ymm1, %ymm1
	vmulpd			%ymm14, %ymm2, %ymm2
//	vmovapd			128(%r13), %ymm0
//	vblendpd		$ 0xf, %ymm15, %ymm0, %ymm15
	vmovapd			%ymm15, 128(%r13)

	vxorpd			%ymm15, %ymm15, %ymm15
	vblendpd		$ 0x1, %ymm15, %ymm1, %ymm1

	movq	ARG2, %r11 // D
	//
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
	//
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vaddpd			%ymm1, %ymm9, %ymm9
	vaddpd			%ymm2, %ymm10, %ymm10
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11, %r14, 1), %ymm9
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	32(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11, %r14, 1), %ymm9
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	64(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmulpd			%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	96(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	//
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	0(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11, %r14, 1), %ymm9
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	32(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11, %r14, 1), %ymm9
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	64(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	96(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	//
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	0(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
	vpermpd	$ 0x55, %ymm15, %ymm15  // beta

	// sixth column
102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 40(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	addq	$ 128, %r11
	vmovsd			40(%r11, %r14, 1), %xmm14 // alpha
	vfmadd231sd		%xmm14, %xmm14, %xmm15 // beta
	vsqrtsd			%xmm15, %xmm15, %xmm15 // beta
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm13 // mask
#else
	vmovsd			LC00(%rip), %xmm13 // mask
#endif
	vandpd			%xmm13, %xmm14, %xmm12
	vxorpd			%xmm13, %xmm12, %xmm12
	vxorpd			%xmm12, %xmm15, %xmm15 // beta
	vmovsd			%xmm15, 40(%r11, %r14, 1) // pD[0+ps*0]
	vsubsd			%xmm14, %xmm15, %xmm14 // beta-alpha
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC01(%rip), %xmm12
#else
	vmovapd			LC01(%rip), %xmm12
#endif
	vmovsd			%xmm14, %xmm12, %xmm12
	vmovddup		%xmm14, %xmm14
	vmovsd			%xmm15, %xmm14, %xmm14
	vdivpd			%xmm14, %xmm12, %xmm14
	vmovsd			%xmm14, 40(%r12) // dD[0]
	vxorpd			%xmm13, %xmm14, %xmm12
	vmovsd			%xmm12, 168(%r13, %r15, 1) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			32(%r11), %ymm0
	vmovapd			32(%r11, %r14, 1), %ymm1
	vmovapd			32(%r11, %r14, 2), %ymm2
	vbroadcastsd	72(%r11, %r14, 1), %ymm9
	vbroadcastsd	104(%r11, %r14, 1), %ymm10
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vfmadd231pd		64(%r11), %ymm9, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		96(%r11), %ymm10, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm10, %ymm2
	vmovsd			%xmm9, 72(%r11, %r14, 1)
	vmovsd			%xmm10, 104(%r11, %r14, 1)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	8(%r11, %r14, 1), %ymm8
	vbroadcastsd	40(%r11, %r14, 1), %ymm9
	vbroadcastsd	72(%r11, %r14, 1), %ymm10
	vbroadcastsd	104(%r11, %r14, 1), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		32(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm10, %ymm2
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm11, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm11, %ymm2
	vmovsd			%xmm8, 8(%r11, %r14, 1)
	vmovsd			%xmm9, 40(%r11, %r14, 1)
	vmovsd			%xmm10, 72(%r11, %r14, 1)
	vmovsd			%xmm11, 104(%r11, %r14, 1)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	8(%r11, %r14, 1), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vmovsd			%xmm8, 8(%r11, %r14, 1)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vxorpd			%xmm12, %xmm12, %xmm12
	//
	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm15
	//
	vmovapd			32(%r13), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vpermpd			$ 0x55, %ymm0, %ymm13
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vmovapd			64(%r13), %ymm14
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm14
	vpermpd			$ 0xaa, %ymm0, %ymm13
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vmovapd			96(%r13), %ymm14
	vpermpd			$ 0xff, %ymm0, %ymm13
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vmovapd			128(%r13), %ymm14
	vmovapd			128(%r13, %r15, 1), %ymm11
	vblendpd		$ 0x1, %ymm11, %ymm12, %ymm11
	vpermpd			$ 0x00, %ymm1, %ymm13 // vv
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmulpd			%ymm11, %ymm13, %ymm11
	//
	vbroadcastsd	168(%r13, %r15, 1), %ymm14
	vmulpd			%ymm14, %ymm15, %ymm15
	vmulpd			%ymm14, %ymm11, %ymm11
	vmulpd			%ymm14, %ymm1, %ymm1
	vmulpd			%ymm14, %ymm2, %ymm2
	vmovapd			160(%r13, %r15, 1), %ymm0
	vblendpd		$ 0x1, %ymm11, %ymm0, %ymm11
	vmovapd			%ymm15, 160(%r13)
	vmovapd			%ymm11, 160(%r13, %r15, 1)

	vxorpd			%ymm15, %ymm15, %ymm15
	vblendpd		$ 0x3, %ymm15, %ymm1, %ymm1

	movq	ARG2, %r11 // D
	//
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
	//
	vmovapd			32(%r11, %r14, 1), %ymm9
	vmovapd			32(%r11, %r14, 2), %ymm10
	vaddpd			%ymm1, %ymm9, %ymm9
	vaddpd			%ymm2, %ymm10, %ymm10
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11, %r14, 1), %ymm9
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	72(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	104(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmulpd			%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	//
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	8(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11, %r14, 1), %ymm9
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	40(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11, %r14, 1), %ymm9
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	72(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	104(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	//
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	8(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
	vpermpd	$ 0xaa, %ymm15, %ymm15  // beta

	// seventh column
102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 40(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	addq	$ 128, %r11
	vmovsd			80(%r11, %r14, 1), %xmm14 // alpha
	vfmadd231sd		%xmm14, %xmm14, %xmm15 // beta
	vsqrtsd			%xmm15, %xmm15, %xmm15 // beta
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm13 // mask
#else
	vmovsd			LC00(%rip), %xmm13 // mask
#endif
	vandpd			%xmm13, %xmm14, %xmm12
	vxorpd			%xmm13, %xmm12, %xmm12
	vxorpd			%xmm12, %xmm15, %xmm15 // beta
	vmovsd			%xmm15, 80(%r11, %r14, 1) // pD[0+ps*0]
	vsubsd			%xmm14, %xmm15, %xmm14 // beta-alpha
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC01(%rip), %xmm12
#else
	vmovapd			LC01(%rip), %xmm12
#endif
	vmovsd			%xmm14, %xmm12, %xmm12
	vmovddup		%xmm14, %xmm14
	vmovsd			%xmm15, %xmm14, %xmm14
	vdivpd			%xmm14, %xmm12, %xmm14
	vmovsd			%xmm14, 48(%r12) // dD[0]
	vxorpd			%xmm13, %xmm14, %xmm12
	vmovsd			%xmm12, 208(%r13, %r15, 1) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			64(%r11), %ymm0
	vmovapd			64(%r11, %r14, 1), %ymm1
	vmovapd			64(%r11, %r14, 2), %ymm2
	vbroadcastsd	112(%r11, %r14, 1), %ymm10
	vmulpd			%ymm15, %ymm10, %ymm10
	vfmadd231pd		96(%r11), %ymm10, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm10, %ymm2
	vmovsd			%xmm10, 112(%r11, %r14, 1)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	16(%r11, %r14, 1), %ymm8
	vbroadcastsd	48(%r11, %r14, 1), %ymm9
	vbroadcastsd	80(%r11, %r14, 1), %ymm10
	vbroadcastsd	112(%r11, %r14, 1), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		32(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm10, %ymm2
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm11, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm11, %ymm2
	vmovsd			%xmm8, 16(%r11, %r14, 1)
	vmovsd			%xmm9, 48(%r11, %r14, 1)
	vmovsd			%xmm10, 80(%r11, %r14, 1)
	vmovsd			%xmm11, 112(%r11, %r14, 1)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	16(%r11, %r14, 1), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vmovsd			%xmm8, 16(%r11, %r14, 1)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vxorpd			%xmm12, %xmm12, %xmm12
	//
//	vpermpd			$ 0x00, %ymm0, %ymm13
	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm15
	//
	vpermpd			$ 0x55, %ymm0, %ymm13
	vmovapd			32(%r13), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vpermpd			$ 0xaa, %ymm0, %ymm13
	vmovapd			64(%r13), %ymm14
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vpermpd			$ 0xff, %ymm0, %ymm13
	vmovapd			96(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vpermpd			$ 0x00, %ymm1, %ymm13 // vv
	vmovapd			128(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			128(%r13, %r15, 1), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm13, %ymm11
	//
	vpermpd			$ 0x55, %ymm1, %ymm13 // vv
	vmovapd			160(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			160(%r13, %r15, 1), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	//
	vbroadcastsd	208(%r13, %r15, 1), %ymm14
	vmulpd			%ymm14, %ymm15, %ymm15
	vmulpd			%ymm14, %ymm11, %ymm11
	vmulpd			%ymm14, %ymm1, %ymm1
	vmulpd			%ymm14, %ymm2, %ymm2
	vmovapd			192(%r13, %r15, 1), %ymm0
	vblendpd		$ 0x3, %ymm11, %ymm0, %ymm11
	vmovapd			%ymm15, 192(%r13)
	vmovapd			%ymm11, 192(%r13, %r15, 1)

	vxorpd			%ymm15, %ymm15, %ymm15
	vblendpd		$ 0x7, %ymm15, %ymm1, %ymm1

	movq	ARG2, %r11 // D
	//
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
	//
	vmovapd			64(%r11, %r14, 1), %ymm9
	vmovapd			64(%r11, %r14, 2), %ymm10
	vaddpd			%ymm1, %ymm9, %ymm9
	vaddpd			%ymm2, %ymm10, %ymm10
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	112(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vxorpd			%ymm15, %ymm15, %ymm15
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	//
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	16(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11, %r14, 1), %ymm9
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	48(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11, %r14, 1), %ymm9
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	80(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	112(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	//
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	16(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
	vpermpd	$ 0xff, %ymm15, %ymm15  // beta

	// eight column
102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 40(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	addq	$ 128, %r11
	vmovsd			120(%r11, %r14, 1), %xmm14 // alpha
	vfmadd231sd		%xmm14, %xmm14, %xmm15 // beta
	vsqrtsd			%xmm15, %xmm15, %xmm15 // beta
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm13 // mask
#else
	vmovsd			LC00(%rip), %xmm13 // mask
#endif
	vandpd			%xmm13, %xmm14, %xmm12
	vxorpd			%xmm13, %xmm12, %xmm12
	vxorpd			%xmm12, %xmm15, %xmm15 // beta
	vmovsd			%xmm15, 120(%r11, %r14, 1) // pD[0+ps*0]
	vsubsd			%xmm14, %xmm15, %xmm14 // beta-alpha
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC01(%rip), %xmm12
#else
	vmovapd			LC01(%rip), %xmm12
#endif
	vmovsd			%xmm14, %xmm12, %xmm12
	vmovddup		%xmm14, %xmm14
	vmovsd			%xmm15, %xmm14, %xmm14
	vdivpd			%xmm14, %xmm12, %xmm14
	vmovsd			%xmm14, 56(%r12) // dD[0]
	vxorpd			%xmm13, %xmm14, %xmm12
	vmovsd			%xmm12, 248(%r13, %r15, 1) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			96(%r11), %ymm0
	vmovapd			96(%r11, %r14, 1), %ymm1
	vmovapd			96(%r11, %r14, 2), %ymm2
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	24(%r11, %r14, 1), %ymm8
	vbroadcastsd	56(%r11, %r14, 1), %ymm9
	vbroadcastsd	88(%r11, %r14, 1), %ymm10
	vbroadcastsd	120(%r11, %r14, 1), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		32(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm10, %ymm2
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm11, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm11, %ymm2
	vmovsd			%xmm8, 24(%r11, %r14, 1)
	vmovsd			%xmm9, 56(%r11, %r14, 1)
	vmovsd			%xmm10, 88(%r11, %r14, 1)
	vmovsd			%xmm11, 120(%r11, %r14, 1)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	24(%r11, %r14, 1), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vmovsd			%xmm8, 24(%r11, %r14, 1)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vxorpd			%xmm12, %xmm12, %xmm12
	//
//	vpermpd			$ 0x00, %ymm0, %ymm13
	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm15
	//
	vpermpd			$ 0x55, %ymm0, %ymm13
	vmovapd			32(%r13), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vpermpd			$ 0xaa, %ymm0, %ymm13
	vmovapd			64(%r13), %ymm14
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vpermpd			$ 0xff, %ymm0, %ymm13
	vmovapd			96(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vpermpd			$ 0x00, %ymm1, %ymm13
	vmovapd			128(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			128(%r13, %r15, 1), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm13, %ymm11
	//
	vpermpd			$ 0x55, %ymm1, %ymm13
	vmovapd			160(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			160(%r13, %r15, 1), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	//
	vpermpd			$ 0xaa, %ymm1, %ymm13
	vmovapd			192(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			192(%r13, %r15, 1), %ymm14
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	//
	vbroadcastsd	248(%r13, %r15, 1), %ymm14
	vmulpd			%ymm14, %ymm15, %ymm15
	vmulpd			%ymm14, %ymm11, %ymm11
//	vmulpd			%ymm14, %ymm1, %ymm1
	vmulpd			%ymm14, %ymm2, %ymm2
	vmovapd			224(%r13, %r15, 1), %ymm0
	vblendpd		$ 0x7, %ymm11, %ymm0, %ymm11
	vmovapd			%ymm15, 224(%r13)
	vmovapd			%ymm11, 224(%r13, %r15, 1)

//	vxorpd			%ymm15, %ymm15, %ymm15
//	vblendpd		$ 0xf, %ymm15, %ymm1, %ymm1

	movq	ARG2, %r11 // D
	//
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
	//
	vmovapd			96(%r11, %r14, 2), %ymm10
	vaddpd			%ymm2, %ymm10, %ymm10
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
	//
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	24(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	56(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmulpd			%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	88(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	120(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	//
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	24(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	56(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	88(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	120(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	//
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	24(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
//	vpermpd	$ 0x00, %ymm15, %ymm15  // beta

	// ninth column
102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 40(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	addq	$ 256, %r11
	vmovsd			0(%r11, %r14, 2), %xmm14 // alpha
	vfmadd231sd		%xmm14, %xmm14, %xmm15 // beta
	vsqrtsd			%xmm15, %xmm15, %xmm15 // beta
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm13 // mask
#else
	vmovsd			LC00(%rip), %xmm13 // mask
#endif
	vandpd			%xmm13, %xmm14, %xmm12
	vxorpd			%xmm13, %xmm12, %xmm12
	vxorpd			%xmm12, %xmm15, %xmm15 // beta
	vmovsd			%xmm15, 0(%r11, %r14, 2) // pD[0+ps*0]
	vsubsd			%xmm14, %xmm15, %xmm14 // beta-alpha
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC01(%rip), %xmm12
#else
	vmovapd			LC01(%rip), %xmm12
#endif
	vmovsd			%xmm14, %xmm12, %xmm12
	vmovddup		%xmm14, %xmm14
	vmovsd			%xmm15, %xmm14, %xmm14
	vdivpd			%xmm14, %xmm12, %xmm14
	vmovsd			%xmm14, 64(%r12) // dD[0]
	vxorpd			%xmm13, %xmm14, %xmm12
	vmovsd			%xmm12, 256(%r13, %r15, 2) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			0(%r11), %ymm0
	vmovapd			0(%r11, %r14, 1), %ymm1
	vmovapd			0(%r11, %r14, 2), %ymm2
	vbroadcastsd	32(%r11, %r14, 2), %ymm8
	vbroadcastsd	64(%r11, %r14, 2), %ymm9
	vbroadcastsd	96(%r11, %r14, 2), %ymm10
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vfmadd231pd		32(%r11), %ymm8, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		32(%r11, %r14, 2), %ymm8, %ymm2
	vfmadd231pd		64(%r11), %ymm9, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		96(%r11), %ymm10, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm10, %ymm2
	vmovsd			%xmm8, 32(%r11, %r14, 2)
	vmovsd			%xmm9, 64(%r11, %r14, 2)
	vmovsd			%xmm10, 96(%r11, %r14, 2)
	movq	ARG1, %r10 // n
	subl	$ 12, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	0(%r11, %r14, 2), %ymm8
	vbroadcastsd	32(%r11, %r14, 2), %ymm9
	vbroadcastsd	64(%r11, %r14, 2), %ymm10
	vbroadcastsd	96(%r11, %r14, 2), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		32(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm10, %ymm2
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm11, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm11, %ymm2
	vmovsd			%xmm8, 0(%r11, %r14, 2)
	vmovsd			%xmm9, 32(%r11, %r14, 2)
	vmovsd			%xmm10, 64(%r11, %r14, 2)
	vmovsd			%xmm11, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	0(%r11, %r14, 2), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vmovsd			%xmm8, 0(%r11, %r14, 2)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vxorpd			%xmm12, %xmm12, %xmm12
	//
//	vpermpd			$ 0x00, %ymm0, %ymm13
	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm15
	//
	vpermpd			$ 0x55, %ymm0, %ymm13
	vmovapd			32(%r13), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vpermpd			$ 0xaa, %ymm0, %ymm13
	vmovapd			64(%r13), %ymm14
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vpermpd			$ 0xff, %ymm0, %ymm13
	vmovapd			96(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vpermpd			$ 0x00, %ymm1, %ymm13
	vmovapd			128(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			128(%r13, %r15, 1), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm13, %ymm11
	//
	vpermpd			$ 0x55, %ymm1, %ymm13
	vmovapd			160(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			160(%r13, %r15, 1), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	//
	vpermpd			$ 0xaa, %ymm1, %ymm13
	vmovapd			192(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			192(%r13, %r15, 1), %ymm14
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	//
	vpermpd			$ 0xff, %ymm1, %ymm13
	vmovapd			224(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			224(%r13, %r15, 1), %ymm14
//	vblendpd		$ 0xf, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	//
	vbroadcastsd	256(%r13, %r15, 2), %ymm14
	vmulpd			%ymm14, %ymm15, %ymm15
	vmulpd			%ymm14, %ymm11, %ymm11
//	vmulpd			%ymm14, %ymm1, %ymm1
	vmulpd			%ymm14, %ymm2, %ymm2
//	vmovapd			224(%r13, %r15, 1), %ymm0
//	vblendpd		$ 0xf, %ymm11, %ymm0, %ymm11
	vmovapd			%ymm15, 256(%r13)
	vmovapd			%ymm11, 256(%r13, %r15, 1)

	vxorpd			%ymm15, %ymm15, %ymm15
	vblendpd		$ 0x1, %ymm15, %ymm2, %ymm2

	movq	ARG2, %r11 // D
	//
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
	//
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
	//
	vmovapd			0(%r11, %r14, 2), %ymm10
	vaddpd			%ymm2, %ymm10, %ymm10
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	32(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	64(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmulpd			%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	96(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	//
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	0(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	32(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	64(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	96(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	//
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	0(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
	vpermpd	$ 0x55, %ymm15, %ymm15  // beta

	// tenth column
102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 40(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	addq	$ 256, %r11
	vmovsd			40(%r11, %r14, 2), %xmm14 // alpha
	vfmadd231sd		%xmm14, %xmm14, %xmm15 // beta
	vsqrtsd			%xmm15, %xmm15, %xmm15 // beta
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm13 // mask
#else
	vmovsd			LC00(%rip), %xmm13 // mask
#endif
	vandpd			%xmm13, %xmm14, %xmm12
	vxorpd			%xmm13, %xmm12, %xmm12
	vxorpd			%xmm12, %xmm15, %xmm15 // beta
	vmovsd			%xmm15, 40(%r11, %r14, 2) // pD[0+ps*0]
	vsubsd			%xmm14, %xmm15, %xmm14 // beta-alpha
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC01(%rip), %xmm12
#else
	vmovapd			LC01(%rip), %xmm12
#endif
	vmovsd			%xmm14, %xmm12, %xmm12
	vmovddup		%xmm14, %xmm14
	vmovsd			%xmm15, %xmm14, %xmm14
	vdivpd			%xmm14, %xmm12, %xmm14
	vmovsd			%xmm14, 72(%r12) // dD[0]
	vxorpd			%xmm13, %xmm14, %xmm12
	vmovsd			%xmm12, 296(%r13, %r15, 2) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			32(%r11), %ymm0
	vmovapd			32(%r11, %r14, 1), %ymm1
	vmovapd			32(%r11, %r14, 2), %ymm2
	vbroadcastsd	72(%r11, %r14, 2), %ymm9
	vbroadcastsd	104(%r11, %r14, 2), %ymm10
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vfmadd231pd		64(%r11), %ymm9, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		96(%r11), %ymm10, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm10, %ymm2
	vmovsd			%xmm9, 72(%r11, %r14, 2)
	vmovsd			%xmm10, 104(%r11, %r14, 2)
	movq	ARG1, %r10 // n
	subl	$ 12, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	8(%r11, %r14, 2), %ymm8
	vbroadcastsd	40(%r11, %r14, 2), %ymm9
	vbroadcastsd	72(%r11, %r14, 2), %ymm10
	vbroadcastsd	104(%r11, %r14, 2), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		32(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm10, %ymm2
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm11, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm11, %ymm2
	vmovsd			%xmm8, 8(%r11, %r14, 2)
	vmovsd			%xmm9, 40(%r11, %r14, 2)
	vmovsd			%xmm10, 72(%r11, %r14, 2)
	vmovsd			%xmm11, 104(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	8(%r11, %r14, 2), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vmovsd			%xmm8, 8(%r11, %r14, 2)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vxorpd			%xmm12, %xmm12, %xmm12
	//
//	vpermpd			$ 0x00, %ymm0, %ymm13
	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm15
	//
	vpermpd			$ 0x55, %ymm0, %ymm13
	vmovapd			32(%r13), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vpermpd			$ 0xaa, %ymm0, %ymm13
	vmovapd			64(%r13), %ymm14
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vpermpd			$ 0xff, %ymm0, %ymm13
	vmovapd			96(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vpermpd			$ 0x00, %ymm1, %ymm13
	vmovapd			128(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			128(%r13, %r15, 1), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm13, %ymm11
	//
	vpermpd			$ 0x55, %ymm1, %ymm13
	vmovapd			160(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			160(%r13, %r15, 1), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	//
	vpermpd			$ 0xaa, %ymm1, %ymm13
	vmovapd			192(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			192(%r13, %r15, 1), %ymm14
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	//
	vpermpd			$ 0xff, %ymm1, %ymm13
	vmovapd			224(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			224(%r13, %r15, 1), %ymm14
//	vblendpd		$ 0xf, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	//
	vpermpd			$ 0x00, %ymm2, %ymm13
	vmovapd			256(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			256(%r13, %r15, 1), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	vmovapd			256(%r13, %r15, 2), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm13, %ymm10
	//
	vbroadcastsd	296(%r13, %r15, 2), %ymm14
	vmulpd			%ymm14, %ymm15, %ymm15
	vmulpd			%ymm14, %ymm11, %ymm11
	vmulpd			%ymm14, %ymm10, %ymm10
	vmulpd			%ymm14, %ymm2, %ymm2
	vmovapd			288(%r13, %r15, 2), %ymm0
	vblendpd		$ 0x1, %ymm10, %ymm0, %ymm10
	vmovapd			%ymm15, 288(%r13)
	vmovapd			%ymm11, 288(%r13, %r15, 1)
	vmovapd			%ymm10, 288(%r13, %r15, 2)

	vxorpd			%ymm15, %ymm15, %ymm15
	vblendpd		$ 0x3, %ymm15, %ymm2, %ymm2

	movq	ARG2, %r11 // D
	//
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
	//
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
	//
	vmovapd			32(%r11, %r14, 2), %ymm10
	vaddpd			%ymm2, %ymm10, %ymm10
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	72(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	104(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmulpd			%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	//
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	8(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	40(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	72(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	104(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	//
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	8(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
	vpermpd	$ 0xaa, %ymm15, %ymm15  // beta

	// eleventh column
102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 40(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	addq	$ 256, %r11
	vmovsd			80(%r11, %r14, 2), %xmm14 // alpha
	vfmadd231sd		%xmm14, %xmm14, %xmm15 // beta
	vsqrtsd			%xmm15, %xmm15, %xmm15 // beta
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm13 // mask
#else
	vmovsd			LC00(%rip), %xmm13 // mask
#endif
	vandpd			%xmm13, %xmm14, %xmm12
	vxorpd			%xmm13, %xmm12, %xmm12
	vxorpd			%xmm12, %xmm15, %xmm15 // beta
	vmovsd			%xmm15, 80(%r11, %r14, 2) // pD[0+ps*0]
	vsubsd			%xmm14, %xmm15, %xmm14 // beta-alpha
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC01(%rip), %xmm12
#else
	vmovapd			LC01(%rip), %xmm12
#endif
	vmovsd			%xmm14, %xmm12, %xmm12
	vmovddup		%xmm14, %xmm14
	vmovsd			%xmm15, %xmm14, %xmm14
	vdivpd			%xmm14, %xmm12, %xmm14
	vmovsd			%xmm14, 80(%r12) // dD[0]
	vxorpd			%xmm13, %xmm14, %xmm12
	vmovsd			%xmm12, 336(%r13, %r15, 2) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			64(%r11), %ymm0
	vmovapd			64(%r11, %r14, 1), %ymm1
	vmovapd			64(%r11, %r14, 2), %ymm2
	vbroadcastsd	112(%r11, %r14, 2), %ymm10
	vmulpd			%ymm15, %ymm10, %ymm10
	vfmadd231pd		96(%r11), %ymm10, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm10, %ymm2
	vmovsd			%xmm10, 112(%r11, %r14, 2)
	movq	ARG1, %r10 // n
	subl	$ 12, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	16(%r11, %r14, 2), %ymm8
	vbroadcastsd	48(%r11, %r14, 2), %ymm9
	vbroadcastsd	80(%r11, %r14, 2), %ymm10
	vbroadcastsd	112(%r11, %r14, 2), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		32(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm10, %ymm2
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm11, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm11, %ymm2
	vmovsd			%xmm8, 16(%r11, %r14, 2)
	vmovsd			%xmm9, 48(%r11, %r14, 2)
	vmovsd			%xmm10, 80(%r11, %r14, 2)
	vmovsd			%xmm11, 112(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	16(%r11, %r14, 2), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vmovsd			%xmm8, 16(%r11, %r14, 2)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vxorpd			%xmm12, %xmm12, %xmm12
	//
//	vpermpd			$ 0x00, %ymm0, %ymm13
	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm15
	//
	vpermpd			$ 0x55, %ymm0, %ymm13
	vmovapd			32(%r13), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vpermpd			$ 0xaa, %ymm0, %ymm13
	vmovapd			64(%r13), %ymm14
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vpermpd			$ 0xff, %ymm0, %ymm13
	vmovapd			96(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vpermpd			$ 0x00, %ymm1, %ymm13
	vmovapd			128(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			128(%r13, %r15, 1), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm13, %ymm11
	//
	vpermpd			$ 0x55, %ymm1, %ymm13
	vmovapd			160(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			160(%r13, %r15, 1), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	//
	vpermpd			$ 0xaa, %ymm1, %ymm13
	vmovapd			192(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			192(%r13, %r15, 1), %ymm14
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	//
	vpermpd			$ 0xff, %ymm1, %ymm13
	vmovapd			224(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			224(%r13, %r15, 1), %ymm14
//	vblendpd		$ 0xf, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	//
	vpermpd			$ 0x00, %ymm2, %ymm13
	vmovapd			256(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			256(%r13, %r15, 1), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	vmovapd			256(%r13, %r15, 2), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm13, %ymm10
	//
	vpermpd			$ 0x55, %ymm2, %ymm13
	vmovapd			288(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			288(%r13, %r15, 1), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	vmovapd			288(%r13, %r15, 2), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm10
	//
	vbroadcastsd	336(%r13, %r15, 2), %ymm14
	vmulpd			%ymm14, %ymm15, %ymm15
	vmulpd			%ymm14, %ymm11, %ymm11
	vmulpd			%ymm14, %ymm10, %ymm10
	vmulpd			%ymm14, %ymm2, %ymm2
	vmovapd			320(%r13, %r15, 2), %ymm0
	vblendpd		$ 0x3, %ymm10, %ymm0, %ymm10
	vmovapd			%ymm15, 320(%r13)
	vmovapd			%ymm11, 320(%r13, %r15, 1)
	vmovapd			%ymm10, 320(%r13, %r15, 2)

	vxorpd			%ymm15, %ymm15, %ymm15
	vblendpd		$ 0x7, %ymm15, %ymm2, %ymm2

	movq	ARG2, %r11 // D
	//
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
	//
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
	//
	vmovapd			64(%r11, %r14, 2), %ymm10
	vaddpd			%ymm2, %ymm10, %ymm10
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	112(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vxorpd			%ymm15, %ymm15, %ymm15
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	//
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	16(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	48(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	80(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	112(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	//
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	16(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
	vpermpd	$ 0xff, %ymm15, %ymm15  // beta

	// twelveth
102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 40(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	addq	$ 256, %r11
	vmovsd			120(%r11, %r14, 2), %xmm14 // alpha
	vfmadd231sd		%xmm14, %xmm14, %xmm15 // beta
	vsqrtsd			%xmm15, %xmm15, %xmm15 // beta
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm13 // mask
#else
	vmovsd			LC00(%rip), %xmm13 // mask
#endif
	vandpd			%xmm13, %xmm14, %xmm12
	vxorpd			%xmm13, %xmm12, %xmm12
	vxorpd			%xmm12, %xmm15, %xmm15 // beta
	vmovsd			%xmm15, 120(%r11, %r14, 2) // pD[0+ps*0]
	vsubsd			%xmm14, %xmm15, %xmm14 // beta-alpha
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC01(%rip), %xmm12
#else
	vmovapd			LC01(%rip), %xmm12
#endif
	vmovsd			%xmm14, %xmm12, %xmm12
	vmovddup		%xmm14, %xmm14
	vmovsd			%xmm15, %xmm14, %xmm14
	vdivpd			%xmm14, %xmm12, %xmm14
	vmovsd			%xmm14, 88(%r12) // dD[0]
	vxorpd			%xmm13, %xmm14, %xmm12
	vmovsd			%xmm12, 376(%r13, %r15, 2) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			96(%r11), %ymm0
	vmovapd			96(%r11, %r14, 1), %ymm1
	vmovapd			96(%r11, %r14, 2), %ymm2
	movq	ARG1, %r10 // n
	subl	$ 12, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	24(%r11, %r14, 2), %ymm8
	vbroadcastsd	56(%r11, %r14, 2), %ymm9
	vbroadcastsd	88(%r11, %r14, 2), %ymm10
	vbroadcastsd	120(%r11, %r14, 2), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		32(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm10, %ymm2
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm11, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm11, %ymm2
	vmovsd			%xmm8, 24(%r11, %r14, 2)
	vmovsd			%xmm9, 56(%r11, %r14, 2)
	vmovsd			%xmm10, 88(%r11, %r14, 2)
	vmovsd			%xmm11, 120(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	24(%r11, %r14, 2), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vmovsd			%xmm8, 24(%r11, %r14, 2)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vxorpd			%xmm12, %xmm12, %xmm12
	//
//	vpermpd			$ 0x00, %ymm0, %ymm13
	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm15
	//
	vpermpd			$ 0x55, %ymm0, %ymm13
	vmovapd			32(%r13), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vpermpd			$ 0xaa, %ymm0, %ymm13
	vmovapd			64(%r13), %ymm14
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vpermpd			$ 0xff, %ymm0, %ymm13
	vmovapd			96(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vpermpd			$ 0x00, %ymm1, %ymm13
	vmovapd			128(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			128(%r13, %r15, 1), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm13, %ymm11
	//
	vpermpd			$ 0x55, %ymm1, %ymm13
	vmovapd			160(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			160(%r13, %r15, 1), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	//
	vpermpd			$ 0xaa, %ymm1, %ymm13
	vmovapd			192(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			192(%r13, %r15, 1), %ymm14
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	//
	vpermpd			$ 0xff, %ymm1, %ymm13
	vmovapd			224(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			224(%r13, %r15, 1), %ymm14
//	vblendpd		$ 0xf, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	//
	vpermpd			$ 0x00, %ymm2, %ymm13
	vmovapd			256(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			256(%r13, %r15, 1), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	vmovapd			256(%r13, %r15, 2), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm13, %ymm10
	//
	vpermpd			$ 0x55, %ymm2, %ymm13
	vmovapd			288(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			288(%r13, %r15, 1), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	vmovapd			288(%r13, %r15, 2), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm10
	//
	vpermpd			$ 0xaa, %ymm2, %ymm13
	vmovapd			320(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			320(%r13, %r15, 1), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	vmovapd			320(%r13, %r15, 2), %ymm14
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm10
	//
	vbroadcastsd	376(%r13, %r15, 2), %ymm14
	vmulpd			%ymm14, %ymm15, %ymm15
	vmulpd			%ymm14, %ymm11, %ymm11
	vmulpd			%ymm14, %ymm10, %ymm10
//	vmulpd			%ymm14, %ymm2, %ymm2
	vmovapd			352(%r13, %r15, 2), %ymm0
	vblendpd		$ 0x7, %ymm10, %ymm0, %ymm10
	vmovapd			%ymm15, 352(%r13)
	vmovapd			%ymm11, 352(%r13, %r15, 1)
	vmovapd			%ymm10, 352(%r13, %r15, 2)

102:

	EPILOGUE
	
	ret

#if defined(OS_LINUX)
	.size	kernel_dgelqf_dlarft12_12_lib4, .-kernel_dgelqf_dlarft12_12_lib4
#endif





//                                   1      2           3        4           5
// void kernel_dgelqf_dlarft4_12_lib4(int n, double *pD, int sdd, double *dD, double *pT)

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dgelqf_dlarft4_12_lib4
	.type kernel_dgelqf_dlarft4_12_lib4, @function
kernel_dgelqf_dlarft4_12_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dgelqf_dlarft4_12_lib4
_kernel_dgelqf_dlarft4_12_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dgelqf_dlarft4_12_lib4
	.def kernel_dgelqf_dlarft4_12_lib4; .scl 2; .type 32; .endef
kernel_dgelqf_dlarft4_12_lib4:
#endif
	
	PROLOGUE

	// zero T

	movq	ARG5, %r10 // T

	vxorpd			%ymm15, %ymm15, %ymm15
	vmovapd			%ymm15, 0(%r10)
	vmovapd			%ymm15, 32(%r10)
	vmovapd			%ymm15, 64(%r10)
	vmovapd			%ymm15, 96(%r10)

	// first column

	movq	ARG2, %r11 // D
	movq	ARG3, %r14 // sdd
	sall	$ 5, %r14d
	movq	ARG4, %r12 // dD
	movq	ARG5, %r13 // T

	vxorpd			%xmm15, %xmm15, %xmm15
	movq	ARG1, %r10 // n
	subl	$ 1, %r10d
	addq	$ 32, %r11
100:
	vmovsd			0(%r11), %xmm14
	vfmadd231sd		%xmm14, %xmm14, %xmm15
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		100b

	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		101f
	vmovsd			%xmm14, 0(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	vmovsd			0(%r11), %xmm14 // alpha
	vfmadd231sd		%xmm14, %xmm14, %xmm15 // beta
	vsqrtsd			%xmm15, %xmm15, %xmm15 // beta
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm13 // mask
#else
	vmovsd			LC00(%rip), %xmm13 // mask
#endif
	vandpd			%xmm13, %xmm14, %xmm12
	vxorpd			%xmm13, %xmm12, %xmm12
	vxorpd			%xmm12, %xmm15, %xmm15 // beta
	vmovsd			%xmm15, 0(%r11) // pD[0+ps*0]
	vsubsd			%xmm14, %xmm15, %xmm14 // beta-alpha
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC01(%rip), %xmm12
#else
	vmovapd			LC01(%rip), %xmm12
#endif
	vmovsd			%xmm14, %xmm12, %xmm12
	vmovddup		%xmm14, %xmm14
	vmovsd			%xmm15, %xmm14, %xmm14
	vdivpd			%xmm14, %xmm12, %xmm14
	vmovsd			%xmm14, 0(%r12) // dD[0]
	vxorpd			%xmm13, %xmm14, %xmm12
	vmovsd			%xmm12, 0(%r13) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			0(%r11), %ymm0
	vmovapd			0(%r11, %r14, 1), %ymm1
	vmovapd			0(%r11, %r14, 2), %ymm2
	vbroadcastsd	32(%r11), %ymm8
	vbroadcastsd	64(%r11), %ymm9
	vbroadcastsd	96(%r11), %ymm10
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vfmadd231pd		32(%r11), %ymm8, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		32(%r11, %r14, 2), %ymm8, %ymm2
	vfmadd231pd		64(%r11), %ymm9, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		96(%r11), %ymm10, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm10, %ymm2
	vmovsd			%xmm8, 32(%r11)
	vmovsd			%xmm9, 64(%r11)
	vmovsd			%xmm10, 96(%r11)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	0(%r11), %ymm8
	vbroadcastsd	32(%r11), %ymm9
	vbroadcastsd	64(%r11), %ymm10
	vbroadcastsd	96(%r11), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		32(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm10, %ymm2
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm11, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm11, %ymm2
	vmovsd			%xmm8, 0(%r11)
	vmovsd			%xmm9, 32(%r11)
	vmovsd			%xmm10, 64(%r11)
	vmovsd			%xmm11, 96(%r11)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	0(%r11), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vmovsd			%xmm8, 0(%r11)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vbroadcastsd	0(%r13), %ymm15
	vmulpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm15, %ymm2, %ymm2

	vxorpd			%ymm15, %ymm15, %ymm15
	vblendpd		$ 0x1, %ymm15, %ymm0, %ymm0

	movq	ARG2, %r11 // D
	//
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vaddpd			%ymm0, %ymm8, %ymm8
	vaddpd			%ymm1, %ymm9, %ymm9
	vaddpd			%ymm2, %ymm10, %ymm10
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11), %ymm8
	vmovapd			32(%r11, %r14, 1), %ymm9
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	32(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmovapd			%ymm8, 32(%r11)
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11), %ymm8
	vmovapd			64(%r11, %r14, 1), %ymm9
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	64(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmulpd			%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 64(%r11)
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11), %ymm8
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	96(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 96(%r11)
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	//
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	0(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11), %ymm8
	vmovapd			32(%r11, %r14, 1), %ymm9
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	32(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 32(%r11)
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11), %ymm8
	vmovapd			64(%r11, %r14, 1), %ymm9
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	64(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 64(%r11)
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11), %ymm8
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	96(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 96(%r11)
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	//
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	0(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
	vpermpd	$ 0x55, %ymm15, %ymm15  // beta

	// second column
102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 8(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	vmovsd			40(%r11), %xmm14 // alpha
	vfmadd231sd		%xmm14, %xmm14, %xmm15 // beta
	vsqrtsd			%xmm15, %xmm15, %xmm15 // beta
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm13 // mask
#else
	vmovsd			LC00(%rip), %xmm13 // mask
#endif
	vandpd			%xmm13, %xmm14, %xmm12
	vxorpd			%xmm13, %xmm12, %xmm12
	vxorpd			%xmm12, %xmm15, %xmm15 // beta
	vmovsd			%xmm15, 40(%r11) // pD[0+ps*0]
	vsubsd			%xmm14, %xmm15, %xmm14 // beta-alpha
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC01(%rip), %xmm12
#else
	vmovapd			LC01(%rip), %xmm12
#endif
	vmovsd			%xmm14, %xmm12, %xmm12
	vmovddup		%xmm14, %xmm14
	vmovsd			%xmm15, %xmm14, %xmm14
	vdivpd			%xmm14, %xmm12, %xmm14
	vmovsd			%xmm14, 8(%r12) // dD[0]
	vxorpd			%xmm13, %xmm14, %xmm12
	vmovsd			%xmm12, 40(%r13) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			32(%r11), %ymm0
	vmovapd			32(%r11, %r14, 1), %ymm1
	vmovapd			32(%r11, %r14, 2), %ymm2
	vbroadcastsd	72(%r11), %ymm9
	vbroadcastsd	104(%r11), %ymm10
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vfmadd231pd		64(%r11), %ymm9, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		96(%r11), %ymm10, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm10, %ymm2
	vmovsd			%xmm9, 72(%r11)
	vmovsd			%xmm10, 104(%r11)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	8(%r11), %ymm8
	vbroadcastsd	40(%r11), %ymm9
	vbroadcastsd	72(%r11), %ymm10
	vbroadcastsd	104(%r11), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		32(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm10, %ymm2
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm11, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm11, %ymm2
	vmovsd			%xmm8, 8(%r11)
	vmovsd			%xmm9, 40(%r11)
	vmovsd			%xmm10, 72(%r11)
	vmovsd			%xmm11, 104(%r11)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	8(%r11), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vmovsd			%xmm8, 8(%r11)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC02(%rip), %ymm12
#else
	vmovapd			LC02(%rip), %ymm12
#endif
	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm0
	vbroadcastsd	40(%r13), %ymm15
	vmulpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm15, %ymm2, %ymm2
	vmovsd			%xmm0, 32(%r13)

	vxorpd			%ymm12, %ymm12, %ymm12
	vblendpd		$ 0x3, %ymm12, %ymm0, %ymm0

	movq	ARG2, %r11 // D
	//
	vmovapd			32(%r11), %ymm8
	vmovapd			32(%r11, %r14, 1), %ymm9
	vmovapd			32(%r11, %r14, 2), %ymm10
	vaddpd			%ymm0, %ymm8, %ymm8
	vaddpd			%ymm1, %ymm9, %ymm9
	vaddpd			%ymm2, %ymm10, %ymm10
	vmovapd			%ymm8, 32(%r11)
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11), %ymm8
	vmovapd			64(%r11, %r14, 1), %ymm9
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	72(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmovapd			%ymm8, 64(%r11)
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11), %ymm8
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	104(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmulpd			%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 96(%r11)
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	//
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	8(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11), %ymm8
	vmovapd			32(%r11, %r14, 1), %ymm9
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	40(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 32(%r11)
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11), %ymm8
	vmovapd			64(%r11, %r14, 1), %ymm9
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	72(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 64(%r11)
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11), %ymm8
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	104(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 96(%r11)
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	8(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
	vpermpd	$ 0xaa, %ymm15, %ymm15  // beta

	// third column
102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 16(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	vmovsd			80(%r11), %xmm14 // alpha
	vfmadd231sd		%xmm14, %xmm14, %xmm15 // beta
	vsqrtsd			%xmm15, %xmm15, %xmm15 // beta
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm13 // mask
#else
	vmovsd			LC00(%rip), %xmm13 // mask
#endif
	vandpd			%xmm13, %xmm14, %xmm12
	vxorpd			%xmm13, %xmm12, %xmm12
	vxorpd			%xmm12, %xmm15, %xmm15 // beta
	vmovsd			%xmm15, 80(%r11) // pD[0+ps*0]
	vsubsd			%xmm14, %xmm15, %xmm14 // beta-alpha
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC01(%rip), %xmm12
#else
	vmovapd			LC01(%rip), %xmm12
#endif
	vmovsd			%xmm14, %xmm12, %xmm12
	vmovddup		%xmm14, %xmm14
	vmovsd			%xmm15, %xmm14, %xmm14
	vdivpd			%xmm14, %xmm12, %xmm14
	vmovsd			%xmm14, 16(%r12) // dD[0]
	vxorpd			%xmm13, %xmm14, %xmm12
	vmovsd			%xmm12, 80(%r13) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			64(%r11), %ymm0
	vmovapd			64(%r11, %r14, 1), %ymm1
	vmovapd			64(%r11, %r14, 2), %ymm2
	vbroadcastsd	112(%r11), %ymm10
	vmulpd			%ymm15, %ymm10, %ymm10
	vfmadd231pd		96(%r11), %ymm10, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm10, %ymm2
	vmovsd			%xmm10, 112(%r11)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	16(%r11), %ymm8
	vbroadcastsd	48(%r11), %ymm9
	vbroadcastsd	80(%r11), %ymm10
	vbroadcastsd	112(%r11), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		32(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm10, %ymm2
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm11, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm11, %ymm2
	vmovsd			%xmm8, 16(%r11)
	vmovsd			%xmm9, 48(%r11)
	vmovsd			%xmm10, 80(%r11)
	vmovsd			%xmm11, 112(%r11)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	16(%r11), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vmovsd			%xmm8, 16(%r11)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vxorpd			%xmm12, %xmm12, %xmm12
	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm15
	vmovapd			32(%r13), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vpermpd			$ 0x55, %ymm0, %ymm13
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vblendpd		$ 0x7, %ymm15, %ymm0, %ymm0
	vbroadcastsd	80(%r13), %ymm14
	vmulpd			%ymm14, %ymm0, %ymm0
	vmulpd			%ymm14, %ymm1, %ymm1
	vmulpd			%ymm14, %ymm2, %ymm2
	vmovapd			%xmm0, 64(%r13)

	vxorpd			%ymm12, %ymm12, %ymm12
	vblendpd		$ 0x7, %ymm12, %ymm0, %ymm0

	movq	ARG2, %r11 // D
	//
	vmovapd			64(%r11), %ymm8
	vmovapd			64(%r11, %r14, 1), %ymm9
	vmovapd			64(%r11, %r14, 2), %ymm10
	vaddpd			%ymm0, %ymm8, %ymm8
	vaddpd			%ymm1, %ymm9, %ymm9
	vaddpd			%ymm2, %ymm10, %ymm10
	vmovapd			%ymm8, 64(%r11)
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11), %ymm8
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	112(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vxorpd			%ymm15, %ymm15, %ymm15
	vmovapd			%ymm8, 96(%r11)
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	//
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	16(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11), %ymm8
	vmovapd			32(%r11, %r14, 1), %ymm9
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	48(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 32(%r11)
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11), %ymm8
	vmovapd			64(%r11, %r14, 1), %ymm9
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	80(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 64(%r11)
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11), %ymm8
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	112(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 96(%r11)
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	16(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
	vpermpd	$ 0xff, %ymm15, %ymm15  // beta

102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 24(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	vmovsd			120(%r11), %xmm14 // alpha
	vfmadd231sd		%xmm14, %xmm14, %xmm15 // beta
	vsqrtsd			%xmm15, %xmm15, %xmm15 // beta
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm13 // mask
#else
	vmovsd			LC00(%rip), %xmm13 // mask
#endif
	vandpd			%xmm13, %xmm14, %xmm12
	vxorpd			%xmm13, %xmm12, %xmm12
	vxorpd			%xmm12, %xmm15, %xmm15 // beta
	vmovsd			%xmm15, 120(%r11) // pD[0+ps*0]
	vsubsd			%xmm14, %xmm15, %xmm14 // beta-alpha
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC01(%rip), %xmm12
#else
	vmovapd			LC01(%rip), %xmm12
#endif
	vmovsd			%xmm14, %xmm12, %xmm12
	vmovddup		%xmm14, %xmm14
	vmovsd			%xmm15, %xmm14, %xmm14
	vdivpd			%xmm14, %xmm12, %xmm14
	vmovsd			%xmm14, 24(%r12) // dD[0]
	vxorpd			%xmm13, %xmm14, %xmm12
	vmovsd			%xmm12, 120(%r13) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			96(%r11), %ymm0
	vmovapd			96(%r11, %r14, 1), %ymm1
	vmovapd			96(%r11, %r14, 2), %ymm2
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	24(%r11), %ymm8
	vbroadcastsd	56(%r11), %ymm9
	vbroadcastsd	88(%r11), %ymm10
	vbroadcastsd	120(%r11), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		32(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm10, %ymm2
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm11, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm11, %ymm2
	vmovsd			%xmm8, 24(%r11)
	vmovsd			%xmm9, 56(%r11)
	vmovsd			%xmm10, 88(%r11)
	vmovsd			%xmm11, 120(%r11)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	24(%r11), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vmovsd			%xmm8, 24(%r11)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vxorpd			%xmm12, %xmm12, %xmm12

	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm15

	vmovapd			32(%r13), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vpermpd			$ 0x55, %ymm0, %ymm13
	vfmadd231pd		%ymm14, %ymm13, %ymm15

	vmovapd			64(%r13), %ymm14
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm14
	vpermpd			$ 0xaa, %ymm0, %ymm13
	vfmadd231pd		%ymm14, %ymm13, %ymm15

	vbroadcastsd	120(%r13), %ymm14
	vmulpd			%ymm14, %ymm15, %ymm15
	vmulpd			%ymm14, %ymm1, %ymm1
	vmulpd			%ymm14, %ymm2, %ymm2
	vmovapd			96(%r13), %ymm0
	vblendpd		$ 0x7, %ymm15, %ymm0, %ymm0
	vmovapd			%ymm0, 96(%r13)

	movq	ARG2, %r11 // D
	//
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vaddpd			%ymm1, %ymm9, %ymm9
	vaddpd			%ymm2, %ymm10, %ymm10
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	//
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	24(%r11), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11, %r14, 1), %ymm9
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	56(%r11), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11, %r14, 1), %ymm9
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	88(%r11), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	120(%r11), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	24(%r11), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:

102:

	EPILOGUE
	
	ret

#if defined(OS_LINUX)
	.size	kernel_dgelqf_dlarft4_12_lib4, .-kernel_dgelqf_dlarft4_12_lib4
#endif





//                                  1      2           3        4           5
// void kernel_dgelqf_dlarft4_8_lib4(int n, double *pD, int sdd, double *dD, double *pT)

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dgelqf_dlarft4_8_lib4
	.type kernel_dgelqf_dlarft4_8_lib4, @function
kernel_dgelqf_dlarft4_8_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dgelqf_dlarft4_8_lib4
_kernel_dgelqf_dlarft4_8_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dgelqf_dlarft4_8_lib4
	.def kernel_dgelqf_dlarft4_8_lib4; .scl 2; .type 32; .endef
kernel_dgelqf_dlarft4_8_lib4:
#endif
	
	PROLOGUE

	// zero T

	movq	ARG5, %r10 // T

	vxorpd			%ymm15, %ymm15, %ymm15
	vmovapd			%ymm15, 0(%r10)
	vmovapd			%ymm15, 32(%r10)
	vmovapd			%ymm15, 64(%r10)
	vmovapd			%ymm15, 96(%r10)

	// first column

	movq	ARG2, %r11 // D
	movq	ARG3, %r14 // sdd
	sall	$ 5, %r14d
	movq	ARG4, %r12 // dD
	movq	ARG5, %r13 // T

	vxorpd			%xmm15, %xmm15, %xmm15
	movq	ARG1, %r10 // n
	subl	$ 1, %r10d
	addq	$ 32, %r11
100:
	vmovsd			0(%r11), %xmm14
	vfmadd231sd		%xmm14, %xmm14, %xmm15
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		100b

	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		101f
	vmovsd			%xmm14, 0(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	vmovsd			0(%r11), %xmm14 // alpha
	vfmadd231sd		%xmm14, %xmm14, %xmm15 // beta
	vsqrtsd			%xmm15, %xmm15, %xmm15 // beta
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm13 // mask
#else
	vmovsd			LC00(%rip), %xmm13 // mask
#endif
	vandpd			%xmm13, %xmm14, %xmm12
	vxorpd			%xmm13, %xmm12, %xmm12
	vxorpd			%xmm12, %xmm15, %xmm15 // beta
	vmovsd			%xmm15, 0(%r11) // pD[0+ps*0]
	vsubsd			%xmm14, %xmm15, %xmm14 // beta-alpha
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC01(%rip), %xmm12
#else
	vmovapd			LC01(%rip), %xmm12
#endif
	vmovsd			%xmm14, %xmm12, %xmm12
	vmovddup		%xmm14, %xmm14
	vmovsd			%xmm15, %xmm14, %xmm14
	vdivpd			%xmm14, %xmm12, %xmm14
	vmovsd			%xmm14, 0(%r12) // dD[0]
	vxorpd			%xmm13, %xmm14, %xmm12
	vmovsd			%xmm12, 0(%r13) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			0(%r11), %ymm0
	vmovapd			0(%r11, %r14, 1), %ymm1
	vbroadcastsd	32(%r11), %ymm8
	vbroadcastsd	64(%r11), %ymm9
	vbroadcastsd	96(%r11), %ymm10
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vfmadd231pd		32(%r11), %ymm8, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		64(%r11), %ymm9, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		96(%r11), %ymm10, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm10, %ymm1
	vmovsd			%xmm8, 32(%r11)
	vmovsd			%xmm9, 64(%r11)
	vmovsd			%xmm10, 96(%r11)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	0(%r11), %ymm8
	vbroadcastsd	32(%r11), %ymm9
	vbroadcastsd	64(%r11), %ymm10
	vbroadcastsd	96(%r11), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm11, %ymm1
	vmovsd			%xmm8, 0(%r11)
	vmovsd			%xmm9, 32(%r11)
	vmovsd			%xmm10, 64(%r11)
	vmovsd			%xmm11, 96(%r11)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	0(%r11), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vmovsd			%xmm8, 0(%r11)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vbroadcastsd	0(%r13), %ymm15
	vmulpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm15, %ymm1, %ymm1

	vxorpd			%ymm15, %ymm15, %ymm15
	vblendpd		$ 0x1, %ymm15, %ymm0, %ymm0

	movq	ARG2, %r11 // D
	//
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r14, 1), %ymm9
	vaddpd			%ymm0, %ymm8, %ymm8
	vaddpd			%ymm1, %ymm9, %ymm9
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	//
	vmovapd			32(%r11), %ymm8
	vmovapd			32(%r11, %r14, 1), %ymm9
	vbroadcastsd	32(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vmovapd			%ymm8, 32(%r11)
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	//
	vmovapd			64(%r11), %ymm8
	vmovapd			64(%r11, %r14, 1), %ymm9
	vbroadcastsd	64(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vmulpd			%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 64(%r11)
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	//
	vmovapd			96(%r11), %ymm8
	vmovapd			96(%r11, %r14, 1), %ymm9
	vbroadcastsd	96(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 96(%r11)
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	//
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r14, 1), %ymm9
	vbroadcastsd	0(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	//
	vmovapd			32(%r11), %ymm8
	vmovapd			32(%r11, %r14, 1), %ymm9
	vbroadcastsd	32(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 32(%r11)
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	//
	vmovapd			64(%r11), %ymm8
	vmovapd			64(%r11, %r14, 1), %ymm9
	vbroadcastsd	64(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 64(%r11)
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	//
	vmovapd			96(%r11), %ymm8
	vmovapd			96(%r11, %r14, 1), %ymm9
	vbroadcastsd	96(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 96(%r11)
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	//
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r14, 1), %ymm9
	vbroadcastsd	0(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
	vpermpd	$ 0x55, %ymm15, %ymm15  // beta

	// second column
102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 8(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	vmovsd			40(%r11), %xmm14 // alpha
	vfmadd231sd		%xmm14, %xmm14, %xmm15 // beta
	vsqrtsd			%xmm15, %xmm15, %xmm15 // beta
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm13 // mask
#else
	vmovsd			LC00(%rip), %xmm13 // mask
#endif
	vandpd			%xmm13, %xmm14, %xmm12
	vxorpd			%xmm13, %xmm12, %xmm12
	vxorpd			%xmm12, %xmm15, %xmm15 // beta
	vmovsd			%xmm15, 40(%r11) // pD[0+ps*0]
	vsubsd			%xmm14, %xmm15, %xmm14 // beta-alpha
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC01(%rip), %xmm12
#else
	vmovapd			LC01(%rip), %xmm12
#endif
	vmovsd			%xmm14, %xmm12, %xmm12
	vmovddup		%xmm14, %xmm14
	vmovsd			%xmm15, %xmm14, %xmm14
	vdivpd			%xmm14, %xmm12, %xmm14
	vmovsd			%xmm14, 8(%r12) // dD[0]
	vxorpd			%xmm13, %xmm14, %xmm12
	vmovsd			%xmm12, 40(%r13) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			32(%r11), %ymm0
	vmovapd			32(%r11, %r14, 1), %ymm1
	vbroadcastsd	72(%r11), %ymm9
	vbroadcastsd	104(%r11), %ymm10
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vfmadd231pd		64(%r11), %ymm9, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		96(%r11), %ymm10, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm10, %ymm1
	vmovsd			%xmm9, 72(%r11)
	vmovsd			%xmm10, 104(%r11)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	8(%r11), %ymm8
	vbroadcastsd	40(%r11), %ymm9
	vbroadcastsd	72(%r11), %ymm10
	vbroadcastsd	104(%r11), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm11, %ymm1
	vmovsd			%xmm8, 8(%r11)
	vmovsd			%xmm9, 40(%r11)
	vmovsd			%xmm10, 72(%r11)
	vmovsd			%xmm11, 104(%r11)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	8(%r11), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vmovsd			%xmm8, 8(%r11)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC02(%rip), %ymm12
#else
	vmovapd			LC02(%rip), %ymm12
#endif
	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm0
	vbroadcastsd	40(%r13), %ymm15
	vmulpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm15, %ymm1, %ymm1
	vmovsd			%xmm0, 32(%r13)

	vxorpd			%ymm12, %ymm12, %ymm12
	vblendpd		$ 0x3, %ymm12, %ymm0, %ymm0

	movq	ARG2, %r11 // D
	//
	vmovapd			32(%r11), %ymm8
	vmovapd			32(%r11, %r14, 1), %ymm9
	vaddpd			%ymm0, %ymm8, %ymm8
	vaddpd			%ymm1, %ymm9, %ymm9
	vmovapd			%ymm8, 32(%r11)
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	//
	vmovapd			64(%r11), %ymm8
	vmovapd			64(%r11, %r14, 1), %ymm9
	vbroadcastsd	72(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vmovapd			%ymm8, 64(%r11)
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	//
	vmovapd			96(%r11), %ymm8
	vmovapd			96(%r11, %r14, 1), %ymm9
	vbroadcastsd	104(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vmulpd			%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 96(%r11)
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	//
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r14, 1), %ymm9
	vbroadcastsd	8(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	//
	vmovapd			32(%r11), %ymm8
	vmovapd			32(%r11, %r14, 1), %ymm9
	vbroadcastsd	40(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 32(%r11)
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	//
	vmovapd			64(%r11), %ymm8
	vmovapd			64(%r11, %r14, 1), %ymm9
	vbroadcastsd	72(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 64(%r11)
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	//
	vmovapd			96(%r11), %ymm8
	vmovapd			96(%r11, %r14, 1), %ymm9
	vbroadcastsd	104(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 96(%r11)
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r14, 1), %ymm9
	vbroadcastsd	8(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
	vpermpd	$ 0xaa, %ymm15, %ymm15  // beta

	// third column
102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 16(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	vmovsd			80(%r11), %xmm14 // alpha
	vfmadd231sd		%xmm14, %xmm14, %xmm15 // beta
	vsqrtsd			%xmm15, %xmm15, %xmm15 // beta
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm13 // mask
#else
	vmovsd			LC00(%rip), %xmm13 // mask
#endif
	vandpd			%xmm13, %xmm14, %xmm12
	vxorpd			%xmm13, %xmm12, %xmm12
	vxorpd			%xmm12, %xmm15, %xmm15 // beta
	vmovsd			%xmm15, 80(%r11) // pD[0+ps*0]
	vsubsd			%xmm14, %xmm15, %xmm14 // beta-alpha
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC01(%rip), %xmm12
#else
	vmovapd			LC01(%rip), %xmm12
#endif
	vmovsd			%xmm14, %xmm12, %xmm12
	vmovddup		%xmm14, %xmm14
	vmovsd			%xmm15, %xmm14, %xmm14
	vdivpd			%xmm14, %xmm12, %xmm14
	vmovsd			%xmm14, 16(%r12) // dD[0]
	vxorpd			%xmm13, %xmm14, %xmm12
	vmovsd			%xmm12, 80(%r13) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			64(%r11), %ymm0
	vmovapd			64(%r11, %r14, 1), %ymm1
	vbroadcastsd	112(%r11), %ymm10
	vmulpd			%ymm15, %ymm10, %ymm10
	vfmadd231pd		96(%r11), %ymm10, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm10, %ymm1
	vmovsd			%xmm10, 112(%r11)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	16(%r11), %ymm8
	vbroadcastsd	48(%r11), %ymm9
	vbroadcastsd	80(%r11), %ymm10
	vbroadcastsd	112(%r11), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm11, %ymm1
	vmovsd			%xmm8, 16(%r11)
	vmovsd			%xmm9, 48(%r11)
	vmovsd			%xmm10, 80(%r11)
	vmovsd			%xmm11, 112(%r11)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	16(%r11), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vmovsd			%xmm8, 16(%r11)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vxorpd			%xmm12, %xmm12, %xmm12
	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm15
	vmovapd			32(%r13), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vpermpd			$ 0x55, %ymm0, %ymm13
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vblendpd		$ 0x7, %ymm15, %ymm0, %ymm0
	vbroadcastsd	80(%r13), %ymm14
	vmulpd			%ymm14, %ymm0, %ymm0
	vmulpd			%ymm14, %ymm1, %ymm1
	vmovapd			%xmm0, 64(%r13)

	vxorpd			%ymm12, %ymm12, %ymm12
	vblendpd		$ 0x7, %ymm12, %ymm0, %ymm0

	movq	ARG2, %r11 // D
	//
	vmovapd			64(%r11), %ymm8
	vmovapd			64(%r11, %r14, 1), %ymm9
	vaddpd			%ymm0, %ymm8, %ymm8
	vaddpd			%ymm1, %ymm9, %ymm9
	vmovapd			%ymm8, 64(%r11)
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	//
	vmovapd			96(%r11), %ymm8
	vmovapd			96(%r11, %r14, 1), %ymm9
	vbroadcastsd	112(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vxorpd			%ymm15, %ymm15, %ymm15
	vmovapd			%ymm8, 96(%r11)
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	//
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r14, 1), %ymm9
	vbroadcastsd	16(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	//
	vmovapd			32(%r11), %ymm8
	vmovapd			32(%r11, %r14, 1), %ymm9
	vbroadcastsd	48(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 32(%r11)
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	//
	vmovapd			64(%r11), %ymm8
	vmovapd			64(%r11, %r14, 1), %ymm9
	vbroadcastsd	80(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 64(%r11)
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	//
	vmovapd			96(%r11), %ymm8
	vmovapd			96(%r11, %r14, 1), %ymm9
	vbroadcastsd	112(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 96(%r11)
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r14, 1), %ymm9
	vbroadcastsd	16(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
	vpermpd	$ 0xff, %ymm15, %ymm15  // beta

102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 24(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	vmovsd			120(%r11), %xmm14 // alpha
	vfmadd231sd		%xmm14, %xmm14, %xmm15 // beta
	vsqrtsd			%xmm15, %xmm15, %xmm15 // beta
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm13 // mask
#else
	vmovsd			LC00(%rip), %xmm13 // mask
#endif
	vandpd			%xmm13, %xmm14, %xmm12
	vxorpd			%xmm13, %xmm12, %xmm12
	vxorpd			%xmm12, %xmm15, %xmm15 // beta
	vmovsd			%xmm15, 120(%r11) // pD[0+ps*0]
	vsubsd			%xmm14, %xmm15, %xmm14 // beta-alpha
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC01(%rip), %xmm12
#else
	vmovapd			LC01(%rip), %xmm12
#endif
	vmovsd			%xmm14, %xmm12, %xmm12
	vmovddup		%xmm14, %xmm14
	vmovsd			%xmm15, %xmm14, %xmm14
	vdivpd			%xmm14, %xmm12, %xmm14
	vmovsd			%xmm14, 24(%r12) // dD[0]
	vxorpd			%xmm13, %xmm14, %xmm12
	vmovsd			%xmm12, 120(%r13) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			96(%r11), %ymm0
	vmovapd			96(%r11, %r14, 1), %ymm1
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	24(%r11), %ymm8
	vbroadcastsd	56(%r11), %ymm9
	vbroadcastsd	88(%r11), %ymm10
	vbroadcastsd	120(%r11), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm11, %ymm1
	vmovsd			%xmm8, 24(%r11)
	vmovsd			%xmm9, 56(%r11)
	vmovsd			%xmm10, 88(%r11)
	vmovsd			%xmm11, 120(%r11)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	24(%r11), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vmovsd			%xmm8, 24(%r11)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vxorpd			%xmm12, %xmm12, %xmm12

	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm15

	vmovapd			32(%r13), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vpermpd			$ 0x55, %ymm0, %ymm13
	vfmadd231pd		%ymm14, %ymm13, %ymm15

	vmovapd			64(%r13), %ymm14
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm14
	vpermpd			$ 0xaa, %ymm0, %ymm13
	vfmadd231pd		%ymm14, %ymm13, %ymm15

	vbroadcastsd	120(%r13), %ymm14
	vmulpd			%ymm14, %ymm15, %ymm15
	vmulpd			%ymm14, %ymm1, %ymm1
	vmovapd			96(%r13), %ymm0
	vblendpd		$ 0x7, %ymm15, %ymm0, %ymm0
	vmovapd			%ymm0, 96(%r13)

	movq	ARG2, %r11 // D
	//
	vmovapd			96(%r11, %r14, 1), %ymm9
	vaddpd			%ymm1, %ymm9, %ymm9
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	//
	vmovapd			0(%r11, %r14, 1), %ymm9
	vbroadcastsd	24(%r11), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	//
	vmovapd			32(%r11, %r14, 1), %ymm9
	vbroadcastsd	56(%r11), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	//
	vmovapd			64(%r11, %r14, 1), %ymm9
	vbroadcastsd	88(%r11), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	//
	vmovapd			96(%r11, %r14, 1), %ymm9
	vbroadcastsd	120(%r11), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	vmovapd			0(%r11, %r14, 1), %ymm9
	vbroadcastsd	24(%r11), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:

102:

	EPILOGUE
	
	ret

#if defined(OS_LINUX)
	.size	kernel_dgelqf_dlarft4_8_lib4, .-kernel_dgelqf_dlarft4_8_lib4
#endif





//                                  1      2           3           4
// void kernel_dgelqf_dlarft4_4_lib4(int n, double *pD, double *dD, double *pT, double *beta)

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dgelqf_dlarft4_4_lib4
	.type kernel_dgelqf_dlarft4_4_lib4, @function
kernel_dgelqf_dlarft4_4_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dgelqf_dlarft4_4_lib4
_kernel_dgelqf_dlarft4_4_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dgelqf_dlarft4_4_lib4
	.def kernel_dgelqf_dlarft4_4_lib4; .scl 2; .type 32; .endef
kernel_dgelqf_dlarft4_4_lib4:
#endif
	
	PROLOGUE

	// zero T

	movq	ARG4, %r10 // T

	vxorpd			%ymm15, %ymm15, %ymm15
	vmovapd			%ymm15, 0(%r10)
	vmovapd			%ymm15, 32(%r10)
	vmovapd			%ymm15, 64(%r10)
	vmovapd			%ymm15, 96(%r10)

	// first column

	movq	ARG2, %r11 // D
	movq	ARG3, %r12 // dD
	movq	ARG4, %r13 // T

	vxorpd			%xmm15, %xmm15, %xmm15
	movq	ARG1, %r10 // n
	subl	$ 1, %r10d
	addq	$ 32, %r11
100:
	vmovsd			0(%r11), %xmm14
	vfmadd231sd		%xmm14, %xmm14, %xmm15
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		100b

	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		101f
	vmovsd			%xmm14, 0(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	vmovsd			0(%r11), %xmm14 // alpha
	vfmadd231sd		%xmm14, %xmm14, %xmm15 // beta
	vsqrtsd			%xmm15, %xmm15, %xmm15 // beta
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm13 // mask
#else
	vmovsd			LC00(%rip), %xmm13 // mask
#endif
	vandpd			%xmm13, %xmm14, %xmm12
	vxorpd			%xmm13, %xmm12, %xmm12
	vxorpd			%xmm12, %xmm15, %xmm15 // beta
	vmovsd			%xmm15, 0(%r11) // pD[0+ps*0]
	vsubsd			%xmm14, %xmm15, %xmm14 // beta-alpha
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC01(%rip), %xmm12
#else
	vmovapd			LC01(%rip), %xmm12
#endif
	vmovsd			%xmm14, %xmm12, %xmm12
	vmovddup		%xmm14, %xmm14
	vmovsd			%xmm15, %xmm14, %xmm14
	vdivpd			%xmm14, %xmm12, %xmm14
	vmovsd			%xmm14, 0(%r12) // dD[0]
	vxorpd			%xmm13, %xmm14, %xmm12
	vmovsd			%xmm12, 0(%r13) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			0(%r11), %ymm0
	vbroadcastsd	32(%r11), %ymm8
	vbroadcastsd	64(%r11), %ymm9
	vbroadcastsd	96(%r11), %ymm10
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vfmadd231pd		32(%r11), %ymm8, %ymm0
	vfmadd231pd		64(%r11), %ymm9, %ymm0
	vfmadd231pd		96(%r11), %ymm10, %ymm0
	vmovsd			%xmm8, 32(%r11)
	vmovsd			%xmm9, 64(%r11)
	vmovsd			%xmm10, 96(%r11)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	0(%r11), %ymm8
	vbroadcastsd	32(%r11), %ymm9
	vbroadcastsd	64(%r11), %ymm10
	vbroadcastsd	96(%r11), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vmovsd			%xmm8, 0(%r11)
	vmovsd			%xmm9, 32(%r11)
	vmovsd			%xmm10, 64(%r11)
	vmovsd			%xmm11, 96(%r11)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	0(%r11), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vmovsd			%xmm8, 0(%r11)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vbroadcastsd	0(%r13), %ymm15
	vmulpd			%ymm15, %ymm0, %ymm0

	vxorpd			%ymm15, %ymm15, %ymm15
	vblendpd		$ 0x1, %ymm15, %ymm0, %ymm0

	movq	ARG2, %r11 // D
	vmovapd			0(%r11), %ymm8
	vmovapd			32(%r11), %ymm9
	vmovapd			64(%r11), %ymm10
	vmovapd			96(%r11), %ymm11
	vaddpd			%ymm0, %ymm8, %ymm8
	vbroadcastsd	32(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm9
	vbroadcastsd	64(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm10
	vbroadcastsd	96(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm11
	vmulpd			%ymm10, %ymm10, %ymm15
	vfmadd231pd		%ymm11, %ymm11, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 32(%r11)
	vmovapd			%ymm10, 64(%r11)
	vmovapd			%ymm11, 96(%r11)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	vmovapd			0(%r11), %ymm8
	vmovapd			32(%r11), %ymm9
	vmovapd			64(%r11), %ymm10
	vmovapd			96(%r11), %ymm11
	vbroadcastsd	0(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vbroadcastsd	32(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm9
	vbroadcastsd	64(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm10
	vbroadcastsd	96(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm11
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vfmadd231pd		%ymm11, %ymm11, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 32(%r11)
	vmovapd			%ymm10, 64(%r11)
	vmovapd			%ymm11, 96(%r11)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	vmovapd			0(%r11), %ymm8
	vbroadcastsd	0(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 0(%r11)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
	vpermpd	$ 0x55, %ymm15, %ymm15  // beta

	// second column
102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 8(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	vmovsd			40(%r11), %xmm14 // alpha
	vfmadd231sd		%xmm14, %xmm14, %xmm15 // beta
	vsqrtsd			%xmm15, %xmm15, %xmm15 // beta
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm13 // mask
#else
	vmovsd			LC00(%rip), %xmm13 // mask
#endif
	vandpd			%xmm13, %xmm14, %xmm12
	vxorpd			%xmm13, %xmm12, %xmm12
	vxorpd			%xmm12, %xmm15, %xmm15 // beta
	vmovsd			%xmm15, 40(%r11) // pD[0+ps*0]
	vsubsd			%xmm14, %xmm15, %xmm14 // beta-alpha
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC01(%rip), %xmm12
#else
	vmovapd			LC01(%rip), %xmm12
#endif
	vmovsd			%xmm14, %xmm12, %xmm12
	vmovddup		%xmm14, %xmm14
	vmovsd			%xmm15, %xmm14, %xmm14
	vdivpd			%xmm14, %xmm12, %xmm14
	vmovsd			%xmm14, 8(%r12) // dD[0]
	vxorpd			%xmm13, %xmm14, %xmm12
	vmovsd			%xmm12, 40(%r13) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			32(%r11), %ymm0
	vbroadcastsd	72(%r11), %ymm9
	vbroadcastsd	104(%r11), %ymm10
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vfmadd231pd		64(%r11), %ymm9, %ymm0
	vfmadd231pd		96(%r11), %ymm10, %ymm0
	vmovsd			%xmm9, 72(%r11)
	vmovsd			%xmm10, 104(%r11)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	8(%r11), %ymm8
	vbroadcastsd	40(%r11), %ymm9
	vbroadcastsd	72(%r11), %ymm10
	vbroadcastsd	104(%r11), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vmovsd			%xmm8, 8(%r11)
	vmovsd			%xmm9, 40(%r11)
	vmovsd			%xmm10, 72(%r11)
	vmovsd			%xmm11, 104(%r11)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	8(%r11), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vmovsd			%xmm8, 8(%r11)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC02(%rip), %ymm12
#else
	vmovapd			LC02(%rip), %ymm12
#endif
	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm0
	vbroadcastsd	40(%r13), %ymm15
	vmulpd			%ymm15, %ymm0, %ymm0
	vmovsd			%xmm0, 32(%r13)

	vxorpd			%ymm12, %ymm12, %ymm12
	vblendpd		$ 0x3, %ymm12, %ymm0, %ymm0

	movq	ARG2, %r11 // D
	vmovapd			32(%r11), %ymm9
	vmovapd			64(%r11), %ymm10
	vmovapd			96(%r11), %ymm11
	vaddpd			%ymm0, %ymm9, %ymm9
	vbroadcastsd	72(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm10
	vbroadcastsd	104(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm11
	vmulpd			%ymm11, %ymm11, %ymm15
	vmovapd			%ymm9, 32(%r11)
	vmovapd			%ymm10, 64(%r11)
	vmovapd			%ymm11, 96(%r11)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	vmovapd			0(%r11), %ymm8
	vmovapd			32(%r11), %ymm9
	vmovapd			64(%r11), %ymm10
	vmovapd			96(%r11), %ymm11
	vbroadcastsd	8(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vbroadcastsd	40(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm9
	vbroadcastsd	72(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm10
	vbroadcastsd	104(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm11
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vfmadd231pd		%ymm11, %ymm11, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 32(%r11)
	vmovapd			%ymm10, 64(%r11)
	vmovapd			%ymm11, 96(%r11)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	vmovapd			0(%r11), %ymm8
	vbroadcastsd	8(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 0(%r11)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
	vpermpd	$ 0xaa, %ymm15, %ymm15  // beta

	// third column
102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 16(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	vmovsd			80(%r11), %xmm14 // alpha
	vfmadd231sd		%xmm14, %xmm14, %xmm15 // beta
	vsqrtsd			%xmm15, %xmm15, %xmm15 // beta
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm13 // mask
#else
	vmovsd			LC00(%rip), %xmm13 // mask
#endif
	vandpd			%xmm13, %xmm14, %xmm12
	vxorpd			%xmm13, %xmm12, %xmm12
	vxorpd			%xmm12, %xmm15, %xmm15 // beta
	vmovsd			%xmm15, 80(%r11) // pD[0+ps*0]
	vsubsd			%xmm14, %xmm15, %xmm14 // beta-alpha
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC01(%rip), %xmm12
#else
	vmovapd			LC01(%rip), %xmm12
#endif
	vmovsd			%xmm14, %xmm12, %xmm12
	vmovddup		%xmm14, %xmm14
	vmovsd			%xmm15, %xmm14, %xmm14
	vdivpd			%xmm14, %xmm12, %xmm14
	vmovsd			%xmm14, 16(%r12) // dD[0]
	vxorpd			%xmm13, %xmm14, %xmm12
	vmovsd			%xmm12, 80(%r13) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			64(%r11), %ymm0
	vbroadcastsd	112(%r11), %ymm10
	vmulpd			%ymm15, %ymm10, %ymm10
	vfmadd231pd		96(%r11), %ymm10, %ymm0
	vmovsd			%xmm10, 112(%r11)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	16(%r11), %ymm8
	vbroadcastsd	48(%r11), %ymm9
	vbroadcastsd	80(%r11), %ymm10
	vbroadcastsd	112(%r11), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vmovsd			%xmm8, 16(%r11)
	vmovsd			%xmm9, 48(%r11)
	vmovsd			%xmm10, 80(%r11)
	vmovsd			%xmm11, 112(%r11)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	16(%r11), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vmovsd			%xmm8, 16(%r11)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vxorpd			%xmm12, %xmm12, %xmm12
	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm1
	vmovapd			32(%r13), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vpermpd			$ 0x55, %ymm0, %ymm13
	vfmadd231pd		%ymm14, %ymm13, %ymm1
	vblendpd		$ 0x7, %ymm1, %ymm0, %ymm0
	vbroadcastsd	80(%r13), %ymm15
	vmulpd			%ymm15, %ymm0, %ymm0
	vmovapd			%xmm0, 64(%r13)

	vxorpd			%ymm12, %ymm12, %ymm12
	vblendpd		$ 0x7, %ymm12, %ymm0, %ymm0

	movq	ARG2, %r11 // D
	vmovapd			64(%r11), %ymm10
	vmovapd			96(%r11), %ymm11
	vaddpd			%ymm0, %ymm10, %ymm10
	vbroadcastsd	112(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm11
	vxorpd			%ymm15, %ymm15, %ymm15
	vmovapd			%ymm10, 64(%r11)
	vmovapd			%ymm11, 96(%r11)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	vmovapd			0(%r11), %ymm8
	vmovapd			32(%r11), %ymm9
	vmovapd			64(%r11), %ymm10
	vmovapd			96(%r11), %ymm11
	vbroadcastsd	16(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vbroadcastsd	48(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm9
	vbroadcastsd	80(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm10
	vbroadcastsd	112(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm11
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vfmadd231pd		%ymm11, %ymm11, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 32(%r11)
	vmovapd			%ymm10, 64(%r11)
	vmovapd			%ymm11, 96(%r11)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	vmovapd			0(%r11), %ymm8
	vbroadcastsd	16(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 0(%r11)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
	vpermpd	$ 0xff, %ymm15, %ymm15  // beta

102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 24(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	vmovsd			120(%r11), %xmm14 // alpha
	vfmadd231sd		%xmm14, %xmm14, %xmm15 // beta
	vsqrtsd			%xmm15, %xmm15, %xmm15 // beta
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm13 // mask
#else
	vmovsd			LC00(%rip), %xmm13 // mask
#endif
	vandpd			%xmm13, %xmm14, %xmm12
	vxorpd			%xmm13, %xmm12, %xmm12
	vxorpd			%xmm12, %xmm15, %xmm15 // beta
	vmovsd			%xmm15, 120(%r11) // pD[0+ps*0]
	vsubsd			%xmm14, %xmm15, %xmm14 // beta-alpha
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC01(%rip), %xmm12
#else
	vmovapd			LC01(%rip), %xmm12
#endif
	vmovsd			%xmm14, %xmm12, %xmm12
	vmovddup		%xmm14, %xmm14
	vmovsd			%xmm15, %xmm14, %xmm14
	vdivpd			%xmm14, %xmm12, %xmm14
	vmovsd			%xmm14, 24(%r12) // dD[0]
	vxorpd			%xmm13, %xmm14, %xmm12
	vmovsd			%xmm12, 120(%r13) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			96(%r11), %ymm0
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	24(%r11), %ymm8
	vbroadcastsd	56(%r11), %ymm9
	vbroadcastsd	88(%r11), %ymm10
	vbroadcastsd	120(%r11), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vmovsd			%xmm8, 24(%r11)
	vmovsd			%xmm9, 56(%r11)
	vmovsd			%xmm10, 88(%r11)
	vmovsd			%xmm11, 120(%r11)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	24(%r11), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vmovsd			%xmm8, 24(%r11)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vxorpd			%xmm12, %xmm12, %xmm12

	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm1

	vmovapd			32(%r13), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vpermpd			$ 0x55, %ymm0, %ymm13
	vfmadd231pd		%ymm14, %ymm13, %ymm1

	vmovapd			64(%r13), %ymm14
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm14
	vpermpd			$ 0xaa, %ymm0, %ymm13
	vfmadd231pd		%ymm14, %ymm13, %ymm1

	vbroadcastsd	120(%r13), %ymm15
	vmulpd			%ymm15, %ymm1, %ymm1
	vmovapd			96(%r13), %ymm0
	vblendpd		$ 0x7, %ymm1, %ymm0, %ymm0
	vmovapd			%ymm0, 96(%r13)

102:

	EPILOGUE
	
	ret

#if defined(OS_LINUX)
	.size	kernel_dgelqf_dlarft4_4_lib4, .-kernel_dgelqf_dlarft4_4_lib4
#endif





//                            1           2
// void kernel_dlarfb_12_lib4(double *dK, double *pT)

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dlarfb_12_lib4
	.type kernel_dlarfb_12_lib4, @function
kernel_dlarfb_12_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dlarfb_12_lib4
_kernel_dlarfb_12_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dlarfb_12_lib4
	.def kernel_dlarfb_12_lib4; .scl 2; .type 32; .endef
kernel_dlarfb_12_lib4:
#endif
	
	PROLOGUE

	movq	ARG1, %r10 // K
	movq	ARG2, %r11 // T
	movq	$ 384, %r12 // sdt !!!!!!!!!!!!!!!!!!!!!!!!!

	//
	vmovapd			352(%r10), %ymm12
	vbroadcastsd	376(%r11, %r12, 2), %ymm13
	vmulpd			%ymm12, %ymm13, %ymm11
	//
	vmovapd			320(%r10), %ymm12
	vbroadcastsd	368(%r11, %r12, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11
	vbroadcastsd	336(%r11, %r12, 2), %ymm13
	vmulpd			%ymm12, %ymm13, %ymm10
	//
	vmovapd			288(%r10), %ymm12
	vbroadcastsd	360(%r11, %r12, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11
	vbroadcastsd	328(%r11, %r12, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	296(%r11, %r12, 2), %ymm13
	vmulpd			%ymm12, %ymm13, %ymm9
	//
	vmovapd			256(%r10), %ymm12
	vbroadcastsd	352(%r11, %r12, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11
	vbroadcastsd	320(%r11, %r12, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	288(%r11, %r12, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	256(%r11, %r12, 2), %ymm13
	vmulpd			%ymm12, %ymm13, %ymm8
	//
	vmovapd			224(%r10), %ymm12
	vbroadcastsd	376(%r11, %r12, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11
	vbroadcastsd	344(%r11, %r12, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	312(%r11, %r12, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	280(%r11, %r12, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	248(%r11, %r12, 1), %ymm13
	vmulpd			%ymm12, %ymm13, %ymm7
	//
	vmovapd			192(%r10), %ymm12
	vbroadcastsd	368(%r11, %r12, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11
	vbroadcastsd	336(%r11, %r12, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	304(%r11, %r12, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	272(%r11, %r12, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	240(%r11, %r12, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	208(%r11, %r12, 1), %ymm13
	vmulpd			%ymm12, %ymm13, %ymm6
	//
	vmovapd			160(%r10), %ymm12
	vbroadcastsd	360(%r11, %r12, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11
	vbroadcastsd	328(%r11, %r12, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	296(%r11, %r12, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	264(%r11, %r12, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	232(%r11, %r12, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	200(%r11, %r12, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	168(%r11, %r12, 1), %ymm13
	vmulpd			%ymm12, %ymm13, %ymm5
	//
	vmovapd			128(%r10), %ymm12
	vbroadcastsd	352(%r11, %r12, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11
	vbroadcastsd	320(%r11, %r12, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	288(%r11, %r12, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	256(%r11, %r12, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	224(%r11, %r12, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	192(%r11, %r12, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	160(%r11, %r12, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	128(%r11, %r12, 1), %ymm13
	vmulpd			%ymm12, %ymm13, %ymm4
	//
	vmovapd			96(%r10), %ymm12
	vbroadcastsd	376(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11
	vbroadcastsd	344(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	312(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	280(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	248(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	216(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	184(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	152(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	120(%r11), %ymm13
	vmulpd			%ymm12, %ymm13, %ymm3
	//
	vmovapd			64(%r10), %ymm12
	vbroadcastsd	368(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11
	vbroadcastsd	336(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	304(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	272(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	240(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	208(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	176(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	144(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	112(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	80(%r11), %ymm13
	vmulpd			%ymm12, %ymm13, %ymm2
	//
	vmovapd			32(%r10), %ymm12
	vbroadcastsd	360(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11
	vbroadcastsd	328(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	296(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	264(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	232(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	200(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	168(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	136(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	104(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	72(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	40(%r11), %ymm13
	vmulpd			%ymm12, %ymm13, %ymm1
	//
	vmovapd			0(%r10), %ymm12
	vbroadcastsd	352(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11
	vbroadcastsd	320(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	288(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	256(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	224(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	192(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	160(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	128(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	96(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	64(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	32(%r11), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	0(%r11), %ymm13
	vmulpd			%ymm12, %ymm13, %ymm0

	vmovapd			%ymm11, 352(%r10)
	vmovapd			%ymm10, 320(%r10)
	vmovapd			%ymm9, 288(%r10)
	vmovapd			%ymm8, 256(%r10)
	vmovapd			%ymm7, 224(%r10)
	vmovapd			%ymm6, 192(%r10)
	vmovapd			%ymm5, 160(%r10)
	vmovapd			%ymm4, 128(%r10)
	vmovapd			%ymm3, 96(%r10)
	vmovapd			%ymm2, 64(%r10)
	vmovapd			%ymm1, 32(%r10)
	vmovapd			%ymm0, 0(%r10)

	EPILOGUE
	
	ret

#if defined(OS_LINUX)
	.size	kernel_dlarfb_12_lib4, .-kernel_dlarfb_12_lib4
#endif





//                                        1      2           3        4           5
// void kernel_dgelqf_pd_dlarft12_12_lib4(int n, double *pD, int sdd, double *dD, double *pT)

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dgelqf_pd_dlarft12_12_lib4
	.type kernel_dgelqf_pd_dlarft12_12_lib4, @function
kernel_dgelqf_pd_dlarft12_12_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dgelqf_pd_dlarft12_12_lib4
_kernel_dgelqf_pd_dlarft12_12_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dgelqf_pd_dlarft12_12_lib4
	.def kernel_dgelqf_pd_dlarft12_12_lib4; .scl 2; .type 32; .endef
kernel_dgelqf_pd_dlarft12_12_lib4:
#endif
	
	PROLOGUE

	// zero T

	movq	ARG5, %r10 // T

	vxorpd			%ymm15, %ymm15, %ymm15
	vmovapd			%ymm15, 0(%r10)
	vmovapd			%ymm15, 32(%r10)
	vmovapd			%ymm15, 64(%r10)
	vmovapd			%ymm15, 96(%r10)

	// first column

	movq	ARG2, %r11 // D
	movq	ARG3, %r14 // sdd
	sall	$ 5, %r14d
	movq	ARG4, %r12 // dD
	movq	ARG5, %r13 // T
	movq	$ 384, %r15 // sdt !!!!!!!!!!!!!!!!!!!!!!!!!

	vxorpd			%xmm15, %xmm15, %xmm15
	movq	ARG1, %r10 // n
	subl	$ 1, %r10d
	addq	$ 32, %r11
100:
	vmovsd			0(%r11), %xmm14
	vfmadd231sd		%xmm14, %xmm14, %xmm15
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		100b

	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		101f
	vmovsd			%xmm14, 0(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	vmovsd			0(%r11), %xmm14 // alpha
	vmovapd			%xmm15, %xmm13 // beta
	vfmadd231sd		%xmm14, %xmm14, %xmm13 // beta
	vsqrtsd			%xmm13, %xmm13, %xmm13 // beta
	vmovsd			%xmm13, 0(%r11) // pD[0+ps*0]

	vxorpd			%xmm12, %xmm12, %xmm12
	vucomisd		%xmm12, %xmm14
	jbe		111f
	// alpha>0
	vaddsd			%xmm13, %xmm14, %xmm14 // tmp
	vdivsd			%xmm14, %xmm15, %xmm14 // tmp
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm14, %xmm14
	jmp		112f
111: // alpha<=0
	vsubsd			%xmm13, %xmm14, %xmm14 // tmp
112:
	vmulsd			%xmm14, %xmm14, %xmm12 // tmp*tmp
	vaddsd			%xmm12, %xmm15, %xmm15 // sigma+tmp*tmp
	vaddsd			%xmm12, %xmm12, %xmm12 // 2*tmp*tmp
	vdivsd			%xmm15, %xmm12, %xmm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC02(%rip), %xmm12
#else
	vmovapd			LC02(%rip), %xmm12
#endif
	vdivsd			%xmm14, %xmm12, %xmm14 // tmp
	vmovddup		%xmm14, %xmm14

	vmovsd			%xmm15, 0(%r12) // dD[0]
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm15, %xmm15
	vmovsd			%xmm15, 0(%r13) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			0(%r11), %ymm0
	vmovapd			0(%r11, %r14, 1), %ymm1
	vmovapd			0(%r11, %r14, 2), %ymm2
	vbroadcastsd	32(%r11), %ymm8
	vbroadcastsd	64(%r11), %ymm9
	vbroadcastsd	96(%r11), %ymm10
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vfmadd231pd		32(%r11), %ymm8, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		32(%r11, %r14, 2), %ymm8, %ymm2
	vfmadd231pd		64(%r11), %ymm9, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		96(%r11), %ymm10, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm10, %ymm2
	vmovsd			%xmm8, 32(%r11)
	vmovsd			%xmm9, 64(%r11)
	vmovsd			%xmm10, 96(%r11)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	0(%r11), %ymm8
	vbroadcastsd	32(%r11), %ymm9
	vbroadcastsd	64(%r11), %ymm10
	vbroadcastsd	96(%r11), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		32(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm10, %ymm2
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm11, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm11, %ymm2
	vmovsd			%xmm8, 0(%r11)
	vmovsd			%xmm9, 32(%r11)
	vmovsd			%xmm10, 64(%r11)
	vmovsd			%xmm11, 96(%r11)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	0(%r11), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vmovsd			%xmm8, 0(%r11)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vbroadcastsd	0(%r13), %ymm15
	vmulpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm15, %ymm2, %ymm2

	vxorpd			%ymm15, %ymm15, %ymm15
	vblendpd		$ 0x1, %ymm15, %ymm0, %ymm0

	movq	ARG2, %r11 // D
	//
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vaddpd			%ymm0, %ymm8, %ymm8
	vaddpd			%ymm1, %ymm9, %ymm9
	vaddpd			%ymm2, %ymm10, %ymm10
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11), %ymm8
	vmovapd			32(%r11, %r14, 1), %ymm9
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	32(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmovapd			%ymm8, 32(%r11)
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11), %ymm8
	vmovapd			64(%r11, %r14, 1), %ymm9
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	64(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmulpd			%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 64(%r11)
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11), %ymm8
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	96(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 96(%r11)
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	//
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	0(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11), %ymm8
	vmovapd			32(%r11, %r14, 1), %ymm9
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	32(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 32(%r11)
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11), %ymm8
	vmovapd			64(%r11, %r14, 1), %ymm9
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	64(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 64(%r11)
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11), %ymm8
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	96(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 96(%r11)
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	//
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	0(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
	vpermpd	$ 0x55, %ymm15, %ymm15  // beta

	// second column
102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 8(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	vmovsd			40(%r11), %xmm14 // alpha
	vmovapd			%xmm15, %xmm13 // beta
	vfmadd231sd		%xmm14, %xmm14, %xmm13 // beta
	vsqrtsd			%xmm13, %xmm13, %xmm13 // beta
	vmovsd			%xmm13, 40(%r11) // pD[0+ps*0]

	vxorpd			%xmm12, %xmm12, %xmm12
	vucomisd		%xmm12, %xmm14
	jbe		111f
	// alpha>0
	vaddsd			%xmm13, %xmm14, %xmm14 // tmp
	vdivsd			%xmm14, %xmm15, %xmm14 // tmp
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm14, %xmm14
	jmp		112f
111: // alpha<=0
	vsubsd			%xmm13, %xmm14, %xmm14 // tmp
112:
	vmulsd			%xmm14, %xmm14, %xmm12 // tmp*tmp
	vaddsd			%xmm12, %xmm15, %xmm15 // sigma+tmp*tmp
	vaddsd			%xmm12, %xmm12, %xmm12 // 2*tmp*tmp
	vdivsd			%xmm15, %xmm12, %xmm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC02(%rip), %xmm12
#else
	vmovapd			LC02(%rip), %xmm12
#endif
	vdivsd			%xmm14, %xmm12, %xmm14 // tmp
	vmovddup		%xmm14, %xmm14

	vmovsd			%xmm15, 8(%r12) // dD[0]
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm15, %xmm15
	vmovsd			%xmm15, 40(%r13) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			32(%r11), %ymm0
	vmovapd			32(%r11, %r14, 1), %ymm1
	vmovapd			32(%r11, %r14, 2), %ymm2
	vbroadcastsd	72(%r11), %ymm9
	vbroadcastsd	104(%r11), %ymm10
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vfmadd231pd		64(%r11), %ymm9, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		96(%r11), %ymm10, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm10, %ymm2
	vmovsd			%xmm9, 72(%r11)
	vmovsd			%xmm10, 104(%r11)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	8(%r11), %ymm8
	vbroadcastsd	40(%r11), %ymm9
	vbroadcastsd	72(%r11), %ymm10
	vbroadcastsd	104(%r11), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		32(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm10, %ymm2
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm11, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm11, %ymm2
	vmovsd			%xmm8, 8(%r11)
	vmovsd			%xmm9, 40(%r11)
	vmovsd			%xmm10, 72(%r11)
	vmovsd			%xmm11, 104(%r11)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	8(%r11), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vmovsd			%xmm8, 8(%r11)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC02(%rip), %ymm12
#else
	vmovapd			LC02(%rip), %ymm12
#endif
	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm0
	vbroadcastsd	40(%r13), %ymm15
	vmulpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm15, %ymm1, %ymm1
	vmulpd			%ymm15, %ymm2, %ymm2
	vmovsd			%xmm0, 32(%r13)

	vxorpd			%ymm12, %ymm12, %ymm12
	vblendpd		$ 0x3, %ymm12, %ymm0, %ymm0

	movq	ARG2, %r11 // D
	//
	vmovapd			32(%r11), %ymm8
	vmovapd			32(%r11, %r14, 1), %ymm9
	vmovapd			32(%r11, %r14, 2), %ymm10
	vaddpd			%ymm0, %ymm8, %ymm8
	vaddpd			%ymm1, %ymm9, %ymm9
	vaddpd			%ymm2, %ymm10, %ymm10
	vmovapd			%ymm8, 32(%r11)
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11), %ymm8
	vmovapd			64(%r11, %r14, 1), %ymm9
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	72(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmovapd			%ymm8, 64(%r11)
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11), %ymm8
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	104(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmulpd			%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 96(%r11)
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	//
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	8(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11), %ymm8
	vmovapd			32(%r11, %r14, 1), %ymm9
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	40(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 32(%r11)
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11), %ymm8
	vmovapd			64(%r11, %r14, 1), %ymm9
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	72(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 64(%r11)
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11), %ymm8
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	104(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 96(%r11)
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	8(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
	vpermpd	$ 0xaa, %ymm15, %ymm15  // beta

	// third column
102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 16(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	vmovsd			80(%r11), %xmm14 // alpha
	vmovapd			%xmm15, %xmm13 // beta
	vfmadd231sd		%xmm14, %xmm14, %xmm13 // beta
	vsqrtsd			%xmm13, %xmm13, %xmm13 // beta
	vmovsd			%xmm13, 80(%r11) // pD[0+ps*0]

	vxorpd			%xmm12, %xmm12, %xmm12
	vucomisd		%xmm12, %xmm14
	jbe		111f
	// alpha>0
	vaddsd			%xmm13, %xmm14, %xmm14 // tmp
	vdivsd			%xmm14, %xmm15, %xmm14 // tmp
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm14, %xmm14
	jmp		112f
111: // alpha<=0
	vsubsd			%xmm13, %xmm14, %xmm14 // tmp
112:
	vmulsd			%xmm14, %xmm14, %xmm12 // tmp*tmp
	vaddsd			%xmm12, %xmm15, %xmm15 // sigma+tmp*tmp
	vaddsd			%xmm12, %xmm12, %xmm12 // 2*tmp*tmp
	vdivsd			%xmm15, %xmm12, %xmm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC02(%rip), %xmm12
#else
	vmovapd			LC02(%rip), %xmm12
#endif
	vdivsd			%xmm14, %xmm12, %xmm14 // tmp
	vmovddup		%xmm14, %xmm14

	vmovsd			%xmm15, 16(%r12) // dD[0]
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm15, %xmm15
	vmovsd			%xmm15, 80(%r13) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			64(%r11), %ymm0
	vmovapd			64(%r11, %r14, 1), %ymm1
	vmovapd			64(%r11, %r14, 2), %ymm2
	vbroadcastsd	112(%r11), %ymm10
	vmulpd			%ymm15, %ymm10, %ymm10
	vfmadd231pd		96(%r11), %ymm10, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm10, %ymm2
	vmovsd			%xmm10, 112(%r11)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	16(%r11), %ymm8
	vbroadcastsd	48(%r11), %ymm9
	vbroadcastsd	80(%r11), %ymm10
	vbroadcastsd	112(%r11), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		32(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm10, %ymm2
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm11, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm11, %ymm2
	vmovsd			%xmm8, 16(%r11)
	vmovsd			%xmm9, 48(%r11)
	vmovsd			%xmm10, 80(%r11)
	vmovsd			%xmm11, 112(%r11)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	16(%r11), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vmovsd			%xmm8, 16(%r11)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vxorpd			%xmm12, %xmm12, %xmm12
	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm15
	vmovapd			32(%r13), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vpermpd			$ 0x55, %ymm0, %ymm13
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vblendpd		$ 0x7, %ymm15, %ymm0, %ymm0
	vbroadcastsd	80(%r13), %ymm14
	vmulpd			%ymm14, %ymm0, %ymm0
	vmulpd			%ymm14, %ymm1, %ymm1
	vmulpd			%ymm14, %ymm2, %ymm2
	vmovapd			%xmm0, 64(%r13)

	vxorpd			%ymm12, %ymm12, %ymm12
	vblendpd		$ 0x7, %ymm12, %ymm0, %ymm0

	movq	ARG2, %r11 // D
	//
	vmovapd			64(%r11), %ymm8
	vmovapd			64(%r11, %r14, 1), %ymm9
	vmovapd			64(%r11, %r14, 2), %ymm10
	vaddpd			%ymm0, %ymm8, %ymm8
	vaddpd			%ymm1, %ymm9, %ymm9
	vaddpd			%ymm2, %ymm10, %ymm10
	vmovapd			%ymm8, 64(%r11)
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11), %ymm8
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	112(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vxorpd			%ymm15, %ymm15, %ymm15
	vmovapd			%ymm8, 96(%r11)
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	//
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	16(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11), %ymm8
	vmovapd			32(%r11, %r14, 1), %ymm9
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	48(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 32(%r11)
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11), %ymm8
	vmovapd			64(%r11, %r14, 1), %ymm9
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	80(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 64(%r11)
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11), %ymm8
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	112(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 96(%r11)
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	16(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
	vpermpd	$ 0xff, %ymm15, %ymm15  // beta

	// fourth column
102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 24(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	vmovsd			120(%r11), %xmm14 // alpha
	vmovapd			%xmm15, %xmm13 // beta
	vfmadd231sd		%xmm14, %xmm14, %xmm13 // beta
	vsqrtsd			%xmm13, %xmm13, %xmm13 // beta
	vmovsd			%xmm13, 120(%r11) // pD[0+ps*0]

	vxorpd			%xmm12, %xmm12, %xmm12
	vucomisd		%xmm12, %xmm14
	jbe		111f
	// alpha>0
	vaddsd			%xmm13, %xmm14, %xmm14 // tmp
	vdivsd			%xmm14, %xmm15, %xmm14 // tmp
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm14, %xmm14
	jmp		112f
111: // alpha<=0
	vsubsd			%xmm13, %xmm14, %xmm14 // tmp
112:
	vmulsd			%xmm14, %xmm14, %xmm12 // tmp*tmp
	vaddsd			%xmm12, %xmm15, %xmm15 // sigma+tmp*tmp
	vaddsd			%xmm12, %xmm12, %xmm12 // 2*tmp*tmp
	vdivsd			%xmm15, %xmm12, %xmm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC02(%rip), %xmm12
#else
	vmovapd			LC02(%rip), %xmm12
#endif
	vdivsd			%xmm14, %xmm12, %xmm14 // tmp
	vmovddup		%xmm14, %xmm14

	vmovsd			%xmm15, 24(%r12) // dD[0]
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm15, %xmm15
	vmovsd			%xmm15, 120(%r13) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			96(%r11), %ymm0
	vmovapd			96(%r11, %r14, 1), %ymm1
	vmovapd			96(%r11, %r14, 2), %ymm2
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	24(%r11), %ymm8
	vbroadcastsd	56(%r11), %ymm9
	vbroadcastsd	88(%r11), %ymm10
	vbroadcastsd	120(%r11), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		32(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm10, %ymm2
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm11, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm11, %ymm2
	vmovsd			%xmm8, 24(%r11)
	vmovsd			%xmm9, 56(%r11)
	vmovsd			%xmm10, 88(%r11)
	vmovsd			%xmm11, 120(%r11)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	24(%r11), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vmovsd			%xmm8, 24(%r11)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vxorpd			%xmm12, %xmm12, %xmm12
	//
	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm15
	//
	vmovapd			32(%r13), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vpermpd			$ 0x55, %ymm0, %ymm13
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vmovapd			64(%r13), %ymm14
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm14
	vpermpd			$ 0xaa, %ymm0, %ymm13
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vbroadcastsd	120(%r13), %ymm14
	vmulpd			%ymm14, %ymm15, %ymm15
	vmulpd			%ymm14, %ymm1, %ymm1
	vmulpd			%ymm14, %ymm2, %ymm2
	vmovapd			96(%r13), %ymm0
	vblendpd		$ 0x7, %ymm15, %ymm0, %ymm0
	vmovapd			%ymm0, 96(%r13)

	movq	ARG2, %r11 // D
	//
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vaddpd			%ymm1, %ymm9, %ymm9
	vaddpd			%ymm2, %ymm10, %ymm10
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
	//
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	24(%r11), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11, %r14, 1), %ymm9
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	56(%r11), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmulpd			%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11, %r14, 1), %ymm9
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	88(%r11), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	120(%r11), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	//
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	24(%r11), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11, %r14, 1), %ymm9
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	56(%r11), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11, %r14, 1), %ymm9
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	88(%r11), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	120(%r11), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	24(%r11), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
//	vpermpd	$ 0x00, %ymm15, %ymm15  // beta

	// fifth column
102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 32(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	addq	$ 128, %r11
	vmovsd			0(%r11, %r14, 1), %xmm14 // alpha
	vmovapd			%xmm15, %xmm13 // beta
	vfmadd231sd		%xmm14, %xmm14, %xmm13 // beta
	vsqrtsd			%xmm13, %xmm13, %xmm13 // beta
	vmovsd			%xmm13, 0(%r11, %r14, 1) // pD[0+ps*0]

	vxorpd			%xmm12, %xmm12, %xmm12
	vucomisd		%xmm12, %xmm14
	jbe		111f
	// alpha>0
	vaddsd			%xmm13, %xmm14, %xmm14 // tmp
	vdivsd			%xmm14, %xmm15, %xmm14 // tmp
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm14, %xmm14
	jmp		112f
111: // alpha<=0
	vsubsd			%xmm13, %xmm14, %xmm14 // tmp
112:
	vmulsd			%xmm14, %xmm14, %xmm12 // tmp*tmp
	vaddsd			%xmm12, %xmm15, %xmm15 // sigma+tmp*tmp
	vaddsd			%xmm12, %xmm12, %xmm12 // 2*tmp*tmp
	vdivsd			%xmm15, %xmm12, %xmm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC02(%rip), %xmm12
#else
	vmovapd			LC02(%rip), %xmm12
#endif
	vdivsd			%xmm14, %xmm12, %xmm14 // tmp
	vmovddup		%xmm14, %xmm14

	vmovsd			%xmm15, 32(%r12) // dD[0]
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm15, %xmm15
	vmovsd			%xmm15, 128(%r13, %r15, 1) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			0(%r11), %ymm0
	vmovapd			0(%r11, %r14, 1), %ymm1
	vmovapd			0(%r11, %r14, 2), %ymm2
	vbroadcastsd	32(%r11, %r14, 1), %ymm8
	vbroadcastsd	64(%r11, %r14, 1), %ymm9
	vbroadcastsd	96(%r11, %r14, 1), %ymm10
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vfmadd231pd		32(%r11), %ymm8, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		32(%r11, %r14, 2), %ymm8, %ymm2
	vfmadd231pd		64(%r11), %ymm9, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		96(%r11), %ymm10, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm10, %ymm2
	vmovsd			%xmm8, 32(%r11, %r14, 1)
	vmovsd			%xmm9, 64(%r11, %r14, 1)
	vmovsd			%xmm10, 96(%r11, %r14, 1)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	0(%r11, %r14, 1), %ymm8
	vbroadcastsd	32(%r11, %r14, 1), %ymm9
	vbroadcastsd	64(%r11, %r14, 1), %ymm10
	vbroadcastsd	96(%r11, %r14, 1), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		32(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm10, %ymm2
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm11, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm11, %ymm2
	vmovsd			%xmm8, 0(%r11, %r14, 1)
	vmovsd			%xmm9, 32(%r11, %r14, 1)
	vmovsd			%xmm10, 64(%r11, %r14, 1)
	vmovsd			%xmm11, 96(%r11, %r14, 1)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	0(%r11, %r14, 1), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vmovsd			%xmm8, 0(%r11, %r14, 1)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vxorpd			%xmm12, %xmm12, %xmm12
	//
	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm15
	//
	vmovapd			32(%r13), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vpermpd			$ 0x55, %ymm0, %ymm13
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vmovapd			64(%r13), %ymm14
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm14
	vpermpd			$ 0xaa, %ymm0, %ymm13
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vmovapd			96(%r13), %ymm14
	vpermpd			$ 0xff, %ymm0, %ymm13
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vbroadcastsd	128(%r13, %r15, 1), %ymm14
	vmulpd			%ymm14, %ymm15, %ymm15
	vmulpd			%ymm14, %ymm1, %ymm1
	vmulpd			%ymm14, %ymm2, %ymm2
//	vmovapd			128(%r13), %ymm0
//	vblendpd		$ 0xf, %ymm15, %ymm0, %ymm15
	vmovapd			%ymm15, 128(%r13)

	vxorpd			%ymm15, %ymm15, %ymm15
	vblendpd		$ 0x1, %ymm15, %ymm1, %ymm1

	movq	ARG2, %r11 // D
	//
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
	//
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vaddpd			%ymm1, %ymm9, %ymm9
	vaddpd			%ymm2, %ymm10, %ymm10
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11, %r14, 1), %ymm9
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	32(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11, %r14, 1), %ymm9
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	64(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmulpd			%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	96(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	//
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	0(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11, %r14, 1), %ymm9
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	32(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11, %r14, 1), %ymm9
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	64(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	96(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	//
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	0(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
	vpermpd	$ 0x55, %ymm15, %ymm15  // beta

	// sixth column
102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 40(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	addq	$ 128, %r11
	vmovsd			40(%r11, %r14, 1), %xmm14 // alpha
	vmovapd			%xmm15, %xmm13 // beta
	vfmadd231sd		%xmm14, %xmm14, %xmm13 // beta
	vsqrtsd			%xmm13, %xmm13, %xmm13 // beta
	vmovsd			%xmm13, 40(%r11, %r14, 1) // pD[0+ps*0]

	vxorpd			%xmm12, %xmm12, %xmm12
	vucomisd		%xmm12, %xmm14
	jbe		111f
	// alpha>0
	vaddsd			%xmm13, %xmm14, %xmm14 // tmp
	vdivsd			%xmm14, %xmm15, %xmm14 // tmp
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm14, %xmm14
	jmp		112f
111: // alpha<=0
	vsubsd			%xmm13, %xmm14, %xmm14 // tmp
112:
	vmulsd			%xmm14, %xmm14, %xmm12 // tmp*tmp
	vaddsd			%xmm12, %xmm15, %xmm15 // sigma+tmp*tmp
	vaddsd			%xmm12, %xmm12, %xmm12 // 2*tmp*tmp
	vdivsd			%xmm15, %xmm12, %xmm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC02(%rip), %xmm12
#else
	vmovapd			LC02(%rip), %xmm12
#endif
	vdivsd			%xmm14, %xmm12, %xmm14 // tmp
	vmovddup		%xmm14, %xmm14

	vmovsd			%xmm15, 40(%r12) // dD[0]
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm15, %xmm15
	vmovsd			%xmm15, 168(%r13, %r15, 1) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			32(%r11), %ymm0
	vmovapd			32(%r11, %r14, 1), %ymm1
	vmovapd			32(%r11, %r14, 2), %ymm2
	vbroadcastsd	72(%r11, %r14, 1), %ymm9
	vbroadcastsd	104(%r11, %r14, 1), %ymm10
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vfmadd231pd		64(%r11), %ymm9, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		96(%r11), %ymm10, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm10, %ymm2
	vmovsd			%xmm9, 72(%r11, %r14, 1)
	vmovsd			%xmm10, 104(%r11, %r14, 1)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	8(%r11, %r14, 1), %ymm8
	vbroadcastsd	40(%r11, %r14, 1), %ymm9
	vbroadcastsd	72(%r11, %r14, 1), %ymm10
	vbroadcastsd	104(%r11, %r14, 1), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		32(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm10, %ymm2
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm11, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm11, %ymm2
	vmovsd			%xmm8, 8(%r11, %r14, 1)
	vmovsd			%xmm9, 40(%r11, %r14, 1)
	vmovsd			%xmm10, 72(%r11, %r14, 1)
	vmovsd			%xmm11, 104(%r11, %r14, 1)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	8(%r11, %r14, 1), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vmovsd			%xmm8, 8(%r11, %r14, 1)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vxorpd			%xmm12, %xmm12, %xmm12
	//
	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm15
	//
	vmovapd			32(%r13), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vpermpd			$ 0x55, %ymm0, %ymm13
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vmovapd			64(%r13), %ymm14
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm14
	vpermpd			$ 0xaa, %ymm0, %ymm13
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vmovapd			96(%r13), %ymm14
	vpermpd			$ 0xff, %ymm0, %ymm13
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vmovapd			128(%r13), %ymm14
	vmovapd			128(%r13, %r15, 1), %ymm11
	vblendpd		$ 0x1, %ymm11, %ymm12, %ymm11
	vpermpd			$ 0x00, %ymm1, %ymm13 // vv
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmulpd			%ymm11, %ymm13, %ymm11
	//
	vbroadcastsd	168(%r13, %r15, 1), %ymm14
	vmulpd			%ymm14, %ymm15, %ymm15
	vmulpd			%ymm14, %ymm11, %ymm11
	vmulpd			%ymm14, %ymm1, %ymm1
	vmulpd			%ymm14, %ymm2, %ymm2
	vmovapd			160(%r13, %r15, 1), %ymm0
	vblendpd		$ 0x1, %ymm11, %ymm0, %ymm11
	vmovapd			%ymm15, 160(%r13)
	vmovapd			%ymm11, 160(%r13, %r15, 1)

	vxorpd			%ymm15, %ymm15, %ymm15
	vblendpd		$ 0x3, %ymm15, %ymm1, %ymm1

	movq	ARG2, %r11 // D
	//
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
	//
	vmovapd			32(%r11, %r14, 1), %ymm9
	vmovapd			32(%r11, %r14, 2), %ymm10
	vaddpd			%ymm1, %ymm9, %ymm9
	vaddpd			%ymm2, %ymm10, %ymm10
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11, %r14, 1), %ymm9
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	72(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	104(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmulpd			%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	//
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	8(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11, %r14, 1), %ymm9
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	40(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11, %r14, 1), %ymm9
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	72(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	104(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	//
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	8(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
	vpermpd	$ 0xaa, %ymm15, %ymm15  // beta

	// seventh column
102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 40(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	addq	$ 128, %r11
	vmovsd			80(%r11, %r14, 1), %xmm14 // alpha
	vmovapd			%xmm15, %xmm13 // beta
	vfmadd231sd		%xmm14, %xmm14, %xmm13 // beta
	vsqrtsd			%xmm13, %xmm13, %xmm13 // beta
	vmovsd			%xmm13, 80(%r11, %r14, 1) // pD[0+ps*0]

	vxorpd			%xmm12, %xmm12, %xmm12
	vucomisd		%xmm12, %xmm14
	jbe		111f
	// alpha>0
	vaddsd			%xmm13, %xmm14, %xmm14 // tmp
	vdivsd			%xmm14, %xmm15, %xmm14 // tmp
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm14, %xmm14
	jmp		112f
111: // alpha<=0
	vsubsd			%xmm13, %xmm14, %xmm14 // tmp
112:
	vmulsd			%xmm14, %xmm14, %xmm12 // tmp*tmp
	vaddsd			%xmm12, %xmm15, %xmm15 // sigma+tmp*tmp
	vaddsd			%xmm12, %xmm12, %xmm12 // 2*tmp*tmp
	vdivsd			%xmm15, %xmm12, %xmm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC02(%rip), %xmm12
#else
	vmovapd			LC02(%rip), %xmm12
#endif
	vdivsd			%xmm14, %xmm12, %xmm14 // tmp
	vmovddup		%xmm14, %xmm14

	vmovsd			%xmm15, 48(%r12) // dD[0]
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm15, %xmm15
	vmovsd			%xmm15, 208(%r13, %r15, 1) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			64(%r11), %ymm0
	vmovapd			64(%r11, %r14, 1), %ymm1
	vmovapd			64(%r11, %r14, 2), %ymm2
	vbroadcastsd	112(%r11, %r14, 1), %ymm10
	vmulpd			%ymm15, %ymm10, %ymm10
	vfmadd231pd		96(%r11), %ymm10, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm10, %ymm2
	vmovsd			%xmm10, 112(%r11, %r14, 1)
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	16(%r11, %r14, 1), %ymm8
	vbroadcastsd	48(%r11, %r14, 1), %ymm9
	vbroadcastsd	80(%r11, %r14, 1), %ymm10
	vbroadcastsd	112(%r11, %r14, 1), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		32(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm10, %ymm2
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm11, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm11, %ymm2
	vmovsd			%xmm8, 16(%r11, %r14, 1)
	vmovsd			%xmm9, 48(%r11, %r14, 1)
	vmovsd			%xmm10, 80(%r11, %r14, 1)
	vmovsd			%xmm11, 112(%r11, %r14, 1)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	16(%r11, %r14, 1), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vmovsd			%xmm8, 16(%r11, %r14, 1)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vxorpd			%xmm12, %xmm12, %xmm12
	//
//	vpermpd			$ 0x00, %ymm0, %ymm13
	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm15
	//
	vpermpd			$ 0x55, %ymm0, %ymm13
	vmovapd			32(%r13), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vpermpd			$ 0xaa, %ymm0, %ymm13
	vmovapd			64(%r13), %ymm14
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vpermpd			$ 0xff, %ymm0, %ymm13
	vmovapd			96(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vpermpd			$ 0x00, %ymm1, %ymm13 // vv
	vmovapd			128(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			128(%r13, %r15, 1), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm13, %ymm11
	//
	vpermpd			$ 0x55, %ymm1, %ymm13 // vv
	vmovapd			160(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			160(%r13, %r15, 1), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	//
	vbroadcastsd	208(%r13, %r15, 1), %ymm14
	vmulpd			%ymm14, %ymm15, %ymm15
	vmulpd			%ymm14, %ymm11, %ymm11
	vmulpd			%ymm14, %ymm1, %ymm1
	vmulpd			%ymm14, %ymm2, %ymm2
	vmovapd			192(%r13, %r15, 1), %ymm0
	vblendpd		$ 0x3, %ymm11, %ymm0, %ymm11
	vmovapd			%ymm15, 192(%r13)
	vmovapd			%ymm11, 192(%r13, %r15, 1)

	vxorpd			%ymm15, %ymm15, %ymm15
	vblendpd		$ 0x7, %ymm15, %ymm1, %ymm1

	movq	ARG2, %r11 // D
	//
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
	//
	vmovapd			64(%r11, %r14, 1), %ymm9
	vmovapd			64(%r11, %r14, 2), %ymm10
	vaddpd			%ymm1, %ymm9, %ymm9
	vaddpd			%ymm2, %ymm10, %ymm10
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	112(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vxorpd			%ymm15, %ymm15, %ymm15
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	//
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	16(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11, %r14, 1), %ymm9
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	48(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11, %r14, 1), %ymm9
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	80(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11, %r14, 1), %ymm9
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	112(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	//
	vmovapd			0(%r11, %r14, 1), %ymm9
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	16(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
	vpermpd	$ 0xff, %ymm15, %ymm15  // beta

	// eight column
102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 40(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	addq	$ 128, %r11
	vmovsd			120(%r11, %r14, 1), %xmm14 // alpha
	vmovapd			%xmm15, %xmm13 // beta
	vfmadd231sd		%xmm14, %xmm14, %xmm13 // beta
	vsqrtsd			%xmm13, %xmm13, %xmm13 // beta
	vmovsd			%xmm13, 120(%r11, %r14, 1) // pD[0+ps*0]

	vxorpd			%xmm12, %xmm12, %xmm12
	vucomisd		%xmm12, %xmm14
	jbe		111f
	// alpha>0
	vaddsd			%xmm13, %xmm14, %xmm14 // tmp
	vdivsd			%xmm14, %xmm15, %xmm14 // tmp
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm14, %xmm14
	jmp		112f
111: // alpha<=0
	vsubsd			%xmm13, %xmm14, %xmm14 // tmp
112:
	vmulsd			%xmm14, %xmm14, %xmm12 // tmp*tmp
	vaddsd			%xmm12, %xmm15, %xmm15 // sigma+tmp*tmp
	vaddsd			%xmm12, %xmm12, %xmm12 // 2*tmp*tmp
	vdivsd			%xmm15, %xmm12, %xmm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC02(%rip), %xmm12
#else
	vmovapd			LC02(%rip), %xmm12
#endif
	vdivsd			%xmm14, %xmm12, %xmm14 // tmp
	vmovddup		%xmm14, %xmm14

	vmovsd			%xmm15, 56(%r12) // dD[0]
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm15, %xmm15
	vmovsd			%xmm15, 248(%r13, %r15, 1) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			96(%r11), %ymm0
	vmovapd			96(%r11, %r14, 1), %ymm1
	vmovapd			96(%r11, %r14, 2), %ymm2
	movq	ARG1, %r10 // n
	subl	$ 8, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	24(%r11, %r14, 1), %ymm8
	vbroadcastsd	56(%r11, %r14, 1), %ymm9
	vbroadcastsd	88(%r11, %r14, 1), %ymm10
	vbroadcastsd	120(%r11, %r14, 1), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		32(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm10, %ymm2
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm11, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm11, %ymm2
	vmovsd			%xmm8, 24(%r11, %r14, 1)
	vmovsd			%xmm9, 56(%r11, %r14, 1)
	vmovsd			%xmm10, 88(%r11, %r14, 1)
	vmovsd			%xmm11, 120(%r11, %r14, 1)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	24(%r11, %r14, 1), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vmovsd			%xmm8, 24(%r11, %r14, 1)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vxorpd			%xmm12, %xmm12, %xmm12
	//
//	vpermpd			$ 0x00, %ymm0, %ymm13
	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm15
	//
	vpermpd			$ 0x55, %ymm0, %ymm13
	vmovapd			32(%r13), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vpermpd			$ 0xaa, %ymm0, %ymm13
	vmovapd			64(%r13), %ymm14
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vpermpd			$ 0xff, %ymm0, %ymm13
	vmovapd			96(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vpermpd			$ 0x00, %ymm1, %ymm13
	vmovapd			128(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			128(%r13, %r15, 1), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm13, %ymm11
	//
	vpermpd			$ 0x55, %ymm1, %ymm13
	vmovapd			160(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			160(%r13, %r15, 1), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	//
	vpermpd			$ 0xaa, %ymm1, %ymm13
	vmovapd			192(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			192(%r13, %r15, 1), %ymm14
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	//
	vbroadcastsd	248(%r13, %r15, 1), %ymm14
	vmulpd			%ymm14, %ymm15, %ymm15
	vmulpd			%ymm14, %ymm11, %ymm11
//	vmulpd			%ymm14, %ymm1, %ymm1
	vmulpd			%ymm14, %ymm2, %ymm2
	vmovapd			224(%r13, %r15, 1), %ymm0
	vblendpd		$ 0x7, %ymm11, %ymm0, %ymm11
	vmovapd			%ymm15, 224(%r13)
	vmovapd			%ymm11, 224(%r13, %r15, 1)

//	vxorpd			%ymm15, %ymm15, %ymm15
//	vblendpd		$ 0xf, %ymm15, %ymm1, %ymm1

	movq	ARG2, %r11 // D
	//
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
	//
	vmovapd			96(%r11, %r14, 2), %ymm10
	vaddpd			%ymm2, %ymm10, %ymm10
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
	//
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	24(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	56(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmulpd			%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	88(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	120(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	//
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	24(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	56(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	88(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	120(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	//
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	24(%r11, %r14, 1), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
//	vpermpd	$ 0x00, %ymm15, %ymm15  // beta

	// ninth column
102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 40(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	addq	$ 256, %r11
	vmovsd			0(%r11, %r14, 2), %xmm14 // alpha
	vmovapd			%xmm15, %xmm13 // beta
	vfmadd231sd		%xmm14, %xmm14, %xmm13 // beta
	vsqrtsd			%xmm13, %xmm13, %xmm13 // beta
	vmovsd			%xmm13, 0(%r11, %r14, 2) // pD[0+ps*0]

	vxorpd			%xmm12, %xmm12, %xmm12
	vucomisd		%xmm12, %xmm14
	jbe		111f
	// alpha>0
	vaddsd			%xmm13, %xmm14, %xmm14 // tmp
	vdivsd			%xmm14, %xmm15, %xmm14 // tmp
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm14, %xmm14
	jmp		112f
111: // alpha<=0
	vsubsd			%xmm13, %xmm14, %xmm14 // tmp
112:
	vmulsd			%xmm14, %xmm14, %xmm12 // tmp*tmp
	vaddsd			%xmm12, %xmm15, %xmm15 // sigma+tmp*tmp
	vaddsd			%xmm12, %xmm12, %xmm12 // 2*tmp*tmp
	vdivsd			%xmm15, %xmm12, %xmm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC02(%rip), %xmm12
#else
	vmovapd			LC02(%rip), %xmm12
#endif
	vdivsd			%xmm14, %xmm12, %xmm14 // tmp
	vmovddup		%xmm14, %xmm14

	vmovsd			%xmm15, 64(%r12) // dD[0]
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm15, %xmm15
	vmovsd			%xmm15, 256(%r13, %r15, 2) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			0(%r11), %ymm0
	vmovapd			0(%r11, %r14, 1), %ymm1
	vmovapd			0(%r11, %r14, 2), %ymm2
	vbroadcastsd	32(%r11, %r14, 2), %ymm8
	vbroadcastsd	64(%r11, %r14, 2), %ymm9
	vbroadcastsd	96(%r11, %r14, 2), %ymm10
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vfmadd231pd		32(%r11), %ymm8, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		32(%r11, %r14, 2), %ymm8, %ymm2
	vfmadd231pd		64(%r11), %ymm9, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		96(%r11), %ymm10, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm10, %ymm2
	vmovsd			%xmm8, 32(%r11, %r14, 2)
	vmovsd			%xmm9, 64(%r11, %r14, 2)
	vmovsd			%xmm10, 96(%r11, %r14, 2)
	movq	ARG1, %r10 // n
	subl	$ 12, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	0(%r11, %r14, 2), %ymm8
	vbroadcastsd	32(%r11, %r14, 2), %ymm9
	vbroadcastsd	64(%r11, %r14, 2), %ymm10
	vbroadcastsd	96(%r11, %r14, 2), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		32(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm10, %ymm2
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm11, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm11, %ymm2
	vmovsd			%xmm8, 0(%r11, %r14, 2)
	vmovsd			%xmm9, 32(%r11, %r14, 2)
	vmovsd			%xmm10, 64(%r11, %r14, 2)
	vmovsd			%xmm11, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	0(%r11, %r14, 2), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vmovsd			%xmm8, 0(%r11, %r14, 2)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vxorpd			%xmm12, %xmm12, %xmm12
	//
//	vpermpd			$ 0x00, %ymm0, %ymm13
	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm15
	//
	vpermpd			$ 0x55, %ymm0, %ymm13
	vmovapd			32(%r13), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vpermpd			$ 0xaa, %ymm0, %ymm13
	vmovapd			64(%r13), %ymm14
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vpermpd			$ 0xff, %ymm0, %ymm13
	vmovapd			96(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vpermpd			$ 0x00, %ymm1, %ymm13
	vmovapd			128(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			128(%r13, %r15, 1), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm13, %ymm11
	//
	vpermpd			$ 0x55, %ymm1, %ymm13
	vmovapd			160(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			160(%r13, %r15, 1), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	//
	vpermpd			$ 0xaa, %ymm1, %ymm13
	vmovapd			192(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			192(%r13, %r15, 1), %ymm14
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	//
	vpermpd			$ 0xff, %ymm1, %ymm13
	vmovapd			224(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			224(%r13, %r15, 1), %ymm14
//	vblendpd		$ 0xf, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	//
	vbroadcastsd	256(%r13, %r15, 2), %ymm14
	vmulpd			%ymm14, %ymm15, %ymm15
	vmulpd			%ymm14, %ymm11, %ymm11
//	vmulpd			%ymm14, %ymm1, %ymm1
	vmulpd			%ymm14, %ymm2, %ymm2
//	vmovapd			224(%r13, %r15, 1), %ymm0
//	vblendpd		$ 0xf, %ymm11, %ymm0, %ymm11
	vmovapd			%ymm15, 256(%r13)
	vmovapd			%ymm11, 256(%r13, %r15, 1)

	vxorpd			%ymm15, %ymm15, %ymm15
	vblendpd		$ 0x1, %ymm15, %ymm2, %ymm2

	movq	ARG2, %r11 // D
	//
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
	//
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
	//
	vmovapd			0(%r11, %r14, 2), %ymm10
	vaddpd			%ymm2, %ymm10, %ymm10
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	32(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	64(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmulpd			%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	96(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	//
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	0(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	32(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	64(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	96(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	//
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	0(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
	vpermpd	$ 0x55, %ymm15, %ymm15  // beta

	// tenth column
102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 40(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	addq	$ 256, %r11
	vmovsd			40(%r11, %r14, 2), %xmm14 // alpha
	vmovapd			%xmm15, %xmm13 // beta
	vfmadd231sd		%xmm14, %xmm14, %xmm13 // beta
	vsqrtsd			%xmm13, %xmm13, %xmm13 // beta
	vmovsd			%xmm13, 40(%r11, %r14, 2) // pD[0+ps*0]

	vxorpd			%xmm12, %xmm12, %xmm12
	vucomisd		%xmm12, %xmm14
	jbe		111f
	// alpha>0
	vaddsd			%xmm13, %xmm14, %xmm14 // tmp
	vdivsd			%xmm14, %xmm15, %xmm14 // tmp
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm14, %xmm14
	jmp		112f
111: // alpha<=0
	vsubsd			%xmm13, %xmm14, %xmm14 // tmp
112:
	vmulsd			%xmm14, %xmm14, %xmm12 // tmp*tmp
	vaddsd			%xmm12, %xmm15, %xmm15 // sigma+tmp*tmp
	vaddsd			%xmm12, %xmm12, %xmm12 // 2*tmp*tmp
	vdivsd			%xmm15, %xmm12, %xmm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC02(%rip), %xmm12
#else
	vmovapd			LC02(%rip), %xmm12
#endif
	vdivsd			%xmm14, %xmm12, %xmm14 // tmp
	vmovddup		%xmm14, %xmm14

	vmovsd			%xmm15, 72(%r12) // dD[0]
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm15, %xmm15
	vmovsd			%xmm15, 296(%r13, %r15, 2) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			32(%r11), %ymm0
	vmovapd			32(%r11, %r14, 1), %ymm1
	vmovapd			32(%r11, %r14, 2), %ymm2
	vbroadcastsd	72(%r11, %r14, 2), %ymm9
	vbroadcastsd	104(%r11, %r14, 2), %ymm10
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vfmadd231pd		64(%r11), %ymm9, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		96(%r11), %ymm10, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm10, %ymm2
	vmovsd			%xmm9, 72(%r11, %r14, 2)
	vmovsd			%xmm10, 104(%r11, %r14, 2)
	movq	ARG1, %r10 // n
	subl	$ 12, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	8(%r11, %r14, 2), %ymm8
	vbroadcastsd	40(%r11, %r14, 2), %ymm9
	vbroadcastsd	72(%r11, %r14, 2), %ymm10
	vbroadcastsd	104(%r11, %r14, 2), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		32(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm10, %ymm2
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm11, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm11, %ymm2
	vmovsd			%xmm8, 8(%r11, %r14, 2)
	vmovsd			%xmm9, 40(%r11, %r14, 2)
	vmovsd			%xmm10, 72(%r11, %r14, 2)
	vmovsd			%xmm11, 104(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	8(%r11, %r14, 2), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vmovsd			%xmm8, 8(%r11, %r14, 2)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vxorpd			%xmm12, %xmm12, %xmm12
	//
//	vpermpd			$ 0x00, %ymm0, %ymm13
	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm15
	//
	vpermpd			$ 0x55, %ymm0, %ymm13
	vmovapd			32(%r13), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vpermpd			$ 0xaa, %ymm0, %ymm13
	vmovapd			64(%r13), %ymm14
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vpermpd			$ 0xff, %ymm0, %ymm13
	vmovapd			96(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vpermpd			$ 0x00, %ymm1, %ymm13
	vmovapd			128(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			128(%r13, %r15, 1), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm13, %ymm11
	//
	vpermpd			$ 0x55, %ymm1, %ymm13
	vmovapd			160(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			160(%r13, %r15, 1), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	//
	vpermpd			$ 0xaa, %ymm1, %ymm13
	vmovapd			192(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			192(%r13, %r15, 1), %ymm14
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	//
	vpermpd			$ 0xff, %ymm1, %ymm13
	vmovapd			224(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			224(%r13, %r15, 1), %ymm14
//	vblendpd		$ 0xf, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	//
	vpermpd			$ 0x00, %ymm2, %ymm13
	vmovapd			256(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			256(%r13, %r15, 1), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	vmovapd			256(%r13, %r15, 2), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm13, %ymm10
	//
	vbroadcastsd	296(%r13, %r15, 2), %ymm14
	vmulpd			%ymm14, %ymm15, %ymm15
	vmulpd			%ymm14, %ymm11, %ymm11
	vmulpd			%ymm14, %ymm10, %ymm10
	vmulpd			%ymm14, %ymm2, %ymm2
	vmovapd			288(%r13, %r15, 2), %ymm0
	vblendpd		$ 0x1, %ymm10, %ymm0, %ymm10
	vmovapd			%ymm15, 288(%r13)
	vmovapd			%ymm11, 288(%r13, %r15, 1)
	vmovapd			%ymm10, 288(%r13, %r15, 2)

	vxorpd			%ymm15, %ymm15, %ymm15
	vblendpd		$ 0x3, %ymm15, %ymm2, %ymm2

	movq	ARG2, %r11 // D
	//
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
	//
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
	//
	vmovapd			32(%r11, %r14, 2), %ymm10
	vaddpd			%ymm2, %ymm10, %ymm10
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	72(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	104(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vmulpd			%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	//
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	8(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	40(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	72(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	104(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	//
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	8(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
	vpermpd	$ 0xaa, %ymm15, %ymm15  // beta

	// eleventh column
102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 40(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	addq	$ 256, %r11
	vmovsd			80(%r11, %r14, 2), %xmm14 // alpha
	vmovapd			%xmm15, %xmm13 // beta
	vfmadd231sd		%xmm14, %xmm14, %xmm13 // beta
	vsqrtsd			%xmm13, %xmm13, %xmm13 // beta
	vmovsd			%xmm13, 80(%r11, %r14, 2) // pD[0+ps*0]

	vxorpd			%xmm12, %xmm12, %xmm12
	vucomisd		%xmm12, %xmm14
	jbe		111f
	// alpha>0
	vaddsd			%xmm13, %xmm14, %xmm14 // tmp
	vdivsd			%xmm14, %xmm15, %xmm14 // tmp
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm14, %xmm14
	jmp		112f
111: // alpha<=0
	vsubsd			%xmm13, %xmm14, %xmm14 // tmp
112:
	vmulsd			%xmm14, %xmm14, %xmm12 // tmp*tmp
	vaddsd			%xmm12, %xmm15, %xmm15 // sigma+tmp*tmp
	vaddsd			%xmm12, %xmm12, %xmm12 // 2*tmp*tmp
	vdivsd			%xmm15, %xmm12, %xmm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC02(%rip), %xmm12
#else
	vmovapd			LC02(%rip), %xmm12
#endif
	vdivsd			%xmm14, %xmm12, %xmm14 // tmp
	vmovddup		%xmm14, %xmm14

	vmovsd			%xmm15, 80(%r12) // dD[0]
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm15, %xmm15
	vmovsd			%xmm15, 336(%r13, %r15, 2) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			64(%r11), %ymm0
	vmovapd			64(%r11, %r14, 1), %ymm1
	vmovapd			64(%r11, %r14, 2), %ymm2
	vbroadcastsd	112(%r11, %r14, 2), %ymm10
	vmulpd			%ymm15, %ymm10, %ymm10
	vfmadd231pd		96(%r11), %ymm10, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm10, %ymm2
	vmovsd			%xmm10, 112(%r11, %r14, 2)
	movq	ARG1, %r10 // n
	subl	$ 12, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	16(%r11, %r14, 2), %ymm8
	vbroadcastsd	48(%r11, %r14, 2), %ymm9
	vbroadcastsd	80(%r11, %r14, 2), %ymm10
	vbroadcastsd	112(%r11, %r14, 2), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		32(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm10, %ymm2
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm11, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm11, %ymm2
	vmovsd			%xmm8, 16(%r11, %r14, 2)
	vmovsd			%xmm9, 48(%r11, %r14, 2)
	vmovsd			%xmm10, 80(%r11, %r14, 2)
	vmovsd			%xmm11, 112(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	16(%r11, %r14, 2), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vmovsd			%xmm8, 16(%r11, %r14, 2)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vxorpd			%xmm12, %xmm12, %xmm12
	//
//	vpermpd			$ 0x00, %ymm0, %ymm13
	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm15
	//
	vpermpd			$ 0x55, %ymm0, %ymm13
	vmovapd			32(%r13), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vpermpd			$ 0xaa, %ymm0, %ymm13
	vmovapd			64(%r13), %ymm14
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vpermpd			$ 0xff, %ymm0, %ymm13
	vmovapd			96(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vpermpd			$ 0x00, %ymm1, %ymm13
	vmovapd			128(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			128(%r13, %r15, 1), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm13, %ymm11
	//
	vpermpd			$ 0x55, %ymm1, %ymm13
	vmovapd			160(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			160(%r13, %r15, 1), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	//
	vpermpd			$ 0xaa, %ymm1, %ymm13
	vmovapd			192(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			192(%r13, %r15, 1), %ymm14
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	//
	vpermpd			$ 0xff, %ymm1, %ymm13
	vmovapd			224(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			224(%r13, %r15, 1), %ymm14
//	vblendpd		$ 0xf, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	//
	vpermpd			$ 0x00, %ymm2, %ymm13
	vmovapd			256(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			256(%r13, %r15, 1), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	vmovapd			256(%r13, %r15, 2), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm13, %ymm10
	//
	vpermpd			$ 0x55, %ymm2, %ymm13
	vmovapd			288(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			288(%r13, %r15, 1), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	vmovapd			288(%r13, %r15, 2), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm10
	//
	vbroadcastsd	336(%r13, %r15, 2), %ymm14
	vmulpd			%ymm14, %ymm15, %ymm15
	vmulpd			%ymm14, %ymm11, %ymm11
	vmulpd			%ymm14, %ymm10, %ymm10
	vmulpd			%ymm14, %ymm2, %ymm2
	vmovapd			320(%r13, %r15, 2), %ymm0
	vblendpd		$ 0x3, %ymm10, %ymm0, %ymm10
	vmovapd			%ymm15, 320(%r13)
	vmovapd			%ymm11, 320(%r13, %r15, 1)
	vmovapd			%ymm10, 320(%r13, %r15, 2)

	vxorpd			%ymm15, %ymm15, %ymm15
	vblendpd		$ 0x7, %ymm15, %ymm2, %ymm2

	movq	ARG2, %r11 // D
	//
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
	//
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
	//
	vmovapd			64(%r11, %r14, 2), %ymm10
	vaddpd			%ymm2, %ymm10, %ymm10
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	112(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vxorpd			%ymm15, %ymm15, %ymm15
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	//
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	16(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	//
	vmovapd			32(%r11, %r14, 2), %ymm10
	vbroadcastsd	48(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 32(%r11, %r14, 2)
	//
	vmovapd			64(%r11, %r14, 2), %ymm10
	vbroadcastsd	80(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 64(%r11, %r14, 2)
	//
	vmovapd			96(%r11, %r14, 2), %ymm10
	vbroadcastsd	112(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 96(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	//
	vmovapd			0(%r11, %r14, 2), %ymm10
	vbroadcastsd	16(%r11, %r14, 2), %ymm14
	vfmadd231pd		%ymm2, %ymm14, %ymm10
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vmovapd			%ymm10, 0(%r11, %r14, 2)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
	vpermpd	$ 0xff, %ymm15, %ymm15  // beta

	// twelveth
102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 40(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	addq	$ 256, %r11
	vmovsd			120(%r11, %r14, 2), %xmm14 // alpha
	vmovapd			%xmm15, %xmm13 // beta
	vfmadd231sd		%xmm14, %xmm14, %xmm13 // beta
	vsqrtsd			%xmm13, %xmm13, %xmm13 // beta
	vmovsd			%xmm13, 120(%r11, %r14, 2) // pD[0+ps*0]

	vxorpd			%xmm12, %xmm12, %xmm12
	vucomisd		%xmm12, %xmm14
	jbe		111f
	// alpha>0
	vaddsd			%xmm13, %xmm14, %xmm14 // tmp
	vdivsd			%xmm14, %xmm15, %xmm14 // tmp
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm14, %xmm14
	jmp		112f
111: // alpha<=0
	vsubsd			%xmm13, %xmm14, %xmm14 // tmp
112:
	vmulsd			%xmm14, %xmm14, %xmm12 // tmp*tmp
	vaddsd			%xmm12, %xmm15, %xmm15 // sigma+tmp*tmp
	vaddsd			%xmm12, %xmm12, %xmm12 // 2*tmp*tmp
	vdivsd			%xmm15, %xmm12, %xmm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC02(%rip), %xmm12
#else
	vmovapd			LC02(%rip), %xmm12
#endif
	vdivsd			%xmm14, %xmm12, %xmm14 // tmp
	vmovddup		%xmm14, %xmm14

	vmovsd			%xmm15, 88(%r12) // dD[0]
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm15, %xmm15
	vmovsd			%xmm15, 376(%r13, %r15, 2) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			96(%r11), %ymm0
	vmovapd			96(%r11, %r14, 1), %ymm1
	vmovapd			96(%r11, %r14, 2), %ymm2
	movq	ARG1, %r10 // n
	subl	$ 12, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	24(%r11, %r14, 2), %ymm8
	vbroadcastsd	56(%r11, %r14, 2), %ymm9
	vbroadcastsd	88(%r11, %r14, 2), %ymm10
	vbroadcastsd	120(%r11, %r14, 2), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		32(%r11, %r14, 2), %ymm9, %ymm2
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		64(%r11, %r14, 2), %ymm10, %ymm2
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm11, %ymm1
	vfmadd231pd		96(%r11, %r14, 2), %ymm11, %ymm2
	vmovsd			%xmm8, 24(%r11, %r14, 2)
	vmovsd			%xmm9, 56(%r11, %r14, 2)
	vmovsd			%xmm10, 88(%r11, %r14, 2)
	vmovsd			%xmm11, 120(%r11, %r14, 2)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	24(%r11, %r14, 2), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		0(%r11, %r14, 2), %ymm8, %ymm2
	vmovsd			%xmm8, 24(%r11, %r14, 2)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vxorpd			%xmm12, %xmm12, %xmm12
	//
//	vpermpd			$ 0x00, %ymm0, %ymm13
	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm15
	//
	vpermpd			$ 0x55, %ymm0, %ymm13
	vmovapd			32(%r13), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vpermpd			$ 0xaa, %ymm0, %ymm13
	vmovapd			64(%r13), %ymm14
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vpermpd			$ 0xff, %ymm0, %ymm13
	vmovapd			96(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	//
	vpermpd			$ 0x00, %ymm1, %ymm13
	vmovapd			128(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			128(%r13, %r15, 1), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm13, %ymm11
	//
	vpermpd			$ 0x55, %ymm1, %ymm13
	vmovapd			160(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			160(%r13, %r15, 1), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	//
	vpermpd			$ 0xaa, %ymm1, %ymm13
	vmovapd			192(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			192(%r13, %r15, 1), %ymm14
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	//
	vpermpd			$ 0xff, %ymm1, %ymm13
	vmovapd			224(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			224(%r13, %r15, 1), %ymm14
//	vblendpd		$ 0xf, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	//
	vpermpd			$ 0x00, %ymm2, %ymm13
	vmovapd			256(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			256(%r13, %r15, 1), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	vmovapd			256(%r13, %r15, 2), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm13, %ymm10
	//
	vpermpd			$ 0x55, %ymm2, %ymm13
	vmovapd			288(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			288(%r13, %r15, 1), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	vmovapd			288(%r13, %r15, 2), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm10
	//
	vpermpd			$ 0xaa, %ymm2, %ymm13
	vmovapd			320(%r13), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vmovapd			320(%r13, %r15, 1), %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm11
	vmovapd			320(%r13, %r15, 2), %ymm14
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm14
	vfmadd231pd		%ymm14, %ymm13, %ymm10
	//
	vbroadcastsd	376(%r13, %r15, 2), %ymm14
	vmulpd			%ymm14, %ymm15, %ymm15
	vmulpd			%ymm14, %ymm11, %ymm11
	vmulpd			%ymm14, %ymm10, %ymm10
//	vmulpd			%ymm14, %ymm2, %ymm2
	vmovapd			352(%r13, %r15, 2), %ymm0
	vblendpd		$ 0x7, %ymm10, %ymm0, %ymm10
	vmovapd			%ymm15, 352(%r13)
	vmovapd			%ymm11, 352(%r13, %r15, 1)
	vmovapd			%ymm10, 352(%r13, %r15, 2)

102:

	EPILOGUE
	
	ret

#if defined(OS_LINUX)
	.size	kernel_dgelqf_pd_dlarft12_12_lib4, .-kernel_dgelqf_pd_dlarft12_12_lib4
#endif





//                                      1      2           3        4           5
// void kernel_dgelqf_pd_dlarft4_8_lib4(int n, double *pD, int sdd, double *dD, double *pT)

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dgelqf_pd_dlarft4_8_lib4
	.type kernel_dgelqf_pd_dlarft4_8_lib4, @function
kernel_dgelqf_pd_dlarft4_8_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dgelqf_pd_dlarft4_8_lib4
_kernel_dgelqf_pd_dlarft4_8_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dgelqf_pd_dlarft4_8_lib4
	.def kernel_dgelqf_pd_dlarft4_8_lib4; .scl 2; .type 32; .endef
kernel_dgelqf_pd_dlarft4_8_lib4:
#endif
	
	PROLOGUE

	// zero T

	movq	ARG5, %r10 // T

	vxorpd			%ymm15, %ymm15, %ymm15
	vmovapd			%ymm15, 0(%r10)
	vmovapd			%ymm15, 32(%r10)
	vmovapd			%ymm15, 64(%r10)
	vmovapd			%ymm15, 96(%r10)

	// first column

	movq	ARG2, %r11 // D
	movq	ARG3, %r14 // sdd
	sall	$ 5, %r14d
	movq	ARG4, %r12 // dD
	movq	ARG5, %r13 // T

	vxorpd			%xmm15, %xmm15, %xmm15
	movq	ARG1, %r10 // n
	subl	$ 1, %r10d
	addq	$ 32, %r11
100:
	vmovsd			0(%r11), %xmm14
	vfmadd231sd		%xmm14, %xmm14, %xmm15
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		100b

	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		101f
	vmovsd			%xmm14, 0(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	vmovsd			0(%r11), %xmm14 // alpha
	vmovapd			%xmm15, %xmm13 // beta
	vfmadd231sd		%xmm14, %xmm14, %xmm13 // beta
	vsqrtsd			%xmm13, %xmm13, %xmm13 // beta
	vmovsd			%xmm13, 0(%r11) // pD[0+ps*0]

	vxorpd			%xmm12, %xmm12, %xmm12
	vucomisd		%xmm12, %xmm14
	jbe		111f
	// alpha>0
	vaddsd			%xmm13, %xmm14, %xmm14 // tmp
	vdivsd			%xmm14, %xmm15, %xmm14 // tmp
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm14, %xmm14
	jmp		112f
111: // alpha<=0
	vsubsd			%xmm13, %xmm14, %xmm14 // tmp
112:
	vmulsd			%xmm14, %xmm14, %xmm12 // tmp*tmp
	vaddsd			%xmm12, %xmm15, %xmm15 // sigma+tmp*tmp
	vaddsd			%xmm12, %xmm12, %xmm12 // 2*tmp*tmp
	vdivsd			%xmm15, %xmm12, %xmm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC02(%rip), %xmm12
#else
	vmovapd			LC02(%rip), %xmm12
#endif
	vdivsd			%xmm14, %xmm12, %xmm14 // tmp
	vmovddup		%xmm14, %xmm14

	vmovsd			%xmm15, 0(%r12) // dD[0]
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm15, %xmm15
	vmovsd			%xmm15, 0(%r13) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			0(%r11), %ymm0
	vmovapd			0(%r11, %r14, 1), %ymm1
	vbroadcastsd	32(%r11), %ymm8
	vbroadcastsd	64(%r11), %ymm9
	vbroadcastsd	96(%r11), %ymm10
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vfmadd231pd		32(%r11), %ymm8, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		64(%r11), %ymm9, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		96(%r11), %ymm10, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm10, %ymm1
	vmovsd			%xmm8, 32(%r11)
	vmovsd			%xmm9, 64(%r11)
	vmovsd			%xmm10, 96(%r11)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	0(%r11), %ymm8
	vbroadcastsd	32(%r11), %ymm9
	vbroadcastsd	64(%r11), %ymm10
	vbroadcastsd	96(%r11), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm11, %ymm1
	vmovsd			%xmm8, 0(%r11)
	vmovsd			%xmm9, 32(%r11)
	vmovsd			%xmm10, 64(%r11)
	vmovsd			%xmm11, 96(%r11)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	0(%r11), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vmovsd			%xmm8, 0(%r11)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vbroadcastsd	0(%r13), %ymm15
	vmulpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm15, %ymm1, %ymm1

	vxorpd			%ymm15, %ymm15, %ymm15
	vblendpd		$ 0x1, %ymm15, %ymm0, %ymm0

	movq	ARG2, %r11 // D
	//
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r14, 1), %ymm9
	vaddpd			%ymm0, %ymm8, %ymm8
	vaddpd			%ymm1, %ymm9, %ymm9
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	//
	vmovapd			32(%r11), %ymm8
	vmovapd			32(%r11, %r14, 1), %ymm9
	vbroadcastsd	32(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vmovapd			%ymm8, 32(%r11)
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	//
	vmovapd			64(%r11), %ymm8
	vmovapd			64(%r11, %r14, 1), %ymm9
	vbroadcastsd	64(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vmulpd			%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 64(%r11)
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	//
	vmovapd			96(%r11), %ymm8
	vmovapd			96(%r11, %r14, 1), %ymm9
	vbroadcastsd	96(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 96(%r11)
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	//
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r14, 1), %ymm9
	vbroadcastsd	0(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	//
	vmovapd			32(%r11), %ymm8
	vmovapd			32(%r11, %r14, 1), %ymm9
	vbroadcastsd	32(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 32(%r11)
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	//
	vmovapd			64(%r11), %ymm8
	vmovapd			64(%r11, %r14, 1), %ymm9
	vbroadcastsd	64(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 64(%r11)
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	//
	vmovapd			96(%r11), %ymm8
	vmovapd			96(%r11, %r14, 1), %ymm9
	vbroadcastsd	96(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 96(%r11)
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	//
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r14, 1), %ymm9
	vbroadcastsd	0(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
	vpermpd	$ 0x55, %ymm15, %ymm15  // beta

	// second column
102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 8(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	vmovsd			40(%r11), %xmm14 // alpha
	vmovapd			%xmm15, %xmm13 // beta
	vfmadd231sd		%xmm14, %xmm14, %xmm13 // beta
	vsqrtsd			%xmm13, %xmm13, %xmm13 // beta
	vmovsd			%xmm13, 40(%r11) // pD[0+ps*0]

	vxorpd			%xmm12, %xmm12, %xmm12
	vucomisd		%xmm12, %xmm14
	jbe		111f
	// alpha>0
	vaddsd			%xmm13, %xmm14, %xmm14 // tmp
	vdivsd			%xmm14, %xmm15, %xmm14 // tmp
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm14, %xmm14
	jmp		112f
111: // alpha<=0
	vsubsd			%xmm13, %xmm14, %xmm14 // tmp
112:
	vmulsd			%xmm14, %xmm14, %xmm12 // tmp*tmp
	vaddsd			%xmm12, %xmm15, %xmm15 // sigma+tmp*tmp
	vaddsd			%xmm12, %xmm12, %xmm12 // 2*tmp*tmp
	vdivsd			%xmm15, %xmm12, %xmm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC02(%rip), %xmm12
#else
	vmovapd			LC02(%rip), %xmm12
#endif
	vdivsd			%xmm14, %xmm12, %xmm14 // tmp
	vmovddup		%xmm14, %xmm14

	vmovsd			%xmm15, 8(%r12) // dD[0]
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm15, %xmm15
	vmovsd			%xmm15, 40(%r13) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			32(%r11), %ymm0
	vmovapd			32(%r11, %r14, 1), %ymm1
	vbroadcastsd	72(%r11), %ymm9
	vbroadcastsd	104(%r11), %ymm10
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vfmadd231pd		64(%r11), %ymm9, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		96(%r11), %ymm10, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm10, %ymm1
	vmovsd			%xmm9, 72(%r11)
	vmovsd			%xmm10, 104(%r11)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	8(%r11), %ymm8
	vbroadcastsd	40(%r11), %ymm9
	vbroadcastsd	72(%r11), %ymm10
	vbroadcastsd	104(%r11), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm11, %ymm1
	vmovsd			%xmm8, 8(%r11)
	vmovsd			%xmm9, 40(%r11)
	vmovsd			%xmm10, 72(%r11)
	vmovsd			%xmm11, 104(%r11)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	8(%r11), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vmovsd			%xmm8, 8(%r11)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC02(%rip), %ymm12
#else
	vmovapd			LC02(%rip), %ymm12
#endif
	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm0
	vbroadcastsd	40(%r13), %ymm15
	vmulpd			%ymm15, %ymm0, %ymm0
	vmulpd			%ymm15, %ymm1, %ymm1
	vmovsd			%xmm0, 32(%r13)

	vxorpd			%ymm12, %ymm12, %ymm12
	vblendpd		$ 0x3, %ymm12, %ymm0, %ymm0

	movq	ARG2, %r11 // D
	//
	vmovapd			32(%r11), %ymm8
	vmovapd			32(%r11, %r14, 1), %ymm9
	vaddpd			%ymm0, %ymm8, %ymm8
	vaddpd			%ymm1, %ymm9, %ymm9
	vmovapd			%ymm8, 32(%r11)
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	//
	vmovapd			64(%r11), %ymm8
	vmovapd			64(%r11, %r14, 1), %ymm9
	vbroadcastsd	72(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vmovapd			%ymm8, 64(%r11)
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	//
	vmovapd			96(%r11), %ymm8
	vmovapd			96(%r11, %r14, 1), %ymm9
	vbroadcastsd	104(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vmulpd			%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 96(%r11)
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	//
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r14, 1), %ymm9
	vbroadcastsd	8(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	//
	vmovapd			32(%r11), %ymm8
	vmovapd			32(%r11, %r14, 1), %ymm9
	vbroadcastsd	40(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 32(%r11)
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	//
	vmovapd			64(%r11), %ymm8
	vmovapd			64(%r11, %r14, 1), %ymm9
	vbroadcastsd	72(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 64(%r11)
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	//
	vmovapd			96(%r11), %ymm8
	vmovapd			96(%r11, %r14, 1), %ymm9
	vbroadcastsd	104(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 96(%r11)
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r14, 1), %ymm9
	vbroadcastsd	8(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
	vpermpd	$ 0xaa, %ymm15, %ymm15  // beta

	// third column
102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 16(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	vmovsd			80(%r11), %xmm14 // alpha
	vmovapd			%xmm15, %xmm13 // beta
	vfmadd231sd		%xmm14, %xmm14, %xmm13 // beta
	vsqrtsd			%xmm13, %xmm13, %xmm13 // beta
	vmovsd			%xmm13, 80(%r11) // pD[0+ps*0]

	vxorpd			%xmm12, %xmm12, %xmm12
	vucomisd		%xmm12, %xmm14
	jbe		111f
	// alpha>0
	vaddsd			%xmm13, %xmm14, %xmm14 // tmp
	vdivsd			%xmm14, %xmm15, %xmm14 // tmp
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm14, %xmm14
	jmp		112f
111: // alpha<=0
	vsubsd			%xmm13, %xmm14, %xmm14 // tmp
112:
	vmulsd			%xmm14, %xmm14, %xmm12 // tmp*tmp
	vaddsd			%xmm12, %xmm15, %xmm15 // sigma+tmp*tmp
	vaddsd			%xmm12, %xmm12, %xmm12 // 2*tmp*tmp
	vdivsd			%xmm15, %xmm12, %xmm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC02(%rip), %xmm12
#else
	vmovapd			LC02(%rip), %xmm12
#endif
	vdivsd			%xmm14, %xmm12, %xmm14 // tmp
	vmovddup		%xmm14, %xmm14

	vmovsd			%xmm15, 16(%r12) // dD[0]
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm15, %xmm15
	vmovsd			%xmm15, 80(%r13) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			64(%r11), %ymm0
	vmovapd			64(%r11, %r14, 1), %ymm1
	vbroadcastsd	112(%r11), %ymm10
	vmulpd			%ymm15, %ymm10, %ymm10
	vfmadd231pd		96(%r11), %ymm10, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm10, %ymm1
	vmovsd			%xmm10, 112(%r11)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	16(%r11), %ymm8
	vbroadcastsd	48(%r11), %ymm9
	vbroadcastsd	80(%r11), %ymm10
	vbroadcastsd	112(%r11), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm11, %ymm1
	vmovsd			%xmm8, 16(%r11)
	vmovsd			%xmm9, 48(%r11)
	vmovsd			%xmm10, 80(%r11)
	vmovsd			%xmm11, 112(%r11)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	16(%r11), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vmovsd			%xmm8, 16(%r11)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vxorpd			%xmm12, %xmm12, %xmm12
	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm15
	vmovapd			32(%r13), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vpermpd			$ 0x55, %ymm0, %ymm13
	vfmadd231pd		%ymm14, %ymm13, %ymm15
	vblendpd		$ 0x7, %ymm15, %ymm0, %ymm0
	vbroadcastsd	80(%r13), %ymm14
	vmulpd			%ymm14, %ymm0, %ymm0
	vmulpd			%ymm14, %ymm1, %ymm1
	vmovapd			%xmm0, 64(%r13)

	vxorpd			%ymm12, %ymm12, %ymm12
	vblendpd		$ 0x7, %ymm12, %ymm0, %ymm0

	movq	ARG2, %r11 // D
	//
	vmovapd			64(%r11), %ymm8
	vmovapd			64(%r11, %r14, 1), %ymm9
	vaddpd			%ymm0, %ymm8, %ymm8
	vaddpd			%ymm1, %ymm9, %ymm9
	vmovapd			%ymm8, 64(%r11)
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	//
	vmovapd			96(%r11), %ymm8
	vmovapd			96(%r11, %r14, 1), %ymm9
	vbroadcastsd	112(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vxorpd			%ymm15, %ymm15, %ymm15
	vmovapd			%ymm8, 96(%r11)
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	//
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r14, 1), %ymm9
	vbroadcastsd	16(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	//
	vmovapd			32(%r11), %ymm8
	vmovapd			32(%r11, %r14, 1), %ymm9
	vbroadcastsd	48(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 32(%r11)
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	//
	vmovapd			64(%r11), %ymm8
	vmovapd			64(%r11, %r14, 1), %ymm9
	vbroadcastsd	80(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 64(%r11)
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	//
	vmovapd			96(%r11), %ymm8
	vmovapd			96(%r11, %r14, 1), %ymm9
	vbroadcastsd	112(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 96(%r11)
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	vmovapd			0(%r11), %ymm8
	vmovapd			0(%r11, %r14, 1), %ymm9
	vbroadcastsd	16(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
	vpermpd	$ 0xff, %ymm15, %ymm15  // beta

102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 24(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	vmovsd			120(%r11), %xmm14 // alpha
	vmovapd			%xmm15, %xmm13 // beta
	vfmadd231sd		%xmm14, %xmm14, %xmm13 // beta
	vsqrtsd			%xmm13, %xmm13, %xmm13 // beta
	vmovsd			%xmm13, 120(%r11) // pD[0+ps*0]

	vxorpd			%xmm12, %xmm12, %xmm12
	vucomisd		%xmm12, %xmm14
	jbe		111f
	// alpha>0
	vaddsd			%xmm13, %xmm14, %xmm14 // tmp
	vdivsd			%xmm14, %xmm15, %xmm14 // tmp
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm14, %xmm14
	jmp		112f
111: // alpha<=0
	vsubsd			%xmm13, %xmm14, %xmm14 // tmp
112:
	vmulsd			%xmm14, %xmm14, %xmm12 // tmp*tmp
	vaddsd			%xmm12, %xmm15, %xmm15 // sigma+tmp*tmp
	vaddsd			%xmm12, %xmm12, %xmm12 // 2*tmp*tmp
	vdivsd			%xmm15, %xmm12, %xmm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC02(%rip), %xmm12
#else
	vmovapd			LC02(%rip), %xmm12
#endif
	vdivsd			%xmm14, %xmm12, %xmm14 // tmp
	vmovddup		%xmm14, %xmm14

	vmovsd			%xmm15, 24(%r12) // dD[0]
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm15, %xmm15
	vmovsd			%xmm15, 120(%r13) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			96(%r11), %ymm0
	vmovapd			96(%r11, %r14, 1), %ymm1
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	24(%r11), %ymm8
	vbroadcastsd	56(%r11), %ymm9
	vbroadcastsd	88(%r11), %ymm10
	vbroadcastsd	120(%r11), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		32(%r11, %r14, 1), %ymm9, %ymm1
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		64(%r11, %r14, 1), %ymm10, %ymm1
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vfmadd231pd		96(%r11, %r14, 1), %ymm11, %ymm1
	vmovsd			%xmm8, 24(%r11)
	vmovsd			%xmm9, 56(%r11)
	vmovsd			%xmm10, 88(%r11)
	vmovsd			%xmm11, 120(%r11)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	24(%r11), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		0(%r11, %r14, 1), %ymm8, %ymm1
	vmovsd			%xmm8, 24(%r11)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vxorpd			%xmm12, %xmm12, %xmm12

	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm15

	vmovapd			32(%r13), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vpermpd			$ 0x55, %ymm0, %ymm13
	vfmadd231pd		%ymm14, %ymm13, %ymm15

	vmovapd			64(%r13), %ymm14
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm14
	vpermpd			$ 0xaa, %ymm0, %ymm13
	vfmadd231pd		%ymm14, %ymm13, %ymm15

	vbroadcastsd	120(%r13), %ymm14
	vmulpd			%ymm14, %ymm15, %ymm15
	vmulpd			%ymm14, %ymm1, %ymm1
	vmovapd			96(%r13), %ymm0
	vblendpd		$ 0x7, %ymm15, %ymm0, %ymm0
	vmovapd			%ymm0, 96(%r13)

	movq	ARG2, %r11 // D
	//
	vmovapd			96(%r11, %r14, 1), %ymm9
	vaddpd			%ymm1, %ymm9, %ymm9
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	//
	vmovapd			0(%r11, %r14, 1), %ymm9
	vbroadcastsd	24(%r11), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	//
	vmovapd			32(%r11, %r14, 1), %ymm9
	vbroadcastsd	56(%r11), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vmovapd			%ymm9, 32(%r11, %r14, 1)
	//
	vmovapd			64(%r11, %r14, 1), %ymm9
	vbroadcastsd	88(%r11), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vmovapd			%ymm9, 64(%r11, %r14, 1)
	//
	vmovapd			96(%r11, %r14, 1), %ymm9
	vbroadcastsd	120(%r11), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vmovapd			%ymm9, 96(%r11, %r14, 1)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	vmovapd			0(%r11, %r14, 1), %ymm9
	vbroadcastsd	24(%r11), %ymm14
	vfmadd231pd		%ymm1, %ymm14, %ymm9
	vmovapd			%ymm9, 0(%r11, %r14, 1)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:

102:

	EPILOGUE
	
	ret

#if defined(OS_LINUX)
	.size	kernel_dgelqf_pd_dlarft4_8_lib4, .-kernel_dgelqf_pd_dlarft4_8_lib4
#endif





//                                      1      2           3           4
// void kernel_dgelqf_pd_dlarft4_4_lib4(int n, double *pD, double *dD, double *pT, double *beta)

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dgelqf_pd_dlarft4_4_lib4
	.type kernel_dgelqf_pd_dlarft4_4_lib4, @function
kernel_dgelqf_pd_dlarft4_4_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dgelqf_pd_dlarft4_4_lib4
_kernel_dgelqf_pd_dlarft4_4_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dgelqf_pd_dlarft4_4_lib4
	.def kernel_dgelqf_pd_dlarft4_4_lib4; .scl 2; .type 32; .endef
kernel_dgelqf_pd_dlarft4_4_lib4:
#endif
	
	PROLOGUE

	// zero T

	movq	ARG4, %r10 // T

	vxorpd			%ymm15, %ymm15, %ymm15
	vmovapd			%ymm15, 0(%r10)
	vmovapd			%ymm15, 32(%r10)
	vmovapd			%ymm15, 64(%r10)
	vmovapd			%ymm15, 96(%r10)

	// first column

	movq	ARG2, %r11 // D
	movq	ARG3, %r12 // dD
	movq	ARG4, %r13 // T

	vxorpd			%xmm15, %xmm15, %xmm15 // sigma
	movq	ARG1, %r10 // n
	subl	$ 1, %r10d
	addq	$ 32, %r11
100:
	vmovsd			0(%r11), %xmm14
	vfmadd231sd		%xmm14, %xmm14, %xmm15 // sigma
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		100b

	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		101f
	vmovsd			%xmm14, 0(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	vmovsd			0(%r11), %xmm14 // alpha
	vmovapd			%xmm15, %xmm13 // beta
	vfmadd231sd		%xmm14, %xmm14, %xmm13 // beta
	vsqrtsd			%xmm13, %xmm13, %xmm13 // beta
	vmovsd			%xmm13, 0(%r11) // pD[0+ps*0]

	vxorpd			%xmm12, %xmm12, %xmm12
	vucomisd		%xmm12, %xmm14
	jbe		111f
	// alpha>0
	vaddsd			%xmm13, %xmm14, %xmm14 // tmp
	vdivsd			%xmm14, %xmm15, %xmm14 // tmp
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm14, %xmm14
	jmp		112f
111: // alpha<=0
	vsubsd			%xmm13, %xmm14, %xmm14 // tmp
112:
	vmulsd			%xmm14, %xmm14, %xmm12 // tmp*tmp
	vaddsd			%xmm12, %xmm15, %xmm15 // sigma+tmp*tmp
	vaddsd			%xmm12, %xmm12, %xmm12 // 2*tmp*tmp
	vdivsd			%xmm15, %xmm12, %xmm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC02(%rip), %xmm12
#else
	vmovapd			LC02(%rip), %xmm12
#endif
	vdivsd			%xmm14, %xmm12, %xmm14 // tmp
	vmovddup		%xmm14, %xmm14

	vmovsd			%xmm15, 0(%r12) // dD[0]
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm15, %xmm15
	vmovsd			%xmm15, 0(%r13) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			0(%r11), %ymm0
	vbroadcastsd	32(%r11), %ymm8
	vbroadcastsd	64(%r11), %ymm9
	vbroadcastsd	96(%r11), %ymm10
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vfmadd231pd		32(%r11), %ymm8, %ymm0
	vfmadd231pd		64(%r11), %ymm9, %ymm0
	vfmadd231pd		96(%r11), %ymm10, %ymm0
	vmovsd			%xmm8, 32(%r11)
	vmovsd			%xmm9, 64(%r11)
	vmovsd			%xmm10, 96(%r11)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	0(%r11), %ymm8
	vbroadcastsd	32(%r11), %ymm9
	vbroadcastsd	64(%r11), %ymm10
	vbroadcastsd	96(%r11), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vmovsd			%xmm8, 0(%r11)
	vmovsd			%xmm9, 32(%r11)
	vmovsd			%xmm10, 64(%r11)
	vmovsd			%xmm11, 96(%r11)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	0(%r11), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vmovsd			%xmm8, 0(%r11)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vbroadcastsd	0(%r13), %ymm15
	vmulpd			%ymm15, %ymm0, %ymm0

	vxorpd			%ymm15, %ymm15, %ymm15
	vblendpd		$ 0x1, %ymm15, %ymm0, %ymm0

	movq	ARG2, %r11 // D
	vmovapd			0(%r11), %ymm8
	vmovapd			32(%r11), %ymm9
	vmovapd			64(%r11), %ymm10
	vmovapd			96(%r11), %ymm11
	vaddpd			%ymm0, %ymm8, %ymm8
	vbroadcastsd	32(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm9
	vbroadcastsd	64(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm10
	vbroadcastsd	96(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm11
	vmulpd			%ymm10, %ymm10, %ymm15
	vfmadd231pd		%ymm11, %ymm11, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 32(%r11)
	vmovapd			%ymm10, 64(%r11)
	vmovapd			%ymm11, 96(%r11)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	vmovapd			0(%r11), %ymm8
	vmovapd			32(%r11), %ymm9
	vmovapd			64(%r11), %ymm10
	vmovapd			96(%r11), %ymm11
	vbroadcastsd	0(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vbroadcastsd	32(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm9
	vbroadcastsd	64(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm10
	vbroadcastsd	96(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm11
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vfmadd231pd		%ymm11, %ymm11, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 32(%r11)
	vmovapd			%ymm10, 64(%r11)
	vmovapd			%ymm11, 96(%r11)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	vmovapd			0(%r11), %ymm8
	vbroadcastsd	0(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 0(%r11)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
	vpermpd	$ 0x55, %ymm15, %ymm15  // sigma

	// second column
102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 8(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	vmovsd			40(%r11), %xmm14 // alpha
	vmovapd			%xmm15, %xmm13 // beta
	vfmadd231sd		%xmm14, %xmm14, %xmm13 // beta
	vsqrtsd			%xmm13, %xmm13, %xmm13 // beta
	vmovsd			%xmm13, 40(%r11) // pD[0+ps*0]

	vxorpd			%xmm12, %xmm12, %xmm12
	vucomisd		%xmm12, %xmm14
	jbe		111f
	// alpha>0
	vaddsd			%xmm13, %xmm14, %xmm14 // tmp
	vdivsd			%xmm14, %xmm15, %xmm14 // tmp
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm14, %xmm14
	jmp		112f
111: // alpha<=0
	vsubsd			%xmm13, %xmm14, %xmm14 // tmp
112:
	vmulsd			%xmm14, %xmm14, %xmm12 // tmp*tmp
	vaddsd			%xmm12, %xmm15, %xmm15 // sigma+tmp*tmp
	vaddsd			%xmm12, %xmm12, %xmm12 // 2*tmp*tmp
	vdivsd			%xmm15, %xmm12, %xmm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC02(%rip), %xmm12
#else
	vmovapd			LC02(%rip), %xmm12
#endif
	vdivsd			%xmm14, %xmm12, %xmm14 // tmp
	vmovddup		%xmm14, %xmm14

	vmovsd			%xmm15, 8(%r12) // dD[0]
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm15, %xmm15
	vmovsd			%xmm15, 40(%r13) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			32(%r11), %ymm0
	vbroadcastsd	72(%r11), %ymm9
	vbroadcastsd	104(%r11), %ymm10
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vfmadd231pd		64(%r11), %ymm9, %ymm0
	vfmadd231pd		96(%r11), %ymm10, %ymm0
	vmovsd			%xmm9, 72(%r11)
	vmovsd			%xmm10, 104(%r11)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	8(%r11), %ymm8
	vbroadcastsd	40(%r11), %ymm9
	vbroadcastsd	72(%r11), %ymm10
	vbroadcastsd	104(%r11), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vmovsd			%xmm8, 8(%r11)
	vmovsd			%xmm9, 40(%r11)
	vmovsd			%xmm10, 72(%r11)
	vmovsd			%xmm11, 104(%r11)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	8(%r11), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vmovsd			%xmm8, 8(%r11)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC02(%rip), %ymm12
#else
	vmovapd			LC02(%rip), %ymm12
#endif
	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm0
	vbroadcastsd	40(%r13), %ymm15
	vmulpd			%ymm15, %ymm0, %ymm0
	vmovsd			%xmm0, 32(%r13)

	vxorpd			%ymm12, %ymm12, %ymm12
	vblendpd		$ 0x3, %ymm12, %ymm0, %ymm0

	movq	ARG2, %r11 // D
	vmovapd			32(%r11), %ymm9
	vmovapd			64(%r11), %ymm10
	vmovapd			96(%r11), %ymm11
	vaddpd			%ymm0, %ymm9, %ymm9
	vbroadcastsd	72(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm10
	vbroadcastsd	104(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm11
	vmulpd			%ymm11, %ymm11, %ymm15
	vmovapd			%ymm9, 32(%r11)
	vmovapd			%ymm10, 64(%r11)
	vmovapd			%ymm11, 96(%r11)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	vmovapd			0(%r11), %ymm8
	vmovapd			32(%r11), %ymm9
	vmovapd			64(%r11), %ymm10
	vmovapd			96(%r11), %ymm11
	vbroadcastsd	8(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vbroadcastsd	40(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm9
	vbroadcastsd	72(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm10
	vbroadcastsd	104(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm11
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vfmadd231pd		%ymm11, %ymm11, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 32(%r11)
	vmovapd			%ymm10, 64(%r11)
	vmovapd			%ymm11, 96(%r11)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	vmovapd			0(%r11), %ymm8
	vbroadcastsd	8(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 0(%r11)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
	vpermpd	$ 0xaa, %ymm15, %ymm15  // sigma

	// third column
102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 16(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	vmovsd			80(%r11), %xmm14 // alpha
	vmovapd			%xmm15, %xmm13 // beta
	vfmadd231sd		%xmm14, %xmm14, %xmm13 // beta
	vsqrtsd			%xmm13, %xmm13, %xmm13 // beta
	vmovsd			%xmm13, 80(%r11) // pD[0+ps*0]

	vxorpd			%xmm12, %xmm12, %xmm12
	vucomisd		%xmm12, %xmm14
	jbe		111f
	// alpha>0
	vaddsd			%xmm13, %xmm14, %xmm14 // tmp
	vdivsd			%xmm14, %xmm15, %xmm14 // tmp
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm14, %xmm14
	jmp		112f
111: // alpha<=0
	vsubsd			%xmm13, %xmm14, %xmm14 // tmp
112:
	vmulsd			%xmm14, %xmm14, %xmm12 // tmp*tmp
	vaddsd			%xmm12, %xmm15, %xmm15 // sigma+tmp*tmp
	vaddsd			%xmm12, %xmm12, %xmm12 // 2*tmp*tmp
	vdivsd			%xmm15, %xmm12, %xmm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC02(%rip), %xmm12
#else
	vmovapd			LC02(%rip), %xmm12
#endif
	vdivsd			%xmm14, %xmm12, %xmm14 // tmp
	vmovddup		%xmm14, %xmm14

	vmovsd			%xmm15, 16(%r12) // dD[0]
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm15, %xmm15
	vmovsd			%xmm15, 80(%r13) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			64(%r11), %ymm0
	vbroadcastsd	112(%r11), %ymm10
	vmulpd			%ymm15, %ymm10, %ymm10
	vfmadd231pd		96(%r11), %ymm10, %ymm0
	vmovsd			%xmm10, 112(%r11)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	16(%r11), %ymm8
	vbroadcastsd	48(%r11), %ymm9
	vbroadcastsd	80(%r11), %ymm10
	vbroadcastsd	112(%r11), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vmovsd			%xmm8, 16(%r11)
	vmovsd			%xmm9, 48(%r11)
	vmovsd			%xmm10, 80(%r11)
	vmovsd			%xmm11, 112(%r11)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	16(%r11), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vmovsd			%xmm8, 16(%r11)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vxorpd			%xmm12, %xmm12, %xmm12
	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm1
	vmovapd			32(%r13), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vpermpd			$ 0x55, %ymm0, %ymm13
	vfmadd231pd		%ymm14, %ymm13, %ymm1
	vblendpd		$ 0x7, %ymm1, %ymm0, %ymm0
	vbroadcastsd	80(%r13), %ymm15
	vmulpd			%ymm15, %ymm0, %ymm0
	vmovapd			%xmm0, 64(%r13)

	vxorpd			%ymm12, %ymm12, %ymm12
	vblendpd		$ 0x7, %ymm12, %ymm0, %ymm0

	movq	ARG2, %r11 // D
	vmovapd			64(%r11), %ymm10
	vmovapd			96(%r11), %ymm11
	vaddpd			%ymm0, %ymm10, %ymm10
	vbroadcastsd	112(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm11
	vxorpd			%ymm15, %ymm15, %ymm15
	vmovapd			%ymm10, 64(%r11)
	vmovapd			%ymm11, 96(%r11)
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		110f
106:
	vmovapd			0(%r11), %ymm8
	vmovapd			32(%r11), %ymm9
	vmovapd			64(%r11), %ymm10
	vmovapd			96(%r11), %ymm11
	vbroadcastsd	16(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vbroadcastsd	48(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm9
	vbroadcastsd	80(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm10
	vbroadcastsd	112(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm11
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vfmadd231pd		%ymm9, %ymm9, %ymm15
	vfmadd231pd		%ymm10, %ymm10, %ymm15
	vfmadd231pd		%ymm11, %ymm11, %ymm15
	vmovapd			%ymm8, 0(%r11)
	vmovapd			%ymm9, 32(%r11)
	vmovapd			%ymm10, 64(%r11)
	vmovapd			%ymm11, 96(%r11)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		106b
110:
	cmpl	$ 0, %r10d
	jle		107f
108:
	vmovapd			0(%r11), %ymm8
	vbroadcastsd	16(%r11), %ymm14
	vfmadd231pd		%ymm0, %ymm14, %ymm8
	vfmadd231pd		%ymm8, %ymm8, %ymm15
	vmovapd			%ymm8, 0(%r11)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		108b
107:
	vpermpd	$ 0xff, %ymm15, %ymm15  // sigma

102:
	vxorpd			%xmm14, %xmm14, %xmm14
	vucomisd		%xmm14, %xmm15
	jne		101f
//	jp		111f
	vmovsd			%xmm14, 24(%r12)
	jmp		102f

101:
	movq	ARG2, %r11 // D
	vmovsd			120(%r11), %xmm14 // alpha
	vmovapd			%xmm15, %xmm13 // beta
	vfmadd231sd		%xmm14, %xmm14, %xmm13 // beta
	vsqrtsd			%xmm13, %xmm13, %xmm13 // beta
	vmovsd			%xmm13, 120(%r11) // pD[0+ps*0]

	vxorpd			%xmm12, %xmm12, %xmm12
	vucomisd		%xmm12, %xmm14
	jbe		111f
	// alpha>0
	vaddsd			%xmm13, %xmm14, %xmm14 // tmp
	vdivsd			%xmm14, %xmm15, %xmm14 // tmp
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm14, %xmm14
	jmp		112f
111: // alpha<=0
	vsubsd			%xmm13, %xmm14, %xmm14 // tmp
112:
	vmulsd			%xmm14, %xmm14, %xmm12 // tmp*tmp
	vaddsd			%xmm12, %xmm15, %xmm15 // sigma+tmp*tmp
	vaddsd			%xmm12, %xmm12, %xmm12 // 2*tmp*tmp
	vdivsd			%xmm15, %xmm12, %xmm15

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovapd			.LC02(%rip), %xmm12
#else
	vmovapd			LC02(%rip), %xmm12
#endif
	vdivsd			%xmm14, %xmm12, %xmm14 // tmp
	vmovddup		%xmm14, %xmm14

	vmovsd			%xmm15, 24(%r12) // dD[0]
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovsd			.LC00(%rip), %xmm12 // mask
#else
	vmovsd			LC00(%rip), %xmm12 // mask
#endif
	vxorpd			%xmm12, %xmm15, %xmm15
	vmovsd			%xmm15, 120(%r13) // pT[0+ps*0]

	vpermpd			$ 0x55, %ymm14, %ymm15 // tmp

	vmovapd			96(%r11), %ymm0
	movq	ARG1, %r10 // n
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jle		109f
103:
	vbroadcastsd	24(%r11), %ymm8
	vbroadcastsd	56(%r11), %ymm9
	vbroadcastsd	88(%r11), %ymm10
	vbroadcastsd	120(%r11), %ymm11
	vmulpd			%ymm15, %ymm8, %ymm8
	vmulpd			%ymm15, %ymm9, %ymm9
	vmulpd			%ymm15, %ymm10, %ymm10
	vmulpd			%ymm15, %ymm11, %ymm11
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vfmadd231pd		32(%r11), %ymm9, %ymm0
	vfmadd231pd		64(%r11), %ymm10, %ymm0
	vfmadd231pd		96(%r11), %ymm11, %ymm0
	vmovsd			%xmm8, 24(%r11)
	vmovsd			%xmm9, 56(%r11)
	vmovsd			%xmm10, 88(%r11)
	vmovsd			%xmm11, 120(%r11)
	subl	$ 4, %r10d
	addq	$ 128, %r11
	cmpl	$ 3, %r10d
	jg		103b
109:
	cmpl	$ 0, %r10d
	jle		104f
105:
	vbroadcastsd	24(%r11), %ymm8
	vmulpd			%ymm15, %ymm8, %ymm8
	vfmadd231pd		0(%r11), %ymm8, %ymm0
	vmovsd			%xmm8, 24(%r11)
	subl	$ 1, %r10d
	addq	$ 32, %r11
	cmpl	$ 0, %r10d
	jg		105b
104:

	vxorpd			%xmm12, %xmm12, %xmm12

	vmovapd			0(%r13), %ymm14
	vblendpd		$ 0x1, %ymm14, %ymm12, %ymm14
	vmulpd			%ymm14, %ymm0, %ymm1

	vmovapd			32(%r13), %ymm14
	vblendpd		$ 0x3, %ymm14, %ymm12, %ymm14
	vpermpd			$ 0x55, %ymm0, %ymm13
	vfmadd231pd		%ymm14, %ymm13, %ymm1

	vmovapd			64(%r13), %ymm14
	vblendpd		$ 0x7, %ymm14, %ymm12, %ymm14
	vpermpd			$ 0xaa, %ymm0, %ymm13
	vfmadd231pd		%ymm14, %ymm13, %ymm1

	vbroadcastsd	120(%r13), %ymm15
	vmulpd			%ymm15, %ymm1, %ymm1
	vmovapd			96(%r13), %ymm0
	vblendpd		$ 0x7, %ymm1, %ymm0, %ymm0
	vmovapd			%ymm0, 96(%r13)

102:

	EPILOGUE
	
	ret

#if defined(OS_LINUX)
	.size	kernel_dgelqf_pd_dlarft4_4_lib4, .-kernel_dgelqf_pd_dlarft4_4_lib4
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
.LC00: // { 100...0 100...0 100...0 100...0 }
#elif defined(OS_MAC)
LC00: // { 100...0 100...0 100...0 100...0 }
	.align 5
#endif
	.long	0x00000000
	.long	0x80000000
	.long	0x00000000
	.long	0x80000000
	.long	0x00000000
	.long	0x80000000
	.long	0x00000000
	.long	0x80000000

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC01:
#elif defined(OS_MAC)
	.align 5
LC01:
#endif
	.double	-1.0
	.double	-1.0
	.double	-1.0
	.double	-1.0

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC02:
#elif defined(OS_MAC)
	.align 5
LC02:
#endif
	.double	1.0
	.double	1.0
	.double	1.0
	.double	1.0





#if defined(OS_LINUX)
	.section	.note.GNU-stack,"",@progbits
#elif defined(OS_MAC)
	.subsections_via_symbols
#endif

