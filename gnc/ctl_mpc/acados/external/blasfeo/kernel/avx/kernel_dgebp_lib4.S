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





//                               1      2          3        4          5
// void kernel_dger4_sub_8r_lib4(int k, double *A, int sda, double *B, double *C)

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dger4_sub_8r_lib4
	.type kernel_dger4_sub_8r_lib4, @function
kernel_dger4_sub_8r_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dger4_sub_8r_lib4
_kernel_dger4_sub_8r_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dger4_sub_8r_lib4
	.def kernel_dger4_sub_8r_lib4; .scl 2; .type 32; .endef
kernel_dger4_sub_8r_lib4:
#endif
	
	PROLOGUE

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // C
	movq	ARG6, %r15 // C
	sall	$ 5, %r15d // 4*sdc*sizeof(double)

	cmpl	$ 0, %r10d
	jle		0f // return

	// load block from A
	vmovapd	0(%r11), %ymm0
	vmovapd	32(%r11), %ymm1
	vmovapd	64(%r11), %ymm2
	vmovapd	96(%r11), %ymm3

	vmovapd	0(%r11, %r12, 1), %ymm4
	vmovapd	32(%r11, %r12, 1), %ymm5
	vmovapd	64(%r11, %r12, 1), %ymm6
	vmovapd	96(%r11, %r12, 1), %ymm7

	cmpl	$ 3, %r10d
	jle		2f // cleanup loop

	// main loop
	.p2align 3
1:
	vmovapd			0(%r14), %ymm8
	vmovapd			0(%r14, %r15, 1), %ymm9
	vbroadcastsd	0(%r13), %ymm15
	vmulpd			%ymm0, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm4, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vbroadcastsd	8(%r13), %ymm15
	subl	$ 4, %r10d
	vmulpd			%ymm1, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm5, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vbroadcastsd	16(%r13), %ymm15
	vmulpd			%ymm2, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm6, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vbroadcastsd	24(%r13), %ymm15
	vmulpd			%ymm3, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm7, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vmovapd			%ymm8, 0(%r14)
	vmovapd			%ymm9, 0(%r14, %r15, 1)

	vmovapd			32(%r14), %ymm8
	vmovapd			32(%r14, %r15, 1), %ymm9
	vbroadcastsd	32(%r13), %ymm15
	vmulpd			%ymm0, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm4, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vbroadcastsd	40(%r13), %ymm15
	vmulpd			%ymm1, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm5, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vbroadcastsd	48(%r13), %ymm15
	vmulpd			%ymm2, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm6, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vbroadcastsd	56(%r13), %ymm15
	vmulpd			%ymm3, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm7, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vmovapd			%ymm8, 32(%r14)
	vmovapd			%ymm9, 32(%r14, %r15, 1)

	vmovapd			64(%r14), %ymm8
	vmovapd			64(%r14, %r15, 1), %ymm9
	vbroadcastsd	64(%r13), %ymm15
	vmulpd			%ymm0, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm4, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vbroadcastsd	72(%r13), %ymm15
	vmulpd			%ymm1, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm5, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vbroadcastsd	80(%r13), %ymm15
	vmulpd			%ymm2, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm6, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vbroadcastsd	88(%r13), %ymm15
	vmulpd			%ymm3, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm7, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vmovapd			%ymm8, 64(%r14)
	vmovapd			%ymm9, 64(%r14, %r15, 1)

	vmovapd			96(%r14), %ymm8
	vmovapd			96(%r14, %r15, 1), %ymm9
	vbroadcastsd	96(%r13), %ymm15
	addq	$ 128, %r13
	vmulpd			%ymm0, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm4, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vbroadcastsd	-24(%r13), %ymm15
	vmulpd			%ymm1, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm5, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vbroadcastsd	-16(%r13), %ymm15
	vmulpd			%ymm2, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm6, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vbroadcastsd	-8(%r13), %ymm15
	addq	$ 128, %r14
	vmulpd			%ymm3, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm7, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vmovapd			%ymm8, -32(%r14)
	vmovapd			%ymm9, -32(%r14, %r15, 1)

	cmpl	$ 3, %r10d
	jg		1b // main loop

	cmpl	$ 0, %r10d
	jle		0f // return

	// cleanup loop
2:
	vmovapd			0(%r14), %ymm8
	vmovapd			0(%r14, %r15, 1), %ymm9
	vbroadcastsd	0(%r13), %ymm15
	vmulpd			%ymm0, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm4, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vbroadcastsd	8(%r13), %ymm15
	vmulpd			%ymm1, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm5, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vbroadcastsd	16(%r13), %ymm15
	vmulpd			%ymm2, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm6, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vbroadcastsd	24(%r13), %ymm15
	vmulpd			%ymm3, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm7, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vmovapd			%ymm8, 0(%r14)
	vmovapd			%ymm9, 0(%r14, %r15, 1)

	addq	$ 32, %r13
	addq	$ 32, %r14

	subl	$ 1, %r10d
	cmpl	$ 0, %r10d
	jg		2b // main loop

	// return
0:

	EPILOGUE
	
	ret

#if defined(OS_LINUX)
	.size	kernel_dger4_sub_8r_lib4, .-kernel_dger4_sub_8r_lib4
#endif





//                                 1      2          3        4          5          6        7
// void kernel_dger4_sub_8_vs_lib4(int k, double *A, int sda, double *B, double *C, int sdc, int km)

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dger4_sub_8r_vs_lib4
	.type kernel_dger4_sub_8r_vs_lib4, @function
kernel_dger4_sub_8r_vs_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dger4_sub_8r_vs_lib4
_kernel_dger4_sub_8r_vs_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dger4_sub_8r_vs_lib4
	.def kernel_dger4_sub_8r_vs_lib4; .scl 2; .type 32; .endef
kernel_dger4_sub_8r_vs_lib4:
#endif
	
	PROLOGUE

	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG4, %r13 // B
	movq	ARG5, %r14 // C
	movq	ARG6, %r15 // C
	sall	$ 5, %r15d // 4*sdc*sizeof(double)
	movq	ARG7, %rax // km

	cmpl	$ 0, %r10d
	jle		0f // return

	vcvtsi2sd	%eax, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC01(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC01(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm15

	// load block from A
	vmovapd	0(%r11), %ymm0
	vmovapd	32(%r11), %ymm1
	vmovapd	64(%r11), %ymm2
	vmovapd	96(%r11), %ymm3

	vmaskmovpd	0(%r11, %r12, 1), %ymm15, %ymm4
	vmaskmovpd	32(%r11, %r12, 1), %ymm15, %ymm5
	vmaskmovpd	64(%r11, %r12, 1), %ymm15, %ymm6
	vmaskmovpd	96(%r11, %r12, 1), %ymm15, %ymm7

	cmpl	$ 3, %r10d
	jle		2f // cleanup loop

	// main loop
	.p2align 3
1:
	vmovapd			0(%r14), %ymm8
	vmovapd			0(%r14, %r15, 1), %ymm9
	vbroadcastsd	0(%r13), %ymm15
	vmulpd			%ymm0, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm4, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vbroadcastsd	8(%r13), %ymm15
	subl	$ 4, %r10d
	vmulpd			%ymm1, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm5, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vbroadcastsd	16(%r13), %ymm15
	vmulpd			%ymm2, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm6, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vbroadcastsd	24(%r13), %ymm15
	vmulpd			%ymm3, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm7, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vmovapd			%ymm8, 0(%r14)
	vmovapd			%ymm9, 0(%r14, %r15, 1)

	vmovapd			32(%r14), %ymm8
	vmovapd			32(%r14, %r15, 1), %ymm9
	vbroadcastsd	32(%r13), %ymm15
	vmulpd			%ymm0, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm4, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vbroadcastsd	40(%r13), %ymm15
	vmulpd			%ymm1, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm5, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vbroadcastsd	48(%r13), %ymm15
	vmulpd			%ymm2, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm6, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vbroadcastsd	56(%r13), %ymm15
	vmulpd			%ymm3, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm7, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vmovapd			%ymm8, 32(%r14)
	vmovapd			%ymm9, 32(%r14, %r15, 1)

	vmovapd			64(%r14), %ymm8
	vmovapd			64(%r14, %r15, 1), %ymm9
	vbroadcastsd	64(%r13), %ymm15
	vmulpd			%ymm0, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm4, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vbroadcastsd	72(%r13), %ymm15
	vmulpd			%ymm1, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm5, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vbroadcastsd	80(%r13), %ymm15
	vmulpd			%ymm2, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm6, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vbroadcastsd	88(%r13), %ymm15
	vmulpd			%ymm3, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm7, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vmovapd			%ymm8, 64(%r14)
	vmovapd			%ymm9, 64(%r14, %r15, 1)

	vmovapd			96(%r14), %ymm8
	vmovapd			96(%r14, %r15, 1), %ymm9
	vbroadcastsd	96(%r13), %ymm15
	addq	$ 128, %r13
	vmulpd			%ymm0, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm4, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vbroadcastsd	-24(%r13), %ymm15
	vmulpd			%ymm1, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm5, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vbroadcastsd	-16(%r13), %ymm15
	vmulpd			%ymm2, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm6, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vbroadcastsd	-8(%r13), %ymm15
	addq	$ 128, %r14
	vmulpd			%ymm3, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm7, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vmovapd			%ymm8, -32(%r14)
	vmovapd			%ymm9, -32(%r14, %r15, 1)

	cmpl	$ 3, %r10d
	jg		1b // main loop

	cmpl	$ 0, %r10d
	jle		0f // return

	// cleanup loop
2:
	vmovapd			0(%r14), %ymm8
	vmovapd			0(%r14, %r15, 1), %ymm9
	vbroadcastsd	0(%r13), %ymm15
	vmulpd			%ymm0, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm4, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vbroadcastsd	8(%r13), %ymm15
	vmulpd			%ymm1, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm5, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vbroadcastsd	16(%r13), %ymm15
	vmulpd			%ymm2, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm6, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vbroadcastsd	24(%r13), %ymm15
	vmulpd			%ymm3, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm8, %ymm8
	vmulpd			%ymm7, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm9, %ymm9
	vmovapd			%ymm8, 0(%r14)
	vmovapd			%ymm9, 0(%r14, %r15, 1)

	addq	$ 32, %r13
	addq	$ 32, %r14

	subl	$ 1, %r10d
	cmpl	$ 0, %r10d
	jg		2b // main loop

	// return
0:

	EPILOGUE
	
	ret

#if defined(OS_LINUX)
	.size	kernel_dger4_sub_8r_vs_lib4, .-kernel_dger4_sub_8r_vs_lib4
#endif





//                               1      2          3          4
// void kernel_dger4_sub_4r_lib4(int n, double *A, double *B, double *C)

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dger4_sub_4r_lib4
	.type kernel_dger4_sub_4r_lib4, @function
kernel_dger4_sub_4r_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dger4_sub_4r_lib4
_kernel_dger4_sub_4r_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dger4_sub_4r_lib4
	.def kernel_dger4_sub_4r_lib4; .scl 2; .type 32; .endef
kernel_dger4_sub_4r_lib4:
#endif
	
	PROLOGUE

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	movq	ARG4, %r13

	cmpl	$ 0, %r10d
	jle		0f // return

	// load block from A
	vmovapd	0(%r11), %ymm0
	vmovapd	32(%r11), %ymm1
	vmovapd	64(%r11), %ymm2
	vmovapd	96(%r11), %ymm3

	cmpl	$ 3, %r10d
	jle		2f // cleanup loop

	// main loop
	.p2align 3
1:
	vmovapd			0(%r13), %ymm4
	vbroadcastsd	0(%r12), %ymm15
	vmulpd			%ymm0, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vbroadcastsd	8(%r12), %ymm15
	subl	$ 4, %r10d
	vmulpd			%ymm1, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vbroadcastsd	16(%r12), %ymm15
	vmulpd			%ymm2, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vbroadcastsd	24(%r12), %ymm15
	vmulpd			%ymm3, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vmovapd			%ymm4, 0(%r13)

	vmovapd			32(%r13), %ymm4
	vbroadcastsd	32(%r12), %ymm15
	vmulpd			%ymm0, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vbroadcastsd	40(%r12), %ymm15
	vmulpd			%ymm1, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vbroadcastsd	48(%r12), %ymm15
	vmulpd			%ymm2, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vbroadcastsd	56(%r12), %ymm15
	vmulpd			%ymm3, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vmovapd			%ymm4, 32(%r13)

	vmovapd			64(%r13), %ymm4
	vbroadcastsd	64(%r12), %ymm15
	vmulpd			%ymm0, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vbroadcastsd	72(%r12), %ymm15
	vmulpd			%ymm1, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vbroadcastsd	80(%r12), %ymm15
	vmulpd			%ymm2, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vbroadcastsd	88(%r12), %ymm15
	vmulpd			%ymm3, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vmovapd			%ymm4, 64(%r13)

	vmovapd			96(%r13), %ymm4
	vbroadcastsd	96(%r12), %ymm15
	addq	$ 128, %r12
	vmulpd			%ymm0, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vbroadcastsd	-24(%r12), %ymm15
	vmulpd			%ymm1, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vbroadcastsd	-16(%r12), %ymm15
	vmulpd			%ymm2, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vbroadcastsd	-8(%r12), %ymm15
	addq	$ 128, %r13
	vmulpd			%ymm3, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vmovapd			%ymm4, -32(%r13)

	cmpl	$ 3, %r10d
	jg		1b // main loop

	cmpl	$ 0, %r10d
	jle		0f // return

	// cleanup loop
2:
	vmovapd			0(%r13), %ymm4
	vbroadcastsd	0(%r12), %ymm15
	vmulpd			%ymm0, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vbroadcastsd	8(%r12), %ymm15
	vmulpd			%ymm1, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vbroadcastsd	16(%r12), %ymm15
	vmulpd			%ymm2, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vbroadcastsd	24(%r12), %ymm15
	vmulpd			%ymm3, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vmovapd			%ymm4, 0(%r13)

	addq	$ 32, %r12
	addq	$ 32, %r13

	subl	$ 1, %r10d
	cmpl	$ 0, %r10d
	jg		2b // main loop

	// return
0:

	EPILOGUE
	
	ret

#if defined(OS_LINUX)
	.size	kernel_dger4_sub_4r_lib4, .-kernel_dger4_sub_4r_lib4
#endif





//                                 1      2          3          4          5
// void kernel_dger4_sub_4_vs_lib4(int n, double *A, double *B, double *C, int km)

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dger4_sub_4r_vs_lib4
	.type kernel_dger4_sub_4r_vs_lib4, @function
kernel_dger4_sub_4r_vs_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dger4_sub_4r_vs_lib4
_kernel_dger4_sub_4r_vs_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dger4_sub_4r_vs_lib4
	.def kernel_dger4_sub_4r_vs_lib4; .scl 2; .type 32; .endef
kernel_dger4_sub_4r_vs_lib4:
#endif
	
	PROLOGUE

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	movq	ARG4, %r13
	movq	ARG5, %r14

	cmpl	$ 0, %r10d
	jle		0f // return

	vcvtsi2sd	%r14d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC00(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC00(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm15

	// load block from A
	vmaskmovpd	0(%r11), %ymm15, %ymm0
	vmaskmovpd	32(%r11), %ymm15, %ymm1
	vmaskmovpd	64(%r11), %ymm15, %ymm2
	vmaskmovpd	96(%r11), %ymm15, %ymm3

	cmpl	$ 3, %r10d
	jle		2f // cleanup loop

	// main loop
	.p2align 3
1:
	vmovapd			0(%r13), %ymm4
	vbroadcastsd	0(%r12), %ymm15
	vmulpd			%ymm0, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vbroadcastsd	8(%r12), %ymm15
	subl	$ 4, %r10d
	vmulpd			%ymm1, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vbroadcastsd	16(%r12), %ymm15
	vmulpd			%ymm2, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vbroadcastsd	24(%r12), %ymm15
	vmulpd			%ymm3, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vmovapd			%ymm4, 0(%r13)

	vmovapd			32(%r13), %ymm4
	vbroadcastsd	32(%r12), %ymm15
	vmulpd			%ymm0, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vbroadcastsd	40(%r12), %ymm15
	vmulpd			%ymm1, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vbroadcastsd	48(%r12), %ymm15
	vmulpd			%ymm2, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vbroadcastsd	56(%r12), %ymm15
	vmulpd			%ymm3, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vmovapd			%ymm4, 32(%r13)

	vmovapd			64(%r13), %ymm4
	vbroadcastsd	64(%r12), %ymm15
	vmulpd			%ymm0, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vbroadcastsd	72(%r12), %ymm15
	vmulpd			%ymm1, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vbroadcastsd	80(%r12), %ymm15
	vmulpd			%ymm2, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vbroadcastsd	88(%r12), %ymm15
	vmulpd			%ymm3, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vmovapd			%ymm4, 64(%r13)

	vmovapd			96(%r13), %ymm4
	vbroadcastsd	96(%r12), %ymm15
	addq	$ 128, %r12
	vmulpd			%ymm0, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vbroadcastsd	-24(%r12), %ymm15
	vmulpd			%ymm1, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vbroadcastsd	-16(%r12), %ymm15
	vmulpd			%ymm2, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vbroadcastsd	-8(%r12), %ymm15
	addq	$ 128, %r13
	vmulpd			%ymm3, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vmovapd			%ymm4, -32(%r13)

	cmpl	$ 3, %r10d
	jg		1b // main loop

	cmpl	$ 0, %r10d
	jle		0f // return

	// cleanup loop
2:
	vmovapd			0(%r13), %ymm4
	vbroadcastsd	0(%r12), %ymm15
	vmulpd			%ymm0, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vbroadcastsd	8(%r12), %ymm15
	vmulpd			%ymm1, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vbroadcastsd	16(%r12), %ymm15
	vmulpd			%ymm2, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vbroadcastsd	24(%r12), %ymm15
	vmulpd			%ymm3, %ymm15, %ymm14
	vsubpd			%ymm14, %ymm4, %ymm4
	vmovapd			%ymm4, 0(%r13)

	addq	$ 32, %r12
	addq	$ 32, %r13

	subl	$ 1, %r10d
	cmpl	$ 0, %r10d
	jg		2b // main loop

	// return
0:

	EPILOGUE
	
	ret

#if defined(OS_LINUX)
	.size	kernel_dger4_sub_4r_vs_lib4, .-kernel_dger4_sub_4r_vs_lib4
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
.LC00:
#elif defined(OS_MAC)
LC00:
	.align 5
#endif
	.double 0.5
	.double 1.5
	.double 2.5
	.double 3.5

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC01:
#elif defined(OS_MAC)
LC01:
	.align 5
#endif
	.double 4.5
	.double 5.5
	.double 6.5
	.double 7.5

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC02:
#elif defined(OS_MAC)
LC02:
	.align 5
#endif
	.double 8.5
	.double 9.5
	.double 10.5
	.double 11.5





#if defined(OS_LINUX)
	.section	.note.GNU-stack,"",@progbits
#elif defined(OS_MAC)
	.subsections_via_symbols
#endif


