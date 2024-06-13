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
// void kernel_dger4_sub_12_lib4(int k, double *A, int sda, double *B, double *C)

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dger4_sub_12r_lib4
	.type kernel_dger4_sub_12r_lib4, @function
kernel_dger4_sub_12r_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dger4_sub_12r_lib4
_kernel_dger4_sub_12r_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dger4_sub_12r_lib4
	.def kernel_dger4_sub_12r_lib4; .scl 2; .type 32; .endef
kernel_dger4_sub_12r_lib4:
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

	vmovapd	0(%r11, %r12, 2), %ymm8
	vmovapd	32(%r11, %r12, 2), %ymm9
	vmovapd	64(%r11, %r12, 2), %ymm10
	vmovapd	96(%r11, %r12, 2), %ymm11

	cmpl	$ 3, %r10d
	jle		2f // cleanup loop

	// main loop
	.p2align 3
1:
	vmovapd			0(%r14), %ymm12
	vmovapd			0(%r14, %r15, 1), %ymm13
	vmovapd			0(%r14, %r15, 2), %ymm14
	vbroadcastsd	0(%r13), %ymm15
	vfnmadd231pd	%ymm0, %ymm15, %ymm12
	vfnmadd231pd	%ymm4, %ymm15, %ymm13
	vfnmadd231pd	%ymm8, %ymm15, %ymm14
	vbroadcastsd	8(%r13), %ymm15
	subl	$ 4, %r10d
	vfnmadd231pd	%ymm1, %ymm15, %ymm12
	vfnmadd231pd	%ymm5, %ymm15, %ymm13
	vfnmadd231pd	%ymm9, %ymm15, %ymm14
	vbroadcastsd	16(%r13), %ymm15
	vfnmadd231pd	%ymm2, %ymm15, %ymm12
	vfnmadd231pd	%ymm6, %ymm15, %ymm13
	vfnmadd231pd	%ymm10, %ymm15, %ymm14
	vbroadcastsd	24(%r13), %ymm15
	vfnmadd231pd	%ymm3, %ymm15, %ymm12
	vfnmadd231pd	%ymm7, %ymm15, %ymm13
	vfnmadd231pd	%ymm11, %ymm15, %ymm14
	vmovapd			%ymm12, 0(%r14)
	vmovapd			%ymm13, 0(%r14, %r15, 1)
	vmovapd			%ymm14, 0(%r14, %r15, 2)

	vmovapd			32(%r14), %ymm12
	vmovapd			32(%r14, %r15, 1), %ymm13
	vmovapd			32(%r14, %r15, 2), %ymm14
	vbroadcastsd	32(%r13), %ymm15
	vfnmadd231pd	%ymm0, %ymm15, %ymm12
	vfnmadd231pd	%ymm4, %ymm15, %ymm13
	vfnmadd231pd	%ymm8, %ymm15, %ymm14
	vbroadcastsd	40(%r13), %ymm15
	vfnmadd231pd	%ymm1, %ymm15, %ymm12
	vfnmadd231pd	%ymm5, %ymm15, %ymm13
	vfnmadd231pd	%ymm9, %ymm15, %ymm14
	vbroadcastsd	48(%r13), %ymm15
	vfnmadd231pd	%ymm2, %ymm15, %ymm12
	vfnmadd231pd	%ymm6, %ymm15, %ymm13
	vfnmadd231pd	%ymm10, %ymm15, %ymm14
	vbroadcastsd	56(%r13), %ymm15
	vfnmadd231pd	%ymm3, %ymm15, %ymm12
	vfnmadd231pd	%ymm7, %ymm15, %ymm13
	vfnmadd231pd	%ymm11, %ymm15, %ymm14
	vmovapd			%ymm12, 32(%r14)
	vmovapd			%ymm13, 32(%r14, %r15, 1)
	vmovapd			%ymm14, 32(%r14, %r15, 2)

	vmovapd			64(%r14), %ymm12
	vmovapd			64(%r14, %r15, 1), %ymm13
	vmovapd			64(%r14, %r15, 2), %ymm14
	vbroadcastsd	64(%r13), %ymm15
	vfnmadd231pd	%ymm0, %ymm15, %ymm12
	vfnmadd231pd	%ymm4, %ymm15, %ymm13
	vfnmadd231pd	%ymm8, %ymm15, %ymm14
	vbroadcastsd	72(%r13), %ymm15
	vfnmadd231pd	%ymm1, %ymm15, %ymm12
	vfnmadd231pd	%ymm5, %ymm15, %ymm13
	vfnmadd231pd	%ymm9, %ymm15, %ymm14
	vbroadcastsd	80(%r13), %ymm15
	vfnmadd231pd	%ymm2, %ymm15, %ymm12
	vfnmadd231pd	%ymm6, %ymm15, %ymm13
	vfnmadd231pd	%ymm10, %ymm15, %ymm14
	vbroadcastsd	88(%r13), %ymm15
	vfnmadd231pd	%ymm3, %ymm15, %ymm12
	vfnmadd231pd	%ymm7, %ymm15, %ymm13
	vfnmadd231pd	%ymm11, %ymm15, %ymm14
	vmovapd			%ymm12, 64(%r14)
	vmovapd			%ymm13, 64(%r14, %r15, 1)
	vmovapd			%ymm14, 64(%r14, %r15, 2)

	vmovapd			96(%r14), %ymm12
	vmovapd			96(%r14, %r15, 1), %ymm13
	vmovapd			96(%r14, %r15, 2), %ymm14
	vbroadcastsd	96(%r13), %ymm15
	addq	$ 128, %r13
	vfnmadd231pd	%ymm0, %ymm15, %ymm12
	vfnmadd231pd	%ymm4, %ymm15, %ymm13
	vfnmadd231pd	%ymm8, %ymm15, %ymm14
	vbroadcastsd	-24(%r13), %ymm15
	vfnmadd231pd	%ymm1, %ymm15, %ymm12
	vfnmadd231pd	%ymm5, %ymm15, %ymm13
	vfnmadd231pd	%ymm9, %ymm15, %ymm14
	vbroadcastsd	-16(%r13), %ymm15
	vfnmadd231pd	%ymm2, %ymm15, %ymm12
	vfnmadd231pd	%ymm6, %ymm15, %ymm13
	vfnmadd231pd	%ymm10, %ymm15, %ymm14
	vbroadcastsd	-8(%r13), %ymm15
	addq	$ 128, %r14
	vfnmadd231pd	%ymm3, %ymm15, %ymm12
	vfnmadd231pd	%ymm7, %ymm15, %ymm13
	vfnmadd231pd	%ymm11, %ymm15, %ymm14
	vmovapd			%ymm12, -32(%r14)
	vmovapd			%ymm13, -32(%r14, %r15, 1)
	vmovapd			%ymm14, -32(%r14, %r15, 2)

	cmpl	$ 3, %r10d
	jg		1b // main loop

	cmpl	$ 0, %r10d
	jle		0f // return

	// cleanup loop
2:
	vmovapd			0(%r14), %ymm12
	vmovapd			0(%r14, %r15, 1), %ymm13
	vmovapd			0(%r14, %r15, 2), %ymm14
	vbroadcastsd	0(%r13), %ymm15
	vfnmadd231pd	%ymm0, %ymm15, %ymm12
	vfnmadd231pd	%ymm4, %ymm15, %ymm13
	vfnmadd231pd	%ymm8, %ymm15, %ymm14
	vbroadcastsd	8(%r13), %ymm15
	vfnmadd231pd	%ymm1, %ymm15, %ymm12
	vfnmadd231pd	%ymm5, %ymm15, %ymm13
	vfnmadd231pd	%ymm9, %ymm15, %ymm14
	vbroadcastsd	16(%r13), %ymm15
	vfnmadd231pd	%ymm2, %ymm15, %ymm12
	vfnmadd231pd	%ymm6, %ymm15, %ymm13
	vfnmadd231pd	%ymm10, %ymm15, %ymm14
	vbroadcastsd	24(%r13), %ymm15
	vfnmadd231pd	%ymm3, %ymm15, %ymm12
	vfnmadd231pd	%ymm7, %ymm15, %ymm13
	vfnmadd231pd	%ymm11, %ymm15, %ymm14
	vmovapd			%ymm12, 0(%r14)
	vmovapd			%ymm13, 0(%r14, %r15, 1)
	vmovapd			%ymm14, 0(%r14, %r15, 2)

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
	.size	kernel_dger4_sub_12r_lib4, .-kernel_dger4_sub_12r_lib4
#endif





//                                  1      2          3        4          5          6        7
// void kernel_dger4_sub_12_vs_lib4(int k, double *A, int sda, double *B, double *C, int sdc, int km)

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dger4_sub_12r_vs_lib4
	.type kernel_dger4_sub_12r_vs_lib4, @function
kernel_dger4_sub_12r_vs_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dger4_sub_12r_vs_lib4
_kernel_dger4_sub_12r_vs_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dger4_sub_12r_vs_lib4
	.def kernel_dger4_sub_12r_vs_lib4; .scl 2; .type 32; .endef
kernel_dger4_sub_12r_vs_lib4:
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
	vmovupd		.LC02(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC02(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$ 1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm15

	// load block from A
	vmovapd	0(%r11), %ymm0
	vmovapd	32(%r11), %ymm1
	vmovapd	64(%r11), %ymm2
	vmovapd	96(%r11), %ymm3

	vmovapd	0(%r11, %r12, 1), %ymm4
	vmovapd	32(%r11, %r12, 1), %ymm5
	vmovapd	64(%r11, %r12, 1), %ymm6
	vmovapd	96(%r11, %r12, 1), %ymm7

	vmaskmovpd	0(%r11, %r12, 2), %ymm15, %ymm8
	vmaskmovpd	32(%r11, %r12, 2), %ymm15, %ymm9
	vmaskmovpd	64(%r11, %r12, 2), %ymm15, %ymm10
	vmaskmovpd	96(%r11, %r12, 2), %ymm15, %ymm11

	cmpl	$ 3, %r10d
	jle		2f // cleanup loop

	// main loop
	.p2align 3
1:
	vmovapd			0(%r14), %ymm12
	vmovapd			0(%r14, %r15, 1), %ymm13
	vmovapd			0(%r14, %r15, 2), %ymm14
	vbroadcastsd	0(%r13), %ymm15
	vfnmadd231pd	%ymm0, %ymm15, %ymm12
	vfnmadd231pd	%ymm4, %ymm15, %ymm13
	vfnmadd231pd	%ymm8, %ymm15, %ymm14
	vbroadcastsd	8(%r13), %ymm15
	subl	$ 4, %r10d
	vfnmadd231pd	%ymm1, %ymm15, %ymm12
	vfnmadd231pd	%ymm5, %ymm15, %ymm13
	vfnmadd231pd	%ymm9, %ymm15, %ymm14
	vbroadcastsd	16(%r13), %ymm15
	vfnmadd231pd	%ymm2, %ymm15, %ymm12
	vfnmadd231pd	%ymm6, %ymm15, %ymm13
	vfnmadd231pd	%ymm10, %ymm15, %ymm14
	vbroadcastsd	24(%r13), %ymm15
	vfnmadd231pd	%ymm3, %ymm15, %ymm12
	vfnmadd231pd	%ymm7, %ymm15, %ymm13
	vfnmadd231pd	%ymm11, %ymm15, %ymm14
	vmovapd			%ymm12, 0(%r14)
	vmovapd			%ymm13, 0(%r14, %r15, 1)
	vmovapd			%ymm14, 0(%r14, %r15, 2)

	vmovapd			32(%r14), %ymm12
	vmovapd			32(%r14, %r15, 1), %ymm13
	vmovapd			32(%r14, %r15, 2), %ymm14
	vbroadcastsd	32(%r13), %ymm15
	vfnmadd231pd	%ymm0, %ymm15, %ymm12
	vfnmadd231pd	%ymm4, %ymm15, %ymm13
	vfnmadd231pd	%ymm8, %ymm15, %ymm14
	vbroadcastsd	40(%r13), %ymm15
	vfnmadd231pd	%ymm1, %ymm15, %ymm12
	vfnmadd231pd	%ymm5, %ymm15, %ymm13
	vfnmadd231pd	%ymm9, %ymm15, %ymm14
	vbroadcastsd	48(%r13), %ymm15
	vfnmadd231pd	%ymm2, %ymm15, %ymm12
	vfnmadd231pd	%ymm6, %ymm15, %ymm13
	vfnmadd231pd	%ymm10, %ymm15, %ymm14
	vbroadcastsd	56(%r13), %ymm15
	vfnmadd231pd	%ymm3, %ymm15, %ymm12
	vfnmadd231pd	%ymm7, %ymm15, %ymm13
	vfnmadd231pd	%ymm11, %ymm15, %ymm14
	vmovapd			%ymm12, 32(%r14)
	vmovapd			%ymm13, 32(%r14, %r15, 1)
	vmovapd			%ymm14, 32(%r14, %r15, 2)

	vmovapd			64(%r14), %ymm12
	vmovapd			64(%r14, %r15, 1), %ymm13
	vmovapd			64(%r14, %r15, 2), %ymm14
	vbroadcastsd	64(%r13), %ymm15
	vfnmadd231pd	%ymm0, %ymm15, %ymm12
	vfnmadd231pd	%ymm4, %ymm15, %ymm13
	vfnmadd231pd	%ymm8, %ymm15, %ymm14
	vbroadcastsd	72(%r13), %ymm15
	vfnmadd231pd	%ymm1, %ymm15, %ymm12
	vfnmadd231pd	%ymm5, %ymm15, %ymm13
	vfnmadd231pd	%ymm9, %ymm15, %ymm14
	vbroadcastsd	80(%r13), %ymm15
	vfnmadd231pd	%ymm2, %ymm15, %ymm12
	vfnmadd231pd	%ymm6, %ymm15, %ymm13
	vfnmadd231pd	%ymm10, %ymm15, %ymm14
	vbroadcastsd	88(%r13), %ymm15
	vfnmadd231pd	%ymm3, %ymm15, %ymm12
	vfnmadd231pd	%ymm7, %ymm15, %ymm13
	vfnmadd231pd	%ymm11, %ymm15, %ymm14
	vmovapd			%ymm12, 64(%r14)
	vmovapd			%ymm13, 64(%r14, %r15, 1)
	vmovapd			%ymm14, 64(%r14, %r15, 2)

	vmovapd			96(%r14), %ymm12
	vmovapd			96(%r14, %r15, 1), %ymm13
	vmovapd			96(%r14, %r15, 2), %ymm14
	vbroadcastsd	96(%r13), %ymm15
	addq	$ 128, %r13
	vfnmadd231pd	%ymm0, %ymm15, %ymm12
	vfnmadd231pd	%ymm4, %ymm15, %ymm13
	vfnmadd231pd	%ymm8, %ymm15, %ymm14
	vbroadcastsd	-24(%r13), %ymm15
	vfnmadd231pd	%ymm1, %ymm15, %ymm12
	vfnmadd231pd	%ymm5, %ymm15, %ymm13
	vfnmadd231pd	%ymm9, %ymm15, %ymm14
	vbroadcastsd	-16(%r13), %ymm15
	vfnmadd231pd	%ymm2, %ymm15, %ymm12
	vfnmadd231pd	%ymm6, %ymm15, %ymm13
	vfnmadd231pd	%ymm10, %ymm15, %ymm14
	vbroadcastsd	-8(%r13), %ymm15
	addq	$ 128, %r14
	vfnmadd231pd	%ymm3, %ymm15, %ymm12
	vfnmadd231pd	%ymm7, %ymm15, %ymm13
	vfnmadd231pd	%ymm11, %ymm15, %ymm14
	vmovapd			%ymm12, -32(%r14)
	vmovapd			%ymm13, -32(%r14, %r15, 1)
	vmovapd			%ymm14, -32(%r14, %r15, 2)

	cmpl	$ 3, %r10d
	jg		1b // main loop

	cmpl	$ 0, %r10d
	jle		0f // return

	// cleanup loop
2:
	vmovapd			0(%r14), %ymm12
	vmovapd			0(%r14, %r15, 1), %ymm13
	vmovapd			0(%r14, %r15, 2), %ymm14
	vbroadcastsd	0(%r13), %ymm15
	vfnmadd231pd	%ymm0, %ymm15, %ymm12
	vfnmadd231pd	%ymm4, %ymm15, %ymm13
	vfnmadd231pd	%ymm8, %ymm15, %ymm14
	vbroadcastsd	8(%r13), %ymm15
	vfnmadd231pd	%ymm1, %ymm15, %ymm12
	vfnmadd231pd	%ymm5, %ymm15, %ymm13
	vfnmadd231pd	%ymm9, %ymm15, %ymm14
	vbroadcastsd	16(%r13), %ymm15
	vfnmadd231pd	%ymm2, %ymm15, %ymm12
	vfnmadd231pd	%ymm6, %ymm15, %ymm13
	vfnmadd231pd	%ymm10, %ymm15, %ymm14
	vbroadcastsd	24(%r13), %ymm15
	vfnmadd231pd	%ymm3, %ymm15, %ymm12
	vfnmadd231pd	%ymm7, %ymm15, %ymm13
	vfnmadd231pd	%ymm11, %ymm15, %ymm14
	vmovapd			%ymm12, 0(%r14)
	vmovapd			%ymm13, 0(%r14, %r15, 1)
	vmovapd			%ymm14, 0(%r14, %r15, 2)

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
	.size	kernel_dger4_sub_12r_vs_lib4, .-kernel_dger4_sub_12r_vs_lib4
#endif





//                              1      2          3        4          5
// void kernel_dger4_sub_8_lib4(int k, double *A, int sda, double *B, double *C)

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
	vfnmadd231pd	%ymm0, %ymm15, %ymm8
	vfnmadd231pd	%ymm4, %ymm15, %ymm9
	vbroadcastsd	8(%r13), %ymm15
	subl	$ 4, %r10d
	vfnmadd231pd	%ymm1, %ymm15, %ymm8
	vfnmadd231pd	%ymm5, %ymm15, %ymm9
	vbroadcastsd	16(%r13), %ymm15
	vfnmadd231pd	%ymm2, %ymm15, %ymm8
	vfnmadd231pd	%ymm6, %ymm15, %ymm9
	vbroadcastsd	24(%r13), %ymm15
	vfnmadd231pd	%ymm3, %ymm15, %ymm8
	vfnmadd231pd	%ymm7, %ymm15, %ymm9
	vmovapd			%ymm8, 0(%r14)
	vmovapd			%ymm9, 0(%r14, %r15, 1)

	vmovapd			32(%r14), %ymm8
	vmovapd			32(%r14, %r15, 1), %ymm9
	vbroadcastsd	32(%r13), %ymm15
	vfnmadd231pd	%ymm0, %ymm15, %ymm8
	vfnmadd231pd	%ymm4, %ymm15, %ymm9
	vbroadcastsd	40(%r13), %ymm15
	vfnmadd231pd	%ymm1, %ymm15, %ymm8
	vfnmadd231pd	%ymm5, %ymm15, %ymm9
	vbroadcastsd	48(%r13), %ymm15
	vfnmadd231pd	%ymm2, %ymm15, %ymm8
	vfnmadd231pd	%ymm6, %ymm15, %ymm9
	vbroadcastsd	56(%r13), %ymm15
	vfnmadd231pd	%ymm3, %ymm15, %ymm8
	vfnmadd231pd	%ymm7, %ymm15, %ymm9
	vmovapd			%ymm8, 32(%r14)
	vmovapd			%ymm9, 32(%r14, %r15, 1)

	vmovapd			64(%r14), %ymm8
	vmovapd			64(%r14, %r15, 1), %ymm9
	vbroadcastsd	64(%r13), %ymm15
	vfnmadd231pd	%ymm0, %ymm15, %ymm8
	vfnmadd231pd	%ymm4, %ymm15, %ymm9
	vbroadcastsd	72(%r13), %ymm15
	vfnmadd231pd	%ymm1, %ymm15, %ymm8
	vfnmadd231pd	%ymm5, %ymm15, %ymm9
	vbroadcastsd	80(%r13), %ymm15
	vfnmadd231pd	%ymm2, %ymm15, %ymm8
	vfnmadd231pd	%ymm6, %ymm15, %ymm9
	vbroadcastsd	88(%r13), %ymm15
	vfnmadd231pd	%ymm3, %ymm15, %ymm8
	vfnmadd231pd	%ymm7, %ymm15, %ymm9
	vmovapd			%ymm8, 64(%r14)
	vmovapd			%ymm9, 64(%r14, %r15, 1)

	vmovapd			96(%r14), %ymm8
	vmovapd			96(%r14, %r15, 1), %ymm9
	vbroadcastsd	96(%r13), %ymm15
	addq	$ 128, %r13
	vfnmadd231pd	%ymm0, %ymm15, %ymm8
	vfnmadd231pd	%ymm4, %ymm15, %ymm9
	vbroadcastsd	-24(%r13), %ymm15
	vfnmadd231pd	%ymm1, %ymm15, %ymm8
	vfnmadd231pd	%ymm5, %ymm15, %ymm9
	vbroadcastsd	-16(%r13), %ymm15
	vfnmadd231pd	%ymm2, %ymm15, %ymm8
	vfnmadd231pd	%ymm6, %ymm15, %ymm9
	vbroadcastsd	-8(%r13), %ymm15
	addq	$ 128, %r14
	vfnmadd231pd	%ymm3, %ymm15, %ymm8
	vfnmadd231pd	%ymm7, %ymm15, %ymm9
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
	vfnmadd231pd	%ymm0, %ymm15, %ymm8
	vfnmadd231pd	%ymm4, %ymm15, %ymm9
	vbroadcastsd	8(%r13), %ymm15
	vfnmadd231pd	%ymm1, %ymm15, %ymm8
	vfnmadd231pd	%ymm5, %ymm15, %ymm9
	vbroadcastsd	16(%r13), %ymm15
	vfnmadd231pd	%ymm2, %ymm15, %ymm8
	vfnmadd231pd	%ymm6, %ymm15, %ymm9
	vbroadcastsd	24(%r13), %ymm15
	vfnmadd231pd	%ymm3, %ymm15, %ymm8
	vfnmadd231pd	%ymm7, %ymm15, %ymm9
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
	vfnmadd231pd	%ymm0, %ymm15, %ymm8
	vfnmadd231pd	%ymm4, %ymm15, %ymm9
	vbroadcastsd	8(%r13), %ymm15
	subl	$ 4, %r10d
	vfnmadd231pd	%ymm1, %ymm15, %ymm8
	vfnmadd231pd	%ymm5, %ymm15, %ymm9
	vbroadcastsd	16(%r13), %ymm15
	vfnmadd231pd	%ymm2, %ymm15, %ymm8
	vfnmadd231pd	%ymm6, %ymm15, %ymm9
	vbroadcastsd	24(%r13), %ymm15
	vfnmadd231pd	%ymm3, %ymm15, %ymm8
	vfnmadd231pd	%ymm7, %ymm15, %ymm9
	vmovapd			%ymm8, 0(%r14)
	vmovapd			%ymm9, 0(%r14, %r15, 1)

	vmovapd			32(%r14), %ymm8
	vmovapd			32(%r14, %r15, 1), %ymm9
	vbroadcastsd	32(%r13), %ymm15
	vfnmadd231pd	%ymm0, %ymm15, %ymm8
	vfnmadd231pd	%ymm4, %ymm15, %ymm9
	vbroadcastsd	40(%r13), %ymm15
	vfnmadd231pd	%ymm1, %ymm15, %ymm8
	vfnmadd231pd	%ymm5, %ymm15, %ymm9
	vbroadcastsd	48(%r13), %ymm15
	vfnmadd231pd	%ymm2, %ymm15, %ymm8
	vfnmadd231pd	%ymm6, %ymm15, %ymm9
	vbroadcastsd	56(%r13), %ymm15
	vfnmadd231pd	%ymm3, %ymm15, %ymm8
	vfnmadd231pd	%ymm7, %ymm15, %ymm9
	vmovapd			%ymm8, 32(%r14)
	vmovapd			%ymm9, 32(%r14, %r15, 1)

	vmovapd			64(%r14), %ymm8
	vmovapd			64(%r14, %r15, 1), %ymm9
	vbroadcastsd	64(%r13), %ymm15
	vfnmadd231pd	%ymm0, %ymm15, %ymm8
	vfnmadd231pd	%ymm4, %ymm15, %ymm9
	vbroadcastsd	72(%r13), %ymm15
	vfnmadd231pd	%ymm1, %ymm15, %ymm8
	vfnmadd231pd	%ymm5, %ymm15, %ymm9
	vbroadcastsd	80(%r13), %ymm15
	vfnmadd231pd	%ymm2, %ymm15, %ymm8
	vfnmadd231pd	%ymm6, %ymm15, %ymm9
	vbroadcastsd	88(%r13), %ymm15
	vfnmadd231pd	%ymm3, %ymm15, %ymm8
	vfnmadd231pd	%ymm7, %ymm15, %ymm9
	vmovapd			%ymm8, 64(%r14)
	vmovapd			%ymm9, 64(%r14, %r15, 1)

	vmovapd			96(%r14), %ymm8
	vmovapd			96(%r14, %r15, 1), %ymm9
	vbroadcastsd	96(%r13), %ymm15
	addq	$ 128, %r13
	vfnmadd231pd	%ymm0, %ymm15, %ymm8
	vfnmadd231pd	%ymm4, %ymm15, %ymm9
	vbroadcastsd	-24(%r13), %ymm15
	vfnmadd231pd	%ymm1, %ymm15, %ymm8
	vfnmadd231pd	%ymm5, %ymm15, %ymm9
	vbroadcastsd	-16(%r13), %ymm15
	vfnmadd231pd	%ymm2, %ymm15, %ymm8
	vfnmadd231pd	%ymm6, %ymm15, %ymm9
	vbroadcastsd	-8(%r13), %ymm15
	addq	$ 128, %r14
	vfnmadd231pd	%ymm3, %ymm15, %ymm8
	vfnmadd231pd	%ymm7, %ymm15, %ymm9
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
	vfnmadd231pd	%ymm0, %ymm15, %ymm8
	vfnmadd231pd	%ymm4, %ymm15, %ymm9
	vbroadcastsd	8(%r13), %ymm15
	vfnmadd231pd	%ymm1, %ymm15, %ymm8
	vfnmadd231pd	%ymm5, %ymm15, %ymm9
	vbroadcastsd	16(%r13), %ymm15
	vfnmadd231pd	%ymm2, %ymm15, %ymm8
	vfnmadd231pd	%ymm6, %ymm15, %ymm9
	vbroadcastsd	24(%r13), %ymm15
	vfnmadd231pd	%ymm3, %ymm15, %ymm8
	vfnmadd231pd	%ymm7, %ymm15, %ymm9
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





//                               1      2          3          4        5
// void kernel_dger12_add_4r_lib4(int n, double *A, double *B, int sdb, double *C)

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dger12_add_4r_lib4
	.type kernel_dger12_add_4r_lib4, @function
kernel_dger12_add_4r_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dger12_add_4r_lib4
_kernel_dger12_add_4r_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dger12_add_4r_lib4
	.def kernel_dger12_add_4r_lib4; .scl 2; .type 32; .endef
kernel_dger12_add_4r_lib4:
#endif
	
	PROLOGUE

	movq	ARG1, %r10 // n
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // B
	movq	ARG4, %r13 // sdb
	sall	$ 5, %r13d
	movq	ARG5, %r14 // C

	cmpl	$ 0, %r10d
	jle		0f // return

	cmpl	$ 11, %r10d
	jle		2f // cleanup loop

	// main loop
	.p2align 3
1:
	// load block from C
	vmovapd	0(%r14), %ymm0
	vmovapd	32(%r14), %ymm1
	vmovapd	64(%r14), %ymm2
	vmovapd	96(%r14), %ymm3
	vmovapd	128(%r14), %ymm4
	vmovapd	160(%r14), %ymm5
	vmovapd	192(%r14), %ymm6
	vmovapd	224(%r14), %ymm7
	vmovapd	256(%r14), %ymm8
	vmovapd	288(%r14), %ymm9
	vmovapd	320(%r14), %ymm10
	vmovapd	352(%r14), %ymm11

	// 0
	vmovapd			0(%r11), %ymm12
	vbroadcastsd	0(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	32(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	64(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	96(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	128(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	160(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	192(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	224(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	256(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	288(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	320(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	352(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11

	// 1
	vmovapd			32(%r11), %ymm12
	vbroadcastsd	8(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	40(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	72(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	104(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	136(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	168(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	200(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	232(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	264(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	296(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	328(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	360(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11

	// 2
	vmovapd			64(%r11), %ymm12
	vbroadcastsd	16(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	48(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	80(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	112(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	144(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	176(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	208(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	240(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	272(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	304(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	336(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	368(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11

	// 3
	vmovapd			96(%r11), %ymm12
	vbroadcastsd	24(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	56(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	88(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	120(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	152(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	184(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	216(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	248(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	280(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	312(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	344(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	376(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11

	// 4
	vmovapd			128(%r11), %ymm12
	vbroadcastsd	0(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	32(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	64(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	96(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	128(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	160(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	192(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	224(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	256(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	288(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	320(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	352(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11

	// 5
	vmovapd			160(%r11), %ymm12
	vbroadcastsd	8(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	40(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	72(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	104(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	136(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	168(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	200(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	232(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	264(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	296(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	328(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	360(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11

	// 6
	vmovapd			192(%r11), %ymm12
	vbroadcastsd	16(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	48(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	80(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	112(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	144(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	176(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	208(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	240(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	272(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	304(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	336(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	368(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11

	// 7
	vmovapd			224(%r11), %ymm12
	vbroadcastsd	24(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	56(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	88(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	120(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	152(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	184(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	216(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	248(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	280(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	312(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	344(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	376(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11

	// 8
	vmovapd			256(%r11), %ymm12
	vbroadcastsd	0(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	32(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	64(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	96(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	128(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	160(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	192(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	224(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	256(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	288(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	320(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	352(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11

	// 9
	vmovapd			288(%r11), %ymm12
	vbroadcastsd	8(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	40(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	72(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	104(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	136(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	168(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	200(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	232(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	264(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	296(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	328(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	360(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11

	// 10
	vmovapd			320(%r11), %ymm12
	vbroadcastsd	16(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	48(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	80(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	112(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	144(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	176(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	208(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	240(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	272(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	304(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	336(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	368(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11

	// 11
	vmovapd			352(%r11), %ymm12
	vbroadcastsd	24(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	56(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	88(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	120(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	152(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	184(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	216(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	248(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	280(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	312(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	344(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	376(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11

	// store block to C
	vmovapd	%ymm0, 0(%r14)
	vmovapd	%ymm1, 32(%r14)
	vmovapd	%ymm2, 64(%r14)
	vmovapd	%ymm3, 96(%r14)
	vmovapd	%ymm4, 128(%r14)
	vmovapd	%ymm5, 160(%r14)
	vmovapd	%ymm6, 192(%r14)
	vmovapd	%ymm7, 224(%r14)
	vmovapd	%ymm8, 256(%r14)
	vmovapd	%ymm9, 288(%r14)
	vmovapd	%ymm10, 320(%r14)
	vmovapd	%ymm11, 352(%r14)

	addq	$ 384, %r12
	addq	$ 384, %r14
	subl	$ 12, %r10d

	cmpl	$ 11, %r10d
	jg		1b // main loop

2:
	cmpl	$ 3, %r10d
	jle		2f // return

	// cleanup loop
1:
	// load block from C
	vmovapd	0(%r14), %ymm0
	vmovapd	32(%r14), %ymm1
	vmovapd	64(%r14), %ymm2
	vmovapd	96(%r14), %ymm3

	// 0
	vmovapd			0(%r11), %ymm12
	vbroadcastsd	0(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	32(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	64(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	96(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3

	// 1
	vmovapd			32(%r11), %ymm12
	vbroadcastsd	8(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	40(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	72(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	104(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3

	// 2
	vmovapd			64(%r11), %ymm12
	vbroadcastsd	16(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	48(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	80(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	112(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3

	// 3
	vmovapd			96(%r11), %ymm12
	vbroadcastsd	24(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	56(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	88(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	120(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3

	// 4
	vmovapd			128(%r11), %ymm12
	vbroadcastsd	0(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	32(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	64(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	96(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3

	// 5
	vmovapd			160(%r11), %ymm12
	vbroadcastsd	8(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	40(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	72(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	104(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3

	// 6
	vmovapd			192(%r11), %ymm12
	vbroadcastsd	16(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	48(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	80(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	112(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3

	// 7
	vmovapd			224(%r11), %ymm12
	vbroadcastsd	24(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	56(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	88(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	120(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3

	// 8
	vmovapd			256(%r11), %ymm12
	vbroadcastsd	0(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	32(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	64(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	96(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3

	// 9
	vmovapd			288(%r11), %ymm12
	vbroadcastsd	8(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	40(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	72(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	104(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3

	// 10
	vmovapd			320(%r11), %ymm12
	vbroadcastsd	16(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	48(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	80(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	112(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3

	// 11
	vmovapd			352(%r11), %ymm12
	vbroadcastsd	24(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	56(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	88(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	120(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3

	// store block to C
	vmovapd	%ymm0, 0(%r14)
	vmovapd	%ymm1, 32(%r14)
	vmovapd	%ymm2, 64(%r14)
	vmovapd	%ymm3, 96(%r14)

	addq	$ 128, %r12
	addq	$ 128, %r14
	subl	$ 4, %r10d

	cmpl	$ 3, %r10d
	jg		1b // main loop

2:
	cmpl	$ 0, %r10d
	jle		0f // return

	// cleanup loop
1:
	// load block from C
	vmovapd	0(%r14), %ymm0

	// 0
	vmovapd			0(%r11), %ymm12
	vbroadcastsd	0(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0

	// 1
	vmovapd			32(%r11), %ymm12
	vbroadcastsd	8(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0

	// 2
	vmovapd			64(%r11), %ymm12
	vbroadcastsd	16(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0

	// 3
	vmovapd			96(%r11), %ymm12
	vbroadcastsd	24(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0

	// 4
	vmovapd			128(%r11), %ymm12
	vbroadcastsd	0(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0

	// 5
	vmovapd			160(%r11), %ymm12
	vbroadcastsd	8(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0

	// 6
	vmovapd			192(%r11), %ymm12
	vbroadcastsd	16(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0

	// 7
	vmovapd			224(%r11), %ymm12
	vbroadcastsd	24(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0

	// 8
	vmovapd			256(%r11), %ymm12
	vbroadcastsd	0(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0

	// 9
	vmovapd			288(%r11), %ymm12
	vbroadcastsd	8(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0

	// 10
	vmovapd			320(%r11), %ymm12
	vbroadcastsd	16(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0

	// 11
	vmovapd			352(%r11), %ymm12
	vbroadcastsd	24(%r12, %r13, 2), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0

	// store block to C
	vmovapd	%ymm0, 0(%r14)

	addq	$ 32, %r12
	addq	$ 32, %r14
	subl	$ 1, %r10d

	cmpl	$ 0, %r10d
	jg		1b // main loop

	// return
0:

	EPILOGUE
	
	ret

#if defined(OS_LINUX)
	.size	kernel_dger12_add_4r_lib4, .-kernel_dger12_add_4r_lib4
#endif





//                               1      2          3          4        5
// void kernel_dger8_add_4r_lib4(int n, double *A, double *B, int sdb, double *C)

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dger8_add_4r_lib4
	.type kernel_dger8_add_4r_lib4, @function
kernel_dger8_add_4r_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dger8_add_4r_lib4
_kernel_dger8_add_4r_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dger8_add_4r_lib4
	.def kernel_dger8_add_4r_lib4; .scl 2; .type 32; .endef
kernel_dger8_add_4r_lib4:
#endif
	
	PROLOGUE

	movq	ARG1, %r10 // n
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // B
	movq	ARG4, %r13 // sdb
	sall	$ 5, %r13d
	movq	ARG5, %r14 // C

	cmpl	$ 0, %r10d
	jle		0f // return

	cmpl	$ 11, %r10d
	jle		2f // cleanup loop

	// main loop
	.p2align 3
1:
	// load block from C
	vmovapd	0(%r14), %ymm0
	vmovapd	32(%r14), %ymm1
	vmovapd	64(%r14), %ymm2
	vmovapd	96(%r14), %ymm3
	vmovapd	128(%r14), %ymm4
	vmovapd	160(%r14), %ymm5
	vmovapd	192(%r14), %ymm6
	vmovapd	224(%r14), %ymm7
	vmovapd	256(%r14), %ymm8
	vmovapd	288(%r14), %ymm9
	vmovapd	320(%r14), %ymm10
	vmovapd	352(%r14), %ymm11

	// 0
	vmovapd			0(%r11), %ymm12
	vbroadcastsd	0(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	32(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	64(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	96(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	128(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	160(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	192(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	224(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	256(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	288(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	320(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	352(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11

	// 1
	vmovapd			32(%r11), %ymm12
	vbroadcastsd	8(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	40(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	72(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	104(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	136(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	168(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	200(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	232(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	264(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	296(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	328(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	360(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11

	// 2
	vmovapd			64(%r11), %ymm12
	vbroadcastsd	16(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	48(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	80(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	112(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	144(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	176(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	208(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	240(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	272(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	304(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	336(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	368(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11

	// 3
	vmovapd			96(%r11), %ymm12
	vbroadcastsd	24(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	56(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	88(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	120(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	152(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	184(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	216(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	248(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	280(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	312(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	344(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	376(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11

	// 4
	vmovapd			128(%r11), %ymm12
	vbroadcastsd	0(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	32(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	64(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	96(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	128(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	160(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	192(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	224(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	256(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	288(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	320(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	352(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11

	// 5
	vmovapd			160(%r11), %ymm12
	vbroadcastsd	8(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	40(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	72(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	104(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	136(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	168(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	200(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	232(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	264(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	296(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	328(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	360(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11

	// 6
	vmovapd			192(%r11), %ymm12
	vbroadcastsd	16(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	48(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	80(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	112(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	144(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	176(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	208(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	240(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	272(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	304(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	336(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	368(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11

	// 7
	vmovapd			224(%r11), %ymm12
	vbroadcastsd	24(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	56(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	88(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	120(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3
	vbroadcastsd	152(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm4
	vbroadcastsd	184(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm5
	vbroadcastsd	216(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm6
	vbroadcastsd	248(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm7
	vbroadcastsd	280(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm8
	vbroadcastsd	312(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm9
	vbroadcastsd	344(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm10
	vbroadcastsd	376(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm11

	// store block to C
	vmovapd	%ymm0, 0(%r14)
	vmovapd	%ymm1, 32(%r14)
	vmovapd	%ymm2, 64(%r14)
	vmovapd	%ymm3, 96(%r14)
	vmovapd	%ymm4, 128(%r14)
	vmovapd	%ymm5, 160(%r14)
	vmovapd	%ymm6, 192(%r14)
	vmovapd	%ymm7, 224(%r14)
	vmovapd	%ymm8, 256(%r14)
	vmovapd	%ymm9, 288(%r14)
	vmovapd	%ymm10, 320(%r14)
	vmovapd	%ymm11, 352(%r14)

	addq	$ 384, %r12
	addq	$ 384, %r14
	subl	$ 12, %r10d

	cmpl	$ 11, %r10d
	jg		1b // main loop

2:
	cmpl	$ 3, %r10d
	jle		2f // return

	// cleanup loop
1:
	// load block from C
	vmovapd	0(%r14), %ymm0
	vmovapd	32(%r14), %ymm1
	vmovapd	64(%r14), %ymm2
	vmovapd	96(%r14), %ymm3

	// 0
	vmovapd			0(%r11), %ymm12
	vbroadcastsd	0(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	32(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	64(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	96(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3

	// 1
	vmovapd			32(%r11), %ymm12
	vbroadcastsd	8(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	40(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	72(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	104(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3

	// 2
	vmovapd			64(%r11), %ymm12
	vbroadcastsd	16(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	48(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	80(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	112(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3

	// 3
	vmovapd			96(%r11), %ymm12
	vbroadcastsd	24(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	56(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	88(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	120(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3

	// 4
	vmovapd			128(%r11), %ymm12
	vbroadcastsd	0(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	32(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	64(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	96(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3

	// 5
	vmovapd			160(%r11), %ymm12
	vbroadcastsd	8(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	40(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	72(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	104(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3

	// 6
	vmovapd			192(%r11), %ymm12
	vbroadcastsd	16(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	48(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	80(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	112(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3

	// 7
	vmovapd			224(%r11), %ymm12
	vbroadcastsd	24(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0
	vbroadcastsd	56(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm1
	vbroadcastsd	88(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm2
	vbroadcastsd	120(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm3

	// store block to C
	vmovapd	%ymm0, 0(%r14)
	vmovapd	%ymm1, 32(%r14)
	vmovapd	%ymm2, 64(%r14)
	vmovapd	%ymm3, 96(%r14)

	addq	$ 128, %r12
	addq	$ 128, %r14
	subl	$ 4, %r10d

	cmpl	$ 3, %r10d
	jg		1b // main loop

2:
	cmpl	$ 0, %r10d
	jle		0f // return

	// cleanup loop
1:
	// load block from C
	vmovapd	0(%r14), %ymm0

	// 0
	vmovapd			0(%r11), %ymm12
	vbroadcastsd	0(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0

	// 1
	vmovapd			32(%r11), %ymm12
	vbroadcastsd	8(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0

	// 2
	vmovapd			64(%r11), %ymm12
	vbroadcastsd	16(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0

	// 3
	vmovapd			96(%r11), %ymm12
	vbroadcastsd	24(%r12), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0

	// 4
	vmovapd			128(%r11), %ymm12
	vbroadcastsd	0(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0

	// 5
	vmovapd			160(%r11), %ymm12
	vbroadcastsd	8(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0

	// 6
	vmovapd			192(%r11), %ymm12
	vbroadcastsd	16(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0

	// 7
	vmovapd			224(%r11), %ymm12
	vbroadcastsd	24(%r12, %r13, 1), %ymm13
	vfmadd231pd		%ymm12, %ymm13, %ymm0

	// store block to C
	vmovapd	%ymm0, 0(%r14)

	addq	$ 32, %r12
	addq	$ 32, %r14
	subl	$ 1, %r10d

	cmpl	$ 0, %r10d
	jg		1b // main loop

	// return
0:

	EPILOGUE
	
	ret

#if defined(OS_LINUX)
	.size	kernel_dger8_add_4r_lib4, .-kernel_dger8_add_4r_lib4
#endif





#if 0
//                               1      2          3          4        5
// void kernel_dger8_sub_4r_lib4(int n, double *A, double *B, int sdb, double *C)

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dger8_add_4r_lib4
	.type kernel_dger8_add_4r_lib4, @function
kernel_dger8_add_4r_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dger8_add_4r_lib4
_kernel_dger8_add_4r_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dger8_add_4r_lib4
	.def kernel_dger8_add_4r_lib4; .scl 2; .type 32; .endef
kernel_dger8_add_4r_lib4:
#endif
	
	PROLOGUE

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12
	movq	ARG4, %r13
	sall	$ 5, %r13d
	movq	ARG5, %r14

	cmpl	$ 0, %r10d
	jle		0f // return

	// load block from A
	vmovapd	0(%r11), %ymm0
	vmovapd	32(%r11), %ymm1
	vmovapd	64(%r11), %ymm2
	vmovapd	96(%r11), %ymm3
	vmovapd	128(%r11), %ymm4
	vmovapd	160(%r11), %ymm5
	vmovapd	192(%r11), %ymm6
	vmovapd	224(%r11), %ymm7

	cmpl	$ 7, %r10d
	jle		2f // cleanup loop

	// main loop
	.p2align 3
1:
	// 04
	vmovapd			0(%r14), %ymm12
	vbroadcastsd	0(%r12), %ymm15
	vfmadd231pd		%ymm0, %ymm15, %ymm12
	vbroadcastsd	8(%r12), %ymm15
	vfmadd231pd		%ymm1, %ymm15, %ymm12
	vbroadcastsd	16(%r12), %ymm15
	vfmadd231pd		%ymm2, %ymm15, %ymm12
	vbroadcastsd	24(%r12), %ymm15
	vfmadd231pd		%ymm3, %ymm15, %ymm12
	vmovapd			%ymm12, 0(%r14)

	// 14
	vmovapd			32(%r14), %ymm12
	vbroadcastsd	32(%r12), %ymm15
	vfmadd231pd		%ymm0, %ymm15, %ymm12
	vbroadcastsd	40(%r12), %ymm15
	vfmadd231pd		%ymm1, %ymm15, %ymm12
	vbroadcastsd	48(%r12), %ymm15
	vfmadd231pd		%ymm2, %ymm15, %ymm12
	vbroadcastsd	56(%r12), %ymm15
	vfmadd231pd		%ymm3, %ymm15, %ymm12
	vmovapd			%ymm12, 32(%r14)

	// 24
	vmovapd			64(%r14), %ymm12
	vbroadcastsd	64(%r12), %ymm15
	vfmadd231pd		%ymm0, %ymm15, %ymm12
	vbroadcastsd	72(%r12), %ymm15
	vfmadd231pd		%ymm1, %ymm15, %ymm12
	vbroadcastsd	80(%r12), %ymm15
	vfmadd231pd		%ymm2, %ymm15, %ymm12
	vbroadcastsd	88(%r12), %ymm15
	vfmadd231pd		%ymm3, %ymm15, %ymm12
	vmovapd			%ymm12, 64(%r14)

	// 34
	vmovapd			96(%r14), %ymm12
	vbroadcastsd	96(%r12), %ymm15
	vfmadd231pd		%ymm0, %ymm15, %ymm12
	vbroadcastsd	104(%r12), %ymm15
	vfmadd231pd		%ymm1, %ymm15, %ymm12
	vbroadcastsd	112(%r12), %ymm15
	vfmadd231pd		%ymm2, %ymm15, %ymm12
	vbroadcastsd	120(%r12), %ymm15
	vfmadd231pd		%ymm3, %ymm15, %ymm12
	vmovapd			%ymm12, 96(%r14)

	// 44
	vmovapd			128(%r14), %ymm12
	vbroadcastsd	128(%r12), %ymm15
	vfmadd231pd		%ymm0, %ymm15, %ymm12
	vbroadcastsd	136(%r12), %ymm15
	vfmadd231pd		%ymm1, %ymm15, %ymm12
	vbroadcastsd	144(%r12), %ymm15
	vfmadd231pd		%ymm2, %ymm15, %ymm12
	vbroadcastsd	152(%r12), %ymm15
	vfmadd231pd		%ymm3, %ymm15, %ymm12
	vmovapd			%ymm12, 128(%r14)

	// 54
	vmovapd			160(%r14), %ymm12
	vbroadcastsd	160(%r12), %ymm15
	vfmadd231pd		%ymm0, %ymm15, %ymm12
	vbroadcastsd	168(%r12), %ymm15
	vfmadd231pd		%ymm1, %ymm15, %ymm12
	vbroadcastsd	176(%r12), %ymm15
	vfmadd231pd		%ymm2, %ymm15, %ymm12
	vbroadcastsd	184(%r12), %ymm15
	vfmadd231pd		%ymm3, %ymm15, %ymm12
	vmovapd			%ymm12, 160(%r14)

	// 64
	vmovapd			192(%r14), %ymm12
	vbroadcastsd	192(%r12), %ymm15
	vfmadd231pd		%ymm0, %ymm15, %ymm12
	vbroadcastsd	200(%r12), %ymm15
	vfmadd231pd		%ymm1, %ymm15, %ymm12
	vbroadcastsd	208(%r12), %ymm15
	vfmadd231pd		%ymm2, %ymm15, %ymm12
	vbroadcastsd	216(%r12), %ymm15
	vfmadd231pd		%ymm3, %ymm15, %ymm12
	vmovapd			%ymm12, 192(%r14)

	// 74
	vmovapd			224(%r14), %ymm12
	vbroadcastsd	224(%r12), %ymm15
	vfmadd231pd		%ymm0, %ymm15, %ymm12
	vbroadcastsd	232(%r12), %ymm15
	vfmadd231pd		%ymm1, %ymm15, %ymm12
	vbroadcastsd	240(%r12), %ymm15
	vfmadd231pd		%ymm2, %ymm15, %ymm12
	vbroadcastsd	248(%r12), %ymm15
	vfmadd231pd		%ymm3, %ymm15, %ymm12
	vmovapd			%ymm12, 224(%r14)

	// 08
	vmovapd			0(%r14), %ymm12
	vbroadcastsd	0(%r12, %r13, 1), %ymm15
	vfmadd231pd		%ymm4, %ymm15, %ymm12
	vbroadcastsd	8(%r12, %r13, 1), %ymm15
	vfmadd231pd		%ymm5, %ymm15, %ymm12
	vbroadcastsd	16(%r12, %r13, 1), %ymm15
	vfmadd231pd		%ymm6, %ymm15, %ymm12
	vbroadcastsd	24(%r12, %r13, 1), %ymm15
	vfmadd231pd		%ymm7, %ymm15, %ymm12
	vmovapd			%ymm12, 0(%r14)

	// 18
	vmovapd			32(%r14), %ymm12
	vbroadcastsd	32(%r12, %r13, 1), %ymm15
	vfmadd231pd		%ymm4, %ymm15, %ymm12
	vbroadcastsd	40(%r12, %r13, 1), %ymm15
	vfmadd231pd		%ymm5, %ymm15, %ymm12
	vbroadcastsd	48(%r12, %r13, 1), %ymm15
	vfmadd231pd		%ymm6, %ymm15, %ymm12
	vbroadcastsd	56(%r12, %r13, 1), %ymm15
	vfmadd231pd		%ymm7, %ymm15, %ymm12
	vmovapd			%ymm12, 32(%r14)

	// 28
	vmovapd			64(%r14), %ymm12
	vbroadcastsd	64(%r12, %r13, 1), %ymm15
	vfmadd231pd		%ymm4, %ymm15, %ymm12
	vbroadcastsd	71(%r12, %r13, 1), %ymm15
	vfmadd231pd		%ymm5, %ymm15, %ymm12
	vbroadcastsd	80(%r12, %r13, 1), %ymm15
	vfmadd231pd		%ymm6, %ymm15, %ymm12
	vbroadcastsd	88(%r12, %r13, 1), %ymm15
	vfmadd231pd		%ymm7, %ymm15, %ymm12
	vmovapd			%ymm12, 64(%r14)

	// 38
	vmovapd			96(%r14), %ymm12
	vbroadcastsd	96(%r12, %r13, 1), %ymm15
	vfmadd231pd		%ymm4, %ymm15, %ymm12
	vbroadcastsd	104(%r12, %r13, 1), %ymm15
	vfmadd231pd		%ymm5, %ymm15, %ymm12
	vbroadcastsd	112(%r12, %r13, 1), %ymm15
	vfmadd231pd		%ymm6, %ymm15, %ymm12
	vbroadcastsd	120(%r12, %r13, 1), %ymm15
	vfmadd231pd		%ymm7, %ymm15, %ymm12
	vmovapd			%ymm12, 96(%r14)

	// 48
	vmovapd			128(%r14), %ymm12
	vbroadcastsd	128(%r12, %r13, 1), %ymm15
	vfmadd231pd		%ymm4, %ymm15, %ymm12
	vbroadcastsd	136(%r12, %r13, 1), %ymm15
	vfmadd231pd		%ymm5, %ymm15, %ymm12
	vbroadcastsd	144(%r12, %r13, 1), %ymm15
	vfmadd231pd		%ymm6, %ymm15, %ymm12
	vbroadcastsd	152(%r12, %r13, 1), %ymm15
	vfmadd231pd		%ymm7, %ymm15, %ymm12
	vmovapd			%ymm12, 128(%r14)

	// 58
	vmovapd			160(%r14), %ymm12
	vbroadcastsd	160(%r12, %r13, 1), %ymm15
	vfmadd231pd		%ymm4, %ymm15, %ymm12
	vbroadcastsd	168(%r12, %r13, 1), %ymm15
	vfmadd231pd		%ymm5, %ymm15, %ymm12
	vbroadcastsd	176(%r12, %r13, 1), %ymm15
	vfmadd231pd		%ymm6, %ymm15, %ymm12
	vbroadcastsd	184(%r12, %r13, 1), %ymm15
	vfmadd231pd		%ymm7, %ymm15, %ymm12
	vmovapd			%ymm12, 160(%r14)

	// 68
	vmovapd			192(%r14), %ymm12
	vbroadcastsd	192(%r12, %r13, 1), %ymm15
	vfmadd231pd		%ymm4, %ymm15, %ymm12
	vbroadcastsd	200(%r12, %r13, 1), %ymm15
	vfmadd231pd		%ymm5, %ymm15, %ymm12
	vbroadcastsd	208(%r12, %r13, 1), %ymm15
	vfmadd231pd		%ymm6, %ymm15, %ymm12
	vbroadcastsd	216(%r12, %r13, 1), %ymm15
	vfmadd231pd		%ymm7, %ymm15, %ymm12
	vmovapd			%ymm12, 192(%r14)

	// 78
	vmovapd			224(%r14), %ymm12
	vbroadcastsd	224(%r12, %r13, 1), %ymm15
	vfmadd231pd		%ymm4, %ymm15, %ymm12
	vbroadcastsd	232(%r12, %r13, 1), %ymm15
	vfmadd231pd		%ymm5, %ymm15, %ymm12
	vbroadcastsd	240(%r12, %r13, 1), %ymm15
	vfmadd231pd		%ymm6, %ymm15, %ymm12
	vbroadcastsd	248(%r12, %r13, 1), %ymm15
	vfmadd231pd		%ymm7, %ymm15, %ymm12
	vmovapd			%ymm12, 224(%r14)

	addq	$ 256, %r12
	addq	$ 256, %r14
	subl	$ 8, %r10d

	cmpl	$ 7, %r10d
	jg		1b // main loop

	cmpl	$ 0, %r10d
	jle		0f // return

	// cleanup loop
2:
	vmovapd			0(%r14), %ymm12
	vbroadcastsd	0(%r12), %ymm15
	vfmadd231pd		%ymm0, %ymm15, %ymm12
	vbroadcastsd	8(%r12), %ymm15
	vfmadd231pd		%ymm1, %ymm15, %ymm12
	vbroadcastsd	16(%r12), %ymm15
	vfmadd231pd		%ymm2, %ymm15, %ymm12
	vbroadcastsd	24(%r12), %ymm15
	vfmadd231pd		%ymm3, %ymm15, %ymm12
	vbroadcastsd	0(%r12, %r13, 1), %ymm15
	vfmadd231pd		%ymm4, %ymm15, %ymm12
	vbroadcastsd	8(%r12, %r13, 1), %ymm15
	vfmadd231pd		%ymm5, %ymm15, %ymm12
	vbroadcastsd	16(%r12, %r13, 1), %ymm15
	vfmadd231pd		%ymm6, %ymm15, %ymm12
	vbroadcastsd	24(%r12, %r13, 1), %ymm15
	vfmadd231pd		%ymm7, %ymm15, %ymm12
	vmovapd			%ymm12, 0(%r14)

	addq	$ 32, %r12
	addq	$ 32, %r14

	subl	$ 1, %r10d
	cmpl	$ 0, %r10d
	jg		2b // main loop

	// return
0:

	EPILOGUE
	
	ret

#if defined(OS_LINUX)
	.size	kernel_dger8_add_4r_lib4, .-kernel_dger8_add_4r_lib4
#endif
#endif





//                              1      2          3          4
// void kernel_dger4_sub_4_lib4(int n, double *A, double *B, double *C)

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
	vfnmadd231pd	%ymm0, %ymm15, %ymm4
	vbroadcastsd	8(%r12), %ymm15
	subl	$ 4, %r10d
	vfnmadd231pd	%ymm1, %ymm15, %ymm4
	vbroadcastsd	16(%r12), %ymm15
	vfnmadd231pd	%ymm2, %ymm15, %ymm4
	vbroadcastsd	24(%r12), %ymm15
	vfnmadd231pd	%ymm3, %ymm15, %ymm4
	vmovapd			%ymm4, 0(%r13)

	vmovapd			32(%r13), %ymm4
	vbroadcastsd	32(%r12), %ymm15
	vfnmadd231pd	%ymm0, %ymm15, %ymm4
	vbroadcastsd	40(%r12), %ymm15
	vfnmadd231pd	%ymm1, %ymm15, %ymm4
	vbroadcastsd	48(%r12), %ymm15
	vfnmadd231pd	%ymm2, %ymm15, %ymm4
	vbroadcastsd	56(%r12), %ymm15
	vfnmadd231pd	%ymm3, %ymm15, %ymm4
	vmovapd			%ymm4, 32(%r13)

	vmovapd			64(%r13), %ymm4
	vbroadcastsd	64(%r12), %ymm15
	vfnmadd231pd	%ymm0, %ymm15, %ymm4
	vbroadcastsd	72(%r12), %ymm15
	vfnmadd231pd	%ymm1, %ymm15, %ymm4
	vbroadcastsd	80(%r12), %ymm15
	vfnmadd231pd	%ymm2, %ymm15, %ymm4
	vbroadcastsd	88(%r12), %ymm15
	vfnmadd231pd	%ymm3, %ymm15, %ymm4
	vmovapd			%ymm4, 64(%r13)

	vmovapd			96(%r13), %ymm4
	vbroadcastsd	96(%r12), %ymm15
	addq	$ 128, %r12
	vfnmadd231pd	%ymm0, %ymm15, %ymm4
	vbroadcastsd	-24(%r12), %ymm15
	vfnmadd231pd	%ymm1, %ymm15, %ymm4
	vbroadcastsd	-16(%r12), %ymm15
	vfnmadd231pd	%ymm2, %ymm15, %ymm4
	vbroadcastsd	-8(%r12), %ymm15
	addq	$ 128, %r13
	vfnmadd231pd	%ymm3, %ymm15, %ymm4
	vmovapd			%ymm4, -32(%r13)

	cmpl	$ 3, %r10d
	jg		1b // main loop

	cmpl	$ 0, %r10d
	jle		0f // return

	// cleanup loop
2:
	vmovapd			0(%r13), %ymm4
	vbroadcastsd	0(%r12), %ymm15
	vfnmadd231pd	%ymm0, %ymm15, %ymm4
	vbroadcastsd	8(%r12), %ymm15
	vfnmadd231pd	%ymm1, %ymm15, %ymm4
	vbroadcastsd	16(%r12), %ymm15
	vfnmadd231pd	%ymm2, %ymm15, %ymm4
	vbroadcastsd	24(%r12), %ymm15
	vfnmadd231pd	%ymm3, %ymm15, %ymm4
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





//                              1      2          3          4
// void kernel_dger2_sub_4_lib4(int n, double *A, double *B, double *C)

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dger2_sub_4r_lib4
	.type kernel_dger2_sub_4r_lib4, @function
kernel_dger2_sub_4r_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dger2_sub_4r_lib4
_kernel_dger2_sub_4r_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dger2_sub_4r_lib4
	.def kernel_dger2_sub_4r_lib4; .scl 2; .type 32; .endef
kernel_dger2_sub_4r_lib4:
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

	cmpl	$ 3, %r10d
	jle		2f // cleanup loop

	// main loop
	.p2align 3
1:
	vmovapd			0(%r13), %ymm4
	vbroadcastsd	0(%r12), %ymm15
	vfnmadd231pd	%ymm0, %ymm15, %ymm4
	vbroadcastsd	8(%r12), %ymm15
	subl	$ 4, %r10d
	vfnmadd231pd	%ymm1, %ymm15, %ymm4
	vmovapd			%ymm4, 0(%r13)

	vmovapd			32(%r13), %ymm4
	vbroadcastsd	32(%r12), %ymm15
	vfnmadd231pd	%ymm0, %ymm15, %ymm4
	vbroadcastsd	40(%r12), %ymm15
	vfnmadd231pd	%ymm1, %ymm15, %ymm4
	vmovapd			%ymm4, 32(%r13)

	vmovapd			64(%r13), %ymm4
	vbroadcastsd	64(%r12), %ymm15
	vfnmadd231pd	%ymm0, %ymm15, %ymm4
	vbroadcastsd	72(%r12), %ymm15
	vfnmadd231pd	%ymm1, %ymm15, %ymm4
	vmovapd			%ymm4, 64(%r13)

	vmovapd			96(%r13), %ymm4
	vbroadcastsd	96(%r12), %ymm15
	addq	$ 128, %r12
	vfnmadd231pd	%ymm0, %ymm15, %ymm4
	vbroadcastsd	-24(%r12), %ymm15
	vfnmadd231pd	%ymm1, %ymm15, %ymm4
	addq	$ 128, %r13
	vmovapd			%ymm4, -32(%r13)

	cmpl	$ 3, %r10d
	jg		1b // main loop

	cmpl	$ 0, %r10d
	jle		0f // return

	// cleanup loop
2:
	vmovapd			0(%r13), %ymm4
	vbroadcastsd	0(%r12), %ymm15
	vfnmadd231pd	%ymm0, %ymm15, %ymm4
	vbroadcastsd	8(%r12), %ymm15
	vfnmadd231pd	%ymm1, %ymm15, %ymm4
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
	.size	kernel_dger2_sub_4r_lib4, .-kernel_dger2_sub_4r_lib4
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
	vfnmadd231pd	%ymm0, %ymm15, %ymm4
	vbroadcastsd	8(%r12), %ymm15
	subl	$ 4, %r10d
	vfnmadd231pd	%ymm1, %ymm15, %ymm4
	vbroadcastsd	16(%r12), %ymm15
	vfnmadd231pd	%ymm2, %ymm15, %ymm4
	vbroadcastsd	24(%r12), %ymm15
	vfnmadd231pd	%ymm3, %ymm15, %ymm4
	vmovapd			%ymm4, 0(%r13)

	vmovapd			32(%r13), %ymm4
	vbroadcastsd	32(%r12), %ymm15
	vfnmadd231pd	%ymm0, %ymm15, %ymm4
	vbroadcastsd	40(%r12), %ymm15
	vfnmadd231pd	%ymm1, %ymm15, %ymm4
	vbroadcastsd	48(%r12), %ymm15
	vfnmadd231pd	%ymm2, %ymm15, %ymm4
	vbroadcastsd	56(%r12), %ymm15
	vfnmadd231pd	%ymm3, %ymm15, %ymm4
	vmovapd			%ymm4, 32(%r13)

	vmovapd			64(%r13), %ymm4
	vbroadcastsd	64(%r12), %ymm15
	vfnmadd231pd	%ymm0, %ymm15, %ymm4
	vbroadcastsd	72(%r12), %ymm15
	vfnmadd231pd	%ymm1, %ymm15, %ymm4
	vbroadcastsd	80(%r12), %ymm15
	vfnmadd231pd	%ymm2, %ymm15, %ymm4
	vbroadcastsd	88(%r12), %ymm15
	vfnmadd231pd	%ymm3, %ymm15, %ymm4
	vmovapd			%ymm4, 64(%r13)

	vmovapd			96(%r13), %ymm4
	vbroadcastsd	96(%r12), %ymm15
	addq	$ 128, %r12
	vfnmadd231pd	%ymm0, %ymm15, %ymm4
	vbroadcastsd	-24(%r12), %ymm15
	vfnmadd231pd	%ymm1, %ymm15, %ymm4
	vbroadcastsd	-16(%r12), %ymm15
	vfnmadd231pd	%ymm2, %ymm15, %ymm4
	vbroadcastsd	-8(%r12), %ymm15
	addq	$ 128, %r13
	vfnmadd231pd	%ymm3, %ymm15, %ymm4
	vmovapd			%ymm4, -32(%r13)

	cmpl	$ 3, %r10d
	jg		1b // main loop

	cmpl	$ 0, %r10d
	jle		0f // return

	// cleanup loop
2:
	vmovapd			0(%r13), %ymm4
	vbroadcastsd	0(%r12), %ymm15
	vfnmadd231pd	%ymm0, %ymm15, %ymm4
	vbroadcastsd	8(%r12), %ymm15
	vfnmadd231pd	%ymm1, %ymm15, %ymm4
	vbroadcastsd	16(%r12), %ymm15
	vfnmadd231pd	%ymm2, %ymm15, %ymm4
	vbroadcastsd	24(%r12), %ymm15
	vfnmadd231pd	%ymm3, %ymm15, %ymm4
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

