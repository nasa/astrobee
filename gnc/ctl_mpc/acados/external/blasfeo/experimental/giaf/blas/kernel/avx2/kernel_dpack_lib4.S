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



#if defined(OS_LINUX)
	.text
#elif defined(OS_MAC)
	.section	__TEXT,__text,regular,pure_instructions
#endif





#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DPACK_TN_4_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_dpack_tn_4_lib4, @function
inner_kernel_dpack_tn_4_lib4:
#elif defined(OS_MAC)
_inner_kernel_dpack_tn_4_lib4:
#elif defined(OS_WINDOWS)
	.def inner_kernel_dpack_tn_4_lib4; .scl 2; .type 32; .endef
inner_kernel_dpack_tn_4_lib4:
#endif
#endif

	cmpl	$0, %r10d
	jle		0f // return


	movq	%r11, %r14
	addq	%r12, %r14
	addq	%r12, %r14


	cmpl	$3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovupd		0(%r11), %xmm0
	vmovupd		0(%r14), %xmm15
	vinsertf128	$0x1, %xmm15, %ymm0, %ymm0
	vmovupd		0(%r11, %r12, 1), %xmm1
	vmovupd		0(%r14, %r12, 1), %xmm15
	vinsertf128	$0x1, %xmm15, %ymm1, %ymm1
	vmovupd		16(%r11), %xmm2
	vmovupd		16(%r14), %xmm15
	vinsertf128	$0x1, %xmm15, %ymm2, %ymm2
	vmovupd		16(%r11, %r12, 1), %xmm3
	vmovupd		16(%r14, %r12, 1), %xmm15
	vinsertf128	$0x1, %xmm15, %ymm3, %ymm3

	vunpcklpd	%ymm1, %ymm0, %ymm4
	vmovapd		%ymm4, 0(%r13)
	vunpckhpd	%ymm1, %ymm0, %ymm5
	vmovapd		%ymm5, 32(%r13)
	vunpcklpd	%ymm3, %ymm2, %ymm6
	vmovapd		%ymm6, 64(%r13)
	vunpckhpd	%ymm3, %ymm2, %ymm7
	vmovapd		%ymm7, 96(%r13)

	subl	$4, %r10d
	addq	$32, %r11
	addq	$32, %r14
	addq	$128, %r13

	cmpl	$3, %r10d
	jg		1b


	cmpl	$0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13)
	vmovsd		0(%r11, %r12, 1), %xmm0
	vmovsd		%xmm0, 8(%r13)
	vmovsd		0(%r14), %xmm0
	vmovsd		%xmm0, 16(%r13)
	vmovsd		0(%r14, %r12, 1), %xmm0
	vmovsd		%xmm0, 24(%r13)

	subl	$1, %r10d
	addq	$8, %r11
	addq	$8, %r14
	addq	$32, %r13

	cmpl	$0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_dpack_tn_4_lib4, .-inner_kernel_dpack_tn_4_lib4
#endif
#endif





#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DPACK_TN_3_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_dpack_tn_3_lib4, @function
inner_kernel_dpack_tn_3_lib4:
#elif defined(OS_MAC)
_inner_kernel_dpack_tn_3_lib4:
#elif defined(OS_WINDOWS)
	.def inner_kernel_dpack_tn_3_lib4; .scl 2; .type 32; .endef
inner_kernel_dpack_tn_3_lib4:
#endif
#endif

	cmpl	$0, %r10d
	jle		0f // return


	movq	%r11, %r14
	addq	%r12, %r14
	addq	%r12, %r14


	cmpl	$3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovupd		0(%r11), %xmm0
	vmovupd		0(%r14), %xmm15
	vinsertf128	$0x1, %xmm15, %ymm0, %ymm0
	vmovupd		0(%r11, %r12, 1), %xmm1
	vmovupd		16(%r11), %xmm2
	vmovupd		16(%r14), %xmm15
	vinsertf128	$0x1, %xmm15, %ymm2, %ymm2
	vmovupd		16(%r11, %r12, 1), %xmm3

	vunpcklpd	%ymm1, %ymm0, %ymm4
	vmovapd		%ymm4, 0(%r13)
	vunpckhpd	%ymm1, %ymm0, %ymm5
	vmovapd		%ymm5, 32(%r13)
	vunpcklpd	%ymm3, %ymm2, %ymm6
	vmovapd		%ymm6, 64(%r13)
	vunpckhpd	%ymm3, %ymm2, %ymm7
	vmovapd		%ymm7, 96(%r13)

	subl	$4, %r10d
	addq	$32, %r11
	addq	$32, %r14
	addq	$128, %r13

	cmpl	$3, %r10d
	jg		1b


	cmpl	$0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13)
	vmovsd		0(%r11, %r12, 1), %xmm0
	vmovsd		%xmm0, 8(%r13)
	vmovsd		0(%r14), %xmm0
	vmovsd		%xmm0, 16(%r13)

	subl	$1, %r10d
	addq	$8, %r11
	addq	$8, %r14
	addq	$32, %r13

	cmpl	$0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_dpack_tn_3_lib4, .-inner_kernel_dpack_tn_3_lib4
#endif
#endif





#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DPACK_TN_2_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_dpack_tn_2_lib4, @function
inner_kernel_dpack_tn_2_lib4:
#elif defined(OS_MAC)
_inner_kernel_dpack_tn_2_lib4:
#elif defined(OS_WINDOWS)
	.def inner_kernel_dpack_tn_2_lib4; .scl 2; .type 32; .endef
inner_kernel_dpack_tn_2_lib4:
#endif
#endif

	cmpl	$0, %r10d
	jle		0f // return


	movq	%r11, %r14
	addq	%r12, %r14
	addq	%r12, %r14


	cmpl	$3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovupd		0(%r11), %xmm0
	vmovupd		0(%r11, %r12, 1), %xmm1
	vmovupd		16(%r11), %xmm2
	vmovupd		16(%r11, %r12, 1), %xmm3

	vunpcklpd	%ymm1, %ymm0, %ymm4
	vmovapd		%ymm4, 0(%r13)
	vunpckhpd	%ymm1, %ymm0, %ymm5
	vmovapd		%ymm5, 32(%r13)
	vunpcklpd	%ymm3, %ymm2, %ymm6
	vmovapd		%ymm6, 64(%r13)
	vunpckhpd	%ymm3, %ymm2, %ymm7
	vmovapd		%ymm7, 96(%r13)

	subl	$4, %r10d
	addq	$32, %r11
	addq	$32, %r14
	addq	$128, %r13

	cmpl	$3, %r10d
	jg		1b


	cmpl	$0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13)
	vmovsd		0(%r11, %r12, 1), %xmm0
	vmovsd		%xmm0, 8(%r13)

	subl	$1, %r10d
	addq	$8, %r11
	addq	$8, %r14
	addq	$32, %r13

	cmpl	$0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_dpack_tn_2_lib4, .-inner_kernel_dpack_tn_2_lib4
#endif
#endif





#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DPACK_TN_1_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_dpack_tn_1_lib4, @function
inner_kernel_dpack_tn_1_lib4:
#elif defined(OS_MAC)
_inner_kernel_dpack_tn_1_lib4:
#elif defined(OS_WINDOWS)
	.def inner_kernel_dpack_tn_1_lib4; .scl 2; .type 32; .endef
inner_kernel_dpack_tn_1_lib4:
#endif
#endif

	cmpl	$0, %r10d
	jle		0f // return


	cmpl	$3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13)
	vmovsd		8(%r11), %xmm0
	vmovsd		%xmm0, 32(%r13)
	vmovsd		16(%r11), %xmm0
	vmovsd		%xmm0, 64(%r13)
	vmovsd		24(%r11), %xmm0
	vmovsd		%xmm0, 96(%r13)

	subl	$4, %r10d
	addq	$32, %r11
	addq	$128, %r13

	cmpl	$3, %r10d
	jg		1b


	cmpl	$0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13)

	subl	$1, %r10d
	addq	$8, %r11
	addq	$8, %r14
	addq	$32, %r13

	cmpl	$0, %r10d
	jg		2b


0: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

#if defined(OS_LINUX)
	.size	inner_kernel_dpack_tn_1_lib4, .-inner_kernel_dpack_tn_1_lib4
#endif
#endif





//                              1         2          3        4          5
// void kernel_dpack_nn_12_lib4(int kmax, double *A, int lda, double *C, int sdc)

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dpack_nn_12_lib4
	.type kernel_dpack_nn_12_lib4, @function
kernel_dpack_nn_12_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dpack_nn_12_lib4
_kernel_dpack_nn_12_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dpack_nn_12_lib4
	.def kernel_dpack_nn_12_lib4; .scl 2; .type 32; .endef
kernel_dpack_nn_12_lib4:
#endif

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$3, %r12d
	movq	ARG4, %r13 // C
	movq	ARG5, %r14 // sdc
	sall	$5, %r14d


	cmpl	$0, %r10d
	jle		0f // return


	cmpl	$3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13)
	vmovupd		32(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13, %r14, 1)
	vmovupd		64(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13, %r14, 2)
	addq	%r12, %r11

	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 32(%r13)
	vmovupd		32(%r11), %ymm0
	vmovapd		%ymm0, 32(%r13, %r14, 1)
	vmovupd		64(%r11), %ymm0
	vmovapd		%ymm0, 32(%r13, %r14, 2)
	addq	%r12, %r11

	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 64(%r13)
	vmovupd		32(%r11), %ymm0
	vmovapd		%ymm0, 64(%r13, %r14, 1)
	vmovupd		64(%r11), %ymm0
	vmovapd		%ymm0, 64(%r13, %r14, 2)
	addq	%r12, %r11

	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 96(%r13)
	vmovupd		32(%r11), %ymm0
	vmovapd		%ymm0, 96(%r13, %r14, 1)
	vmovupd		64(%r11), %ymm0
	vmovapd		%ymm0, 96(%r13, %r14, 2)
	addq	%r12, %r11

	subl	$4, %r10d
	addq	$128, %r13

	cmpl	$3, %r10d
	jg		1b


	cmpl	$0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13)
	vmovupd		32(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13, %r14, 1)
	vmovupd		64(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13, %r14, 2)
	addq	%r12, %r11

	subl	$1, %r10d
	addq	$32, %r13

	cmpl	$0, %r10d
	jg		2b


0: // return

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dpack_nn_12_lib4, .-kernel_dpack_nn_12_lib4
#endif





//                                 1         2          3        4          5        6
// void kernel_dpack_nn_12_vs_lib4(int kmax, double *A, int lda, double *C, int sdc, int m1)

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dpack_nn_12_vs_lib4
	.type kernel_dpack_nn_12_vs_lib4, @function
kernel_dpack_nn_12_vs_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dpack_nn_12_vs_lib4
_kernel_dpack_nn_12_vs_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dpack_nn_12_vs_lib4
	.def kernel_dpack_nn_12_vs_lib4; .scl 2; .type 32; .endef
kernel_dpack_nn_12_vs_lib4:
#endif

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$3, %r12d
	movq	ARG4, %r13 // C
	movq	ARG5, %r14 // sdc
	sall	$5, %r14d
	movq	ARG6, %r15 // m1


	cmpl	$0, %r10d
	jle		0f // return


	vcvtsi2sd	%r15d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC02(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC02(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm15


	cmpl	$3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13)
	vmovupd		32(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13, %r14, 1)
	vmaskmovpd	64(%r11), %ymm15, %ymm0
	vmovapd		%ymm0, 0(%r13, %r14, 2)
	addq	%r12, %r11

	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 32(%r13)
	vmovupd		32(%r11), %ymm0
	vmovapd		%ymm0, 32(%r13, %r14, 1)
	vmaskmovpd	64(%r11), %ymm15, %ymm0
	vmovapd		%ymm0, 32(%r13, %r14, 2)
	addq	%r12, %r11

	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 64(%r13)
	vmovupd		32(%r11), %ymm0
	vmovapd		%ymm0, 64(%r13, %r14, 1)
	vmaskmovpd	64(%r11), %ymm15, %ymm0
	vmovapd		%ymm0, 64(%r13, %r14, 2)
	addq	%r12, %r11

	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 96(%r13)
	vmovupd		32(%r11), %ymm0
	vmovapd		%ymm0, 96(%r13, %r14, 1)
	vmaskmovpd	64(%r11), %ymm15, %ymm0
	vmovapd		%ymm0, 96(%r13, %r14, 2)
	addq	%r12, %r11

	subl	$4, %r10d
	addq	$128, %r13

	cmpl	$3, %r10d
	jg		1b


	cmpl	$0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13)
	vmovupd		32(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13, %r14, 1)
	vmaskmovpd	64(%r11), %ymm15, %ymm0
	vmovapd		%ymm0, 0(%r13, %r14, 2)
	addq	%r12, %r11

	subl	$1, %r10d
	addq	$32, %r13

	cmpl	$0, %r10d
	jg		2b


0: // return

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dpack_nn_12_vs_lib4, .-kernel_dpack_nn_12_vs_lib4
#endif





//                             1         2          3        4          5
// void kernel_dpack_nn_8_lib4(int kmax, double *A, int lda, double *C, int sdc)

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dpack_nn_8_lib4
	.type kernel_dpack_nn_8_lib4, @function
kernel_dpack_nn_8_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dpack_nn_8_lib4
_kernel_dpack_nn_8_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dpack_nn_8_lib4
	.def kernel_dpack_nn_8_lib4; .scl 2; .type 32; .endef
kernel_dpack_nn_8_lib4:
#endif

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$3, %r12d
	movq	ARG4, %r13 // C
	movq	ARG5, %r14 // sdc
	sall	$5, %r14d


	cmpl	$0, %r10d
	jle		0f // return


	cmpl	$3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13)
	vmovupd		32(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13, %r14, 1)
	addq	%r12, %r11

	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 32(%r13)
	vmovupd		32(%r11), %ymm0
	vmovapd		%ymm0, 32(%r13, %r14, 1)
	addq	%r12, %r11

	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 64(%r13)
	vmovupd		32(%r11), %ymm0
	vmovapd		%ymm0, 64(%r13, %r14, 1)
	addq	%r12, %r11

	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 96(%r13)
	vmovupd		32(%r11), %ymm0
	vmovapd		%ymm0, 96(%r13, %r14, 1)
	addq	%r12, %r11

	subl	$4, %r10d
	addq	$128, %r13

	cmpl	$3, %r10d
	jg		1b


	cmpl	$0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13)
	vmovupd		32(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13, %r14, 1)
	addq	%r12, %r11

	subl	$1, %r10d
	addq	$32, %r13

	cmpl	$0, %r10d
	jg		2b


0: // return

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dpack_nn_8_lib4, .-kernel_dpack_nn_8_lib4
#endif





//                                1         2          3        4          5        6
// void kernel_dpack_nn_8_vs_lib4(int kmax, double *A, int lda, double *C, int sdc, int m1)

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dpack_nn_8_vs_lib4
	.type kernel_dpack_nn_8_vs_lib4, @function
kernel_dpack_nn_8_vs_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dpack_nn_8_vs_lib4
_kernel_dpack_nn_8_vs_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dpack_nn_8_vs_lib4
	.def kernel_dpack_nn_8_vs_lib4; .scl 2; .type 32; .endef
kernel_dpack_nn_8_vs_lib4:
#endif

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$3, %r12d
	movq	ARG4, %r13 // C
	movq	ARG5, %r14 // sdc
	sall	$5, %r14d
	movq	ARG6, %r15 // m1


	cmpl	$0, %r10d
	jle		0f // return


	vcvtsi2sd	%r15d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC01(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC01(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm15


	cmpl	$3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13)
	vmaskmovpd	32(%r11), %ymm15, %ymm0
	vmovapd		%ymm0, 0(%r13, %r14, 1)
	addq	%r12, %r11

	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 32(%r13)
	vmaskmovpd	32(%r11), %ymm15, %ymm0
	vmovapd		%ymm0, 32(%r13, %r14, 1)
	addq	%r12, %r11

	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 64(%r13)
	vmaskmovpd	32(%r11), %ymm15, %ymm0
	vmovapd		%ymm0, 64(%r13, %r14, 1)
	addq	%r12, %r11

	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 96(%r13)
	vmaskmovpd	32(%r11), %ymm15, %ymm0
	vmovapd		%ymm0, 96(%r13, %r14, 1)
	addq	%r12, %r11

	subl	$4, %r10d
	addq	$128, %r13

	cmpl	$3, %r10d
	jg		1b


	cmpl	$0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13)
	vmaskmovpd	32(%r11), %ymm15, %ymm0
	vmovapd		%ymm0, 0(%r13, %r14, 1)
	addq	%r12, %r11

	subl	$1, %r10d
	addq	$32, %r13

	cmpl	$0, %r10d
	jg		2b


0: // return

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dpack_nn_8_vs_lib4, .-kernel_dpack_nn_8_vs_lib4
#endif





//                             1         2          3        4
// void kernel_dpack_nn_4_lib4(int kmax, double *A, int lda, double *C)

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dpack_nn_4_lib4
	.type kernel_dpack_nn_4_lib4, @function
kernel_dpack_nn_4_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dpack_nn_4_lib4
_kernel_dpack_nn_4_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dpack_nn_4_lib4
	.def kernel_dpack_nn_4_lib4; .scl 2; .type 32; .endef
kernel_dpack_nn_4_lib4:
#endif

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$3, %r12d
	movq	ARG4, %r13 // C


	cmpl	$0, %r10d
	jle		0f // return


	cmpl	$3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13)
	addq	%r12, %r11

	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 32(%r13)
	addq	%r12, %r11

	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 64(%r13)
	addq	%r12, %r11

	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 96(%r13)
	addq	%r12, %r11

	subl	$4, %r10d
	addq	$128, %r13

	cmpl	$3, %r10d
	jg		1b


	cmpl	$0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovupd		0(%r11), %ymm0
	vmovapd		%ymm0, 0(%r13)
	addq	%r12, %r11

	subl	$1, %r10d
	addq	$32, %r13

	cmpl	$0, %r10d
	jg		2b


0: // return

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dpack_nn_4_lib4, .-kernel_dpack_nn_4_lib4
#endif





//                                1         2          3        4          5
// void kernel_dpack_nn_4_vs_lib4(int kmax, double *A, int lda, double *C, int m1)

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dpack_nn_4_vs_lib4
	.type kernel_dpack_nn_4_vs_lib4, @function
kernel_dpack_nn_4_vs_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dpack_nn_4_vs_lib4
_kernel_dpack_nn_4_vs_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dpack_nn_4_vs_lib4
	.def kernel_dpack_nn_4_vs_lib4; .scl 2; .type 32; .endef
kernel_dpack_nn_4_vs_lib4:
#endif

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$3, %r12d
	movq	ARG4, %r13 // C
	movq	ARG5, %r14 // m1


	cmpl	$0, %r10d
	jle		0f // return


	vcvtsi2sd	%r14d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC00(%rip), %ymm14
#elif defined(OS_MAC)
	vmovupd		LC00(%rip), %ymm14
#endif
	vmovddup	%xmm15, %xmm15
	vinsertf128	$1, %xmm15, %ymm15, %ymm15
	vsubpd		%ymm15, %ymm14, %ymm15


	cmpl	$3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmaskmovpd	0(%r11), %ymm15, %ymm0
	vmovapd		%ymm0, 0(%r13)
	addq	%r12, %r11

	vmaskmovpd	0(%r11), %ymm15, %ymm0
	vmovapd		%ymm0, 32(%r13)
	addq	%r12, %r11

	vmaskmovpd	0(%r11), %ymm15, %ymm0
	vmovapd		%ymm0, 64(%r13)
	addq	%r12, %r11

	vmaskmovpd	0(%r11), %ymm15, %ymm0
	vmovapd		%ymm0, 96(%r13)
	addq	%r12, %r11

	subl	$4, %r10d
	addq	$128, %r13

	cmpl	$3, %r10d
	jg		1b


	cmpl	$0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmaskmovpd	0(%r11), %ymm15, %ymm0
	vmovapd		%ymm0, 0(%r13)
	addq	%r12, %r11

	subl	$1, %r10d
	addq	$32, %r13

	cmpl	$0, %r10d
	jg		2b


0: // return

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dpack_nn_4_vs_lib4, .-kernel_dpack_nn_4_vs_lib4
#endif





//                             1         2          3        4
// void kernel_dpack_tn_4_lib4(int kmax, double *A, int lda, double *C)

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dpack_tn_4_lib4
	.type kernel_dpack_tn_4_lib4, @function
kernel_dpack_tn_4_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dpack_tn_4_lib4
_kernel_dpack_tn_4_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dpack_tn_4_lib4
	.def kernel_dpack_tn_4_lib4; .scl 2; .type 32; .endef
kernel_dpack_tn_4_lib4:
#endif

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$3, %r12d
	movq	ARG4, %r13 // C


	cmpl	$0, %r10d
	jle		0f // return


	movq	%r11, %r14
	addq	%r12, %r14
	addq	%r12, %r14


	cmpl	$3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovupd		0(%r11), %xmm0
	vmovupd		0(%r14), %xmm15
	vinsertf128	$0x1, %xmm15, %ymm0, %ymm0
	vmovupd		0(%r11, %r12, 1), %xmm1
	vmovupd		0(%r14, %r12, 1), %xmm15
	vinsertf128	$0x1, %xmm15, %ymm1, %ymm1
	vmovupd		16(%r11), %xmm2
	vmovupd		16(%r14), %xmm15
	vinsertf128	$0x1, %xmm15, %ymm2, %ymm2
	vmovupd		16(%r11, %r12, 1), %xmm3
	vmovupd		16(%r14, %r12, 1), %xmm15
	vinsertf128	$0x1, %xmm15, %ymm3, %ymm3

	vunpcklpd	%ymm1, %ymm0, %ymm4
	vmovapd		%ymm4, 0(%r13)
	vunpckhpd	%ymm1, %ymm0, %ymm5
	vmovapd		%ymm5, 32(%r13)
	vunpcklpd	%ymm3, %ymm2, %ymm6
	vmovapd		%ymm6, 64(%r13)
	vunpckhpd	%ymm3, %ymm2, %ymm7
	vmovapd		%ymm7, 96(%r13)

	subl	$4, %r10d
	addq	$32, %r11
	addq	$32, %r14
	addq	$128, %r13

	cmpl	$3, %r10d
	jg		1b


	cmpl	$0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13)
	vmovsd		0(%r11, %r12, 1), %xmm0
	vmovsd		%xmm0, 8(%r13)
	vmovsd		0(%r14), %xmm0
	vmovsd		%xmm0, 16(%r13)
	vmovsd		0(%r14, %r12, 1), %xmm0
	vmovsd		%xmm0, 24(%r13)

	subl	$1, %r10d
	addq	$8, %r11
	addq	$8, %r14
	addq	$32, %r13

	cmpl	$0, %r10d
	jg		2b


0: // return

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dpack_tn_4_lib4, .-kernel_dpack_tn_4_lib4
#endif





//                                1         2          3        4          5
// void kernel_dpack_tn_4_vs_lib4(int kmax, double *A, int lda, double *C, int m1)

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dpack_tn_4_vs_lib4
	.type kernel_dpack_tn_4_vs_lib4, @function
kernel_dpack_tn_4_vs_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dpack_tn_4_vs_lib4
_kernel_dpack_tn_4_vs_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dpack_tn_4_vs_lib4
	.def kernel_dpack_tn_4_vs_lib4; .scl 2; .type 32; .endef
kernel_dpack_tn_4_vs_lib4:
#endif

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$3, %r12d
	movq	ARG4, %r13 // C

	movq	ARG5, %r14 // m1
	cmpl	$1, %r14d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DPACK_TN_1_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_dpack_tn_1_lib4
#elif defined(OS_MAC)
	callq _inner_kernel_dpack_tn_1_lib4
#endif
#endif

	jmp		103f

100:

	movq	ARG5, %r14 // m1
	cmpl	$2, %r14d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DPACK_TN_2_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_dpack_tn_2_lib4
#elif defined(OS_MAC)
	callq _inner_kernel_dpack_tn_2_lib4
#endif
#endif
	
	jmp		103f

101:

	movq	ARG5, %r14 // m1
	cmpl	$3, %r14d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DPACK_TN_3_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_dpack_tn_3_lib4
#elif defined(OS_MAC)
	callq _inner_kernel_dpack_tn_3_lib4
#endif
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DPACK_TN_4_LIB4
#else
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	call inner_kernel_dpack_tn_4_lib4
#elif defined(OS_MAC)
	callq _inner_kernel_dpack_tn_4_lib4
#endif
#endif

103:


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dpack_tn_4_vs_lib4, .-kernel_dpack_tn_4_vs_lib4
#endif





#if 0
//                             1         2          3        4
// void kernel_dpack_tn_3_lib4(int kmax, double *A, int lda, double *C)

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dpack_tn_3_lib4
	.type kernel_dpack_tn_3_lib4, @function
kernel_dpack_tn_3_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dpack_tn_3_lib4
_kernel_dpack_tn_3_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dpack_tn_3_lib4
	.def kernel_dpack_tn_3_lib4; .scl 2; .type 32; .endef
kernel_dpack_tn_3_lib4:
#endif

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$3, %r12d
	movq	ARG4, %r13 // C


	cmpl	$0, %r10d
	jle		0f // return


	movq	%r11, %r14
	addq	%r12, %r14
	addq	%r12, %r14


	cmpl	$3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovupd		0(%r11), %xmm0
	vmovupd		0(%r14), %xmm15
	vinsertf128	$0x1, %xmm15, %ymm0, %ymm0
	vmovupd		0(%r11, %r12, 1), %xmm1
	vmovupd		16(%r11), %xmm2
	vmovupd		16(%r14), %xmm15
	vinsertf128	$0x1, %xmm15, %ymm2, %ymm2
	vmovupd		16(%r11, %r12, 1), %xmm3

	vunpcklpd	%ymm1, %ymm0, %ymm4
	vmovapd		%ymm4, 0(%r13)
	vunpckhpd	%ymm1, %ymm0, %ymm5
	vmovapd		%ymm5, 32(%r13)
	vunpcklpd	%ymm3, %ymm2, %ymm6
	vmovapd		%ymm6, 64(%r13)
	vunpckhpd	%ymm3, %ymm2, %ymm7
	vmovapd		%ymm7, 96(%r13)

	subl	$4, %r10d
	addq	$32, %r11
	addq	$32, %r14
	addq	$128, %r13

	cmpl	$3, %r10d
	jg		1b


	cmpl	$0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13)
	vmovsd		0(%r11, %r12, 1), %xmm0
	vmovsd		%xmm0, 8(%r13)
	vmovsd		0(%r14), %xmm0
	vmovsd		%xmm0, 16(%r13)

	subl	$1, %r10d
	addq	$8, %r11
	addq	$8, %r14
	addq	$32, %r13

	cmpl	$0, %r10d
	jg		2b


0: // return

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dpack_tn_3_lib4, .-kernel_dpack_tn_3_lib4
#endif





//                             1         2          3        4
// void kernel_dpack_tn_2_lib4(int kmax, double *A, int lda, double *C)

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dpack_tn_2_lib4
	.type kernel_dpack_tn_2_lib4, @function
kernel_dpack_tn_2_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dpack_tn_2_lib4
_kernel_dpack_tn_2_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dpack_tn_2_lib4
	.def kernel_dpack_tn_2_lib4; .scl 2; .type 32; .endef
kernel_dpack_tn_2_lib4:
#endif

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$3, %r12d
	movq	ARG4, %r13 // C


	cmpl	$0, %r10d
	jle		0f // return


	movq	%r11, %r14
	addq	%r12, %r14
	addq	%r12, %r14


	cmpl	$3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovupd		0(%r11), %xmm0
	vmovupd		0(%r11, %r12, 1), %xmm1
	vmovupd		16(%r11), %xmm2
	vmovupd		16(%r11, %r12, 1), %xmm3

	vunpcklpd	%ymm1, %ymm0, %ymm4
	vmovapd		%ymm4, 0(%r13)
	vunpckhpd	%ymm1, %ymm0, %ymm5
	vmovapd		%ymm5, 32(%r13)
	vunpcklpd	%ymm3, %ymm2, %ymm6
	vmovapd		%ymm6, 64(%r13)
	vunpckhpd	%ymm3, %ymm2, %ymm7
	vmovapd		%ymm7, 96(%r13)

	subl	$4, %r10d
	addq	$32, %r11
	addq	$32, %r14
	addq	$128, %r13

	cmpl	$3, %r10d
	jg		1b


	cmpl	$0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13)
	vmovsd		0(%r11, %r12, 1), %xmm0
	vmovsd		%xmm0, 8(%r13)

	subl	$1, %r10d
	addq	$8, %r11
	addq	$8, %r14
	addq	$32, %r13

	cmpl	$0, %r10d
	jg		2b


0: // return

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dpack_tn_2_lib4, .-kernel_dpack_tn_2_lib4
#endif





//                             1         2          3        4
// void kernel_dpack_tn_1_lib4(int kmax, double *A, int lda, double *C)

	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_dpack_tn_1_lib4
	.type kernel_dpack_tn_1_lib4, @function
kernel_dpack_tn_1_lib4:
#elif defined(OS_MAC)
	.globl _kernel_dpack_tn_1_lib4
_kernel_dpack_tn_1_lib4:
#elif defined(OS_WINDOWS)
	.globl kernel_dpack_tn_1_lib4
	.def kernel_dpack_tn_1_lib4; .scl 2; .type 32; .endef
kernel_dpack_tn_1_lib4:
#endif

	PROLOGUE


	movq	ARG1, %r10 // kmax
	movq	ARG2, %r11 // A
	movq	ARG3, %r12 // lda
	sall	$3, %r12d
	movq	ARG4, %r13 // C


	cmpl	$0, %r10d
	jle		0f // return


	cmpl	$3, %r10d
	jle		2f // clean-up loop


	// main loop
	.p2align 3
1: // main loop
	
	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13)
	vmovsd		8(%r11), %xmm0
	vmovsd		%xmm0, 32(%r13)
	vmovsd		16(%r11), %xmm0
	vmovsd		%xmm0, 64(%r13)
	vmovsd		24(%r11), %xmm0
	vmovsd		%xmm0, 96(%r13)

	subl	$4, %r10d
	addq	$32, %r11
	addq	$128, %r13

	cmpl	$3, %r10d
	jg		1b


	cmpl	$0, %r10d
	jle		0f // return


2: // clean-up loop
	
	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r13)

	subl	$1, %r10d
	addq	$8, %r11
	addq	$8, %r14
	addq	$32, %r13

	cmpl	$0, %r10d
	jg		2b


0: // return

	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dpack_tn_1_lib4, .-kernel_dpack_tn_1_lib4
#endif
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
	.align 5
LC00:
#endif
	.double 0.5
	.double 1.5
	.double 2.5
	.double 3.5


#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC01:
#elif defined(OS_MAC)
	.align 5
LC01:
#endif
	.double 4.5
	.double 5.5
	.double 6.5
	.double 7.5


#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC02:
#elif defined(OS_MAC)
	.align 5
LC02:
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

