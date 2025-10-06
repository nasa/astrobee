/**************************************************************************************************
*                                                                                                 *
* This file is part of BLASFEO.                                                                   *
*                                                                                                 *
* BLASFEO -- BLAS For Embedded Optimization.                                                      *
* Copyright (C) 2020 by Gianluca Frison.                                                          *
* All rights reserved.                                                                            *
*                                                                                                 *
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





//                         1      2
// void kernel_dvecld_inc1(int k, double *x)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dvecld_inc1)
	
	PROLOGUE
//	vzeroupper


	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // x


	// early return
	cmpl	$ 0, %r10d
	jle		2f // return


	// consider clean-up loop
	cmpl	$ 15, %r10d
	jle		0f


	// main loop
1:

	vmovupd		0(%r11), %ymm0
	vmovupd		32(%r11), %ymm0
	vmovupd		64(%r11), %ymm0
	vmovupd		96(%r11), %ymm0

	subl	$ 16, %r10d
	addq	$ 128, %r11

	cmpl	$ 15, %r10d
	jg		1b

0:

	// consider clean-up loop
	cmpl	$ 3, %r10d
	jle		0f


	// main loop
1:

	vmovupd		0(%r11), %ymm0

	subl	$ 4, %r10d
	addq	$ 32, %r11

	cmpl	$ 3, %r10d
	jg		1b

0:

	cmpl	$ 0, %r10d
	jle		2f // return


	// clean up loop
1:

	vmovsd		0(%r11), %xmm0

	subl	$ 1, %r10d
	addq	$ 8, %r11

	cmpl	$ 0, %r10d
	jg		1b


2:
	EPILOGUE
//	vzeroupper

	ret

	FUN_END(kernel_dvecld_inc1)





//                         1      2          3
// void kernel_dveccp_inc1(int k, double *x, double *y)

	.p2align 4,,15
	GLOB_FUN_START(kernel_dveccp_inc1)
	
	PROLOGUE
//	vzeroupper


	movq	ARG1, %r10 // k
	movq	ARG2, %r11 // x
	movq	ARG3, %r12 // y


	// early return
	cmpl	$ 0, %r10d
	jle		2f // return


	// consider clean-up loop
	cmpl	$ 15, %r10d
	jle		0f


	// main loop
1:

	vmovupd		0(%r11), %ymm0
	vmovupd		%ymm0, 0(%r12)
	vmovupd		32(%r11), %ymm0
	vmovupd		%ymm0, 32(%r12)
	vmovupd		64(%r11), %ymm0
	vmovupd		%ymm0, 64(%r12)
	vmovupd		96(%r11), %ymm0
	vmovupd		%ymm0, 96(%r12)

	subl	$ 16, %r10d
	addq	$ 128, %r11
	addq	$ 128, %r12

	cmpl	$ 15, %r10d
	jg		1b

0:

	// consider clean-up loop
	cmpl	$ 3, %r10d
	jle		0f


	// main loop
1:

	vmovupd		0(%r11), %ymm0
	vmovupd		%ymm0, 0(%r12)

	subl	$ 4, %r10d
	addq	$ 32, %r11
	addq	$ 32, %r12

	cmpl	$ 3, %r10d
	jg		1b

0:

	cmpl	$ 0, %r10d
	jle		2f // return


	// clean up loop
1:

	vmovsd		0(%r11), %xmm0
	vmovsd		%xmm0, 0(%r12)

	subl	$ 1, %r10d
	addq	$ 8, %r11
	addq	$ 8, %r12

	cmpl	$ 0, %r10d
	jg		1b


2:
	EPILOGUE
//	vzeroupper

	ret

	FUN_END(kernel_dveccp_inc1)





