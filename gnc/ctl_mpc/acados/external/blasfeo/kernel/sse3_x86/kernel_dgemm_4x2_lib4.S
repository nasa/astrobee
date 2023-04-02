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

#if defined(OS_LINUX)

#define STACKSIZE 16
#define ARG1  STACKSIZE +  4(%esp)
#define ARG2  STACKSIZE +  8(%esp)
#define ARG3  STACKSIZE + 12(%esp)
#define ARG4  STACKSIZE + 16(%esp)
#define ARG5  STACKSIZE + 20(%esp)
#define ARG6  STACKSIZE + 24(%esp)
#define ARG7  STACKSIZE + 28(%esp)
#define ARG8  STACKSIZE + 32(%esp)
#define ARG9  STACKSIZE + 36(%esp)
#define ARG10 STACKSIZE + 40(%esp)
#define ARG11 STACKSIZE + 44(%esp)
#define ARG12 STACKSIZE + 48(%esp)

#if 1

#define PROLOGUE \
	subl	$ 16, %esp; \
	movl	%ebx, 0(%esp); \
	movl	%esi, 4(%esp); \
	movl	%edi, 8(%esp); \
	movl	%ebp, 12(%esp);
#define EPILOGUE \
	movl	0(%esp), %ebx; \
	movl	4(%esp), %esi; \
	movl	8(%esp), %edi; \
	movl	12(%esp), %ebp; \
	addl	$ 16, %esp;

#else

#define PROLOGUE \
	pushl	%ebp; \
	pushl	%edi; \
	pushl	%esi; \
	pushl	%ebx;
#define EPILOGUE \
	popl	%ebx; \
	popl	%esi; \
	popl	%edi; \
	popl	%ebp;

#endif

#else

#error wrong OS

#endif



	.text



// common inner routine with file scope
//
// input arguments:
// eax   <- k
// ebx   <- A
// ecx   <- B
// xmm0 <- [d00 d10]
// xmm1 <- [d01 d11]
// xmm2 <- [d20 d30]
// xmm3 <- [d21 d31]

//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NT_4X2_LIB4
#else
	.align 16
	.type inner_kernel_gemm_nt_4x2_lib4, @function
inner_kernel_gemm_nt_4x2_lib4:
#endif

	cmpl	$ 0, %eax
	jle		2f // return

	// preload

	cmpl	$ 4, %eax
	jle		0f // consider clean-up loop

	// main loop
	.align 8
1: // main loop

	// unroll 0
	movddup		0(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		0(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		8(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1
	movapd		16(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	// unroll 1
	movddup		32(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		32(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		40(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1
	movapd		48(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	// unroll 2
	movddup		64(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		64(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		72(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1
	movapd		80(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	// unroll 3
	movddup		96(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		96(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		104(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1
	movapd		112(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2


	subl	$ 4, %eax
	addl	$ 128, %ecx
	addl	$ 128, %ebx

	cmpl	$ 4, %eax
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %eax
	jle		4f // clean1

	// unroll 0
	movddup		0(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		0(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		8(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1
	movapd		16(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	// unroll 1
	movddup		32(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		32(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		40(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1
	movapd		48(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	// unroll 2
	movddup		64(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		64(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		72(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1
	movapd		80(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	// unroll 3
	movddup		96(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		96(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		104(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1
	movapd		112(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	subl	$ 4, %eax
	addl	$ 128, %ecx
	addl	$ 128, %ebx

//	cmpl	$ 3, %eax
	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %eax
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	movddup		0(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		0(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		8(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1
	movapd		16(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	subl	$ 1, %eax
	addl	$ 32, %ecx
	addl	$ 32, %ebx

	cmpl	$ 0, %eax
	jg		3b // clean up loop

2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	.size	inner_kernel_gemm_nt_4x2_lib4, .-inner_kernel_gemm_nt_4x2_lib4
#endif





// common inner routine with file scope
//
// input arguments:
// eax  <- k
// ebx   <- A
// ecx   <- B
// edx   <- 4*sdb*sizeof(double)
// xmm0 <- [d00 d10]
// xmm1 <- [d01 d11]
// xmm2 <- [d20 d30]
// xmm3 <- [d21 d31]

//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NN_4X2_LIB4
#else
	.p2align 4,,15
	.type inner_kernel_gemm_add_nn_4x2_lib4, @function
inner_kernel_gemm_add_nn_4x2_lib4:
#endif

	cmpl	$ 0, %eax
	jle		2f // return

	// preload

	cmpl	$ 4, %eax
	jle		0f // consider clean-up loop

	// main loop
	.align 8
1: // main loop

	prefetcht0	0(%ecx, %edx, 1) // software prefetch

	// unroll 0
	movddup		0(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		0(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		32(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1
	movapd		16(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	// unroll 1
	movddup		8(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		32(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		40(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1
	movapd		48(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	// unroll 2
	movddup		16(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		64(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		48(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1
	movapd		80(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	// unroll 3
	movddup		24(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		96(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		56(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1
	movapd		112(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	subl	$ 4, %eax
	addl	$ 128, %ebx
	addl	%edx, %ecx

	cmpl	$ 4, %eax
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %eax
	jle		4f // clean1

	// unroll 0
	movddup		0(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		0(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		32(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1
	movapd		16(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	// unroll 1
	movddup		8(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		32(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		40(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1
	movapd		48(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	// unroll 2
	movddup		16(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		64(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		48(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1
	movapd		80(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	// unroll 3
	movddup		24(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		96(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		56(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1
	movapd		112(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	subl	$ 4, %eax
	addl	$ 128, %ebx
	addl	%edx, %ecx

//	cmpl	$ 3, %eax
	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %eax
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	movddup		0(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		0(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		32(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1
	movapd		16(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	subl	$ 1, %eax
	addl	$ 32, %ebx
	addl	$ 8, %ecx

	cmpl	$ 0, %eax
	jg		3b // clean up loop

2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	.size	inner_kernel_gemm_add_nn_4x2_lib4, .-inner_kernel_gemm_add_nn_4x2_lib4
#endif





// common inner routine with file scope
//
// input arguments:
// eax   <- k
// ebx   <- A
// ecx   <- B
// xmm0 <- [d00 d10]
// xmm1 <- [d01 d11]
// xmm2 <- [d20 d30]
// xmm3 <- [d21 d31]

//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_SUB_NT_4X2_LIB4
#else
	.align 16
	.type inner_kernel_gemm_sub_nt_4x2_lib4, @function
inner_kernel_gemm_sub_nt_4x2_lib4:
#endif

	cmpl	$ 0, %eax
	jle		2f // return

	// preload

	cmpl	$ 4, %eax
	jle		0f // consider clean-up loop

	// main loop
	.align 8
1: // main loop

	// unroll 0
	movddup		0(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		0(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	subpd		%xmm5, %xmm0
	movddup		8(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	subpd		%xmm4, %xmm1
	movapd		16(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	subpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	subpd		%xmm4, %xmm2

	// unroll 1
	movddup		32(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		32(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	subpd		%xmm5, %xmm0
	movddup		40(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	subpd		%xmm4, %xmm1
	movapd		48(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	subpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	subpd		%xmm4, %xmm2

	// unroll 2
	movddup		64(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		64(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	subpd		%xmm5, %xmm0
	movddup		72(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	subpd		%xmm4, %xmm1
	movapd		80(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	subpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	subpd		%xmm4, %xmm2

	// unroll 3
	movddup		96(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		96(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	subpd		%xmm5, %xmm0
	movddup		104(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	subpd		%xmm4, %xmm1
	movapd		112(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	subpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	subpd		%xmm4, %xmm2


	subl	$ 4, %eax
	addl	$ 128, %ecx
	addl	$ 128, %ebx

	cmpl	$ 4, %eax
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %eax
	jle		4f // clean1

	// unroll 0
	movddup		0(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		0(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	subpd		%xmm5, %xmm0
	movddup		8(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	subpd		%xmm4, %xmm1
	movapd		16(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	subpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	subpd		%xmm4, %xmm2

	// unroll 1
	movddup		32(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		32(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	subpd		%xmm5, %xmm0
	movddup		40(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	subpd		%xmm4, %xmm1
	movapd		48(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	subpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	subpd		%xmm4, %xmm2

	// unroll 2
	movddup		64(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		64(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	subpd		%xmm5, %xmm0
	movddup		72(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	subpd		%xmm4, %xmm1
	movapd		80(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	subpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	subpd		%xmm4, %xmm2

	// unroll 3
	movddup		96(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		96(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	subpd		%xmm5, %xmm0
	movddup		104(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	subpd		%xmm4, %xmm1
	movapd		112(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	subpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	subpd		%xmm4, %xmm2

	subl	$ 4, %eax
	addl	$ 128, %ecx
	addl	$ 128, %ebx

//	cmpl	$ 3, %eax
	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %eax
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	movddup		0(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		0(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	subpd		%xmm5, %xmm0
	movddup		8(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	subpd		%xmm4, %xmm1
	movapd		16(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	subpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	subpd		%xmm4, %xmm2

	subl	$ 1, %eax
	addl	$ 32, %ecx
	addl	$ 32, %ebx

	cmpl	$ 0, %eax
	jg		3b // clean up loop

2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	.size	inner_kernel_gemm_sub_nt_4x2_lib4, .-inner_kernel_gemm_sub_nt_4x2_lib4
#endif





// common inner routine with file scope
//
// edge for B unaligned, A aligned
//
// input arguments:
// eax   <- k
// ebx   <- A
// ecx   <- B
// edx   <- bs*sdb*sizeof(double)
// esi   <- offB
// xmm0 <- [d00 d10]
// xmm1 <- [d01 d11]
// xmm2 <- [d20 d30]
// xmm3 <- [d21 d31]

//
// output arguments:


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_GEMM_ADD_NN_4X2_LIB4
#else
	.p2align 4,,15
	.type inner_edge_gemm_add_nn_4x2_lib4, @function
inner_edge_gemm_add_nn_4x2_lib4:
#endif

	cmpl			$ 0, %esi				// offset==0
	jle				2f						// end

	cmpl			$ 0, %eax				// k==0
	jle				2f						// end

	movl			%esi, %edi				// load offsetB
	sall			$ 3, %edi				// offsetB*sizeof(double)
	addl			%edi, %ecx				// B+offsetB*sizeof(double)

	movl			$ 4, %edi				// load 4
	subl			%esi, %edi				// 4-offsetB
	cmpl			%eax, %edi				// k > 4-offsetB
	cmovgl			%eax, %edi				// kend=min(k,4-offsetB)

1:
	// unroll 0
	movddup		0(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		0(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		32(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1
	movapd		16(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	subl			$ 1, %eax				// k=-1
	subl			$ 1, %edi				// k_panel=-1
	addl			$ 32, %ebx				// A=+bs
	addl			$ 8, %ecx				// B=+1

	cmpl			$ 0, %edi				// if k_panel=0
	jg				1b						// loop 1

	cmpl			$ 0, %eax				// if k=0
	jle				2f						// end

	addl			%edx, %ecx				// B=Boff+sdb*bs
	subl			$ 32, %ecx				// B-=4*sizeof(double) (loop+offsetB)

2:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	.size	inner_edge_gemm_add_nn_4x2_lib4, .-inner_edge_gemm_add_nn_4x2_lib4
#endif





// common inner routine with file scope
//
// edge for B lower triangular
//
// input arguments:
// eax   <- k
// ebx   <- A
// ecx   <- B
// edx   <- bs*sdb*sizeof(double)
// esi   <- offB
// xmm0 <- [d00 d10]
// xmm1 <- [d01 d11]
// xmm2 <- [d20 d30]
// xmm3 <- [d21 d31]

//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRMM_NN_RL_4X2_LIB4
#else
	.align 16
	.type inner_edge_trmm_nn_rl_4x2_lib4, @function
inner_edge_trmm_nn_rl_4x2_lib4:
#endif
	
	cmpl	$ 0, %esi
	jg		0f

	// offB==0

	// unroll 0
	movddup		0(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		0(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movapd		16(%ebx), %xmm4 // A2
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	// unroll 1
	movddup		8(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		32(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		40(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1
	movapd		48(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	// unroll 2
	movddup		16(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		64(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		48(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1
	movapd		80(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	// unroll 3
	movddup		24(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		96(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		56(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1
	movapd		112(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	subl	$ 4, %eax // k-4
	addl	$ 128, %ebx // A+4*bs*sizeof(double)
	addl	%edx, %ecx // B+bs*sdb*sizeof(double)

	jmp		3f

0:
	cmpl	$ 1, %esi
	jg		1f

	// offB==1

	addl			$ 8, %ecx // B+1*sizeof(double)

	// unroll 0
	movddup		0(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		0(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movapd		16(%ebx), %xmm4 // A2
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	// unroll 1
	movddup		8(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		32(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		40(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1
	movapd		48(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	// unroll 2
	movddup		16(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		64(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		48(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1
	movapd		80(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	subl	$ 3, %eax // k-3
	addl	$ 96, %ebx // A+3*bs*sizeof(double)
	addl	%edx, %ecx
	subl			$ 8, %ecx // B+bs*sdb*sizeof(double)-1

	jmp		3f

1:
	cmpl	$ 2, %esi
	jg		2f

	// offB==2

	addl	$ 16, %ecx // B+2*sizeof(double)

	// unroll 0
	movddup		0(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		0(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movapd		16(%ebx), %xmm4 // A2
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	// unroll 1
	movddup		8(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		32(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		40(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1
	movapd		48(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	subl	$ 2, %eax // k-2
	addl	$ 64, %ebx // A+2*bs*sizeof(double)
	addl	%edx, %ecx
	subl	$ 16, %ecx // B+bs*sdb*sizeof(double)-2

	jmp		3f

2:
	// offB==3

	addl	$ 24, %ecx // B+3*sizeof(double)

	// unroll 0
	movddup		0(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		0(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movapd		16(%ebx), %xmm4 // A2
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	subl	$ 1, %eax // k-1
	addl	$ 32, %ebx // A+1*bs*sizeof(double)
	addl	%edx, %ecx
	subl	$ 24, %ecx // B+bs*sdb*sizeof(double)-3

3:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	.size	inner_edge_trmm_nn_rl_4x2_lib4, .-inner_edge_trmm_nn_rl_4x2_lib4
#endif





// common inner routine with file scope
//
// edge for B lower triangular
//
// input arguments:
// eax   <- k
// ebx   <- A
// ecx   <- B
// edx   <- bs*sdb*sizeof(double)
// esi   <- offB
// xmm0 <- [d00 d10]
// xmm1 <- [d01 d11]
// xmm2 <- [d20 d30]
// xmm3 <- [d21 d31]

//
// output arguments:


#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRMM_NN_RL_4X2_VS_LIB4
#else
	.p2align 16
	.type inner_edge_trmm_nn_rl_4x2_vs_lib4, @function
inner_edge_trmm_nn_rl_4x2_vs_lib4:
#endif
	
	cmpl			$ 0, %eax
	jle				3f // end

	cmpl			$ 0, %esi
	jg				0f // offB>0

	// offB==0

	movddup		0(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		0(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movapd		16(%ebx), %xmm4 // A2
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	subl			$ 1, %eax // k-1
	addl			$ 32, %ebx // A+1*bs*sizeof(double)
	addl			$ 8, %ecx // B+1*sizeof(double)

	cmpl			$ 0, %eax
	jle				3f // end

	movddup		0(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		0(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		32(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1
	movapd		16(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	subl			$ 1, %eax // k-1
	addl			$ 32, %ebx // A+1*bs*sizeof(double)
	addl			$ 8, %ecx // B+1*sizeof(double)

	cmpl			$ 0, %eax
	jle				3f // end

	movddup		0(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		0(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		32(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1
	movapd		16(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	subl			$ 1, %eax // k-1
	addl			$ 32, %ebx // A+1*bs*sizeof(double)
	addl			$ 8, %ecx // B+1*sizeof(double)

	cmpl			$ 0, %eax
	jle				3f // end

	movddup		0(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		0(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		32(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1
	movapd		16(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	subl			$ 1, %eax // k-1
	addl			$ 32, %ebx // A+1*bs*sizeof(double)
	addl			%edx, %ecx
	subl			$ 24, %ecx // B+bs*sdb*sizeof(double)-(bs-1)*sizeof(double)

	jmp				3f // end

0:
	cmpl			$ 1, %esi
	jg				1f // offB>1

	// offB==1

	addl			$ 8, %ecx // B+1*sizeof(double)

	movddup		0(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		0(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movapd		16(%ebx), %xmm4 // A2
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	subl			$ 1, %eax // k-1
	addl			$ 32, %ebx // A+1*bs*sizeof(double)
	addl			$ 8, %ecx // B+1*sizeof(double)

	cmpl			$ 0, %eax
	jle				3f // end

	movddup		0(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		0(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		32(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1
	movapd		16(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	subl			$ 1, %eax // k-1
	addl			$ 32, %ebx // A+1*bs*sizeof(double)
	addl			$ 8, %ecx // B+1*sizeof(double)

	cmpl			$ 0, %eax
	jle				3f // end

	movddup		0(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		0(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		32(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1
	movapd		16(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	subl			$ 1, %eax // k-1
	addl			$ 32, %ebx // A+1*bs*sizeof(double)
	addl			%edx, %ecx
	subl			$ 24, %ecx // B+bs*sdb*sizeof(double)-(bs-1)*sizeof(double)

	jmp				3f // end

1:
	cmpl			$ 2, %esi
	jg				2f // offB>2

	// offB==2

	addl			$ 16, %ecx // B+2*sizeof(double)

	movddup		0(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		0(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movapd		16(%ebx), %xmm4 // A2
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	subl			$ 1, %eax // k-1
	addl			$ 32, %ebx // A+1*bs*sizeof(double)
	addl			$ 8, %ecx // B+1*sizeof(double)

	cmpl			$ 0, %eax
	jle				3f // end

	movddup		0(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		0(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movddup		32(%ecx), %xmm5 // B1
	mulpd		%xmm5, %xmm4
	addpd		%xmm4, %xmm1
	movapd		16(%ebx), %xmm4 // A2
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm3
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	subl			$ 1, %eax // k-2
	addl			$ 32, %ebx // A+2*bs*sizeof(double)
	addl			%edx, %ecx
	subl			$ 24, %ecx // B+bs*sdb*sizeof(double)-(bs-1)*sizeof(double)

	jmp				3f

2:
	// offB==3

	addl			$ 24, %ecx // B+3*sizeof(double)

	movddup		0(%ecx), %xmm5 // B0
	movapd		%xmm5, %xmm6 // B0
	movapd		0(%ebx), %xmm4 // A0
	mulpd		%xmm4, %xmm5
	addpd		%xmm5, %xmm0
	movapd		16(%ebx), %xmm4 // A2
	mulpd		%xmm6, %xmm4
	addpd		%xmm4, %xmm2

	subl			$ 1, %eax // k-1
	addl			$ 32, %ebx // A+1*bs*sizeof(double)
	addl			%edx, %ecx
	subl			$ 24, %ecx // B+bs*sdb*sizeof(double)-(bs-1)*sizeof(double)

3:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	.size	inner_edge_trmm_nn_rl_4x2_vs_lib4, .-inner_edge_trmm_nn_rl_4x2_vs_lib4
#endif





// common inner routine with file scope
//
// cholesky factorization 
//
// input arguments:
// eax  <- inv_diag_E
// ebx <- kn
// xmm0 <- [d00 d10]
// xmm1 <- [d01 d11]
// xmm2 <- [d20 d30]
// xmm3 <- [d21 d31]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_POTRF_4X2_VS_LIB4
#else
	.align 16
	.type inner_edge_potrf_4x2_vs_lib4, @function
inner_edge_potrf_4x2_vs_lib4:
#endif
	
	xorpd			%xmm7, %xmm7 // 0.0

	movsd			%xmm0, %xmm6
	ucomisd			%xmm7, %xmm6 // d_00 > 0.0 ?
	jbe				1f
	sqrtsd			%xmm6, %xmm6
	movsd			.LC00, %xmm5 // 1.0
	divsd			%xmm6, %xmm5
2:
	cmpl			$ 2, %ebx
	movsd			%xmm5, 0(%eax)
	movddup			%xmm5, %xmm5
	mulpd			%xmm5, %xmm0
	mulpd			%xmm5, %xmm2

	jl				0f // ret

	movapd			%xmm0, %xmm6
	shufpd			$ 0x3, %xmm6, %xmm6
	movapd			%xmm6, %xmm5
	mulpd			%xmm0, %xmm6
	mulpd			%xmm2, %xmm5
	subpd			%xmm6, %xmm1
	subpd			%xmm5, %xmm3
	movapd			%xmm1, %xmm6
	shufpd			$ 0x3, %xmm6, %xmm6 // 0x1 ???
	ucomisd			%xmm7, %xmm6 // d_11 > 0.0 ?
	jbe				3f
	sqrtsd			%xmm6, %xmm6
	movsd			.LC00, %xmm5 // 1.0
	divsd			%xmm6, %xmm5
4:
	movsd			%xmm5, 8(%eax)
	movddup			%xmm5, %xmm5
	mulpd			%xmm5, %xmm1
	mulpd			%xmm5, %xmm3

	jmp		0f
	
1:
	xorpd	%xmm5, %xmm5
	jmp		2b

3:
	xorpd	%xmm5, %xmm5
	jmp		4b

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	.size	inner_edge_potrf_4x2_vs_lib4, .-inner_edge_potrf_4x2_vs_lib4
#endif





// common inner routine with file scope
//
// triangular substitution for cholesky factorization 
//
// input arguments:
// eax  <- E
// ebx  <- inv_diag_E
// xmm0 <- [d00 d10]
// xmm1 <- [d01 d11]
// xmm2 <- [d20 d30]
// xmm3 <- [d21 d31]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRSM_RLT_INV_4X2_LIB4
#else
	.align 16
	.type inner_edge_trsm_rlt_inv_4x2_lib4, @function
inner_edge_trsm_rlt_inv_4x2_lib4:
#endif
	
	movddup			0(%ebx), %xmm7
	mulpd			%xmm7, %xmm0
	mulpd			%xmm7, %xmm2

	movddup			8(%eax), %xmm7
	movapd			%xmm7, %xmm6
	mulpd			%xmm0, %xmm7
	mulpd			%xmm2, %xmm6
	subpd			%xmm7, %xmm1
	subpd			%xmm6, %xmm3
	movddup			8(%ebx), %xmm7
	mulpd			%xmm7, %xmm1
	mulpd			%xmm7, %xmm3

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	.size	inner_edge_trsm_rlt_inv_4x2_lib4, .-inner_edge_trsm_rlt_inv_4x2_lib4
#endif





// common inner routine with file scope
//
// triangular substitution for cholesky factorization 
//
// input arguments:
// eax  <- E
// ebx  <- inv_diag_E
// ecx  <- kn
// xmm0 <- [d00 d10]
// xmm1 <- [d01 d11]
// xmm2 <- [d20 d30]
// xmm3 <- [d21 d31]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRSM_RLT_INV_4X2_VS_LIB4
#else
	.align 16
	.type inner_edge_trsm_rlt_inv_4x2_vs_lib4, @function
inner_edge_trsm_rlt_inv_4x2_vs_lib4:
#endif
	
	movddup			0(%ebx), %xmm7
	mulpd			%xmm7, %xmm0
	mulpd			%xmm7, %xmm2

	cmpl			$ 2, %ecx
	jl				0f // ret

	movddup			8(%eax), %xmm7
	movapd			%xmm7, %xmm6
	mulpd			%xmm0, %xmm7
	mulpd			%xmm2, %xmm6
	subpd			%xmm7, %xmm1
	subpd			%xmm6, %xmm3
	movddup			8(%ebx), %xmm7
	mulpd			%xmm7, %xmm1
	mulpd			%xmm7, %xmm3

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	.size	inner_edge_trsm_rlt_inv_4x2_vs_lib4, .-inner_edge_trsm_rlt_inv_4x2_vs_lib4
#endif





// common inner routine with file scope
//
// blend for generic alpha and beta
//
// input arguments:
// eax   <- alpha
// ebx   <- beta
// ecx   <- C
// xmm0 <- [d00 d10]
// xmm1 <- [d01 d11]
// xmm2 <- [d20 d30]
// xmm3 <- [d21 d31]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_4X2_LIB4
#else
	.align 16
	.type inner_scale_ab_4x2_lib4, @function
inner_scale_ab_4x2_lib4:
#endif

	// alpha
	movddup		0(%eax), %xmm7

	mulpd		%xmm7, %xmm0
	mulpd		%xmm7, %xmm1
	mulpd		%xmm7, %xmm2
	mulpd		%xmm7, %xmm3

	// beta
	movddup		0(%ebx), %xmm6

	xorpd		%xmm7, %xmm7 // 0.0

	ucomisd		%xmm7, %xmm6 // beta==0.0 ?
	je			0f // end

	movapd		0(%ecx), %xmm7
	mulpd		%xmm6, %xmm7
	addpd		%xmm7, %xmm0
	movapd		16(%ecx), %xmm7
	mulpd		%xmm6, %xmm7
	addpd		%xmm7, %xmm2
	movapd		32(%ecx), %xmm7
	mulpd		%xmm6, %xmm7
	addpd		%xmm7, %xmm1
	movapd		48(%ecx), %xmm7
	mulpd		%xmm6, %xmm7
	addpd		%xmm7, %xmm3

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	.size	inner_scale_ab_4x2_lib4, .-inner_scale_ab_4x2_lib4
#endif





// common inner routine with file scope
//
// blend for generic alpha and beta
//
// input arguments:
// eax   <- alpha
// xmm0 <- [d00 d10]
// xmm1 <- [d01 d11]
// xmm2 <- [d20 d30]
// xmm3 <- [d21 d31]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_A0_4X2_LIB4
#else
	.align 16
	.type inner_scale_a0_4x2_lib4, @function
inner_scale_a0_4x2_lib4:
#endif

	// alpha
	movddup		0(%eax), %xmm7

	mulpd		%xmm7, %xmm0
	mulpd		%xmm7, %xmm1
	mulpd		%xmm7, %xmm2
	mulpd		%xmm7, %xmm3

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	.size	inner_scale_a0_4x2_lib4, .-inner_scale_a0_4x2_lib4
#endif





// common inner routine with file scope
//
// blend for generic alpha=1 and beta
//
// input arguments:
// eax   <- beta
// ebx   <- C
// xmm0 <- [d00 d10]
// xmm1 <- [d01 d11]
// xmm2 <- [d20 d30]
// xmm3 <- [d21 d31]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M1B_4X2_LIB4
#else
	.align 16
	.type inner_scale_m1b_4x2_lib4, @function
inner_scale_m1b_4x2_lib4:
#endif

	// beta
	movddup	0(%eax), %xmm6

	xorpd		%xmm7, %xmm7 // 0.0
	ucomisd		%xmm7, %xmm6 // beta==0.0 ?
	je			0f // end

	movapd		0(%ebx), %xmm7
	mulpd		%xmm6, %xmm7
	subpd		%xmm0, %xmm7
	movapd		%xmm7, %xmm0
	movapd		16(%ebx), %xmm7
	mulpd		%xmm6, %xmm7
	subpd		%xmm1, %xmm7
	movapd		%xmm7, %xmm1
	movapd		32(%ebx), %xmm7
	mulpd		%xmm6, %xmm7
	subpd		%xmm2, %xmm7
	movapd		%xmm7, %xmm2
	movapd		48(%ebx), %xmm7
	mulpd		%xmm6, %xmm7
	subpd		%xmm3, %xmm7
	movapd		%xmm7, %xmm3

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	.size	inner_scale_m1b_4x2_lib4, .-inner_scale_m1b_4x2_lib4
#endif





// common inner routine with file scope
//
// blend for generic alpha=1 and beta=1
//
// input arguments:
// eax   <- C
// xmm0 <- [d00 d10]
// xmm1 <- [d01 d11]
// xmm2 <- [d20 d30]
// xmm3 <- [d21 d31]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M11_4X2_LIB4
#else
	.align 16
	.type inner_scale_m11_4x2_lib4, @function
inner_scale_m11_4x2_lib4:
#endif

	movapd		0(%eax), %xmm7
	subpd		%xmm0, %xmm7
	movapd		%xmm7, %xmm0
	movapd		16(%eax), %xmm7
	subpd		%xmm1, %xmm7
	movapd		%xmm7, %xmm1
	movapd		32(%eax), %xmm7
	subpd		%xmm2, %xmm7
	movapd		%xmm7, %xmm2
	movapd		48(%eax), %xmm7
	subpd		%xmm3, %xmm7
	movapd		%xmm7, %xmm3

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	.size	inner_scale_m11_4x2_lib4, .-inner_scale_m11_4x2_lib4
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// eax  <- D
// xmm0 <- [d00 d10]
// xmm1 <- [d01 d11]
// xmm2 <- [d20 d30]
// xmm3 <- [d21 d31]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X2_LIB4
#else
	.align 16
	.type inner_store_4x2_lib4, @function
inner_store_4x2_lib4:
#endif

	movapd	%xmm0,  0(%eax)
	movapd	%xmm2, 16(%eax)
	movapd	%xmm1, 32(%eax)
	movapd	%xmm3, 48(%eax)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	.size	inner_store_4x2_lib4, .-inner_store_4x2_lib4
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// eax  <- D
// ebx  <- m1
// ecx  <- n1
// xmm0 <- [d00 d10]
// xmm1 <- [d01 d11]
// xmm2 <- [d20 d30]
// xmm3 <- [d21 d31]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X2_VS_LIB4
#else
	.align 16
	.type inner_store_4x2_vs_lib4, @function
inner_store_4x2_vs_lib4:
#endif

	cmpl		$ 2, %ebx
	jg			1f
	je			0f

	// km==1
	movsd		%xmm0,  0(%eax)
	cmpl		$ 2, %ecx
	jl			4f // end
	movsd		%xmm1, 32(%eax)

	jmp		4f

0:
	// km==2
	movapd		%xmm0,  0(%eax)
	cmpl		$ 2, %ecx
	jl			4f // end
	movapd		%xmm1, 32(%eax)

	jmp		4f

1:
	cmpl		$ 3, %ebx
	jg			2f

	// km==3
	movapd		%xmm0,   0(%eax)
	movsd		%xmm2,  16(%eax)
	cmpl		$ 2, %ecx
	jl			4f // end
	movapd		%xmm1,  32(%eax)
	movsd		%xmm3,  48(%eax)

	jmp		4f

2:
	// km==4
	movapd		%xmm0,   0(%eax)
	movapd		%xmm2,  16(%eax)
	cmpl		$ 2, %ecx
	jl			4f // end
	movapd		%xmm1,  32(%eax)
	movapd		%xmm3,  48(%eax)

4:



#if MACRO_LEVEL>=1
	.endm
#else
	ret

	.size	inner_store_4x2_vs_lib4, .-inner_store_4x2_vs_lib4
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// eax  <- D
// xmm0 <- [d00 d10]
// xmm1 <- [d01 d11]
// xmm2 <- [d20 d30]
// xmm3 <- [d21 d31]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_4X2_LIB4
#else
	.align 16
	.type inner_store_l_4x2_lib4, @function
inner_store_l_4x2_lib4:
#endif

	movapd	%xmm0,  0(%eax)
	movapd	%xmm2, 16(%eax)
	movhpd	%xmm1, 40(%eax)
	movapd	%xmm3, 48(%eax)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	.size	inner_store_l_4x2_lib4, .-inner_store_l_4x2_lib4
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// eax  <- D
// ebx  <- m1
// ecx  <- n1
// xmm0 <- [d00 d10]
// xmm1 <- [d01 d11]
// xmm2 <- [d20 d30]
// xmm3 <- [d21 d31]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_4X2_VS_LIB4
#else
	.align 16
	.type inner_store_l_4x2_vs_lib4, @function
inner_store_l_4x2_vs_lib4:
#endif

	cmpl		$ 2, %ebx
	jg			1f
	je			0f

	// km==1
	movsd		%xmm0,  0(%eax)

	jmp		4f

0:
	// km==2
	movapd		%xmm0,  0(%eax)
	cmpl		$ 2, %ecx
	jl			4f // end
	movhpd		%xmm1, 40(%eax)

	jmp		4f

1:
	cmpl		$ 3, %ebx
	jg			2f

	// km==3
	movapd		%xmm0,   0(%eax)
	movsd		%xmm2,  16(%eax)
	cmpl		$ 2, %ecx
	jl			4f // end
	movhpd		%xmm1,  40(%eax)
	movsd		%xmm3,  48(%eax)

	jmp		4f

2:
	// km==4
	movapd		%xmm0,   0(%eax)
	movapd		%xmm2,  16(%eax)
	cmpl		$ 2, %ecx
	jl			4f // end
	movhpd		%xmm1,  40(%eax)
	movapd		%xmm3,  48(%eax)

4:



#if MACRO_LEVEL>=1
	.endm
#else
	ret

	.size	inner_store_l_4x2_vs_lib4, .-inner_store_l_4x2_vs_lib4
#endif





//                               1      2              3          4          5             6          7
// void kernel_dgemm_nt_4x2_lib4(int k, double *alpha, double *A, double *B, double *beta, double *C, double *D);

	.align 16
	.globl kernel_dgemm_nt_4x2_lib4
	.type kernel_dgemm_nt_4x2_lib4, @function
kernel_dgemm_nt_4x2_lib4:

	PROLOGUE

	// zero accumulation registers

	xorpd	%xmm0, %xmm0
	movapd	%xmm0, %xmm1
	movapd	%xmm0, %xmm2
	movapd	%xmm0, %xmm3


	// call inner gemm kernel nt

	movl	ARG1, %eax // k
	movl	ARG3, %ebx  // A
	movl	ARG4, %ecx  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X2_LIB4
#else
	call inner_kernel_gemm_nt_4x2_lib4
#endif


	// call inner blend scale

	movl	ARG2, %eax // alpha
	movl	ARG5, %ebx // beta
	movl	ARG6, %ecx   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X2_LIB4
#else
	call inner_scale_ab_4x2_lib4
#endif


	// store n

	movl	ARG7, %eax // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X2_LIB4
#else
	call inner_store_4x2_lib4
#endif

	EPILOGUE

	ret

	.size	kernel_dgemm_nt_4x2_lib4, .-kernel_dgemm_nt_4x2_lib4





//                                  1      2              3          4          5             6          7          8       9
// void kernel_dgemm_nt_4x2_vs_lib4(int k, double *alpha, double *A, double *B, double *beta, double *C, double *D, int m1, int n1);

	.align 16
	.globl kernel_dgemm_nt_4x2_vs_lib4
	.type kernel_dgemm_nt_4x2_vs_lib4, @function
kernel_dgemm_nt_4x2_vs_lib4:

	PROLOGUE

	// zero accumulation registers

	xorpd	%xmm0, %xmm0
	movapd	%xmm0, %xmm1
	movapd	%xmm0, %xmm2
	movapd	%xmm0, %xmm3


	// call inner gemm kernel nt

	movl	ARG1, %eax // k
	movl	ARG3, %ebx  // A
	movl	ARG4, %ecx  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X2_LIB4
#else
	call inner_kernel_gemm_nt_4x2_lib4
#endif


	// call inner blend scale

	movl	ARG2, %eax // alpha
	movl	ARG5, %ebx // beta
	movl	ARG6, %ecx   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X2_LIB4
#else
	call inner_scale_ab_4x2_lib4
#endif


	// store n

	movl	ARG7, %eax // D
	movl	ARG8, %ebx // m1
	movl	ARG9, %ecx // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X2_VS_LIB4
#else
	call inner_store_4x2_vs_lib4
#endif

	EPILOGUE

	ret

	.size	kernel_dgemm_nt_4x2_vs_lib4, .-kernel_dgemm_nt_4x2_vs_lib4





//                               1      2              3          4            5          6        7             8          9
// void kernel_dgemm_nn_4x2_lib4(int k, double *alpha, double *A, int offsetB, double *B, int sdb, double *beta, double *C, double *D);

	.align 16
	.globl kernel_dgemm_nn_4x2_lib4
	.type kernel_dgemm_nn_4x2_lib4, @function
kernel_dgemm_nn_4x2_lib4:

	PROLOGUE

	// zero accumulation registers

	xorpd	%xmm0, %xmm0
	movapd	%xmm0, %xmm1
	movapd	%xmm0, %xmm2
	movapd	%xmm0, %xmm3


	// call inner gemm kernel nn

	movl	ARG1, %eax // k
	movl	ARG3, %ebx  // A
	movl	ARG5, %ecx  // B
	movl	ARG6, %edx // sdb
	sall	$ 5, %edx // 4*sdb*sizeof(double)
	movl	ARG4, %esi // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_GEMM_ADD_NN_4X2_LIB4
#else
	call inner_edge_gemm_add_nn_4x2_lib4
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X2_LIB4
#else
	call inner_kernel_gemm_add_nn_4x2_lib4
#endif


	// call inner blend

	movl	ARG2, %eax // alpha
	movl	ARG7, %ebx // beta
	movl	ARG8, %ecx   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X2_LIB4
#else
	call inner_scale_ab_4x2_lib4
#endif


	// store n

	movl	ARG9, %eax // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X2_LIB4
#else
	call inner_store_4x2_lib4
#endif


	EPILOGUE

	ret

	.size	kernel_dgemm_nn_4x2_lib4, .-kernel_dgemm_nn_4x2_lib4





//                                  1      2              3          4            5          6        7             8          9          10      11
// void kernel_dgemm_nn_4x2_vs_lib4(int k, double *alpha, double *A, int offsetB, double *B, int sdb, double *beta, double *C, double *D, int m1, int n1);

	.align 16
	.globl kernel_dgemm_nn_4x2_vs_lib4
	.type kernel_dgemm_nn_4x2_vs_lib4, @function
kernel_dgemm_nn_4x2_vs_lib4:

	PROLOGUE

	// zero accumulation registers

	xorpd	%xmm0, %xmm0
	movapd	%xmm0, %xmm1
	movapd	%xmm0, %xmm2
	movapd	%xmm0, %xmm3


	// call inner gemm kernel nn

	movl	ARG1, %eax // k
	movl	ARG3, %ebx  // A
	movl	ARG5, %ecx  // B
	movl	ARG6, %edx // sdb
	sall	$ 5, %edx // 4*sdb*sizeof(double)
	movl	ARG4, %esi // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_GEMM_ADD_NN_4X2_LIB4
#else
	call inner_edge_gemm_add_nn_4x2_lib4
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X2_LIB4
#else
	call inner_kernel_gemm_add_nn_4x2_lib4
#endif


	// call inner blend

	movl	ARG2, %eax // alpha
	movl	ARG7, %ebx // beta
	movl	ARG8, %ecx   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X2_LIB4
#else
	call inner_scale_ab_4x2_lib4
#endif


	// store n

	movl	ARG9, %eax // D
	movl	ARG10, %ebx // m1
	movl	ARG11, %ecx // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X2_VS_LIB4
#else
	call inner_store_4x2_vs_lib4
#endif


	EPILOGUE

	ret

	.size	kernel_dgemm_nn_4x2_vs_lib4, .-kernel_dgemm_nn_4x2_vs_lib4





//                                 1      2              3          4          5             6          7
// void kernel_dsyrk_nt_l_4x2_lib4(int k, double *alpha, double *A, double *B, double *beta, double *C, double *D);

	.align 16
	.globl kernel_dsyrk_nt_l_4x2_lib4
	.type kernel_dsyrk_nt_l_4x2_lib4, @function
kernel_dsyrk_nt_l_4x2_lib4:

	PROLOGUE

	// zero accumulation registers

	xorpd	%xmm0, %xmm0
	movapd	%xmm0, %xmm1
	movapd	%xmm0, %xmm2
	movapd	%xmm0, %xmm3


	// call inner gemm kernel nt

	movl	ARG1, %eax // k
	movl	ARG3, %ebx  // A
	movl	ARG4, %ecx  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X2_LIB4
#else
	call inner_kernel_gemm_nt_4x2_lib4
#endif


	// call inner blend scale

	movl	ARG2, %eax // alpha
	movl	ARG5, %ebx // beta
	movl	ARG6, %ecx   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X2_LIB4
#else
	call inner_scale_ab_4x2_lib4
#endif


	// store n

	movl	ARG7, %eax // D

#if MACRO_LEVEL>=1
	INNER_STORE_L_4X2_LIB4
#else
	call inner_store_l_4x2_lib4
#endif

	EPILOGUE

	ret

	.size	kernel_dsyrk_nt_l_4x2_lib4, .-kernel_dsyrk_nt_l_4x2_lib4





//                                    1      2              3          4          5             6          7          8       9
// void kernel_dsyrk_nt_l_4x2_vs_lib4(int k, double *alpha, double *A, double *B, double *beta, double *C, double *D, int m1, int n1);

	.align 16
	.globl kernel_dsyrk_nt_l_4x2_vs_lib4
	.type kernel_dsyrk_nt_l_4x2_vs_lib4, @function
kernel_dsyrk_nt_l_4x2_vs_lib4:

	PROLOGUE

	// zero accumulation registers

	xorpd	%xmm0, %xmm0
	movapd	%xmm0, %xmm1
	movapd	%xmm0, %xmm2
	movapd	%xmm0, %xmm3


	// call inner gemm kernel nt

	movl	ARG1, %eax // k
	movl	ARG3, %ebx  // A
	movl	ARG4, %ecx  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X2_LIB4
#else
	call inner_kernel_gemm_nt_4x2_lib4
#endif


	// call inner blend scale

	movl	ARG2, %eax // alpha
	movl	ARG5, %ebx // beta
	movl	ARG6, %ecx   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X2_LIB4
#else
	call inner_scale_ab_4x2_lib4
#endif


	// store n

	movl	ARG7, %eax // D
	movl	ARG8, %ebx // m1
	movl	ARG9, %ecx // n1

#if MACRO_LEVEL>=1
	INNER_STORE_L_4X2_VS_LIB4
#else
	call inner_store_l_4x2_vs_lib4
#endif

	EPILOGUE

	ret

	.size	kernel_dsyrk_nt_l_4x2_vs_lib4, .-kernel_dsyrk_nt_l_4x2_vs_lib4





//                                      1      2          3          4             5          6          7          8
// void kernel_dtrsm_nt_rl_inv_4x2_lib4(int k, double *A, double *B, double *beta, double *C, double *D, double *E, double *inv_diag_E);

	.align 16
	.globl kernel_dtrsm_nt_rl_inv_4x2_lib4
	.type kernel_dtrsm_nt_rl_inv_4x2_lib4, @function
kernel_dtrsm_nt_rl_inv_4x2_lib4:
	
	PROLOGUE

	// zero accumulation registers

	xorpd	%xmm0, %xmm0
	movapd	%xmm0, %xmm1
	movapd	%xmm0, %xmm2
	movapd	%xmm0, %xmm3


	// call inner dgemm kernel nt 

	movl	ARG1, %eax // kmax
	movl	ARG2, %ebx // A
	movl	ARG3, %ecx // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X2_LIB4
#else
	call inner_kernel_gemm_nt_4x2_lib4
#endif


	// call inner blender_loader nn

	movl	ARG4, %eax // beta
	movl	ARG5, %ebx // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X2_LIB4
#else
	call inner_scale_m1b_4x2_lib4
#endif


	// solve

	movl	ARG7, %eax  // E 
	movl	ARG8, %ebx  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_INV_4X2_LIB4
#else
	call inner_edge_trsm_rlt_inv_4x2_lib4
#endif


	// store

	movl	ARG6, %eax // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X2_LIB4
#else
	call inner_store_4x2_lib4
#endif


	EPILOGUE

	ret

	.size	kernel_dtrsm_nt_rl_inv_4x2_lib4, .-kernel_dtrsm_nt_rl_inv_4x2_lib4





//                                         1      2          3          4             5          6          7          8                   9       10 
// void kernel_dtrsm_nt_rl_inv_4x2_vs_lib4(int k, double *A, double *B, double *beta, double *C, double *D, double *E, double *inv_diag_E, int m1, int n1);

	.align 16
	.globl kernel_dtrsm_nt_rl_inv_4x2_vs_lib4
	.type kernel_dtrsm_nt_rl_inv_4x2_vs_lib4, @function
kernel_dtrsm_nt_rl_inv_4x2_vs_lib4:
	
	PROLOGUE

	// zero accumulation registers

	xorpd	%xmm0, %xmm0
	movapd	%xmm0, %xmm1
	movapd	%xmm0, %xmm2
	movapd	%xmm0, %xmm3


	// call inner dgemm kernel nt 

	movl	ARG1, %eax // kmax
	movl	ARG2, %ebx // A
	movl	ARG3, %ecx // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X2_LIB4
#else
	call inner_kernel_gemm_nt_4x2_lib4
#endif


	// call inner blender_loader nn

	movl	ARG4, %eax // beta
	movl	ARG5, %ebx // beta

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X2_LIB4
#else
	call inner_scale_m1b_4x2_lib4
#endif


	// solve

	movl	ARG7, %eax  // E 
	movl	ARG8, %ebx  // inv_diag_E 
	movl	ARG10, %ecx // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_INV_4X2_VS_LIB4
#else
	call inner_edge_trsm_rlt_inv_4x2_vs_lib4
#endif


	// store

	movl	ARG6, %eax // D
	movl	ARG9, %ebx // m1
	movl	ARG10, %ecx // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X2_VS_LIB4
#else
	call inner_store_4x2_vs_lib4
#endif


	EPILOGUE

	ret

	.size	kernel_dtrsm_nt_rl_inv_4x2_vs_lib4, .-kernel_dtrsm_nt_rl_inv_4x2_vs_lib4





//                                  1      2          3          4          5          6
// void kernel_dpotrf_nt_l_4x2_lib4(int k, double *A, double *B, double *C, double *D, double *inv_diag_D);

	.align 16
	.globl kernel_dpotrf_nt_l_4x2_lib4
	.type kernel_dpotrf_nt_l_4x2_lib4, @function
kernel_dpotrf_nt_l_4x2_lib4:
	
	PROLOGUE

	// zero accumulation registers

	xorpd	%xmm0, %xmm0
	movapd	%xmm0, %xmm1
	movapd	%xmm0, %xmm2
	movapd	%xmm0, %xmm3


	// call inner dgemm kernel nt

	movl	ARG1, %eax
	movl	ARG2, %ebx
	movl	ARG3, %ecx

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X2_LIB4
#else
	call inner_kernel_gemm_nt_4x2_lib4
#endif


	// call inner blender_loader nn

	movl	ARG4, %eax // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_4X2_LIB4
#else
	call inner_scale_m11_4x2_lib4
#endif


	// factorization

	movl	ARG6, %eax  // inv_diag_D 
	movl	$ 4, %ebx // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_POTRF_4X2_VS_LIB4
#else
	call inner_edge_potrf_4x2_vs_lib4
#endif


	// store

	movl	ARG5, %eax // D

#if MACRO_LEVEL>=1
	INNER_STORE_L_4X2_LIB4
#else
	call inner_store_l_4x2_lib4
#endif


	EPILOGUE

	ret

	.size	kernel_dpotrf_nt_l_4x2_lib4, .-kernel_dpotrf_nt_l_4x2_lib4





//                                     1      2          3          4          5          6                   7       8
// void kernel_dpotrf_nt_l_4x2_vs_lib4(int k, double *A, double *B, double *C, double *D, double *inv_diag_D, int m1, int n1);

	.align 16
	.globl kernel_dpotrf_nt_l_4x2_vs_lib4
	.type kernel_dpotrf_nt_l_4x2_vs_lib4, @function
kernel_dpotrf_nt_l_4x2_vs_lib4:
	
	PROLOGUE

	// zero accumulation registers

	xorpd	%xmm0, %xmm0
	movapd	%xmm0, %xmm1
	movapd	%xmm0, %xmm2
	movapd	%xmm0, %xmm3


	// call inner dgemm kernel nt

	movl	ARG1, %eax
	movl	ARG2, %ebx
	movl	ARG3, %ecx

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X2_LIB4
#else
	call inner_kernel_gemm_nt_4x2_lib4
#endif


	// call inner blender_loader nn

	movl	ARG4, %eax // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_4X2_LIB4
#else
	call inner_scale_m11_4x2_lib4
#endif


	// factorization

	movl	ARG6, %eax  // inv_diag_D 
	movl	ARG8, %ebx // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_POTRF_4X2_VS_LIB4
#else
	call inner_edge_potrf_4x2_vs_lib4
#endif


	// store

	movl	ARG5, %eax // D
	movl	ARG7, %ebx // m1
	movl	ARG8, %ecx // n1

#if MACRO_LEVEL>=1
	INNER_STORE_L_4X2_VS_LIB4
#else
	call inner_store_l_4x2_vs_lib4
#endif


	EPILOGUE

	ret

	.size	kernel_dpotrf_nt_l_4x2_vs_lib4, .-kernel_dpotrf_nt_l_4x2_vs_lib4





//                                            1       2           3           4       5           6           7          8          9          10
// void kernel_dgemm_dtrsm_nt_rl_inv_4x2_lib4(int kp, double *Ap, double *Bp, int km, double *Am, double *Bm, double *C, double *D, double *E, double *inv_diag_E);

	.align 16
	.globl kernel_dgemm_dtrsm_nt_rl_inv_4x2_lib4
	.type kernel_dgemm_dtrsm_nt_rl_inv_4x2_lib4, @function
kernel_dgemm_dtrsm_nt_rl_inv_4x2_lib4:
	
	PROLOGUE

	// zero accumulation registers

	xorpd	%xmm0, %xmm0
	movapd	%xmm0, %xmm1
	movapd	%xmm0, %xmm2
	movapd	%xmm0, %xmm3


	// call inner dgemm kernel nt add

	movl	ARG1, %eax // kp
	movl	ARG2, %ebx  // Ap
	movl	ARG3, %ecx  // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X2_LIB4
#else
	call inner_kernel_gemm_nt_4x2_lib4
#endif


	// change sign
	movapd		.LC11, %xmm7
	xorpd		%xmm7, %xmm0
	xorpd		%xmm7, %xmm1
	xorpd		%xmm7, %xmm2
	xorpd		%xmm7, %xmm3


	// call inner dgemm kernel nt sub

	movl	ARG4, %eax // km
	movl	ARG5, %ebx   // Am
	movl	ARG6, %ecx   // Bm

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X2_LIB4
#else
	call inner_kernel_gemm_nt_4x2_lib4
#endif


	// call inner blender_loader nn

	movl	ARG7, %eax   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_4X2_LIB4
#else
	call inner_scale_m11_4x2_lib4
#endif


	// solve

	movl	ARG9, %eax  // E 
	movl	ARG10, %ebx  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_INV_4X2_LIB4
#else
	call inner_edge_trsm_rlt_inv_4x2_lib4
#endif


	// store

	movl	ARG8, %eax   // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X2_LIB4
#else
	call inner_store_4x2_lib4
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dgemm_dtrsm_nt_rl_inv_4x2_lib4, .-kernel_dgemm_dtrsm_nt_rl_inv_4x2_lib4
#endif





//                                               1       2           3           4       5           6           7          8          9          10                  11      12
// void kernel_dgemm_dtrsm_nt_rl_inv_4x2_vs_lib4(int kp, double *Ap, double *Bp, int km, double *Am, double *Bm, double *C, double *D, double *E, double *inv_diag_E, int km, int kn);

	.align 16
	.globl kernel_dgemm_dtrsm_nt_rl_inv_4x2_vs_lib4
	.type kernel_dgemm_dtrsm_nt_rl_inv_4x2_vs_lib4, @function
kernel_dgemm_dtrsm_nt_rl_inv_4x2_vs_lib4:
	
	PROLOGUE

	// zero accumulation registers

	xorpd	%xmm0, %xmm0
	movapd	%xmm0, %xmm1
	movapd	%xmm0, %xmm2
	movapd	%xmm0, %xmm3


	// call inner dgemm kernel nt add

	movl	ARG1, %eax // kp
	movl	ARG2, %ebx  // Ap
	movl	ARG3, %ecx  // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X2_LIB4
#else
	call inner_kernel_gemm_nt_4x2_lib4
#endif


	// change sign
	movapd		.LC11, %xmm7
	xorpd		%xmm7, %xmm0
	xorpd		%xmm7, %xmm1
	xorpd		%xmm7, %xmm2
	xorpd		%xmm7, %xmm3


	// call inner dgemm kernel nt sub

	movl	ARG4, %eax // km
	movl	ARG5, %ebx   // Am
	movl	ARG6, %ecx   // Bm

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X2_LIB4
#else
	call inner_kernel_gemm_nt_4x2_lib4
#endif


	// call inner blender_loader nn

	movl	ARG7, %eax  // C 

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_4X2_LIB4
#else
	call inner_scale_m11_4x2_lib4
#endif


	// solve

	movl	ARG9, %eax  // E 
	movl	ARG10, %ebx  // inv_diag_E 
	movl	ARG12, %ecx // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_INV_4X2_VS_LIB4
#else
	call inner_edge_trsm_rlt_inv_4x2_vs_lib4
#endif


	// store

	movl	ARG8, %eax // D 
	movl	ARG11, %ebx // km 
	movl	ARG12, %ecx // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_4X2_VS_LIB4
#else
	call inner_store_4x2_vs_lib4
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dgemm_dtrsm_nt_rl_inv_4x2_vs_lib4, .-kernel_dgemm_dtrsm_nt_rl_inv_4x2_vs_lib4
#endif





//                                        1       2           3           4       5           6           7          8          9
// void kernel_dsyrk_dpotrf_nt_l_4x2_lib4(int kp, double *Ap, double *Bp, int km, double *Am, double *Bm, double *C, double *D, double *inv_diag_D);

	.align 16
	.globl kernel_dsyrk_dpotrf_nt_l_4x2_lib4
	.type kernel_dsyrk_dpotrf_nt_l_4x2_lib4, @function
kernel_dsyrk_dpotrf_nt_l_4x2_lib4:
	
	PROLOGUE

	// zero accumulation registers

	xorpd	%xmm0, %xmm0
	movapd	%xmm0, %xmm1
	movapd	%xmm0, %xmm2
	movapd	%xmm0, %xmm3


	// call inner dgemm kernel nt add

	movl	ARG1, %eax // kp
	movl	ARG2, %ebx  // Ap
	movl	ARG3, %ecx  // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X2_LIB4
#else
	call inner_kernel_gemm_nt_4x2_lib4
#endif


	// change sign
	movapd		.LC11, %xmm7
	xorpd		%xmm7, %xmm0
	xorpd		%xmm7, %xmm1
	xorpd		%xmm7, %xmm2
	xorpd		%xmm7, %xmm3


	// call inner dgemm kernel nt sub

	movl	ARG4, %eax // km
	movl	ARG5, %ebx   // Am
	movl	ARG6, %ecx   // Bm

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X2_LIB4
#else
	call inner_kernel_gemm_nt_4x2_lib4
#endif


	// call inner blender_loader nn

	movl	ARG7, %eax   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_4X2_LIB4
#else
	call inner_scale_m11_4x2_lib4
#endif


	// factorization

	movl	ARG9, %eax  // inv_diag_D 
	movl	$ 4, %ebx

#if MACRO_LEVEL>=1
	INNER_EDGE_POTRF_4X2_VS_LIB4
#else
	call inner_edge_potrf_4x2_vs_lib4
#endif


	// store

	movl	ARG8, %eax  // D 

#if MACRO_LEVEL>=1
	INNER_STORE_L_4X2_LIB4
#else
	call inner_store_l_4x2_lib4
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dsyrk_dpotrf_nt_l_4x2_lib4, .-kernel_dsyrk_dpotrf_nt_l_4x2_lib4
#endif





//                                           1       2           3           4       5           6           7          8          9                   10      11
// void kernel_dsyrk_dpotrf_nt_l_4x2_vs_lib4(int kp, double *Ap, double *Bp, int km, double *Am, double *Bm, double *C, double *D, double *inv_diag_D, int km, int kn);

	.align 16
	.globl kernel_dsyrk_dpotrf_nt_l_4x2_vs_lib4
	.type kernel_dsyrk_dpotrf_nt_l_4x2_vs_lib4, @function
kernel_dsyrk_dpotrf_nt_l_4x2_vs_lib4:
	
	PROLOGUE

	// zero accumulation registers

	xorpd	%xmm0, %xmm0
	movapd	%xmm0, %xmm1
	movapd	%xmm0, %xmm2
	movapd	%xmm0, %xmm3


	// call inner dgemm kernel nt add

	movl	ARG1, %eax // kp
	movl	ARG2, %ebx  // Ap
	movl	ARG3, %ecx  // Bp

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X2_LIB4
#else
	call inner_kernel_gemm_nt_4x2_lib4
#endif


	// change sign
	movapd		.LC11, %xmm7
	xorpd		%xmm7, %xmm0
	xorpd		%xmm7, %xmm1
	xorpd		%xmm7, %xmm2
	xorpd		%xmm7, %xmm3


	// call inner dgemm kernel nt sub

	movl	ARG4, %eax // km
	movl	ARG5, %ebx   // Am
	movl	ARG6, %ecx   // Bm

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X2_LIB4
#else
	call inner_kernel_gemm_nt_4x2_lib4
#endif


	// call inner blender_loader nn

	movl	ARG7, %eax   // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_4X2_LIB4
#else
	call inner_scale_m11_4x2_lib4
#endif


	// factorization

	movl	ARG9, %eax  // inv_diag_D 
	movl	ARG11, %ebx // kn 

#if MACRO_LEVEL>=1
	INNER_EDGE_POTRF_4X2_VS_LIB4
#else
	call inner_edge_potrf_4x2_vs_lib4
#endif


	// store

	movl	ARG8, %eax  // D 
	movl	ARG10, %ebx // km 
	movl	ARG11, %ecx // kn 

#if MACRO_LEVEL>=1
	INNER_STORE_L_4X2_VS_LIB4
#else
	call inner_store_l_4x2_vs_lib4
#endif


	EPILOGUE

	ret

#if defined(OS_LINUX)
	.size	kernel_dsyrk_dpotrf_nt_l_4x2_vs_lib4, .-kernel_dsyrk_dpotrf_nt_l_4x2_vs_lib4
#endif





//                                  1      2              3          4            5          6        7
// void kernel_dtrmm_nn_rl_4x2_lib4(int k, double *alpha, double *A, int offsetB, double *B, int sdb, double *D);

	.align 16
	.globl kernel_dtrmm_nn_rl_4x2_lib4
	.type kernel_dtrmm_nn_rl_4x2_lib4, @function
kernel_dtrmm_nn_rl_4x2_lib4:
	
	PROLOGUE

	// zero accumulation registers

	xorpd	%xmm0, %xmm0
	movapd	%xmm0, %xmm1
	movapd	%xmm0, %xmm2
	movapd	%xmm0, %xmm3



	// initial triangle

	movl	ARG1, %eax // k
	movl	ARG3, %ebx // A
	movl	ARG5, %ecx // B
	movl	ARG6, %edx // sdb
	sall	$ 5, %edx // 4*sdb*sizeof(double)
	movl	ARG4, %esi // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMM_NN_RL_4X2_LIB4
#else
	call inner_edge_trmm_nn_rl_4x2_lib4
#endif

	// call inner dgemm kernel nt after initial triangle

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X2_LIB4
#else
	call inner_kernel_gemm_add_nn_4x2_lib4
#endif


	// call inner scale

	movl	ARG2, %eax // alpha

#if MACRO_LEVEL>=1
	INNER_SCALE_A0_4X2_LIB4
#else
	call inner_scale_a0_4x2_lib4
#endif


	// store n

	movl	ARG7, %eax // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X2_LIB4
#else
	call inner_store_4x2_lib4
#endif


	EPILOGUE

	ret

	.size	kernel_dtrmm_nn_rl_4x2_lib4, .-kernel_dtrmm_nn_rl_4x2_lib4





//                                     1      2              3          4            5          6        7          8       9
// void kernel_dtrmm_nn_rl_4x2_vs_lib4(int k, double *alpha, double *A, int offsetB, double *B, int sdb, double *D, int m1, int n1);

	.align 16
	.globl kernel_dtrmm_nn_rl_4x2_vs_lib4
	.type kernel_dtrmm_nn_rl_4x2_vs_lib4, @function
kernel_dtrmm_nn_rl_4x2_vs_lib4:
	
	PROLOGUE

	// zero accumulation registers

	xorpd	%xmm0, %xmm0
	movapd	%xmm0, %xmm1
	movapd	%xmm0, %xmm2
	movapd	%xmm0, %xmm3



	// initial triangle

	movl	ARG1, %eax // k
	movl	ARG3, %ebx // A
	movl	ARG5, %ecx // B
	movl	ARG6, %edx // sdb
	sall	$ 5, %edx // 4*sdb*sizeof(double)
	movl	ARG4, %esi // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMM_NN_RL_4X2_VS_LIB4
#else
	call inner_edge_trmm_nn_rl_4x2_vs_lib4
#endif

	// call inner dgemm kernel nt after initial triangle

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X2_LIB4
#else
	call inner_kernel_gemm_add_nn_4x2_lib4
#endif


	// call inner scale

	movl	ARG2, %eax // alpha

#if MACRO_LEVEL>=1
	INNER_SCALE_A0_4X2_LIB4
#else
	call inner_scale_a0_4x2_lib4
#endif


	// store n

	movl	ARG7, %eax // D
	movl	ARG8, %ebx // m1
	movl	ARG9, %ecx // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X2_VS_LIB4
#else
	call inner_store_4x2_vs_lib4
#endif


	EPILOGUE

	ret

	.size	kernel_dtrmm_nn_rl_4x2_vs_lib4, .-kernel_dtrmm_nn_rl_4x2_vs_lib4





//#if defined(BLAS_API)
#if ( defined(BLAS_API) | ( defined(LA_HIGH_PERFORMANCE) & defined(MF_COLMAJ) ) )

#include "kernel_dgemm_4x2_lib.S"

#endif





	// read-only data
	.section	.rodata.cst32,"aM",@progbits,32

	.align 32
.LC00: // { 1.0 1.0 1.0 1.0 }
	.long	0
	.long	1072693248
	.long	0
	.long	1072693248
	.long	0
	.long	1072693248
	.long	0
	.long	1072693248

#if defined(OS_LINUX) | defined(OS_WINDOWS)
	.align 32
.LC11:
#elif defined(OS_MAC)
	.align 5
LC11:
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



	.section	.note.GNU-stack,"",@progbits

