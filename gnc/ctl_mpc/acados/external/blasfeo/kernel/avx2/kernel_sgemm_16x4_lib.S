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





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- 8*sda*sizeof(float)
// r13   <- B
// r14   <- ldb
// ymm0  <- []
// ymm1  <- []
// ymm2  <- []
// ymm3  <- []
// ymm4  <- []
// ymm5  <- []
// ymm6  <- []
// ymm7  <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NT_16X4_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemm_nt_16x4_lib8c)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vmovups			0(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vmovups			32(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vmovups			64(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vmovups			96(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 128, %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vmovups			0(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vmovups			32(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vmovups			64(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vmovups			96(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 128, %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vmovups			0(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	subl	$ 1, %r10d
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nt_16x4_lib8c)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- 8*sda*sizeof(float)
// r13   <- B
// r14   <- ldb
// ymm0  <- []
// ymm1  <- []
// ymm2  <- []
// ymm3  <- []
// ymm4  <- []
// ymm5  <- []
// ymm6  <- []
// ymm7  <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NT_16X3_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemm_nt_16x3_lib8c)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vmovups			0(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vmovups			32(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vmovups			64(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vmovups			96(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 128, %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vmovups			0(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vmovups			32(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vmovups			64(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vmovups			96(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 128, %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vmovups			0(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	subl	$ 1, %r10d
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nt_16x3_lib8c)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- 8*sda*sizeof(float)
// r13   <- B
// r14   <- ldb
// ymm0  <- []
// ymm1  <- []
// ymm2  <- []
// ymm3  <- []
// ymm4  <- []
// ymm5  <- []
// ymm6  <- []
// ymm7  <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NT_16X2_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemm_nt_16x2_lib8c)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vmovups			0(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	8(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vmovups			32(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	8(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vmovups			64(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	8(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vmovups			96(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	8(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 128, %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vmovups			0(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	8(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vmovups			32(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	8(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vmovups			64(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	8(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vmovups			96(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	8(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 128, %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vmovups			0(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	8(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	subl	$ 1, %r10d
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nt_16x2_lib8c)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- 8*sda*sizeof(float)
// r13   <- B
// r14   <- ldb
// ymm0  <- []
// ymm1  <- []
// ymm2  <- []
// ymm3  <- []
// ymm4  <- []
// ymm5  <- []
// ymm6  <- []
// ymm7  <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NT_16X1_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemm_nt_16x1_lib8c)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vmovups			0(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
//	vbroadcastss	4(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r13, %r14, 8)
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	8(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vmovups			32(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
//	vbroadcastss	4(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r13, %r14, 8)
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	8(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vmovups			64(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
//	vbroadcastss	4(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r13, %r14, 8)
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	8(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vmovups			96(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
//	vbroadcastss	4(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r13, %r14, 8)
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	8(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 128, %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vmovups			0(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
//	vbroadcastss	4(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	8(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vmovups			32(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
//	vbroadcastss	4(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	8(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vmovups			64(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
//	vbroadcastss	4(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	8(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vmovups			96(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
//	vbroadcastss	4(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	8(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 128, %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vmovups			0(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
//	vbroadcastss	4(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	8(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r14, %r13

	subl	$ 1, %r10d
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nt_16x1_lib8c)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- 8*sda*sizeof(float)
// r13   <- B
// r14   <- ldb
// ymm0  <- []
// ymm1  <- []
// ymm2  <- []
// ymm3  <- []
// ymm4  <- []
// ymm5  <- []
// ymm6  <- []
// ymm7  <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_16X4_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemm_nn_16x4_lib8c)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	leaq	0(%r14, %r14, 2), %r15

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0	0(%r12, %r13, 2) // software prefetch
//	prefetcht0	64(%r12, %r13, 2) // software prefetch

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vmovups			0(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	0(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vbroadcastss	0(%r13, %r15), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vfmadd231ps		%ymm14, %ymm12, %ymm7

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vmovups			32(%r11, %r12), %ymm14 // A
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	4(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	4(%r13, %r14, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vbroadcastss	4(%r13, %r15), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vfmadd231ps		%ymm14, %ymm12, %ymm7

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vmovups			64(%r11, %r12), %ymm14 // A
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	8(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	8(%r13, %r14, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vbroadcastss	8(%r13, %r15), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vfmadd231ps		%ymm14, %ymm12, %ymm7

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vmovups			96(%r11, %r12), %ymm14 // A
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	12(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	12(%r13, %r14, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vbroadcastss	12(%r13, %r15), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vfmadd231ps		%ymm14, %ymm12, %ymm7

	subl	$ 4, %r10d
	addq	$ 16, %r13
	addq	$ 128, %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vmovups			0(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	0(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vbroadcastss	0(%r13, %r15), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vfmadd231ps		%ymm14, %ymm12, %ymm7

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vmovups			32(%r11, %r12), %ymm14 // A
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	4(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	4(%r13, %r14, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vbroadcastss	4(%r13, %r15), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vfmadd231ps		%ymm14, %ymm12, %ymm7

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vmovups			64(%r11, %r12), %ymm14 // A
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	8(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	8(%r13, %r14, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vbroadcastss	8(%r13, %r15), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vfmadd231ps		%ymm14, %ymm12, %ymm7

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vmovups			96(%r11, %r12), %ymm14 // A
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	12(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	12(%r13, %r14, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vbroadcastss	12(%r13, %r15), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vfmadd231ps		%ymm14, %ymm12, %ymm7

	subl	$ 4, %r10d
	addq	$ 16, %r13
	addq	$ 128, %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vmovups			0(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	0(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vbroadcastss	0(%r13, %r15), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vfmadd231ps		%ymm14, %ymm12, %ymm7

	subl	$ 1, %r10d
	addq	$ 4, %r13
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nn_16x4_lib8c)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- 8*sda*sizeof(float)
// r13   <- B
// r14   <- ldb
// ymm0  <- []
// ymm1  <- []
// ymm2  <- []
// ymm3  <- []
// ymm4  <- []
// ymm5  <- []
// ymm6  <- []
// ymm7  <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_16X3_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemm_nn_16x3_lib8c)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

//	leaq	0(%r14, %r14, 2), %r15

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0	0(%r12, %r13, 2) // software prefetch
//	prefetcht0	64(%r12, %r13, 2) // software prefetch

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vmovups			0(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	0(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	0(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vmovups			32(%r11, %r12), %ymm14 // A
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	4(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	4(%r13, %r14, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	4(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vmovups			64(%r11, %r12), %ymm14 // A
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	8(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	8(%r13, %r14, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	8(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vmovups			96(%r11, %r12), %ymm14 // A
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	12(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	12(%r13, %r14, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7

	subl	$ 4, %r10d
	addq	$ 16, %r13
	addq	$ 128, %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vmovups			0(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	0(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	0(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vmovups			32(%r11, %r12), %ymm14 // A
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	4(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	4(%r13, %r14, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	4(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vmovups			64(%r11, %r12), %ymm14 // A
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	8(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	8(%r13, %r14, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	8(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vmovups			96(%r11, %r12), %ymm14 // A
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	12(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	12(%r13, %r14, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7

	subl	$ 4, %r10d
	addq	$ 16, %r13
	addq	$ 128, %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vmovups			0(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	0(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	0(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7

	subl	$ 1, %r10d
	addq	$ 4, %r13
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nn_16x3_lib8c)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- 8*sda*sizeof(float)
// r13   <- B
// r14   <- ldb
// ymm0  <- []
// ymm1  <- []
// ymm2  <- []
// ymm3  <- []
// ymm4  <- []
// ymm5  <- []
// ymm6  <- []
// ymm7  <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_16X2_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemm_nn_16x2_lib8c)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

//	leaq	0(%r14, %r14, 2), %r15

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0	0(%r12, %r13, 2) // software prefetch
//	prefetcht0	64(%r12, %r13, 2) // software prefetch

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vmovups			0(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	0(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	0(%r13, %r14, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	0(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vmovups			32(%r11, %r12), %ymm14 // A
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	4(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	4(%r13, %r14, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	4(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vmovups			64(%r11, %r12), %ymm14 // A
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	8(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	8(%r13, %r14, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	8(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vmovups			96(%r11, %r12), %ymm14 // A
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	12(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	12(%r13, %r14, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7

	subl	$ 4, %r10d
	addq	$ 16, %r13
	addq	$ 128, %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vmovups			0(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	0(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	0(%r13, %r14, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	0(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vmovups			32(%r11, %r12), %ymm14 // A
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	4(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	4(%r13, %r14, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	4(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vmovups			64(%r11, %r12), %ymm14 // A
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	8(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	8(%r13, %r14, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	8(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vmovups			96(%r11, %r12), %ymm14 // A
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	12(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	12(%r13, %r14, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7

	subl	$ 4, %r10d
	addq	$ 16, %r13
	addq	$ 128, %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vmovups			0(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vbroadcastss	0(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	0(%r13, %r14, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	0(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7

	subl	$ 1, %r10d
	addq	$ 4, %r13
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nn_16x2_lib8c)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- 8*sda*sizeof(float)
// r13   <- B
// r14   <- ldb
// ymm0  <- []
// ymm1  <- []
// ymm2  <- []
// ymm3  <- []
// ymm4  <- []
// ymm5  <- []
// ymm6  <- []
// ymm7  <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_16X1_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemm_nn_16x1_lib8c)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

//	leaq	0(%r14, %r14, 2), %r15

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0	0(%r12, %r13, 2) // software prefetch
//	prefetcht0	64(%r12, %r13, 2) // software prefetch

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vmovups			0(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
//	vbroadcastss	0(%r13, %r14), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	0(%r13, %r14, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	0(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vmovups			32(%r11, %r12), %ymm14 // A
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
//	vbroadcastss	4(%r13, %r14), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	4(%r13, %r14, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	4(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vmovups			64(%r11, %r12), %ymm14 // A
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
//	vbroadcastss	8(%r13, %r14), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	8(%r13, %r14, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	8(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vmovups			96(%r11, %r12), %ymm14 // A
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
//	vbroadcastss	12(%r13, %r14), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	12(%r13, %r14, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7

	subl	$ 4, %r10d
	addq	$ 16, %r13
	addq	$ 128, %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vmovups			0(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
//	vbroadcastss	0(%r13, %r14), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	0(%r13, %r14, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	0(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vmovups			32(%r11, %r12), %ymm14 // A
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
//	vbroadcastss	4(%r13, %r14), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	4(%r13, %r14, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	4(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vmovups			64(%r11, %r12), %ymm14 // A
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
//	vbroadcastss	8(%r13, %r14), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	8(%r13, %r14, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	8(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vmovups			96(%r11, %r12), %ymm14 // A
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
//	vbroadcastss	12(%r13, %r14), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	12(%r13, %r14, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7

	subl	$ 4, %r10d
	addq	$ 16, %r13
	addq	$ 128, %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vmovups			0(%r11, %r12), %ymm14 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
//	vbroadcastss	0(%r13, %r14), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	0(%r13, %r14, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	0(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7

	subl	$ 1, %r10d
	addq	$ 4, %r13
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nn_16x1_lib8c)
#endif





// common inner routine with file scope
//
// transpose
//
// input arguments:
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_TRAN_16X4_LIB
#else
	FUN_START(inner_tran_16x4_lib)
#endif
	
	// transpose
	vunpcklps	%ymm1, %ymm0, %ymm13
	vunpckhps	%ymm1, %ymm0, %ymm12
	vunpcklps	%ymm3, %ymm2, %ymm15
	vunpckhps	%ymm3, %ymm2, %ymm14

	vunpcklpd	%ymm15, %ymm13, %ymm0
	vunpckhpd	%ymm15, %ymm13, %ymm1
	vunpcklpd	%ymm14, %ymm12, %ymm2
	vunpckhpd	%ymm14, %ymm12, %ymm3

	vunpcklps	%ymm5, %ymm4, %ymm13
	vunpckhps	%ymm5, %ymm4, %ymm12
	vunpcklps	%ymm7, %ymm6, %ymm15
	vunpckhps	%ymm7, %ymm6, %ymm14

	vunpcklpd	%ymm15, %ymm13, %ymm4
	vunpckhpd	%ymm15, %ymm13, %ymm5
	vunpcklpd	%ymm14, %ymm12, %ymm6
	vunpckhpd	%ymm14, %ymm12, %ymm7

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_tran_16x4_lib)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- ldc*sizeof(float)
// ymm0 <- []
// ...
// ymm7 <- []
//
// output arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- ldc*sizeof(float)
// ymm0 <- []
// ...
// ymm7 <- []

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_16X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_16x4_lib)
#endif
	
	// alpha
	vbroadcastss	0(%r10), %ymm11

	vmulps		%ymm0, %ymm11, %ymm0
	vmulps		%ymm1, %ymm11, %ymm1
	vmulps		%ymm2, %ymm11, %ymm2
	vmulps		%ymm3, %ymm11, %ymm3

	vmulps		%ymm4, %ymm11, %ymm4
	vmulps		%ymm5, %ymm11, %ymm5
	vmulps		%ymm6, %ymm11, %ymm6
	vmulps		%ymm7, %ymm11, %ymm7

	// beta
	vbroadcastss	0(%r11), %ymm14

	vxorps		%ymm15, %ymm15, %ymm15 // 0.0

	vucomiss	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	leaq		0(%r13, %r13, 2), %r14

	vmovups		0(%r12), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm0
	vmovups		32(%r12), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm4
	vmovups		0(%r12, %r13), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm1
	vmovups		32(%r12, %r13), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm5
	vmovups		0(%r12, %r13, 2), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm2
	vmovups		32(%r12, %r13, 2), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm6
	vmovups		0(%r12, %r14), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm3
	vmovups		32(%r12, %r14), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm7

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_16x4_lib)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- ldc*sizeof(float)
// r14   <- km
// r15   <- kn
// ymm0 <- []
// ...
// ymm7 <- []
//
// output arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- ldc*sizeof(float)
// r14   <- km
// r15   <- kn
// ymm0 <- []
// ...
// ymm7 <- []

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_16X4_VS_LIB
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_16x4_vs_lib)
#endif
	
	// alpha
	vbroadcastss	0(%r10), %ymm11

	vmulps		%ymm0, %ymm11, %ymm0
	vmulps		%ymm1, %ymm11, %ymm1
	vmulps		%ymm2, %ymm11, %ymm2
	vmulps		%ymm3, %ymm11, %ymm3

	vmulps		%ymm4, %ymm11, %ymm4
	vmulps		%ymm5, %ymm11, %ymm5
	vmulps		%ymm6, %ymm11, %ymm6
	vmulps		%ymm7, %ymm11, %ymm7

	// beta
	vbroadcastss	0(%r11), %ymm14

	vxorps		%ymm15, %ymm15, %ymm15 // 0.0

	vucomiss	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	// compute mask for rows
	vcvtsi2ss	%r14d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC01(%rip), %ymm13
#elif defined(OS_MAC)
	vmovups		LC01(%rip), %ymm13
#endif
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%ymm15, %ymm13, %ymm13

	leaq		0(%r13, %r13, 2), %rax

	vmovups		0(%r12), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm0
	vmaskmovps	32(%r12), %ymm13, %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm4
	cmpl		$ 2, %r13d
	jl			0f // end
	vmovups		0(%r12, %r13), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm1
	vmaskmovps	32(%r12, %r13), %ymm13, %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm5
	cmpl		$ 3, %r13d
	jl			0f // end
	vmovups		0(%r12, %r13, 2), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm2
	vmaskmovps	32(%r12, %r13, 2), %ymm13, %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm6
	je			0f // end
	vmovups		0(%r12, %rax), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm3
	vmaskmovps	32(%r12, %rax), %ymm13, %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm7

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_16x4_vs_lib)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- ldc*sizeof(float)
// ymm0 <- []
// ...
// ymm7 <- []
//
// output arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- ldc*sizeof(float)
// ymm0 <- []
// ...
// ymm7 <- []

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_4X16_LIB
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_4x16_lib)
#endif
	
	// alpha
	vbroadcastss	0(%r10), %ymm11

	vmulps		%ymm0, %ymm11, %ymm0
	vmulps		%ymm1, %ymm11, %ymm1
	vmulps		%ymm2, %ymm11, %ymm2
	vmulps		%ymm3, %ymm11, %ymm3

	vmulps		%ymm4, %ymm11, %ymm4
	vmulps		%ymm5, %ymm11, %ymm5
	vmulps		%ymm6, %ymm11, %ymm6
	vmulps		%ymm7, %ymm11, %ymm7

	// beta
	vbroadcastss	0(%r11), %ymm14

	vxorps		%ymm15, %ymm15, %ymm15 // 0.0

	vucomiss	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	vmovups		0(%r12), %xmm15
	vinsertf128	$ 1, 0(%r12, %r13, 4), %ymm15, %ymm15
	add			%r13, %r12
	vfmadd231ps	%ymm15, %ymm14, %ymm0
	vmovups		0(%r12), %xmm15
	vinsertf128	$ 1, 0(%r12, %r13, 4), %ymm15, %ymm15
	add			%r13, %r12
	vfmadd231ps	%ymm15, %ymm14, %ymm1
	vmovups		0(%r12), %xmm15
	vinsertf128	$ 1, 0(%r12, %r13, 4), %ymm15, %ymm15
	add			%r13, %r12
	vfmadd231ps	%ymm15, %ymm14, %ymm2
	vmovups		0(%r12), %xmm15
	vinsertf128	$ 1, 0(%r12, %r13, 4), %ymm15, %ymm15
	add			%r13, %r12
	vfmadd231ps	%ymm15, %ymm14, %ymm3

	leaq		0(%r12, %r13, 4), %r12

	vmovups		0(%r12), %xmm15
	vinsertf128	$ 1, 0(%r12, %r13, 4), %ymm15, %ymm15
	add			%r13, %r12
	vfmadd231ps	%ymm15, %ymm14, %ymm4
	vmovups		0(%r12), %xmm15
	vinsertf128	$ 1, 0(%r12, %r13, 4), %ymm15, %ymm15
	add			%r13, %r12
	vfmadd231ps	%ymm15, %ymm14, %ymm5
	vmovups		0(%r12), %xmm15
	vinsertf128	$ 1, 0(%r12, %r13, 4), %ymm15, %ymm15
	add			%r13, %r12
	vfmadd231ps	%ymm15, %ymm14, %ymm6
	vmovups		0(%r12), %xmm15
	vinsertf128	$ 1, 0(%r12, %r13, 4), %ymm15, %ymm15
	add			%r13, %r12
	vfmadd231ps	%ymm15, %ymm14, %ymm7

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_4x16_lib)
#endif





// common inner routine with file scope
//
// scale for generic alpha and beta
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- ldc
// r14d   <- km
// r15d   <- kn
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_4X16_VS_LIB
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_4x16_vs_lib)
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

	// beta
	vbroadcastss	0(%r11), %ymm14

	vxorps		%xmm15, %xmm15, %xmm15 // 0.0
	vucomiss	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end


	vcvtsi2ss	%r14d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %ymm13
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %ymm13
#endif
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
//	vinsertf128	$ 0x1, %xmm15, %xmm15, %xmm15
	vsubps		%xmm15, %xmm13, %xmm13


	vmaskmovps	0(%r12), %xmm13, %xmm15
	vmaskmovps	0(%r12, %r13, 4), %xmm13, %xmm12
	vinsertf128	$ 1, %xmm12, %ymm15, %ymm15
	add			%r13, %r12
	vfmadd231ps	%ymm15, %ymm14, %ymm0
	vmaskmovps	0(%r12), %xmm13, %xmm15
	vmaskmovps	0(%r12, %r13, 4), %xmm13, %xmm12
	vinsertf128	$ 1, %xmm12, %ymm15, %ymm15
	add			%r13, %r12
	vfmadd231ps	%ymm15, %ymm14, %ymm1
	vmaskmovps	0(%r12), %xmm13, %xmm15
	vmaskmovps	0(%r12, %r13, 4), %xmm13, %xmm12
	vinsertf128	$ 1, %xmm12, %ymm15, %ymm15
	add			%r13, %r12
	vfmadd231ps	%ymm15, %ymm14, %ymm2
	vmaskmovps	0(%r12), %xmm13, %xmm15
	vmaskmovps	0(%r12, %r13, 4), %xmm13, %xmm12
	vinsertf128	$ 1, %xmm12, %ymm15, %ymm15
	add			%r13, %r12
	vfmadd231ps	%ymm15, %ymm14, %ymm3

	leaq		0(%r12, %r13, 4), %r12
	vxorps		%xmm12, %xmm12, %xmm12 // 0.0

	vmaskmovps	0(%r12), %xmm13, %xmm15
	cmpl		$ 13, %r15d
	jl			1f
	vmaskmovps	0(%r12, %r13, 4), %xmm13, %xmm12
1:
	vinsertf128	$ 1, %xmm12, %ymm15, %ymm15
	add			%r13, %r12
	vfmadd231ps	%ymm15, %ymm14, %ymm4
	cmpl		$ 10, %r15d
	jl			0f // end
	vmaskmovps	0(%r12), %xmm13, %xmm15
	cmpl		$ 14, %r15d
	jl			1f // end
	vmaskmovps	0(%r12, %r13, 4), %xmm13, %xmm12
1:
	vinsertf128	$ 1, %xmm12, %ymm15, %ymm15
	add			%r13, %r12
	vfmadd231ps	%ymm15, %ymm14, %ymm5
	cmpl		$ 11, %r15d
	jl			0f // end
	vmaskmovps	0(%r12), %xmm13, %xmm15
	cmpl		$ 15, %r15d
	jl			1f // end
	vmaskmovps	0(%r12, %r13, 4), %xmm13, %xmm12
1:
	vinsertf128	$ 1, %xmm12, %ymm15, %ymm15
	add			%r13, %r12
	vfmadd231ps	%ymm15, %ymm14, %ymm6
	cmpl		$ 12, %r15d
	jl			0f // end
	vmaskmovps	0(%r12), %xmm13, %xmm15
	cmpl		$ 15, %r15d
	je			1f // end
	vmaskmovps	0(%r12, %r13, 4), %xmm13, %xmm12
1:
	vinsertf128	$ 1, %xmm12, %ymm15, %ymm15
	add			%r13, %r12
	vfmadd231ps	%ymm15, %ymm14, %ymm7

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_4x16_vs_lib)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// r11  <- ldd*sizeof(float)
// ymm0 <- []
// ...
// ymm7 <- []
//
// output arguments:
// r10  <- D
// r11  <- ldd*sizeof(float)
// ymm0 <- []
// ...
// ymm7 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_16X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_store_16x4_lib)
#endif
	
	leaq		0(%r11, %r11, 2), %r12

	vmovups 	%ymm0,  0(%r10)
	vmovups 	%ymm4,  32(%r10)
	vmovups 	%ymm1,  0(%r10, %r11)
	vmovups 	%ymm5,  32(%r10, %r11)
	vmovups 	%ymm2,  0(%r10, %r11, 2)
	vmovups 	%ymm6,  32(%r10, %r11, 2)
	vmovups 	%ymm3,  0(%r10, %r12)
	vmovups 	%ymm7,  32(%r10, %r12)


#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_16x4_lib)
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
// ...
// ymm7 <- []
//
// output arguments:
// r10  <- D
// r11  <- 4*sdd*sizeof(double)
// r12  <- km
// r13  <- kn
// ymm0 <- []
// ...
// ymm7 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_16X4_VS_LIB
#else
	.p2align 4,,15
	FUN_START(inner_store_16x4_vs_lib)
#endif
	
	// compute mask for rows
	vcvtsi2ss	%r12d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC01(%rip), %ymm13
#elif defined(OS_MAC)
	vmovups		LC01(%rip), %ymm13
#endif
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%ymm15, %ymm13, %ymm15

	leaq		0(%r11, %r11, 2), %r14

	vmovups		%ymm0, 0(%r10)
	vmaskmovps	%ymm4, %ymm15, 32(%r10)
	cmpl		$ 2, %r13d
	jl			0f // end
	vmovups		%ymm1, 0(%r10, %r11)
	vmaskmovps	%ymm5, %ymm15, 32(%r10, %r11)
	cmpl		$ 3, %r13d
	jl			0f // end
	vmovups		%ymm2, 0(%r10, %r11, 2)
	vmaskmovps	%ymm6, %ymm15, 32(%r10, %r11, 2)
	je			0f // end
	vmovups		%ymm3, 0(%r10, %r14)
	vmaskmovps	%ymm7, %ymm15, 32(%r10, %r14)
	//
//	jmp		0f

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_16x4_vs_lib)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// r11  <- ldd*sizeof(float)
// ymm0 <- []
// ...
// ymm7 <- []
//
// output arguments:
// r10  <- D
// r11  <- ldd*sizeof(float)
// ymm0 <- []
// ...
// ymm7 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X16_LIB
#else
	.p2align 4,,15
	FUN_START(inner_store_4x16_lib)
#endif
	
	vmovups 		%xmm0,  0(%r10)
	addq			%r11, %r10
	vmovups 		%xmm1,  0(%r10)
	addq			%r11, %r10
	vmovups 		%xmm2,  0(%r10)
	addq			%r11, %r10
	vmovups 		%xmm3,  0(%r10)
	addq			%r11, %r10
	vextractf128	$ 1, %ymm0, %xmm15
	vmovups 		%xmm15,  0(%r10)
	addq			%r11, %r10
	vextractf128	$ 1, %ymm1, %xmm15
	vmovups 		%xmm15,  0(%r10)
	addq			%r11, %r10
	vextractf128	$ 1, %ymm2, %xmm15
	vmovups 		%xmm15,  0(%r10)
	addq			%r11, %r10
	vextractf128	$ 1, %ymm3, %xmm15
	vmovups 		%xmm15,  0(%r10)
	addq			%r11, %r10

	vmovups 		%xmm4,  0(%r10)
	addq			%r11, %r10
	vmovups 		%xmm5,  0(%r10)
	addq			%r11, %r10
	vmovups 		%xmm6,  0(%r10)
	addq			%r11, %r10
	vmovups 		%xmm7,  0(%r10)
	addq			%r11, %r10
	vextractf128	$ 1, %ymm4, %xmm15
	vmovups 		%xmm15,  0(%r10)
	addq			%r11, %r10
	vextractf128	$ 1, %ymm5, %xmm15
	vmovups 		%xmm15,  0(%r10)
	addq			%r11, %r10
	vextractf128	$ 1, %ymm6, %xmm15
	vmovups 		%xmm15,  0(%r10)
	addq			%r11, %r10
	vextractf128	$ 1, %ymm7, %xmm15
	vmovups 		%xmm15,  0(%r10)
	addq			%r11, %r10

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4x16_lib)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// r11  <- ldd*sizeof(float)
// r12  <- km
// r13  <- kn
// ymm0 <- []
// ...
// ymm7 <- []
//
// output arguments:
// r10  <- D
// r11  <- ldd*sizeof(float)
// r12  <- km
// r13  <- kn
// ymm0 <- []
// ...
// ymm7 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X16_VS_LIB
#else
	.p2align 4,,15
	FUN_START(inner_store_4x16_vs_lib)
#endif
	
	// compute mask for rows
	vcvtsi2ss	%r12d, %xmm14, %xmm14
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %ymm12
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %ymm12
#endif
	vshufps		$ 0x00, %xmm14, %xmm14, %xmm14
//	vinsertf128	$ 0x1, %xmm14, %xmm14, %xmm14
	vsubps		%xmm14, %xmm12, %xmm14

	vmaskmovps		%xmm0, %xmm14, 0(%r10)
	addq			%r11, %r10
	vmaskmovps		%xmm1, %xmm14, 0(%r10)
	addq			%r11, %r10
	vmaskmovps		%xmm2, %xmm14, 0(%r10)
	addq			%r11, %r10
	vmaskmovps		%xmm3, %xmm14, 0(%r10)
	addq			%r11, %r10
	vextractf128	$ 1, %ymm0, %xmm15
	vmaskmovps		%xmm15, %xmm14, 0(%r10)
	addq			%r11, %r10
	vextractf128	$ 1, %ymm1, %xmm15
	vmaskmovps		%xmm15, %xmm14, 0(%r10)
	addq			%r11, %r10
	vextractf128	$ 1, %ymm2, %xmm15
	vmaskmovps		%xmm15, %xmm14, 0(%r10)
	addq			%r11, %r10
	vextractf128	$ 1, %ymm3, %xmm15
	vmaskmovps		%xmm15, %xmm14, 0(%r10)
	addq			%r11, %r10

	vmaskmovps		%xmm4, %xmm14, 0(%r10)
	addq			%r11, %r10
	cmpl		$ 10, %r13d
	jl			0f // end
	vmaskmovps		%xmm5, %xmm14, 0(%r10)
	addq			%r11, %r10
	cmpl		$ 11, %r13d
	jl			0f // end
	vmaskmovps		%xmm6, %xmm14, 0(%r10)
	addq			%r11, %r10
	cmpl		$ 12, %r13d
	jl			0f // end
	vmaskmovps		%xmm7, %xmm14, 0(%r10)
	addq			%r11, %r10
	cmpl		$ 13, %r13d
	jl			0f // end
	vextractf128	$ 1, %ymm4, %xmm15
	vmaskmovps		%xmm15, %xmm14, 0(%r10)
	addq			%r11, %r10
	cmpl		$ 14, %r13d
	jl			0f // end
	vextractf128	$ 1, %ymm5, %xmm15
	vmaskmovps		%xmm15, %xmm14, 0(%r10)
	addq			%r11, %r10
	cmpl		$ 15, %r13d
	jl			0f // end
	vextractf128	$ 1, %ymm6, %xmm15
	vmaskmovps		%xmm15, %xmm14, 0(%r10)
	addq			%r11, %r10
	cmpl		$ 15, %r13d
	je			0f // end
	vextractf128	$ 1, %ymm7, %xmm15
	vmaskmovps		%xmm15, %xmm14, 0(%r10)
//	addq			%r11, %r10

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4x16_vs_lib)
#endif





// common inner routine with file scope
//
// prefetch
//
// input arguments:
// r10  <- D
// r11  <- ldd
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_PREFETCH0_16X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_prefetch0_16x4_lib)
#endif

	leaq		0(%r11, %r11, 2), %r12
	prefetcht0	0(%r10)				// 0
	prefetcht0	63(%r10)			// 0
	prefetcht0	0(%r10, %r11)	// 1
	prefetcht0	63(%r10, %r11)	// 1
	prefetcht0	0(%r10, %r11, 2)	// 2
	prefetcht0	63(%r10, %r11, 2)	// 2
	prefetcht0	0(%r10, %r12)	// 3
	prefetcht0	63(%r10, %r12)	// 3

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_prefetch0_16x4_lib)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// r11  <- ldd
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_PREFETCH0_4X16_LIBC
#else
	.p2align 4,,15
	FUN_START(inner_prefetch0_4x16_libc)
#endif

	leaq		0(%r11, %r11, 2), %r12

	prefetcht0	0(%r10)
	prefetcht0	15(%r10)
	prefetcht0	0(%r10, %r11)
	prefetcht0	15(%r10, %r11)
	prefetcht0	0(%r10, %r11, 2)
	prefetcht0	15(%r10, %r11, 2)
	prefetcht0	0(%r10, %r12)
	prefetcht0	15(%r10, %r12)

	leaq		0(%r10, %r11, 4), %r10

	prefetcht0	0(%r10)
	prefetcht0	15(%r10)
	prefetcht0	0(%r10, %r11)
	prefetcht0	15(%r10, %r11)
	prefetcht0	0(%r10, %r11, 2)
	prefetcht0	15(%r10, %r11, 2)
	prefetcht0	0(%r10, %r12)
	prefetcht0	15(%r10, %r12)

	leaq		0(%r10, %r11, 4), %r10

	prefetcht0	0(%r10)
	prefetcht0	15(%r10)
	prefetcht0	0(%r10, %r11)
	prefetcht0	15(%r10, %r11)
	prefetcht0	0(%r10, %r11, 2)
	prefetcht0	15(%r10, %r11, 2)
	prefetcht0	0(%r10, %r12)
	prefetcht0	15(%r10, %r12)

	leaq		0(%r10, %r11, 4), %r10

	prefetcht0	0(%r10)
	prefetcht0	15(%r10)
	prefetcht0	0(%r10, %r11)
	prefetcht0	15(%r10, %r11)
	prefetcht0	0(%r10, %r11, 2)
	prefetcht0	15(%r10, %r11, 2)
	prefetcht0	0(%r10, %r12)
	prefetcht0	15(%r10, %r12)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_prefetch0_4x16_libc)
#endif





//                                   1      2              3          4        5          6        7             8          9        10         11
// void kernel_sgemm_nt_16x4_lib8ccc(int k, float *alpha, float *A, int sda, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_nt_16x4_lib8ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 2, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_16X4_LIB8C
#else
	CALL(inner_kernel_gemm_nt_16x4_lib8c)
#endif


	// prefetch D

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_16X4_LIB
#else
	CALL(inner_prefetch0_16x4_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 2, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_16X4_LIB
#else
	CALL(inner_scale_ab_16x4_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_16X4_LIB
#else
	CALL(inner_store_16x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_nt_16x4_lib8ccc)





//                                      1      2              3          4        5          6        7             8          9        10         11       12      13
// void kernel_sgemm_nt_16x4_vs_lib8ccc(int k, float *alpha, float *A, int sda, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_nt_16x4_vs_lib8ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 2, %r14d

	movq	ARG13, %r15  // n1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_16X1_LIB8C
#else
	CALL(inner_kernel_gemm_nt_16x1_lib8c)
#endif
	
	jmp		103f

100:

	movq	ARG13, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_16X2_LIB8C
#else
	CALL(inner_kernel_gemm_nt_16x2_lib8c)
#endif

	jmp		103f

101:

	movq	ARG13, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_16X3_LIB8C
#else
	CALL(inner_kernel_gemm_nt_16x3_lib8c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_16X4_LIB8C
#else
	CALL(inner_kernel_gemm_nt_16x4_lib8c)
#endif

103:


	// prefetch D
	// TODO prefetch vs !!!

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_16X4_LIB
#else
	CALL(inner_prefetch0_16x4_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 2, %r13d
	movq	ARG12, %r14 // m1
	movq	ARG13, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_16X4_VS_LIB
#else
	CALL(inner_scale_ab_16x4_vs_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 2, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_16X4_VS_LIB
#else
	CALL(inner_store_16x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_nt_16x4_vs_lib8ccc)





//                                   1      2             3         4        5         6            7         8        9         10
// void kernel_sgemm_nt_16x4_lib88cc(int k, float *alpha, float *A, int sda, float *B, float *beta, float *C, int ldc, float *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_nt_16x4_lib88cc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// prefetch C

	movq		ARG6, %r10 // beta
	vmovss		0(%r10), %xmm14
	vxorps		%xmm15, %xmm15, %xmm15 // 0.0
	vucomiss	%xmm15, %xmm14 // beta==0.0 ?
	je			100f // end

	movq	ARG7, %r10 // C
	movq	ARG8, %r11 // ldc
	sall	$ 2, %r11d // ldc*sizeof(float)

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_16X4_LIB
#else
	CALL(inner_prefetch0_16x4_lib)
#endif

100:


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG5, %r13  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_16X4_LIB8
#else
	CALL(inner_kernel_gemm_add_nt_16x4_lib8)
#endif


	// call inner scale

	movq	%rsi, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movl	ARG8, %r13d // ldc
	sall	$ 2, %r13d // ldc*sizeof(float)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_16X4_LIB
#else
	CALL(inner_scale_ab_16x4_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movl	ARG10, %r11d // ldd
	sall	$ 2, %r11d // ldd*sizeof(float)

#if MACRO_LEVEL>=1
	INNER_STORE_16X4_LIB
#else
	CALL(inner_store_16x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_nt_16x4_lib88cc)





//                                      1      2             3         4        5         6            7         8        9         10       12      13
// void kernel_sgemm_nt_16x4_vs_lib88cc(int k, float *alpha, float *A, int sda, float *B, float *beta, float *C, int ldc, float *D, int ldd, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_nt_16x4_vs_lib88cc)
	
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
	INNER_KERNEL_GEMM_ADD_NT_16X4_LIB8
#else
	CALL(inner_kernel_gemm_add_nt_16x4_lib8)
#endif


	// call inner scale

	movq	%rsi, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movl	ARG8, %r13d // ldc
	sall	$ 2, %r13d // ldc*sizeof(float)
	movq	ARG11, %r14 // km
	movq	ARG12, %r15 // kn

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_16X4_VS_LIB
#else
	CALL(inner_scale_ab_16x4_vs_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movl	ARG10, %r11d // ldd
	sall	$ 2, %r11d // ldd*sizeof(float)
	movq	ARG11, %r12 // km
	movq	ARG12, %r13 // kn

#if MACRO_LEVEL>=1
	INNER_STORE_16X4_VS_LIB
#else
	CALL(inner_store_16x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_nt_16x4_vs_lib88cc)





//                                 1      2              3          4        5          6        7             8          9        10         11
// void kernel_sgemm_nn_16x4_lib8ccc(int k, float *alpha, float *A, int sda, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd,);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_nn_16x4_lib8ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 2, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_16X4_LIB8C
#else
	CALL(inner_kernel_gemm_nn_16x4_lib8c)
#endif


	// prefetch D

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_16X4_LIB
#else
	CALL(inner_prefetch0_16x4_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 2, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_16X4_LIB
#else
	CALL(inner_scale_ab_16x4_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_16X4_LIB
#else
	CALL(inner_store_16x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_nn_16x4_lib8ccc)





//                                      1      2              3          4        5          6        7             8          9        10         11       12      13
// void kernel_sgemm_nn_16x4_vs_lib8ccc(int k, float *alpha, float *A, int sda, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_nn_16x4_vs_lib8ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 2, %r14d

	movq	ARG13, %r15  // n1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_16X1_LIB8C
#else
	CALL(inner_kernel_gemm_nn_16x1_lib8c)
#endif
	
	jmp		103f

100:

	movq	ARG13, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_16X2_LIB8C
#else
	CALL(inner_kernel_gemm_nn_16x2_lib8c)
#endif

	jmp		103f

101:

	movq	ARG13, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_16X3_LIB8C
#else
	CALL(inner_kernel_gemm_nn_16x3_lib8c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_16X4_LIB8C
#else
	CALL(inner_kernel_gemm_nn_16x4_lib8c)
#endif

103:


	// prefetch D
	// TODO prefetch vs !!!

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_16X4_LIB
#else
	CALL(inner_prefetch0_16x4_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 2, %r13d
	movq	ARG12, %r14 // m1
	movq	ARG13, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_16X4_VS_LIB
#else
	CALL(inner_scale_ab_16x4_vs_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 2, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_16X4_VS_LIB
#else
	CALL(inner_store_16x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_nn_16x4_vs_lib8ccc)





//                                   1      2             3         4         5        6        7            8         9        10        11
// void kernel_sgemm_nt_4x16_libc8cc(int k, float *alpha, float *A, int lda, float *B, int sdb, float *beta, float *C, int ldc, float *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_nt_4x16_libc8cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG5, %r11  // B
	movq	ARG6, %r12  // sdb
	sall	$ 5, %r12d
	movq	ARG3, %r13  // A
	movq	ARG4, %r14  // lda
	sall	$ 2, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_16X4_LIB8C
#else
	CALL(inner_kernel_gemm_nt_16x4_lib8c)
#endif


	// prefetch D

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_4X16_LIBC
#else
	CALL(inner_prefetch0_4x16_libc)
#endif


#if MACRO_LEVEL>=1
	INNER_TRAN_16X4_LIB
#else
	CALL(inner_tran_16x4_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 2, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X16_LIB
#else
	CALL(inner_scale_ab_4x16_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_4X16_LIB
#else
	CALL(inner_store_4x16_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_nt_4x16_libc8cc)





//                                      1      2             3         4         5        6        7            8         9        10        11       12      13
// void kernel_sgemm_nt_4x16_vs_libc8cc(int k, float *alpha, float *A, int lda, float *B, int sdb, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_nt_4x16_vs_libc8cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG5, %r11  // B
	movq	ARG6, %r12  // sdb
	sall	$ 5, %r12d
	movq	ARG3, %r13  // A
	movq	ARG4, %r14  // lda
	sall	$ 2, %r14d


	movq	ARG12, %r15  // m1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_16X1_LIB8C
#else
	CALL(inner_kernel_gemm_nt_16x1_lib8c)
#endif
	
	jmp		103f

100:

	movq	ARG12, %r15  // m1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_16X2_LIB8C
#else
	CALL(inner_kernel_gemm_nt_16x2_lib8c)
#endif

	jmp		103f

101:

	movq	ARG12, %r15  // m1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_16X3_LIB8C
#else
	CALL(inner_kernel_gemm_nt_16x3_lib8c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_16X4_LIB8C
#else
	CALL(inner_kernel_gemm_nt_16x4_lib8c)
#endif

103:


	// prefetch D
	// TODO prefetch vs !!!

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_4X16_LIBC
#else
	CALL(inner_prefetch0_4x16_libc)
#endif


#if MACRO_LEVEL>=1
	INNER_TRAN_16X4_LIB
#else
	CALL(inner_tran_16x4_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 2, %r13d
	movq	ARG12, %r14 // m1
	movq	ARG13, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X16_VS_LIB
#else
	CALL(inner_scale_ab_4x16_vs_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 2, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X16_VS_LIB
#else
	CALL(inner_store_4x16_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_nt_4x16_vs_libc8cc)





//                                   1      2             3         4         5        6        7            8         9        10        11
// void kernel_sgemm_tt_4x16_libc8cc(int k, float *alpha, float *A, int lda, float *B, int sdb, float *beta, float *C, int ldc, float *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_tt_4x16_libc8cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG5, %r11  // B
	movq	ARG6, %r12  // sdb
	sall	$ 5, %r12d
	movq	ARG3, %r13  // A
	movq	ARG4, %r14  // lda
	sall	$ 2, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_16X4_LIB8C
#else
	CALL(inner_kernel_gemm_nn_16x4_lib8c)
#endif


	// prefetch D

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_4X16_LIBC
#else
	CALL(inner_prefetch0_4x16_libc)
#endif


#if MACRO_LEVEL>=1
	INNER_TRAN_16X4_LIB
#else
	CALL(inner_tran_16x4_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 2, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X16_LIB
#else
	CALL(inner_scale_ab_4x16_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_4X16_LIB
#else
	CALL(inner_store_4x16_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_tt_4x16_libc8cc)





//                                      1      2             3         4         5        6        7            8         9        10        11       12      13
// void kernel_sgemm_tt_4x16_vs_libc8cc(int k, float *alpha, float *A, int lda, float *B, int sdb, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_tt_4x16_vs_libc8cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG5, %r11  // B
	movq	ARG6, %r12  // sdb
	sall	$ 5, %r12d
	movq	ARG3, %r13  // A
	movq	ARG4, %r14  // lda
	sall	$ 2, %r14d


	movq	ARG12, %r15  // m1
	cmpl	$ 1, %r15d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_16X1_LIB8C
#else
	CALL(inner_kernel_gemm_nn_16x1_lib8c)
#endif
	
	jmp		103f

100:

	movq	ARG12, %r15  // m1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_16X2_LIB8C
#else
	CALL(inner_kernel_gemm_nn_16x2_lib8c)
#endif

	jmp		103f

101:

	movq	ARG12, %r15  // m1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_16X3_LIB8C
#else
	CALL(inner_kernel_gemm_nn_16x3_lib8c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_16X4_LIB8C
#else
	CALL(inner_kernel_gemm_nn_16x4_lib8c)
#endif

103:


	// prefetch D
	// TODO prefetch vs !!!

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_4X16_LIBC
#else
	CALL(inner_prefetch0_4x16_libc)
#endif


#if MACRO_LEVEL>=1
	INNER_TRAN_16X4_LIB
#else
	CALL(inner_tran_16x4_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 2, %r13d
	movq	ARG12, %r14 // m1
	movq	ARG13, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X16_VS_LIB
#else
	CALL(inner_scale_ab_4x16_vs_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 2, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X16_VS_LIB
#else
	CALL(inner_store_4x16_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_tt_4x16_vs_libc8cc)






