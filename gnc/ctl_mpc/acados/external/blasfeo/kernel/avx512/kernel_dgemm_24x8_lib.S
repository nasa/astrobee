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



// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- sda
// r13   <- B
// r14   <- ldb
// zmm0  <- []
// ...
// zmm23 <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_24X8_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_24x8_lib8c)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	// preload

	prefetcht0		0(%r13)
	prefetcht0		63(%r13)
	prefetcht0		0(%r13, %r14)
	prefetcht0		63(%r13, %r14)
	prefetcht0		0(%r13, %r14, 2)
	prefetcht0		63(%r13, %r14, 2)
	leaq			0(%r14, %r14, 2), %r15
	prefetcht0		0(%r13, %r15)
	prefetcht0		63(%r13, %r15)

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

	// unroll 0
	prefetcht0		0(%r13, %r14, 4)
	prefetcht0		63(%r13, %r14, 4)
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	48(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
	vbroadcastsd	56(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm26, %zmm24, %zmm15
	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	// unroll 1
	prefetcht0		0(%r13, %r14, 4)
	prefetcht0		63(%r13, %r14, 4)
	vmovapd			1*64(%r11), %zmm25 // A
	vmovapd			1*64(%r11, %r12), %zmm26 // A
	vmovapd			1*64(%r11, %r12, 2), %zmm27 // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	48(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
	vbroadcastsd	56(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm26, %zmm24, %zmm15
	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	// unroll 2
	prefetcht0		0(%r13, %r14, 4)
	prefetcht0		63(%r13, %r14, 4)
	vmovapd			2*64(%r11), %zmm25 // A
	vmovapd			2*64(%r11, %r12), %zmm26 // A
	vmovapd			2*64(%r11, %r12, 2), %zmm27 // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	48(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
	vbroadcastsd	56(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm26, %zmm24, %zmm15
	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	// unroll 3
	prefetcht0		0(%r13, %r14, 4)
	prefetcht0		63(%r13, %r14, 4)
	vmovapd			3*64(%r11), %zmm25 // A
	vmovapd			3*64(%r11, %r12), %zmm26 // A
	vmovapd			3*64(%r11, %r12, 2), %zmm27 // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	48(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
	vbroadcastsd	56(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm26, %zmm24, %zmm15
	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 4*64, %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	48(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
	vbroadcastsd	56(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm26, %zmm24, %zmm15
	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vmovapd			1*64(%r11, %r12), %zmm26 // A
	vmovapd			1*64(%r11, %r12, 2), %zmm27 // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	48(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
	vbroadcastsd	56(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm26, %zmm24, %zmm15
	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vmovapd			2*64(%r11, %r12), %zmm26 // A
	vmovapd			2*64(%r11, %r12, 2), %zmm27 // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	48(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
	vbroadcastsd	56(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm26, %zmm24, %zmm15
	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vmovapd			3*64(%r11, %r12), %zmm26 // A
	vmovapd			3*64(%r11, %r12, 2), %zmm27 // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	48(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
	vbroadcastsd	56(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm26, %zmm24, %zmm15
	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 4*64, %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	48(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
	vbroadcastsd	56(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm26, %zmm24, %zmm15
	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	subl	$ 1, %r10d
	addq	$ 1*64, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_24x8_lib8c)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- sda
// r13   <- B
// r14   <- ldb
// k1    <- m_mask
// zmm0  <- []
// ...
// zmm23 <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_3VX8_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_3vx8_lib8c)
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
//	prefetcht0		0(%r13, %r14, 8)
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	48(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
	vbroadcastsd	56(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm26, %zmm24, %zmm15
	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	// unroll 1
//	prefetcht0		0(%r13, %r14, 8)
	vmovapd			1*64(%r11), %zmm25 // A
	vmovapd			1*64(%r11, %r12), %zmm26 // A
	vmovapd			1*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	48(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
	vbroadcastsd	56(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm26, %zmm24, %zmm15
	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	// unroll 2
//	prefetcht0		0(%r13, %r14, 8)
	vmovapd			2*64(%r11), %zmm25 // A
	vmovapd			2*64(%r11, %r12), %zmm26 // A
	vmovapd			2*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	48(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
	vbroadcastsd	56(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm26, %zmm24, %zmm15
	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	// unroll 3
//	prefetcht0		0(%r13, %r14, 8)
	vmovapd			3*64(%r11), %zmm25 // A
	vmovapd			3*64(%r11, %r12), %zmm26 // A
	vmovapd			3*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	48(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
	vbroadcastsd	56(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm26, %zmm24, %zmm15
	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 4*64, %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	48(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
	vbroadcastsd	56(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm26, %zmm24, %zmm15
	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vmovapd			1*64(%r11, %r12), %zmm26 // A
	vmovapd			1*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	48(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
	vbroadcastsd	56(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm26, %zmm24, %zmm15
	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vmovapd			2*64(%r11, %r12), %zmm26 // A
	vmovapd			2*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	48(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
	vbroadcastsd	56(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm26, %zmm24, %zmm15
	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vmovapd			3*64(%r11, %r12), %zmm26 // A
	vmovapd			3*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	48(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
	vbroadcastsd	56(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm26, %zmm24, %zmm15
	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 4*64, %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	48(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
	vbroadcastsd	56(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm26, %zmm24, %zmm15
	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	subl	$ 1, %r10d
	addq	$ 1*64, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_3vx8_lib8c)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- sda
// r13   <- B
// r14   <- ldb
// k1    <- m_mask
// zmm0  <- []
// ...
// zmm23 <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_3VX7_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_3vx7_lib8c)
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
//	prefetcht0		0(%r13, %r14, 8)
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	48(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	56(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	// unroll 1
//	prefetcht0		0(%r13, %r14, 8)
	vmovapd			1*64(%r11), %zmm25 // A
	vmovapd			1*64(%r11, %r12), %zmm26 // A
	vmovapd			1*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	48(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	56(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	// unroll 2
//	prefetcht0		0(%r13, %r14, 8)
	vmovapd			2*64(%r11), %zmm25 // A
	vmovapd			2*64(%r11, %r12), %zmm26 // A
	vmovapd			2*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	48(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	56(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	// unroll 3
//	prefetcht0		0(%r13, %r14, 8)
	vmovapd			3*64(%r11), %zmm25 // A
	vmovapd			3*64(%r11, %r12), %zmm26 // A
	vmovapd			3*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	48(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	56(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 4*64, %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	48(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	56(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vmovapd			1*64(%r11, %r12), %zmm26 // A
	vmovapd			1*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	48(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	56(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vmovapd			2*64(%r11, %r12), %zmm26 // A
	vmovapd			2*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	48(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	56(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vmovapd			3*64(%r11, %r12), %zmm26 // A
	vmovapd			3*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	48(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	56(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 4*64, %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	48(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	56(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	subl	$ 1, %r10d
	addq	$ 1*64, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_3vx7_lib8c)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- sda
// r13   <- B
// r14   <- ldb
// k1    <- m_mask
// zmm0  <- []
// ...
// zmm23 <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_3VX6_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_3vx6_lib8c)
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
//	prefetcht0		0(%r13, %r14, 8)
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
//	vbroadcastsd	48(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm26, %zmm24, %zmm14
//	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	56(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	// unroll 1
//	prefetcht0		0(%r13, %r14, 8)
	vmovapd			1*64(%r11), %zmm25 // A
	vmovapd			1*64(%r11, %r12), %zmm26 // A
	vmovapd			1*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
//	vbroadcastsd	48(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm26, %zmm24, %zmm14
//	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	56(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	// unroll 2
//	prefetcht0		0(%r13, %r14, 8)
	vmovapd			2*64(%r11), %zmm25 // A
	vmovapd			2*64(%r11, %r12), %zmm26 // A
	vmovapd			2*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
//	vbroadcastsd	48(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm26, %zmm24, %zmm14
//	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	56(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	// unroll 3
//	prefetcht0		0(%r13, %r14, 8)
	vmovapd			3*64(%r11), %zmm25 // A
	vmovapd			3*64(%r11, %r12), %zmm26 // A
	vmovapd			3*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
//	vbroadcastsd	48(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm26, %zmm24, %zmm14
//	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	56(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 4*64, %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
//	vbroadcastsd	48(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm26, %zmm24, %zmm14
//	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	56(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vmovapd			1*64(%r11, %r12), %zmm26 // A
	vmovapd			1*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
//	vbroadcastsd	48(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm26, %zmm24, %zmm14
//	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	56(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vmovapd			2*64(%r11, %r12), %zmm26 // A
	vmovapd			2*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
//	vbroadcastsd	48(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm26, %zmm24, %zmm14
//	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	56(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vmovapd			3*64(%r11, %r12), %zmm26 // A
	vmovapd			3*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
//	vbroadcastsd	48(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm26, %zmm24, %zmm14
//	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	56(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 4*64, %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	40(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
//	vbroadcastsd	48(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm26, %zmm24, %zmm14
//	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	56(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	subl	$ 1, %r10d
	addq	$ 1*64, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_3vx6_lib8c)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- sda
// r13   <- B
// r14   <- ldb
// k1    <- m_mask
// zmm0  <- []
// ...
// zmm23 <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_3VX5_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_3vx5_lib8c)
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
//	prefetcht0		0(%r13, %r14, 8)
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
//	vbroadcastsd	40(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vfmadd231pd		%zmm26, %zmm24, %zmm13
//	vfmadd231pd		%zmm27, %zmm24, %zmm21
//	vbroadcastsd	48(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm26, %zmm24, %zmm14
//	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	56(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	// unroll 1
//	prefetcht0		0(%r13, %r14, 8)
	vmovapd			1*64(%r11), %zmm25 // A
	vmovapd			1*64(%r11, %r12), %zmm26 // A
	vmovapd			1*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
//	vbroadcastsd	40(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vfmadd231pd		%zmm26, %zmm24, %zmm13
//	vfmadd231pd		%zmm27, %zmm24, %zmm21
//	vbroadcastsd	48(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm26, %zmm24, %zmm14
//	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	56(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	// unroll 2
//	prefetcht0		0(%r13, %r14, 8)
	vmovapd			2*64(%r11), %zmm25 // A
	vmovapd			2*64(%r11, %r12), %zmm26 // A
	vmovapd			2*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
//	vbroadcastsd	40(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vfmadd231pd		%zmm26, %zmm24, %zmm13
//	vfmadd231pd		%zmm27, %zmm24, %zmm21
//	vbroadcastsd	48(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm26, %zmm24, %zmm14
//	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	56(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	// unroll 3
//	prefetcht0		0(%r13, %r14, 8)
	vmovapd			3*64(%r11), %zmm25 // A
	vmovapd			3*64(%r11, %r12), %zmm26 // A
	vmovapd			3*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
//	vbroadcastsd	40(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vfmadd231pd		%zmm26, %zmm24, %zmm13
//	vfmadd231pd		%zmm27, %zmm24, %zmm21
//	vbroadcastsd	48(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm26, %zmm24, %zmm14
//	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	56(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 4*64, %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
//	vbroadcastsd	40(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vfmadd231pd		%zmm26, %zmm24, %zmm13
//	vfmadd231pd		%zmm27, %zmm24, %zmm21
//	vbroadcastsd	48(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm26, %zmm24, %zmm14
//	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	56(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vmovapd			1*64(%r11, %r12), %zmm26 // A
	vmovapd			1*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
//	vbroadcastsd	40(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vfmadd231pd		%zmm26, %zmm24, %zmm13
//	vfmadd231pd		%zmm27, %zmm24, %zmm21
//	vbroadcastsd	48(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm26, %zmm24, %zmm14
//	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	56(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vmovapd			2*64(%r11, %r12), %zmm26 // A
	vmovapd			2*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
//	vbroadcastsd	40(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vfmadd231pd		%zmm26, %zmm24, %zmm13
//	vfmadd231pd		%zmm27, %zmm24, %zmm21
//	vbroadcastsd	48(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm26, %zmm24, %zmm14
//	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	56(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vmovapd			3*64(%r11, %r12), %zmm26 // A
	vmovapd			3*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
//	vbroadcastsd	40(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vfmadd231pd		%zmm26, %zmm24, %zmm13
//	vfmadd231pd		%zmm27, %zmm24, %zmm21
//	vbroadcastsd	48(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm26, %zmm24, %zmm14
//	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	56(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 4*64, %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	32(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
//	vbroadcastsd	40(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vfmadd231pd		%zmm26, %zmm24, %zmm13
//	vfmadd231pd		%zmm27, %zmm24, %zmm21
//	vbroadcastsd	48(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm26, %zmm24, %zmm14
//	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	56(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23
	addq	%r14, %r13

	subl	$ 1, %r10d
	addq	$ 1*64, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_3vx5_lib8c)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- sda
// r13   <- B
// r14   <- ldb
// k1    <- m_mask
// zmm0  <- []
// ...
// zmm23 <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_3VX4_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_3vx4_lib8c)
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
//	prefetcht0		0(%r13, %r14, 8)
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	addq	%r14, %r13

	// unroll 1
//	prefetcht0		0(%r13, %r14, 8)
	vmovapd			1*64(%r11), %zmm25 // A
	vmovapd			1*64(%r11, %r12), %zmm26 // A
	vmovapd			1*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	addq	%r14, %r13

	// unroll 2
//	prefetcht0		0(%r13, %r14, 8)
	vmovapd			2*64(%r11), %zmm25 // A
	vmovapd			2*64(%r11, %r12), %zmm26 // A
	vmovapd			2*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	addq	%r14, %r13

	// unroll 3
//	prefetcht0		0(%r13, %r14, 8)
	vmovapd			3*64(%r11), %zmm25 // A
	vmovapd			3*64(%r11, %r12), %zmm26 // A
	vmovapd			3*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 4*64, %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	addq	%r14, %r13

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vmovapd			1*64(%r11, %r12), %zmm26 // A
	vmovapd			1*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	addq	%r14, %r13

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vmovapd			2*64(%r11, %r12), %zmm26 // A
	vmovapd			2*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	addq	%r14, %r13

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vmovapd			3*64(%r11, %r12), %zmm26 // A
	vmovapd			3*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 4*64, %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	24(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	addq	%r14, %r13

	subl	$ 1, %r10d
	addq	$ 1*64, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_3vx4_lib8c)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- sda
// r13   <- B
// r14   <- ldb
// k1    <- m_mask
// zmm0  <- []
// ...
// zmm23 <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_3VX3_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_3vx3_lib8c)
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
//	prefetcht0		0(%r13, %r14, 8)
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	24(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19
	addq	%r14, %r13

	// unroll 1
//	prefetcht0		0(%r13, %r14, 8)
	vmovapd			1*64(%r11), %zmm25 // A
	vmovapd			1*64(%r11, %r12), %zmm26 // A
	vmovapd			1*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	24(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19
	addq	%r14, %r13

	// unroll 2
//	prefetcht0		0(%r13, %r14, 8)
	vmovapd			2*64(%r11), %zmm25 // A
	vmovapd			2*64(%r11, %r12), %zmm26 // A
	vmovapd			2*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	24(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19
	addq	%r14, %r13

	// unroll 3
//	prefetcht0		0(%r13, %r14, 8)
	vmovapd			3*64(%r11), %zmm25 // A
	vmovapd			3*64(%r11, %r12), %zmm26 // A
	vmovapd			3*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	24(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 4*64, %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	24(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19
	addq	%r14, %r13

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vmovapd			1*64(%r11, %r12), %zmm26 // A
	vmovapd			1*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	24(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19
	addq	%r14, %r13

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vmovapd			2*64(%r11, %r12), %zmm26 // A
	vmovapd			2*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	24(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19
	addq	%r14, %r13

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vmovapd			3*64(%r11, %r12), %zmm26 // A
	vmovapd			3*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	24(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 4*64, %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	16(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	24(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19
	addq	%r14, %r13

	subl	$ 1, %r10d
	addq	$ 1*64, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_3vx3_lib8c)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- sda
// r13   <- B
// r14   <- ldb
// k1    <- m_mask
// zmm0  <- []
// ...
// zmm23 <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_3VX2_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_3vx2_lib8c)
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
//	prefetcht0		0(%r13, %r14, 8)
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
//	vbroadcastsd	16(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm26, %zmm24, %zmm10
//	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	24(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19
	addq	%r14, %r13

	// unroll 1
//	prefetcht0		0(%r13, %r14, 8)
	vmovapd			1*64(%r11), %zmm25 // A
	vmovapd			1*64(%r11, %r12), %zmm26 // A
	vmovapd			1*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
//	vbroadcastsd	16(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm26, %zmm24, %zmm10
//	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	24(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19
	addq	%r14, %r13

	// unroll 2
//	prefetcht0		0(%r13, %r14, 8)
	vmovapd			2*64(%r11), %zmm25 // A
	vmovapd			2*64(%r11, %r12), %zmm26 // A
	vmovapd			2*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
//	vbroadcastsd	16(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm26, %zmm24, %zmm10
//	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	24(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19
	addq	%r14, %r13

	// unroll 3
//	prefetcht0		0(%r13, %r14, 8)
	vmovapd			3*64(%r11), %zmm25 // A
	vmovapd			3*64(%r11, %r12), %zmm26 // A
	vmovapd			3*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
//	vbroadcastsd	16(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm26, %zmm24, %zmm10
//	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	24(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 4*64, %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
//	vbroadcastsd	16(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm26, %zmm24, %zmm10
//	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	24(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19
	addq	%r14, %r13

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vmovapd			1*64(%r11, %r12), %zmm26 // A
	vmovapd			1*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
//	vbroadcastsd	16(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm26, %zmm24, %zmm10
//	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	24(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19
	addq	%r14, %r13

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vmovapd			2*64(%r11, %r12), %zmm26 // A
	vmovapd			2*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
//	vbroadcastsd	16(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm26, %zmm24, %zmm10
//	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	24(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19
	addq	%r14, %r13

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vmovapd			3*64(%r11, %r12), %zmm26 // A
	vmovapd			3*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
//	vbroadcastsd	16(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm26, %zmm24, %zmm10
//	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	24(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 4*64, %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
//	vbroadcastsd	16(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm26, %zmm24, %zmm10
//	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	24(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19
	addq	%r14, %r13

	subl	$ 1, %r10d
	addq	$ 1*64, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_3vx2_lib8c)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- sda
// r13   <- B
// r14   <- ldb
// k1    <- m_mask
// zmm0  <- []
// ...
// zmm23 <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_3VX1_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_3vx1_lib8c)
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
//	prefetcht0		0(%r13, %r14, 8)
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
//	vbroadcastsd	8(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vfmadd231pd		%zmm26, %zmm24, %zmm9
//	vfmadd231pd		%zmm27, %zmm24, %zmm17
//	vbroadcastsd	16(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm26, %zmm24, %zmm10
//	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	24(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19
	addq	%r14, %r13

	// unroll 1
//	prefetcht0		0(%r13, %r14, 8)
	vmovapd			1*64(%r11), %zmm25 // A
	vmovapd			1*64(%r11, %r12), %zmm26 // A
	vmovapd			1*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
//	vbroadcastsd	8(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vfmadd231pd		%zmm26, %zmm24, %zmm9
//	vfmadd231pd		%zmm27, %zmm24, %zmm17
//	vbroadcastsd	16(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm26, %zmm24, %zmm10
//	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	24(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19
	addq	%r14, %r13

	// unroll 2
//	prefetcht0		0(%r13, %r14, 8)
	vmovapd			2*64(%r11), %zmm25 // A
	vmovapd			2*64(%r11, %r12), %zmm26 // A
	vmovapd			2*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
//	vbroadcastsd	8(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vfmadd231pd		%zmm26, %zmm24, %zmm9
//	vfmadd231pd		%zmm27, %zmm24, %zmm17
//	vbroadcastsd	16(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm26, %zmm24, %zmm10
//	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	24(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19
	addq	%r14, %r13

	// unroll 3
//	prefetcht0		0(%r13, %r14, 8)
	vmovapd			3*64(%r11), %zmm25 // A
	vmovapd			3*64(%r11, %r12), %zmm26 // A
	vmovapd			3*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
//	vbroadcastsd	8(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vfmadd231pd		%zmm26, %zmm24, %zmm9
//	vfmadd231pd		%zmm27, %zmm24, %zmm17
//	vbroadcastsd	16(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm26, %zmm24, %zmm10
//	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	24(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 4*64, %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
//	vbroadcastsd	8(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vfmadd231pd		%zmm26, %zmm24, %zmm9
//	vfmadd231pd		%zmm27, %zmm24, %zmm17
//	vbroadcastsd	16(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm26, %zmm24, %zmm10
//	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	24(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19
	addq	%r14, %r13

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vmovapd			1*64(%r11, %r12), %zmm26 // A
	vmovapd			1*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
//	vbroadcastsd	8(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vfmadd231pd		%zmm26, %zmm24, %zmm9
//	vfmadd231pd		%zmm27, %zmm24, %zmm17
//	vbroadcastsd	16(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm26, %zmm24, %zmm10
//	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	24(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19
	addq	%r14, %r13

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vmovapd			2*64(%r11, %r12), %zmm26 // A
	vmovapd			2*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
//	vbroadcastsd	8(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vfmadd231pd		%zmm26, %zmm24, %zmm9
//	vfmadd231pd		%zmm27, %zmm24, %zmm17
//	vbroadcastsd	16(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm26, %zmm24, %zmm10
//	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	24(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19
	addq	%r14, %r13

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vmovapd			3*64(%r11, %r12), %zmm26 // A
	vmovapd			3*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
//	vbroadcastsd	8(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vfmadd231pd		%zmm26, %zmm24, %zmm9
//	vfmadd231pd		%zmm27, %zmm24, %zmm17
//	vbroadcastsd	16(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm26, %zmm24, %zmm10
//	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	24(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19
	addq	%r14, %r13

	subl	$ 4, %r10d
	addq	$ 4*64, %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
//	vbroadcastsd	8(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vfmadd231pd		%zmm26, %zmm24, %zmm9
//	vfmadd231pd		%zmm27, %zmm24, %zmm17
//	vbroadcastsd	16(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm26, %zmm24, %zmm10
//	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	24(%r13), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19
	addq	%r14, %r13

	subl	$ 1, %r10d
	addq	$ 1*64, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_3vx1_lib8c)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- 4*sda*sizeof(double)
// r13   <- B
// r14   <- ldb
// r15   <- m1
// rax   <- n1
// zmm0  <- []
// ...
// zmm23 <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+8*k*sizeof(double)
// r12   <- 4*sda*sizeof(double)
// r13   <- B+8*k*sizeof(double)
// r14   <- ldb
// r15   <- m1
// rax   <- n1
// zmm0  <- []
// ...
// zmm23 <- []

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NT_24X8_VS_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nt_24x8_vs_lib8c)
#endif

	// compute mask for rows
	vcvtsi2sd	%r15d, %xmm25, %xmm25
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC01(%rip), %zmm24
#elif defined(OS_MAC)
	vmovupd		LC01(%rip), %zmm24
#endif
	vbroadcastsd	%xmm25, %zmm25
	vsubpd		%zmm25, %zmm24, %zmm25
	vpmovq2m	%zmm25, %k1

	cmpl	$ 1, %eax
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_3VX1_LIB8C
#else
	CALL(inner_kernel_dgemm_nt_3vx1_lib8c)
#endif
	
	jmp		107f

100:

	cmpl	$ 2, %eax
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_3VX2_LIB8C
#else
	CALL(inner_kernel_dgemm_nt_3vx2_lib8c)
#endif
	
	jmp		107f

101:

	cmpl	$ 3, %eax
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_3VX3_LIB8C
#else
	CALL(inner_kernel_dgemm_nt_3vx3_lib8c)
#endif
	
	jmp		107f

102:

	cmpl	$ 4, %eax
	jg		103f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_3VX4_LIB8C
#else
	CALL(inner_kernel_dgemm_nt_3vx4_lib8c)
#endif
	
	jmp		107f

103:

	cmpl	$ 5, %eax
	jg		104f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_3VX5_LIB8C
#else
	CALL(inner_kernel_dgemm_nt_3vx5_lib8c)
#endif
	
	jmp		107f

104:

	cmpl	$ 6, %eax
	jg		105f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_3VX6_LIB8C
#else
	CALL(inner_kernel_dgemm_nt_3vx6_lib8c)
#endif
	
	jmp		107f

105:

	cmpl	$ 7, %eax
	jg		106f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_3VX7_LIB8C
#else
	CALL(inner_kernel_dgemm_nt_3vx7_lib8c)
#endif
	
	jmp		107f

106:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_3VX8_LIB8C
#else
	CALL(inner_kernel_dgemm_nt_3vx8_lib8c)
#endif

107:

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nt_24x8_vs_lib8c)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- sda
// r13   <- B
// r14   <- ldb
// zmm0  <- []
// ...
// zmm23 <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_24X8_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_24x8_lib8c)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	leaq	0(%r13, %r14, 4), %r15
	leaq	0(%r14, %r14, 2), %rax

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0		32(%r13)
//	...

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 // A
	vbroadcastsd	0*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	0*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	0*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	0*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	0*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	0*8(%r15, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	0*8(%r15, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
	vbroadcastsd	0*8(%r15, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm26, %zmm24, %zmm15
	vfmadd231pd		%zmm27, %zmm24, %zmm23

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vmovapd			1*64(%r11, %r12), %zmm26 // A
	vmovapd			1*64(%r11, %r12, 2), %zmm27 // A
	vbroadcastsd	1*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	1*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	1*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	1*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	1*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	1*8(%r15, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	1*8(%r15, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
	vbroadcastsd	1*8(%r15, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm26, %zmm24, %zmm15
	vfmadd231pd		%zmm27, %zmm24, %zmm23

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vmovapd			2*64(%r11, %r12), %zmm26 // A
	vmovapd			2*64(%r11, %r12, 2), %zmm27 // A
	vbroadcastsd	2*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	2*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	2*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	2*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	2*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	2*8(%r15, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	2*8(%r15, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
	vbroadcastsd	2*8(%r15, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm26, %zmm24, %zmm15
	vfmadd231pd		%zmm27, %zmm24, %zmm23

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vmovapd			3*64(%r11, %r12), %zmm26 // A
	vmovapd			3*64(%r11, %r12, 2), %zmm27 // A
	vbroadcastsd	3*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	3*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	3*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	3*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	3*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	3*8(%r15, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	3*8(%r15, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
	vbroadcastsd	3*8(%r15, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm26, %zmm24, %zmm15
	vfmadd231pd		%zmm27, %zmm24, %zmm23

	subl	$ 4, %r10d
	addq	$ 4*64, %r11
	addq	$ 4*8, %r13
	addq	$ 4*8, %r15

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 // A
	vbroadcastsd	0*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	0*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	0*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	0*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	0*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	0*8(%r15, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	0*8(%r15, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
	vbroadcastsd	0*8(%r15, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm26, %zmm24, %zmm15
	vfmadd231pd		%zmm27, %zmm24, %zmm23

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vmovapd			1*64(%r11, %r12), %zmm26 // A
	vmovapd			1*64(%r11, %r12, 2), %zmm27 // A
	vbroadcastsd	1*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	1*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	1*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	1*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	1*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	1*8(%r15, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	1*8(%r15, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
	vbroadcastsd	1*8(%r15, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm26, %zmm24, %zmm15
	vfmadd231pd		%zmm27, %zmm24, %zmm23

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vmovapd			2*64(%r11, %r12), %zmm26 // A
	vmovapd			2*64(%r11, %r12, 2), %zmm27 // A
	vbroadcastsd	2*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	2*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	2*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	2*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	2*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	2*8(%r15, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	2*8(%r15, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
	vbroadcastsd	2*8(%r15, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm26, %zmm24, %zmm15
	vfmadd231pd		%zmm27, %zmm24, %zmm23

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vmovapd			3*64(%r11, %r12), %zmm26 // A
	vmovapd			3*64(%r11, %r12, 2), %zmm27 // A
	vbroadcastsd	3*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	3*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	3*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	3*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	3*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	3*8(%r15, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	3*8(%r15, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
	vbroadcastsd	3*8(%r15, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm26, %zmm24, %zmm15
	vfmadd231pd		%zmm27, %zmm24, %zmm23

	subl	$ 4, %r10d
	addq	$ 4*64, %r11
	addq	$ 4*8, %r13
	addq	$ 4*8, %r15

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 // A
	vbroadcastsd	0*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	0*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	0*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	0*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	0*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	0*8(%r15, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	0*8(%r15, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
	vbroadcastsd	0*8(%r15, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm26, %zmm24, %zmm15
	vfmadd231pd		%zmm27, %zmm24, %zmm23

	subl	$ 1, %r10d
	addq	$ 1*64, %r11
	addq	$ 1*8, %r13
	addq	$ 1*8, %r15

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_24x8_lib8c)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- sda
// r13   <- B
// r14   <- ldb
// k1    <- m_mask
// zmm0  <- []
// ...
// zmm23 <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_3VX8_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_3vx8_lib8c)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	leaq	0(%r13, %r14, 4), %r15
	leaq	0(%r14, %r14, 2), %rax

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0		32(%r13)
//	...

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	0*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	0*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	0*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	0*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	0*8(%r15, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	0*8(%r15, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
	vbroadcastsd	0*8(%r15, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm26, %zmm24, %zmm15
	vfmadd231pd		%zmm27, %zmm24, %zmm23

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vmovapd			1*64(%r11, %r12), %zmm26 // A
	vmovapd			1*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	1*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	1*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	1*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	1*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	1*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	1*8(%r15, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	1*8(%r15, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
	vbroadcastsd	1*8(%r15, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm26, %zmm24, %zmm15
	vfmadd231pd		%zmm27, %zmm24, %zmm23

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vmovapd			2*64(%r11, %r12), %zmm26 // A
	vmovapd			2*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	2*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	2*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	2*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	2*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	2*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	2*8(%r15, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	2*8(%r15, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
	vbroadcastsd	2*8(%r15, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm26, %zmm24, %zmm15
	vfmadd231pd		%zmm27, %zmm24, %zmm23

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vmovapd			3*64(%r11, %r12), %zmm26 // A
	vmovapd			3*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	3*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	3*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	3*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	3*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	3*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	3*8(%r15, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	3*8(%r15, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
	vbroadcastsd	3*8(%r15, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm26, %zmm24, %zmm15
	vfmadd231pd		%zmm27, %zmm24, %zmm23

	subl	$ 4, %r10d
	addq	$ 4*64, %r11
	addq	$ 4*8, %r13
	addq	$ 4*8, %r15

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	0*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	0*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	0*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	0*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	0*8(%r15, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	0*8(%r15, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
	vbroadcastsd	0*8(%r15, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm26, %zmm24, %zmm15
	vfmadd231pd		%zmm27, %zmm24, %zmm23

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vmovapd			1*64(%r11, %r12), %zmm26 // A
	vmovapd			1*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	1*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	1*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	1*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	1*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	1*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	1*8(%r15, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	1*8(%r15, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
	vbroadcastsd	1*8(%r15, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm26, %zmm24, %zmm15
	vfmadd231pd		%zmm27, %zmm24, %zmm23

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vmovapd			2*64(%r11, %r12), %zmm26 // A
	vmovapd			2*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	2*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	2*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	2*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	2*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	2*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	2*8(%r15, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	2*8(%r15, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
	vbroadcastsd	2*8(%r15, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm26, %zmm24, %zmm15
	vfmadd231pd		%zmm27, %zmm24, %zmm23

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vmovapd			3*64(%r11, %r12), %zmm26 // A
	vmovapd			3*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	3*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	3*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	3*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	3*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	3*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	3*8(%r15, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	3*8(%r15, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
	vbroadcastsd	3*8(%r15, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm26, %zmm24, %zmm15
	vfmadd231pd		%zmm27, %zmm24, %zmm23

	subl	$ 4, %r10d
	addq	$ 4*64, %r11
	addq	$ 4*8, %r13
	addq	$ 4*8, %r15

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	0*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	0*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	0*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	0*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	0*8(%r15, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	0*8(%r15, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
	vbroadcastsd	0*8(%r15, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm7
	vfmadd231pd		%zmm26, %zmm24, %zmm15
	vfmadd231pd		%zmm27, %zmm24, %zmm23

	subl	$ 1, %r10d
	addq	$ 1*64, %r11
	addq	$ 1*8, %r13
	addq	$ 1*8, %r15

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_3vx8_lib8c)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- sda
// r13   <- B
// r14   <- ldb
// k1    <- m_mask
// zmm0  <- []
// ...
// zmm23 <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_3VX7_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_3vx7_lib8c)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	leaq	0(%r13, %r14, 4), %r15
	leaq	0(%r14, %r14, 2), %rax

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0		32(%r13)
//	...

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	0*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	0*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	0*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	0*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	0*8(%r15, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	0*8(%r15, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	0*8(%r15, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vmovapd			1*64(%r11, %r12), %zmm26 // A
	vmovapd			1*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	1*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	1*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	1*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	1*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	1*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	1*8(%r15, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	1*8(%r15, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	1*8(%r15, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vmovapd			2*64(%r11, %r12), %zmm26 // A
	vmovapd			2*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	2*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	2*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	2*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	2*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	2*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	2*8(%r15, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	2*8(%r15, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	2*8(%r15, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vmovapd			3*64(%r11, %r12), %zmm26 // A
	vmovapd			3*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	3*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	3*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	3*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	3*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	3*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	3*8(%r15, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	3*8(%r15, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	3*8(%r15, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23

	subl	$ 4, %r10d
	addq	$ 4*64, %r11
	addq	$ 4*8, %r13
	addq	$ 4*8, %r15

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	0*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	0*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	0*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	0*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	0*8(%r15, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	0*8(%r15, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	0*8(%r15, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vmovapd			1*64(%r11, %r12), %zmm26 // A
	vmovapd			1*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	1*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	1*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	1*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	1*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	1*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	1*8(%r15, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	1*8(%r15, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	1*8(%r15, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vmovapd			2*64(%r11, %r12), %zmm26 // A
	vmovapd			2*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	2*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	2*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	2*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	2*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	2*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	2*8(%r15, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	2*8(%r15, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	2*8(%r15, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vmovapd			3*64(%r11, %r12), %zmm26 // A
	vmovapd			3*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	3*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	3*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	3*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	3*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	3*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	3*8(%r15, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	3*8(%r15, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	3*8(%r15, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23

	subl	$ 4, %r10d
	addq	$ 4*64, %r11
	addq	$ 4*8, %r13
	addq	$ 4*8, %r15

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	0*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	0*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	0*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	0*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	0*8(%r15, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
	vbroadcastsd	0*8(%r15, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm6
	vfmadd231pd		%zmm26, %zmm24, %zmm14
	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	0*8(%r15, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23

	subl	$ 1, %r10d
	addq	$ 1*64, %r11
	addq	$ 1*8, %r13
	addq	$ 1*8, %r15

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_3vx7_lib8c)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- sda
// r13   <- B
// r14   <- ldb
// k1    <- m_mask
// zmm0  <- []
// ...
// zmm23 <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_3VX6_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_3vx6_lib8c)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	leaq	0(%r13, %r14, 4), %r15
	leaq	0(%r14, %r14, 2), %rax

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0		32(%r13)
//	...

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	0*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	0*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	0*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	0*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	0*8(%r15, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
//	vbroadcastsd	0*8(%r15, %r14, 2), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm26, %zmm24, %zmm14
//	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	0*8(%r15, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vmovapd			1*64(%r11, %r12), %zmm26 // A
	vmovapd			1*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	1*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	1*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	1*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	1*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	1*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	1*8(%r15, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
//	vbroadcastsd	1*8(%r15, %r14, 2), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm26, %zmm24, %zmm14
//	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	1*8(%r15, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vmovapd			2*64(%r11, %r12), %zmm26 // A
	vmovapd			2*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	2*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	2*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	2*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	2*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	2*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	2*8(%r15, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
//	vbroadcastsd	2*8(%r15, %r14, 2), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm26, %zmm24, %zmm14
//	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	2*8(%r15, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vmovapd			3*64(%r11, %r12), %zmm26 // A
	vmovapd			3*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	3*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	3*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	3*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	3*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	3*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	3*8(%r15, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
//	vbroadcastsd	3*8(%r15, %r14, 2), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm26, %zmm24, %zmm14
//	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	3*8(%r15, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23

	subl	$ 4, %r10d
	addq	$ 4*64, %r11
	addq	$ 4*8, %r13
	addq	$ 4*8, %r15

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	0*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	0*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	0*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	0*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	0*8(%r15, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
//	vbroadcastsd	0*8(%r15, %r14, 2), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm26, %zmm24, %zmm14
//	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	0*8(%r15, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vmovapd			1*64(%r11, %r12), %zmm26 // A
	vmovapd			1*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	1*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	1*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	1*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	1*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	1*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	1*8(%r15, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
//	vbroadcastsd	1*8(%r15, %r14, 2), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm26, %zmm24, %zmm14
//	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	1*8(%r15, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vmovapd			2*64(%r11, %r12), %zmm26 // A
	vmovapd			2*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	2*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	2*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	2*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	2*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	2*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	2*8(%r15, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
//	vbroadcastsd	2*8(%r15, %r14, 2), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm26, %zmm24, %zmm14
//	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	2*8(%r15, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vmovapd			3*64(%r11, %r12), %zmm26 // A
	vmovapd			3*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	3*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	3*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	3*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	3*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	3*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	3*8(%r15, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
//	vbroadcastsd	3*8(%r15, %r14, 2), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm26, %zmm24, %zmm14
//	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	3*8(%r15, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23

	subl	$ 4, %r10d
	addq	$ 4*64, %r11
	addq	$ 4*8, %r13
	addq	$ 4*8, %r15

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	0*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	0*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	0*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	0*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
	vbroadcastsd	0*8(%r15, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm5
	vfmadd231pd		%zmm26, %zmm24, %zmm13
	vfmadd231pd		%zmm27, %zmm24, %zmm21
//	vbroadcastsd	0*8(%r15, %r14, 2), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm26, %zmm24, %zmm14
//	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	0*8(%r15, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23

	subl	$ 1, %r10d
	addq	$ 1*64, %r11
	addq	$ 1*8, %r13
	addq	$ 1*8, %r15

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_3vx6_lib8c)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- sda
// r13   <- B
// r14   <- ldb
// k1    <- m_mask
// zmm0  <- []
// ...
// zmm23 <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_3VX5_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_3vx5_lib8c)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	leaq	0(%r13, %r14, 4), %r15
	leaq	0(%r14, %r14, 2), %rax

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0		32(%r13)
//	...

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	0*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	0*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	0*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	0*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
//	vbroadcastsd	0*8(%r15, %r14), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vfmadd231pd		%zmm26, %zmm24, %zmm13
//	vfmadd231pd		%zmm27, %zmm24, %zmm21
//	vbroadcastsd	0*8(%r15, %r14, 2), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm26, %zmm24, %zmm14
//	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	0*8(%r15, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vmovapd			1*64(%r11, %r12), %zmm26 // A
	vmovapd			1*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	1*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	1*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	1*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	1*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	1*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
//	vbroadcastsd	1*8(%r15, %r14), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vfmadd231pd		%zmm26, %zmm24, %zmm13
//	vfmadd231pd		%zmm27, %zmm24, %zmm21
//	vbroadcastsd	1*8(%r15, %r14, 2), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm26, %zmm24, %zmm14
//	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	1*8(%r15, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vmovapd			2*64(%r11, %r12), %zmm26 // A
	vmovapd			2*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	2*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	2*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	2*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	2*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	2*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
//	vbroadcastsd	2*8(%r15, %r14), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vfmadd231pd		%zmm26, %zmm24, %zmm13
//	vfmadd231pd		%zmm27, %zmm24, %zmm21
//	vbroadcastsd	2*8(%r15, %r14, 2), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm26, %zmm24, %zmm14
//	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	2*8(%r15, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vmovapd			3*64(%r11, %r12), %zmm26 // A
	vmovapd			3*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	3*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	3*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	3*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	3*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	3*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
//	vbroadcastsd	3*8(%r15, %r14), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vfmadd231pd		%zmm26, %zmm24, %zmm13
//	vfmadd231pd		%zmm27, %zmm24, %zmm21
//	vbroadcastsd	3*8(%r15, %r14, 2), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm26, %zmm24, %zmm14
//	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	3*8(%r15, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23

	subl	$ 4, %r10d
	addq	$ 4*64, %r11
	addq	$ 4*8, %r13
	addq	$ 4*8, %r15

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	0*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	0*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	0*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	0*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
//	vbroadcastsd	0*8(%r15, %r14), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vfmadd231pd		%zmm26, %zmm24, %zmm13
//	vfmadd231pd		%zmm27, %zmm24, %zmm21
//	vbroadcastsd	0*8(%r15, %r14, 2), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm26, %zmm24, %zmm14
//	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	0*8(%r15, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vmovapd			1*64(%r11, %r12), %zmm26 // A
	vmovapd			1*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	1*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	1*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	1*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	1*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	1*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
//	vbroadcastsd	1*8(%r15, %r14), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vfmadd231pd		%zmm26, %zmm24, %zmm13
//	vfmadd231pd		%zmm27, %zmm24, %zmm21
//	vbroadcastsd	1*8(%r15, %r14, 2), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm26, %zmm24, %zmm14
//	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	1*8(%r15, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vmovapd			2*64(%r11, %r12), %zmm26 // A
	vmovapd			2*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	2*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	2*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	2*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	2*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	2*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
//	vbroadcastsd	2*8(%r15, %r14), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vfmadd231pd		%zmm26, %zmm24, %zmm13
//	vfmadd231pd		%zmm27, %zmm24, %zmm21
//	vbroadcastsd	2*8(%r15, %r14, 2), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm26, %zmm24, %zmm14
//	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	2*8(%r15, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vmovapd			3*64(%r11, %r12), %zmm26 // A
	vmovapd			3*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	3*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	3*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	3*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	3*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	3*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
//	vbroadcastsd	3*8(%r15, %r14), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vfmadd231pd		%zmm26, %zmm24, %zmm13
//	vfmadd231pd		%zmm27, %zmm24, %zmm21
//	vbroadcastsd	3*8(%r15, %r14, 2), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm26, %zmm24, %zmm14
//	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	3*8(%r15, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23

	subl	$ 4, %r10d
	addq	$ 4*64, %r11
	addq	$ 4*8, %r13
	addq	$ 4*8, %r15

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	0*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	0*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	0*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19
	vbroadcastsd	0*8(%r15), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm4
	vfmadd231pd		%zmm26, %zmm24, %zmm12
	vfmadd231pd		%zmm27, %zmm24, %zmm20
//	vbroadcastsd	0*8(%r15, %r14), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm5
//	vfmadd231pd		%zmm26, %zmm24, %zmm13
//	vfmadd231pd		%zmm27, %zmm24, %zmm21
//	vbroadcastsd	0*8(%r15, %r14, 2), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm6
//	vfmadd231pd		%zmm26, %zmm24, %zmm14
//	vfmadd231pd		%zmm27, %zmm24, %zmm22
//	vbroadcastsd	0*8(%r15, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm7
//	vfmadd231pd		%zmm26, %zmm24, %zmm15
//	vfmadd231pd		%zmm27, %zmm24, %zmm23

	subl	$ 1, %r10d
	addq	$ 1*64, %r11
	addq	$ 1*8, %r13
	addq	$ 1*8, %r15

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_3vx5_lib8c)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- sda
// r13   <- B
// r14   <- ldb
// k1    <- m_mask
// zmm0  <- []
// ...
// zmm23 <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_3VX4_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_3vx4_lib8c)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	leaq	0(%r13, %r14, 4), %r15
	leaq	0(%r14, %r14, 2), %rax

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0		32(%r13)
//	...

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	0*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	0*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	0*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vmovapd			1*64(%r11, %r12), %zmm26 // A
	vmovapd			1*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	1*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	1*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	1*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	1*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vmovapd			2*64(%r11, %r12), %zmm26 // A
	vmovapd			2*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	2*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	2*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	2*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	2*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vmovapd			3*64(%r11, %r12), %zmm26 // A
	vmovapd			3*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	3*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	3*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	3*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	3*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19

	subl	$ 4, %r10d
	addq	$ 4*64, %r11
	addq	$ 4*8, %r13
	addq	$ 4*8, %r15

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	0*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	0*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	0*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vmovapd			1*64(%r11, %r12), %zmm26 // A
	vmovapd			1*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	1*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	1*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	1*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	1*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vmovapd			2*64(%r11, %r12), %zmm26 // A
	vmovapd			2*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	2*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	2*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	2*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	2*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vmovapd			3*64(%r11, %r12), %zmm26 // A
	vmovapd			3*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	3*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	3*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	3*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	3*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19

	subl	$ 4, %r10d
	addq	$ 4*64, %r11
	addq	$ 4*8, %r13
	addq	$ 4*8, %r15

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	0*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	0*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
	vbroadcastsd	0*8(%r13, %rax), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm3
	vfmadd231pd		%zmm26, %zmm24, %zmm11
	vfmadd231pd		%zmm27, %zmm24, %zmm19

	subl	$ 1, %r10d
	addq	$ 1*64, %r11
	addq	$ 1*8, %r13
	addq	$ 1*8, %r15

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_3vx4_lib8c)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- sda
// r13   <- B
// r14   <- ldb
// k1    <- m_mask
// zmm0  <- []
// ...
// zmm23 <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_3VX3_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_3vx3_lib8c)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	leaq	0(%r13, %r14, 4), %r15
	leaq	0(%r14, %r14, 2), %rax

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0		32(%r13)
//	...

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	0*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	0*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	0*8(%r13, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vmovapd			1*64(%r11, %r12), %zmm26 // A
	vmovapd			1*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	1*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	1*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	1*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	1*8(%r13, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vmovapd			2*64(%r11, %r12), %zmm26 // A
	vmovapd			2*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	2*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	2*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	2*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	2*8(%r13, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vmovapd			3*64(%r11, %r12), %zmm26 // A
	vmovapd			3*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	3*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	3*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	3*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	3*8(%r13, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19

	subl	$ 4, %r10d
	addq	$ 4*64, %r11
	addq	$ 4*8, %r13
	addq	$ 4*8, %r15

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	0*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	0*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	0*8(%r13, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vmovapd			1*64(%r11, %r12), %zmm26 // A
	vmovapd			1*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	1*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	1*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	1*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	1*8(%r13, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vmovapd			2*64(%r11, %r12), %zmm26 // A
	vmovapd			2*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	2*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	2*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	2*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	2*8(%r13, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vmovapd			3*64(%r11, %r12), %zmm26 // A
	vmovapd			3*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	3*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	3*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	3*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	3*8(%r13, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19

	subl	$ 4, %r10d
	addq	$ 4*64, %r11
	addq	$ 4*8, %r13
	addq	$ 4*8, %r15

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	0*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
	vbroadcastsd	0*8(%r13, %r14, 2), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm2
	vfmadd231pd		%zmm26, %zmm24, %zmm10
	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	0*8(%r13, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19

	subl	$ 1, %r10d
	addq	$ 1*64, %r11
	addq	$ 1*8, %r13
	addq	$ 1*8, %r15

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_3vx3_lib8c)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- sda
// r13   <- B
// r14   <- ldb
// k1    <- m_mask
// zmm0  <- []
// ...
// zmm23 <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_3VX2_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_3vx2_lib8c)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	leaq	0(%r13, %r14, 4), %r15
	leaq	0(%r14, %r14, 2), %rax

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0		32(%r13)
//	...

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	0*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
//	vbroadcastsd	0*8(%r13, %r14, 2), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm26, %zmm24, %zmm10
//	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	0*8(%r13, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vmovapd			1*64(%r11, %r12), %zmm26 // A
	vmovapd			1*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	1*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	1*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
//	vbroadcastsd	1*8(%r13, %r14, 2), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm26, %zmm24, %zmm10
//	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	1*8(%r13, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vmovapd			2*64(%r11, %r12), %zmm26 // A
	vmovapd			2*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	2*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	2*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
//	vbroadcastsd	2*8(%r13, %r14, 2), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm26, %zmm24, %zmm10
//	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	2*8(%r13, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vmovapd			3*64(%r11, %r12), %zmm26 // A
	vmovapd			3*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	3*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	3*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
//	vbroadcastsd	3*8(%r13, %r14, 2), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm26, %zmm24, %zmm10
//	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	3*8(%r13, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19

	subl	$ 4, %r10d
	addq	$ 4*64, %r11
	addq	$ 4*8, %r13
	addq	$ 4*8, %r15

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	0*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
//	vbroadcastsd	0*8(%r13, %r14, 2), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm26, %zmm24, %zmm10
//	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	0*8(%r13, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vmovapd			1*64(%r11, %r12), %zmm26 // A
	vmovapd			1*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	1*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	1*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
//	vbroadcastsd	1*8(%r13, %r14, 2), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm26, %zmm24, %zmm10
//	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	1*8(%r13, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vmovapd			2*64(%r11, %r12), %zmm26 // A
	vmovapd			2*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	2*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	2*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
//	vbroadcastsd	2*8(%r13, %r14, 2), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm26, %zmm24, %zmm10
//	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	2*8(%r13, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vmovapd			3*64(%r11, %r12), %zmm26 // A
	vmovapd			3*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	3*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	3*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
//	vbroadcastsd	3*8(%r13, %r14, 2), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm26, %zmm24, %zmm10
//	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	3*8(%r13, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19

	subl	$ 4, %r10d
	addq	$ 4*64, %r11
	addq	$ 4*8, %r13
	addq	$ 4*8, %r15

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
	vbroadcastsd	0*8(%r13, %r14), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm1
	vfmadd231pd		%zmm26, %zmm24, %zmm9
	vfmadd231pd		%zmm27, %zmm24, %zmm17
//	vbroadcastsd	0*8(%r13, %r14, 2), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm26, %zmm24, %zmm10
//	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	0*8(%r13, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19

	subl	$ 1, %r10d
	addq	$ 1*64, %r11
	addq	$ 1*8, %r13
	addq	$ 1*8, %r15

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_3vx2_lib8c)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- sda
// r13   <- B
// r14   <- ldb
// k1    <- m_mask
// zmm0  <- []
// ...
// zmm23 <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_3VX1_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_3vx1_lib8c)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	leaq	0(%r13, %r14, 4), %r15
	leaq	0(%r14, %r14, 2), %rax

	// preload


	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0		32(%r13)
//	...

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
//	vbroadcastsd	0*8(%r13, %r14), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vfmadd231pd		%zmm26, %zmm24, %zmm9
//	vfmadd231pd		%zmm27, %zmm24, %zmm17
//	vbroadcastsd	0*8(%r13, %r14, 2), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm26, %zmm24, %zmm10
//	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	0*8(%r13, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vmovapd			1*64(%r11, %r12), %zmm26 // A
	vmovapd			1*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	1*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
//	vbroadcastsd	1*8(%r13, %r14), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vfmadd231pd		%zmm26, %zmm24, %zmm9
//	vfmadd231pd		%zmm27, %zmm24, %zmm17
//	vbroadcastsd	1*8(%r13, %r14, 2), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm26, %zmm24, %zmm10
//	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	1*8(%r13, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vmovapd			2*64(%r11, %r12), %zmm26 // A
	vmovapd			2*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	2*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
//	vbroadcastsd	2*8(%r13, %r14), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vfmadd231pd		%zmm26, %zmm24, %zmm9
//	vfmadd231pd		%zmm27, %zmm24, %zmm17
//	vbroadcastsd	2*8(%r13, %r14, 2), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm26, %zmm24, %zmm10
//	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	2*8(%r13, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vmovapd			3*64(%r11, %r12), %zmm26 // A
	vmovapd			3*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	3*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
//	vbroadcastsd	3*8(%r13, %r14), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vfmadd231pd		%zmm26, %zmm24, %zmm9
//	vfmadd231pd		%zmm27, %zmm24, %zmm17
//	vbroadcastsd	3*8(%r13, %r14, 2), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm26, %zmm24, %zmm10
//	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	3*8(%r13, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19

	subl	$ 4, %r10d
	addq	$ 4*64, %r11
	addq	$ 4*8, %r13
	addq	$ 4*8, %r15

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
//	vbroadcastsd	0*8(%r13, %r14), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vfmadd231pd		%zmm26, %zmm24, %zmm9
//	vfmadd231pd		%zmm27, %zmm24, %zmm17
//	vbroadcastsd	0*8(%r13, %r14, 2), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm26, %zmm24, %zmm10
//	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	0*8(%r13, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19

	// unroll 1
	vmovapd			1*64(%r11), %zmm25 // A
	vmovapd			1*64(%r11, %r12), %zmm26 // A
	vmovapd			1*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	1*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
//	vbroadcastsd	1*8(%r13, %r14), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vfmadd231pd		%zmm26, %zmm24, %zmm9
//	vfmadd231pd		%zmm27, %zmm24, %zmm17
//	vbroadcastsd	1*8(%r13, %r14, 2), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm26, %zmm24, %zmm10
//	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	1*8(%r13, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19

	// unroll 2
	vmovapd			2*64(%r11), %zmm25 // A
	vmovapd			2*64(%r11, %r12), %zmm26 // A
	vmovapd			2*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	2*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
//	vbroadcastsd	2*8(%r13, %r14), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vfmadd231pd		%zmm26, %zmm24, %zmm9
//	vfmadd231pd		%zmm27, %zmm24, %zmm17
//	vbroadcastsd	2*8(%r13, %r14, 2), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm26, %zmm24, %zmm10
//	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	2*8(%r13, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19

	// unroll 3
	vmovapd			3*64(%r11), %zmm25 // A
	vmovapd			3*64(%r11, %r12), %zmm26 // A
	vmovapd			3*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	3*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
//	vbroadcastsd	3*8(%r13, %r14), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vfmadd231pd		%zmm26, %zmm24, %zmm9
//	vfmadd231pd		%zmm27, %zmm24, %zmm17
//	vbroadcastsd	3*8(%r13, %r14, 2), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm26, %zmm24, %zmm10
//	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	3*8(%r13, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19

	subl	$ 4, %r10d
	addq	$ 4*64, %r11
	addq	$ 4*8, %r13
	addq	$ 4*8, %r15

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovapd			0*64(%r11), %zmm25 // A
	vmovapd			0*64(%r11, %r12), %zmm26 // A
	vmovapd			0*64(%r11, %r12, 2), %zmm27 {%k1}{z} // A
	vbroadcastsd	0*8(%r13), %zmm24 // B
	vfmadd231pd		%zmm25, %zmm24, %zmm0
	vfmadd231pd		%zmm26, %zmm24, %zmm8
	vfmadd231pd		%zmm27, %zmm24, %zmm16
//	vbroadcastsd	0*8(%r13, %r14), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm1
//	vfmadd231pd		%zmm26, %zmm24, %zmm9
//	vfmadd231pd		%zmm27, %zmm24, %zmm17
//	vbroadcastsd	0*8(%r13, %r14, 2), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm2
//	vfmadd231pd		%zmm26, %zmm24, %zmm10
//	vfmadd231pd		%zmm27, %zmm24, %zmm18
//	vbroadcastsd	0*8(%r13, %rax), %zmm24 // B
//	vfmadd231pd		%zmm25, %zmm24, %zmm3
//	vfmadd231pd		%zmm26, %zmm24, %zmm11
//	vfmadd231pd		%zmm27, %zmm24, %zmm19

	subl	$ 1, %r10d
	addq	$ 1*64, %r11
	addq	$ 1*8, %r13
	addq	$ 1*8, %r15

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_3vx1_lib8c)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- 4*sda*sizeof(double)
// r13   <- B
// r14   <- ldb
// r15   <- m1
// rax   <- n1
// zmm0  <- []
// ...
// zmm23 <- []

//
// output arguments:
// r10d  <- 0
// r11   <- A+8*k*sizeof(double)
// r12   <- 4*sda*sizeof(double)
// r13   <- B+8*k*sizeof(double)
// r14   <- ldb
// r15   <- m1
// rax   <- n1
// zmm0  <- []
// ...
// zmm23 <- []

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_DGEMM_NN_24X8_VS_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_dgemm_nn_24x8_vs_lib8c)
#endif

	// compute mask for rows
	vcvtsi2sd	%r15d, %xmm25, %xmm25
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC01(%rip), %zmm24
#elif defined(OS_MAC)
	vmovupd		LC01(%rip), %zmm24
#endif
	vbroadcastsd	%xmm25, %zmm25
	vsubpd		%zmm25, %zmm24, %zmm25
	vpmovq2m	%zmm25, %k1

	cmpl	$ 1, %eax
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_3VX1_LIB8C
#else
	CALL(inner_kernel_dgemm_nn_3vx1_lib8c)
#endif
	
	jmp		107f

100:

	cmpl	$ 2, %eax
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_3VX2_LIB8C
#else
	CALL(inner_kernel_dgemm_nn_3vx2_lib8c)
#endif
	
	jmp		107f

101:

	cmpl	$ 3, %eax
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_3VX3_LIB8C
#else
	CALL(inner_kernel_dgemm_nn_3vx3_lib8c)
#endif
	
	jmp		107f

102:

	cmpl	$ 4, %eax
	jg		103f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_3VX4_LIB8C
#else
	CALL(inner_kernel_dgemm_nn_3vx4_lib8c)
#endif
	
	jmp		107f

103:

	cmpl	$ 5, %eax
	jg		104f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_3VX5_LIB8C
#else
	CALL(inner_kernel_dgemm_nn_3vx5_lib8c)
#endif
	
	jmp		107f

104:

	cmpl	$ 6, %eax
	jg		105f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_3VX6_LIB8C
#else
	CALL(inner_kernel_dgemm_nn_3vx6_lib8c)
#endif
	
	jmp		107f

105:

	cmpl	$ 7, %eax
	jg		106f

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_3VX7_LIB8C
#else
	CALL(inner_kernel_dgemm_nn_3vx7_lib8c)
#endif
	
	jmp		107f

106:

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_3VX8_LIB8C
#else
	CALL(inner_kernel_dgemm_nn_3vx8_lib8c)
#endif

107:

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_dgemm_nn_24x8_vs_lib8c)
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
// zmm0 <- []
// ...
// zmm15 <- []
//
// output arguments:
// r10   <- alpha
// r11   <- beta
// r10   <- C
// r13   <- ldc
// zmm0 <- []
// ...
// zmm15 <- []

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_24X8_LIB
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_24x8_lib)
#endif
	
	// alpha
	vbroadcastsd	0(%r10), %zmm25

	vmulpd		%zmm0, %zmm25, %zmm0
	vmulpd		%zmm1, %zmm25, %zmm1
	vmulpd		%zmm2, %zmm25, %zmm2
	vmulpd		%zmm3, %zmm25, %zmm3
	vmulpd		%zmm4, %zmm25, %zmm4
	vmulpd		%zmm5, %zmm25, %zmm5
	vmulpd		%zmm6, %zmm25, %zmm6
	vmulpd		%zmm7, %zmm25, %zmm7
	vmulpd		%zmm8, %zmm25, %zmm8
	vmulpd		%zmm9, %zmm25, %zmm9
	vmulpd		%zmm10, %zmm25, %zmm10
	vmulpd		%zmm11, %zmm25, %zmm11
	vmulpd		%zmm12, %zmm25, %zmm12
	vmulpd		%zmm13, %zmm25, %zmm13
	vmulpd		%zmm14, %zmm25, %zmm14
	vmulpd		%zmm15, %zmm25, %zmm15
	vmulpd		%zmm16, %zmm25, %zmm16
	vmulpd		%zmm17, %zmm25, %zmm17
	vmulpd		%zmm18, %zmm25, %zmm18
	vmulpd		%zmm19, %zmm25, %zmm19
	vmulpd		%zmm20, %zmm25, %zmm20
	vmulpd		%zmm21, %zmm25, %zmm21
	vmulpd		%zmm22, %zmm25, %zmm22
	vmulpd		%zmm23, %zmm25, %zmm23

	// beta
	vbroadcastsd	0(%r11), %zmm24

	vxorpd		%zmm25, %zmm25, %zmm25 // 0.0

	vucomisd	%xmm25, %xmm24 // beta==0.0 ?
	je			0f // end

	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm0
	vmovupd		64(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm8
	vmovupd		128(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm16
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm1
	vmovupd		64(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm9
	vmovupd		128(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm17
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm2
	vmovupd		64(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm10
	vmovupd		128(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm18
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm3
	vmovupd		64(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm11
	vmovupd		128(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm19
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm4
	vmovupd		64(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm12
	vmovupd		128(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm20
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm5
	vmovupd		64(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm13
	vmovupd		128(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm21
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm6
	vmovupd		64(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm14
	vmovupd		128(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm22
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm7
	vmovupd		64(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm15
	vmovupd		128(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm23
//	addq		%r13, %r12

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_24x8_lib)
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
// zmm0 <- []
// ...
// zmm23 <- []
//
// output arguments:
// r10   <- alpha
// r11   <- beta
// r10   <- C
// r13   <- ldc
// zmm0 <- []
// ...
// zmm23 <- []

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_8X24_LIB
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_8x24_lib)
#endif
	
	// alpha
	vbroadcastsd	0(%r10), %zmm25

	vmulpd		%zmm0, %zmm25, %zmm0
	vmulpd		%zmm1, %zmm25, %zmm1
	vmulpd		%zmm2, %zmm25, %zmm2
	vmulpd		%zmm3, %zmm25, %zmm3
	vmulpd		%zmm4, %zmm25, %zmm4
	vmulpd		%zmm5, %zmm25, %zmm5
	vmulpd		%zmm6, %zmm25, %zmm6
	vmulpd		%zmm7, %zmm25, %zmm7
	vmulpd		%zmm8, %zmm25, %zmm8
	vmulpd		%zmm9, %zmm25, %zmm9
	vmulpd		%zmm10, %zmm25, %zmm10
	vmulpd		%zmm11, %zmm25, %zmm11
	vmulpd		%zmm12, %zmm25, %zmm12
	vmulpd		%zmm13, %zmm25, %zmm13
	vmulpd		%zmm14, %zmm25, %zmm14
	vmulpd		%zmm15, %zmm25, %zmm15
	vmulpd		%zmm16, %zmm25, %zmm16
	vmulpd		%zmm17, %zmm25, %zmm17
	vmulpd		%zmm18, %zmm25, %zmm18
	vmulpd		%zmm19, %zmm25, %zmm19
	vmulpd		%zmm20, %zmm25, %zmm20
	vmulpd		%zmm21, %zmm25, %zmm21
	vmulpd		%zmm22, %zmm25, %zmm22
	vmulpd		%zmm23, %zmm25, %zmm23

	// beta
	vbroadcastsd	0(%r11), %zmm24

	vxorpd		%zmm25, %zmm25, %zmm25 // 0.0

	vucomisd	%xmm25, %xmm24 // beta==0.0 ?
	je			0f // end

	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm0
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm1
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm2
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm3
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm4
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm5
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm6
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm7
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm8
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm9
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm10
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm11
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm12
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm13
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm14
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm15
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm16
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm17
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm18
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm19
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm20
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm21
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm22
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm23
//	addq		%r13, %r12

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_8x24_lib)
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
// r14   <- m1
// r15   <- n1
// zmm0 <- []
// ...
// zmm15 <- []
//
// output arguments:
// r10   <- alpha
// r11   <- beta
// r10   <- C
// r13   <- ldc
// r14   <- m1
// r15   <- n1
// zmm0 <- []
// ...
// zmm15 <- []

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_24X8_VS_LIB
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_24x8_vs_lib)
#endif
	
	// alpha
	vbroadcastsd	0(%r10), %zmm25

	vmulpd		%zmm0, %zmm25, %zmm0
	vmulpd		%zmm1, %zmm25, %zmm1
	vmulpd		%zmm2, %zmm25, %zmm2
	vmulpd		%zmm3, %zmm25, %zmm3
	vmulpd		%zmm4, %zmm25, %zmm4
	vmulpd		%zmm5, %zmm25, %zmm5
	vmulpd		%zmm6, %zmm25, %zmm6
	vmulpd		%zmm7, %zmm25, %zmm7
	vmulpd		%zmm8, %zmm25, %zmm8
	vmulpd		%zmm9, %zmm25, %zmm9
	vmulpd		%zmm10, %zmm25, %zmm10
	vmulpd		%zmm11, %zmm25, %zmm11
	vmulpd		%zmm12, %zmm25, %zmm12
	vmulpd		%zmm13, %zmm25, %zmm13
	vmulpd		%zmm14, %zmm25, %zmm14
	vmulpd		%zmm15, %zmm25, %zmm15
	vmulpd		%zmm16, %zmm25, %zmm16
	vmulpd		%zmm17, %zmm25, %zmm17
	vmulpd		%zmm18, %zmm25, %zmm18
	vmulpd		%zmm19, %zmm25, %zmm19
	vmulpd		%zmm20, %zmm25, %zmm20
	vmulpd		%zmm21, %zmm25, %zmm21
	vmulpd		%zmm22, %zmm25, %zmm22
	vmulpd		%zmm23, %zmm25, %zmm23

	// beta
	vbroadcastsd	0(%r11), %zmm24

	vxorpd		%zmm25, %zmm25, %zmm25 // 0.0

	vucomisd	%xmm25, %xmm24 // beta==0.0 ?
	je			0f // end

	// compute mask for rows
	vcvtsi2sd	%r14d, %xmm25, %xmm25
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC02(%rip), %zmm26
#elif defined(OS_MAC)
	vmovupd		LC02(%rip), %zmm26
#endif
	vbroadcastsd	%xmm25, %zmm25
	vsubpd		%zmm25, %zmm26, %zmm25
	vpmovq2m	%zmm25, %k1

	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm0
	vmovupd		64(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm8
	vmovupd		128(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm16
	addq		%r13, %r12
	cmpl		$ 2, %r15d
	jl			0f // end
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm1
	vmovupd		64(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm9
	vmovupd		128(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm17
	addq		%r13, %r12
	cmpl		$ 3, %r15d
	jl			0f // end
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm2
	vmovupd		64(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm10
	vmovupd		128(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm18
	addq		%r13, %r12
	cmpl		$ 4, %r15d
	jl			0f // end
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm3
	vmovupd		64(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm11
	vmovupd		128(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm19
	addq		%r13, %r12
	cmpl		$ 5, %r15d
	jl			0f // end
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm4
	vmovupd		64(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm12
	vmovupd		128(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm20
	addq		%r13, %r12
	cmpl		$ 6, %r15d
	jl			0f // end
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm5
	vmovupd		64(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm13
	vmovupd		128(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm21
	addq		%r13, %r12
	cmpl		$ 7, %r15d
	jl			0f // end
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm6
	vmovupd		64(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm14
	vmovupd		128(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm22
	addq		%r13, %r12
	cmpl		$ 7, %r15d
	je			0f // end
	vmovupd		0(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm7
	vmovupd		64(%r12), %zmm25
	vfmadd231pd	%zmm24, %zmm25, %zmm15
	vmovupd		128(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm23
//	addq		%r13, %r12

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_24x8_vs_lib)
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
// r14   <- m1
// r15   <- n1
// zmm0 <- []
// ...
// zmm23 <- []
//
// output arguments:
// r10   <- alpha
// r11   <- beta
// r10   <- C
// r13   <- ldc
// r14   <- m1
// r15   <- n1
// zmm0 <- []
// ...
// zmm23 <- []

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_8X24_VS_LIB
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_8x24_vs_lib)
#endif
	
	// alpha
	vbroadcastsd	0(%r10), %zmm25

	vmulpd		%zmm0, %zmm25, %zmm0
	vmulpd		%zmm1, %zmm25, %zmm1
	vmulpd		%zmm2, %zmm25, %zmm2
	vmulpd		%zmm3, %zmm25, %zmm3
	vmulpd		%zmm4, %zmm25, %zmm4
	vmulpd		%zmm5, %zmm25, %zmm5
	vmulpd		%zmm6, %zmm25, %zmm6
	vmulpd		%zmm7, %zmm25, %zmm7
	vmulpd		%zmm8, %zmm25, %zmm8
	vmulpd		%zmm9, %zmm25, %zmm9
	vmulpd		%zmm10, %zmm25, %zmm10
	vmulpd		%zmm11, %zmm25, %zmm11
	vmulpd		%zmm12, %zmm25, %zmm12
	vmulpd		%zmm13, %zmm25, %zmm13
	vmulpd		%zmm14, %zmm25, %zmm14
	vmulpd		%zmm15, %zmm25, %zmm15
	vmulpd		%zmm16, %zmm25, %zmm16
	vmulpd		%zmm17, %zmm25, %zmm17
	vmulpd		%zmm18, %zmm25, %zmm18
	vmulpd		%zmm19, %zmm25, %zmm19
	vmulpd		%zmm20, %zmm25, %zmm20
	vmulpd		%zmm21, %zmm25, %zmm21
	vmulpd		%zmm22, %zmm25, %zmm22
	vmulpd		%zmm23, %zmm25, %zmm23

	// beta
	vbroadcastsd	0(%r11), %zmm24

	vxorpd		%zmm25, %zmm25, %zmm25 // 0.0

	vucomisd	%xmm25, %xmm24 // beta==0.0 ?
	je			0f // end

	// compute mask for rows
	vcvtsi2sd	%r14d, %xmm25, %xmm25
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC00(%rip), %zmm26
#elif defined(OS_MAC)
	vmovupd		LC00(%rip), %zmm26
#endif
	vbroadcastsd	%xmm25, %zmm25
	vsubpd		%zmm25, %zmm26, %zmm25
	vpmovq2m	%zmm25, %k1

	vmovupd		0(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm0
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm1
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm2
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm3
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm4
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm5
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm6
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm7
	addq		%r13, %r12

	vmovupd		0(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm8
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm9
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm10
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm11
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm12
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm13
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm14
	addq		%r13, %r12
	vmovupd		0(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm15
	addq		%r13, %r12

	vmovupd		0(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm16
	addq		%r13, %r12
	cmpl		$ 18, %r15d
	jl			0f // end
	vmovupd		0(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm17
	addq		%r13, %r12
	cmpl		$ 19, %r15d
	jl			0f // end
	vmovupd		0(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm18
	addq		%r13, %r12
	cmpl		$ 20, %r15d
	jl			0f // end
	vmovupd		0(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm19
	addq		%r13, %r12
	cmpl		$ 21, %r15d
	jl			0f // end
	vmovupd		0(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm20
	addq		%r13, %r12
	cmpl		$ 22, %r15d
	jl			0f // end
	vmovupd		0(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm21
	addq		%r13, %r12
	cmpl		$ 23, %r15d
	jl			0f // end
	vmovupd		0(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm22
	addq		%r13, %r12
	cmpl		$ 23, %r15d
	je			0f // end
	vmovupd		0(%r12), %zmm25 {%k1}{z}
	vfmadd231pd	%zmm24, %zmm25, %zmm23
//	addq		%r13, %r12

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_8x24_vs_lib)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// r11  <- ldd
// zmm0 <- []
// ...
// zmm15 <- []
//
// output arguments:
// r10  <- D
// r11  <- ldd
// zmm0 <- []
// ...
// zmm15 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_24X8_LIB
#else
	.p2align 4,,15
	FUN_START(inner_store_24x8_lib)
#endif
	
	vmovupd %zmm0, 0(%r10)
	vmovupd %zmm8, 64(%r10)
	vmovupd %zmm16, 128(%r10)
	addq		%r11, %r10
	vmovupd %zmm1, 0(%r10)
	vmovupd %zmm9, 64(%r10)
	vmovupd %zmm17, 128(%r10)
	addq		%r11, %r10
	vmovupd %zmm2, 0(%r10)
	vmovupd %zmm10, 64(%r10)
	vmovupd %zmm18, 128(%r10)
	addq		%r11, %r10
	vmovupd %zmm3, 0(%r10)
	vmovupd %zmm11, 64(%r10)
	vmovupd %zmm19, 128(%r10)
	addq		%r11, %r10
	vmovupd %zmm4, 0(%r10)
	vmovupd %zmm12, 64(%r10)
	vmovupd %zmm20, 128(%r10)
	addq		%r11, %r10
	vmovupd %zmm5, 0(%r10)
	vmovupd %zmm13, 64(%r10)
	vmovupd %zmm21, 128(%r10)
	addq		%r11, %r10
	vmovupd %zmm6, 0(%r10)
	vmovupd %zmm14, 64(%r10)
	vmovupd %zmm22, 128(%r10)
	addq		%r11, %r10
	vmovupd %zmm7, 0(%r10)
	vmovupd %zmm15, 64(%r10)
	vmovupd %zmm23, 128(%r10)
//	addq		%r11, %r10

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_24x8_lib)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// r11  <- ldd
// zmm0 <- []
// ...
// zmm23 <- []
//
// output arguments:
// r10  <- D
// r11  <- ldd
// zmm0 <- []
// ...
// zmm23 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_8X24_LIB
#else
	.p2align 4,,15
	FUN_START(inner_store_8x24_lib)
#endif
	
	vmovupd %zmm0, 0(%r10)
	addq		%r11, %r10
	vmovupd %zmm1, 0(%r10)
	addq		%r11, %r10
	vmovupd %zmm2, 0(%r10)
	addq		%r11, %r10
	vmovupd %zmm3, 0(%r10)
	addq		%r11, %r10
	vmovupd %zmm4, 0(%r10)
	addq		%r11, %r10
	vmovupd %zmm5, 0(%r10)
	addq		%r11, %r10
	vmovupd %zmm6, 0(%r10)
	addq		%r11, %r10
	vmovupd %zmm7, 0(%r10)
	addq		%r11, %r10
	vmovupd %zmm8, 0(%r10)
	addq		%r11, %r10
	vmovupd %zmm9, 0(%r10)
	addq		%r11, %r10
	vmovupd %zmm10, 0(%r10)
	addq		%r11, %r10
	vmovupd %zmm11, 0(%r10)
	addq		%r11, %r10
	vmovupd %zmm12, 0(%r10)
	addq		%r11, %r10
	vmovupd %zmm13, 0(%r10)
	addq		%r11, %r10
	vmovupd %zmm14, 0(%r10)
	addq		%r11, %r10
	vmovupd %zmm15, 0(%r10)
	addq		%r11, %r10
	vmovupd %zmm16, 0(%r10)
	addq		%r11, %r10
	vmovupd %zmm17, 0(%r10)
	addq		%r11, %r10
	vmovupd %zmm18, 0(%r10)
	addq		%r11, %r10
	vmovupd %zmm19, 0(%r10)
	addq		%r11, %r10
	vmovupd %zmm20, 0(%r10)
	addq		%r11, %r10
	vmovupd %zmm21, 0(%r10)
	addq		%r11, %r10
	vmovupd %zmm22, 0(%r10)
	addq		%r11, %r10
	vmovupd %zmm23, 0(%r10)
//	addq		%r11, %r10

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_8x24_lib)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// r11  <- ldd
// r12  <- m1
// r13  <- n1
// zmm0 <- []
// ...
// zmm15 <- []
//
// output arguments:
// r10  <- D
// r11  <- ldd
// r12  <- m1
// r13  <- n1
// zmm0 <- []
// ...
// zmm15 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_24X8_VS_LIB
#else
	.p2align 4,,15
	FUN_START(inner_store_24x8_vs_lib)
#endif
	
	// compute mask for rows
	vcvtsi2sd	%r12d, %xmm25, %xmm25
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC02(%rip), %zmm24
#elif defined(OS_MAC)
	vmovupd		LC02(%rip), %zmm24
#endif
	vbroadcastsd	%xmm25, %zmm25
	vsubpd		%zmm25, %zmm24, %zmm25
	vpmovq2m	%zmm25, %k1

	vmovupd %zmm0, 0(%r10)
	vmovupd %zmm8, 64(%r10)
	vmovupd %zmm16, 128(%r10) {%k1}
	addq		%r11, %r10
	cmpl		$ 2, %r13d
	jl			0f // end
	vmovupd %zmm1, 0(%r10)
	vmovupd %zmm9, 64(%r10)
	vmovupd %zmm17, 128(%r10) {%k1}
	addq		%r11, %r10
	cmpl		$ 3, %r13d
	jl			0f // end
	vmovupd %zmm2, 0(%r10)
	vmovupd %zmm10, 64(%r10)
	vmovupd %zmm18, 128(%r10) {%k1}
	addq		%r11, %r10
	cmpl		$ 4, %r13d
	jl			0f // end
	vmovupd %zmm3, 0(%r10)
	vmovupd %zmm11, 64(%r10)
	vmovupd %zmm19, 128(%r10) {%k1}
	addq		%r11, %r10
	cmpl		$ 5, %r13d
	jl			0f // end
	vmovupd %zmm4, 0(%r10)
	vmovupd %zmm12, 64(%r10)
	vmovupd %zmm20, 128(%r10) {%k1}
	addq		%r11, %r10
	cmpl		$ 6, %r13d
	jl			0f // end
	vmovupd %zmm5, 0(%r10)
	vmovupd %zmm13, 64(%r10)
	vmovupd %zmm21, 128(%r10) {%k1}
	addq		%r11, %r10
	cmpl		$ 7, %r13d
	jl			0f // end
	vmovupd %zmm6, 0(%r10)
	vmovupd %zmm14, 64(%r10)
	vmovupd %zmm22, 128(%r10) {%k1}
	addq		%r11, %r10
	cmpl		$ 7, %r13d
	je			0f // end
	vmovupd %zmm7, 0(%r10)
	vmovupd %zmm15, 64(%r10)
	vmovupd %zmm23, 128(%r10) {%k1}
//	addq		%r11, %r10

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_24x8_vs_lib)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// r11  <- ldd
// r12  <- m1
// r13  <- n1
// zmm0 <- []
// ...
// zmm23 <- []
//
// output arguments:
// r10  <- D
// r11  <- ldd
// r12  <- m1
// r13  <- n1
// zmm0 <- []
// ...
// zmm23 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_8X24_VS_LIB
#else
	.p2align 4,,15
	FUN_START(inner_store_8x24_vs_lib)
#endif
	
	// compute mask for rows
	vcvtsi2sd	%r12d, %xmm25, %xmm25
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC00(%rip), %zmm24
#elif defined(OS_MAC)
	vmovupd		LC00(%rip), %zmm24
#endif
	vbroadcastsd	%xmm25, %zmm25
	vsubpd		%zmm25, %zmm24, %zmm25
	vpmovq2m	%zmm25, %k1

	vmovupd %zmm0, 0(%r10) {%k1}
	addq		%r11, %r10
	vmovupd %zmm1, 0(%r10) {%k1}
	addq		%r11, %r10
	vmovupd %zmm2, 0(%r10) {%k1}
	addq		%r11, %r10
	vmovupd %zmm3, 0(%r10) {%k1}
	addq		%r11, %r10
	vmovupd %zmm4, 0(%r10) {%k1}
	addq		%r11, %r10
	vmovupd %zmm5, 0(%r10) {%k1}
	addq		%r11, %r10
	vmovupd %zmm6, 0(%r10) {%k1}
	addq		%r11, %r10
	vmovupd %zmm7, 0(%r10) {%k1}
	addq		%r11, %r10

	vmovupd %zmm8, 0(%r10) {%k1}
	addq		%r11, %r10
	vmovupd %zmm9, 0(%r10) {%k1}
	addq		%r11, %r10
	vmovupd %zmm10, 0(%r10) {%k1}
	addq		%r11, %r10
	vmovupd %zmm11, 0(%r10) {%k1}
	addq		%r11, %r10
	vmovupd %zmm12, 0(%r10) {%k1}
	addq		%r11, %r10
	vmovupd %zmm13, 0(%r10) {%k1}
	addq		%r11, %r10
	vmovupd %zmm14, 0(%r10) {%k1}
	addq		%r11, %r10
	vmovupd %zmm15, 0(%r10) {%k1}
	addq		%r11, %r10

	vmovupd %zmm16, 0(%r10) {%k1}
	addq		%r11, %r10
	cmpl		$ 18, %r13d
	jl			0f // end
	vmovupd %zmm17, 0(%r10) {%k1}
	addq		%r11, %r10
	cmpl		$ 19, %r13d
	jl			0f // end
	vmovupd %zmm18, 0(%r10) {%k1}
	addq		%r11, %r10
	cmpl		$ 20, %r13d
	jl			0f // end
	vmovupd %zmm19, 0(%r10) {%k1}
	addq		%r11, %r10
	cmpl		$ 21, %r13d
	jl			0f // end
	vmovupd %zmm20, 0(%r10) {%k1}
	addq		%r11, %r10
	cmpl		$ 22, %r13d
	jl			0f // end
	vmovupd %zmm21, 0(%r10) {%k1}
	addq		%r11, %r10
	cmpl		$ 23, %r13d
	jl			0f // end
	vmovupd %zmm22, 0(%r10) {%k1}
	addq		%r11, %r10
	cmpl		$ 23, %r13d
	je			0f // end
	vmovupd %zmm23, 0(%r10) {%k1}
//	addq		%r11, %r10

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_8x24_vs_lib)
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
	.macro INNER_PREFETCH0_24X8_LIB
#else
	.p2align 4,,15
	FUN_START(inner_prefetch0_24x8_lib)
#endif

	leaq		0(%r11, %r11, 2), %r12
	leaq		0(%r10, %r11, 4), %r13

	prefetcht0	0(%r10)
	prefetcht0	64(%r10)
	prefetcht0	128(%r10)
	prefetcht0	191(%r10)
	prefetcht0	0(%r10, %r11)
	prefetcht0	64(%r10, %r11)
	prefetcht0	128(%r10, %r11)
	prefetcht0	191(%r10, %r11)
	prefetcht0	0(%r10, %r11, 2)
	prefetcht0	64(%r10, %r11, 2)
	prefetcht0	128(%r10, %r11, 2)
	prefetcht0	191(%r10, %r11, 2)
	prefetcht0	0(%r10, %r12)
	prefetcht0	64(%r10, %r12)
	prefetcht0	128(%r10, %r12)
	prefetcht0	191(%r10, %r12)
	prefetcht0	0(%r13)
	prefetcht0	64(%r13)
	prefetcht0	128(%r13)
	prefetcht0	191(%r13)
	prefetcht0	0(%r13, %r11)
	prefetcht0	64(%r13, %r11)
	prefetcht0	128(%r13, %r11)
	prefetcht0	191(%r13, %r11)
	prefetcht0	0(%r13, %r11, 2)
	prefetcht0	64(%r13, %r11, 2)
	prefetcht0	128(%r13, %r11, 2)
	prefetcht0	191(%r13, %r11, 2)
	prefetcht0	0(%r13, %r12)
	prefetcht0	64(%r13, %r12)
	prefetcht0	128(%r13, %r12)
	prefetcht0	191(%r13, %r12)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_prefetch0_24x8_lib)
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
	.macro INNER_PREFETCH0_8X24_LIB
#else
	.p2align 4,,15
	FUN_START(inner_prefetch0_8x24_lib)
#endif

	leaq		0(%r11, %r11, 2), %r12
	leaq		0(%r10, %r11, 4), %r13

	prefetcht0	0(%r10)
	prefetcht0	63(%r10)
	prefetcht0	0(%r10, %r11)
	prefetcht0	63(%r10, %r11)
	prefetcht0	0(%r10, %r11, 2)
	prefetcht0	63(%r10, %r11, 2)
	prefetcht0	0(%r10, %r12)
	prefetcht0	63(%r10, %r12)
	prefetcht0	0(%r13)
	prefetcht0	63(%r13)
	prefetcht0	0(%r13, %r11)
	prefetcht0	63(%r13, %r11)
	prefetcht0	0(%r13, %r11, 2)
	prefetcht0	63(%r13, %r11, 2)
	prefetcht0	0(%r13, %r12)
	prefetcht0	63(%r13, %r12)

	leaq		0(%r10, %r11, 8), %r10
	leaq		0(%r13, %r11, 8), %r13

	prefetcht0	0(%r10)
	prefetcht0	63(%r10)
	prefetcht0	0(%r10, %r11)
	prefetcht0	63(%r10, %r11)
	prefetcht0	0(%r10, %r11, 2)
	prefetcht0	63(%r10, %r11, 2)
	prefetcht0	0(%r10, %r12)
	prefetcht0	63(%r10, %r12)
	prefetcht0	0(%r13)
	prefetcht0	63(%r13)
	prefetcht0	0(%r13, %r11)
	prefetcht0	63(%r13, %r11)
	prefetcht0	0(%r13, %r11, 2)
	prefetcht0	63(%r13, %r11, 2)
	prefetcht0	0(%r13, %r12)
	prefetcht0	63(%r13, %r12)

	leaq		0(%r10, %r11, 8), %r10
	leaq		0(%r13, %r11, 8), %r13

	prefetcht0	0(%r10)
	prefetcht0	63(%r10)
	prefetcht0	0(%r10, %r11)
	prefetcht0	63(%r10, %r11)
	prefetcht0	0(%r10, %r11, 2)
	prefetcht0	63(%r10, %r11, 2)
	prefetcht0	0(%r10, %r12)
	prefetcht0	63(%r10, %r12)
	prefetcht0	0(%r13)
	prefetcht0	63(%r13)
	prefetcht0	0(%r13, %r11)
	prefetcht0	63(%r13, %r11)
	prefetcht0	0(%r13, %r11, 2)
	prefetcht0	63(%r13, %r11, 2)
	prefetcht0	0(%r13, %r12)
	prefetcht0	63(%r13, %r12)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_prefetch0_8x24_lib)
#endif





//                                   1      2              3          4        5          6             7          8        9          10
// void kernel_dgemm_nt_24x8_lib88cc(int k, double *alpha, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_24x8_lib88cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// prefetch C

	movq		ARG6, %r10 // beta
	vmovsd		0(%r10), %xmm24
	vxorpd		%xmm25, %xmm25, %xmm25 // 0.0
	vucomisd	%xmm25, %xmm24 // beta==0.0 ?
	je			100f // end

	movq	ARG7, %r10 // C
	movq	ARG8, %r11 // ldc
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_24X8_LIB
#else
	CALL(inner_prefetch0_24x8_lib)
#endif

100:


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 6, %r12d // 8*sda*sizeof(double)
	movq	ARG5, %r13  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_24X8_LIB8
#else
	CALL(inner_kernel_dgemm_nt_24x8_lib8)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_24X8_LIB
#else
	CALL(inner_scale_ab_24x8_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_24X8_LIB
#else
	CALL(inner_store_24x8_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_24x8_lib88cc)





//                                      1      2              3          4        5          6             7          8        9          10       11      12
// void kernel_dgemm_nt_24x8_vs_lib88cc(int k, double *alpha, double *A, int sda, double *B, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_24x8_vs_lib88cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 6, %r12d // 8*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG11, %r14 // m1
	movq	ARG12, %r15 // n1

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_24X8_VS_LIB8
#else
	CALL(inner_kernel_dgemm_nt_24x8_vs_lib8)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG11, %r14 // m1
	movq	ARG12, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_24X8_VS_LIB
#else
	CALL(inner_scale_ab_24x8_vs_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG11, %r12 // m1
	movq	ARG12, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_24X8_VS_LIB
#else
	CALL(inner_store_24x8_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_24x8_vs_lib88cc)





//                                  1      2              3          4        5          6        7             8          9        10         11
// void kernel_dgemm_nt_24x8_lib8ccc(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_24x8_lib8ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 6, %r12d // 8*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_24X8_LIB8C
#else
	CALL(inner_kernel_dgemm_nt_24x8_lib8c)
#endif


	// prefetch D

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_24X8_LIB
#else
	CALL(inner_prefetch0_24x8_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_24X8_LIB
#else
	CALL(inner_scale_ab_24x8_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_24X8_LIB
#else
	CALL(inner_store_24x8_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_24x8_lib8ccc)






//                                      1      2              3          4        5          6        7             8          9        10         11       12      13
// void kernel_dgemm_nt_24x8_vs_lib8ccc(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_24x8_vs_lib8ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 6, %r12d // 8*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d
	movq	ARG12, %r15 // m1
	movq	ARG13, %rax // n1

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_24X8_VS_LIB8C
#else
	CALL(inner_kernel_dgemm_nt_24x8_vs_lib8c)
#endif


	// prefetch D
	// TODO prefetch vs !!!

//	movq	ARG9, %r10 // D
//	movq	ARG10, %r11 // ldd
//	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
//	INNER_PREFETCH0_24X8_LIB
#else
//	CALL(inner_prefetch0_24x8_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG12, %r14 // m1
	movq	ARG13, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_24X8_VS_LIB
#else
	CALL(inner_scale_ab_24x8_vs_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_24X8_VS_LIB
#else
	CALL(inner_store_24x8_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_24x8_vs_lib8ccc)





//                                  1      2              3          4        5          6        7             8          9        10         11
// void kernel_dgemm_nt_8x24_libc8cc(int k, double *alpha, double *A, int lda, double *B, int sdb, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_8x24_libc8cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG5, %r11  // B
	movq	ARG6, %r12 // sdb
	sall	$ 6, %r12d // 8*sdb*sizeof(double)
	movq	ARG3, %r13  // A
	movq	ARG4, %r14  // lda
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_24X8_LIB8C
#else
	CALL(inner_kernel_dgemm_nt_24x8_lib8c)
#endif

#if MACRO_LEVEL>=1
	INNER_TRAN_24X8_LIB8
#else
	CALL(inner_tran_24x8_lib8)
#endif


	// prefetch D TODO

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_8X24_LIB
#else
	CALL(inner_prefetch0_8x24_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X24_LIB
#else
	CALL(inner_scale_ab_8x24_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_8X24_LIB
#else
	CALL(inner_store_8x24_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_8x24_libc8cc)






//                                      1      2              3          4        5          6        7             8          9        10         11       12      13
// void kernel_dgemm_nt_8x24_vs_libc8cc(int k, double *alpha, double *A, int lda, double *B, int sdb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_8x24_vs_libc8cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG5, %r11  // B
	movq	ARG6, %r12 // sdb
	sall	$ 6, %r12d // 8*sdb*sizeof(double)
	movq	ARG3, %r13  // A
	movq	ARG4, %r14  // lda
	sall	$ 3, %r14d
	movq	ARG13, %r15 // n1
	movq	ARG12, %rax // m1

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_24X8_VS_LIB8C
#else
	CALL(inner_kernel_dgemm_nt_24x8_vs_lib8c)
#endif

#if MACRO_LEVEL>=1
	INNER_TRAN_24X8_LIB8
#else
	CALL(inner_tran_24x8_lib8)
#endif


	// prefetch D
	// TODO prefetch vs !!!

//	movq	ARG9, %r10 // D
//	movq	ARG10, %r11 // ldd
//	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
//	INNER_PREFETCH0_24X8_LIB
#else
//	CALL(inner_prefetch0_24x8_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG12, %r14 // m1
	movq	ARG13, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X24_VS_LIB
#else
	CALL(inner_scale_ab_8x24_vs_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_8X24_VS_LIB
#else
	CALL(inner_store_8x24_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_8x24_vs_libc8cc)





//                                  1      2              3          4        5          6        7             8          9        10         11
// void kernel_dgemm_nn_24x8_lib8ccc(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nn_24x8_lib8ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 6, %r12d // 8*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_24X8_LIB8C
#else
	CALL(inner_kernel_dgemm_nn_24x8_lib8c)
#endif


	// prefetch D

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_24X8_LIB
#else
	CALL(inner_prefetch0_24x8_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_24X8_LIB
#else
	CALL(inner_scale_ab_24x8_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_24X8_LIB
#else
	CALL(inner_store_24x8_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nn_24x8_lib8ccc)





//                                      1      2              3          4        5          6        7             8          9        10         11       12      13
// void kernel_dgemm_nn_24x8_vs_lib8ccc(int k, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nn_24x8_vs_lib8ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 6, %r12d // 8*sda*sizeof(double)
	movq	ARG5, %r13  // B
	movq	ARG6, %r14  // ldb
	sall	$ 3, %r14d
	movq	ARG12, %r15 // m1
	movq	ARG13, %rax // n1

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_24X8_VS_LIB8C
#else
	CALL(inner_kernel_dgemm_nn_24x8_vs_lib8c)
#endif


	// prefetch D
	// TODO prefetch vs !!!

//	movq	ARG9, %r10 // D
//	movq	ARG10, %r11 // ldd
//	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
//	INNER_PREFETCH0_24X8_LIB
#else
//	CALL(inner_prefetch0_24x8_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG12, %r14 // m1
	movq	ARG13, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_24X8_VS_LIB
#else
	CALL(inner_scale_ab_24x8_vs_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_24X8_VS_LIB
#else
	CALL(inner_store_24x8_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nn_24x8_vs_lib8ccc)





//                                  1      2              3          4        5          6        7             8          9        10         11
// void kernel_dgemm_tt_8x24_libc8cc(int k, double *alpha, double *A, int lda, double *B, int sdb, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_tt_8x24_libc8cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG5, %r11  // B
	movq	ARG6, %r12 // sdb
	sall	$ 6, %r12d // 8*sdb*sizeof(double)
	movq	ARG3, %r13  // A
	movq	ARG4, %r14  // lda
	sall	$ 3, %r14d

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_24X8_LIB8C
#else
	CALL(inner_kernel_dgemm_nn_24x8_lib8c)
#endif

#if MACRO_LEVEL>=1
	INNER_TRAN_24X8_LIB8
#else
	CALL(inner_tran_24x8_lib8)
#endif


	// prefetch D

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_8X24_LIB
#else
	CALL(inner_prefetch0_8x24_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X24_LIB
#else
	CALL(inner_scale_ab_8x24_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_8X24_LIB
#else
	CALL(inner_store_8x24_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_tt_8x24_libc8cc)






//                                      1      2              3          4        5          6        7             8          9        10         11       12      13
// void kernel_dgemm_tt_8x24_vs_libc8cc(int k, double *alpha, double *A, int lda, double *B, int sdb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_tt_8x24_vs_libc8cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG5, %r11  // B
	movq	ARG6, %r12 // sdb
	sall	$ 6, %r12d // 8*sdb*sizeof(double)
	movq	ARG3, %r13  // A
	movq	ARG4, %r14  // lda
	sall	$ 3, %r14d
	movq	ARG13, %r15 // n1
	movq	ARG12, %rax // m1

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NN_24X8_VS_LIB8C
#else
	CALL(inner_kernel_dgemm_nn_24x8_vs_lib8c)
#endif

#if MACRO_LEVEL>=1
	INNER_TRAN_24X8_LIB8
#else
	CALL(inner_tran_24x8_lib8)
#endif


	// prefetch D
	// TODO prefetch vs !!!

//	movq	ARG9, %r10 // D
//	movq	ARG10, %r11 // ldd
//	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
//	INNER_PREFETCH0_8X24_LIB
#else
//	CALL(inner_prefetch0_8x24_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 3, %r13d
	movq	ARG12, %r14 // m1
	movq	ARG13, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X24_VS_LIB
#else
	CALL(inner_scale_ab_8x24_vs_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 3, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_8X24_VS_LIB
#else
	CALL(inner_store_8x24_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_tt_8x24_vs_libc8cc)






