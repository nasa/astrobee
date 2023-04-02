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
// r10d   <- k
// r11   <- A
// r12   <- B
// r13   <- ldb
// ymm0  <- [d00 d10 d20 d30 d40 d50 d60 d70]
// ymm1  <- [d01 d11 d21 d31 d41 d51 d61 d71]
// ymm2  <- [d02 d12 d22 d32 d42 d52 d62 d72]
// ymm3  <- [d03 d13 d23 d33 d43 d53 d63 d73]

//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NT_8X4_LIB8C
#else
	FUN_START(inner_kernel_gemm_nt_8x4_lib8c)
#endif
	
// broadcast scheme

	cmpl	$ 0, %r10d
	jle		5f // return

	// prefetch
//	prefetcht0		0(%r12) // software prefetch
//	prefetcht0		0+64(%r12) // software prefetch

	// preload
	vmovaps 		0(%r11), %ymm13 // A

	vxorps			%ymm4, %ymm4, %ymm4
	vmovaps			%ymm4, %ymm5
	vmovaps			%ymm4, %ymm6
	vmovaps			%ymm4, %ymm7

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop
	
	// unroll 0
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vmovaps			32(%r11), %ymm14 // A
	vbroadcastss	4(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r12, %r13, 8)
	vbroadcastss	8(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vbroadcastss	12(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	addq	%r13, %r12

	// unroll 0
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vmovaps			64(%r11), %ymm13 // A
	vbroadcastss	4(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	subl	$ 4, %r10d
	vbroadcastss	8(%r12), %ymm12 // B
	prefetcht0		0(%r12, %r13, 8)
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	addq	$ 128, %r11
	vbroadcastss	12(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r13, %r12

	// unroll 0
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vmovaps			-32(%r11), %ymm14 // A
	vbroadcastss	4(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r12, %r13, 8)
	vbroadcastss	8(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vbroadcastss	12(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	addq	%r13, %r12

	// unroll 0
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vmovaps			0(%r11), %ymm13 // A
	vbroadcastss	4(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	prefetcht0		0(%r12, %r13, 8)
	vbroadcastss	8(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vbroadcastss	12(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r13, %r12

	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1


	// unroll 0
	vbroadcastss	0(%r12), %ymm12 // b
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vmovaps			32(%r11), %ymm14 // a
	vbroadcastss	4(%r12), %ymm12 // b
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vbroadcastss	8(%r12), %ymm12 // b
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vbroadcastss	12(%r12), %ymm12 // b
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	addq	%r13, %r12

	// unroll 0
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vmovaps			64(%r11), %ymm13 // A
	vbroadcastss	4(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	subl	$ 4, %r10d
	vbroadcastss	8(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	addq	$ 128, %r11
	vbroadcastss	12(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r13, %r12

	// unroll 0
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vmovaps			-32(%r11), %ymm14 // A
	vbroadcastss	4(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vbroadcastss	8(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vbroadcastss	12(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	addq	%r13, %r12

	// unroll 0
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm4
//	vmovaps			0(%r11), %ymm13 // A
	vbroadcastss	4(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	8(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vbroadcastss	12(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r13, %r12

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovaps			0(%r11), %ymm13 // A
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vbroadcastss	4(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	subl	$ 1, %r10d
	vbroadcastss	8(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	addq	$ 32, %r11
	vbroadcastss	12(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	addq	%r13, %r12

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // reduce

	vaddps			%ymm4, %ymm0, %ymm0
	vaddps			%ymm5, %ymm1, %ymm1
	vaddps			%ymm6, %ymm2, %ymm2
	vaddps			%ymm7, %ymm3, %ymm3

5: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nt_8x4_lib8c)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d   <- k
// r11   <- A
// r12   <- B
// r13   <- ldb
// ymm0  <- [d00 d10 d20 d30 d40 d50 d60 d70]
// ymm1  <- [d01 d11 d21 d31 d41 d51 d61 d71]
// ymm2  <- [d02 d12 d22 d32 d42 d52 d62 d72]
// ymm3  <- [d03 d13 d23 d33 d43 d53 d63 d73]

//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NT_8X3_LIB8C
#else
	FUN_START(inner_kernel_gemm_nt_8x3_lib8c)
#endif
	
// broadcast scheme

	cmpl	$ 0, %r10d
	jle		5f // return

	// prefetch
//	prefetcht0		0(%r12) // software prefetch
//	prefetcht0		0+64(%r12) // software prefetch

	// preload
	vmovaps 		0(%r11), %ymm13 // A

	vxorps			%ymm4, %ymm4, %ymm4
	vmovaps			%ymm4, %ymm5
	vmovaps			%ymm4, %ymm6
//	vmovaps			%ymm4, %ymm7

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop
	
	// unroll 0
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vmovaps			32(%r11), %ymm14 // A
	vbroadcastss	4(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	prefetcht0	128(%r12) // software prefetch
	vbroadcastss	8(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	prefetcht0	128+64(%r12) // software prefetch
//	vbroadcastss	12(%r12), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
	addq	%r13, %r12

	// unroll 0
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vmovaps			64(%r11), %ymm13 // A
	vbroadcastss	4(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	subl	$ 4, %r10d
	vbroadcastss	8(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	addq	$ 128, %r11
//	vbroadcastss	12(%r12), %ymm12 // B
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r13, %r12

	// unroll 0
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vmovaps			-32(%r11), %ymm14 // A
	vbroadcastss	4(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vbroadcastss	8(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vbroadcastss	12(%r12), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
	addq	%r13, %r12

	// unroll 0
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vmovaps			0(%r11), %ymm13 // A
	vbroadcastss	4(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	8(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r12), %ymm12 // B
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r13, %r12

	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1


	// unroll 0
	vbroadcastss	0(%r12), %ymm12 // b
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vmovaps			32(%r11), %ymm14 // a
	vbroadcastss	4(%r12), %ymm12 // b
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vbroadcastss	8(%r12), %ymm12 // b
	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vbroadcastss	12(%r12), %ymm12 // b
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
	addq	%r13, %r12

	// unroll 0
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vmovaps			64(%r11), %ymm13 // A
	vbroadcastss	4(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	subl	$ 4, %r10d
	vbroadcastss	8(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	addq	$ 128, %r11
//	vbroadcastss	12(%r12), %ymm12 // B
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r13, %r12

	// unroll 0
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vmovaps			-32(%r11), %ymm14 // A
	vbroadcastss	4(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vbroadcastss	8(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vbroadcastss	12(%r12), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
	addq	%r13, %r12

	// unroll 0
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm4
//	vmovaps			0(%r11), %ymm13 // A
	vbroadcastss	4(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vbroadcastss	8(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r12), %ymm12 // B
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r13, %r12

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovaps			0(%r11), %ymm13 // a
	vbroadcastss	0(%r12), %ymm12 // b
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vbroadcastss	4(%r12), %ymm12 // b
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	subl	$ 1, %r10d
	vbroadcastss	8(%r12), %ymm12 // b
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	addq	$ 32, %r11
//	vbroadcastss	12(%r12), %ymm12 // b
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
	addq	%r13, %r12

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // reduce

	vaddps			%ymm4, %ymm0, %ymm0
	vaddps			%ymm5, %ymm1, %ymm1
	vaddps			%ymm6, %ymm2, %ymm2
//	vaddps			%ymm7, %ymm3, %ymm3

5: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nt_8x3_lib8c)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d   <- k
// r11   <- A
// r12   <- B
// r13   <- ldb
// ymm0  <- [d00 d10 d20 d30 d40 d50 d60 d70]
// ymm1  <- [d01 d11 d21 d31 d41 d51 d61 d71]
// ymm2  <- [d02 d12 d22 d32 d42 d52 d62 d72]
// ymm3  <- [d03 d13 d23 d33 d43 d53 d63 d73]

//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NT_8X2_LIB8C
#else
	FUN_START(inner_kernel_gemm_nt_8x2_lib8c)
#endif
	
// broadcast scheme

	cmpl	$ 0, %r10d
	jle		5f // return

	// prefetch
//	prefetcht0		0(%r12) // software prefetch
//	prefetcht0		0+64(%r12) // software prefetch

	// preload
	vmovaps 		0(%r11), %ymm13 // A

	vxorps			%ymm4, %ymm4, %ymm4
	vmovaps			%ymm4, %ymm5
//	vmovaps			%ymm4, %ymm6
//	vmovaps			%ymm4, %ymm7

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop
	
	// unroll 0
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vmovaps			32(%r11), %ymm14 // A
	vbroadcastss	4(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	prefetcht0	128(%r12) // software prefetch
//	vbroadcastss	8(%r12), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	prefetcht0	128+64(%r12) // software prefetch
//	vbroadcastss	12(%r12), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
	addq	%r13, %r12

	// unroll 0
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vmovaps			64(%r11), %ymm13 // A
	vbroadcastss	4(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	subl	$ 4, %r10d
//	vbroadcastss	8(%r12), %ymm12 // B
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
	addq	$ 128, %r11
//	vbroadcastss	12(%r12), %ymm12 // B
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r13, %r12

	// unroll 0
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vmovaps			-32(%r11), %ymm14 // A
	vbroadcastss	4(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vbroadcastss	8(%r12), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vbroadcastss	12(%r12), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
	addq	%r13, %r12

	// unroll 0
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vmovaps			0(%r11), %ymm13 // A
	vbroadcastss	4(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	8(%r12), %ymm12 // B
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r12), %ymm12 // B
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r13, %r12

	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1


	// unroll 0
	vbroadcastss	0(%r12), %ymm12 // b
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vmovaps			32(%r11), %ymm14 // a
	vbroadcastss	4(%r12), %ymm12 // b
	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vbroadcastss	8(%r12), %ymm12 // b
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vbroadcastss	12(%r12), %ymm12 // b
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
	addq	%r13, %r12

	// unroll 0
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vmovaps			64(%r11), %ymm13 // A
	vbroadcastss	4(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	subl	$ 4, %r10d
//	vbroadcastss	8(%r12), %ymm12 // B
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
	addq	$ 128, %r11
//	vbroadcastss	12(%r12), %ymm12 // B
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r13, %r12

	// unroll 0
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vmovaps			-32(%r11), %ymm14 // A
	vbroadcastss	4(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vbroadcastss	8(%r12), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vbroadcastss	12(%r12), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
	addq	%r13, %r12

	// unroll 0
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm4
//	vmovaps			0(%r11), %ymm13 // A
	vbroadcastss	4(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	8(%r12), %ymm12 // B
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r12), %ymm12 // B
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r13, %r12

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovaps			0(%r11), %ymm13 // a
	vbroadcastss	0(%r12), %ymm12 // b
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vbroadcastss	4(%r12), %ymm12 // b
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	subl	$ 1, %r10d
//	vbroadcastss	8(%r12), %ymm12 // b
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
	addq	$ 32, %r11
//	vbroadcastss	12(%r12), %ymm12 // b
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
	addq	%r13, %r12

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // reduce

	vaddps			%ymm4, %ymm0, %ymm0
	vaddps			%ymm5, %ymm1, %ymm1
//	vaddps			%ymm6, %ymm2, %ymm2
//	vaddps			%ymm7, %ymm3, %ymm3

5: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nt_8x2_lib8c)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d   <- k
// r11   <- A
// r12   <- B
// r13   <- ldb
// ymm0  <- [d00 d10 d20 d30 d40 d50 d60 d70]
// ymm1  <- [d01 d11 d21 d31 d41 d51 d61 d71]
// ymm2  <- [d02 d12 d22 d32 d42 d52 d62 d72]
// ymm3  <- [d03 d13 d23 d33 d43 d53 d63 d73]

//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NT_8X1_LIB8C
#else
	FUN_START(inner_kernel_gemm_nt_8x1_lib8c)
#endif
	
// broadcast scheme

	cmpl	$ 0, %r10d
	jle		5f // return

	// prefetch
//	prefetcht0		0(%r12) // software prefetch
//	prefetcht0		0+64(%r12) // software prefetch

	// preload
	vmovaps 		0(%r11), %ymm13 // A

	vxorps			%ymm4, %ymm4, %ymm4
//	vmovaps			%ymm4, %ymm5
//	vmovaps			%ymm4, %ymm6
//	vmovaps			%ymm4, %ymm7

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop
	
	// unroll 0
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vmovaps			32(%r11), %ymm14 // A
//	vbroadcastss	4(%r12), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	prefetcht0	128(%r12) // software prefetch
//	vbroadcastss	8(%r12), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	prefetcht0	128+64(%r12) // software prefetch
//	vbroadcastss	12(%r12), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
	addq	%r13, %r12

	// unroll 0
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vmovaps			64(%r11), %ymm13 // A
//	vbroadcastss	4(%r12), %ymm12 // B
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
	subl	$ 4, %r10d
//	vbroadcastss	8(%r12), %ymm12 // B
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
	addq	$ 128, %r11
//	vbroadcastss	12(%r12), %ymm12 // B
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r13, %r12

	// unroll 0
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vmovaps			-32(%r11), %ymm14 // A
//	vbroadcastss	4(%r12), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vbroadcastss	8(%r12), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vbroadcastss	12(%r12), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
	addq	%r13, %r12

	// unroll 0
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vmovaps			0(%r11), %ymm13 // A
//	vbroadcastss	4(%r12), %ymm12 // B
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	8(%r12), %ymm12 // B
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r12), %ymm12 // B
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r13, %r12

	cmpl	$ 4, %r10d
	jg		1b // main loop 


0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1


	// unroll 0
	vbroadcastss	0(%r12), %ymm12 // b
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vmovaps			32(%r11), %ymm14 // a
//	vbroadcastss	4(%r12), %ymm12 // b
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vbroadcastss	8(%r12), %ymm12 // b
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vbroadcastss	12(%r12), %ymm12 // b
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
	addq	%r13, %r12

	// unroll 0
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vmovaps			64(%r11), %ymm13 // A
//	vbroadcastss	4(%r12), %ymm12 // B
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
	subl	$ 4, %r10d
//	vbroadcastss	8(%r12), %ymm12 // B
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
	addq	$ 128, %r11
//	vbroadcastss	12(%r12), %ymm12 // B
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r13, %r12

	// unroll 0
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vmovaps			-32(%r11), %ymm14 // A
//	vbroadcastss	4(%r12), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vbroadcastss	8(%r12), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vbroadcastss	12(%r12), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
	addq	%r13, %r12

	// unroll 0
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm14, %ymm12, %ymm4
//	vmovaps			0(%r11), %ymm13 // A
//	vbroadcastss	4(%r12), %ymm12 // B
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vbroadcastss	8(%r12), %ymm12 // B
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vbroadcastss	12(%r12), %ymm12 // B
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
	addq	%r13, %r12

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovaps			0(%r11), %ymm13 // a
	vbroadcastss	0(%r12), %ymm12 // b
	vfmadd231ps		%ymm13, %ymm12, %ymm0
//	vbroadcastss	4(%r12), %ymm12 // b
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
	subl	$ 1, %r10d
//	vbroadcastss	8(%r12), %ymm12 // b
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
	addq	$ 32, %r11
//	vbroadcastss	12(%r12), %ymm12 // b
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
	addq	%r13, %r12

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // reduce

	vaddps			%ymm4, %ymm0, %ymm0
//	vaddps			%ymm5, %ymm1, %ymm1
//	vaddps			%ymm6, %ymm2, %ymm2
//	vaddps			%ymm7, %ymm3, %ymm3

5: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nt_8x1_lib8c)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// r13   <- ldb
// ymm0  <- []
// ymm1  <- []
// ymm2  <- []
// ymm3  <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_8X4_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemm_nn_8x4_lib8c)
#endif

	cmpl	$ 0, %r10d
	jle		5f // return

	leaq	0(%r13, %r13, 2), %r15

	// preload

	vxorps			%ymm4, %ymm4, %ymm4
	vmovaps			%ymm4, %ymm5
	vmovaps			%ymm4, %ymm6
	vmovaps			%ymm4, %ymm7

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0	0(%r12, %r13, 2) // software prefetch
//	prefetcht0	64(%r12, %r13, 2) // software prefetch

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vbroadcastss	0(%r12, %r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vbroadcastss	0(%r12, %r13, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vbroadcastss	0(%r12, %r15), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vbroadcastss	4(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm4
	vbroadcastss	4(%r12, %r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm5
	vbroadcastss	4(%r12, %r13, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm6
	vbroadcastss	4(%r12, %r15), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm7

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vbroadcastss	8(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vbroadcastss	8(%r12, %r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vbroadcastss	8(%r12, %r13, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vbroadcastss	8(%r12, %r15), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vbroadcastss	12(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm4
	vbroadcastss	12(%r12, %r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm5
	vbroadcastss	12(%r12, %r13, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm6
	vbroadcastss	12(%r12, %r15), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm7

	subl	$ 4, %r10d
	addq	$ 16, %r12
	addq	$ 128, %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vbroadcastss	0(%r12, %r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vbroadcastss	0(%r12, %r13, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vbroadcastss	0(%r12, %r15), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vbroadcastss	4(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm4
	vbroadcastss	4(%r12, %r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm5
	vbroadcastss	4(%r12, %r13, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm6
	vbroadcastss	4(%r12, %r15), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm7

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vbroadcastss	8(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vbroadcastss	8(%r12, %r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vbroadcastss	8(%r12, %r13, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vbroadcastss	8(%r12, %r15), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vbroadcastss	12(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm4
	vbroadcastss	12(%r12, %r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm5
	vbroadcastss	12(%r12, %r13, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm6
	vbroadcastss	12(%r12, %r15), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm7

	subl	$ 4, %r10d
	addq	$ 16, %r12
	addq	$ 128, %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vbroadcastss	0(%r12, %r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vbroadcastss	0(%r12, %r13, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vbroadcastss	0(%r12, %r15), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3

	subl	$ 1, %r10d
	addq	$ 4, %r12
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

	vaddps			%ymm4, %ymm0, %ymm0
	vaddps			%ymm5, %ymm1, %ymm1
	vaddps			%ymm6, %ymm2, %ymm2
	vaddps			%ymm7, %ymm3, %ymm3

5: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nn_8x4_lib8c)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// r13   <- ldb
// ymm0  <- []
// ymm1  <- []
// ymm2  <- []
// ymm3  <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_8X3_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemm_nn_8x3_lib8c)
#endif

	cmpl	$ 0, %r10d
	jle		5f // return

	leaq	0(%r13, %r13, 2), %r15

	// preload

	vxorps			%ymm4, %ymm4, %ymm4
	vmovaps			%ymm4, %ymm5
	vmovaps			%ymm4, %ymm6
//	vmovaps			%ymm4, %ymm7

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0	0(%r12, %r13, 2) // software prefetch
//	prefetcht0	64(%r12, %r13, 2) // software prefetch

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vbroadcastss	0(%r12, %r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vbroadcastss	0(%r12, %r13, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vbroadcastss	0(%r12, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vbroadcastss	4(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm4
	vbroadcastss	4(%r12, %r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm5
	vbroadcastss	4(%r12, %r13, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm6
//	vbroadcastss	4(%r12, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm7

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vbroadcastss	8(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vbroadcastss	8(%r12, %r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vbroadcastss	8(%r12, %r13, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vbroadcastss	8(%r12, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vbroadcastss	12(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm4
	vbroadcastss	12(%r12, %r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm5
	vbroadcastss	12(%r12, %r13, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm6
//	vbroadcastss	12(%r12, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm7

	subl	$ 4, %r10d
	addq	$ 16, %r12
	addq	$ 128, %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vbroadcastss	0(%r12, %r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vbroadcastss	0(%r12, %r13, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vbroadcastss	0(%r12, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vbroadcastss	4(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm4
	vbroadcastss	4(%r12, %r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm5
	vbroadcastss	4(%r12, %r13, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm6
//	vbroadcastss	4(%r12, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm7

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vbroadcastss	8(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vbroadcastss	8(%r12, %r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vbroadcastss	8(%r12, %r13, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vbroadcastss	8(%r12, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vbroadcastss	12(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm4
	vbroadcastss	12(%r12, %r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm5
	vbroadcastss	12(%r12, %r13, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm6
//	vbroadcastss	12(%r12, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm7

	subl	$ 4, %r10d
	addq	$ 16, %r12
	addq	$ 128, %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vbroadcastss	0(%r12, %r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vbroadcastss	0(%r12, %r13, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vbroadcastss	0(%r12, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3

	subl	$ 1, %r10d
	addq	$ 4, %r12
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

	vaddps			%ymm4, %ymm0, %ymm0
	vaddps			%ymm5, %ymm1, %ymm1
	vaddps			%ymm6, %ymm2, %ymm2
//	vaddps			%ymm7, %ymm3, %ymm3

5: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nn_8x3_lib8c)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// r13   <- ldb
// ymm0  <- []
// ymm1  <- []
// ymm2  <- []
// ymm3  <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_8X2_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemm_nn_8x2_lib8c)
#endif

	cmpl	$ 0, %r10d
	jle		5f // return

	leaq	0(%r13, %r13, 2), %r15

	// preload

	vxorps			%ymm4, %ymm4, %ymm4
	vmovaps			%ymm4, %ymm5
//	vmovaps			%ymm4, %ymm6
//	vmovaps			%ymm4, %ymm7

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0	0(%r12, %r13, 2) // software prefetch
//	prefetcht0	64(%r12, %r13, 2) // software prefetch

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vbroadcastss	0(%r12, %r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vbroadcastss	0(%r12, %r13, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vbroadcastss	0(%r12, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vbroadcastss	4(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm4
	vbroadcastss	4(%r12, %r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm5
//	vbroadcastss	4(%r12, %r13, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm6
//	vbroadcastss	4(%r12, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm7

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vbroadcastss	8(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vbroadcastss	8(%r12, %r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vbroadcastss	8(%r12, %r13, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vbroadcastss	8(%r12, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vbroadcastss	12(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm4
	vbroadcastss	12(%r12, %r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm5
//	vbroadcastss	12(%r12, %r13, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm6
//	vbroadcastss	12(%r12, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm7

	subl	$ 4, %r10d
	addq	$ 16, %r12
	addq	$ 128, %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vbroadcastss	0(%r12, %r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vbroadcastss	0(%r12, %r13, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vbroadcastss	0(%r12, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vbroadcastss	4(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm4
	vbroadcastss	4(%r12, %r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm5
//	vbroadcastss	4(%r12, %r13, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm6
//	vbroadcastss	4(%r12, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm7

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vbroadcastss	8(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vbroadcastss	8(%r12, %r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vbroadcastss	8(%r12, %r13, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vbroadcastss	8(%r12, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vbroadcastss	12(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm4
	vbroadcastss	12(%r12, %r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm5
//	vbroadcastss	12(%r12, %r13, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm6
//	vbroadcastss	12(%r12, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm7

	subl	$ 4, %r10d
	addq	$ 16, %r12
	addq	$ 128, %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vbroadcastss	0(%r12, %r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vbroadcastss	0(%r12, %r13, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vbroadcastss	0(%r12, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3

	subl	$ 1, %r10d
	addq	$ 4, %r12
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

	vaddps			%ymm4, %ymm0, %ymm0
	vaddps			%ymm5, %ymm1, %ymm1
//	vaddps			%ymm6, %ymm2, %ymm2
//	vaddps			%ymm7, %ymm3, %ymm3

5: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nn_8x2_lib8c)
#endif





// common inner routine with file scope
//
// input arguments:
// r10d  <- k
// r11   <- A
// r12   <- B
// r13   <- ldb
// ymm0  <- []
// ymm1  <- []
// ymm2  <- []
// ymm3  <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_8X1_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemm_nn_8x1_lib8c)
#endif

	cmpl	$ 0, %r10d
	jle		5f // return

	leaq	0(%r13, %r13, 2), %r15

	// preload

	vxorps			%ymm4, %ymm4, %ymm4
//	vmovaps			%ymm4, %ymm5
//	vmovaps			%ymm4, %ymm6
//	vmovaps			%ymm4, %ymm7

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop

//	prefetcht0	0(%r12, %r13, 2) // software prefetch
//	prefetcht0	64(%r12, %r13, 2) // software prefetch

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
//	vbroadcastss	0(%r12, %r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vbroadcastss	0(%r12, %r13, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vbroadcastss	0(%r12, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vbroadcastss	4(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm4
//	vbroadcastss	4(%r12, %r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm5
//	vbroadcastss	4(%r12, %r13, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm6
//	vbroadcastss	4(%r12, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm7

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vbroadcastss	8(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
//	vbroadcastss	8(%r12, %r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vbroadcastss	8(%r12, %r13, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vbroadcastss	8(%r12, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vbroadcastss	12(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm4
//	vbroadcastss	12(%r12, %r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm5
//	vbroadcastss	12(%r12, %r13, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm6
//	vbroadcastss	12(%r12, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm7

	subl	$ 4, %r10d
	addq	$ 16, %r12
	addq	$ 128, %r11

	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
//	vbroadcastss	0(%r12, %r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vbroadcastss	0(%r12, %r13, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vbroadcastss	0(%r12, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vbroadcastss	4(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm4
//	vbroadcastss	4(%r12, %r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm5
//	vbroadcastss	4(%r12, %r13, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm6
//	vbroadcastss	4(%r12, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm7

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vbroadcastss	8(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
//	vbroadcastss	8(%r12, %r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vbroadcastss	8(%r12, %r13, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vbroadcastss	8(%r12, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vbroadcastss	12(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm4
//	vbroadcastss	12(%r12, %r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm5
//	vbroadcastss	12(%r12, %r13, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm6
//	vbroadcastss	12(%r12, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm7

	subl	$ 4, %r10d
	addq	$ 16, %r12
	addq	$ 128, %r11

	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vbroadcastss	0(%r12), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
//	vbroadcastss	0(%r12, %r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vbroadcastss	0(%r12, %r13, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vbroadcastss	0(%r12, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3

	subl	$ 1, %r10d
	addq	$ 4, %r12
	addq	$ 32, %r11

	cmpl	$ 0, %r10d
	jg		3b // clean up loop


2: // return

	vaddps			%ymm4, %ymm0, %ymm0
//	vaddps			%ymm5, %ymm1, %ymm1
//	vaddps			%ymm6, %ymm2, %ymm2
//	vaddps			%ymm7, %ymm3, %ymm3

5: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nn_8x1_lib8c)
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
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []
//
// output arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- ldc
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_8X4_LIB
#else
	FUN_START(inner_scale_ab_8x4_lib)
#endif
	
	// alpha
	vbroadcastss	0(%r10), %ymm15

	vmulps		%ymm0, %ymm15, %ymm0
	vmulps		%ymm1, %ymm15, %ymm1
	vmulps		%ymm2, %ymm15, %ymm2
	vmulps		%ymm3, %ymm15, %ymm3

	// beta
	vbroadcastss	0(%r11), %ymm14

	vxorps		%ymm15, %ymm15, %ymm15 // 0.0

	vucomiss	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	vmovups		0(%r12), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm0
	addq		%r13, %r12
	vmovups		0(%r12), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm1
	addq		%r13, %r12
	vmovups		0(%r12), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm2
	addq		%r13, %r12
	vmovups		0(%r12), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm3
//	addq		%r13, %r12

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_8x4_lib)
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
// ymm0 <- [d00 d11 d22 d33]
// ymm1 <- [d01 d10 d23 d32]
// ymm2 <- [d03 d12 d21 d30]
// ymm3 <- [d02 d13 d20 d31]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_8X4_VS_LIB
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_8x4_vs_lib)
#endif

	// alpha
	vbroadcastss	0(%r10), %ymm15

	vmulps		%ymm0, %ymm15, %ymm0
	vmulps		%ymm1, %ymm15, %ymm1
	vmulps		%ymm2, %ymm15, %ymm2
	vmulps		%ymm3, %ymm15, %ymm3

	// beta
	vbroadcastss	0(%r11), %ymm14

	vxorps		%ymm15, %ymm15, %ymm15 // 0.0
	vucomiss	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end


	vcvtsi2ss	%r14d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %ymm13
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %ymm13
#endif
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%ymm15, %ymm13, %ymm13


	vmaskmovps	0(%r12), %ymm13, %ymm15
	vfmadd231ps	%ymm14, %ymm15, %ymm0
	addq		%r13, %r12
	cmpl		$ 2, %r15d
	jl			0f // end
	vmaskmovps	0(%r12), %ymm13, %ymm15
	vfmadd231ps	%ymm14, %ymm15, %ymm1
	addq		%r13, %r12
	cmpl		$ 3, %r15d
	jl			0f // end
	vmaskmovps	0(%r12), %ymm13, %ymm15
	vfmadd231ps	%ymm14, %ymm15, %ymm2
	addq		%r13, %r12
	cmpl		$ 3, %r15d
	je			0f // end
	vmaskmovps	0(%r12), %ymm13, %ymm15
	vfmadd231ps	%ymm14, %ymm15, %ymm3
//	addq		%r13, %r12

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_8x4_vs_lib)
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
//
// output arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- ldc

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_4X8_LIB
#else
	FUN_START(inner_scale_ab_8x4_lib)
#endif
	
	// alpha
	vbroadcastss	0(%r10), %xmm15

	vmulps		%xmm0, %xmm15, %xmm0
	vmulps		%xmm1, %xmm15, %xmm1
	vmulps		%xmm2, %xmm15, %xmm2
	vmulps		%xmm3, %xmm15, %xmm3
	vmulps		%xmm4, %xmm15, %xmm4
	vmulps		%xmm5, %xmm15, %xmm5
	vmulps		%xmm6, %xmm15, %xmm6
	vmulps		%xmm7, %xmm15, %xmm7

	// beta
	vbroadcastss	0(%r11), %xmm14

	vxorps		%xmm15, %xmm15, %xmm15 // 0.0

	vucomiss	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	vmovups		0(%r12), %xmm15
	vfmadd231ps	%xmm15, %xmm14, %xmm0
	addq		%r13, %r12
	vmovups		0(%r12), %xmm15
	vfmadd231ps	%xmm15, %xmm14, %xmm1
	addq		%r13, %r12
	vmovups		0(%r12), %xmm15
	vfmadd231ps	%xmm15, %xmm14, %xmm2
	addq		%r13, %r12
	vmovups		0(%r12), %xmm15
	vfmadd231ps	%xmm15, %xmm14, %xmm3
	addq		%r13, %r12
	vmovups		0(%r12), %xmm15
	vfmadd231ps	%xmm15, %xmm14, %xmm4
	addq		%r13, %r12
	vmovups		0(%r12), %xmm15
	vfmadd231ps	%xmm15, %xmm14, %xmm5
	addq		%r13, %r12
	vmovups		0(%r12), %xmm15
	vfmadd231ps	%xmm15, %xmm14, %xmm6
	addq		%r13, %r12
	vmovups		0(%r12), %xmm15
	vfmadd231ps	%xmm15, %xmm14, %xmm7
	addq		%r13, %r12

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_4x8_lib)
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
	.macro INNER_SCALE_AB_4X8_VS_LIB
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_4x8_vs_lib)
#endif

	// alpha
	vbroadcastss	0(%r10), %xmm15

	vmulps		%xmm0, %xmm15, %xmm0
	vmulps		%xmm1, %xmm15, %xmm1
	vmulps		%xmm2, %xmm15, %xmm2
	vmulps		%xmm3, %xmm15, %xmm3
	vmulps		%xmm4, %xmm15, %xmm4
	vmulps		%xmm5, %xmm15, %xmm5
	vmulps		%xmm6, %xmm15, %xmm6
	vmulps		%xmm7, %xmm15, %xmm7

	// beta
	vbroadcastss	0(%r11), %xmm14

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
	vfmadd231ps	%xmm14, %xmm15, %xmm0
	addq		%r13, %r12
	cmpl		$ 2, %r15d
	jl			0f // end
	vmaskmovps	0(%r12), %xmm13, %xmm15
	vfmadd231ps	%xmm14, %xmm15, %xmm1
	addq		%r13, %r12
	cmpl		$ 3, %r15d
	jl			0f // end
	vmaskmovps	0(%r12), %xmm13, %xmm15
	vfmadd231ps	%xmm14, %xmm15, %xmm2
	addq		%r13, %r12
	cmpl		$ 4, %r15d
	jl			0f // end
	vmaskmovps	0(%r12), %xmm13, %xmm15
	vfmadd231ps	%xmm14, %xmm15, %xmm3
	addq		%r13, %r12
	cmpl		$ 5, %r15d
	jl			0f // end
	vmaskmovps	0(%r12), %xmm13, %xmm15
	vfmadd231ps	%xmm14, %xmm15, %xmm4
	addq		%r13, %r12
	cmpl		$ 6, %r15d
	jl			0f // end
	vmaskmovps	0(%r12), %xmm13, %xmm15
	vfmadd231ps	%xmm14, %xmm15, %xmm5
	addq		%r13, %r12
	cmpl		$ 7, %r15d
	jl			0f // end
	vmaskmovps	0(%r12), %xmm13, %xmm15
	vfmadd231ps	%xmm14, %xmm15, %xmm6
	addq		%r13, %r12
	cmpl		$ 7, %r15d
	je			0f // end
	vmaskmovps	0(%r12), %xmm13, %xmm15
	vfmadd231ps	%xmm14, %xmm15, %xmm7
	addq		%r13, %r12

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_4x8_vs_lib)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// r11  <- ldd
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_8X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_store_8x4_lib)
#endif

	vmovups		%ymm0, 0(%r10)
	addq		%r11, %r10
	vmovups		%ymm1, 0(%r10)
	addq		%r11, %r10
	vmovups		%ymm2, 0(%r10)
	addq		%r11, %r10
	vmovups		%ymm3, 0(%r10)
//	addq	%r11, %r10

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_8x4_lib)
#endif





// common inner routine with file scope
//
// store n vs
//
// input arguments:
// r10  <- D
// r12  <- km
// r13  <- kn
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []
//
// output arguments:
// r10  <- D
// r12  <- km
// r13  <- kn
// ymm0 <- []
// ymm1 <- []
// ymm2 <- []
// ymm3 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_8X4_VS_LIB
#else
	FUN_START(inner_store_8x4_vs_lib)
#endif
	
	// compute mask for rows
	vcvtsi2ss	%r12d, %xmm14, %xmm14
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC00(%rip), %ymm12
#elif defined(OS_MAC)
	vmovups		LC00(%rip), %ymm12
#endif
	vshufps		$ 0x00, %xmm14, %xmm14, %xmm14
	vinsertf128	$ 0x1, %xmm14, %ymm14, %ymm14
	vsubps		%ymm14, %ymm12, %ymm14

	// offset==0
	vmaskmovps	%ymm0, %ymm14, 0(%r10)
	addq		%r11, %r10
	cmpl		$ 2, %r13d
	jl			0f // end
	vmaskmovps	%ymm1, %ymm14, 0(%r10)
	addq		%r11, %r10
	cmpl		$ 3, %r13d
	jl			0f // end
	vmaskmovps	%ymm2, %ymm14, 0(%r10)
	addq		%r11, %r10
	cmpl		$ 3, %r13d
	je			0f // end
	vmaskmovps	%ymm3, %ymm14, 0(%r10)
//	addq		%r11, %r10

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_8x4_vs_lib)
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
	.macro INNER_STORE_4X8_LIB
#else
	.p2align 4,,15
	FUN_START(inner_store_4x8_lib)
#endif

	vmovups		%xmm0, 0(%r10)
	addq		%r11, %r10
	vmovups		%xmm1, 0(%r10)
	addq		%r11, %r10
	vmovups		%xmm2, 0(%r10)
	addq		%r11, %r10
	vmovups		%xmm3, 0(%r10)
	addq		%r11, %r10
	vmovups		%xmm4, 0(%r10)
	addq		%r11, %r10
	vmovups		%xmm5, 0(%r10)
	addq		%r11, %r10
	vmovups		%xmm6, 0(%r10)
	addq		%r11, %r10
	vmovups		%xmm7, 0(%r10)
	addq		%r11, %r10

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4x8_lib)
#endif





// common inner routine with file scope
//
// store n vs
//
// input arguments:
// r10  <- D
// r11  <- ldd
// r12  <- km
// r13  <- kn
//
// output arguments:
// r10  <- D
// r11  <- ldd
// r12  <- km
// r13  <- kn

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X8_VS_LIB
#else
	FUN_START(inner_store_4x8_vs_lib)
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

	// offset==0
	vmaskmovps	%xmm0, %xmm14, 0(%r10)
	addq		%r11, %r10
	cmpl		$ 2, %r13d
	jl			0f // end
	vmaskmovps	%xmm1, %xmm14, 0(%r10)
	addq		%r11, %r10
	cmpl		$ 3, %r13d
	jl			0f // end
	vmaskmovps	%xmm2, %xmm14, 0(%r10)
	addq		%r11, %r10
	cmpl		$ 4, %r13d
	jl			0f // end
	vmaskmovps	%xmm3, %xmm14, 0(%r10)
	addq		%r11, %r10
	cmpl		$ 5, %r13d
	jl			0f // end
	vmaskmovps	%xmm4, %xmm14, 0(%r10)
	addq		%r11, %r10
	cmpl		$ 6, %r13d
	jl			0f // end
	vmaskmovps	%xmm5, %xmm14, 0(%r10)
	addq		%r11, %r10
	cmpl		$ 7, %r13d
	jl			0f // end
	vmaskmovps	%xmm6, %xmm14, 0(%r10)
	addq		%r11, %r10
	cmpl		$ 7, %r13d
	je			0f // end
	vmaskmovps	%xmm7, %xmm14, 0(%r10)
//	addq		%r11, %r10

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4x8_vs_lib)
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
	.macro INNER_PREFETCH0_8X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_prefetch0_8x4_lib)
#endif

	leaq		0(%r11, %r11, 2), %r12
	prefetcht0	0(%r10)
	prefetcht0	31(%r10)
	prefetcht0	0(%r10, %r11)
	prefetcht0	31(%r10, %r11)
	prefetcht0	0(%r10, %r11, 2)
	prefetcht0	31(%r10, %r11, 2)
	prefetcht0	0(%r10, %r12)
	prefetcht0	31(%r10, %r12)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_prefetch0_8x4_lib)
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
	.macro INNER_PREFETCH0_4X8_LIBC
#else
	.p2align 4,,15
	FUN_START(inner_prefetch0_4x8_libc)
#endif

	leaq		0(%r11, %r11, 2), %r12
	leaq		0(%r10, %r11, 4), %r13
	prefetcht0	0(%r10)
	prefetcht0	15(%r10)
	prefetcht0	0(%r10, %r11)
	prefetcht0	15(%r10, %r11)
	prefetcht0	0(%r10, %r11, 2)
	prefetcht0	15(%r10, %r11, 2)
	prefetcht0	0(%r10, %r12)
	prefetcht0	15(%r10, %r12)
	prefetcht0	0(%r13)
	prefetcht0	15(%r13)
	prefetcht0	0(%r13, %r11)
	prefetcht0	15(%r13, %r11)
	prefetcht0	0(%r13, %r11, 2)
	prefetcht0	15(%r13, %r11, 2)
	prefetcht0	0(%r13, %r12)
	prefetcht0	15(%r13, %r12)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_prefetch0_4x8_libc)
#endif





//                                 1      2             3         4         5        6            7         8        9         10
// void kernel_sgemm_nt_8x4_lib8ccc(int k, float *alpha, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_nt_8x4_lib8ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B
	movq	ARG5, %r13  // ldb
	sall	$ 2, %r13d

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_8X4_LIB8C
#else
	CALL(inner_kernel_gemm_nt_8x4_lib8c)
#endif


	// prefetch D

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_8X4_LIB
#else
	CALL(inner_prefetch0_8x4_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // ldc
	sall	$ 2, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB
#else
	CALL(inner_scale_ab_8x4_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB
#else
	CALL(inner_store_8x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_nt_8x4_lib8ccc)





//                                    1      2             3         4         5        6            7         8        9         10       11      12
// void kernel_sgemm_nt_8x4_vs_lib8ccc(int k, float *alpha, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_nt_8x4_vs_lib8ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B
	movq	ARG5, %r13  // ldb
	sall	$ 2, %r13d


	movq	ARG12, %r14  // n1
	cmpl	$ 1, %r14d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_8X1_LIB8C
#else
	CALL(inner_kernel_gemm_nt_8x1_lib8c)
#endif
	
	jmp		103f

100:

	movq	ARG12, %r14  // n1
	cmpl	$ 2, %r14d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_8X2_LIB8C
#else
	CALL(inner_kernel_gemm_nt_8x2_lib8c)
#endif

	jmp		103f

101:

	movq	ARG12, %r14  // n1
	cmpl	$ 3, %r14d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_8X3_LIB8C
#else
	CALL(inner_kernel_gemm_nt_8x3_lib8c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_8X4_LIB8C
#else
	CALL(inner_kernel_gemm_nt_8x4_lib8c)
#endif

103:


	// prefetch D
	// TODO prefetch vs !!!

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_8X4_LIB
#else
	CALL(inner_prefetch0_8x4_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // ldc
	sall	$ 2, %r13d
	movq	ARG11, %r14 // m1
	movq	ARG12, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_VS_LIB
#else
	CALL(inner_scale_ab_8x4_vs_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 2, %r11d
	movq	ARG11, %r12 // m1
	movq	ARG12, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_VS_LIB
#else
	CALL(inner_store_8x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_nt_8x4_vs_lib8ccc)





//                                 1      2             3         4         5        6            7         8        9         10
// void kernel_sgemm_nt_4x8_libc8cc(int k, float *alpha, float *A, int lda, float *B, float *beta, float *C, int ldc, float *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_nt_4x8_libc8cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG5, %r11  // B
	movq	ARG3, %r12  // A
	movq	ARG4, %r13  // lda
	sall	$ 2, %r13d

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_8X4_LIB8C
#else
	CALL(inner_kernel_gemm_nt_8x4_lib8c)
#endif


	// prefetch D

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_4X8_LIBC
#else
	CALL(inner_prefetch0_4x8_libc)
#endif


#if MACRO_LEVEL>=1
	INNER_TRAN_8X4_LIB8
#else
	CALL(inner_tran_8x4_lib8)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // ldc
	sall	$ 2, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X8_LIB
#else
	CALL(inner_scale_ab_4x8_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_LIB
#else
	CALL(inner_store_4x8_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_nt_4x8_libc8cc)





//                                    1      2             3         4         5        6            7         8        9         10       11      12
// void kernel_sgemm_nt_4x8_vs_libc8cc(int k, float *alpha, float *A, int lda, float *B, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_nt_4x8_vs_libc8cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG5, %r11  // B
	movq	ARG3, %r12  // A
	movq	ARG4, %r13  // lda
	sall	$ 2, %r13d


	movq	ARG11, %r14  // m1
	cmpl	$ 1, %r14d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_8X1_LIB8C
#else
	CALL(inner_kernel_gemm_nt_8x1_lib8c)
#endif
	
	jmp		103f

100:

	movq	ARG11, %r14  // m1
	cmpl	$ 2, %r14d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_8X2_LIB8C
#else
	CALL(inner_kernel_gemm_nt_8x2_lib8c)
#endif

	jmp		103f

101:

	movq	ARG11, %r14  // m1
	cmpl	$ 3, %r14d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_8X3_LIB8C
#else
	CALL(inner_kernel_gemm_nt_8x3_lib8c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_8X4_LIB8C
#else
	CALL(inner_kernel_gemm_nt_8x4_lib8c)
#endif

103:


	// prefetch D
	// TODO prefetch vs !!!

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_4X8_LIBC
#else
	CALL(inner_prefetch0_4x8_libc)
#endif


#if MACRO_LEVEL>=1
	INNER_TRAN_8X4_LIB8
#else
	CALL(inner_tran_8x4_lib8)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // ldc
	sall	$ 2, %r13d
	movq	ARG11, %r14 // m1
	movq	ARG12, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X8_VS_LIB
#else
	CALL(inner_scale_ab_4x8_vs_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 2, %r11d
	movq	ARG11, %r12 // m1
	movq	ARG12, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_VS_LIB
#else
	CALL(inner_store_4x8_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_nt_4x8_vs_libc8cc)





//                                  1      2             3         4         5            6         7        8         9
// void kernel_sgemm_nt_8x4_lib88cc(int k, float *alpha, float *A, float *B, float *beta, float *C, int ldc, float *D, int ldd);

	GLOB_FUN_START(kernel_sgemm_nt_8x4_lib88cc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// prefetch C

	movq		ARG5, %r10 // beta
	vmovss		0(%r10), %xmm14
	vxorps		%xmm15, %xmm15, %xmm15 // 0.0
	vucomiss	%xmm15, %xmm14 // beta==0.0 ?
	je			100f // end

	movq	ARG6, %r10 // C
	movq	ARG7, %r11 // ldc
	sall	$ 2, %r11d // ldc*sizeof(float)

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_8X4_LIB
#else
	CALL(inner_prefetch0_8x4_lib)
#endif

100:


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB8
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib8)
#endif


	// call inner scale

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12   // C
	movq	ARG7, %r13   // ldc
	sall	$ 2, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB
#else
	CALL(inner_scale_ab_8x4_lib)
#endif


	// store n

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB
#else
	CALL(inner_store_8x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_nt_8x4_lib88cc)





//                                    1      2              3          4          5             6          7        8          9        10      11
// void kernel_sgemm_nt_8x4_vs_lib88cc(int k, float *alpha, float *A, float *B, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_nt_8x4_vs_lib88cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner sgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB8
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib8)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12   // C
	movq	ARG7, %r13   // ldc
	sall	$ 2, %r13d
	movq	ARG10, %r14 // m1
	movq	ARG11, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_VS_LIB
#else
	CALL(inner_scale_ab_8x4_vs_lib)
#endif


	// store n

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // ldd
	sall	$ 2, %r11d
	movq	ARG10, %r12 // m1
	movq	ARG11, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_VS_LIB
#else
	CALL(inner_store_8x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_nt_8x4_vs_lib88cc)





//                                 1      2             3         4         5        6            7         8        9         10
// void kernel_sgemm_nn_8x4_lib8ccc(int k, float *alpha, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_nn_8x4_lib8ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B
	movq	ARG5, %r13  // ldb
	sall	$ 2, %r13d

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_8X4_LIB8C
#else
	CALL(inner_kernel_gemm_nn_8x4_lib8c)
#endif


	// prefetch D

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_8X4_LIB
#else
	CALL(inner_prefetch0_8x4_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // ldc
	sall	$ 2, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB
#else
	CALL(inner_scale_ab_8x4_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB
#else
	CALL(inner_store_8x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_nn_8x4_lib8ccc)





//                                    1      2             3         4         5        6            7         8        9         10       11      12
// void kernel_sgemm_nn_8x4_vs_lib8ccc(int k, float *alpha, float *A, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_nn_8x4_vs_lib8ccc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B
	movq	ARG5, %r13  // ldb
	sall	$ 2, %r13d


	movq	ARG12, %r14  // n1
	cmpl	$ 1, %r14d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_8X1_LIB8C
#else
	CALL(inner_kernel_gemm_nn_8x1_lib8c)
#endif
	
	jmp		103f

100:

	movq	ARG12, %r14  // n1
	cmpl	$ 2, %r14d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_8X2_LIB8C
#else
	CALL(inner_kernel_gemm_nn_8x2_lib8c)
#endif

	jmp		103f

101:

	movq	ARG12, %r14  // n1
	cmpl	$ 3, %r14d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_8X3_LIB8C
#else
	CALL(inner_kernel_gemm_nn_8x3_lib8c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_8X4_LIB8C
#else
	CALL(inner_kernel_gemm_nn_8x4_lib8c)
#endif

103:


	// prefetch D
	// TODO prefetch vs !!!

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_8X4_LIB
#else
	CALL(inner_prefetch0_8x4_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // ldc
	sall	$ 2, %r13d
	movq	ARG11, %r14 // m1
	movq	ARG12, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_VS_LIB
#else
	CALL(inner_scale_ab_8x4_vs_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 2, %r11d
	movq	ARG11, %r12 // m1
	movq	ARG12, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_VS_LIB
#else
	CALL(inner_store_8x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_nn_8x4_vs_lib8ccc)





//                                 1      2             3         4         5        6            7         8        9         10
// void kernel_sgemm_tt_4x8_libc8cc(int k, float *alpha, float *A, int lda, float *B, float *beta, float *C, int ldc, float *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_tt_4x8_libc8cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG5, %r11  // B
	movq	ARG3, %r12  // A
	movq	ARG4, %r13  // lda
	sall	$ 2, %r13d

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_8X4_LIB8C
#else
	CALL(inner_kernel_gemm_nn_8x4_lib8c)
#endif


	// prefetch D

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_4X8_LIBC
#else
	CALL(inner_prefetch0_4x8_libc)
#endif


#if MACRO_LEVEL>=1
	INNER_TRAN_8X4_LIB8
#else
	CALL(inner_tran_8x4_lib8)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // ldc
	sall	$ 2, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X8_LIB
#else
	CALL(inner_scale_ab_4x8_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_LIB
#else
	CALL(inner_store_4x8_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_tt_4x8_libc8cc)





//                                    1      2             3         4         5        6            7         8        9         10       11      12
// void kernel_sgemm_tt_4x8_vs_libc8cc(int k, float *alpha, float *A, int lda, float *B, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_tt_4x8_vs_libc8cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG5, %r11  // B
	movq	ARG3, %r12  // A
	movq	ARG4, %r13  // lda
	sall	$ 2, %r13d


	movq	ARG11, %r14  // m1
	cmpl	$ 1, %r14d
	jg		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_8X1_LIB8C
#else
	CALL(inner_kernel_gemm_nn_8x1_lib8c)
#endif
	
	jmp		103f

100:

	movq	ARG11, %r14  // m1
	cmpl	$ 2, %r14d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_8X2_LIB8C
#else
	CALL(inner_kernel_gemm_nn_8x2_lib8c)
#endif

	jmp		103f

101:

	movq	ARG11, %r14  // m1
	cmpl	$ 3, %r14d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_8X3_LIB8C
#else
	CALL(inner_kernel_gemm_nn_8x3_lib8c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_8X4_LIB8C
#else
	CALL(inner_kernel_gemm_nn_8x4_lib8c)
#endif

103:


	// prefetch D
	// TODO prefetch vs !!!

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_4X8_LIBC
#else
	CALL(inner_prefetch0_4x8_libc)
#endif


#if MACRO_LEVEL>=1
	INNER_TRAN_8X4_LIB8
#else
	CALL(inner_tran_8x4_lib8)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movq	ARG8, %r13   // ldc
	sall	$ 2, %r13d
	movq	ARG11, %r14 // m1
	movq	ARG12, %r15 // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X8_VS_LIB
#else
	CALL(inner_scale_ab_4x8_vs_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movq	ARG10, %r11 // ldd
	sall	$ 2, %r11d
	movq	ARG11, %r12 // m1
	movq	ARG12, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_VS_LIB
#else
	CALL(inner_store_4x8_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_tt_4x8_vs_libc8cc)






