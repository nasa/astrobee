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
// r12   <- 4*sda*sizeof(double)
// r13   <- B
// r14   <- A_p
// r15   <- B_p
// ymm0  <- ...
// ...
// ymm11 <- ...

//
// output arguments:
// r10d  <- 0
// r11   <- A+4*k*sizeof(double)
// r12   <- 4*sda*sizeof(double)
// r13   <- B+4*k*sizeof(double)
// r14   <- A_p
// r15   <- B_p
// ymm0  <- ...
// ...
// ymm11 <- ...


// TODO pack more prefetch in clean-4 and no need for clean-8 for 2-ahead prefetch offset !!!

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NT_24X4_P0_LIB8
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemm_nt_24x4_p0_lib8)
#endif
	
// broadcast scheme

	cmpl	$ 0, %r10d
	jle		2f // return

	// prefetch

	// preload
	vmovaps 		0(%r11), %ymm13 // A
	vmovaps 		0(%r11, %r12, 1), %ymm14 // A
	vmovaps 		0(%r11, %r12, 2), %ymm15 // A

	cmpl	$ 4, %r10d
	jle		0f // consider clean-up loop

	cmpl	$ 8, %r10d
	jle		5f // consider clean-up loop

	// main loop
	.p2align 3
1: // main loop
	
	// unroll 0
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	prefetcht0	2*128(%r11) // prefetch A
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	prefetcht0	2*128(%r11, %r12) // prefetch A
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0	2*128(%r11, %r12, 2) // prefetch A
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	prefetcht0	2*128(%r13) // prefetch B
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	subl	$ 4, %r10d
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vmovaps			32(%r11), %ymm13 // A
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vmovaps			32(%r11, %r12, 1), %ymm14 // A
	vfmadd231ps		%ymm15, %ymm12, %ymm11
	vmovaps			32(%r11, %r12, 2), %ymm15 // A

	// unroll 0
	vbroadcastss	32(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	36(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	40(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	44(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vmovaps			64(%r11), %ymm13 // A
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vmovaps			64(%r11, %r12, 1), %ymm14 // A
	vfmadd231ps		%ymm15, %ymm12, %ymm11
	vmovaps			64(%r11, %r12, 2), %ymm15 // A

	// unroll 0
	vbroadcastss	64(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	prefetcht0	2*128+64(%r11) // prefetch A
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	prefetcht0	2*128+64(%r11, %r12) // prefetch A
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	68(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0	2*128+64(%r11, %r12, 2) // prefetch A
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	prefetcht0	2*128+64(%r13) // prefetch B
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	72(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	addq	$ 128, %r11
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	76(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vmovaps			-32(%r11), %ymm13 // A
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vmovaps			-32(%r11, %r12, 1), %ymm14 // A
	vfmadd231ps		%ymm15, %ymm12, %ymm11
	vmovaps			-32(%r11, %r12, 2), %ymm15 // A

	// unroll 0
	vbroadcastss	96(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	addq	$ 128, %r13
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	-28(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	-24(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	-20(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vmovaps			0(%r11), %ymm13 // A
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vmovaps			0(%r11, %r12, 1), %ymm14 // A
	vfmadd231ps		%ymm15, %ymm12, %ymm11
	vmovaps			0(%r11, %r12, 2), %ymm15 // A


//	cmpl	$ 4, %r10d
	cmpl	$ 8, %r10d
	jg		1b // main loop



5: // consider clean8-up


	// unroll 0
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	prefetcht0	0(%r14) // prefetch A_p
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	prefetcht0	0(%r14, %r12) // prefetch A_p
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0	0(%r14, %r12, 2) // prefetch A_p
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	prefetcht0	0(%r15) // prefetch B
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	subl	$ 4, %r10d
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vmovaps			32(%r11), %ymm13 // A
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vmovaps			32(%r11, %r12, 1), %ymm14 // A
	vfmadd231ps		%ymm15, %ymm12, %ymm11
	vmovaps			32(%r11, %r12, 2), %ymm15 // A

	// unroll 0
	vbroadcastss	32(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	36(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	40(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	44(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vmovaps			64(%r11), %ymm13 // A
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vmovaps			64(%r11, %r12, 1), %ymm14 // A
	vfmadd231ps		%ymm15, %ymm12, %ymm11
	vmovaps			64(%r11, %r12, 2), %ymm15 // A

	// unroll 0
	vbroadcastss	64(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	prefetcht0	64(%r14) // prefetch A_p
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	prefetcht0	64(%r14, %r12) // prefetch A_p
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	68(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0	64(%r14, %r12, 2) // prefetch A_p
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	prefetcht0	64(%r15) // prefetch B
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	72(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	addq	$ 128, %r11
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	76(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vmovaps			-32(%r11), %ymm13 // A
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vmovaps			-32(%r11, %r12, 1), %ymm14 // A
	vfmadd231ps		%ymm15, %ymm12, %ymm11
	vmovaps			-32(%r11, %r12, 2), %ymm15 // A

	// unroll 0
	vbroadcastss	96(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	addq	$ 128, %r13
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	-28(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	-24(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	-20(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vmovaps			0(%r11), %ymm13 // A
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vmovaps			0(%r11, %r12, 1), %ymm14 // A
	vfmadd231ps		%ymm15, %ymm12, %ymm11
	vmovaps			0(%r11, %r12, 2), %ymm15 // A



0: // consider clean4-up
	
	cmpl	$ 3, %r10d
	jle		4f // clean1


	// unroll 0
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	prefetcht0	128(%r14) // prefetch A_p
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	prefetcht0	128(%r14, %r12) // prefetch A_p
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0	128(%r14, %r12, 2) // prefetch A_p
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	prefetcht0	128(%r15) // prefetch B
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	subl	$ 4, %r10d
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vmovaps			32(%r11), %ymm13 // A
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vmovaps			32(%r11, %r12, 1), %ymm14 // A
	vfmadd231ps		%ymm15, %ymm12, %ymm11
	vmovaps			32(%r11, %r12, 2), %ymm15 // A

	// unroll 0
	vbroadcastss	32(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	36(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	40(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	44(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vmovaps			64(%r11), %ymm13 // A
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vmovaps			64(%r11, %r12, 1), %ymm14 // A
	vfmadd231ps		%ymm15, %ymm12, %ymm11
	vmovaps			64(%r11, %r12, 2), %ymm15 // A

	// unroll 0
	vbroadcastss	64(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	prefetcht0	128+64(%r14) // prefetch A_p
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	prefetcht0	128+64(%r14, %r12) // prefetch A_p
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	68(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0	128+64(%r14, %r12, 2) // prefetch A_p
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	prefetcht0	128+64(%r15) // prefetch B
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	72(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	addq	$ 128, %r11
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	76(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vmovaps			-32(%r11), %ymm13 // A
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vmovaps			-32(%r11, %r12, 1), %ymm14 // A
	vfmadd231ps		%ymm15, %ymm12, %ymm11
	vmovaps			-32(%r11, %r12, 2), %ymm15 // A

	// unroll 0
	vbroadcastss	96(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	addq	$ 128, %r13
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	-28(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	-24(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	-20(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vmovaps			0(%r11), %ymm13 // A
	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vmovaps			0(%r11, %r12, 1), %ymm14 // A
	vfmadd231ps		%ymm15, %ymm12, %ymm11
//	vmovaps			0(%r11, %r12, 2), %ymm15 // A


	jmp		2f // return


4: // consider clean1-up loop

	cmpl	$ 0, %r10d
	jle		2f // return

	// clean-up loop
3: // clean up loop
	
	// unroll 0
	vmovaps			0(%r11), %ymm13 // a
	vmovaps			0(%r11, %r12, 1), %ymm14 // A
	vmovaps			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // b
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13), %ymm12 // b
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	subl	$ 1, %r10d
	vbroadcastss	8(%r13), %ymm12 // b
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	addq	$ 32, %r11
	vbroadcastss	12(%r13), %ymm12 // b
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vfmadd231ps		%ymm15, %ymm12, %ymm11
	addq	$ 32, %r13

	cmpl	$ 0, %r10d
	jg		3b // clean up loop 


2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nt_24x4_p0_lib8)
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
// ymm8  <- []
// ymm9  <- []
// ymm10 <- []
// ymm11 <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NT_24X4_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemm_nt_24x4_lib8c)
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
	vmovups			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	prefetcht0		15(%r13, %r14, 8)
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vfmadd231ps		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vmovups			32(%r11, %r12), %ymm14 // A
	vmovups			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	prefetcht0		15(%r13, %r14, 8)
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vfmadd231ps		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vmovups			64(%r11, %r12), %ymm14 // A
	vmovups			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	prefetcht0		15(%r13, %r14, 8)
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	subl	$ 4, %r10d
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vfmadd231ps		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vmovups			96(%r11, %r12), %ymm14 // A
	vmovups			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	prefetcht0		15(%r13, %r14, 8)
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	addq	$ 128, %r11
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	addq	%r14, %r13
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vfmadd231ps		%ymm15, %ymm12, %ymm11


	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vmovups			0(%r11, %r12), %ymm14 // A
	vmovups			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vfmadd231ps		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vmovups			32(%r11, %r12), %ymm14 // A
	vmovups			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vfmadd231ps		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vmovups			64(%r11, %r12), %ymm14 // A
	vmovups			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vfmadd231ps		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vmovups			96(%r11, %r12), %ymm14 // A
	vmovups			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vfmadd231ps		%ymm15, %ymm12, %ymm11
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
	vmovups			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vfmadd231ps		%ymm15, %ymm12, %ymm11
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

	FUN_END(inner_kernel_gemm_nt_24x4_lib8c)
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
// ymm8  <- []
// ymm9  <- []
// ymm10 <- []
// ymm11 <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NT_24X3_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemm_nt_24x3_lib8c)
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
	vmovups			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	prefetcht0		15(%r13, %r14, 8)
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vmovups			32(%r11, %r12), %ymm14 // A
	vmovups			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	prefetcht0		15(%r13, %r14, 8)
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vmovups			64(%r11, %r12), %ymm14 // A
	vmovups			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	prefetcht0		15(%r13, %r14, 8)
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	subl	$ 4, %r10d
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vmovups			96(%r11, %r12), %ymm14 // A
	vmovups			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	prefetcht0		15(%r13, %r14, 8)
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	addq	$ 128, %r11
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
	addq	%r14, %r13
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11


	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vmovups			0(%r11, %r12), %ymm14 // A
	vmovups			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vmovups			32(%r11, %r12), %ymm14 // A
	vmovups			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vmovups			64(%r11, %r12), %ymm14 // A
	vmovups			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vmovups			96(%r11, %r12), %ymm14 // A
	vmovups			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11
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
	vmovups			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11
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

	FUN_END(inner_kernel_gemm_nt_24x3_lib8c)
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
// ymm8  <- []
// ymm9  <- []
// ymm10 <- []
// ymm11 <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NT_24X2_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemm_nt_24x2_lib8c)
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
	vmovups			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	prefetcht0		15(%r13, %r14, 8)
	vfmadd231ps		%ymm15, %ymm12, %ymm9
//	vbroadcastss	8(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vmovups			32(%r11, %r12), %ymm14 // A
	vmovups			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	prefetcht0		15(%r13, %r14, 8)
	vfmadd231ps		%ymm15, %ymm12, %ymm9
//	vbroadcastss	8(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vmovups			64(%r11, %r12), %ymm14 // A
	vmovups			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	prefetcht0		15(%r13, %r14, 8)
	vfmadd231ps		%ymm15, %ymm12, %ymm9
//	vbroadcastss	8(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
	subl	$ 4, %r10d
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vmovups			96(%r11, %r12), %ymm14 // A
	vmovups			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r13, %r14, 8)
	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	prefetcht0		15(%r13, %r14, 8)
	vfmadd231ps		%ymm15, %ymm12, %ymm9
//	vbroadcastss	8(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
	addq	$ 128, %r11
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
	addq	%r14, %r13
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11


	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vmovups			0(%r11, %r12), %ymm14 // A
	vmovups			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
//	vbroadcastss	8(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vmovups			32(%r11, %r12), %ymm14 // A
	vmovups			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
//	vbroadcastss	8(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vmovups			64(%r11, %r12), %ymm14 // A
	vmovups			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
//	vbroadcastss	8(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vmovups			96(%r11, %r12), %ymm14 // A
	vmovups			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
//	vbroadcastss	8(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11
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
	vmovups			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
//	vbroadcastss	8(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11
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

	FUN_END(inner_kernel_gemm_nt_24x2_lib8c)
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
// ymm8  <- []
// ymm9  <- []
// ymm10 <- []
// ymm11 <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NT_24X1_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemm_nt_24x1_lib8c)
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
	vmovups			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
//	vbroadcastss	4(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r13, %r14, 8)
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	prefetcht0		15(%r13, %r14, 8)
//	vfmadd231ps		%ymm15, %ymm12, %ymm9
//	vbroadcastss	8(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vmovups			32(%r11, %r12), %ymm14 // A
	vmovups			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
//	vbroadcastss	4(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r13, %r14, 8)
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	prefetcht0		15(%r13, %r14, 8)
//	vfmadd231ps		%ymm15, %ymm12, %ymm9
//	vbroadcastss	8(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vmovups			64(%r11, %r12), %ymm14 // A
	vmovups			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
//	vbroadcastss	4(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r13, %r14, 8)
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	prefetcht0		15(%r13, %r14, 8)
//	vfmadd231ps		%ymm15, %ymm12, %ymm9
//	vbroadcastss	8(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
	subl	$ 4, %r10d
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vmovups			96(%r11, %r12), %ymm14 // A
	vmovups			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
//	vbroadcastss	4(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		0(%r13, %r14, 8)
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	prefetcht0		15(%r13, %r14, 8)
//	vfmadd231ps		%ymm15, %ymm12, %ymm9
//	vbroadcastss	8(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
	addq	$ 128, %r11
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
	addq	%r14, %r13
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11


	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vmovups			0(%r11, %r12), %ymm14 // A
	vmovups			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
//	vbroadcastss	4(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vfmadd231ps		%ymm15, %ymm12, %ymm9
//	vbroadcastss	8(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vmovups			32(%r11, %r12), %ymm14 // A
	vmovups			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
//	vbroadcastss	4(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vfmadd231ps		%ymm15, %ymm12, %ymm9
//	vbroadcastss	8(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vmovups			64(%r11, %r12), %ymm14 // A
	vmovups			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
//	vbroadcastss	4(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vfmadd231ps		%ymm15, %ymm12, %ymm9
//	vbroadcastss	8(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11
	addq	%r14, %r13

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vmovups			96(%r11, %r12), %ymm14 // A
	vmovups			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
//	vbroadcastss	4(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vfmadd231ps		%ymm15, %ymm12, %ymm9
//	vbroadcastss	8(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11
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
	vmovups			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
//	vbroadcastss	4(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vfmadd231ps		%ymm15, %ymm12, %ymm9
//	vbroadcastss	8(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	12(%r13), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11
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

	FUN_END(inner_kernel_gemm_nt_24x1_lib8c)
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
// ymm7  <- []
// ymm8  <- []
// ymm9  <- []
// ymm10 <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_24X4_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemm_nn_24x4_lib8c)
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


	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vmovups			0(%r11, %r12), %ymm14 // A
	vmovups			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	prefetcht0		1*16(%r13)
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	0(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		1*16(%r13, %r14)
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	prefetcht0		1*16(%r13, %r14, 2)
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	0(%r13, %r15), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	prefetcht0		1*16(%r13, %r15)
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vfmadd231ps		%ymm15, %ymm12, %ymm11

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vmovups			32(%r11, %r12), %ymm14 // A
	vmovups			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	4(%r13, %r14, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	subl	$ 4, %r10d
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	4(%r13, %r15), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vfmadd231ps		%ymm15, %ymm12, %ymm11

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vmovups			64(%r11, %r12), %ymm14 // A
	vmovups			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	8(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	8(%r13, %r14, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	8(%r13, %r15), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vfmadd231ps		%ymm15, %ymm12, %ymm11

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vmovups			96(%r11, %r12), %ymm14 // A
	vmovups			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	addq	$ 128, %r11
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	12(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	12(%r13, %r14, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	12(%r13, %r15), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	addq	$ 16, %r13
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vfmadd231ps		%ymm15, %ymm12, %ymm11


	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vmovups			0(%r11, %r12), %ymm14 // A
	vmovups			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	0(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	0(%r13, %r15), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vfmadd231ps		%ymm15, %ymm12, %ymm11

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vmovups			32(%r11, %r12), %ymm14 // A
	vmovups			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	4(%r13, %r14, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	4(%r13, %r15), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vfmadd231ps		%ymm15, %ymm12, %ymm11

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vmovups			64(%r11, %r12), %ymm14 // A
	vmovups			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	8(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	8(%r13, %r14, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	8(%r13, %r15), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vfmadd231ps		%ymm15, %ymm12, %ymm11

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vmovups			96(%r11, %r12), %ymm14 // A
	vmovups			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	12(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	12(%r13, %r14, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	12(%r13, %r15), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vfmadd231ps		%ymm15, %ymm12, %ymm11

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
	vmovups			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	0(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
	vbroadcastss	0(%r13, %r15), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm3
	vfmadd231ps		%ymm14, %ymm12, %ymm7
	vfmadd231ps		%ymm15, %ymm12, %ymm11

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

	FUN_END(inner_kernel_gemm_nn_24x4_lib8c)
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
// ymm7  <- []
// ymm8  <- []
// ymm9  <- []
// ymm10 <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_24X3_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemm_nn_24x3_lib8c)
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


	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vmovups			0(%r11, %r12), %ymm14 // A
	vmovups			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	prefetcht0		1*16(%r13)
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	0(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		1*16(%r13, %r14)
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	prefetcht0		1*16(%r13, %r14, 2)
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	0(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	prefetcht0		1*16(%r13, %r15)
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vmovups			32(%r11, %r12), %ymm14 // A
	vmovups			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	4(%r13, %r14, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	subl	$ 4, %r10d
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	4(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vmovups			64(%r11, %r12), %ymm14 // A
	vmovups			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	8(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	8(%r13, %r14, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	8(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vmovups			96(%r11, %r12), %ymm14 // A
	vmovups			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	addq	$ 128, %r11
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	12(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	12(%r13, %r14, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	12(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
	addq	$ 16, %r13
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11


	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vmovups			0(%r11, %r12), %ymm14 // A
	vmovups			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	0(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	0(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vmovups			32(%r11, %r12), %ymm14 // A
	vmovups			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	4(%r13, %r14, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	4(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vmovups			64(%r11, %r12), %ymm14 // A
	vmovups			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	8(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	8(%r13, %r14, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	8(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vmovups			96(%r11, %r12), %ymm14 // A
	vmovups			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	12(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	12(%r13, %r14, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	12(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11

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
	vmovups			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	0(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
	vbroadcastss	0(%r13, %r14, 2), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm2
	vfmadd231ps		%ymm14, %ymm12, %ymm6
	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	0(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11

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

	FUN_END(inner_kernel_gemm_nn_24x3_lib8c)
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
// ymm7  <- []
// ymm8  <- []
// ymm9  <- []
// ymm10 <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_24X2_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemm_nn_24x2_lib8c)
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


	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vmovups			0(%r11, %r12), %ymm14 // A
	vmovups			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	prefetcht0		1*16(%r13)
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	0(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	prefetcht0		1*16(%r13, %r14)
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
//	vbroadcastss	0(%r13, %r14, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	prefetcht0		1*16(%r13, %r14, 2)
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	0(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	prefetcht0		1*16(%r13, %r15)
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vmovups			32(%r11, %r12), %ymm14 // A
	vmovups			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
//	vbroadcastss	4(%r13, %r14, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
	subl	$ 4, %r10d
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	4(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vmovups			64(%r11, %r12), %ymm14 // A
	vmovups			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	8(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
//	vbroadcastss	8(%r13, %r14, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	8(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vmovups			96(%r11, %r12), %ymm14 // A
	vmovups			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	addq	$ 128, %r11
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	12(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
//	vbroadcastss	12(%r13, %r14, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	12(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
	addq	$ 16, %r13
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11


	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vmovups			0(%r11, %r12), %ymm14 // A
	vmovups			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	0(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
//	vbroadcastss	0(%r13, %r14, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	0(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vmovups			32(%r11, %r12), %ymm14 // A
	vmovups			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	4(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
//	vbroadcastss	4(%r13, %r14, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	4(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vmovups			64(%r11, %r12), %ymm14 // A
	vmovups			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	8(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
//	vbroadcastss	8(%r13, %r14, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	8(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vmovups			96(%r11, %r12), %ymm14 // A
	vmovups			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	12(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
//	vbroadcastss	12(%r13, %r14, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	12(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11

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
	vmovups			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
	vbroadcastss	0(%r13, %r14), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm1
	vfmadd231ps		%ymm14, %ymm12, %ymm5
	vfmadd231ps		%ymm15, %ymm12, %ymm9
//	vbroadcastss	0(%r13, %r14, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	0(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11

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

	FUN_END(inner_kernel_gemm_nn_24x2_lib8c)
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
// ymm7  <- []
// ymm8  <- []
// ymm9  <- []
// ymm10 <- []
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_24X1_LIB8C
#else
	.p2align 4,,15
	FUN_START(inner_kernel_gemm_nn_24x1_lib8c)
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


	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vmovups			0(%r11, %r12), %ymm14 // A
	vmovups			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	prefetcht0		1*16(%r13)
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
//	vbroadcastss	0(%r13, %r14), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	prefetcht0		1*16(%r13, %r14)
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vfmadd231ps		%ymm15, %ymm12, %ymm9
//	vbroadcastss	0(%r13, %r14, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	prefetcht0		1*16(%r13, %r14, 2)
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	0(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	prefetcht0		1*16(%r13, %r15)
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vmovups			32(%r11, %r12), %ymm14 // A
	vmovups			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
//	vbroadcastss	4(%r13, %r14), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vfmadd231ps		%ymm15, %ymm12, %ymm9
//	vbroadcastss	4(%r13, %r14, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
	subl	$ 4, %r10d
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	4(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vmovups			64(%r11, %r12), %ymm14 // A
	vmovups			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
//	vbroadcastss	8(%r13, %r14), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vfmadd231ps		%ymm15, %ymm12, %ymm9
//	vbroadcastss	8(%r13, %r14, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	8(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vmovups			96(%r11, %r12), %ymm14 // A
	vmovups			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	addq	$ 128, %r11
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
//	vbroadcastss	12(%r13, %r14), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vfmadd231ps		%ymm15, %ymm12, %ymm9
//	vbroadcastss	12(%r13, %r14, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	12(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
	addq	$ 16, %r13
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11


	cmpl	$ 4, %r10d
	jg		1b // main loop


0: // consider clean4-up

	cmpl	$ 3, %r10d
	jle		4f // clean1

	// unroll 0
	vmovups			0(%r11), %ymm13 // A
	vmovups			0(%r11, %r12), %ymm14 // A
	vmovups			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
//	vbroadcastss	0(%r13, %r14), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vfmadd231ps		%ymm15, %ymm12, %ymm9
//	vbroadcastss	0(%r13, %r14, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	0(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11

	// unroll 1
	vmovups			32(%r11), %ymm13 // A
	vmovups			32(%r11, %r12), %ymm14 // A
	vmovups			32(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	4(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
//	vbroadcastss	4(%r13, %r14), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vfmadd231ps		%ymm15, %ymm12, %ymm9
//	vbroadcastss	4(%r13, %r14, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	4(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11

	// unroll 2
	vmovups			64(%r11), %ymm13 // A
	vmovups			64(%r11, %r12), %ymm14 // A
	vmovups			64(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	8(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
//	vbroadcastss	8(%r13, %r14), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vfmadd231ps		%ymm15, %ymm12, %ymm9
//	vbroadcastss	8(%r13, %r14, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	8(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11

	// unroll 3
	vmovups			96(%r11), %ymm13 // A
	vmovups			96(%r11, %r12), %ymm14 // A
	vmovups			96(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	12(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
//	vbroadcastss	12(%r13, %r14), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vfmadd231ps		%ymm15, %ymm12, %ymm9
//	vbroadcastss	12(%r13, %r14, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	12(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11

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
	vmovups			0(%r11, %r12, 2), %ymm15 // A
	vbroadcastss	0(%r13), %ymm12 // B
	vfmadd231ps		%ymm13, %ymm12, %ymm0
	vfmadd231ps		%ymm14, %ymm12, %ymm4
	vfmadd231ps		%ymm15, %ymm12, %ymm8
//	vbroadcastss	0(%r13, %r14), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm1
//	vfmadd231ps		%ymm14, %ymm12, %ymm5
//	vfmadd231ps		%ymm15, %ymm12, %ymm9
//	vbroadcastss	0(%r13, %r14, 2), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm2
//	vfmadd231ps		%ymm14, %ymm12, %ymm6
//	vfmadd231ps		%ymm15, %ymm12, %ymm10
//	vbroadcastss	0(%r13, %r15), %ymm12 // B
//	vfmadd231ps		%ymm13, %ymm12, %ymm3
//	vfmadd231ps		%ymm14, %ymm12, %ymm7
//	vfmadd231ps		%ymm15, %ymm12, %ymm11

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

	FUN_END(inner_kernel_gemm_nn_24x1_lib8c)
#endif





// common inner routine with file scope
//
// transpose
//
// input arguments:
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_TRAN_24X4_LIB
#else
	FUN_START(inner_tran_24x4_lib)
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

	vunpcklps	%ymm9, %ymm8, %ymm13
	vunpckhps	%ymm9, %ymm8, %ymm12
	vunpcklps	%ymm11, %ymm10, %ymm15
	vunpckhps	%ymm11, %ymm10, %ymm14

	vunpcklpd	%ymm15, %ymm13, %ymm8
	vunpckhpd	%ymm15, %ymm13, %ymm9
	vunpcklpd	%ymm14, %ymm12, %ymm10
	vunpckhpd	%ymm14, %ymm12, %ymm11

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_tran_24x4_lib)
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
// ymm0  <- []
// ...
// ymm11 <- []
//
// output arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- ldc*sizeof(float)
// ymm0  <- []
// ...
// ymm11 <- []

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_24X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_24x4_lib)
#endif
	
	// alpha
	vbroadcastss	0(%r10), %ymm12

	vmulps		%ymm0, %ymm12, %ymm0
	vmulps		%ymm1, %ymm12, %ymm1
	vmulps		%ymm2, %ymm12, %ymm2
	vmulps		%ymm3, %ymm12, %ymm3

	vmulps		%ymm4, %ymm12, %ymm4
	vmulps		%ymm5, %ymm12, %ymm5
	vmulps		%ymm6, %ymm12, %ymm6
	vmulps		%ymm7, %ymm12, %ymm7

	vmulps		%ymm8, %ymm12, %ymm8
	vmulps		%ymm9, %ymm12, %ymm9
	vmulps		%ymm10, %ymm12, %ymm10
	vmulps		%ymm11, %ymm12, %ymm11

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
	vmovups		64(%r12), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm8

	vmovups		0(%r12, %r13), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm1
	vmovups		32(%r12, %r13), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm5
	vmovups		64(%r12, %r13), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm9

	vmovups		0(%r12, %r13, 2), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm2
	vmovups		32(%r12, %r13, 2), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm6
	vmovups		64(%r12, %r13, 2), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm10

	vmovups		0(%r12, %r14), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm3
	vmovups		32(%r12, %r14), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm7
	vmovups		64(%r12, %r14), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_24x4_lib)
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
// ymm0  <- []
// ...
// ymm11 <- []
//
// output arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- ldc*sizeof(float)
// r14   <- km
// r15   <- kn
// ymm0  <- []
// ...
// ymm11 <- []

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_24X4_VS_LIB
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_24x4_vs_lib)
#endif
	
	// alpha
	vbroadcastss	0(%r10), %ymm12

	vmulps		%ymm0, %ymm12, %ymm0
	vmulps		%ymm1, %ymm12, %ymm1
	vmulps		%ymm2, %ymm12, %ymm2
	vmulps		%ymm3, %ymm12, %ymm3

	vmulps		%ymm4, %ymm12, %ymm4
	vmulps		%ymm5, %ymm12, %ymm5
	vmulps		%ymm6, %ymm12, %ymm6
	vmulps		%ymm7, %ymm12, %ymm7

	vmulps		%ymm8, %ymm12, %ymm8
	vmulps		%ymm9, %ymm12, %ymm9
	vmulps		%ymm10, %ymm12, %ymm10
	vmulps		%ymm11, %ymm12, %ymm11

	// beta
	vbroadcastss	0(%r11), %ymm14

	vxorps		%ymm15, %ymm15, %ymm15 // 0.0

	vucomiss	%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	// compute mask for rows
	vcvtsi2ss	%r14d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC02(%rip), %ymm13
#elif defined(OS_MAC)
	vmovups		LC02(%rip), %ymm13
#endif
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%ymm15, %ymm13, %ymm13

	leaq		0(%r13, %r13, 2), %rax

	vmovups		0(%r12), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm0
	vmovups		32(%r12), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm4
	vmaskmovps	64(%r12), %ymm13, %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm8
	cmpl		$ 2, %r13d
	jl			0f // end
	vmovups		0(%r12, %r13), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm1
	vmovups		32(%r12, %r13), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm5
	vmaskmovps	64(%r12, %r13), %ymm13, %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm9
	cmpl		$ 3, %r13d
	jl			0f // end
	vmovups		0(%r12, %r13, 2), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm2
	vmovups		32(%r12, %r13, 2), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm6
	vmaskmovps	64(%r12, %r13, 2), %ymm13, %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm10
	je			0f // end
	vmovups		0(%r12, %rax), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm3
	vmovups		32(%r12, %rax), %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm7
	vmaskmovps	64(%r12, %rax), %ymm13, %ymm15
	vfmadd231ps	%ymm15, %ymm14, %ymm11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_24x4_vs_lib)
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
	.macro INNER_SCALE_AB_4X24_LIB
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_4x24_lib)
#endif
	
	// alpha
	vbroadcastss	0(%r10), %ymm12

	vmulps		%ymm0, %ymm12, %ymm0
	vmulps		%ymm1, %ymm12, %ymm1
	vmulps		%ymm2, %ymm12, %ymm2
	vmulps		%ymm3, %ymm12, %ymm3

	vmulps		%ymm4, %ymm12, %ymm4
	vmulps		%ymm5, %ymm12, %ymm5
	vmulps		%ymm6, %ymm12, %ymm6
	vmulps		%ymm7, %ymm12, %ymm7

	vmulps		%ymm8, %ymm12, %ymm8
	vmulps		%ymm9, %ymm12, %ymm9
	vmulps		%ymm10, %ymm12, %ymm10
	vmulps		%ymm11, %ymm12, %ymm11

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

	leaq		0(%r12, %r13, 4), %r12

	vmovups		0(%r12), %xmm15
	vinsertf128	$ 1, 0(%r12, %r13, 4), %ymm15, %ymm15
	add			%r13, %r12
	vfmadd231ps	%ymm15, %ymm14, %ymm8
	vmovups		0(%r12), %xmm15
	vinsertf128	$ 1, 0(%r12, %r13, 4), %ymm15, %ymm15
	add			%r13, %r12
	vfmadd231ps	%ymm15, %ymm14, %ymm9
	vmovups		0(%r12), %xmm15
	vinsertf128	$ 1, 0(%r12, %r13, 4), %ymm15, %ymm15
	add			%r13, %r12
	vfmadd231ps	%ymm15, %ymm14, %ymm10
	vmovups		0(%r12), %xmm15
	vinsertf128	$ 1, 0(%r12, %r13, 4), %ymm15, %ymm15
	add			%r13, %r12
	vfmadd231ps	%ymm15, %ymm14, %ymm11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_4x24_lib)
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
	.macro INNER_SCALE_AB_4X24_VS_LIB
#else
	.p2align 4,,15
	FUN_START(inner_scale_ab_4x24_vs_lib)
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

	vmulps		%ymm8, %ymm15, %ymm8
	vmulps		%ymm9, %ymm15, %ymm9
	vmulps		%ymm10, %ymm15, %ymm10
	vmulps		%ymm11, %ymm15, %ymm11

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

	vmaskmovps	0(%r12), %xmm13, %xmm15
	vmaskmovps	0(%r12, %r13, 4), %xmm13, %xmm12
	vinsertf128	$ 1, %xmm12, %ymm15, %ymm15
	add			%r13, %r12
	vfmadd231ps	%ymm15, %ymm14, %ymm4
	vmaskmovps	0(%r12), %xmm13, %xmm15
	vmaskmovps	0(%r12, %r13, 4), %xmm13, %xmm12
	vinsertf128	$ 1, %xmm12, %ymm15, %ymm15
	add			%r13, %r12
	vfmadd231ps	%ymm15, %ymm14, %ymm5
	vmaskmovps	0(%r12), %xmm13, %xmm15
	vmaskmovps	0(%r12, %r13, 4), %xmm13, %xmm12
	vinsertf128	$ 1, %xmm12, %ymm15, %ymm15
	add			%r13, %r12
	vfmadd231ps	%ymm15, %ymm14, %ymm6
	vmaskmovps	0(%r12), %xmm13, %xmm15
	vmaskmovps	0(%r12, %r13, 4), %xmm13, %xmm12
	vinsertf128	$ 1, %xmm12, %ymm15, %ymm15
	add			%r13, %r12
	vfmadd231ps	%ymm15, %ymm14, %ymm7

	leaq		0(%r12, %r13, 4), %r12
	vxorps		%xmm12, %xmm12, %xmm12 // 0.0

	vmaskmovps	0(%r12), %xmm13, %xmm15
	cmpl		$ 13, %r15d
	jl			1f
	vmaskmovps	0(%r12, %r13, 4), %xmm13, %xmm12
1:
	vinsertf128	$ 1, %xmm12, %ymm15, %ymm15
	add			%r13, %r12
	vfmadd231ps	%ymm15, %ymm14, %ymm8
	cmpl		$ 10, %r15d
	jl			0f // end
	vmaskmovps	0(%r12), %xmm13, %xmm15
	cmpl		$ 14, %r15d
	jl			1f // end
	vmaskmovps	0(%r12, %r13, 4), %xmm13, %xmm12
1:
	vinsertf128	$ 1, %xmm12, %ymm15, %ymm15
	add			%r13, %r12
	vfmadd231ps	%ymm15, %ymm14, %ymm9
	cmpl		$ 11, %r15d
	jl			0f // end
	vmaskmovps	0(%r12), %xmm13, %xmm15
	cmpl		$ 15, %r15d
	jl			1f // end
	vmaskmovps	0(%r12, %r13, 4), %xmm13, %xmm12
1:
	vinsertf128	$ 1, %xmm12, %ymm15, %ymm15
	add			%r13, %r12
	vfmadd231ps	%ymm15, %ymm14, %ymm10
	cmpl		$ 12, %r15d
	jl			0f // end
	vmaskmovps	0(%r12), %xmm13, %xmm15
	cmpl		$ 15, %r15d
	je			1f // end
	vmaskmovps	0(%r12, %r13, 4), %xmm13, %xmm12
1:
	vinsertf128	$ 1, %xmm12, %ymm15, %ymm15
	add			%r13, %r12
	vfmadd231ps	%ymm15, %ymm14, %ymm11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_4x24_vs_lib)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// r11  <- ldd*sizeof(float)
// ymm0  <- []
// ...
// ymm11 <- []
//
// output arguments:
// r10  <- D
// r11  <- ldd*sizeof(float)
// ymm0  <- []
// ...
// ymm11 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_24X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_store_24x4_lib)
#endif
	
	leaq		0(%r11, %r11, 2), %r12

	vmovups 	%ymm0,  0(%r10)
	vmovups 	%ymm4,  32(%r10)
	vmovups 	%ymm8,  64(%r10)

	vmovups 	%ymm1,  0(%r10, %r11)
	vmovups 	%ymm5,  32(%r10, %r11)
	vmovups 	%ymm9,  64(%r10, %r11)

	vmovups 	%ymm2,  0(%r10, %r11, 2)
	vmovups 	%ymm6,  32(%r10, %r11, 2)
	vmovups 	%ymm10,  64(%r10, %r11, 2)

	vmovups 	%ymm3,  0(%r10, %r12)
	vmovups 	%ymm7,  32(%r10, %r12)
	vmovups 	%ymm11,  64(%r10, %r12)


#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_24x4_lib)
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
// ymm11 <- []
//
// output arguments:
// r10  <- D
// r11  <- 4*sdd*sizeof(double)
// r12  <- km
// r13  <- kn
// ymm0 <- []
// ...
// ymm11 <- []

#if MACRO_LEVEL>=1
	.macro INNER_STORE_24X4_VS_LIB
#else
	.p2align 4,,15
	FUN_START(inner_store_24x4_vs_lib)
#endif
	
	// compute mask for rows
	vcvtsi2ss	%r12d, %xmm15, %xmm15
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovups		.LC02(%rip), %ymm13
#elif defined(OS_MAC)
	vmovups		LC02(%rip), %ymm13
#endif
	vshufps		$ 0x00, %xmm15, %xmm15, %xmm15
	vinsertf128	$ 0x1, %xmm15, %ymm15, %ymm15
	vsubps		%ymm15, %ymm13, %ymm15

	leaq		0(%r11, %r11, 2), %r14

	vmovups		%ymm0, 0(%r10)
	vmovups		%ymm4, 32(%r10)
	vmaskmovps	%ymm8, %ymm15, 64(%r10)
	cmpl		$ 2, %r13d
	jl			0f // end
	vmovups		%ymm1, 0(%r10, %r11)
	vmovups		%ymm5, 32(%r10, %r11)
	vmaskmovps	%ymm9, %ymm15, 64(%r10, %r11)
	cmpl		$ 3, %r13d
	jl			0f // end
	vmovups		%ymm2, 0(%r10, %r11, 2)
	vmovups		%ymm6, 32(%r10, %r11, 2)
	vmaskmovps	%ymm10, %ymm15, 64(%r10, %r11, 2)
	je			0f // end
	vmovups		%ymm3, 0(%r10, %r14)
	vmovups		%ymm7, 32(%r10, %r14)
	vmaskmovps	%ymm11, %ymm15, 64(%r10, %r14)
	//
//	jmp		0f

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_24x4_vs_lib)
#endif





// common inner routine with file scope
//
// scale store for generic alpha and beta=1
//
// input arguments:
// r10   <- alpha
// r11   <- D
// r12   <- ldd
// ymm0  <- ...
// ...
// ymm11 <- ...
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_A1_STORE_24X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_scale_a1_store_24x4_lib)
#endif

	// alpha
	vbroadcastss	0(%r10), %ymm15

	vmovups		0(%r11), %ymm12
	vfmadd231ps	%ymm0, %ymm15, %ymm12
	vmovups		32(%r11), %ymm13
	vfmadd231ps	%ymm4, %ymm15, %ymm13
	vmovups		64(%r11), %ymm14
	vfmadd231ps	%ymm8, %ymm15, %ymm14
	vmovups		%ymm12, 0(%r11)
	vmovups		%ymm13, 32(%r11)
	vmovups		%ymm14, 64(%r11)
	addq		%r12, %r11

	vmovups		0(%r11), %ymm12
	vfmadd231ps	%ymm1, %ymm15, %ymm12
	vmovups		32(%r11), %ymm13
	vfmadd231ps	%ymm5, %ymm15, %ymm13
	vmovups		64(%r11), %ymm14
	vfmadd231ps	%ymm9, %ymm15, %ymm14
	vmovups		%ymm12, 0(%r11)
	vmovups		%ymm13, 32(%r11)
	vmovups		%ymm14, 64(%r11)
	addq		%r12, %r11

	vmovups		0(%r11), %ymm12
	vfmadd231ps	%ymm2, %ymm15, %ymm12
	vmovups		32(%r11), %ymm13
	vfmadd231ps	%ymm6, %ymm15, %ymm13
	vmovups		64(%r11), %ymm14
	vfmadd231ps	%ymm10, %ymm15, %ymm14
	vmovups		%ymm12, 0(%r11)
	vmovups		%ymm13, 32(%r11)
	vmovups		%ymm14, 64(%r11)
	addq		%r12, %r11

	vmovups		0(%r11), %ymm12
	vfmadd231ps	%ymm3, %ymm15, %ymm12
	vmovups		32(%r11), %ymm13
	vfmadd231ps	%ymm7, %ymm15, %ymm13
	vmovups		64(%r11), %ymm14
	vfmadd231ps	%ymm11, %ymm15, %ymm14
	vmovups		%ymm12, 0(%r11)
	vmovups		%ymm13, 32(%r11)
	vmovups		%ymm14, 64(%r11)
//	addq		%r12, %r11

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_a1_store_24x4_lib)
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
	.macro INNER_STORE_4X24_LIB
#else
	.p2align 4,,15
	FUN_START(inner_store_4x24_lib)
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

	vmovups 		%xmm8,  0(%r10)
	addq			%r11, %r10
	vmovups 		%xmm9,  0(%r10)
	addq			%r11, %r10
	vmovups 		%xmm10,  0(%r10)
	addq			%r11, %r10
	vmovups 		%xmm11,  0(%r10)
	addq			%r11, %r10
	vextractf128	$ 1, %ymm8, %xmm15
	vmovups 		%xmm15,  0(%r10)
	addq			%r11, %r10
	vextractf128	$ 1, %ymm9, %xmm15
	vmovups 		%xmm15,  0(%r10)
	addq			%r11, %r10
	vextractf128	$ 1, %ymm10, %xmm15
	vmovups 		%xmm15,  0(%r10)
	addq			%r11, %r10
	vextractf128	$ 1, %ymm11, %xmm15
	vmovups 		%xmm15,  0(%r10)
	addq			%r11, %r10

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4x24_lib)
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
	.macro INNER_STORE_4X24_VS_LIB
#else
	.p2align 4,,15
	FUN_START(inner_store_4x24_vs_lib)
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
	vmaskmovps		%xmm5, %xmm14, 0(%r10)
	addq			%r11, %r10
	vmaskmovps		%xmm6, %xmm14, 0(%r10)
	addq			%r11, %r10
	vmaskmovps		%xmm7, %xmm14, 0(%r10)
	addq			%r11, %r10
	vextractf128	$ 1, %ymm4, %xmm15
	vmaskmovps		%xmm15, %xmm14, 0(%r10)
	addq			%r11, %r10
	vextractf128	$ 1, %ymm5, %xmm15
	vmaskmovps		%xmm15, %xmm14, 0(%r10)
	addq			%r11, %r10
	vextractf128	$ 1, %ymm6, %xmm15
	vmaskmovps		%xmm15, %xmm14, 0(%r10)
	addq			%r11, %r10
	vextractf128	$ 1, %ymm7, %xmm15
	vmaskmovps		%xmm15, %xmm14, 0(%r10)
	addq			%r11, %r10

	vmaskmovps		%xmm8, %xmm14, 0(%r10)
	addq			%r11, %r10
	cmpl		$ 18, %r13d
	jl			0f // end
	vmaskmovps		%xmm9, %xmm14, 0(%r10)
	addq			%r11, %r10
	cmpl		$ 19, %r13d
	jl			0f // end
	vmaskmovps		%xmm10, %xmm14, 0(%r10)
	addq			%r11, %r10
	cmpl		$ 20, %r13d
	jl			0f // end
	vmaskmovps		%xmm11, %xmm14, 0(%r10)
	addq			%r11, %r10
	cmpl		$ 21, %r13d
	jl			0f // end
	vextractf128	$ 1, %ymm8, %xmm15
	vmaskmovps		%xmm15, %xmm14, 0(%r10)
	addq			%r11, %r10
	cmpl		$ 22, %r13d
	jl			0f // end
	vextractf128	$ 1, %ymm9, %xmm15
	vmaskmovps		%xmm15, %xmm14, 0(%r10)
	addq			%r11, %r10
	cmpl		$ 23, %r13d
	jl			0f // end
	vextractf128	$ 1, %ymm10, %xmm15
	vmaskmovps		%xmm15, %xmm14, 0(%r10)
	addq			%r11, %r10
	cmpl		$ 23, %r13d
	je			0f // end
	vextractf128	$ 1, %ymm11, %xmm15
	vmaskmovps		%xmm15, %xmm14, 0(%r10)
//	addq			%r11, %r10

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4x24_vs_lib)
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
	.macro INNER_PREFETCH0_24X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_prefetch0_24x4_lib)
#endif

	leaq		0(%r11, %r11, 2), %r12
	prefetcht0	0(%r10)				// 0
	prefetcht0	64(%r10)			// 0
	prefetcht0	95(%r10)			// 0
	prefetcht0	0(%r10, %r11)	// 1
	prefetcht0	64(%r10, %r11)	// 1
	prefetcht0	95(%r10, %r11)	// 1
	prefetcht0	0(%r10, %r11, 2)	// 2
	prefetcht0	64(%r10, %r11, 2)	// 2
	prefetcht0	95(%r10, %r11, 2)	// 2
	prefetcht0	0(%r10, %r12)	// 3
	prefetcht0	64(%r10, %r12)	// 3
	prefetcht0	95(%r10, %r12)	// 3

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_prefetch0_24x4_lib)
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
	.macro INNER_PREFETCH1_24X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_prefetch1_24x4_lib)
#endif

	leaq		0(%r11, %r11, 2), %r12
	prefetcht1	0(%r10)				// 0
	prefetcht1	64(%r10)			// 0
	prefetcht1	95(%r10)			// 0
	prefetcht1	0(%r10, %r11)	// 1
	prefetcht1	64(%r10, %r11)	// 1
	prefetcht1	95(%r10, %r11)	// 1
	prefetcht1	0(%r10, %r11, 2)	// 2
	prefetcht1	64(%r10, %r11, 2)	// 2
	prefetcht1	95(%r10, %r11, 2)	// 2
	prefetcht1	0(%r10, %r12)	// 3
	prefetcht1	64(%r10, %r12)	// 3
	prefetcht1	95(%r10, %r12)	// 3

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_prefetch1_24x4_lib)
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
	.macro INNER_PREFETCH0_4X24_LIBC
#else
	.p2align 4,,15
	FUN_START(inner_prefetch0_4x24_libc)
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

	FUN_END(inner_prefetch0_4x24_libc)
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
	.macro INNER_PREFETCH1_4X24_LIBC
#else
	.p2align 4,,15
	FUN_START(inner_prefetch1_4x24_libc)
#endif

	leaq		0(%r11, %r11, 2), %r12

	prefetcht1	0(%r10)
	prefetcht1	15(%r10)
	prefetcht1	0(%r10, %r11)
	prefetcht1	15(%r10, %r11)
	prefetcht1	0(%r10, %r11, 2)
	prefetcht1	15(%r10, %r11, 2)
	prefetcht1	0(%r10, %r12)
	prefetcht1	15(%r10, %r12)

	leaq		0(%r10, %r11, 4), %r10

	prefetcht1	0(%r10)
	prefetcht1	15(%r10)
	prefetcht1	0(%r10, %r11)
	prefetcht1	15(%r10, %r11)
	prefetcht1	0(%r10, %r11, 2)
	prefetcht1	15(%r10, %r11, 2)
	prefetcht1	0(%r10, %r12)
	prefetcht1	15(%r10, %r12)

	leaq		0(%r10, %r11, 4), %r10

	prefetcht1	0(%r10)
	prefetcht1	15(%r10)
	prefetcht1	0(%r10, %r11)
	prefetcht1	15(%r10, %r11)
	prefetcht1	0(%r10, %r11, 2)
	prefetcht1	15(%r10, %r11, 2)
	prefetcht1	0(%r10, %r12)
	prefetcht1	15(%r10, %r12)

	leaq		0(%r10, %r11, 4), %r10

	prefetcht1	0(%r10)
	prefetcht1	15(%r10)
	prefetcht1	0(%r10, %r11)
	prefetcht1	15(%r10, %r11)
	prefetcht1	0(%r10, %r11, 2)
	prefetcht1	15(%r10, %r11, 2)
	prefetcht1	0(%r10, %r12)
	prefetcht1	15(%r10, %r12)

	leaq		0(%r10, %r11, 4), %r10

	prefetcht1	0(%r10)
	prefetcht1	15(%r10)
	prefetcht1	0(%r10, %r11)
	prefetcht1	15(%r10, %r11)
	prefetcht1	0(%r10, %r11, 2)
	prefetcht1	15(%r10, %r11, 2)
	prefetcht1	0(%r10, %r12)
	prefetcht1	15(%r10, %r12)

	leaq		0(%r10, %r11, 4), %r10

	prefetcht1	0(%r10)
	prefetcht1	15(%r10)
	prefetcht1	0(%r10, %r11)
	prefetcht1	15(%r10, %r11)
	prefetcht1	0(%r10, %r11, 2)
	prefetcht1	15(%r10, %r11, 2)
	prefetcht1	0(%r10, %r12)
	prefetcht1	15(%r10, %r12)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_prefetch1_4x24_libc)
#endif





//                                   1      2              3          4        5          6        7             8          9        10         11
// void kernel_sgemm_nt_24x4_lib8ccc(int k, float *alpha, float *A, int sda, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_nt_24x4_lib8ccc)

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
	INNER_KERNEL_GEMM_NT_24X4_LIB8C
#else
	CALL(inner_kernel_gemm_nt_24x4_lib8c)
#endif


	// prefetch D

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_24X4_LIB
#else
	CALL(inner_prefetch0_24x4_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 2, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_24X4_LIB
#else
	CALL(inner_scale_ab_24x4_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_24X4_LIB
#else
	CALL(inner_store_24x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_nt_24x4_lib8ccc)





//                                      1      2              3          4        5          6        7             8          9        10         11       12      13
// void kernel_sgemm_nt_24x4_vs_lib8ccc(int k, float *alpha, float *A, int sda, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_nt_24x4_vs_lib8ccc)

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
	INNER_KERNEL_GEMM_NT_24X1_LIB8C
#else
	CALL(inner_kernel_gemm_nt_24x1_lib8c)
#endif
	
	jmp		103f

100:

	movq	ARG13, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_24X2_LIB8C
#else
	CALL(inner_kernel_gemm_nt_24x2_lib8c)
#endif

	jmp		103f

101:

	movq	ARG13, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_24X3_LIB8C
#else
	CALL(inner_kernel_gemm_nt_24x3_lib8c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_24X4_LIB8C
#else
	CALL(inner_kernel_gemm_nt_24x4_lib8c)
#endif

103:


	// prefetch D
	// TODO prefetch vs !!!

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_24X4_LIB
#else
	CALL(inner_prefetch0_24x4_lib)
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
	INNER_SCALE_AB_24X4_VS_LIB
#else
	CALL(inner_scale_ab_24x4_vs_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 2, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_24X4_VS_LIB
#else
	CALL(inner_store_24x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_nt_24x4_vs_lib8ccc)





//                                   1      2             3         4        5         6            7         8        9         10
// void kernel_sgemm_nt_24x4_lib88cc(int k, float *alpha, float *A, int sda, float *B, float *beta, float *C, int ldc, float *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_nt_24x4_lib88cc)
	
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
	INNER_PREFETCH0_24X4_LIB
#else
	CALL(inner_prefetch0_24x4_lib)
#endif

100:


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12 // sda
	sall	$ 5, %r12d // 8*sda*sizeof(float)
	movq	ARG5, %r13  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_24X4_LIB8
#else
	CALL(inner_kernel_gemm_add_nt_24x4_lib8)
#endif


	// call inner scale

	movq	%rsi, %r10 // alpha
	movq	ARG6, %r11 // beta
	movq	ARG7, %r12   // C
	movl	ARG8, %r13d // ldc
	sall	$ 2, %r13d // ldc*sizeof(float)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_24X4_LIB
#else
	CALL(inner_scale_ab_24x4_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movl	ARG10, %r11d // ldd
	sall	$ 2, %r11d // ldd*sizeof(float)

#if MACRO_LEVEL>=1
	INNER_STORE_24X4_LIB
#else
	CALL(inner_store_24x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_nt_24x4_lib88cc)





//                                      1      2             3         4        5         6            7         8        9         10       12      13
// void kernel_sgemm_nt_24x4_vs_lib88cc(int k, float *alpha, float *A, int sda, float *B, float *beta, float *C, int ldc, float *D, int ldd, int km, int kn);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_nt_24x4_vs_lib88cc)
	
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
	INNER_KERNEL_GEMM_ADD_NT_24X4_LIB8
#else
	CALL(inner_kernel_gemm_add_nt_24x4_lib8)
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
	INNER_SCALE_AB_24X4_VS_LIB
#else
	CALL(inner_scale_ab_24x4_vs_lib)
#endif


	// store n

	movq	ARG9, %r10 // D
	movl	ARG10, %r11d // ldd
	sall	$ 2, %r11d // ldd*sizeof(float)
	movq	ARG11, %r12 // km
	movq	ARG12, %r13 // kn

#if MACRO_LEVEL>=1
	INNER_STORE_24X4_VS_LIB
#else
	CALL(inner_store_24x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_nt_24x4_vs_lib88cc)





//                                      1      2      3             4         5        6         7        8            9         10       11        12       13          14
// void kernel_sgemm_nt_24xn_p0_lib88cc(int n, int k, float *alpha, float *A, int sda, float *B, int sdb, float *beta, float *C, int ldc, float *D, int ldd, float *A_p, float *B_p);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_nt_24xn_p0_lib88cc)

	PROLOGUE

	// zero accumulation registers


//	movq	ARG6, %rbp // B
//	movq	ARG9, %rax // C
//	movq	ARG11, %rbx // D


	// loop over n
//1001:
	cmpq	$ 7, ARG1
	jle		1000f // consider clean-up loop
1001:


	// left block


	ZERO_ACC


	// prefetch C

	prefetcht1	0(ARG3) // alpha
	movq		ARG8, %r10 // beta
	vmovss		0(%r10), %xmm14
	vxorps		%xmm15, %xmm15, %xmm15 // 0.0
	vucomiss	%xmm15, %xmm14 // beta==0.0 ?
	je			100f // end

	movq	ARG9, %r10 // C
	movq	ARG10, %r11 // ldc
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH1_24X4_LIB
#else
	CALL(inner_prefetch1_24x4_lib)
#endif

100:


	// call inner dgemm kernel nn

	movq	ARG2, %r10 // k
	movq	ARG4, %r11  // A
	movq	ARG5, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG6, %r13  // B

	movq	ARG4, %r14  // A_p
//	cmpq	$ 4, ARG1
//	cmoveq	ARG13, %r14  // A_p

//	movq	ARG7, %r15 // sdb
//	sall	$ 5, %r15d // 4*sdb*sizeof(double)
//	addq	ARG6, %r15 // B_p
//	cmpq	$ 4, ARG1
//	cmoveq	ARG14, %r15  // B_p
	movq	ARG6, %r15  // B_p

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_24X4_P0_LIB8
//	INNER_KERNEL_GEMM_ADD_NT_24X4_LIB8
#else
	CALL(inner_kernel_gemm_nt_24x4_p0_lib8)
//	CALL(inner_kernel_gemm_add_nt_24x4_lib8)
#endif



	movq	ARG11, %r10 // D
	cmpq	ARG9, %r10
	jne			100f // end

	movq	ARG12, %r10 // ldd
	cmpq	ARG10, %r10
	jne			100f // end

	movq	ARG8, %r10 // beta
	vbroadcastss	0(%r10), %ymm14
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovaps		.LC03(%rip), %ymm15 // 1.0
#else
	vmovaps		LC03(%rip), %ymm15 // 1.0
#endif
	vucomiss	%xmm15, %xmm14 // beta==1.0 ?
	jne			100f // end



	movq	ARG3, %r10 // alpha
	movq	ARG11, %r11   // D
	movq	ARG12, %r12   // ldd
	sall	$ 2, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_A1_STORE_24X4_LIB
#else
	CALL(inner_scale_a1_store_24x4_lib)
#endif


	jmp		101f


100:


	// call inner blend

	movq	ARG3, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12   // C
	movq	ARG10, %r13   // ldc
	sall	$ 2, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_24X4_LIB
#else
	CALL(inner_scale_ab_24x4_lib)
#endif


	// store n

	movq	ARG11, %r10 // D
	movq	ARG12, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_24X4_LIB
#else
	CALL(inner_store_24x4_lib)
#endif


101:


	// loop increments
	// n
//	movzx	%edi, %rdi //ARG1
	subq	$ 4, ARG1
	// B
//	movq	ARG7, %r10
//	sall	$ 5, %r10d
//	addq	%r10, ARG6
	// C
	movq	ARG10, %r10
	sall	$ 4, %r10d
	addq	%r10, ARG9
	// D
	movq	ARG12, %r10
	sall	$ 4, %r10d
	addq	%r10, ARG11


	// right block


	ZERO_ACC


	// prefetch C

	prefetcht1	0(ARG3) // alpha
	movq		ARG8, %r10 // beta
	vmovss		0(%r10), %xmm14
	vxorps		%xmm15, %xmm15, %xmm15 // 0.0
	vucomiss	%xmm15, %xmm14 // beta==0.0 ?
	je			100f // end

	movq	ARG9, %r10 // C
	movq	ARG10, %r11 // ldc
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH1_24X4_LIB
#else
	CALL(inner_prefetch1_24x4_lib)
#endif

100:


	// call inner dgemm kernel nn

	movq	ARG2, %r10 // k
	movq	ARG4, %r11  // A
	movq	ARG5, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG6, %r13  // B
	addq	$ 16, %r13 // B+4

	movq	ARG4, %r14  // A_p
	cmpq	$ 4, ARG1
	cmoveq	ARG13, %r14  // A_p

	movq	ARG7, %r15 // sdb
	sall	$ 5, %r15d // 4*sdb*sizeof(double)
	addq	ARG6, %r15 // B_p
//	addq	$ 16, %r15 // B_p+4
	cmpq	$ 4, ARG1
	cmoveq	ARG14, %r15  // B_p

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_24X4_P0_LIB8
//	INNER_KERNEL_GEMM_ADD_NT_24X4_LIB8
#else
	CALL(inner_kernel_gemm_nt_24x4_p0_lib8)
//	CALL(inner_kernel_gemm_add_nt_24x4_lib8)
#endif



	movq	ARG11, %r10 // D
	cmpq	ARG9, %r10
	jne			100f // end

	movq	ARG12, %r10 // ldd
	cmpq	ARG10, %r10
	jne			100f // end

	movq	ARG8, %r10 // beta
	vbroadcastss	0(%r10), %ymm14
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovaps		.LC03(%rip), %ymm15 // 1.0
#else
	vmovaps		LC03(%rip), %ymm15 // 1.0
#endif
	vucomiss	%xmm15, %xmm14 // beta==1.0 ?
	jne			100f // end



	movq	ARG3, %r10 // alpha
	movq	ARG11, %r11   // D
	movq	ARG12, %r12   // ldd
	sall	$ 2, %r12d

#if MACRO_LEVEL>=1
	INNER_SCALE_A1_STORE_24X4_LIB
#else
	CALL(inner_scale_a1_store_24x4_lib)
#endif


	jmp		101f


100:


	// call inner blend

	movq	ARG3, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12   // C
	movq	ARG10, %r13   // ldc
	sall	$ 2, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_24X4_LIB
#else
	CALL(inner_scale_ab_24x4_lib)
#endif


	// store n

	movq	ARG11, %r10 // D
	movq	ARG12, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_24X4_LIB
#else
	CALL(inner_store_24x4_lib)
#endif


101:


	// loop increments
	// n
//	movzx	%edi, %rdi //ARG1
	subq	$ 4, ARG1
	// B
	movq	ARG7, %r10
	sall	$ 5, %r10d
	addq	%r10, ARG6
	// C
	movq	ARG10, %r10
	sall	$ 4, %r10d
	addq	%r10, ARG9
	// D
	movq	ARG12, %r10
	sall	$ 4, %r10d
	addq	%r10, ARG11


	cmpq	$ 7, ARG1
	jg		1001b // main loop


1000:
	// TODO clean-up
	cmpq	$ 4, ARG1
	jle		2000f // consider clean-up loop


// TODO
	// left block


	ZERO_ACC


	// prefetch C

	prefetcht1	0(ARG3) // alpha
	movq		ARG8, %r10 // beta
	vmovss		0(%r10), %xmm14
	vxorps		%xmm15, %xmm15, %xmm15 // 0.0
	vucomiss	%xmm15, %xmm14 // beta==0.0 ?
	je			100f // end

	movq	ARG9, %r10 // C
	movq	ARG10, %r11 // ldc
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH1_24X4_LIB
#else
	CALL(inner_prefetch1_24x4_lib)
#endif

100:


	// call inner dgemm kernel nn

	movq	ARG2, %r10 // k
	movq	ARG4, %r11  // A
	movq	ARG5, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG6, %r13  // B

	movq	ARG4, %r14  // A_p
	movq	ARG6, %r15  // B_p

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_24X4_P0_LIB8
//	INNER_KERNEL_GEMM_ADD_NT_24X4_LIB8
#else
	CALL(inner_kernel_gemm_nt_24x4_p0_lib8)
//	CALL(inner_kernel_gemm_add_nt_24x4_lib8)
#endif



	// call inner blend

	movq	ARG3, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12   // C
	movq	ARG10, %r13   // ldc
	sall	$ 2, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_24X4_LIB
#else
	CALL(inner_scale_ab_24x4_lib)
#endif


	// store n

	movq	ARG11, %r10 // D
	movq	ARG12, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_24X4_LIB
#else
	CALL(inner_store_24x4_lib)
#endif


	// loop increments
	// n
//	movzx	%edi, %rdi //ARG1
	subq	$ 4, ARG1
	// B
//	movq	ARG7, %r10
//	sall	$ 5, %r10d
//	addq	%r10, ARG6
	// C
	movq	ARG10, %r10
	sall	$ 4, %r10d
	addq	%r10, ARG9
	// D
	movq	ARG12, %r10
	sall	$ 4, %r10d
	addq	%r10, ARG11


	// right block


	ZERO_ACC


	// prefetch C

	prefetcht1	0(ARG3) // alpha
	movq		ARG8, %r10 // beta
	vmovss		0(%r10), %xmm14
	vxorps		%xmm15, %xmm15, %xmm15 // 0.0
	vucomiss	%xmm15, %xmm14 // beta==0.0 ?
	je			100f // end

	movq	ARG9, %r10 // C
	movq	ARG10, %r11 // ldc
	sall	$ 2, %r11d

	// TODO prefetch vs

#if MACRO_LEVEL>=1
	INNER_PREFETCH1_24X4_LIB
#else
	CALL(inner_prefetch1_24x4_lib)
#endif

100:


	// call inner dgemm kernel nn

	movq	ARG2, %r10 // k
	movq	ARG4, %r11  // A
	movq	ARG5, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG6, %r13  // B
	addq	$ 16, %r13 // B+4

	movq	ARG4, %r14  // A_p
	addq	ARG6, %r15 // B_p

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_24X4_P0_LIB8
//	INNER_KERNEL_GEMM_ADD_NT_24X4_LIB8
#else
	CALL(inner_kernel_gemm_nt_24x4_p0_lib8)
//	CALL(inner_kernel_gemm_add_nt_24x4_lib8)
#endif



	// call inner blend

	movq	ARG3, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12   // C
	movq	ARG10, %r13   // ldc
	sall	$ 2, %r13d
	movq	$ 24, %r14   // m1
	movq	ARG1, %r15   // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_24X4_VS_LIB
#else
	CALL(inner_scale_ab_24x4_vs_lib)
#endif


	// store n

	movq	ARG11, %r10 // D
	movq	ARG12, %r11 // ldd
	sall	$ 2, %r11d
	movq	$ 24, %r12   // m1
	movq	ARG1, %r13   // n1

#if MACRO_LEVEL>=1
	INNER_STORE_24X4_VS_LIB
#else
	CALL(inner_store_24x4_vs_lib)
#endif


101:


#if 0
	// loop increments
	// n
//	movzx	%edi, %rdi //ARG1
	subq	$ 4, ARG1
	// B
	movq	ARG7, %r10
	sall	$ 5, %r10d
	addq	%r10, ARG6
	// C
	movq	ARG10, %r10
	sall	$ 4, %r10d
	addq	%r10, ARG9
	// D
	movq	ARG12, %r10
	sall	$ 4, %r10d
	addq	%r10, ARG11
#endif


	jmp		1000f


2000:
	cmpq	$ 0, ARG1
	jle		1000f // return


	ZERO_ACC


	// prefetch C

	prefetcht1	0(ARG3) // alpha
	movq		ARG8, %r10 // beta
	vmovss		0(%r10), %xmm14
	vxorps		%xmm15, %xmm15, %xmm15 // 0.0
	vucomiss	%xmm15, %xmm14 // beta==0.0 ?
	je			100f // end

	movq	ARG9, %r10 // C
	movq	ARG10, %r11 // ldc
	sall	$ 3, %r11d

	// TODO prefetch vs

#if MACRO_LEVEL>=1
	INNER_PREFETCH1_24X4_LIB
#else
	CALL(inner_prefetch1_24x4_lib)
#endif

100:


	// call inner dgemm kernel nn

	movq	ARG2, %r10 // k
	movq	ARG4, %r11  // A
	movq	ARG5, %r12 // sda
	sall	$ 5, %r12d // 4*sda*sizeof(double)
	movq	ARG6, %r13  // B
	movq	ARG4, %r14  // A_p
	addq	ARG6, %r15 // B_p

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_24X4_P0_LIB8
//	INNER_KERNEL_GEMM_ADD_NT_24X4_LIB8
#else
	CALL(inner_kernel_gemm_nt_24x4_p0_lib8)
//	CALL(inner_kernel_gemm_add_nt_24x4_lib8)
#endif




	// call inner blend

	movq	ARG3, %r10 // alpha
	movq	ARG8, %r11 // beta
	movq	ARG9, %r12   // C
	movq	ARG10, %r13   // ldc
	sall	$ 2, %r13d
	movq	$ 24, %r14   // m1
	movq	ARG1, %r15   // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_24X4_VS_LIB
#else
	CALL(inner_scale_ab_24x4_vs_lib)
#endif


	// store n

	movq	ARG11, %r10 // D
	movq	ARG12, %r11 // ldd
	sall	$ 2, %r11d
	movq	$ 24, %r12   // m1
	movq	ARG1, %r13   // n1

#if MACRO_LEVEL>=1
	INNER_STORE_24X4_VS_LIB
#else
	CALL(inner_store_24x4_vs_lib)
#endif







	// loop increments
#if 0
	// n
//	movzx	%edi, %rdi //ARG1
	subq	$ 4, ARG1
	// B
	movq	ARG7, %r10
	sall	$ 5, %r10d
	addq	%r10, ARG6
	// C
	movq	ARG10, %r10
	sall	$ 4, %r10d
	addq	%r10, ARG9
	// D
	movq	ARG12, %r10
	sall	$ 4, %r10d
	addq	%r10, ARG11
#endif



1000:
	// return

	EPILOGUE

	ret

	FUN_END(kernel_sgemm_nt_24xn_p0_lib88cc)





//                                 1      2              3          4        5          6        7             8          9        10         11
// void kernel_sgemm_nn_24x4_lib8ccc(int k, float *alpha, float *A, int sda, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd,);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_nn_24x4_lib8ccc)

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
	INNER_KERNEL_GEMM_NN_24X4_LIB8C
#else
	CALL(inner_kernel_gemm_nn_24x4_lib8c)
#endif


	// prefetch D

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_24X4_LIB
#else
	CALL(inner_prefetch0_24x4_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 2, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_24X4_LIB
#else
	CALL(inner_scale_ab_24x4_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_24X4_LIB
#else
	CALL(inner_store_24x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_nn_24x4_lib8ccc)





//                                      1      2              3          4        5          6        7             8          9        10         11       12      13
// void kernel_sgemm_nn_24x4_vs_lib8ccc(int k, float *alpha, float *A, int sda, float *B, int ldb, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_nn_24x4_vs_lib8ccc)

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
	INNER_KERNEL_GEMM_NN_24X1_LIB8C
#else
	CALL(inner_kernel_gemm_nn_24x1_lib8c)
#endif
	
	jmp		103f

100:

	movq	ARG13, %r15  // n1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_24X2_LIB8C
#else
	CALL(inner_kernel_gemm_nn_24x2_lib8c)
#endif

	jmp		103f

101:

	movq	ARG13, %r15  // n1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_24X3_LIB8C
#else
	CALL(inner_kernel_gemm_nn_24x3_lib8c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_24X4_LIB8C
#else
	CALL(inner_kernel_gemm_nn_24x4_lib8c)
#endif

103:


	// prefetch D
	// TODO prefetch vs !!!

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_24X4_LIB
#else
	CALL(inner_prefetch0_24x4_lib)
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
	INNER_SCALE_AB_24X4_VS_LIB
#else
	CALL(inner_scale_ab_24x4_vs_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 2, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_24X4_VS_LIB
#else
	CALL(inner_store_24x4_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_nn_24x4_vs_lib8ccc)





//                                   1      2             3         4         5        6        7            8         9        10        11
// void kernel_sgemm_nt_4x24_libc8cc(int k, float *alpha, float *A, int lda, float *B, int sdb, float *beta, float *C, int ldc, float *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_nt_4x24_libc8cc)

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
	INNER_KERNEL_GEMM_NT_24X4_LIB8C
#else
	CALL(inner_kernel_gemm_nt_24x4_lib8c)
#endif


	// prefetch D

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_4X24_LIBC
#else
	CALL(inner_prefetch0_4x24_libc)
#endif


#if MACRO_LEVEL>=1
	INNER_TRAN_24X4_LIB
#else
	CALL(inner_tran_24x4_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 2, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X24_LIB
#else
	CALL(inner_scale_ab_4x24_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_4X24_LIB
#else
	CALL(inner_store_4x24_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_nt_4x24_libc8cc)





//                                      1      2             3         4         5        6        7            8         9        10        11       12      13
// void kernel_sgemm_nt_4x24_vs_libc8cc(int k, float *alpha, float *A, int lda, float *B, int sdb, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_nt_4x24_vs_libc8cc)

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
	INNER_KERNEL_GEMM_NT_24X1_LIB8C
#else
	CALL(inner_kernel_gemm_nt_24x1_lib8c)
#endif
	
	jmp		103f

100:

	movq	ARG12, %r15  // m1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_24X2_LIB8C
#else
	CALL(inner_kernel_gemm_nt_24x2_lib8c)
#endif

	jmp		103f

101:

	movq	ARG12, %r15  // m1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_24X3_LIB8C
#else
	CALL(inner_kernel_gemm_nt_24x3_lib8c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_24X4_LIB8C
#else
	CALL(inner_kernel_gemm_nt_24x4_lib8c)
#endif

103:


	// prefetch D
	// TODO prefetch vs !!!

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_4X24_LIBC
#else
	CALL(inner_prefetch0_4x24_libc)
#endif


#if MACRO_LEVEL>=1
	INNER_TRAN_24X4_LIB
#else
	CALL(inner_tran_24x4_lib)
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
	INNER_SCALE_AB_4X24_VS_LIB
#else
	CALL(inner_scale_ab_4x24_vs_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 2, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X24_VS_LIB
#else
	CALL(inner_store_4x24_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_nt_4x24_vs_libc8cc)





//                                   1      2             3         4         5        6        7            8         9        10        11
// void kernel_sgemm_tt_4x24_libc8cc(int k, float *alpha, float *A, int lda, float *B, int sdb, float *beta, float *C, int ldc, float *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_tt_4x24_libc8cc)

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
	INNER_KERNEL_GEMM_NN_24X4_LIB8C
#else
	CALL(inner_kernel_gemm_nn_24x4_lib8c)
#endif


	// prefetch D

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_4X24_LIBC
#else
	CALL(inner_prefetch0_4x24_libc)
#endif


#if MACRO_LEVEL>=1
	INNER_TRAN_24X4_LIB
#else
	CALL(inner_tran_24x4_lib)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG7, %r11 // beta
	movq	ARG8, %r12   // C
	movq	ARG9, %r13   // ldc
	sall	$ 2, %r13d

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X24_LIB
#else
	CALL(inner_scale_ab_4x24_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_4X24_LIB
#else
	CALL(inner_store_4x24_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_tt_4x24_libc8cc)





//                                      1      2             3         4         5        6        7            8         9        10        11       12      13
// void kernel_sgemm_tt_4x24_vs_libc8cc(int k, float *alpha, float *A, int lda, float *B, int sdb, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_tt_4x24_vs_libc8cc)

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
	INNER_KERNEL_GEMM_NN_24X1_LIB8C
#else
	CALL(inner_kernel_gemm_nn_24x1_lib8c)
#endif
	
	jmp		103f

100:

	movq	ARG12, %r15  // m1
	cmpl	$ 2, %r15d
	jg		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_24X2_LIB8C
#else
	CALL(inner_kernel_gemm_nn_24x2_lib8c)
#endif

	jmp		103f

101:

	movq	ARG12, %r15  // m1
	cmpl	$ 3, %r15d
	jg		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_24X3_LIB8C
#else
	CALL(inner_kernel_gemm_nn_24x3_lib8c)
#endif

	jmp		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_24X4_LIB8C
#else
	CALL(inner_kernel_gemm_nn_24x4_lib8c)
#endif

103:


	// prefetch D
	// TODO prefetch vs !!!

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_PREFETCH0_4X24_LIBC
#else
	CALL(inner_prefetch0_4x24_libc)
#endif


#if MACRO_LEVEL>=1
	INNER_TRAN_24X4_LIB
#else
	CALL(inner_tran_24x4_lib)
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
	INNER_SCALE_AB_4X24_VS_LIB
#else
	CALL(inner_scale_ab_4x24_vs_lib)
#endif


	// store n

	movq	ARG10, %r10 // D
	movq	ARG11, %r11 // ldd
	sall	$ 2, %r11d
	movq	ARG12, %r12 // m1
	movq	ARG13, %r13 // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X24_VS_LIB
#else
	CALL(inner_store_4x24_vs_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_tt_4x24_vs_libc8cc)







