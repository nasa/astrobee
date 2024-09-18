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
// r11   <- x
// r12   <- C
// r13   <- ldc*sizeof(double)
// r14   <- D
// r15   <- ldd*sizeof(double)
// ymm0  <- y_0
// ymm1  <- y_1
// ymm2  <- y_2
// ymm3  <- y_3

//
// output arguments:
// r10d  <- 0
// r11   <-
// r12   <-
// r13   <-
// r14   <-
// r15   <-
// ymm0  <-
// ymm1  <-
// ymm2  <-
// ymm3  <-

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GER_4_LIBC
#else
	.p2align 4,,15
	FUN_START(inner_kernel_ger_4_libc)
#endif

	cmpl	$ 0, %r10d
	jle		2f // return

	leaq	0(%r13, %r13, 2), %rax
	leaq	0(%r15, %r15, 2), %rbx

	cmpl	$ 4, %r10d
	jl		0f // clean-up loop

	// main loop
	.p2align 3
1: // main loop
	
	vmovupd	0(%r11), %ymm12

	vmovupd		0(%r12), %ymm4
	vfmadd231pd	%ymm12, %ymm0, %ymm4

	vmovupd		0(%r12, %r13), %ymm5
	vfmadd231pd	%ymm12, %ymm1, %ymm5

	vmovupd		0(%r12, %r13, 2), %ymm6
	vfmadd231pd	%ymm12, %ymm2, %ymm6

	vmovupd		0(%r12, %rax), %ymm7
	vfmadd231pd	%ymm12, %ymm3, %ymm7

	vmovupd		%ymm4, 0(%r14) 
	vmovupd		%ymm5, 0(%r14, %r15) 
	vmovupd		%ymm6, 0(%r14, %r15, 2) 
	vmovupd		%ymm7, 0(%r14, %rbx) 

	subl	$ 4, %r10d
	addq	$ 32, %r11
	addq	$ 32, %r12
	addq	$ 32, %r14
	
	cmpl	$ 3, %r10d

	jg		1b // main loop 


	// consider clean-up
	cmpl	$ 0, %r10d
	jle		2f // return

0: // clean-up
	
	vcvtsi2sd	%r10d, %xmm14, %xmm14
#if defined(OS_LINUX) | defined(OS_WINDOWS)
	vmovupd		.LC00(%rip), %ymm13
#elif defined(OS_MAC)
	vmovupd		LC00(%rip), %ymm13
#endif
	vmovddup	%xmm14, %xmm14
	vinsertf128	$ 1, %xmm14, %ymm14, %ymm14
	vsubpd		%ymm14, %ymm13, %ymm13


	vmaskmovpd	0(%r11), %ymm13, %ymm12

	vmaskmovpd	0(%r12), %ymm13, %ymm4
	vfmadd231pd	%ymm12, %ymm0, %ymm4

	vmaskmovpd	0(%r12, %r13), %ymm13, %ymm5
	vfmadd231pd	%ymm12, %ymm1, %ymm5

	vmaskmovpd	0(%r12, %r13, 2), %ymm13, %ymm6
	vfmadd231pd	%ymm12, %ymm2, %ymm6

	vmaskmovpd	0(%r12, %rax), %ymm13, %ymm7
	vfmadd231pd	%ymm12, %ymm3, %ymm7

	vmaskmovpd	%ymm4, %ymm13, 0(%r14) 
	vmaskmovpd	%ymm5, %ymm13, 0(%r14, %r15) 
	vmaskmovpd	%ymm6, %ymm13, 0(%r14, %r15, 2) 
	vmaskmovpd	%ymm7, %ymm13, 0(%r14, %rbx) 


	sall	$ 3, %r10d // *sizeof(double)
	addq	%r10, %r11
	addq	%r10, %r12
	addq	%r10, %r14
	xorl	%r10d, %r10d
	
	
2: // return

#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_ger_4_libc)
#endif





//                         1      2              3          4          5          6        7          8
// void kernel_dger_4_libc(int k, double *alpha, double *x, double *y, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dger_4_libc)
	

	PROLOGUE


	// initialize x_n
	movq	ARG2, %r10 // alpha
	vbroadcastsd 0(%r10), %ymm15

	movq	ARG4, %r10 // y

	vbroadcastsd 0(%r10), %ymm0
	vmulpd		%ymm15, %ymm0, %ymm0
	vbroadcastsd 8(%r10), %ymm1
	vmulpd		%ymm15, %ymm1, %ymm1
	vbroadcastsd 16(%r10), %ymm2
	vmulpd		%ymm15, %ymm2, %ymm2
	vbroadcastsd 24(%r10), %ymm3
	vmulpd		%ymm15, %ymm3, %ymm3


	// inner edge dsyrk & kernel dgemv nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11 // x
	movq	ARG5, %r12 // C
	movq	ARG6, %r13 // ldc
	sall	$ 3, %r13d // ldc*sizeof(double)
	movq	ARG7, %r14 // D
	movq	ARG8, %r15 // ldd
	sall	$ 3, %r15d // ldd*sizeof(double)

#if MACRO_LEVEL>=2
	INNER_KERNEL_GER_4_LIBC
#else
	CALL(inner_kernel_ger_4_libc)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dger_4_libc)






