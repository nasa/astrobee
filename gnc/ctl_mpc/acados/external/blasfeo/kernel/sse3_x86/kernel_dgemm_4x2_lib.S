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
// blend for generic alpha and beta
//
// input arguments:
// eax   <- alpha
// ebx   <- beta
// ecx   <- C
// edx   <- ldc
// xmm0 <- [d00 d10]
// xmm1 <- [d01 d11]
// xmm2 <- [d20 d30]
// xmm3 <- [d21 d31]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_4X2_LIB
#else
	.align 16
	.type inner_scale_ab_4x2_lib, @function
inner_scale_ab_4x2_lib:
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

	movupd		0(%ecx), %xmm7
	mulpd		%xmm6, %xmm7
	addpd		%xmm7, %xmm0
	movupd		16(%ecx), %xmm7
	mulpd		%xmm6, %xmm7
	addpd		%xmm7, %xmm2
	addl		%edx, %ecx
	movupd		0(%ecx), %xmm7
	mulpd		%xmm6, %xmm7
	addpd		%xmm7, %xmm1
	movupd		16(%ecx), %xmm7
	mulpd		%xmm6, %xmm7
	addpd		%xmm7, %xmm3

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	.size	inner_scale_ab_4x2_lib, .-inner_scale_ab_4x2_lib
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// eax  <- D
// ebx  <- ldd
// xmm0 <- [d00 d10]
// xmm1 <- [d01 d11]
// xmm2 <- [d20 d30]
// xmm3 <- [d21 d31]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X2_LIB
#else
	.align 16
	.type inner_store_4x2_lib, @function
inner_store_4x2_lib:
#endif

	movupd	%xmm0,  0(%eax)
	movupd	%xmm2, 16(%eax)
	addl	%ebx, %eax
	movupd	%xmm1,  0(%eax)
	movupd	%xmm3, 16(%eax)

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	.size	inner_store_4x2_lib, .-inner_store_4x2_lib
#endif





//                                 1      2              3          4          5             6          7        8          9
// void kernel_dgemm_nt_4x2_lib44cc(int k, double *alpha, double *A, double *B, double *beta, double *C, int ldc, double *D, int ldd);

	.align 16
	.globl kernel_dgemm_nt_4x2_lib44cc
	.type kernel_dgemm_nt_4x2_lib44cc, @function
kernel_dgemm_nt_4x2_lib44cc:

	PROLOGUE

	// zero accumulation registers

	xorpd	%xmm0, %xmm0
	movapd	%xmm0, %xmm1
	movapd	%xmm0, %xmm2
	movapd	%xmm0, %xmm3


	// call inner dgemm kernel nn

	movl	ARG1, %eax // k
	movl	ARG3, %ebx  // A
	movl	ARG4, %ecx  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X2_LIB4
#else
	call inner_kernel_gemm_nt_4x2_lib4
#endif


	// call inner blend

	movl	ARG2, %eax // alpha
	movl	ARG5, %ebx // beta
	movl	ARG6, %ecx   // C
	movl	ARG7, %edx   // ldc
	sall	$ 3, %edx

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X2_LIB
#else
	call inner_scale_ab_4x2_lib
#endif


	// store n

	movl	ARG8, %eax // D
	movl	ARG9, %ebx // ldd
	sall	$ 3, %ebx

#if MACRO_LEVEL>=1
	INNER_STORE_4X2_LIB
#else
	call inner_store_4x2_lib
#endif


	EPILOGUE

	ret

	.size	kernel_dgemm_nt_4x2_lib44cc, .-kernel_dgemm_nt_4x2_lib44cc






