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
// scale for generic alpha and beta
//
// input arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// r13   <- ldc
// xmm0  <- []
// xmm1  <- []
// xmm2  <- []
// xmm3  <- []

// output arguments:
// r10   <- alpha
// r11   <- beta
// r12   <- C
// xmm0  <- [d00 d10 d20 d30]
// xmm1  <- [d01 d11 d21 d31]
// xmm2  <- [d02 d12 d22 d32]
// xmm3  <- [d03 d13 d23 d33]

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_SCALE_AB_4X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_blend_scale_ab_4x4_lib)
#endif

	movaps		%xmm1, %xmm12
	shufps		$ 0xd8, %xmm0, %xmm1
	shufps		$ 0xd8, %xmm3, %xmm0
	shufps		$ 0xd8, %xmm2, %xmm3
	shufps		$ 0xd8, %xmm12, %xmm2

	movaps		%xmm0, %xmm12
	shufps		$ 0xd8, %xmm2, %xmm0
	shufps		$ 0xd8, %xmm12, %xmm2
	movaps		%xmm1, %xmm12
	shufps		$ 0xd8, %xmm3, %xmm1
	shufps		$ 0xd8, %xmm12, %xmm3

	// alpha
	movss		0(%r10), %xmm15
	shufps		$ 0x00, %xmm15, %xmm15

	mulps		%xmm15, %xmm0
	mulps		%xmm15, %xmm1
	mulps		%xmm15, %xmm2
	mulps		%xmm15, %xmm3

	// beta
	movss		0(%r11), %xmm14
	shufps		$ 0x00, %xmm14, %xmm14

	xorps		%xmm15, %xmm15 // 0.0
	ucomiss		%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	movups		0(%r12), %xmm15
	mulps		%xmm14, %xmm15
	addps		%xmm15, %xmm0
	addq		%r13, %r12
	movups		0(%r12), %xmm15
	mulps		%xmm14, %xmm15
	addps		%xmm15, %xmm1
	addq		%r13, %r12
	movups		0(%r12), %xmm15
	mulps		%xmm14, %xmm15
	addps		%xmm15, %xmm2
	addq		%r13, %r12
	movups		0(%r12), %xmm15
	mulps		%xmm14, %xmm15
	addps		%xmm15, %xmm3
//	addq		%r13, %r12

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_blend_scale_ab_4x4_lib)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// r11  <- ldd
// xmm0  <- [d00 d10 d20 d30]
// xmm1  <- [d01 d11 d21 d31]
// xmm2  <- [d02 d12 d22 d32]
// xmm3  <- [d03 d13 d23 d33]
//
// output arguments:
// r10  <- D
// r11  <- ldd
// xmm0  <- [d00 d10 d20 d30]
// xmm1  <- [d01 d11 d21 d31]
// xmm2  <- [d02 d12 d22 d32]
// xmm3  <- [d03 d13 d23 d33]

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_store_4x4_lib)
#endif
	
	movups		%xmm0,  0(%r10)
	addq		%r11, %r10
	movups 		%xmm1,  0(%r10)
	addq		%r11, %r10
	movups 		%xmm2,  0(%r10)
	addq		%r11, %r10
	movups 		%xmm3,  0(%r10)
	addq		%r11, %r10
	
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4x4_lib)
#endif





//                               1      2             3         4         5            6         7        8         9
// void kernel_sgemm_nt_4x4_lib4(int k, float *alpha, float *A, float *B, float *beta, float *C, int ldc, float *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_sgemm_nt_4x4_lib44cc)
	
	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nt

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_SGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_sgemm_nt_4x4_lib4)
#endif


	// call inner blend scale

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12   // C
	movq	ARG7, %r13   // ldc
	sall	$ 2, %r13d

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_AB_4X4_LIB
#else
	CALL(inner_blend_scale_ab_4x4_lib)
#endif


	// store n

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // ldd
	sall	$ 2, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
	CALL(inner_store_4x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_sgemm_nt_4x4_lib44cc)






