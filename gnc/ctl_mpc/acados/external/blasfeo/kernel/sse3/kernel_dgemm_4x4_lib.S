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
// triangular substitution:
// side = right
// uplo = lower
// tran = transposed
// requires explicit inverse of diagonal
//
// input arguments:
// r10  <- E
// r11  <- lde
// r12  <- inv_diag_E
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_DTRSM_RLT_INV_4X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_edge_dtrsm_rlt_inv_4x4_lib)
#endif
	
	movddup			0(%r12), %xmm13
	mulpd			%xmm13, %xmm0
	mulpd			%xmm13, %xmm4
	movddup			8(%r10), %xmm13
	movapd			%xmm13, %xmm12
	mulpd			%xmm0, %xmm13
	mulpd			%xmm4, %xmm12
	subpd			%xmm13, %xmm1
	subpd			%xmm12, %xmm5
	movddup			16(%r10), %xmm13
	movapd			%xmm13, %xmm12
	mulpd			%xmm0, %xmm12
	mulpd			%xmm4, %xmm13
	subpd			%xmm12, %xmm2
	subpd			%xmm13, %xmm6
	movddup			24(%r10), %xmm13
	movapd			%xmm13, %xmm12
	mulpd			%xmm0, %xmm12
	mulpd			%xmm4, %xmm13
	subpd			%xmm12, %xmm3
	subpd			%xmm13, %xmm7
	addq	%r11, %r10

	movddup			8(%r12), %xmm13
	mulpd			%xmm13, %xmm1
	mulpd			%xmm13, %xmm5
	movddup			16(%r10), %xmm13
	movapd			%xmm13, %xmm12
	mulpd			%xmm1, %xmm12
	mulpd			%xmm5, %xmm13
	subpd			%xmm12, %xmm2
	subpd			%xmm13, %xmm6
	movddup			24(%r10), %xmm13
	movapd			%xmm13, %xmm12
	mulpd			%xmm1, %xmm12
	mulpd			%xmm5, %xmm13
	subpd			%xmm12, %xmm3
	subpd			%xmm13, %xmm7
	addq	%r11, %r10

	movddup			16(%r12), %xmm13
	mulpd			%xmm13, %xmm2
	mulpd			%xmm13, %xmm6
	movddup			24(%r10), %xmm13
	movapd			%xmm13, %xmm12
	mulpd			%xmm2, %xmm12
	mulpd			%xmm6, %xmm13
	subpd			%xmm12, %xmm3
	subpd			%xmm13, %xmm7
	addq	%r11, %r10

	movddup			24(%r12), %xmm13
	mulpd			%xmm13, %xmm3
	mulpd			%xmm13, %xmm7

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_dtrsm_rlt_inv_4x4_lib)
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
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_SCALE_AB_4X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_blend_scale_ab_4x4_lib)
#endif

	movapd	%xmm0, %xmm8
	movsd	%xmm1, %xmm0
	movsd	%xmm8, %xmm1

	movapd	%xmm2, %xmm8
	movsd	%xmm3, %xmm2
	movsd	%xmm8, %xmm3

	movapd	%xmm4, %xmm8
	movsd	%xmm5, %xmm4
	movsd	%xmm8, %xmm5

	movapd	%xmm6, %xmm8
	movsd	%xmm7, %xmm6
	movsd	%xmm8, %xmm7

	// alpha
	movddup	0(%r10), %xmm15

	mulpd	%xmm15, %xmm0
	mulpd	%xmm15, %xmm1
	mulpd	%xmm15, %xmm2
	mulpd	%xmm15, %xmm3
	mulpd	%xmm15, %xmm4
	mulpd	%xmm15, %xmm5
	mulpd	%xmm15, %xmm6
	mulpd	%xmm15, %xmm7

	// beta
	movddup	0(%r11), %xmm14

	xorpd		%xmm15, %xmm15 // 0.0
	ucomisd		%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	movupd		0(%r12), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm0
	movupd		16(%r12), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm4
	addq		%r13, %r12
	movupd		0(%r12), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm1
	movupd		16(%r12), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm5
	addq		%r13, %r12
	movupd		0(%r12), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm2
	movupd		16(%r12), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm6
	addq		%r13, %r12
	movupd		0(%r12), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm3
	movupd		16(%r12), %xmm15
	mulpd		%xmm14, %xmm15
	addpd		%xmm15, %xmm7
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
// scale for alpha=-1 and beta=1
//
// input arguments:
// r10   <- C
// r11   <- ldc
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_SCALE_M11_4X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_blend_scale_m11_4x4_lib)
#endif

	movapd	%xmm0, %xmm8
	movsd	%xmm1, %xmm0
	movsd	%xmm8, %xmm1

	movapd	%xmm2, %xmm8
	movsd	%xmm3, %xmm2
	movsd	%xmm8, %xmm3

	movapd	%xmm4, %xmm8
	movsd	%xmm5, %xmm4
	movsd	%xmm8, %xmm5

	movapd	%xmm6, %xmm8
	movsd	%xmm7, %xmm6
	movsd	%xmm8, %xmm7

	movupd		0(%r10), %xmm15
	subpd		%xmm0, %xmm15
	movapd		%xmm15, %xmm0
	movupd		16(%r10), %xmm15
	subpd		%xmm4, %xmm15
	movapd		%xmm15, %xmm4
	addq		%r11, %r10
	movupd		0(%r10), %xmm15
	subpd		%xmm1, %xmm15
	movapd		%xmm15, %xmm1
	movupd		16(%r10), %xmm15
	subpd		%xmm5, %xmm15
	movapd		%xmm15, %xmm5
	addq		%r11, %r10
	movupd		0(%r10), %xmm15
	subpd		%xmm2, %xmm15
	movapd		%xmm15, %xmm2
	movupd		16(%r10), %xmm15
	subpd		%xmm6, %xmm15
	movapd		%xmm15, %xmm6
	addq		%r11, %r10
	movupd		0(%r10), %xmm15
	subpd		%xmm3, %xmm15
	movapd		%xmm15, %xmm3
	movupd		16(%r10), %xmm15
	subpd		%xmm7, %xmm15
	movapd		%xmm15, %xmm7
//	addq		%r11, %r10

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_blend_scale_m11_4x4_lib)
#endif





// common inner routine with file scope
//
// scale for alpha=1 and generic beta
//
// input arguments:
// r10   <- beta
// r11   <- C
// r12   <- ldc
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_BLEND_SCALE_M1B_4X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_blend_scale_m1b_4x4_lib)
#endif

	movapd	%xmm0, %xmm8
	movsd	%xmm1, %xmm0
	movsd	%xmm8, %xmm1

	movapd	%xmm2, %xmm8
	movsd	%xmm3, %xmm2
	movsd	%xmm8, %xmm3

	movapd	%xmm4, %xmm8
	movsd	%xmm5, %xmm4
	movsd	%xmm8, %xmm5

	movapd	%xmm6, %xmm8
	movsd	%xmm7, %xmm6
	movsd	%xmm8, %xmm7

	// beta
	movddup	0(%r10), %xmm14

	xorpd		%xmm15, %xmm15 // 0.0
	ucomisd		%xmm15, %xmm14 // beta==0.0 ?
	je			0f // end

	movupd		0(%r11), %xmm15
	mulpd		%xmm14, %xmm15
	subpd		%xmm0, %xmm15
	movapd		%xmm15, %xmm0
	movupd		16(%r11), %xmm15
	mulpd		%xmm14, %xmm15
	subpd		%xmm4, %xmm15
	movapd		%xmm15, %xmm4
	addq		%r12, %r11
	movupd		0(%r11), %xmm15
	mulpd		%xmm14, %xmm15
	subpd		%xmm1, %xmm15
	movapd		%xmm15, %xmm1
	movupd		16(%r11), %xmm15
	mulpd		%xmm14, %xmm15
	subpd		%xmm5, %xmm15
	movapd		%xmm15, %xmm5
	addq		%r12, %r11
	movupd		0(%r11), %xmm15
	mulpd		%xmm14, %xmm15
	subpd		%xmm2, %xmm15
	movapd		%xmm15, %xmm2
	movupd		16(%r11), %xmm15
	mulpd		%xmm14, %xmm15
	subpd		%xmm6, %xmm15
	movapd		%xmm15, %xmm6
	addq		%r12, %r11
	movupd		0(%r11), %xmm15
	mulpd		%xmm14, %xmm15
	subpd		%xmm3, %xmm15
	movapd		%xmm15, %xmm3
	movupd		16(%r11), %xmm15
	mulpd		%xmm14, %xmm15
	subpd		%xmm7, %xmm15
	movapd		%xmm15, %xmm7
//	addq		%r12, %r11

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_blend_scale_m1b_4x4_lib)
#endif





// common inner routine with file scope
//
// store n
//
// input arguments:
// r10  <- D
// r11  <- ldd
// xmm0  <- [d00 d10]
// xmm1  <- [d20 d30]
// xmm2  <- [d01 d11]
// xmm3  <- [d21 d31]
// xmm0  <- [d02 d12]
// xmm1  <- [d22 d32]
// xmm2  <- [d03 d13]
// xmm3  <- [d23 d33]
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X4_LIB
#else
	.p2align 4,,15
	FUN_START(inner_store_4x4_lib)
#endif

	movupd		%xmm0, 0(%r10)
	movupd		%xmm4, 16(%r10)
	addq		%r11, %r10
	movupd		%xmm1, 0(%r10)
	movupd		%xmm5, 16(%r10)
	addq		%r11, %r10
	movupd		%xmm2, 0(%r10)
	movupd		%xmm6, 16(%r10)
	addq		%r11, %r10
	movupd		%xmm3, 0(%r10)
	movupd		%xmm7, 16(%r10)
//	addq	%r11, %r10

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4x4_lib)
#endif





//                                 1      2              3          4          5             6          7        8          9
// void kernel_dgemm_nt_4x4_lib44cc(int k, double *alpha, double *A, double *B, double *beta, double *C, int ldc, double *D, int ldd);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dgemm_nt_4x4_lib44cc)

	PROLOGUE

	// zero accumulation registers

	ZERO_ACC


	// call inner dgemm kernel nn

	movq	ARG1, %r10 // k
	movq	ARG3, %r11  // A
	movq	ARG4, %r12  // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blend

	movq	ARG2, %r10 // alpha
	movq	ARG5, %r11 // beta
	movq	ARG6, %r12   // C
	movq	ARG7, %r13   // ldc
	sall	$ 3, %r13d

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_AB_4X4_LIB
#else
	CALL(inner_blend_scale_ab_4x4_lib)
#endif


	// store n

	movq	ARG8, %r10 // D
	movq	ARG9, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
	CALL(inner_store_4x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dgemm_nt_4x4_lib44cc)





//                                          1      2          3          4         5        6          7        8          9        10             11
// void kernel_dtrsm_nt_rl_inv_4x4_lib44ccc(int k, double *A, double *B, double *beta, double *C, int ldc, double *D, int ldd, double *E, int lde, double *inv_diag_E);

	.p2align 4,,15
	GLOB_FUN_START(kernel_dtrsm_nt_rl_inv_4x4_lib44ccc)
	
	PROLOGUE

	// zero accumulation registers


	ZERO_ACC


	// call inner dgemm kernel nt 

	movq	ARG1, %r10
	movq	ARG2, %r11
	movq	ARG3, %r12

#if MACRO_LEVEL>=2
	INNER_KERNEL_DGEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_dgemm_nt_4x4_lib4)
#endif


	// call inner blender_loader nn

	movq	ARG4, %r10 // beta
	movq	ARG5, %r11 // C
	movq	ARG6, %r12 // ldc
	sall	$ 3, %r12d

#if MACRO_LEVEL>=1
	INNER_BLEND_SCALE_M1B_4X4_LIB
#else
	CALL(inner_blend_scale_m1b_4x4_lib)
#endif


	// solve

	movq	ARG9, %r10  // E 
	movq	ARG10, %r11 // lde
	sall	$ 3, %r11d
	movq	ARG11, %r12  // inv_diag_E 

#if MACRO_LEVEL>=1
	INNER_EDGE_DTRSM_RLT_INV_4X4_LIB
#else
	CALL(inner_edge_dtrsm_rlt_inv_4x4_lib)
#endif


	// store

	movq	ARG7, %r10 // D
	movq	ARG8, %r11 // ldd
	sall	$ 3, %r11d

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
	CALL(inner_store_4x4_lib)
#endif


	EPILOGUE

	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_4x4_lib44ccc)





