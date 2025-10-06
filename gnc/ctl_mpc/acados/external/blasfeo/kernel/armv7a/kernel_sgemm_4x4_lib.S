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





// subroutine
//
// input arguments:
// r4   <- alpha
// r5   <- beta
// r6   <- C
// r7   <- ldc*sizeof(float)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_4X4_LIB lc_zero
#else
	.align 3
99: // 0
	.word 0
	.word 0
//	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_scale_ab_4x4_lib, %function
inner_scale_ab_4x4_lib:
#elif defined(OS_MAC)
_inner_scale_ab_4x4_lib:
#endif
#endif

	flds		s8, [r4, #0] // alpha
	flds		s9, [r5, #0] // beta

#if MACRO_LEVEL>=2
	flds		s10, \lc_zero // 0.0
#else
	flds		s10, 99b // 0.0
#endif

	vmul.f32	q4, q4, d4[0]
	vmul.f32	q5, q5, d4[0]
	vmul.f32	q6, q6, d4[0]
	fcmpes		s9, s10
	vmul.f32	q7, q7, d4[0]
	fmstat

	beq		0f // end

	vld1.64		{d0, d1}, [r6], r7
	vld1.64		{d2, d3}, [r6], r7
	vmla.f32	q4, q0, d4[1]
	vmla.f32	q5, q1, d4[1]
	vld1.64		{d0, d1}, [r6], r7
	vld1.64		{d2, d3}, [r6], r7
	vmla.f32	q6, q0, d4[1]
	vmla.f32	q7, q1, d4[1]

0:

#if MACRO_LEVEL>=1
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_scale_ab_4x4_lib, .-inner_scale_ab_4x4_lib
#endif
#endif





// subroutine
//
// input arguments:
// r4   <- D
// r5   <- ldd*sizeof(float)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X4_LIB
#else
//	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_store_4x4_lib, %function
inner_store_4x4_lib:
#elif defined(OS_MAC)
_inner_store_4x4_lib4:
#endif
#endif

	vst1.64		{d8, d9}, [r4], r5
	vst1.64		{d10, d11}, [r4], r5
	vst1.64		{d12, d13}, [r4], r5
	vst1.64		{d14, d15}, [r4], r5

#if MACRO_LEVEL>=1
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_store_4x4_lib, .-inner_store_4x4_lib
#endif
#endif





	.align 3
99: // 0
	.word 0
	.word 0




//                                 r0        r1             r2         r3         sp+0          sp+4       sp+8     sp+12      sp+16
// void kernel_dgemm_nt_4x4_lib44cc(int kmax, double *alpha, double *A, double *B, double *beta, double *C, int ldc, double *D, int ldd)

//	.p2align 4,,15
#if defined(OS_LINUX)
	.global	kernel_sgemm_nt_4x4_lib44cc
	.type	kernel_sgemm_nt_4x4_lib44cc, %function
kernel_sgemm_nt_4x4_lib44cc:
#elif defined(OS_MAC)
	.global	kernel_sgemm_nt_4x4_lib44cc
_kernel_sgemm_nt_4x4_lib44cc:
#endif

	PROLOGUE


	// zero accumulation registers
	vldr	d8, 99b
	vldr	d9, 99b
	vmov	q5, q4
	vmov	q6, q4
	vmov	q7, q4



	// call inner kernel dgemm nt
	mov		r4, r0 // kmax
	mov		r5, r2 // A
	mov		r6, r3 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_4X4_LIB4
#else
#if defined(OS_LINUX)
	bl	inner_kernel_gemm_add_nt_4x4_lib4
#elif defined(OS_MAC)
	bl	_inner_kernel_gemm_add_nt_4x4_lib4
#endif
#endif



	// call inner blend for generic alpha and beta
	mov		r4, r1 // alpha
	ldr		r5, [fp, #0] // beta
	ldr		r6, [fp, #4] // C
	ldr		r7, [fp, #8] // ldc
	lsl		r7, r7, #2 // ldc*sizeof(float)

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB 99f
#else
#if defined(OS_LINUX)
	bl inner_scale_ab_4x4_lib
#elif defined(OS_MAC)
	bl _inner_scale_ab_4x4_lib
#endif
#endif



	// store n
	ldr		r4, [fp, #12] // D
	ldr		r5, [fp, #16] // ldd
	lsl		r5, r5, #2 // ldd*sizeof(float)

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
#if defined(OS_LINUX)
	bl inner_store_4x4_lib
#elif defined(OS_MAC)
	bl _inner_store_4x4_lib
#endif
#endif



	EPILOGUE

#if defined(OS_LINUX)
	.size	kernel_sgemm_nt_4x4_lib44cc, .-kernel_sgemm_nt_4x4_lib44cc
#endif





	.align 3
99: // 0
	.word 0
	.word 0






