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

//	// prologue
//	stmdb	sp!, {r4 - r10, fp, lr} // save GP registers
//	add		fp, sp, #36 // fp to old sp position
//	fstmfdd	sp!, {d8-d15} // save FP registers
#define PROLOGUE \
	stmdb	sp!, {r4 - r10, fp, lr}; \
	add		fp, sp, #36; \
	fstmfdd	sp!, {d8-d15};
//	// epilogue
//	fldmfdd	sp!, {d8-d15} // load FP registers
//	ldmia	sp!, {r4 - r10, fp, pc} // load GP registers and return
#define EPILOGUE \
	fldmfdd	sp!, {d8-d15}; \
	ldmia	sp!, {r4 - r10, fp, pc};



#if defined(OS_LINUX)
	.text
#elif defined(OS_MAC)
	.section	__TEXT,__text,regular,pure_instructions
#endif



// subroutine
//
// input arguments:
// r4   <- k
// r5   <- A
// r6   <- sda
// r7   <- B
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NT_12X4_LIB4
#else
//	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_gemm_add_nt_12x4_lib4, %function
inner_kernel_gemm_add_nt_12x4_lib4:
#elif defined(OS_MAC)
_inner_kernel_gemm_add_nt_12x4_lib4:
#endif
#endif

	// early return
	cmp		r4, #0
	ble		2f // return

	add		r8, r5, r6 // A1
	add		r9, r8, r6 // A2

	// prefetch
	pld			[r5, #0] // A0
	pld			[r7, #0] // B
	pld			[r8, #0] // A1
	pld			[r9, #0] // A2

	// preload
	vld1.64		{d0, d1}, [r7:128] // B
	vld1.64		{d2, d3}, [r5:128] // A0
	vld1.64		{d4, d5}, [r8:128] // A1
//	vld1.64		{d6, d7}, [r9:128] // A2

	cmp		r4, #4
	ble		0f // consider clean up loop

	// prefetch
	pld			[r5, #64] // A0
	pld			[r7, #64] // B
	pld			[r8, #64] // A1
	pld			[r9, #64] // A2

	// main loop
1:
	
	// unroll 0
	vmla.f32	q4, q1, d0[0]
	vldr		d6, [r9, #0] // A2
	vmla.f32	q5, q1, d0[1]
	vldr		d7, [r9, #8] // A2
	vmla.f32	q6, q1, d1[0]
	pld			[r7, #128]
	vmla.f32	q7, q1, d1[1]
	vldr		d2, [r5, #16] // A0
	vmla.f32	q8, q2, d0[0]
	vldr		d3, [r5, #24] // A0
	vmla.f32	q9, q2, d0[1]
	pld			[r5, #128]
	vmla.f32	q10, q2, d1[0]
	pld			[r8, #128]
	vmla.f32	q11, q2, d1[1]
	vldr		d4, [r7, #16] // B
	vmla.f32	q12, q3, d0[0]
	vldr		d5, [r7, #24] // B
	vmla.f32	q13, q3, d0[1]
	vldr		d0, [r8, #16] // A1
	vmla.f32	q14, q3, d1[0]
	pld			[r9, #128]
	vmla.f32	q15, q3, d1[1]
	vldr		d1, [r8, #24] // A1

	// unroll 1
	vmla.f32	q4, q1, d4[0]
	vldr		d6, [r9, #16] // A2
	vmla.f32	q5, q1, d4[1]
	vldr		d7, [r9, #24] // A2
	vmla.f32	q6, q1, d5[0]
	sub		r4, r4, #4
	vmla.f32	q7, q1, d5[1]
	vldr		d2, [r5, #32] // A0
	vmla.f32	q8, q0, d4[0]
	vldr		d3, [r5, #40] // A0
	vmla.f32	q9, q0, d4[1]
	vmla.f32	q10, q0, d5[0]
	vmla.f32	q11, q0, d5[1]
	vldr		d0, [r7, #32] // B
	vmla.f32	q12, q3, d4[0]
	vldr		d1, [r7, #40] // B
	vmla.f32	q13, q3, d4[1]
	vldr		d4, [r8, #32] // A1
	vmla.f32	q14, q3, d5[0]
	vmla.f32	q15, q3, d5[1]
	vldr		d5, [r8, #40] // A1

	// unroll 2
	vmla.f32	q4, q1, d0[0]
	vldr		d6, [r9, #32] // A2
	vmla.f32	q5, q1, d0[1]
	vldr		d7, [r9, #40] // A2
	vmla.f32	q6, q1, d1[0]
	vmla.f32	q7, q1, d1[1]
	vldr		d2, [r5, #48] // A0
	vmla.f32	q8, q2, d0[0]
	vldr		d3, [r5, #56] // A0
	vmla.f32	q9, q2, d0[1]
	vmla.f32	q10, q2, d1[0]
	add		r5, r5, #64
	vmla.f32	q11, q2, d1[1]
	vldr		d4, [r7, #48] // B
	vmla.f32	q12, q3, d0[0]
	vldr		d5, [r7, #56] // B
	vmla.f32	q13, q3, d0[1]
	vldr		d0, [r8, #48] // A1
	vmla.f32	q14, q3, d1[0]
	add		r7, r7, #64
	vmla.f32	q15, q3, d1[1]
	vldr		d1, [r8, #56] // A1

	// unroll 3
	vmla.f32	q4, q1, d4[0]
	vldr		d6, [r9, #48] // A2
	vmla.f32	q5, q1, d4[1]
	vldr		d7, [r9, #56] // A2
	vmla.f32	q6, q1, d5[0]
	add		r8, r8, #64
	vmla.f32	q7, q1, d5[1]
	vldr		d2, [r5, #0] // A0
	vmla.f32	q8, q0, d4[0]
	vldr		d3, [r5, #8] // A0
	vmla.f32	q9, q0, d4[1]
	add		r9, r9, #64
	vmla.f32	q10, q0, d5[0]
	cmp		r4, #4
	vmla.f32	q11, q0, d5[1]
	vldr		d0, [r7, #0] // B
	vmla.f32	q12, q3, d4[0]
	vldr		d1, [r7, #8] // B
	vmla.f32	q13, q3, d4[1]
	vldr		d4, [r8, #0] // A1
	vmla.f32	q14, q3, d5[0]
	vmla.f32	q15, q3, d5[1]
	vldr		d5, [r8, #8] // A1


	bgt		1b

0:

	cmp		r4, #3
	ble		4f


	// unroll 0
	vmla.f32	q4, q1, d0[0]
	vldr		d6, [r9, #0] // A2
	vmla.f32	q5, q1, d0[1]
	vldr		d7, [r9, #8] // A2
	vmla.f32	q6, q1, d1[0]
	pld			[r7, #64]
	vmla.f32	q7, q1, d1[1]
	vldr		d2, [r5, #16] // A0
	vmla.f32	q8, q2, d0[0]
	vldr		d3, [r5, #24] // A0
	vmla.f32	q9, q2, d0[1]
	pld			[r5, #64]
	vmla.f32	q10, q2, d1[0]
	pld			[r8, #64]
	vmla.f32	q11, q2, d1[1]
	vldr		d4, [r7, #16] // B
	vmla.f32	q12, q3, d0[0]
	vldr		d5, [r7, #24] // B
	vmla.f32	q13, q3, d0[1]
	vldr		d0, [r8, #16] // A1
	vmla.f32	q14, q3, d1[0]
	pld			[r9, #64]
	vmla.f32	q15, q3, d1[1]
	vldr		d1, [r8, #24] // A1

	// unroll 1
	vmla.f32	q4, q1, d4[0]
	vldr		d6, [r9, #16] // A2
	vmla.f32	q5, q1, d4[1]
	vldr		d7, [r9, #24] // A2
	vmla.f32	q6, q1, d5[0]
	sub		r4, r4, #4
	vmla.f32	q7, q1, d5[1]
	vldr		d2, [r5, #32] // A0
	vmla.f32	q8, q0, d4[0]
	vldr		d3, [r5, #40] // A0
	vmla.f32	q9, q0, d4[1]
	vmla.f32	q10, q0, d5[0]
	vmla.f32	q11, q0, d5[1]
	vldr		d0, [r7, #32] // B
	vmla.f32	q12, q3, d4[0]
	vldr		d1, [r7, #40] // B
	vmla.f32	q13, q3, d4[1]
	vldr		d4, [r8, #32] // A1
	vmla.f32	q14, q3, d5[0]
	vmla.f32	q15, q3, d5[1]
	vldr		d5, [r8, #40] // A1

	// unroll 2
	vmla.f32	q4, q1, d0[0]
	vldr		d6, [r9, #32] // A2
	vmla.f32	q5, q1, d0[1]
	vldr		d7, [r9, #40] // A2
	vmla.f32	q6, q1, d1[0]
	vmla.f32	q7, q1, d1[1]
	vldr		d2, [r5, #48] // A0
	vmla.f32	q8, q2, d0[0]
	vldr		d3, [r5, #56] // A0
	vmla.f32	q9, q2, d0[1]
	vmla.f32	q10, q2, d1[0]
	add		r5, r5, #64
	vmla.f32	q11, q2, d1[1]
	vldr		d4, [r7, #48] // B
	vmla.f32	q12, q3, d0[0]
	vldr		d5, [r7, #56] // B
	vmla.f32	q13, q3, d0[1]
	vldr		d0, [r8, #48] // A1
	vmla.f32	q14, q3, d1[0]
	add		r7, r7, #64
	vmla.f32	q15, q3, d1[1]
	vldr		d1, [r8, #56] // A1

	// unroll 3
	vmla.f32	q4, q1, d4[0]
	vldr		d6, [r9, #48] // A2
	vmla.f32	q5, q1, d4[1]
	vldr		d7, [r9, #56] // A2
	vmla.f32	q6, q1, d5[0]
	add		r9, r9, #64
	vmla.f32	q7, q1, d5[1]
//	vldr		d2, [r5, #0] // A0
	vmla.f32	q8, q0, d4[0]
//	vldr		d3, [r5, #8] // A0
	vmla.f32	q9, q0, d4[1]
	vmla.f32	q10, q0, d5[0]
	add		r8, r8, #64
	vmla.f32	q11, q0, d5[1]
//	vldr		d0, [r7, #0] // B
	vmla.f32	q12, q3, d4[0]
//	vldr		d1, [r7, #8] // B
	vmla.f32	q13, q3, d4[1]
//	vldr		d4, [r8, #0] // A1
	vmla.f32	q14, q3, d5[0]
	vmla.f32	q15, q3, d5[1]
//	vldr		d5, [r8, #8] // A1


	b		2f // return

4: // consider clean1-up loop

	cmp		r4, #0
	ble		2f // return


3: // clean1-up loop

	// unroll 0
	vld1.64		{d4, d5}, [r7:128]! // B
	vld1.64		{d0, d1}, [r5:128]! // A0
	vmla.f32	q4, q0, d4[0]
	sub		r4, r4, #1
	vmla.f32	q5, q0, d4[1]
	vmla.f32	q6, q0, d5[0]
	vmla.f32	q7, q0, d5[1]
	vld1.64		{d0, d1}, [r8:128]! // A1
	vmla.f32	q8, q0, d4[0]
	vmla.f32	q9, q0, d4[1]
	vmla.f32	q10, q0, d5[0]
	vmla.f32	q11, q0, d5[1]
	vld1.64		{d0, d1}, [r8:128]! // A1
	vmla.f32	q12, q0, d4[0]
	vmla.f32	q13, q0, d4[1]
	vmla.f32	q14, q0, d5[0]
	vmla.f32	q15, q0, d5[1]

	cmp		r4, #0
	bgt		3b

2: // return

	
#if MACRO_LEVEL>=2
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_kernel_gemm_add_nt_12x4_lib4, .-inner_kernel_gemm_add_nt_12x4_lib4
#endif
#endif





// subroutine
//
// cholesky factorization 
//
// input arguments:
// r4   <- inv_diag_D
//
// output arguments:
// r4   <- inv_diag_D

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_POTRF_12X4_LIB4 lc_zero
#else
	.align 3
99: // { 0 }
	.word 0
	.word 0

	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_edge_potrf_12x4_lib4, %function
inner_edge_potrf_12x4_lib4:
#elif defined(OS_MAC)
_inner_edge_potrf_12x4_lib4:
#endif
#endif
	
	fconsts		s1, #112 // 1.0
#if MACRO_LEVEL>=1
	flds		s0, \lc_zero // 0.0
#else
	flds		s0, 99b // 0.0
#endif

	// first column
	fcmpes		s16, s0
	fmstat
	ble			1f
	fsqrts		s2, s16
	fdivs		s2, s1, s2
	fsts		s2, [r4, #0]
2:
	vmul.f32	q4, q4, d1[0]
	vmul.f32	q8, q8, d1[0]
	vmul.f32	q12, q12, d1[0]

	// second column
	vmls.f32	q5, q4, d8[1]
	vmls.f32	q9, q8, d8[1]
	vmls.f32	q13, q12, d8[1]
	fcmpes		s21, s0
	fmstat
	ble			3f
	fsqrts		s2, s21
	fdivs		s2, s1, s2
	fsts		s2, [r4, #4]
4:
	vmul.f32	q5, q5, d1[0]
	vmul.f32	q9, q9, d1[0]
	vmul.f32	q13, q13, d1[0]

	// third column
	vmls.f32	q6, q4, d9[0]
	vmls.f32	q10, q8, d9[0]
	vmls.f32	q14, q12, d9[0]
	vmls.f32	q6, q5, d11[0]
	vmls.f32	q10, q9, d11[0]
	vmls.f32	q14, q13, d11[0]
	fcmpes		s16, s0
	fmstat
	ble			5f
	fsqrts		s2, s26
	fdivs		s2, s1, s2
	fsts		s2, [r4, #8]
6:
	vmul.f32	q6, q6, d1[0]
	vmul.f32	q10, q10, d1[0]
	vmul.f32	q14, q14, d1[0]

	// fourth column
	vmls.f32	q7, q4, d9[1]
	vmls.f32	q11, q8, d9[1]
	vmls.f32	q15, q12, d9[1]
	vmls.f32	q7, q5, d11[1]
	vmls.f32	q11, q9, d11[1]
	vmls.f32	q15, q13, d11[1]
	vmls.f32	q7, q6, d13[1]
	vmls.f32	q11, q10, d13[1]
	vmls.f32	q15, q14, d13[1]
	fcmpes		s31, s0
	fmstat
	ble			7f
	fsqrts		s31, s31
	fdivs		s2, s1, s31
	fsts		s2, [r4, #12]
8:
	vmul.f32	q11, q11, d1[0]
	vmul.f32	q15, q15, d1[0]

	b			0f

1:
#if MACRO_LEVEL>=1
	flds		s16, \lc_zero // 0.0
#else
	flds		s16, 99b // 0.0
#endif
	b			2b

3:
#if MACRO_LEVEL>=1
	flds		s21, \lc_zero // 0.0
#else
	flds		s21, 99b // 0.0
#endif
	b			4b

5:
#if MACRO_LEVEL>=1
	flds		s26, \lc_zero // 0.0
#else
	flds		s26, 99b // 0.0
#endif
	b			6b

7:
#if MACRO_LEVEL>=1
	flds		s31, \lc_zero // 0.0
#else
	flds		s31, 99b // 0.0
#endif
	b			8b

0:
	
#if MACRO_LEVEL>=1
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_edge_potrf_12x4_lib4, .-inner_edge_potrf_12x4_lib4
#endif
#endif




// subroutine
//
// triangular substitution:
// side = right
// uplo = lower
// tran = transposed
// requires explicit inverse of diagonal
//
// input arguments:
// r4   <- E
// r5   <- inv_diag_E
//
// output arguments:
// r4   <- E
// r5   <- inv_diag_E

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRSM_RLT_INV_12X4_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_edge_trsm_rlt_inv_12x4_lib4, %function
inner_edge_trsm_rlt_inv_12x4_lib4:
#elif defined(OS_MAC)
inner_edge_trsm_rlt_inv_12x4_lib4:
#endif
#endif
	
	// first column
	vldr.32		d0, [r5, #0] // E_inv[0]
	vmul.f32	q4, q4, d0[0];
	vmul.f32	q8, q8, d0[0];
	vmul.f32	q12, q12, d0[0];

	// second column
	vldr.32		d0, [r4, #4] // E[1+4*0]
	vmls.f32	q5, q4, d0[0];
	vmls.f32	q9, q8, d0[0];
	vmls.f32	q13, q12, d0[0];
	vldr.32		d0, [r5, #4] // E_inv[1]
	vmul.f32	q5, q5, d0[0];
	vmul.f32	q9, q9, d0[0];
	vmul.f32	q13, q13, d0[0];

	// thirs column
	vldr.32		d0, [r4, #8] // E[2+4*0]
	vmls.f32	q6, q4, d0[0];
	vmls.f32	q10, q8, d0[0];
	vmls.f32	q14, q12, d0[0];
	vldr.32		d0, [r4, #24] // E[2+4*1]
	vmls.f32	q6, q5, d0[0];
	vmls.f32	q10, q9, d0[0];
	vmls.f32	q14, q13, d0[0];
	vldr.32		d0, [r5, #8] // E_inv[2]
	vmul.f32	q6, q6, d0[0];
	vmul.f32	q10, q10, d0[0];
	vmul.f32	q14, q14, d0[0];

	// fourth column
	vldr.32		d0, [r4, #12] // E[3+4*0]
	vmls.f32	q7, q4, d0[0];
	vmls.f32	q11, q8, d0[0];
	vmls.f32	q15, q12, d0[0];
	vldr.32		d0, [r4, #28] // E[3+4*1]
	vmls.f32	q7, q5, d0[0];
	vmls.f32	q11, q9, d0[0];
	vmls.f32	q15, q13, d0[0];
	vldr.32		d0, [r4, #44] // E[3+4*2]
	vmls.f32	q7, q6, d0[0];
	vmls.f32	q11, q10, d0[0];
	vmls.f32	q15, q14, d0[0];
	vldr.32		d0, [r5, #12] // E_inv[3]
	vmul.f32	q7, q7, d0[0];
	vmul.f32	q11, q11, d0[0];
	vmul.f32	q15, q15, d0[0];

#if MACRO_LEVEL>=1
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_edge_trsm_rlt_inv_12x4_lib4, .-inner_edge_trsm_rlt_inv_12x4_lib4
#endif
#endif





	.align 3
99: // { 0 }
	.word 0
	.word 0





// subroutine
//
// input arguments:
// r4   <- alpha
// r5   <- beta
// r6   <- C
// r7   <- sdc
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_12X4_LIB4 lc_zero
#else
	.align 3
99: // 00
	.word 0
	.word 0

//	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_scale_ab_12x4_lib4, %function
inner_scale_ab_12x4_lib4:
#elif defined(OS_MAC)
_inner_scale_ab_12x4_lib4:
#endif
#endif

	flds		s8, [r4, #0] // alpha

	vmul.f32	q4, q4, d4[0]
	flds		s9, [r5, #0] // beta
	vmul.f32	q5, q5, d4[0]
#if MACRO_LEVEL>=1
	flds		s10, \lc_zero // 0.0
#else
	flds		s10, 99b // 0.0
#endif
	vmul.f32	q6, q6, d4[0]
	vmul.f32	q7, q7, d4[0]
	fcmpes		s9, s10
	vmul.f32	q8, q8, d4[0]
	vmul.f32	q9, q9, d4[0]
	vmul.f32	q10, q10, d4[0]
	vmul.f32	q11, q11, d4[0]
	vmul.f32	q12, q12, d4[0]
	vmul.f32	q13, q13, d4[0]
	vmul.f32	q14, q14, d4[0]
	vmul.f32	q15, q15, d4[0]
	fmstat

	beq		0f // end

	add		r8, r6, r7
	add		r9, r8, r7

	vld1.64		{d0, d1, d2, d3}, [r6:128]!
	vmla.f32	q4, q0, d4[1]
	vmla.f32	q5, q1, d4[1]
	vld1.64		{d0, d1, d2, d3}, [r6:128]!
	vmla.f32	q6, q0, d4[1]
	vmla.f32	q7, q1, d4[1]
	vld1.64		{d0, d1, d2, d3}, [r8:128]!
	vmla.f32	q8, q0, d4[1]
	vmla.f32	q9, q1, d4[1]
	vld1.64		{d0, d1, d2, d3}, [r8:128]!
	vmla.f32	q10, q0, d4[1]
	vmla.f32	q11, q1, d4[1]
	vld1.64		{d0, d1, d2, d3}, [r9:128]!
	vmla.f32	q12, q0, d4[1]
	vmla.f32	q13, q1, d4[1]
	vld1.64		{d0, d1, d2, d3}, [r9:128]!
	vmla.f32	q14, q0, d4[1]
	vmla.f32	q15, q1, d4[1]

0:

#if MACRO_LEVEL>=1
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_scale_ab_12x4_lib4, .-inner_scale_ab_12x4_lib4
#endif
#endif





// subroutine
//
// input arguments:
// r4   <- beta
// r5   <- C
// r7   <- sdc
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M1B_12X4_LIB4 lc_zero
#else
	.align 3
99: // 00
	.word 0
	.word 0

//	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_scale_m1b_12x4_lib4, %function
inner_scale_m1b_12x4_lib4:
#elif defined(OS_MAC)
_inner_scale_m1b_12x4_lib4:
#endif
#endif

	vldr	d0, 99b
	vldr	d1, 99b

	vsub.f32	q4, q0, q4
	vsub.f32	q5, q0, q5
	vsub.f32	q6, q0, q6
	vsub.f32	q7, q0, q7
	vsub.f32	q8, q0, q8
	vsub.f32	q9, q0, q9
	vsub.f32	q10, q0, q10
	vsub.f32	q11, q0, q11
	vsub.f32	q12, q0, q12
	vsub.f32	q13, q0, q13
	vsub.f32	q14, q0, q14
	vsub.f32	q15, q0, q15

	flds		s9, [r4, #0] // beta
#if MACRO_LEVEL>=1
	flds		s10, \lc_zero // 0.0
#else
	flds		s10, 99b // 0.0
#endif
	fcmpes		s9, s10
	fmstat

	beq		0f // end

	add		r8, r5, r6
	add		r9, r8, r7

	vld1.64		{d0, d1, d2, d3}, [r5:128]!
	vmla.f32	q4, q0, d4[1]
	vmla.f32	q5, q1, d4[1]
	vld1.64		{d0, d1, d2, d3}, [r5:128]!
	vmla.f32	q6, q0, d4[1]
	vmla.f32	q7, q1, d4[1]
	vld1.64		{d0, d1, d2, d3}, [r8:128]!
	vmla.f32	q8, q0, d4[1]
	vmla.f32	q9, q1, d4[1]
	vld1.64		{d0, d1, d2, d3}, [r8:128]!
	vmla.f32	q10, q0, d4[1]
	vmla.f32	q11, q1, d4[1]
	vld1.64		{d0, d1, d2, d3}, [r9:128]!
	vmla.f32	q12, q0, d4[1]
	vmla.f32	q13, q1, d4[1]
	vld1.64		{d0, d1, d2, d3}, [r9:128]!
	vmla.f32	q14, q0, d4[1]
	vmla.f32	q15, q1, d4[1]

0:

#if MACRO_LEVEL>=1
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_scale_m1b_12x4_lib4, .-inner_scale_m1b_12x4_lib4
#endif
#endif





// subroutine
//
// input arguments:
// r4   <- C
// r5   <- sdc
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M11_12X4_LIB4
#else
//	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_scale_m11_12x4_lib4, %function
inner_scale_m11_12x4_lib4:
#elif defined(OS_MAC)
_inner_scale_m11_12x4_lib4:
#endif
#endif

	add		r6, r4, r5
	add		r7, r6, r5

	vld1.64		{d0, d1, d2, d3}, [r4:128]!
	vsub.f32	q4, q0, q4
	vsub.f32	q5, q1, q5
	vld1.64		{d0, d1, d2, d3}, [r4:128]!
	vsub.f32	q6, q0, q6
	vsub.f32	q7, q1, q7
	vld1.64		{d0, d1, d2, d3}, [r6:128]!
	vsub.f32	q8, q0, q8
	vsub.f32	q9, q1, q9
	vld1.64		{d0, d1, d2, d3}, [r6:128]!
	vsub.f32	q10, q0, q10
	vsub.f32	q11, q1, q11
	vld1.64		{d0, d1, d2, d3}, [r7:128]!
	vsub.f32	q12, q0, q12
	vsub.f32	q13, q1, q13
	vld1.64		{d0, d1, d2, d3}, [r7:128]!
	vsub.f32	q14, q0, q14
	vsub.f32	q15, q1, q15

#if MACRO_LEVEL>=1
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_scale_m11_12x4_lib4, .-inner_scale_m11_12x4_lib4
#endif
#endif





// subroutine
//
// input arguments:
// r4   <- C
// r5   <- sdc
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_11_12X4_LIB4
#else
//	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_scale_11_12x4_lib4, %function
inner_scale_11_12x4_lib4:
#elif defined(OS_MAC)
_inner_scale_11_12x4_lib4:
#endif
#endif

	add		r6, r4, r5
	add		r7, r6, r5

	vld1.64		{d0, d1, d2, d3}, [r4:128]!
	vadd.f32	q4, q0, q4
	vadd.f32	q5, q1, q5
	vld1.64		{d0, d1, d2, d3}, [r4:128]!
	vadd.f32	q6, q0, q6
	vadd.f32	q7, q1, q7
	vld1.64		{d0, d1, d2, d3}, [r6:128]!
	vadd.f32	q8, q0, q8
	vadd.f32	q9, q1, q9
	vld1.64		{d0, d1, d2, d3}, [r6:128]!
	vadd.f32	q10, q0, q10
	vadd.f32	q11, q1, q11
	vld1.64		{d0, d1, d2, d3}, [r7:128]!
	vadd.f32	q12, q0, q12
	vadd.f32	q13, q1, q13
	vld1.64		{d0, d1, d2, d3}, [r7:128]!
	vadd.f32	q14, q0, q14
	vadd.f32	q15, q1, q15

#if MACRO_LEVEL>=1
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_scale_11_12x4_lib4, .-inner_scale_11_12x4_lib4
#endif
#endif





// subroutine
//
// input arguments:
// r4   <- D
// r5   <- sdd
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_12X4_LIB4
#else
//	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_store_12x4_lib4, %function
inner_store_12x4_lib4:
#elif defined(OS_MAC)
_inner_store_12x4_lib4:
#endif
#endif

	add		r6, r4, r5
	add		r7, r6, r5

	vst1.64		{d8, d9, d10, d11}, [r4:128]!
	vst1.64		{d12, d13, d14, d15}, [r4:128]!
	vst1.64		{d16, d17, d18, d19}, [r6:128]!
	vst1.64		{d20, d21, d22, d23}, [r6:128]!
	vst1.64		{d24, d25, d26, d27}, [r7:128]!
	vst1.64		{d28, d29, d30, d31}, [r7:128]!

#if MACRO_LEVEL>=1
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_store_12x4_lib4, .-inner_store_12x4_lib4
#endif
#endif





// subroutine
//
// input arguments:
// r4   <- D
// r5   <- sdd
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_12X4_L_LIB4
#else
//	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_store_12x4_l_lib4, %function
inner_store_12x4_l_lib4:
#elif defined(OS_MAC)
_inner_store_12x4_l_lib4:
#endif
#endif

	add		r6, r4, r5
	add		r7, r6, r5

	// top 4x4
	// first column
	vstr.64		d8, [r4, #0]
	vstr.64		d9, [r4, #8]
	// second column
	vstr.32		s21, [r4, #20]
	vstr.64		d11, [r4, #24]
	// third column
	vstr.64		d13, [r4, #40]
	// fourth column
	vstr.64		s31, [r4, #60]
	// middle 4x4
	vst1.64		{d16, d17, d18, d19}, [r6:128]!
	vst1.64		{d20, d21, d22, d23}, [r6:128]!
	// bottom 4x4
	vst1.64		{d24, d25, d26, d27}, [r7:128]!
	vst1.64		{d28, d29, d30, d31}, [r7:128]!


#if MACRO_LEVEL>=1
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_store_12x4_l_lib4, .-inner_store_12x4_l_lib4
#endif
#endif





// zero double word
	.align 3
99: // { 0 }
	.word 0
	.word 0
	.word 0
	.word 0





//                                r0        r1             r2         r3       sp+0       sp+4          sp+8       sp+12    sp+16   sp+20
// void kernel_sgemm_nt_12x4_lib4(int kmax, double *alpha, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd)

//	.p2align 4,,15
#if defined(OS_LINUX)
	.global	kernel_sgemm_nt_12x4_lib4
	.type	kernel_sgemm_nt_12x4_lib4, %function
kernel_sgemm_nt_12x4_lib4:
#elif defined(OS_MAC)
	.global	kernel_sgemm_nt_12x4_lib4
_kernel_sgemm_nt_12x4_lib4:
#endif

	// prologue

	// save GP registers
	stmdb	sp!, {r4 - r10, fp, lr} // save registers
	add		fp, sp, #36 // fp to old sp position

	// save FP registers
	fstmfdd	sp!, {d8-d15}



	// zero accumulation registers
	vldr	d8, 99b
	vldr	d9, 99b
	vmov	q5, q4
	vmov	q6, q4
	vmov	q7, q4
	vmov	q8, q4
	vmov	q9, q4
	vmov	q10, q4
	vmov	q11, q4
	vmov	q12, q4
	vmov	q13, q4
	vmov	q14, q4
	vmov	q15, q4



	// call inner kernel dgemm nt
	mov		r4, r0 // kmax
	mov		r5, r2 // A
	mov		r6, r3 // sda
	lsl		r6, r6, #4 // 4*sizeof(float)*sda
	ldr		r7, [fp, #0] // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_12X4_LIB4
#else
#if defined(OS_LINUX)
	bl	inner_kernel_gemm_add_nt_12x4_lib4
#elif defined(OS_MAC)
	bl	_inner_kernel_gemm_add_nt_12x4_lib4
#endif
#endif



	// call inner blend for generic alpha and beta
	mov		r4, r1 // alpha
	ldr		r5, [fp, #4] // beta
	ldr		r6, [fp, #8] // C
	ldr		r7, [fp, #12] // sdc
	lsl		r7, r7, #4 // 4*sizeof(float)*sdc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_12X4_LIB4 99f
#else
#if defined(OS_LINUX)
	bl inner_scale_ab_12x4_lib4
#elif defined(OS_MAC)
	bl _inner_scale_ab_12x4_lib4
#endif
#endif



	// store n
	ldr		r4, [fp, #16] // D
	ldr		r5, [fp, #20] // sdd
	lsl		r5, r5, #4 // 4*sizeof(float)*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
#if defined(OS_LINUX)
	bl inner_store_12x4_lib4
#elif defined(OS_MAC)
	bl _inner_store_12x4_lib4
#endif
#endif



	// epilogue

	// load FP registers
	fldmfdd	sp!, {d8-d15}

	// load GP registers and return
//	ldmia	sp!, {r4 - r10, fp, lr} // load registers
//	mov		pc, lr // return
	ldmia	sp!, {r4 - r10, fp, pc} // load registers and return

#if defined(OS_LINUX)
	.size	kernel_sgemm_nt_12x4_lib4, .-kernel_sgemm_nt_12x4_lib4
#endif





	.align 3
99: // 0
	.word 0
	.word 0


//                                      r0        r1         r2       r3         sp+0       sp+4     rsp+8      rsp+12   rsp+16     rsp+20        rsp+24
// void kernel_strsm_nt_rl_inv_12x4_lib4(int kmax, double *A, int sda, double *B, double *beta, double *C, int sdc, double *D, int sdd, double *E, double *inv_diag_E);

//	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_strsm_nt_rl_inv_12x4_lib4
	.type kernel_strsm_nt_rl_inv_12x4_lib4, %function
kernel_strsm_nt_rl_inv_12x4_lib4:
#elif defined(OS_MAC)
	.globl _kernel_strsm_nt_rl_inv_12x4_lib4
_kernel_strsm_nt_rl_inv_12x4_lib4:
#endif

	PROLOGUE



	// zero accumulation registers
	vldr	d8, 99b
	vldr	d9, 99b
	vmov	q5, q4
	vmov	q6, q4
	vmov	q7, q4
	vmov	q8, q4
	vmov	q9, q4
	vmov	q10, q4
	vmov	q11, q4
	vmov	q12, q4
	vmov	q13, q4
	vmov	q14, q4
	vmov	q15, q4



	// call inner kernel dgemm nt
	mov		r4, r0 // kmax
	mov		r5, r1 // A
	mov		r6, r2 // sda
	lsl		r6, r6, #4 // 4*sizeof(float)*sda
	mov		r7, r3 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_12X4_LIB4
#else
#if defined(OS_LINUX)
	bl	inner_kernel_gemm_add_nt_12x4_lib4
#elif defined(OS_MAC)
	bl	_inner_kernel_gemm_add_nt_12x4_lib4
#endif
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	ldr		r4, [fp, #0] // beta
	ldr		r5, [fp, #4] // C
	ldr		r6, [fp, #8] // sdd
	lsl		r6, r6, #4 // 4*sizeof(float)*sdd

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_12X4_LIB4 99f
#else
#if defined(OS_LINUX)
	bl inner_scale_m1b_12x4_lib4
#elif defined(OS_MAC)
	bl _inner_scale_m1b_12x4_lib4
#endif
#endif



	// factorization
	ldr		r4, [fp, #20] // E
	ldr		r5, [fp, #24] // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_INV_12X4_LIB4
#else
#if defined(OS_LINUX)
	bl inner_edge_trsm_rlt_inv_12x4_lib4
#elif defined(OS_MAC)
	bl _inner_edge_trsm_rlt_inv_12x4_lib4
#endif
#endif



	// store l
	ldr		r4, [fp, #12] // D
	ldr		r5, [fp, #16] // sdd
	lsl		r5, r5, #4 // 4*sizeof(float)*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_LIB4
#else
#if defined(OS_LINUX)
	bl inner_store_12x4_lib4
#elif defined(OS_MAC)
	bl _inner_store_12x4_lib4
#endif
#endif



	EPILOGUE

#if defined(OS_LINUX)
	.size	kernel_strsm_nt_rl_inv_12x4_lib4, .-kernel_strsm_nt_rl_inv_12x4_lib4
#endif





	.align 3
99: // 0
	.word 0
	.word 0





//                                  r0        r1         r2       r3         sp+0       sp+4     sp+8       sp+12    sp+16
// void kernel_spotrf_nt_l_12x4_lib4(int kmax, double *A, int sda, double *B, double *C, int sdc, double *D, int sdd, double *inv_diag_D);

//	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_spotrf_nt_l_12x4_lib4
	.type kernel_spotrf_nt_l_12x4_lib4, %function
kernel_spotrf_nt_l_12x4_lib4:
#elif defined(OS_MAC)
	.globl _kernel_spotrf_nt_l_12x4_lib4
_kernel_spotrf_nt_l_12x4_lib4:
#endif

	PROLOGUE



	// zero accumulation registers
	vldr	d8, 99b
	vldr	d9, 99b
	vmov	q5, q4
	vmov	q6, q4
	vmov	q7, q4
	vmov	q8, q4
	vmov	q9, q4
	vmov	q10, q4
	vmov	q11, q4
	vmov	q12, q4
	vmov	q13, q4
	vmov	q14, q4
	vmov	q15, q4



	// call inner kernel dgemm nt
	mov		r4, r0 // kmax
	mov		r5, r1 // A
	mov		r6, r2 // sda
	lsl		r6, r6, #4 // 4*sizeof(float)*sda
	mov		r7, r3 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_12X4_LIB4
#else
#if defined(OS_LINUX)
	bl	inner_kernel_gemm_add_nt_12x4_lib4
#elif defined(OS_MAC)
	bl	_inner_kernel_gemm_add_nt_12x4_lib4
#endif
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	ldr		r4, [fp, #0] // C
	ldr		r5, [fp, #4] // sdd
	lsl		r5, r5, #4 // 4*sizeof(float)*sdd

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_12X4_LIB4
#else
#if defined(OS_LINUX)
	bl inner_scale_m11_12x4_lib4
#elif defined(OS_MAC)
	bl _inner_scale_m11_12x4_lib4
#endif
#endif



	// factorization
	ldr		r4, [fp, #16] // inv_diag_D

#if MACRO_LEVEL>=1
	INNER_EDGE_POTRF_12X4_LIB4 99f
#else
#if defined(OS_LINUX)
	bl inner_edge_potrf_12x4_lib4
#elif defined(OS_MAC)
	bl _inner_edge_potrf_12x4_lib4
#endif
#endif



	// store l
	ldr		r4, [fp, #8] // D
	ldr		r5, [fp, #12] // sdd
	lsl		r5, r5, #4 // 4*sizeof(float)*sdd

#if MACRO_LEVEL>=1
	INNER_STORE_12X4_L_LIB4
#else
#if defined(OS_LINUX)
	bl inner_store_12x4_l_lib4
#elif defined(OS_MAC)
	bl _inner_store_12x4_l_lib4
#endif
#endif



	EPILOGUE

#if defined(OS_LINUX)
	.size	kernel_spotrf_nt_l_12x4_lib4, .-kernel_spotrf_nt_l_12x4_lib4
#endif





	.align 3
99: // { 0 }
	.word 0
	.word 0






