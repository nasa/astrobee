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
// r6   <- B
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NT_4X4_LIB4
#else
//	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_gemm_add_nt_4x4_lib4, %function
inner_kernel_gemm_add_nt_4x4_lib4:
#elif defined(OS_MAC)
_inner_kernel_gemm_add_nt_4x4_lib4:
#endif
#endif

	// early return
	cmp		r4, #0
	ble		2f // return

	// prefetch
	pld		[r5, #0]
	pld		[r6, #0]
#if defined(TARGET_ARMV7A_ARM_CORTEX_A9) | defined(TARGET_ARMV7A_ARM_CORTEX_A7)
#if defined(TARGET_ARMV7A_ARM_CORTEX_A9)
	pld		[r5, #32]
	pld		[r6, #32]
#endif
	pld		[r5, #64]
	pld		[r6, #64]
#if defined(TARGET_ARMV7A_ARM_CORTEX_A9)
	pld		[r5, #96]
	pld		[r6, #96]
#endif
	pld		[r5, #64]
#else // cortex a15
	// preload
	vld1.64		{d0, d1}, [r5:128]! // A
	vld1.64		{d4, d5}, [r6:128]! // B
#endif

	cmp		r4, #4
	ble		0f // consider clean up loop

	// main loop
1:
	
#if defined(TARGET_ARMV7A_ARM_CORTEX_A9) | defined(TARGET_ARMV7A_ARM_CORTEX_A7)

	vld1.64		{d0, d1}, [r6:128]! // B
	vld1.64		{d16, d17}, [r5:128]! // A

	vld1.64		{d2, d3}, [r6:128]! // B
	vld1.64		{d18, d19}, [r5:128]! // A

	vld1.64		{d4, d5}, [r6:128]! // B
	vld1.64		{d20, d21}, [r5:128]! // A

	vld1.64		{d6, d7}, [r6:128]! // B
	vld1.64		{d22, d23}, [r5:128]! // A

	// prefetch

	// unroll 0
	vmla.f32	q4, q8, d0[0]
	pld		[r6, #64]
	vmla.f32	q5, q8, d0[1]
	pld		[r5, #64]
	vmla.f32	q6, q8, d1[0]
#if defined(TARGET_ARMV7A_ARM_CORTEX_A9)
	pld		[r6, #96]
#endif
	vmla.f32	q7, q8, d1[1]
#if defined(TARGET_ARMV7A_ARM_CORTEX_A9)
	pld		[r5, #96]
#endif

	// unroll 1
	vmla.f32	q4, q9, d2[0]
	vmla.f32	q5, q9, d2[1]
	vmla.f32	q6, q9, d3[0]
	vmla.f32	q7, q9, d3[1]

	// unroll 2
	vmla.f32	q4, q10, d4[0]
	vmla.f32	q5, q10, d4[1]
	vmla.f32	q6, q10, d5[0]
	vmla.f32	q7, q10, d5[1]

	// unroll 3
	vmla.f32	q4, q11, d6[0]
	vmla.f32	q5, q11, d6[1]
	vmla.f32	q6, q11, d7[0]
	vmla.f32	q7, q11, d7[1]

	sub		r4, r4, #4

#else // cortex a15

	// prefetch
	pld		[r5, #64]
	pld		[r6, #64]

	// unroll 0
	vmla.f32	q4, q0, d4[0]
	vld1.64		{d2, d3}, [r5:128]! // A
	vmla.f32	q5, q0, d4[1]
	vld1.64		{d6, d7}, [r6:128]! // B
	vmla.f32	q6, q0, d5[0]
	vmla.f32	q7, q0, d5[1]

	// unroll 1
	vmla.f32	q4, q1, d6[0]
	vld1.64		{d0, d1}, [r5:128]! // A
	vmla.f32	q5, q1, d6[1]
	vld1.64		{d4, d5}, [r6:128]! // B
	vmla.f32	q6, q1, d7[0]
	vmla.f32	q7, q1, d7[1]

	// unroll 2
	vmla.f32	q4, q0, d4[0]
	vld1.64		{d2, d3}, [r5:128]! // A
	vmla.f32	q5, q0, d4[1]
	vld1.64		{d6, d7}, [r6:128]! // B
	vmla.f32	q6, q0, d5[0]
	vmla.f32	q7, q0, d5[1]

	// unroll 3
	vmla.f32	q4, q1, d6[0]
	vld1.64		{d0, d1}, [r5:128]! // A
	vmla.f32	q5, q1, d6[1]
	vld1.64		{d4, d5}, [r6:128]! // B
	vmla.f32	q6, q1, d7[0]
	vmla.f32	q7, q1, d7[1]

	sub		r4, r4, #4

#endif

	cmp		r4, #4
	bgt		1b

0:

	cmp		r4, #3
	ble		4f

#if defined(TARGET_ARMV7A_ARM_CORTEX_A9) | defined(TARGET_ARMV7A_ARM_CORTEX_A7)

	vld1.64		{d0, d1}, [r6:128]! // B
	vld1.64		{d16, d17}, [r5:128]! // A

	vld1.64		{d2, d3}, [r6:128]! // B
	vld1.64		{d18, d19}, [r5:128]! // A

	vld1.64		{d4, d5}, [r6:128]! // B
	vld1.64		{d20, d21}, [r5:128]! // A

	vld1.64		{d6, d7}, [r6:128]! // B
	vld1.64		{d22, d23}, [r5:128]! // A

	// prefetch

	// unroll 0
	vmla.f32	q4, q8, d0[0]
//	pld		[r5, #64]
	vmla.f32	q5, q8, d0[1]
//	pld		[r6, #64]
	vmla.f32	q6, q8, d1[0]
	vmla.f32	q7, q8, d1[1]

	// unroll 1
	vmla.f32	q4, q9, d2[0]
	vmla.f32	q5, q9, d2[1]
	vmla.f32	q6, q9, d3[0]
	vmla.f32	q7, q9, d3[1]

	// unroll 2
	vmla.f32	q4, q10, d4[0]
	vmla.f32	q5, q10, d4[1]
	vmla.f32	q6, q10, d5[0]
	vmla.f32	q7, q10, d5[1]

	// unroll 3
	vmla.f32	q4, q11, d6[0]
	vmla.f32	q5, q11, d6[1]
	vmla.f32	q6, q11, d7[0]
	vmla.f32	q7, q11, d7[1]

	sub		r4, r4, #4

#else // cortex a15

	// unroll 0
	vmla.f32	q4, q0, d4[0]
	vld1.64		{d2, d3}, [r5:128]! // A
	vmla.f32	q5, q0, d4[1]
	vld1.64		{d6, d7}, [r6:128]! // B
	vmla.f32	q6, q0, d5[0]
	vmla.f32	q7, q0, d5[1]

	// unroll 1
	vmla.f32	q4, q1, d6[0]
	vld1.64		{d0, d1}, [r5:128]! // A
	vmla.f32	q5, q1, d6[1]
	vld1.64		{d4, d5}, [r6:128]! // B
	vmla.f32	q6, q1, d7[0]
	vmla.f32	q7, q1, d7[1]

	// unroll 2
	vmla.f32	q4, q0, d4[0]
	vld1.64		{d2, d3}, [r5:128]! // A
	vmla.f32	q5, q0, d4[1]
	vld1.64		{d6, d7}, [r6:128]! // B
	vmla.f32	q6, q0, d5[0]
	vmla.f32	q7, q0, d5[1]

	// unroll 3
	vmla.f32	q4, q1, d6[0]
//	vld1.64		{d0, d1}, [r5:128]! // A
	vmla.f32	q5, q1, d6[1]
//	vld1.64		{d4, d5}, [r6:128]! // B
	vmla.f32	q6, q1, d7[0]
	vmla.f32	q7, q1, d7[1]

	sub		r4, r4, #4

#endif

	b		2f // return

4: // consider clean1-up loop

	cmp		r4, #0
	ble		2f // return

#if defined(TARGET_ARMV7A_ARM_CORTEX_A9) | defined(TARGET_ARMV7A_ARM_CORTEX_A7)

#else // cortex a15
	sub		r5, r5, #16
	sub		r6, r6, #16
#endif

3: // clean1-up loop

	// unroll 0
	vld1.64		{d0, d1}, [r5:128]! // A
	vld1.64		{d4, d5}, [r6:128]! // B

	vmla.f32	q4, q0, d4[0]
	vmla.f32	q5, q0, d4[1]
	vmla.f32	q6, q0, d5[0]
	vmla.f32	q7, q0, d5[1]

	sub		r4, r4, #1
	cmp		r4, #0
	bgt		3b

2: // return

	
#if MACRO_LEVEL>=2
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_kernel_gemm_add_nt_4x4_lib4, .-inner_kernel_gemm_add_nt_4x4_lib4
#endif
#endif





// subroutine
//
// input arguments:
// r4   <- k
// r5   <- A
// r6   <- B
// r7   <- 4*sdb*sizeof(float)
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NN_4X4_LIB4
#else
//	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_kernel_gemm_add_nn_4x4_lib4, %function
inner_kernel_gemm_add_nn_4x4_lib4:
#elif defined(OS_MAC)
_inner_kernel_gemm_add_nn_4x4_lib4:
#endif
#endif

	// early return
	cmp		r4, #0
	ble		2f // return

	// prefetch
	pld		[r5, #0]
	pld		[r6, #0]
#if defined(TARGET_ARMV7A_ARM_CORTEX_A9) | defined(TARGET_ARMV7A_ARM_CORTEX_A7)
#if defined(TARGET_ARMV7A_ARM_CORTEX_A9)
	add		r8, r7, r7
	pld		[r5, #32]
	add		r9, r7, #32
	pld		[r6, #32]
#endif
	pld		[r5, #64]
	pld		[r6, r7]
#if defined(TARGET_ARMV7A_ARM_CORTEX_A9)
	pld		[r5, #96]
	pld		[r6, r9]
	add		r9, r9, r7
#endif
	pld		[r5, #64]
#else // cortex a15
	// preload
	vld1.64		{d0, d1}, [r5:128]! // A
	vldr		d4, [r6, #0]   // B[0,1]
	vldr		d5, [r6, #16]  // B[4,5]
	vldr		d6, [r6, #32]  // B[8,9]
	vldr		d7, [r6, #48]  // B[12,13]
#endif

	cmp		r4, #4
	ble		0f // consider clean up loop

	// main loop
1:
	
#if defined(TARGET_ARMV7A_ARM_CORTEX_A9) | defined(TARGET_ARMV7A_ARM_CORTEX_A7)

	// prefetch

	vld1.64		{d0, d1, d2, d3}, [r6:128]! // B
	vld1.64		{d16, d17, d18, d19}, [r5:128]! // A

	vld1.64		{d4, d5, d6, d7}, [r6:128]! // B
	vld1.64		{d20, d21, d22, d23}, [r5:128]! // A

	sub		r6, r6, #64

	// unroll 0
	vmla.f32	q4, q8, d0[0]
	pld		[r6, r8]
	vmla.f32	q5, q8, d2[0]
	pld		[r5, #64]
	vmla.f32	q6, q8, d4[0]
#if defined(TARGET_ARMV7A_ARM_CORTEX_A9)
	pld		[r6, r9]
#endif
	vmla.f32	q7, q8, d6[0]
#if defined(TARGET_ARMV7A_ARM_CORTEX_A9)
	pld		[r5, #96]
#endif

	// unroll 1
	vmla.f32	q4, q9, d0[1]
	vmla.f32	q5, q9, d2[1]
	vmla.f32	q6, q9, d4[1]
	vmla.f32	q7, q9, d6[1]

	// unroll 2
	vmla.f32	q4, q10, d1[0]
	vmla.f32	q5, q10, d3[0]
	vmla.f32	q6, q10, d5[0]
	vmla.f32	q7, q10, d7[0]

	// unroll 3
	vmla.f32	q4, q11, d1[1]
	vmla.f32	q5, q11, d3[1]
	vmla.f32	q6, q11, d5[1]
	vmla.f32	q7, q11, d7[1]

	add		r6, r6, r7
	sub		r4, r4, #4

#else // cortex a15

	// prefetch
	pld		[r5, #64]
	pld		[r6, r7]

	// unroll 0
	vmla.f32	q4, q0, d4[0]
	vld1.64		{d2, d3}, [r5:128]! // A
	vmla.f32	q5, q0, d5[0]
	vmla.f32	q6, q0, d6[0]
	vmla.f32	q7, q0, d7[0]

	// unroll 1
	vld1.64		{d0, d1}, [r5:128]! // A
	vmla.f32	q4, q1, d4[1]
	vldr		d4, [r6, #8]  // B[2,3]
	vmla.f32	q5, q1, d5[1]
	vldr		d5, [r6, #24] // B[6,7]
	vmla.f32	q6, q1, d6[1]
	vldr		d6, [r6, #40] // B[10,11]
	vmla.f32	q7, q1, d7[1]
	vldr		d7, [r6, #56] // B[14,15]

	// unroll 2
	vmla.f32	q4, q0, d4[0]
	vld1.64		{d2, d3}, [r5:128]! // A
	vmla.f32	q5, q0, d5[0]
	add		r6, r6, r7
	vmla.f32	q6, q0, d6[0]
	vmla.f32	q7, q0, d7[0]

	// unroll 3
	vld1.64		{d0, d1}, [r5:128]! // A
	vmla.f32	q4, q1, d4[1]
	vldr		d4, [r6, #0]   // B[0,1]
	vmla.f32	q5, q1, d5[1]
	vldr		d5, [r6, #16]  // B[4,5]
	vmla.f32	q6, q1, d6[1]
	vldr		d6, [r6, #32]  // B[8,9]
	vmla.f32	q7, q1, d7[1]
	vldr		d7, [r6, #48]  // B[12,13]

	sub		r4, r4, #4

#endif

	cmp		r4, #4
	bgt		1b

0:

	cmp		r4, #3
	ble		4f

#if defined(TARGET_ARMV7A_ARM_CORTEX_A9) | defined(TARGET_ARMV7A_ARM_CORTEX_A7)

	vld1.64		{d0, d1, d2, d3}, [r6:128]! // B
	vld1.64		{d16, d17, d18, d19}, [r5:128]! // A

	vld1.64		{d4, d5, d6, d7}, [r6:128]! // B
	vld1.64		{d20, d21, d22, d23}, [r5:128]! // A

	// prefetch

	// unroll 0
	vmla.f32	q4, q8, d0[0]
//	pld		[r5, #64]
	vmla.f32	q5, q8, d2[0]
//	pld		[r6, #64]
	vmla.f32	q6, q8, d4[0]
	vmla.f32	q7, q8, d6[0]

	// unroll 1
	vmla.f32	q4, q9, d0[1]
	vmla.f32	q5, q9, d2[1]
	vmla.f32	q6, q9, d4[1]
	vmla.f32	q7, q9, d6[1]

	// unroll 2
	vmla.f32	q4, q10, d1[0]
	vmla.f32	q5, q10, d3[0]
	vmla.f32	q6, q10, d5[0]
	vmla.f32	q7, q10, d7[0]

	// unroll 3
	vmla.f32	q4, q11, d1[1]
	vmla.f32	q5, q11, d3[1]
	vmla.f32	q6, q11, d5[1]
	vmla.f32	q7, q11, d7[1]

	add		r6, r6, r7
	sub		r4, r4, #4
	sub		r6, r6, #64

#else // cortex a15

	// unroll 0
	vmla.f32	q4, q0, d4[0]
	vld1.64		{d2, d3}, [r5:128]! // A
	vmla.f32	q5, q0, d5[0]
	vmla.f32	q6, q0, d6[0]
	vmla.f32	q7, q0, d7[0]

	// unroll 1
	vld1.64		{d0, d1}, [r5:128]! // A
	vmla.f32	q4, q1, d4[1]
	vldr		d4, [r6, #8]  // B[2,3]
	vmla.f32	q5, q1, d5[1]
	vldr		d5, [r6, #24] // B[6,7]
	vmla.f32	q6, q1, d6[1]
	vldr		d6, [r6, #40] // B[10,11]
	vmla.f32	q7, q1, d7[1]
	vldr		d7, [r6, #56] // B[14,15]

	// unroll 2
	vmla.f32	q4, q0, d4[0]
	vld1.64		{d2, d3}, [r5:128]! // A
	vmla.f32	q5, q0, d5[0]
	add		r6, r6, r7
	vmla.f32	q6, q0, d6[0]
	vmla.f32	q7, q0, d7[0]

	// unroll 3
//	vld1.64		{d0, d1}, [r5:128]! // A
	vmla.f32	q4, q1, d4[1]
//	vldr		d4, [r6, #0]   // B[0,1]
	vmla.f32	q5, q1, d5[1]
//	vldr		d5, [r6, #16]  // B[4,5]
	vmla.f32	q6, q1, d6[1]
//	vldr		d6, [r6, #32]  // B[8,9]
	vmla.f32	q7, q1, d7[1]
//	vldr		d7, [r6, #48]  // B[12,13]

	sub		r4, r4, #4

#endif

	b		2f // return

4: // consider clean1-up loop

	cmp		r4, #0
	ble		2f // return

#if defined(TARGET_ARMV7A_ARM_CORTEX_A9) | defined(TARGET_ARMV7A_ARM_CORTEX_A7)

#else // cortex a15
	sub		r5, r5, #16
#endif

3: // clean1-up loop

	// unroll 0
#if defined(TARGET_ARMV7A_ARM_CORTEX_A9) | defined(TARGET_ARMV7A_ARM_CORTEX_A7)

	vld1.64		{d0, d1}, [r5:128]! // A
	vldr		s8, [r6, #0]  // B[0]
	vldr		s9, [r6, #16] // B[4]
	vldr		s10, [r6, #32] // B[8]
	vldr		s11, [r6, #48] // B[12]
	vmla.f32	q4, q0, d4[0]
	vmla.f32	q5, q0, d4[1]
	vmla.f32	q6, q0, d5[0]
	vmla.f32	q7, q0, d5[1]

#else // cortex a15

	vld1.64		{d0, d1}, [r5:128]! // A
	vldr		s8, [r6, #0]  // B[0]
	vmla.f32	q4, q0, d4[0]
	vldr		s8, [r6, #16] // B[4]
	vmla.f32	q5, q0, d4[0]
	vldr		s8, [r6, #32] // B[8]
	vmla.f32	q6, q0, d4[0]
	vldr		s8, [r6, #48] // B[12]
	vmla.f32	q7, q0, d4[0]

#endif

	sub		r4, r4, #1
	add		r6, r6, #4
	cmp		r4, #0
	bgt		3b

2: // return

	
#if MACRO_LEVEL>=2
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_kernel_gemm_add_nn_4x4_lib4, .-inner_kernel_gemm_add_nn_4x4_lib4
#endif
#endif





// subroutine
//
// input arguments:
// r4   <- k
// r5   <- A
// r6   <- B
// r7   <- bs*sdb*sizeof(float)
// r8   <- offsetB

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_GEMM_ADD_NN_4X4_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_edge_gemm_add_nn_4x4_lib4, %function
inner_edge_gemm_add_nn_4x4_lib4:
#elif defined(OS_MAC)
_inner_edge_gemm_add_nn_4x4_lib4:
#endif
#endif

	cmp		r8, #0
	ble		2f // return

	cmp		r4, #0
	ble		2f // return


	rsb		r9, r8, #4 // 4-offsetB
	cmp		r9, r4
//	ble		0f
//	mov		r9, r4 // kend=min(k,4-offsetB(
//0:
	movgt	r9, r4 // kend=min(k,4-offsetB(
	
//	lsl		r10, r8, #2 // offsetB*sizeof(float)
	add		r6, r6, r8, LSL #2 // B + offsetB*sizeof(float)

1:
#if defined(TARGET_ARMV7A_ARM_CORTEX_A9) | defined(TARGET_ARMV7A_ARM_CORTEX_A7)

	vld1.64		{d0, d1}, [r5:128]! // A
	vldr		s8, [r6, #0]  // B[0]
	vldr		s9, [r6, #16] // B[4]
	vldr		s10, [r6, #32] // B[8]
	vldr		s11, [r6, #48] // B[12]
	vmla.f32	q4, q0, d4[0]
	vmla.f32	q5, q0, d4[1]
	vmla.f32	q6, q0, d5[0]
	vmla.f32	q7, q0, d5[1]

#else

	vld1.64		{d0, d1}, [r5:128]! // A
	vldr		s8, [r6, #0]  // B[0]
	vmla.f32	q4, q0, d4[0]
	vldr		s8, [r6, #16] // B[4]
	vmla.f32	q5, q0, d4[0]
	vldr		s8, [r6, #32] // B[8]
	vmla.f32	q6, q0, d4[0]
	vldr		s8, [r6, #48] // B[12]
	vmla.f32	q7, q0, d4[0]

#endif

	sub		r9, r9, #1
	sub		r4, r4, #1
	add		r6, r6, #4

	cmp		r9, #0
	bgt		1b

	cmp		r4, #0
	ble		2f // return

	add		r6, r6, r7
	sub		r6, r6, #16

2: // return

#if MACRO_LEVEL>=1
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_edge_gemm_add_nn_4x4_lib4, .-inner_edge_gemm_add_nn_4x4_lib4
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
	.macro INNER_EDGE_POTRF_4X4_LIB4 lc_zero
#else
	.align 3
99: // 0
	.word 0
	.word 0

	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_edge_potrf_4x4_lib4, %function
inner_edge_potrf_4x4_lib4:
#elif defined(OS_MAC)
_inner_edge_potrf_4x4_lib4:
#endif
#endif
	
	fconsts		s1, #112 // 1.0
#if MACRO_LEVEL>=1
	flds		s0, \lc_zero // 0.0
#else
	flds		s0, 99b // 0.0
#endif

#if 0 // scalar

	// first column
	fcmpes		s16, s0
	fmstat
	ble			1f
	fsqrts		s16, s16
	fdivs		s2, s1, s16
	fsts		s2, [r4, #0]
2:
	fmuls		s17, s17, s2
	fmuls		s18, s18, s2
	fmuls		s19, s19, s2

	// second column
	fnmacs		s21, s17, s17
	fnmacs		s22, s17, s18
	fnmacs		s23, s17, s19
	fcmpes		s21, s0
	fmstat
	ble			3f
	fsqrts		s21, s21
	fdivs		s2, s1, s21
	fsts		s2, [r4, #4]
4:
	fmuls		s22, s22, s2
	fmuls		s23, s23, s2

	// third column
	fnmacs		s26, s18, s18
	fnmacs		s27, s18, s19
	fnmacs		s26, s22, s22
	fnmacs		s27, s22, s23
	fcmpes		s16, s0
	fmstat
	ble			5f
	fsqrts		s26, s26
	fdivs		s2, s1, s26
	fsts		s2, [r4, #8]
6:
	fmuls		s27, s27, s2

	// fourth column
	fnmacs		s31, s19, s19
	fnmacs		s31, s23, s23
	fnmacs		s31, s27, s27
	fcmpes		s31, s0
	fmstat
	ble			7f
	fsqrts		s31, s31
	fdivs		s2, s1, s31
	fsts		s2, [r4, #12]

#else // vector

	// first column
	fcmpes		s16, s0
	fmstat
	ble			1f
	fsqrts		s2, s16
	fdivs		s2, s1, s2
	fsts		s2, [r4, #0]
2:
	vmul.f32	q4, q4, d1[0]

	// second column
	vmls.f32	q5, q4, d8[1]
	fcmpes		s21, s0
	fmstat
	ble			3f
	fsqrts		s2, s21
	fdivs		s2, s1, s2
	fsts		s2, [r4, #4]
4:
	vmul.f32	q5, q5, d1[0]

	// third column
	vmls.f32	q6, q4, d9[0]
	vmls.f32	q6, q5, d11[0]
	fcmpes		s16, s0
	fmstat
	ble			5f
	fsqrts		s2, s26
	fdivs		s2, s1, s2
	fsts		s2, [r4, #8]
6:
	vmul.f32	q6, q6, d1[0]

	// fourth column
	vmls.f32	q7, q4, d9[1]
	vmls.f32	q7, q5, d11[1]
	vmls.f32	q7, q6, d13[1]
	fcmpes		s31, s0
	fmstat
	ble			7f
	fsqrts		s31, s31
	fdivs		s2, s1, s31
	fsts		s2, [r4, #12]

#endif

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

0:
	
#if MACRO_LEVEL>=1
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_edge_potrf_4x4_lib4, .-inner_edge_potrf_4x4_lib4
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
	.macro INNER_EDGE_TRSM_RLT_INV_4X4_LIB4
#else
	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_edge_trsm_rlt_inv_4x4_lib4, %function
inner_edge_trsm_rlt_inv_4x4_lib4:
#elif defined(OS_MAC)
inner_edge_trsm_rlt_inv_4x4_lib4:
#endif
#endif
	
	// first column
	vldr.32		d0, [r5, #0] // E_inv[0]
	vmul.f32	q4, q4, d0[0];

	// second column
	vldr.32		d0, [r4, #4] // E[1+4*0]
	vmls.f32	q5, q4, d0[0];
	vldr.32		d0, [r5, #4] // E_inv[1]
	vmul.f32	q5, q5, d0[0];

	// thirs column
	vldr.32		d0, [r4, #8] // E[2+4*0]
	vmls.f32	q6, q4, d0[0];
	vldr.32		d0, [r4, #24] // E[2+4*1]
	vmls.f32	q6, q5, d0[0];
	vldr.32		d0, [r5, #8] // E_inv[2]
	vmul.f32	q6, q6, d0[0];

	// fourth column
	vldr.32		d0, [r4, #12] // E[3+4*0]
	vmls.f32	q7, q4, d0[0];
	vldr.32		d0, [r4, #28] // E[3+4*1]
	vmls.f32	q7, q5, d0[0];
	vldr.32		d0, [r4, #44] // E[3+4*2]
	vmls.f32	q7, q6, d0[0];
	vldr.32		d0, [r5, #12] // E_inv[3]
	vmul.f32	q7, q7, d0[0];

#if MACRO_LEVEL>=1
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_edge_trsm_rlt_inv_4x4_lib4, .-inner_edge_trsm_rlt_inv_4x4_lib4
#endif
#endif





// subroutine
//
// input arguments:
// r4   <- alpha
// r5   <- beta
// r6   <- C
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_4X4_LIB4 lc_zero
#else
	.align 3
99: // 0
	.word 0
	.word 0
//	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_scale_ab_4x4_lib4, %function
inner_scale_ab_4x4_lib4:
#elif defined(OS_MAC)
_inner_scale_ab_4x4_lib4:
#endif
#endif

	flds		s8, [r4, #0] // alpha
	flds		s9, [r5, #0] // beta
#if MACRO_LEVEL>=2
	flds		s10, \lc_zero // 0.0
#else
	flds		s10, 99b // 0.0
#endif

	fcmpes		s9, s10
	vmul.f32	q4, q4, d4[0]
	vmul.f32	q5, q5, d4[0]
	vmul.f32	q6, q6, d4[0]
	vmul.f32	q7, q7, d4[0]
	fmstat

	beq		0f // end

	vld1.64		{d0, d1, d2, d3}, [r6:128]!
	vmla.f32	q4, q0, d4[1]
	vmla.f32	q5, q1, d4[1]
	vld1.64		{d0, d1, d2, d3}, [r6:128]!
	vmla.f32	q6, q0, d4[1]
	vmla.f32	q7, q1, d4[1]

0:

#if MACRO_LEVEL>=1
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_scale_ab_4x4_lib4, .-inner_scale_ab_4x4_lib4
#endif
#endif





// subroutine
//
// input arguments:
// r4   <- beta
// r5   <- C
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M1B_4X4_LIB4 lc_zero
#else
	.align 3
99: // 0
	.word 0
	.word 0
//	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_scale_m1b_4x4_lib4, %function
inner_scale_m1b_4x4_lib4:
#elif defined(OS_MAC)
_inner_scale_m1b_4x4_lib4:
#endif
#endif

	flds		s8, [r4, #0] // beta
#if MACRO_LEVEL>=2
	flds		s9, \lc_zero // 0.0
#else
	flds		s9, 99b // 0.0
#endif

	fcmpes		s8, s9
	vneg.f32	q4, q4
	vneg.f32	q5, q5
	vneg.f32	q6, q6
	vneg.f32	q7, q7
	fmstat

	beq			0f // end

	vld1.64		{d0, d1, d2, d3}, [r5:128]!
	vmla.f32	q4, q0, d4[0]
	vmla.f32	q5, q1, d4[0]
	vld1.64		{d0, d1, d2, d3}, [r5:128]!
	vmla.f32	q6, q0, d4[0]
	vmla.f32	q7, q1, d4[0]

0:

#if MACRO_LEVEL>=1
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_scale_m1b_4x4_lib4, .-inner_scale_m1b_4x4_lib4
#endif
#endif





// subroutine
//
// input arguments:
// r4   <- C
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M11_4X4_LIB4
#else
//	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_scale_m11_4x4_lib4, %function
inner_scale_m11_4x4_lib4:
#elif defined(OS_MAC)
_inner_scale_11_4x4_lib4:
#endif
#endif

	vld1.64		{d0, d1, d2, d3}, [r4:128]!
	vsub.f32	q4, q0, q4
	vsub.f32	q5, q1, q5
	vld1.64		{d0, d1, d2, d3}, [r4:128]!
	vsub.f32	q6, q0, q6
	vsub.f32	q7, q1, q7

#if MACRO_LEVEL>=1
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_scale_m11_4x4_lib4, .-inner_scale_m11_4x4_lib4
#endif
#endif





// subroutine
//
// input arguments:
// r4   <- D
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X4_LIB4
#else
//	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_store_4x4_lib4, %function
inner_store_4x4_lib4:
#elif defined(OS_MAC)
_inner_store_4x4_lib4:
#endif
#endif

	vst1.64		{d8, d9, d10, d11}, [r4:128]!
	vst1.64		{d12, d13, d14, d15}, [r4:128]!

#if MACRO_LEVEL>=1
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_store_4x4_lib4, .-inner_store_4x4_lib4
#endif
#endif





// subroutine
//
// input arguments:
// r4   <- D
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X4_L_LIB4
#else
//	.p2align 4,,15
#if defined(OS_LINUX)
	.type inner_store_4x4_l_lib4, %function
inner_store_4x4_l_lib4:
#elif defined(OS_MAC)
_inner_store_4x4_l_lib4:
#endif
#endif

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

#if MACRO_LEVEL>=1
	.endm
#else
	mov		pc, lr // return

#if defined(OS_LINUX)
	.size	inner_store_4x4_l_lib4, .-inner_store_4x4_l_lib4
#endif
#endif





	.align 3
99: // 0
	.word 0
	.word 0





//                               r0        r1             r2         r3         sp+0          sp+4       sp+8
// void kernel_sgemm_nt_4x4_lib4(int kmax, double *alpha, double *A, double *B, double *beta, double *C, double *D)

//	.p2align 4,,15
#if defined(OS_LINUX)
	.global	kernel_sgemm_nt_4x4_lib4
	.type	kernel_sgemm_nt_4x4_lib4, %function
kernel_sgemm_nt_4x4_lib4:
#elif defined(OS_MAC)
	.global	kernel_sgemm_nt_4x4_lib4
_kernel_sgemm_nt_4x4_lib4:
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

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4 99f
#else
#if defined(OS_LINUX)
	bl inner_scale_ab_4x4_lib4
#elif defined(OS_MAC)
	bl _inner_scale_ab_4x4_lib4
#endif
#endif



	// store n
	ldr		r4, [fp, #8] // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
#if defined(OS_LINUX)
	bl inner_store_4x4_lib4
#elif defined(OS_MAC)
	bl _inner_store_4x4_lib4
#endif
#endif



	EPILOGUE

#if defined(OS_LINUX)
	.size	kernel_sgemm_nt_4x4_lib4, .-kernel_sgemm_nt_4x4_lib4
#endif





	.align 3
99: // 0
	.word 0
	.word 0





//                               r0        r1             r2         r3           sp+0       sp+4     sp+8          sp+12      sp+16
// void kernel_sgemm_nn_4x4_lib4(int kmax, double *alpha, double *A, int offsetB, double *B, int sdb, double *beta, double *C, double *D)

//	.p2align 4,,15
#if defined(OS_LINUX)
	.global	kernel_sgemm_nn_4x4_lib4
	.type	kernel_sgemm_nn_4x4_lib4, %function
kernel_sgemm_nn_4x4_lib4:
#elif defined(OS_MAC)
	.global	kernel_sgemm_nn_4x4_lib4
_kernel_sgemm_nn_4x4_lib4:
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
	ldr		r6, [fp, #0] // B
	ldr		r7, [fp, #4] // sdb
	lsl		r7, r7, #4 // 4*sizeof(float)*sdb
	mov		r8, r3 // offsetB

#if MACRO_LEVEL>=1
	INNER_EDGE_GEMM_ADD_NN_4X4_LIB4
#else
#if defined(OS_LINUX)
	bl	inner_edge_gemm_add_nn_4x4_lib4
#elif defined(OS_MAC)
	bl	_inner_edge_gemm_add_nn_4x4_lib4
#endif
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X4_LIB4
#else
#if defined(OS_LINUX)
	bl	inner_kernel_gemm_add_nn_4x4_lib4
#elif defined(OS_MAC)
	bl	_inner_kernel_gemm_add_nn_4x4_lib4
#endif
#endif



	// call inner blend for generic alpha and beta
	mov		r4, r1 // alpha
	ldr		r5, [fp, #8] // beta
	ldr		r6, [fp, #12] // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4 99f
#else
#if defined(OS_LINUX)
	bl inner_scale_ab_4x4_lib4
#elif defined(OS_MAC)
	bl _inner_scale_ab_4x4_lib4
#endif
#endif



	// store n
	ldr		r4, [fp, #16] // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
#if defined(OS_LINUX)
	bl inner_store_4x4_lib4
#elif defined(OS_MAC)
	bl _inner_store_4x4_lib4
#endif
#endif



	EPILOGUE

#if defined(OS_LINUX)
	.size	kernel_sgemm_nn_4x4_lib4, .-kernel_sgemm_nn_4x4_lib4
#endif





	.align 3
99: // { 0 }
	.word 0
	.word 0





//                                      r0        r1         r2         r3         sp+0       sp+4       rsp+8         esp+12
// void kernel_strsm_nt_rl_inv_4x4_lib4(int kmax, double *A, double *B, double *beta, double *C, double *D, double *E, double *inv_diag_E);

//	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_strsm_nt_rl_inv_4x4_lib4
	.type kernel_strsm_nt_rl_inv_4x4_lib4, %function
kernel_strsm_nt_rl_inv_4x4_lib4:
#elif defined(OS_MAC)
	.globl _kernel_strsm_nt_rl_inv_4x4_lib4
_kernel_strsm_nt_rl_inv_4x4_lib4:
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
	mov		r5, r1 // A
	mov		r6, r2 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_4X4_LIB4
#else
#if defined(OS_LINUX)
	bl	inner_kernel_gemm_add_nt_4x4_lib4
#elif defined(OS_MAC)
	bl	_inner_kernel_gemm_add_nt_4x4_lib4
#endif
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		r4, r3 // beta
	ldr		r5, [fp, #0] // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_LIB4
#else
#if defined(OS_LINUX)
	bl inner_scale_m1b_4x4_lib4
#elif defined(OS_MAC)
	bl _inner_scale_m1b_4x4_lib4
#endif
#endif



	// factorization
	ldr		r4, [fp, #8] // E
	ldr		r5, [fp, #12] // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_INV_4X4_LIB4
#else
#if defined(OS_LINUX)
	bl inner_edge_trsm_rlt_inv_4x4_lib4
#elif defined(OS_MAC)
	bl _inner_edge_trsm_rlt_inv_4x4_lib4
#endif
#endif



	// store l
	ldr		r4, [fp, #4] // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
#if defined(OS_LINUX)
	bl inner_store_4x4_lib4
#elif defined(OS_MAC)
	bl _inner_store_4x4_lib4
#endif
#endif



	EPILOGUE

#if defined(OS_LINUX)
	.size	kernel_strsm_nt_rl_inv_4x4_lib4, .-kernel_strsm_nt_rl_inv_4x4_lib4
#endif





	.align 3
99: // 0
	.word 0
	.word 0





//                                  r0        r1         r2         r3         sp+0       sp+4
// void kernel_spotrf_nt_l_4x4_lib4(int kmax, double *A, double *B, double *C, double *D, double *inv_diag_D);

//	.p2align 4,,15
#if defined(OS_LINUX)
	.globl kernel_spotrf_nt_l_4x4_lib4
	.type kernel_spotrf_nt_l_4x4_lib4, %function
kernel_spotrf_nt_l_4x4_lib4:
#elif defined(OS_MAC)
	.globl _kernel_spotrf_nt_l_4x4_lib4
_kernel_spotrf_nt_l_4x4_lib4:
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
	mov		r5, r1 // A
	mov		r6, r2 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_4X4_LIB4
#else
#if defined(OS_LINUX)
	bl	inner_kernel_gemm_add_nt_4x4_lib4
#elif defined(OS_MAC)
	bl	_inner_kernel_gemm_add_nt_4x4_lib4
#endif
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		r4, r3 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_4X4_LIB4
#else
#if defined(OS_LINUX)
	bl inner_scale_m11_4x4_lib4
#elif defined(OS_MAC)
	bl _inner_scale_m11_4x4_lib4
#endif
#endif



	// factorization
	ldr		r4, [fp, #4] // inv_diag_D

#if MACRO_LEVEL>=1
	INNER_EDGE_POTRF_4X4_LIB4 99f
#else
#if defined(OS_LINUX)
	bl inner_edge_potrf_4x4_lib4
#elif defined(OS_MAC)
	bl _inner_edge_potrf_4x4_lib4
#endif
#endif



	// store l
	ldr		r4, [fp, #0] // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_L_LIB4
#else
#if defined(OS_LINUX)
	bl inner_store_4x4_l_lib4
#elif defined(OS_MAC)
	bl _inner_store_4x4_l_lib4
#endif
#endif



	EPILOGUE

#if defined(OS_LINUX)
	.size	kernel_spotrf_nt_l_4x4_lib4, .-kernel_spotrf_nt_l_4x4_lib4
#endif





	.align 3
99: // { 0 }
	.word 0
	.word 0
 




//#if defined(BLAS_API)
#if ( defined(BLAS_API) | ( defined(LA_HIGH_PERFORMANCE) & defined(MF_COLMAJ) ) )

#include "kernel_sgemm_4x4_lib.S"

#endif
