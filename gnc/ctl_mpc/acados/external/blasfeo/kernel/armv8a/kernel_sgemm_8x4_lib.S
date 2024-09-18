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
// w8   <- k
// x9   <- A
// x10  <- sda
// x11   <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_8x4_lib4c)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x19, x9, x10

	add		x13, x12, x12 // 2
	add		x14, x13, x12 // 3
	add		x15, x13, x13 // 4
	add		x16, x14, x13 // 5
	add		x17, x14, x14 // 6
	add		x18, x15, x14 // 7

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x19, #0]
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x11, x14]

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x19, #64]
	prfm	PLDL1KEEP, [x11, x15]
	prfm	PLDL1KEEP, [x11, x16]
	prfm	PLDL1KEEP, [x11, x17]
	prfm	PLDL1KEEP, [x11, x18]

	// main loop
1:
	
	ldr		q28, [x11]
	ldr		q29, [x11, x12]
	ldp		q24, q25, [x9], #32
	ldp		q20, q21, [x19], #32

	ldr		q30, [x11, x13]
	ldr		q31, [x11, x14]
	ldp		q26, q27, [x9], #32
	ldp		q22, q23, [x19], #32

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	add		x11, x11, x15
	fmla	v1.4s, v24.4s, v28.s[1]
	prfm	PLDL1KEEP, [x9, #64]
	fmla	v2.4s, v24.4s, v28.s[2]
	prfm	PLDL1KEEP, [x19, #64]
	fmla	v3.4s, v24.4s, v28.s[3]
	prfm	PLDL1KEEP, [x11, x15]
	fmla	v4.4s, v20.4s, v28.s[0]
	prfm	PLDL1KEEP, [x11, x16]
	fmla	v5.4s, v20.4s, v28.s[1]
	prfm	PLDL1KEEP, [x11, x17]
	fmla	v6.4s, v20.4s, v28.s[2]
	prfm	PLDL1KEEP, [x11, x18]
	fmla	v7.4s, v20.4s, v28.s[3]

	// unroll 1
	fmla	v0.4s, v25.4s, v29.s[0]
	fmla	v1.4s, v25.4s, v29.s[1]
	fmla	v2.4s, v25.4s, v29.s[2]
	fmla	v3.4s, v25.4s, v29.s[3]
	fmla	v4.4s, v21.4s, v29.s[0]
	fmla	v5.4s, v21.4s, v29.s[1]
	fmla	v6.4s, v21.4s, v29.s[2]
	fmla	v7.4s, v21.4s, v29.s[3]

	// unroll 2
	fmla	v0.4s, v26.4s, v30.s[0]
	fmla	v1.4s, v26.4s, v30.s[1]
	fmla	v2.4s, v26.4s, v30.s[2]
	fmla	v3.4s, v26.4s, v30.s[3]
	fmla	v4.4s, v22.4s, v30.s[0]
	fmla	v5.4s, v22.4s, v30.s[1]
	fmla	v6.4s, v22.4s, v30.s[2]
	fmla	v7.4s, v22.4s, v30.s[3]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v31.s[0]
	fmla	v1.4s, v27.4s, v31.s[1]
	fmla	v2.4s, v27.4s, v31.s[2]
	fmla	v3.4s, v27.4s, v31.s[3]
	fmla	v4.4s, v23.4s, v31.s[0]
	fmla	v5.4s, v23.4s, v31.s[1]
	fmla	v6.4s, v23.4s, v31.s[2]
	fmla	v7.4s, v23.4s, v31.s[3]

	cmp		w8, #4
	bgt		1b

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	ldr		q28, [x11]
	ldr		q29, [x11, x12]
	ldp		q24, q25, [x9], #32
	ldp		q20, q21, [x19], #32

	ldr		q30, [x11, x13]
	ldr		q31, [x11, x14]
	ldp		q26, q27, [x9], #32
	ldp		q22, q23, [x19], #32

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	add		x11, x11, x15
	fmla	v1.4s, v24.4s, v28.s[1]
//	prfm	PLDL1KEEP, [x9, #64]
	fmla	v2.4s, v24.4s, v28.s[2]
//	prfm	PLDL1KEEP, [x19, #64]
	fmla	v3.4s, v24.4s, v28.s[3]
//	prfm	PLDL1KEEP, [x11, x15]
	fmla	v4.4s, v20.4s, v28.s[0]
//	prfm	PLDL1KEEP, [x11, x16]
	fmla	v5.4s, v20.4s, v28.s[1]
//	prfm	PLDL1KEEP, [x11, x17]
	fmla	v6.4s, v20.4s, v28.s[2]
//	prfm	PLDL1KEEP, [x11, x18]
	fmla	v7.4s, v20.4s, v28.s[3]

	// unroll 1
	fmla	v0.4s, v25.4s, v29.s[0]
	fmla	v1.4s, v25.4s, v29.s[1]
	fmla	v2.4s, v25.4s, v29.s[2]
	fmla	v3.4s, v25.4s, v29.s[3]
	fmla	v4.4s, v21.4s, v29.s[0]
	fmla	v5.4s, v21.4s, v29.s[1]
	fmla	v6.4s, v21.4s, v29.s[2]
	fmla	v7.4s, v21.4s, v29.s[3]

	// unroll 2
	fmla	v0.4s, v26.4s, v30.s[0]
	fmla	v1.4s, v26.4s, v30.s[1]
	fmla	v2.4s, v26.4s, v30.s[2]
	fmla	v3.4s, v26.4s, v30.s[3]
	fmla	v4.4s, v22.4s, v30.s[0]
	fmla	v5.4s, v22.4s, v30.s[1]
	fmla	v6.4s, v22.4s, v30.s[2]
	fmla	v7.4s, v22.4s, v30.s[3]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v31.s[0]
	fmla	v1.4s, v27.4s, v31.s[1]
	fmla	v2.4s, v27.4s, v31.s[2]
	fmla	v3.4s, v27.4s, v31.s[3]
	fmla	v4.4s, v23.4s, v31.s[0]
	fmla	v5.4s, v23.4s, v31.s[1]
	fmla	v6.4s, v23.4s, v31.s[2]
	fmla	v7.4s, v23.4s, v31.s[3]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9], #16
	ldr		q20, [x19], #16
	ldr		q28, [x11]
	fmla	v0.4s, v24.4s, v28.s[0]
	add		x11, x11, x12
	fmla	v1.4s, v24.4s, v28.s[1]
	fmla	v2.4s, v24.4s, v28.s[2]
	fmla	v3.4s, v24.4s, v28.s[3]
	fmla	v4.4s, v20.4s, v28.s[0]
	fmla	v5.4s, v20.4s, v28.s[1]
	fmla	v6.4s, v20.4s, v28.s[2]
	fmla	v7.4s, v20.4s, v28.s[3]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return




#else // cortex a57 vs a53



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x19, x9, x10 // A1
	add		x20, x11, #28 // B1

	add		x13, x12, x12 // 2
	add		x14, x13, x12 // 3
	add		x15, x13, x13 // 4
//	add		x16, x14, x13 // 5
//	add		x17, x14, x14 // 6
//	add		x18, x15, x14 // 7

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x19, #0]
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x20]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x20, x12]
//	prfm	PLDL1KEEP, [x11, x13]
//	prfm	PLDL1KEEP, [x11, x14]

	// preload
	ldr		q28, [x11]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x20, x13]
	add		x11, x11, x12
	add		x20, x20, x12

	ldr		q29, [x11]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x20, x13]
	add		x11, x11, x12
	add		x20, x20, x12

	ldr		q30, [x11]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x20, x13]
	add		x11, x11, x12
	add		x20, x20, x12

//	ldr		q31, [x11]
//	prfm	PLDL1KEEP, [x11, x13]
//	prfm	PLDL1KEEP, [x20, x13]
//	add		x11, x11, x12
//	add		x20, x20, x12

	ldp		q24, q25, [x9], #32
	ldp		q20, q21, [x19], #32

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #32]
	prfm	PLDL1KEEP, [x19, #32]
//	prfm	PLDL1KEEP, [x11, x15]
//	prfm	PLDL1KEEP, [x11, x16]
//	prfm	PLDL1KEEP, [x11, x17]
//	prfm	PLDL1KEEP, [x11, x18]

	// main loop
1:
	

	// unroll 0
	ldr		q31, [x11]
	fmla	v0.4s, v24.4s, v28.s[0]
	prfm	PLDL1KEEP, [x11, x13]
	fmla	v1.4s, v24.4s, v28.s[1]
	prfm	PLDL1KEEP, [x20, x13]
	fmla	v2.4s, v24.4s, v28.s[2]
	add		x11, x11, x12
	fmla	v3.4s, v24.4s, v28.s[3]
	add		x20, x20, x12
	fmla	v4.4s, v20.4s, v28.s[0]
	ldp		q26, q27, [x9], #32
	fmla	v5.4s, v20.4s, v28.s[1]
	ldp		q22, q23, [x19], #32
	fmla	v6.4s, v20.4s, v28.s[2]
	prfm	PLDL1KEEP, [x9, #64]
	fmla	v7.4s, v20.4s, v28.s[3]
	prfm	PLDL1KEEP, [x19, #64]

	// unroll 1
	ldr		q28, [x11]
	fmla	v0.4s, v25.4s, v29.s[0]
	prfm	PLDL1KEEP, [x11, x13]
	fmla	v1.4s, v25.4s, v29.s[1]
	prfm	PLDL1KEEP, [x20, x13]
	fmla	v2.4s, v25.4s, v29.s[2]
	add		x11, x11, x12
	fmla	v3.4s, v25.4s, v29.s[3]
	add		x20, x20, x12
	fmla	v4.4s, v21.4s, v29.s[0]
	ldp		q24, q25, [x9], #32
	fmla	v5.4s, v21.4s, v29.s[1]
	fmla	v6.4s, v21.4s, v29.s[2]
	fmla	v7.4s, v21.4s, v29.s[3]
	ldp		q20, q21, [x19], #32

	// unroll 2
	ldr		q29, [x11]
	fmla	v0.4s, v26.4s, v30.s[0]
	prfm	PLDL1KEEP, [x11, x13]
	fmla	v1.4s, v26.4s, v30.s[1]
	prfm	PLDL1KEEP, [x20, x13]
	fmla	v2.4s, v26.4s, v30.s[2]
	add		x11, x11, x12
	fmla	v3.4s, v26.4s, v30.s[3]
	add		x20, x20, x12
	fmla	v4.4s, v22.4s, v30.s[0]
	fmla	v5.4s, v22.4s, v30.s[1]
	fmla	v6.4s, v22.4s, v30.s[2]
	sub		w8, w8, #4
	fmla	v7.4s, v22.4s, v30.s[3]

	// unroll 3
	ldr		q30, [x11]
	fmla	v0.4s, v27.4s, v31.s[0]
	prfm	PLDL1KEEP, [x11, x13]
	fmla	v1.4s, v27.4s, v31.s[1]
	prfm	PLDL1KEEP, [x20, x13]
	fmla	v2.4s, v27.4s, v31.s[2]
	add		x11, x11, x12
	fmla	v3.4s, v27.4s, v31.s[3]
	add		x20, x20, x12
	fmla	v4.4s, v23.4s, v31.s[0]
	fmla	v5.4s, v23.4s, v31.s[1]
	fmla	v6.4s, v23.4s, v31.s[2]
	fmla	v7.4s, v23.4s, v31.s[3]

	cmp		w8, #4
	bgt		1b

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		q31, [x11]
	fmla	v0.4s, v24.4s, v28.s[0]
//	prfm	PLDL1KEEP, [x11, x13]
	fmla	v1.4s, v24.4s, v28.s[1]
//	prfm	PLDL1KEEP, [x20, x13]
	fmla	v2.4s, v24.4s, v28.s[2]
	add		x11, x11, x12
	fmla	v3.4s, v24.4s, v28.s[3]
	add		x20, x20, x12
	fmla	v4.4s, v20.4s, v28.s[0]
	ldp		q26, q27, [x9], #32
	fmla	v5.4s, v20.4s, v28.s[1]
	ldp		q22, q23, [x19], #32
	fmla	v6.4s, v20.4s, v28.s[2]
	fmla	v7.4s, v20.4s, v28.s[3]

	// unroll 1
//	ldr		q28, [x11]
	fmla	v0.4s, v25.4s, v29.s[0]
//	prfm	PLDL1KEEP, [x11, x17]
	fmla	v1.4s, v25.4s, v29.s[1]
//	prfm	PLDL1KEEP, [x11, x18]
	fmla	v2.4s, v25.4s, v29.s[2]
	fmla	v3.4s, v25.4s, v29.s[3]
//	ldp		q24, q25, [x9], #32
	fmla	v4.4s, v21.4s, v29.s[0]
	fmla	v5.4s, v21.4s, v29.s[1]
	fmla	v6.4s, v21.4s, v29.s[2]
	fmla	v7.4s, v21.4s, v29.s[3]
//	ldp		q20, q21, [x19], #32

	// unroll 2
	fmla	v0.4s, v26.4s, v30.s[0]
//	ldr		q29, [x11, x12]
	fmla	v1.4s, v26.4s, v30.s[1]
	fmla	v2.4s, v26.4s, v30.s[2]
	sub		w8, w8, #4
	fmla	v3.4s, v26.4s, v30.s[3]
	fmla	v4.4s, v22.4s, v30.s[0]
	fmla	v5.4s, v22.4s, v30.s[1]
	fmla	v6.4s, v22.4s, v30.s[2]
	fmla	v7.4s, v22.4s, v30.s[3]
//	ldr		q30, [x11, x13]

	// unroll 3
	fmla	v0.4s, v27.4s, v31.s[0]
	fmla	v1.4s, v27.4s, v31.s[1]
	fmla	v2.4s, v27.4s, v31.s[2]
	fmla	v3.4s, v27.4s, v31.s[3]
	fmla	v4.4s, v23.4s, v31.s[0]
	fmla	v5.4s, v23.4s, v31.s[1]
	fmla	v6.4s, v23.4s, v31.s[2]
	fmla	v7.4s, v23.4s, v31.s[3]
//	ldr		q31, [x11, x14]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x9, x9, #32
	sub		x19, x19, #32
//	sub		x11, x11, #32
//	sub		x11, x11, x15
	sub		x11, x11, x14

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9], #16
	ldr		q20, [x19], #16
	ldr		q28, [x11]
	fmla	v0.4s, v24.4s, v28.s[0]
	add		x11, x11, x12
	fmla	v1.4s, v24.4s, v28.s[1]
	fmla	v2.4s, v24.4s, v28.s[2]
	fmla	v3.4s, v24.4s, v28.s[3]
	fmla	v4.4s, v20.4s, v28.s[0]
	fmla	v5.4s, v20.4s, v28.s[1]
	fmla	v6.4s, v20.4s, v28.s[2]
	fmla	v7.4s, v20.4s, v28.s[3]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_8x4_lib4c)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- sda
// x11   <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NT_8X3_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_8x3_lib4c)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x19, x9, x10

	add		x13, x12, x12 // 2
	add		x14, x13, x12 // 3
	add		x15, x13, x13 // 4
	add		x16, x14, x13 // 5
	add		x17, x14, x14 // 6
	add		x18, x15, x14 // 7

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x19, #0]
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x11, x14]

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x19, #64]
	prfm	PLDL1KEEP, [x11, x15]
	prfm	PLDL1KEEP, [x11, x16]
	prfm	PLDL1KEEP, [x11, x17]
	prfm	PLDL1KEEP, [x11, x18]

	// main loop
1:
	
	ldr		q28, [x11] // XXX also loading tail
	ldr		q29, [x11, x12] // XXX also loading tail
	ldp		q24, q25, [x9], #32
	ldp		q20, q21, [x19], #32

	ldr		q30, [x11, x13] // XXX also loading tail
	ldr		q31, [x11, x14] // XXX also loading tail
	ldp		q26, q27, [x9], #32
	ldp		q22, q23, [x19], #32

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	add		x11, x11, x15
	fmla	v1.4s, v24.4s, v28.s[1]
	prfm	PLDL1KEEP, [x9, #64]
	fmla	v2.4s, v24.4s, v28.s[2]
	prfm	PLDL1KEEP, [x19, #64]
//	fmla	v3.4s, v24.4s, v28.s[3]
	prfm	PLDL1KEEP, [x11, x15]
	fmla	v4.4s, v20.4s, v28.s[0]
	prfm	PLDL1KEEP, [x11, x16]
	fmla	v5.4s, v20.4s, v28.s[1]
	prfm	PLDL1KEEP, [x11, x17]
	fmla	v6.4s, v20.4s, v28.s[2]
	prfm	PLDL1KEEP, [x11, x18]
//	fmla	v7.4s, v20.4s, v28.s[3]

	// unroll 1
	fmla	v0.4s, v25.4s, v29.s[0]
	fmla	v1.4s, v25.4s, v29.s[1]
	fmla	v2.4s, v25.4s, v29.s[2]
//	fmla	v3.4s, v25.4s, v29.s[3]
	fmla	v4.4s, v21.4s, v29.s[0]
	fmla	v5.4s, v21.4s, v29.s[1]
	fmla	v6.4s, v21.4s, v29.s[2]
//	fmla	v7.4s, v21.4s, v29.s[3]

	// unroll 2
	fmla	v0.4s, v26.4s, v30.s[0]
	fmla	v1.4s, v26.4s, v30.s[1]
	fmla	v2.4s, v26.4s, v30.s[2]
//	fmla	v3.4s, v26.4s, v30.s[3]
	fmla	v4.4s, v22.4s, v30.s[0]
	fmla	v5.4s, v22.4s, v30.s[1]
	fmla	v6.4s, v22.4s, v30.s[2]
//	fmla	v7.4s, v22.4s, v30.s[3]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v31.s[0]
	fmla	v1.4s, v27.4s, v31.s[1]
	fmla	v2.4s, v27.4s, v31.s[2]
//	fmla	v3.4s, v27.4s, v31.s[3]
	fmla	v4.4s, v23.4s, v31.s[0]
	fmla	v5.4s, v23.4s, v31.s[1]
	fmla	v6.4s, v23.4s, v31.s[2]
//	fmla	v7.4s, v23.4s, v31.s[3]

	cmp		w8, #4
	bgt		1b

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	ldr		q28, [x11] // XXX also loading tail
	ldr		q29, [x11, x12] // XXX also loading tail
	ldp		q24, q25, [x9], #32
	ldp		q20, q21, [x19], #32

	ldr		q30, [x11, x13] // XXX also loading tail
	ldr		q31, [x11, x14] // XXX also loading tail
	ldp		q26, q27, [x9], #32
	ldp		q22, q23, [x19], #32

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	add		x11, x11, x15
	fmla	v1.4s, v24.4s, v28.s[1]
//	prfm	PLDL1KEEP, [x9, #64]
	fmla	v2.4s, v24.4s, v28.s[2]
//	prfm	PLDL1KEEP, [x19, #64]
//	fmla	v3.4s, v24.4s, v28.s[3]
//	prfm	PLDL1KEEP, [x11, x15]
	fmla	v4.4s, v20.4s, v28.s[0]
//	prfm	PLDL1KEEP, [x11, x16]
	fmla	v5.4s, v20.4s, v28.s[1]
//	prfm	PLDL1KEEP, [x11, x17]
	fmla	v6.4s, v20.4s, v28.s[2]
//	prfm	PLDL1KEEP, [x11, x18]
//	fmla	v7.4s, v20.4s, v28.s[3]

	// unroll 1
	fmla	v0.4s, v25.4s, v29.s[0]
	fmla	v1.4s, v25.4s, v29.s[1]
	fmla	v2.4s, v25.4s, v29.s[2]
//	fmla	v3.4s, v25.4s, v29.s[3]
	fmla	v4.4s, v21.4s, v29.s[0]
	fmla	v5.4s, v21.4s, v29.s[1]
	fmla	v6.4s, v21.4s, v29.s[2]
//	fmla	v7.4s, v21.4s, v29.s[3]

	// unroll 2
	fmla	v0.4s, v26.4s, v30.s[0]
	fmla	v1.4s, v26.4s, v30.s[1]
	fmla	v2.4s, v26.4s, v30.s[2]
//	fmla	v3.4s, v26.4s, v30.s[3]
	fmla	v4.4s, v22.4s, v30.s[0]
	fmla	v5.4s, v22.4s, v30.s[1]
	fmla	v6.4s, v22.4s, v30.s[2]
//	fmla	v7.4s, v22.4s, v30.s[3]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v31.s[0]
	fmla	v1.4s, v27.4s, v31.s[1]
	fmla	v2.4s, v27.4s, v31.s[2]
//	fmla	v3.4s, v27.4s, v31.s[3]
	fmla	v4.4s, v23.4s, v31.s[0]
	fmla	v5.4s, v23.4s, v31.s[1]
	fmla	v6.4s, v23.4s, v31.s[2]
//	fmla	v7.4s, v23.4s, v31.s[3]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9], #16
	ldr		q20, [x19], #16
	ldr		q28, [x11] // XXX also loading tail
	fmla	v0.4s, v24.4s, v28.s[0]
	add		x11, x11, x12
	fmla	v1.4s, v24.4s, v28.s[1]
	fmla	v2.4s, v24.4s, v28.s[2]
//	fmla	v3.4s, v24.4s, v28.s[3]
	fmla	v4.4s, v20.4s, v28.s[0]
	fmla	v5.4s, v20.4s, v28.s[1]
	fmla	v6.4s, v20.4s, v28.s[2]
//	fmla	v7.4s, v20.4s, v28.s[3]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return




#else // cortex a57 vs a53



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x19, x9, x10

	add		x13, x12, x12 // 2
	add		x14, x13, x12 // 3
	add		x15, x13, x13 // 4
	add		x16, x14, x13 // 5
	add		x17, x14, x14 // 6
	add		x18, x15, x14 // 7

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x19, #0]
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x11, x14]

	// preload
	ldr		q28, [x11] // XXX also loading tail
	ldr		q29, [x11, x12] // XXX also loading tail
	ldr		q30, [x11, x13] // XXX also loading tail
	ldr		q31, [x11, x14] // XXX also loading tail

	ldp		q24, q25, [x9], #32
	ldp		q20, q21, [x19], #32

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #32]
	prfm	PLDL1KEEP, [x19, #32]
	prfm	PLDL1KEEP, [x11, x15]
	prfm	PLDL1KEEP, [x11, x16]
	prfm	PLDL1KEEP, [x11, x17]
	prfm	PLDL1KEEP, [x11, x18]

	// main loop
1:
	

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	ldp		q26, q27, [x9], #32
	fmla	v1.4s, v24.4s, v28.s[1]
	ldp		q22, q23, [x19], #32
	fmla	v2.4s, v24.4s, v28.s[2]
	prfm	PLDL1KEEP, [x9, #64]
//	fmla	v3.4s, v24.4s, v28.s[3]
	prfm	PLDL1KEEP, [x19, #64]
	fmla	v4.4s, v20.4s, v28.s[0]
	add		x11, x11, x15
	fmla	v5.4s, v20.4s, v28.s[1]
	prfm	PLDL1KEEP, [x11, x15]
	fmla	v6.4s, v20.4s, v28.s[2]
	prfm	PLDL1KEEP, [x11, x16]
//	fmla	v7.4s, v20.4s, v28.s[3]
	ldr		q28, [x11] // XXX also loading tail

	// unroll 1
	fmla	v0.4s, v25.4s, v29.s[0]
	prfm	PLDL1KEEP, [x11, x17]
	fmla	v1.4s, v25.4s, v29.s[1]
	prfm	PLDL1KEEP, [x11, x18]
	fmla	v2.4s, v25.4s, v29.s[2]
//	fmla	v3.4s, v25.4s, v29.s[3]
	ldp		q24, q25, [x9], #32
	fmla	v4.4s, v21.4s, v29.s[0]
	fmla	v5.4s, v21.4s, v29.s[1]
	fmla	v6.4s, v21.4s, v29.s[2]
//	fmla	v7.4s, v21.4s, v29.s[3]
	ldp		q20, q21, [x19], #32

	// unroll 2
	fmla	v0.4s, v26.4s, v30.s[0]
	ldr		q29, [x11, x12] // XXX also loading tail
	fmla	v1.4s, v26.4s, v30.s[1]
	fmla	v2.4s, v26.4s, v30.s[2]
	sub		w8, w8, #4
//	fmla	v3.4s, v26.4s, v30.s[3]
	fmla	v4.4s, v22.4s, v30.s[0]
	fmla	v5.4s, v22.4s, v30.s[1]
	fmla	v6.4s, v22.4s, v30.s[2]
//	fmla	v7.4s, v22.4s, v30.s[3]
	ldr		q30, [x11, x13] // XXX also loading tail

	// unroll 3
	fmla	v0.4s, v27.4s, v31.s[0]
	fmla	v1.4s, v27.4s, v31.s[1]
	fmla	v2.4s, v27.4s, v31.s[2]
//	fmla	v3.4s, v27.4s, v31.s[3]
	fmla	v4.4s, v23.4s, v31.s[0]
	fmla	v5.4s, v23.4s, v31.s[1]
	fmla	v6.4s, v23.4s, v31.s[2]
//	fmla	v7.4s, v23.4s, v31.s[3]
	ldr		q31, [x11, x14] // XXX also loading tail

	cmp		w8, #4
	bgt		1b

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	ldp		q26, q27, [x9], #32
	fmla	v1.4s, v24.4s, v28.s[1]
	ldp		q22, q23, [x19], #32
	fmla	v2.4s, v24.4s, v28.s[2]
//	prfm	PLDL1KEEP, [x9, #64]
//	fmla	v3.4s, v24.4s, v28.s[3]
//	prfm	PLDL1KEEP, [x19, #64]
	fmla	v4.4s, v20.4s, v28.s[0]
	add		x11, x11, x15
	fmla	v5.4s, v20.4s, v28.s[1]
//	prfm	PLDL1KEEP, [x11, x15]
	fmla	v6.4s, v20.4s, v28.s[2]
//	prfm	PLDL1KEEP, [x11, x16]
//	fmla	v7.4s, v20.4s, v28.s[3]
//	ldr		q28, [x11] // XXX also loading tail

	// unroll 1
	fmla	v0.4s, v25.4s, v29.s[0]
//	prfm	PLDL1KEEP, [x11, x17]
	fmla	v1.4s, v25.4s, v29.s[1]
//	prfm	PLDL1KEEP, [x11, x18]
	fmla	v2.4s, v25.4s, v29.s[2]
//	fmla	v3.4s, v25.4s, v29.s[3]
//	ldp		q24, q25, [x9], #32
	fmla	v4.4s, v21.4s, v29.s[0]
	fmla	v5.4s, v21.4s, v29.s[1]
	fmla	v6.4s, v21.4s, v29.s[2]
//	fmla	v7.4s, v21.4s, v29.s[3]
//	ldp		q20, q21, [x19], #32

	// unroll 2
	fmla	v0.4s, v26.4s, v30.s[0]
//	ldr		q29, [x11, x12] // XXX also loading tail
	fmla	v1.4s, v26.4s, v30.s[1]
	fmla	v2.4s, v26.4s, v30.s[2]
	sub		w8, w8, #4
//	fmla	v3.4s, v26.4s, v30.s[3]
	fmla	v4.4s, v22.4s, v30.s[0]
	fmla	v5.4s, v22.4s, v30.s[1]
	fmla	v6.4s, v22.4s, v30.s[2]
//	fmla	v7.4s, v22.4s, v30.s[3]
//	ldr		q30, [x11, x13] // XXX also loading tail

	// unroll 3
	fmla	v0.4s, v27.4s, v31.s[0]
	fmla	v1.4s, v27.4s, v31.s[1]
	fmla	v2.4s, v27.4s, v31.s[2]
//	fmla	v3.4s, v27.4s, v31.s[3]
	fmla	v4.4s, v23.4s, v31.s[0]
	fmla	v5.4s, v23.4s, v31.s[1]
	fmla	v6.4s, v23.4s, v31.s[2]
//	fmla	v7.4s, v23.4s, v31.s[3]
//	ldr		q31, [x11, x14] // XXX also loading tail

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x9, x9, #32
	sub		x19, x19, #32
//	sub		x11, x11, #32
//	sub		x11, x11, x15

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9], #16
	ldr		q20, [x19], #16
	ldr		q28, [x11] // XXX also loading tail
	fmla	v0.4s, v24.4s, v28.s[0]
	add		x11, x11, x12
	fmla	v1.4s, v24.4s, v28.s[1]
	fmla	v2.4s, v24.4s, v28.s[2]
//	fmla	v3.4s, v24.4s, v28.s[3]
	fmla	v4.4s, v20.4s, v28.s[0]
	fmla	v5.4s, v20.4s, v28.s[1]
	fmla	v6.4s, v20.4s, v28.s[2]
//	fmla	v7.4s, v20.4s, v28.s[3]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_8x3_lib4c)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- sda
// x11   <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NT_8X2_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_8x2_lib4c)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x19, x9, x10

	add		x13, x12, x12 // 2
	add		x14, x13, x12 // 3
	add		x15, x13, x13 // 4
	add		x16, x14, x13 // 5
	add		x17, x14, x14 // 6
	add		x18, x15, x14 // 7

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x19, #0]
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x11, x14]

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x19, #64]
	prfm	PLDL1KEEP, [x11, x15]
	prfm	PLDL1KEEP, [x11, x16]
	prfm	PLDL1KEEP, [x11, x17]
	prfm	PLDL1KEEP, [x11, x18]

	// main loop
1:
	
	ldr		d28, [x11]
	ldr		d29, [x11, x12]
	ldp		q24, q25, [x9], #32
	ldp		q20, q21, [x19], #32

	ldr		d30, [x11, x13]
	ldr		d31, [x11, x14]
	ldp		q26, q27, [x9], #32
	ldp		q22, q23, [x19], #32

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	add		x11, x11, x15
	fmla	v1.4s, v24.4s, v28.s[1]
	prfm	PLDL1KEEP, [x9, #64]
//	fmla	v2.4s, v24.4s, v28.s[2]
	prfm	PLDL1KEEP, [x19, #64]
//	fmla	v3.4s, v24.4s, v28.s[3]
	prfm	PLDL1KEEP, [x11, x15]
	fmla	v4.4s, v20.4s, v28.s[0]
	prfm	PLDL1KEEP, [x11, x16]
	fmla	v5.4s, v20.4s, v28.s[1]
	prfm	PLDL1KEEP, [x11, x17]
//	fmla	v6.4s, v20.4s, v28.s[2]
	prfm	PLDL1KEEP, [x11, x18]
//	fmla	v7.4s, v20.4s, v28.s[3]

	// unroll 1
	fmla	v0.4s, v25.4s, v29.s[0]
	fmla	v1.4s, v25.4s, v29.s[1]
//	fmla	v2.4s, v25.4s, v29.s[2]
//	fmla	v3.4s, v25.4s, v29.s[3]
	fmla	v4.4s, v21.4s, v29.s[0]
	fmla	v5.4s, v21.4s, v29.s[1]
//	fmla	v6.4s, v21.4s, v29.s[2]
//	fmla	v7.4s, v21.4s, v29.s[3]

	// unroll 2
	fmla	v0.4s, v26.4s, v30.s[0]
	fmla	v1.4s, v26.4s, v30.s[1]
//	fmla	v2.4s, v26.4s, v30.s[2]
//	fmla	v3.4s, v26.4s, v30.s[3]
	fmla	v4.4s, v22.4s, v30.s[0]
	fmla	v5.4s, v22.4s, v30.s[1]
//	fmla	v6.4s, v22.4s, v30.s[2]
//	fmla	v7.4s, v22.4s, v30.s[3]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v31.s[0]
	fmla	v1.4s, v27.4s, v31.s[1]
//	fmla	v2.4s, v27.4s, v31.s[2]
//	fmla	v3.4s, v27.4s, v31.s[3]
	fmla	v4.4s, v23.4s, v31.s[0]
	fmla	v5.4s, v23.4s, v31.s[1]
//	fmla	v6.4s, v23.4s, v31.s[2]
//	fmla	v7.4s, v23.4s, v31.s[3]

	cmp		w8, #4
	bgt		1b

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	ldr		d28, [x11]
	ldr		d29, [x11, x12]
	ldp		q24, q25, [x9], #32
	ldp		q20, q21, [x19], #32

	ldr		d30, [x11, x13]
	ldr		d31, [x11, x14]
	ldp		q26, q27, [x9], #32
	ldp		q22, q23, [x19], #32

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	add		x11, x11, x15
	fmla	v1.4s, v24.4s, v28.s[1]
//	prfm	PLDL1KEEP, [x9, #64]
//	fmla	v2.4s, v24.4s, v28.s[2]
//	prfm	PLDL1KEEP, [x19, #64]
//	fmla	v3.4s, v24.4s, v28.s[3]
//	prfm	PLDL1KEEP, [x11, x15]
	fmla	v4.4s, v20.4s, v28.s[0]
//	prfm	PLDL1KEEP, [x11, x16]
	fmla	v5.4s, v20.4s, v28.s[1]
//	prfm	PLDL1KEEP, [x11, x17]
//	fmla	v6.4s, v20.4s, v28.s[2]
//	prfm	PLDL1KEEP, [x11, x18]
//	fmla	v7.4s, v20.4s, v28.s[3]

	// unroll 1
	fmla	v0.4s, v25.4s, v29.s[0]
	fmla	v1.4s, v25.4s, v29.s[1]
//	fmla	v2.4s, v25.4s, v29.s[2]
//	fmla	v3.4s, v25.4s, v29.s[3]
	fmla	v4.4s, v21.4s, v29.s[0]
	fmla	v5.4s, v21.4s, v29.s[1]
//	fmla	v6.4s, v21.4s, v29.s[2]
//	fmla	v7.4s, v21.4s, v29.s[3]

	// unroll 2
	fmla	v0.4s, v26.4s, v30.s[0]
	fmla	v1.4s, v26.4s, v30.s[1]
//	fmla	v2.4s, v26.4s, v30.s[2]
//	fmla	v3.4s, v26.4s, v30.s[3]
	fmla	v4.4s, v22.4s, v30.s[0]
	fmla	v5.4s, v22.4s, v30.s[1]
//	fmla	v6.4s, v22.4s, v30.s[2]
//	fmla	v7.4s, v22.4s, v30.s[3]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v31.s[0]
	fmla	v1.4s, v27.4s, v31.s[1]
//	fmla	v2.4s, v27.4s, v31.s[2]
//	fmla	v3.4s, v27.4s, v31.s[3]
	fmla	v4.4s, v23.4s, v31.s[0]
	fmla	v5.4s, v23.4s, v31.s[1]
//	fmla	v6.4s, v23.4s, v31.s[2]
//	fmla	v7.4s, v23.4s, v31.s[3]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9], #16
	ldr		q20, [x19], #16
	ldr		d28, [x11]
	fmla	v0.4s, v24.4s, v28.s[0]
	add		x11, x11, x12
	fmla	v1.4s, v24.4s, v28.s[1]
//	fmla	v2.4s, v24.4s, v28.s[2]
//	fmla	v3.4s, v24.4s, v28.s[3]
	fmla	v4.4s, v20.4s, v28.s[0]
	fmla	v5.4s, v20.4s, v28.s[1]
//	fmla	v6.4s, v20.4s, v28.s[2]
//	fmla	v7.4s, v20.4s, v28.s[3]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return




#else // cortex a57 vs a53



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x19, x9, x10

	add		x13, x12, x12 // 2
	add		x14, x13, x12 // 3
	add		x15, x13, x13 // 4
	add		x16, x14, x13 // 5
	add		x17, x14, x14 // 6
	add		x18, x15, x14 // 7

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x19, #0]
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x11, x14]

	// preload
	ldr		d28, [x11]
	ldr		d29, [x11, x12]
	ldr		d30, [x11, x13]
	ldr		d31, [x11, x14]

	ldp		q24, q25, [x9], #32
	ldp		q20, q21, [x19], #32

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #32]
	prfm	PLDL1KEEP, [x19, #32]
	prfm	PLDL1KEEP, [x11, x15]
	prfm	PLDL1KEEP, [x11, x16]
	prfm	PLDL1KEEP, [x11, x17]
	prfm	PLDL1KEEP, [x11, x18]

	// main loop
1:
	

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	ldp		q26, q27, [x9], #32
	fmla	v1.4s, v24.4s, v28.s[1]
	ldp		q22, q23, [x19], #32
//	fmla	v2.4s, v24.4s, v28.s[2]
	prfm	PLDL1KEEP, [x9, #64]
//	fmla	v3.4s, v24.4s, v28.s[3]
	prfm	PLDL1KEEP, [x19, #64]
	fmla	v4.4s, v20.4s, v28.s[0]
	add		x11, x11, x15
	fmla	v5.4s, v20.4s, v28.s[1]
	prfm	PLDL1KEEP, [x11, x15]
//	fmla	v6.4s, v20.4s, v28.s[2]
	prfm	PLDL1KEEP, [x11, x16]
//	fmla	v7.4s, v20.4s, v28.s[3]
	ldr		d28, [x11]

	// unroll 1
	fmla	v0.4s, v25.4s, v29.s[0]
	prfm	PLDL1KEEP, [x11, x17]
	fmla	v1.4s, v25.4s, v29.s[1]
	prfm	PLDL1KEEP, [x11, x18]
//	fmla	v2.4s, v25.4s, v29.s[2]
//	fmla	v3.4s, v25.4s, v29.s[3]
	ldp		q24, q25, [x9], #32
	fmla	v4.4s, v21.4s, v29.s[0]
	fmla	v5.4s, v21.4s, v29.s[1]
//	fmla	v6.4s, v21.4s, v29.s[2]
//	fmla	v7.4s, v21.4s, v29.s[3]
	ldp		q20, q21, [x19], #32

	// unroll 2
	fmla	v0.4s, v26.4s, v30.s[0]
	ldr		d29, [x11, x12]
	fmla	v1.4s, v26.4s, v30.s[1]
//	fmla	v2.4s, v26.4s, v30.s[2]
	sub		w8, w8, #4
//	fmla	v3.4s, v26.4s, v30.s[3]
	fmla	v4.4s, v22.4s, v30.s[0]
	fmla	v5.4s, v22.4s, v30.s[1]
//	fmla	v6.4s, v22.4s, v30.s[2]
//	fmla	v7.4s, v22.4s, v30.s[3]
	ldr		d30, [x11, x13]

	// unroll 3
	fmla	v0.4s, v27.4s, v31.s[0]
	fmla	v1.4s, v27.4s, v31.s[1]
//	fmla	v2.4s, v27.4s, v31.s[2]
//	fmla	v3.4s, v27.4s, v31.s[3]
	fmla	v4.4s, v23.4s, v31.s[0]
	fmla	v5.4s, v23.4s, v31.s[1]
//	fmla	v6.4s, v23.4s, v31.s[2]
//	fmla	v7.4s, v23.4s, v31.s[3]
	ldr		d31, [x11, x14]

	cmp		w8, #4
	bgt		1b

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	ldp		q26, q27, [x9], #32
	fmla	v1.4s, v24.4s, v28.s[1]
	ldp		q22, q23, [x19], #32
//	fmla	v2.4s, v24.4s, v28.s[2]
//	prfm	PLDL1KEEP, [x9, #64]
//	fmla	v3.4s, v24.4s, v28.s[3]
//	prfm	PLDL1KEEP, [x19, #64]
	fmla	v4.4s, v20.4s, v28.s[0]
	add		x11, x11, x15
	fmla	v5.4s, v20.4s, v28.s[1]
//	prfm	PLDL1KEEP, [x11, x15]
//	fmla	v6.4s, v20.4s, v28.s[2]
//	prfm	PLDL1KEEP, [x11, x16]
//	fmla	v7.4s, v20.4s, v28.s[3]
//	ldr		q28, [x11] // XXX also loading tail

	// unroll 1
	fmla	v0.4s, v25.4s, v29.s[0]
//	prfm	PLDL1KEEP, [x11, x17]
	fmla	v1.4s, v25.4s, v29.s[1]
//	prfm	PLDL1KEEP, [x11, x18]
//	fmla	v2.4s, v25.4s, v29.s[2]
//	fmla	v3.4s, v25.4s, v29.s[3]
//	ldp		q24, q25, [x9], #32
	fmla	v4.4s, v21.4s, v29.s[0]
	fmla	v5.4s, v21.4s, v29.s[1]
//	fmla	v6.4s, v21.4s, v29.s[2]
//	fmla	v7.4s, v21.4s, v29.s[3]
//	ldp		q20, q21, [x19], #32

	// unroll 2
	fmla	v0.4s, v26.4s, v30.s[0]
//	ldr		q29, [x11, x12] // XXX also loading tail
	fmla	v1.4s, v26.4s, v30.s[1]
//	fmla	v2.4s, v26.4s, v30.s[2]
	sub		w8, w8, #4
//	fmla	v3.4s, v26.4s, v30.s[3]
	fmla	v4.4s, v22.4s, v30.s[0]
	fmla	v5.4s, v22.4s, v30.s[1]
//	fmla	v6.4s, v22.4s, v30.s[2]
//	fmla	v7.4s, v22.4s, v30.s[3]
//	ldr		q30, [x11, x13] // XXX also loading tail

	// unroll 3
	fmla	v0.4s, v27.4s, v31.s[0]
	fmla	v1.4s, v27.4s, v31.s[1]
//	fmla	v2.4s, v27.4s, v31.s[2]
//	fmla	v3.4s, v27.4s, v31.s[3]
	fmla	v4.4s, v23.4s, v31.s[0]
	fmla	v5.4s, v23.4s, v31.s[1]
//	fmla	v6.4s, v23.4s, v31.s[2]
//	fmla	v7.4s, v23.4s, v31.s[3]
//	ldr		q31, [x11, x14] // XXX also loading tail

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x9, x9, #32
	sub		x19, x19, #32
//	sub		x11, x11, #32
//	sub		x11, x11, x15

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9], #16
	ldr		q20, [x19], #16
	ldr		d28, [x11]
	fmla	v0.4s, v24.4s, v28.s[0]
	add		x11, x11, x12
	fmla	v1.4s, v24.4s, v28.s[1]
//	fmla	v2.4s, v24.4s, v28.s[2]
//	fmla	v3.4s, v24.4s, v28.s[3]
	fmla	v4.4s, v20.4s, v28.s[0]
	fmla	v5.4s, v20.4s, v28.s[1]
//	fmla	v6.4s, v20.4s, v28.s[2]
//	fmla	v7.4s, v20.4s, v28.s[3]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_8x2_lib4c)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- sda
// x11   <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NT_8X1_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_8x1_lib4c)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x19, x9, x10

	add		x13, x12, x12 // 2
	add		x14, x13, x12 // 3
	add		x15, x13, x13 // 4
	add		x16, x14, x13 // 5
	add		x17, x14, x14 // 6
	add		x18, x15, x14 // 7

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x19, #0]
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x11, x14]

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x19, #64]
	prfm	PLDL1KEEP, [x11, x15]
	prfm	PLDL1KEEP, [x11, x16]
	prfm	PLDL1KEEP, [x11, x17]
	prfm	PLDL1KEEP, [x11, x18]

	// main loop
1:
	
	ldr		s28, [x11]
	ldr		s29, [x11, x12]
	ldp		q24, q25, [x9], #32
	ldp		q20, q21, [x19], #32

	ldr		s30, [x11, x13]
	ldr		s31, [x11, x14]
	ldp		q26, q27, [x9], #32
	ldp		q22, q23, [x19], #32

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	add		x11, x11, x15
//	fmla	v1.4s, v24.4s, v28.s[1]
	prfm	PLDL1KEEP, [x9, #64]
//	fmla	v2.4s, v24.4s, v28.s[2]
	prfm	PLDL1KEEP, [x19, #64]
//	fmla	v3.4s, v24.4s, v28.s[3]
	prfm	PLDL1KEEP, [x11, x15]
	fmla	v4.4s, v20.4s, v28.s[0]
	prfm	PLDL1KEEP, [x11, x16]
//	fmla	v5.4s, v20.4s, v28.s[1]
	prfm	PLDL1KEEP, [x11, x17]
//	fmla	v6.4s, v20.4s, v28.s[2]
	prfm	PLDL1KEEP, [x11, x18]
//	fmla	v7.4s, v20.4s, v28.s[3]

	// unroll 1
	fmla	v0.4s, v25.4s, v29.s[0]
//	fmla	v1.4s, v25.4s, v29.s[1]
//	fmla	v2.4s, v25.4s, v29.s[2]
//	fmla	v3.4s, v25.4s, v29.s[3]
	fmla	v4.4s, v21.4s, v29.s[0]
//	fmla	v5.4s, v21.4s, v29.s[1]
//	fmla	v6.4s, v21.4s, v29.s[2]
//	fmla	v7.4s, v21.4s, v29.s[3]

	// unroll 2
	fmla	v0.4s, v26.4s, v30.s[0]
//	fmla	v1.4s, v26.4s, v30.s[1]
//	fmla	v2.4s, v26.4s, v30.s[2]
//	fmla	v3.4s, v26.4s, v30.s[3]
	fmla	v4.4s, v22.4s, v30.s[0]
//	fmla	v5.4s, v22.4s, v30.s[1]
//	fmla	v6.4s, v22.4s, v30.s[2]
//	fmla	v7.4s, v22.4s, v30.s[3]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v31.s[0]
//	fmla	v1.4s, v27.4s, v31.s[1]
//	fmla	v2.4s, v27.4s, v31.s[2]
//	fmla	v3.4s, v27.4s, v31.s[3]
	fmla	v4.4s, v23.4s, v31.s[0]
//	fmla	v5.4s, v23.4s, v31.s[1]
//	fmla	v6.4s, v23.4s, v31.s[2]
//	fmla	v7.4s, v23.4s, v31.s[3]

	cmp		w8, #4
	bgt		1b

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	ldr		s28, [x11]
	ldr		s29, [x11, x12]
	ldp		q24, q25, [x9], #32
	ldp		q20, q21, [x19], #32

	ldr		s30, [x11, x13]
	ldr		s31, [x11, x14]
	ldp		q26, q27, [x9], #32
	ldp		q22, q23, [x19], #32

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	add		x11, x11, x15
//	fmla	v1.4s, v24.4s, v28.s[1]
//	prfm	PLDL1KEEP, [x9, #64]
//	fmla	v2.4s, v24.4s, v28.s[2]
//	prfm	PLDL1KEEP, [x19, #64]
//	fmla	v3.4s, v24.4s, v28.s[3]
//	prfm	PLDL1KEEP, [x11, x15]
	fmla	v4.4s, v20.4s, v28.s[0]
//	prfm	PLDL1KEEP, [x11, x16]
//	fmla	v5.4s, v20.4s, v28.s[1]
//	prfm	PLDL1KEEP, [x11, x17]
//	fmla	v6.4s, v20.4s, v28.s[2]
//	prfm	PLDL1KEEP, [x11, x18]
//	fmla	v7.4s, v20.4s, v28.s[3]

	// unroll 1
	fmla	v0.4s, v25.4s, v29.s[0]
//	fmla	v1.4s, v25.4s, v29.s[1]
//	fmla	v2.4s, v25.4s, v29.s[2]
//	fmla	v3.4s, v25.4s, v29.s[3]
	fmla	v4.4s, v21.4s, v29.s[0]
//	fmla	v5.4s, v21.4s, v29.s[1]
//	fmla	v6.4s, v21.4s, v29.s[2]
//	fmla	v7.4s, v21.4s, v29.s[3]

	// unroll 2
	fmla	v0.4s, v26.4s, v30.s[0]
//	fmla	v1.4s, v26.4s, v30.s[1]
//	fmla	v2.4s, v26.4s, v30.s[2]
//	fmla	v3.4s, v26.4s, v30.s[3]
	fmla	v4.4s, v22.4s, v30.s[0]
//	fmla	v5.4s, v22.4s, v30.s[1]
//	fmla	v6.4s, v22.4s, v30.s[2]
//	fmla	v7.4s, v22.4s, v30.s[3]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v31.s[0]
//	fmla	v1.4s, v27.4s, v31.s[1]
//	fmla	v2.4s, v27.4s, v31.s[2]
//	fmla	v3.4s, v27.4s, v31.s[3]
	fmla	v4.4s, v23.4s, v31.s[0]
//	fmla	v5.4s, v23.4s, v31.s[1]
//	fmla	v6.4s, v23.4s, v31.s[2]
//	fmla	v7.4s, v23.4s, v31.s[3]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9], #16
	ldr		q20, [x19], #16
	ldr		s28, [x11]
	fmla	v0.4s, v24.4s, v28.s[0]
	add		x11, x11, x12
//	fmla	v1.4s, v24.4s, v28.s[1]
//	fmla	v2.4s, v24.4s, v28.s[2]
//	fmla	v3.4s, v24.4s, v28.s[3]
	fmla	v4.4s, v20.4s, v28.s[0]
//	fmla	v5.4s, v20.4s, v28.s[1]
//	fmla	v6.4s, v20.4s, v28.s[2]
//	fmla	v7.4s, v20.4s, v28.s[3]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return




#else // cortex a57 vs a53



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x19, x9, x10

	add		x13, x12, x12 // 2
	add		x14, x13, x12 // 3
	add		x15, x13, x13 // 4
	add		x16, x14, x13 // 5
	add		x17, x14, x14 // 6
	add		x18, x15, x14 // 7

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x19, #0]
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x11, x14]

	// preload
	ldr		s28, [x11]
	ldr		s29, [x11, x12]
	ldr		s30, [x11, x13]
	ldr		s31, [x11, x14]

	ldp		q24, q25, [x9], #32
	ldp		q20, q21, [x19], #32

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #32]
	prfm	PLDL1KEEP, [x19, #32]
	prfm	PLDL1KEEP, [x11, x15]
	prfm	PLDL1KEEP, [x11, x16]
	prfm	PLDL1KEEP, [x11, x17]
	prfm	PLDL1KEEP, [x11, x18]

	// main loop
1:
	

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	ldp		q26, q27, [x9], #32
//	fmla	v1.4s, v24.4s, v28.s[1]
	ldp		q22, q23, [x19], #32
//	fmla	v2.4s, v24.4s, v28.s[2]
	prfm	PLDL1KEEP, [x9, #64]
//	fmla	v3.4s, v24.4s, v28.s[3]
	prfm	PLDL1KEEP, [x19, #64]
	fmla	v4.4s, v20.4s, v28.s[0]
	add		x11, x11, x15
//	fmla	v5.4s, v20.4s, v28.s[1]
	prfm	PLDL1KEEP, [x11, x15]
//	fmla	v6.4s, v20.4s, v28.s[2]
	prfm	PLDL1KEEP, [x11, x16]
//	fmla	v7.4s, v20.4s, v28.s[3]
	ldr		s28, [x11]

	// unroll 1
	fmla	v0.4s, v25.4s, v29.s[0]
	prfm	PLDL1KEEP, [x11, x17]
//	fmla	v1.4s, v25.4s, v29.s[1]
	prfm	PLDL1KEEP, [x11, x18]
//	fmla	v2.4s, v25.4s, v29.s[2]
//	fmla	v3.4s, v25.4s, v29.s[3]
	ldp		q24, q25, [x9], #32
	fmla	v4.4s, v21.4s, v29.s[0]
//	fmla	v5.4s, v21.4s, v29.s[1]
//	fmla	v6.4s, v21.4s, v29.s[2]
//	fmla	v7.4s, v21.4s, v29.s[3]
	ldp		q20, q21, [x19], #32

	// unroll 2
	fmla	v0.4s, v26.4s, v30.s[0]
	ldr		s29, [x11, x12]
//	fmla	v1.4s, v26.4s, v30.s[1]
//	fmla	v2.4s, v26.4s, v30.s[2]
	sub		w8, w8, #4
//	fmla	v3.4s, v26.4s, v30.s[3]
	fmla	v4.4s, v22.4s, v30.s[0]
//	fmla	v5.4s, v22.4s, v30.s[1]
//	fmla	v6.4s, v22.4s, v30.s[2]
//	fmla	v7.4s, v22.4s, v30.s[3]
	ldr		s30, [x11, x13]

	// unroll 3
	fmla	v0.4s, v27.4s, v31.s[0]
//	fmla	v1.4s, v27.4s, v31.s[1]
//	fmla	v2.4s, v27.4s, v31.s[2]
//	fmla	v3.4s, v27.4s, v31.s[3]
	fmla	v4.4s, v23.4s, v31.s[0]
//	fmla	v5.4s, v23.4s, v31.s[1]
//	fmla	v6.4s, v23.4s, v31.s[2]
//	fmla	v7.4s, v23.4s, v31.s[3]
	ldr		d31, [x11, x14]

	cmp		w8, #4
	bgt		1b

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	ldp		q26, q27, [x9], #32
//	fmla	v1.4s, v24.4s, v28.s[1]
	ldp		q22, q23, [x19], #32
//	fmla	v2.4s, v24.4s, v28.s[2]
//	prfm	PLDL1KEEP, [x9, #64]
//	fmla	v3.4s, v24.4s, v28.s[3]
//	prfm	PLDL1KEEP, [x19, #64]
	fmla	v4.4s, v20.4s, v28.s[0]
	add		x11, x11, x15
//	fmla	v5.4s, v20.4s, v28.s[1]
//	prfm	PLDL1KEEP, [x11, x15]
//	fmla	v6.4s, v20.4s, v28.s[2]
//	prfm	PLDL1KEEP, [x11, x16]
//	fmla	v7.4s, v20.4s, v28.s[3]
//	ldr		q28, [x11] // XXX also loading tail

	// unroll 1
	fmla	v0.4s, v25.4s, v29.s[0]
//	prfm	PLDL1KEEP, [x11, x17]
//	fmla	v1.4s, v25.4s, v29.s[1]
//	prfm	PLDL1KEEP, [x11, x18]
//	fmla	v2.4s, v25.4s, v29.s[2]
//	fmla	v3.4s, v25.4s, v29.s[3]
//	ldp		q24, q25, [x9], #32
	fmla	v4.4s, v21.4s, v29.s[0]
//	fmla	v5.4s, v21.4s, v29.s[1]
//	fmla	v6.4s, v21.4s, v29.s[2]
//	fmla	v7.4s, v21.4s, v29.s[3]
//	ldp		q20, q21, [x19], #32

	// unroll 2
	fmla	v0.4s, v26.4s, v30.s[0]
//	ldr		q29, [x11, x12] // XXX also loading tail
//	fmla	v1.4s, v26.4s, v30.s[1]
//	fmla	v2.4s, v26.4s, v30.s[2]
	sub		w8, w8, #4
//	fmla	v3.4s, v26.4s, v30.s[3]
	fmla	v4.4s, v22.4s, v30.s[0]
//	fmla	v5.4s, v22.4s, v30.s[1]
//	fmla	v6.4s, v22.4s, v30.s[2]
//	fmla	v7.4s, v22.4s, v30.s[3]
//	ldr		q30, [x11, x13] // XXX also loading tail

	// unroll 3
	fmla	v0.4s, v27.4s, v31.s[0]
//	fmla	v1.4s, v27.4s, v31.s[1]
//	fmla	v2.4s, v27.4s, v31.s[2]
//	fmla	v3.4s, v27.4s, v31.s[3]
	fmla	v4.4s, v23.4s, v31.s[0]
//	fmla	v5.4s, v23.4s, v31.s[1]
//	fmla	v6.4s, v23.4s, v31.s[2]
//	fmla	v7.4s, v23.4s, v31.s[3]
//	ldr		q31, [x11, x14] // XXX also loading tail

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x9, x9, #32
	sub		x19, x19, #32
//	sub		x11, x11, #32
//	sub		x11, x11, x15

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9], #16
	ldr		q20, [x19], #16
	ldr		s28, [x11]
	fmla	v0.4s, v24.4s, v28.s[0]
	add		x11, x11, x12
//	fmla	v1.4s, v24.4s, v28.s[1]
//	fmla	v2.4s, v24.4s, v28.s[2]
//	fmla	v3.4s, v24.4s, v28.s[3]
	fmla	v4.4s, v20.4s, v28.s[0]
//	fmla	v5.4s, v20.4s, v28.s[1]
//	fmla	v6.4s, v20.4s, v28.s[2]
//	fmla	v7.4s, v20.4s, v28.s[3]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_8x1_lib4c)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- sda
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NN_8X4_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_8x4_lib4c)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x16, x9, x10

	add		x13, x11, x12
	add		x14, x13, x12
	add		x15, x14, x12

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x16, #0]
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x16, #64]

	// main loop
1:
	
	ldr		q28, [x11], #16
	ldr		q29, [x13], #16
	ldp		q24, q25, [x9], #32
	ldp		q20, q21, [x16], #32

	ldr		q30, [x14], #16
	ldr		q31, [x15], #16
	ldp		q26, q27, [x9], #32
	ldp		q22, q23, [x16], #32

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	prfm	PLDL1KEEP, [x9, #64]
	fmla	v1.4s, v24.4s, v29.s[0]
	prfm	PLDL1KEEP, [x16, #64]
	fmla	v2.4s, v24.4s, v30.s[0]
	prfm	PLDL1KEEP, [x11, #16]
	fmla	v3.4s, v24.4s, v31.s[0]
	prfm	PLDL1KEEP, [x13, #16]
	fmla	v4.4s, v20.4s, v28.s[0]
	prfm	PLDL1KEEP, [x14, #16]
	fmla	v5.4s, v20.4s, v29.s[0]
	prfm	PLDL1KEEP, [x15, #16]
	fmla	v6.4s, v20.4s, v30.s[0]
	fmla	v7.4s, v20.4s, v31.s[0]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
	fmla	v1.4s, v25.4s, v29.s[1]
	fmla	v2.4s, v25.4s, v30.s[1]
	fmla	v3.4s, v25.4s, v31.s[1]
	fmla	v4.4s, v21.4s, v28.s[1]
	fmla	v5.4s, v21.4s, v29.s[1]
	fmla	v6.4s, v21.4s, v30.s[1]
	fmla	v7.4s, v21.4s, v31.s[1]

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
	fmla	v1.4s, v26.4s, v29.s[2]
	fmla	v2.4s, v26.4s, v30.s[2]
	fmla	v3.4s, v26.4s, v31.s[2]
	fmla	v4.4s, v22.4s, v28.s[2]
	fmla	v5.4s, v22.4s, v29.s[2]
	fmla	v6.4s, v22.4s, v30.s[2]
	fmla	v7.4s, v22.4s, v31.s[2]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v28.s[3]
	fmla	v1.4s, v27.4s, v29.s[3]
	fmla	v2.4s, v27.4s, v30.s[3]
	fmla	v3.4s, v27.4s, v31.s[3]
	fmla	v4.4s, v23.4s, v28.s[3]
	fmla	v5.4s, v23.4s, v29.s[3]
	fmla	v6.4s, v23.4s, v30.s[3]
	fmla	v7.4s, v23.4s, v31.s[3]

	cmp		w8, #4
	bgt		1b

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	ldr		q28, [x11], #16
	ldr		q29, [x13], #16
	ldp		q24, q25, [x9], #32
	ldp		q20, q21, [x16], #32

	ldr		q30, [x14], #16
	ldr		q31, [x15], #16
	ldp		q26, q27, [x9], #32
	ldp		q22, q23, [x16], #32

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v29.s[0]
	fmla	v2.4s, v24.4s, v30.s[0]
	fmla	v3.4s, v24.4s, v31.s[0]
	fmla	v4.4s, v20.4s, v28.s[0]
	fmla	v5.4s, v20.4s, v29.s[0]
	fmla	v6.4s, v20.4s, v30.s[0]
	fmla	v7.4s, v20.4s, v31.s[0]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
	fmla	v1.4s, v25.4s, v29.s[1]
	fmla	v2.4s, v25.4s, v30.s[1]
	fmla	v3.4s, v25.4s, v31.s[1]
	fmla	v4.4s, v21.4s, v28.s[1]
	fmla	v5.4s, v21.4s, v29.s[1]
	fmla	v6.4s, v21.4s, v30.s[1]
	fmla	v7.4s, v21.4s, v31.s[1]

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
	fmla	v1.4s, v26.4s, v29.s[2]
	fmla	v2.4s, v26.4s, v30.s[2]
	fmla	v3.4s, v26.4s, v31.s[2]
	fmla	v4.4s, v22.4s, v28.s[2]
	fmla	v5.4s, v22.4s, v29.s[2]
	fmla	v6.4s, v22.4s, v30.s[2]
	fmla	v7.4s, v22.4s, v31.s[2]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v28.s[3]
	fmla	v1.4s, v27.4s, v29.s[3]
	fmla	v2.4s, v27.4s, v30.s[3]
	fmla	v3.4s, v27.4s, v31.s[3]
	fmla	v4.4s, v23.4s, v28.s[3]
	fmla	v5.4s, v23.4s, v29.s[3]
	fmla	v6.4s, v23.4s, v30.s[3]
	fmla	v7.4s, v23.4s, v31.s[3]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9], #16
	ldr		q20, [x16], #16
	ldr		s28, [x11], #4
	ldr		s29, [x13], #4
	ldr		s30, [x14], #4
	ldr		s31, [x15], #4
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v29.s[0]
	fmla	v2.4s, v24.4s, v30.s[0]
	fmla	v3.4s, v24.4s, v31.s[0]
	fmla	v4.4s, v20.4s, v28.s[0]
	fmla	v5.4s, v20.4s, v29.s[0]
	fmla	v6.4s, v20.4s, v30.s[0]
	fmla	v7.4s, v20.4s, v31.s[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else // cortex a53



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x16, x9, x10

	add		x13, x11, x12
	add		x14, x13, x12
	add		x15, x14, x12

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x16, #0]
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]

	// preload
	ldr		q28, [x11], #16
	ldr		q29, [x13], #16
	ldr		q30, [x14], #16
	ldr		q31, [x15], #16

	ldp		q24, q25, [x9], #32
	ldp		q20, q21, [x16], #32

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #32]
	prfm	PLDL1KEEP, [x16, #64]

	// main loop
1:
	

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	ldp		q26, q27, [x9], #32
	fmla	v1.4s, v24.4s, v29.s[0]
	ldp		q22, q23, [x16], #32
	fmla	v2.4s, v24.4s, v30.s[0]
	prfm	PLDL1KEEP, [x9, #64]
	fmla	v3.4s, v24.4s, v31.s[0]
	prfm	PLDL1KEEP, [x16, #64]
	fmla	v4.4s, v20.4s, v28.s[0]
	prfm	PLDL1KEEP, [x11, #16]
	fmla	v5.4s, v20.4s, v29.s[0]
	prfm	PLDL1KEEP, [x13, #16]
	fmla	v6.4s, v20.4s, v30.s[0]
	prfm	PLDL1KEEP, [x14, #16]
	fmla	v7.4s, v20.4s, v31.s[0]
	prfm	PLDL1KEEP, [x15, #16]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
	fmla	v1.4s, v25.4s, v29.s[1]
	fmla	v2.4s, v25.4s, v30.s[1]
	fmla	v3.4s, v25.4s, v31.s[1]
	ldp		q24, q25, [x9], #32
	fmla	v4.4s, v21.4s, v28.s[1]
	fmla	v5.4s, v21.4s, v29.s[1]
	fmla	v6.4s, v21.4s, v30.s[1]
	fmla	v7.4s, v21.4s, v31.s[1]
	ldp		q20, q21, [x16], #32

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
	fmla	v1.4s, v26.4s, v29.s[2]
	fmla	v2.4s, v26.4s, v30.s[2]
	fmla	v3.4s, v26.4s, v31.s[2]
	fmla	v4.4s, v22.4s, v28.s[2]
	fmla	v5.4s, v22.4s, v29.s[2]
	fmla	v6.4s, v22.4s, v30.s[2]
	fmla	v7.4s, v22.4s, v31.s[2]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v28.s[3]
	fmla	v1.4s, v27.4s, v29.s[3]
	fmla	v2.4s, v27.4s, v30.s[3]
	fmla	v3.4s, v27.4s, v31.s[3]
	fmla	v4.4s, v23.4s, v28.s[3]
	ldr		q28, [x11], #16
	fmla	v5.4s, v23.4s, v29.s[3]
	ldr		q29, [x13], #16
	fmla	v6.4s, v23.4s, v30.s[3]
	ldr		q30, [x14], #16
	fmla	v7.4s, v23.4s, v31.s[3]
	ldr		q31, [x15], #16

	cmp		w8, #4
	bgt		1b

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	ldp		q26, q27, [x9], #32
	fmla	v1.4s, v24.4s, v29.s[0]
	ldp		q22, q23, [x16], #32
	fmla	v2.4s, v24.4s, v30.s[0]
//	prfm	PLDL1KEEP, [x9, #64]
	fmla	v3.4s, v24.4s, v31.s[0]
//	prfm	PLDL1KEEP, [x16, #64]
	fmla	v4.4s, v20.4s, v28.s[0]
//	prfm	PLDL1KEEP, [x11, #16]
	fmla	v5.4s, v20.4s, v29.s[0]
//	prfm	PLDL1KEEP, [x13, #16]
	fmla	v6.4s, v20.4s, v30.s[0]
//	prfm	PLDL1KEEP, [x14, #16]
	fmla	v7.4s, v20.4s, v31.s[0]
//	prfm	PLDL1KEEP, [x15, #16]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
	fmla	v1.4s, v25.4s, v29.s[1]
	fmla	v2.4s, v25.4s, v30.s[1]
	fmla	v3.4s, v25.4s, v31.s[1]
//	ldp		q24, q25, [x9], #32
	fmla	v4.4s, v21.4s, v28.s[1]
	fmla	v5.4s, v21.4s, v29.s[1]
	fmla	v6.4s, v21.4s, v30.s[1]
	fmla	v7.4s, v21.4s, v31.s[1]
//	ldp		q20, q21, [x16], #32

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
	fmla	v1.4s, v26.4s, v29.s[2]
	fmla	v2.4s, v26.4s, v30.s[2]
	fmla	v3.4s, v26.4s, v31.s[2]
	fmla	v4.4s, v22.4s, v28.s[2]
	fmla	v5.4s, v22.4s, v29.s[2]
	fmla	v6.4s, v22.4s, v30.s[2]
	fmla	v7.4s, v22.4s, v31.s[2]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v28.s[3]
	fmla	v1.4s, v27.4s, v29.s[3]
	fmla	v2.4s, v27.4s, v30.s[3]
	fmla	v3.4s, v27.4s, v31.s[3]
	fmla	v4.4s, v23.4s, v28.s[3]
//	ldr		q28, [x11], #16
	fmla	v5.4s, v23.4s, v29.s[3]
//	ldr		q29, [x13], #16
	fmla	v6.4s, v23.4s, v30.s[3]
//	ldr		q30, [x14], #16
	fmla	v7.4s, v23.4s, v31.s[3]
//	ldr		q31, [x15], #16

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x9, x9, #32
	sub		x16, x16, #32
	sub		x11, x11, #16
	sub		x13, x13, #16
	sub		x14, x14, #16
	sub		x15, x15, #16

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9], #16
	ldr		q20, [x16], #16
	ldr		s28, [x11], #4
	ldr		s29, [x13], #4
	ldr		s30, [x14], #4
	ldr		s31, [x15], #4
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v29.s[0]
	fmla	v2.4s, v24.4s, v30.s[0]
	fmla	v3.4s, v24.4s, v31.s[0]
	fmla	v4.4s, v20.4s, v28.s[0]
	fmla	v5.4s, v20.4s, v29.s[0]
	fmla	v6.4s, v20.4s, v30.s[0]
	fmla	v7.4s, v20.4s, v31.s[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_8x4_lib4c)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- sda
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NN_8X3_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_8x3_lib4c)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x16, x9, x10

	add		x13, x11, x12
	add		x14, x13, x12
//	add		x15, x14, x12

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x16, #0]
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
//	prfm	PLDL1KEEP, [x15, #0]

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x16, #64]

	// main loop
1:
	
	ldr		q28, [x11], #16
	ldr		q29, [x13], #16
	ldp		q24, q25, [x9], #32
	ldp		q20, q21, [x16], #32

	ldr		q30, [x14], #16
//	ldr		q31, [x15], #16
	ldp		q26, q27, [x9], #32
	ldp		q22, q23, [x16], #32

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	prfm	PLDL1KEEP, [x9, #64]
	fmla	v1.4s, v24.4s, v29.s[0]
	prfm	PLDL1KEEP, [x16, #64]
	fmla	v2.4s, v24.4s, v30.s[0]
	prfm	PLDL1KEEP, [x11, #16]
//	fmla	v3.4s, v24.4s, v31.s[0]
	prfm	PLDL1KEEP, [x13, #16]
	fmla	v4.4s, v20.4s, v28.s[0]
	prfm	PLDL1KEEP, [x14, #16]
	fmla	v5.4s, v20.4s, v29.s[0]
//	prfm	PLDL1KEEP, [x15, #16]
	fmla	v6.4s, v20.4s, v30.s[0]
//	fmla	v7.4s, v20.4s, v31.s[0]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
	fmla	v1.4s, v25.4s, v29.s[1]
	fmla	v2.4s, v25.4s, v30.s[1]
//	fmla	v3.4s, v25.4s, v31.s[1]
	fmla	v4.4s, v21.4s, v28.s[1]
	fmla	v5.4s, v21.4s, v29.s[1]
	fmla	v6.4s, v21.4s, v30.s[1]
//	fmla	v7.4s, v21.4s, v31.s[1]

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
	fmla	v1.4s, v26.4s, v29.s[2]
	fmla	v2.4s, v26.4s, v30.s[2]
//	fmla	v3.4s, v26.4s, v31.s[2]
	fmla	v4.4s, v22.4s, v28.s[2]
	fmla	v5.4s, v22.4s, v29.s[2]
	fmla	v6.4s, v22.4s, v30.s[2]
//	fmla	v7.4s, v22.4s, v31.s[2]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v28.s[3]
	fmla	v1.4s, v27.4s, v29.s[3]
	fmla	v2.4s, v27.4s, v30.s[3]
//	fmla	v3.4s, v27.4s, v31.s[3]
	fmla	v4.4s, v23.4s, v28.s[3]
	fmla	v5.4s, v23.4s, v29.s[3]
	fmla	v6.4s, v23.4s, v30.s[3]
//	fmla	v7.4s, v23.4s, v31.s[3]

	cmp		w8, #4
	bgt		1b

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	ldr		q28, [x11], #16
	ldr		q29, [x13], #16
	ldp		q24, q25, [x9], #32
	ldp		q20, q21, [x16], #32

	ldr		q30, [x14], #16
//	ldr		q31, [x15], #16
	ldp		q26, q27, [x9], #32
	ldp		q22, q23, [x16], #32

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v29.s[0]
	fmla	v2.4s, v24.4s, v30.s[0]
//	fmla	v3.4s, v24.4s, v31.s[0]
	fmla	v4.4s, v20.4s, v28.s[0]
	fmla	v5.4s, v20.4s, v29.s[0]
	fmla	v6.4s, v20.4s, v30.s[0]
//	fmla	v7.4s, v20.4s, v31.s[0]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
	fmla	v1.4s, v25.4s, v29.s[1]
	fmla	v2.4s, v25.4s, v30.s[1]
//	fmla	v3.4s, v25.4s, v31.s[1]
	fmla	v4.4s, v21.4s, v28.s[1]
	fmla	v5.4s, v21.4s, v29.s[1]
	fmla	v6.4s, v21.4s, v30.s[1]
//	fmla	v7.4s, v21.4s, v31.s[1]

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
	fmla	v1.4s, v26.4s, v29.s[2]
	fmla	v2.4s, v26.4s, v30.s[2]
//	fmla	v3.4s, v26.4s, v31.s[2]
	fmla	v4.4s, v22.4s, v28.s[2]
	fmla	v5.4s, v22.4s, v29.s[2]
	fmla	v6.4s, v22.4s, v30.s[2]
//	fmla	v7.4s, v22.4s, v31.s[2]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v28.s[3]
	fmla	v1.4s, v27.4s, v29.s[3]
	fmla	v2.4s, v27.4s, v30.s[3]
//	fmla	v3.4s, v27.4s, v31.s[3]
	fmla	v4.4s, v23.4s, v28.s[3]
	fmla	v5.4s, v23.4s, v29.s[3]
	fmla	v6.4s, v23.4s, v30.s[3]
//	fmla	v7.4s, v23.4s, v31.s[3]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9], #16
	ldr		q20, [x16], #16
	ldr		s28, [x11], #4
	ldr		s29, [x13], #4
	ldr		s30, [x14], #4
//	ldr		s31, [x15], #4
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v29.s[0]
	fmla	v2.4s, v24.4s, v30.s[0]
//	fmla	v3.4s, v24.4s, v31.s[0]
	fmla	v4.4s, v20.4s, v28.s[0]
	fmla	v5.4s, v20.4s, v29.s[0]
	fmla	v6.4s, v20.4s, v30.s[0]
//	fmla	v7.4s, v20.4s, v31.s[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else // cortex a53



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x16, x9, x10

	add		x13, x11, x12
	add		x14, x13, x12
//	add		x15, x14, x12

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x16, #0]
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
//	prfm	PLDL1KEEP, [x15, #0]

	// preload
	ldr		q28, [x11], #16
	ldr		q29, [x13], #16
	ldr		q30, [x14], #16
//	ldr		q31, [x15], #16

	ldp		q24, q25, [x9], #32
	ldp		q20, q21, [x16], #32

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #32]
	prfm	PLDL1KEEP, [x16, #64]

	// main loop
1:
	

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	ldp		q26, q27, [x9], #32
	fmla	v1.4s, v24.4s, v29.s[0]
	ldp		q22, q23, [x16], #32
	fmla	v2.4s, v24.4s, v30.s[0]
	prfm	PLDL1KEEP, [x9, #64]
//	fmla	v3.4s, v24.4s, v31.s[0]
	prfm	PLDL1KEEP, [x16, #64]
	fmla	v4.4s, v20.4s, v28.s[0]
	prfm	PLDL1KEEP, [x11, #16]
	fmla	v5.4s, v20.4s, v29.s[0]
	prfm	PLDL1KEEP, [x13, #16]
	fmla	v6.4s, v20.4s, v30.s[0]
	prfm	PLDL1KEEP, [x14, #16]
//	fmla	v7.4s, v20.4s, v31.s[0]
//	prfm	PLDL1KEEP, [x15, #16]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
	fmla	v1.4s, v25.4s, v29.s[1]
	fmla	v2.4s, v25.4s, v30.s[1]
//	fmla	v3.4s, v25.4s, v31.s[1]
	ldp		q24, q25, [x9], #32
	fmla	v4.4s, v21.4s, v28.s[1]
	fmla	v5.4s, v21.4s, v29.s[1]
	fmla	v6.4s, v21.4s, v30.s[1]
//	fmla	v7.4s, v21.4s, v31.s[1]
	ldp		q20, q21, [x16], #32

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
	fmla	v1.4s, v26.4s, v29.s[2]
	fmla	v2.4s, v26.4s, v30.s[2]
//	fmla	v3.4s, v26.4s, v31.s[2]
	fmla	v4.4s, v22.4s, v28.s[2]
	fmla	v5.4s, v22.4s, v29.s[2]
	fmla	v6.4s, v22.4s, v30.s[2]
//	fmla	v7.4s, v22.4s, v31.s[2]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v28.s[3]
	fmla	v1.4s, v27.4s, v29.s[3]
	fmla	v2.4s, v27.4s, v30.s[3]
//	fmla	v3.4s, v27.4s, v31.s[3]
	fmla	v4.4s, v23.4s, v28.s[3]
	ldr		q28, [x11], #16
	fmla	v5.4s, v23.4s, v29.s[3]
	ldr		q29, [x13], #16
	fmla	v6.4s, v23.4s, v30.s[3]
	ldr		q30, [x14], #16
//	fmla	v7.4s, v23.4s, v31.s[3]
//	ldr		q31, [x15], #16

	cmp		w8, #4
	bgt		1b

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	ldp		q26, q27, [x9], #32
	fmla	v1.4s, v24.4s, v29.s[0]
	ldp		q22, q23, [x16], #32
	fmla	v2.4s, v24.4s, v30.s[0]
//	prfm	PLDL1KEEP, [x9, #64]
//	fmla	v3.4s, v24.4s, v31.s[0]
//	prfm	PLDL1KEEP, [x16, #64]
	fmla	v4.4s, v20.4s, v28.s[0]
//	prfm	PLDL1KEEP, [x11, #16]
	fmla	v5.4s, v20.4s, v29.s[0]
//	prfm	PLDL1KEEP, [x13, #16]
	fmla	v6.4s, v20.4s, v30.s[0]
//	prfm	PLDL1KEEP, [x14, #16]
//	fmla	v7.4s, v20.4s, v31.s[0]
//	prfm	PLDL1KEEP, [x15, #16]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
	fmla	v1.4s, v25.4s, v29.s[1]
	fmla	v2.4s, v25.4s, v30.s[1]
//	fmla	v3.4s, v25.4s, v31.s[1]
//	ldp		q24, q25, [x9], #32
	fmla	v4.4s, v21.4s, v28.s[1]
	fmla	v5.4s, v21.4s, v29.s[1]
	fmla	v6.4s, v21.4s, v30.s[1]
//	fmla	v7.4s, v21.4s, v31.s[1]
//	ldp		q20, q21, [x16], #32

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
	fmla	v1.4s, v26.4s, v29.s[2]
	fmla	v2.4s, v26.4s, v30.s[2]
//	fmla	v3.4s, v26.4s, v31.s[2]
	fmla	v4.4s, v22.4s, v28.s[2]
	fmla	v5.4s, v22.4s, v29.s[2]
	fmla	v6.4s, v22.4s, v30.s[2]
//	fmla	v7.4s, v22.4s, v31.s[2]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v28.s[3]
	fmla	v1.4s, v27.4s, v29.s[3]
	fmla	v2.4s, v27.4s, v30.s[3]
//	fmla	v3.4s, v27.4s, v31.s[3]
	fmla	v4.4s, v23.4s, v28.s[3]
//	ldr		q28, [x11], #16
	fmla	v5.4s, v23.4s, v29.s[3]
//	ldr		q29, [x13], #16
	fmla	v6.4s, v23.4s, v30.s[3]
//	ldr		q30, [x14], #16
//	fmla	v7.4s, v23.4s, v31.s[3]
//	ldr		q31, [x15], #16

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x9, x9, #32
	sub		x16, x16, #32
	sub		x11, x11, #16
	sub		x13, x13, #16
	sub		x14, x14, #16
//	sub		x15, x15, #16

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9], #16
	ldr		q20, [x16], #16
	ldr		s28, [x11], #4
	ldr		s29, [x13], #4
	ldr		s30, [x14], #4
//	ldr		s31, [x15], #4
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v29.s[0]
	fmla	v2.4s, v24.4s, v30.s[0]
//	fmla	v3.4s, v24.4s, v31.s[0]
	fmla	v4.4s, v20.4s, v28.s[0]
	fmla	v5.4s, v20.4s, v29.s[0]
	fmla	v6.4s, v20.4s, v30.s[0]
//	fmla	v7.4s, v20.4s, v31.s[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_8x3_lib4c)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- sda
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NN_8X2_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_8x2_lib4c)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x16, x9, x10

	add		x13, x11, x12
//	add		x14, x13, x12
//	add		x15, x14, x12

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x16, #0]
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x13, #0]
//	prfm	PLDL1KEEP, [x14, #0]
//	prfm	PLDL1KEEP, [x15, #0]

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x16, #64]

	// main loop
1:
	
	ldr		q28, [x11], #16
	ldr		q29, [x13], #16
	ldp		q24, q25, [x9], #32
	ldp		q20, q21, [x16], #32

//	ldr		q30, [x14], #16
//	ldr		q31, [x15], #16
	ldp		q26, q27, [x9], #32
	ldp		q22, q23, [x16], #32

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	prfm	PLDL1KEEP, [x9, #64]
	fmla	v1.4s, v24.4s, v29.s[0]
	prfm	PLDL1KEEP, [x16, #64]
//	fmla	v2.4s, v24.4s, v30.s[0]
	prfm	PLDL1KEEP, [x11, #16]
//	fmla	v3.4s, v24.4s, v31.s[0]
	prfm	PLDL1KEEP, [x13, #16]
	fmla	v4.4s, v20.4s, v28.s[0]
//	prfm	PLDL1KEEP, [x14, #16]
	fmla	v5.4s, v20.4s, v29.s[0]
//	prfm	PLDL1KEEP, [x15, #16]
//	fmla	v6.4s, v20.4s, v30.s[0]
//	fmla	v7.4s, v20.4s, v31.s[0]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
	fmla	v1.4s, v25.4s, v29.s[1]
//	fmla	v2.4s, v25.4s, v30.s[1]
//	fmla	v3.4s, v25.4s, v31.s[1]
	fmla	v4.4s, v21.4s, v28.s[1]
	fmla	v5.4s, v21.4s, v29.s[1]
//	fmla	v6.4s, v21.4s, v30.s[1]
//	fmla	v7.4s, v21.4s, v31.s[1]

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
	fmla	v1.4s, v26.4s, v29.s[2]
//	fmla	v2.4s, v26.4s, v30.s[2]
//	fmla	v3.4s, v26.4s, v31.s[2]
	fmla	v4.4s, v22.4s, v28.s[2]
	fmla	v5.4s, v22.4s, v29.s[2]
//	fmla	v6.4s, v22.4s, v30.s[2]
//	fmla	v7.4s, v22.4s, v31.s[2]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v28.s[3]
	fmla	v1.4s, v27.4s, v29.s[3]
//	fmla	v2.4s, v27.4s, v30.s[3]
//	fmla	v3.4s, v27.4s, v31.s[3]
	fmla	v4.4s, v23.4s, v28.s[3]
	fmla	v5.4s, v23.4s, v29.s[3]
//	fmla	v6.4s, v23.4s, v30.s[3]
//	fmla	v7.4s, v23.4s, v31.s[3]

	cmp		w8, #4
	bgt		1b

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	ldr		q28, [x11], #16
	ldr		q29, [x13], #16
	ldp		q24, q25, [x9], #32
	ldp		q20, q21, [x16], #32

//	ldr		q30, [x14], #16
//	ldr		q31, [x15], #16
	ldp		q26, q27, [x9], #32
	ldp		q22, q23, [x16], #32

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v29.s[0]
//	fmla	v2.4s, v24.4s, v30.s[0]
//	fmla	v3.4s, v24.4s, v31.s[0]
	fmla	v4.4s, v20.4s, v28.s[0]
	fmla	v5.4s, v20.4s, v29.s[0]
//	fmla	v6.4s, v20.4s, v30.s[0]
//	fmla	v7.4s, v20.4s, v31.s[0]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
	fmla	v1.4s, v25.4s, v29.s[1]
//	fmla	v2.4s, v25.4s, v30.s[1]
//	fmla	v3.4s, v25.4s, v31.s[1]
	fmla	v4.4s, v21.4s, v28.s[1]
	fmla	v5.4s, v21.4s, v29.s[1]
//	fmla	v6.4s, v21.4s, v30.s[1]
//	fmla	v7.4s, v21.4s, v31.s[1]

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
	fmla	v1.4s, v26.4s, v29.s[2]
//	fmla	v2.4s, v26.4s, v30.s[2]
//	fmla	v3.4s, v26.4s, v31.s[2]
	fmla	v4.4s, v22.4s, v28.s[2]
	fmla	v5.4s, v22.4s, v29.s[2]
//	fmla	v6.4s, v22.4s, v30.s[2]
//	fmla	v7.4s, v22.4s, v31.s[2]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v28.s[3]
	fmla	v1.4s, v27.4s, v29.s[3]
//	fmla	v2.4s, v27.4s, v30.s[3]
//	fmla	v3.4s, v27.4s, v31.s[3]
	fmla	v4.4s, v23.4s, v28.s[3]
	fmla	v5.4s, v23.4s, v29.s[3]
//	fmla	v6.4s, v23.4s, v30.s[3]
//	fmla	v7.4s, v23.4s, v31.s[3]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9], #16
	ldr		q20, [x16], #16
	ldr		s28, [x11], #4
	ldr		s29, [x13], #4
//	ldr		s30, [x14], #4
//	ldr		s31, [x15], #4
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v29.s[0]
//	fmla	v2.4s, v24.4s, v30.s[0]
//	fmla	v3.4s, v24.4s, v31.s[0]
	fmla	v4.4s, v20.4s, v28.s[0]
	fmla	v5.4s, v20.4s, v29.s[0]
//	fmla	v6.4s, v20.4s, v30.s[0]
//	fmla	v7.4s, v20.4s, v31.s[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else // cortex a53



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x16, x9, x10

	add		x13, x11, x12
//	add		x14, x13, x12
//	add		x15, x14, x12

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x16, #0]
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x13, #0]
//	prfm	PLDL1KEEP, [x14, #0]
//	prfm	PLDL1KEEP, [x15, #0]

	// preload
	ldr		q28, [x11], #16
	ldr		q29, [x13], #16
//	ldr		q30, [x14], #16
//	ldr		q31, [x15], #16

	ldp		q24, q25, [x9], #32
	ldp		q20, q21, [x16], #32

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #32]
	prfm	PLDL1KEEP, [x16, #64]

	// main loop
1:
	

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	ldp		q26, q27, [x9], #32
	fmla	v1.4s, v24.4s, v29.s[0]
	ldp		q22, q23, [x16], #32
//	fmla	v2.4s, v24.4s, v30.s[0]
	prfm	PLDL1KEEP, [x9, #64]
//	fmla	v3.4s, v24.4s, v31.s[0]
	prfm	PLDL1KEEP, [x16, #64]
	fmla	v4.4s, v20.4s, v28.s[0]
	prfm	PLDL1KEEP, [x11, #16]
	fmla	v5.4s, v20.4s, v29.s[0]
	prfm	PLDL1KEEP, [x13, #16]
//	fmla	v6.4s, v20.4s, v30.s[0]
//	prfm	PLDL1KEEP, [x14, #16]
//	fmla	v7.4s, v20.4s, v31.s[0]
//	prfm	PLDL1KEEP, [x15, #16]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
	fmla	v1.4s, v25.4s, v29.s[1]
//	fmla	v2.4s, v25.4s, v30.s[1]
//	fmla	v3.4s, v25.4s, v31.s[1]
	ldp		q24, q25, [x9], #32
	fmla	v4.4s, v21.4s, v28.s[1]
	fmla	v5.4s, v21.4s, v29.s[1]
//	fmla	v6.4s, v21.4s, v30.s[1]
//	fmla	v7.4s, v21.4s, v31.s[1]
	ldp		q20, q21, [x16], #32

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
	fmla	v1.4s, v26.4s, v29.s[2]
//	fmla	v2.4s, v26.4s, v30.s[2]
//	fmla	v3.4s, v26.4s, v31.s[2]
	fmla	v4.4s, v22.4s, v28.s[2]
	fmla	v5.4s, v22.4s, v29.s[2]
//	fmla	v6.4s, v22.4s, v30.s[2]
//	fmla	v7.4s, v22.4s, v31.s[2]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v28.s[3]
	fmla	v1.4s, v27.4s, v29.s[3]
//	fmla	v2.4s, v27.4s, v30.s[3]
//	fmla	v3.4s, v27.4s, v31.s[3]
	fmla	v4.4s, v23.4s, v28.s[3]
	ldr		q28, [x11], #16
	fmla	v5.4s, v23.4s, v29.s[3]
	ldr		q29, [x13], #16
//	fmla	v6.4s, v23.4s, v30.s[3]
//	ldr		q30, [x14], #16
//	fmla	v7.4s, v23.4s, v31.s[3]
//	ldr		q31, [x15], #16

	cmp		w8, #4
	bgt		1b

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	ldp		q26, q27, [x9], #32
	fmla	v1.4s, v24.4s, v29.s[0]
	ldp		q22, q23, [x16], #32
//	fmla	v2.4s, v24.4s, v30.s[0]
//	prfm	PLDL1KEEP, [x9, #64]
//	fmla	v3.4s, v24.4s, v31.s[0]
//	prfm	PLDL1KEEP, [x16, #64]
	fmla	v4.4s, v20.4s, v28.s[0]
//	prfm	PLDL1KEEP, [x11, #16]
	fmla	v5.4s, v20.4s, v29.s[0]
//	prfm	PLDL1KEEP, [x13, #16]
//	fmla	v6.4s, v20.4s, v30.s[0]
//	prfm	PLDL1KEEP, [x14, #16]
//	fmla	v7.4s, v20.4s, v31.s[0]
//	prfm	PLDL1KEEP, [x15, #16]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
	fmla	v1.4s, v25.4s, v29.s[1]
//	fmla	v2.4s, v25.4s, v30.s[1]
//	fmla	v3.4s, v25.4s, v31.s[1]
//	ldp		q24, q25, [x9], #32
	fmla	v4.4s, v21.4s, v28.s[1]
	fmla	v5.4s, v21.4s, v29.s[1]
//	fmla	v6.4s, v21.4s, v30.s[1]
//	fmla	v7.4s, v21.4s, v31.s[1]
//	ldp		q20, q21, [x16], #32

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
	fmla	v1.4s, v26.4s, v29.s[2]
//	fmla	v2.4s, v26.4s, v30.s[2]
//	fmla	v3.4s, v26.4s, v31.s[2]
	fmla	v4.4s, v22.4s, v28.s[2]
	fmla	v5.4s, v22.4s, v29.s[2]
//	fmla	v6.4s, v22.4s, v30.s[2]
//	fmla	v7.4s, v22.4s, v31.s[2]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v28.s[3]
	fmla	v1.4s, v27.4s, v29.s[3]
//	fmla	v2.4s, v27.4s, v30.s[3]
//	fmla	v3.4s, v27.4s, v31.s[3]
	fmla	v4.4s, v23.4s, v28.s[3]
//	ldr		q28, [x11], #16
	fmla	v5.4s, v23.4s, v29.s[3]
//	ldr		q29, [x13], #16
//	fmla	v6.4s, v23.4s, v30.s[3]
//	ldr		q30, [x14], #16
//	fmla	v7.4s, v23.4s, v31.s[3]
//	ldr		q31, [x15], #16

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x9, x9, #32
	sub		x16, x16, #32
	sub		x11, x11, #16
	sub		x13, x13, #16
//	sub		x14, x14, #16
//	sub		x15, x15, #16

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9], #16
	ldr		q20, [x16], #16
	ldr		s28, [x11], #4
	ldr		s29, [x13], #4
//	ldr		s30, [x14], #4
//	ldr		s31, [x15], #4
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v29.s[0]
//	fmla	v2.4s, v24.4s, v30.s[0]
//	fmla	v3.4s, v24.4s, v31.s[0]
	fmla	v4.4s, v20.4s, v28.s[0]
	fmla	v5.4s, v20.4s, v29.s[0]
//	fmla	v6.4s, v20.4s, v30.s[0]
//	fmla	v7.4s, v20.4s, v31.s[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_8x2_lib4c)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- sda
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NN_8X1_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_8x1_lib4c)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x16, x9, x10

//	add		x13, x11, x12
//	add		x14, x13, x12
//	add		x15, x14, x12

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x16, #0]
	prfm	PLDL1KEEP, [x11, #0]
//	prfm	PLDL1KEEP, [x13, #0]
//	prfm	PLDL1KEEP, [x14, #0]
//	prfm	PLDL1KEEP, [x15, #0]

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x16, #64]

	// main loop
1:
	
	ldr		q28, [x11], #16
//	ldr		q29, [x13], #16
	ldp		q24, q25, [x9], #32
	ldp		q20, q21, [x16], #32

//	ldr		q30, [x14], #16
//	ldr		q31, [x15], #16
	ldp		q26, q27, [x9], #32
	ldp		q22, q23, [x16], #32

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	prfm	PLDL1KEEP, [x9, #64]
//	fmla	v1.4s, v24.4s, v29.s[0]
	prfm	PLDL1KEEP, [x16, #64]
//	fmla	v2.4s, v24.4s, v30.s[0]
	prfm	PLDL1KEEP, [x11, #16]
//	fmla	v3.4s, v24.4s, v31.s[0]
//	prfm	PLDL1KEEP, [x13, #16]
	fmla	v4.4s, v20.4s, v28.s[0]
//	prfm	PLDL1KEEP, [x14, #16]
//	fmla	v5.4s, v20.4s, v29.s[0]
//	prfm	PLDL1KEEP, [x15, #16]
//	fmla	v6.4s, v20.4s, v30.s[0]
//	fmla	v7.4s, v20.4s, v31.s[0]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
//	fmla	v1.4s, v25.4s, v29.s[1]
//	fmla	v2.4s, v25.4s, v30.s[1]
//	fmla	v3.4s, v25.4s, v31.s[1]
	fmla	v4.4s, v21.4s, v28.s[1]
//	fmla	v5.4s, v21.4s, v29.s[1]
//	fmla	v6.4s, v21.4s, v30.s[1]
//	fmla	v7.4s, v21.4s, v31.s[1]

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
//	fmla	v1.4s, v26.4s, v29.s[2]
//	fmla	v2.4s, v26.4s, v30.s[2]
//	fmla	v3.4s, v26.4s, v31.s[2]
	fmla	v4.4s, v22.4s, v28.s[2]
//	fmla	v5.4s, v22.4s, v29.s[2]
//	fmla	v6.4s, v22.4s, v30.s[2]
//	fmla	v7.4s, v22.4s, v31.s[2]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v28.s[3]
//	fmla	v1.4s, v27.4s, v29.s[3]
//	fmla	v2.4s, v27.4s, v30.s[3]
//	fmla	v3.4s, v27.4s, v31.s[3]
	fmla	v4.4s, v23.4s, v28.s[3]
//	fmla	v5.4s, v23.4s, v29.s[3]
//	fmla	v6.4s, v23.4s, v30.s[3]
//	fmla	v7.4s, v23.4s, v31.s[3]

	cmp		w8, #4
	bgt		1b

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	ldr		q28, [x11], #16
//	ldr		q29, [x13], #16
	ldp		q24, q25, [x9], #32
	ldp		q20, q21, [x16], #32

//	ldr		q30, [x14], #16
//	ldr		q31, [x15], #16
	ldp		q26, q27, [x9], #32
	ldp		q22, q23, [x16], #32

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
//	fmla	v1.4s, v24.4s, v29.s[0]
//	fmla	v2.4s, v24.4s, v30.s[0]
//	fmla	v3.4s, v24.4s, v31.s[0]
	fmla	v4.4s, v20.4s, v28.s[0]
//	fmla	v5.4s, v20.4s, v29.s[0]
//	fmla	v6.4s, v20.4s, v30.s[0]
//	fmla	v7.4s, v20.4s, v31.s[0]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
//	fmla	v1.4s, v25.4s, v29.s[1]
//	fmla	v2.4s, v25.4s, v30.s[1]
//	fmla	v3.4s, v25.4s, v31.s[1]
	fmla	v4.4s, v21.4s, v28.s[1]
//	fmla	v5.4s, v21.4s, v29.s[1]
//	fmla	v6.4s, v21.4s, v30.s[1]
//	fmla	v7.4s, v21.4s, v31.s[1]

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
//	fmla	v1.4s, v26.4s, v29.s[2]
//	fmla	v2.4s, v26.4s, v30.s[2]
//	fmla	v3.4s, v26.4s, v31.s[2]
	fmla	v4.4s, v22.4s, v28.s[2]
//	fmla	v5.4s, v22.4s, v29.s[2]
//	fmla	v6.4s, v22.4s, v30.s[2]
//	fmla	v7.4s, v22.4s, v31.s[2]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v28.s[3]
//	fmla	v1.4s, v27.4s, v29.s[3]
//	fmla	v2.4s, v27.4s, v30.s[3]
//	fmla	v3.4s, v27.4s, v31.s[3]
	fmla	v4.4s, v23.4s, v28.s[3]
//	fmla	v5.4s, v23.4s, v29.s[3]
//	fmla	v6.4s, v23.4s, v30.s[3]
//	fmla	v7.4s, v23.4s, v31.s[3]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9], #16
	ldr		q20, [x16], #16
	ldr		s28, [x11], #4
//	ldr		s29, [x13], #4
//	ldr		s30, [x14], #4
//	ldr		s31, [x15], #4
	fmla	v0.4s, v24.4s, v28.s[0]
//	fmla	v1.4s, v24.4s, v29.s[0]
//	fmla	v2.4s, v24.4s, v30.s[0]
//	fmla	v3.4s, v24.4s, v31.s[0]
	fmla	v4.4s, v20.4s, v28.s[0]
//	fmla	v5.4s, v20.4s, v29.s[0]
//	fmla	v6.4s, v20.4s, v30.s[0]
//	fmla	v7.4s, v20.4s, v31.s[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else // cortex a53



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x16, x9, x10

//	add		x13, x11, x12
//	add		x14, x13, x12
//	add		x15, x14, x12

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x16, #0]
	prfm	PLDL1KEEP, [x11, #0]
//	prfm	PLDL1KEEP, [x13, #0]
//	prfm	PLDL1KEEP, [x14, #0]
//	prfm	PLDL1KEEP, [x15, #0]

	// preload
	ldr		q28, [x11], #16
//	ldr		q29, [x13], #16
//	ldr		q30, [x14], #16
//	ldr		q31, [x15], #16

	ldp		q24, q25, [x9], #32
	ldp		q20, q21, [x16], #32

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #32]
	prfm	PLDL1KEEP, [x16, #64]

	// main loop
1:
	

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	ldp		q26, q27, [x9], #32
//	fmla	v1.4s, v24.4s, v29.s[0]
	ldp		q22, q23, [x16], #32
//	fmla	v2.4s, v24.4s, v30.s[0]
	prfm	PLDL1KEEP, [x9, #64]
//	fmla	v3.4s, v24.4s, v31.s[0]
	prfm	PLDL1KEEP, [x16, #64]
	fmla	v4.4s, v20.4s, v28.s[0]
	prfm	PLDL1KEEP, [x11, #16]
//	fmla	v5.4s, v20.4s, v29.s[0]
//	prfm	PLDL1KEEP, [x13, #16]
//	fmla	v6.4s, v20.4s, v30.s[0]
//	prfm	PLDL1KEEP, [x14, #16]
//	fmla	v7.4s, v20.4s, v31.s[0]
//	prfm	PLDL1KEEP, [x15, #16]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
//	fmla	v1.4s, v25.4s, v29.s[1]
//	fmla	v2.4s, v25.4s, v30.s[1]
//	fmla	v3.4s, v25.4s, v31.s[1]
	ldp		q24, q25, [x9], #32
	fmla	v4.4s, v21.4s, v28.s[1]
//	fmla	v5.4s, v21.4s, v29.s[1]
//	fmla	v6.4s, v21.4s, v30.s[1]
//	fmla	v7.4s, v21.4s, v31.s[1]
	ldp		q20, q21, [x16], #32

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
//	fmla	v1.4s, v26.4s, v29.s[2]
//	fmla	v2.4s, v26.4s, v30.s[2]
//	fmla	v3.4s, v26.4s, v31.s[2]
	fmla	v4.4s, v22.4s, v28.s[2]
//	fmla	v5.4s, v22.4s, v29.s[2]
//	fmla	v6.4s, v22.4s, v30.s[2]
//	fmla	v7.4s, v22.4s, v31.s[2]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v28.s[3]
//	fmla	v1.4s, v27.4s, v29.s[3]
//	fmla	v2.4s, v27.4s, v30.s[3]
//	fmla	v3.4s, v27.4s, v31.s[3]
	fmla	v4.4s, v23.4s, v28.s[3]
	ldr		q28, [x11], #16
//	fmla	v5.4s, v23.4s, v29.s[3]
//	ldr		q29, [x13], #16
//	fmla	v6.4s, v23.4s, v30.s[3]
//	ldr		q30, [x14], #16
//	fmla	v7.4s, v23.4s, v31.s[3]
//	ldr		q31, [x15], #16

	cmp		w8, #4
	bgt		1b

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	ldp		q26, q27, [x9], #32
//	fmla	v1.4s, v24.4s, v29.s[0]
	ldp		q22, q23, [x16], #32
//	fmla	v2.4s, v24.4s, v30.s[0]
//	prfm	PLDL1KEEP, [x9, #64]
//	fmla	v3.4s, v24.4s, v31.s[0]
//	prfm	PLDL1KEEP, [x16, #64]
	fmla	v4.4s, v20.4s, v28.s[0]
//	prfm	PLDL1KEEP, [x11, #16]
//	fmla	v5.4s, v20.4s, v29.s[0]
//	prfm	PLDL1KEEP, [x13, #16]
//	fmla	v6.4s, v20.4s, v30.s[0]
//	prfm	PLDL1KEEP, [x14, #16]
//	fmla	v7.4s, v20.4s, v31.s[0]
//	prfm	PLDL1KEEP, [x15, #16]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
//	fmla	v1.4s, v25.4s, v29.s[1]
//	fmla	v2.4s, v25.4s, v30.s[1]
//	fmla	v3.4s, v25.4s, v31.s[1]
//	ldp		q24, q25, [x9], #32
	fmla	v4.4s, v21.4s, v28.s[1]
//	fmla	v5.4s, v21.4s, v29.s[1]
//	fmla	v6.4s, v21.4s, v30.s[1]
//	fmla	v7.4s, v21.4s, v31.s[1]
//	ldp		q20, q21, [x16], #32

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
//	fmla	v1.4s, v26.4s, v29.s[2]
//	fmla	v2.4s, v26.4s, v30.s[2]
//	fmla	v3.4s, v26.4s, v31.s[2]
	fmla	v4.4s, v22.4s, v28.s[2]
//	fmla	v5.4s, v22.4s, v29.s[2]
//	fmla	v6.4s, v22.4s, v30.s[2]
//	fmla	v7.4s, v22.4s, v31.s[2]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v28.s[3]
//	fmla	v1.4s, v27.4s, v29.s[3]
//	fmla	v2.4s, v27.4s, v30.s[3]
//	fmla	v3.4s, v27.4s, v31.s[3]
	fmla	v4.4s, v23.4s, v28.s[3]
//	ldr		q28, [x11], #16
//	fmla	v5.4s, v23.4s, v29.s[3]
//	ldr		q29, [x13], #16
//	fmla	v6.4s, v23.4s, v30.s[3]
//	ldr		q30, [x14], #16
//	fmla	v7.4s, v23.4s, v31.s[3]
//	ldr		q31, [x15], #16

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x9, x9, #32
	sub		x16, x16, #32
	sub		x11, x11, #16
//	sub		x13, x13, #16
//	sub		x14, x14, #16
//	sub		x15, x15, #16

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9], #16
	ldr		q20, [x16], #16
	ldr		s28, [x11], #4
//	ldr		s29, [x13], #4
//	ldr		s30, [x14], #4
//	ldr		s31, [x15], #4
	fmla	v0.4s, v24.4s, v28.s[0]
//	fmla	v1.4s, v24.4s, v29.s[0]
//	fmla	v2.4s, v24.4s, v30.s[0]
//	fmla	v3.4s, v24.4s, v31.s[0]
	fmla	v4.4s, v20.4s, v28.s[0]
//	fmla	v5.4s, v20.4s, v29.s[0]
//	fmla	v6.4s, v20.4s, v30.s[0]
//	fmla	v7.4s, v20.4s, v31.s[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_8x1_lib4c)
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
// x8   <- E
// x9   <- lde
// x10  <- inv_diag_E
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRSM_RLT_INV_8X4_LIB
#else
	.align 4
	FUN_START(inner_edge_trsm_rlt_inv_8x4_lib)
#endif

	ldr			s16, [x10, #0] // E_inv[0]
	fmul		v0.4s, v0.4s, v16.s[0]
	fmul		v4.4s, v4.4s, v16.s[0]
	ldr			s16, [x8, #4] // E[1+4*0]
	fmls		v1.4s, v0.4s, v16.s[0]
	fmls		v5.4s, v4.4s, v16.s[0]
	ldr			s16, [x8, #8] // E[2+4*0]
	fmls		v2.4s, v0.4s, v16.s[0]
	fmls		v6.4s, v4.4s, v16.s[0]
	ldr			s16, [x8, #12] // E[3+4*0]
	fmls		v3.4s, v0.4s, v16.s[0]
	fmls		v7.4s, v4.4s, v16.s[0]
	add			x8, x8, x9

	ldr			s16, [x10, #4] // E_inv[1]
	fmul		v1.4s, v1.4s, v16.s[0]
	fmul		v5.4s, v5.4s, v16.s[0]
	ldr			s16, [x8, #8] // E[2+4*1]
	fmls		v2.4s, v1.4s, v16.s[0]
	fmls		v6.4s, v5.4s, v16.s[0]
	ldr			s16, [x8, #12] // E[3+4*1]
	fmls		v3.4s, v1.4s, v16.s[0]
	fmls		v7.4s, v5.4s, v16.s[0]
	add			x8, x8, x9

	ldr			s16, [x10, #8] // E_inv[2]
	fmul		v2.4s, v2.4s, v16.s[0]
	fmul		v6.4s, v6.4s, v16.s[0]
	ldr			s16, [x8, #12] // E[3+4*2]
	fmls		v3.4s, v2.4s, v16.s[0]
	fmls		v7.4s, v6.4s, v16.s[0]
//	add			x8, x8, x9

	ldr			s16, [x10, #12] // E_inv[3]
	fmul		v3.4s, v3.4s, v16.s[0]
	fmul		v7.4s, v7.4s, v16.s[0]
//	add			x8, x8, x9

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trsm_rlt_inv_8x4_lib)
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
// x8   <- E
// w9   <- lde
// x10  <- inv_diag_E
// w11  <- n1
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRSM_RLT_INV_8X4_VS_LIB
#else
	.align 4
	FUN_START(inner_edge_trsm_rlt_inv_8x4_vs_lib)
#endif
	
	// first column
	ldr			s16, [x10, #0] // E_inv[0]
	fmul		v0.4s, v0.4s, v16.s[0]
	fmul		v4.4s, v4.4s, v16.s[0]
	cmp			w11, #2
	blt			0f // return

	// second column
	ldr			s16, [x8, #4] // E[1+4*0]
	fmls		v1.4s, v0.4s, v16.s[0]
	fmls		v5.4s, v4.4s, v16.s[0]
	ldr			s16, [x10, #4] // E_inv[1]
	fmul		v1.4s, v1.4s, v16.s[0]
	fmul		v5.4s, v5.4s, v16.s[0]
	cmp			w11, #3
	blt			0f // return

	// third column
	add			x12, x8, x9
	ldr			s16, [x8, #8] // E[2+4*0]
	fmls		v2.4s, v0.4s, v16.s[0]
	fmls		v6.4s, v4.4s, v16.s[0]
	ldr			s16, [x12, #8] // E[2+4*1]
	fmls		v2.4s, v1.4s, v16.s[0]
	fmls		v6.4s, v5.4s, v16.s[0]
	ldr			s16, [x10, #8] // E_inv[2]
	fmul		v2.4s, v2.4s, v16.s[0]
	fmul		v6.4s, v6.4s, v16.s[0]
	cmp			w11, #4
	blt			0f // return

	// forth column
	add			x13, x12, x9
	ldr			s16, [x8, #12] // E[3+4*0]
	fmls		v3.4s, v0.4s, v16.s[0]
	fmls		v7.4s, v4.4s, v16.s[0]
	ldr			s16, [x12, #12] // E[3+4*1]
	fmls		v3.4s, v1.4s, v16.s[0]
	fmls		v7.4s, v5.4s, v16.s[0]
	ldr			s16, [x13, #12] // E[3+4*2]
	fmls		v3.4s, v2.4s, v16.s[0]
	fmls		v7.4s, v6.4s, v16.s[0]
	ldr			s16, [x10, #12] // E_inv[3]
	fmul		v3.4s, v3.4s, v16.s[0]
	fmul		v7.4s, v7.4s, v16.s[0]

0:
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trsm_rlt_inv_8x4_vs_lib)
#endif





// subroutine
//
// input arguments:
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_TRAN_8X4_LIB
#else
	.align	4
	FUN_START(inner_tran_8x4_lib)
#endif

	trn1	v8.4s, v0.4s, v1.4s
	trn2	v9.4s, v0.4s, v1.4s
	trn1	v10.4s, v2.4s, v3.4s
	trn2	v11.4s, v2.4s, v3.4s

	trn1	v0.2d, v8.2d, v10.2d
	trn2	v2.2d, v8.2d, v10.2d
	trn1	v1.2d, v9.2d, v11.2d
	trn2	v3.2d, v9.2d, v11.2d

	trn1	v8.4s, v4.4s, v5.4s
	trn2	v9.4s, v4.4s, v5.4s
	trn1	v10.4s, v6.4s, v7.4s
	trn2	v11.4s, v6.4s, v7.4s

	trn1	v4.2d, v8.2d, v10.2d
	trn2	v6.2d, v8.2d, v10.2d
	trn1	v5.2d, v9.2d, v11.2d
	trn2	v7.2d, v9.2d, v11.2d

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_tran_8x4_lib)
#endif





// subroutine
//
// input arguments:
// x8   <- alpha
// x9   <- beta
// x10  <- C
// x11  <- ldc*sizeof(float)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_8X4_LIB
#else
	.align	4
	FUN_START(inner_scale_ab_8x4_lib)
#endif

	ld1		{v28.4s}, [x8]

	ld1		{v29.4s}, [x9]

	fmul	v0.4s, v0.4s, v28.s[0]
	fmul	v1.4s, v1.4s, v28.s[0]
	fmul	v2.4s, v2.4s, v28.s[0]
	fmul	v3.4s, v3.4s, v28.s[0]
	fmul	v4.4s, v4.4s, v28.s[0]
	fmul	v5.4s, v5.4s, v28.s[0]
	fmul	v6.4s, v6.4s, v28.s[0]
	fmul	v7.4s, v7.4s, v28.s[0]

	fcmpe	s29, #0.0
	beq		0f

	ldp		q24, q25, [x10, #0]
	add		x10, x10, x11
	ldp		q26, q27, [x10, #0]
	add		x10, x10, x11

	fmla	v0.4s, v24.4s, v29.s[0]
	fmla	v4.4s, v25.4s, v29.s[0]
	fmla	v1.4s, v26.4s, v29.s[0]
	fmla	v5.4s, v27.4s, v29.s[0]

	ldp		q24, q25, [x10, #0]
	add		x10, x10, x11
	ldp		q26, q27, [x10, #0]
	add		x10, x10, x11

	fmla	v2.4s, v24.4s, v29.s[0]
	fmla	v6.4s, v25.4s, v29.s[0]
	fmla	v3.4s, v26.4s, v29.s[0]
	fmla	v7.4s, v27.4s, v29.s[0]

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_8x4_lib)
#endif





// subroutine
//
// input arguments:
// x8   <- alpha
// x9   <- beta
// x10  <- C
// x11  <- ldc*sizeof(float)
// x12  <- km
// x13  <- kn
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_8X4_VS_LIB
#else
	.align	4
	FUN_START(inner_scale_ab_8x4_vs_lib)
#endif

	ld1		{v28.4s}, [x8]

	ld1		{v29.4s}, [x9]

	fmul	v0.4s, v0.4s, v28.s[0]
	fmul	v1.4s, v1.4s, v28.s[0]
	fmul	v2.4s, v2.4s, v28.s[0]
	fmul	v3.4s, v3.4s, v28.s[0]
	fmul	v4.4s, v4.4s, v28.s[0]
	fmul	v5.4s, v5.4s, v28.s[0]
	fmul	v6.4s, v6.4s, v28.s[0]
	fmul	v7.4s, v7.4s, v28.s[0]

	fcmpe	d29, #0.0
	beq		0f

	cmp		w12, #8
	blt		1f

	ldp		q24, q25, [x10, #0]
	add		x10, x10, x11
	fmla	v0.4s, v24.4s, v29.s[0]
	fmla	v4.4s, v25.4s, v29.s[0]

	cmp		w13, #1
	ble		0f

	ldp		q24, q25, [x10, #0]
	add		x10, x10, x11
	fmla	v1.4s, v24.4s, v29.s[0]
	fmla	v5.4s, v25.4s, v29.s[0]

	cmp		w13, #2
	ble		0f

	ldp		q24, q25, [x10, #0]
	add		x10, x10, x11
	fmla	v2.4s, v24.4s, v29.s[0]
	fmla	v6.4s, v25.4s, v29.s[0]

	cmp		w13, #3
	ble		0f

	ldp		q24, q25, [x10, #0]
	add		x10, x10, x11
	fmla	v3.4s, v24.4s, v29.s[0]
	fmla	v7.4s, v25.4s, v29.s[0]

	b 0f

1:
	cmp		w12, #7
	blt		2f

	ldr		q24, [x10, #0]
	ldr		d25, [x10, #16]
	ldr		s26, [x10, #24]
	ins		v25.s[2], v26.s[0]
	add		x10, x10, x11
	fmla	v0.4s, v24.4s, v29.s[0]
	fmla	v4.4s, v25.4s, v29.s[0]

	cmp		w13, #1
	ble		0f

	ldr		q24, [x10, #0]
	ldr		d25, [x10, #16]
	ldr		s26, [x10, #24]
	ins		v25.s[2], v26.s[0]
	add		x10, x10, x11
	fmla	v1.4s, v24.4s, v29.s[0]
	fmla	v5.4s, v25.4s, v29.s[0]

	cmp		w13, #2
	ble		0f

	ldr		q24, [x10, #0]
	ldr		d25, [x10, #16]
	ldr		s26, [x10, #24]
	ins		v25.s[2], v26.s[0]
	add		x10, x10, x11
	fmla	v2.4s, v24.4s, v29.s[0]
	fmla	v6.4s, v25.4s, v29.s[0]

	cmp		w13, #3
	ble		0f

	ldr		q24, [x10, #0]
	ldr		d25, [x10, #16]
	ldr		s26, [x10, #24]
	ins		v25.s[2], v26.s[0]
	add		x10, x10, x11
	fmla	v3.4s, v24.4s, v29.s[0]
	fmla	v7.4s, v25.4s, v29.s[0]

	b 0f

2:
	cmp		w12, #6
	blt		3f

	ldr		q24, [x10, #0]
	ldr		d25, [x10, #16]
	add		x10, x10, x11
	fmla	v0.4s, v24.4s, v29.s[0]
	fmla	v4.4s, v25.4s, v29.s[0]

	cmp		w13, #1
	ble		0f

	ldr		q24, [x10, #0]
	ldr		d25, [x10, #16]
	add		x10, x10, x11
	fmla	v1.4s, v24.4s, v29.s[0]
	fmla	v5.4s, v25.4s, v29.s[0]

	cmp		w13, #2
	ble		0f

	ldr		q24, [x10, #0]
	ldr		d25, [x10, #16]
	add		x10, x10, x11
	fmla	v2.4s, v24.4s, v29.s[0]
	fmla	v6.4s, v25.4s, v29.s[0]

	cmp		w13, #3
	ble		0f

	ldr		q24, [x10, #0]
	ldr		d25, [x10, #16]
	add		x10, x10, x11
	fmla	v3.4s, v24.4s, v29.s[0]
	fmla	v7.4s, v25.4s, v29.s[0]

	b 0f

3:
	cmp		w12, #5
	blt		0f

	ldr		q24, [x10, #0]
	ldr		s25, [x10, #16]
	add		x10, x10, x11
	fmla	v0.4s, v24.4s, v29.s[0]
	fmla	v4.4s, v25.4s, v29.s[0]

	cmp		w13, #1
	ble		0f

	ldr		q24, [x10, #0]
	ldr		s25, [x10, #16]
	add		x10, x10, x11
	fmla	v1.4s, v24.4s, v29.s[0]
	fmla	v5.4s, v25.4s, v29.s[0]

	cmp		w13, #2
	ble		0f

	ldr		q24, [x10, #0]
	ldr		s25, [x10, #16]
	add		x10, x10, x11
	fmla	v2.4s, v24.4s, v29.s[0]
	fmla	v6.4s, v25.4s, v29.s[0]

	cmp		w13, #3
	ble		0f

	ldr		q24, [x10, #0]
	ldr		s25, [x10, #16]
	add		x10, x10, x11
	fmla	v3.4s, v24.4s, v29.s[0]
	fmla	v7.4s, v25.4s, v29.s[0]

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_8x4_vs_lib)
#endif





// subroutine
//
// input arguments:
// x8   <- beta
// x9  <- C
// x11  <- ldc*sizeof(float)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M1B_8X4_LIB
#else
	.align	4
	FUN_START(inner_scale_m1b_8x4_lib)
#endif

	ld1		{v29.4s}, [x8]

	fneg	v0.4s, v0.4s
	fneg	v1.4s, v1.4s
	fneg	v2.4s, v2.4s
	fneg	v3.4s, v3.4s
	fneg	v4.4s, v4.4s
	fneg	v5.4s, v5.4s
	fneg	v6.4s, v6.4s
	fneg	v7.4s, v7.4s

	fcmpe	s29, #0.0
	beq		0f

	ldp		q24, q25, [x9, #0]
	add		x9, x9, x10
	ldp		q26, q27, [x9, #0]
	add		x9, x9, x10

	fmla	v0.4s, v24.4s, v29.s[0]
	fmla	v4.4s, v25.4s, v29.s[0]
	fmla	v1.4s, v26.4s, v29.s[0]
	fmla	v5.4s, v27.4s, v29.s[0]

	ldp		q24, q25, [x9, #0]
	add		x9, x9, x10
	ldp		q26, q27, [x9, #0]
	add		x9, x9, x10

	fmla	v2.4s, v24.4s, v29.s[0]
	fmla	v6.4s, v25.4s, v29.s[0]
	fmla	v3.4s, v26.4s, v29.s[0]
	fmla	v7.4s, v27.4s, v29.s[0]

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m1b_8x4_lib)
#endif





// subroutine
//
// input arguments:
// x8   <- beta
// x9  <- C
// x10  <- ldc*sizeof(float)
// x11  <- km
// x12  <- kn
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M1B_8X4_VS_LIB
#else
	.align	4
	FUN_START(inner_scale_m1b_8x4_vs_lib)
#endif

	ld1		{v29.4s}, [x8]

	fneg	v0.4s, v0.4s
	fneg	v1.4s, v1.4s
	fneg	v2.4s, v2.4s
	fneg	v3.4s, v3.4s
	fneg	v4.4s, v4.4s
	fneg	v5.4s, v5.4s
	fneg	v6.4s, v6.4s
	fneg	v7.4s, v7.4s

	fcmpe	d29, #0.0
	beq		0f

	cmp		w11, #8
	blt		1f

	ldp		q24, q25, [x9, #0]
	add		x9, x9, x10
	fmla	v0.4s, v24.4s, v29.s[0]
	fmla	v4.4s, v25.4s, v29.s[0]

	cmp		w12, #1
	ble		0f

	ldp		q24, q25, [x9, #0]
	add		x9, x9, x10
	fmla	v1.4s, v24.4s, v29.s[0]
	fmla	v5.4s, v25.4s, v29.s[0]

	cmp		w12, #2
	ble		0f

	ldp		q24, q25, [x9, #0]
	add		x9, x9, x10
	fmla	v2.4s, v24.4s, v29.s[0]
	fmla	v6.4s, v25.4s, v29.s[0]

	cmp		w12, #3
	ble		0f

	ldp		q24, q25, [x9, #0]
	add		x9, x9, x10
	fmla	v3.4s, v24.4s, v29.s[0]
	fmla	v7.4s, v25.4s, v29.s[0]

	b 0f

1:
	cmp		w11, #7
	blt		2f

	ldr		q24, [x9, #0]
	ldr		d25, [x9, #16]
	ldr		s26, [x9, #24]
	ins		v25.s[2], v26.s[0]
	add		x9, x9, x10
	fmla	v0.4s, v24.4s, v29.s[0]
	fmla	v4.4s, v25.4s, v29.s[0]

	cmp		w12, #1
	ble		0f

	ldr		q24, [x9, #0]
	ldr		d25, [x9, #16]
	ldr		s26, [x9, #24]
	ins		v25.s[2], v26.s[0]
	add		x9, x9, x10
	fmla	v1.4s, v24.4s, v29.s[0]
	fmla	v5.4s, v25.4s, v29.s[0]

	cmp		w12, #2
	ble		0f

	ldr		q24, [x9, #0]
	ldr		d25, [x9, #16]
	ldr		s26, [x9, #24]
	ins		v25.s[2], v26.s[0]
	add		x9, x9, x10
	fmla	v2.4s, v24.4s, v29.s[0]
	fmla	v6.4s, v25.4s, v29.s[0]

	cmp		w12, #3
	ble		0f

	ldr		q24, [x9, #0]
	ldr		d25, [x9, #16]
	ldr		s26, [x9, #24]
	ins		v25.s[2], v26.s[0]
	add		x9, x9, x10
	fmla	v3.4s, v24.4s, v29.s[0]
	fmla	v7.4s, v25.4s, v29.s[0]

	b 0f

2:
	cmp		w11, #6
	blt		3f

	ldr		q24, [x9, #0]
	ldr		d25, [x9, #16]
	add		x9, x9, x10
	fmla	v0.4s, v24.4s, v29.s[0]
	fmla	v4.4s, v25.4s, v29.s[0]

	cmp		w12, #1
	ble		0f

	ldr		q24, [x9, #0]
	ldr		d25, [x9, #16]
	add		x9, x9, x10
	fmla	v1.4s, v24.4s, v29.s[0]
	fmla	v5.4s, v25.4s, v29.s[0]

	cmp		w12, #2
	ble		0f

	ldr		q24, [x9, #0]
	ldr		d25, [x9, #16]
	add		x9, x9, x10
	fmla	v2.4s, v24.4s, v29.s[0]
	fmla	v6.4s, v25.4s, v29.s[0]

	cmp		w12, #3
	ble		0f

	ldr		q24, [x9, #0]
	ldr		d25, [x9, #16]
	add		x9, x9, x10
	fmla	v3.4s, v24.4s, v29.s[0]
	fmla	v7.4s, v25.4s, v29.s[0]

	b 0f

3:
	cmp		w11, #5
	blt		0f

	ldr		q24, [x9, #0]
	ldr		s25, [x9, #16]
	add		x9, x9, x10
	fmla	v0.4s, v24.4s, v29.s[0]
	fmla	v4.4s, v25.4s, v29.s[0]

	cmp		w12, #1
	ble		0f

	ldr		q24, [x9, #0]
	ldr		s25, [x9, #16]
	add		x9, x9, x10
	fmla	v1.4s, v24.4s, v29.s[0]
	fmla	v5.4s, v25.4s, v29.s[0]

	cmp		w12, #2
	ble		0f

	ldr		q24, [x9, #0]
	ldr		s25, [x9, #16]
	add		x9, x9, x10
	fmla	v2.4s, v24.4s, v29.s[0]
	fmla	v6.4s, v25.4s, v29.s[0]

	cmp		w12, #3
	ble		0f

	ldr		q24, [x9, #0]
	ldr		s25, [x9, #16]
	add		x9, x9, x10
	fmla	v3.4s, v24.4s, v29.s[0]
	fmla	v7.4s, v25.4s, v29.s[0]

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m1b_8x4_vs_lib)
#endif





// subroutine
//
// input arguments:
// x8  <- C
// x9  <- ldc*sizeof(float)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M11_8X4_LIB
#else
	.align	4
	FUN_START(inner_scale_m11_8x4_lib)
#endif

	ldp		q24, q25, [x8, #0]
	add		x8, x8, x9
	ldp		q26, q27, [x8, #0]
	add		x8, x8, x9

	fsub	v0.4s, v24.4s, v0.4s
	fsub	v4.4s, v25.4s, v4.4s
	fsub	v1.4s, v26.4s, v1.4s
	fsub	v5.4s, v27.4s, v5.4s

	ldp		q24, q25, [x8, #0]
	add		x8, x8, x9
	ldp		q26, q27, [x8, #0]
	add		x8, x8, x9

	fsub	v2.4s, v24.4s, v2.4s
	fsub	v6.4s, v25.4s, v6.4s
	fsub	v3.4s, v26.4s, v3.4s
	fsub	v7.4s, v27.4s, v7.4s

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m11_8x4_lib)
#endif





// subroutine
//
// input arguments:
// x8   <- alpha
// x9   <- beta
// x10  <- C
// x11  <- ldc*sizeof(float)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_4X8_LIB
#else
	.align	4
	FUN_START(inner_scale_ab_4x8_lib)
#endif

	ld1		{v28.4s}, [x8]

	ld1		{v29.4s}, [x9]

	fmul	v0.4s, v0.4s, v28.s[0]
	fmul	v1.4s, v1.4s, v28.s[0]
	fmul	v2.4s, v2.4s, v28.s[0]
	fmul	v3.4s, v3.4s, v28.s[0]
	fmul	v4.4s, v4.4s, v28.s[0]
	fmul	v5.4s, v5.4s, v28.s[0]
	fmul	v6.4s, v6.4s, v28.s[0]
	fmul	v7.4s, v7.4s, v28.s[0]

	fcmpe	s29, #0.0
	beq		0f

	ldr		q24, [x10, #0]
	add		x10, x10, x11
	ldr		q25, [x10, #0]
	add		x10, x10, x11
	ldr		q26, [x10, #0]
	add		x10, x10, x11
	ldr		q27, [x10, #0]
	add		x10, x10, x11
	fmla	v0.4s, v24.4s, v29.s[0]
	fmla	v1.4s, v25.4s, v29.s[0]
	fmla	v2.4s, v26.4s, v29.s[0]
	fmla	v3.4s, v27.4s, v29.s[0]

	ldr		q24, [x10, #0]
	add		x10, x10, x11
	ldr		q25, [x10, #0]
	add		x10, x10, x11
	ldr		q26, [x10, #0]
	add		x10, x10, x11
	ldr		q27, [x10, #0]
	add		x10, x10, x11
	fmla	v4.4s, v24.4s, v29.s[0]
	fmla	v5.4s, v25.4s, v29.s[0]
	fmla	v6.4s, v26.4s, v29.s[0]
	fmla	v7.4s, v27.4s, v29.s[0]

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_4x8_lib)
#endif





// subroutine
//
// input arguments:
// x8   <- alpha
// x9   <- beta
// x10  <- C
// x11  <- ldc*sizeof(float)
// x12  <- km
// x13  <- kn
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_4X8_VS_LIB
#else
	.align	4
	FUN_START(inner_scale_ab_4x8_vs_lib)
#endif

	ld1		{v28.4s}, [x8]

	ld1		{v29.4s}, [x9]

	fmul	v0.4s, v0.4s, v28.s[0]
	fmul	v1.4s, v1.4s, v28.s[0]
	fmul	v2.4s, v2.4s, v28.s[0]
	fmul	v3.4s, v3.4s, v28.s[0]
	fmul	v4.4s, v4.4s, v28.s[0]
	fmul	v5.4s, v5.4s, v28.s[0]
	fmul	v6.4s, v6.4s, v28.s[0]
	fmul	v7.4s, v7.4s, v28.s[0]

	fcmpe	d29, #0.0
	beq		0f

	cmp		w12, #4
	blt		1f

	// 1
	ldr		q24, [x10, #0]
	add		x10, x10, x11
	fmla	v0.4s, v24.4s, v29.s[0]
	// 2
	ldr		q24, [x10, #0]
	add		x10, x10, x11
	fmla	v1.4s, v24.4s, v29.s[0]
	// 3
	ldr		q24, [x10, #0]
	add		x10, x10, x11
	fmla	v2.4s, v24.4s, v29.s[0]
	// 4
	ldr		q24, [x10, #0]
	add		x10, x10, x11
	fmla	v3.4s, v24.4s, v29.s[0]
	// 5
	ldr		q24, [x10, #0]
	add		x10, x10, x11
	fmla	v4.4s, v24.4s, v29.s[0]
	cmp		w13, #5
	ble		0f
	// 6
	ldr		q24, [x10, #0]
	add		x10, x10, x11
	fmla	v5.4s, v24.4s, v29.s[0]
	cmp		w13, #6
	ble		0f
	// 7
	ldr		q24, [x10, #0]
	add		x10, x10, x11
	fmla	v6.4s, v24.4s, v29.s[0]
	cmp		w13, #7
	ble		0f
	// 8
	ldr		q24, [x10, #0]
	add		x10, x10, x11
	fmla	v7.4s, v24.4s, v29.s[0]

	b 0f

1:
	cmp		w12, #3
	blt		2f

	// 1
	ldr		d24, [x10, #0]
	ldr		s25, [x10, #8]
	ins		v24.s[2], v25.s[0]
	add		x10, x10, x11
	fmla	v0.4s, v24.4s, v29.s[0]
	// 2
	ldr		d24, [x10, #0]
	ldr		s25, [x10, #8]
	ins		v24.s[2], v25.s[0]
	add		x10, x10, x11
	fmla	v1.4s, v24.4s, v29.s[0]
	// 3
	ldr		d24, [x10, #0]
	ldr		s25, [x10, #8]
	ins		v24.s[2], v25.s[0]
	add		x10, x10, x11
	fmla	v2.4s, v24.4s, v29.s[0]
	// 4
	ldr		d24, [x10, #0]
	ldr		s25, [x10, #8]
	ins		v24.s[2], v25.s[0]
	add		x10, x10, x11
	fmla	v3.4s, v24.4s, v29.s[0]
	// 5
	ldr		d24, [x10, #0]
	ldr		s25, [x10, #8]
	ins		v24.s[2], v25.s[0]
	add		x10, x10, x11
	fmla	v4.4s, v24.4s, v29.s[0]
	cmp		w13, #5
	ble		0f
	// 6
	ldr		d24, [x10, #0]
	ldr		s25, [x10, #8]
	ins		v24.s[2], v25.s[0]
	add		x10, x10, x11
	fmla	v5.4s, v24.4s, v29.s[0]
	cmp		w13, #6
	ble		0f
	// 7
	ldr		d24, [x10, #0]
	ldr		s25, [x10, #8]
	ins		v24.s[2], v25.s[0]
	add		x10, x10, x11
	fmla	v6.4s, v24.4s, v29.s[0]
	cmp		w13, #7
	ble		0f
	// 8
	ldr		d24, [x10, #0]
	ldr		s25, [x10, #8]
	ins		v24.s[2], v25.s[0]
	add		x10, x10, x11
	fmla	v7.4s, v24.4s, v29.s[0]

	b 0f

2:
	cmp		w12, #2
	blt		3f

	// 1
	ldr		d24, [x10, #0]
	add		x10, x10, x11
	fmla	v0.4s, v24.4s, v29.s[0]
	// 2
	ldr		d24, [x10, #0]
	add		x10, x10, x11
	fmla	v1.4s, v24.4s, v29.s[0]
	// 3
	ldr		d24, [x10, #0]
	add		x10, x10, x11
	fmla	v2.4s, v24.4s, v29.s[0]
	// 4
	ldr		d24, [x10, #0]
	add		x10, x10, x11
	fmla	v3.4s, v24.4s, v29.s[0]
	// 5
	ldr		d24, [x10, #0]
	add		x10, x10, x11
	fmla	v4.4s, v24.4s, v29.s[0]
	cmp		w13, #5
	ble		0f
	// 6
	ldr		d24, [x10, #0]
	add		x10, x10, x11
	fmla	v5.4s, v24.4s, v29.s[0]
	cmp		w13, #6
	ble		0f
	// 7
	ldr		d24, [x10, #0]
	add		x10, x10, x11
	fmla	v6.4s, v24.4s, v29.s[0]
	cmp		w13, #7
	ble		0f
	// 8
	ldr		d24, [x10, #0]
	add		x10, x10, x11
	fmla	v7.4s, v24.4s, v29.s[0]

	b 0f

3:
	cmp		w12, #1
	blt		0f

	// 1
	ldr		s24, [x10, #0]
	add		x10, x10, x11
	fmla	v0.4s, v24.4s, v29.s[0]
	// 2
	ldr		s24, [x10, #0]
	add		x10, x10, x11
	fmla	v1.4s, v24.4s, v29.s[0]
	// 3
	ldr		s24, [x10, #0]
	add		x10, x10, x11
	fmla	v2.4s, v24.4s, v29.s[0]
	// 4
	ldr		s24, [x10, #0]
	add		x10, x10, x11
	fmla	v3.4s, v24.4s, v29.s[0]
	// 5
	ldr		s24, [x10, #0]
	add		x10, x10, x11
	fmla	v4.4s, v24.4s, v29.s[0]
	cmp		w13, #5
	ble		0f
	// 6
	ldr		s24, [x10, #0]
	add		x10, x10, x11
	fmla	v5.4s, v24.4s, v29.s[0]
	cmp		w13, #6
	ble		0f
	// 7
	ldr		s24, [x10, #0]
	add		x10, x10, x11
	fmla	v6.4s, v24.4s, v29.s[0]
	cmp		w13, #7
	ble		0f
	// 8
	ldr		s24, [x10, #0]
	add		x10, x10, x11
	fmla	v7.4s, v24.4s, v29.s[0]

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_4x8_vs_lib)
#endif





// subroutine
//
// input arguments:
// x8   <- D
// x9   <- ldd*sizeof(float)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_8X4_LIB
#else
	.align 4
	FUN_START(inner_store_8x4_lib)
#endif

	stp		q0, q4, [x8, #0]
	add		x8, x8, x9
	stp		q1, q5, [x8, #0]
	add		x8, x8, x9
	stp		q2, q6, [x8, #0]
	add		x8, x8, x9
	stp		q3, q7, [x8, #0]

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_8x4_lib)
#endif





// subroutine
//
// input arguments:
// x8   <- D
// x9   <- ldd*sizeof(float)
// x10  <- km
// x11  <- kn
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_8X4_VS_LIB
#else
	.align 4
	FUN_START(inner_store_8x4_vs_lib)
#endif

	cmp		w10, #8
	bge		1f

	mov		x12, x8

	ldr		q16, [x12, #16]
	add		x12, x12, x9
	ldr		q17, [x12, #16]
	add		x12, x12, x9
	ldr		q18, [x12, #16]
	add		x12, x12, x9
	ldr		q19, [x12, #16]

	// 4th row
	ins		v4.s[3], v16.s[3]
	ins		v5.s[3], v17.s[3]
	ins		v6.s[3], v18.s[3]
	ins		v7.s[3], v19.s[3]
	cmp		w10, #7
	bge		1f
	// 3th row
	ins		v4.s[2], v16.s[2]
	ins		v5.s[2], v17.s[2]
	ins		v6.s[2], v18.s[2]
	ins		v7.s[2], v19.s[2]
	cmp		w10, #6
	bge		1f
	// 2nd row
	ins		v4.s[1], v16.s[1]
	ins		v5.s[1], v17.s[1]
	ins		v6.s[1], v18.s[1]
	ins		v7.s[1], v19.s[1]
	cmp		w10, #5
	bge		1f
	// 1st row
	ins		v4.s[0], v16.s[0]
	ins		v5.s[0], v17.s[0]
	ins		v6.s[0], v18.s[0]
	ins		v7.s[0], v19.s[0]

1:
	// 1st col
	stp		q0, q4, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	stp		q1, q5, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
	stp		q2, q6, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #3
	beq		0f
	// 4th col
	stp		q3, q7, [x8, #0]

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_8x4_vs_lib)
#endif





// subroutine
//
// input arguments:
// x8   <- D
// x9   <- ldd*sizeof(float)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_8X4_LIB
#else
	.align 4
	FUN_START(inner_store_l_8x4_lib)
#endif

	mov		x12, x8

	add		x12, x12, x9
	ldr		q16, [x12, #0]
	add		x12, x12, x9
	ldr		q17, [x12, #0]
	add		x12, x12, x9
	ldr		q18, [x12, #0]

	ins		v1.s[0], v16.s[0]
	ins		v2.d[0], v17.d[0]
	ins		v3.d[0], v18.d[0]
	ins		v3.s[2], v18.s[2]

	stp		q0, q4, [x8, #0]
	add		x8, x8, x9
	stp		q1, q5, [x8, #0]
	add		x8, x8, x9
	stp		q2, q6, [x8, #0]
	add		x8, x8, x9
	stp		q3, q7, [x8, #0]

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_8x4_lib)
#endif





// subroutine
//
// input arguments:
// x8   <- D
// x9   <- ldd*sizeof(float)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X8_LIB
#else
	.align 4
	FUN_START(inner_store_4x8_lib)
#endif

	str		q0, [x8, #0]
	add		x8, x8, x9

	str		q1, [x8, #0]
	add		x8, x8, x9

	str		q2, [x8, #0]
	add		x8, x8, x9

	str		q3, [x8, #0]
	add		x8, x8, x9

	str		q4, [x8, #0]
	add		x8, x8, x9

	str		q5, [x8, #0]
	add		x8, x8, x9

	str		q6, [x8, #0]
	add		x8, x8, x9

	str		q7, [x8, #0]
//	add		x8, x8, x9

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4x8_lib)
#endif





// subroutine
//
// input arguments:
// x8   <- D
// x9   <- ldd*sizeof(float)
// x10  <- km
// x11  <- kn
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X8_VS_LIB
#else
	.align 4
	FUN_START(inner_store_4x8_vs_lib)
#endif

	cmp		w10, #4
	bge		1f

	mov		x12, x8

	ldr		q16, [x12, #0]
	add		x12, x12, x9
	ldr		q17, [x12, #0]
	add		x12, x12, x9
	ldr		q18, [x12, #0]
	add		x12, x12, x9
	ldr		q19, [x12, #0]
	add		x12, x12, x9
	ldr		q20, [x12, #0]
	add		x12, x12, x9
	ldr		q21, [x12, #0]
	add		x12, x12, x9
	ldr		q22, [x12, #0]
	add		x12, x12, x9
	ldr		q23, [x12, #0]
//	add		x12, x12, x9

	// 4th row
	ins		v0.s[3], v16.s[3]
	ins		v1.s[3], v17.s[3]
	ins		v2.s[3], v18.s[3]
	ins		v3.s[3], v19.s[3]
	ins		v4.s[3], v20.s[3]
	ins		v5.s[3], v21.s[3]
	ins		v6.s[3], v22.s[3]
	ins		v7.s[3], v23.s[3]
	cmp		w10, #3
	bge		1f
	// 3th row
	ins		v0.s[2], v16.s[2]
	ins		v1.s[2], v17.s[2]
	ins		v2.s[2], v18.s[2]
	ins		v3.s[2], v19.s[2]
	ins		v4.s[2], v20.s[2]
	ins		v5.s[2], v21.s[2]
	ins		v6.s[2], v22.s[2]
	ins		v7.s[2], v23.s[2]
	cmp		w10, #2
	bge		1f
	// 2nd row
	ins		v0.s[1], v16.s[1]
	ins		v1.s[1], v17.s[1]
	ins		v2.s[1], v18.s[1]
	ins		v3.s[1], v19.s[1]
	ins		v4.s[1], v20.s[1]
	ins		v5.s[1], v21.s[1]
	ins		v6.s[1], v22.s[1]
	ins		v7.s[1], v23.s[1]
	cmp		w10, #1
	bge		1f
	// 1st row
	ins		v0.s[0], v16.s[0]
	ins		v1.s[0], v17.s[0]
	ins		v2.s[0], v18.s[0]
	ins		v3.s[0], v19.s[0]
	ins		v4.s[0], v20.s[0]
	ins		v5.s[0], v21.s[0]
	ins		v6.s[0], v22.s[0]
	ins		v7.s[0], v23.s[0]

1:
	// 1st col
	str		q0, [x8, #0]
	add		x8, x8, x9
	// 2nd col
	str		q1, [x8, #0]
	add		x8, x8, x9
	// 3rd col
	str		q2, [x8, #0]
	add		x8, x8, x9
	// 4th col
	str		q3, [x8, #0]
	add		x8, x8, x9
	// 5th col
	str		q4, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #6
	blt		0f
	// 6th col
	str		q5, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #7
	blt		0f
	// 7th col
	str		q6, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #7
	beq		0f
	// 8th col
	str		q7, [x8, #0]
//	add		x8, x8, x9

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4x8_vs_lib)
#endif





// subroutine
//
// input arguments:
// x8   <- D
// x9   <- ldd*sizeof(float)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_PREFETCH_8X4_LIB
#else
	.align 4
	FUN_START(inner_prefetch_8x4_lib)
#endif

	prfm	PLDL1KEEP, [x8, #0]
//	prfm	PLDL1KEEP, [x8, #28]
	prfm	PLDL1KEEP, [x8, #24]
	add		x8, x8, x9

	prfm	PLDL1KEEP, [x8, #0]
//	prfm	PLDL1KEEP, [x8, #28]
	prfm	PLDL1KEEP, [x8, #24]
	add		x8, x8, x9

	prfm	PLDL1KEEP, [x8, #0]
//	prfm	PLDL1KEEP, [x8, #28]
	prfm	PLDL1KEEP, [x8, #24]
	add		x8, x8, x9

	prfm	PLDL1KEEP, [x8, #0]
//	prfm	PLDL1KEEP, [x8, #28]
	prfm	PLDL1KEEP, [x8, #24]
//	add		x8, x8, x9

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_prefetch_8x4_lib)
#endif





// subroutine
//
// input arguments:
// x8   <- D
// x9   <- ldd*sizeof(float)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_PREFETCH_4X8_LIB
#else
	.align 4
	FUN_START(inner_prefetch_4x8_lib)
#endif

	prfm	PLDL1KEEP, [x8, #0]
//	prfm	PLDL1KEEP, [x8, #12]
	prfm	PLDL1KEEP, [x8, #8]
	add		x8, x8, x9

	prfm	PLDL1KEEP, [x8, #0]
//	prfm	PLDL1KEEP, [x8, #12]
	prfm	PLDL1KEEP, [x8, #8]
	add		x8, x8, x9

	prfm	PLDL1KEEP, [x8, #0]
//	prfm	PLDL1KEEP, [x8, #12]
	prfm	PLDL1KEEP, [x8, #8]
	add		x8, x8, x9

	prfm	PLDL1KEEP, [x8, #0]
//	prfm	PLDL1KEEP, [x8, #12]
	prfm	PLDL1KEEP, [x8, #8]
	add		x8, x8, x9

	prfm	PLDL1KEEP, [x8, #0]
//	prfm	PLDL1KEEP, [x8, #12]
	prfm	PLDL1KEEP, [x8, #8]
	add		x8, x8, x9

	prfm	PLDL1KEEP, [x8, #0]
//	prfm	PLDL1KEEP, [x8, #12]
	prfm	PLDL1KEEP, [x8, #8]
	add		x8, x8, x9

	prfm	PLDL1KEEP, [x8, #0]
//	prfm	PLDL1KEEP, [x8, #12]
	prfm	PLDL1KEEP, [x8, #8]
	add		x8, x8, x9

	prfm	PLDL1KEEP, [x8, #0]
//	prfm	PLDL1KEEP, [x8, #12]
	prfm	PLDL1KEEP, [x8, #8]
//	add		x8, x8, x9

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_prefetch_4x8_lib)
#endif





//                                 w0        x1             x2        w3        x4       x5           x6        w7       sp+0      sp+8
// void kernel_sgemm_nt_8x4_lib44cc(int kmax, float *alpha, float *A, int sda, float *B, float *beta, float *C, int ldc, float *D, int ldd)

	.align	4
	GLOB(kernel_sgemm_nt_8x4_lib44cc)
	FUN_START(kernel_sgemm_nt_8x4_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #4 // 16*sda
	mov		x11, x4 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 0)] // ldd
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #2 // 4*ldd

#if MACRO_LEVEL>=1
	INNER_PREFETCH_8X4_LIB
#else
	CALL(inner_prefetch_8x4_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // ldc
	lsl		w11, w11, #2 // 4*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB
#else
	CALL(inner_scale_ab_8x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // ldd
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #2 // 4*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB
#else
	CALL(inner_store_8x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_sgemm_nt_8x4_lib44cc)





//                                    w0        x1             x2        w3        x4       x5           x6        w7       sp+0      sp+8     sp+16   sp+24
// void kernel_sgemm_nt_8x4_vs_lib44cc(int kmax, float *alpha, float *A, int sda, float *B, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_sgemm_nt_8x4_vs_lib44cc)
	FUN_START(kernel_sgemm_nt_8x4_vs_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #4 // 16*sda
	mov		x11, x4 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4)
#endif



	// prefetch
//	ldr		x8, [sp, #(STACKSIZE + 0)] // ldd
//	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
//	lsl		w9, w9, #2 // 4*ldd

#if MACRO_LEVEL>=1
//	INNER_PREFETCH_8X4_LIB
#else
//	CALL(inner_prefetch_8x4_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // ldc
	lsl		w11, w11, #2 // 4*ldc
	ldr		w12, [sp, #(STACKSIZE + 16)] // m1
	ldr		w13, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_VS_LIB
#else
	CALL(inner_scale_ab_8x4_vs_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // ldd
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #2 // 4*ldd
	ldr		w10, [sp, #(STACKSIZE + 16)] // m1
	ldr		w11, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_VS_LIB
#else
	CALL(inner_store_8x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_sgemm_nt_8x4_vs_lib44cc)





//                                  w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16
// void kernel_sgemm_nt_8x4_lib4ccc(int kmax, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_sgemm_nt_8x4_lib4ccc)
	FUN_START(kernel_sgemm_nt_8x4_lib4ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #4 // 16*sda
	mov		x11, x4 // B
	mov		w12, w5 // ldb
	lsl		w12, w12, #2 // 4*ldb

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4c)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #2 // 4*sdd

#if MACRO_LEVEL>=1
	INNER_PREFETCH_8X4_LIB
#else
	CALL(inner_prefetch_8x4_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x6 // beta
	mov		x10, x7 // C
	ldr		w11, [sp, #(STACKSIZE + 0)] // ldc
	lsl		w11, w11, #2 // 4*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB
#else
	CALL(inner_scale_ab_8x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #2 // 4*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB
#else
	CALL(inner_store_8x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_sgemm_nt_8x4_lib4ccc)





//                                     w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16    sp+24   sp+32
// void kernel_sgemm_nt_8x4_vs_lib4ccc(int kmax, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_sgemm_nt_8x4_vs_lib4ccc)
	FUN_START(kernel_sgemm_nt_8x4_vs_lib4ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #4 // 16*sda
	mov		x11, x4 // B
	mov		w12, w5 // ldb
	lsl		w12, w12, #2 // 4*ldb

	ldr		w13, [sp, #(STACKSIZE + 32)] // n1
	cmp		w13, #1
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X1_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_8x1_lib4c)
#endif
	
	b		103f

100:

	ldr		w13, [sp, #(STACKSIZE + 32)] // n1
	cmp		w13, #2
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X2_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_8x2_lib4c)
#endif
	
	b		103f

101:

	ldr		w13, [sp, #(STACKSIZE + 32)] // n1
	cmp		w13, #3
	bgt		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X3_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_8x3_lib4c)
#endif
	
	b		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4c)
#endif

103:



	// prefetch
//	ldr		x8, [sp, #(STACKSIZE + 8)] // D
//	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
//	lsl		w9, w9, #2 // 4*sdd

#if MACRO_LEVEL>=1
//	INNER_PREFETCH_8X4_LIB
#else
//	CALL(inner_prefetch_8x4_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x6 // beta
	mov		x10, x7 // C
	ldr		w11, [sp, #(STACKSIZE + 0)] // ldc
	lsl		w11, w11, #2 // 4*ldc
	ldr		w12, [sp, #(STACKSIZE + 24)] // m1
	ldr		w13, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_VS_LIB
#else
	CALL(inner_scale_ab_8x4_vs_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #2 // 4*ldd
	ldr		w10, [sp, #(STACKSIZE + 24)] // m1
	ldr		w11, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_VS_LIB
#else
	CALL(inner_store_8x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_sgemm_nt_8x4_vs_lib4ccc)





//                                 w0        x1             x2         x3         x4        w5       x6            x7         sp+0     sp+8       sp+16
// void kernel_sgemm_nt_4x8_libc4cc(int kmax, double *alpha, double *A, int lda, double *B, int sdb, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_sgemm_nt_4x8_libc4cc)
	FUN_START(kernel_sgemm_nt_4x8_libc4cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x4 // B
	mov		w10, w5 // sdb
	lsl		w10, w10, #4 // 16*sdb
	mov		x11, x2 // A
	mov		w12, w3 // lda
	lsl		w12, w12, #2 // 4*lda

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4c)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #2 // 4*ldd

#if MACRO_LEVEL>=1
	INNER_PREFETCH_4X8_LIB
#else
	CALL(inner_prefetch_4x8_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x6 // beta
	mov		x10, x7 // C
	ldr		w11, [sp, #(STACKSIZE + 0)] // ldc
	lsl		w11, w11, #2 // 4*ldc

#if MACRO_LEVEL>=1
	INNER_TRAN_8X4_LIB
#else
	CALL(inner_tran_8x4_lib)
#endif


#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X8_LIB
#else
	CALL(inner_scale_ab_4x8_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #2 // 4*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_LIB
#else
	CALL(inner_store_4x8_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_sgemm_nt_4x8_libc4cc)





//                                     w0        x1             x2         x3         x4       w5       x6            x7         sp+0     sp+8       sp+16    sp+24   sp+32
// void kernel_sgemm_nt_4x8_vs_libc4cc(int kmax, double *alpha, double *A, int lda, double *B, int sdb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_sgemm_nt_4x8_vs_libc4cc)
	FUN_START(kernel_sgemm_nt_4x8_vs_libc4cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x4 // B
	mov		w10, w5 // sdb
	lsl		w10, w10, #4 // 16*sdb
	mov		x11, x2 // A
	mov		w12, w3 // lda
	lsl		w12, w12, #2 // 4*lda

	ldr		w13, [sp, #(STACKSIZE + 24)] // m1
	cmp		w13, #1
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X1_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_8x1_lib4c)
#endif
	
	b		103f

100:

	ldr		w13, [sp, #(STACKSIZE + 24)] // m1
	cmp		w13, #2
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X2_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_8x2_lib4c)
#endif
	
	b		103f

101:

	ldr		w13, [sp, #(STACKSIZE + 24)] // m1
	cmp		w13, #3
	bgt		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X3_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_8x3_lib4c)
#endif
	
	b		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4c)
#endif

103:



	// prefetch
//	ldr		x8, [sp, #(STACKSIZE + 8)] // D
//	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
//	lsl		w9, w9, #2 // 4*ldd

#if MACRO_LEVEL>=1
//	INNER_PREFETCH_4X8_LIB
#else
//	CALL(inner_prefetch_4x8_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x6 // beta
	mov		x10, x7 // C
	ldr		w11, [sp, #(STACKSIZE + 0)] // ldc
	lsl		w11, w11, #2 // 4*ldc
	ldr		w12, [sp, #(STACKSIZE + 24)] // m1
	ldr		w13, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_TRAN_8X4_LIB
#else
	CALL(inner_tran_8x4_lib)
#endif


#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X8_VS_LIB
#else
	CALL(inner_scale_ab_4x8_vs_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #2 // 4*ldd
	ldr		w10, [sp, #(STACKSIZE + 24)] // m1
	ldr		w11, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_VS_LIB
#else
	CALL(inner_store_4x8_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_sgemm_nt_4x8_vs_libc4cc)





//                                  w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16
// void kernel_sgemm_nn_8x4_lib4ccc(int kmax, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_sgemm_nn_8x4_lib4ccc)
	FUN_START(kernel_sgemm_nn_8x4_lib4ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #4 // 16*sda
	mov		x11, x4 // B
	mov		w12, w5 // ldb
	lsl		w12, w12, #2 // 4*ldb

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_8x4_lib4c)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #2 // 4*sdd

#if MACRO_LEVEL>=1
	INNER_PREFETCH_8X4_LIB
#else
	CALL(inner_prefetch_8x4_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x6 // beta
	mov		x10, x7 // C
	ldr		w11, [sp, #(STACKSIZE + 0)] // ldc
	lsl		w11, w11, #2 // 4*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_LIB
#else
	CALL(inner_scale_ab_8x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #2 // 4*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB
#else
	CALL(inner_store_8x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_sgemm_nn_8x4_lib4ccc)





//                                     w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16    sp+24   sp+32
// void kernel_sgemm_nn_8x4_vs_lib4ccc(int kmax, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_sgemm_nn_8x4_vs_lib4ccc)
	FUN_START(kernel_sgemm_nn_8x4_vs_lib4ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #4 // 16*sda
	mov		x11, x4 // B
	mov		w12, w5 // ldb
	lsl		w12, w12, #2 // 4*ldb

	ldr		w13, [sp, #(STACKSIZE + 32)] // n1
	cmp		w13, #1
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X1_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_8x1_lib4c)
#endif
	
	b		103f

100:

	ldr		w13, [sp, #(STACKSIZE + 32)] // n1
	cmp		w13, #2
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X2_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_8x2_lib4c)
#endif
	
	b		103f

101:

	ldr		w13, [sp, #(STACKSIZE + 32)] // n1
	cmp		w13, #3
	bgt		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X3_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_8x3_lib4c)
#endif
	
	b		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_8x4_lib4c)
#endif

103:



	// prefetch
//	ldr		x8, [sp, #(STACKSIZE + 8)] // D
//	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
//	lsl		w9, w9, #2 // 4*sdd

#if MACRO_LEVEL>=1
//	INNER_PREFETCH_8X4_LIB
#else
//	CALL(inner_prefetch_8x4_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x6 // beta
	mov		x10, x7 // C
	ldr		w11, [sp, #(STACKSIZE + 0)] // ldc
	lsl		w11, w11, #2 // 4*ldc
	ldr		w12, [sp, #(STACKSIZE + 24)] // m1
	ldr		w13, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X4_VS_LIB
#else
	CALL(inner_scale_ab_8x4_vs_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #2 // 4*ldd
	ldr		w10, [sp, #(STACKSIZE + 24)] // m1
	ldr		w11, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_VS_LIB
#else
	CALL(inner_store_8x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_sgemm_nn_8x4_vs_lib4ccc)





//                                 w0        x1             x2         x3         x4        w5       x6            x7         sp+0     sp+8       sp+16
// void kernel_sgemm_tt_4x8_libc4cc(int kmax, double *alpha, double *A, int lda, double *B, int sdb, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_sgemm_tt_4x8_libc4cc)
	FUN_START(kernel_sgemm_tt_4x8_libc4cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x4 // B
	mov		w10, w5 // sdb
	lsl		w10, w10, #4 // 16*sdb
	mov		x11, x2 // A
	mov		w12, w3 // lda
	lsl		w12, w12, #2 // 4*lda

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_8x4_lib4c)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #2 // 4*ldd

#if MACRO_LEVEL>=1
	INNER_PREFETCH_4X8_LIB
#else
	CALL(inner_prefetch_4x8_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x6 // beta
	mov		x10, x7 // C
	ldr		w11, [sp, #(STACKSIZE + 0)] // ldc
	lsl		w11, w11, #2 // 4*ldc

#if MACRO_LEVEL>=1
	INNER_TRAN_8X4_LIB
#else
	CALL(inner_tran_8x4_lib)
#endif


#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X8_LIB
#else
	CALL(inner_scale_ab_4x8_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #2 // 4*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_LIB
#else
	CALL(inner_store_4x8_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_sgemm_tt_4x8_libc4cc)





//                                     w0        x1             x2         x3         x4       w5       x6            x7         sp+0     sp+8       sp+16    sp+24   sp+32
// void kernel_sgemm_tt_4x8_vs_libc4cc(int kmax, double *alpha, double *A, int lda, double *B, int sdb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_sgemm_tt_4x8_vs_libc4cc)
	FUN_START(kernel_sgemm_tt_4x8_vs_libc4cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x4 // B
	mov		w10, w5 // sdb
	lsl		w10, w10, #4 // 16*sdb
	mov		x11, x2 // A
	mov		w12, w3 // lda
	lsl		w12, w12, #2 // 4*lda

	ldr		w13, [sp, #(STACKSIZE + 24)] // m1
	cmp		w13, #1
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X1_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_8x1_lib4c)
#endif
	
	b		103f

100:

	ldr		w13, [sp, #(STACKSIZE + 24)] // m1
	cmp		w13, #2
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X2_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_8x2_lib4c)
#endif
	
	b		103f

101:

	ldr		w13, [sp, #(STACKSIZE + 24)] // m1
	cmp		w13, #3
	bgt		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X3_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_8x3_lib4c)
#endif
	
	b		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_8X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_8x4_lib4c)
#endif

103:



	// prefetch
//	ldr		x8, [sp, #(STACKSIZE + 8)] // D
//	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
//	lsl		w9, w9, #2 // 4*ldd

#if MACRO_LEVEL>=1
//	INNER_PREFETCH_4X8_LIB
#else
//	CALL(inner_prefetch_4x8_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x6 // beta
	mov		x10, x7 // C
	ldr		w11, [sp, #(STACKSIZE + 0)] // ldc
	lsl		w11, w11, #2 // 4*ldc
	ldr		w12, [sp, #(STACKSIZE + 24)] // m1
	ldr		w13, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_TRAN_8X4_LIB
#else
	CALL(inner_tran_8x4_lib)
#endif


#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X8_VS_LIB
#else
	CALL(inner_scale_ab_4x8_vs_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #2 // 4*ldd
	ldr		w10, [sp, #(STACKSIZE + 24)] // m1
	ldr		w11, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X8_VS_LIB
#else
	CALL(inner_store_4x8_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_sgemm_tt_4x8_vs_libc4cc)





//                                          w0        x1        x2        x3           x4        w5       x6        w7       sp+0      sp+8     sp+16    sp+24
// void kernel_strsm_nt_rl_inv_8x4_lib44ccc(int kmax, float *A, int sda, float *B, float *beta, float *C, int ldc, float *D, int ldd, float *E, int lde, float *inv_diag_E)

	.align	4
	GLOB(kernel_strsm_nt_rl_inv_8x4_lib44ccc)
	FUN_START(kernel_strsm_nt_rl_inv_8x4_lib44ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #4 // 16*sda
	mov		x11, x3 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x4 // beta
	mov		x9, x5 // C
	mov		w10, w6 // ldc
	lsl		w10, w10, #2 // 4*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_8X4_LIB
#else
	CALL(inner_scale_m1b_8x4_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 8)] // E
	ldr		w9, [sp, #(STACKSIZE + 16)] // sde
	lsl		w9, w9, #2 // 4*ldc
	ldr		x10, [sp, #(STACKSIZE + 24)] // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_INV_8X4_LIB
#else
	CALL(inner_edge_trsm_rlt_inv_8x4_lib)
#endif



	// store l
	mov		x8, x7 // D
	ldr		w9, [sp, #(STACKSIZE + 0)] // ldd
	lsl		w9, w9, #2 // 4*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_LIB
#else
	CALL(inner_store_8x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_strsm_nt_rl_inv_8x4_lib44ccc)





//                                             w0        x1        x2        x3           x4        w5       x6        w7       sp+0      sp+8     sp+16    sp+24              sp+32   sp+40
// void kernel_strsm_nt_rl_inv_8x4_vs_lib44ccc(int kmax, float *A, int sda, float *B, float *beta, float *C, int ldc, float *D, int ldd, float *E, int lde, float *inv_diag_E, int m1, int n1)

	.align	4
	GLOB(kernel_strsm_nt_rl_inv_8x4_vs_lib44ccc)
	FUN_START(kernel_strsm_nt_rl_inv_8x4_vs_lib44ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #4 // 16*sda
	mov		x11, x3 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x4 // beta
	mov		x9, x5 // C
	mov		w10, w6 // ldc
	lsl		w10, w10, #2 // 4*ldc
	ldr		w11, [sp, #(STACKSIZE + 32)] // m1
	ldr		w12, [sp, #(STACKSIZE + 40)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_8X4_VS_LIB
#else
	CALL(inner_scale_m1b_8x4_vs_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 8)] // E
	ldr		w9, [sp, #(STACKSIZE + 16)] // sde
	lsl		w9, w9, #2 // 4*ldc
	ldr		x10, [sp, #(STACKSIZE + 24)] // inv_diag_E
	ldr		w11, [sp, #(STACKSIZE + 40)] // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_INV_8X4_VS_LIB
#else
	CALL(inner_edge_trsm_rlt_inv_8x4_vs_lib)
#endif



	// store l
	mov		x8, x7 // D
	ldr		w9, [sp, #(STACKSIZE + 0)] // ldd
	lsl		w9, w9, #2 // 4*ldd
	ldr		w10, [sp, #(STACKSIZE + 32)] // m1
	ldr		w11, [sp, #(STACKSIZE + 40)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_8X4_VS_LIB
#else
	CALL(inner_store_8x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_strsm_nt_rl_inv_8x4_vs_lib44ccc)





//                                     w0        x1        x2        x3        w4       x5        w6       x7       sp+0
// void kernel_spotrf_nt_l_8x4_lib44cc(int kmax, float *A, int sda, float *B, float *C, int ldc, float *D, int ldd, float *inv_diag_D)

	.align	4
	GLOB(kernel_spotrf_nt_l_8x4_lib44cc)
	FUN_START(kernel_spotrf_nt_l_8x4_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		w10, w2 // sda
	lsl		w10, w10, #4 // 16*sda
	mov		x11, x3 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x4 // C
	mov		w9, w5 // ldc
	lsl		w9, w9, #2 // 4*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_8X4_LIB
#else
	CALL(inner_scale_m11_8x4_lib)
#endif



	// factorization
	ldr		x8, [sp, #(STACKSIZE + 0)] // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_POTRF_8X4_LIB4
#else
	CALL(inner_edge_potrf_8x4_lib4)
#endif



	// store l
	mov		x8, x6 // D
	mov		w9, w7 // ldd
	lsl		w9, w9, #2 // 4*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_L_8X4_LIB
#else
	CALL(inner_store_l_8x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_spotrf_nt_l_8x4_lib44cc)






