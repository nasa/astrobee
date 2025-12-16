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
// x10   <- B
// x11  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NT_4X4_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_4x4_lib4c)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x12, x11, x11 // 2
	add		x13, x12, x11 // 3
	add		x14, x12, x12 // 4
	add		x15, x13, x12 // 5
	add		x16, x13, x13 // 6
	add		x17, x14, x13 // 7

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x10]
	prfm	PLDL1KEEP, [x10, x11]
	prfm	PLDL1KEEP, [x10, x12]
	prfm	PLDL1KEEP, [x10, x13]

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x10, x14]
	prfm	PLDL1KEEP, [x10, x15]
	prfm	PLDL1KEEP, [x10, x16]
	prfm	PLDL1KEEP, [x10, x17]

	// main loop
1:
	
	ldr		q28, [x10]
	ldr		q29, [x10, x11]
	ldp		q24, q25, [x9], #32

	ldr		q30, [x10, x12]
	ldr		q31, [x10, x13]
	ldp		q26, q27, [x9], #32

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	add		x10, x10, x14
	fmla	v1.4s, v24.4s, v28.s[1]
	prfm	PLDL1KEEP, [x9, #64]
	fmla	v2.4s, v24.4s, v28.s[2]
	prfm	PLDL1KEEP, [x10, x14]
	fmla	v3.4s, v24.4s, v28.s[3]
	prfm	PLDL1KEEP, [x10, x15]

	// unroll 1
	fmla	v0.4s, v25.4s, v29.s[0]
	prfm	PLDL1KEEP, [x10, x16]
	fmla	v1.4s, v25.4s, v29.s[1]
	prfm	PLDL1KEEP, [x10, x17]
	fmla	v2.4s, v25.4s, v29.s[2]
	fmla	v3.4s, v25.4s, v29.s[3]

	// unroll 2
	fmla	v0.4s, v26.4s, v30.s[0]
	fmla	v1.4s, v26.4s, v30.s[1]
	fmla	v2.4s, v26.4s, v30.s[2]
	fmla	v3.4s, v26.4s, v30.s[3]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v31.s[0]
	fmla	v1.4s, v27.4s, v31.s[1]
	fmla	v2.4s, v27.4s, v31.s[2]
	fmla	v3.4s, v27.4s, v31.s[3]

	cmp		w8, #4
	bgt		1b

//	sub		x9, x9, #32
//	sub		x10, x10, #32

0:

	cmp		w8, #3
	ble		4f

	ldr		q28, [x10]
	ldr		q29, [x10, x11]
	ldp		q24, q25, [x9], #32

	ldr		q30, [x10, x12]
	ldr		q31, [x10, x13]
	ldp		q26, q27, [x9], #32

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	add		x10, x10, x14
	fmla	v1.4s, v24.4s, v28.s[1]
//	prfm	PLDL1KEEP, [x9, #64]
	fmla	v2.4s, v24.4s, v28.s[2]
	fmla	v3.4s, v24.4s, v28.s[3]
//	prfm	PLDL1KEEP, [x10, #128]

	// unroll 1
	fmla	v0.4s, v25.4s, v29.s[0]
	fmla	v1.4s, v25.4s, v29.s[1]
	fmla	v2.4s, v25.4s, v29.s[2]
	fmla	v3.4s, v25.4s, v29.s[3]

	// unroll 2
	fmla	v0.4s, v26.4s, v30.s[0]
	fmla	v1.4s, v26.4s, v30.s[1]
	fmla	v2.4s, v26.4s, v30.s[2]
	fmla	v3.4s, v26.4s, v30.s[3]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v31.s[0]
	fmla	v1.4s, v27.4s, v31.s[1]
	fmla	v2.4s, v27.4s, v31.s[2]
	fmla	v3.4s, v27.4s, v31.s[3]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9], #16
	ldr		q28, [x10]
	fmla	v0.4s, v24.4s, v28.s[0]
	add		x10, x10, x11
	fmla	v1.4s, v24.4s, v28.s[1]
	fmla	v2.4s, v24.4s, v28.s[2]
	fmla	v3.4s, v24.4s, v28.s[3]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return




#else // cortex a57 vs a53



	// early return
	cmp		w8, #0
	ble		2f // return

//	add		x20, x11, #12 // B1 TODO

	add		x12, x11, x11 // 2
	add		x13, x12, x11 // 3
	add		x14, x12, x12 // 4
	add		x15, x13, x12 // 5
//	add		x16, x13, x13 // 6
//	add		x17, x14, x13 // 7

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x10]
	prfm	PLDL1KEEP, [x10, x11]
//	prfm	PLDL1KEEP, [x10, x12]
//	prfm	PLDL1KEEP, [x10, x13]

	// preload
	ldr		q28, [x10]
	prfm	PLDL1KEEP, [x10, x12]
	add		x10, x10, x11

	ldr		q29, [x10]
	prfm	PLDL1KEEP, [x10, x12]
	add		x10, x10, x11

	ldr		q30, [x10]
	prfm	PLDL1KEEP, [x10, x12]
	add		x10, x10, x11

//	ldr		q31, [x10]
//	prfm	PLDL1KEEP, [x10, x12]
//	add		x10, x10, x11

	ldp		q24, q25, [x9], #32

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #32]
//	prfm	PLDL1KEEP, [x10, #64]
//	prfm	PLDL1KEEP, [x10, x14]
//	prfm	PLDL1KEEP, [x10, x15]
//	prfm	PLDL1KEEP, [x10, x16]
//	prfm	PLDL1KEEP, [x10, x17]

	// main loop
1:
	

	// unroll 0
	ldp		q26, q27, [x9], #32
	fmla	v0.4s, v24.4s, v28.s[0]
	ldr		q31, [x10]
	fmla	v1.4s, v24.4s, v28.s[1]
	prfm	PLDL1KEEP, [x10, x12]
	fmla	v2.4s, v24.4s, v28.s[2]
	prfm	PLDL1KEEP, [x9, #64]
	fmla	v3.4s, v24.4s, v28.s[3]

	// unroll 1
	ldr		q28, [x10, x11]
	fmla	v0.4s, v25.4s, v29.s[0]
	prfm	PLDL1KEEP, [x10, x13]
	fmla	v1.4s, v25.4s, v29.s[1]
	fmla	v2.4s, v25.4s, v29.s[2]
	sub		w8, w8, #4
	fmla	v3.4s, v25.4s, v29.s[3]

	// unroll 2
	ldp		q24, q25, [x9], #32
	fmla	v0.4s, v26.4s, v30.s[0]
	ldr		q29, [x10, x12]
	fmla	v1.4s, v26.4s, v30.s[1]
	prfm	PLDL1KEEP, [x10, x14]
	fmla	v2.4s, v26.4s, v30.s[2]
	fmla	v3.4s, v26.4s, v30.s[3]

	// unroll 3
	ldr		q30, [x10, x13]
	fmla	v0.4s, v27.4s, v31.s[0]
	prfm	PLDL1KEEP, [x10, x15]
	fmla	v1.4s, v27.4s, v31.s[1]
	add		x10, x10, x14
	fmla	v2.4s, v27.4s, v31.s[2]
	fmla	v3.4s, v27.4s, v31.s[3]

	cmp		w8, #4
	bgt		1b

//	sub		x9, x9, #32
//	sub		x10, x10, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		q31, [x10]
	fmla	v0.4s, v24.4s, v28.s[0]
	ldp		q26, q27, [x9], #32
	fmla	v1.4s, v24.4s, v28.s[1]
//	prfm	PLDL1KEEP, [x10, x12]
	add		x10, x10, x11
	fmla	v2.4s, v24.4s, v28.s[2]
//	prfm	PLDL1KEEP, [x9, #64]
	fmla	v3.4s, v24.4s, v28.s[3]

	// unroll 1
//	ldr		q28, [x10]
	fmla	v0.4s, v25.4s, v29.s[0]
//	prfm	PLDL1KEEP, [x10, x12]
	fmla	v1.4s, v25.4s, v29.s[1]
//	add		x10, x10, x11
	fmla	v2.4s, v25.4s, v29.s[2]
	sub		w8, w8, #4
	fmla	v3.4s, v25.4s, v29.s[3]

	// unroll 2
//	ldr		q29, [x10]
	fmla	v0.4s, v26.4s, v30.s[0]
//	ldp		q24, q25, [x9], #32
	fmla	v1.4s, v26.4s, v30.s[1]
//	prfm	PLDL1KEEP, [x10, x12]
	fmla	v2.4s, v26.4s, v30.s[2]
//	add		x10, x10, x11
	fmla	v3.4s, v26.4s, v30.s[3]

	// unroll 3
//	ldr		q30, [x10]
	fmla	v0.4s, v27.4s, v31.s[0]
//	prfm	PLDL1KEEP, [x10, x12]
	fmla	v1.4s, v27.4s, v31.s[1]
//	add		x10, x10, x11
	fmla	v2.4s, v27.4s, v31.s[2]
	fmla	v3.4s, v27.4s, v31.s[3]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x9, x9, #32
//	sub		x10, x10, #32
	sub		x10, x10, x13

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9], #16
	ldr		q28, [x10]
	fmla	v0.4s, v24.4s, v28.s[0]
	add		x10, x10, x11
	fmla	v1.4s, v24.4s, v28.s[1]
	fmla	v2.4s, v24.4s, v28.s[2]
	fmla	v3.4s, v24.4s, v28.s[3]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_4x4_lib4c)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10   <- B
// x11  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NT_4X3_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_4x3_lib4c)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x12, x11, x11 // 2
	add		x13, x12, x11 // 3
	add		x14, x12, x12 // 4
	add		x15, x13, x12 // 5
	add		x16, x13, x13 // 6
	add		x17, x14, x13 // 7

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x10]
	prfm	PLDL1KEEP, [x10, x11]
	prfm	PLDL1KEEP, [x10, x12]
	prfm	PLDL1KEEP, [x10, x13]

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x10, x14]
	prfm	PLDL1KEEP, [x10, x15]
	prfm	PLDL1KEEP, [x10, x16]
	prfm	PLDL1KEEP, [x10, x17]

	// main loop
1:
	
	ldr		q28, [x10] // XXX also loading tail
	ldr		q29, [x10, x11] // XXX also loading tail
	ldp		q24, q25, [x9], #32

	ldr		q30, [x10, x12] // XXX also loading tail
	ldr		q31, [x10, x13] // XXX also loading tail
	ldp		q26, q27, [x9], #32

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	add		x10, x10, x14
	fmla	v1.4s, v24.4s, v28.s[1]
	prfm	PLDL1KEEP, [x9, #64]
	fmla	v2.4s, v24.4s, v28.s[2]
	prfm	PLDL1KEEP, [x10, x14]
//	fmla	v3.4s, v24.4s, v28.s[3]
	prfm	PLDL1KEEP, [x10, x15]

	// unroll 1
	fmla	v0.4s, v25.4s, v29.s[0]
	prfm	PLDL1KEEP, [x10, x16]
	fmla	v1.4s, v25.4s, v29.s[1]
	prfm	PLDL1KEEP, [x10, x17]
	fmla	v2.4s, v25.4s, v29.s[2]
//	fmla	v3.4s, v25.4s, v29.s[3]

	// unroll 2
	fmla	v0.4s, v26.4s, v30.s[0]
	fmla	v1.4s, v26.4s, v30.s[1]
	fmla	v2.4s, v26.4s, v30.s[2]
//	fmla	v3.4s, v26.4s, v30.s[3]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v31.s[0]
	fmla	v1.4s, v27.4s, v31.s[1]
	fmla	v2.4s, v27.4s, v31.s[2]
//	fmla	v3.4s, v27.4s, v31.s[3]

	cmp		w8, #4
	bgt		1b

//	sub		x9, x9, #32
//	sub		x10, x10, #32

0:

	cmp		w8, #3
	ble		4f

	ldr		q28, [x10] // XXX also loading tail
	ldr		q29, [x10, x11] // XXX also loading tail
	ldp		q24, q25, [x9], #32

	ldr		q30, [x10, x12] // XXX also loading tail
	ldr		q31, [x10, x13] // XXX also loading tail
	ldp		q26, q27, [x9], #32

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	add		x10, x10, x14
	fmla	v1.4s, v24.4s, v28.s[1]
//	prfm	PLDL1KEEP, [x9, #64]
	fmla	v2.4s, v24.4s, v28.s[2]
//	fmla	v3.4s, v24.4s, v28.s[3]
//	prfm	PLDL1KEEP, [x10, #128]

	// unroll 1
	fmla	v0.4s, v25.4s, v29.s[0]
	fmla	v1.4s, v25.4s, v29.s[1]
	fmla	v2.4s, v25.4s, v29.s[2]
//	fmla	v3.4s, v25.4s, v29.s[3]

	// unroll 2
	fmla	v0.4s, v26.4s, v30.s[0]
	fmla	v1.4s, v26.4s, v30.s[1]
	fmla	v2.4s, v26.4s, v30.s[2]
//	fmla	v3.4s, v26.4s, v30.s[3]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v31.s[0]
	fmla	v1.4s, v27.4s, v31.s[1]
	fmla	v2.4s, v27.4s, v31.s[2]
//	fmla	v3.4s, v27.4s, v31.s[3]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9], #16
	ldr		q28, [x10] // XXX also loading tail
	fmla	v0.4s, v24.4s, v28.s[0]
	add		x10, x10, x11
	fmla	v1.4s, v24.4s, v28.s[1]
	fmla	v2.4s, v24.4s, v28.s[2]
//	fmla	v3.4s, v24.4s, v28.s[3]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return




#else // cortex a57 vs a53



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x12, x11, x11 // 2
	add		x13, x12, x11 // 3
	add		x14, x12, x12 // 4
	add		x15, x13, x12 // 5
	add		x16, x13, x13 // 6
	add		x17, x14, x13 // 7

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x10]
	prfm	PLDL1KEEP, [x10, x11]
	prfm	PLDL1KEEP, [x10, x12]
	prfm	PLDL1KEEP, [x10, x13]

	// preload
	ldr		q28, [x10] // XXX also loading tail
	ldr		q29, [x10, x11] // XXX also loading tail
	ldr		q30, [x10, x12] // XXX also loading tail
	ldr		q31, [x10, x13] // XXX also loading tail

	ldp		q24, q25, [x9], #32

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #32]
//	prfm	PLDL1KEEP, [x10, #64]
	prfm	PLDL1KEEP, [x10, x14]
	prfm	PLDL1KEEP, [x10, x15]
	prfm	PLDL1KEEP, [x10, x16]
	prfm	PLDL1KEEP, [x10, x17]

	// main loop
1:
	

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	ldp		q26, q27, [x9], #32
	fmla	v1.4s, v24.4s, v28.s[1]
	prfm	PLDL1KEEP, [x9, #64]
	fmla	v2.4s, v24.4s, v28.s[2]
	add		x10, x10, x14
//	fmla	v3.4s, v24.4s, v28.s[3]
	ldr		q28, [x10] // XXX also loading tail

	// unroll 1
	fmla	v0.4s, v25.4s, v29.s[0]
	prfm	PLDL1KEEP, [x10, x14]
	fmla	v1.4s, v25.4s, v29.s[1]
	prfm	PLDL1KEEP, [x10, x15]
	fmla	v2.4s, v25.4s, v29.s[2]
	prfm	PLDL1KEEP, [x10, x16]
//	fmla	v3.4s, v25.4s, v29.s[3]
	ldp		q24, q25, [x9], #32

	// unroll 2
	fmla	v0.4s, v26.4s, v30.s[0]
	ldr		q29, [x10, x11] // XXX also loading tail
	fmla	v1.4s, v26.4s, v30.s[1]
	prfm	PLDL1KEEP, [x10, x17]
	fmla	v2.4s, v26.4s, v30.s[2]
	sub		w8, w8, #4
//	fmla	v3.4s, v26.4s, v30.s[3]
	ldr		q30, [x10, x12] // XXX also loading tail

	// unroll 3
	fmla	v0.4s, v27.4s, v31.s[0]
	fmla	v1.4s, v27.4s, v31.s[1]
	fmla	v2.4s, v27.4s, v31.s[2]
//	fmla	v3.4s, v27.4s, v31.s[3]
	ldr		q31, [x10, x13] // XXX also loading tail

	cmp		w8, #4
	bgt		1b

//	sub		x9, x9, #32
//	sub		x10, x10, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	ldp		q26, q27, [x9], #32
	fmla	v1.4s, v24.4s, v28.s[1]
//	prfm	PLDL1KEEP, [x9, #64]
	fmla	v2.4s, v24.4s, v28.s[2]
//	add		x10, x10, x14
//	fmla	v3.4s, v24.4s, v28.s[3]
//	ldr		q28, [x10]

	// unroll 1
	fmla	v0.4s, v25.4s, v29.s[0]
//	prfm	PLDL1KEEP, [x10, x14]
	fmla	v1.4s, v25.4s, v29.s[1]
//	prfm	PLDL1KEEP, [x10, x15]
	fmla	v2.4s, v25.4s, v29.s[2]
//	prfm	PLDL1KEEP, [x10, x16]
//	fmla	v3.4s, v25.4s, v29.s[3]
//	ldp		q24, q25, [x9], #32

	// unroll 2
	fmla	v0.4s, v26.4s, v30.s[0]
//	ldr		q29, [x10, x11]
	fmla	v1.4s, v26.4s, v30.s[1]
//	prfm	PLDL1KEEP, [x10, x17]
	fmla	v2.4s, v26.4s, v30.s[2]
	sub		w8, w8, #4
//	fmla	v3.4s, v26.4s, v30.s[3]
//	ldr		q30, [x10, x12]

	// unroll 3
	fmla	v0.4s, v27.4s, v31.s[0]
	fmla	v1.4s, v27.4s, v31.s[1]
	fmla	v2.4s, v27.4s, v31.s[2]
//	fmla	v3.4s, v27.4s, v31.s[3]
//	ldr		q31, [x10, x13]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x9, x9, #32
//	sub		x10, x10, #32
//	sub		x10, x10, x14

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9], #16
	ldr		q28, [x10] // XXX also loading tail
	fmla	v0.4s, v24.4s, v28.s[0]
	add		x10, x10, x11
	fmla	v1.4s, v24.4s, v28.s[1]
	fmla	v2.4s, v24.4s, v28.s[2]
//	fmla	v3.4s, v24.4s, v28.s[3]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_4x3_lib4c)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10   <- B
// x11  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NT_4X2_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_4x2_lib4c)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x12, x11, x11 // 2
	add		x13, x12, x11 // 3
	add		x14, x12, x12 // 4
	add		x15, x13, x12 // 5
	add		x16, x13, x13 // 6
	add		x17, x14, x13 // 7

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x10]
	prfm	PLDL1KEEP, [x10, x11]
	prfm	PLDL1KEEP, [x10, x12]
	prfm	PLDL1KEEP, [x10, x13]

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x10, x14]
	prfm	PLDL1KEEP, [x10, x15]
	prfm	PLDL1KEEP, [x10, x16]
	prfm	PLDL1KEEP, [x10, x17]

	// main loop
1:
	
	ldr		d28, [x10]
	ldr		d29, [x10, x11]
	ldp		q24, q25, [x9], #32

	ldr		d30, [x10, x12]
	ldr		d31, [x10, x13]
	ldp		q26, q27, [x9], #32

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	add		x10, x10, x14
	fmla	v1.4s, v24.4s, v28.s[1]
	prfm	PLDL1KEEP, [x9, #64]
//	fmla	v2.4s, v24.4s, v28.s[2]
	prfm	PLDL1KEEP, [x10, x14]
//	fmla	v3.4s, v24.4s, v28.s[3]
	prfm	PLDL1KEEP, [x10, x15]

	// unroll 1
	fmla	v0.4s, v25.4s, v29.s[0]
	prfm	PLDL1KEEP, [x10, x16]
	fmla	v1.4s, v25.4s, v29.s[1]
	prfm	PLDL1KEEP, [x10, x17]
//	fmla	v2.4s, v25.4s, v29.s[2]
//	fmla	v3.4s, v25.4s, v29.s[3]

	// unroll 2
	fmla	v0.4s, v26.4s, v30.s[0]
	fmla	v1.4s, v26.4s, v30.s[1]
//	fmla	v2.4s, v26.4s, v30.s[2]
//	fmla	v3.4s, v26.4s, v30.s[3]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v31.s[0]
	fmla	v1.4s, v27.4s, v31.s[1]
//	fmla	v2.4s, v27.4s, v31.s[2]
//	fmla	v3.4s, v27.4s, v31.s[3]

	cmp		w8, #4
	bgt		1b

//	sub		x9, x9, #32
//	sub		x10, x10, #32

0:

	cmp		w8, #3
	ble		4f

	ldr		d28, [x10]
	ldr		d29, [x10, x11]
	ldp		q24, q25, [x9], #32

	ldr		d30, [x10, x12]
	ldr		d31, [x10, x13]
	ldp		q26, q27, [x9], #32

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	add		x10, x10, x14
	fmla	v1.4s, v24.4s, v28.s[1]
//	prfm	PLDL1KEEP, [x9, #64]
//	fmla	v2.4s, v24.4s, v28.s[2]
//	fmla	v3.4s, v24.4s, v28.s[3]
//	prfm	PLDL1KEEP, [x10, #128]

	// unroll 1
	fmla	v0.4s, v25.4s, v29.s[0]
	fmla	v1.4s, v25.4s, v29.s[1]
//	fmla	v2.4s, v25.4s, v29.s[2]
//	fmla	v3.4s, v25.4s, v29.s[3]

	// unroll 2
	fmla	v0.4s, v26.4s, v30.s[0]
	fmla	v1.4s, v26.4s, v30.s[1]
//	fmla	v2.4s, v26.4s, v30.s[2]
//	fmla	v3.4s, v26.4s, v30.s[3]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v31.s[0]
	fmla	v1.4s, v27.4s, v31.s[1]
//	fmla	v2.4s, v27.4s, v31.s[2]
//	fmla	v3.4s, v27.4s, v31.s[3]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9], #16
	ldr		d28, [x10]
	fmla	v0.4s, v24.4s, v28.s[0]
	add		x10, x10, x11
	fmla	v1.4s, v24.4s, v28.s[1]
//	fmla	v2.4s, v24.4s, v28.s[2]
//	fmla	v3.4s, v24.4s, v28.s[3]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return




#else // cortex a57 vs a53



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x12, x11, x11 // 2
	add		x13, x12, x11 // 3
	add		x14, x12, x12 // 4
	add		x15, x13, x12 // 5
	add		x16, x13, x13 // 6
	add		x17, x14, x13 // 7

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x10]
	prfm	PLDL1KEEP, [x10, x11]
	prfm	PLDL1KEEP, [x10, x12]
	prfm	PLDL1KEEP, [x10, x13]

	// preload
	ldr		d28, [x10]
	ldr		d29, [x10, x11]
	ldr		d30, [x10, x12]
	ldr		d31, [x10, x13]

	ldp		q24, q25, [x9], #32

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #32]
//	prfm	PLDL1KEEP, [x10, #64]
	prfm	PLDL1KEEP, [x10, x14]
	prfm	PLDL1KEEP, [x10, x15]
	prfm	PLDL1KEEP, [x10, x16]
	prfm	PLDL1KEEP, [x10, x17]

	// main loop
1:
	

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	ldp		q26, q27, [x9], #32
	fmla	v1.4s, v24.4s, v28.s[1]
	prfm	PLDL1KEEP, [x9, #64]
//	fmla	v2.4s, v24.4s, v28.s[2]
	add		x10, x10, x14
//	fmla	v3.4s, v24.4s, v28.s[3]
	ldr		d28, [x10]

	// unroll 1
	fmla	v0.4s, v25.4s, v29.s[0]
	prfm	PLDL1KEEP, [x10, x14]
	fmla	v1.4s, v25.4s, v29.s[1]
	prfm	PLDL1KEEP, [x10, x15]
//	fmla	v2.4s, v25.4s, v29.s[2]
	prfm	PLDL1KEEP, [x10, x16]
//	fmla	v3.4s, v25.4s, v29.s[3]
	ldp		q24, q25, [x9], #32

	// unroll 2
	fmla	v0.4s, v26.4s, v30.s[0]
	ldr		d29, [x10, x11]
	fmla	v1.4s, v26.4s, v30.s[1]
	prfm	PLDL1KEEP, [x10, x17]
//	fmla	v2.4s, v26.4s, v30.s[2]
	sub		w8, w8, #4
//	fmla	v3.4s, v26.4s, v30.s[3]
	ldr		d30, [x10, x12]

	// unroll 3
	fmla	v0.4s, v27.4s, v31.s[0]
	fmla	v1.4s, v27.4s, v31.s[1]
//	fmla	v2.4s, v27.4s, v31.s[2]
//	fmla	v3.4s, v27.4s, v31.s[3]
	ldr		d31, [x10, x13]

	cmp		w8, #4
	bgt		1b

//	sub		x9, x9, #32
//	sub		x10, x10, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	ldp		q26, q27, [x9], #32
	fmla	v1.4s, v24.4s, v28.s[1]
//	prfm	PLDL1KEEP, [x9, #64]
//	fmla	v2.4s, v24.4s, v28.s[2]
//	add		x10, x10, x14
//	fmla	v3.4s, v24.4s, v28.s[3]
//	ldr		q28, [x10]

	// unroll 1
	fmla	v0.4s, v25.4s, v29.s[0]
//	prfm	PLDL1KEEP, [x10, x14]
	fmla	v1.4s, v25.4s, v29.s[1]
//	prfm	PLDL1KEEP, [x10, x15]
//	fmla	v2.4s, v25.4s, v29.s[2]
//	prfm	PLDL1KEEP, [x10, x16]
//	fmla	v3.4s, v25.4s, v29.s[3]
//	ldp		q24, q25, [x9], #32

	// unroll 2
	fmla	v0.4s, v26.4s, v30.s[0]
//	ldr		q29, [x10, x11]
	fmla	v1.4s, v26.4s, v30.s[1]
//	prfm	PLDL1KEEP, [x10, x17]
//	fmla	v2.4s, v26.4s, v30.s[2]
	sub		w8, w8, #4
//	fmla	v3.4s, v26.4s, v30.s[3]
//	ldr		q30, [x10, x12]

	// unroll 3
	fmla	v0.4s, v27.4s, v31.s[0]
	fmla	v1.4s, v27.4s, v31.s[1]
//	fmla	v2.4s, v27.4s, v31.s[2]
//	fmla	v3.4s, v27.4s, v31.s[3]
//	ldr		q31, [x10, x13]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x9, x9, #32
//	sub		x10, x10, #32
//	sub		x10, x10, x14

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9], #16
	ldr		d28, [x10]
	fmla	v0.4s, v24.4s, v28.s[0]
	add		x10, x10, x11
	fmla	v1.4s, v24.4s, v28.s[1]
//	fmla	v2.4s, v24.4s, v28.s[2]
//	fmla	v3.4s, v24.4s, v28.s[3]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_4x2_lib4c)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10   <- B
// x11  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NT_4X1_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_4x1_lib4c)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x12, x11, x11 // 2
	add		x13, x12, x11 // 3
	add		x14, x12, x12 // 4
	add		x15, x13, x12 // 5
	add		x16, x13, x13 // 6
	add		x17, x14, x13 // 7

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x10]
	prfm	PLDL1KEEP, [x10, x11]
	prfm	PLDL1KEEP, [x10, x12]
	prfm	PLDL1KEEP, [x10, x13]

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x10, x14]
	prfm	PLDL1KEEP, [x10, x15]
	prfm	PLDL1KEEP, [x10, x16]
	prfm	PLDL1KEEP, [x10, x17]

	// main loop
1:
	
	ldr		s28, [x10]
	ldr		s29, [x10, x11]
	ldp		q24, q25, [x9], #32

	ldr		s30, [x10, x12]
	ldr		s31, [x10, x13]
	ldp		q26, q27, [x9], #32

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	add		x10, x10, x14
//	fmla	v1.4s, v24.4s, v28.s[1]
	prfm	PLDL1KEEP, [x9, #64]
//	fmla	v2.4s, v24.4s, v28.s[2]
	prfm	PLDL1KEEP, [x10, x14]
//	fmla	v3.4s, v24.4s, v28.s[3]
	prfm	PLDL1KEEP, [x10, x15]

	// unroll 1
	fmla	v0.4s, v25.4s, v29.s[0]
	prfm	PLDL1KEEP, [x10, x16]
//	fmla	v1.4s, v25.4s, v29.s[1]
	prfm	PLDL1KEEP, [x10, x17]
//	fmla	v2.4s, v25.4s, v29.s[2]
//	fmla	v3.4s, v25.4s, v29.s[3]

	// unroll 2
	fmla	v0.4s, v26.4s, v30.s[0]
//	fmla	v1.4s, v26.4s, v30.s[1]
//	fmla	v2.4s, v26.4s, v30.s[2]
//	fmla	v3.4s, v26.4s, v30.s[3]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v31.s[0]
//	fmla	v1.4s, v27.4s, v31.s[1]
//	fmla	v2.4s, v27.4s, v31.s[2]
//	fmla	v3.4s, v27.4s, v31.s[3]

	cmp		w8, #4
	bgt		1b

//	sub		x9, x9, #32
//	sub		x10, x10, #32

0:

	cmp		w8, #3
	ble		4f

	ldr		s28, [x10]
	ldr		s29, [x10, x11]
	ldp		q24, q25, [x9], #32

	ldr		s30, [x10, x12]
	ldr		s31, [x10, x13]
	ldp		q26, q27, [x9], #32

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	add		x10, x10, x14
//	fmla	v1.4s, v24.4s, v28.s[1]
//	prfm	PLDL1KEEP, [x9, #64]
//	fmla	v2.4s, v24.4s, v28.s[2]
//	fmla	v3.4s, v24.4s, v28.s[3]
//	prfm	PLDL1KEEP, [x10, #128]

	// unroll 1
	fmla	v0.4s, v25.4s, v29.s[0]
//	fmla	v1.4s, v25.4s, v29.s[1]
//	fmla	v2.4s, v25.4s, v29.s[2]
//	fmla	v3.4s, v25.4s, v29.s[3]

	// unroll 2
	fmla	v0.4s, v26.4s, v30.s[0]
//	fmla	v1.4s, v26.4s, v30.s[1]
//	fmla	v2.4s, v26.4s, v30.s[2]
//	fmla	v3.4s, v26.4s, v30.s[3]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v31.s[0]
//	fmla	v1.4s, v27.4s, v31.s[1]
//	fmla	v2.4s, v27.4s, v31.s[2]
//	fmla	v3.4s, v27.4s, v31.s[3]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9], #16
	ldr		s28, [x10]
	fmla	v0.4s, v24.4s, v28.s[0]
	add		x10, x10, x11
//	fmla	v1.4s, v24.4s, v28.s[1]
//	fmla	v2.4s, v24.4s, v28.s[2]
//	fmla	v3.4s, v24.4s, v28.s[3]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return




#else // cortex a57 vs a53



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x12, x11, x11 // 2
	add		x13, x12, x11 // 3
	add		x14, x12, x12 // 4
	add		x15, x13, x12 // 5
	add		x16, x13, x13 // 6
	add		x17, x14, x13 // 7

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x10]
	prfm	PLDL1KEEP, [x10, x11]
	prfm	PLDL1KEEP, [x10, x12]
	prfm	PLDL1KEEP, [x10, x13]

	// preload
	ldr		s28, [x10]
	ldr		s29, [x10, x11]
	ldr		s30, [x10, x12]
	ldr		s31, [x10, x13]

	ldp		q24, q25, [x9], #32

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #32]
//	prfm	PLDL1KEEP, [x10, #64]
	prfm	PLDL1KEEP, [x10, x14]
	prfm	PLDL1KEEP, [x10, x15]
	prfm	PLDL1KEEP, [x10, x16]
	prfm	PLDL1KEEP, [x10, x17]

	// main loop
1:
	

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	ldp		q26, q27, [x9], #32
//	fmla	v1.4s, v24.4s, v28.s[1]
	prfm	PLDL1KEEP, [x9, #64]
//	fmla	v2.4s, v24.4s, v28.s[2]
	add		x10, x10, x14
//	fmla	v3.4s, v24.4s, v28.s[3]
	ldr		s28, [x10]

	// unroll 1
	fmla	v0.4s, v25.4s, v29.s[0]
	prfm	PLDL1KEEP, [x10, x14]
//	fmla	v1.4s, v25.4s, v29.s[1]
	prfm	PLDL1KEEP, [x10, x15]
//	fmla	v2.4s, v25.4s, v29.s[2]
	prfm	PLDL1KEEP, [x10, x16]
//	fmla	v3.4s, v25.4s, v29.s[3]
	ldp		q24, q25, [x9], #32

	// unroll 2
	fmla	v0.4s, v26.4s, v30.s[0]
	ldr		s29, [x10, x11]
//	fmla	v1.4s, v26.4s, v30.s[1]
	prfm	PLDL1KEEP, [x10, x17]
//	fmla	v2.4s, v26.4s, v30.s[2]
	sub		w8, w8, #4
//	fmla	v3.4s, v26.4s, v30.s[3]
	ldr		s30, [x10, x12]

	// unroll 3
	fmla	v0.4s, v27.4s, v31.s[0]
//	fmla	v1.4s, v27.4s, v31.s[1]
//	fmla	v2.4s, v27.4s, v31.s[2]
//	fmla	v3.4s, v27.4s, v31.s[3]
	ldr		s31, [x10, x13]

	cmp		w8, #4
	bgt		1b

//	sub		x9, x9, #32
//	sub		x10, x10, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	ldp		q26, q27, [x9], #32
//	fmla	v1.4s, v24.4s, v28.s[1]
//	prfm	PLDL1KEEP, [x9, #64]
//	fmla	v2.4s, v24.4s, v28.s[2]
//	add		x10, x10, x14
//	fmla	v3.4s, v24.4s, v28.s[3]
//	ldr		q28, [x10]

	// unroll 1
	fmla	v0.4s, v25.4s, v29.s[0]
//	prfm	PLDL1KEEP, [x10, x14]
//	fmla	v1.4s, v25.4s, v29.s[1]
//	prfm	PLDL1KEEP, [x10, x15]
//	fmla	v2.4s, v25.4s, v29.s[2]
//	prfm	PLDL1KEEP, [x10, x16]
//	fmla	v3.4s, v25.4s, v29.s[3]
//	ldp		q24, q25, [x9], #32

	// unroll 2
	fmla	v0.4s, v26.4s, v30.s[0]
//	ldr		q29, [x10, x11]
//	fmla	v1.4s, v26.4s, v30.s[1]
//	prfm	PLDL1KEEP, [x10, x17]
//	fmla	v2.4s, v26.4s, v30.s[2]
	sub		w8, w8, #4
//	fmla	v3.4s, v26.4s, v30.s[3]
//	ldr		q30, [x10, x12]

	// unroll 3
	fmla	v0.4s, v27.4s, v31.s[0]
//	fmla	v1.4s, v27.4s, v31.s[1]
//	fmla	v2.4s, v27.4s, v31.s[2]
//	fmla	v3.4s, v27.4s, v31.s[3]
//	ldr		q31, [x10, x13]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x9, x9, #32
//	sub		x10, x10, #32
//	sub		x10, x10, x14

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9], #16
	ldr		s28, [x10]
	fmla	v0.4s, v24.4s, v28.s[0]
	add		x10, x10, x11
//	fmla	v1.4s, v24.4s, v28.s[1]
//	fmla	v2.4s, v24.4s, v28.s[2]
//	fmla	v3.4s, v24.4s, v28.s[3]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_4x1_lib4c)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- B
// x11  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NN_4X4_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_4x4_lib4c)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x12, x10, x11
	add		x13, x12, x11
	add		x14, x13, x11

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x10, #0]
	prfm	PLDL1KEEP, [x12, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #64]
//	prfm	PLDL1KEEP, [x10, x11]

	// main loop
1:
	
	ldr		q28, [x10], #16
	ldr		q29, [x12], #16
	ldp		q24, q25, [x9], #32

	ldr		q30, [x13], #16
	ldr		q31, [x14], #16
	ldp		q26, q27, [x9], #32

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	prfm	PLDL1KEEP, [x9, #64]
	fmla	v1.4s, v24.4s, v29.s[0]
	prfm	PLDL1KEEP, [x10, #16]
	fmla	v2.4s, v24.4s, v30.s[0]
	prfm	PLDL1KEEP, [x12, #16]
	fmla	v3.4s, v24.4s, v31.s[0]
	prfm	PLDL1KEEP, [x13, #16]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
	prfm	PLDL1KEEP, [x14, #16]
	fmla	v1.4s, v25.4s, v29.s[1]
	fmla	v2.4s, v25.4s, v30.s[1]
	fmla	v3.4s, v25.4s, v31.s[1]

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
	fmla	v1.4s, v26.4s, v29.s[2]
	fmla	v2.4s, v26.4s, v30.s[2]
	fmla	v3.4s, v26.4s, v31.s[2]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v28.s[3]
	fmla	v1.4s, v27.4s, v29.s[3]
	fmla	v2.4s, v27.4s, v30.s[3]
	fmla	v3.4s, v27.4s, v31.s[3]

	cmp		w8, #4
	bgt		1b

//	sub		x9, x9, #32
//	sub		x10, x10, #32

0:

	cmp		w8, #3
	ble		4f

	ldr		q28, [x10], #16
	ldr		q29, [x12], #16
	ldp		q24, q25, [x9], #32

	ldr		q30, [x13], #16
	ldr		q31, [x14], #16
	ldp		q26, q27, [x9], #32

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v29.s[0]
	fmla	v2.4s, v24.4s, v30.s[0]
	fmla	v3.4s, v24.4s, v31.s[0]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
	fmla	v1.4s, v25.4s, v29.s[1]
	fmla	v2.4s, v25.4s, v30.s[1]
	fmla	v3.4s, v25.4s, v31.s[1]

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
	fmla	v1.4s, v26.4s, v29.s[2]
	fmla	v2.4s, v26.4s, v30.s[2]
	fmla	v3.4s, v26.4s, v31.s[2]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v28.s[3]
	fmla	v1.4s, v27.4s, v29.s[3]
	fmla	v2.4s, v27.4s, v30.s[3]
	fmla	v3.4s, v27.4s, v31.s[3]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9], #16
	ldr		s28, [x10], #4
	ldr		s29, [x12], #4
	ldr		s30, [x13], #4
	ldr		s31, [x14], #4
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v29.s[0]
	fmla	v2.4s, v24.4s, v30.s[0]
	fmla	v3.4s, v24.4s, v31.s[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else // cortex a53



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x12, x10, x11
	add		x13, x12, x11
	add		x14, x13, x11

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x10, #0]
	prfm	PLDL1KEEP, [x12, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]

	// preload
	ldr		q28, [x10], #16
	ldr		q29, [x12], #16
	ldr		q30, [x13], #16
	ldr		q31, [x14], #16

	ldp		q24, q25, [x9], #32

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #32]
//	prfm	PLDL1KEEP, [x10, x11]

	// main loop
1:
	

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	ldp		q26, q27, [x9], #32
	fmla	v1.4s, v24.4s, v29.s[0]
	prfm	PLDL1KEEP, [x9, #64]
	fmla	v2.4s, v24.4s, v30.s[0]
	prfm	PLDL1KEEP, [x10, #16]
	fmla	v3.4s, v24.4s, v31.s[0]
	prfm	PLDL1KEEP, [x12, #16]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
	prfm	PLDL1KEEP, [x13, #16]
	fmla	v1.4s, v25.4s, v29.s[1]
	prfm	PLDL1KEEP, [x14, #16]
	fmla	v2.4s, v25.4s, v30.s[1]
	fmla	v3.4s, v25.4s, v31.s[1]
	ldp		q24, q25, [x9], #32

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
	fmla	v1.4s, v26.4s, v29.s[2]
	fmla	v2.4s, v26.4s, v30.s[2]
	fmla	v3.4s, v26.4s, v31.s[2]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v28.s[3]
	ldr		q28, [x10], #16
	fmla	v1.4s, v27.4s, v29.s[3]
	ldr		q29, [x12], #16
	fmla	v2.4s, v27.4s, v30.s[3]
	ldr		q30, [x13], #16
	fmla	v3.4s, v27.4s, v31.s[3]
	ldr		q31, [x14], #16

	cmp		w8, #4
	bgt		1b

//	sub		x9, x9, #32
//	sub		x10, x10, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	ldp		q26, q27, [x9], #32
	fmla	v1.4s, v24.4s, v29.s[0]
//	prfm	PLDL1KEEP, [x9, #64]
	fmla	v2.4s, v24.4s, v30.s[0]
//	prfm	PLDL1KEEP, [x10, #16]
	fmla	v3.4s, v24.4s, v31.s[0]
//	prfm	PLDL1KEEP, [x12, #16]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
//	prfm	PLDL1KEEP, [x13, #16]
	fmla	v1.4s, v25.4s, v29.s[1]
//	prfm	PLDL1KEEP, [x14, #16]
	fmla	v2.4s, v25.4s, v30.s[1]
	fmla	v3.4s, v25.4s, v31.s[1]
//	ldp		q24, q25, [x9], #32

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
	fmla	v1.4s, v26.4s, v29.s[2]
	fmla	v2.4s, v26.4s, v30.s[2]
	fmla	v3.4s, v26.4s, v31.s[2]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v28.s[3]
//	ldr		q28, [x10], #16
	fmla	v1.4s, v27.4s, v29.s[3]
//	ldr		q29, [x12], #16
	fmla	v2.4s, v27.4s, v30.s[3]
//	ldr		q30, [x13], #16
	fmla	v3.4s, v27.4s, v31.s[3]
//	ldr		q31, [x14], #16

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x9, x9, #32
	sub		x10, x10, #16
	sub		x12, x12, #16
	sub		x13, x13, #16
	sub		x14, x14, #16

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9], #16
	ldr		s28, [x10], #4
	ldr		s29, [x12], #4
	ldr		s30, [x13], #4
	ldr		s31, [x14], #4
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v29.s[0]
	fmla	v2.4s, v24.4s, v30.s[0]
	fmla	v3.4s, v24.4s, v31.s[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_4x4_lib4c)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- B
// x11  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NN_4X3_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_4x3_lib4c)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x12, x10, x11
	add		x13, x12, x11
//	add		x14, x13, x11

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x10, #0]
	prfm	PLDL1KEEP, [x12, #0]
	prfm	PLDL1KEEP, [x13, #0]
//	prfm	PLDL1KEEP, [x14, #0]

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #64]
//	prfm	PLDL1KEEP, [x10, x11]

	// main loop
1:
	
	ldr		q28, [x10], #16
	ldr		q29, [x12], #16
	ldp		q24, q25, [x9], #32

	ldr		q30, [x13], #16
//	ldr		q31, [x14], #16
	ldp		q26, q27, [x9], #32

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	prfm	PLDL1KEEP, [x9, #64]
	fmla	v1.4s, v24.4s, v29.s[0]
	prfm	PLDL1KEEP, [x10, #16]
	fmla	v2.4s, v24.4s, v30.s[0]
	prfm	PLDL1KEEP, [x12, #16]
//	fmla	v3.4s, v24.4s, v31.s[0]
	prfm	PLDL1KEEP, [x13, #16]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
//	prfm	PLDL1KEEP, [x14, #16]
	fmla	v1.4s, v25.4s, v29.s[1]
	fmla	v2.4s, v25.4s, v30.s[1]
//	fmla	v3.4s, v25.4s, v31.s[1]

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
	fmla	v1.4s, v26.4s, v29.s[2]
	fmla	v2.4s, v26.4s, v30.s[2]
//	fmla	v3.4s, v26.4s, v31.s[2]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v28.s[3]
	fmla	v1.4s, v27.4s, v29.s[3]
	fmla	v2.4s, v27.4s, v30.s[3]
//	fmla	v3.4s, v27.4s, v31.s[3]

	cmp		w8, #4
	bgt		1b

//	sub		x9, x9, #32
//	sub		x10, x10, #32

0:

	cmp		w8, #3
	ble		4f

	ldr		q28, [x10], #16
	ldr		q29, [x12], #16
	ldp		q24, q25, [x9], #32

	ldr		q30, [x13], #16
//	ldr		q31, [x14], #16
	ldp		q26, q27, [x9], #32

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v29.s[0]
	fmla	v2.4s, v24.4s, v30.s[0]
//	fmla	v3.4s, v24.4s, v31.s[0]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
	fmla	v1.4s, v25.4s, v29.s[1]
	fmla	v2.4s, v25.4s, v30.s[1]
//	fmla	v3.4s, v25.4s, v31.s[1]

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
	fmla	v1.4s, v26.4s, v29.s[2]
	fmla	v2.4s, v26.4s, v30.s[2]
//	fmla	v3.4s, v26.4s, v31.s[2]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v28.s[3]
	fmla	v1.4s, v27.4s, v29.s[3]
	fmla	v2.4s, v27.4s, v30.s[3]
//	fmla	v3.4s, v27.4s, v31.s[3]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9], #16
	ldr		s28, [x10], #4
	ldr		s29, [x12], #4
	ldr		s30, [x13], #4
//	ldr		s31, [x14], #4
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v29.s[0]
	fmla	v2.4s, v24.4s, v30.s[0]
//	fmla	v3.4s, v24.4s, v31.s[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else // cortex a53



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x12, x10, x11
	add		x13, x12, x11
//	add		x14, x13, x11

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x10, #0]
	prfm	PLDL1KEEP, [x12, #0]
	prfm	PLDL1KEEP, [x13, #0]
//	prfm	PLDL1KEEP, [x14, #0]

	// preload
	ldr		q28, [x10], #16
	ldr		q29, [x12], #16
	ldr		q30, [x13], #16
//	ldr		q31, [x14], #16

	ldp		q24, q25, [x9], #32

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #32]
//	prfm	PLDL1KEEP, [x10, x11]

	// main loop
1:
	

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	ldp		q26, q27, [x9], #32
	fmla	v1.4s, v24.4s, v29.s[0]
	prfm	PLDL1KEEP, [x9, #64]
	fmla	v2.4s, v24.4s, v30.s[0]
	prfm	PLDL1KEEP, [x10, #16]
//	fmla	v3.4s, v24.4s, v31.s[0]
	prfm	PLDL1KEEP, [x12, #16]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
	prfm	PLDL1KEEP, [x13, #16]
	fmla	v1.4s, v25.4s, v29.s[1]
//	prfm	PLDL1KEEP, [x14, #16]
	fmla	v2.4s, v25.4s, v30.s[1]
//	fmla	v3.4s, v25.4s, v31.s[1]
	ldp		q24, q25, [x9], #32

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
	fmla	v1.4s, v26.4s, v29.s[2]
	fmla	v2.4s, v26.4s, v30.s[2]
//	fmla	v3.4s, v26.4s, v31.s[2]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v28.s[3]
	ldr		q28, [x10], #16
	fmla	v1.4s, v27.4s, v29.s[3]
	ldr		q29, [x12], #16
	fmla	v2.4s, v27.4s, v30.s[3]
	ldr		q30, [x13], #16
//	fmla	v3.4s, v27.4s, v31.s[3]
//	ldr		q31, [x14], #16

	cmp		w8, #4
	bgt		1b

//	sub		x9, x9, #32
//	sub		x10, x10, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	ldp		q26, q27, [x9], #32
	fmla	v1.4s, v24.4s, v29.s[0]
//	prfm	PLDL1KEEP, [x9, #64]
	fmla	v2.4s, v24.4s, v30.s[0]
//	prfm	PLDL1KEEP, [x10, #16]
//	fmla	v3.4s, v24.4s, v31.s[0]
//	prfm	PLDL1KEEP, [x12, #16]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
//	prfm	PLDL1KEEP, [x13, #16]
	fmla	v1.4s, v25.4s, v29.s[1]
//	prfm	PLDL1KEEP, [x14, #16]
	fmla	v2.4s, v25.4s, v30.s[1]
//	fmla	v3.4s, v25.4s, v31.s[1]
//	ldp		q24, q25, [x9], #32

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
	fmla	v1.4s, v26.4s, v29.s[2]
	fmla	v2.4s, v26.4s, v30.s[2]
//	fmla	v3.4s, v26.4s, v31.s[2]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v28.s[3]
//	ldr		q28, [x10], #16
	fmla	v1.4s, v27.4s, v29.s[3]
//	ldr		q29, [x12], #16
	fmla	v2.4s, v27.4s, v30.s[3]
//	ldr		q30, [x13], #16
//	fmla	v3.4s, v27.4s, v31.s[3]
//	ldr		q31, [x14], #16

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x9, x9, #32
	sub		x10, x10, #16
	sub		x12, x12, #16
	sub		x13, x13, #16
//	sub		x14, x14, #16

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9], #16
	ldr		s28, [x10], #4
	ldr		s29, [x12], #4
	ldr		s30, [x13], #4
//	ldr		s31, [x14], #4
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v29.s[0]
	fmla	v2.4s, v24.4s, v30.s[0]
//	fmla	v3.4s, v24.4s, v31.s[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_4x3_lib4c)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- B
// x11  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NN_4X2_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_4x2_lib4c)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x12, x10, x11
//	add		x13, x12, x11
//	add		x14, x13, x11

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x10, #0]
	prfm	PLDL1KEEP, [x12, #0]
//	prfm	PLDL1KEEP, [x13, #0]
//	prfm	PLDL1KEEP, [x14, #0]

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #64]
//	prfm	PLDL1KEEP, [x10, x11]

	// main loop
1:
	
	ldr		q28, [x10], #16
	ldr		q29, [x12], #16
	ldp		q24, q25, [x9], #32

//	ldr		q30, [x13], #16
//	ldr		q31, [x14], #16
	ldp		q26, q27, [x9], #32

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	prfm	PLDL1KEEP, [x9, #64]
	fmla	v1.4s, v24.4s, v29.s[0]
	prfm	PLDL1KEEP, [x10, #16]
//	fmla	v2.4s, v24.4s, v30.s[0]
	prfm	PLDL1KEEP, [x12, #16]
//	fmla	v3.4s, v24.4s, v31.s[0]
	prfm	PLDL1KEEP, [x13, #16]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
//	prfm	PLDL1KEEP, [x14, #16]
	fmla	v1.4s, v25.4s, v29.s[1]
//	fmla	v2.4s, v25.4s, v30.s[1]
//	fmla	v3.4s, v25.4s, v31.s[1]

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
	fmla	v1.4s, v26.4s, v29.s[2]
//	fmla	v2.4s, v26.4s, v30.s[2]
//	fmla	v3.4s, v26.4s, v31.s[2]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v28.s[3]
	fmla	v1.4s, v27.4s, v29.s[3]
//	fmla	v2.4s, v27.4s, v30.s[3]
//	fmla	v3.4s, v27.4s, v31.s[3]

	cmp		w8, #4
	bgt		1b

//	sub		x9, x9, #32
//	sub		x10, x10, #32

0:

	cmp		w8, #3
	ble		4f

	ldr		q28, [x10], #16
	ldr		q29, [x12], #16
	ldp		q24, q25, [x9], #32

//	ldr		q30, [x13], #16
//	ldr		q31, [x14], #16
	ldp		q26, q27, [x9], #32

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v29.s[0]
//	fmla	v2.4s, v24.4s, v30.s[0]
//	fmla	v3.4s, v24.4s, v31.s[0]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
	fmla	v1.4s, v25.4s, v29.s[1]
//	fmla	v2.4s, v25.4s, v30.s[1]
//	fmla	v3.4s, v25.4s, v31.s[1]

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
	fmla	v1.4s, v26.4s, v29.s[2]
//	fmla	v2.4s, v26.4s, v30.s[2]
//	fmla	v3.4s, v26.4s, v31.s[2]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v28.s[3]
	fmla	v1.4s, v27.4s, v29.s[3]
//	fmla	v2.4s, v27.4s, v30.s[3]
//	fmla	v3.4s, v27.4s, v31.s[3]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9], #16
	ldr		s28, [x10], #4
	ldr		s29, [x12], #4
//	ldr		s30, [x13], #4
//	ldr		s31, [x14], #4
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v29.s[0]
//	fmla	v2.4s, v24.4s, v30.s[0]
//	fmla	v3.4s, v24.4s, v31.s[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else // cortex a53



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x12, x10, x11
//	add		x13, x12, x11
//	add		x14, x13, x11

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x10, #0]
	prfm	PLDL1KEEP, [x12, #0]
//	prfm	PLDL1KEEP, [x13, #0]
//	prfm	PLDL1KEEP, [x14, #0]

	// preload
	ldr		q28, [x10], #16
	ldr		q29, [x12], #16
//	ldr		q30, [x13], #16
//	ldr		q31, [x14], #16

	ldp		q24, q25, [x9], #32

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #32]
//	prfm	PLDL1KEEP, [x10, x11]

	// main loop
1:
	

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	ldp		q26, q27, [x9], #32
	fmla	v1.4s, v24.4s, v29.s[0]
	prfm	PLDL1KEEP, [x9, #64]
//	fmla	v2.4s, v24.4s, v30.s[0]
	prfm	PLDL1KEEP, [x10, #16]
//	fmla	v3.4s, v24.4s, v31.s[0]
	prfm	PLDL1KEEP, [x12, #16]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
//	prfm	PLDL1KEEP, [x13, #16]
	fmla	v1.4s, v25.4s, v29.s[1]
//	prfm	PLDL1KEEP, [x14, #16]
//	fmla	v2.4s, v25.4s, v30.s[1]
//	fmla	v3.4s, v25.4s, v31.s[1]
	ldp		q24, q25, [x9], #32

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
	fmla	v1.4s, v26.4s, v29.s[2]
//	fmla	v2.4s, v26.4s, v30.s[2]
//	fmla	v3.4s, v26.4s, v31.s[2]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v28.s[3]
	ldr		q28, [x10], #16
	fmla	v1.4s, v27.4s, v29.s[3]
	ldr		q29, [x12], #16
//	fmla	v2.4s, v27.4s, v30.s[3]
//	ldr		q30, [x13], #16
//	fmla	v3.4s, v27.4s, v31.s[3]
//	ldr		q31, [x14], #16

	cmp		w8, #4
	bgt		1b

//	sub		x9, x9, #32
//	sub		x10, x10, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	ldp		q26, q27, [x9], #32
	fmla	v1.4s, v24.4s, v29.s[0]
//	prfm	PLDL1KEEP, [x9, #64]
//	fmla	v2.4s, v24.4s, v30.s[0]
//	prfm	PLDL1KEEP, [x10, #16]
//	fmla	v3.4s, v24.4s, v31.s[0]
//	prfm	PLDL1KEEP, [x12, #16]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
//	prfm	PLDL1KEEP, [x13, #16]
	fmla	v1.4s, v25.4s, v29.s[1]
//	prfm	PLDL1KEEP, [x14, #16]
//	fmla	v2.4s, v25.4s, v30.s[1]
//	fmla	v3.4s, v25.4s, v31.s[1]
//	ldp		q24, q25, [x9], #32

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
	fmla	v1.4s, v26.4s, v29.s[2]
//	fmla	v2.4s, v26.4s, v30.s[2]
//	fmla	v3.4s, v26.4s, v31.s[2]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v28.s[3]
//	ldr		q28, [x10], #16
	fmla	v1.4s, v27.4s, v29.s[3]
//	ldr		q29, [x12], #16
//	fmla	v2.4s, v27.4s, v30.s[3]
//	ldr		q30, [x13], #16
//	fmla	v3.4s, v27.4s, v31.s[3]
//	ldr		q31, [x14], #16

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x9, x9, #32
	sub		x10, x10, #16
	sub		x12, x12, #16
//	sub		x13, x13, #16
//	sub		x14, x14, #16

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9], #16
	ldr		s28, [x10], #4
	ldr		s29, [x12], #4
//	ldr		s30, [x13], #4
//	ldr		s31, [x14], #4
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v29.s[0]
//	fmla	v2.4s, v24.4s, v30.s[0]
//	fmla	v3.4s, v24.4s, v31.s[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_4x2_lib4c)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- B
// x11  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NN_4X1_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_4x1_lib4c)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

//	add		x12, x10, x11
//	add		x13, x12, x11
//	add		x14, x13, x11

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x10, #0]
//	prfm	PLDL1KEEP, [x12, #0]
//	prfm	PLDL1KEEP, [x13, #0]
//	prfm	PLDL1KEEP, [x14, #0]

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #64]
//	prfm	PLDL1KEEP, [x10, x11]

	// main loop
1:
	
	ldr		q28, [x10], #16
//	ldr		q29, [x12], #16
	ldp		q24, q25, [x9], #32

//	ldr		q30, [x13], #16
//	ldr		q31, [x14], #16
	ldp		q26, q27, [x9], #32

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	prfm	PLDL1KEEP, [x9, #64]
//	fmla	v1.4s, v24.4s, v29.s[0]
	prfm	PLDL1KEEP, [x10, #16]
//	fmla	v2.4s, v24.4s, v30.s[0]
//	prfm	PLDL1KEEP, [x12, #16]
//	fmla	v3.4s, v24.4s, v31.s[0]
	prfm	PLDL1KEEP, [x13, #16]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
//	prfm	PLDL1KEEP, [x14, #16]
//	fmla	v1.4s, v25.4s, v29.s[1]
//	fmla	v2.4s, v25.4s, v30.s[1]
//	fmla	v3.4s, v25.4s, v31.s[1]

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
//	fmla	v1.4s, v26.4s, v29.s[2]
//	fmla	v2.4s, v26.4s, v30.s[2]
//	fmla	v3.4s, v26.4s, v31.s[2]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v28.s[3]
//	fmla	v1.4s, v27.4s, v29.s[3]
//	fmla	v2.4s, v27.4s, v30.s[3]
//	fmla	v3.4s, v27.4s, v31.s[3]

	cmp		w8, #4
	bgt		1b

//	sub		x9, x9, #32
//	sub		x10, x10, #32

0:

	cmp		w8, #3
	ble		4f

	ldr		q28, [x10], #16
//	ldr		q29, [x12], #16
	ldp		q24, q25, [x9], #32

//	ldr		q30, [x13], #16
//	ldr		q31, [x14], #16
	ldp		q26, q27, [x9], #32

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
//	fmla	v1.4s, v24.4s, v29.s[0]
//	fmla	v2.4s, v24.4s, v30.s[0]
//	fmla	v3.4s, v24.4s, v31.s[0]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
//	fmla	v1.4s, v25.4s, v29.s[1]
//	fmla	v2.4s, v25.4s, v30.s[1]
//	fmla	v3.4s, v25.4s, v31.s[1]

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
//	fmla	v1.4s, v26.4s, v29.s[2]
//	fmla	v2.4s, v26.4s, v30.s[2]
//	fmla	v3.4s, v26.4s, v31.s[2]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v28.s[3]
//	fmla	v1.4s, v27.4s, v29.s[3]
//	fmla	v2.4s, v27.4s, v30.s[3]
//	fmla	v3.4s, v27.4s, v31.s[3]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9], #16
	ldr		s28, [x10], #4
//	ldr		s29, [x12], #4
//	ldr		s30, [x13], #4
//	ldr		s31, [x14], #4
	fmla	v0.4s, v24.4s, v28.s[0]
//	fmla	v1.4s, v24.4s, v29.s[0]
//	fmla	v2.4s, v24.4s, v30.s[0]
//	fmla	v3.4s, v24.4s, v31.s[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else // cortex a53



	// early return
	cmp		w8, #0
	ble		2f // return

//	add		x12, x10, x11
//	add		x13, x12, x11
//	add		x14, x13, x11

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x10, #0]
//	prfm	PLDL1KEEP, [x12, #0]
//	prfm	PLDL1KEEP, [x13, #0]
//	prfm	PLDL1KEEP, [x14, #0]

	// preload
	ldr		q28, [x10], #16
//	ldr		q29, [x12], #16
//	ldr		q30, [x13], #16
//	ldr		q31, [x14], #16

	ldp		q24, q25, [x9], #32

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #32]
//	prfm	PLDL1KEEP, [x10, x11]

	// main loop
1:
	

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	ldp		q26, q27, [x9], #32
//	fmla	v1.4s, v24.4s, v29.s[0]
	prfm	PLDL1KEEP, [x9, #64]
//	fmla	v2.4s, v24.4s, v30.s[0]
	prfm	PLDL1KEEP, [x10, #16]
//	fmla	v3.4s, v24.4s, v31.s[0]
//	prfm	PLDL1KEEP, [x12, #16]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
//	prfm	PLDL1KEEP, [x13, #16]
//	fmla	v1.4s, v25.4s, v29.s[1]
//	prfm	PLDL1KEEP, [x14, #16]
//	fmla	v2.4s, v25.4s, v30.s[1]
//	fmla	v3.4s, v25.4s, v31.s[1]
	ldp		q24, q25, [x9], #32

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
//	fmla	v1.4s, v26.4s, v29.s[2]
//	fmla	v2.4s, v26.4s, v30.s[2]
//	fmla	v3.4s, v26.4s, v31.s[2]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v28.s[3]
	ldr		q28, [x10], #16
//	fmla	v1.4s, v27.4s, v29.s[3]
//	ldr		q29, [x12], #16
//	fmla	v2.4s, v27.4s, v30.s[3]
//	ldr		q30, [x13], #16
//	fmla	v3.4s, v27.4s, v31.s[3]
//	ldr		q31, [x14], #16

	cmp		w8, #4
	bgt		1b

//	sub		x9, x9, #32
//	sub		x10, x10, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	ldp		q26, q27, [x9], #32
//	fmla	v1.4s, v24.4s, v29.s[0]
//	prfm	PLDL1KEEP, [x9, #64]
//	fmla	v2.4s, v24.4s, v30.s[0]
//	prfm	PLDL1KEEP, [x10, #16]
//	fmla	v3.4s, v24.4s, v31.s[0]
//	prfm	PLDL1KEEP, [x12, #16]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
//	prfm	PLDL1KEEP, [x13, #16]
//	fmla	v1.4s, v25.4s, v29.s[1]
//	prfm	PLDL1KEEP, [x14, #16]
//	fmla	v2.4s, v25.4s, v30.s[1]
//	fmla	v3.4s, v25.4s, v31.s[1]
//	ldp		q24, q25, [x9], #32

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
//	fmla	v1.4s, v26.4s, v29.s[2]
//	fmla	v2.4s, v26.4s, v30.s[2]
//	fmla	v3.4s, v26.4s, v31.s[2]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v28.s[3]
//	ldr		q28, [x10], #16
//	fmla	v1.4s, v27.4s, v29.s[3]
//	ldr		q29, [x12], #16
//	fmla	v2.4s, v27.4s, v30.s[3]
//	ldr		q30, [x13], #16
//	fmla	v3.4s, v27.4s, v31.s[3]
//	ldr		q31, [x14], #16

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x9, x9, #32
	sub		x10, x10, #16
//	sub		x12, x12, #16
//	sub		x13, x13, #16
//	sub		x14, x14, #16

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9], #16
	ldr		s28, [x10], #4
//	ldr		s29, [x12], #4
//	ldr		s30, [x13], #4
//	ldr		s31, [x14], #4
	fmla	v0.4s, v24.4s, v28.s[0]
//	fmla	v1.4s, v24.4s, v29.s[0]
//	fmla	v2.4s, v24.4s, v30.s[0]
//	fmla	v3.4s, v24.4s, v31.s[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_4x1_lib4c)
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
	.macro INNER_EDGE_TRSM_RLT_INV_4X4_LIB
#else
	.align 4
	FUN_START(inner_edge_trsm_rlt_inv_4x4_lib)
#endif
	
	ldr			s16, [x10, #0] // E_inv[0]
	fmul		v0.4s, v0.4s, v16.s[0]
	ldr			s16, [x8, #4] // E[1+4*0]
	fmls		v1.4s, v0.4s, v16.s[0]
	ldr			s16, [x8, #8] // E[2+4*0]
	fmls		v2.4s, v0.4s, v16.s[0]
	ldr			s16, [x8, #12] // E[3+4*0]
	fmls		v3.4s, v0.4s, v16.s[0]
	add			x8, x8, x9

	ldr			s16, [x10, #4] // E_inv[1]
	fmul		v1.4s, v1.4s, v16.s[0]
	ldr			s16, [x8, #8] // E[2+4*1]
	fmls		v2.4s, v1.4s, v16.s[0]
	ldr			s16, [x8, #12] // E[3+4*1]
	fmls		v3.4s, v1.4s, v16.s[0]
	add			x8, x8, x9

	ldr			s16, [x10, #8] // E_inv[2]
	fmul		v2.4s, v2.4s, v16.s[0]
	ldr			s16, [x8, #12] // E[3+4*1]
	fmls		v3.4s, v2.4s, v16.s[0]
//	add			x8, x8, x9

	ldr			s16, [x10, #12] // E_inv[2]
	fmul		v3.4s, v3.4s, v16.s[0]
//	add			x8, x8, x9

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trsm_rlt_inv_4x4_lib)
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
	.macro INNER_EDGE_TRSM_RLT_INV_4X4_VS_LIB
#else
	.align 4
	FUN_START(inner_edge_trsm_rlt_inv_4x4_vs_lib)
#endif
	
	// first column
	ldr			s16, [x10, #0] // E_inv[0]
	fmul		v0.4s, v0.4s, v16.s[0]
	cmp			w11, #2
	blt			0f // return

	// second column
	ldr			s16, [x8, #4] // E[1+4*0]
	fmls		v1.4s, v0.4s, v16.s[0]
	ldr			s16, [x10, #4] // E_inv[1]
	fmul		v1.4s, v1.4s, v16.s[0]
	cmp			w11, #3
	blt			0f // return

	// third column
	add			x12, x8, x9
	ldr			s16, [x8, #8] // E[2+4*0]
	fmls		v2.4s, v0.4s, v16.s[0]
	ldr			s16, [x12, #8] // E[2+4*1]
	fmls		v2.4s, v1.4s, v16.s[0]
	ldr			s16, [x10, #8] // E_inv[2]
	fmul		v2.4s, v2.4s, v16.s[0]
	cmp			w11, #4
	blt			0f // return

	// forth column
	add			x13, x12, x9
	ldr			s16, [x8, #12] // E[3+4*0]
	fmls		v3.4s, v0.4s, v16.s[0]
	ldr			s16, [x12, #12] // E[3+4*1]
	fmls		v3.4s, v1.4s, v16.s[0]
	ldr			s16, [x13, #12] // E[3+4*2]
	fmls		v3.4s, v2.4s, v16.s[0]
	ldr			s16, [x10, #12] // E_inv[3]
	fmul		v3.4s, v3.4s, v16.s[0]

0:
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trsm_rlt_inv_4x4_vs_lib)
#endif





// subroutine
//
// input arguments:
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_TRAN_4X4_LIB
#else
	.align	4
	FUN_START(inner_tran_4x4_lib)
#endif

	trn1	v4.4s, v0.4s, v1.4s
	trn2	v5.4s, v0.4s, v1.4s
	trn1	v6.4s, v2.4s, v3.4s
	trn2	v7.4s, v2.4s, v3.4s

	trn1	v0.2d, v4.2d, v6.2d
	trn2	v2.2d, v4.2d, v6.2d
	trn1	v1.2d, v5.2d, v7.2d
	trn2	v3.2d, v5.2d, v7.2d

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_tran_4x4_lib)
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
	.macro INNER_SCALE_AB_4X4_LIB
#else
	.align	4
	FUN_START(inner_scale_ab_4x4_lib)
#endif

	ld1		{v28.4s}, [x8]

	ld1		{v29.4s}, [x9]

	fmul	v0.4s, v0.4s, v28.s[0]
	fmul	v1.4s, v1.4s, v28.s[0]
	fmul	v2.4s, v2.4s, v28.s[0]
	fmul	v3.4s, v3.4s, v28.s[0]

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

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_4x4_lib)
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
	.macro INNER_SCALE_AB_4X4_VS_LIB
#else
	.align	4
	FUN_START(inner_scale_ab_4x4_vs_lib)
#endif

	ld1		{v28.4s}, [x8]

	ld1		{v29.4s}, [x9]

	fmul	v0.4s, v0.4s, v28.s[0]
	fmul	v1.4s, v1.4s, v28.s[0]
	fmul	v2.4s, v2.4s, v28.s[0]
	fmul	v3.4s, v3.4s, v28.s[0]

	fcmpe	d29, #0.0
	beq		0f

	cmp		w12, #4
	blt		1f

	ldr		q24, [x10, #0]
	add		x10, x10, x11
	fmla	v0.4s, v24.4s, v29.s[0]

	cmp		w13, #1
	ble		0f

	ldr		q24, [x10, #0]
	add		x10, x10, x11
	fmla	v1.4s, v24.4s, v29.s[0]

	cmp		w13, #2
	ble		0f

	ldr		q24, [x10, #0]
	add		x10, x10, x11
	fmla	v2.4s, v24.4s, v29.s[0]

	cmp		w13, #3
	ble		0f

	ldr		q24, [x10, #0]
	add		x10, x10, x11
	fmla	v3.4s, v24.4s, v29.s[0]

	b 0f

1:
	cmp		w12, #3
	blt		2f

	ldr		d24, [x10, #0]
	ldr		s25, [x10, #8]
	ins		v24.s[2], v25.s[0]
	add		x10, x10, x11
	fmla	v0.4s, v24.4s, v29.s[0]

	cmp		w13, #1
	ble		0f

	ldr		d24, [x10, #0]
	ldr		s25, [x10, #8]
	ins		v24.s[2], v25.s[0]
	add		x10, x10, x11
	fmla	v1.4s, v24.4s, v29.s[0]

	cmp		w13, #2
	ble		0f

	ldr		d24, [x10, #0]
	ldr		s25, [x10, #8]
	ins		v24.s[2], v25.s[0]
	add		x10, x10, x11
	fmla	v2.4s, v24.4s, v29.s[0]

	cmp		w13, #3
	ble		0f

	ldr		d24, [x10, #0]
	ldr		s25, [x10, #8]
	ins		v24.s[2], v25.s[0]
	add		x10, x10, x11
	fmla	v3.4s, v24.4s, v29.s[0]

	b 0f

2:
	cmp		w12, #2
	blt		3f

	ldr		d24, [x10, #0]
	add		x10, x10, x11
	fmla	v0.4s, v24.4s, v29.s[0]

	cmp		w13, #1
	ble		0f

	ldr		d24, [x10, #0]
	add		x10, x10, x11
	fmla	v1.4s, v24.4s, v29.s[0]

	cmp		w13, #2
	ble		0f

	ldr		d24, [x10, #0]
	add		x10, x10, x11
	fmla	v2.4s, v24.4s, v29.s[0]

	cmp		w13, #3
	ble		0f

	ldr		d24, [x10, #0]
	add		x10, x10, x11
	fmla	v3.4s, v24.4s, v29.s[0]

	b 0f

3:
	cmp		w12, #1
	blt		0f

	ldr		s24, [x10, #0]
	add		x10, x10, x11
	fmla	v0.4s, v24.4s, v29.s[0]

	cmp		w13, #1
	ble		0f

	ldr		s24, [x10, #0]
	add		x10, x10, x11
	fmla	v1.4s, v24.4s, v29.s[0]

	cmp		w13, #2
	ble		0f

	ldr		s24, [x10, #0]
	add		x10, x10, x11
	fmla	v2.4s, v24.4s, v29.s[0]

	cmp		w13, #3
	ble		0f

	ldr		s24, [x10, #0]
	add		x10, x10, x11
	fmla	v3.4s, v24.4s, v29.s[0]

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_4x4_vs_lib)
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
	.macro INNER_SCALE_M1B_4X4_LIB
#else
	.align	4
	FUN_START(inner_scale_m1b_4x4_lib)
#endif

	ld1		{v29.4s}, [x8]

	fneg	v0.4s, v0.4s
	fneg	v1.4s, v1.4s
	fneg	v2.4s, v2.4s
	fneg	v3.4s, v3.4s

	fcmpe	s29, #0.0
	beq		0f

	ldr		q24, [x9, #0]
	add		x9, x9, x10
	ldr		q25, [x9, #0]
	add		x9, x9, x10
	ldr		q26, [x9, #0]
	add		x9, x9, x10
	ldr		q27, [x9, #0]
	add		x9, x9, x10
	fmla	v0.4s, v24.4s, v29.s[0]
	fmla	v1.4s, v25.4s, v29.s[0]
	fmla	v2.4s, v26.4s, v29.s[0]
	fmla	v3.4s, v27.4s, v29.s[0]

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m1b_4x4_lib)
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
	.macro INNER_SCALE_M1B_4X4_VS_LIB
#else
	.align	4
	FUN_START(inner_scale_m1b_4x4_vs_lib)
#endif

	ld1		{v29.4s}, [x8]

	fneg	v0.4s, v0.4s
	fneg	v1.4s, v1.4s
	fneg	v2.4s, v2.4s
	fneg	v3.4s, v3.4s

	fcmpe	d29, #0.0
	beq		0f

	cmp		w11, #4
	blt		1f

	ldr		q24, [x9, #0]
	add		x9, x9, x10
	fmla	v0.4s, v24.4s, v29.s[0]

	cmp		w12, #1
	ble		0f

	ldr		q24, [x9, #0]
	add		x9, x9, x10
	fmla	v1.4s, v24.4s, v29.s[0]

	cmp		w12, #2
	ble		0f

	ldr		q24, [x9, #0]
	add		x9, x9, x10
	fmla	v2.4s, v24.4s, v29.s[0]

	cmp		w12, #3
	ble		0f

	ldr		q24, [x9, #0]
	add		x9, x9, x10
	fmla	v3.4s, v24.4s, v29.s[0]

	b 0f

1:
	cmp		w11, #3
	blt		2f

	ldr		d24, [x9, #0]
	ldr		s25, [x9, #8]
	ins		v24.s[2], v25.s[0]
	add		x9, x9, x10
	fmla	v0.4s, v24.4s, v29.s[0]

	cmp		w12, #1
	ble		0f

	ldr		d24, [x9, #0]
	ldr		s25, [x9, #8]
	ins		v24.s[2], v25.s[0]
	add		x9, x9, x10
	fmla	v1.4s, v24.4s, v29.s[0]

	cmp		w12, #2
	ble		0f

	ldr		d24, [x9, #0]
	ldr		s25, [x9, #8]
	ins		v24.s[2], v25.s[0]
	add		x9, x9, x10
	fmla	v2.4s, v24.4s, v29.s[0]

	cmp		w12, #3
	ble		0f

	ldr		d24, [x9, #0]
	ldr		s25, [x9, #8]
	ins		v24.s[2], v25.s[0]
	add		x9, x9, x10
	fmla	v3.4s, v24.4s, v29.s[0]

	b 0f

2:
	cmp		w11, #2
	blt		3f

	ldr		d24, [x9, #0]
	add		x9, x9, x10
	fmla	v0.4s, v24.4s, v29.s[0]

	cmp		w12, #1
	ble		0f

	ldr		d24, [x9, #0]
	add		x9, x9, x10
	fmla	v1.4s, v24.4s, v29.s[0]

	cmp		w12, #2
	ble		0f

	ldr		d24, [x9, #0]
	add		x9, x9, x10
	fmla	v2.4s, v24.4s, v29.s[0]

	cmp		w12, #3
	ble		0f

	ldr		d24, [x9, #0]
	add		x9, x9, x10
	fmla	v3.4s, v24.4s, v29.s[0]

	b 0f

3:
	cmp		w11, #1
	blt		0f

	ldr		s24, [x9, #0]
	add		x9, x9, x10
	fmla	v0.4s, v24.4s, v29.s[0]

	cmp		w12, #1
	ble		0f

	ldr		s24, [x9, #0]
	add		x9, x9, x10
	fmla	v1.4s, v24.4s, v29.s[0]

	cmp		w12, #2
	ble		0f

	ldr		s24, [x9, #0]
	add		x9, x9, x10
	fmla	v2.4s, v24.4s, v29.s[0]

	cmp		w12, #3
	ble		0f

	ldr		s24, [x9, #0]
	add		x9, x9, x10
	fmla	v3.4s, v24.4s, v29.s[0]

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m1b_4x4_vs_lib)
#endif





// subroutine
//
// input arguments:
// x8  <- C
// x9  <- ldc*sizeof(float)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M11_4X4_LIB
#else
	.align	4
	FUN_START(inner_scale_m11_4x4_lib)
#endif

	ldr		q24, [x8, #0]
	add		x8, x8, x9
	ldr		q25, [x8, #0]
	add		x8, x8, x9
	ldr		q26, [x8, #0]
	add		x8, x8, x9
	ldr		q27, [x8, #0]
	add		x8, x8, x9
	fsub	v0.4s, v24.4s, v0.4s
	fsub	v1.4s, v25.4s, v1.4s
	fsub	v2.4s, v26.4s, v2.4s
	fsub	v3.4s, v27.4s, v3.4s

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m11_4x4_lib)
#endif





// subroutine
//
// input arguments:
// x8  <- C
// x9  <- ldc*sizeof(float)
// x10  <- km
// x11  <- kn
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M11_4X4_VS_LIB
#else
	.align	4
	FUN_START(inner_scale_m11_4x4_vs_lib)
#endif

	cmp		w10, #4
	blt		1f

	ldr		q24, [x8, #0]
	add		x8, x8, x9
	fsub	v0.4s, v24.4s, v0.4s

	cmp		w11, #1
	ble		0f

	ldr		q24, [x8, #0]
	add		x8, x8, x9
	fsub	v1.4s, v24.4s, v1.4s

	cmp		w11, #2
	ble		0f

	ldr		q24, [x8, #0]
	add		x8, x8, x9
	fsub	v2.4s, v24.4s, v2.4s

	cmp		w11, #3
	ble		0f

	ldr		q24, [x8, #0]
	add		x8, x8, x9
	fsub	v3.4s, v24.4s, v3.4s

	b 0f

1:
	cmp		w10, #3
	blt		2f

	ldr		d24, [x8, #0]
	ldr		s25, [x8, #8]
	ins		v24.s[2], v25.s[0]
	add		x8, x8, x9
	fsub	v0.4s, v24.4s, v0.4s

	cmp		w11, #1
	ble		0f

	ldr		d24, [x8, #0]
	ldr		s25, [x8, #8]
	ins		v24.s[2], v25.s[0]
	add		x8, x8, x9
	fsub	v1.4s, v24.4s, v1.4s

	cmp		w11, #2
	ble		0f

	ldr		d24, [x8, #0]
	ldr		s25, [x8, #8]
	ins		v24.s[2], v25.s[0]
	add		x8, x8, x9
	fsub	v2.4s, v24.4s, v2.4s

	cmp		w11, #3
	ble		0f

	ldr		d24, [x8, #0]
	ldr		s25, [x8, #8]
	ins		v24.s[2], v25.s[0]
	add		x8, x8, x9
	fsub	v3.4s, v24.4s, v3.4s

	b 0f

2:
	cmp		w10, #2
	blt		3f

	ldr		d24, [x8, #0]
	add		x8, x8, x9
	fsub	v0.4s, v24.4s, v0.4s

	cmp		w11, #1
	ble		0f

	ldr		d24, [x8, #0]
	add		x8, x8, x9
	fsub	v1.4s, v24.4s, v1.4s

	cmp		w11, #2
	ble		0f

	ldr		d24, [x8, #0]
	add		x8, x8, x9
	fsub	v2.4s, v24.4s, v2.4s

	cmp		w11, #3
	ble		0f

	ldr		d24, [x8, #0]
	add		x8, x8, x9
	fsub	v3.4s, v24.4s, v3.4s

	b 0f

3:
	cmp		w10, #1
	blt		0f

	ldr		s24, [x8, #0]
	add		x8, x8, x9
	fsub	v0.4s, v24.4s, v0.4s

	cmp		w11, #1
	ble		0f

	ldr		s24, [x8, #0]
	add		x8, x8, x9
	fsub	v1.4s, v24.4s, v1.4s

	cmp		w11, #2
	ble		0f

	ldr		s24, [x8, #0]
	add		x8, x8, x9
	fsub	v2.4s, v24.4s, v2.4s

	cmp		w11, #3
	ble		0f

	ldr		s24, [x8, #0]
	add		x8, x8, x9
	fsub	v3.4s, v24.4s, v3.4s

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_m11_4x4_vs_lib)
#endif





// subroutine
//
// input arguments:
// x8   <- D
// x9   <- ldd*sizeof(float)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X4_LIB
#else
	.align 4
	FUN_START(inner_store_4x4_lib)
#endif

	str		q0, [x8, #0]
	add		x8, x8, x9
	str		q1, [x8, #0]
	add		x8, x8, x9
	str		q2, [x8, #0]
	add		x8, x8, x9
	str		q3, [x8, #0]

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4x4_lib)
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
	.macro INNER_STORE_4X4_VS_LIB
#else
	.align 4
	FUN_START(inner_store_4x4_vs_lib)
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

	// 4th row
	ins		v0.s[3], v16.s[3]
	ins		v1.s[3], v17.s[3]
	ins		v2.s[3], v18.s[3]
	ins		v3.s[3], v19.s[3]
	cmp		w10, #3
	bge		1f
	// 3th row
	ins		v0.s[2], v16.s[2]
	ins		v1.s[2], v17.s[2]
	ins		v2.s[2], v18.s[2]
	ins		v3.s[2], v19.s[2]
	cmp		w10, #2
	bge		1f
	// 2nd row
	ins		v0.s[1], v16.s[1]
	ins		v1.s[1], v17.s[1]
	ins		v2.s[1], v18.s[1]
	ins		v3.s[1], v19.s[1]
	cmp		w10, #1
	bge		1f
	// 1st row
	ins		v0.s[0], v16.s[0]
	ins		v1.s[0], v17.s[0]
	ins		v2.s[0], v18.s[0]
	ins		v3.s[0], v19.s[0]

1:
	// 1st col
	str		q0, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	str		q1, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
	str		q2, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #3
	beq		0f
	// 4th col
	str		q3, [x8, #0]

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_4x4_vs_lib)
#endif





// subroutine
//
// input arguments:
// x8   <- D
// x9   <- ldd*sizeof(float)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_4X4_LIB
#else
	.align 4
	FUN_START(inner_store_l_4x4_lib)
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

	str		q0, [x8, #0]
	add		x8, x8, x9
	str		q1, [x8, #0]
	add		x8, x8, x9
	str		q2, [x8, #0]
	add		x8, x8, x9
	str		q3, [x8, #0]

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_4x4_lib)
#endif





// subroutine
//
// input arguments:
// x8   <- D
// x9   <- ldd*sizeof(double)
// x10  <- km
// x11  <- kn
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_4X4_VS_LIB
#else
	.align 4
	FUN_START(inner_store_l_4x4_vs_lib)
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

	// 4th row
	ins		v0.s[3], v16.s[3]
	ins		v1.s[3], v17.s[3]
	ins		v2.s[3], v18.s[3]
	ins		v3.s[3], v19.s[3]
	cmp		w10, #3
	bge		1f
	// 3th row
	ins		v0.s[2], v16.s[2]
	ins		v1.s[2], v17.s[2]
	ins		v2.s[2], v18.s[2]
	ins		v3.s[2], v19.s[2]
	cmp		w10, #2
	bge		1f
	// 2nd row
	ins		v0.s[1], v16.s[1]
	ins		v1.s[1], v17.s[1]
	ins		v2.s[1], v18.s[1]
	ins		v3.s[1], v19.s[1]
	cmp		w10, #1
	bge		1f
	// 1st row
	ins		v0.s[0], v16.s[0]
	ins		v1.s[0], v17.s[0]
	ins		v2.s[0], v18.s[0]
	ins		v3.s[0], v19.s[0]

1:
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

	// 1st col
	str		q0, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	str		q1, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
	str		q2, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #3
	beq		0f
	// 4th col
	str		q3, [x8, #0]

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_l_4x4_vs_lib)
#endif





// subroutine
//
// input arguments:
// x8   <- D
// x9   <- ldd*sizeof(float)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_PREFETCH_4X4_LIB
#else
	.align 4
	FUN_START(inner_prefetch_4x4_lib)
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
//	add		x8, x8, x9

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_prefetch_4x4_lib)
#endif





//                                 w0        x1             x2        x3        x4           x5        w6       x7        sp+0
// void kernel_sgemm_nt_4x4_lib44cc(int kmax, float *alpha, float *A, float *B, float *beta, float *C, int ldc, float *D, int ldd)

	.align	4
	GLOB(kernel_sgemm_nt_4x4_lib44cc)
	FUN_START(kernel_sgemm_nt_4x4_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_4X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_4x4_lib4)
#endif



	// prefetch
	mov		x8, x7 // D
	ldr		w9, [sp, #(STACKSIZE + 0)] // ldd
	lsl		w9, w9, #2 // 4*ldd

#if MACRO_LEVEL>=1
	INNER_PREFETCH_4X4_LIB
#else
	CALL(inner_prefetch_4x4_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x4 // beta
	mov		x10, x5 // C
	mov		w11, w6 // ldc
	lsl		w11, w11, #2 // 4*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB
#else
	CALL(inner_scale_ab_4x4_lib)
#endif



	// store n
	mov		x8, x7 // D
	ldr		w9, [sp, #(STACKSIZE + 0)] // ldd
	lsl		w9, w9, #2 // 4*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
	CALL(inner_store_4x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_sgemm_nt_4x4_lib44cc)





//                                     w0        x1            x2        x3        x4           x5        w6       x7        sp+0     sp+8    sp+16
// void kernel_sgemm_nt_4x4_vs_lib44cc(int kmax, float *alpha, float *A, float *B, float *beta, float *C, int ldc, float *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_sgemm_nt_4x4_vs_lib44cc)
	FUN_START(kernel_sgemm_nt_4x4_vs_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_4X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_4x4_lib4)
#endif



	// prefetch
//	mov		x8, x7 // D
//	ldr		w9, [sp, #(STACKSIZE + 0)] // ldd
//	lsl		w9, w9, #2 // 4*ldd

#if MACRO_LEVEL>=1
//	INNER_PREFETCH_4X4_LIB
#else
//	CALL(inner_prefetch_4x4_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x4 // beta
	mov		x10, x5 // C
	mov		w11, w6 // ldc
	lsl		w11, w11, #2 // 4*ldc
	ldr		w12, [sp, #(STACKSIZE + 8)] // m1
	ldr		w13, [sp, #(STACKSIZE + 16)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_VS_LIB
#else
	CALL(inner_scale_ab_4x4_vs_lib)
#endif



	// store n
	mov		x8, x7 // D
	ldr		w9, [sp, #(STACKSIZE + 0)] // ldd
	lsl		w9, w9, #2 // 4*ldd
	ldr		w10, [sp, #(STACKSIZE + 8)] // m1
	ldr		w11, [sp, #(STACKSIZE + 16)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB
#else
	CALL(inner_store_4x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_sgemm_nt_4x4_vs_lib44cc)





//                                 w0        x1             x2         x3         x4       x5            w6         x7       sp+0       sp+8
// void kernel_sgemm_nt_4x4_lib4ccc(int kmax, double *alpha, double *A, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_sgemm_nt_4x4_lib4ccc)
	FUN_START(kernel_sgemm_nt_4x4_lib4ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // B
	mov		w11, w4 // ldb
	lsl		w11, w11, #2 // 4*ldb

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_4x4_lib4c)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #2 // 4*sdd

#if MACRO_LEVEL>=1
	INNER_PREFETCH_4X4_LIB
#else
	CALL(inner_prefetch_4x4_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // ldc
	lsl		w11, w11, #2 // 4*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB
#else
	CALL(inner_scale_ab_4x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #2 // 4*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
	CALL(inner_store_4x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_sgemm_nt_4x4_lib4ccc)





//                                     w0        x1             x2         x3         x4       x5            w6         x7       sp+0       sp+8     sp+16   sp+24
// void kernel_sgemm_nt_4x4_vs_lib4ccc(int kmax, double *alpha, double *A, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_sgemm_nt_4x4_vs_lib4ccc)
	FUN_START(kernel_sgemm_nt_4x4_vs_lib4ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // B
	mov		w11, w4 // ldb
	lsl		w11, w11, #2 // 4*ldb

	ldr		w12, [sp, #(STACKSIZE + 24)] // n1
	cmp		w12, #1
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_4X1_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_4x1_lib4c)
#endif
	
	b		103f

100:

	ldr		w12, [sp, #(STACKSIZE + 24)] // n1
	cmp		w12, #2
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_4X2_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_4x2_lib4c)
#endif
	
	b		103f

101:

	ldr		w12, [sp, #(STACKSIZE + 24)] // n1
	cmp		w12, #3
	bgt		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_4X3_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_4x3_lib4c)
#endif
	
	b		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_4x4_lib4c)
#endif

103:



	// prefetch
	// TODO prefethc vs
//	ldr		x8, [sp, #(STACKSIZE + 0)] // D
//	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
//	lsl		w9, w9, #2 // 4*sdd

#if MACRO_LEVEL>=1
//	INNER_PREFETCH_4X4_LIB
#else
//	CALL(inner_prefetch_4x4_lib)
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
	INNER_SCALE_AB_4X4_VS_LIB
#else
	CALL(inner_scale_ab_4x4_vs_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #2 // 4*ldd
	ldr		w10, [sp, #(STACKSIZE + 16)] // m1
	ldr		w11, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB
#else
	CALL(inner_store_4x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_sgemm_nt_4x4_vs_lib4ccc)





//                                 w0        x1             x2         x3         x4       x5            w6         x7       sp+0       sp+8
// void kernel_sgemm_nt_4x4_libc4cc(int kmax, double *alpha, double *A, int lda, double *B, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_sgemm_nt_4x4_libc4cc)
	FUN_START(kernel_sgemm_nt_4x4_libc4cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x4 // B
	mov		x10, x2 // A
	mov		w11, w3 // lda
	lsl		w11, w11, #2 // 4*lda

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_4x4_lib4c)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #2 // 4*ldd

#if MACRO_LEVEL>=1
	INNER_PREFETCH_4X4_LIB
#else
	CALL(inner_prefetch_4x4_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // ldc
	lsl		w11, w11, #2 // 4*ldc

#if MACRO_LEVEL>=1
	INNER_TRAN_4X4_LIB
#else
	CALL(inner_tran_4x4_lib)
#endif


#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB
#else
	CALL(inner_scale_ab_4x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #2 // 4*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
	CALL(inner_store_4x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_sgemm_nt_4x4_libc4cc)





//                                     w0        x1             x2         x3         x4       x5            w6         x7       sp+0       sp+8     sp+16   sp+24
// void kernel_sgemm_nt_4x4_vs_libc4cc(int kmax, double *alpha, double *A, int lda, double *B, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_sgemm_nt_4x4_vs_libc4cc)
	FUN_START(kernel_sgemm_nt_4x4_vs_libc4cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x4 // B
	mov		x10, x2 // A
	mov		w11, w3 // lda
	lsl		w11, w11, #2 // 4*lda

	ldr		w12, [sp, #(STACKSIZE + 16)] // m1
	cmp		w12, #1
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_4X1_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_4x1_lib4c)
#endif
	
	b		103f

100:

	ldr		w12, [sp, #(STACKSIZE + 16)] // m1
	cmp		w12, #2
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_4X2_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_4x2_lib4c)
#endif
	
	b		103f

101:

	ldr		w12, [sp, #(STACKSIZE + 16)] // m1
	cmp		w12, #3
	bgt		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_4X3_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_4x3_lib4c)
#endif
	
	b		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_4x4_lib4c)
#endif

103:



	// prefetch
//	ldr		x8, [sp, #(STACKSIZE + 0)] // D
//	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
//	lsl		w9, w9, #2 // 4*ldd

#if MACRO_LEVEL>=1
//	INNER_PREFETCH_4X4_LIB
#else
//	CALL(inner_prefetch_4x4_lib)
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
	INNER_TRAN_4X4_LIB
#else
	CALL(inner_tran_4x4_lib)
#endif


#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_VS_LIB
#else
	CALL(inner_scale_ab_4x4_vs_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #2 // 4*ldd
	ldr		w10, [sp, #(STACKSIZE + 16)] // m1
	ldr		w11, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB
#else
	CALL(inner_store_4x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_sgemm_nt_4x4_vs_libc4cc)





//                                 w0        x1             x2         x3         x4       x5            w6         x7       sp+0       sp+8
// void kernel_sgemm_nn_4x4_lib4ccc(int kmax, double *alpha, double *A, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_sgemm_nn_4x4_lib4ccc)
	FUN_START(kernel_sgemm_nn_4x4_lib4ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // B
	mov		w11, w4 // ldb
	lsl		w11, w11, #2 // 4*ldb

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_4x4_lib4c)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #2 // 4*sdd

#if MACRO_LEVEL>=1
	INNER_PREFETCH_4X4_LIB
#else
	CALL(inner_prefetch_4x4_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // ldc
	lsl		w11, w11, #2 // 4*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB
#else
	CALL(inner_scale_ab_4x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #2 // 4*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
	CALL(inner_store_4x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_sgemm_nn_4x4_lib4ccc)





//                                     w0        x1             x2         x3         x4       x5            w6         x7       sp+0       sp+8     sp+16   sp+24
// void kernel_sgemm_nn_4x4_vs_lib4ccc(int kmax, double *alpha, double *A, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_sgemm_nn_4x4_vs_lib4ccc)
	FUN_START(kernel_sgemm_nn_4x4_vs_lib4ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // B
	mov		w11, w4 // ldb
	lsl		w11, w11, #2 // 4*ldb

	ldr		w12, [sp, #(STACKSIZE + 24)] // n1
	cmp		w12, #1
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X1_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_4x1_lib4c)
#endif

	b		103f

100:

	ldr		w12, [sp, #(STACKSIZE + 24)] // n1
	cmp		w12, #2
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X2_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_4x2_lib4c)
#endif
	
	b		103f

101:

	ldr		w12, [sp, #(STACKSIZE + 24)] // n1
	cmp		w12, #3
	bgt		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X3_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_4x3_lib4c)
#endif
	
	b		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_4x4_lib4c)
#endif

103:



	// prefetch
	// TODO prefethc vs
//	ldr		x8, [sp, #(STACKSIZE + 0)] // D
//	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
//	lsl		w9, w9, #2 // 4*sdd

#if MACRO_LEVEL>=1
//	INNER_PREFETCH_4X4_LIB
#else
//	CALL(inner_prefetch_4x4_lib)
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
	INNER_SCALE_AB_4X4_VS_LIB
#else
	CALL(inner_scale_ab_4x4_vs_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #2 // 4*ldd
	ldr		w10, [sp, #(STACKSIZE + 16)] // m1
	ldr		w11, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB
#else
	CALL(inner_store_4x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_sgemm_nn_4x4_vs_lib4ccc)





//                                 w0        x1             x2         x3         x4       x5            w6         x7       sp+0       sp+8
// void kernel_sgemm_tt_4x4_libc4cc(int kmax, double *alpha, double *A, int lda, double *B, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_sgemm_tt_4x4_libc4cc)
	FUN_START(kernel_sgemm_tt_4x4_libc4cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x4 // B
	mov		x10, x2 // A
	mov		w11, w3 // lda
	lsl		w11, w11, #2 // 4*lda

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_4x4_lib4c)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #2 // 4*ldd

#if MACRO_LEVEL>=1
	INNER_PREFETCH_4X4_LIB
#else
	CALL(inner_prefetch_4x4_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // ldc
	lsl		w11, w11, #2 // 4*ldc

#if MACRO_LEVEL>=1
	INNER_TRAN_4X4_LIB
#else
	CALL(inner_tran_4x4_lib)
#endif


#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB
#else
	CALL(inner_scale_ab_4x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #2 // 4*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
	CALL(inner_store_4x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_sgemm_tt_4x4_libc4cc)





//                                     w0        x1             x2         x3         x4       x5            w6         x7       sp+0       sp+8     sp+16   sp+24
// void kernel_sgemm_tt_4x4_vs_libc4cc(int kmax, double *alpha, double *A, int lda, double *B, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_sgemm_tt_4x4_vs_libc4cc)
	FUN_START(kernel_sgemm_tt_4x4_vs_libc4cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x4 // B
	mov		x10, x2 // A
	mov		w11, w3 // lda
	lsl		w11, w11, #2 // 4*lda

	ldr		w12, [sp, #(STACKSIZE + 16)] // m1
	cmp		w12, #1
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X1_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_4x1_lib4c)
#endif
	
	b		103f

100:

	ldr		w12, [sp, #(STACKSIZE + 16)] // m1
	cmp		w12, #2
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X2_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_4x2_lib4c)
#endif
	
	b		103f

101:

	ldr		w12, [sp, #(STACKSIZE + 16)] // m1
	cmp		w12, #3
	bgt		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X3_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_4x3_lib4c)
#endif
	
	b		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NN_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_4x4_lib4c)
#endif

103:



	// prefetch
//	ldr		x8, [sp, #(STACKSIZE + 0)] // D
//	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
//	lsl		w9, w9, #2 // 4*ldd

#if MACRO_LEVEL>=1
//	INNER_PREFETCH_4X4_LIB
#else
//	CALL(inner_prefetch_4x4_lib)
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
	INNER_TRAN_4X4_LIB
#else
	CALL(inner_tran_4x4_lib)
#endif


#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_VS_LIB
#else
	CALL(inner_scale_ab_4x4_vs_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #2 // 4*ldd
	ldr		w10, [sp, #(STACKSIZE + 16)] // m1
	ldr		w11, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB
#else
	CALL(inner_store_4x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_sgemm_tt_4x4_vs_libc4cc)





//                                          w0        x1        x2        x3           x4        w5       x6        w7       sp+0      sp+8     sp+16
// void kernel_strsm_nt_rl_inv_4x4_lib44ccc(int kmax, float *A, float *B, float *beta, float *C, int ldc, float *D, int ldd, float *E, int lde, float *inv_diag_E)

	.align	4
	GLOB(kernel_strsm_nt_rl_inv_4x4_lib44ccc)
	FUN_START(kernel_strsm_nt_rl_inv_4x4_lib44ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		x10, x2 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_4X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_4x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x3 // beta
	mov		x9, x4 // C
	mov		w10, w5 // ldc
	lsl		w10, w10, #2 // 4*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_LIB
#else
	CALL(inner_scale_m1b_4x4_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 0)] // E
	ldr		w9, [sp, #(STACKSIZE + 8)] // sde
	lsl		w9, w9, #2 // 4*ldc
	ldr		x10, [sp, #(STACKSIZE + 16)] // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_INV_4X4_LIB
#else
	CALL(inner_edge_trsm_rlt_inv_4x4_lib)
#endif



	// store l
	mov		x8, x6 // D
	mov		w9, w7 // ldd
	lsl		w9, w9, #2 // 4*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
	CALL(inner_store_4x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_strsm_nt_rl_inv_4x4_lib44ccc)





//                                             w0        x1         x2         x3            x4         w5       x6         w7       sp+0       sp+8     sp+16               sp+24   sp+32
// void kernel_strsm_nt_rl_inv_4x4_vs_lib44ccc(int kmax, float *A, float *B, float *beta, float *C, int ldc, float *D, int ldd, float *E, int lde, float *inv_diag_E, int m1, int n1)

	.align	4
	GLOB(kernel_strsm_nt_rl_inv_4x4_vs_lib44ccc)
	FUN_START(kernel_strsm_nt_rl_inv_4x4_vs_lib44ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		x10, x2 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_4X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_4x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x3 // beta
	mov		x9, x4 // C
	mov		w10, w5 // ldc
	lsl		w10, w10, #2 // 4*ldc
	ldr		w11, [sp, #(STACKSIZE + 24)] // m1
	ldr		w12, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_VS_LIB
#else
	CALL(inner_scale_m1b_4x4_vs_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 0)] // E
	ldr		w9, [sp, #(STACKSIZE + 8)] // sde
	lsl		w9, w9, #2 // 4*ldc
	ldr		x10, [sp, #(STACKSIZE + 16)] // inv_diag_E
	ldr		w11, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_INV_4X4_VS_LIB
#else
	CALL(inner_edge_trsm_rlt_inv_4x4_vs_lib)
#endif



	// store l
	mov		x8, x6 // D
	mov		w9, w7 // ldd
	lsl		w9, w9, #2 // 4*ldd
	ldr		w10, [sp, #(STACKSIZE + 24)] // m1
	ldr		w11, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB
#else
	CALL(inner_store_4x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_strsm_nt_rl_inv_4x4_vs_lib44ccc)





//                                          w0        x1        x2        x3           w4       x5        w6       x7        sp+0     sp+8      sp+16    sp+24
// void kernel_strsm_nt_rl_inv_4x4_lib4cccc(int kmax, float *A, float *B, float *beta, int ldb, float *C, int ldc, float *D, int ldd, float *E, int lde, float *inv_diag_E)

	.align	4
	GLOB(kernel_strsm_nt_rl_inv_4x4_lib4cccc)
	FUN_START(kernel_strsm_nt_rl_inv_4x4_lib4cccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		x10, x2 // B
	mov		w11, w3 // ldb
	lsl		w11, w11, #2 // 4*ldb

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_4x4_lib4c)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x4 // beta
	mov		x9, x5 // C
	mov		w10, w6 // ldc
	lsl		w10, w10, #2 // 4*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_LIB
#else
	CALL(inner_scale_m1b_4x4_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 8)] // E
	ldr		w9, [sp, #(STACKSIZE + 16)] // sde
	lsl		w9, w9, #2 // 4*ldc
	ldr		x10, [sp, #(STACKSIZE + 24)] // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_INV_4X4_LIB
#else
	CALL(inner_edge_trsm_rlt_inv_4x4_lib)
#endif



	// store l
	mov		x8, x7 // D
	ldr		w9, [sp, #(STACKSIZE + 0)] // ldd
	lsl		w9, w9, #2 // 4*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
	CALL(inner_store_4x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_strsm_nt_rl_inv_4x4_lib4cccc)





//                                     w0        x1        x2        x3        w4       x5        w6       x7
// void kernel_spotrf_nt_l_4x4_lib44cc(int kmax, float *A, float *B, float *C, int ldc, float *D, int ldd, float *inv_diag_D)

	.align	4
	GLOB(kernel_spotrf_nt_l_4x4_lib44cc)
	FUN_START(kernel_spotrf_nt_l_4x4_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		x10, x2 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_4X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_4x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x3 // C
	mov		w9, w4 // ldc
	lsl		w9, w9, #2 // 4*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_4X4_LIB
#else
	CALL(inner_scale_m11_4x4_lib)
#endif



	// factorization
	mov		x8, x7 // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_POTRF_4X4_LIB4
#else
	CALL(inner_edge_potrf_4x4_lib4)
#endif



	// store l
	mov		x8, x5 // D
	mov		w9, w6 // ldd
	lsl		w9, w9, #2 // 4*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_L_4X4_LIB
#else
	CALL(inner_store_l_4x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_spotrf_nt_l_4x4_lib44cc)





//                                       w0        x1         x2         x3         w4       x5         w6       x7                  sp+0    sp+1
// void kernel_spotrf_nt_l_4x4_vs_lib44cc(int kmax, float *A, float *B, float *C, int ldc, float *D, int ldd, float *inv_diag_D, int m1, int n1)

	.align	4
	GLOB(kernel_spotrf_nt_l_4x4_vs_lib44cc)
	FUN_START(kernel_spotrf_nt_l_4x4_vs_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		x10, x2 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_4X4_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_4x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x3 // C
	mov		w9, w4 // ldc
	lsl		w9, w9, #2 // 4*ldc
	ldr		w10, [sp, #(STACKSIZE + 0)] // m1
	ldr		w11, [sp, #(STACKSIZE + 8)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M11_4X4_VS_LIB
#else
	CALL(inner_scale_m11_4x4_vs_lib)
#endif



	// factorization
	mov		x8, x7 // inv_diag_E
	ldr		w9, [sp, #(STACKSIZE + 8)] // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_POTRF_4X4_VS_LIB4
#else
	CALL(inner_edge_potrf_4x4_vs_lib4)
#endif



	// store l
	mov		x8, x5 // D
	mov		w9, w6 // ldd
	lsl		w9, w9, #2 // 4*ldd
	ldr		w10, [sp, #(STACKSIZE + 0)] // m1
	ldr		w11, [sp, #(STACKSIZE + 8)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_L_4X4_VS_LIB
#else
	CALL(inner_store_l_4x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_spotrf_nt_l_4x4_vs_lib44cc)






