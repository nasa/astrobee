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
// x10  <- B
// x11  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NT_4X4_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_nt_4x4_lib4c)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldp		q16, q17, [x9], #32
	ldp		q24, q25, [x10]
	add		x10, x10, x11
	ldp		q18, q19, [x9], #32
	ldp		q26, q27, [x10]
	add		x10, x10, x11
	ldp		q20, q21, [x9], #32
	ldp		q28, q29, [x10]
	add		x10, x10, x11
	ldp		q22, q23, [x9], #32
	ldp		q30, q31, [x10]
	add		x10, x10, x11

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v3.2d, v21.2d, v28.d[1]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v5.2d, v21.2d, v29.d[0]
	fmla	v6.2d, v20.2d, v29.d[1]
	fmla	v7.2d, v21.2d, v29.d[1]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]
	fmla	v1.2d, v23.2d, v30.d[0]
	fmla	v2.2d, v22.2d, v30.d[1]
	fmla	v3.2d, v23.2d, v30.d[1]
	fmla	v4.2d, v22.2d, v31.d[0]
	fmla	v5.2d, v23.2d, v31.d[0]
	fmla	v6.2d, v22.2d, v31.d[1]
	fmla	v7.2d, v23.2d, v31.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldp		q16, q17, [x9], #32
	ldp		q24, q25, [x10]
	add		x10, x10, x11
	ldp		q18, q19, [x9], #32
	ldp		q26, q27, [x10]
	add		x10, x10, x11
	ldp		q20, q21, [x9], #32
	ldp		q28, q29, [x10]
	add		x10, x10, x11
	ldp		q22, q23, [x9], #32
	ldp		q30, q31, [x10]
	add		x10, x10, x11

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v3.2d, v21.2d, v28.d[1]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v5.2d, v21.2d, v29.d[0]
	fmla	v6.2d, v20.2d, v29.d[1]
	fmla	v7.2d, v21.2d, v29.d[1]

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]
	fmla	v1.2d, v23.2d, v30.d[0]
	fmla	v2.2d, v22.2d, v30.d[1]
	fmla	v3.2d, v23.2d, v30.d[1]
	fmla	v4.2d, v22.2d, v31.d[0]
	fmla	v5.2d, v23.2d, v31.d[0]
	fmla	v6.2d, v22.2d, v31.d[1]
	fmla	v7.2d, v23.2d, v31.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9], #32
	ldp		q28, q29, [x10]
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	add		x10, x10, x11
	fmla	v2.2d, v24.2d, v28.d[1]
	fmla	v3.2d, v25.2d, v28.d[1]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v29.d[0]
	fmla	v5.2d, v25.2d, v29.d[0]
	cmp		w8, #0
	fmla	v6.2d, v24.2d, v29.d[1]
	fmla	v7.2d, v25.2d, v29.d[1]

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x12, x11, x11
	add		x13, x12, x11
	add		x14, x12, x12
	add		x15, x13, x12
	add		x16, x13, x13
	add		x17, x14, x13

	// prefetch
	prfm	PLDL1KEEP, [x10]
	prfm	PLDL1KEEP, [x10, x11]
	prfm	PLDL1KEEP, [x10, x12]
	prfm	PLDL1KEEP, [x10, x13]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #64]

	// preload
	ldp		q24, q25, [x10, #(0*8)]
	add		x10, x10, x11
	ldp		q26, q27, [x10, #(0*8)]
	add		x10, x10, x11
	ldp		q28, q29, [x10, #(0*8)]
	add		x10, x10, x11
	ldp		q30, q31, [x10, #(0*8)]
	add		x10, x10, x11
	ldp		q16, q17, [x9, #(0*8+0*32)]

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x10]
	prfm	PLDL1KEEP, [x10, x11]
	prfm	PLDL1KEEP, [x10, x12]
	prfm	PLDL1KEEP, [x10, x13]
	prfm	PLDL1KEEP, [x9, #128]
	prfm	PLDL1KEEP, [x9, #192]

	// zero tmp acc
	fmov	d8, xzr
	fmov    d9, d8
	fmov    d10, d8
	fmov    d11, d8
	fmov    d12, d8
	fmov    d13, d8
	fmov    d14, d8
	fmov    d15, d8

	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9, #(0*8+1*32)]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, #256]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	prfm	PLDL1KEEP, [x9, #320]
//	prfm	PLDL1KEEP, [x9, #192]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
//	prfm	PLDL1KEEP, [x10]
	prfm	PLDL1KEEP, [x10, x14]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]

	// unroll 1
	ldp		q16, q17, [x9, #(0*8+2*32)]
	fmla	v8.2d, v18.2d, v26.d[0]
	fmla	v9.2d, v19.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x10, x11]
	prfm	PLDL1KEEP, [x10, x15]
	fmla	v10.2d, v18.2d, v26.d[1]
	fmla	v11.2d, v19.2d, v26.d[1]
//	prfm	PLDL1KEEP, [x10, x12]
	prfm	PLDL1KEEP, [x10, x16]
	fmla	v12.2d, v18.2d, v27.d[0]
	fmla	v13.2d, v19.2d, v27.d[0]
//	prfm	PLDL1KEEP, [x10, x13]
	prfm	PLDL1KEEP, [x10, x17]
	fmla	v14.2d, v18.2d, v27.d[1]
	fmla	v15.2d, v19.2d, v27.d[1]

	// unroll 2
	ldp		q18, q19, [x9, #(0*8+3*32)]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	add		x9, x9, #128
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v29.d[1]
	fmla	v7.2d, v17.2d, v29.d[1]

	// unroll 3
	ldp		q16, q17, [x9, #(0*8+0*32)]
	fmla	v8.2d, v18.2d, v30.d[0]
	fmla	v9.2d, v19.2d, v30.d[0]
	ldp		q24, q25, [x10, #(0*8)]
	fmla	v10.2d, v18.2d, v30.d[1]
	add		x10, x10, x11
	fmla	v11.2d, v19.2d, v30.d[1]
	ldp		q26, q27, [x10, #(0*8)]
	fmla	v12.2d, v18.2d, v31.d[0]
	add		x10, x10, x11
	fmla	v13.2d, v19.2d, v31.d[0]
	ldp		q28, q29, [x10, #(0*8)]
	fmla	v14.2d, v18.2d, v31.d[1]
	add		x10, x10, x11
	fmla	v15.2d, v19.2d, v31.d[1]
	ldp		q30, q31, [x10, #(0*8)]
	add		x10, x10, x11

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d
	fadd	v1.2d, v1.2d, v9.2d
	fadd	v2.2d, v2.2d, v10.2d
	fadd	v3.2d, v3.2d, v11.2d
	fadd	v4.2d, v4.2d, v12.2d
	fadd	v5.2d, v5.2d, v13.2d
	fadd	v6.2d, v6.2d, v14.2d
	fadd	v7.2d, v7.2d, v15.2d

//	sub		x9, x9, #32
//	sub		x10, x10, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldp		q18, q19, [x9, #(0*8+1*32)]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x9, #192]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
//	prfm	PLDL1KEEP, [x10, #128]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]

	// unroll 1
//	prfm	PLDL1KEEP, [x10, #192]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldp		q16, q17, [x9, #(0*8+2*32)]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	sub		w8, w8, #4
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]

	// unroll 2
	ldp		q18, q19, [x9, #(0*8+3*32)]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	add		x9, x9, #128
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
//	cmp		w8, #4
	fmla	v6.2d, v16.2d, v29.d[1]
	fmla	v7.2d, v17.2d, v29.d[1]

	// unroll 3
//	ldp		q16, q17, [x9, #(0*8+0*32)]
	fmla	v0.2d, v18.2d, v30.d[0]
	fmla	v1.2d, v19.2d, v30.d[0]
//	ldp		q24, q25, [x10, #(0*8+0*32)]
	fmla	v2.2d, v18.2d, v30.d[1]
//	add		x10, x10, x11
	fmla	v3.2d, v19.2d, v30.d[1]
//	ldp		q26, q27, [x10, #(0*8+1*32)]
	fmla	v4.2d, v18.2d, v31.d[0]
//	add		x10, x10, x11
	fmla	v5.2d, v19.2d, v31.d[0]
//	ldp		q28, q29, [x10, #(0*8+2*32)]
	fmla	v6.2d, v18.2d, v31.d[1]
//	add		x10, x10, x11
	fmla	v7.2d, v19.2d, v31.d[1]
//	ldp		q30, q31, [x10, #(0*8+3*32)]
//	add		x10, x10, x11

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x10, x10, x11
//	sub		x10, x10, x11
//	sub		x10, x10, x11
//	sub		x10, x10, x11
	sub		x10, x10, x11, lsl #2

3: // clean1-up loop

	// unroll 0
	ld1		{v24.2d, v25.2d}, [x9], #32
	ld1		{v28.2d, v29.2d}, [x10]
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	add		x10, x10, x11
	fmla	v2.2d, v24.2d, v28.d[1]
	fmla	v3.2d, v25.2d, v28.d[1]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v29.d[0]
	fmla	v5.2d, v25.2d, v29.d[0]
	cmp		w8, #0
	fmla	v6.2d, v24.2d, v29.d[1]
	fmla	v7.2d, v25.2d, v29.d[1]

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nt_4x4_lib4c)
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
	.macro INNER_KERNEL_GEMM_NT_4X3_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_nt_4x3_lib4c)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ld1		{v16.2d, v17.2d}, [x9], #32
	ldr		q24, [x10, #0]
	ldr		d25, [x10, #16]
	add		x10, x10, x11
	ld1		{v18.2d, v19.2d}, [x9], #32
	ldr		q26, [x10, #0]
	ldr		d27, [x10, #16]
	add		x10, x10, x11
	ld1		{v20.2d, v21.2d}, [x9], #32
	ldr		q28, [x10, #0]
	ldr		d29, [x10, #16]
	add		x10, x10, x11
	ld1		{v22.2d, v23.2d}, [x9], #32
	ldr		q30, [x10, #0]
	ldr		d31, [x10, #16]
	add		x10, x10, x11

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v3.2d, v21.2d, v28.d[1]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v5.2d, v21.2d, v29.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]
	fmla	v1.2d, v23.2d, v30.d[0]
	fmla	v2.2d, v22.2d, v30.d[1]
	fmla	v3.2d, v23.2d, v30.d[1]
	fmla	v4.2d, v22.2d, v31.d[0]
	fmla	v5.2d, v23.2d, v31.d[0]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ld1		{v16.2d, v17.2d}, [x9], #32
	ldr		q24, [x10, #0]
	ldr		d25, [x10, #16]
	add		x10, x10, x11
	ld1		{v18.2d, v19.2d}, [x9], #32
	ldr		q26, [x10, #0]
	ldr		d27, [x10, #16]
	add		x10, x10, x11
	ld1		{v20.2d, v21.2d}, [x9], #32
	ldr		q28, [x10, #0]
	ldr		d29, [x10, #16]
	add		x10, x10, x11
	ld1		{v22.2d, v23.2d}, [x9], #32
	ldr		q30, [x10, #0]
	ldr		d31, [x10, #16]
	add		x10, x10, x11

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v3.2d, v21.2d, v28.d[1]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v5.2d, v21.2d, v29.d[0]

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]
	fmla	v1.2d, v23.2d, v30.d[0]
	fmla	v2.2d, v22.2d, v30.d[1]
	fmla	v3.2d, v23.2d, v30.d[1]
	fmla	v4.2d, v22.2d, v31.d[0]
	fmla	v5.2d, v23.2d, v31.d[0]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ld1		{v24.2d, v25.2d}, [x9], #32
	ldr		q28, [x10, #0]
	ldr		d29, [x10, #16]
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	add		x10, x10, x11
	fmla	v2.2d, v24.2d, v28.d[1]
	fmla	v3.2d, v25.2d, v28.d[1]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v29.d[0]
	fmla	v5.2d, v25.2d, v29.d[0]
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x12, x11, x11
	add		x13, x12, x11
	add		x14, x12, x12
	add		x15, x13, x12
	add		x16, x13, x13
	add		x17, x14, x13

	// prefetch
	prfm	PLDL1KEEP, [x10]
	prfm	PLDL1KEEP, [x10, x11]
	prfm	PLDL1KEEP, [x10, x12]
	prfm	PLDL1KEEP, [x10, x13]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #64]

	// preload
	ldr		q24, [x10, #(0*8)]
	ldr		d25, [x10, #(2*8)]
	add		x10, x10, x11
	ldr		q26, [x10, #(0*8)]
	ldr		d27, [x10, #(2*8)]
	add		x10, x10, x11
	ldr		q28, [x10, #(0*8)]
	ldr		d29, [x10, #(2*8)]
	add		x10, x10, x11
	ldr		q30, [x10, #(0*8)]
	ldr		d31, [x10, #(2*8)]
	add		x10, x10, x11
	ldp		q16, q17, [x9, #(0*8+0*32)]

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x10]
	prfm	PLDL1KEEP, [x10, x11]
	prfm	PLDL1KEEP, [x10, x12]
	prfm	PLDL1KEEP, [x10, x13]
	prfm	PLDL1KEEP, [x9, #128]
	prfm	PLDL1KEEP, [x9, #192]

	// zero tmp acc
	fmov	d8, xzr
	fmov    d9, d8
	fmov    d10, d8
	fmov    d11, d8
	fmov    d12, d8
	fmov    d13, d8

	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9, #(0*8+1*32)]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, #256]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	prfm	PLDL1KEEP, [x9, #320]
//	prfm	PLDL1KEEP, [x9, #192]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
//	prfm	PLDL1KEEP, [x10]
	prfm	PLDL1KEEP, [x10, x14]

	// unroll 1
	ldp		q16, q17, [x9, #(0*8+2*32)]
	fmla	v8.2d, v18.2d, v26.d[0]
	fmla	v9.2d, v19.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x10, x11]
	prfm	PLDL1KEEP, [x10, x15]
	fmla	v10.2d, v18.2d, v26.d[1]
	fmla	v11.2d, v19.2d, v26.d[1]
//	prfm	PLDL1KEEP, [x10, x12]
	prfm	PLDL1KEEP, [x10, x16]
	fmla	v12.2d, v18.2d, v27.d[0]
	fmla	v13.2d, v19.2d, v27.d[0]
//	prfm	PLDL1KEEP, [x10, x13]
	prfm	PLDL1KEEP, [x10, x17]

	// unroll 2
	ldp		q18, q19, [x9, #(0*8+3*32)]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	add		x9, x9, #128
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	cmp		w8, #4

	// unroll 3
	ldp		q16, q17, [x9, #(0*8+0*32)]
	fmla	v8.2d, v18.2d, v30.d[0]
	fmla	v9.2d, v19.2d, v30.d[0]
	ldr		q24, [x10, #(0*8)]
	ldr		d25, [x10, #(2*8)]
	fmla	v10.2d, v18.2d, v30.d[1]
	add		x10, x10, x11
	fmla	v11.2d, v19.2d, v30.d[1]
	ldr		q26, [x10, #(0*8)]
	ldr		d27, [x10, #(2*8)]
	fmla	v12.2d, v18.2d, v31.d[0]
	add		x10, x10, x11
	fmla	v13.2d, v19.2d, v31.d[0]
	ldr		q28, [x10, #(0*8)]
	ldr		d29, [x10, #(2*8)]
	add		x10, x10, x11
	ldr		q30, [x10, #(0*8)]
	ldr		d31, [x10, #(2*8)]
	add		x10, x10, x11

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d
	fadd	v1.2d, v1.2d, v9.2d
	fadd	v2.2d, v2.2d, v10.2d
	fadd	v3.2d, v3.2d, v11.2d
	fadd	v4.2d, v4.2d, v12.2d
	fadd	v5.2d, v5.2d, v13.2d

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldp		q18, q19, [x9, #(0*8+1*32)]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x9, #192]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
//	prfm	PLDL1KEEP, [x10, #128]

	// unroll 1
//	prfm	PLDL1KEEP, [x10, #192]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldp		q16, q17, [x9, #(0*8+2*32)]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9, #(0*8+3*32)]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	add		x9, x9, #128
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
//	cmp		w8, #4

	// unroll 3
//	ldp		q16, q17, [x9, #(0*8+0*32)]
	fmla	v0.2d, v18.2d, v30.d[0]
	fmla	v1.2d, v19.2d, v30.d[0]
//	ldp		q24, q25, [x10, #(0*8+0*32)]
	fmla	v2.2d, v18.2d, v30.d[1]
//	add		x10, x10, x11
	fmla	v3.2d, v19.2d, v30.d[1]
//	ldp		q26, q27, [x10, #(0*8+1*32)]
	fmla	v4.2d, v18.2d, v31.d[0]
//	add		x10, x10, x11
	fmla	v5.2d, v19.2d, v31.d[0]
//	ldp		q28, q29, [x10, #(0*8+2*32)]
//	add		x10, x10, x11
//	ldp		q30, q31, [x10, #(0*8+3*32)]
//	add		x10, x10, x11

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x10, x10, x11
//	sub		x10, x10, x11
//	sub		x10, x10, x11
//	sub		x10, x10, x11
	sub		x10, x10, x11, lsl #2

3: // clean1-up loop

	// unroll 0
	ld1		{v24.2d, v25.2d}, [x9], #32
	ldr		q28, [x10, #(0*8)]
	ldr		d29, [x10, #(2*8)]
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	add		x10, x10, x11
	fmla	v2.2d, v24.2d, v28.d[1]
	fmla	v3.2d, v25.2d, v28.d[1]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v29.d[0]
	fmla	v5.2d, v25.2d, v29.d[0]
	cmp		w8, #0

	bgt		3b

2: // return

	

#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nt_4x3_lib4c)
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
	.macro INNER_KERNEL_GEMM_NT_4X2_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_nt_4x2_lib4c)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ld1		{v16.2d, v17.2d}, [x9], #32
	ldr		q24, [x10, #0]
	add		x10, x10, x11
	ld1		{v18.2d, v19.2d}, [x9], #32
	ldr		q26, [x10, #0]
	add		x10, x10, x11
	ld1		{v20.2d, v21.2d}, [x9], #32
	ldr		q28, [x10, #0]
	add		x10, x10, x11
	ld1		{v22.2d, v23.2d}, [x9], #32
	ldr		q30, [x10, #0]
	add		x10, x10, x11

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v3.2d, v21.2d, v28.d[1]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]
	fmla	v1.2d, v23.2d, v30.d[0]
	fmla	v2.2d, v22.2d, v30.d[1]
	fmla	v3.2d, v23.2d, v30.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ld1		{v16.2d, v17.2d}, [x9], #32
	ldr		q24, [x10, #0]
	add		x10, x10, x11
	ld1		{v18.2d, v19.2d}, [x9], #32
	ldr		q26, [x10, #0]
	add		x10, x10, x11
	ld1		{v20.2d, v21.2d}, [x9], #32
	ldr		q28, [x10, #0]
	add		x10, x10, x11
	ld1		{v22.2d, v23.2d}, [x9], #32
	ldr		q30, [x10, #0]
	add		x10, x10, x11

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v3.2d, v21.2d, v28.d[1]

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]
	fmla	v1.2d, v23.2d, v30.d[0]
	fmla	v2.2d, v22.2d, v30.d[1]
	fmla	v3.2d, v23.2d, v30.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ld1		{v24.2d, v25.2d}, [x9], #32
	ldr		q28, [x10, #0]
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	add		x10, x10, x11
	fmla	v2.2d, v24.2d, v28.d[1]
	fmla	v3.2d, v25.2d, v28.d[1]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x12, x11, x11
	add		x13, x12, x11
	add		x14, x12, x12
	add		x15, x13, x12
	add		x16, x13, x13
	add		x17, x14, x13

	// prefetch
	prfm	PLDL1KEEP, [x10]
	prfm	PLDL1KEEP, [x10, x11]
	prfm	PLDL1KEEP, [x10, x12]
	prfm	PLDL1KEEP, [x10, x13]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #64]

	// preload
	ldr		q24, [x10, #(0*8)]
	add		x10, x10, x11
	ldr		q26, [x10, #(0*8)]
	add		x10, x10, x11
	ldr		q28, [x10, #(0*8)]
	add		x10, x10, x11
	ldr		q30, [x10, #(0*8)]
	add		x10, x10, x11
	ldp		q16, q17, [x9, #(0*8+0*32)]

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x10]
	prfm	PLDL1KEEP, [x10, x11]
	prfm	PLDL1KEEP, [x10, x12]
	prfm	PLDL1KEEP, [x10, x13]
	prfm	PLDL1KEEP, [x9, #128]
	prfm	PLDL1KEEP, [x9, #192]

	// zero tmp acc
	fmov	d8, xzr
	fmov    d9, d8
	fmov    d10, d8
	fmov    d11, d8

	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9, #(0*8+1*32)]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, #256]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	prfm	PLDL1KEEP, [x9, #320]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x10]
	prfm	PLDL1KEEP, [x10, x14]

	// unroll 1
	ldp		q16, q17, [x9, #(0*8+2*32)]
	fmla	v8.2d, v18.2d, v26.d[0]
	fmla	v9.2d, v19.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x10, x11]
	prfm	PLDL1KEEP, [x10, x15]
	fmla	v10.2d, v18.2d, v26.d[1]
	fmla	v11.2d, v19.2d, v26.d[1]
//	prfm	PLDL1KEEP, [x10, x12]
	prfm	PLDL1KEEP, [x10, x16]
//	prfm	PLDL1KEEP, [x10, x13]
	prfm	PLDL1KEEP, [x10, x17]

	// unroll 2
	ldp		q18, q19, [x9, #(0*8+3*32)]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	add		x9, x9, #128
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	sub		w8, w8, #4
	cmp		w8, #4

	// unroll 3
	ldp		q16, q17, [x9, #(0*8+0*32)]
	fmla	v8.2d, v18.2d, v30.d[0]
	fmla	v9.2d, v19.2d, v30.d[0]
	ldr		q24, [x10, #(0*8)]
	fmla	v10.2d, v18.2d, v30.d[1]
	add		x10, x10, x11
	fmla	v11.2d, v19.2d, v30.d[1]
	ldr		q26, [x10, #(0*8)]
	add		x10, x10, x11
	ldr		q28, [x10, #(0*8)]
	add		x10, x10, x11
	ldr		q30, [x10, #(0*8)]
	add		x10, x10, x11

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d
	fadd	v1.2d, v1.2d, v9.2d
	fadd	v2.2d, v2.2d, v10.2d
	fadd	v3.2d, v3.2d, v11.2d

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldp		q18, q19, [x9, #(0*8+1*32)]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x10, #128]

	// unroll 1
//	prfm	PLDL1KEEP, [x10, #192]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldp		q16, q17, [x9, #(0*8+2*32)]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9, #(0*8+3*32)]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	add		x9, x9, #128
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
//	cmp		w8, #4

	// unroll 3
//	ldp		q16, q17, [x9, #(0*8+0*32)]
	fmla	v0.2d, v18.2d, v30.d[0]
	fmla	v1.2d, v19.2d, v30.d[0]
//	ldp		q24, q25, [x10, #(0*8+0*32)]
	fmla	v2.2d, v18.2d, v30.d[1]
//	add		x10, x10, x11
	fmla	v3.2d, v19.2d, v30.d[1]
//	ldp		q26, q27, [x10, #(0*8+1*32)]
//	add		x10, x10, x11
//	ldp		q28, q29, [x10, #(0*8+2*32)]
//	add		x10, x10, x11
//	ldp		q30, q31, [x10, #(0*8+3*32)]
//	add		x10, x10, x11

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x10, x10, x11
//	sub		x10, x10, x11
//	sub		x10, x10, x11
//	sub		x10, x10, x11
	sub		x10, x10, x11, lsl #2

3: // clean1-up loop

	// unroll 0
	ld1		{v24.2d, v25.2d}, [x9], #32
	ldr		q28, [x10, #(0*8)]
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	add		x10, x10, x11
	fmla	v2.2d, v24.2d, v28.d[1]
	fmla	v3.2d, v25.2d, v28.d[1]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return

	

#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nt_4x2_lib4c)
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
	.macro INNER_KERNEL_GEMM_NT_4X1_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_nt_4x1_lib4c)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ld1		{v16.2d, v17.2d}, [x9], #32
	ldr		d24, [x10, #0]
	add		x10, x10, x11
	ld1		{v18.2d, v19.2d}, [x9], #32
	ldr		d26, [x10, #0]
	add		x10, x10, x11
	ld1		{v20.2d, v21.2d}, [x9], #32
	ldr		d28, [x10, #0]
	add		x10, x10, x11
	ld1		{v22.2d, v23.2d}, [x9], #32
	ldr		d30, [x10, #0]
	add		x10, x10, x11

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]
	fmla	v1.2d, v23.2d, v30.d[0]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ld1		{v16.2d, v17.2d}, [x9], #32
	ldr		d24, [x10, #0]
	add		x10, x10, x11
	ld1		{v18.2d, v19.2d}, [x9], #32
	ldr		d26, [x10, #0]
	add		x10, x10, x11
	ld1		{v20.2d, v21.2d}, [x9], #32
	ldr		d28, [x10, #0]
	add		x10, x10, x11
	ld1		{v22.2d, v23.2d}, [x9], #32
	ldr		d30, [x10, #0]
	add		x10, x10, x11

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]
	fmla	v1.2d, v23.2d, v30.d[0]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ld1		{v24.2d, v25.2d}, [x9], #32
	ldr		d28, [x10, #0]
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	add		x10, x10, x11
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x12, x11, x11
	add		x13, x12, x11
	add		x14, x12, x12
	add		x15, x13, x12
	add		x16, x13, x13
	add		x17, x14, x13

	// prefetch
	prfm	PLDL1KEEP, [x10]
	prfm	PLDL1KEEP, [x10, x11]
	prfm	PLDL1KEEP, [x10, x12]
	prfm	PLDL1KEEP, [x10, x13]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #64]

	// preload
	ldr		d24, [x10, #(0*8)]
	add		x10, x10, x11
	ldr		d26, [x10, #(0*8)]
	add		x10, x10, x11
	ldr		d28, [x10, #(0*8)]
	add		x10, x10, x11
	ldr		d30, [x10, #(0*8)]
	add		x10, x10, x11
	ldp		q16, q17, [x9, #(0*8+0*32)]

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x10]
	prfm	PLDL1KEEP, [x10, x11]
	prfm	PLDL1KEEP, [x10, x12]
	prfm	PLDL1KEEP, [x10, x13]
	prfm	PLDL1KEEP, [x9, #128]
	prfm	PLDL1KEEP, [x9, #192]

	// zero tmp acc
	fmov	d8, xzr
	fmov    d9, d8

	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9, #(0*8+1*32)]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, #256]
//	prfm	PLDL1KEEP, [x9, #128]
	prfm	PLDL1KEEP, [x9, #320]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x10]
	prfm	PLDL1KEEP, [x10, x14]

	// unroll 1
	ldp		q16, q17, [x9, #(0*8+2*32)]
	fmla	v8.2d, v18.2d, v26.d[0]
	fmla	v9.2d, v19.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x10, x11]
	prfm	PLDL1KEEP, [x10, x15]
//	prfm	PLDL1KEEP, [x10, x12]
	prfm	PLDL1KEEP, [x10, x16]
//	prfm	PLDL1KEEP, [x10, x13]
	prfm	PLDL1KEEP, [x10, x17]

	// unroll 2
	ldp		q18, q19, [x9, #(0*8+3*32)]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	add		x9, x9, #128
	sub		w8, w8, #4
	cmp		w8, #4

	// unroll 3
	ldp		q16, q17, [x9, #(0*8+0*32)]
	fmla	v8.2d, v18.2d, v30.d[0]
	fmla	v9.2d, v19.2d, v30.d[0]
	ldr		d24, [x10, #(0*8)]
	add		x10, x10, x11
	ldr		d26, [x10, #(0*8)]
	add		x10, x10, x11
	ldr		d28, [x10, #(0*8)]
	add		x10, x10, x11
	ldr		d30, [x10, #(0*8)]
	add		x10, x10, x11

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d
	fadd	v1.2d, v1.2d, v9.2d

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldp		q18, q19, [x9, #(0*8+1*32)]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x10, #128]

	// unroll 1
//	prfm	PLDL1KEEP, [x10, #192]
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	ldp		q16, q17, [x9, #(0*8+2*32)]
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9, #(0*8+3*32)]
	fmla	v0.2d, v16.2d, v28.d[0]
	fmla	v1.2d, v17.2d, v28.d[0]
	add		x9, x9, #128
//	cmp		w8, #4

	// unroll 3
//	ldp		q16, q17, [x9, #(0*8+0*32)]
	fmla	v0.2d, v18.2d, v30.d[0]
	fmla	v1.2d, v19.2d, v30.d[0]
//	ldp		q24, q25, [x10, #(0*8+0*32)]
//	add		x10, x10, x11
//	ldp		q26, q27, [x10, #(0*8+1*32)]
//	add		x10, x10, x11
//	ldp		q28, q29, [x10, #(0*8+2*32)]
//	add		x10, x10, x11
//	ldp		q30, q31, [x10, #(0*8+3*32)]
//	add		x10, x10, x11

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x10, x10, x11
//	sub		x10, x10, x11
//	sub		x10, x10, x11
//	sub		x10, x10, x11
	sub		x10, x10, x11, lsl #2

3: // clean1-up loop

	// unroll 0
	ld1		{v24.2d, v25.2d}, [x9], #32
	ldr		d28, [x10, #(0*8)]
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	add		x10, x10, x11
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return

	

#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nt_4x1_lib4c)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10   <- B
// x11   <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_4X4_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_nn_4x4_lib4c)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x12, x10, x11
	add		x13, x12, x11
	add		x14, x13, x11

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x10], #32
	ldp		q26, q27, [x12], #32
	ldp		q28, q29, [x13], #32
	ldp		q30, q31, [x14], #32
	ldp		q16, q17, [x9], #32
	ldp		q18, q19, [x9], #32
	ldp		q20, q21, [x9], #32
	ldp		q22, q23, [x9], #32

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
	fmla	v6.2d, v16.2d, v30.d[0]
	fmla	v7.2d, v17.2d, v30.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	fmla	v6.2d, v18.2d, v30.d[1]
	fmla	v7.2d, v19.2d, v30.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	fmla	v1.2d, v21.2d, v25.d[0]
	fmla	v2.2d, v20.2d, v27.d[0]
	fmla	v3.2d, v21.2d, v27.d[0]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v5.2d, v21.2d, v29.d[0]
	fmla	v6.2d, v20.2d, v31.d[0]
	fmla	v7.2d, v21.2d, v31.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]
	fmla	v1.2d, v23.2d, v25.d[1]
	fmla	v2.2d, v22.2d, v27.d[1]
	fmla	v3.2d, v23.2d, v27.d[1]
	fmla	v4.2d, v22.2d, v29.d[1]
	fmla	v5.2d, v23.2d, v29.d[1]
	fmla	v6.2d, v22.2d, v31.d[1]
	fmla	v7.2d, v23.2d, v31.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x10], #32
	ldp		q26, q27, [x12], #32
	ldp		q28, q29, [x13], #32
	ldp		q30, q31, [x14], #32
	ldp		q16, q17, [x9], #32
	ldp		q18, q19, [x9], #32
	ldp		q20, q21, [x9], #32
	ldp		q22, q23, [x9], #32

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
	fmla	v6.2d, v16.2d, v30.d[0]
	fmla	v7.2d, v17.2d, v30.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	fmla	v6.2d, v18.2d, v30.d[1]
	fmla	v7.2d, v19.2d, v30.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	fmla	v1.2d, v21.2d, v25.d[0]
	fmla	v2.2d, v20.2d, v27.d[0]
	fmla	v3.2d, v21.2d, v27.d[0]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v5.2d, v21.2d, v29.d[0]
	fmla	v6.2d, v20.2d, v31.d[0]
	fmla	v7.2d, v21.2d, v31.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]
	fmla	v1.2d, v23.2d, v25.d[1]
	fmla	v2.2d, v22.2d, v27.d[1]
	fmla	v3.2d, v23.2d, v27.d[1]
	fmla	v4.2d, v22.2d, v29.d[1]
	fmla	v5.2d, v23.2d, v29.d[1]
	fmla	v6.2d, v22.2d, v31.d[1]
	fmla	v7.2d, v23.2d, v31.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldr		d28, [x10], #8
	ldr		d29, [x12], #8
	ldr		d30, [x13], #8
	ldr		d31, [x14], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	add		x9, x9, #32
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	cmp		w8, #0
	fmla	v6.2d, v24.2d, v31.d[0]
	fmla	v7.2d, v25.2d, v31.d[0]

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x12, x10, x11
	add		x13, x12, x11
	add		x14, x13, x11

	// prefetch
	prfm	PLDL1KEEP, [x10, #0]
	prfm	PLDL1KEEP, [x12, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #64]

	// preload
	ldp		q24, q25, [x10], #32
	ldp		q26, q27, [x12], #32
	ldp		q28, q29, [x13], #32
	ldp		q30, q31, [x14], #32
	ldp		q16, q17, [x9, #(0*8+0*32)]

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x10, #0]
//	prfm	PLDL1KEEP, [x12, #0]
//	prfm	PLDL1KEEP, [x13, #0]
//	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x9, #128]
	prfm	PLDL1KEEP, [x9, #192]

	// zero tmp acc
	fmov	d8, xzr
	fmov    d9, d8
	fmov    d10, d8
	fmov    d11, d8
	fmov    d12, d8
	fmov    d13, d8
	fmov    d14, d8
	fmov    d15, d8

//	add		x12, x11, #64
//	add		x12, x11, x11
//	add		x13, x12, #64

	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9, #(0*8+1*32)]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, #128]
	prfm	PLDL1KEEP, [x9, #256]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x9, #320]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
	prfm	PLDL1KEEP, [x10, #32]
	fmla	v6.2d, v16.2d, v30.d[0]
	fmla	v7.2d, v17.2d, v30.d[0]

	// unroll 1
	prfm	PLDL1KEEP, [x12, #32]
	fmla	v8.2d, v18.2d, v24.d[1]
	fmla	v9.2d, v19.2d, v24.d[1]
	ldp		q16, q17, [x9, #(0*8+2*32)]
	fmla	v10.2d, v18.2d, v26.d[1]
	fmla	v11.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x13, #32]
	fmla	v12.2d, v18.2d, v28.d[1]
	fmla	v13.2d, v19.2d, v28.d[1]
	prfm	PLDL1KEEP, [x14, #32]
	fmla	v14.2d, v18.2d, v30.d[1]
	fmla	v15.2d, v19.2d, v30.d[1]

	// unroll 2
	ldp		q18, q19, [x9, #(0*8+3*32)]
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	add		x9, x9, #128
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v31.d[0]
	fmla	v7.2d, v17.2d, v31.d[0]

	// unroll 3
	ldp		q16, q17, [x9, #(0*8+0*32)]
	fmla	v8.2d, v18.2d, v25.d[1]
	fmla	v9.2d, v19.2d, v25.d[1]
	ldp		q24, q25, [x10], #32
	fmla	v10.2d, v18.2d, v27.d[1]
	fmla	v11.2d, v19.2d, v27.d[1]
	ldp		q26, q27, [x12], #32
	fmla	v12.2d, v18.2d, v29.d[1]
	fmla	v13.2d, v19.2d, v29.d[1]
	ldp		q28, q29, [x13], #32
	fmla	v14.2d, v18.2d, v31.d[1]
	fmla	v15.2d, v19.2d, v31.d[1]
	ldp		q30, q31, [x14], #32

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d
	fadd	v1.2d, v1.2d, v9.2d
	fadd	v2.2d, v2.2d, v10.2d
	fadd	v3.2d, v3.2d, v11.2d
	fadd	v4.2d, v4.2d, v12.2d
	fadd	v5.2d, v5.2d, v13.2d
	fadd	v6.2d, v6.2d, v14.2d
	fadd	v7.2d, v7.2d, v15.2d

//	sub		x9, x9, #32
//	sub		x10, x10, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldp		q18, q19, [x9, #(0*8+1*32)]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, #256]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x9, #320]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x10, #256]
	fmla	v6.2d, v16.2d, v30.d[0]
	fmla	v7.2d, v17.2d, v30.d[0]

	// unroll 1
//	prfm	PLDL1KEEP, [x10, #320]
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	ldp		q16, q17, [x9, #(0*8+2*32)]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
//	add		x10, x10, x11
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	sub		w8, w8, #4
	fmla	v6.2d, v18.2d, v30.d[1]
	fmla	v7.2d, v19.2d, v30.d[1]

	// unroll 2
	ldp		q18, q19, [x9, #(0*8+3*32)]
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	add		x9, x9, #128
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v31.d[0]
	fmla	v7.2d, v17.2d, v31.d[0]

	// unroll 3
//	ldp		q16, q17, [x9, #(0*8+0*32)]
	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
//	ldp		q24, q25, [x10, #(0*8+0*32)]
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
//	ldp		q26, q27, [x10, #(0*8+1*32)]
	fmla	v4.2d, v18.2d, v29.d[1]
	fmla	v5.2d, v19.2d, v29.d[1]
//	ldp		q28, q29, [x10, #(0*8+2*32)]
	fmla	v6.2d, v18.2d, v31.d[1]
	fmla	v7.2d, v19.2d, v31.d[1]
//	ldp		q30, q31, [x10, #(0*8+3*32)]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x10, x10, #32
	sub		x12, x12, #32
	sub		x13, x13, #32
	sub		x14, x14, #32

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldr		d28, [x10], #8
	ldr		d29, [x12], #8
	ldr		d30, [x13], #8
	ldr		d31, [x14], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	add		x9, x9, #32
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	cmp		w8, #0
	fmla	v6.2d, v24.2d, v31.d[0]
	fmla	v7.2d, v25.2d, v31.d[0]

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nn_4x4_lib4c)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10   <- B
// x11   <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_4X3_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_nn_4x3_lib4c)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x12, x10, x11
	add		x13, x12, x11

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x10], #32
	ldp		q26, q27, [x12], #32
	ldp		q28, q29, [x13], #32
	ldp		q16, q17, [x9], #32
	ldp		q18, q19, [x9], #32
	ldp		q20, q21, [x9], #32
	ldp		q22, q23, [x9], #32

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	fmla	v1.2d, v21.2d, v25.d[0]
	fmla	v2.2d, v20.2d, v27.d[0]
	fmla	v3.2d, v21.2d, v27.d[0]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v5.2d, v21.2d, v29.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]
	fmla	v1.2d, v23.2d, v25.d[1]
	fmla	v2.2d, v22.2d, v27.d[1]
	fmla	v3.2d, v23.2d, v27.d[1]
	fmla	v4.2d, v22.2d, v29.d[1]
	fmla	v5.2d, v23.2d, v29.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x10], #32
	ldp		q26, q27, [x12], #32
	ldp		q28, q29, [x13], #32
	ldp		q16, q17, [x9], #32
	ldp		q18, q19, [x9], #32
	ldp		q20, q21, [x9], #32
	ldp		q22, q23, [x9], #32

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	fmla	v1.2d, v21.2d, v25.d[0]
	fmla	v2.2d, v20.2d, v27.d[0]
	fmla	v3.2d, v21.2d, v27.d[0]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v5.2d, v21.2d, v29.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]
	fmla	v1.2d, v23.2d, v25.d[1]
	fmla	v2.2d, v22.2d, v27.d[1]
	fmla	v3.2d, v23.2d, v27.d[1]
	fmla	v4.2d, v22.2d, v29.d[1]
	fmla	v5.2d, v23.2d, v29.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldr		d28, [x10], #8
	ldr		d29, [x12], #8
	ldr		d30, [x13], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	add		x9, x9, #32
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x12, x10, x11
	add		x13, x12, x11

	// prefetch
	prfm	PLDL1KEEP, [x10, #0]
	prfm	PLDL1KEEP, [x12, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #64]

	// preload
	ldp		q24, q25, [x10], #32
	ldp		q26, q27, [x12], #32
	ldp		q28, q29, [x13], #32
	ldp		q16, q17, [x9, #(0*8+0*32)]

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x10, #0]
//	prfm	PLDL1KEEP, [x12, #0]
//	prfm	PLDL1KEEP, [x13, #0]
//	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x9, #128]
	prfm	PLDL1KEEP, [x9, #192]

	// zero tmp acc
	fmov	d8, xzr
	fmov    d9, d8
	fmov    d10, d8
	fmov    d11, d8
	fmov    d12, d8
	fmov    d13, d8

	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9, #(0*8+1*32)]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, #128]
	prfm	PLDL1KEEP, [x9, #256]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x9, #320]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
	prfm	PLDL1KEEP, [x10, #32]

	// unroll 1
	prfm	PLDL1KEEP, [x12, #32]
	fmla	v8.2d, v18.2d, v24.d[1]
	fmla	v9.2d, v19.2d, v24.d[1]
	ldp		q16, q17, [x9, #(0*8+2*32)]
	fmla	v10.2d, v18.2d, v26.d[1]
	fmla	v11.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x13, #32]
	fmla	v12.2d, v18.2d, v28.d[1]
	fmla	v13.2d, v19.2d, v28.d[1]
//	prfm	PLDL1KEEP, [x14, #32]

	// unroll 2
	ldp		q18, q19, [x9, #(0*8+3*32)]
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	add		x9, x9, #128
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	cmp		w8, #4

	// unroll 3
	ldp		q16, q17, [x9, #(0*8+0*32)]
	fmla	v8.2d, v18.2d, v25.d[1]
	fmla	v9.2d, v19.2d, v25.d[1]
	ldp		q24, q25, [x10], #32
	fmla	v10.2d, v18.2d, v27.d[1]
	fmla	v11.2d, v19.2d, v27.d[1]
	ldp		q26, q27, [x12], #32
	fmla	v12.2d, v18.2d, v29.d[1]
	fmla	v13.2d, v19.2d, v29.d[1]
	ldp		q28, q29, [x13], #32

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d
	fadd	v1.2d, v1.2d, v9.2d
	fadd	v2.2d, v2.2d, v10.2d
	fadd	v3.2d, v3.2d, v11.2d
	fadd	v4.2d, v4.2d, v12.2d
	fadd	v5.2d, v5.2d, v13.2d

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldp		q18, q19, [x9, #(0*8+1*32)]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, #256]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x9, #320]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x10, #256]

	// unroll 1
//	prfm	PLDL1KEEP, [x10, #320]
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	ldp		q16, q17, [x9, #(0*8+2*32)]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
//	add		x10, x10, x11
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9, #(0*8+3*32)]
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	add		x9, x9, #128
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	cmp		w8, #4

	// unroll 3
//	ldp		q16, q17, [x9, #(0*8+0*32)]
	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
//	ldp		q24, q25, [x10, #(0*8+0*32)]
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
//	ldp		q26, q27, [x10, #(0*8+1*32)]
	fmla	v4.2d, v18.2d, v29.d[1]
	fmla	v5.2d, v19.2d, v29.d[1]
//	ldp		q28, q29, [x10, #(0*8+2*32)]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x10, x10, #32
	sub		x12, x12, #32
	sub		x13, x13, #32

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldr		d28, [x10], #8
	ldr		d29, [x12], #8
	ldr		d30, [x13], #8
//	ldr		d31, [x14], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	add		x9, x9, #32
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nn_4x3_lib4c)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10   <- B
// x11   <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_4X2_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_nn_4x2_lib4c)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x12, x10, x11

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x10], #32
	ldp		q26, q27, [x12], #32
	ldp		q16, q17, [x9], #32
	ldp		q18, q19, [x9], #32
	ldp		q20, q21, [x9], #32
	ldp		q22, q23, [x9], #32

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	fmla	v1.2d, v21.2d, v25.d[0]
	fmla	v2.2d, v20.2d, v27.d[0]
	fmla	v3.2d, v21.2d, v27.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]
	fmla	v1.2d, v23.2d, v25.d[1]
	fmla	v2.2d, v22.2d, v27.d[1]
	fmla	v3.2d, v23.2d, v27.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x10], #32
	ldp		q26, q27, [x12], #32
	ldp		q16, q17, [x9], #32
	ldp		q18, q19, [x9], #32
	ldp		q20, q21, [x9], #32
	ldp		q22, q23, [x9], #32

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	fmla	v1.2d, v21.2d, v25.d[0]
	fmla	v2.2d, v20.2d, v27.d[0]
	fmla	v3.2d, v21.2d, v27.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]
	fmla	v1.2d, v23.2d, v25.d[1]
	fmla	v2.2d, v22.2d, v27.d[1]
	fmla	v3.2d, v23.2d, v27.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldr		d28, [x10], #8
	ldr		d29, [x12], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	add		x9, x9, #32
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x12, x10, x11

	// prefetch
	prfm	PLDL1KEEP, [x10, #0]
	prfm	PLDL1KEEP, [x12, #0]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #64]

	// preload
	ldp		q24, q25, [x10], #32
	ldp		q26, q27, [x12], #32
	ldp		q16, q17, [x9, #(0*8+0*32)]

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x10, #0]
//	prfm	PLDL1KEEP, [x12, #0]
//	prfm	PLDL1KEEP, [x13, #0]
//	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x9, #128]
	prfm	PLDL1KEEP, [x9, #192]

	// zero tmp acc
	fmov	d8, xzr
	fmov    d9, d8
	fmov    d10, d8
	fmov    d11, d8

	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9, #(0*8+1*32)]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, #128]
	prfm	PLDL1KEEP, [x9, #256]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x9, #320]
	prfm	PLDL1KEEP, [x10, #32]

	// unroll 1
	prfm	PLDL1KEEP, [x12, #32]
	fmla	v8.2d, v18.2d, v24.d[1]
	fmla	v9.2d, v19.2d, v24.d[1]
	ldp		q16, q17, [x9, #(0*8+2*32)]
	fmla	v10.2d, v18.2d, v26.d[1]
	fmla	v11.2d, v19.2d, v26.d[1]

	// unroll 2
	ldp		q18, q19, [x9, #(0*8+3*32)]
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	add		x9, x9, #128
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	sub		w8, w8, #4
	cmp		w8, #4

	// unroll 3
	ldp		q16, q17, [x9, #(0*8+0*32)]
	fmla	v8.2d, v18.2d, v25.d[1]
	fmla	v9.2d, v19.2d, v25.d[1]
	ldp		q24, q25, [x10], #32
	fmla	v10.2d, v18.2d, v27.d[1]
	fmla	v11.2d, v19.2d, v27.d[1]
	ldp		q26, q27, [x12], #32

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d
	fadd	v1.2d, v1.2d, v9.2d
	fadd	v2.2d, v2.2d, v10.2d
	fadd	v3.2d, v3.2d, v11.2d

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldp		q18, q19, [x9, #(0*8+1*32)]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, #256]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x9, #320]
//	prfm	PLDL1KEEP, [x10, #256]

	// unroll 1
//	prfm	PLDL1KEEP, [x10, #320]
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	ldp		q16, q17, [x9, #(0*8+2*32)]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
//	add		x10, x10, x11
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9, #(0*8+3*32)]
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	add		x9, x9, #128
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	cmp		w8, #4

	// unroll 3
//	ldp		q16, q17, [x9, #(0*8+0*32)]
	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
//	ldp		q24, q25, [x10, #(0*8+0*32)]
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
//	ldp		q26, q27, [x10, #(0*8+1*32)]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x10, x10, #32
	sub		x12, x12, #32

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldr		d28, [x10], #8
	ldr		d29, [x12], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	add		x9, x9, #32
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nn_4x2_lib4c)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10   <- B
// x11   <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_4X1_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_nn_4x1_lib4c)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x10], #32
	ldp		q16, q17, [x9], #32
	ldp		q18, q19, [x9], #32
	ldp		q20, q21, [x9], #32
	ldp		q22, q23, [x9], #32

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	fmla	v1.2d, v21.2d, v25.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]
	fmla	v1.2d, v23.2d, v25.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x10], #32
	ldp		q16, q17, [x9], #32
	ldp		q18, q19, [x9], #32
	ldp		q20, q21, [x9], #32
	ldp		q22, q23, [x9], #32

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	fmla	v1.2d, v21.2d, v25.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]
	fmla	v1.2d, v23.2d, v25.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldr		d28, [x10], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	add		x9, x9, #32
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	// prefetch
	prfm	PLDL1KEEP, [x10, #0]
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x9, #64]

	// preload
	ldp		q24, q25, [x10], #32
	ldp		q16, q17, [x9, #(0*8+0*32)]

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #128]
	prfm	PLDL1KEEP, [x9, #192]

	// zero tmp acc
	fmov	d8, xzr
	fmov    d9, d8

	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9, #(0*8+1*32)]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, #128]
	prfm	PLDL1KEEP, [x9, #256]
//	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x9, #320]
	prfm	PLDL1KEEP, [x10, #32]

	// unroll 1
	fmla	v8.2d, v18.2d, v24.d[1]
	fmla	v9.2d, v19.2d, v24.d[1]
	ldp		q16, q17, [x9, #(0*8+2*32)]

	// unroll 2
	ldp		q18, q19, [x9, #(0*8+3*32)]
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	add		x9, x9, #128
	sub		w8, w8, #4
	cmp		w8, #4

	// unroll 3
	ldp		q16, q17, [x9, #(0*8+0*32)]
	fmla	v8.2d, v18.2d, v25.d[1]
	fmla	v9.2d, v19.2d, v25.d[1]
	ldp		q24, q25, [x10], #32

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d
	fadd	v1.2d, v1.2d, v9.2d

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldp		q18, q19, [x9, #(0*8+1*32)]
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, #256]
//	prfm	PLDL1KEEP, [x9, #320]
//	prfm	PLDL1KEEP, [x10, #256]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	ldp		q16, q17, [x9, #(0*8+2*32)]
//	add		x10, x10, x11
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9, #(0*8+3*32)]
	fmla	v0.2d, v16.2d, v25.d[0]
	fmla	v1.2d, v17.2d, v25.d[0]
	add		x9, x9, #128
	cmp		w8, #4

	// unroll 3
//	ldp		q16, q17, [x9, #(0*8+0*32)]
	fmla	v0.2d, v18.2d, v25.d[1]
	fmla	v1.2d, v19.2d, v25.d[1]
//	ldp		q24, q25, [x10, #(0*8+0*32)]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x10, x10, #32

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9, #0]
	ldr		d28, [x10], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	add		x9, x9, #32
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nn_4x1_lib4c)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NT_4X4_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nt_4x4_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11]
	add		x11, x11, x12
	ldp		q26, q27, [x11]
	add		x11, x11, x12
	ldp		q28, q29, [x11]
	add		x11, x11, x12
	ldp		q30, q31, [x11]
	add		x11, x11, x12
	ldp		q16, q17, [x9]
	add		x9, x9, x10
	ldp		q18, q19, [x9]
	add		x9, x9, x10
	ldp		q20, q21, [x9]
	add		x9, x9, x10
	ldp		q22, q23, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v3.2d, v21.2d, v28.d[1]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v5.2d, v21.2d, v29.d[0]
	fmla	v6.2d, v20.2d, v29.d[1]
	fmla	v7.2d, v21.2d, v29.d[1]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]
	fmla	v1.2d, v23.2d, v30.d[0]
	fmla	v2.2d, v22.2d, v30.d[1]
	fmla	v3.2d, v23.2d, v30.d[1]
	fmla	v4.2d, v22.2d, v31.d[0]
	fmla	v5.2d, v23.2d, v31.d[0]
	fmla	v6.2d, v22.2d, v31.d[1]
	fmla	v7.2d, v23.2d, v31.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11]
	add		x11, x11, x12
	ldp		q26, q27, [x11]
	add		x11, x11, x12
	ldp		q28, q29, [x11]
	add		x11, x11, x12
	ldp		q30, q31, [x11]
	add		x11, x11, x12
	ldp		q16, q17, [x9]
	add		x9, x9, x10
	ldp		q18, q19, [x9]
	add		x9, x9, x10
	ldp		q20, q21, [x9]
	add		x9, x9, x10
	ldp		q22, q23, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v3.2d, v21.2d, v28.d[1]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v5.2d, v21.2d, v29.d[0]
	fmla	v6.2d, v20.2d, v29.d[1]
	fmla	v7.2d, v21.2d, v29.d[1]

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]
	fmla	v1.2d, v23.2d, v30.d[0]
	fmla	v2.2d, v22.2d, v30.d[1]
	fmla	v3.2d, v23.2d, v30.d[1]
	fmla	v4.2d, v22.2d, v31.d[0]
	fmla	v5.2d, v23.2d, v31.d[0]
	fmla	v6.2d, v22.2d, v31.d[1]
	fmla	v7.2d, v23.2d, v31.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9]
	add		x9, x9, x10
	ldp		q28, q29, [x11]
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v24.2d, v28.d[1]
	fmla	v3.2d, v25.2d, v28.d[1]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v29.d[0]
	fmla	v5.2d, v25.2d, v29.d[0]
	cmp		w8, #0
	fmla	v6.2d, v24.2d, v29.d[1]
	fmla	v7.2d, v25.2d, v29.d[1]

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x12, x12
	add		x14, x13, x12
	
	add		x15, x10, x10

	// prefetch
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x15]

	// preload
	ldp		q24, q25, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q26, q27, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q28, q29, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q30, q31, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q16, q17, [x9]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x11]
//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x13]
//	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x9, x15]

	// zero tmp acc
	fmov	d8, xzr
	fmov    d9, d8
	fmov    d10, d8
	fmov    d11, d8
	fmov    d12, d8
	fmov    d13, d8
	fmov    d14, d8
	fmov    d15, d8

	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	prfm	PLDL1KEEP, [x11]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	prfm	PLDL1KEEP, [x11, x12]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]

	// unroll 1
	ldp		q16, q17, [x9]
	fmla	v8.2d, v18.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v19.2d, v26.d[0]
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v10.2d, v18.2d, v26.d[1]
	fmla	v11.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x11, x13]
	fmla	v12.2d, v18.2d, v27.d[0]
	fmla	v13.2d, v19.2d, v27.d[0]
	prfm	PLDL1KEEP, [x11, x14]
	fmla	v14.2d, v18.2d, v27.d[1]
	sub		w8, w8, #4
	fmla	v15.2d, v19.2d, v27.d[1]

	// unroll 2
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v28.d[0]
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v2.2d, v16.2d, v28.d[1]
	cmp		w8, #4
	fmla	v3.2d, v17.2d, v28.d[1]
	ldp		q24, q25, [x11, #(0*8)]
	fmla	v4.2d, v16.2d, v29.d[0]
	add		x11, x11, x12
	fmla	v5.2d, v17.2d, v29.d[0]
	ldp		q26, q27, [x11, #(0*8)]
	fmla	v6.2d, v16.2d, v29.d[1]
	add		x11, x11, x12
	fmla	v7.2d, v17.2d, v29.d[1]

	// unroll 3
	ldp		q16, q17, [x9]
	fmla	v8.2d, v18.2d, v30.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v19.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v10.2d, v18.2d, v30.d[1]
	ldp		q28, q29, [x11, #(0*8)]
	fmla	v11.2d, v19.2d, v30.d[1]
	add		x11, x11, x12
	fmla	v12.2d, v18.2d, v31.d[0]
	fmla	v13.2d, v19.2d, v31.d[0]
	fmla	v14.2d, v18.2d, v31.d[1]
	fmla	v15.2d, v19.2d, v31.d[1]
	ldp		q30, q31, [x11, #(0*8)]
	add		x11, x11, x12

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d
	fadd	v1.2d, v1.2d, v9.2d
	fadd	v2.2d, v2.2d, v10.2d
	fadd	v3.2d, v3.2d, v11.2d
	fadd	v4.2d, v4.2d, v12.2d
	fadd	v5.2d, v5.2d, v13.2d
	fadd	v6.2d, v6.2d, v14.2d
	fadd	v7.2d, v7.2d, v15.2d

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x9, #192]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
//	prfm	PLDL1KEEP, [x11, #128]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]

	// unroll 1
//	prfm	PLDL1KEEP, [x11, #192]
	ldp		q16, q17, [x9]
	fmla	v0.2d, v18.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	sub		w8, w8, #4
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]

	// unroll 2
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v28.d[0]
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
//	cmp		w8, #4
	fmla	v6.2d, v16.2d, v29.d[1]
	fmla	v7.2d, v17.2d, v29.d[1]

	// unroll 3
//	ldp		q16, q17, [x9]
	fmla	v0.2d, v18.2d, v30.d[0]
//	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v30.d[0]
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v2.2d, v18.2d, v30.d[1]
//	add		x11, x11, x12
	fmla	v3.2d, v19.2d, v30.d[1]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
	fmla	v4.2d, v18.2d, v31.d[0]
//	add		x11, x11, x12
	fmla	v5.2d, v19.2d, v31.d[0]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
	fmla	v6.2d, v18.2d, v31.d[1]
//	add		x11, x11, x12
	fmla	v7.2d, v19.2d, v31.d[1]
//	ldp		q30, q31, [x11, #(0*8+3*32)]
//	add		x11, x11, x12

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x11, x11, x12
//	sub		x11, x11, x12
//	sub		x11, x11, x12
//	sub		x11, x11, x12
	sub		x11, x11, x12, lsl #2
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9]
	add		x9, x9, x10
	ldp		q28, q29, [x11]
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v24.2d, v28.d[1]
	fmla	v3.2d, v25.2d, v28.d[1]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v29.d[0]
	fmla	v5.2d, v25.2d, v29.d[0]
	cmp		w8, #0
	fmla	v6.2d, v24.2d, v29.d[1]
	fmla	v7.2d, v25.2d, v29.d[1]

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nt_4x4_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NT_3X4_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nt_3x4_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11]
	add		x11, x11, x12
	ldp		q26, q27, [x11]
	add		x11, x11, x12
	ldp		q28, q29, [x11]
	add		x11, x11, x12
	ldp		q30, q31, [x11]
	add		x11, x11, x12
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	add		x9, x9, x10
	ldr		q20, [x9]
	ldr		d21, [x9, #16]
	add		x9, x9, x10
	ldr		q22, [x9]
	ldr		d23, [x9, #16]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v3.2d, v21.2d, v28.d[1]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v5.2d, v21.2d, v29.d[0]
	fmla	v6.2d, v20.2d, v29.d[1]
	fmla	v7.2d, v21.2d, v29.d[1]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]
	fmla	v1.2d, v23.2d, v30.d[0]
	fmla	v2.2d, v22.2d, v30.d[1]
	fmla	v3.2d, v23.2d, v30.d[1]
	fmla	v4.2d, v22.2d, v31.d[0]
	fmla	v5.2d, v23.2d, v31.d[0]
	fmla	v6.2d, v22.2d, v31.d[1]
	fmla	v7.2d, v23.2d, v31.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11]
	add		x11, x11, x12
	ldp		q26, q27, [x11]
	add		x11, x11, x12
	ldp		q28, q29, [x11]
	add		x11, x11, x12
	ldp		q30, q31, [x11]
	add		x11, x11, x12
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	add		x9, x9, x10
	ldr		q20, [x9]
	ldr		d21, [x9, #16]
	add		x9, x9, x10
	ldr		q22, [x9]
	ldr		d23, [x9, #16]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v3.2d, v21.2d, v28.d[1]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v5.2d, v21.2d, v29.d[0]
	fmla	v6.2d, v20.2d, v29.d[1]
	fmla	v7.2d, v21.2d, v29.d[1]

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]
	fmla	v1.2d, v23.2d, v30.d[0]
	fmla	v2.2d, v22.2d, v30.d[1]
	fmla	v3.2d, v23.2d, v30.d[1]
	fmla	v4.2d, v22.2d, v31.d[0]
	fmla	v5.2d, v23.2d, v31.d[0]
	fmla	v6.2d, v22.2d, v31.d[1]
	fmla	v7.2d, v23.2d, v31.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9]
	ldr		d25, [x9, #16]
	add		x9, x9, x10
	ldp		q28, q29, [x11]
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v24.2d, v28.d[1]
	fmla	v3.2d, v25.2d, v28.d[1]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v29.d[0]
	fmla	v5.2d, v25.2d, v29.d[0]
	cmp		w8, #0
	fmla	v6.2d, v24.2d, v29.d[1]
	fmla	v7.2d, v25.2d, v29.d[1]

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x12, x12
	add		x14, x13, x12
	
	add		x15, x10, x10

	// prefetch
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x15]

	// preload
	ldp		q24, q25, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q26, q27, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q28, q29, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q30, q31, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x11]
//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x13]
//	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x9, x15]

	// zero tmp acc
	fmov	d8, xzr
	fmov    d9, d8
	fmov    d10, d8
	fmov    d11, d8
	fmov    d12, d8
	fmov    d13, d8
	fmov    d14, d8
	fmov    d15, d8

	// main loop
1:
	
	// unroll 0
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	prfm	PLDL1KEEP, [x11]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	prfm	PLDL1KEEP, [x11, x12]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]

	// unroll 1
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v8.2d, v18.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v19.2d, v26.d[0]
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v10.2d, v18.2d, v26.d[1]
	fmla	v11.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x11, x13]
	fmla	v12.2d, v18.2d, v27.d[0]
	fmla	v13.2d, v19.2d, v27.d[0]
	prfm	PLDL1KEEP, [x11, x14]
	fmla	v14.2d, v18.2d, v27.d[1]
	sub		w8, w8, #4
	fmla	v15.2d, v19.2d, v27.d[1]

	// unroll 2
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v28.d[0]
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v2.2d, v16.2d, v28.d[1]
	cmp		w8, #4
	fmla	v3.2d, v17.2d, v28.d[1]
	ldp		q24, q25, [x11, #(0*8)]
	fmla	v4.2d, v16.2d, v29.d[0]
	add		x11, x11, x12
	fmla	v5.2d, v17.2d, v29.d[0]
	ldp		q26, q27, [x11, #(0*8)]
	fmla	v6.2d, v16.2d, v29.d[1]
	add		x11, x11, x12
	fmla	v7.2d, v17.2d, v29.d[1]

	// unroll 3
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v8.2d, v18.2d, v30.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v19.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v10.2d, v18.2d, v30.d[1]
	ldp		q28, q29, [x11, #(0*8)]
	fmla	v11.2d, v19.2d, v30.d[1]
	add		x11, x11, x12
	fmla	v12.2d, v18.2d, v31.d[0]
	fmla	v13.2d, v19.2d, v31.d[0]
	fmla	v14.2d, v18.2d, v31.d[1]
	fmla	v15.2d, v19.2d, v31.d[1]
	ldp		q30, q31, [x11, #(0*8)]
	add		x11, x11, x12

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d
	fadd	v1.2d, v1.2d, v9.2d
	fadd	v2.2d, v2.2d, v10.2d
	fadd	v3.2d, v3.2d, v11.2d
	fadd	v4.2d, v4.2d, v12.2d
	fadd	v5.2d, v5.2d, v13.2d
	fadd	v6.2d, v6.2d, v14.2d
	fadd	v7.2d, v7.2d, v15.2d

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x9, #192]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
//	prfm	PLDL1KEEP, [x11, #128]
	fmla	v6.2d, v16.2d, v25.d[1]
	fmla	v7.2d, v17.2d, v25.d[1]

	// unroll 1
//	prfm	PLDL1KEEP, [x11, #192]
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v0.2d, v18.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	sub		w8, w8, #4
	fmla	v6.2d, v18.2d, v27.d[1]
	fmla	v7.2d, v19.2d, v27.d[1]

	// unroll 2
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v28.d[0]
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
//	cmp		w8, #4
	fmla	v6.2d, v16.2d, v29.d[1]
	fmla	v7.2d, v17.2d, v29.d[1]

	// unroll 3
//	ldr		q16, [x9]
//	ldr		d17, [x9, #16]
	fmla	v0.2d, v18.2d, v30.d[0]
//	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v30.d[0]
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v2.2d, v18.2d, v30.d[1]
//	add		x11, x11, x12
	fmla	v3.2d, v19.2d, v30.d[1]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
	fmla	v4.2d, v18.2d, v31.d[0]
//	add		x11, x11, x12
	fmla	v5.2d, v19.2d, v31.d[0]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
	fmla	v6.2d, v18.2d, v31.d[1]
//	add		x11, x11, x12
	fmla	v7.2d, v19.2d, v31.d[1]
//	ldp		q30, q31, [x11, #(0*8+3*32)]
//	add		x11, x11, x12

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x11, x11, x12
//	sub		x11, x11, x12
//	sub		x11, x11, x12
//	sub		x11, x11, x12
	sub		x11, x11, x12, lsl #2
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9]
	ldr		d25, [x9, #16]
	add		x9, x9, x10
	ldp		q28, q29, [x11]
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v24.2d, v28.d[1]
	fmla	v3.2d, v25.2d, v28.d[1]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v29.d[0]
	fmla	v5.2d, v25.2d, v29.d[0]
	cmp		w8, #0
	fmla	v6.2d, v24.2d, v29.d[1]
	fmla	v7.2d, v25.2d, v29.d[1]

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nt_3x4_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NT_2X4_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nt_2x4_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11]
	add		x11, x11, x12
	ldp		q26, q27, [x11]
	add		x11, x11, x12
	ldp		q28, q29, [x11]
	add		x11, x11, x12
	ldp		q30, q31, [x11]
	add		x11, x11, x12
	ldr		q16, [x9]
	add		x9, x9, x10
	ldr		q18, [x9]
	add		x9, x9, x10
	ldr		q20, [x9]
	add		x9, x9, x10
	ldr		q22, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v6.2d, v18.2d, v27.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v6.2d, v20.2d, v29.d[1]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]
	fmla	v2.2d, v22.2d, v30.d[1]
	fmla	v4.2d, v22.2d, v31.d[0]
	fmla	v6.2d, v22.2d, v31.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11]
	add		x11, x11, x12
	ldp		q26, q27, [x11]
	add		x11, x11, x12
	ldp		q28, q29, [x11]
	add		x11, x11, x12
	ldp		q30, q31, [x11]
	add		x11, x11, x12
	ldr		q16, [x9]
	add		x9, x9, x10
	ldr		q18, [x9]
	add		x9, x9, x10
	ldr		q20, [x9]
	add		x9, x9, x10
	ldr		q22, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v6.2d, v18.2d, v27.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v6.2d, v20.2d, v29.d[1]

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]
	fmla	v2.2d, v22.2d, v30.d[1]
	fmla	v4.2d, v22.2d, v31.d[0]
	fmla	v6.2d, v22.2d, v31.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9]
	add		x9, x9, x10
	ldp		q28, q29, [x11]
	fmla	v0.2d, v24.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v24.2d, v28.d[1]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v29.d[0]
	cmp		w8, #0
	fmla	v6.2d, v24.2d, v29.d[1]

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x12, x12
	add		x14, x13, x12
	
	add		x15, x10, x10

	// prefetch
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x15]

	// preload
	ldp		q24, q25, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q26, q27, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q28, q29, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q30, q31, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		q16, [x9]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x11]
//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x13]
//	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x9, x15]

	// zero tmp acc
	fmov	d8, xzr
	fmov    d10, d8
	fmov    d12, d8
	fmov    d14, d8

	// main loop
1:
	
	// unroll 0
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v2.2d, v16.2d, v24.d[1]
	prfm	PLDL1KEEP, [x11]
	fmla	v4.2d, v16.2d, v25.d[0]
	prfm	PLDL1KEEP, [x11, x12]
	fmla	v6.2d, v16.2d, v25.d[1]

	// unroll 1
	ldr		q16, [x9]
	fmla	v8.2d, v18.2d, v26.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v10.2d, v18.2d, v26.d[1]
	prfm	PLDL1KEEP, [x11, x13]
	fmla	v12.2d, v18.2d, v27.d[0]
	prfm	PLDL1KEEP, [x11, x14]
	fmla	v14.2d, v18.2d, v27.d[1]
	sub		w8, w8, #4

	// unroll 2
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v28.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v2.2d, v16.2d, v28.d[1]
	cmp		w8, #4
	ldp		q24, q25, [x11, #(0*8)]
	fmla	v4.2d, v16.2d, v29.d[0]
	add		x11, x11, x12
	ldp		q26, q27, [x11, #(0*8)]
	fmla	v6.2d, v16.2d, v29.d[1]
	add		x11, x11, x12

	// unroll 3
	ldr		q16, [x9]
	fmla	v8.2d, v18.2d, v30.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v10.2d, v18.2d, v30.d[1]
	ldp		q28, q29, [x11, #(0*8)]
	add		x11, x11, x12
	fmla	v12.2d, v18.2d, v31.d[0]
	fmla	v14.2d, v18.2d, v31.d[1]
	ldp		q30, q31, [x11, #(0*8)]
	add		x11, x11, x12

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d
	fadd	v2.2d, v2.2d, v10.2d
	fadd	v4.2d, v4.2d, v12.2d
	fadd	v6.2d, v6.2d, v14.2d

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v2.2d, v16.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x9, #192]
	fmla	v4.2d, v16.2d, v25.d[0]
//	prfm	PLDL1KEEP, [x11, #128]
	fmla	v6.2d, v16.2d, v25.d[1]

	// unroll 1
//	prfm	PLDL1KEEP, [x11, #192]
	ldr		q16, [x9]
	fmla	v0.2d, v18.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	sub		w8, w8, #4
	fmla	v6.2d, v18.2d, v27.d[1]

	// unroll 2
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v4.2d, v16.2d, v29.d[0]
//	cmp		w8, #4
	fmla	v6.2d, v16.2d, v29.d[1]

	// unroll 3
//	ldr		q16, [x9]
	fmla	v0.2d, v18.2d, v30.d[0]
//	add		x9, x9, x10
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v2.2d, v18.2d, v30.d[1]
//	add		x11, x11, x12
//	ldp		q26, q27, [x11, #(0*8+1*32)]
	fmla	v4.2d, v18.2d, v31.d[0]
//	add		x11, x11, x12
//	ldp		q28, q29, [x11, #(0*8+2*32)]
	fmla	v6.2d, v18.2d, v31.d[1]
//	add		x11, x11, x12
//	ldp		q30, q31, [x11, #(0*8+3*32)]
//	add		x11, x11, x12

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x11, x11, x12
//	sub		x11, x11, x12
//	sub		x11, x11, x12
//	sub		x11, x11, x12
	sub		x11, x11, x12, lsl #2
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9]
	add		x9, x9, x10
	ldp		q28, q29, [x11]
	fmla	v0.2d, v24.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v24.2d, v28.d[1]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v29.d[0]
	cmp		w8, #0
	fmla	v6.2d, v24.2d, v29.d[1]

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nt_2x4_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NT_1X4_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nt_1x4_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11]
	add		x11, x11, x12
	ldp		q26, q27, [x11]
	add		x11, x11, x12
	ldp		q28, q29, [x11]
	add		x11, x11, x12
	ldp		q30, q31, [x11]
	add		x11, x11, x12
	ldr		d16, [x9]
	add		x9, x9, x10
	ldr		d18, [x9]
	add		x9, x9, x10
	ldr		d20, [x9]
	add		x9, x9, x10
	ldr		d22, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v6.2d, v18.2d, v27.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v6.2d, v20.2d, v29.d[1]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]
	fmla	v2.2d, v22.2d, v30.d[1]
	fmla	v4.2d, v22.2d, v31.d[0]
	fmla	v6.2d, v22.2d, v31.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11]
	add		x11, x11, x12
	ldp		q26, q27, [x11]
	add		x11, x11, x12
	ldp		q28, q29, [x11]
	add		x11, x11, x12
	ldp		q30, q31, [x11]
	add		x11, x11, x12
	ldr		d16, [x9]
	add		x9, x9, x10
	ldr		d18, [x9]
	add		x9, x9, x10
	ldr		d20, [x9]
	add		x9, x9, x10
	ldr		d22, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v6.2d, v16.2d, v25.d[1]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v6.2d, v18.2d, v27.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v6.2d, v20.2d, v29.d[1]

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]
	fmla	v2.2d, v22.2d, v30.d[1]
	fmla	v4.2d, v22.2d, v31.d[0]
	fmla	v6.2d, v22.2d, v31.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		d24, [x9]
	add		x9, x9, x10
	ldp		q28, q29, [x11]
	fmla	v0.2d, v24.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v24.2d, v28.d[1]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v29.d[0]
	cmp		w8, #0
	fmla	v6.2d, v24.2d, v29.d[1]

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x12, x12
	add		x14, x13, x12
	
	add		x15, x10, x10

	// prefetch
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x15]

	// preload
	ldp		q24, q25, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q26, q27, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q28, q29, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q30, q31, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		d16, [x9]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x11]
//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x13]
//	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x9, x15]

	// zero tmp acc
	fmov	d8, xzr
	fmov    d9, d8
	fmov    d10, d8
	fmov    d11, d8
	fmov    d12, d8
	fmov    d13, d8
	fmov    d14, d8
	fmov    d15, d8

	// main loop
1:
	
	// unroll 0
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v2.2d, v16.2d, v24.d[1]
	prfm	PLDL1KEEP, [x11]
	fmla	v4.2d, v16.2d, v25.d[0]
	prfm	PLDL1KEEP, [x11, x12]
	fmla	v6.2d, v16.2d, v25.d[1]

	// unroll 1
	ldr		d16, [x9]
	fmla	v8.2d, v18.2d, v26.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v10.2d, v18.2d, v26.d[1]
	prfm	PLDL1KEEP, [x11, x13]
	fmla	v12.2d, v18.2d, v27.d[0]
	prfm	PLDL1KEEP, [x11, x14]
	fmla	v14.2d, v18.2d, v27.d[1]
	sub		w8, w8, #4

	// unroll 2
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v28.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v2.2d, v16.2d, v28.d[1]
	cmp		w8, #4
	ldp		q24, q25, [x11, #(0*8)]
	fmla	v4.2d, v16.2d, v29.d[0]
	add		x11, x11, x12
	ldp		q26, q27, [x11, #(0*8)]
	fmla	v6.2d, v16.2d, v29.d[1]
	add		x11, x11, x12

	// unroll 3
	ldr		d16, [x9]
	fmla	v8.2d, v18.2d, v30.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v10.2d, v18.2d, v30.d[1]
	ldp		q28, q29, [x11, #(0*8)]
	add		x11, x11, x12
	fmla	v12.2d, v18.2d, v31.d[0]
	fmla	v14.2d, v18.2d, v31.d[1]
	ldp		q30, q31, [x11, #(0*8)]
	add		x11, x11, x12

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d
	fadd	v1.2d, v1.2d, v9.2d
	fadd	v2.2d, v2.2d, v10.2d
	fadd	v3.2d, v3.2d, v11.2d
	fadd	v4.2d, v4.2d, v12.2d
	fadd	v5.2d, v5.2d, v13.2d
	fadd	v6.2d, v6.2d, v14.2d
	fadd	v7.2d, v7.2d, v15.2d

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v2.2d, v16.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x9, #192]
	fmla	v4.2d, v16.2d, v25.d[0]
//	prfm	PLDL1KEEP, [x11, #128]
	fmla	v6.2d, v16.2d, v25.d[1]

	// unroll 1
//	prfm	PLDL1KEEP, [x11, #192]
	ldr		d16, [x9]
	fmla	v0.2d, v18.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	sub		w8, w8, #4
	fmla	v6.2d, v18.2d, v27.d[1]

	// unroll 2
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v4.2d, v16.2d, v29.d[0]
//	cmp		w8, #4
	fmla	v6.2d, v16.2d, v29.d[1]

	// unroll 3
//	ldr		d16, [x9]
	fmla	v0.2d, v18.2d, v30.d[0]
//	add		x9, x9, x10
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v2.2d, v18.2d, v30.d[1]
//	add		x11, x11, x12
//	ldp		q26, q27, [x11, #(0*8+1*32)]
	fmla	v4.2d, v18.2d, v31.d[0]
//	add		x11, x11, x12
//	ldp		q28, q29, [x11, #(0*8+2*32)]
	fmla	v6.2d, v18.2d, v31.d[1]
//	add		x11, x11, x12
//	ldp		q30, q31, [x11, #(0*8+3*32)]
//	add		x11, x11, x12

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x11, x11, x12
//	sub		x11, x11, x12
//	sub		x11, x11, x12
//	sub		x11, x11, x12
	sub		x11, x11, x12, lsl #2
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldr		d24, [x9]
	add		x9, x9, x10
	ldp		q28, q29, [x11]
	fmla	v0.2d, v24.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v24.2d, v28.d[1]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v29.d[0]
	cmp		w8, #0
	fmla	v6.2d, v24.2d, v29.d[1]

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nt_1x4_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NT_4X3_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nt_4x3_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldr		q24, [x11, #0]
	ldr		d25, [x11, #16]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	ldr		d27, [x11, #16]
	add		x11, x11, x12
	ldr		q28, [x11, #0]
	ldr		d29, [x11, #16]
	add		x11, x11, x12
	ldr		q30, [x11, #0]
	ldr		d31, [x11, #16]
	add		x11, x11, x12
	ldp		q16, q17, [x9]
	add		x9, x9, x10
	ldp		q18, q19, [x9]
	add		x9, x9, x10
	ldp		q20, q21, [x9]
	add		x9, x9, x10
	ldp		q22, q23, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v3.2d, v21.2d, v28.d[1]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v5.2d, v21.2d, v29.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]
	fmla	v1.2d, v23.2d, v30.d[0]
	fmla	v2.2d, v22.2d, v30.d[1]
	fmla	v3.2d, v23.2d, v30.d[1]
	fmla	v4.2d, v22.2d, v31.d[0]
	fmla	v5.2d, v23.2d, v31.d[0]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldr		q24, [x11, #0]
	ldr		d25, [x11, #16]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	ldr		d27, [x11, #16]
	add		x11, x11, x12
	ldr		q28, [x11, #0]
	ldr		d29, [x11, #16]
	add		x11, x11, x12
	ldr		q30, [x11, #0]
	ldr		d31, [x11, #16]
	add		x11, x11, x12
	ldp		q16, q17, [x9]
	add		x9, x9, x10
	ldp		q18, q19, [x9]
	add		x9, x9, x10
	ldp		q20, q21, [x9]
	add		x9, x9, x10
	ldp		q22, q23, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v3.2d, v21.2d, v28.d[1]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v5.2d, v21.2d, v29.d[0]

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]
	fmla	v1.2d, v23.2d, v30.d[0]
	fmla	v2.2d, v22.2d, v30.d[1]
	fmla	v3.2d, v23.2d, v30.d[1]
	fmla	v4.2d, v22.2d, v31.d[0]
	fmla	v5.2d, v23.2d, v31.d[0]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9]
	add		x9, x9, x10
	ldr		q28, [x11, #0]
	ldr		d29, [x11, #16]
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v24.2d, v28.d[1]
	fmla	v3.2d, v25.2d, v28.d[1]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v29.d[0]
	fmla	v5.2d, v25.2d, v29.d[0]
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)


	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x12, x12
	add		x14, x13, x12
	
	add		x15, x10, x10

	// prefetch
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x15]

	// preload
	ldr		q24, [x11, #(0*8)]
	ldr		d25, [x11, #(2*8)]
	add		x11, x11, x12
	ldr		q26, [x11, #(0*8)]
	ldr		d27, [x11, #(2*8)]
	add		x11, x11, x12
	ldr		q28, [x11, #(0*8)]
	ldr		d29, [x11, #(2*8)]
	add		x11, x11, x12
	ldr		q30, [x11, #(0*8)]
	ldr		d31, [x11, #(2*8)]
	add		x11, x11, x12
	ldp		q16, q17, [x9]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x11]
//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x13]
//	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x9, x15]

	// zero tmp acc
	fmov	d8, xzr
	fmov    d9, d8
	fmov    d10, d8
	fmov    d11, d8
	fmov    d12, d8
	fmov    d13, d8

	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	prfm	PLDL1KEEP, [x11]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	prfm	PLDL1KEEP, [x11, x12]

	// unroll 1
	ldp		q16, q17, [x9]
	fmla	v8.2d, v18.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v19.2d, v26.d[0]
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v10.2d, v18.2d, v26.d[1]
	fmla	v11.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x11, x13]
	fmla	v12.2d, v18.2d, v27.d[0]
	fmla	v13.2d, v19.2d, v27.d[0]
	prfm	PLDL1KEEP, [x11, x14]
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v28.d[0]
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v2.2d, v16.2d, v28.d[1]
	cmp		w8, #4
	fmla	v3.2d, v17.2d, v28.d[1]
	ldr		q24, [x11, #(0*8)]
	ldr		d25, [x11, #(2*8)]
	fmla	v4.2d, v16.2d, v29.d[0]
	add		x11, x11, x12
	fmla	v5.2d, v17.2d, v29.d[0]
	ldr		q26, [x11, #(0*8)]
	ldr		d27, [x11, #(2*8)]
	add		x11, x11, x12

	// unroll 3
	ldp		q16, q17, [x9]
	fmla	v8.2d, v18.2d, v30.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v19.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v10.2d, v18.2d, v30.d[1]
	ldr		q28, [x11, #(0*8)]
	ldr		d29, [x11, #(2*8)]
	fmla	v11.2d, v19.2d, v30.d[1]
	add		x11, x11, x12
	fmla	v12.2d, v18.2d, v31.d[0]
	fmla	v13.2d, v19.2d, v31.d[0]
	ldr		q30, [x11, #(0*8)]
	ldr		d31, [x11, #(2*8)]
	add		x11, x11, x12

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d
	fadd	v1.2d, v1.2d, v9.2d
	fadd	v2.2d, v2.2d, v10.2d
	fadd	v3.2d, v3.2d, v11.2d
	fadd	v4.2d, v4.2d, v12.2d
	fadd	v5.2d, v5.2d, v13.2d

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x9, #192]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
//	prfm	PLDL1KEEP, [x11, #128]

	// unroll 1
//	prfm	PLDL1KEEP, [x11, #192]
	ldp		q16, q17, [x9]
	fmla	v0.2d, v18.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v28.d[0]
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
//	cmp		w8, #4

	// unroll 3
//	ldp		q16, q17, [x9]
	fmla	v0.2d, v18.2d, v30.d[0]
//	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v30.d[0]
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v2.2d, v18.2d, v30.d[1]
//	add		x11, x11, x12
	fmla	v3.2d, v19.2d, v30.d[1]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
	fmla	v4.2d, v18.2d, v31.d[0]
//	add		x11, x11, x12
	fmla	v5.2d, v19.2d, v31.d[0]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	add		x11, x11, x12
//	ldr		q30, [x11, #(0*8)]
//	ldr		d31, [x11, #(2*8)]
//	add		x11, x11, x12

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x11, x11, x12
//	sub		x11, x11, x12
//	sub		x11, x11, x12
//	sub		x11, x11, x12
	sub		x11, x11, x12, lsl #2
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9]
	add		x9, x9, x10
	ldr		q28, [x11, #(0*8)]
	ldr		d29, [x11, #(2*8)]
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v24.2d, v28.d[1]
	fmla	v3.2d, v25.2d, v28.d[1]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v29.d[0]
	fmla	v5.2d, v25.2d, v29.d[0]
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nt_4x3_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NT_3X3_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nt_3x3_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldr		q24, [x11, #0]
	ldr		d25, [x11, #16]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	ldr		d27, [x11, #16]
	add		x11, x11, x12
	ldr		q28, [x11, #0]
	ldr		d29, [x11, #16]
	add		x11, x11, x12
	ldr		q30, [x11, #0]
	ldr		d31, [x11, #16]
	add		x11, x11, x12
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	add		x9, x9, x10
	ldr		q20, [x9]
	ldr		d21, [x9, #16]
	add		x9, x9, x10
	ldr		q22, [x9]
	ldr		d23, [x9, #16]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v3.2d, v21.2d, v28.d[1]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v5.2d, v21.2d, v29.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]
	fmla	v1.2d, v23.2d, v30.d[0]
	fmla	v2.2d, v22.2d, v30.d[1]
	fmla	v3.2d, v23.2d, v30.d[1]
	fmla	v4.2d, v22.2d, v31.d[0]
	fmla	v5.2d, v23.2d, v31.d[0]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldr		q24, [x11, #0]
	ldr		d25, [x11, #16]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	ldr		d27, [x11, #16]
	add		x11, x11, x12
	ldr		q28, [x11, #0]
	ldr		d29, [x11, #16]
	add		x11, x11, x12
	ldr		q30, [x11, #0]
	ldr		d31, [x11, #16]
	add		x11, x11, x12
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	add		x9, x9, x10
	ldr		q20, [x9]
	ldr		d21, [x9, #16]
	add		x9, x9, x10
	ldr		q22, [x9]
	ldr		d23, [x9, #16]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v3.2d, v21.2d, v28.d[1]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v5.2d, v21.2d, v29.d[0]

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]
	fmla	v1.2d, v23.2d, v30.d[0]
	fmla	v2.2d, v22.2d, v30.d[1]
	fmla	v3.2d, v23.2d, v30.d[1]
	fmla	v4.2d, v22.2d, v31.d[0]
	fmla	v5.2d, v23.2d, v31.d[0]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9]
	ldr		d25, [x9, #16]
	add		x9, x9, x10
	ldr		q28, [x11, #0]
	ldr		d29, [x11, #16]
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v24.2d, v28.d[1]
	fmla	v3.2d, v25.2d, v28.d[1]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v29.d[0]
	fmla	v5.2d, v25.2d, v29.d[0]
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x12, x12
	add		x14, x13, x12
	
	add		x15, x10, x10

	// prefetch
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x15]

	// preload
	ldr		q24, [x11, #(0*8)]
	ldr		d25, [x11, #(2*8)]
	add		x11, x11, x12
	ldr		q26, [x11, #(0*8)]
	ldr		d27, [x11, #(2*8)]
	add		x11, x11, x12
	ldr		q28, [x11, #(0*8)]
	ldr		d29, [x11, #(2*8)]
	add		x11, x11, x12
	ldr		q30, [x11, #(0*8)]
	ldr		d31, [x11, #(2*8)]
	add		x11, x11, x12
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x11]
//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x13]
//	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x9, x15]

	// zero tmp acc
	fmov	d8, xzr
	fmov    d9, d8
	fmov    d10, d8
	fmov    d11, d8
	fmov    d12, d8
	fmov    d13, d8

	// main loop
1:
	
	// unroll 0
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	prfm	PLDL1KEEP, [x11]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
	prfm	PLDL1KEEP, [x11, x12]

	// unroll 1
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v8.2d, v18.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v19.2d, v26.d[0]
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v10.2d, v18.2d, v26.d[1]
	fmla	v11.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x11, x13]
	fmla	v12.2d, v18.2d, v27.d[0]
	fmla	v13.2d, v19.2d, v27.d[0]
	prfm	PLDL1KEEP, [x11, x14]
	sub		w8, w8, #4

	// unroll 2
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v28.d[0]
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v2.2d, v16.2d, v28.d[1]
	cmp		w8, #4
	fmla	v3.2d, v17.2d, v28.d[1]
	ldr		q24, [x11, #(0*8)]
	ldr		d25, [x11, #(2*8)]
	fmla	v4.2d, v16.2d, v29.d[0]
	add		x11, x11, x12
	fmla	v5.2d, v17.2d, v29.d[0]
	ldr		q26, [x11, #(0*8)]
	ldr		d27, [x11, #(2*8)]
	add		x11, x11, x12

	// unroll 3
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v8.2d, v18.2d, v30.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v19.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v10.2d, v18.2d, v30.d[1]
	ldr		q28, [x11, #(0*8)]
	ldr		d29, [x11, #(2*8)]
	fmla	v11.2d, v19.2d, v30.d[1]
	add		x11, x11, x12
	fmla	v12.2d, v18.2d, v31.d[0]
	fmla	v13.2d, v19.2d, v31.d[0]
	ldr		q30, [x11, #(0*8)]
	ldr		d31, [x11, #(2*8)]
	add		x11, x11, x12

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d
	fadd	v1.2d, v1.2d, v9.2d
	fadd	v2.2d, v2.2d, v10.2d
	fadd	v3.2d, v3.2d, v11.2d
	fadd	v4.2d, v4.2d, v12.2d
	fadd	v5.2d, v5.2d, v13.2d

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x9, #192]
	fmla	v4.2d, v16.2d, v25.d[0]
	fmla	v5.2d, v17.2d, v25.d[0]
//	prfm	PLDL1KEEP, [x11, #128]

	// unroll 1
//	prfm	PLDL1KEEP, [x11, #192]
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v0.2d, v18.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	fmla	v5.2d, v19.2d, v27.d[0]
	sub		w8, w8, #4

	// unroll 2
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v28.d[0]
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
//	cmp		w8, #4

	// unroll 3
//	ldr		q16, [x9]
//	ldr		d17, [x9, #16]
	fmla	v0.2d, v18.2d, v30.d[0]
//	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v30.d[0]
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v2.2d, v18.2d, v30.d[1]
//	add		x11, x11, x12
	fmla	v3.2d, v19.2d, v30.d[1]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
	fmla	v4.2d, v18.2d, v31.d[0]
//	add		x11, x11, x12
	fmla	v5.2d, v19.2d, v31.d[0]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	add		x11, x11, x12
//	ldr		q30, [x11, #(0*8)]
//	ldr		d31, [x11, #(2*8)]
//	add		x11, x11, x12

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x11, x11, x12
//	sub		x11, x11, x12
//	sub		x11, x11, x12
//	sub		x11, x11, x12
	sub		x11, x11, x12, lsl #2
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9]
	ldr		d25, [x9, #16]
	add		x9, x9, x10
	ldr		q28, [x11, #(0*8)]
	ldr		d29, [x11, #(2*8)]
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v24.2d, v28.d[1]
	fmla	v3.2d, v25.2d, v28.d[1]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v29.d[0]
	fmla	v5.2d, v25.2d, v29.d[0]
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nt_3x3_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NT_2X3_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nt_2x3_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldr		q24, [x11, #0]
	ldr		d25, [x11, #16]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	ldr		d27, [x11, #16]
	add		x11, x11, x12
	ldr		q28, [x11, #0]
	ldr		d29, [x11, #16]
	add		x11, x11, x12
	ldr		q30, [x11, #0]
	ldr		d31, [x11, #16]
	add		x11, x11, x12
	ldr		q16, [x9]
	add		x9, x9, x10
	ldr		q18, [x9]
	add		x9, x9, x10
	ldr		q20, [x9]
	add		x9, x9, x10
	ldr		q22, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v4.2d, v20.2d, v29.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]
	fmla	v2.2d, v22.2d, v30.d[1]
	fmla	v4.2d, v22.2d, v31.d[0]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldr		q24, [x11, #0]
	ldr		d25, [x11, #16]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	ldr		d27, [x11, #16]
	add		x11, x11, x12
	ldr		q28, [x11, #0]
	ldr		d29, [x11, #16]
	add		x11, x11, x12
	ldr		q30, [x11, #0]
	ldr		d31, [x11, #16]
	add		x11, x11, x12
	ldr		q16, [x9]
	add		x9, x9, x10
	ldr		q18, [x9]
	add		x9, x9, x10
	ldr		q20, [x9]
	add		x9, x9, x10
	ldr		q22, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v4.2d, v20.2d, v29.d[0]

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]
	fmla	v2.2d, v22.2d, v30.d[1]
	fmla	v4.2d, v22.2d, v31.d[0]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9]
	add		x9, x9, x10
	ldr		q28, [x11, #0]
	ldr		d29, [x11, #16]
	fmla	v0.2d, v24.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v24.2d, v28.d[1]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v29.d[0]
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x12, x12
	add		x14, x13, x12
	
	add		x15, x10, x10

	// prefetch
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x15]

	// preload
	ldr		q24, [x11, #(0*8)]
	ldr		d25, [x11, #(2*8)]
	add		x11, x11, x12
	ldr		q26, [x11, #(0*8)]
	ldr		d27, [x11, #(2*8)]
	add		x11, x11, x12
	ldr		q28, [x11, #(0*8)]
	ldr		d29, [x11, #(2*8)]
	add		x11, x11, x12
	ldr		q30, [x11, #(0*8)]
	ldr		d31, [x11, #(2*8)]
	add		x11, x11, x12
	ldr		q16, [x9]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x11]
//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x13]
//	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x9, x15]

	// zero tmp acc
	fmov	d8, xzr
	fmov    d10, d8
	fmov    d12, d8

	// main loop
1:
	
	// unroll 0
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v2.2d, v16.2d, v24.d[1]
	prfm	PLDL1KEEP, [x11]
	fmla	v4.2d, v16.2d, v25.d[0]
	prfm	PLDL1KEEP, [x11, x12]

	// unroll 1
	ldr		q16, [x9]
	fmla	v8.2d, v18.2d, v26.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v10.2d, v18.2d, v26.d[1]
	prfm	PLDL1KEEP, [x11, x13]
	fmla	v12.2d, v18.2d, v27.d[0]
	prfm	PLDL1KEEP, [x11, x14]
	sub		w8, w8, #4

	// unroll 2
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v28.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v2.2d, v16.2d, v28.d[1]
	cmp		w8, #4
	ldr		q24, [x11, #(0*8)]
	ldr		d25, [x11, #(2*8)]
	fmla	v4.2d, v16.2d, v29.d[0]
	add		x11, x11, x12
	ldr		q26, [x11, #(0*8)]
	ldr		d27, [x11, #(2*8)]
	add		x11, x11, x12

	// unroll 3
	ldr		q16, [x9]
	fmla	v8.2d, v18.2d, v30.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v10.2d, v18.2d, v30.d[1]
	ldr		q28, [x11, #(0*8)]
	ldr		d29, [x11, #(2*8)]
	add		x11, x11, x12
	fmla	v12.2d, v18.2d, v31.d[0]
	ldr		q30, [x11, #(0*8)]
	ldr		d31, [x11, #(2*8)]
	add		x11, x11, x12

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d
	fadd	v2.2d, v2.2d, v10.2d
	fadd	v4.2d, v4.2d, v12.2d

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v2.2d, v16.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x9, #192]
	fmla	v4.2d, v16.2d, v25.d[0]
//	prfm	PLDL1KEEP, [x11, #128]

	// unroll 1
//	prfm	PLDL1KEEP, [x11, #192]
	ldr		q16, [x9]
	fmla	v0.2d, v18.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	sub		w8, w8, #4

	// unroll 2
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v4.2d, v16.2d, v29.d[0]
//	cmp		w8, #4

	// unroll 3
//	ldr		q16, [x9]
	fmla	v0.2d, v18.2d, v30.d[0]
//	add		x9, x9, x10
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v2.2d, v18.2d, v30.d[1]
//	add		x11, x11, x12
//	ldp		q26, q27, [x11, #(0*8+1*32)]
	fmla	v4.2d, v18.2d, v31.d[0]
//	add		x11, x11, x12
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	add		x11, x11, x12
//	ldr		q30, [x11, #(0*8)]
//	ldr		d31, [x11, #(2*8)]
//	add		x11, x11, x12

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x11, x11, x12
//	sub		x11, x11, x12
//	sub		x11, x11, x12
//	sub		x11, x11, x12
	sub		x11, x11, x12, lsl #2
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9]
	add		x9, x9, x10
	ldr		q28, [x11, #(0*8)]
	ldr		d29, [x11, #(2*8)]
	fmla	v0.2d, v24.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v24.2d, v28.d[1]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v29.d[0]
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nt_2x3_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NT_1X3_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nt_1x3_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldr		q24, [x11, #0]
	ldr		d25, [x11, #16]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	ldr		d27, [x11, #16]
	add		x11, x11, x12
	ldr		q28, [x11, #0]
	ldr		d29, [x11, #16]
	add		x11, x11, x12
	ldr		q30, [x11, #0]
	ldr		d31, [x11, #16]
	add		x11, x11, x12
	ldr		d16, [x9]
	add		x9, x9, x10
	ldr		d18, [x9]
	add		x9, x9, x10
	ldr		d20, [x9]
	add		x9, x9, x10
	ldr		d22, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v4.2d, v20.2d, v29.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]
	fmla	v2.2d, v22.2d, v30.d[1]
	fmla	v4.2d, v22.2d, v31.d[0]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldr		q24, [x11, #0]
	ldr		d25, [x11, #16]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	ldr		d27, [x11, #16]
	add		x11, x11, x12
	ldr		q28, [x11, #0]
	ldr		d29, [x11, #16]
	add		x11, x11, x12
	ldr		q30, [x11, #0]
	ldr		d31, [x11, #16]
	add		x11, x11, x12
	ldr		d16, [x9]
	add		x9, x9, x10
	ldr		d18, [x9]
	add		x9, x9, x10
	ldr		d20, [x9]
	add		x9, x9, x10
	ldr		d22, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v4.2d, v16.2d, v25.d[0]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v4.2d, v20.2d, v29.d[0]

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]
	fmla	v2.2d, v22.2d, v30.d[1]
	fmla	v4.2d, v22.2d, v31.d[0]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		d24, [x9]
	add		x9, x9, x10
	ldr		q28, [x11, #0]
	ldr		d29, [x11, #16]
	fmla	v0.2d, v24.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v24.2d, v28.d[1]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v29.d[0]
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x12, x12
	add		x14, x13, x12
	
	add		x15, x10, x10

	// prefetch
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x15]

	// preload
	ldr		q24, [x11, #(0*8)]
	ldr		d25, [x11, #(2*8)]
	add		x11, x11, x12
	ldr		q26, [x11, #(0*8)]
	ldr		d27, [x11, #(2*8)]
	add		x11, x11, x12
	ldr		q28, [x11, #(0*8)]
	ldr		d29, [x11, #(2*8)]
	add		x11, x11, x12
	ldr		q30, [x11, #(0*8)]
	ldr		d31, [x11, #(2*8)]
	add		x11, x11, x12
	ldr		d16, [x9]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x11]
//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x13]
//	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x9, x15]

	// zero tmp acc
	fmov	d8, xzr
	fmov    d10, d8
	fmov    d12, d8

	// main loop
1:
	
	// unroll 0
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v2.2d, v16.2d, v24.d[1]
	prfm	PLDL1KEEP, [x11]
	fmla	v4.2d, v16.2d, v25.d[0]
	prfm	PLDL1KEEP, [x11, x12]

	// unroll 1
	ldr		d16, [x9]
	fmla	v8.2d, v18.2d, v26.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v10.2d, v18.2d, v26.d[1]
	prfm	PLDL1KEEP, [x11, x13]
	fmla	v12.2d, v18.2d, v27.d[0]
	prfm	PLDL1KEEP, [x11, x14]
	sub		w8, w8, #4

	// unroll 2
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v28.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v2.2d, v16.2d, v28.d[1]
	cmp		w8, #4
	ldr		q24, [x11, #(0*8)]
	ldr		d25, [x11, #(2*8)]
	fmla	v4.2d, v16.2d, v29.d[0]
	add		x11, x11, x12
	ldr		q26, [x11, #(0*8)]
	ldr		d27, [x11, #(2*8)]
	add		x11, x11, x12

	// unroll 3
	ldr		d16, [x9]
	fmla	v8.2d, v18.2d, v30.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v10.2d, v18.2d, v30.d[1]
	ldr		q28, [x11, #(0*8)]
	ldr		d29, [x11, #(2*8)]
	add		x11, x11, x12
	fmla	v12.2d, v18.2d, v31.d[0]
	ldr		q30, [x11, #(0*8)]
	ldr		d31, [x11, #(2*8)]
	add		x11, x11, x12

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d
	fadd	v2.2d, v2.2d, v10.2d
	fadd	v4.2d, v4.2d, v12.2d

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v2.2d, v16.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x9, #192]
	fmla	v4.2d, v16.2d, v25.d[0]
//	prfm	PLDL1KEEP, [x11, #128]

	// unroll 1
//	prfm	PLDL1KEEP, [x11, #192]
	ldr		d16, [x9]
	fmla	v0.2d, v18.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v27.d[0]
	sub		w8, w8, #4

	// unroll 2
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v4.2d, v16.2d, v29.d[0]
//	cmp		w8, #4

	// unroll 3
//	ldr		d16, [x9]
	fmla	v0.2d, v18.2d, v30.d[0]
//	add		x9, x9, x10
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v2.2d, v18.2d, v30.d[1]
//	add		x11, x11, x12
//	ldp		q26, q27, [x11, #(0*8+1*32)]
	fmla	v4.2d, v18.2d, v31.d[0]
//	add		x11, x11, x12
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	add		x11, x11, x12
//	ldr		q30, [x11, #(0*8)]
//	ldr		d31, [x11, #(2*8)]
//	add		x11, x11, x12

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x11, x11, x12
//	sub		x11, x11, x12
//	sub		x11, x11, x12
//	sub		x11, x11, x12
	sub		x11, x11, x12, lsl #2
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldr		d24, [x9]
	add		x9, x9, x10
	ldr		q28, [x11, #(0*8)]
	ldr		d29, [x11, #(2*8)]
	fmla	v0.2d, v24.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v24.2d, v28.d[1]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v29.d[0]
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nt_1x3_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NT_4X2_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nt_4x2_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldr		q24, [x11, #0]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	add		x11, x11, x12
	ldr		q28, [x11, #0]
	add		x11, x11, x12
	ldr		q30, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9]
	add		x9, x9, x10
	ldp		q18, q19, [x9]
	add		x9, x9, x10
	ldp		q20, q21, [x9]
	add		x9, x9, x10
	ldp		q22, q23, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v3.2d, v21.2d, v28.d[1]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]
	fmla	v1.2d, v23.2d, v30.d[0]
	fmla	v2.2d, v22.2d, v30.d[1]
	fmla	v3.2d, v23.2d, v30.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldr		q24, [x11, #0]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	add		x11, x11, x12
	ldr		q28, [x11, #0]
	add		x11, x11, x12
	ldr		q30, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9]
	add		x9, x9, x10
	ldp		q18, q19, [x9]
	add		x9, x9, x10
	ldp		q20, q21, [x9]
	add		x9, x9, x10
	ldp		q22, q23, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v3.2d, v21.2d, v28.d[1]

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]
	fmla	v1.2d, v23.2d, v30.d[0]
	fmla	v2.2d, v22.2d, v30.d[1]
	fmla	v3.2d, v23.2d, v30.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9]
	add		x9, x9, x10
	ldr		q28, [x11, #0]
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v24.2d, v28.d[1]
	fmla	v3.2d, v25.2d, v28.d[1]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x12, x12
	add		x14, x13, x12
	
	add		x15, x10, x10

	// prefetch
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x15]

	// preload
	ldr		q24, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		q26, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		q28, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		q30, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q16, q17, [x9]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x11]
//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x13]
//	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x9, x15]

	// zero tmp acc
	fmov	d8, xzr
	fmov    d9, d8
	fmov    d10, d8
	fmov    d11, d8

	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]

	// unroll 1
	ldp		q16, q17, [x9]
	fmla	v8.2d, v18.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v19.2d, v26.d[0]
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v10.2d, v18.2d, v26.d[1]
	fmla	v11.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x11, x14]
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v28.d[0]
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v2.2d, v16.2d, v28.d[1]
	cmp		w8, #4
	fmla	v3.2d, v17.2d, v28.d[1]
	ldr		q24, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		q26, [x11, #(0*8)]
	add		x11, x11, x12

	// unroll 3
	ldp		q16, q17, [x9]
	fmla	v8.2d, v18.2d, v30.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v19.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v10.2d, v18.2d, v30.d[1]
	ldr		q28, [x11, #(0*8)]
	fmla	v11.2d, v19.2d, v30.d[1]
	add		x11, x11, x12
	ldr		q30, [x11, #(0*8)]
	add		x11, x11, x12

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d
	fadd	v1.2d, v1.2d, v9.2d
	fadd	v2.2d, v2.2d, v10.2d
	fadd	v3.2d, v3.2d, v11.2d

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x11, #128]

	// unroll 1
//	prfm	PLDL1KEEP, [x11, #192]
	ldp		q16, q17, [x9]
	fmla	v0.2d, v18.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v28.d[0]
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
//	cmp		w8, #4

	// unroll 3
//	ldp		q16, q17, [x9]
	fmla	v0.2d, v18.2d, v30.d[0]
//	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v30.d[0]
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v2.2d, v18.2d, v30.d[1]
//	add		x11, x11, x12
	fmla	v3.2d, v19.2d, v30.d[1]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
//	add		x11, x11, x12
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	add		x11, x11, x12
//	ldp		q30, q31, [x11, #(0*8+3*32)]
//	add		x11, x11, x12

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x11, x11, x12
//	sub		x11, x11, x12
//	sub		x11, x11, x12
//	sub		x11, x11, x12
	sub		x11, x11, x12, lsl #2
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9]
	add		x9, x9, x10
	ldr		q28, [x11, #(0*8)]
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v24.2d, v28.d[1]
	fmla	v3.2d, v25.2d, v28.d[1]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nt_4x2_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NT_3X2_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nt_3x2_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldr		q24, [x11, #0]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	add		x11, x11, x12
	ldr		q28, [x11, #0]
	add		x11, x11, x12
	ldr		q30, [x11, #0]
	add		x11, x11, x12
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	add		x9, x9, x10
	ldr		q20, [x9]
	ldr		d21, [x9, #16]
	add		x9, x9, x10
	ldr		q22, [x9]
	ldr		d23, [x9, #16]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v3.2d, v21.2d, v28.d[1]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]
	fmla	v1.2d, v23.2d, v30.d[0]
	fmla	v2.2d, v22.2d, v30.d[1]
	fmla	v3.2d, v23.2d, v30.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldr		q24, [x11, #0]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	add		x11, x11, x12
	ldr		q28, [x11, #0]
	add		x11, x11, x12
	ldr		q30, [x11, #0]
	add		x11, x11, x12
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	add		x9, x9, x10
	ldr		q20, [x9]
	ldr		d21, [x9, #16]
	add		x9, x9, x10
	ldr		q22, [x9]
	ldr		d23, [x9, #16]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	fmla	v2.2d, v20.2d, v28.d[1]
	fmla	v3.2d, v21.2d, v28.d[1]

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]
	fmla	v1.2d, v23.2d, v30.d[0]
	fmla	v2.2d, v22.2d, v30.d[1]
	fmla	v3.2d, v23.2d, v30.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9]
	ldr		d25, [x9, #16]
	add		x9, x9, x10
	ldr		q28, [x11, #0]
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v24.2d, v28.d[1]
	fmla	v3.2d, v25.2d, v28.d[1]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x12, x12
	add		x14, x13, x12
	
	add		x15, x10, x10

	// prefetch
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x15]

	// preload
	ldr		q24, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		q26, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		q28, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		q30, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x11]
//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x13]
//	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x9, x15]

	// zero tmp acc
	fmov	d8, xzr
	fmov    d9, d8
	fmov    d10, d8
	fmov    d11, d8

	// main loop
1:
	
	// unroll 0
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]

	// unroll 1
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v8.2d, v18.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v19.2d, v26.d[0]
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v10.2d, v18.2d, v26.d[1]
	fmla	v11.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x11, x14]
	sub		w8, w8, #4

	// unroll 2
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v28.d[0]
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v2.2d, v16.2d, v28.d[1]
	cmp		w8, #4
	fmla	v3.2d, v17.2d, v28.d[1]
	ldr		q24, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		q26, [x11, #(0*8)]
	add		x11, x11, x12

	// unroll 3
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v8.2d, v18.2d, v30.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v19.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v10.2d, v18.2d, v30.d[1]
	ldr		q28, [x11, #(0*8)]
	fmla	v11.2d, v19.2d, v30.d[1]
	add		x11, x11, x12
	ldr		q30, [x11, #(0*8)]
	add		x11, x11, x12

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d
	fadd	v1.2d, v1.2d, v9.2d
	fadd	v2.2d, v2.2d, v10.2d
	fadd	v3.2d, v3.2d, v11.2d

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x11, #128]

	// unroll 1
//	prfm	PLDL1KEEP, [x11, #192]
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v0.2d, v18.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	sub		w8, w8, #4

	// unroll 2
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v28.d[0]
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
//	cmp		w8, #4

	// unroll 3
//	ldr		q16, [x9]
//	ldr		d17, [x9, #16]
	fmla	v0.2d, v18.2d, v30.d[0]
//	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v30.d[0]
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v2.2d, v18.2d, v30.d[1]
//	add		x11, x11, x12
	fmla	v3.2d, v19.2d, v30.d[1]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
//	add		x11, x11, x12
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	add		x11, x11, x12
//	ldp		q30, q31, [x11, #(0*8+3*32)]
//	add		x11, x11, x12

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x11, x11, x12
//	sub		x11, x11, x12
//	sub		x11, x11, x12
//	sub		x11, x11, x12
	sub		x11, x11, x12, lsl #2
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9]
	ldr		d25, [x9, #16]
	add		x9, x9, x10
	ldr		q28, [x11, #(0*8)]
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v24.2d, v28.d[1]
	fmla	v3.2d, v25.2d, v28.d[1]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nt_3x2_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NT_2X2_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nt_2x2_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldr		q24, [x11, #0]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	add		x11, x11, x12
	ldr		q28, [x11, #0]
	add		x11, x11, x12
	ldr		q30, [x11, #0]
	add		x11, x11, x12
	ldr		q16, [x9]
	add		x9, x9, x10
	ldr		q18, [x9]
	add		x9, x9, x10
	ldr		q20, [x9]
	add		x9, x9, x10
	ldr		q22, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v2.2d, v20.2d, v28.d[1]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]
	fmla	v2.2d, v22.2d, v30.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldr		q24, [x11, #0]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	add		x11, x11, x12
	ldr		q28, [x11, #0]
	add		x11, x11, x12
	ldr		q30, [x11, #0]
	add		x11, x11, x12
	ldr		q16, [x9]
	add		x9, x9, x10
	ldr		q18, [x9]
	add		x9, x9, x10
	ldr		q20, [x9]
	add		x9, x9, x10
	ldr		q22, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v2.2d, v20.2d, v28.d[1]

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]
	fmla	v2.2d, v22.2d, v30.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9]
	add		x9, x9, x10
	ldr		q28, [x11, #0]
	fmla	v0.2d, v24.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v24.2d, v28.d[1]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x12, x12
	add		x14, x13, x12
	
	add		x15, x10, x10

	// prefetch
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x15]

	// preload
	ldr		q24, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		q26, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		q28, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		q30, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		q16, [x9]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x11]
//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x13]
//	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x9, x15]

	// zero tmp acc
	fmov	d8, xzr
	fmov    d10, d8

	// main loop
1:
	
	// unroll 0
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v2.2d, v16.2d, v24.d[1]
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]

	// unroll 1
	ldr		q16, [x9]
	fmla	v8.2d, v18.2d, v26.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v10.2d, v18.2d, v26.d[1]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x11, x14]
	sub		w8, w8, #4

	// unroll 2
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v28.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v2.2d, v16.2d, v28.d[1]
	cmp		w8, #4
	ldr		q24, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		q26, [x11, #(0*8)]
	add		x11, x11, x12

	// unroll 3
	ldr		q16, [x9]
	fmla	v8.2d, v18.2d, v30.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v10.2d, v18.2d, v30.d[1]
	ldr		q28, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		q30, [x11, #(0*8)]
	add		x11, x11, x12

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d
	fadd	v2.2d, v2.2d, v10.2d

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x11, #128]

	// unroll 1
//	prfm	PLDL1KEEP, [x11, #192]
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v0.2d, v18.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	sub		w8, w8, #4

	// unroll 2
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v28.d[0]
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
//	cmp		w8, #4

	// unroll 3
//	ldr		q16, [x9]
//	ldr		d17, [x9, #16]
	fmla	v0.2d, v18.2d, v30.d[0]
//	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v30.d[0]
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v2.2d, v18.2d, v30.d[1]
//	add		x11, x11, x12
	fmla	v3.2d, v19.2d, v30.d[1]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
//	add		x11, x11, x12
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	add		x11, x11, x12
//	ldp		q30, q31, [x11, #(0*8+3*32)]
//	add		x11, x11, x12

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x11, x11, x12
//	sub		x11, x11, x12
//	sub		x11, x11, x12
//	sub		x11, x11, x12
	sub		x11, x11, x12, lsl #2
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9]
	add		x9, x9, x10
	ldr		q28, [x11, #(0*8)]
	fmla	v0.2d, v24.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v24.2d, v28.d[1]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nt_2x2_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NT_1X2_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nt_1x2_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldr		q24, [x11, #0]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	add		x11, x11, x12
	ldr		q28, [x11, #0]
	add		x11, x11, x12
	ldr		q30, [x11, #0]
	add		x11, x11, x12
	ldr		d16, [x9]
	add		x9, x9, x10
	ldr		d18, [x9]
	add		x9, x9, x10
	ldr		d20, [x9]
	add		x9, x9, x10
	ldr		d22, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v2.2d, v20.2d, v28.d[1]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]
	fmla	v2.2d, v22.2d, v30.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldr		q24, [x11, #0]
	add		x11, x11, x12
	ldr		q26, [x11, #0]
	add		x11, x11, x12
	ldr		q28, [x11, #0]
	add		x11, x11, x12
	ldr		q30, [x11, #0]
	add		x11, x11, x12
	ldr		d16, [x9]
	add		x9, x9, x10
	ldr		d18, [x9]
	add		x9, x9, x10
	ldr		d20, [x9]
	add		x9, x9, x10
	ldr		d22, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v24.d[1]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v2.2d, v20.2d, v28.d[1]

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]
	fmla	v2.2d, v22.2d, v30.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		d24, [x9]
	add		x9, x9, x10
	ldr		q28, [x11, #0]
	fmla	v0.2d, v24.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v24.2d, v28.d[1]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x12, x12
	add		x14, x13, x12
	
	add		x15, x10, x10

	// prefetch
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x15]

	// preload
	ldr		q24, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		q26, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		q28, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		q30, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		d16, [x9]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x11]
//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x13]
//	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x9, x15]

	// zero tmp acc
	fmov	d8, xzr
	fmov    d10, d8

	// main loop
1:
	
	// unroll 0
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v2.2d, v16.2d, v24.d[1]
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]

	// unroll 1
	ldr		d16, [x9]
	fmla	v8.2d, v18.2d, v26.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v10.2d, v18.2d, v26.d[1]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x11, x14]
	sub		w8, w8, #4

	// unroll 2
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v28.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v2.2d, v16.2d, v28.d[1]
	cmp		w8, #4
	ldr		q24, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		q26, [x11, #(0*8)]
	add		x11, x11, x12

	// unroll 3
	ldr		d16, [x9]
	fmla	v8.2d, v18.2d, v30.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x15]
	fmla	v10.2d, v18.2d, v30.d[1]
	ldr		q28, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		q30, [x11, #(0*8)]
	add		x11, x11, x12

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d
	fadd	v2.2d, v2.2d, v10.2d

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		d18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v2.2d, v16.2d, v24.d[1]
	fmla	v3.2d, v17.2d, v24.d[1]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x11, #128]

	// unroll 1
//	prfm	PLDL1KEEP, [x11, #192]
	ldr		d16, [x9]
	ldr		d17, [x9, #16]
	fmla	v0.2d, v18.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v26.d[0]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	sub		w8, w8, #4

	// unroll 2
	ldr		d18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v28.d[0]
	fmla	v2.2d, v16.2d, v28.d[1]
	fmla	v3.2d, v17.2d, v28.d[1]
//	cmp		w8, #4

	// unroll 3
//	ldr		d16, [x9]
//	ldr		d17, [x9, #16]
	fmla	v0.2d, v18.2d, v30.d[0]
//	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v30.d[0]
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v2.2d, v18.2d, v30.d[1]
//	add		x11, x11, x12
	fmla	v3.2d, v19.2d, v30.d[1]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
//	add		x11, x11, x12
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	add		x11, x11, x12
//	ldp		q30, q31, [x11, #(0*8+3*32)]
//	add		x11, x11, x12

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x11, x11, x12
//	sub		x11, x11, x12
//	sub		x11, x11, x12
//	sub		x11, x11, x12
	sub		x11, x11, x12, lsl #2
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldr		d24, [x9]
	add		x9, x9, x10
	ldr		q28, [x11, #(0*8)]
	fmla	v0.2d, v24.2d, v28.d[0]
	add		x11, x11, x12
	fmla	v2.2d, v24.2d, v28.d[1]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nt_1x2_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NT_4X1_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nt_4x1_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldr		d24, [x11, #0]
	add		x11, x11, x12
	ldr		d26, [x11, #0]
	add		x11, x11, x12
	ldr		d28, [x11, #0]
	add		x11, x11, x12
	ldr		d30, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9]
	add		x9, x9, x10
	ldp		q18, q19, [x9]
	add		x9, x9, x10
	ldp		q20, q21, [x9]
	add		x9, x9, x10
	ldp		q22, q23, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]
	fmla	v1.2d, v23.2d, v30.d[0]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldr		d24, [x11, #0]
	add		x11, x11, x12
	ldr		d26, [x11, #0]
	add		x11, x11, x12
	ldr		d28, [x11, #0]
	add		x11, x11, x12
	ldr		d30, [x11, #0]
	add		x11, x11, x12
	ldp		q16, q17, [x9]
	add		x9, x9, x10
	ldp		q18, q19, [x9]
	add		x9, x9, x10
	ldp		q20, q21, [x9]
	add		x9, x9, x10
	ldp		q22, q23, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]
	fmla	v1.2d, v23.2d, v30.d[0]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9]
	add		x9, x9, x10
	ldr		d28, [x11, #0]
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	add		x11, x11, x12
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x12, x12
	add		x14, x13, x12
	
	add		x15, x10, x10

	// prefetch
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x15]

	// preload
	ldr		d24, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		d26, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		d28, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		d30, [x11, #(0*8)]
	add		x11, x11, x12
	ldp		q16, q17, [x9]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x11]
//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x13]
//	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x9, x15]

	// zero tmp acc
	fmov	d8, xzr
	fmov    d9, d8

	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x15]
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]

	// unroll 1
	ldp		q16, q17, [x9]
	fmla	v8.2d, v18.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v19.2d, v26.d[0]
	prfm	PLDL1KEEP, [x9, x15]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x11, x14]
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v28.d[0]
	prfm	PLDL1KEEP, [x9, x15]
	cmp		w8, #4
	ldr		d24, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		d26, [x11, #(0*8)]
	add		x11, x11, x12

	// unroll 3
	ldp		q16, q17, [x9]
	fmla	v8.2d, v18.2d, v30.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v19.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x15]
	ldr		d28, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		d30, [x11, #(0*8)]
	add		x11, x11, x12

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d
	fadd	v1.2d, v1.2d, v9.2d

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x11, #128]

	// unroll 1
//	prfm	PLDL1KEEP, [x11, #192]
	ldp		q16, q17, [x9]
	fmla	v0.2d, v18.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v26.d[0]
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v28.d[0]
//	cmp		w8, #4

	// unroll 3
//	ldp		q16, q17, [x9]
	fmla	v0.2d, v18.2d, v30.d[0]
//	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v30.d[0]
//	ldp		q24, q25, [x11, #(0*8+0*32)]
//	add		x11, x11, x12
//	ldp		q26, q27, [x11, #(0*8+1*32)]
//	add		x11, x11, x12
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	add		x11, x11, x12
//	ldp		q30, q31, [x11, #(0*8+3*32)]
//	add		x11, x11, x12

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x11, x11, x12
//	sub		x11, x11, x12
//	sub		x11, x11, x12
//	sub		x11, x11, x12
	sub		x11, x11, x12, lsl #2
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9]
	add		x9, x9, x10
	ldr		d28, [x11, #(0*8)]
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	add		x11, x11, x12
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nt_4x1_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NT_3X1_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nt_3x1_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldr		d24, [x11, #0]
	add		x11, x11, x12
	ldr		d26, [x11, #0]
	add		x11, x11, x12
	ldr		d28, [x11, #0]
	add		x11, x11, x12
	ldr		d30, [x11, #0]
	add		x11, x11, x12
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	add		x9, x9, x10
	ldr		q20, [x9]
	ldr		d21, [x9, #16]
	add		x9, x9, x10
	ldr		q22, [x9]
	ldr		d23, [x9, #16]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]
	fmla	v1.2d, v23.2d, v30.d[0]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldr		d24, [x11, #0]
	add		x11, x11, x12
	ldr		d26, [x11, #0]
	add		x11, x11, x12
	ldr		d28, [x11, #0]
	add		x11, x11, x12
	ldr		d30, [x11, #0]
	add		x11, x11, x12
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	add		x9, x9, x10
	ldr		q20, [x9]
	ldr		d21, [x9, #16]
	add		x9, x9, x10
	ldr		q22, [x9]
	ldr		d23, [x9, #16]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	fmla	v1.2d, v19.2d, v26.d[0]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	fmla	v1.2d, v21.2d, v28.d[0]

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]
	fmla	v1.2d, v23.2d, v30.d[0]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9]
	ldr		d25, [x9, #16]
	add		x9, x9, x10
	ldr		d28, [x11, #0]
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	add		x11, x11, x12
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x12, x12
	add		x14, x13, x12
	
	add		x15, x10, x10

	// prefetch
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x15]

	// preload
	ldr		d24, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		d26, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		d28, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		d30, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x11]
//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x13]
//	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x9, x15]

	// zero tmp acc
	fmov	d8, xzr
	fmov    d9, d8

	// main loop
1:
	
	// unroll 0
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x15]
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]

	// unroll 1
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v8.2d, v18.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v19.2d, v26.d[0]
	prfm	PLDL1KEEP, [x9, x15]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x11, x14]
	sub		w8, w8, #4

	// unroll 2
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v28.d[0]
	prfm	PLDL1KEEP, [x9, x15]
	cmp		w8, #4
	ldr		d24, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		d26, [x11, #(0*8)]
	add		x11, x11, x12

	// unroll 3
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v8.2d, v18.2d, v30.d[0]
	add		x9, x9, x10
	fmla	v9.2d, v19.2d, v30.d[0]
	prfm	PLDL1KEEP, [x9, x15]
	ldr		d28, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		d30, [x11, #(0*8)]
	add		x11, x11, x12

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d
	fadd	v1.2d, v1.2d, v9.2d

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x11, #128]

	// unroll 1
//	prfm	PLDL1KEEP, [x11, #192]
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v0.2d, v18.2d, v26.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v26.d[0]
	sub		w8, w8, #4

	// unroll 2
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v28.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v28.d[0]
//	cmp		w8, #4

	// unroll 3
//	ldr		q16, [x9]
//	ldr		d17, [x9, #16]
	fmla	v0.2d, v18.2d, v30.d[0]
//	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v30.d[0]
//	ldp		q24, q25, [x11, #(0*8+0*32)]
//	add		x11, x11, x12
//	ldp		q26, q27, [x11, #(0*8+1*32)]
//	add		x11, x11, x12
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	add		x11, x11, x12
//	ldp		q30, q31, [x11, #(0*8+3*32)]
//	add		x11, x11, x12

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x11, x11, x12
//	sub		x11, x11, x12
//	sub		x11, x11, x12
//	sub		x11, x11, x12
	sub		x11, x11, x12, lsl #2
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9]
	ldr		d25, [x9, #16]
	add		x9, x9, x10
	ldr		d28, [x11, #(0*8)]
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	add		x11, x11, x12
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nt_3x1_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NT_2X1_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nt_2x1_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldr		d24, [x11, #0]
	add		x11, x11, x12
	ldr		d26, [x11, #0]
	add		x11, x11, x12
	ldr		d28, [x11, #0]
	add		x11, x11, x12
	ldr		d30, [x11, #0]
	add		x11, x11, x12
	ldr		q16, [x9]
	add		x9, x9, x10
	ldr		q18, [x9]
	add		x9, x9, x10
	ldr		q20, [x9]
	add		x9, x9, x10
	ldr		q22, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldr		d24, [x11, #0]
	add		x11, x11, x12
	ldr		d26, [x11, #0]
	add		x11, x11, x12
	ldr		d28, [x11, #0]
	add		x11, x11, x12
	ldr		d30, [x11, #0]
	add		x11, x11, x12
	ldr		q16, [x9]
	add		x9, x9, x10
	ldr		q18, [x9]
	add		x9, x9, x10
	ldr		q20, [x9]
	add		x9, x9, x10
	ldr		q22, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9]
	add		x9, x9, x10
	ldr		d28, [x11, #0]
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	add		x11, x11, x12
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x12, x12
	add		x14, x13, x12
	
	add		x15, x10, x10

	// prefetch
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x15]

	// preload
	ldr		d24, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		d26, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		d28, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		d30, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		q16, [x9]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x11]
//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x13]
//	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x9, x15]

	// zero tmp acc
	fmov	d8, xzr

	// main loop
1:
	
	// unroll 0
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x15]
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]

	// unroll 1
	ldr		q16, [x9]
	fmla	v8.2d, v18.2d, v26.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x15]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x11, x14]
	sub		w8, w8, #4

	// unroll 2
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v28.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x15]
	cmp		w8, #4
	ldr		d24, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		d26, [x11, #(0*8)]
	add		x11, x11, x12

	// unroll 3
	ldr		q16, [x9]
	fmla	v8.2d, v18.2d, v30.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x15]
	ldr		d28, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		d30, [x11, #(0*8)]
	add		x11, x11, x12

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x11, #128]

	// unroll 1
//	prfm	PLDL1KEEP, [x11, #192]
	ldr		q16, [x9]
	fmla	v0.2d, v18.2d, v26.d[0]
	add		x9, x9, x10
	sub		w8, w8, #4

	// unroll 2
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v28.d[0]
	add		x9, x9, x10
//	cmp		w8, #4

	// unroll 3
//	ldr		q16, [x9]
	fmla	v0.2d, v18.2d, v30.d[0]
//	add		x9, x9, x10
//	ldp		q24, q25, [x11, #(0*8+0*32)]
//	add		x11, x11, x12
//	ldp		q26, q27, [x11, #(0*8+1*32)]
//	add		x11, x11, x12
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	add		x11, x11, x12
//	ldp		q30, q31, [x11, #(0*8+3*32)]
//	add		x11, x11, x12

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x11, x11, x12
//	sub		x11, x11, x12
//	sub		x11, x11, x12
//	sub		x11, x11, x12
	sub		x11, x11, x12, lsl #2
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9]
	add		x9, x9, x10
	ldr		d28, [x11, #(0*8)]
	fmla	v0.2d, v24.2d, v28.d[0]
	add		x11, x11, x12
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nt_2x1_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NT_1X1_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nt_1x1_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldr		d24, [x11, #0]
	add		x11, x11, x12
	ldr		d26, [x11, #0]
	add		x11, x11, x12
	ldr		d28, [x11, #0]
	add		x11, x11, x12
	ldr		d30, [x11, #0]
	add		x11, x11, x12
	ldr		d16, [x9]
	add		x9, x9, x10
	ldr		d18, [x9]
	add		x9, x9, x10
	ldr		d20, [x9]
	add		x9, x9, x10
	ldr		d22, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldr		d24, [x11, #0]
	add		x11, x11, x12
	ldr		d26, [x11, #0]
	add		x11, x11, x12
	ldr		d28, [x11, #0]
	add		x11, x11, x12
	ldr		d30, [x11, #0]
	add		x11, x11, x12
	ldr		d16, [x9]
	add		x9, x9, x10
	ldr		d18, [x9]
	add		x9, x9, x10
	ldr		d20, [x9]
	add		x9, x9, x10
	ldr		d22, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]

	// unroll  1
	fmla	v0.2d, v18.2d, v26.d[0]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v28.d[0]

	// unroll 3
	fmla	v0.2d, v22.2d, v30.d[0]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		d24, [x9]
	add		x9, x9, x10
	ldr		d28, [x11, #0]
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	add		x11, x11, x12
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x12, x12
	add		x14, x13, x12
	
	add		x15, x10, x10

	// prefetch
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x15]

	// preload
	ldr		d24, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		d26, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		d28, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		d30, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		d16, [x9]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x11]
//	prfm	PLDL1KEEP, [x11, x12]
//	prfm	PLDL1KEEP, [x11, x13]
//	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x9, x15]

	// zero tmp acc
	fmov	d8, xzr

	// main loop
1:
	
	// unroll 0
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x15]
	prfm	PLDL1KEEP, [x11]
	prfm	PLDL1KEEP, [x11, x12]

	// unroll 1
	ldr		d16, [x9]
	fmla	v8.2d, v18.2d, v26.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x15]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x11, x14]
	sub		w8, w8, #4

	// unroll 2
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v28.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x15]
	cmp		w8, #4
	ldr		d24, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		d26, [x11, #(0*8)]
	add		x11, x11, x12

	// unroll 3
	ldr		d16, [x9]
	fmla	v8.2d, v18.2d, v30.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x15]
	ldr		d28, [x11, #(0*8)]
	add		x11, x11, x12
	ldr		d30, [x11, #(0*8)]
	add		x11, x11, x12

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
//	prfm	PLDL1KEEP, [x11, #128]

	// unroll 1
//	prfm	PLDL1KEEP, [x11, #192]
	ldr		d16, [x9]
	fmla	v0.2d, v18.2d, v26.d[0]
	add		x9, x9, x10
	sub		w8, w8, #4

	// unroll 2
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v28.d[0]
	add		x9, x9, x10
//	cmp		w8, #4

	// unroll 3
//	ldr		d16, [x9]
	fmla	v0.2d, v18.2d, v30.d[0]
//	add		x9, x9, x10
//	ldp		q24, q25, [x11, #(0*8+0*32)]
//	add		x11, x11, x12
//	ldp		q26, q27, [x11, #(0*8+1*32)]
//	add		x11, x11, x12
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	add		x11, x11, x12
//	ldp		q30, q31, [x11, #(0*8+3*32)]
//	add		x11, x11, x12

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x11, x11, x12
//	sub		x11, x11, x12
//	sub		x11, x11, x12
//	sub		x11, x11, x12
	sub		x11, x11, x12, lsl #2
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldr		d24, [x9]
	add		x9, x9, x10
	ldr		d28, [x11, #(0*8)]
	fmla	v0.2d, v24.2d, v28.d[0]
	add		x11, x11, x12
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nt_1x1_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
// x13  <- m1
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NT_4X4_VS_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nt_4x4_vs_libcc)

	// save old return address
	sub sp, sp, #16
	str x30, [sp, #0]
#endif

	cmp		w13, #1
	bgt		90f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_1X4_LIBCC
#else
	CALL(inner_kernel_gemm_nt_1x4_libcc)
#endif

	b		93f

90:

	cmp		w13, #2
	bgt		91f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_2X4_LIBCC
#else
	CALL(inner_kernel_gemm_nt_2x4_libcc)
#endif
	
	b		93f

91:

	cmp		w13, #3
	bgt		92f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_3X4_LIBCC
#else
	CALL(inner_kernel_gemm_nt_3x4_libcc)
#endif
	
	b		93f

92:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIBCC
#else
	CALL(inner_kernel_gemm_nt_4x4_libcc)
#endif

93:
// XXX x13 possibly used as working register on exit !!!

#if MACRO_LEVEL>=2
	.endm
#else
	// load old return address
	ldr x30, [sp, #0]
	add sp, sp, #16

	ret

	FUN_END(inner_kernel_gemm_nt_4x4_vs_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
// x13  <- m1
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NT_4X3_VS_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nt_4x3_vs_libcc)

	// save old return address
	sub sp, sp, #16
	str x30, [sp, #0]
#endif

	cmp		w13, #1
	bgt		90f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_1X3_LIBCC
#else
	CALL(inner_kernel_gemm_nt_1x3_libcc)
#endif

	b		93f

90:

	cmp		w13, #2
	bgt		91f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_2X3_LIBCC
#else
	CALL(inner_kernel_gemm_nt_2x3_libcc)
#endif
	
	b		93f

91:

	cmp		w13, #3
	bgt		92f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_3X3_LIBCC
#else
	CALL(inner_kernel_gemm_nt_3x3_libcc)
#endif
	
	b		93f

92:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X3_LIBCC
#else
	CALL(inner_kernel_gemm_nt_4x3_libcc)
#endif

93:
// XXX x13 possibly used as working register on exit !!!

#if MACRO_LEVEL>=2
	.endm
#else
	// load old return address
	ldr x30, [sp, #0]
	add sp, sp, #16

	ret

	FUN_END(inner_kernel_gemm_nt_4x3_vs_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
// x13  <- m1
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NT_4X2_VS_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nt_4x2_vs_libcc)

	// save old return address
	sub sp, sp, #16
	str x30, [sp, #0]
#endif

	cmp		w13, #1
	bgt		90f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_1X2_LIBCC
#else
	CALL(inner_kernel_gemm_nt_1x2_libcc)
#endif

	b		93f

90:

	cmp		w13, #2
	bgt		91f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_2X2_LIBCC
#else
	CALL(inner_kernel_gemm_nt_2x2_libcc)
#endif
	
	b		93f

91:

	cmp		w13, #3
	bgt		92f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_3X2_LIBCC
#else
	CALL(inner_kernel_gemm_nt_3x2_libcc)
#endif
	
	b		93f

92:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X2_LIBCC
#else
	CALL(inner_kernel_gemm_nt_4x2_libcc)
#endif

93:
// XXX x13 possibly used as working register on exit !!!

#if MACRO_LEVEL>=2
	.endm
#else
	// load old return address
	ldr x30, [sp, #0]
	add sp, sp, #16

	ret

	FUN_END(inner_kernel_gemm_nt_4x2_vs_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
// x13  <- m1
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NT_4X1_VS_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nt_4x1_vs_libcc)

	// save old return address
	sub sp, sp, #16
	str x30, [sp, #0]
#endif

	cmp		w13, #1
	bgt		90f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_1X1_LIBCC
#else
	CALL(inner_kernel_gemm_nt_1x1_libcc)
#endif

	b		93f

90:

	cmp		w13, #2
	bgt		91f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_2X1_LIBCC
#else
	CALL(inner_kernel_gemm_nt_2x1_libcc)
#endif
	
	b		93f

91:

	cmp		w13, #3
	bgt		92f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_3X1_LIBCC
#else
	CALL(inner_kernel_gemm_nt_3x1_libcc)
#endif
	
	b		93f

92:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X1_LIBCC
#else
	CALL(inner_kernel_gemm_nt_4x1_libcc)
#endif

93:
// XXX x13 possibly used as working register on exit !!!

#if MACRO_LEVEL>=2
	.endm
#else
	// load old return address
	ldr x30, [sp, #0]
	add sp, sp, #16

	ret

	FUN_END(inner_kernel_gemm_nt_4x1_vs_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_4X4_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nn_4x4_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x11, x12
	add		x14, x13, x12
	add		x15, x14, x12

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldp		q30, q31, [x15], #32
	ldp		q16, q17, [x9]
	add		x9, x9, x10
	ldp		q18, q19, [x9]
	add		x9, x9, x10
	ldp		q20, q21, [x9]
	add		x9, x9, x10
	ldp		q22, q23, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
	fmla	v6.2d, v16.2d, v30.d[0]
	fmla	v7.2d, v17.2d, v30.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	fmla	v6.2d, v18.2d, v30.d[1]
	fmla	v7.2d, v19.2d, v30.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	fmla	v1.2d, v21.2d, v25.d[0]
	fmla	v2.2d, v20.2d, v27.d[0]
	fmla	v3.2d, v21.2d, v27.d[0]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v5.2d, v21.2d, v29.d[0]
	fmla	v6.2d, v20.2d, v31.d[0]
	fmla	v7.2d, v21.2d, v31.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]
	fmla	v1.2d, v23.2d, v25.d[1]
	fmla	v2.2d, v22.2d, v27.d[1]
	fmla	v3.2d, v23.2d, v27.d[1]
	fmla	v4.2d, v22.2d, v29.d[1]
	fmla	v5.2d, v23.2d, v29.d[1]
	fmla	v6.2d, v22.2d, v31.d[1]
	fmla	v7.2d, v23.2d, v31.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldp		q30, q31, [x15], #32
	ldp		q16, q17, [x9]
	add		x9, x9, x10
	ldp		q18, q19, [x9]
	add		x9, x9, x10
	ldp		q20, q21, [x9]
	add		x9, x9, x10
	ldp		q22, q23, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
	fmla	v6.2d, v16.2d, v30.d[0]
	fmla	v7.2d, v17.2d, v30.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	fmla	v6.2d, v18.2d, v30.d[1]
	fmla	v7.2d, v19.2d, v30.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	fmla	v1.2d, v21.2d, v25.d[0]
	fmla	v2.2d, v20.2d, v27.d[0]
	fmla	v3.2d, v21.2d, v27.d[0]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v5.2d, v21.2d, v29.d[0]
	fmla	v6.2d, v20.2d, v31.d[0]
	fmla	v7.2d, v21.2d, v31.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]
	fmla	v1.2d, v23.2d, v25.d[1]
	fmla	v2.2d, v22.2d, v27.d[1]
	fmla	v3.2d, v23.2d, v27.d[1]
	fmla	v4.2d, v22.2d, v29.d[1]
	fmla	v5.2d, v23.2d, v29.d[1]
	fmla	v6.2d, v22.2d, v31.d[1]
	fmla	v7.2d, v23.2d, v31.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x13], #8
	ldr		d30, [x14], #8
	ldr		d31, [x15], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	cmp		w8, #0
	fmla	v6.2d, v24.2d, v31.d[0]
	fmla	v7.2d, v25.2d, v31.d[0]

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x11, x12
	add		x14, x13, x12
	add		x15, x14, x12

//	lsl		x16, x10, #2 // 4*lda*...
	lsl		x16, x10, #1 // 2*lda*...

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x16]

	// preload
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldp		q30, q31, [x15], #32
	ldp		q16, q17, [x9]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x11, #0]
//	prfm	PLDL1KEEP, [x13, #0]
//	prfm	PLDL1KEEP, [x14, #0]
//	prfm	PLDL1KEEP, [x15, #0]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x9, x16]

	// zero tmp acc
	fmov	d8, xzr
	fmov    d9, d8
	fmov    d10, d8
	fmov    d11, d8
	fmov    d12, d8
	fmov    d13, d8
	fmov    d14, d8
	fmov    d15, d8

//	add		x13, x12, #64
//	add		x13, x12, x12
//	add		x14, x13, #64

	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
	prfm	PLDL1KEEP, [x11, #32]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
	prfm	PLDL1KEEP, [x13, #32]
	fmla	v6.2d, v16.2d, v30.d[0]
	fmla	v7.2d, v17.2d, v30.d[0]

	// unroll 1
	ldp		q16, q17, [x9]
	fmla	v8.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v9.2d, v19.2d, v24.d[1]
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v10.2d, v18.2d, v26.d[1]
	fmla	v11.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x14, #32]
	fmla	v12.2d, v18.2d, v28.d[1]
	fmla	v13.2d, v19.2d, v28.d[1]
	prfm	PLDL1KEEP, [x15, #32]
	fmla	v14.2d, v18.2d, v30.d[1]
	fmla	v15.2d, v19.2d, v30.d[1]

	// unroll 2
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v25.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v25.d[0]
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v31.d[0]
	fmla	v7.2d, v17.2d, v31.d[0]

	// unroll 3
	ldp		q16, q17, [x9]
	fmla	v8.2d, v18.2d, v25.d[1]
	add		x9, x9, x10
	fmla	v9.2d, v19.2d, v25.d[1]
	ldp		q24, q25, [x11], #32
	fmla	v10.2d, v18.2d, v27.d[1]
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v11.2d, v19.2d, v27.d[1]
	ldp		q26, q27, [x13], #32
	fmla	v12.2d, v18.2d, v29.d[1]
	fmla	v13.2d, v19.2d, v29.d[1]
	ldp		q28, q29, [x14], #32
	fmla	v14.2d, v18.2d, v31.d[1]
	fmla	v15.2d, v19.2d, v31.d[1]
	ldp		q30, q31, [x15], #32

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d
	fadd	v1.2d, v1.2d, v9.2d
	fadd	v2.2d, v2.2d, v10.2d
	fadd	v3.2d, v3.2d, v11.2d
	fadd	v4.2d, v4.2d, v12.2d
	fadd	v5.2d, v5.2d, v13.2d
	fadd	v6.2d, v6.2d, v14.2d
	fadd	v7.2d, v7.2d, v15.2d

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, #256]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x9, #320]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x11, #256]
	fmla	v6.2d, v16.2d, v30.d[0]
	fmla	v7.2d, v17.2d, v30.d[0]

	// unroll 1
//	prfm	PLDL1KEEP, [x11, #320]
	ldp		q16, q17, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
//	add		x11, x11, x12
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	sub		w8, w8, #4
	fmla	v6.2d, v18.2d, v30.d[1]
	fmla	v7.2d, v19.2d, v30.d[1]

	// unroll 2
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v25.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v25.d[0]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v31.d[0]
	fmla	v7.2d, v17.2d, v31.d[0]

	// unroll 3
//	ldp		q16, q17, [x9]
	fmla	v0.2d, v18.2d, v25.d[1]
//	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v25.d[1]
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
	fmla	v4.2d, v18.2d, v29.d[1]
	fmla	v5.2d, v19.2d, v29.d[1]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
	fmla	v6.2d, v18.2d, v31.d[1]
	fmla	v7.2d, v19.2d, v31.d[1]
//	ldp		q30, q31, [x11, #(0*8+3*32)]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x13, x13, #32
	sub		x14, x14, #32
	sub		x15, x15, #32
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x13], #8
	ldr		d30, [x14], #8
	ldr		d31, [x15], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	cmp		w8, #0
	fmla	v6.2d, v24.2d, v31.d[0]
	fmla	v7.2d, v25.2d, v31.d[0]

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nn_4x4_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_3X4_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nn_3x4_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x11, x12
	add		x14, x13, x12
	add		x15, x14, x12

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldp		q30, q31, [x15], #32
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	add		x9, x9, x10
	ldr		q20, [x9]
	ldr		d21, [x9, #16]
	add		x9, x9, x10
	ldr		q22, [x9]
	ldr		d23, [x9, #16]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
	fmla	v6.2d, v16.2d, v30.d[0]
	fmla	v7.2d, v17.2d, v30.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	fmla	v6.2d, v18.2d, v30.d[1]
	fmla	v7.2d, v19.2d, v30.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	fmla	v1.2d, v21.2d, v25.d[0]
	fmla	v2.2d, v20.2d, v27.d[0]
	fmla	v3.2d, v21.2d, v27.d[0]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v5.2d, v21.2d, v29.d[0]
	fmla	v6.2d, v20.2d, v31.d[0]
	fmla	v7.2d, v21.2d, v31.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]
	fmla	v1.2d, v23.2d, v25.d[1]
	fmla	v2.2d, v22.2d, v27.d[1]
	fmla	v3.2d, v23.2d, v27.d[1]
	fmla	v4.2d, v22.2d, v29.d[1]
	fmla	v5.2d, v23.2d, v29.d[1]
	fmla	v6.2d, v22.2d, v31.d[1]
	fmla	v7.2d, v23.2d, v31.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldp		q30, q31, [x15], #32
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	add		x9, x9, x10
	ldr		q20, [x9]
	ldr		d21, [x9, #16]
	add		x9, x9, x10
	ldr		q22, [x9]
	ldr		d23, [x9, #16]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
	fmla	v6.2d, v16.2d, v30.d[0]
	fmla	v7.2d, v17.2d, v30.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	fmla	v6.2d, v18.2d, v30.d[1]
	fmla	v7.2d, v19.2d, v30.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	fmla	v1.2d, v21.2d, v25.d[0]
	fmla	v2.2d, v20.2d, v27.d[0]
	fmla	v3.2d, v21.2d, v27.d[0]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v5.2d, v21.2d, v29.d[0]
	fmla	v6.2d, v20.2d, v31.d[0]
	fmla	v7.2d, v21.2d, v31.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]
	fmla	v1.2d, v23.2d, v25.d[1]
	fmla	v2.2d, v22.2d, v27.d[1]
	fmla	v3.2d, v23.2d, v27.d[1]
	fmla	v4.2d, v22.2d, v29.d[1]
	fmla	v5.2d, v23.2d, v29.d[1]
	fmla	v6.2d, v22.2d, v31.d[1]
	fmla	v7.2d, v23.2d, v31.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9]
	ldr		d25, [x9, #16]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x13], #8
	ldr		d30, [x14], #8
	ldr		d31, [x15], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	cmp		w8, #0
	fmla	v6.2d, v24.2d, v31.d[0]
	fmla	v7.2d, v25.2d, v31.d[0]

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x11, x12
	add		x14, x13, x12
	add		x15, x14, x12

//	lsl		x16, x10, #2 // 4*lda*...
	lsl		x16, x10, #1 // 2*lda*...

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x16]

	// preload
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldp		q30, q31, [x15], #32
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x11, #0]
//	prfm	PLDL1KEEP, [x13, #0]
//	prfm	PLDL1KEEP, [x14, #0]
//	prfm	PLDL1KEEP, [x15, #0]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x9, x16]

	// zero tmp acc
	fmov	d8, xzr
	fmov    d9, d8
	fmov    d10, d8
	fmov    d11, d8
	fmov    d12, d8
	fmov    d13, d8
	fmov    d14, d8
	fmov    d15, d8

//	add		x13, x12, #64
//	add		x13, x12, x12
//	add		x14, x13, #64

	// main loop
1:
	
	// unroll 0
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
	prfm	PLDL1KEEP, [x11, #32]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
	prfm	PLDL1KEEP, [x13, #32]
	fmla	v6.2d, v16.2d, v30.d[0]
	fmla	v7.2d, v17.2d, v30.d[0]

	// unroll 1
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v8.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v9.2d, v19.2d, v24.d[1]
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v10.2d, v18.2d, v26.d[1]
	fmla	v11.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x14, #32]
	fmla	v12.2d, v18.2d, v28.d[1]
	fmla	v13.2d, v19.2d, v28.d[1]
	prfm	PLDL1KEEP, [x15, #32]
	fmla	v14.2d, v18.2d, v30.d[1]
	fmla	v15.2d, v19.2d, v30.d[1]

	// unroll 2
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v25.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v25.d[0]
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v31.d[0]
	fmla	v7.2d, v17.2d, v31.d[0]

	// unroll 3
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v8.2d, v18.2d, v25.d[1]
	add		x9, x9, x10
	fmla	v9.2d, v19.2d, v25.d[1]
	ldp		q24, q25, [x11], #32
	fmla	v10.2d, v18.2d, v27.d[1]
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v11.2d, v19.2d, v27.d[1]
	ldp		q26, q27, [x13], #32
	fmla	v12.2d, v18.2d, v29.d[1]
	fmla	v13.2d, v19.2d, v29.d[1]
	ldp		q28, q29, [x14], #32
	fmla	v14.2d, v18.2d, v31.d[1]
	fmla	v15.2d, v19.2d, v31.d[1]
	ldp		q30, q31, [x15], #32

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d
	fadd	v1.2d, v1.2d, v9.2d
	fadd	v2.2d, v2.2d, v10.2d
	fadd	v3.2d, v3.2d, v11.2d
	fadd	v4.2d, v4.2d, v12.2d
	fadd	v5.2d, v5.2d, v13.2d
	fadd	v6.2d, v6.2d, v14.2d
	fadd	v7.2d, v7.2d, v15.2d

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, #256]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x9, #320]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x11, #256]
	fmla	v6.2d, v16.2d, v30.d[0]
	fmla	v7.2d, v17.2d, v30.d[0]

	// unroll 1
//	prfm	PLDL1KEEP, [x11, #320]
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v0.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
//	add		x11, x11, x12
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	sub		w8, w8, #4
	fmla	v6.2d, v18.2d, v30.d[1]
	fmla	v7.2d, v19.2d, v30.d[1]

	// unroll 2
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v25.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v25.d[0]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v31.d[0]
	fmla	v7.2d, v17.2d, v31.d[0]

	// unroll 3
//	ldr		q16, [x9]
//	ldr		d17, [x9, #16]
	fmla	v0.2d, v18.2d, v25.d[1]
//	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v25.d[1]
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
	fmla	v4.2d, v18.2d, v29.d[1]
	fmla	v5.2d, v19.2d, v29.d[1]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
	fmla	v6.2d, v18.2d, v31.d[1]
	fmla	v7.2d, v19.2d, v31.d[1]
//	ldp		q30, q31, [x11, #(0*8+3*32)]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x13, x13, #32
	sub		x14, x14, #32
	sub		x15, x15, #32
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9]
	ldr		d25, [x9, #16]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x13], #8
	ldr		d30, [x14], #8
	ldr		d31, [x15], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	cmp		w8, #0
	fmla	v6.2d, v24.2d, v31.d[0]
	fmla	v7.2d, v25.2d, v31.d[0]

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nn_3x4_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_2X4_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nn_2x4_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x11, x12
	add		x14, x13, x12
	add		x15, x14, x12

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldp		q30, q31, [x15], #32
	ldr		q16, [x9]
	add		x9, x9, x10
	ldr		q18, [x9]
	add		x9, x9, x10
	ldr		q20, [x9]
	add		x9, x9, x10
	ldr		q22, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v6.2d, v16.2d, v30.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v6.2d, v18.2d, v30.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	fmla	v2.2d, v20.2d, v27.d[0]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v6.2d, v20.2d, v31.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]
	fmla	v2.2d, v22.2d, v27.d[1]
	fmla	v4.2d, v22.2d, v29.d[1]
	fmla	v6.2d, v22.2d, v31.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldp		q30, q31, [x15], #32
	ldr		q16, [x9]
	add		x9, x9, x10
	ldr		q18, [x9]
	add		x9, x9, x10
	ldr		q20, [x9]
	add		x9, x9, x10
	ldr		q22, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v6.2d, v16.2d, v30.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v6.2d, v18.2d, v30.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	fmla	v2.2d, v20.2d, v27.d[0]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v6.2d, v20.2d, v31.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]
	fmla	v2.2d, v22.2d, v27.d[1]
	fmla	v4.2d, v22.2d, v29.d[1]
	fmla	v6.2d, v22.2d, v31.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x13], #8
	ldr		d30, [x14], #8
	ldr		d31, [x15], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v30.d[0]
	cmp		w8, #0
	fmla	v6.2d, v24.2d, v31.d[0]

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x11, x12
	add		x14, x13, x12
	add		x15, x14, x12

//	lsl		x16, x10, #2 // 4*lda*...
	lsl		x16, x10, #1 // 2*lda*...

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x16]

	// preload
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldp		q30, q31, [x15], #32
	ldr		q16, [x9]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x11, #0]
//	prfm	PLDL1KEEP, [x13, #0]
//	prfm	PLDL1KEEP, [x14, #0]
//	prfm	PLDL1KEEP, [x15, #0]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x9, x16]

	// zero tmp acc
	fmov	d8, xzr
	fmov    d10, d8
	fmov    d12, d8
	fmov    d14, d8

//	add		x13, x12, #64
//	add		x13, x12, x12
//	add		x14, x13, #64

	// main loop
1:
	
	// unroll 0
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v2.2d, v16.2d, v26.d[0]
	prfm	PLDL1KEEP, [x11, #32]
	fmla	v4.2d, v16.2d, v28.d[0]
	prfm	PLDL1KEEP, [x13, #32]
	fmla	v6.2d, v16.2d, v30.d[0]

	// unroll 1
	ldr		q16, [x9]
	fmla	v8.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v10.2d, v18.2d, v26.d[1]
	prfm	PLDL1KEEP, [x14, #32]
	fmla	v12.2d, v18.2d, v28.d[1]
	prfm	PLDL1KEEP, [x15, #32]
	fmla	v14.2d, v18.2d, v30.d[1]

	// unroll 2
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v25.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v2.2d, v16.2d, v27.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v29.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v31.d[0]

	// unroll 3
	ldr		q16, [x9]
	fmla	v8.2d, v18.2d, v25.d[1]
	add		x9, x9, x10
	ldp		q24, q25, [x11], #32
	fmla	v10.2d, v18.2d, v27.d[1]
	prfm	PLDL1KEEP, [x9, x16]
	ldp		q26, q27, [x13], #32
	fmla	v12.2d, v18.2d, v29.d[1]
	ldp		q28, q29, [x14], #32
	fmla	v14.2d, v18.2d, v31.d[1]
	ldp		q30, q31, [x15], #32

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d
	fadd	v2.2d, v2.2d, v10.2d
	fadd	v4.2d, v4.2d, v12.2d
	fadd	v6.2d, v6.2d, v14.2d

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
//	prfm	PLDL1KEEP, [x9, #256]
	fmla	v2.2d, v16.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x9, #320]
	fmla	v4.2d, v16.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x11, #256]
	fmla	v6.2d, v16.2d, v30.d[0]

	// unroll 1
//	prfm	PLDL1KEEP, [x11, #320]
	ldr		q16, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v2.2d, v18.2d, v26.d[1]
//	add		x11, x11, x12
	fmla	v4.2d, v18.2d, v28.d[1]
	sub		w8, w8, #4
	fmla	v6.2d, v18.2d, v30.d[1]

	// unroll 2
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v25.d[0]
	add		x9, x9, x10
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v4.2d, v16.2d, v29.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v31.d[0]

	// unroll 3
//	ldr		q16, [x9]
	fmla	v0.2d, v18.2d, v25.d[1]
//	add		x9, x9, x10
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v2.2d, v18.2d, v27.d[1]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
	fmla	v4.2d, v18.2d, v29.d[1]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
	fmla	v6.2d, v18.2d, v31.d[1]
//	ldp		q30, q31, [x11, #(0*8+3*32)]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x13, x13, #32
	sub		x14, x14, #32
	sub		x15, x15, #32
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x13], #8
	ldr		d30, [x14], #8
	ldr		d31, [x15], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v30.d[0]
	cmp		w8, #0
	fmla	v6.2d, v24.2d, v31.d[0]

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nn_2x4_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_1X4_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nn_1x4_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x11, x12
	add		x14, x13, x12
	add		x15, x14, x12

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldp		q30, q31, [x15], #32
	ldr		d16, [x9]
	add		x9, x9, x10
	ldr		d18, [x9]
	add		x9, x9, x10
	ldr		d20, [x9]
	add		x9, x9, x10
	ldr		d22, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v6.2d, v16.2d, v30.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v6.2d, v18.2d, v30.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	fmla	v2.2d, v20.2d, v27.d[0]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v6.2d, v20.2d, v31.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]
	fmla	v2.2d, v22.2d, v27.d[1]
	fmla	v4.2d, v22.2d, v29.d[1]
	fmla	v6.2d, v22.2d, v31.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldp		q30, q31, [x15], #32
	ldr		d16, [x9]
	add		x9, x9, x10
	ldr		d18, [x9]
	add		x9, x9, x10
	ldr		d20, [x9]
	add		x9, x9, x10
	ldr		d22, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v6.2d, v16.2d, v30.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v6.2d, v18.2d, v30.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	fmla	v2.2d, v20.2d, v27.d[0]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v6.2d, v20.2d, v31.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]
	fmla	v2.2d, v22.2d, v27.d[1]
	fmla	v4.2d, v22.2d, v29.d[1]
	fmla	v6.2d, v22.2d, v31.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		d24, [x9]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x13], #8
	ldr		d30, [x14], #8
	ldr		d31, [x15], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v30.d[0]
	cmp		w8, #0
	fmla	v6.2d, v24.2d, v31.d[0]

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x11, x12
	add		x14, x13, x12
	add		x15, x14, x12

//	lsl		x16, x10, #2 // 4*lda*...
	lsl		x16, x10, #1 // 2*lda*...

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x16]

	// preload
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldp		q30, q31, [x15], #32
	ldr		d16, [x9]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x11, #0]
//	prfm	PLDL1KEEP, [x13, #0]
//	prfm	PLDL1KEEP, [x14, #0]
//	prfm	PLDL1KEEP, [x15, #0]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x9, x16]

	// zero tmp acc
	fmov	d8, xzr
	fmov    d10, d8
	fmov    d12, d8
	fmov    d14, d8

//	add		x13, x12, #64
//	add		x13, x12, x12
//	add		x14, x13, #64

	// main loop
1:
	
	// unroll 0
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v2.2d, v16.2d, v26.d[0]
	prfm	PLDL1KEEP, [x11, #32]
	fmla	v4.2d, v16.2d, v28.d[0]
	prfm	PLDL1KEEP, [x13, #32]
	fmla	v6.2d, v16.2d, v30.d[0]

	// unroll 1
	ldr		d16, [x9]
	fmla	v8.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v10.2d, v18.2d, v26.d[1]
	prfm	PLDL1KEEP, [x14, #32]
	fmla	v12.2d, v18.2d, v28.d[1]
	prfm	PLDL1KEEP, [x15, #32]
	fmla	v14.2d, v18.2d, v30.d[1]

	// unroll 2
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v25.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v2.2d, v16.2d, v27.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v29.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v31.d[0]

	// unroll 3
	ldr		d16, [x9]
	fmla	v8.2d, v18.2d, v25.d[1]
	add		x9, x9, x10
	ldp		q24, q25, [x11], #32
	fmla	v10.2d, v18.2d, v27.d[1]
	prfm	PLDL1KEEP, [x9, x16]
	ldp		q26, q27, [x13], #32
	fmla	v12.2d, v18.2d, v29.d[1]
	ldp		q28, q29, [x14], #32
	fmla	v14.2d, v18.2d, v31.d[1]
	ldp		q30, q31, [x15], #32

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d
	fadd	v2.2d, v2.2d, v10.2d
	fadd	v4.2d, v4.2d, v12.2d
	fadd	v6.2d, v6.2d, v14.2d

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
//	prfm	PLDL1KEEP, [x9, #256]
	fmla	v2.2d, v16.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x9, #320]
	fmla	v4.2d, v16.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x11, #256]
	fmla	v6.2d, v16.2d, v30.d[0]

	// unroll 1
//	prfm	PLDL1KEEP, [x11, #320]
	ldr		d16, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v2.2d, v18.2d, v26.d[1]
//	add		x11, x11, x12
	fmla	v4.2d, v18.2d, v28.d[1]
	sub		w8, w8, #4
	fmla	v6.2d, v18.2d, v30.d[1]

	// unroll 2
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v25.d[0]
	add		x9, x9, x10
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v4.2d, v16.2d, v29.d[0]
	cmp		w8, #4
	fmla	v6.2d, v16.2d, v31.d[0]

	// unroll 3
//	ldr		d16, [x9]
	fmla	v0.2d, v18.2d, v25.d[1]
//	add		x9, x9, x10
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v2.2d, v18.2d, v27.d[1]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
	fmla	v4.2d, v18.2d, v29.d[1]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
	fmla	v6.2d, v18.2d, v31.d[1]
//	ldp		q30, q31, [x11, #(0*8+3*32)]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x13, x13, #32
	sub		x14, x14, #32
	sub		x15, x15, #32
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldr		d24, [x9]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x13], #8
	ldr		d30, [x14], #8
	ldr		d31, [x15], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v30.d[0]
	cmp		w8, #0
	fmla	v6.2d, v24.2d, v31.d[0]

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nn_1x4_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_4X3_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nn_4x3_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x11, x12
	add		x14, x13, x12

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldp		q16, q17, [x9]
	add		x9, x9, x10
	ldp		q18, q19, [x9]
	add		x9, x9, x10
	ldp		q20, q21, [x9]
	add		x9, x9, x10
	ldp		q22, q23, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	fmla	v1.2d, v21.2d, v25.d[0]
	fmla	v2.2d, v20.2d, v27.d[0]
	fmla	v3.2d, v21.2d, v27.d[0]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v5.2d, v21.2d, v29.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]
	fmla	v1.2d, v23.2d, v25.d[1]
	fmla	v2.2d, v22.2d, v27.d[1]
	fmla	v3.2d, v23.2d, v27.d[1]
	fmla	v4.2d, v22.2d, v29.d[1]
	fmla	v5.2d, v23.2d, v29.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldp		q16, q17, [x9]
	add		x9, x9, x10
	ldp		q18, q19, [x9]
	add		x9, x9, x10
	ldp		q20, q21, [x9]
	add		x9, x9, x10
	ldp		q22, q23, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	fmla	v1.2d, v21.2d, v25.d[0]
	fmla	v2.2d, v20.2d, v27.d[0]
	fmla	v3.2d, v21.2d, v27.d[0]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v5.2d, v21.2d, v29.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]
	fmla	v1.2d, v23.2d, v25.d[1]
	fmla	v2.2d, v22.2d, v27.d[1]
	fmla	v3.2d, v23.2d, v27.d[1]
	fmla	v4.2d, v22.2d, v29.d[1]
	fmla	v5.2d, v23.2d, v29.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x13], #8
	ldr		d30, [x14], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x11, x12
	add		x14, x13, x12

//	lsl		x16, x10, #2 // 4*lda*...
	lsl		x16, x10, #1 // 2*lda*...

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x16]

	// preload
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldp		q16, q17, [x9]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x11, #0]
//	prfm	PLDL1KEEP, [x13, #0]
//	prfm	PLDL1KEEP, [x14, #0]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x9, x16]

	// zero tmp acc
	fmov	d8, xzr
	fmov    d9, d8
	fmov    d10, d8
	fmov    d11, d8
	fmov    d12, d8
	fmov    d13, d8

//	add		x13, x12, #64
//	add		x13, x12, x12
//	add		x14, x13, #64

	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
	prfm	PLDL1KEEP, [x11, #32]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
	prfm	PLDL1KEEP, [x13, #32]

	// unroll 1
	ldp		q16, q17, [x9]
	fmla	v8.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v9.2d, v19.2d, v24.d[1]
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v10.2d, v18.2d, v26.d[1]
	fmla	v11.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x14, #32]
	fmla	v12.2d, v18.2d, v28.d[1]
	fmla	v13.2d, v19.2d, v28.d[1]

	// unroll 2
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v25.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v25.d[0]
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	cmp		w8, #4

	// unroll 3
	ldp		q16, q17, [x9]
	fmla	v8.2d, v18.2d, v25.d[1]
	add		x9, x9, x10
	fmla	v9.2d, v19.2d, v25.d[1]
	ldp		q24, q25, [x11], #32
	fmla	v10.2d, v18.2d, v27.d[1]
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v11.2d, v19.2d, v27.d[1]
	ldp		q26, q27, [x13], #32
	fmla	v12.2d, v18.2d, v29.d[1]
	fmla	v13.2d, v19.2d, v29.d[1]
	ldp		q28, q29, [x14], #32

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d
	fadd	v1.2d, v1.2d, v9.2d
	fadd	v2.2d, v2.2d, v10.2d
	fadd	v3.2d, v3.2d, v11.2d
	fadd	v4.2d, v4.2d, v12.2d
	fadd	v5.2d, v5.2d, v13.2d

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, #256]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x9, #320]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x11, #256]

	// unroll 1
//	prfm	PLDL1KEEP, [x11, #320]
	ldp		q16, q17, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
//	add		x11, x11, x12
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v25.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v25.d[0]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	cmp		w8, #4

	// unroll 3
//	ldp		q16, q17, [x9]
	fmla	v0.2d, v18.2d, v25.d[1]
//	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v25.d[1]
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
	fmla	v4.2d, v18.2d, v29.d[1]
	fmla	v5.2d, v19.2d, v29.d[1]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	ldp		q30, q31, [x11, #(0*8+3*32)]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x13, x13, #32
	sub		x14, x14, #32
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x13], #8
	ldr		d30, [x14], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nn_4x3_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_3X3_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nn_3x3_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x11, x12
	add		x14, x13, x12

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	add		x9, x9, x10
	ldr		q20, [x9]
	ldr		d21, [x9, #16]
	add		x9, x9, x10
	ldr		q22, [x9]
	ldr		d23, [x9, #16]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	fmla	v1.2d, v21.2d, v25.d[0]
	fmla	v2.2d, v20.2d, v27.d[0]
	fmla	v3.2d, v21.2d, v27.d[0]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v5.2d, v21.2d, v29.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]
	fmla	v1.2d, v23.2d, v25.d[1]
	fmla	v2.2d, v22.2d, v27.d[1]
	fmla	v3.2d, v23.2d, v27.d[1]
	fmla	v4.2d, v22.2d, v29.d[1]
	fmla	v5.2d, v23.2d, v29.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	add		x9, x9, x10
	ldr		q20, [x9]
	ldr		d21, [x9, #16]
	add		x9, x9, x10
	ldr		q22, [x9]
	ldr		d23, [x9, #16]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	fmla	v1.2d, v21.2d, v25.d[0]
	fmla	v2.2d, v20.2d, v27.d[0]
	fmla	v3.2d, v21.2d, v27.d[0]
	fmla	v4.2d, v20.2d, v29.d[0]
	fmla	v5.2d, v21.2d, v29.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]
	fmla	v1.2d, v23.2d, v25.d[1]
	fmla	v2.2d, v22.2d, v27.d[1]
	fmla	v3.2d, v23.2d, v27.d[1]
	fmla	v4.2d, v22.2d, v29.d[1]
	fmla	v5.2d, v23.2d, v29.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9]
	ldr		d25, [x9, #16]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x13], #8
	ldr		d30, [x14], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x11, x12
	add		x14, x13, x12

//	lsl		x16, x10, #2 // 4*lda*...
	lsl		x16, x10, #1 // 2*lda*...

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x16]

	// preload
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x11, #0]
//	prfm	PLDL1KEEP, [x13, #0]
//	prfm	PLDL1KEEP, [x14, #0]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x9, x16]

	// zero tmp acc
	fmov	d8, xzr
	fmov    d9, d8
	fmov    d10, d8
	fmov    d11, d8
	fmov    d12, d8
	fmov    d13, d8

//	add		x13, x12, #64
//	add		x13, x12, x12
//	add		x14, x13, #64

	// main loop
1:
	
	// unroll 0
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
	prfm	PLDL1KEEP, [x11, #32]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
	prfm	PLDL1KEEP, [x13, #32]

	// unroll 1
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v8.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v9.2d, v19.2d, v24.d[1]
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v10.2d, v18.2d, v26.d[1]
	fmla	v11.2d, v19.2d, v26.d[1]
	prfm	PLDL1KEEP, [x14, #32]
	fmla	v12.2d, v18.2d, v28.d[1]
	fmla	v13.2d, v19.2d, v28.d[1]

	// unroll 2
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v25.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v25.d[0]
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	cmp		w8, #4

	// unroll 3
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v8.2d, v18.2d, v25.d[1]
	add		x9, x9, x10
	fmla	v9.2d, v19.2d, v25.d[1]
	ldp		q24, q25, [x11], #32
	fmla	v10.2d, v18.2d, v27.d[1]
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v11.2d, v19.2d, v27.d[1]
	ldp		q26, q27, [x13], #32
	fmla	v12.2d, v18.2d, v29.d[1]
	fmla	v13.2d, v19.2d, v29.d[1]
	ldp		q28, q29, [x14], #32

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d
	fadd	v1.2d, v1.2d, v9.2d
	fadd	v2.2d, v2.2d, v10.2d
	fadd	v3.2d, v3.2d, v11.2d
	fadd	v4.2d, v4.2d, v12.2d
	fadd	v5.2d, v5.2d, v13.2d

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, #256]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x9, #320]
	fmla	v4.2d, v16.2d, v28.d[0]
	fmla	v5.2d, v17.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x11, #256]

	// unroll 1
//	prfm	PLDL1KEEP, [x11, #320]
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v0.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
//	add		x11, x11, x12
	fmla	v4.2d, v18.2d, v28.d[1]
	fmla	v5.2d, v19.2d, v28.d[1]
	sub		w8, w8, #4

	// unroll 2
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v25.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v25.d[0]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	fmla	v4.2d, v16.2d, v29.d[0]
	fmla	v5.2d, v17.2d, v29.d[0]
	cmp		w8, #4

	// unroll 3
//	ldr		q16, [x9]
//	ldr		d17, [x9, #16]
	fmla	v0.2d, v18.2d, v25.d[1]
//	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v25.d[1]
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
	fmla	v4.2d, v18.2d, v29.d[1]
	fmla	v5.2d, v19.2d, v29.d[1]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	ldp		q30, q31, [x11, #(0*8+3*32)]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x13, x13, #32
	sub		x14, x14, #32
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9]
	ldr		d25, [x9, #16]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x13], #8
	ldr		d30, [x14], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nn_3x3_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_2X3_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nn_2x3_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x11, x12
	add		x14, x13, x12

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		q16, [x9]
	add		x9, x9, x10
	ldr		q18, [x9]
	add		x9, x9, x10
	ldr		q20, [x9]
	add		x9, x9, x10
	ldr		q22, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v4.2d, v16.2d, v28.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	fmla	v2.2d, v20.2d, v27.d[0]
	fmla	v4.2d, v20.2d, v29.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]
	fmla	v2.2d, v22.2d, v27.d[1]
	fmla	v4.2d, v22.2d, v29.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		q16, [x9]
	add		x9, x9, x10
	ldr		q18, [x9]
	add		x9, x9, x10
	ldr		q20, [x9]
	add		x9, x9, x10
	ldr		q22, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v4.2d, v16.2d, v28.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	fmla	v2.2d, v20.2d, v27.d[0]
	fmla	v4.2d, v20.2d, v29.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]
	fmla	v2.2d, v22.2d, v27.d[1]
	fmla	v4.2d, v22.2d, v29.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x13], #8
	ldr		d30, [x14], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v30.d[0]
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x11, x12
	add		x14, x13, x12

//	lsl		x16, x10, #2 // 4*lda*...
	lsl		x16, x10, #1 // 2*lda*...

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x16]

	// preload
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		q16, [x9]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x11, #0]
//	prfm	PLDL1KEEP, [x13, #0]
//	prfm	PLDL1KEEP, [x14, #0]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x9, x16]

	// zero tmp acc
	fmov	d8, xzr
	fmov    d10, d8
	fmov    d12, d8

//	add		x13, x12, #64
//	add		x13, x12, x12
//	add		x14, x13, #64

	// main loop
1:
	
	// unroll 0
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v2.2d, v16.2d, v26.d[0]
	prfm	PLDL1KEEP, [x11, #32]
	fmla	v4.2d, v16.2d, v28.d[0]
	prfm	PLDL1KEEP, [x13, #32]

	// unroll 1
	ldr		q16, [x9]
	fmla	v8.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v10.2d, v18.2d, v26.d[1]
	prfm	PLDL1KEEP, [x14, #32]
	fmla	v12.2d, v18.2d, v28.d[1]

	// unroll 2
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v25.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v2.2d, v16.2d, v27.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v29.d[0]
	cmp		w8, #4

	// unroll 3
	ldr		q16, [x9]
	fmla	v8.2d, v18.2d, v25.d[1]
	add		x9, x9, x10
	ldp		q24, q25, [x11], #32
	fmla	v10.2d, v18.2d, v27.d[1]
	prfm	PLDL1KEEP, [x9, x16]
	ldp		q26, q27, [x13], #32
	fmla	v12.2d, v18.2d, v29.d[1]
	ldp		q28, q29, [x14], #32

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d
	fadd	v2.2d, v2.2d, v10.2d
	fadd	v4.2d, v4.2d, v12.2d

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
//	prfm	PLDL1KEEP, [x9, #256]
	fmla	v2.2d, v16.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x9, #320]
	fmla	v4.2d, v16.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x11, #256]

	// unroll 1
//	prfm	PLDL1KEEP, [x11, #320]
	ldr		q16, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v2.2d, v18.2d, v26.d[1]
//	add		x11, x11, x12
	fmla	v4.2d, v18.2d, v28.d[1]
	sub		w8, w8, #4

	// unroll 2
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v25.d[0]
	add		x9, x9, x10
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v4.2d, v16.2d, v29.d[0]
	cmp		w8, #4

	// unroll 3
//	ldr		q16, [x9]
	fmla	v0.2d, v18.2d, v25.d[1]
//	add		x9, x9, x10
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v2.2d, v18.2d, v27.d[1]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
	fmla	v4.2d, v18.2d, v29.d[1]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	ldp		q30, q31, [x11, #(0*8+3*32)]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x13, x13, #32
	sub		x14, x14, #32
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x13], #8
	ldr		d30, [x14], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nn_2x3_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_1X3_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nn_1x3_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x11, x12
	add		x14, x13, x12

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d16, [x9]
	add		x9, x9, x10
	ldr		d18, [x9]
	add		x9, x9, x10
	ldr		d20, [x9]
	add		x9, x9, x10
	ldr		d22, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v4.2d, v16.2d, v28.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	fmla	v2.2d, v20.2d, v27.d[0]
	fmla	v4.2d, v20.2d, v29.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]
	fmla	v2.2d, v22.2d, v27.d[1]
	fmla	v4.2d, v22.2d, v29.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d16, [x9]
	add		x9, x9, x10
	ldr		d18, [x9]
	add		x9, x9, x10
	ldr		d20, [x9]
	add		x9, x9, x10
	ldr		d22, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v4.2d, v16.2d, v28.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v4.2d, v18.2d, v28.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	fmla	v2.2d, v20.2d, v27.d[0]
	fmla	v4.2d, v20.2d, v29.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]
	fmla	v2.2d, v22.2d, v27.d[1]
	fmla	v4.2d, v22.2d, v29.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		d24, [x9]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x13], #8
	ldr		d30, [x14], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v30.d[0]
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x11, x12
	add		x14, x13, x12

//	lsl		x16, x10, #2 // 4*lda*...
	lsl		x16, x10, #1 // 2*lda*...

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x16]

	// preload
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x13], #32
	ldp		q28, q29, [x14], #32
	ldr		d16, [x9]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x11, #0]
//	prfm	PLDL1KEEP, [x13, #0]
//	prfm	PLDL1KEEP, [x14, #0]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x9, x16]

	// zero tmp acc
	fmov	d8, xzr
	fmov    d10, d8
	fmov    d12, d8

//	add		x13, x12, #64
//	add		x13, x12, x12
//	add		x14, x13, #64

	// main loop
1:
	
	// unroll 0
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v2.2d, v16.2d, v26.d[0]
	prfm	PLDL1KEEP, [x11, #32]
	fmla	v4.2d, v16.2d, v28.d[0]
	prfm	PLDL1KEEP, [x13, #32]

	// unroll 1
	ldr		d16, [x9]
	fmla	v8.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v10.2d, v18.2d, v26.d[1]
	prfm	PLDL1KEEP, [x14, #32]
	fmla	v12.2d, v18.2d, v28.d[1]

	// unroll 2
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v25.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v2.2d, v16.2d, v27.d[0]
	sub		w8, w8, #4
	fmla	v4.2d, v16.2d, v29.d[0]
	cmp		w8, #4

	// unroll 3
	ldr		d16, [x9]
	fmla	v8.2d, v18.2d, v25.d[1]
	add		x9, x9, x10
	ldp		q24, q25, [x11], #32
	fmla	v10.2d, v18.2d, v27.d[1]
	prfm	PLDL1KEEP, [x9, x16]
	ldp		q26, q27, [x13], #32
	fmla	v12.2d, v18.2d, v29.d[1]
	ldp		q28, q29, [x14], #32

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d
	fadd	v2.2d, v2.2d, v10.2d
	fadd	v4.2d, v4.2d, v12.2d

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
//	prfm	PLDL1KEEP, [x9, #256]
	fmla	v2.2d, v16.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x9, #320]
	fmla	v4.2d, v16.2d, v28.d[0]
//	prfm	PLDL1KEEP, [x11, #256]

	// unroll 1
//	prfm	PLDL1KEEP, [x11, #320]
	ldr		d16, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v2.2d, v18.2d, v26.d[1]
//	add		x11, x11, x12
	fmla	v4.2d, v18.2d, v28.d[1]
	sub		w8, w8, #4

	// unroll 2
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v25.d[0]
	add		x9, x9, x10
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v4.2d, v16.2d, v29.d[0]
	cmp		w8, #4

	// unroll 3
//	ldr		d16, [x9]
	fmla	v0.2d, v18.2d, v25.d[1]
//	add		x9, x9, x10
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v2.2d, v18.2d, v27.d[1]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
	fmla	v4.2d, v18.2d, v29.d[1]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	ldp		q30, q31, [x11, #(0*8+3*32)]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x13, x13, #32
	sub		x14, x14, #32
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldr		d24, [x9]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x13], #8
	ldr		d30, [x14], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	sub		w8, w8, #1
	fmla	v4.2d, v24.2d, v30.d[0]
	fmla	v5.2d, v25.2d, v30.d[0]
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nn_1x3_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_4X2_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nn_4x2_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x11, x12

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x13], #32
	ldp		q16, q17, [x9]
	add		x9, x9, x10
	ldp		q18, q19, [x9]
	add		x9, x9, x10
	ldp		q20, q21, [x9]
	add		x9, x9, x10
	ldp		q22, q23, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	fmla	v1.2d, v21.2d, v25.d[0]
	fmla	v2.2d, v20.2d, v27.d[0]
	fmla	v3.2d, v21.2d, v27.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]
	fmla	v1.2d, v23.2d, v25.d[1]
	fmla	v2.2d, v22.2d, v27.d[1]
	fmla	v3.2d, v23.2d, v27.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x13], #32
	ldp		q16, q17, [x9]
	add		x9, x9, x10
	ldp		q18, q19, [x9]
	add		x9, x9, x10
	ldp		q20, q21, [x9]
	add		x9, x9, x10
	ldp		q22, q23, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	fmla	v1.2d, v21.2d, v25.d[0]
	fmla	v2.2d, v20.2d, v27.d[0]
	fmla	v3.2d, v21.2d, v27.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]
	fmla	v1.2d, v23.2d, v25.d[1]
	fmla	v2.2d, v22.2d, v27.d[1]
	fmla	v3.2d, v23.2d, v27.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x13], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x11, x12

//	lsl		x16, x10, #2 // 4*lda*...
	lsl		x16, x10, #1 // 2*lda*...

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x13, #0]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x16]

	// preload
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x13], #32
	ldp		q16, q17, [x9]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x11, #0]
//	prfm	PLDL1KEEP, [x13, #0]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x9, x16]

	// zero tmp acc
	fmov	d8, xzr
	fmov    d9, d8
	fmov    d10, d8
	fmov    d11, d8

//	add		x13, x12, #64
//	add		x13, x12, x12
//	add		x14, x13, #64

	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
	prfm	PLDL1KEEP, [x11, #32]
	prfm	PLDL1KEEP, [x13, #32]

	// unroll 1
	ldp		q16, q17, [x9]
	fmla	v8.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v9.2d, v19.2d, v24.d[1]
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v10.2d, v18.2d, v26.d[1]
	fmla	v11.2d, v19.2d, v26.d[1]

	// unroll 2
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v25.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v25.d[0]
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	sub		w8, w8, #4
	cmp		w8, #4

	// unroll 3
	ldp		q16, q17, [x9]
	fmla	v8.2d, v18.2d, v25.d[1]
	add		x9, x9, x10
	fmla	v9.2d, v19.2d, v25.d[1]
	ldp		q24, q25, [x11], #32
	fmla	v10.2d, v18.2d, v27.d[1]
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v11.2d, v19.2d, v27.d[1]
	ldp		q26, q27, [x13], #32

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d
	fadd	v1.2d, v1.2d, v9.2d
	fadd	v2.2d, v2.2d, v10.2d
	fadd	v3.2d, v3.2d, v11.2d

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, #256]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x9, #320]
//	prfm	PLDL1KEEP, [x11, #256]

	// unroll 1
//	prfm	PLDL1KEEP, [x11, #320]
	ldp		q16, q17, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
//	add		x11, x11, x12
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v25.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v25.d[0]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	cmp		w8, #4

	// unroll 3
//	ldp		q16, q17, [x9]
	fmla	v0.2d, v18.2d, v25.d[1]
//	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v25.d[1]
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	ldp		q30, q31, [x11, #(0*8+3*32)]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x13, x13, #32
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x13], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nn_4x2_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_3X2_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nn_3x2_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x11, x12

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x13], #32
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	add		x9, x9, x10
	ldr		q20, [x9]
	ldr		d21, [x9, #16]
	add		x9, x9, x10
	ldr		q22, [x9]
	ldr		d23, [x9, #16]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	fmla	v1.2d, v21.2d, v25.d[0]
	fmla	v2.2d, v20.2d, v27.d[0]
	fmla	v3.2d, v21.2d, v27.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]
	fmla	v1.2d, v23.2d, v25.d[1]
	fmla	v2.2d, v22.2d, v27.d[1]
	fmla	v3.2d, v23.2d, v27.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x13], #32
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	add		x9, x9, x10
	ldr		q20, [x9]
	ldr		d21, [x9, #16]
	add		x9, x9, x10
	ldr		q22, [x9]
	ldr		d23, [x9, #16]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	fmla	v1.2d, v21.2d, v25.d[0]
	fmla	v2.2d, v20.2d, v27.d[0]
	fmla	v3.2d, v21.2d, v27.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]
	fmla	v1.2d, v23.2d, v25.d[1]
	fmla	v2.2d, v22.2d, v27.d[1]
	fmla	v3.2d, v23.2d, v27.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9]
	ldr		d25, [x9, #16]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x13], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x11, x12

//	lsl		x16, x10, #2 // 4*lda*...
	lsl		x16, x10, #1 // 2*lda*...

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x13, #0]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x16]

	// preload
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x13], #32
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x11, #0]
//	prfm	PLDL1KEEP, [x13, #0]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x9, x16]

	// zero tmp acc
	fmov	d8, xzr
	fmov    d9, d8
	fmov    d10, d8
	fmov    d11, d8

//	add		x13, x12, #64
//	add		x13, x12, x12
//	add		x14, x13, #64

	// main loop
1:
	
	// unroll 0
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
	prfm	PLDL1KEEP, [x11, #32]
	prfm	PLDL1KEEP, [x13, #32]

	// unroll 1
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v8.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v9.2d, v19.2d, v24.d[1]
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v10.2d, v18.2d, v26.d[1]
	fmla	v11.2d, v19.2d, v26.d[1]

	// unroll 2
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v25.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v25.d[0]
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	sub		w8, w8, #4
	cmp		w8, #4

	// unroll 3
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v8.2d, v18.2d, v25.d[1]
	add		x9, x9, x10
	fmla	v9.2d, v19.2d, v25.d[1]
	ldp		q24, q25, [x11], #32
	fmla	v10.2d, v18.2d, v27.d[1]
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v11.2d, v19.2d, v27.d[1]
	ldp		q26, q27, [x13], #32

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d
	fadd	v1.2d, v1.2d, v9.2d
	fadd	v2.2d, v2.2d, v10.2d
	fadd	v3.2d, v3.2d, v11.2d

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, #256]
	fmla	v2.2d, v16.2d, v26.d[0]
	fmla	v3.2d, v17.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x9, #320]
//	prfm	PLDL1KEEP, [x11, #256]

	// unroll 1
//	prfm	PLDL1KEEP, [x11, #320]
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v0.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	fmla	v3.2d, v19.2d, v26.d[1]
//	add		x11, x11, x12
	sub		w8, w8, #4

	// unroll 2
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v25.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v25.d[0]
	fmla	v2.2d, v16.2d, v27.d[0]
	fmla	v3.2d, v17.2d, v27.d[0]
	cmp		w8, #4

	// unroll 3
//	ldr		q16, [x9]
//	ldr		d17, [x9, #16]
	fmla	v0.2d, v18.2d, v25.d[1]
//	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v25.d[1]
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v2.2d, v18.2d, v27.d[1]
	fmla	v3.2d, v19.2d, v27.d[1]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	ldp		q30, q31, [x11, #(0*8+3*32)]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x13, x13, #32
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9]
	ldr		d25, [x9, #16]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x13], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nn_3x2_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_2X2_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nn_2x2_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x11, x12

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x13], #32
	ldr		q16, [x9]
	add		x9, x9, x10
	ldr		q18, [x9]
	add		x9, x9, x10
	ldr		q20, [x9]
	add		x9, x9, x10
	ldr		q22, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	fmla	v2.2d, v20.2d, v27.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]
	fmla	v2.2d, v22.2d, v27.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x13], #32
	ldr		q16, [x9]
	add		x9, x9, x10
	ldr		q18, [x9]
	add		x9, x9, x10
	ldr		q20, [x9]
	add		x9, x9, x10
	ldr		q22, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	fmla	v2.2d, v20.2d, v27.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]
	fmla	v2.2d, v22.2d, v27.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x13], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x11, x12

//	lsl		x16, x10, #2 // 4*lda*...
	lsl		x16, x10, #1 // 2*lda*...

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x13, #0]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x16]

	// preload
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x13], #32
	ldr		q16, [x9]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x11, #0]
//	prfm	PLDL1KEEP, [x13, #0]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x9, x16]

	// zero tmp acc
	fmov	d8, xzr
	fmov    d10, d8

//	add		x13, x12, #64
//	add		x13, x12, x12
//	add		x14, x13, #64

	// main loop
1:
	
	// unroll 0
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v2.2d, v16.2d, v26.d[0]
	prfm	PLDL1KEEP, [x11, #32]
	prfm	PLDL1KEEP, [x13, #32]

	// unroll 1
	ldr		q16, [x9]
	fmla	v8.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v10.2d, v18.2d, v26.d[1]

	// unroll 2
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v25.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v2.2d, v16.2d, v27.d[0]
	sub		w8, w8, #4
	cmp		w8, #4

	// unroll 3
	ldr		q16, [x9]
	fmla	v8.2d, v18.2d, v25.d[1]
	add		x9, x9, x10
	ldp		q24, q25, [x11], #32
	fmla	v10.2d, v18.2d, v27.d[1]
	prfm	PLDL1KEEP, [x9, x16]
	ldp		q26, q27, [x13], #32

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d
	fadd	v2.2d, v2.2d, v10.2d

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
//	prfm	PLDL1KEEP, [x9, #256]
	fmla	v2.2d, v16.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x9, #320]
//	prfm	PLDL1KEEP, [x11, #256]

	// unroll 1
//	prfm	PLDL1KEEP, [x11, #320]
	ldr		q16, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v2.2d, v18.2d, v26.d[1]
//	add		x11, x11, x12
	sub		w8, w8, #4

	// unroll 2
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v25.d[0]
	add		x9, x9, x10
	fmla	v2.2d, v16.2d, v27.d[0]
	cmp		w8, #4

	// unroll 3
//	ldr		q16, [x9]
	fmla	v0.2d, v18.2d, v25.d[1]
//	add		x9, x9, x10
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v2.2d, v18.2d, v27.d[1]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	ldp		q30, q31, [x11, #(0*8+3*32)]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x13, x13, #32
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x13], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nn_2x2_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_1X2_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nn_1x2_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x11, x12

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x13], #32
	ldr		d16, [x9]
	add		x9, x9, x10
	ldr		d18, [x9]
	add		x9, x9, x10
	ldr		d20, [x9]
	add		x9, x9, x10
	ldr		d22, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	fmla	v2.2d, v20.2d, v27.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]
	fmla	v2.2d, v22.2d, v27.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x13], #32
	ldr		d16, [x9]
	add		x9, x9, x10
	ldr		d18, [x9]
	add		x9, x9, x10
	ldr		d20, [x9]
	add		x9, x9, x10
	ldr		d22, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v2.2d, v16.2d, v26.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v2.2d, v18.2d, v26.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	fmla	v2.2d, v20.2d, v27.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]
	fmla	v2.2d, v22.2d, v27.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		d24, [x9]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x13], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x13, x11, x12

//	lsl		x16, x10, #2 // 4*lda*...
	lsl		x16, x10, #1 // 2*lda*...

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x13, #0]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x16]

	// preload
	ldp		q24, q25, [x11], #32
	ldp		q26, q27, [x13], #32
	ldr		d16, [x9]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x11, #0]
//	prfm	PLDL1KEEP, [x13, #0]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x9, x16]

	// zero tmp acc
	fmov	d8, xzr
	fmov    d10, d8

//	add		x13, x12, #64
//	add		x13, x12, x12
//	add		x14, x13, #64

	// main loop
1:
	
	// unroll 0
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v2.2d, v16.2d, v26.d[0]
	prfm	PLDL1KEEP, [x11, #32]
	prfm	PLDL1KEEP, [x13, #32]

	// unroll 1
	ldr		d16, [x9]
	fmla	v8.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v10.2d, v18.2d, v26.d[1]

	// unroll 2
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v25.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x16]
	fmla	v2.2d, v16.2d, v27.d[0]
	sub		w8, w8, #4
	cmp		w8, #4

	// unroll 3
	ldr		d16, [x9]
	fmla	v8.2d, v18.2d, v25.d[1]
	add		x9, x9, x10
	ldp		q24, q25, [x11], #32
	fmla	v10.2d, v18.2d, v27.d[1]
	prfm	PLDL1KEEP, [x9, x16]
	ldp		q26, q27, [x13], #32

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d
	fadd	v2.2d, v2.2d, v10.2d

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
//	prfm	PLDL1KEEP, [x9, #256]
	fmla	v2.2d, v16.2d, v26.d[0]
//	prfm	PLDL1KEEP, [x9, #320]
//	prfm	PLDL1KEEP, [x11, #256]

	// unroll 1
//	prfm	PLDL1KEEP, [x11, #320]
	ldr		d16, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v2.2d, v18.2d, v26.d[1]
//	add		x11, x11, x12
	sub		w8, w8, #4

	// unroll 2
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v25.d[0]
	add		x9, x9, x10
	fmla	v2.2d, v16.2d, v27.d[0]
	cmp		w8, #4

	// unroll 3
//	ldr		d16, [x9]
	fmla	v0.2d, v18.2d, v25.d[1]
//	add		x9, x9, x10
//	ldp		q24, q25, [x11, #(0*8+0*32)]
	fmla	v2.2d, v18.2d, v27.d[1]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	ldp		q30, q31, [x11, #(0*8+3*32)]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x13, x13, #32
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldr		d24, [x9]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	ldr		d29, [x13], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v2.2d, v24.2d, v29.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nn_1x2_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_4X1_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nn_4x1_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11], #32
	ldp		q16, q17, [x9]
	add		x9, x9, x10
	ldp		q18, q19, [x9]
	add		x9, x9, x10
	ldp		q20, q21, [x9]
	add		x9, x9, x10
	ldp		q22, q23, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	fmla	v1.2d, v21.2d, v25.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]
	fmla	v1.2d, v23.2d, v25.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11], #32
	ldp		q16, q17, [x9]
	add		x9, x9, x10
	ldp		q18, q19, [x9]
	add		x9, x9, x10
	ldp		q20, q21, [x9]
	add		x9, x9, x10
	ldp		q22, q23, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	fmla	v1.2d, v21.2d, v25.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]
	fmla	v1.2d, v23.2d, v25.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

//	lsl		x16, x10, #2 // 4*lda*...
	lsl		x16, x10, #1 // 2*lda*...

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x16]

	// preload
	ldp		q24, q25, [x11], #32
	ldp		q16, q17, [x9]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x11, #0]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x9, x16]

	// zero tmp acc
	fmov	d8, xzr
	fmov    d9, d8

//	add		x13, x12, #64
//	add		x13, x12, x12
//	add		x14, x13, #64

	// main loop
1:
	
	// unroll 0
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x16]
	prfm	PLDL1KEEP, [x11, #32]

	// unroll 1
	ldp		q16, q17, [x9]
	fmla	v8.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v9.2d, v19.2d, v24.d[1]
	prfm	PLDL1KEEP, [x9, x16]

	// unroll 2
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v25.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v25.d[0]
	prfm	PLDL1KEEP, [x9, x16]
	sub		w8, w8, #4
	cmp		w8, #4

	// unroll 3
	ldp		q16, q17, [x9]
	fmla	v8.2d, v18.2d, v25.d[1]
	add		x9, x9, x10
	fmla	v9.2d, v19.2d, v25.d[1]
	ldp		q24, q25, [x11], #32
	prfm	PLDL1KEEP, [x9, x16]

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d
	fadd	v1.2d, v1.2d, v9.2d

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, #256]
//	prfm	PLDL1KEEP, [x9, #320]
//	prfm	PLDL1KEEP, [x11, #256]

	// unroll 1
//	prfm	PLDL1KEEP, [x11, #320]
	ldp		q16, q17, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v24.d[1]
//	add		x11, x11, x12
	sub		w8, w8, #4

	// unroll 2
	ldp		q18, q19, [x9]
	fmla	v0.2d, v16.2d, v25.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v25.d[0]
//	cmp		w8, #4

	// unroll 3
//	ldp		q16, q17, [x9]
	fmla	v0.2d, v18.2d, v25.d[1]
//	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v25.d[1]
//	ldp		q24, q25, [x11, #(0*8+0*32)]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	ldp		q30, q31, [x11, #(0*8+3*32)]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldp		q24, q25, [x9]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nn_4x1_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_3X1_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nn_3x1_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11], #32
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	add		x9, x9, x10
	ldr		q20, [x9]
	ldr		d21, [x9, #16]
	add		x9, x9, x10
	ldr		q22, [x9]
	ldr		d23, [x9, #16]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	fmla	v1.2d, v21.2d, v25.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]
	fmla	v1.2d, v23.2d, v25.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11], #32
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	add		x9, x9, x10
	ldr		q20, [x9]
	ldr		d21, [x9, #16]
	add		x9, x9, x10
	ldr		q22, [x9]
	ldr		d23, [x9, #16]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]
	fmla	v1.2d, v17.2d, v24.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	fmla	v1.2d, v19.2d, v24.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	fmla	v1.2d, v21.2d, v25.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]
	fmla	v1.2d, v23.2d, v25.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9]
	ldr		d25, [x9, #16]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

//	lsl		x16, x10, #2 // 4*lda*...
	lsl		x16, x10, #1 // 2*lda*...

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x16]

	// preload
	ldp		q24, q25, [x11], #32
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x11, #0]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x9, x16]

	// zero tmp acc
	fmov	d8, xzr
	fmov    d9, d8

//	add		x13, x12, #64
//	add		x13, x12, x12
//	add		x14, x13, #64

	// main loop
1:
	
	// unroll 0
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
	prfm	PLDL1KEEP, [x9, x16]
	prfm	PLDL1KEEP, [x11, #32]

	// unroll 1
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v8.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v9.2d, v19.2d, v24.d[1]
	prfm	PLDL1KEEP, [x9, x16]

	// unroll 2
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v25.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v25.d[0]
	prfm	PLDL1KEEP, [x9, x16]
	sub		w8, w8, #4
	cmp		w8, #4

	// unroll 3
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v8.2d, v18.2d, v25.d[1]
	add		x9, x9, x10
	fmla	v9.2d, v19.2d, v25.d[1]
	ldp		q24, q25, [x11], #32
	prfm	PLDL1KEEP, [x9, x16]

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d
	fadd	v1.2d, v1.2d, v9.2d

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v24.d[0]
//	prfm	PLDL1KEEP, [x9, #256]
//	prfm	PLDL1KEEP, [x9, #320]
//	prfm	PLDL1KEEP, [x11, #256]

	// unroll 1
//	prfm	PLDL1KEEP, [x11, #320]
	ldr		q16, [x9]
	ldr		d17, [x9, #16]
	fmla	v0.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v24.d[1]
//	add		x11, x11, x12
	sub		w8, w8, #4

	// unroll 2
	ldr		q18, [x9]
	ldr		d19, [x9, #16]
	fmla	v0.2d, v16.2d, v25.d[0]
	add		x9, x9, x10
	fmla	v1.2d, v17.2d, v25.d[0]
//	cmp		w8, #4

	// unroll 3
//	ldr		q16, [x9]
//	ldr		d17, [x9, #16]
	fmla	v0.2d, v18.2d, v25.d[1]
//	add		x9, x9, x10
	fmla	v1.2d, v19.2d, v25.d[1]
//	ldp		q24, q25, [x11, #(0*8+0*32)]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	ldp		q30, q31, [x11, #(0*8+3*32)]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9]
	ldr		d25, [x9, #16]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nn_3x1_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_2X1_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nn_2x1_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11], #32
	ldr		q16, [x9]
	add		x9, x9, x10
	ldr		q18, [x9]
	add		x9, x9, x10
	ldr		q20, [x9]
	add		x9, x9, x10
	ldr		q22, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11], #32
	ldr		q16, [x9]
	add		x9, x9, x10
	ldr		q18, [x9]
	add		x9, x9, x10
	ldr		q20, [x9]
	add		x9, x9, x10
	ldr		q22, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

//	lsl		x16, x10, #2 // 4*lda*...
	lsl		x16, x10, #1 // 2*lda*...

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x16]

	// preload
	ldp		q24, q25, [x11], #32
	ldr		q16, [x9]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x11, #0]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x9, x16]

	// zero tmp acc
	fmov	d8, xzr

//	add		x13, x12, #64
//	add		x13, x12, x12
//	add		x14, x13, #64

	// main loop
1:
	
	// unroll 0
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x16]
	prfm	PLDL1KEEP, [x11, #32]

	// unroll 1
	ldr		q16, [x9]
	fmla	v8.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x16]

	// unroll 2
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v25.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x16]
	sub		w8, w8, #4
	cmp		w8, #4

	// unroll 3
	ldr		q16, [x9]
	fmla	v8.2d, v18.2d, v25.d[1]
	add		x9, x9, x10
	ldp		q24, q25, [x11], #32
	prfm	PLDL1KEEP, [x9, x16]

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
//	prfm	PLDL1KEEP, [x9, #256]
//	prfm	PLDL1KEEP, [x9, #320]
//	prfm	PLDL1KEEP, [x11, #256]

	// unroll 1
//	prfm	PLDL1KEEP, [x11, #320]
	ldr		q16, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
//	add		x11, x11, x12
	sub		w8, w8, #4

	// unroll 2
	ldr		q18, [x9]
	fmla	v0.2d, v16.2d, v25.d[0]
	add		x9, x9, x10
//	cmp		w8, #4

	// unroll 3
//	ldr		q16, [x9]
	fmla	v0.2d, v18.2d, v25.d[1]
//	add		x9, x9, x10
//	ldp		q24, q25, [x11, #(0*8+0*32)]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	ldp		q30, q31, [x11, #(0*8+3*32)]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nn_2x1_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_1X1_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nn_1x1_libcc)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	// prefetch

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch

	// zero tmp acc

	// main loop
1:
	
	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11], #32
	ldr		d16, [x9]
	add		x9, x9, x10
	ldr		d18, [x9]
	add		x9, x9, x10
	ldr		d20, [x9]
	add		x9, x9, x10
	ldr		d22, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]

	bgt		1b


	// reduce

0:

	cmp		w8, #3
	ble		4f

	// load 0 & 1 & 2 & 3
	ldp		q24, q25, [x11], #32
	ldr		d16, [x9]
	add		x9, x9, x10
	ldr		d18, [x9]
	add		x9, x9, x10
	ldr		d20, [x9]
	add		x9, x9, x10
	ldr		d22, [x9]
	add		x9, x9, x10

	// unroll 0
	fmla	v0.2d, v16.2d, v24.d[0]

	// unroll 1
	fmla	v0.2d, v18.2d, v24.d[1]
	sub		w8, w8, #4

	// unroll 2
	fmla	v0.2d, v20.2d, v25.d[0]
//	cmp		w8, #4

	// unroll 3
	fmla	v0.2d, v22.2d, v25.d[1]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		d24, [x9]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	fmla	v1.2d, v25.2d, v28.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else //if defined(TARGET_ARMV8A_ARM_CORTEX_A57)



	// early return
	cmp		w8, #0
	ble		2f // return

//	lsl		x16, x10, #2 // 4*lda*...
	lsl		x16, x10, #1 // 2*lda*...

	// prefetch
	prfm	PLDL1KEEP, [x11, #0]
//	prfm	PLDL1KEEP, [x9, #0]
//	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x9]
	prfm	PLDL1KEEP, [x9, x10]
	prfm	PLDL1KEEP, [x9, x16]

	// preload
	ldp		q24, q25, [x11], #32
	ldr		d16, [x9]
	add		x9, x9, x10

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	prfm	PLDL1KEEP, [x11, #0]
//	prfm	PLDL1KEEP, [x9, #128]
//	prfm	PLDL1KEEP, [x9, #192]
	prfm	PLDL1KEEP, [x9, x16]

	// zero tmp acc
	fmov	d8, xzr

//	add		x13, x12, #64
//	add		x13, x12, x12
//	add		x14, x13, #64

	// main loop
1:
	
	// unroll 0
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x16]
	prfm	PLDL1KEEP, [x11, #32]

	// unroll 1
	ldr		d16, [x9]
	fmla	v8.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x16]

	// unroll 2
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v25.d[0]
	add		x9, x9, x10
	prfm	PLDL1KEEP, [x9, x16]
	sub		w8, w8, #4
	cmp		w8, #4

	// unroll 3
	ldr		d16, [x9]
	fmla	v8.2d, v18.2d, v25.d[1]
	add		x9, x9, x10
	ldp		q24, q25, [x11], #32
	prfm	PLDL1KEEP, [x9, x16]

	bgt		1b


	// reduce
	fadd	v0.2d, v0.2d, v8.2d

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v24.d[0]
	add		x9, x9, x10
//	prfm	PLDL1KEEP, [x9, #256]
//	prfm	PLDL1KEEP, [x9, #320]
//	prfm	PLDL1KEEP, [x11, #256]

	// unroll 1
//	prfm	PLDL1KEEP, [x11, #320]
	ldr		d16, [x9]
	fmla	v0.2d, v18.2d, v24.d[1]
	add		x9, x9, x10
//	add		x11, x11, x12
	sub		w8, w8, #4

	// unroll 2
	ldr		d18, [x9]
	fmla	v0.2d, v16.2d, v25.d[0]
	add		x9, x9, x10
//	cmp		w8, #4

	// unroll 3
//	ldr		d16, [x9]
	fmla	v0.2d, v18.2d, v25.d[1]
//	add		x9, x9, x10
//	ldp		q24, q25, [x11, #(0*8+0*32)]
//	ldp		q26, q27, [x11, #(0*8+1*32)]
//	ldp		q28, q29, [x11, #(0*8+2*32)]
//	ldp		q30, q31, [x11, #(0*8+3*32)]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x11, x11, #32
	sub		x9, x9, x10

3: // clean1-up loop

	// unroll 0
	ldr		d24, [x9]
	add		x9, x9, x10
	ldr		d28, [x11], #8
	fmla	v0.2d, v24.2d, v28.d[0]
	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif
	


#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_nn_1x1_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
// x13  <- m1
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_4X4_VS_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nn_4x4_vs_libcc)

	// save old return address
	sub sp, sp, #16
	str x30, [sp, #0]
#endif

	cmp		w13, #1
	bgt		90f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_1X4_LIBCC
#else
	CALL(inner_kernel_gemm_nn_1x4_libcc)
#endif

	b		93f

90:

	cmp		w13, #2
	bgt		91f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_2X4_LIBCC
#else
	CALL(inner_kernel_gemm_nn_2x4_libcc)
#endif
	
	b		93f

91:

	cmp		w13, #3
	bgt		92f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_3X4_LIBCC
#else
	CALL(inner_kernel_gemm_nn_3x4_libcc)
#endif
	
	b		93f

92:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X4_LIBCC
#else
	CALL(inner_kernel_gemm_nn_4x4_libcc)
#endif

93:
// XXX x13 possibly used as working register on exit !!!

#if MACRO_LEVEL>=2
	.endm
#else
	// load old return address
	ldr x30, [sp, #0]
	add sp, sp, #16

	ret

	FUN_END(inner_kernel_gemm_nn_4x4_vs_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
// x13  <- m1
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_4X3_VS_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nn_4x3_vs_libcc)

	// save old return address
	sub sp, sp, #16
	str x30, [sp, #0]
#endif

	cmp		w13, #1
	bgt		90f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_1X3_LIBCC
#else
	CALL(inner_kernel_gemm_nn_1x3_libcc)
#endif

	b		93f

90:

	cmp		w13, #2
	bgt		91f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_2X3_LIBCC
#else
	CALL(inner_kernel_gemm_nn_2x3_libcc)
#endif
	
	b		93f

91:

	cmp		w13, #3
	bgt		92f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_3X3_LIBCC
#else
	CALL(inner_kernel_gemm_nn_3x3_libcc)
#endif
	
	b		93f

92:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X3_LIBCC
#else
	CALL(inner_kernel_gemm_nn_4x3_libcc)
#endif

93:
// XXX x13 possibly used as working register on exit !!!

#if MACRO_LEVEL>=2
	.endm
#else
	// load old return address
	ldr x30, [sp, #0]
	add sp, sp, #16

	ret

	FUN_END(inner_kernel_gemm_nn_4x3_vs_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
// x13  <- m1
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_4X2_VS_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nn_4x2_vs_libcc)

	// save old return address
	sub sp, sp, #16
	str x30, [sp, #0]
#endif

	cmp		w13, #1
	bgt		90f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_1X2_LIBCC
#else
	CALL(inner_kernel_gemm_nn_1x2_libcc)
#endif

	b		93f

90:

	cmp		w13, #2
	bgt		91f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_2X2_LIBCC
#else
	CALL(inner_kernel_gemm_nn_2x2_libcc)
#endif
	
	b		93f

91:

	cmp		w13, #3
	bgt		92f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_3X2_LIBCC
#else
	CALL(inner_kernel_gemm_nn_3x2_libcc)
#endif
	
	b		93f

92:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X2_LIBCC
#else
	CALL(inner_kernel_gemm_nn_4x2_libcc)
#endif

93:
// XXX x13 possibly used as working register on exit !!!

#if MACRO_LEVEL>=2
	.endm
#else
	// load old return address
	ldr x30, [sp, #0]
	add sp, sp, #16

	ret

	FUN_END(inner_kernel_gemm_nn_4x2_vs_libcc)
#endif





// subroutine
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- lda
// x11  <- B
// x12  <- ldb
// x13  <- m1
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_NN_4X1_VS_LIBCC
#else
	.align	4
	FUN_START(inner_kernel_gemm_nn_4x1_vs_libcc)

	// save old return address
	sub sp, sp, #16
	str x30, [sp, #0]
#endif

	cmp		w13, #1
	bgt		90f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_1X1_LIBCC
#else
	CALL(inner_kernel_gemm_nn_1x1_libcc)
#endif

	b		93f

90:

	cmp		w13, #2
	bgt		91f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_2X1_LIBCC
#else
	CALL(inner_kernel_gemm_nn_2x1_libcc)
#endif
	
	b		93f

91:

	cmp		w13, #3
	bgt		92f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_3X1_LIBCC
#else
	CALL(inner_kernel_gemm_nn_3x1_libcc)
#endif
	
	b		93f

92:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X1_LIBCC
#else
	CALL(inner_kernel_gemm_nn_4x1_libcc)
#endif

93:
// XXX x13 possibly used as working register on exit !!!

#if MACRO_LEVEL>=2
	.endm
#else
	// load old return address
	ldr x30, [sp, #0]
	add sp, sp, #16

	ret

	FUN_END(inner_kernel_gemm_nn_4x1_vs_libcc)
#endif





// subroutine
//
// triangular multiplication:
// side = right
// uplo = lower
// tran = not-transposed
// not-unit diagonal
//
// input arguments:
// w8   <- k
// x9   <- A
// x10   <- B
// x11   <- ldb
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRMM_NN_RL_4X4_LIB4C
#else
	.align 4
	FUN_START(inner_edge_trmm_nn_rl_4x4_lib4c)
#endif

	add		x12, x10, x11
	add		x13, x12, x11
	add		x14, x13, x11

	ldp		q24, q25, [x9, #(0*8+0*32)] // A
	ldr		d26, [x10, #(0*8)] // B
	fmla	v0.2d, v24.2d, v26.d[0]
	fmla	v1.2d, v25.2d, v26.d[0]

	ldp		q24, q25, [x9, #(0*8+1*32)] // A
	ldr		d26, [x10, #(1*8)] // B
	fmla	v0.2d, v24.2d, v26.d[0]
	fmla	v1.2d, v25.2d, v26.d[0]
	ldr		d26, [x12, #(1*8)] // B
	fmla	v2.2d, v24.2d, v26.d[0]
	fmla	v3.2d, v25.2d, v26.d[0]

	ldp		q24, q25, [x9, #(0*8+2*32)] // A
	ldr		d26, [x10, #(2*8)] // B
	fmla	v0.2d, v24.2d, v26.d[0]
	fmla	v1.2d, v25.2d, v26.d[0]
	ldr		d26, [x12, #(2*8)] // B
	fmla	v2.2d, v24.2d, v26.d[0]
	fmla	v3.2d, v25.2d, v26.d[0]
	ldr		d26, [x13, #(2*8)] // B
	fmla	v4.2d, v24.2d, v26.d[0]
	fmla	v5.2d, v25.2d, v26.d[0]

	ldp		q24, q25, [x9, #(0*8+3*32)] // A
	ldr		d26, [x10, #(3*8)] // B
	fmla	v0.2d, v24.2d, v26.d[0]
	fmla	v1.2d, v25.2d, v26.d[0]
	ldr		d26, [x12, #(3*8)] // B
	fmla	v2.2d, v24.2d, v26.d[0]
	fmla	v3.2d, v25.2d, v26.d[0]
	ldr		d26, [x13, #(3*8)] // B
	fmla	v4.2d, v24.2d, v26.d[0]
	fmla	v5.2d, v25.2d, v26.d[0]
	ldr		d26, [x14, #(3*8)] // B
	fmla	v6.2d, v24.2d, v26.d[0]
	fmla	v7.2d, v25.2d, v26.d[0]

	sub		w8, w8, #4
	add		x9, x9, #128
	add		x10, x10, #32

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trmm_nn_rl_4x4_lib4c)
#endif





// subroutine
//
// triangular multiplication:
// side = right
// uplo = lower
// tran = not-transposed
// not-unit diagonal
//
// input arguments:
// w8   <- k
// x9   <- A
// x10   <- B
// x11   <- ldb
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRMM_NN_RL_4X4_VS_LIB4C
#else
	.align 4
	FUN_START(inner_edge_trmm_nn_rl_4x4_vs_lib4c)
#endif

	add		x12, x10, x11
	add		x13, x12, x11
	add		x14, x13, x11

	ldp		q24, q25, [x9, #(0*8+0*32)] // A
	ldr		d26, [x10, #(0*8)] // B
	fmla	v0.2d, v24.2d, v26.d[0]
	fmla	v1.2d, v25.2d, v26.d[0]
	sub		w8, w8, #1
	add		x9, x9, #32
	add		x10, x10, #8
//	add		x12, x12, #8
//	add		x13, x13, #8
//	add		x14, x14, #8

	cmp		w8, #0
	ble		0f

	ldp		q24, q25, [x9, #(0*8+0*32)] // A
	ldr		d26, [x10, #(0*8)] // B
	fmla	v0.2d, v24.2d, v26.d[0]
	fmla	v1.2d, v25.2d, v26.d[0]
	ldr		d26, [x12, #(1*8)] // B XXX
	fmla	v2.2d, v24.2d, v26.d[0]
	fmla	v3.2d, v25.2d, v26.d[0]
	sub		w8, w8, #1
	add		x9, x9, #32
	add		x10, x10, #8
//	add		x12, x12, #8
//	add		x13, x13, #8
//	add		x14, x14, #8

	cmp		w8, #0
	ble		0f

	ldp		q24, q25, [x9, #(0*8+0*32)] // A
	ldr		d26, [x10, #(0*8)] // B
	fmla	v0.2d, v24.2d, v26.d[0]
	fmla	v1.2d, v25.2d, v26.d[0]
	ldr		d26, [x12, #(2*8)] // B XXX
	fmla	v2.2d, v24.2d, v26.d[0]
	fmla	v3.2d, v25.2d, v26.d[0]
	ldr		d26, [x13, #(2*8)] // B XXX
	fmla	v4.2d, v24.2d, v26.d[0]
	fmla	v5.2d, v25.2d, v26.d[0]
	sub		w8, w8, #1
	add		x9, x9, #32
	add		x10, x10, #8
//	add		x12, x12, #8
//	add		x13, x13, #8
//	add		x14, x14, #8

	cmp		w8, #0
	ble		0f

	ldp		q24, q25, [x9, #(0*8+0*32)] // A
	ldr		d26, [x10, #(0*8)] // B
	fmla	v0.2d, v24.2d, v26.d[0]
	fmla	v1.2d, v25.2d, v26.d[0]
	ldr		d26, [x12, #(3*8)] // B XXX
	fmla	v2.2d, v24.2d, v26.d[0]
	fmla	v3.2d, v25.2d, v26.d[0]
	ldr		d26, [x13, #(3*8)] // B XXX
	fmla	v4.2d, v24.2d, v26.d[0]
	fmla	v5.2d, v25.2d, v26.d[0]
	ldr		d26, [x14, #(3*8)] // B XXX
	fmla	v6.2d, v24.2d, v26.d[0]
	fmla	v7.2d, v25.2d, v26.d[0]
	sub		w8, w8, #1
	add		x9, x9, #32
	add		x10, x10, #8
//	add		x12, x12, #8
//	add		x13, x13, #8
//	add		x14, x14, #8

	cmp		w8, #0
	ble		0f

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trmm_nn_rl_4x4_vs_lib4c)
#endif





// subroutine
//
// triangular multiplication:
// side = right
// uplo = lower
// tran = transposed
// not-unit diagonal
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- B
// w11  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRMM_NT_RL_4X4_LIB4C
#else
	.align 4
	FUN_START(inner_edge_trmm_nt_rl_4x4_lib4c)
#endif

	ldp		q24, q25, [x9, #(0*8+0*32)] // A
	ldp		q26, q27, [x10, #(0*8)] // B
	fmla	v0.2d, v24.2d, v26.d[0]
	fmla	v1.2d, v25.2d, v26.d[0]
	fmla	v2.2d, v24.2d, v26.d[1]
	fmla	v3.2d, v25.2d, v26.d[1]
	fmla	v4.2d, v24.2d, v27.d[0]
	fmla	v5.2d, v25.2d, v27.d[0]
	fmla	v6.2d, v24.2d, v27.d[1]
	fmla	v7.2d, v25.2d, v27.d[1]
	add		x10, x10, x11

	ldp		q24, q25, [x9, #(0*8+1*32)] // A
	ldr		d26, [x10, #(1*8)] // B
	ldr		q27, [x10, #(2*8)] // B
	fmla	v2.2d, v24.2d, v26.d[0]
	fmla	v3.2d, v25.2d, v26.d[0]
	fmla	v4.2d, v24.2d, v27.d[0]
	fmla	v5.2d, v25.2d, v27.d[0]
	fmla	v6.2d, v24.2d, v27.d[1]
	fmla	v7.2d, v25.2d, v27.d[1]
	add		x10, x10, x11

	ldp		q24, q25, [x9, #(0*8+2*32)] // A
	ldr		q27, [x10, #(2*8)] // B
	fmla	v4.2d, v24.2d, v27.d[0]
	fmla	v5.2d, v25.2d, v27.d[0]
	fmla	v6.2d, v24.2d, v27.d[1]
	fmla	v7.2d, v25.2d, v27.d[1]
	add		x10, x10, x11

	ldp		q24, q25, [x9, #(0*8+3*32)] // A
	ldr		d27, [x10, #(3*8)] // B
	fmla	v6.2d, v24.2d, v27.d[0]
	fmla	v7.2d, v25.2d, v27.d[0]
	add		x10, x10, x11

	sub		w8, w8, #4
	add		x9, x9, #128

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trmm_nt_rl_4x4_lib4c)
#endif





// subroutine
//
// triangular multiplication:
// side = right
// uplo = lower
// tran = transposed
// not-unit diagonal
//
// input arguments:
// w8   <- k
// x9   <- A
// x10  <- B
// w11  <- ldb
// w12  <- n1
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRMM_NT_RL_4X4_VS_LIB4C
#else
	.align 4
	FUN_START(inner_edge_trmm_nt_rl_4x4_vs_lib4c)
#endif

	cmp		w12, #0
	ble		0f

	cmp		w12, #4
	blt		1f

	ldp		q24, q25, [x9, #(0*8+0*32)] // A
	ldp		q26, q27, [x10, #(0*8)] // B
	fmla	v0.2d, v24.2d, v26.d[0]
	fmla	v1.2d, v25.2d, v26.d[0]
	fmla	v2.2d, v24.2d, v26.d[1]
	fmla	v3.2d, v25.2d, v26.d[1]
	fmla	v4.2d, v24.2d, v27.d[0]
	fmla	v5.2d, v25.2d, v27.d[0]
	fmla	v6.2d, v24.2d, v27.d[1]
	fmla	v7.2d, v25.2d, v27.d[1]
	add		x10, x10, x11

	ldp		q24, q25, [x9, #(0*8+1*32)] // A
	ldr		d26, [x10, #(1*8)] // B
	ldr		q27, [x10, #(2*8)] // B
	fmla	v2.2d, v24.2d, v26.d[0]
	fmla	v3.2d, v25.2d, v26.d[0]
	fmla	v4.2d, v24.2d, v27.d[0]
	fmla	v5.2d, v25.2d, v27.d[0]
	fmla	v6.2d, v24.2d, v27.d[1]
	fmla	v7.2d, v25.2d, v27.d[1]
	add		x10, x10, x11

	ldp		q24, q25, [x9, #(0*8+2*32)] // A
	ldr		q27, [x10, #(2*8)] // B
	fmla	v4.2d, v24.2d, v27.d[0]
	fmla	v5.2d, v25.2d, v27.d[0]
	fmla	v6.2d, v24.2d, v27.d[1]
	fmla	v7.2d, v25.2d, v27.d[1]
	add		x10, x10, x11

	ldp		q24, q25, [x9, #(0*8+3*32)] // A
	ldr		d27, [x10, #(3*8)] // B
	fmla	v6.2d, v24.2d, v27.d[0]
	fmla	v7.2d, v25.2d, v27.d[0]
	add		x10, x10, x11

	sub		w8, w8, #4
	add		x9, x9, #128

	b		0f

1:

	cmp		w12, #3
	blt		1f

	ldp		q24, q25, [x9, #(0*8+0*32)] // A
	ldr		q26, [x10, #(0*8)] // B
	ldr		d27, [x10, #(2*8)] // B
	fmla	v0.2d, v24.2d, v26.d[0]
	fmla	v1.2d, v25.2d, v26.d[0]
	fmla	v2.2d, v24.2d, v26.d[1]
	fmla	v3.2d, v25.2d, v26.d[1]
	fmla	v4.2d, v24.2d, v27.d[0]
	fmla	v5.2d, v25.2d, v27.d[0]
	add		x10, x10, x11

	ldp		q24, q25, [x9, #(0*8+1*32)] // A
	ldr		d26, [x10, #(1*8)] // B
	ldr		d27, [x10, #(2*8)] // B
	fmla	v2.2d, v24.2d, v26.d[0]
	fmla	v3.2d, v25.2d, v26.d[0]
	fmla	v4.2d, v24.2d, v27.d[0]
	fmla	v5.2d, v25.2d, v27.d[0]
	add		x10, x10, x11

	ldp		q24, q25, [x9, #(0*8+2*32)] // A
	ldr		d27, [x10, #(2*8)] // B
	fmla	v4.2d, v24.2d, v27.d[0]
	fmla	v5.2d, v25.2d, v27.d[0]
	add		x10, x10, x11

	sub		w8, w8, #3
	add		x9, x9, #96

	b		0f

1:

	cmp		w12, #2
	blt		1f

	ldp		q24, q25, [x9, #(0*8+0*32)] // A
	ldr		q26, [x10, #(0*8)] // B
	fmla	v0.2d, v24.2d, v26.d[0]
	fmla	v1.2d, v25.2d, v26.d[0]
	fmla	v2.2d, v24.2d, v26.d[1]
	fmla	v3.2d, v25.2d, v26.d[1]
	add		x10, x10, x11

	ldp		q24, q25, [x9, #(0*8+1*32)] // A
	ldr		d26, [x10, #(1*8)] // B
	fmla	v2.2d, v24.2d, v26.d[0]
	fmla	v3.2d, v25.2d, v26.d[0]
	add		x10, x10, x11

	sub		w8, w8, #2
	add		x9, x9, #64

	b		0f

1:

//	cmp		w12, #1
//	blt		0f

	ldp		q24, q25, [x9, #(0*8+0*32)] // A
	ldr		d26, [x10, #(0*8)] // B
	fmla	v0.2d, v24.2d, v26.d[0]
	fmla	v1.2d, v25.2d, v26.d[0]
	add		x10, x10, x11

	sub		w8, w8, #2
	add		x9, x9, #32

	b		0f

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trmm_nt_rl_4x4_vs_lib4c)
#endif





// subroutine
//
// triangular substitution:
// side = left
// uplo = lower
// tran = not-transposed
// unit diagonal
//
// input arguments:
// x8   <- E
// x9   <- lde
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRSM_LLN_ONE_4X4_LIBC
#else
	.align 4
	FUN_START(inner_edge_trsm_lln_one_4x4_libc)
#endif

	ldp		q24, q25, [x8, #0] // E[0+4*0]
	add		x8, x8, x9
	ins		v24.d[0], xzr
	fmls	v0.2d, v24.2d, v0.d[0]
	fmls	v1.2d, v25.2d, v0.d[0]
	fmls	v2.2d, v24.2d, v2.d[0]
	fmls	v3.2d, v25.2d, v2.d[0]
	fmls	v4.2d, v24.2d, v4.d[0]
	fmls	v5.2d, v25.2d, v4.d[0]
	fmls	v6.2d, v24.2d, v6.d[0]
	fmls	v7.2d, v25.2d, v6.d[0]

	ldr		q25, [x8, #16] // E[2+4*1]
	add		x8, x8, x9
	fmls	v1.2d, v25.2d, v0.d[1]
	fmls	v3.2d, v25.2d, v2.d[1]
	fmls	v5.2d, v25.2d, v4.d[1]
	fmls	v7.2d, v25.2d, v6.d[1]

	ldr		q25, [x8, #16] // E[2+4*2]
//	add		x8, x8, x9
	ins		v25.d[0], xzr
	fmls	v1.2d, v25.2d, v1.d[0]
	fmls	v3.2d, v25.2d, v3.d[0]
	fmls	v5.2d, v25.2d, v5.d[0]
	fmls	v7.2d, v25.2d, v7.d[0]

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trsm_lln_one_4x4_libc)
#endif





// subroutine
//
// triangular substitution:
// side = left
// uplo = lower
// tran = not-transposed
// unit diagonal
//
// input arguments:
// x8   <- E
// x9   <- lde
// w10  <- m1
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRSM_LLN_ONE_4X4_VS_LIBC
#else
	.align 4
	FUN_START(inner_edge_trsm_lln_one_4x4_vs_libc)
#endif

	cmp		w10, #1
	bgt		1f

	// 1x1
	b		0f

1:

	// 2x2
	ldr		q24, [x8, #0] // E[0+4*0]
//	add		x8, x8, x9
	ins		v24.d[0], xzr
	fmls	v0.2d, v24.2d, v0.d[0]
	fmls	v2.2d, v24.2d, v2.d[0]
	fmls	v4.2d, v24.2d, v4.d[0]
	fmls	v6.2d, v24.2d, v6.d[0]

	cmp		w10, #3
	blt		0f

	bgt		1f

	// 3x3
	ldr		d25, [x8, #16] // E[2+4*0]
	fmls	v1.2d, v25.2d, v0.d[0]
	fmls	v3.2d, v25.2d, v2.d[0]
	fmls	v5.2d, v25.2d, v4.d[0]
	fmls	v7.2d, v25.2d, v6.d[0]
	add		x8, x8, x9
	ldr		d25, [x8, #16] // E[2+4*1]
	fmls	v1.2d, v25.2d, v0.d[1]
	fmls	v3.2d, v25.2d, v2.d[1]
	fmls	v5.2d, v25.2d, v4.d[1]
	fmls	v7.2d, v25.2d, v6.d[1]
//	sub		x8, x8, x9
	b		0f

1:

	// 4x4
	ldr		q25, [x8, #16] // E[2+4*0]
	fmls	v1.2d, v25.2d, v0.d[0]
	fmls	v3.2d, v25.2d, v2.d[0]
	fmls	v5.2d, v25.2d, v4.d[0]
	fmls	v7.2d, v25.2d, v6.d[0]
	add		x8, x8, x9
	ldr		q25, [x8, #16] // E[2+4*1]
	fmls	v1.2d, v25.2d, v0.d[1]
	fmls	v3.2d, v25.2d, v2.d[1]
	fmls	v5.2d, v25.2d, v4.d[1]
	fmls	v7.2d, v25.2d, v6.d[1]
	add		x8, x8, x9
	ldr		q25, [x8, #16] // E[2+4*2]
	ins		v25.d[0], xzr
	fmls	v1.2d, v25.2d, v1.d[0]
	fmls	v3.2d, v25.2d, v3.d[0]
	fmls	v5.2d, v25.2d, v5.d[0]
	fmls	v7.2d, v25.2d, v7.d[0]
//	sub		x8, x8, x9
//	sub		x8, x8, x9

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trsm_lln_one_4x4_vs_libc)
#endif





// subroutine
//
// triangular substitution:
// side = right
// uplo = lower
// tran = not-transposed
// requires explicit inverse of diagonal
//
// input arguments:
// x8   <- E
// x9   <- lde
// x10  <- inv_diag_E
//
// output arguments:
// x8   <- E
// x9   <- lde
// x10  <- inv_diag_E

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRSM_RLN_INV_4X4_LIB
#else
	.align 4
	FUN_START(inner_edge_trsm_rln_inv_4x4_lib)
#endif
	
	add			x11, x8, x9
	add			x12, x11, x9

	ldr			d16, [x10, #24] // E_inv[3]
	fmul		v6.2d, v6.2d, v16.d[0]
	fmul		v7.2d, v7.2d, v16.d[0]
	ldr			d16, [x12, #24] // E[3+4*2]
	fmls		v4.2d, v6.2d, v16.d[0]
	fmls		v5.2d, v7.2d, v16.d[0]
	ldr			d16, [x11, #24] // E[3+4*1]
	fmls		v2.2d, v6.2d, v16.d[0]
	fmls		v3.2d, v7.2d, v16.d[0]
	ldr			d16, [x8, #24] // E[3+4*0]
	fmls		v0.2d, v6.2d, v16.d[0]
	fmls		v1.2d, v7.2d, v16.d[0]

	ldr			d16, [x10, #16] // E_inv[2]
	fmul		v4.2d, v4.2d, v16.d[0]
	fmul		v5.2d, v5.2d, v16.d[0]
	ldr			d16, [x11, #16] // E[2+4*1]
	fmls		v2.2d, v4.2d, v16.d[0]
	fmls		v3.2d, v5.2d, v16.d[0]
	ldr			d16, [x8, #16] // E[2+4*0]
	fmls		v0.2d, v4.2d, v16.d[0]
	fmls		v1.2d, v5.2d, v16.d[0]

	ldr			d16, [x10, #8] // E_inv[1]
	fmul		v2.2d, v2.2d, v16.d[0]
	fmul		v3.2d, v3.2d, v16.d[0]
	ldr			d16, [x8, #8] // E[1+4*0]
	fmls		v0.2d, v2.2d, v16.d[0]
	fmls		v1.2d, v3.2d, v16.d[0]

	ldr			d16, [x10, #0] // E_inv[0]
	fmul		v0.2d, v0.2d, v16.d[0]
	fmul		v1.2d, v1.2d, v16.d[0]

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trsm_rln_inv_4x4_lib)
#endif





// subroutine
//
// triangular substitution:
// side = right
// uplo = upper
// tran = transposed
// requires explicit inverse of diagonal
//
// input arguments:
// x8   <- E
// x9   <- lde
// x10  <- inv_diag_E
// w11  <- n1
//
// output arguments:
// x8   <- E
// x9   <- lde
// x10   <- inv_diag_E
// w11  <- n1

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRSM_RLN_INV_4X4_VS_LIB
#else
	.align 4
	FUN_START(inner_edge_trsm_rln_inv_4x4_vs_lib)
#endif
	
	add			x12, x8, x9
	add			x13, x12, x9

	cmp		w11, #3
	ble		1f

	ldr			d16, [x10, #24] // E_inv[3]
	fmul		v6.2d, v6.2d, v16.d[0]
	fmul		v7.2d, v7.2d, v16.d[0]
	ldr			d16, [x13, #24] // E[3+4*2]
	fmls		v4.2d, v6.2d, v16.d[0]
	fmls		v5.2d, v7.2d, v16.d[0]
	ldr			d16, [x12, #24] // E[3+4*1]
	fmls		v2.2d, v6.2d, v16.d[0]
	fmls		v3.2d, v7.2d, v16.d[0]
	ldr			d16, [x8, #24] // E[3+4*0]
	fmls		v0.2d, v6.2d, v16.d[0]
	fmls		v1.2d, v7.2d, v16.d[0]

1:
	cmp		w11, #2
	ble		1f

	ldr			d16, [x10, #16] // E_inv[2]
	fmul		v4.2d, v4.2d, v16.d[0]
	fmul		v5.2d, v5.2d, v16.d[0]
	ldr			d16, [x12, #16] // E[2+4*1]
	fmls		v2.2d, v4.2d, v16.d[0]
	fmls		v3.2d, v5.2d, v16.d[0]
	ldr			d16, [x8, #16] // E[2+4*0]
	fmls		v0.2d, v4.2d, v16.d[0]
	fmls		v1.2d, v5.2d, v16.d[0]

1:
	cmp		w11, #1
	ble		1f

	ldr			d16, [x10, #8] // E_inv[1]
	fmul		v2.2d, v2.2d, v16.d[0]
	fmul		v3.2d, v3.2d, v16.d[0]
	ldr			d16, [x8, #8] // E[1+4*0]
	fmls		v0.2d, v2.2d, v16.d[0]
	fmls		v1.2d, v3.2d, v16.d[0]

1:

	ldr			d16, [x10, #0] // E_inv[0]
	fmul		v0.2d, v0.2d, v16.d[0]
	fmul		v1.2d, v1.2d, v16.d[0]

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trsm_rln_inv_4x4_vs_lib)
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
	
	ldr			d16, [x10, #0] // E_inv[0]
	fmul		v0.2d, v0.2d, v16.d[0]
	fmul		v1.2d, v1.2d, v16.d[0]
	ldr			d16, [x8, #8] // E[1+4*0]
	fmls		v2.2d, v0.2d, v16.d[0]
	fmls		v3.2d, v1.2d, v16.d[0]
	ldr			d16, [x8, #16] // E[2+4*0]
	fmls		v4.2d, v0.2d, v16.d[0]
	fmls		v5.2d, v1.2d, v16.d[0]
	ldr			d16, [x8, #24] // E[3+4*0]
	fmls		v6.2d, v0.2d, v16.d[0]
	fmls		v7.2d, v1.2d, v16.d[0]
	add			x8, x8, x9

	ldr			d16, [x10, #8] // E_inv[1]
	fmul		v2.2d, v2.2d, v16.d[0]
	fmul		v3.2d, v3.2d, v16.d[0]
	ldr			d16, [x8, #16] // E[2+4*1]
	fmls		v4.2d, v2.2d, v16.d[0]
	fmls		v5.2d, v3.2d, v16.d[0]
	ldr			d16, [x8, #24] // E[3+4*1]
	fmls		v6.2d, v2.2d, v16.d[0]
	fmls		v7.2d, v3.2d, v16.d[0]
	add			x8, x8, x9

	ldr			d16, [x10, #16] // E_inv[2]
	fmul		v4.2d, v4.2d, v16.d[0]
	fmul		v5.2d, v5.2d, v16.d[0]
	ldr			d16, [x8, #24] // E[3+4*1]
	fmls		v6.2d, v4.2d, v16.d[0]
	fmls		v7.2d, v5.2d, v16.d[0]
//	add			x8, x8, x9

	ldr			d16, [x10, #24] // E_inv[2]
	fmul		v6.2d, v6.2d, v16.d[0]
	fmul		v7.2d, v7.2d, v16.d[0]
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
	ldr			d16, [x10, #0] // E_inv[0]
	fmul		v0.2d, v0.2d, v16.d[0]
	fmul		v1.2d, v1.2d, v16.d[0]
	cmp			w11, #2
	blt			0f // return

	// second column
	ldr			d16, [x8, #8] // E[1+4*0]
	fmls		v2.2d, v0.2d, v16.d[0]
	fmls		v3.2d, v1.2d, v16.d[0]
	ldr			d16, [x10, #8] // E_inv[1]
	fmul		v2.2d, v2.2d, v16.d[0]
	fmul		v3.2d, v3.2d, v16.d[0]
	cmp			w11, #3
	blt			0f // return

	// third column
	add			x12, x8, x9
	ldr			d16, [x8, #16] // E[2+4*0]
	fmls		v4.2d, v0.2d, v16.d[0]
	fmls		v5.2d, v1.2d, v16.d[0]
	ldr			d16, [x12, #16] // E[2+4*1]
	fmls		v4.2d, v2.2d, v16.d[0]
	fmls		v5.2d, v3.2d, v16.d[0]
	ldr			d16, [x10, #16] // E_inv[2]
	fmul		v4.2d, v4.2d, v16.d[0]
	fmul		v5.2d, v5.2d, v16.d[0]
	cmp			w11, #4
	blt			0f // return

	// forth column
	add			x13, x12, x9
	ldr			d16, [x8, #24] // E[3+4*0]
	fmls		v6.2d, v0.2d, v16.d[0]
	fmls		v7.2d, v1.2d, v16.d[0]
	ldr			d16, [x12, #24] // E[3+4*1]
	fmls		v6.2d, v2.2d, v16.d[0]
	fmls		v7.2d, v3.2d, v16.d[0]
	ldr			d16, [x13, #24] // E[3+4*2]
	fmls		v6.2d, v4.2d, v16.d[0]
	fmls		v7.2d, v5.2d, v16.d[0]
	ldr			d16, [x10, #24] // E_inv[3]
	fmul		v6.2d, v6.2d, v16.d[0]
	fmul		v7.2d, v7.2d, v16.d[0]

0:
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trsm_rlt_inv_4x4_vs_lib)
#endif





// subroutine
//
// triangular substitution:
// side = right
// uplo = upper
// tran = not-transposed
// requires explicit inverse of diagonal
//
// input arguments:
// x8   <- E
// x9   <- lde
// x10  <- inv_diag_E
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_EDGE_TRSM_RUN_INV_4X4_LIB
#else
	.align 4
	FUN_START(inner_edge_trsm_run_inv_4x4_lib)
#endif
	
	add			x11, x8, x9
	add			x12, x11, x9
	add			x13, x12, x9

	ldr			d16, [x10, #0] // E_inv[0]
	fmul		v0.2d, v0.2d, v16.d[0]
	fmul		v1.2d, v1.2d, v16.d[0]
	ldr			d16, [x11, #0] // E[0+4*1]
	fmls		v2.2d, v0.2d, v16.d[0]
	fmls		v3.2d, v1.2d, v16.d[0]
	ldr			d16, [x12, #0] // E[0+4*2]
	fmls		v4.2d, v0.2d, v16.d[0]
	fmls		v5.2d, v1.2d, v16.d[0]
	ldr			d16, [x13, #0] // E[0+4*3]
	fmls		v6.2d, v0.2d, v16.d[0]
	fmls		v7.2d, v1.2d, v16.d[0]

	ldr			d16, [x10, #8] // E_inv[1]
	fmul		v2.2d, v2.2d, v16.d[0]
	fmul		v3.2d, v3.2d, v16.d[0]
	ldr			d16, [x12, #8] // E[1+4*2]
	fmls		v4.2d, v2.2d, v16.d[0]
	fmls		v5.2d, v3.2d, v16.d[0]
	ldr			d16, [x13, #8] // E[1+4*3]
	fmls		v6.2d, v2.2d, v16.d[0]
	fmls		v7.2d, v3.2d, v16.d[0]

	ldr			d16, [x10, #16] // E_inv[2]
	fmul		v4.2d, v4.2d, v16.d[0]
	fmul		v5.2d, v5.2d, v16.d[0]
	ldr			d16, [x13, #16] // E[2+4*3]
	fmls		v6.2d, v4.2d, v16.d[0]
	fmls		v7.2d, v5.2d, v16.d[0]

	ldr			d16, [x10, #24] // E_inv[2]
	fmul		v6.2d, v6.2d, v16.d[0]
	fmul		v7.2d, v7.2d, v16.d[0]

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trsm_run_inv_4x4_lib)
#endif





// subroutine
//
// triangular substitution:
// side = right
// uplo = upper
// tran = not-transposed
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
	.macro INNER_EDGE_TRSM_RUN_INV_4X4_VS_LIB
#else
	.align 4
	FUN_START(inner_edge_trsm_run_inv_4x4_vs_lib)
#endif
	
	// first column
	ldr			d16, [x10, #0] // E_inv[0]
	fmul		v0.2d, v0.2d, v16.d[0]
	fmul		v1.2d, v1.2d, v16.d[0]
	cmp			w11, #2
	blt			0f // return

	// second column
	add			x8, x8, x9
	ldr			d16, [x8, #0] // E[0+4*1]
	fmls		v2.2d, v0.2d, v16.d[0]
	fmls		v3.2d, v1.2d, v16.d[0]
	ldr			d16, [x10, #8] // E_inv[1]
	fmul		v2.2d, v2.2d, v16.d[0]
	fmul		v3.2d, v3.2d, v16.d[0]
	cmp			w11, #3
	blt			0f // return

	// third column
	add			x8, x8, x9
	ldr			d16, [x8, #0] // E[0+4*2]
	fmls		v4.2d, v0.2d, v16.d[0]
	fmls		v5.2d, v1.2d, v16.d[0]
	ldr			d16, [x8, #8] // E[1+4*2]
	fmls		v4.2d, v2.2d, v16.d[0]
	fmls		v5.2d, v3.2d, v16.d[0]
	ldr			d16, [x10, #16] // E_inv[2]
	fmul		v4.2d, v4.2d, v16.d[0]
	fmul		v5.2d, v5.2d, v16.d[0]
	cmp			w11, #4
	blt			0f // return

	// forth column
	add			x8, x8, x9
	ldr			d16, [x8, #0] // E[0+4*3]
	fmls		v6.2d, v0.2d, v16.d[0]
	fmls		v7.2d, v1.2d, v16.d[0]
	ldr			d16, [x8, #8] // E[1+4*3]
	fmls		v6.2d, v2.2d, v16.d[0]
	fmls		v7.2d, v3.2d, v16.d[0]
	ldr			d16, [x8, #16] // E[2+4*3]
	fmls		v6.2d, v4.2d, v16.d[0]
	fmls		v7.2d, v5.2d, v16.d[0]
	ldr			d16, [x10, #24] // E_inv[3]
	fmul		v6.2d, v6.2d, v16.d[0]
	fmul		v7.2d, v7.2d, v16.d[0]

0:
#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_edge_trsm_run_inv_4x4_vs_lib)
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

	trn1	v24.2d, v0.2d, v2.2d
	trn2	v2.2d, v0.2d, v2.2d
	trn1	v25.2d, v5.2d, v7.2d
	trn2	v7.2d, v5.2d, v7.2d
	trn1	v26.2d, v1.2d, v3.2d
	trn2	v27.2d, v1.2d, v3.2d
	trn1	v1.2d, v4.2d, v6.2d
	trn2	v3.2d, v4.2d, v6.2d
	mov		v0.16b, v24.16b
	mov		v5.16b, v25.16b
	mov		v4.16b, v26.16b
	mov		v6.16b, v27.16b

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
// x11  <- ldc*sizeof(double)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_AB_4X4_LIB
#else
	.align	4
	FUN_START(inner_scale_ab_4x4_lib)
#endif

	ld1		{v28.2d}, [x8]

	ld1		{v29.2d}, [x9]

	fmul	v0.2d, v0.2d, v28.d[0]
	fmul	v1.2d, v1.2d, v28.d[0]
	fmul	v2.2d, v2.2d, v28.d[0]
	fmul	v3.2d, v3.2d, v28.d[0]
	fmul	v4.2d, v4.2d, v28.d[0]
	fmul	v5.2d, v5.2d, v28.d[0]
	fmul	v6.2d, v6.2d, v28.d[0]
	fmul	v7.2d, v7.2d, v28.d[0]

	fcmpe	d29, #0.0
	beq		0f

	ldp		q24, q25, [x10, #0]
	add		x10, x10, x11
	ldp		q26, q27, [x10, #0]
	add		x10, x10, x11
	fmla	v0.2d, v24.2d, v29.d[0]
	fmla	v1.2d, v25.2d, v29.d[0]
	fmla	v2.2d, v26.2d, v29.d[0]
	fmla	v3.2d, v27.2d, v29.d[0]

	ldp		q24, q25, [x10, #0]
	add		x10, x10, x11
	ldp		q26, q27, [x10, #0]
	add		x10, x10, x11
	fmla	v4.2d, v24.2d, v29.d[0]
	fmla	v5.2d, v25.2d, v29.d[0]
	fmla	v6.2d, v26.2d, v29.d[0]
	fmla	v7.2d, v27.2d, v29.d[0]

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
// x11  <- ldc*sizeof(double)
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

	ld1		{v28.2d}, [x8]

	ld1		{v29.2d}, [x9]

	fmul	v0.2d, v0.2d, v28.d[0]
	fmul	v1.2d, v1.2d, v28.d[0]
	fmul	v2.2d, v2.2d, v28.d[0]
	fmul	v3.2d, v3.2d, v28.d[0]
	fmul	v4.2d, v4.2d, v28.d[0]
	fmul	v5.2d, v5.2d, v28.d[0]
	fmul	v6.2d, v6.2d, v28.d[0]
	fmul	v7.2d, v7.2d, v28.d[0]

	fcmpe	d29, #0.0
	beq		0f

	cmp		w12, #4
	blt		1f

	ldp		q24, q25, [x10, #0]
	add		x10, x10, x11
	fmla	v0.2d, v24.2d, v29.d[0]
	fmla	v1.2d, v25.2d, v29.d[0]

	cmp		w13, #1
	ble		0f

	ldp		q24, q25, [x10, #0]
	add		x10, x10, x11
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]

	cmp		w13, #2
	ble		0f

	ldp		q24, q25, [x10, #0]
	add		x10, x10, x11
	fmla	v4.2d, v24.2d, v29.d[0]
	fmla	v5.2d, v25.2d, v29.d[0]

	cmp		w13, #3
	ble		0f

	ldp		q24, q25, [x10, #0]
	add		x10, x10, x11
	fmla	v6.2d, v24.2d, v29.d[0]
	fmla	v7.2d, v25.2d, v29.d[0]

	b 0f

1:
	cmp		w12, #3
	blt		2f

	ldr		q24, [x10, #0]
	ldr		d25, [x10, #16]
	add		x10, x10, x11
	fmla	v0.2d, v24.2d, v29.d[0]
	fmla	v1.2d, v25.2d, v29.d[0]

	cmp		w13, #1
	ble		0f

	ldr		q24, [x10, #0]
	ldr		d25, [x10, #16]
	add		x10, x10, x11
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]

	cmp		w13, #2
	ble		0f

	ldr		q24, [x10, #0]
	ldr		d25, [x10, #16]
	add		x10, x10, x11
	fmla	v4.2d, v24.2d, v29.d[0]
	fmla	v5.2d, v25.2d, v29.d[0]

	cmp		w13, #3
	ble		0f

	ldr		q24, [x10, #0]
	ldr		d25, [x10, #16]
	add		x10, x10, x11
	fmla	v6.2d, v24.2d, v29.d[0]
	fmla	v7.2d, v25.2d, v29.d[0]

	b 0f

2:
	cmp		w12, #2
	blt		3f

	ldr		q24, [x10, #0]
	add		x10, x10, x11
	fmla	v0.2d, v24.2d, v29.d[0]

	cmp		w13, #1
	ble		0f

	ldr		q24, [x10, #0]
	add		x10, x10, x11
	fmla	v2.2d, v24.2d, v29.d[0]

	cmp		w13, #2
	ble		0f

	ldr		q24, [x10, #0]
	add		x10, x10, x11
	fmla	v4.2d, v24.2d, v29.d[0]

	cmp		w13, #3
	ble		0f

	ldr		q24, [x10, #0]
	add		x10, x10, x11
	fmla	v6.2d, v24.2d, v29.d[0]

	b 0f

3:
	cmp		w12, #1
	blt		0f

	ldr		d24, [x10, #0]
	add		x10, x10, x11
	fmla	v0.2d, v24.2d, v29.d[0]

	cmp		w13, #1
	ble		0f

	ldr		d24, [x10, #0]
	add		x10, x10, x11
	fmla	v2.2d, v24.2d, v29.d[0]

	cmp		w13, #2
	ble		0f

	ldr		d24, [x10, #0]
	add		x10, x10, x11
	fmla	v4.2d, v24.2d, v29.d[0]

	cmp		w13, #3
	ble		0f

	ldr		d24, [x10, #0]
	add		x10, x10, x11
	fmla	v6.2d, v24.2d, v29.d[0]

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
// x8  <- beta
// x9  <- C
// x10 <- ldc
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M1B_4X4_LIB
#else
	.align	4
	FUN_START(inner_scale_m1b_4x4_lib)
#endif

	ld1		{v29.2d}, [x8]

	fneg	v0.2d, v0.2d
	fneg	v1.2d, v1.2d
	fneg	v2.2d, v2.2d
	fneg	v3.2d, v3.2d

	fneg	v4.2d, v4.2d
	fneg	v5.2d, v5.2d
	fneg	v6.2d, v6.2d
	fneg	v7.2d, v7.2d

	fcmpe	d29, #0.0
	beq		0f

	ldp		q24, q25, [x9, #0]
	add		x9, x9, x10
	ldp		q26, q27, [x9, #0]
	add		x9, x9, x10
	fmla	v0.2d, v24.2d, v29.d[0]
	fmla	v1.2d, v25.2d, v29.d[0]
	fmla	v2.2d, v26.2d, v29.d[0]
	fmla	v3.2d, v27.2d, v29.d[0]

	ldp		q24, q25, [x9, #0]
	add		x9, x9, x10
	ldp		q26, q27, [x9, #0]
	add		x9, x9, x10
	fmla	v4.2d, v24.2d, v29.d[0]
	fmla	v5.2d, v25.2d, v29.d[0]
	fmla	v6.2d, v26.2d, v29.d[0]
	fmla	v7.2d, v27.2d, v29.d[0]

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
// x19  <- ldc*sizeof(double)
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

	ld1		{v29.2d}, [x8]

	fneg	v0.2d, v0.2d
	fneg	v1.2d, v1.2d
	fneg	v2.2d, v2.2d
	fneg	v3.2d, v3.2d

	fneg	v4.2d, v4.2d
	fneg	v5.2d, v5.2d
	fneg	v6.2d, v6.2d
	fneg	v7.2d, v7.2d

	fcmpe	d29, #0.0
	beq		0f

	cmp		w11, #4
	blt		1f

	ldp		q24, q25, [x9, #0]
	add		x9, x9, x10
	fmla	v0.2d, v24.2d, v29.d[0]
	fmla	v1.2d, v25.2d, v29.d[0]

	cmp		w12, #1
	ble		0f

	ldp		q24, q25, [x9, #0]
	add		x9, x9, x10
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]

	cmp		w12, #2
	ble		0f

	ldp		q24, q25, [x9, #0]
	add		x9, x9, x10
	fmla	v4.2d, v24.2d, v29.d[0]
	fmla	v5.2d, v25.2d, v29.d[0]

	cmp		w12, #3
	ble		0f

	ldp		q24, q25, [x9, #0]
	add		x9, x9, x10
	fmla	v6.2d, v24.2d, v29.d[0]
	fmla	v7.2d, v25.2d, v29.d[0]

	b 0f

1:
	cmp		w11, #3
	blt		2f

	ldr		q24, [x9, #0]
	ldr		d25, [x9, #16]
	add		x9, x9, x10
	fmla	v0.2d, v24.2d, v29.d[0]
	fmla	v1.2d, v25.2d, v29.d[0]

	cmp		w12, #1
	ble		0f

	ldr		q24, [x9, #0]
	ldr		d25, [x9, #16]
	add		x9, x9, x10
	fmla	v2.2d, v24.2d, v29.d[0]
	fmla	v3.2d, v25.2d, v29.d[0]

	cmp		w12, #2
	ble		0f

	ldr		q24, [x9, #0]
	ldr		d25, [x9, #16]
	add		x9, x9, x10
	fmla	v4.2d, v24.2d, v29.d[0]
	fmla	v5.2d, v25.2d, v29.d[0]

	cmp		w12, #3
	ble		0f

	ldr		q24, [x9, #0]
	ldr		d25, [x9, #16]
	add		x9, x9, x10
	fmla	v6.2d, v24.2d, v29.d[0]
	fmla	v7.2d, v25.2d, v29.d[0]

	b 0f

2:
	cmp		w11, #2
	blt		3f

	ldr		q24, [x9, #0]
	add		x9, x9, x10
	fmla	v0.2d, v24.2d, v29.d[0]

	cmp		w12, #1
	ble		0f

	ldr		q24, [x9, #0]
	add		x9, x9, x10
	fmla	v2.2d, v24.2d, v29.d[0]

	cmp		w12, #2
	ble		0f

	ldr		q24, [x9, #0]
	add		x9, x9, x10
	fmla	v4.2d, v24.2d, v29.d[0]

	cmp		w12, #3
	ble		0f

	ldr		q24, [x9, #0]
	add		x9, x9, x10
	fmla	v6.2d, v24.2d, v29.d[0]

	b 0f

3:
	cmp		w11, #1
	blt		0f

	ldr		d24, [x9, #0]
	add		x9, x9, x10
	fmla	v0.2d, v24.2d, v29.d[0]

	cmp		w12, #1
	ble		0f

	ldr		d24, [x9, #0]
	add		x9, x9, x10
	fmla	v2.2d, v24.2d, v29.d[0]

	cmp		w12, #2
	ble		0f

	ldr		d24, [x9, #0]
	add		x9, x9, x10
	fmla	v4.2d, v24.2d, v29.d[0]

	cmp		w12, #3
	ble		0f

	ldr		d24, [x9, #0]
	add		x9, x9, x10
	fmla	v6.2d, v24.2d, v29.d[0]

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
// x9  <- ldc*sizeof(double)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_SCALE_M11_4X4_LIB
#else
	.align	4
	FUN_START(inner_scale_m11_4x4_lib)
#endif

	ldp		q24, q25, [x8, #0]
	add		x8, x8, x9
	ldp		q26, q27, [x8, #0]
	add		x8, x8, x9
	fsub	v0.2d, v24.2d, v0.2d
	fsub	v1.2d, v25.2d, v1.2d
	fsub	v2.2d, v26.2d, v2.2d
	fsub	v3.2d, v27.2d, v3.2d

	ldp		q24, q25, [x8, #0]
	add		x8, x8, x9
	ldp		q26, q27, [x8, #0]
	add		x8, x8, x9
	fsub	v4.2d, v24.2d, v4.2d
	fsub	v5.2d, v25.2d, v5.2d
	fsub	v6.2d, v26.2d, v6.2d
	fsub	v7.2d, v27.2d, v7.2d

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
// x9  <- ldc*sizeof(double)
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

	ldp		q24, q25, [x8, #0]
	add		x8, x8, x9
	fsub	v0.2d, v24.2d, v0.2d
	fsub	v1.2d, v25.2d, v1.2d

	cmp		w11, #1
	ble		0f

	ldp		q24, q25, [x8, #0]
	add		x8, x8, x9
	fsub	v2.2d, v24.2d, v2.2d
	fsub	v3.2d, v25.2d, v3.2d

	cmp		w11, #2
	ble		0f

	ldp		q24, q25, [x8, #0]
	add		x8, x8, x9
	fsub	v4.2d, v24.2d, v4.2d
	fsub	v5.2d, v25.2d, v5.2d

	cmp		w11, #3
	ble		0f

	ldp		q24, q25, [x8, #0]
	add		x8, x8, x9
	fsub	v6.2d, v24.2d, v6.2d
	fsub	v7.2d, v25.2d, v7.2d

	b 0f

1:
	cmp		w10, #3
	blt		2f

	ldr		q24, [x8, #0]
	ldr		d25, [x8, #16]
	add		x8, x8, x9
	fsub	v0.2d, v24.2d, v0.2d
	fsub	v1.2d, v25.2d, v1.2d

	cmp		w11, #1
	ble		0f

	ldr		q24, [x8, #0]
	ldr		d25, [x8, #16]
	add		x8, x8, x9
	fsub	v2.2d, v24.2d, v2.2d
	fsub	v3.2d, v25.2d, v3.2d

	cmp		w11, #2
	ble		0f

	ldr		q24, [x8, #0]
	ldr		d25, [x8, #16]
	add		x8, x8, x9
	fsub	v4.2d, v24.2d, v4.2d
	fsub	v5.2d, v25.2d, v5.2d

	cmp		w11, #3
	ble		0f

	ldr		q24, [x8, #0]
	ldr		d25, [x8, #16]
	add		x8, x8, x9
	fsub	v6.2d, v24.2d, v6.2d
	fsub	v7.2d, v25.2d, v7.2d

	b 0f

2:
	cmp		w10, #2
	blt		3f

	ldr		q24, [x8, #0]
	add		x8, x8, x9
	fsub	v0.2d, v24.2d, v0.2d

	cmp		w11, #1
	ble		0f

	ldr		q24, [x8, #0]
	add		x8, x8, x9
	fsub	v2.2d, v24.2d, v2.2d

	cmp		w11, #2
	ble		0f

	ldr		q24, [x8, #0]
	add		x8, x8, x9
	fsub	v4.2d, v24.2d, v4.2d

	cmp		w11, #3
	ble		0f

	ldr		q24, [x8, #0]
	add		x8, x8, x9
	fsub	v6.2d, v24.2d, v6.2d

	b 0f

3:
	cmp		w10, #1
	blt		0f

	ldr		d24, [x8, #0]
	add		x8, x8, x9
	fsub	v0.2d, v24.2d, v0.2d

	cmp		w11, #1
	ble		0f

	ldr		d24, [x8, #0]
	add		x8, x8, x9
	fsub	v2.2d, v24.2d, v2.2d

	cmp		w11, #2
	ble		0f

	ldr		d24, [x8, #0]
	add		x8, x8, x9
	fsub	v4.2d, v24.2d, v4.2d

	cmp		w11, #3
	ble		0f

	ldr		d24, [x8, #0]
	add		x8, x8, x9
	fsub	v6.2d, v24.2d, v6.2d

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
// x9   <- ldd*sizeof(double)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_4X4_LIB
#else
	.align 4
	FUN_START(inner_store_4x4_lib)
#endif

	stp		q0, q1, [x8, #0]
	add		x8, x8, x9
	stp		q2, q3, [x8, #0]
	add		x8, x8, x9
	stp		q4, q5, [x8, #0]
	add		x8, x8, x9
	stp		q6, q7, [x8, #0]

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
// x9   <- ldd*sizeof(double)
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
	blt		1f

	// 1st col
	stp		q0, q1, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	stp		q2, q3, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
	stp		q4, q5, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #3
	beq		0f
	// 4th col
	stp		q6, q7, [x8, #0]
	b		0f

1:
	cmp		w10, #3
	blt		1f

	// 1st col
	str		q0, [x8, #0]
	str		d1, [x8, #16]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	str		q2, [x8, #0]
	str		d3, [x8, #16]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
	str		q4, [x8, #0]
	str		d5, [x8, #16]
	add		x8, x8, x9
	cmp		w11, #3
	beq		0f
	// 4th col
	str		q6, [x8, #0]
	str		d7, [x8, #16]
	b		0f

1:
	cmp		w10, #2
	blt		1f

	// 1st col
	str		q0, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	str		q2, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
	str		q4, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #3
	beq		0f
	// 4th col
	str		q6, [x8, #0]
	b		0f

1:
	cmp		w10, #1
	blt		0f

	// 1st col
	str		d0, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	str		d2, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
	str		d4, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #3
	beq		0f
	// 4th col
	str		d6, [x8, #0]
//	b		0f

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
// x9   <- ldd*sizeof(double)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_L_4X4_LIB
#else
	.align 4
	FUN_START(inner_store_l_4x4_lib)
#endif

	ins		v16.d[0], v2.d[1]
	ins		v17.d[0], v7.d[1]

	stp		q0, q1, [x8, #0]
	add		x8, x8, x9
	str		d16, [x8, #8]
	str		q3, [x8, #16]
	add		x8, x8, x9
	str		q5, [x8, #16]
	add		x8, x8, x9
	str		d17, [x8, #24]

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
	blt		1f

	// 1st col
	stp		q0, q1, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	ins		v16.d[0], v2.d[1]
	str		d16, [x8, #8]
	str		q3, [x8, #16]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
	str		q5, [x8, #16]
	add		x8, x8, x9
	cmp		w11, #3
	beq		0f
	// 4th col
	ins		v17.d[0], v7.d[1]
	str		d17, [x8, #24]
	b		0f

1:
	cmp		w10, #3
	blt		1f

	// 1st col
	str		q0, [x8, #0]
	str		d1, [x8, #16]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	ins		v16.d[0], v2.d[1]
	str		d16, [x8, #8]
	str		d3, [x8, #16]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
	str		d5, [x8, #16]
	// 4th col
	b		0f

1:
	cmp		w10, #2
	blt		1f

	// 1st col
	str		q0, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	ins		v16.d[0], v2.d[1]
	str		d16, [x8, #8]
	// 3rd col
	// 4th col
	b		0f

1:
	cmp		w10, #1
	blt		0f

	// 1st col
	str		d0, [x8, #0]
	// 2nd col
	// 3rd col
	// 4th col
//	b		0f

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
// x9   <- ldd*sizeof(double)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_U_4X4_LIB
#else
	.align 4
	FUN_START(inner_store_u_4x4_lib)
#endif

	str		d0, [x8, #0]
	add		x8, x8, x9
	str		q2, [x8, #0]
	add		x8, x8, x9
	str		q4, [x8, #0]
	str		d5, [x8, #16]
	add		x8, x8, x9
	stp		q6, q7, [x8, #0]

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_u_4x4_lib)
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
	.macro INNER_STORE_U_4X4_VS_LIB
#else
	.align 4
	FUN_START(inner_store_u_4x4_vs_lib)
#endif

	cmp		w10, #4
	blt		1f

	// 1st col
	str		d0, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	str		q2, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
	str		q4, [x8, #0]
	str		d5, [x8, #16]
	add		x8, x8, x9
	cmp		w11, #3
	beq		0f
	// 4th col
	stp		q6, q7, [x8, #0]
	b		0f

1:
	cmp		w10, #3
	blt		1f

	// 1st col
	str		d0, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	str		q2, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
	str		q4, [x8, #0]
	str		d5, [x8, #16]
	add		x8, x8, x9
	cmp		w11, #3
	beq		0f
	// 4th col
	str		q6, [x8, #0]
	str		d7, [x8, #16]
	b		0f

1:
	cmp		w10, #2
	blt		1f

	// 1st col
	str		d0, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	str		q2, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
	str		q4, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #3
	beq		0f
	// 4th col
	str		q6, [x8, #0]
	b		0f

1:
	cmp		w10, #1
	blt		0f

	// 1st col
	str		d0, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #2
	blt		0f
	// 2nd col
	str		d2, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #3
	blt		0f
	// 3rd col
	str		d4, [x8, #0]
	add		x8, x8, x9
	cmp		w11, #3
	beq		0f
	// 4th col
	str		d6, [x8, #0]
//	b		0f

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_u_4x4_vs_lib)
#endif





// subroutine
//
// input arguments:
// x8   <- D
// x9   <- ldd*sizeof(double)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_PREFETCH_4X4_LIB
#else
	.align 4
	FUN_START(inner_prefetch_4x4_lib)
#endif

	prfm	PLDL1KEEP, [x8, #0]
	add		x8, x8, x9
	prfm	PLDL1KEEP, [x8, #0]
	add		x8, x8, x9
	prfm	PLDL1KEEP, [x8, #0]
	add		x8, x8, x9
	prfm	PLDL1KEEP, [x8, #0]

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_prefetch_4x4_lib)
#endif





//                                 w0        x1             x2         x3         x4            x5         w6       x7         sp+0
// void kernel_dgemm_nt_4x4_lib44cc(int kmax, double *alpha, double *A, double *B, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dgemm_nt_4x4_lib44cc)
	FUN_START(kernel_dgemm_nt_4x4_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// prefetch C
//	mov		x8, x4 // beta
	ld1		{v29.2d}, [x4]
	fcmpe	d29, #0.0
	beq		100f

	mov		x8, x6 // C
	mov		w9, w7 // ldc
	lsl		w9, w9, #3 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_PREFETCH_4X4_LIB
#else
	CALL(inner_prefetch_4x4_lib)
#endif

100:


	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4)
#endif



	// prefetch
	mov		x8, x7 // D
	ldr		w9, [sp, #(STACKSIZE + 0)] // ldd
	lsl		w9, w9, #3 // 8*ldd

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
	lsl		w11, w11, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB
#else
	CALL(inner_scale_ab_4x4_lib)
#endif



	// store n
	mov		x8, x7 // D
	ldr		w9, [sp, #(STACKSIZE + 0)] // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
	CALL(inner_store_4x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_nt_4x4_lib44cc)





//                                     w0        x1             x2         x3         x4            x5         w6       x7         sp+0     sp+8    sp+16
// void kernel_dgemm_nt_4x4_vs_lib44cc(int kmax, double *alpha, double *A, double *B, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dgemm_nt_4x4_vs_lib44cc)
	FUN_START(kernel_dgemm_nt_4x4_vs_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x4 // beta
	mov		x10, x5 // C
	mov		w11, w6 // ldc
	lsl		w11, w11, #3 // 8*ldc
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
	lsl		w9, w9, #3 // 8*ldd
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

	FUN_END(kernel_dgemm_nt_4x4_vs_lib44cc)





//                                 w0        x1             x2         x3         x4       x5            w6         x7       sp+0       sp+8
// void kernel_dgemm_nt_4x4_lib4ccc(int kmax, double *alpha, double *A, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dgemm_nt_4x4_lib4ccc)
	FUN_START(kernel_dgemm_nt_4x4_lib4ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // B
	mov		w11, w4 // ldb
	lsl		w11, w11, #3 // 8*ldb

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4c)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*sdd

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
	lsl		w11, w11, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB
#else
	CALL(inner_scale_ab_4x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
	CALL(inner_store_4x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_nt_4x4_lib4ccc)






//                                    w0        x1             x2         x3         x4       x5            w6         x7       sp+0       sp+8     sp+16   sp+24
// void kernel_dgemm_nt_4x4_vs_lib4ccc(int kmax, double *alpha, double *A, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dgemm_nt_4x4_vs_lib4ccc)
	FUN_START(kernel_dgemm_nt_4x4_vs_lib4ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // B
	mov		w11, w4 // ldb
	lsl		w11, w11, #3 // 8*ldb

	ldr		w12, [sp, #(STACKSIZE + 24)] // n1
	cmp		w12, #1
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X1_LIB4C
#else
	CALL(inner_kernel_gemm_nt_4x1_lib4c)
#endif
	
	b		103f

100:

	ldr		w12, [sp, #(STACKSIZE + 24)] // n1
	cmp		w12, #2
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X2_LIB4C
#else
	CALL(inner_kernel_gemm_nt_4x2_lib4c)
#endif
	
	b		103f

101:

	ldr		w12, [sp, #(STACKSIZE + 24)] // n1
	cmp		w12, #3
	bgt		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X3_LIB4C
#else
	CALL(inner_kernel_gemm_nt_4x3_lib4c)
#endif
	
	b		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4c)
#endif

103:



	// prefetch
	// TODO prefethc vs
//	ldr		x8, [sp, #(STACKSIZE + 0)] // D
//	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
//	lsl		w9, w9, #3 // 8*sdd

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
	lsl		w11, w11, #3 // 8*ldc
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
	lsl		w9, w9, #3 // 8*ldd
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

	FUN_END(kernel_dgemm_nt_4x4_vs_lib4ccc)






//                                 w0        x1             x2         x3         x4       x5            w6         x7       sp+0       sp+8      sp+16
// void kernel_dgemm_nt_4x4_libcccc(int kmax, double *alpha, double *A, int lda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dgemm_nt_4x4_libcccc)
	FUN_START(kernel_dgemm_nt_4x4_libcccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // lda
	lsl		w10, w10, #3 // 8*lda
	mov		x11, x4 // B
	mov		w12, w5 // ldb
	lsl		w12, w12, #3 // 8*ldb

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIBCC
#else
	CALL(inner_kernel_gemm_nt_4x4_libcc)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_PREFETCH_4X4_LIB
#else
	CALL(inner_prefetch_4x4_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x6 // beta
	mov		x10, x7 // C
	ldr		w11, [sp, #(STACKSIZE + 0)] // ldc
	lsl		w11, w11, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB
#else
	CALL(inner_scale_ab_4x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
	CALL(inner_store_4x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_nt_4x4_libcccc)





//                                     w0        x1             x2         x3         x4       x5            w6         x7       sp+0       sp+8      sp+16   sp+24  sp+32
// void kernel_dgemm_nt_4x4_vs_libcccc(int kmax, double *alpha, double *A, int lda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dgemm_nt_4x4_vs_libcccc)
	FUN_START(kernel_dgemm_nt_4x4_vs_libcccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // lda
	lsl		w10, w10, #3 // 8*lda
	mov		x11, x4 // B
	mov		w12, w5 // ldb
	lsl		w12, w12, #3 // 8*ldb
	ldr		w13, [sp, #(STACKSIZE + 24)] // m1

	ldr		w14, [sp, #(STACKSIZE + 32)] // n1

	cmp		w14, #1
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X1_VS_LIBCC
#else
	CALL(inner_kernel_gemm_nt_4x1_vs_libcc)
#endif

	b		103f

100:

	cmp		w14, #2
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X2_VS_LIBCC
#else
	CALL(inner_kernel_gemm_nt_4x2_vs_libcc)
#endif
	
	b		103f

101:

	cmp		w14, #3
	bgt		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X3_VS_LIBCC
#else
	CALL(inner_kernel_gemm_nt_4x3_vs_libcc)
#endif
	
	b		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_VS_LIBCC
#else
	CALL(inner_kernel_gemm_nt_4x4_vs_libcc)
#endif

103:



	// prefetch
	// TODO prefethc vs
//	ldr		x8, [sp, #(STACKSIZE + 8)] // D
//	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
//	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
//	INNER_PREFETCH_4X4_LIB
#else
//	CALL(inner_prefetch_4x4_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x6 // beta
	mov		x10, x7 // C
	ldr		w11, [sp, #(STACKSIZE + 0)] // ldc
	lsl		w11, w11, #3 // 8*ldc
	ldr		w12, [sp, #(STACKSIZE + 24)] // m1
	ldr		w13, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_VS_LIB
#else
	CALL(inner_scale_ab_4x4_vs_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*ldd
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

	FUN_END(kernel_dgemm_nt_4x4_vs_libcccc)





//                                 w0        x1             x2         x3         x4       x5            w6         x7       sp+0       sp+8
// void kernel_dgemm_nn_4x4_lib4ccc(int kmax, double *alpha, double *A, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dgemm_nn_4x4_lib4ccc)
	FUN_START(kernel_dgemm_nn_4x4_lib4ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // B
	mov		w11, w4 // ldb
	lsl		w11, w11, #3 // 8*ldb

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_nn_4x4_lib4c)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*sdd

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
	lsl		w11, w11, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB
#else
	CALL(inner_scale_ab_4x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
	CALL(inner_store_4x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_nn_4x4_lib4ccc)





//                                    w0        x1             x2         x3         x4       x5            w6         x7       sp+0       sp+8     sp+16   sp+24
// void kernel_dgemm_nn_4x4_vs_lib4ccc(int kmax, double *alpha, double *A, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dgemm_nn_4x4_vs_lib4ccc)
	FUN_START(kernel_dgemm_nn_4x4_vs_lib4ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // B
	mov		w11, w4 // ldb
	lsl		w11, w11, #3 // 8*ldb

	ldr		w12, [sp, #(STACKSIZE + 24)] // n1
	cmp		w12, #1
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X1_LIB4C
#else
	CALL(inner_kernel_gemm_nn_4x1_lib4c)
#endif

	b		103f

100:

	ldr		w12, [sp, #(STACKSIZE + 24)] // n1
	cmp		w12, #2
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X2_LIB4C
#else
	CALL(inner_kernel_gemm_nn_4x2_lib4c)
#endif
	
	b		103f

101:

	ldr		w12, [sp, #(STACKSIZE + 24)] // n1
	cmp		w12, #3
	bgt		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X3_LIB4C
#else
	CALL(inner_kernel_gemm_nn_4x3_lib4c)
#endif
	
	b		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_nn_4x4_lib4c)
#endif

103:



	// prefetch
	// TODO prefethc vs
//	ldr		x8, [sp, #(STACKSIZE + 0)] // D
//	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
//	lsl		w9, w9, #3 // 8*sdd

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
	lsl		w11, w11, #3 // 8*ldc
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
	lsl		w9, w9, #3 // 8*ldd
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

	FUN_END(kernel_dgemm_nn_4x4_vs_lib4ccc)





//                                 w0        x1             x2         x3         x4       x5            w6         x7       sp+0       sp+8      sp+16
// void kernel_dgemm_nn_4x4_libcccc(int kmax, double *alpha, double *A, int lda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dgemm_nn_4x4_libcccc)
	FUN_START(kernel_dgemm_nn_4x4_libcccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // lda
	lsl		w10, w10, #3 // 8*lda
	mov		x11, x4 // B
	mov		w12, w5 // ldb
	lsl		w12, w12, #3 // 8*ldb

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X4_LIBCC
#else
	CALL(inner_kernel_gemm_nn_4x4_libcc)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_PREFETCH_4X4_LIB
#else
	CALL(inner_prefetch_4x4_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x6 // beta
	mov		x10, x7 // C
	ldr		w11, [sp, #(STACKSIZE + 0)] // ldc
	lsl		w11, w11, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB
#else
	CALL(inner_scale_ab_4x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
	CALL(inner_store_4x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_nn_4x4_libcccc)





//                                    w0        x1             x2         x3         x4       x5            w6         x7       sp+0       sp+8     sp+16   sp+24     sp+32
// void kernel_dgemm_nn_4x4_vs_libcccc(int kmax, double *alpha, double *A, int lda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dgemm_nn_4x4_vs_libcccc)
	FUN_START(kernel_dgemm_nn_4x4_vs_libcccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // ldb
	lsl		w10, w10, #3 // 8*lda
	mov		x11, x4 // B
	mov		w12, w5 // ldb
	lsl		w12, w12, #3 // 8*ldb
	ldr		w13, [sp, #(STACKSIZE + 24)] // m1

	ldr		w14, [sp, #(STACKSIZE + 32)] // n1

	cmp		w14, #1
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X1_VS_LIBCC
#else
	CALL(inner_kernel_gemm_nn_4x1_vs_libcc)
#endif

	b		103f

100:

	cmp		w14, #2
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X2_VS_LIBCC
#else
	CALL(inner_kernel_gemm_nn_4x2_vs_libcc)
#endif
	
	b		103f

101:

	cmp		w14, #3
	bgt		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X3_VS_LIBCC
#else
	CALL(inner_kernel_gemm_nn_4x3_vs_libcc)
#endif
	
	b		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X4_VS_LIBCC
#else
	CALL(inner_kernel_gemm_nn_4x4_vs_libcc)
#endif

103:



	// prefetch
	// TODO prefethc vs
//	ldr		x8, [sp, #(STACKSIZE + 0)] // D
//	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
//	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
//	INNER_PREFETCH_4X4_LIB
#else
//	CALL(inner_prefetch_4x4_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x6 // beta
	mov		x10, x7 // C
	ldr		w11, [sp, #(STACKSIZE + 0)] // ldc
	lsl		w11, w11, #3 // 8*ldc
	ldr		w12, [sp, #(STACKSIZE + 24)] // m1
	ldr		w13, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_VS_LIB
#else
	CALL(inner_scale_ab_4x4_vs_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*ldd
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

	FUN_END(kernel_dgemm_nn_4x4_vs_libcccc)





//                                 w0        x1             x2         x3         x4       x5            w6         x7       sp+0       sp+8      sp+16
// void kernel_dgemm_tt_4x4_libcccc(int kmax, double *alpha, double *A, int lda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dgemm_tt_4x4_libcccc)
	FUN_START(kernel_dgemm_tt_4x4_libcccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x4 // B
	mov		w10, w5 // ldb
	lsl		w10, w10, #3 // 8*ldb
	mov		x11, x2 // A
	mov		w12, w3 // lda
	lsl		w12, w12, #3 // 8*lda

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X4_LIBCC
#else
	CALL(inner_kernel_gemm_nn_4x4_libcc)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_PREFETCH_4X4_LIB
#else
	CALL(inner_prefetch_4x4_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x6 // beta
	mov		x10, x7 // C
	ldr		w11, [sp, #(STACKSIZE + 0)] // ldc
	lsl		w11, w11, #3 // 8*ldc

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
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
	CALL(inner_store_4x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_tt_4x4_libcccc)





//                                    w0        x1             x2         x3         x4       x5            w6         x7       sp+0       sp+8     sp+16   sp+24     sp+32
// void kernel_dgemm_tt_4x4_vs_libcccc(int kmax, double *alpha, double *A, int lda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dgemm_tt_4x4_vs_libcccc)
	FUN_START(kernel_dgemm_tt_4x4_vs_libcccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x4 // B
	mov		w10, w5 // ldb
	lsl		w10, w10, #3 // 8*ldb
	mov		x11, x2 // A
	mov		w12, w3 // lda
	lsl		w12, w12, #3 // 8*lda
	ldr		w13, [sp, #(STACKSIZE + 32)] // n1

	ldr		w14, [sp, #(STACKSIZE + 24)] // m1

	cmp		w14, #1
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X1_VS_LIBCC
#else
	CALL(inner_kernel_gemm_nn_4x1_vs_libcc)
#endif

	b		103f

100:

	cmp		w14, #2
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X2_VS_LIBCC
#else
	CALL(inner_kernel_gemm_nn_4x2_vs_libcc)
#endif
	
	b		103f

101:

	cmp		w14, #3
	bgt		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X3_VS_LIBCC
#else
	CALL(inner_kernel_gemm_nn_4x3_vs_libcc)
#endif
	
	b		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X4_VS_LIBCC
#else
	CALL(inner_kernel_gemm_nn_4x4_vs_libcc)
#endif

103:



	// prefetch
	// TODO prefethc vs
//	ldr		x8, [sp, #(STACKSIZE + 0)] // D
//	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
//	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
//	INNER_PREFETCH_4X4_LIB
#else
//	CALL(inner_prefetch_4x4_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x6 // beta
	mov		x10, x7 // C
	ldr		w11, [sp, #(STACKSIZE + 0)] // ldc
	lsl		w11, w11, #3 // 8*ldc
	ldr		w12, [sp, #(STACKSIZE + 24)] // m1
	ldr		w13, [sp, #(STACKSIZE + 32)] // n1

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
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*ldd
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

	FUN_END(kernel_dgemm_nn_4x4_vs_libcccc)





//                                 w0        x1             x2         x3         x4       x5            w6         x7       sp+0       sp+8
// void kernel_dsyrk_nt_l_4x4_lib4ccc(int kmax, double *alpha, double *A, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dsyrk_nt_l_4x4_lib4ccc)
	FUN_START(kernel_dsyrk_nt_l_4x4_lib4ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // B
	mov		w11, w4 // ldb
	lsl		w11, w11, #3 // 8*ldb

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4c)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*sdd

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
	lsl		w11, w11, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB
#else
	CALL(inner_scale_ab_4x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_L_4X4_LIB
#else
	CALL(inner_store_l_4x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dsyrk_nt_l_4x4_lib4ccc)






//                                    w0        x1             x2         x3         x4       x5            w6         x7       sp+0       sp+8     sp+16   sp+24
// void kernel_dsyrk_nt_l_4x4_vs_lib4ccc(int kmax, double *alpha, double *A, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dsyrk_nt_l_4x4_vs_lib4ccc)
	FUN_START(kernel_dsyrk_nt_l_4x4_vs_lib4ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // B
	mov		w11, w4 // ldb
	lsl		w11, w11, #3 // 8*ldb

	ldr		w12, [sp, #(STACKSIZE + 24)] // n1
	cmp		w12, #1
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X1_LIB4C
#else
	CALL(inner_kernel_gemm_nt_4x1_lib4c)
#endif
	
	b		103f

100:

	ldr		w12, [sp, #(STACKSIZE + 24)] // n1
	cmp		w12, #2
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X2_LIB4C
#else
	CALL(inner_kernel_gemm_nt_4x2_lib4c)
#endif
	
	b		103f

101:

	ldr		w12, [sp, #(STACKSIZE + 24)] // n1
	cmp		w12, #3
	bgt		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X3_LIB4C
#else
	CALL(inner_kernel_gemm_nt_4x3_lib4c)
#endif
	
	b		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4c)
#endif

103:



	// prefetch
	// TODO prefethc vs
//	ldr		x8, [sp, #(STACKSIZE + 0)] // D
//	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
//	lsl		w9, w9, #3 // 8*sdd

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
	lsl		w11, w11, #3 // 8*ldc
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
	lsl		w9, w9, #3 // 8*ldd
	ldr		w10, [sp, #(STACKSIZE + 16)] // m1
	ldr		w11, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_L_4X4_VS_LIB
#else
	CALL(inner_store_l_4x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dsyrk_nt_l_4x4_vs_lib4ccc)






//                                   w0        x1             x2         x3         x4            x5         w6       x7         sp+0
// void kernel_dsyrk_nt_l_4x4_lib44cc(int kmax, double *alpha, double *A, double *B, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dsyrk_nt_l_4x4_lib44cc)
	FUN_START(kernel_dsyrk_nt_l_4x4_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x4 // beta
	mov		x10, x5 // C
	mov		w11, w6 // ldc
	lsl		w11, w11, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB
#else
	CALL(inner_scale_ab_4x4_lib)
#endif



	// store n
	mov		x8, x7 // D
	ldr		w9, [sp, #(STACKSIZE + 0)] // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_L_4X4_LIB
#else
	CALL(inner_store_l_4x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dsyrk_nt_l_4x4_lib44cc)





//                                      w0        x1             x2         x3         x4            x5         w6       x7         sp+0     sp+8    sp+16
// void kernel_dsyrk_nt_l_4x4_vs_lib44cc(int kmax, double *alpha, double *A, double *B, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dsyrk_nt_l_4x4_vs_lib44cc)
	FUN_START(kernel_dsyrk_nt_l_4x4_vs_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x4 // beta
	mov		x10, x5 // C
	mov		w11, w6 // ldc
	lsl		w11, w11, #3 // 8*ldc
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
	lsl		w9, w9, #3 // 8*ldd
	ldr		w10, [sp, #(STACKSIZE + 8)] // m1
	ldr		w11, [sp, #(STACKSIZE + 16)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_L_4X4_VS_LIB
#else
	CALL(inner_store_l_4x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dsyrk_nt_l_4x4_vs_lib44cc)






//                                   w0        x1             x2         x3         x4            x5         w6       x7         sp+0
// void kernel_dsyrk_nt_u_4x4_lib44cc(int kmax, double *alpha, double *A, double *B, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dsyrk_nt_u_4x4_lib44cc)
	FUN_START(kernel_dsyrk_nt_u_4x4_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x4 // beta
	mov		x10, x5 // C
	mov		w11, w6 // ldc
	lsl		w11, w11, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB
#else
	CALL(inner_scale_ab_4x4_lib)
#endif



	// store n
	mov		x8, x7 // D
	ldr		w9, [sp, #(STACKSIZE + 0)] // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_U_4X4_LIB
#else
	CALL(inner_store_u_4x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dsyrk_nt_u_4x4_lib44cc)





//                                      w0        x1             x2         x3         x4            x5         w6       x7         sp+0     sp+8    sp+16
// void kernel_dsyrk_nt_u_4x4_vs_lib44cc(int kmax, double *alpha, double *A, double *B, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dsyrk_nt_u_4x4_vs_lib44cc)
	FUN_START(kernel_dsyrk_nt_u_4x4_vs_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x4 // beta
	mov		x10, x5 // C
	mov		w11, w6 // ldc
	lsl		w11, w11, #3 // 8*ldc
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
	lsl		w9, w9, #3 // 8*ldd
	ldr		w10, [sp, #(STACKSIZE + 8)] // m1
	ldr		w11, [sp, #(STACKSIZE + 16)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_U_4X4_VS_LIB
#else
	CALL(inner_store_u_4x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dsyrk_nt_u_4x4_vs_lib44cc)





//                                   w0     x1             x2          x3          w4        x5          x6          w7        sp+0          sp+8       sp+16    sp+24      sp+32
// void kernel_dger2k_nt_4x4_lib4ccc(int k, double *alpha, double *A0, double *B0, int ldb0, double *A1, double *B1, int ldb1, double *beta, double *C, int ldc, double *D, int ldd);

	.align	4
	GLOB(kernel_dger2k_nt_4x4_lib4ccc)
	FUN_START(kernel_dger2k_nt_4x4_lib4ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A0
	mov		x10, x3 // B0
	mov		w11, w4 // ldb0
	lsl		w11, w11, #3 // 8*ldb0

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4c)
#endif

	mov		w8, w0 // kmax
	mov		x9, x5 // A1
	mov		x10, x6 // B1
	mov		w11, w7 // ldb1
	lsl		w11, w11, #3 // 8*ldb1

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4c)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 24)] // D
	ldr		w9, [sp, #(STACKSIZE + 32)] // ldd
	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
	INNER_PREFETCH_4X4_LIB
#else
	CALL(inner_prefetch_4x4_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	ldr		x9, [sp, #(STACKSIZE + 0)] // beta
	ldr		x10, [sp, #(STACKSIZE + 8)] // C
	ldr		w11, [sp, #(STACKSIZE + 16)] // ldc
	lsl		w11, w11, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB
#else
	CALL(inner_scale_ab_4x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 24)] // D
	ldr		w9, [sp, #(STACKSIZE + 32)] // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
	CALL(inner_store_4x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dger2k_nt_4x4_lib4ccc)






//                                   w0     x1             x2          x3          w4        x5          x6          w7        sp+0          sp+8       sp+16    sp+24      sp+32    sp+40   sp+48
// void kernel_dger2k_nt_4x4_lib4ccc(int k, double *alpha, double *A0, double *B0, int ldb0, double *A1, double *B1, int ldb1, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.align	4
	GLOB(kernel_dger2k_nt_4x4_vs_lib4ccc)
	FUN_START(kernel_dger2k_nt_4x4_vs_lib4ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	ldr		w12, [sp, #(STACKSIZE + 48)] // n1
	cmp		w12, #1
	bgt		100f

	mov		w8, w0 // kmax
	mov		x9, x2 // A0
	mov		x10, x3 // B0
	mov		w11, w4 // ldb0
	lsl		w11, w11, #3 // 8*ldb0

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X1_LIB4C
#else
	CALL(inner_kernel_gemm_nt_4x1_lib4c)
#endif

	mov		w8, w0 // kmax
	mov		x9, x5 // A1
	mov		x10, x6 // B1
	mov		w11, w7 // ldb1
	lsl		w11, w11, #3 // 8*ldb1

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X1_LIB4C
#else
	CALL(inner_kernel_gemm_nt_4x1_lib4c)
#endif

	b		103f

100:

	ldr		w12, [sp, #(STACKSIZE + 48)] // n1
	cmp		w12, #2
	bgt		101f

	mov		w8, w0 // kmax
	mov		x9, x2 // A0
	mov		x10, x3 // B0
	mov		w11, w4 // ldb0
	lsl		w11, w11, #3 // 8*ldb0

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X2_LIB4C
#else
	CALL(inner_kernel_gemm_nt_4x2_lib4c)
#endif

	mov		w8, w0 // kmax
	mov		x9, x5 // A1
	mov		x10, x6 // B1
	mov		w11, w7 // ldb1
	lsl		w11, w11, #3 // 8*ldb1

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X2_LIB4C
#else
	CALL(inner_kernel_gemm_nt_4x2_lib4c)
#endif

	b		103f

101:

	ldr		w12, [sp, #(STACKSIZE + 48)] // n1
	cmp		w12, #3
	bgt		102f

	mov		w8, w0 // kmax
	mov		x9, x2 // A0
	mov		x10, x3 // B0
	mov		w11, w4 // ldb0
	lsl		w11, w11, #3 // 8*ldb0

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X3_LIB4C
#else
	CALL(inner_kernel_gemm_nt_4x3_lib4c)
#endif

	mov		w8, w0 // kmax
	mov		x9, x5 // A1
	mov		x10, x6 // B1
	mov		w11, w7 // ldb1
	lsl		w11, w11, #3 // 8*ldb1

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X3_LIB4C
#else
	CALL(inner_kernel_gemm_nt_4x3_lib4c)
#endif

	b		103f

102:

	mov		w8, w0 // kmax
	mov		x9, x2 // A0
	mov		x10, x3 // B0
	mov		w11, w4 // ldb0
	lsl		w11, w11, #3 // 8*ldb0

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4c)
#endif

	mov		w8, w0 // kmax
	mov		x9, x5 // A1
	mov		x10, x6 // B1
	mov		w11, w7 // ldb1
	lsl		w11, w11, #3 // 8*ldb1

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4c)
#endif

103:



	// prefetch
	// TODO prefethc vs
//	ldr		x8, [sp, #(STACKSIZE + 24)] // D
//	ldr		w9, [sp, #(STACKSIZE + 32)] // ldd
//	lsl		w9, w9, #3 // 8*sdd

#if MACRO_LEVEL>=1
//	INNER_PREFETCH_4X4_LIB
#else
//	CALL(inner_prefetch_4x4_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	ldr		x9, [sp, #(STACKSIZE + 0)] // beta
	ldr		x10, [sp, #(STACKSIZE + 8)] // C
	ldr		w11, [sp, #(STACKSIZE + 16)] // ldc
	lsl		w11, w11, #3 // 8*ldc
	ldr		w12, [sp, #(STACKSIZE + 40)] // m1
	ldr		w13, [sp, #(STACKSIZE + 48)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_VS_LIB
#else
	CALL(inner_scale_ab_4x4_vs_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 24)] // D
	ldr		w9, [sp, #(STACKSIZE + 32)] // ldd
	lsl		w9, w9, #3 // 8*ldd
	ldr		w10, [sp, #(STACKSIZE + 40)] // m1
	ldr		w11, [sp, #(STACKSIZE + 48)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB
#else
	CALL(inner_store_4x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dger2k_nt_4x4_vs_lib4ccc)






//                                   w0     x1             x2         x3           x4          x5          x6            x7         sp+0     sp+8       sp+16
// void kernel_dger2k_nt_4x4_lib44cc(int k, double *alpha, double *A0, double *B0, double *A1, double *B1, double *beta, double *C, int ldc, double *D, int ldd);

	.align	4
	GLOB(kernel_dger2k_nt_4x4_lib44cc)
	FUN_START(kernel_dger2k_nt_4x4_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// prefetch C
//	mov		x8, x6 // beta
	ld1		{v29.2d}, [x6]
	fcmpe	d29, #0.0
	beq		100f

	mov		x8, x7 // C
	ldr		w9, [sp, #(STACKSIZE + 0)] // ldc
	lsl		w9, w9, #3 // 8*sdc

#if MACRO_LEVEL>=1
	INNER_PREFETCH_4X4_LIB
#else
	CALL(inner_prefetch_4x4_lib)
#endif

100:


	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A0
	mov		x10, x3 // B0

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4)
#endif

	mov		w8, w0 // kmax
	mov		x9, x4 // A1
	mov		x10, x5 // B1

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_PREFETCH_4X4_LIB
#else
	CALL(inner_prefetch_4x4_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x6 // beta
	mov		x10, x7 // C
	ldr		w11, [sp, #(STACKSIZE + 0)] // ldc
	lsl		w11, w11, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB
#else
	CALL(inner_scale_ab_4x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
	CALL(inner_store_4x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dger2k_nt_4x4_lib44cc)





//                                   w0     x1             x2         x3           x4          x5          x6            x7         sp+0     sp+8       sp+16    sp+24   sp+32
// void kernel_dger2k_nt_4x4_lib44cc(int k, double *alpha, double *A0, double *B0, double *A1, double *B1, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.align	4
	GLOB(kernel_dger2k_nt_4x4_vs_lib44cc)
	FUN_START(kernel_dger2k_nt_4x4_vs_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A0
	mov		x10, x3 // B0

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4)
#endif

	mov		w8, w0 // kmax
	mov		x9, x4 // A1
	mov		x10, x5 // B1

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x6 // beta
	mov		x10, x7 // C
	ldr		w11, [sp, #(STACKSIZE + 0)] // ldc
	lsl		w11, w11, #3 // 8*ldc
	ldr		w12, [sp, #(STACKSIZE + 24)] // m1
	ldr		w13, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_VS_LIB
#else
	CALL(inner_scale_ab_4x4_vs_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*ldd
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

	FUN_END(kernel_dger2k_nt_4x4_vs_lib44cc)





//                                     w0     x1             x2         x3           x4          x5          x6            x7         sp+0     sp+8       sp+16
// void kernel_dsyr2k_nt_l_4x4_lib44cc(int k, double *alpha, double *A0, double *B0, double *A1, double *B1, double *beta, double *C, int ldc, double *D, int ldd);

	.align	4
	GLOB(kernel_dsyr2k_nt_l_4x4_lib44cc)
	FUN_START(kernel_dsyr2k_nt_l_4x4_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A0
	mov		x10, x3 // B0

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4)
#endif

	mov		w8, w0 // kmax
	mov		x9, x4 // A1
	mov		x10, x5 // B1

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_PREFETCH_4X4_LIB
#else
	CALL(inner_prefetch_4x4_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x6 // beta
	mov		x10, x7 // C
	ldr		w11, [sp, #(STACKSIZE + 0)] // ldc
	lsl		w11, w11, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB
#else
	CALL(inner_scale_ab_4x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_L_4X4_LIB
#else
	CALL(inner_store_l_4x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dsyr2k_nt_l_4x4_lib44cc)





//                                     w0     x1             x2         x3           x4          x5          x6            x7         sp+0     sp+8       sp+16    sp+24   sp+32
// void kernel_dsyr2k_nt_l_4x4_lib44cc(int k, double *alpha, double *A0, double *B0, double *A1, double *B1, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1);

	.align	4
	GLOB(kernel_dsyr2k_nt_l_4x4_vs_lib44cc)
	FUN_START(kernel_dsyr2k_nt_l_4x4_vs_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A0
	mov		x10, x3 // B0

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4)
#endif

	mov		w8, w0 // kmax
	mov		x9, x4 // A1
	mov		x10, x5 // B1

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x6 // beta
	mov		x10, x7 // C
	ldr		w11, [sp, #(STACKSIZE + 0)] // ldc
	lsl		w11, w11, #3 // 8*ldc
	ldr		w12, [sp, #(STACKSIZE + 24)] // m1
	ldr		w13, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_VS_LIB
#else
	CALL(inner_scale_ab_4x4_vs_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #3 // 8*ldd
	ldr		w10, [sp, #(STACKSIZE + 24)] // m1
	ldr		w11, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_L_4X4_VS_LIB
#else
	CALL(inner_store_l_4x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dsyr2k_nt_l_4x4_vs_lib44cc)





//                                          w0        x1             x2         x3         x4            x5         x6         w7
// void kernel_dtrmm_nt_rl_4x4_tran_lib444c(int kmax, double *alpha, double *A, double *B, double *beta, double *C, double *D, int ldd)

	.align	4
	GLOB(kernel_dtrmm_nt_rl_4x4_tran_lib444c)
	FUN_START(kernel_dtrmm_nt_rl_4x4_tran_lib444c)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4)
#endif

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMM_NT_RL_4X4_LIB4
#else
	CALL(inner_edge_trmm_nt_rl_4x4_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x4 // beta
	mov		x10, x5 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
	CALL(inner_scale_ab_4x4_lib4)
#endif



	// store n
	mov		x8, x6 // D
	mov		w9, w7 // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_TRAN_4X4_LIB
#else
	CALL(inner_tran_4x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
	CALL(inner_store_4x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrmm_nt_rl_4x4_tran_lib444c)





//                                             w0        x1             x2         x3         x4            x5         x6         w7       sp+0    sp+8
// void kernel_dtrmm_nt_rl_4x4_tran_vs_lib444c(int kmax, double *alpha, double *A, double *B, double *beta, double *C, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dtrmm_nt_rl_4x4_tran_vs_lib444c)
	FUN_START(kernel_dtrmm_nt_rl_4x4_tran_vs_lib444c)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4)
#endif

	ldr		w11, [sp, #(STACKSIZE + 0)] // m1

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMM_NT_RL_4X4_VS_LIB4
#else
	CALL(inner_edge_trmm_nt_rl_4x4_vs_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x4 // beta
	mov		x10, x5 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
	CALL(inner_scale_ab_4x4_lib4)
#endif



	// store n
	mov		x8, x6 // D
	mov		w9, w7 // ldd
	lsl		w9, w9, #3 // 8*ldd
	ldr		w10, [sp, #(STACKSIZE + 0)] // m1
	ldr		w11, [sp, #(STACKSIZE + 8)] // n1

#if MACRO_LEVEL>=1
	INNER_TRAN_4X4_LIB
#else
	CALL(inner_tran_4x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB
#else
	CALL(inner_store_4x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrmm_nt_rl_4x4_tran_vs_lib444c)





//                                          w0        x1             x2         x3         w4       x5            x6         x7         sp+0
// void kernel_dtrmm_nt_rl_4x4_tran_lib4c4c(int kmax, double *alpha, double *A, double *B, int ldb, double *beta, double *C, double *D, int ldd)

	.align	4
	GLOB(kernel_dtrmm_nt_rl_4x4_tran_lib4c4c)
	FUN_START(kernel_dtrmm_nt_rl_4x4_tran_lib4c4c)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // B
	mov		w11, w4 // ldb
	lsl		w11, w11, #3 // 8*ldb

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4c)
#endif

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMM_NT_RL_4X4_LIB4C
#else
	CALL(inner_edge_trmm_nt_rl_4x4_lib4c)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
	CALL(inner_scale_ab_4x4_lib4)
#endif



	// store n
	mov		x8, x7 // D
	ldr		w9, [sp, #(STACKSIZE + 0)] // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_TRAN_4X4_LIB
#else
	CALL(inner_tran_4x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
	CALL(inner_store_4x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrmm_nt_rl_4x4_tran_lib4c4c)





//                                             w0        x1             x2         x3         w4       x5            x6         x7         sp+0     sp+8    sp+16
// void kernel_dtrmm_nt_rl_4x4_tran_vs_lib4c4c(int kmax, double *alpha, double *A, double *B, int ldb, double *beta, double *C, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dtrmm_nt_rl_4x4_tran_vs_lib4c4c)
	FUN_START(kernel_dtrmm_nt_rl_4x4_tran_vs_lib4c4c)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // B
	mov		w11, w4 // ldb
	lsl		w11, w11, #3 // 8*ldb

	ldr		w12, [sp, #(STACKSIZE + 8)] // m1
	cmp		w12, #1
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X1_LIB4C
#else
	CALL(inner_kernel_gemm_nt_4x1_lib4c)
#endif

	b		103f

100:

	ldr		w12, [sp, #(STACKSIZE + 8)] // m1
	cmp		w12, #2
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X2_LIB4C
#else
	CALL(inner_kernel_gemm_nt_4x2_lib4c)
#endif
	
	b		103f

101:

	ldr		w12, [sp, #(STACKSIZE + 8)] // m1
	cmp		w12, #3
	bgt		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X3_LIB4C
#else
	CALL(inner_kernel_gemm_nt_4x3_lib4c)
#endif
	
	b		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4c)
#endif

103:

	ldr		w12, [sp, #(STACKSIZE + 8)] // m1

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMM_NT_RL_4X4_VS_LIB4C
#else
	CALL(inner_edge_trmm_nt_rl_4x4_vs_lib4c)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
	CALL(inner_scale_ab_4x4_lib4)
#endif



	// store n
	mov		x8, x7 // D
	ldr		w9, [sp, #(STACKSIZE + 0)] // ldd
	lsl		w9, w9, #3 // 8*ldd
	ldr		w10, [sp, #(STACKSIZE + 8)] // m1
	ldr		w11, [sp, #(STACKSIZE + 16)] // n1

#if MACRO_LEVEL>=1
	INNER_TRAN_4X4_LIB
#else
	CALL(inner_tran_4x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB
#else
	CALL(inner_store_4x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrmm_nt_rl_4x4_tran_vs_lib4c4c)





//                                     w0        x1             x2         x3         x4            x5         x6         w7
// void kernel_dtrmm_nt_ru_4x4_lib444c(int kmax, double *alpha, double *A, double *B, double *beta, double *C, double *D, int ldd)

	.align	4
	GLOB(kernel_dtrmm_nt_ru_4x4_lib444c)
	FUN_START(kernel_dtrmm_nt_ru_4x4_lib444c)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // B

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMM_NT_RU_4X4_LIB4
#else
	CALL(inner_edge_trmm_nt_ru_4x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x4 // beta
	mov		x10, x5 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
	CALL(inner_scale_ab_4x4_lib4)
#endif



	// store n
	mov		x8, x6 // D
	mov		w9, w7 // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
	CALL(inner_store_4x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrmm_nt_ru_4x4_lib444c)





//                                        w0        x1             x2         x3         x4            x5         x6         w7       sp+0    sp+8
// void kernel_dtrmm_nt_ru_4x4_vs_lib444c(int kmax, double *alpha, double *A, double *B, double *beta, double *C, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dtrmm_nt_ru_4x4_vs_lib444c)
	FUN_START(kernel_dtrmm_nt_ru_4x4_vs_lib444c)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // B

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMM_NT_RU_4X4_VS_LIB4
#else
	CALL(inner_edge_trmm_nt_ru_4x4_vs_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x4 // beta
	mov		x10, x5 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
	CALL(inner_scale_ab_4x4_lib4)
#endif



	// store n
	mov		x8, x6 // D
	mov		w9, w7 // ldd
	lsl		w9, w9, #3 // 8*ldd
	ldr		w10, [sp, #(STACKSIZE + 0)] // m1
	ldr		w11, [sp, #(STACKSIZE + 8)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB
#else
	CALL(inner_store_4x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrmm_nt_ru_4x4_vs_lib444c)





//                                          w0        x1             x2         x3         x4            x5         x6         w7
// void kernel_dtrmm_nt_ru_4x4_tran_lib444c(int kmax, double *alpha, double *A, double *B, double *beta, double *C, double *D, int ldd)

	.align	4
	GLOB(kernel_dtrmm_nt_ru_4x4_tran_lib444c)
	FUN_START(kernel_dtrmm_nt_ru_4x4_tran_lib444c)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // B

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMM_NT_RU_4X4_LIB4
#else
	CALL(inner_edge_trmm_nt_ru_4x4_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x4 // beta
	mov		x10, x5 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
	CALL(inner_scale_ab_4x4_lib4)
#endif



	// store n
	mov		x8, x6 // D
	mov		w9, w7 // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_TRAN_4X4_LIB
#else
	CALL(inner_tran_4x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
	CALL(inner_store_4x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrmm_nt_ru_4x4_tran_lib444c)





//                                             w0        x1             x2         x3         x4            x5         x6         w7       sp+0    sp+8
// void kernel_dtrmm_nt_ru_4x4_tran_vs_lib444c(int kmax, double *alpha, double *A, double *B, double *beta, double *C, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dtrmm_nt_ru_4x4_tran_vs_lib444c)
	FUN_START(kernel_dtrmm_nt_ru_4x4_tran_vs_lib444c)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // B

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMM_NT_RU_4X4_VS_LIB4
#else
	CALL(inner_edge_trmm_nt_ru_4x4_vs_lib4)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x4 // beta
	mov		x10, x5 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
	CALL(inner_scale_ab_4x4_lib4)
#endif



	// store n
	mov		x8, x6 // D
	mov		w9, w7 // ldd
	lsl		w9, w9, #3 // 8*ldd
	ldr		w10, [sp, #(STACKSIZE + 0)] // m1
	ldr		w11, [sp, #(STACKSIZE + 8)] // n1

#if MACRO_LEVEL>=1
	INNER_TRAN_4X4_LIB
#else
	CALL(inner_tran_4x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB
#else
	CALL(inner_store_4x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrmm_nt_ru_4x4_tran_vs_lib444c)





//                                     w0        x1             x2         x3         w4       x5            x6         w7       sp+0       sp+8
// void kernel_dtrmm_nn_rl_4x4_lib4ccc(int kmax, double *alpha, double *A, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dtrmm_nn_rl_4x4_lib4ccc)
	FUN_START(kernel_dtrmm_nn_rl_4x4_lib4ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // B
	mov		w11, w4 // ldb
	lsl		w11, w11, #3 // 8*ldb

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMM_NN_RL_4X4_LIB4C
#else
	CALL(inner_edge_trmm_nn_rl_4x4_lib4c)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_nn_4x4_lib4c)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // ldc
	lsl		w11, w11, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB
#else
	CALL(inner_scale_ab_4x4_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 0)] // D
	ldr		w9, [sp, #(STACKSIZE + 8)] // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
	CALL(inner_store_4x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrmm_nn_rl_4x4_lib4ccc)





//                                        w0        x1             x2         x3         w4       x5            x6         w7       sp+0       sp+8     sp+16   sp+24
// void kernel_dtrmm_nn_rl_4x4_vs_lib4ccc(int kmax, double *alpha, double *A, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dtrmm_nn_rl_4x4_vs_lib4ccc)
	FUN_START(kernel_dtrmm_nn_rl_4x4_vs_lib4ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // B
	mov		w11, w4 // ldb
	lsl		w11, w11, #3 // 8*ldb

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMM_NN_RL_4X4_VS_LIB4C
#else
	CALL(inner_edge_trmm_nn_rl_4x4_vs_lib4c)
#endif

	ldr		w12, [sp, #(STACKSIZE + 24)] // n1
	cmp		w12, #1
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X1_LIB4C
#else
	CALL(inner_kernel_gemm_nn_4x1_lib4c)
#endif

	b		103f

100:

	ldr		w12, [sp, #(STACKSIZE + 24)] // n1
	cmp		w12, #2
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X2_LIB4C
#else
	CALL(inner_kernel_gemm_nn_4x2_lib4c)
#endif
	
	b		103f

101:

	ldr		w12, [sp, #(STACKSIZE + 24)] // n1
	cmp		w12, #3
	bgt		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X3_LIB4C
#else
	CALL(inner_kernel_gemm_nn_4x3_lib4c)
#endif
	
	b		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_nn_4x4_lib4c)
#endif

103:



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // ldc
	lsl		w11, w11, #3 // 8*ldc
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
	lsl		w9, w9, #3 // 8*ldd
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

	FUN_END(kernel_dtrmm_nn_rl_4x4_vs_lib4ccc)





//                                          w0        x1             x2         x3         w4       x5            x6         x7         sp+0
// void kernel_dtrmm_nn_rl_4x4_tran_lib4c4c(int kmax, double *alpha, double *A, double *B, int ldb, double *beta, double *C, double *D, int ldd)

	.align	4
	GLOB(kernel_dtrmm_nn_rl_4x4_tran_lib4c4c)
	FUN_START(kernel_dtrmm_nn_rl_4x4_tran_lib4c4c)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // B
	mov		w11, w4 // ldb
	lsl		w11, w11, #3 // 8*ldb

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMM_NN_RL_4X4_LIB4C
#else
	CALL(inner_edge_trmm_nn_rl_4x4_lib4c)
#endif

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_nn_4x4_lib4c)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
	CALL(inner_scale_ab_4x4_lib4)
#endif



	// store n
	mov		x8, x7 // D
	ldr		w9, [sp, #(STACKSIZE + 0)] // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_TRAN_4X4_LIB
#else
	CALL(inner_tran_4x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
	CALL(inner_store_4x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrmm_nn_rl_4x4_tran_lib4c4c)





//                                             w0        x1             x2         x3         w4       x5            x6         x7         sp+0     sp+8    sp+16
// void kernel_dtrmm_nn_rl_4x4_tran_vs_lib4c4c(int kmax, double *alpha, double *A, double *B, int ldb, double *beta, double *C, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dtrmm_nn_rl_4x4_tran_vs_lib4c4c)
	FUN_START(kernel_dtrmm_nn_rl_4x4_tran_vs_lib4c4c)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		x10, x3 // B
	mov		w11, w4 // ldb
	lsl		w11, w11, #3 // 8*ldb

#if MACRO_LEVEL>=1
	INNER_EDGE_TRMM_NN_RL_4X4_VS_LIB4C
#else
	CALL(inner_edge_trmm_nn_rl_4x4_vs_lib4c)
#endif

	ldr		w12, [sp, #(STACKSIZE + 8)] // m1
	cmp		w12, #1
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X1_LIB4C
#else
	CALL(inner_kernel_gemm_nn_4x1_lib4c)
#endif

	b		103f

100:

	ldr		w12, [sp, #(STACKSIZE + 8)] // m1
	cmp		w12, #2
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X2_LIB4C
#else
	CALL(inner_kernel_gemm_nn_4x2_lib4c)
#endif
	
	b		103f

101:

	ldr		w12, [sp, #(STACKSIZE + 8)] // m1
	cmp		w12, #3
	bgt		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X3_LIB4C
#else
	CALL(inner_kernel_gemm_nn_4x3_lib4c)
#endif
	
	b		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_nn_4x4_lib4c)
#endif

103:



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_4X4_LIB4
#else
	CALL(inner_scale_ab_4x4_lib4)
#endif



	// store n
	mov		x8, x7 // D
	ldr		w9, [sp, #(STACKSIZE + 0)] // ldd
	lsl		w9, w9, #3 // 8*ldd
	ldr		w10, [sp, #(STACKSIZE + 8)] // m1
	ldr		w11, [sp, #(STACKSIZE + 16)] // n1

#if MACRO_LEVEL>=1
	INNER_TRAN_4X4_LIB
#else
	CALL(inner_tran_4x4_lib)
#endif

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB
#else
	CALL(inner_store_4x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrmm_nn_rl_4x4_tran_vs_lib4c4c)





//                                          w0        x1         x2         x3            x4         w5       x6         w7       sp+0       sp+8     sp+16
// void kernel_dtrsm_nt_rl_inv_4x4_lib44ccc(int kmax, double *A, double *B, double *beta, double *C, int ldc, double *D, int ldd, double *E, int lde, double *inv_diag_E)

	.align	4
	GLOB(kernel_dtrsm_nt_rl_inv_4x4_lib44ccc)
	FUN_START(kernel_dtrsm_nt_rl_inv_4x4_lib44ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		x10, x2 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x3 // beta
	mov		x9, x4 // C
	mov		w10, w5 // ldc
	lsl		w10, w10, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_LIB
#else
	CALL(inner_scale_m1b_4x4_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 0)] // E
	ldr		w9, [sp, #(STACKSIZE + 8)] // sde
	lsl		w9, w9, #3 // 8*ldc
	ldr		x10, [sp, #(STACKSIZE + 16)] // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_INV_4X4_LIB
#else
	CALL(inner_edge_trsm_rlt_inv_4x4_lib)
#endif



	// store l
	mov		x8, x6 // D
	mov		w9, w7 // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
	CALL(inner_store_4x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_4x4_lib44ccc)





//                                             w0        x1         x2         x3            x4         w5       x6         w7       sp+0       sp+8     sp+16               sp+24   sp+32
// void kernel_dtrsm_nt_rl_inv_4x4_vs_lib44ccc(int kmax, double *A, double *B, double *beta, double *C, int ldc, double *D, int ldd, double *E, int lde, double *inv_diag_E, int m1, int n1)

	.align	4
	GLOB(kernel_dtrsm_nt_rl_inv_4x4_vs_lib44ccc)
	FUN_START(kernel_dtrsm_nt_rl_inv_4x4_vs_lib44ccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		x10, x2 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x3 // beta
	mov		x9, x4 // C
	mov		w10, w5 // ldc
	lsl		w10, w10, #3 // 8*ldc
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
	lsl		w9, w9, #3 // 8*ldc
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
	lsl		w9, w9, #3 // 8*ldd
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

	FUN_END(kernel_dtrsm_nt_rl_inv_4x4_vs_lib44ccc)





//                                          w0        x1         x2         x3            x4         w5       x6         w7       sp+0       sp+8
// void kernel_dtrsm_nt_rl_inv_4x4_lib44cc4(int kmax, double *A, double *B, double *beta, double *C, int ldc, double *D, int ldd, double *E, double *inv_diag_E)

	.align	4
	GLOB(kernel_dtrsm_nt_rl_inv_4x4_lib44cc4)
	FUN_START(kernel_dtrsm_nt_rl_inv_4x4_lib44cc4)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		x10, x2 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x3 // beta
	mov		x9, x4 // C
	mov		w10, w5 // ldc
	lsl		w10, w10, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_LIB
#else
	CALL(inner_scale_m1b_4x4_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 0)] // E
	ldr		x9, [sp, #(STACKSIZE + 8)] // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_INV_4X4_LIB4
#else
	CALL(inner_edge_trsm_rlt_inv_4x4_lib4)
#endif



	// store l
	mov		x8, x6 // D
	mov		w9, w7 // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
	CALL(inner_store_4x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_4x4_lib44cc4)





//                                             w0        x1         x2         x3            x4         w5       x6         w7       sp+0       sp+8                sp+16   sp+24
// void kernel_dtrsm_nt_rl_inv_4x4_vs_lib44cc4(int kmax, double *A, double *B, double *beta, double *C, int ldc, double *D, int ldd, double *E, double *inv_diag_E, int m1, int n1)

	.align	4
	GLOB(kernel_dtrsm_nt_rl_inv_4x4_vs_lib44cc4)
	FUN_START(kernel_dtrsm_nt_rl_inv_4x4_vs_lib44cc4)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		x10, x2 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x3 // beta
	mov		x9, x4 // C
	mov		w10, w5 // ldc
	lsl		w10, w10, #3 // 8*ldc
	ldr		w11, [sp, #(STACKSIZE + 16)] // m1
	ldr		w12, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_VS_LIB
#else
	CALL(inner_scale_m1b_4x4_vs_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 0)] // E
	ldr		x9, [sp, #(STACKSIZE + 8)] // inv_diag_E
	ldr		w10, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_INV_4X4_VS_LIB4
#else
	CALL(inner_edge_trsm_rlt_inv_4x4_vs_lib4)
#endif



	// store l
	mov		x8, x6 // D
	mov		w9, w7 // ldd
	lsl		w9, w9, #3 // 8*ldd
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

	FUN_END(kernel_dtrsm_nt_rl_inv_4x4_vs_lib44cc4)





//                                          w0        x1         x2         w3       x4            x5         w6       x7         sp+0      sp+8      sp+16     sp+24
// void kernel_dtrsm_nt_rl_inv_4x4_lib4cccc(int kmax, double *A, double *B, int ldb, dobule *beta, double *C, int ldc, double *D, int ldd, double *E, int lde, double *inv_diag_E)

	.align	4
	GLOB(kernel_dtrsm_nt_rl_inv_4x4_lib4cccc)
	FUN_START(kernel_dtrsm_nt_rl_inv_4x4_lib4cccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		x10, x2 // B
	mov		w11, w3 // ldb
	lsl		w11, w11, #3 // 8*ldb

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4c)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x4 // beta
	mov		x9, x5 // C
	mov		w10, w6 // ldc
	lsl		w10, w10, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_LIB
#else
	CALL(inner_scale_m1b_4x4_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 8)] // E
	ldr		w9, [sp, #(STACKSIZE + 16)] // sde
	lsl		w9, w9, #3 // 8*ldc
	ldr		x10, [sp, #(STACKSIZE + 24)] // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_INV_4X4_LIB
#else
	CALL(inner_edge_trsm_rlt_inv_4x4_lib)
#endif



	// store l
	mov		x8, x7 // D
	ldr		w9, [sp, #(STACKSIZE + 0)] // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
	CALL(inner_store_4x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_4x4_lib4cccc)





//                                             w0        x1         x2         w3       x4            x5         w6       x7         sp+0      sp+8      sp+16     sp+24              sp+32   sp+40
// void kernel_dtrsm_nt_rl_inv_4x4_vs_lib4cccc(int kmax, double *A, double *B, int ldb, dobule *beta, double *C, int ldc, double *D, int ldd, double *E, int lde, double *inv_diag_E, int m1, int n1)

	.align	4
	GLOB(kernel_dtrsm_nt_rl_inv_4x4_vs_lib4cccc)
	FUN_START(kernel_dtrsm_nt_rl_inv_4x4_vs_lib4cccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		x10, x2 // B
	mov		w11, w3 // ldb
	lsl		w11, w11, #3 // 8*ldb

	ldr		w12, [sp, #(STACKSIZE + 40)] // n1
	cmp		w12, #1
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X1_LIB4C
#else
	CALL(inner_kernel_gemm_nt_4x1_lib4c)
#endif

	b		103f

100:

	ldr		w12, [sp, #(STACKSIZE + 40)] // n1
	cmp		w12, #2
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X2_LIB4C
#else
	CALL(inner_kernel_gemm_nt_4x2_lib4c)
#endif
	
	b		103f

101:

	ldr		w12, [sp, #(STACKSIZE + 40)] // n1
	cmp		w12, #3
	bgt		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X3_LIB4C
#else
	CALL(inner_kernel_gemm_nt_4x3_lib4c)
#endif
	
	b		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4c)
#endif

103:



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x4 // beta
	mov		x9, x5 // C
	mov		w10, w6 // ldc
	lsl		w10, w10, #3 // 8*ldc
	ldr		w11, [sp, #(STACKSIZE + 32)] // m1
	ldr		w12, [sp, #(STACKSIZE + 40)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_VS_LIB
#else
	CALL(inner_scale_m1b_4x4_vs_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 8)] // E
	ldr		w9, [sp, #(STACKSIZE + 16)] // sde
	lsl		w9, w9, #3 // 8*ldc
	ldr		x10, [sp, #(STACKSIZE + 24)] // inv_diag_E
	ldr		w11, [sp, #(STACKSIZE + 40)] // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_INV_4X4_VS_LIB
#else
	CALL(inner_edge_trsm_rlt_inv_4x4_vs_lib)
#endif



	// store l
	mov		x8, x7 // D
	ldr		w9, [sp, #(STACKSIZE + 0)] // ldd
	lsl		w9, w9, #3 // 8*ldd
	ldr		w10, [sp, #(STACKSIZE + 32)] // m1
	ldr		w11, [sp, #(STACKSIZE + 40)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB
#else
	CALL(inner_store_4x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrsm_nt_rl_inv_4x4_vs_lib4cccc)





//                                         w0        x1         x2         x3            x4         w5       x6         w7       sp+0
// void kernel_dtrsm_nt_rl_one_4x4_lib44cc4(int kmax, double *A, double *B, double *beta, double *C, int ldc, double *D, int ldd, double *E)

	.align	4
	GLOB(kernel_dtrsm_nt_rl_one_4x4_lib44cc4)
	FUN_START(kernel_dtrsm_nt_rl_one_4x4_lib44cc4)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		x10, x2 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4)
#endif



	// call inner blend for alpha=1.0
	mov		x8, x3 // beta
	mov		x9, x4 // C
	mov		w10, w5 // ldc
	lsl		w10, w10, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_LIB
#else
	CALL(inner_scale_m1b_4x4_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 0)] // E

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_ONE_4X4_LIB4
#else
	CALL(inner_edge_trsm_rlt_one_4x4_lib4)
#endif



	// store l
	mov		x8, x6 // D
	mov		w9, w7 // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
	CALL(inner_store_4x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrsm_nt_rl_one_4x4_lib44cc4)





//                                            w0        x1         x2         x3            x4         w5       x6         w7       sp+0       sp+8    sp+16
// void kernel_dtrsm_nt_rl_one_4x4_vs_lib44cc4(int kmax, double *A, double *B, double *beta, double *C, int ldc, double *D, int ldd, double *E, int m1, int n1)

	.align	4
	GLOB(kernel_dtrsm_nt_rl_one_4x4_vs_lib44cc4)
	FUN_START(kernel_dtrsm_nt_rl_one_4x4_vs_lib44cc4)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		x10, x2 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4)
#endif



	// call inner blend for alpha=1.0
	mov		x8, x3 // beta
	mov		x9, x4 // C
	mov		w10, w5 // ldc
	lsl		w10, w10, #3 // 8*ldc
	ldr		w11, [sp, #(STACKSIZE + 8)] // m1
	ldr		w12, [sp, #(STACKSIZE + 16)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_VS_LIB
#else
	CALL(inner_scale_m1b_4x4_vs_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 0)] // E
	ldr		w9, [sp, #(STACKSIZE + 16)] // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLT_ONE_4X4_VS_LIB4
#else
	CALL(inner_edge_trsm_rlt_one_4x4_vs_lib4)
#endif



	// store l
	mov		x8, x6 // D
	mov		w9, w7 // ldd
	lsl		w9, w9, #3 // 8*ldd
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

	FUN_END(kernel_dtrsm_nt_rl_one_4x4_vs_lib44cc4)





//                                          w0        x1         x2         x3            x4         w5       x6         w7       sp+0       sp+8
// void kernel_dtrsm_nt_ru_inv_4x4_lib44cc4(int kmax, double *A, double *B, double *beta, double *C, int ldc, double *D, int ldd, double *E, double *inv_diag_E)

	.align	4
	GLOB(kernel_dtrsm_nt_ru_inv_4x4_lib44cc4)
	FUN_START(kernel_dtrsm_nt_ru_inv_4x4_lib44cc4)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		x10, x2 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x3 // beta
	mov		x9, x4 // C
	mov		w10, w5 // ldc
	lsl		w10, w10, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_LIB
#else
	CALL(inner_scale_m1b_4x4_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 0)] // E
	ldr		x9, [sp, #(STACKSIZE + 8)] // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RUT_INV_4X4_LIB4
#else
	CALL(inner_edge_trsm_rut_inv_4x4_lib4)
#endif



	// store l
	mov		x8, x6 // D
	mov		w9, w7 // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
	CALL(inner_store_4x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrsm_nt_ru_inv_4x4_lib44cc4)





//                                             w0        x1         x2         x3            x4         w5       x6         w7       sp+0       sp+8                sp+16   sp+24
// void kernel_dtrsm_nt_rl_inv_4x4_vs_lib44cc4(int kmax, double *A, double *B, double *beta, double *C, int ldc, double *D, int ldd, double *E, double *inv_diag_E, int m1, int n1)

	.align	4
	GLOB(kernel_dtrsm_nt_ru_inv_4x4_vs_lib44cc4)
	FUN_START(kernel_dtrsm_nt_ru_inv_4x4_vs_lib44cc4)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		x10, x2 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x3 // beta
	mov		x9, x4 // C
	mov		w10, w5 // ldc
	lsl		w10, w10, #3 // 8*ldc
	ldr		w11, [sp, #(STACKSIZE + 16)] // m1
	ldr		w12, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_VS_LIB
#else
	CALL(inner_scale_m1b_4x4_vs_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 0)] // E
	ldr		x9, [sp, #(STACKSIZE + 8)] // inv_diag_E
	ldr		w10, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RUT_INV_4X4_VS_LIB4
#else
	CALL(inner_edge_trsm_rut_inv_4x4_vs_lib4)
#endif



	// store l
	mov		x8, x6 // D
	mov		w9, w7 // ldd
	lsl		w9, w9, #3 // 8*ldd
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

	FUN_END(kernel_dtrsm_nt_ru_inv_4x4_vs_lib44cc4)





//                                    w0        x1         x2         x3         w4       x5         w6       x7
// void kernel_dpotrf_nt_l_4x4_lib44cc(int kmax, double *A, double *B, double *C, int ldc, double *D, int ldd, double *inv_diag_D)

	.align	4
	GLOB(kernel_dpotrf_nt_l_4x4_lib44cc)
	FUN_START(kernel_dpotrf_nt_l_4x4_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		x10, x2 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x3 // C
	mov		w9, w4 // ldc
	lsl		w9, w9, #3 // 8*ldc

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
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_L_4X4_LIB
#else
	CALL(inner_store_l_4x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dpotrf_nt_l_4x4_lib44cc)





//                                       w0        x1         x2         x3         w4       x5         w6       x7                  sp+0    sp+1
// void kernel_dpotrf_nt_l_4x4_vs_lib44cc(int kmax, double *A, double *B, double *C, int ldc, double *D, int ldd, double *inv_diag_D, int m1, int n1)

	.align	4
	GLOB(kernel_dpotrf_nt_l_4x4_vs_lib44cc)
	FUN_START(kernel_dpotrf_nt_l_4x4_vs_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		x10, x2 // B

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x3 // C
	mov		w9, w4 // ldc
	lsl		w9, w9, #3 // 8*ldc
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
	lsl		w9, w9, #3 // 8*ldd
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

	FUN_END(kernel_dpotrf_nt_l_4x4_vs_lib44cc)





//                                         w0        x1         x2         x3       x4            x5         w6       x7         sp+0     sp+8       sp+16
// void kernel_dtrsm_nn_ll_one_4x4_lib4cccc(int kmax, double *A, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, double *E, int lde)

	.align	4
	GLOB(kernel_dtrsm_nn_ll_one_4x4_lib4cccc)
	FUN_START(kernel_dtrsm_nn_ll_one_4x4_lib4cccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		x10, x2 // B
	mov		w11, w3 // ldb
	lsl		w11, w11, #3 // 8*ldb

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_nn_4x4_lib4c)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x4 // beta
	mov		x9, x5 // C
	mov		w10, w6 // ldc
	lsl		w10, w10, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_LIB
#else
	CALL(inner_scale_m1b_4x4_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 8)] // E
	ldr		w9, [sp, #(STACKSIZE + 16)] // sde
	lsl		w9, w9, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_LLN_ONE_4X4_LIBC
#else
	CALL(inner_edge_trsm_lln_one_4x4_libc)
#endif



	// store l
	mov		x8, x7 // D
	ldr		w9, [sp, #(STACKSIZE + 0)] // sdd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
	CALL(inner_store_4x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrsm_nn_ll_one_4x4_lib4cccc)






//                                            w0        x1         x2         x3       x4            x5         w6       x7         sp+0     sp+8       sp+16    sp+24   sp+32
// void kernel_dtrsm_nn_ll_one_4x4_vs_lib4cccc(int kmax, double *A, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd, double *E, int lde, int m1, int n1)

	.align	4
	GLOB(kernel_dtrsm_nn_ll_one_4x4_vs_lib4cccc)
	FUN_START(kernel_dtrsm_nn_ll_one_4x4_vs_lib4cccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		x10, x2 // B
	mov		w11, w3 // ldb
	lsl		w11, w11, #3 // 8*ldb

	ldr		w12, [sp, #(STACKSIZE + 32)] // n1
	cmp		w12, #1
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X1_LIB4C
#else
	CALL(inner_kernel_gemm_nn_4x1_lib4c)
#endif

	b		103f

100:

	ldr		w12, [sp, #(STACKSIZE + 32)] // n1
	cmp		w12, #2
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X2_LIB4C
#else
	CALL(inner_kernel_gemm_nn_4x2_lib4c)
#endif
	
	b		103f

101:

	ldr		w12, [sp, #(STACKSIZE + 32)] // n1
	cmp		w12, #3
	bgt		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X3_LIB4C
#else
	CALL(inner_kernel_gemm_nn_4x3_lib4c)
#endif
	
	b		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_nn_4x4_lib4c)
#endif

103:



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x4 // beta
	mov		x9, x5 // C
	mov		w10, w6 // ldc
	lsl		w10, w10, #3 // 8*ldc
	ldr		w11, [sp, #(STACKSIZE + 24)] // m1
	ldr		w12, [sp, #(STACKSIZE + 32)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_VS_LIB
#else
	CALL(inner_scale_m1b_4x4_vs_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 8)] // E
	ldr		w9, [sp, #(STACKSIZE + 16)] // sde
	lsl		w9, w9, #3 // 8*ldc
	ldr		w10, [sp, #(STACKSIZE + 24)] // m1

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_LLN_ONE_4X4_VS_LIBC
#else
	CALL(inner_edge_trsm_lln_one_4x4_vs_libc)
#endif



	// store l
	mov		x8, x7 // D
	ldr		w9, [sp, #(STACKSIZE + 0)] // sdd
	lsl		w9, w9, #3 // 8*ldd
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

	FUN_END(kernel_dtrsm_nn_ll_one_4x4_vs_lib4cccc)





//                                          w0        x1         x2         w3       x4            x5         w6       x7         sp+0      sp+8      sp+16     sp+24
// void kernel_dtrsm_nn_rl_inv_4x4_lib4cccc(int kmax, double *A, double *B, int ldb, dobule *beta, double *C, int ldc, double *D, int ldd, double *E, int lde, double *inv_diag_E)

	.align	4
	GLOB(kernel_dtrsm_nn_rl_inv_4x4_lib4cccc)
	FUN_START(kernel_dtrsm_nn_rl_inv_4x4_lib4cccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		x10, x2 // B
	mov		w11, w3 // ldb
	lsl		w11, w11, #3 // 8*ldb

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_nn_4x4_lib4c)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x4 // beta
	mov		x9, x5 // C
	mov		w10, w6 // ldc
	lsl		w10, w10, #3 // 8*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_LIB
#else
	CALL(inner_scale_m1b_4x4_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 8)] // E
	ldr		w9, [sp, #(STACKSIZE + 16)] // sde
	lsl		w9, w9, #3 // 8*ldc
	ldr		x10, [sp, #(STACKSIZE + 24)] // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLN_INV_4X4_LIB
#else
	CALL(inner_edge_trsm_rln_inv_4x4_lib)
#endif



	// store l
	mov		x8, x7 // D
	ldr		w9, [sp, #(STACKSIZE + 0)] // ldd
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
	CALL(inner_store_4x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrsm_nn_rl_inv_4x4_lib4cccc)





//                                             w0        x1         x2         w3       x4            x5         w6       x7         sp+0      sp+8      sp+16     sp+24              sp+32   sp+40
// void kernel_dtrsm_nn_rl_inv_4x4_vs_lib4cccc(int kmax, double *A, double *B, int ldb, dobule *beta, double *C, int ldc, double *D, int ldd, double *E, int lde, double *inv_diag_E, int m1, int n1)

	.align	4
	GLOB(kernel_dtrsm_nn_rl_inv_4x4_vs_lib4cccc)
	FUN_START(kernel_dtrsm_nn_rl_inv_4x4_vs_lib4cccc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		x10, x2 // B
	mov		w11, w3 // ldb
	lsl		w11, w11, #3 // 8*ldb

	ldr		w12, [sp, #(STACKSIZE + 40)] // n1
	cmp		w12, #1
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X1_LIB4C
#else
	CALL(inner_kernel_gemm_nn_4x1_lib4c)
#endif

	b		103f

100:

	ldr		w12, [sp, #(STACKSIZE + 40)] // n1
	cmp		w12, #2
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X2_LIB4C
#else
	CALL(inner_kernel_gemm_nn_4x2_lib4c)
#endif
	
	b		103f

101:

	ldr		w12, [sp, #(STACKSIZE + 40)] // n1
	cmp		w12, #3
	bgt		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X3_LIB4C
#else
	CALL(inner_kernel_gemm_nn_4x3_lib4c)
#endif
	
	b		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_nn_4x4_lib4c)
#endif

103:



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x4 // beta
	mov		x9, x5 // C
	mov		w10, w6 // ldc
	lsl		w10, w10, #3 // 8*ldc
	ldr		w11, [sp, #(STACKSIZE + 32)] // m1
	ldr		w12, [sp, #(STACKSIZE + 40)] // n1

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_VS_LIB
#else
	CALL(inner_scale_m1b_4x4_vs_lib)
#endif



	// solution
	ldr		x8, [sp, #(STACKSIZE + 8)] // E
	ldr		w9, [sp, #(STACKSIZE + 16)] // sde
	lsl		w9, w9, #3 // 8*ldc
	ldr		x10, [sp, #(STACKSIZE + 24)] // inv_diag_E
	ldr		w11, [sp, #(STACKSIZE + 40)] // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RLN_INV_4X4_VS_LIB
#else
	CALL(inner_edge_trsm_rln_inv_4x4_vs_lib)
#endif



	// store l
	mov		x8, x7 // D
	ldr		w9, [sp, #(STACKSIZE + 0)] // ldd
	lsl		w9, w9, #3 // 8*ldd
	ldr		w10, [sp, #(STACKSIZE + 32)] // m1
	ldr		w11, [sp, #(STACKSIZE + 40)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB
#else
	CALL(inner_store_4x4_vs_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrsm_nn_rl_inv_4x4_vs_lib4cccc)





//                                          w0        x1         x2         w3       x4            x5         x6         x7         sp+0     sp+8
// void kernel_dtrsm_nn_ru_inv_4x4_lib4c44c(int kmax, double *A, double *B, int ldb, double *beta, double *C, double *D, double *E, int lde, double *inv_diag_E)

	.align	4
	GLOB(kernel_dtrsm_nn_ru_inv_4x4_lib4c44c)
	FUN_START(kernel_dtrsm_nn_ru_inv_4x4_lib4c44c)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		x10, x2 // B
	mov		w11, w3 // ldb
	lsl		w11, w11, #3 // 8*ldb

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_nn_4x4_lib4c)
#endif



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x4 // beta
	mov		x9, x5 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_LIB4
#else
	CALL(inner_scale_m1b_4x4_lib4)
#endif



	// solution
	mov		x8, x7 // E
	ldr		w9, [sp, #(STACKSIZE + 0)] // sde
	lsl		w9, w9, #3 // 8*ldc
	ldr		x10, [sp, #(STACKSIZE + 8)] // inv_diag_E

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RUN_INV_4X4_LIB
#else
	CALL(inner_edge_trsm_run_inv_4x4_lib)
#endif



	// store
	mov		x8, x6 // D

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB4
#else
	CALL(inner_store_4x4_lib4)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrsm_nn_ru_inv_4x4_lib4c44c)






//                                             w0        x1         x2         w3       x4            x5         x6         x7         sp+0     sp+8                sp+16   sp+24
// void kernel_dtrsm_nn_ru_inv_4x4_vs_lib4c44c(int kmax, double *A, double *B, int ldb, double *beta, double *C, double *D, double *E, int lde, double *inv_diag_E, int m1, int n1)

	.align	4
	GLOB(kernel_dtrsm_nn_ru_inv_4x4_vs_lib4c44c)
	FUN_START(kernel_dtrsm_nn_ru_inv_4x4_vs_lib4c44c)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x1 // A
	mov		x10, x2 // B
	mov		w11, w3 // ldb
	lsl		w11, w11, #3 // 8*ldb

	ldr		w12, [sp, #(STACKSIZE + 24)] // n1
	cmp		w12, #1
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X1_LIB4C
#else
	CALL(inner_kernel_gemm_nn_4x1_lib4c)
#endif

	b		103f

100:

	ldr		w12, [sp, #(STACKSIZE + 24)] // n1
	cmp		w12, #2
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X2_LIB4C
#else
	CALL(inner_kernel_gemm_nn_4x2_lib4c)
#endif
	
	b		103f

101:

	ldr		w12, [sp, #(STACKSIZE + 24)] // n1
	cmp		w12, #3
	bgt		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X3_LIB4C
#else
	CALL(inner_kernel_gemm_nn_4x3_lib4c)
#endif
	
	b		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_nn_4x4_lib4c)
#endif

103:



	// call inner blend for alpha=1.0 and beta=1.0
	mov		x8, x4 // beta
	mov		x9, x5 // C

#if MACRO_LEVEL>=1
	INNER_SCALE_M1B_4X4_LIB4
#else
	CALL(inner_scale_m1b_4x4_lib4)
#endif



	// solution
	mov		x8, x7 // E
	ldr		w9, [sp, #(STACKSIZE + 0)] // sde
	lsl		w9, w9, #3 // 8*ldc
	ldr		x10, [sp, #(STACKSIZE + 8)] // inv_diag_E
	ldr		w11, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_EDGE_TRSM_RUN_INV_4X4_VS_LIB
#else
	CALL(inner_edge_trsm_run_inv_4x4_vs_lib)
#endif



	// store
	mov		x8, x6 // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // m1
	ldr		w10, [sp, #(STACKSIZE + 24)] // n1

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_VS_LIB4
#else
	CALL(inner_store_4x4_vs_lib4)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dtrsm_nn_ru_inv_4x4_vs_lib4c44c)





//                                 w0        x1             x2         x3         x4       x5            w6         x7       sp+0       sp+8
// void kernel_dgemm_nt_4x4_libc4cc(int kmax, double *alpha, double *A, int lda, double *B, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dgemm_nt_4x4_libc4cc)
	FUN_START(kernel_dgemm_nt_4x4_libc4cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x4 // B
	mov		x10, x2 // A
	mov		w11, w3 // lda
	lsl		w11, w11, #3 // 8*lda

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4c)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // ldc
	lsl		w11, w11, #3 // 8*ldc

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
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
	CALL(inner_store_4x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_nt_4x4_libc4cc)





//                                    w0        x1             x2         x3         x4       x5            w6         x7       sp+0       sp+8     sp+16   sp+24
// void kernel_dgemm_nt_4x4_vs_libc4cc(int kmax, double *alpha, double *A, int lda, double *B, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dgemm_nt_4x4_vs_libc4cc)
	FUN_START(kernel_dgemm_nt_4x4_vs_libc4cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x4 // B
	mov		x10, x2 // A
	mov		w11, w3 // lda
	lsl		w11, w11, #3 // 8*ldb

	ldr		w12, [sp, #(STACKSIZE + 16)] // m1
	cmp		w12, #1
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X1_LIB4C
#else
	CALL(inner_kernel_gemm_nt_4x1_lib4c)
#endif
	
	b		103f

100:

	ldr		w12, [sp, #(STACKSIZE + 16)] // m1
	cmp		w12, #2
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X2_LIB4C
#else
	CALL(inner_kernel_gemm_nt_4x2_lib4c)
#endif
	
	b		103f

101:

	ldr		w12, [sp, #(STACKSIZE + 16)] // m1
	cmp		w12, #3
	bgt		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X3_LIB4C
#else
	CALL(inner_kernel_gemm_nt_4x3_lib4c)
#endif
	
	b		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NT_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_nt_4x4_lib4c)
#endif

103:



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // ldc
	lsl		w11, w11, #3 // 8*ldc
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
	lsl		w9, w9, #3 // 8*ldd
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

	FUN_END(kernel_dgemm_nt_4x4_vs_libc4cc)





//                                 w0        x1             x2         x3         x4       x5            w6         x7       sp+0       sp+8
// void kernel_dgemm_tt_4x4_libc4cc(int kmax, double *alpha, double *A, int lda, double *B, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_dgemm_tt_4x4_libc4cc)
	FUN_START(kernel_dgemm_tt_4x4_libc4cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x4 // B
	mov		x10, x2 // A
	mov		w11, w3 // lda
	lsl		w11, w11, #3 // 8*lda

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_nn_4x4_lib4c)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // ldc
	lsl		w11, w11, #3 // 8*ldc

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
	lsl		w9, w9, #3 // 8*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_4X4_LIB
#else
	CALL(inner_store_4x4_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_dgemm_tt_4x4_libc4cc)





//                                    w0        x1             x2         x3         x4       x5            w6         x7       sp+0       sp+8     sp+16   sp+24
// void kernel_dgemm_tt_4x4_vs_libc4cc(int kmax, double *alpha, double *A, int lda, double *B, double *beta, double *C, int ldc, double *D, int ldd, int m1, int n1)

	.align	4
	GLOB(kernel_dgemm_tt_4x4_vs_libc4cc)
	FUN_START(kernel_dgemm_tt_4x4_vs_libc4cc)
	


	PROLOGUE



	ZERO_ACC



	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x4 // B
	mov		x10, x2 // A
	mov		w11, w3 // lda
	lsl		w11, w11, #3 // 8*ldb

	ldr		w12, [sp, #(STACKSIZE + 16)] // m1
	cmp		w12, #1
	bgt		100f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X1_LIB4C
#else
	CALL(inner_kernel_gemm_nn_4x1_lib4c)
#endif
	
	b		103f

100:

	ldr		w12, [sp, #(STACKSIZE + 16)] // m1
	cmp		w12, #2
	bgt		101f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X2_LIB4C
#else
	CALL(inner_kernel_gemm_nn_4x2_lib4c)
#endif
	
	b		103f

101:

	ldr		w12, [sp, #(STACKSIZE + 16)] // m1
	cmp		w12, #3
	bgt		102f

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X3_LIB4C
#else
	CALL(inner_kernel_gemm_nn_4x3_lib4c)
#endif
	
	b		103f

102:

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_NN_4X4_LIB4C
#else
	CALL(inner_kernel_gemm_nn_4x4_lib4c)
#endif

103:



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x5 // beta
	mov		x10, x6 // C
	mov		w11, w7 // ldc
	lsl		w11, w11, #3 // 8*ldc
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
	lsl		w9, w9, #3 // 8*ldd
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

	FUN_END(kernel_dgemm_tt_4x4_vs_libc4cc)







