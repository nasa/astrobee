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
// x11  <- B
// x12  <- ldb
//
// output arguments:

#if MACRO_LEVEL>=2
	.macro INNER_KERNEL_GEMM_ADD_NT_8X8_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nt_8x8_lib4c)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x19, x9, x10 // A1
	add		x20, x11, #56 // B1

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
	prfm	PLDL1KEEP, [x20]
	prfm	PLDL1KEEP, [x11, x12]
	prfm	PLDL1KEEP, [x20, x12]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x20, x13]
	prfm	PLDL1KEEP, [x11, x14]
	prfm	PLDL1KEEP, [x20, x14]

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x19, #64]
	prfm	PLDL1KEEP, [x11, x15]
	prfm	PLDL1KEEP, [x20, x15]
	prfm	PLDL1KEEP, [x11, x16]
	prfm	PLDL1KEEP, [x20, x16]
	prfm	PLDL1KEEP, [x11, x17]
	prfm	PLDL1KEEP, [x20, x17]
	prfm	PLDL1KEEP, [x11, x18]
	prfm	PLDL1KEEP, [x20, x18]

	// main loop
1:

	ldp		q28, q29, [x11]
	add		x11, x11, x12
	ldp		q16, q17, [x11]
	add		x11, x11, x12
	ldp		q24, q25, [x9, #(0*16)]
	ldp		q20, q21, [x19, #(0*16)]

	ldp		q30, q31, [x11]
	add		x11, x11, x12
	ldp		q18, q19, [x11]
	add		x11, x11, x12
	ldp		q26, q27, [x9, #(2*16)]
	ldp		q22, q23, [x19, #(2*16)]

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
//	add		x11, x11, x15
	fmla	v1.4s, v24.4s, v28.s[1]
	add		x20, x20, x15
	fmla	v2.4s, v24.4s, v28.s[2]
	prfm	PLDL1KEEP, [x9, #128]
	fmla	v3.4s, v24.4s, v28.s[3]
	prfm	PLDL1KEEP, [x19, #128]
	fmla	v4.4s, v20.4s, v28.s[0]
	prfm	PLDL1KEEP, [x11, x15]
	fmla	v5.4s, v20.4s, v28.s[1]
	prfm	PLDL1KEEP, [x20, x15]
	fmla	v6.4s, v20.4s, v28.s[2]
	prfm	PLDL1KEEP, [x11, x16]
	fmla	v7.4s, v20.4s, v28.s[3]
	prfm	PLDL1KEEP, [x20, x16]
	fmla	v8.4s, v24.4s, v29.s[0]
	prfm	PLDL1KEEP, [x11, x17]
	fmla	v9.4s, v24.4s, v29.s[1]
	prfm	PLDL1KEEP, [x20, x17]
	fmla	v10.4s, v24.4s, v29.s[2]
	prfm	PLDL1KEEP, [x11, x18]
	fmla	v11.4s, v24.4s, v29.s[3]
	prfm	PLDL1KEEP, [x20, x18]
	fmla	v12.4s, v20.4s, v29.s[0]
	fmla	v13.4s, v20.4s, v29.s[1]
	fmla	v14.4s, v20.4s, v29.s[2]
	fmla	v15.4s, v20.4s, v29.s[3]

	// unroll 1
	fmla	v0.4s, v25.4s, v16.s[0]
	fmla	v1.4s, v25.4s, v16.s[1]
	add		x9, x9, #64
	fmla	v2.4s, v25.4s, v16.s[2]
	fmla	v3.4s, v25.4s, v16.s[3]
	add		x19, x19, #64
	fmla	v4.4s, v21.4s, v16.s[0]
	fmla	v5.4s, v21.4s, v16.s[1]
	fmla	v6.4s, v21.4s, v16.s[2]
	fmla	v7.4s, v21.4s, v16.s[3]
	fmla	v8.4s, v25.4s, v17.s[0]
	fmla	v9.4s, v25.4s, v17.s[1]
	fmla	v10.4s, v25.4s, v17.s[2]
	fmla	v11.4s, v25.4s, v17.s[3]
	fmla	v12.4s, v21.4s, v17.s[0]
	fmla	v13.4s, v21.4s, v17.s[1]
	fmla	v14.4s, v21.4s, v17.s[2]
	fmla	v15.4s, v21.4s, v17.s[3]

	// unroll 2
	fmla	v0.4s, v26.4s, v30.s[0]
	fmla	v1.4s, v26.4s, v30.s[1]
	fmla	v2.4s, v26.4s, v30.s[2]
	fmla	v3.4s, v26.4s, v30.s[3]
	fmla	v4.4s, v22.4s, v30.s[0]
	fmla	v5.4s, v22.4s, v30.s[1]
	fmla	v6.4s, v22.4s, v30.s[2]
	fmla	v7.4s, v22.4s, v30.s[3]
	fmla	v8.4s, v26.4s, v31.s[0]
	fmla	v9.4s, v26.4s, v31.s[1]
	fmla	v10.4s, v26.4s, v31.s[2]
	fmla	v11.4s, v26.4s, v31.s[3]
	fmla	v12.4s, v22.4s, v31.s[0]
	fmla	v13.4s, v22.4s, v31.s[1]
	fmla	v14.4s, v22.4s, v31.s[2]
	fmla	v15.4s, v22.4s, v31.s[3]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v18.s[0]
	fmla	v1.4s, v27.4s, v18.s[1]
	fmla	v2.4s, v27.4s, v18.s[2]
	fmla	v3.4s, v27.4s, v18.s[3]
	fmla	v4.4s, v23.4s, v18.s[0]
	fmla	v5.4s, v23.4s, v18.s[1]
	fmla	v6.4s, v23.4s, v18.s[2]
	fmla	v7.4s, v23.4s, v18.s[3]
	fmla	v8.4s, v27.4s, v19.s[0]
	fmla	v9.4s, v27.4s, v19.s[1]
	fmla	v10.4s, v27.4s, v19.s[2]
	fmla	v11.4s, v27.4s, v19.s[3]
	fmla	v12.4s, v23.4s, v19.s[0]
	fmla	v13.4s, v23.4s, v19.s[1]
	fmla	v14.4s, v23.4s, v19.s[2]
	fmla	v15.4s, v23.4s, v19.s[3]

	cmp		w8, #4
	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	ldp		q28, q29, [x11]
	add		x11, x11, x12
	ldp		q16, q17, [x11]
	add		x11, x11, x12
	ldp		q24, q25, [x9, #(0*16)]
	ldp		q20, q21, [x19, #(0*16)]

	ldp		q30, q31, [x11]
	add		x11, x11, x12
	ldp		q18, q19, [x11]
	add		x11, x11, x12
	ldp		q26, q27, [x9, #(2*16)]
	ldp		q22, q23, [x19, #(2*16)]

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
//	add		x11, x11, x15
	fmla	v1.4s, v24.4s, v28.s[1]
	add		x20, x20, x15
	fmla	v2.4s, v24.4s, v28.s[2]
//	prfm	PLDL1KEEP, [x9, #128]
	fmla	v3.4s, v24.4s, v28.s[3]
//	prfm	PLDL1KEEP, [x19, #128]
	fmla	v4.4s, v20.4s, v28.s[0]
//	prfm	PLDL1KEEP, [x11, x15]
	fmla	v5.4s, v20.4s, v28.s[1]
//	prfm	PLDL1KEEP, [x20, x15]
	fmla	v6.4s, v20.4s, v28.s[2]
//	prfm	PLDL1KEEP, [x11, x16]
	fmla	v7.4s, v20.4s, v28.s[3]
//	prfm	PLDL1KEEP, [x20, x16]
	fmla	v8.4s, v24.4s, v29.s[0]
//	prfm	PLDL1KEEP, [x11, x17]
	fmla	v9.4s, v24.4s, v29.s[1]
//	prfm	PLDL1KEEP, [x20, x17]
	fmla	v10.4s, v24.4s, v29.s[2]
//	prfm	PLDL1KEEP, [x11, x18]
	fmla	v11.4s, v24.4s, v29.s[3]
//	prfm	PLDL1KEEP, [x20, x18]
	fmla	v12.4s, v20.4s, v29.s[0]
	fmla	v13.4s, v20.4s, v29.s[1]
	fmla	v14.4s, v20.4s, v29.s[2]
	fmla	v15.4s, v20.4s, v29.s[3]

	// unroll 1
	fmla	v0.4s, v25.4s, v16.s[0]
	fmla	v1.4s, v25.4s, v16.s[1]
	add		x9, x9, #64
	fmla	v2.4s, v25.4s, v16.s[2]
	fmla	v3.4s, v25.4s, v16.s[3]
	add		x19, x19, #64
	fmla	v4.4s, v21.4s, v16.s[0]
	fmla	v5.4s, v21.4s, v16.s[1]
	fmla	v6.4s, v21.4s, v16.s[2]
	fmla	v7.4s, v21.4s, v16.s[3]
	fmla	v8.4s, v25.4s, v17.s[0]
	fmla	v9.4s, v25.4s, v17.s[1]
	fmla	v10.4s, v25.4s, v17.s[2]
	fmla	v11.4s, v25.4s, v17.s[3]
	fmla	v12.4s, v21.4s, v17.s[0]
	fmla	v13.4s, v21.4s, v17.s[1]
	fmla	v14.4s, v21.4s, v17.s[2]
	fmla	v15.4s, v21.4s, v17.s[3]

	// unroll 2
	fmla	v0.4s, v26.4s, v30.s[0]
	fmla	v1.4s, v26.4s, v30.s[1]
	fmla	v2.4s, v26.4s, v30.s[2]
	fmla	v3.4s, v26.4s, v30.s[3]
	fmla	v4.4s, v22.4s, v30.s[0]
	fmla	v5.4s, v22.4s, v30.s[1]
	fmla	v6.4s, v22.4s, v30.s[2]
	fmla	v7.4s, v22.4s, v30.s[3]
	fmla	v8.4s, v26.4s, v31.s[0]
	fmla	v9.4s, v26.4s, v31.s[1]
	fmla	v10.4s, v26.4s, v31.s[2]
	fmla	v11.4s, v26.4s, v31.s[3]
	fmla	v12.4s, v22.4s, v31.s[0]
	fmla	v13.4s, v22.4s, v31.s[1]
	fmla	v14.4s, v22.4s, v31.s[2]
	fmla	v15.4s, v22.4s, v31.s[3]
	sub		w8, w8, #4

	// unroll 3
	fmla	v0.4s, v27.4s, v18.s[0]
	fmla	v1.4s, v27.4s, v18.s[1]
	fmla	v2.4s, v27.4s, v18.s[2]
	fmla	v3.4s, v27.4s, v18.s[3]
	fmla	v4.4s, v23.4s, v18.s[0]
	fmla	v5.4s, v23.4s, v18.s[1]
	fmla	v6.4s, v23.4s, v18.s[2]
	fmla	v7.4s, v23.4s, v18.s[3]
	fmla	v8.4s, v27.4s, v19.s[0]
	fmla	v9.4s, v27.4s, v19.s[1]
	fmla	v10.4s, v27.4s, v19.s[2]
	fmla	v11.4s, v27.4s, v19.s[3]
	fmla	v12.4s, v23.4s, v19.s[0]
	fmla	v13.4s, v23.4s, v19.s[1]
	fmla	v14.4s, v23.4s, v19.s[2]
	fmla	v15.4s, v23.4s, v19.s[3]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x9, x9, #32
//	sub		x19, x19, #32
//	sub		x11, x11, #32
//	sub		x14, x14, #32

3: // clean1-up loop

	// unroll 0

	ldp		q28, q29, [x11]
	ld1		{v24.4s}, [x9], #16
	ld1		{v20.4s}, [x19], #16
	fmla	v0.4s, v24.4s, v28.s[0]
	add		x11, x11, x12
	fmla	v1.4s, v24.4s, v28.s[1]
	fmla	v2.4s, v24.4s, v28.s[2]
	fmla	v3.4s, v24.4s, v28.s[3]
	fmla	v4.4s, v20.4s, v28.s[0]
	fmla	v5.4s, v20.4s, v28.s[1]
	fmla	v6.4s, v20.4s, v28.s[2]
	fmla	v7.4s, v20.4s, v28.s[3]
	fmla	v8.4s, v24.4s, v29.s[0]
	fmla	v9.4s, v24.4s, v29.s[1]
	fmla	v10.4s, v24.4s, v29.s[2]
	fmla	v11.4s, v24.4s, v29.s[3]
	fmla	v12.4s, v20.4s, v29.s[0]
	fmla	v13.4s, v20.4s, v29.s[1]
	fmla	v14.4s, v20.4s, v29.s[2]
	fmla	v15.4s, v20.4s, v29.s[3]

	sub		w8, w8, #1
	cmp		w8, #0
	bgt		3b

2: // return



#else // cortex a53



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x19, x9, x10 // A1
//	add		x14, x11, x12 // B1
	add		x20, x11, #60 // B1

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
//	prfm	PLDL1KEEP, [x20, x13]
//	prfm	PLDL1KEEP, [x11, x14]
//	prfm	PLDL1KEEP, [x20, x14]

	// preload
	ldp		q28, q29, [x11]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x20, x13]
	add		x11, x11, x12
	add		x20, x20, x12

	ldp		q16, q17, [x11]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x20, x13]
	add		x11, x11, x12
	add		x20, x20, x12

	ldp		q30, q31, [x11]
	prfm	PLDL1KEEP, [x11, x13]
	prfm	PLDL1KEEP, [x20, x13]
	add		x11, x11, x12
	add		x20, x20, x12

//	ldp		q18, q19, [x11]
//	prfm	PLDL1KEEP, [x11, x15]
//	prfm	PLDL1KEEP, [x20, x15]
//	add		x11, x11, x12
//	add		x20, x20, x12

	ldp		q24, q25, [x9, #(0*16)]
	ldp		q20, q21, [x19, #(0*16)]

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
//	add		x11, x11, x12
	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x19, #64]
//	prfm	PLDL1KEEP, [x11, x15]
//	prfm	PLDL1KEEP, [x20, x15]
//	prfm	PLDL1KEEP, [x11, x16]
//	prfm	PLDL1KEEP, [x20, x16]
//	prfm	PLDL1KEEP, [x11, x17]
//	prfm	PLDL1KEEP, [x20, x17]
//	prfm	PLDL1KEEP, [x11, x18]
//	prfm	PLDL1KEEP, [x20, x18]
//	sub		x11, x11, x12

	// main loop
1:

	// unroll 0
	ldp		q18, q19, [x11]
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v28.s[1]
	prfm	PLDL1KEEP, [x11, x13]
	fmla	v2.4s, v24.4s, v28.s[2]
	prfm	PLDL1KEEP, [x20, x13]
	fmla	v3.4s, v24.4s, v28.s[3]
	add		x11, x11, x12
	fmla	v4.4s, v20.4s, v28.s[0]
	add		x20, x20, x12
	fmla	v5.4s, v20.4s, v28.s[1]
	fmla	v6.4s, v20.4s, v28.s[2]
	ldp		q26, q27, [x9, #(2*16)]
	fmla	v7.4s, v20.4s, v28.s[3]
	fmla	v8.4s, v24.4s, v29.s[0]
	ldp		q22, q23, [x19, #(2*16)]
	fmla	v9.4s, v24.4s, v29.s[1]
	fmla	v10.4s, v24.4s, v29.s[2]
	prfm	PLDL1KEEP, [x9, #128]
	fmla	v11.4s, v24.4s, v29.s[3]
	fmla	v12.4s, v20.4s, v29.s[0]
	prfm	PLDL1KEEP, [x19, #128]
	fmla	v13.4s, v20.4s, v29.s[1]
	fmla	v14.4s, v20.4s, v29.s[2]
	fmla	v15.4s, v20.4s, v29.s[3]

	// unroll 1
	ldp		q28, q29, [x11]
	fmla	v0.4s, v25.4s, v16.s[0]
	fmla	v1.4s, v25.4s, v16.s[1]
	prfm	PLDL1KEEP, [x11, x13]
	fmla	v2.4s, v25.4s, v16.s[2]
	prfm	PLDL1KEEP, [x20, x13]
	fmla	v3.4s, v25.4s, v16.s[3]
	add		x11, x11, x12
	fmla	v4.4s, v21.4s, v16.s[0]
	add		x20, x20, x12
	fmla	v5.4s, v21.4s, v16.s[1]
	fmla	v6.4s, v21.4s, v16.s[2]
	fmla	v7.4s, v21.4s, v16.s[3]
	fmla	v8.4s, v25.4s, v17.s[0]
	fmla	v9.4s, v25.4s, v17.s[1]
	fmla	v10.4s, v25.4s, v17.s[2]
	fmla	v11.4s, v25.4s, v17.s[3]
	add		x9, x9, #64
	fmla	v12.4s, v21.4s, v17.s[0]
	fmla	v13.4s, v21.4s, v17.s[1]
	add		x19, x19, #64
	fmla	v14.4s, v21.4s, v17.s[2]
	fmla	v15.4s, v21.4s, v17.s[3]

	// unroll 2
	ldp		q16, q17, [x11]
	fmla	v0.4s, v26.4s, v30.s[0]
	fmla	v1.4s, v26.4s, v30.s[1]
	prfm	PLDL1KEEP, [x11, x13]
	fmla	v2.4s, v26.4s, v30.s[2]
	prfm	PLDL1KEEP, [x20, x13]
	fmla	v3.4s, v26.4s, v30.s[3]
	add		x11, x11, x12
	fmla	v4.4s, v22.4s, v30.s[0]
	add		x20, x20, x12
	fmla	v5.4s, v22.4s, v30.s[1]
	fmla	v6.4s, v22.4s, v30.s[2]
	ldp		q24, q25, [x9, #(0*16)]
	fmla	v7.4s, v22.4s, v30.s[3]
	fmla	v8.4s, v26.4s, v31.s[0]
	ldp		q20, q21, [x19, #(0*16)]
	fmla	v9.4s, v26.4s, v31.s[1]
	fmla	v10.4s, v26.4s, v31.s[2]
	fmla	v11.4s, v26.4s, v31.s[3]
	fmla	v12.4s, v22.4s, v31.s[0]
	fmla	v13.4s, v22.4s, v31.s[1]
	sub		w8, w8, #4
	fmla	v14.4s, v22.4s, v31.s[2]
	fmla	v15.4s, v22.4s, v31.s[3]

	// unroll 3
	ldp		q30, q31, [x11]
	fmla	v0.4s, v27.4s, v18.s[0]
	fmla	v1.4s, v27.4s, v18.s[1]
	prfm	PLDL1KEEP, [x11, x13]
	fmla	v2.4s, v27.4s, v18.s[2]
	prfm	PLDL1KEEP, [x20, x13]
	fmla	v3.4s, v27.4s, v18.s[3]
	add		x11, x11, x12
	fmla	v4.4s, v23.4s, v18.s[0]
	add		x20, x20, x12
	fmla	v5.4s, v23.4s, v18.s[1]
	fmla	v6.4s, v23.4s, v18.s[2]
	fmla	v7.4s, v23.4s, v18.s[3]
	fmla	v8.4s, v27.4s, v19.s[0]
	fmla	v9.4s, v27.4s, v19.s[1]
	fmla	v10.4s, v27.4s, v19.s[2]
	fmla	v11.4s, v27.4s, v19.s[3]
	fmla	v12.4s, v23.4s, v19.s[0]
	fmla	v13.4s, v23.4s, v19.s[1]
	fmla	v14.4s, v23.4s, v19.s[2]
	fmla	v15.4s, v23.4s, v19.s[3]

	cmp		w8, #4
	bgt		1b

0:

	cmp		w8, #3
	ble		4f

	// unroll 0
	ldp		q18, q19, [x11]
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v28.s[1]
//	prfm	PLDL1KEEP, [x11, x13]
	fmla	v2.4s, v24.4s, v28.s[2]
//	prfm	PLDL1KEEP, [x20, x13]
	fmla	v3.4s, v24.4s, v28.s[3]
	add		x11, x11, x12
	fmla	v4.4s, v20.4s, v28.s[0]
	add		x20, x20, x12
	fmla	v5.4s, v20.4s, v28.s[1]
	fmla	v6.4s, v20.4s, v28.s[2]
	ldp		q26, q27, [x9, #(2*16)]
	fmla	v7.4s, v20.4s, v28.s[3]
	fmla	v8.4s, v24.4s, v29.s[0]
	ldp		q22, q23, [x19, #(2*16)]
	fmla	v9.4s, v24.4s, v29.s[1]
	fmla	v10.4s, v24.4s, v29.s[2]
	fmla	v11.4s, v24.4s, v29.s[3]
	fmla	v12.4s, v20.4s, v29.s[0]
	fmla	v13.4s, v20.4s, v29.s[1]
	fmla	v14.4s, v20.4s, v29.s[2]
	fmla	v15.4s, v20.4s, v29.s[3]
//	ldp		q28, q29, [x11]

	// unroll 1
	fmla	v0.4s, v25.4s, v16.s[0]
	fmla	v1.4s, v25.4s, v16.s[1]
//	add		x11, x11, x12
	fmla	v2.4s, v25.4s, v16.s[2]
	fmla	v3.4s, v25.4s, v16.s[3]
	add		x9, x9, #64
	fmla	v4.4s, v21.4s, v16.s[0]
	fmla	v5.4s, v21.4s, v16.s[1]
	add		x19, x19, #64
	fmla	v6.4s, v21.4s, v16.s[2]
	fmla	v7.4s, v21.4s, v16.s[3]
	fmla	v8.4s, v25.4s, v17.s[0]
	fmla	v9.4s, v25.4s, v17.s[1]
	fmla	v10.4s, v25.4s, v17.s[2]
	fmla	v11.4s, v25.4s, v17.s[3]
	fmla	v12.4s, v21.4s, v17.s[0]
	fmla	v13.4s, v21.4s, v17.s[1]
	fmla	v14.4s, v21.4s, v17.s[2]
	fmla	v15.4s, v21.4s, v17.s[3]
//	ldp		q16, q17, [x11]

	// unroll 2
	fmla	v0.4s, v26.4s, v30.s[0]
//	add		x11, x11, x12
	fmla	v1.4s, v26.4s, v30.s[1]
//	ldp		q24, q25, [x9, #(0*16)]
	fmla	v2.4s, v26.4s, v30.s[2]
	fmla	v3.4s, v26.4s, v30.s[3]
//	ldp		q20, q21, [x19, #(0*16)]
	fmla	v4.4s, v22.4s, v30.s[0]
	fmla	v5.4s, v22.4s, v30.s[1]
	sub		w8, w8, #4
	fmla	v6.4s, v22.4s, v30.s[2]
	fmla	v7.4s, v22.4s, v30.s[3]
	fmla	v8.4s, v26.4s, v31.s[0]
	fmla	v9.4s, v26.4s, v31.s[1]
	fmla	v10.4s, v26.4s, v31.s[2]
	fmla	v11.4s, v26.4s, v31.s[3]
	fmla	v12.4s, v22.4s, v31.s[0]
	fmla	v13.4s, v22.4s, v31.s[1]
	fmla	v14.4s, v22.4s, v31.s[2]
	fmla	v15.4s, v22.4s, v31.s[3]
//	ldp		q30, q31, [x11]

	// unroll 3
	fmla	v0.4s, v27.4s, v18.s[0]
//	add		x11, x11, x12
	fmla	v1.4s, v27.4s, v18.s[1]
	fmla	v2.4s, v27.4s, v18.s[2]
	fmla	v3.4s, v27.4s, v18.s[3]
	fmla	v4.4s, v23.4s, v18.s[0]
	fmla	v5.4s, v23.4s, v18.s[1]
	fmla	v6.4s, v23.4s, v18.s[2]
	fmla	v7.4s, v23.4s, v18.s[3]
	fmla	v8.4s, v27.4s, v19.s[0]
	fmla	v9.4s, v27.4s, v19.s[1]
	fmla	v10.4s, v27.4s, v19.s[2]
	fmla	v11.4s, v27.4s, v19.s[3]
	fmla	v12.4s, v23.4s, v19.s[0]
	fmla	v13.4s, v23.4s, v19.s[1]
	fmla	v14.4s, v23.4s, v19.s[2]
	fmla	v15.4s, v23.4s, v19.s[3]
//	ldp		q18, q19, [x11]
//	add		x11, x11, x12

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

//	sub		x9, x9, #32
//	sub		x19, x19, #32
//	sub		x11, x11, #32
//	sub		x14, x14, #32
//	sub		x11, x11, x15
	sub		x11, x11, x14

3: // clean1-up loop

	// unroll 0

	ldp		q28, q29, [x11]
	ld1		{v24.4s}, [x9], #16
	ld1		{v20.4s}, [x19], #16
	fmla	v0.4s, v24.4s, v28.s[0]
	add		x11, x11, x12
	fmla	v1.4s, v24.4s, v28.s[1]
	fmla	v2.4s, v24.4s, v28.s[2]
	fmla	v3.4s, v24.4s, v28.s[3]
	fmla	v4.4s, v20.4s, v28.s[0]
	fmla	v5.4s, v20.4s, v28.s[1]
	fmla	v6.4s, v20.4s, v28.s[2]
	fmla	v7.4s, v20.4s, v28.s[3]
	fmla	v8.4s, v24.4s, v29.s[0]
	fmla	v9.4s, v24.4s, v29.s[1]
	fmla	v10.4s, v24.4s, v29.s[2]
	fmla	v11.4s, v24.4s, v29.s[3]
	fmla	v12.4s, v20.4s, v29.s[0]
	fmla	v13.4s, v20.4s, v29.s[1]
	fmla	v14.4s, v20.4s, v29.s[2]
	fmla	v15.4s, v20.4s, v29.s[3]

	sub		w8, w8, #1
	cmp		w8, #0
	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nt_8x8_lib4c)
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
	.macro INNER_KERNEL_GEMM_ADD_NN_8X8_LIB4C
#else
	.align	4
	FUN_START(inner_kernel_gemm_add_nn_8x8_lib4c)
#endif



#if defined(TARGET_ARMV8A_ARM_CORTEX_A53)



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x9, x10

	add		x13, x11, x12 // 2
//	add		x14, x13, x12 // 3
	add		x14, x11, x12, LSL #1 // 3
	add		x15, x14, x12 // 4
	add		x16, x14, x12, LSL #1 // 5
	add		x17, x16, x12 // 6
	add		x18, x16, x12, LSL #1 // 7
	add		x19, x18, x12 // 8

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x20, #0]
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x16, #0]
	prfm	PLDL1KEEP, [x17, #0]
	prfm	PLDL1KEEP, [x18, #0]
	prfm	PLDL1KEEP, [x19, #0]

	// preload

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #64]
	prfm	PLDL1KEEP, [x20, #64]

	// main loop
1:
	
	ldr		q28, [x11], #16
	ldr		q29, [x13], #16
	ldr		q30, [x14], #16
	ldr		q31, [x15], #16
	ldr		q16, [x16], #16
	ldr		q17, [x17], #16
	ldr		q18, [x18], #16
	ldr		q19, [x19], #16

	ldp		q24, q25, [x9], #32
	ldp		q20, q21, [x20], #32
	ldp		q26, q27, [x9], #32
	ldp		q22, q23, [x20], #32

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	prfm	PLDL1KEEP, [x9, #64]
	fmla	v1.4s, v24.4s, v29.s[0]
	prfm	PLDL1KEEP, [x20, #64]
	fmla	v2.4s, v24.4s, v30.s[0]
	prfm	PLDL1KEEP, [x11, #16]
	fmla	v3.4s, v24.4s, v31.s[0]
	prfm	PLDL1KEEP, [x13, #16]
	fmla	v4.4s, v20.4s, v28.s[0]
	prfm	PLDL1KEEP, [x14, #16]
	fmla	v5.4s, v20.4s, v29.s[0]
	prfm	PLDL1KEEP, [x15, #16]
	fmla	v6.4s, v20.4s, v30.s[0]
	prfm	PLDL1KEEP, [x16, #16]
	fmla	v7.4s, v20.4s, v31.s[0]
	prfm	PLDL1KEEP, [x17, #16]
	fmla	v8.4s, v24.4s, v16.s[0]
	prfm	PLDL1KEEP, [x18, #16]
	fmla	v9.4s, v24.4s, v17.s[0]
	prfm	PLDL1KEEP, [x19, #16]
	fmla	v10.4s, v24.4s, v18.s[0]
	fmla	v11.4s, v24.4s, v19.s[0]
	fmla	v12.4s, v20.4s, v16.s[0]
	fmla	v13.4s, v20.4s, v17.s[0]
	fmla	v14.4s, v20.4s, v18.s[0]
	fmla	v15.4s, v20.4s, v19.s[0]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
	fmla	v1.4s, v25.4s, v29.s[1]
	fmla	v2.4s, v25.4s, v30.s[1]
	fmla	v3.4s, v25.4s, v31.s[1]
	fmla	v4.4s, v21.4s, v28.s[1]
	fmla	v5.4s, v21.4s, v29.s[1]
	fmla	v6.4s, v21.4s, v30.s[1]
	fmla	v7.4s, v21.4s, v31.s[1]
	fmla	v8.4s, v25.4s, v16.s[1]
	fmla	v9.4s, v25.4s, v17.s[1]
	fmla	v10.4s, v25.4s, v18.s[1]
	fmla	v11.4s, v25.4s, v19.s[1]
	fmla	v12.4s, v21.4s, v16.s[1]
	fmla	v13.4s, v21.4s, v17.s[1]
	fmla	v14.4s, v21.4s, v18.s[1]
	fmla	v15.4s, v21.4s, v19.s[1]

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
	fmla	v1.4s, v26.4s, v29.s[2]
	fmla	v2.4s, v26.4s, v30.s[2]
	fmla	v3.4s, v26.4s, v31.s[2]
	fmla	v4.4s, v22.4s, v28.s[2]
	fmla	v5.4s, v22.4s, v29.s[2]
	fmla	v6.4s, v22.4s, v30.s[2]
	fmla	v7.4s, v22.4s, v31.s[2]
	fmla	v8.4s, v26.4s, v16.s[2]
	fmla	v9.4s, v26.4s, v17.s[2]
	fmla	v10.4s, v26.4s, v18.s[2]
	fmla	v11.4s, v26.4s, v19.s[2]
	fmla	v12.4s, v22.4s, v16.s[2]
	fmla	v13.4s, v22.4s, v17.s[2]
	fmla	v14.4s, v22.4s, v18.s[2]
	fmla	v15.4s, v22.4s, v19.s[2]
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
	fmla	v8.4s, v27.4s, v16.s[3]
	fmla	v9.4s, v27.4s, v17.s[3]
	fmla	v10.4s, v27.4s, v18.s[3]
	fmla	v11.4s, v27.4s, v19.s[3]
	fmla	v12.4s, v23.4s, v16.s[3]
	fmla	v13.4s, v23.4s, v17.s[3]
	fmla	v14.4s, v23.4s, v18.s[3]
	fmla	v15.4s, v23.4s, v19.s[3]

	cmp		w8, #4
	bgt		1b

//	sub		x9, x9, #32
//	sub		x11, x11, #32

0:

	cmp		w8, #3
	ble		4f

	ldr		q28, [x11], #16
	ldr		q29, [x13], #16
	ldr		q30, [x14], #16
	ldr		q31, [x15], #16
	ldr		q16, [x16], #16
	ldr		q17, [x17], #16
	ldr		q18, [x18], #16
	ldr		q19, [x19], #16

	ldp		q24, q25, [x9], #32
	ldp		q20, q21, [x20], #32
	ldp		q26, q27, [x9], #32
	ldp		q22, q23, [x20], #32

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v29.s[0]
	fmla	v2.4s, v24.4s, v30.s[0]
	fmla	v3.4s, v24.4s, v31.s[0]
	fmla	v4.4s, v20.4s, v28.s[0]
	fmla	v5.4s, v20.4s, v29.s[0]
	fmla	v6.4s, v20.4s, v30.s[0]
	fmla	v7.4s, v20.4s, v31.s[0]
	fmla	v8.4s, v24.4s, v16.s[0]
	fmla	v9.4s, v24.4s, v17.s[0]
	fmla	v10.4s, v24.4s, v18.s[0]
	fmla	v11.4s, v24.4s, v19.s[0]
	fmla	v12.4s, v20.4s, v16.s[0]
	fmla	v13.4s, v20.4s, v17.s[0]
	fmla	v14.4s, v20.4s, v18.s[0]
	fmla	v15.4s, v20.4s, v19.s[0]

	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
	fmla	v1.4s, v25.4s, v29.s[1]
	fmla	v2.4s, v25.4s, v30.s[1]
	fmla	v3.4s, v25.4s, v31.s[1]
	fmla	v4.4s, v21.4s, v28.s[1]
	fmla	v5.4s, v21.4s, v29.s[1]
	fmla	v6.4s, v21.4s, v30.s[1]
	fmla	v7.4s, v21.4s, v31.s[1]
	fmla	v8.4s, v25.4s, v16.s[1]
	fmla	v9.4s, v25.4s, v17.s[1]
	fmla	v10.4s, v25.4s, v18.s[1]
	fmla	v11.4s, v25.4s, v19.s[1]
	fmla	v12.4s, v21.4s, v16.s[1]
	fmla	v13.4s, v21.4s, v17.s[1]
	fmla	v14.4s, v21.4s, v18.s[1]
	fmla	v15.4s, v21.4s, v19.s[1]

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
	fmla	v1.4s, v26.4s, v29.s[2]
	fmla	v2.4s, v26.4s, v30.s[2]
	fmla	v3.4s, v26.4s, v31.s[2]
	fmla	v4.4s, v22.4s, v28.s[2]
	fmla	v5.4s, v22.4s, v29.s[2]
	fmla	v6.4s, v22.4s, v30.s[2]
	fmla	v7.4s, v22.4s, v31.s[2]
	fmla	v8.4s, v26.4s, v16.s[2]
	fmla	v9.4s, v26.4s, v17.s[2]
	fmla	v10.4s, v26.4s, v18.s[2]
	fmla	v11.4s, v26.4s, v19.s[2]
	fmla	v12.4s, v22.4s, v16.s[2]
	fmla	v13.4s, v22.4s, v17.s[2]
	fmla	v14.4s, v22.4s, v18.s[2]
	fmla	v15.4s, v22.4s, v19.s[2]
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
	fmla	v8.4s, v27.4s, v16.s[3]
	fmla	v9.4s, v27.4s, v17.s[3]
	fmla	v10.4s, v27.4s, v18.s[3]
	fmla	v11.4s, v27.4s, v19.s[3]
	fmla	v12.4s, v23.4s, v16.s[3]
	fmla	v13.4s, v23.4s, v17.s[3]
	fmla	v14.4s, v23.4s, v18.s[3]
	fmla	v15.4s, v23.4s, v19.s[3]

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9], #16
	ldr		q20, [x20], #16
	ldr		s28, [x11], #4
	ldr		s29, [x13], #4
	ldr		s30, [x14], #4
	ldr		s31, [x15], #4
	ldr		s16, [x16], #4
	ldr		s17, [x17], #4
	ldr		s18, [x18], #4
	ldr		s19, [x19], #4
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v29.s[0]
	fmla	v2.4s, v24.4s, v30.s[0]
	fmla	v3.4s, v24.4s, v31.s[0]
	fmla	v4.4s, v20.4s, v28.s[0]
	fmla	v5.4s, v20.4s, v29.s[0]
	fmla	v6.4s, v20.4s, v30.s[0]
	fmla	v7.4s, v20.4s, v31.s[0]
	fmla	v8.4s, v24.4s, v16.s[0]
	fmla	v9.4s, v24.4s, v17.s[0]
	fmla	v10.4s, v24.4s, v18.s[0]
	fmla	v11.4s, v24.4s, v19.s[0]
	fmla	v12.4s, v20.4s, v16.s[0]
	fmla	v13.4s, v20.4s, v17.s[0]
	fmla	v14.4s, v20.4s, v18.s[0]
	fmla	v15.4s, v20.4s, v19.s[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#else // cortex a53



	// early return
	cmp		w8, #0
	ble		2f // return

	add		x20, x9, x10

	add		x13, x11, x12 // 2
//	add		x14, x13, x12 // 3
	add		x14, x11, x12, LSL #1 // 3
	add		x15, x14, x12 // 4
	add		x16, x14, x12, LSL #1 // 5
	add		x17, x16, x12 // 6
	add		x18, x16, x12, LSL #1 // 7
	add		x19, x18, x12 // 8

	// prefetch
	prfm	PLDL1KEEP, [x9, #0]
	prfm	PLDL1KEEP, [x20, #0]
	prfm	PLDL1KEEP, [x11, #0]
	prfm	PLDL1KEEP, [x13, #0]
	prfm	PLDL1KEEP, [x14, #0]
	prfm	PLDL1KEEP, [x15, #0]
	prfm	PLDL1KEEP, [x16, #0]
	prfm	PLDL1KEEP, [x17, #0]
	prfm	PLDL1KEEP, [x18, #0]
	prfm	PLDL1KEEP, [x19, #0]

	// preload
	ldr		q28, [x11], #16
	ldr		q29, [x13], #16
	ldr		q30, [x14], #16
	ldr		q31, [x15], #16
	ldr		q16, [x16], #16
	ldr		q17, [x17], #16
	ldr		q18, [x18], #16
	ldr		q19, [x19], #16

	ldp		q24, q25, [x9], #32
	ldp		q20, q21, [x20], #32

	cmp		w8, #4
	ble		0f // consider clean up loop

	// prefetch
	prfm	PLDL1KEEP, [x9, #32]
	prfm	PLDL1KEEP, [x20, #64]

	// main loop
1:
	

	// unroll 0
	fmla	v0.4s, v24.4s, v28.s[0]
	ldp		q26, q27, [x9], #32
	fmla	v1.4s, v24.4s, v29.s[0]
	ldp		q22, q23, [x20], #32
	fmla	v2.4s, v24.4s, v30.s[0]
	prfm	PLDL1KEEP, [x9, #64]
	fmla	v3.4s, v24.4s, v31.s[0]
	prfm	PLDL1KEEP, [x20, #64]
	fmla	v4.4s, v20.4s, v28.s[0]
	prfm	PLDL1KEEP, [x11, #48]
	fmla	v5.4s, v20.4s, v29.s[0]
	prfm	PLDL1KEEP, [x13, #48]
	fmla	v6.4s, v20.4s, v30.s[0]
	prfm	PLDL1KEEP, [x14, #48]
	fmla	v7.4s, v20.4s, v31.s[0]
	prfm	PLDL1KEEP, [x15, #48]
	fmla	v8.4s, v24.4s, v16.s[0]
	prfm	PLDL1KEEP, [x16, #48]
	fmla	v9.4s, v24.4s, v17.s[0]
	prfm	PLDL1KEEP, [x17, #48]
	fmla	v10.4s, v24.4s, v18.s[0]
	prfm	PLDL1KEEP, [x18, #48]
	fmla	v11.4s, v24.4s, v19.s[0]
	prfm	PLDL1KEEP, [x19, #48]
	fmla	v12.4s, v20.4s, v16.s[0]
	fmla	v13.4s, v20.4s, v17.s[0]
	fmla	v14.4s, v20.4s, v18.s[0]
	fmla	v15.4s, v20.4s, v19.s[0]


	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
	fmla	v1.4s, v25.4s, v29.s[1]
	fmla	v2.4s, v25.4s, v30.s[1]
	fmla	v3.4s, v25.4s, v31.s[1]
	fmla	v4.4s, v21.4s, v28.s[1]
	fmla	v5.4s, v21.4s, v29.s[1]
	fmla	v6.4s, v21.4s, v30.s[1]
	fmla	v7.4s, v21.4s, v31.s[1]
	fmla	v8.4s, v25.4s, v16.s[1]
	fmla	v9.4s, v25.4s, v17.s[1]
	fmla	v10.4s, v25.4s, v18.s[1]
	fmla	v11.4s, v25.4s, v19.s[1]
	ldp		q24, q25, [x9], #32
	fmla	v12.4s, v21.4s, v16.s[1]
	fmla	v13.4s, v21.4s, v17.s[1]
	fmla	v14.4s, v21.4s, v18.s[1]
	fmla	v15.4s, v21.4s, v19.s[1]
	ldp		q20, q21, [x20], #32

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
	fmla	v1.4s, v26.4s, v29.s[2]
	fmla	v2.4s, v26.4s, v30.s[2]
	fmla	v3.4s, v26.4s, v31.s[2]
	fmla	v4.4s, v22.4s, v28.s[2]
	fmla	v5.4s, v22.4s, v29.s[2]
	fmla	v6.4s, v22.4s, v30.s[2]
	fmla	v7.4s, v22.4s, v31.s[2]
	fmla	v8.4s, v26.4s, v16.s[2]
	fmla	v9.4s, v26.4s, v17.s[2]
	fmla	v10.4s, v26.4s, v18.s[2]
	fmla	v11.4s, v26.4s, v19.s[2]
	fmla	v12.4s, v22.4s, v16.s[2]
	fmla	v13.4s, v22.4s, v17.s[2]
	fmla	v14.4s, v22.4s, v18.s[2]
	fmla	v15.4s, v22.4s, v19.s[2]
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
	fmla	v8.4s, v27.4s, v16.s[3]
	fmla	v9.4s, v27.4s, v17.s[3]
	fmla	v10.4s, v27.4s, v18.s[3]
	fmla	v11.4s, v27.4s, v19.s[3]
	fmla	v12.4s, v23.4s, v16.s[3]
	ldr		q16, [x16], #16
	fmla	v13.4s, v23.4s, v17.s[3]
	ldr		q17, [x17], #16
	fmla	v14.4s, v23.4s, v18.s[3]
	ldr		q18, [x18], #16
	fmla	v15.4s, v23.4s, v19.s[3]
	ldr		q19, [x19], #16

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
	ldp		q22, q23, [x20], #32
	fmla	v2.4s, v24.4s, v30.s[0]
//	prfm	PLDL1KEEP, [x9, #64]
	fmla	v3.4s, v24.4s, v31.s[0]
//	prfm	PLDL1KEEP, [x20, #64]
	fmla	v4.4s, v20.4s, v28.s[0]
//	prfm	PLDL1KEEP, [x11, #16]
	fmla	v5.4s, v20.4s, v29.s[0]
//	prfm	PLDL1KEEP, [x13, #16]
	fmla	v6.4s, v20.4s, v30.s[0]
//	prfm	PLDL1KEEP, [x14, #16]
	fmla	v7.4s, v20.4s, v31.s[0]
//	prfm	PLDL1KEEP, [x15, #16]
	fmla	v8.4s, v24.4s, v16.s[0]
//	prfm	PLDL1KEEP, [x16, #16]
	fmla	v9.4s, v24.4s, v17.s[0]
//	prfm	PLDL1KEEP, [x17, #16]
	fmla	v10.4s, v24.4s, v18.s[0]
//	prfm	PLDL1KEEP, [x18, #16]
	fmla	v11.4s, v24.4s, v19.s[0]
//	prfm	PLDL1KEEP, [x19, #16]
	fmla	v12.4s, v20.4s, v16.s[0]
	fmla	v13.4s, v20.4s, v17.s[0]
	fmla	v14.4s, v20.4s, v18.s[0]
	fmla	v15.4s, v20.4s, v19.s[0]


	// unroll 1
	fmla	v0.4s, v25.4s, v28.s[1]
	fmla	v1.4s, v25.4s, v29.s[1]
	fmla	v2.4s, v25.4s, v30.s[1]
	fmla	v3.4s, v25.4s, v31.s[1]
	fmla	v4.4s, v21.4s, v28.s[1]
	fmla	v5.4s, v21.4s, v29.s[1]
	fmla	v6.4s, v21.4s, v30.s[1]
	fmla	v7.4s, v21.4s, v31.s[1]
	fmla	v8.4s, v25.4s, v16.s[1]
	fmla	v9.4s, v25.4s, v17.s[1]
	fmla	v10.4s, v25.4s, v18.s[1]
	fmla	v11.4s, v25.4s, v19.s[1]
//	ldp		q24, q25, [x9], #32
	fmla	v12.4s, v21.4s, v16.s[1]
	fmla	v13.4s, v21.4s, v17.s[1]
	fmla	v14.4s, v21.4s, v18.s[1]
	fmla	v15.4s, v21.4s, v19.s[1]
//	ldp		q20, q21, [x20], #32

	// unroll 2
	fmla	v0.4s, v26.4s, v28.s[2]
	fmla	v1.4s, v26.4s, v29.s[2]
	fmla	v2.4s, v26.4s, v30.s[2]
	fmla	v3.4s, v26.4s, v31.s[2]
	fmla	v4.4s, v22.4s, v28.s[2]
	fmla	v5.4s, v22.4s, v29.s[2]
	fmla	v6.4s, v22.4s, v30.s[2]
	fmla	v7.4s, v22.4s, v31.s[2]
	fmla	v8.4s, v26.4s, v16.s[2]
	fmla	v9.4s, v26.4s, v17.s[2]
	fmla	v10.4s, v26.4s, v18.s[2]
	fmla	v11.4s, v26.4s, v19.s[2]
	fmla	v12.4s, v22.4s, v16.s[2]
	fmla	v13.4s, v22.4s, v17.s[2]
	fmla	v14.4s, v22.4s, v18.s[2]
	fmla	v15.4s, v22.4s, v19.s[2]
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
	fmla	v8.4s, v27.4s, v16.s[3]
	fmla	v9.4s, v27.4s, v17.s[3]
	fmla	v10.4s, v27.4s, v18.s[3]
	fmla	v11.4s, v27.4s, v19.s[3]
	fmla	v12.4s, v23.4s, v16.s[3]
//	ldr		q16, [x16], #16
	fmla	v13.4s, v23.4s, v17.s[3]
//	ldr		q17, [x17], #16
	fmla	v14.4s, v23.4s, v18.s[3]
//	ldr		q18, [x18], #16
	fmla	v15.4s, v23.4s, v19.s[3]
//	ldr		q19, [x19], #16

	b		2f // return

4: // consider clean1-up loop

	cmp		w8, #0
	ble		2f // return

	sub		x9, x9, #32
	sub		x20, x20, #32
	sub		x11, x11, #16
	sub		x13, x13, #16
	sub		x14, x14, #16
	sub		x15, x15, #16
	sub		x16, x16, #16
	sub		x17, x17, #16
	sub		x18, x18, #16
	sub		x19, x19, #16

3: // clean1-up loop

	// unroll 0
	ldr		q24, [x9], #16
	ldr		q20, [x20], #16
	ldr		s28, [x11], #4
	ldr		s29, [x13], #4
	ldr		s30, [x14], #4
	ldr		s31, [x15], #4
	ldr		s16, [x16], #4
	ldr		s17, [x17], #4
	ldr		s18, [x18], #4
	ldr		s19, [x19], #4
	fmla	v0.4s, v24.4s, v28.s[0]
	fmla	v1.4s, v24.4s, v29.s[0]
	fmla	v2.4s, v24.4s, v30.s[0]
	fmla	v3.4s, v24.4s, v31.s[0]
	fmla	v4.4s, v20.4s, v28.s[0]
	fmla	v5.4s, v20.4s, v29.s[0]
	fmla	v6.4s, v20.4s, v30.s[0]
	fmla	v7.4s, v20.4s, v31.s[0]
	fmla	v8.4s, v24.4s, v16.s[0]
	fmla	v9.4s, v24.4s, v17.s[0]
	fmla	v10.4s, v24.4s, v18.s[0]
	fmla	v11.4s, v24.4s, v19.s[0]
	fmla	v12.4s, v20.4s, v16.s[0]
	fmla	v13.4s, v20.4s, v17.s[0]
	fmla	v14.4s, v20.4s, v18.s[0]
	fmla	v15.4s, v20.4s, v19.s[0]

	sub		w8, w8, #1
	cmp		w8, #0

	bgt		3b

2: // return



#endif // cortex a53



#if MACRO_LEVEL>=2
	.endm
#else
	ret

	FUN_END(inner_kernel_gemm_add_nn_8x8_lib4c)
#endif





// subroutine
//
// input arguments:
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_TRAN_8X8_LIB
#else
	.align	4
	FUN_START(inner_tran_8x8_lib)
#endif

	trn1	v16.4s, v0.4s, v1.4s
	trn2	v17.4s, v0.4s, v1.4s
	trn1	v18.4s, v2.4s, v3.4s
	trn2	v19.4s, v2.4s, v3.4s

	trn1	v0.2d, v16.2d, v18.2d
	trn2	v2.2d, v16.2d, v18.2d
	trn1	v1.2d, v17.2d, v19.2d
	trn2	v3.2d, v17.2d, v19.2d

	trn1	v16.4s, v12.4s, v13.4s
	trn2	v17.4s, v12.4s, v13.4s
	trn1	v18.4s, v14.4s, v15.4s
	trn2	v19.4s, v14.4s, v15.4s

	trn1	v12.2d, v16.2d, v18.2d
	trn2	v14.2d, v16.2d, v18.2d
	trn1	v13.2d, v17.2d, v19.2d
	trn2	v15.2d, v17.2d, v19.2d

	trn1	v16.4s, v4.4s, v5.4s
	trn2	v17.4s, v4.4s, v5.4s
	trn1	v18.4s, v6.4s, v7.4s
	trn2	v19.4s, v6.4s, v7.4s

	trn1	v20.4s, v8.4s, v9.4s
	trn2	v21.4s, v8.4s, v9.4s
	trn1	v22.4s, v10.4s, v11.4s
	trn2	v23.4s, v10.4s, v11.4s

	trn1	v8.2d, v16.2d, v18.2d
	trn2	v10.2d, v16.2d, v18.2d
	trn1	v9.2d, v17.2d, v19.2d
	trn2	v11.2d, v17.2d, v19.2d

	trn1	v4.2d, v20.2d, v22.2d
	trn2	v6.2d, v20.2d, v22.2d
	trn1	v5.2d, v21.2d, v23.2d
	trn2	v7.2d, v21.2d, v23.2d

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_tran_8x8_lib)
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
	.macro INNER_SCALE_AB_8X8_LIB
#else
	.align	4
	FUN_START(inner_scale_ab_8x8_lib)
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
	fmul	v8.4s, v8.4s, v28.s[0]
	fmul	v9.4s, v9.4s, v28.s[0]
	fmul	v10.4s, v10.4s, v28.s[0]
	fmul	v11.4s, v11.4s, v28.s[0]
	fmul	v12.4s, v12.4s, v28.s[0]
	fmul	v13.4s, v13.4s, v28.s[0]
	fmul	v14.4s, v14.4s, v28.s[0]
	fmul	v15.4s, v15.4s, v28.s[0]

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

	ldp		q24, q25, [x10, #0]
	add		x10, x10, x11
	ldp		q26, q27, [x10, #0]
	add		x10, x10, x11

	fmla	v8.4s, v24.4s, v29.s[0]
	fmla	v12.4s, v25.4s, v29.s[0]
	fmla	v9.4s, v26.4s, v29.s[0]
	fmla	v13.4s, v27.4s, v29.s[0]

	ldp		q24, q25, [x10, #0]
	add		x10, x10, x11
	ldp		q26, q27, [x10, #0]
	add		x10, x10, x11

	fmla	v10.4s, v24.4s, v29.s[0]
	fmla	v14.4s, v25.4s, v29.s[0]
	fmla	v11.4s, v26.4s, v29.s[0]
	fmla	v15.4s, v27.4s, v29.s[0]

0:

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_scale_ab_8x8_lib)
#endif





// subroutine
//
// input arguments:
// x8   <- D
// x9   <- ldd*sizeof(float)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_STORE_8X8_LIB
#else
	.align 4
	FUN_START(inner_store_8x8_lib)
#endif

	str		q0, [x8, #0]
	str		q4, [x8, #16]
	add		x8, x8, x9

	str		q1, [x8, #0]
	str		q5, [x8, #16]
	add		x8, x8, x9

	str		q2, [x8, #0]
	str		q6, [x8, #16]
	add		x8, x8, x9

	str		q3, [x8, #0]
	str		q7, [x8, #16]
	add		x8, x8, x9

	str		q8, [x8, #0]
	str		q12, [x8, #16]
	add		x8, x8, x9

	str		q9, [x8, #0]
	str		q13, [x8, #16]
	add		x8, x8, x9

	str		q10, [x8, #0]
	str		q14, [x8, #16]
	add		x8, x8, x9

	str		q11, [x8, #0]
	str		q15, [x8, #16]
//	add		x8, x8, x9

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_store_8x8_lib)
#endif





// subroutine
//
// input arguments:
// x8   <- D
// x9   <- ldd*sizeof(float)
//
// output arguments:

#if MACRO_LEVEL>=1
	.macro INNER_PREFETCH_8X8_LIB
#else
	.align 4
	FUN_START(inner_prefetch_8x8_lib)
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
	add		x8, x8, x9

	prfm	PLDL1KEEP, [x8, #0]
//	prfm	PLDL1KEEP, [x8, #28]
	prfm	PLDL1KEEP, [x8, #24]
//	add		x8, x8, x9

#if MACRO_LEVEL>=1
	.endm
#else
	ret

	FUN_END(inner_prefetch_8x8_lib)
#endif





//                                 w0        x1             x2        w3        x4       w5       x6           x7        sp+0     sp+8      sp+16
// void kernel_sgemm_nt_8x8_lib44cc(int kmax, float *alpha, float *A, int sda, float *B, int sdb, float *beta, float *C, int ldc, float *D, int ldd)

	.align	4
	GLOB(kernel_sgemm_nt_8x8_lib44cc)
	FUN_START(kernel_sgemm_nt_8x8_lib44cc)
	


	PROLOGUE



	ZERO_ACC



	// prefetch C
//	mov		x8, x6 // beta
	ld1		{v29.4s}, [x6]
	fcmpe	s29, #0.0
	beq		100f

	mov		x8, x7 // C
	ldr		w9, [sp, #(STACKSIZE + 0)] // ldc
	lsl		w9, w9, #2 // 4*ldc

#if MACRO_LEVEL>=1
	INNER_PREFETCH_8X8_LIB
#else
	CALL(inner_prefetch_8x8_lib)
#endif

100:


	// call inner kernel gemm nt
	mov		w8, w0 // kmax
	mov		x9, x2 // A
	mov		w10, w3 // sda
	lsl		w10, w10, #4 // 16*sda
	mov		x11, x4 // B
	mov		w12, w5 // sdb
	lsl		w12, w12, #4 // 16*sdb

#if MACRO_LEVEL>=2
	INNER_KERNEL_GEMM_ADD_NT_8X8_LIB4
#else
	CALL(inner_kernel_gemm_add_nt_8x8_lib4)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #2 // 4*ldd

#if MACRO_LEVEL>=1
	INNER_PREFETCH_8X8_LIB
#else
	CALL(inner_prefetch_8x8_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x6 // beta
	mov		x10, x7 // C
	ldr		w11, [sp, #(STACKSIZE + 0)] // ldc
	lsl		w11, w11, #2 // 4*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X8_LIB
#else
	CALL(inner_scale_ab_8x8_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // ldd
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #2 // 4*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_8X8_LIB
#else
	CALL(inner_store_8x8_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_sgemm_nt_8x8_lib44cc)





//                                  w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16
// void kernel_sgemm_nt_8x8_lib4ccc(int kmax, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_sgemm_nt_8x8_lib4ccc)
	FUN_START(kernel_sgemm_nt_8x8_lib4ccc)
	


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
	INNER_KERNEL_GEMM_ADD_NT_8X8_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_8x8_lib4c)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #2 // 4*sdd

#if MACRO_LEVEL>=1
	INNER_PREFETCH_8X8_LIB
#else
	CALL(inner_prefetch_8x8_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x6 // beta
	mov		x10, x7 // C
	ldr		w11, [sp, #(STACKSIZE + 0)] // ldc
	lsl		w11, w11, #2 // 4*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X8_LIB
#else
	CALL(inner_scale_ab_8x8_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #2 // 4*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_8X8_LIB
#else
	CALL(inner_store_8x8_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_sgemm_nt_8x8_lib4ccc)





//                                  w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16
// void kernel_sgemm_nt_8x8_libc4cc(int kmax, double *alpha, double *A, int lda, double *B, int sdb, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_sgemm_nt_8x8_libc4cc)
	FUN_START(kernel_sgemm_nt_8x8_libc4cc)
	


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
	INNER_KERNEL_GEMM_ADD_NT_8X8_LIB4C
#else
	CALL(inner_kernel_gemm_add_nt_8x8_lib4c)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #2 // 4*sdd

#if MACRO_LEVEL>=1
	INNER_PREFETCH_8X8_LIB
#else
	CALL(inner_prefetch_8x8_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x6 // beta
	mov		x10, x7 // C
	ldr		w11, [sp, #(STACKSIZE + 0)] // ldc
	lsl		w11, w11, #2 // 4*ldc

#if MACRO_LEVEL>=1
	INNER_TRAN_8X8_LIB
#else
	CALL(inner_tran_8x8_lib)
#endif


#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X8_LIB
#else
	CALL(inner_scale_ab_8x8_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #2 // 4*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_8X8_LIB
#else
	CALL(inner_store_8x8_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_sgemm_nt_8x8_libc4cc)





//                                  w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16
// void kernel_sgemm_nn_8x8_lib4ccc(int kmax, double *alpha, double *A, int sda, double *B, int ldb, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_sgemm_nn_8x8_lib4ccc)
	FUN_START(kernel_sgemm_nn_8x8_lib4ccc)
	


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
	INNER_KERNEL_GEMM_ADD_NN_8X8_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_8x8_lib4c)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #2 // 4*sdd

#if MACRO_LEVEL>=1
	INNER_PREFETCH_8X8_LIB
#else
	CALL(inner_prefetch_8x8_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x6 // beta
	mov		x10, x7 // C
	ldr		w11, [sp, #(STACKSIZE + 0)] // ldc
	lsl		w11, w11, #2 // 4*ldc

#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X8_LIB
#else
	CALL(inner_scale_ab_8x8_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #2 // 4*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_8X8_LIB
#else
	CALL(inner_store_8x8_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_sgemm_nn_8x8_lib4ccc)





//                                  w0        x1             x2         w3       x4         w5       x6            x7         sp+0     sp+8       sp+16
// void kernel_sgemm_tt_8x8_libc4cc(int kmax, double *alpha, double *A, int lda, double *B, int sdb, double *beta, double *C, int ldc, double *D, int ldd)

	.align	4
	GLOB(kernel_sgemm_tt_8x8_libc4cc)
	FUN_START(kernel_sgemm_tt_8x8_libc4cc)
	


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
	INNER_KERNEL_GEMM_ADD_NN_8X8_LIB4C
#else
	CALL(inner_kernel_gemm_add_nn_8x8_lib4c)
#endif



	// prefetch
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #2 // 4*sdd

#if MACRO_LEVEL>=1
	INNER_PREFETCH_8X8_LIB
#else
	CALL(inner_prefetch_8x8_lib)
#endif



	// call inner blend for generic alpha and beta
	mov		x8, x1 // alpha
	mov		x9, x6 // beta
	mov		x10, x7 // C
	ldr		w11, [sp, #(STACKSIZE + 0)] // ldc
	lsl		w11, w11, #2 // 4*ldc

#if MACRO_LEVEL>=1
	INNER_TRAN_8X8_LIB
#else
	CALL(inner_tran_8x8_lib)
#endif


#if MACRO_LEVEL>=1
	INNER_SCALE_AB_8X8_LIB
#else
	CALL(inner_scale_ab_8x8_lib)
#endif



	// store n
	ldr		x8, [sp, #(STACKSIZE + 8)] // D
	ldr		w9, [sp, #(STACKSIZE + 16)] // ldd
	lsl		w9, w9, #2 // 4*ldd

#if MACRO_LEVEL>=1
	INNER_STORE_8X8_LIB
#else
	CALL(inner_store_8x8_lib)
#endif



	EPILOGUE

	mov	x0, #0

	ret

	FUN_END(kernel_sgemm_tt_8x8_libc4cc)






